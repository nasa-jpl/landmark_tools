/** 
 * \file image_comparison_main.c
 * \author Tu-Hoa Pham
 * \date 2024
 * 
 * \brief Compare two surface reflectance images using dense patch-based correlation matching
 * \copyright Copyright 2024 California Institute of Technology
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*-----------------------------------------------------------*/
/*------------------------ Includes -------------------------*/
/*-----------------------------------------------------------*/
#include <assert.h>                                         // for assert
#include <math.h>                                           // for fabs, sqrt
#include <string.h>                                         // for strncpy
#include <stdbool.h>                                        // for false, bool
#include <stdint.h>                                         // for int32_t
#include <stdio.h>                                          // for printf, NULL
#include <stdlib.h>                                         // for free, malloc

#include "landmark_tools/feature_tracking/feature_match.h"  // for MatchFeat...
#include "landmark_tools/feature_tracking/parameters.h"            // for FTP, Read...
#include "landmark_tools/math/homography_util.h"
#include "landmark_tools/landmark_util/landmark.h"          // for free_lmk
#include "landmark_tools/utils/parse_args.h"                // for m_getarg
#include "math/mat3/mat3.h"                                 // for mult331
#include "landmark_tools/opencv_tools/feature_matching_2d.h"  // for process_matched_feature_2d, MatchFeatures_local_distortion_2d
#include "landmark_tools/utils/safe_string.h"

// OpenCV C++ code
#include "landmark_tools/opencv_tools/opencv_feature_matching.h" // for calc_homography_from_feature_matching
#include "landmark_tools/opencv_tools/opencv_image_io.h"

#include "landmark_tools/feature_tracking/correlation_results.h"  // for CorrelationResults
#include "landmark_tools/opencv_tools/homography_estimation.h" // for estimateHomographyFromFeatureMatching
#include "landmark_tools/utils/write_array.h"

void show_usage_and_exit()
{
    printf("Usage for landmark_image_comparison:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -base_image    <image_filepath> \n");
    printf("    -child_image   <image_filepath> \n");
    printf("    -output_dir    <output_dir> \n");
    printf("    -output_filename_prefix    <output_filename_prefix> \n");
    printf("  Optional arguments:\n");
    printf("    -base_nan_mask     <mask_out_filepath> \n");
    printf("    -child_nan_mask    <mask_out_filepath> \n");
    printf("    -base_nan_max_count     <-1 to ignore, 0 or greater to filter> \n");
    printf("    -child_nan_max_count     <-1 to ignore, 0 or greater to filter> \n");
    printf("    -warp    <image(default)/template> \n");
    printf("    -homography_max_dist_between_matching_keypoints    <0 or greater> \n");
    printf("    -c    <ftp_config_filepath> \n");
    exit(EXIT_FAILURE);
}


/**
 * @brief Clean up all allocated resources
 * 
 * @param child_image Pointer to child image array
 * @param base_image Pointer to base image array
 * @param child_nan_mask Pointer to child nan mask array
 * @param base_nan_mask Pointer to base nan mask array
 * @param corr_struct Pointer to correlation results structure
 */
static void cleanup_resources(uint8_t **child_image,
                            uint8_t **base_image,
                            uint8_t **child_nan_mask,
                            uint8_t **base_nan_mask,
                            CorrelationResults *corr_struct)
{
    if (child_image != NULL && *child_image != NULL) {
        free_image(child_image);
        *child_image = NULL;
    }
    
    if (base_image != NULL && *base_image != NULL) {
        free_image(base_image);
        *base_image = NULL;
    }
    
    if (child_nan_mask != NULL && *child_nan_mask != NULL) {
        free_image(child_nan_mask);
        *child_nan_mask = NULL;
    }
    
    if (base_nan_mask != NULL && *base_nan_mask != NULL) {
        free_image(base_nan_mask);
        *base_nan_mask = NULL;
    }
    
    if (corr_struct != NULL) {
        destroy_correlation_results(corr_struct);
    }
}


/**
 * @brief Load or create a mask for an image
 * 
 * @param mask_file Path to the mask file (NULL to create a zero mask)
 * @param image_num_cols Number of columns in the image
 * @param image_num_rows Number of rows in the image
 * @param mask Pointer to store the loaded/created mask
 * @return true if mask was successfully loaded/created
 * @return false if there was an error
 */
static bool load_mask(const char *mask_file, 
                     int image_num_cols, 
                     int image_num_rows, 
                     uint8_t **mask)
{
    if (mask_file == NULL) {
        printf("Using zero nan mask\n");
        *mask = malloc(sizeof(uint8_t) * image_num_cols * image_num_rows);
        if (*mask == NULL) {
            printf("Failure to allocate memory for nan mask\n");
            return false;
        }
        memset(*mask, 0, sizeof(uint8_t) * image_num_cols * image_num_rows);
    } else {
        int mask_num_rows, mask_num_cols;
        *mask = readPGMToArray(mask_file, &mask_num_cols, &mask_num_rows);
        if (*mask == NULL || mask_num_cols != image_num_cols || mask_num_rows != image_num_rows) {
            printf("Failed to load mask or size mismatch\n");
            return false;
        }

        for (int32_t i = 0; i < mask_num_cols * mask_num_rows; ++i) {
            (*mask)[i] = (*mask)[i] > 0 ? 1 : 0;
        }
    }
    return true;
}

int32_t main (int32_t argc, char **argv)
{
    // Initialize variables
    char *base_image_file = NULL;
    char *child_image_file = NULL;
    char *base_nan_mask_file = NULL;
    char *child_nan_mask_file = NULL;
    char *warp_image_or_template_str = NULL;
    char *output_dir = NULL;
    char *output_filename_prefix = NULL;
    char *parameter_file = NULL;
    
    // Set this to > 0 to reject matches that are more than N pixels apart on any direction
    // Only use this if images are expected to be in about the same frame
    double homography_max_dist_between_matching_keypoints = 0.0;
    
    // Max number of nans per window
    int32_t base_nan_max_count = -1;
    int32_t child_nan_max_count = -1;

    // Initialize pointers to NULL for cleanup
    uint8_t *child_image = NULL;
    uint8_t *base_image = NULL;
    uint8_t *child_nan_mask = NULL;
    uint8_t *base_nan_mask = NULL;
    CorrelationResults corr_struct = {0};

    argc--;
    argv++;

    if (argc==0) show_usage_and_exit( );

    while (argc>0)
    {
        if (argc==1) show_usage_and_exit( );
        if ((m_getarg(argv, "-base_image",    &base_image_file,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-child_image",   &child_image_file,       CFO_STRING)!=1) &&
            (m_getarg(argv, "-base_nan_mask",     &base_nan_mask_file,       CFO_STRING)!=1) &&
            (m_getarg(argv, "-child_nan_mask",    &child_nan_mask_file,      CFO_STRING)!=1) &&
            (m_getarg(argv, "-base_nan_max_count",     &base_nan_max_count,       CFO_INT)!=1) &&
            (m_getarg(argv, "-child_nan_max_count",    &child_nan_max_count,      CFO_INT)!=1) &&
            (m_getarg(argv, "-warp",    &warp_image_or_template_str,      CFO_STRING)!=1) &&
            (m_getarg(argv, "-output_dir",    &output_dir,         CFO_STRING)!=1) &&
            (m_getarg(argv, "-output_filename_prefix",    &output_filename_prefix,         CFO_STRING)!=1) &&
            (m_getarg(argv, "-homography_max_dist_between_matching_keypoints",    &homography_max_dist_between_matching_keypoints,         CFO_DOUBLE)!=1) &&
            (m_getarg(argv, "-c",    &parameter_file,         CFO_STRING)!=1))
            show_usage_and_exit( );

        argc-=2;
        argv+=2;
    }

    WarpingMethod warp_image_or_template = WarpingMethod_IMAGE;
    if(warp_image_or_template_str != NULL){
        warp_image_or_template = StrToWarpingMethod(warp_image_or_template_str);
    }else{
        printf("No warp method provided. Using image warp\n");
    }

    if (
        (base_image_file == NULL)
        || (child_image_file == NULL)
        || (warp_image_or_template == WarpingMethod_UNDEFINED)
        || (output_filename_prefix == NULL)
    )
    {
        show_usage_and_exit();
    }

    Parameters parameters;
    load_default_parameters(&parameters);
    if(parameter_file != NULL){
        int32_t success = read_parameterfile(parameter_file, &parameters);
        if(!success){
            return EXIT_FAILURE;
        }
    }else{
        printf("No parameter file provided. Using default parameters\n");
    }

    // Load images
    int child_image_num_rows, child_image_num_cols;
    int base_image_num_rows, base_image_num_cols;
    child_image = readPGMToArray(child_image_file, &child_image_num_cols, &child_image_num_rows);
    base_image = readPGMToArray(base_image_file, &base_image_num_cols, &base_image_num_rows);
 
    if(child_image == NULL || base_image == NULL){
        printf("Failed to load images, exiting without output.\n");
        cleanup_resources(&child_image, &base_image, &child_nan_mask, &base_nan_mask, &corr_struct);
        return EXIT_FAILURE;
    }

    // Load masks
    if (!load_mask(base_nan_mask_file, base_image_num_cols, base_image_num_rows, &base_nan_mask)) {
        printf("Failed to load base mask\n");
        cleanup_resources(&child_image, &base_image, &child_nan_mask, &base_nan_mask, &corr_struct);
        return EXIT_FAILURE;
    }

    if (!load_mask(child_nan_mask_file, child_image_num_cols, child_image_num_rows, &child_nan_mask)) {
        printf("Failed to load child mask\n");
        cleanup_resources(&child_image, &base_image, &child_nan_mask, &base_nan_mask, &corr_struct);
        return EXIT_FAILURE;
    }

    if (child_nan_mask == NULL || base_nan_mask == NULL) {
        printf("Failed to load masks, exiting without output.\n");
        cleanup_resources(&child_image, &base_image, &child_nan_mask, &base_nan_mask, &corr_struct);
        return EXIT_FAILURE;
    }

    // Initialize output correlation
    int child_image_num_pixels = child_image_num_rows * child_image_num_cols;
    allocate_correlation_results(&corr_struct, child_image_num_pixels);
    bool success = MatchFeatures_local_distortion_2d(
        parameters,
        &base_image,
        &base_nan_mask,
        &base_image_num_rows,
        &base_image_num_cols,
        &child_image,
        &child_nan_mask,
        &child_image_num_rows,
        &child_image_num_cols,
        &corr_struct,
        warp_image_or_template,
        output_dir,
        homography_max_dist_between_matching_keypoints,
        child_nan_max_count,
        base_nan_max_count
    );

    if (!success)
    {
        printf("Failed to match features. Set corr_struct output to all nan.\n");
        for(size_t i = 0; i < child_image_num_pixels; ++i)
        {
            corr_struct.delta_x[i] = NAN;
            corr_struct.delta_y[i] = NAN;
            corr_struct.delta_z[i] = NAN;
            corr_struct.correlation[i] = NAN;
        }
    }

    size_t output_basepath_size = 512;
    char output_basepath[output_basepath_size];
    snprintf(output_basepath, output_basepath_size, "%s/%s", output_dir, output_filename_prefix);
    SAFE_PRINTF(512, "Saving results to %s\n", output_basepath);

    size_t buf_size = 544;
    char buf[buf_size];
    snprintf(buf, buf_size, "%s_delta_x_%dby%d.raw", output_basepath, child_image_num_cols, child_image_num_rows);
    write_data_to_file(buf, corr_struct.delta_x, sizeof(float), child_image_num_pixels);
    
    snprintf(buf, buf_size, "%s_delta_y_%dby%d.raw", output_basepath, child_image_num_cols, child_image_num_rows);
    write_data_to_file(buf, corr_struct.delta_y, sizeof(float), child_image_num_pixels);
    
    snprintf(buf, buf_size, "%s_corr_%dby%d.raw", output_basepath, child_image_num_cols, child_image_num_rows);
    write_data_to_file(buf, corr_struct.correlation, sizeof(float), child_image_num_pixels);

    cleanup_resources(&child_image, &base_image, &child_nan_mask, &base_nan_mask, &corr_struct);
    return success ? EXIT_SUCCESS : EXIT_FAILURE;
}