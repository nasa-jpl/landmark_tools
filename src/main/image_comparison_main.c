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

// OpenCV C++ code
#include "landmark_tools/opencv_tools/opencv_feature_matching.h" // for calc_homography_from_feature_matching
#include "landmark_tools/opencv_tools/opencv_image_io.h"

/*-----------------------------------------------------------*/
/*----------------------- Variables -------------------------*/
/*-----------------------------------------------------------*/

#define BLOCK_SIZE 200
#define STEP_SIZE 4
#define MIN_N_FEATURES 20
// TODO Hrand's code uses a FEATURE_WINDOW of 5
#define FEATURE_WINDOW 7

typedef struct {
    float* deltax;
    float* deltay;
    float* deltaz;
    float* corr_m;
} CORR_STRUCT;

typedef enum {
    WarpingMethod_IMAGE,
    WarpingMethod_TEMPLATE,
    WarpingMethod_UNDEFINED
} WarpingMethod;

WarpingMethod StrToWarpingMethod(const char* str){
    if(str != NULL){
        if(strncmp(str, "image", strlen(str)) == 0){
            return WarpingMethod_IMAGE;
        }else if(strncmp(str, "template", strlen(str)) == 0){
            return WarpingMethod_TEMPLATE;
        }
    }
    fprintf(stderr, "No WarpingMethod defined that corresponds to %.16s. Valid methods are \"image\" and \"template\"/n",
            str);
    return WarpingMethod_UNDEFINED;
}

void free_image(uint8_t **p_image)
{
    if (*p_image != NULL)
    {
        free(*p_image);
        *p_image = NULL;
    }
}


uint8_t allocate_corr(CORR_STRUCT* corr_struct, size_t num_pixels);
void destroy_corr(CORR_STRUCT* corr_struct);

uint8_t allocate_corr(CORR_STRUCT* corr_struct, size_t num_pixels){
    corr_struct->deltax = (float *)malloc(sizeof(float)*num_pixels);
    corr_struct->deltay = (float *)malloc(sizeof(float)*num_pixels);
    corr_struct->deltaz = (float *)malloc(sizeof(float)*num_pixels);
    corr_struct->corr_m = (float *)malloc(sizeof(float)*num_pixels);

    if(corr_struct->deltax == NULL || corr_struct->deltay == NULL ||
       corr_struct->deltaz == NULL || corr_struct->corr_m == NULL){
        destroy_corr(corr_struct);
        return false;
    }

    for(size_t i = 0; i < num_pixels; ++i)
    {
        corr_struct->deltax[i] = 0;
        corr_struct->deltay[i] = 0;
        corr_struct->deltaz[i] = 0;
        corr_struct->corr_m[i] = 0;
    }

    return true;
}

void destroy_corr(CORR_STRUCT* corr_struct) {
    if(corr_struct->deltax != NULL){
        free(corr_struct->deltax);
    }
    if(corr_struct->deltay != NULL){
        free(corr_struct->deltay);
    }
    if(corr_struct->deltaz != NULL){
        free(corr_struct->deltaz);
    }
    if(corr_struct->corr_m != NULL){
        free(corr_struct->corr_m);
    }
}

/*-----------------------------------------------------------*/
/*----------------------- Functions -------------------------*/
/*-----------------------------------------------------------*/

bool MatchFeatures_local_distortion_2d(
    Parameters parameters,
    uint8_t **p_base_image,
    uint8_t **p_base_nan_mask,
    int *p_base_image_num_rows,
    int *p_base_image_num_cols,
    uint8_t **p_child_image,
    uint8_t **p_child_nan_mask,
    int *p_child_image_num_rows,
    int *p_child_image_num_cols,
    CORR_STRUCT *corr_struct,
    WarpingMethod warp_image_or_template,
    char *output_dir,
    double homography_max_dist_between_matching_keypoints,
    int32_t child_nan_max_count,
    int32_t base_nan_max_count
);


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


int32_t main (int32_t argc, char **argv)
{
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
        WarpingMethod warp_image_or_template = StrToWarpingMethod(warp_image_or_template_str);
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
    uint8_t *child_image = readPGMToArray(child_image_file, &child_image_num_cols, &child_image_num_rows);
    uint8_t *base_image = readPGMToArray(base_image_file, &base_image_num_cols, &base_image_num_rows);
 
    if(child_image == NULL || base_image == NULL){
        printf("Failed to load images, exiting without output.\n");
        free_image(&child_image);
        free_image(&base_image);
        return EXIT_FAILURE;
    }

    // Load masks
    uint8_t *base_nan_mask;
    if(base_nan_mask_file == NULL){
        printf("Using zero nan mask for base image\n");
        base_nan_mask = malloc(sizeof(uint8_t)*base_image_num_cols*base_image_num_rows);
        if(base_nan_mask == NULL){
            printf("Failure to allocate memory for nan mask\n");
            free_image(&child_image);
            free_image(&base_image);
            free_image(&base_nan_mask);
            return EXIT_FAILURE;
        }
        memset(base_nan_mask,0,sizeof(uint8_t)*base_image_num_cols*base_image_num_rows);
    }else{
        int base_nan_mask_num_rows, base_nan_mask_num_cols;
        base_nan_mask = readPGMToArray(base_nan_mask_file, &base_nan_mask_num_cols, &base_nan_mask_num_rows);
        // Check mask sizes
    if ((base_image_num_cols != base_nan_mask_num_cols)
        || (base_image_num_rows != base_nan_mask_num_rows))
        {
            printf("Image and mask size mismatch, exiting without output.\n");
            free_image(&child_image);
            free_image(&base_image);
            free_image(&base_nan_mask);
            return EXIT_FAILURE;
        }
    }

    uint8_t *child_nan_mask;
    if(child_nan_mask_file == NULL){
        printf("Using zero nan mask for child image\n");
        child_nan_mask = malloc(sizeof(uint8_t)*child_image_num_cols*child_image_num_rows);
        if(child_nan_mask == NULL){
            printf("Failure to allocate memory for nan mask\n");
            free_image(&child_image);
            free_image(&base_image);
            free_image(&base_nan_mask);
            free_image(&child_nan_mask);
            return EXIT_FAILURE;
        }
        memset(child_nan_mask,0,sizeof(uint8_t)*child_image_num_cols*child_image_num_rows);
    }else{
        int child_nan_mask_num_rows, child_nan_mask_num_cols;
        child_nan_mask = readPGMToArray(child_nan_mask_file, &child_nan_mask_num_cols, &child_nan_mask_num_rows);
        if ((child_image_num_cols != child_nan_mask_num_cols)
            || (child_image_num_rows != child_nan_mask_num_rows)){
                printf("Image and mask size mismatch, exiting without output.\n");
                free_image(&child_image);
                free_image(&base_image);
                free_image(&child_nan_mask);
                free_image(&base_nan_mask);
                return EXIT_FAILURE;
            }
    }

    if(child_nan_mask == NULL || base_nan_mask == NULL){
        printf("Failed to load masks, exiting without output.\n");
        free_image(&child_image);
        free_image(&base_image);
        free_image(&child_nan_mask);
        free_image(&base_nan_mask);
        return EXIT_FAILURE;
    }

    // Make sure nan masks are zeros and ones
    for (int32_t i = 0; i < child_image_num_rows * child_image_num_cols; ++i)
    {
        if (child_nan_mask[i] > 0)
        {
            child_nan_mask[i] = 1;
        }
        else
        {
            child_nan_mask[i] = 0;
        }
    }
    for (int32_t i = 0; i < base_image_num_rows * base_image_num_cols; ++i)
    {
        if (base_nan_mask[i] > 0)
        {
            base_nan_mask[i] = 1;
        }
        else
        {
            base_nan_mask[i] = 0;
        }
    }

    // Initialize output correlation
    int child_image_num_pixels = child_image_num_rows * child_image_num_cols;
    CORR_STRUCT corr_struct;
    allocate_corr(&corr_struct, child_image_num_pixels);
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
            corr_struct.deltax[i] = NAN;
            corr_struct.deltay[i] = NAN;
            corr_struct.deltaz[i] = NAN;
            corr_struct.corr_m[i] = NAN;
        }
    }

    size_t output_basepath_size = 512;
    char output_basepath[output_basepath_size];
    snprintf(output_basepath, output_basepath_size, "%.256s/%.256s", output_dir, output_filename_prefix);
    printf("Saving results to %.512s\n", output_basepath);

    FILE *fp;
    size_t buf_size = 544;
    char buf[buf_size];
    snprintf(buf, buf_size, "%.544s%dby%d.raw", output_basepath, child_image_num_cols, child_image_num_rows);
    fp = fopen(buf, "wb");
    fwrite(corr_struct.deltax, sizeof(float), child_image_num_pixels, fp);
    fclose(fp);
    snprintf(buf, buf_size, "%.544s_delta_y_%dby%d.raw", output_basepath, child_image_num_cols, child_image_num_rows);
    fp = fopen(buf, "wb");
    fwrite(corr_struct.deltay, sizeof(float), child_image_num_pixels, fp);
    fclose(fp);

    snprintf(buf, buf_size, "%.544s_delta_z_%dby%d.raw", output_basepath, child_image_num_cols, child_image_num_rows);
    fp = fopen(buf, "wb");
    fwrite(corr_struct.deltaz, sizeof(float), child_image_num_pixels, fp);
    fclose(fp);

    snprintf(buf, buf_size, "%.544s_corr_%dby%d.raw", output_basepath, child_image_num_cols, child_image_num_rows);
    fp = fopen(buf, "wb");
    fwrite(corr_struct.corr_m, sizeof(float), child_image_num_pixels, fp);
    fclose(fp);

    free_image(&child_image);
    free_image(&base_image);
    free_image(&child_nan_mask);
    free_image(&base_nan_mask);
    destroy_corr(&corr_struct);
    if(success){
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}

bool estimateHomographyFromFeatureMatching(
    uint8_t *base_image,
    uint8_t *base_nan_mask,
    int base_image_num_rows,
    int base_image_num_cols,
    uint8_t *child_image,
    uint8_t *child_nan_mask,
    int child_image_num_rows,
    int child_image_num_cols,
    double base2child[3][3],
    char *output_dir,
    double homography_max_dist_between_matching_keypoints
)
{
    bool success_homography = false;
    HomographyMatchMethod best_method;
    uint32_t homography_found_inlier_count_best = 0;
    double homography_arr_best[3][3];

    int method_count = 3;
    HomographyMatchMethod method_arr[3] = {SIFT, ORB};

    for (int i_method = 0; i_method < method_count; ++i_method)
    {
        double homography_arr_local[3][3];
        const char* homography_match_method = HomographyMatchMethodToStr(method_arr[i_method]);
        uint32_t homography_min_inlier_count = 4;
        uint32_t homography_found_inlier_count = 0;
        bool do_draw_homography_image = true;
        
        size_t path_string_length = 256;
        char path_draw_match_image[path_string_length];
        char path_draw_inlier_image[path_string_length];
        snprintf(
            path_draw_match_image,
                 path_string_length,
            "%.200s/homography_match_image_%.16s.jpg",
            output_dir,
            homography_match_method
        );
        snprintf(
            path_draw_inlier_image,
                 path_string_length,
            "%.200s/homography_inlier_image_%.16s.jpg",
            output_dir,
            homography_match_method
        );
        printf("Calculate homography with feature matching method %.16s\n", homography_match_method);
        bool success_homography_local = calc_homography_from_feature_matching(
            base_image,
            base_nan_mask,
            base_image_num_rows,
            base_image_num_cols,
            child_image,
            child_nan_mask,
            child_image_num_rows,
            child_image_num_cols,
            homography_arr_local,
            method_arr[i_method],
            homography_min_inlier_count,
            &homography_found_inlier_count,
            do_draw_homography_image,
            path_draw_match_image,
            path_draw_inlier_image,
            homography_max_dist_between_matching_keypoints
        );
        if (success_homography_local)
        {
            success_homography = true; // this becomes true with at least one local success
            printf(
                "Found %d inliers for homography match method %.16s\n",
                homography_found_inlier_count,
                homography_match_method
            );
            if (homography_found_inlier_count > homography_found_inlier_count_best)
            {
                printf("  New best!\n");
                homography_found_inlier_count_best = homography_found_inlier_count;
                best_method = method_arr[i_method];
                for (int row = 0; row < 3; ++row)
                {
                    for (int col = 0; col < 3; ++col)
                    {
                        homography_arr_best[row][col] = homography_arr_local[row][col];
                    }
                }
            }
        }
    }

    if (success_homography)
    {
        printf(
            "Best homography: %.16s with %d inliers\n",
            HomographyMatchMethodToStr(best_method),
            homography_found_inlier_count_best
        );
        // base2child is the inverse of homography_arr_best
        // such that base2child * base_point (modulo normalizations) is in the child image
        inverseHomography33(homography_arr_best, base2child);
    }
    else
    {
        printf("All homography methods failed\n");
    }

    return success_homography;
}

bool MatchFeatures_local_distortion_2d(
    Parameters parameters,
    uint8_t **p_base_image,
    uint8_t **p_base_nan_mask,
    int *p_base_image_num_rows,
    int *p_base_image_num_cols,
    uint8_t **p_child_image,
    uint8_t **p_child_nan_mask,
    int *p_child_image_num_rows,
    int *p_child_image_num_cols,
    CORR_STRUCT* corr_struct,
    WarpingMethod warp_image_or_template,
    char *output_dir,
    double homography_max_dist_between_matching_keypoints,
    int32_t child_nan_max_count,
    int32_t base_nan_max_count
)
{
    double base2child[3][3];
    // [THP 2024/07/23] The 3D version estimates a homography but here we assume images are pre-aligned
    //estimateHomographyUsingCorners(lmk_base, lmk_child, base2child);
    // Initialize to identity matrix instead
    //for (int i = 0; i < 3; i++)
    //{
    //    for (int j = 0; j < 3; j++)
    //    {
    //        base2child[i][j] = (i == j) ? 1. : 0.;
    //    }
    //}

    // [THP 2024/08/12] Calculate homography from feature matching
    bool success_homography = estimateHomographyFromFeatureMatching(
        *p_base_image,
        *p_base_nan_mask,
        *p_base_image_num_rows,
        *p_base_image_num_cols,
        *p_child_image,
        *p_child_nan_mask,
        *p_child_image_num_rows,
        *p_child_image_num_cols,
        base2child,
        output_dir,
        homography_max_dist_between_matching_keypoints
    );

    if (!success_homography)
    {
        printf(
            "MatchFeatures_local_distortion_2d(): homography failed\n"
        );
        return false;
    }

    // [THP 2024/08/14] Optionally, warp the base image onto the child image instead of warping templates later
    if (warp_image_or_template == WarpingMethod_IMAGE)
    {
        // Warp image directly
        uint8_t *warped_base_image = malloc(
            sizeof(uint8_t) * (*p_child_image_num_rows) * (*p_child_image_num_cols)
        );
        transferImage(
            base2child,
            *p_base_image,
            *p_base_image_num_cols,
            *p_base_image_num_rows,
            warped_base_image,
            *p_child_image_num_cols,
            *p_child_image_num_rows
        );
        // transferImage sets pixels that project out of bounds to zero. These should be
        // marked as not to be used in the mask so as nan. Therefore, warp on the opposite
        // mask then flip again.
        uint8_t *base_not_nan_mask = malloc(
            sizeof(uint8_t) * (*p_base_image_num_rows) * (*p_base_image_num_cols)
        );
        // Stretch to 0-to-255 to make interpolation finer-grained
        for (int i = 0; i < (*p_base_image_num_rows) * (*p_base_image_num_cols); ++i)
        {
            if ((*p_base_nan_mask)[i] == 0)
                base_not_nan_mask[i] = 255;
            else
                base_not_nan_mask[i] = 0;
        }
        // warp not nan mask
        uint8_t *warped_base_mask = malloc(
            sizeof(uint8_t) * (*p_child_image_num_rows) * (*p_child_image_num_cols)
        );
        transferImage(
            base2child,
            base_not_nan_mask,
            *p_base_image_num_cols,
            *p_base_image_num_rows,
            warped_base_mask,
            *p_child_image_num_cols,
            *p_child_image_num_rows
        );
        // flip mask back
        for (int i = 0; i < (*p_child_image_num_rows) * (*p_child_image_num_cols); ++i)
        {
            if (warped_base_mask[i] == 0)
                warped_base_mask[i] = 1;
            else
                warped_base_mask[i] = 0;
        }
        // Reassign variables and free memory
        free_image(p_base_image);
        free_image(p_base_nan_mask);
        *p_base_image = warped_base_image;
        *p_base_nan_mask = warped_base_mask;
        *p_base_image_num_rows = *p_child_image_num_rows;
        *p_base_image_num_cols = *p_child_image_num_cols;
        // Set the homography to identity
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                base2child[i][j] = (i == j) ? 1. : 0.;
            }
        }
        
#ifdef DEBUG
        
        // Write warped image for debugging
        size_t path_size = 256;
        char path_warp_image[path_size];
        snprintf(path_warp_image, path_size, "%.256s/base_image_warped_onto_child_image.pgm", output_dir);
        if(!writePGMFromArray(path_warp_image, warped_base_image, p_child_image_num_cols, p_child_image_num_rows)){
            fprintf(stderr, "Failed to write %.256s\n", path_warp_image);
        }
        
#endif
    }
    else if (warp_image_or_template == WarpingMethod_TEMPLATE)
    {
        // Do nothing
    }
    else
    {
        printf(
            "MatchFeatures_local_distortion_2d(): expected warp_image_or_template to be either image or template but got UNDEFINED\n"
        );
        return false;
    }

    int child_image_num_pixels = *p_child_image_num_rows * *p_child_image_num_cols;
    int base_image_num_pixels = *p_base_image_num_rows * *p_base_image_num_cols;

    // Initialize weight matrix
    float *wt = (float *) malloc(sizeof(float) * child_image_num_pixels);
    if (wt == NULL){
        free(wt);
        printf("MatchFeatures_local_distortion_2d(): memory allocation error\n");
        return false;
    }
    for(int32_t i = 0; i<child_image_num_pixels; i++){
        wt[i] = NAN;
    }

    //Sliding window with step size of BLOCK_SIZE
    for(int32_t row_index = 0; row_index < *p_child_image_num_rows; row_index += BLOCK_SIZE)
    {
        printf("line = %d\n", row_index);
        for(int32_t col_index = 0; col_index < *p_child_image_num_cols ; col_index+= BLOCK_SIZE)
        {
            //Copy the row, col coordinates of the patch into pts1
            //Patch is downsampled by STEP_SIZE
            int32_t size = ((BLOCK_SIZE/STEP_SIZE)+1)*((BLOCK_SIZE/STEP_SIZE)+1);
            double *pts_child_patch = (double *)malloc(sizeof(double)*size*2);
            double *pts_base_patch = (double *)malloc(sizeof(double)*size*2);

            if(pts_child_patch == NULL || pts_base_patch == NULL){
                if(pts_child_patch != NULL) free(pts_child_patch);
                if(wt != NULL) free(wt);
                printf("MatchFeatures_local_distortion_2d(): memory allocation error\n");
                return false;
            }

            int32_t pts_in_block = 0;
            for(int32_t m = row_index; m <= row_index+BLOCK_SIZE; m+=STEP_SIZE)
            {
                for(int32_t n = col_index; n <= col_index+BLOCK_SIZE; n+=STEP_SIZE)
                {

                    pts_child_patch[pts_in_block*2] = n;
                    pts_child_patch[pts_in_block*2+1] = m;
                    pts_in_block++;

                }
            }

            assert(pts_in_block <= size);

            // If the patch has at least 40 valid coordinates, find matching features in base map
            // TODO why 40?
            if(pts_in_block > 40)
            {
                double covs[pts_in_block];
                int32_t num_matched_features = MatchFeaturesWithNaNHandling(
                    parameters,
                    *p_child_image,
                    *p_child_nan_mask,
                    *p_child_image_num_cols,
                    *p_child_image_num_rows,
                    child_nan_max_count,
                    *p_base_image,
                    *p_base_nan_mask,
                    *p_base_image_num_cols,
                    *p_base_image_num_rows,
                    base_nan_max_count,
                    base2child,
                    pts_child_patch,
                    pts_base_patch,
                    covs,
                    pts_in_block
                );

                printf("num_matched_features %d\n", num_matched_features);

                if(num_matched_features > MIN_N_FEATURES)
                {
                    printf("i = %d j = %d\n", row_index, col_index);
                    double patch_homo[3][3];
                    getHomographyFromPoints_RANSAC_frame(pts_child_patch, pts_base_patch, num_matched_features, patch_homo, 3);

                    for(int32_t feature_index = 0; feature_index < num_matched_features; ++feature_index)
                    {
                        double reprojection_err[2];
                        homographyTransfer33(patch_homo, (int32_t)pts_child_patch[feature_index * 2], (int32_t)pts_child_patch[feature_index * 2 + 1], reprojection_err);
                        reprojection_err[0] -= pts_base_patch[feature_index * 2];
                        reprojection_err[1] -= pts_base_patch[feature_index * 2 + 1];
                        double mag_reprojection = sqrt(reprojection_err[0] * reprojection_err[0] + reprojection_err[1] * reprojection_err[1]);

                        //If the feature is a homography inlier, calculate the delta between the feature points
                        //TODO Hrand's version mag_reprojection<50
                        if (mag_reprojection < 5)
                        {
                            double p_child[3], p_base[3], p_delta_map[3], p_delta_world[3];
                            // [THP 2024/07/23] We don't have LMK here, instead init p_child and p_base with pixel coordinates and dummy Z=1
                            //LMK_Col_Row2World(lmk_child,  pts_child_patch[feature_index*2], pts_child_patch[feature_index*2+1], p_child);
                            //LMK_Col_Row2World(lmk_base,   pts_base_patch[feature_index*2], pts_base_patch[feature_index*2+1], p_base);
                            // [THP 2024/08/13] TODO? Diff w.r.t. p_child on base image
                            p_child[0] = pts_child_patch[feature_index * 2];
                            p_child[1] = pts_child_patch[feature_index * 2 + 1];
                            p_child[2] = 1.;
                            p_base[0] = pts_base_patch[feature_index * 2];
                            p_base[1] = pts_base_patch[feature_index * 2 + 1];
                            p_base[2] = 1.;

                            int32_t m = (int32_t) pts_child_patch[feature_index * 2 + 1];
                            int32_t n = (int32_t) pts_child_patch[feature_index * 2];
                            sub3(p_base, p_child, p_delta_world);

                            // [THP 2024/07/23] We also don't have a map to world rotation so set p_delta_map to p_delta_world
                            // Rotate p_delta so that it is in local reference frame
                            //mult331(lmk_child->mapRworld, p_delta_world, p_delta_map);
                            p_delta_map[0] = p_delta_world[0];
                            p_delta_map[1] = p_delta_world[1];
                            p_delta_map[2] = p_delta_world[2];

                            //Fill the delta and corr maps for around the feature pt using a distance weighted contribution from the feature
                            for(int32_t mm = m - FEATURE_WINDOW; mm  <= m+FEATURE_WINDOW; ++mm)
                            {
                                for(int32_t nn = n - FEATURE_WINDOW; nn <= n + FEATURE_WINDOW; ++nn)
                                {
                                    double s = sqrt(((double)mm - pts_child_patch[feature_index * 2 + 1])*((double)mm - pts_child_patch[feature_index * 2 + 1]) + ((double)nn - pts_child_patch[feature_index * 2])*((double)nn - pts_child_patch[feature_index * 2]));
                                    float w = (float)exp(-s);
                                    if (mm > 0 && mm < *p_child_image_num_rows && nn > 0 && nn < *p_child_image_num_cols)
                                    {
                                        corr_struct->deltax[mm*(*p_child_image_num_cols) + nn] += (float)(p_delta_map[0] * (double)w);
                                        corr_struct->deltay[mm*(*p_child_image_num_cols) + nn] += (float)(p_delta_map[1] * (double)w);
                                        corr_struct->deltaz[mm*(*p_child_image_num_cols) + nn] += (float)(p_delta_map[2] * (double)w);
                                        corr_struct->corr_m[mm*(*p_child_image_num_cols) + nn] += (float)(covs[feature_index] * (double)w);
                                        if(isnan(wt[mm*(*p_child_image_num_cols) + nn]))
                                            wt[mm*(*p_child_image_num_cols) + nn] = w;
                                        else
                                            wt[mm*(*p_child_image_num_cols) + nn] += w;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if(pts_child_patch != NULL) free(pts_child_patch);
            if(pts_base_patch != NULL) free(pts_base_patch);
        }
    }

    for (int32_t i = 0; i < child_image_num_pixels; ++i)
    {
        if (!isnan(wt[i]))
        {
            corr_struct->deltay[i] = corr_struct->deltay[i] / wt[i];
            corr_struct->deltax[i] = corr_struct->deltax[i] / wt[i];
            corr_struct->deltaz[i] = corr_struct->deltaz[i] / wt[i];
            corr_struct->corr_m[i] = corr_struct->corr_m[i] / wt[i];
        }else{
            corr_struct->deltay[i] = NAN;
            corr_struct->deltax[i] = NAN;
            corr_struct->deltaz[i] = NAN;
            corr_struct->corr_m[i] = NAN;
        }
    }

    //TODO this looks like an outlier filter. How was 300 choosen?
    for (int32_t i = 0; i < child_image_num_pixels; ++i)
    {

        if (fabs(corr_struct->deltay[i]) > 300)  corr_struct->deltay[i] = NAN;
        if (fabs(corr_struct->deltax[i]) > 300)  corr_struct->deltax[i] = NAN;
        if (fabs(corr_struct->deltaz[i]) > 300)  corr_struct->deltaz[i] = NAN;

    }

    free(wt);

    return true;
}
