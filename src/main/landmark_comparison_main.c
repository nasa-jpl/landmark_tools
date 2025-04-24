/** 
 * \file landmark_comparison_main.c
 * \author Yang Cheng
 * 
 * \brief Compare two landmark files using dense patch-based correlation matching
 * 
 * This program compares two landmark files by performing dense patch-based correlation matching.
 * It outputs delta maps (x, y, z) and correlation maps that show the differences between
 * the landmarks. The comparison is done in a sliding window fashion, where each window
 * is processed independently to handle local distortions.
 * 
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
#include <stdbool.h>                                        // for false, bool
#include <stdint.h>                                         // for int32_t
#include <stdio.h>                                          // for printf, NULL
#include <stdlib.h>                                         // for free, malloc

#include "landmark_tools/image_io/image_utils.h"             // for load_cha...
#include "landmark_tools/feature_tracking/feature_match.h"  // for MatchFeat...
#include "landmark_tools/feature_tracking/parameters.h"            // for Parameters, Read...
#include "landmark_tools/math/homography_util.h"
#include "landmark_tools/landmark_util/landmark.h"          // for free_lmk
#include "landmark_tools/landmark_util/estimate_homography.h"
#include "landmark_tools/utils/parse_args.h"                // for m_getarg
#include "math/mat3/mat3.h"                                 // for mult331
#include "landmark_tools/feature_tracking/correlation_results.h"  // for CorrelationResults

/**
 * \brief Display usage information and exit
 * 
 * Prints the command line usage information for the landmark comparison tool
 * and exits with failure status.
 */
void show_usage_and_exit()
{
    printf("Compare landmark files using a dense patch-based correlation matcher\n");
    printf("Usage for landmark_compare:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -l1   <lmk_filepath> - First landmark file to compare\n");
    printf("    -l2   <lmk_filepath> - Second landmark file to compare\n");
    printf("    -o    <output_prefix> - Prefix for output files\n");
    printf("    -c    <parameters_config_filepath> - Configuration file for matching parameters\n");
    printf("    -nan_max_count1     <-1 to ignore, 0 or greater to filter> - Max NaN count for first landmark\n");
    printf("    -nan_max_count2     <-1 to ignore, 0 or greater to filter> - Max NaN count for second landmark\n");
    exit(EXIT_FAILURE);
}

/**
 * \brief Main function for landmark comparison
 * 
 * This function:
 * 1. Parses command line arguments
 * 2. Loads and validates input landmarks
 * 3. Performs feature matching between landmarks
 * 4. Outputs delta maps and correlation results
 *
 * \return EXIT_SUCCESS on success, EXIT_FAILURE on error
 */
int32_t main(int32_t argc, char **argv)
{
    // Parse command line arguments
    char *child_landmark_path = NULL;    // Path to first landmark file
    char *base_landmark_path = NULL;     // Path to second landmark file
    char *output_prefix = NULL;          // Prefix for output files
    char *parameters_path = NULL;        // Path to parameters config file
    char *nan_max_count1_str = NULL;     // String value for first landmark's max NaN count
    char *nan_max_count2_str = NULL;     // String value for second landmark's max NaN count
    
    argc--;
    argv++;
    
    if (argc == 0) show_usage_and_exit();
    
    while (argc > 0)
    {
        if (argc == 1) show_usage_and_exit();
        if ((m_getarg(argv, "-l1", &child_landmark_path, CFO_STRING) != 1) &&
            (m_getarg(argv, "-l2", &base_landmark_path, CFO_STRING) != 1) &&
            (m_getarg(argv, "-o", &output_prefix, CFO_STRING) != 1) &&
            (m_getarg(argv, "-c", &parameters_path, CFO_STRING) != 1) &&
            (m_getarg(argv, "-nan_max_count1", &nan_max_count1_str, CFO_STRING) != 1) &&
            (m_getarg(argv, "-nan_max_count2", &nan_max_count2_str, CFO_STRING) != 1))
            show_usage_and_exit();
        
        argc -= 2;
        argv += 2;
    }
    
    // Load and validate parameters
    Parameters parameters;
    load_default_parameters(&parameters);
    if (parameters_path == NULL) {
        printf("No parameter file provided. Using defaults.\n");
    } else {
        int32_t success = read_parameterfile(parameters_path, &parameters);
        if (!success) {
            printf("Cannot load %.256s\n", parameters_path);
            return EXIT_FAILURE;
        }
    }
    print_parameters(parameters);
    
    // Load landmarks
    LMK child_landmark = {0};
    LMK base_landmark = {0};
    int success = Read_LMK(child_landmark_path, &child_landmark);
    success &= Read_LMK(base_landmark_path, &base_landmark);
    if (!success) {
        free_lmk(&child_landmark);
        free_lmk(&base_landmark);
        return EXIT_FAILURE;
    }

    // Parse NaN count parameters
    int32_t max_nan_count_child = -1; // Default: do not check for NaN in child landmark
    int32_t max_nan_count_base = 0;   // Default: do not allow any NaN in base landmark
    if (nan_max_count1_str != NULL) {
        max_nan_count_child = atoi(nan_max_count1_str);
    }
    if (nan_max_count2_str != NULL) {
        max_nan_count_base = atoi(nan_max_count2_str);
    }
    
#ifdef DEBUG
    write_channel_separated_image("basemap.png", base_landmark.srm, base_landmark.num_cols, base_landmark.num_rows, 1);
    write_channel_separated_image("childmap.png", child_landmark.srm, child_landmark.num_cols, child_landmark.num_rows, 1);
#endif

    // Perform feature matching
    CorrelationResults results;
    allocate_correlation_results(&results, child_landmark.num_pixels);
    success &= MatchFeaturesWithLocalDistortion(
        parameters,
        &base_landmark,
        &child_landmark,
        &results,
        max_nan_count_base,
        max_nan_count_child
    );
    
    if (!success) {
        printf("Failed to match features. Exiting without output.\n");
        free_lmk(&child_landmark);
        free_lmk(&base_landmark);
        destroy_correlation_results(&results);
        return EXIT_FAILURE;
    }
    
    // Save results to output files
    printf("Saving results to %.256s\n", output_prefix);
    
    FILE *fp;
    size_t buf_size = 256;
    char buf[buf_size];
    
    // Save delta x map
    snprintf(buf, buf_size, "%.200s_delta_x_%dby%d.raw", output_prefix, child_landmark.num_cols, child_landmark.num_rows);
    fp = fopen(buf, "wb");
    fwrite(results.delta_x, sizeof(float), child_landmark.num_pixels, fp);
    fclose(fp);
    
    // Save delta y map
    snprintf(buf, buf_size, "%.200s_delta_y_%dby%d.raw", output_prefix, child_landmark.num_cols, child_landmark.num_rows);
    fp = fopen(buf, "wb");
    fwrite(results.delta_y, sizeof(float), child_landmark.num_pixels, fp);
    fclose(fp);
    
    // Save delta z map
    snprintf(buf, buf_size, "%.200s_delta_z_%dby%d.raw", output_prefix, child_landmark.num_cols, child_landmark.num_rows);
    fp = fopen(buf, "wb");
    fwrite(results.delta_z, sizeof(float), child_landmark.num_pixels, fp);
    fclose(fp);
    
    // Save correlation map
    snprintf(buf, buf_size, "%.200s_corr_%dby%d.raw", output_prefix, child_landmark.num_cols, child_landmark.num_rows);
    fp = fopen(buf, "wb");
    fwrite(results.correlation, sizeof(float), child_landmark.num_pixels, fp);
    fclose(fp);
    
    // Cleanup
    free_lmk(&child_landmark);
    free_lmk(&base_landmark);
    destroy_correlation_results(&results);
    
    return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
