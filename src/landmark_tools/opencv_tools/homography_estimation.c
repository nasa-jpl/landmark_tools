/**
 * \file homography_estimation.c
 * \brief Implementation of homography estimation functions
 * 
 * This file implements functions for estimating homography transformations between
 * images using feature matching.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "landmark_tools/opencv_tools/homography_estimation.h"
#include "landmark_tools/opencv_tools/homography_match_method.h"
#include "landmark_tools/opencv_tools/opencv_feature_matching.h"
#include "landmark_tools/math/homography_util.h"

/**
 * \brief Estimate homography between two images using feature matching
 * 
 * This function estimates a homography transformation between a base image and a
 * child image using feature matching. It tries multiple feature matching methods
 * and selects the best one based on the number of inliers.
 * 
 * \param[in] base_image Base image data
 * \param[in] base_nan_mask Mask for NaN pixels in base image
 * \param[in] base_image_num_rows Number of rows in base image
 * \param[in] base_image_num_cols Number of columns in base image
 * \param[in] child_image Child image data
 * \param[in] child_nan_mask Mask for NaN pixels in child image
 * \param[in] child_image_num_rows Number of rows in child image
 * \param[in] child_image_num_cols Number of columns in child image
 * \param[out] base2child Estimated homography matrix (3x3)
 * \param[in] output_dir Directory for saving debug outputs
 * \param[in] homography_max_dist_between_matching_keypoints Maximum allowed distance between matching keypoints
 * \return true if homography estimation was successful, false otherwise
 */
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
) {
    bool success_homography = false;
    HomographyMatchMethod best_method;
    uint32_t homography_found_inlier_count_best = 0;
    double homography_arr_best[3][3];

    // Try multiple feature matching methods
    int method_count = 3;
    HomographyMatchMethod method_arr[3] = {SIFT, ORB};

    for (int i_method = 0; i_method < method_count; ++i_method) {
        double homography_arr_local[3][3];
        const char* homography_match_method = HomographyMatchMethodToStr(method_arr[i_method]);
        uint32_t homography_min_inlier_count = 4;
        uint32_t homography_found_inlier_count = 0;
        bool do_draw_homography_image = true;
        
        // Prepare output paths for debug images
        size_t path_string_length = 256;
        char path_draw_match_image[path_string_length];
        char path_draw_inlier_image[path_string_length];
        snprintf(
            path_draw_match_image,
            path_string_length,
            "%s/homography_match_image_%s.jpg",
            output_dir,
            homography_match_method
        );
        snprintf(
            path_draw_inlier_image,
            path_string_length,
            "%s/homography_inlier_image_%s.jpg",
            output_dir,
            homography_match_method
        );

        printf("Calculate homography with feature matching method %.16s\n", homography_match_method);
        
        // Try to estimate homography with current method
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

        if (success_homography_local) {
            success_homography = true; // At least one method succeeded
            printf(
                "Found %d inliers for homography match method %.16s\n",
                homography_found_inlier_count,
                homography_match_method
            );

            // Update best method if this one has more inliers
            if (homography_found_inlier_count > homography_found_inlier_count_best) {
                printf("  New best!\n");
                homography_found_inlier_count_best = homography_found_inlier_count;
                best_method = method_arr[i_method];
                for (int row = 0; row < 3; ++row) {
                    for (int col = 0; col < 3; ++col) {
                        homography_arr_best[row][col] = homography_arr_local[row][col];
                    }
                }
            }
        }
    }

    if (success_homography) {
        printf(
            "Best homography: %.16s with %d inliers\n",
            HomographyMatchMethodToStr(best_method),
            homography_found_inlier_count_best
        );
        // base2child is the inverse of homography_arr_best
        // such that base2child * base_point (modulo normalizations) is in the child image
        inverseHomography33(homography_arr_best, base2child);
    } else {
        printf("All homography methods failed\n");
    }

    return success_homography;
} 