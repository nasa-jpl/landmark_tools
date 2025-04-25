/**
 * \file feature_matching_2d.c
 * \author Tu-Hoa Pham
 * \date 2024
 * 
 * \brief Implementation of 2D feature matching functions for image comparison
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

#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "landmark_tools/opencv_tools/feature_matching_2d.h"
#include "landmark_tools/feature_tracking/feature_match.h"
#include "landmark_tools/math/homography_util.h"
#include "landmark_tools/opencv_tools/homography_estimation.h"
#include "landmark_tools/opencv_tools/opencv_image_io.h"
#include "math/mat3/mat3.h"                                 // for mult331
#include "landmark_tools/utils/safe_string.h"

WarpingMethod StrToWarpingMethod(const char* str){
    if(str != NULL){
        if(strncmp(str, "image", strlen(str)) == 0){
            return WarpingMethod_IMAGE;
        }else if(strncmp(str, "template", strlen(str)) == 0){
            return WarpingMethod_TEMPLATE;
        }
    }
    SAFE_FPRINTF(stderr, 512, "No WarpingMethod defined that corresponds to %s. Valid methods are \"image\" and \"template\"/n",
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

bool process_matched_feature_2d(
    double child_col,
    double child_row,
    double base_col,
    double base_row,
    double correlation,
    CorrelationResults* results,
    float* weights,
    int32_t num_cols,
    int32_t num_rows,
    int32_t feature_influence_window
) {
    // Convert 2D image coordinates to 3D homogeneous coordinates with Z=1
    double p_child[3], p_base[3], p_delta_map[3];
    p_child[0] = child_col;  // x coordinate
    p_child[1] = child_row;  // y coordinate
    p_child[2] = 1.;         // homogeneous coordinate
    p_base[0] = base_col;
    p_base[1] = base_row;
    p_base[2] = 1.;

    // Compute displacement vector in map coordinates
    // In 2D case, map coordinates are the same as world coordinates
    // since we don't have a map-to-world rotation transformation
    p_delta_map[0] = p_base[0] - p_child[0];  // x displacement
    p_delta_map[1] = p_base[1] - p_child[1];  // y displacement
    p_delta_map[2] = 0.;                      // z displacement (always 0 in 2D case)

    // Convert to integer coordinates for array indexing
    int32_t row = (int32_t)child_row;
    int32_t col = (int32_t)child_col;

    // Update correlation results in a window around the feature point
    // using Gaussian-weighted contributions
    for (int32_t m = row - feature_influence_window; m <= row + feature_influence_window; ++m) {
        for (int32_t n = col - feature_influence_window; n <= col + feature_influence_window; ++n) {
            // Check if the point is within image boundaries
            if (m >= 0 && m < num_rows && n >= 0 && n < num_cols) {
                // Compute Gaussian weight based on Euclidean distance from feature point
                double distance = sqrt(((double)m - child_row) * ((double)m - child_row) + 
                                     ((double)n - child_col) * ((double)n - child_col));
                float weight = (float)exp(-distance);

                // Update correlation results with weighted contributions
                int32_t index = m * num_cols + n;
                results->delta_x[index] += (float)(p_delta_map[0] * weight);
                results->delta_y[index] += (float)(p_delta_map[1] * weight);
                results->delta_z[index] += (float)(p_delta_map[2] * weight);
                results->correlation[index] += (float)(correlation * weight);

                // Update weight matrix for later normalization
                if (isnan(weights[index])) {
                    weights[index] = weight;  // First contribution to this point
                } else {
                    weights[index] += weight; // Accumulate weights for multiple features
                }
            }
        }
    }

    return true;
}

bool MatchFeatures_local_distortion_2d(
    Parameters parameters,
    uint8_t **base_image,
    uint8_t **base_nan_mask,
    int *base_image_num_rows,
    int *base_image_num_cols,
    uint8_t **child_image,
    uint8_t **child_nan_mask,
    int *child_image_num_rows,
    int *child_image_num_cols,
    CorrelationResults *results,
    int warp_image_or_template,
    char *output_dir,
    double homography_max_dist_between_matching_keypoints,
    int32_t child_nan_max_count,
    int32_t base_nan_max_count
) {
    // Initialize homography matrix
    double base2child[3][3];
    
    // Estimate initial homography between images using feature matching
    if( !estimateHomographyFromFeatureMatching(
        *base_image,
        *base_nan_mask,
        *base_image_num_rows,
        *base_image_num_cols,
        *child_image,
        *child_nan_mask,
        *child_image_num_rows,
        *child_image_num_cols,
        base2child,
        output_dir,
        homography_max_dist_between_matching_keypoints
    )) {
        printf("MatchFeatures_local_distortion_2d(): homography estimation failed\n");
        return false;
    }

    // [THP 2024/08/14] Optionally warp the base image onto the child image
    if (warp_image_or_template == WarpingMethod_IMAGE) {
        // Warp image directly
        uint8_t *warped_base_image = malloc(
            sizeof(uint8_t) * (*child_image_num_rows) * (*child_image_num_cols)
        );
        transferImage(
            base2child,
            *base_image,
            *base_image_num_cols,
            *base_image_num_rows,
            warped_base_image,
            *child_image_num_cols,
            *child_image_num_rows
        );
        // transferImage sets pixels that project out of bounds to zero. These should be
        // marked as not to be used in the mask so as nan. Therefore, warp on the opposite
        // mask then flip again.
        uint8_t *base_not_nan_mask = malloc(
            sizeof(uint8_t) * (*base_image_num_rows) * (*base_image_num_cols)
        );
        // Stretch to 0-to-255 to make interpolation finer-grained
        for (int i = 0; i < (*base_image_num_rows) * (*base_image_num_cols); ++i)
        {
            if ((*base_nan_mask)[i] == 0)
                base_not_nan_mask[i] = 255;
            else
                base_not_nan_mask[i] = 0;
        }
        // warp not nan mask
        uint8_t *warped_base_mask = malloc(
            sizeof(uint8_t) * (*child_image_num_rows) * (*child_image_num_cols)
        );
        transferImage(
            base2child,
            base_not_nan_mask,
            *base_image_num_cols,
            *base_image_num_rows,
            warped_base_mask,
            *child_image_num_cols,
            *child_image_num_rows
        );
        // flip mask back
        for (int i = 0; i < (*child_image_num_rows) * (*child_image_num_cols); ++i)
        {
            if (warped_base_mask[i] == 0)
                warped_base_mask[i] = 1;
            else
                warped_base_mask[i] = 0;
        }
        // Reassign variables and free memory
        free_image(base_image);
        free_image(base_nan_mask);
        *base_image = warped_base_image;
        *base_nan_mask = warped_base_mask;
        *base_image_num_rows = *child_image_num_rows;
        *base_image_num_cols = *child_image_num_cols;
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
        snprintf(path_warp_image, path_size, "%s/base_image_warped_onto_child_image.pgm", output_dir);
        if(!writePGMFromArray(path_warp_image, warped_base_image, *child_image_num_cols, *child_image_num_rows)){
            SAFE_FPRINTF(stderr, 512, "Failed to write %s\n", path_warp_image);
        }
#endif
    } else if (warp_image_or_template == WarpingMethod_TEMPLATE) {
        // Do nothing - templates will be warped during matching
    } else {
        printf("MatchFeatures_local_distortion_2d(): invalid warping method\n");
        return false;
    }

    // Initialize arrays for correlation results
    int child_image_num_pixels = *child_image_num_rows * *child_image_num_cols;

    // Initialize weight matrix for normalization
    float *weights = (float *) malloc(sizeof(float) * child_image_num_pixels);
    if (weights == NULL) {
        printf("MatchFeatures_local_distortion_2d(): memory allocation error\n");
        return false;
    }
    for(int32_t i = 0; i < child_image_num_pixels; i++) {
        weights[i] = NAN;
    }

    // Process image in sliding windows
    for(int32_t row_index = 0; row_index < *child_image_num_rows; row_index += parameters.sliding.block_size) {
        SAFE_PRINTF(512, "Processing row %d of %d\n", row_index, *child_image_num_rows);
        
        for(int32_t col_index = 0; col_index < *child_image_num_cols; col_index += parameters.sliding.block_size) {
            // Extract feature points at regular intervals within the current block
            int32_t size = ((parameters.sliding.block_size/parameters.sliding.step_size)+1) * 
                          ((parameters.sliding.block_size/parameters.sliding.step_size)+1);
            double *child_points = (double *)malloc(sizeof(double)*size*2);
            double *base_points = (double *)malloc(sizeof(double)*size*2);

            if(child_points == NULL || base_points == NULL) {
                if(child_points != NULL) free(child_points);
                if(weights != NULL) free(weights);
                printf("MatchFeatures_local_distortion_2d(): memory allocation error\n");
                return false;
            }

            // Sample points at regular intervals within the block
            int32_t pts_in_block = 0;
            for(int32_t m = row_index; m <= row_index+parameters.sliding.block_size; m+=parameters.sliding.step_size) {
                for(int32_t n = col_index; n <= col_index+parameters.sliding.block_size; n+=parameters.sliding.step_size) {
                    child_points[pts_in_block*2] = n;
                    child_points[pts_in_block*2+1] = m;
                    pts_in_block++;
                }
            }

            double correlation_values[pts_in_block];
            int32_t num_matched_features = MatchFeaturesWithNaNHandling(
                parameters,
                *child_image,
                *child_nan_mask,
                *child_image_num_cols,
                *child_image_num_rows,
                child_nan_max_count,
                *base_image,
                *base_nan_mask,
                *base_image_num_cols,
                *base_image_num_rows,
                base_nan_max_count,
                base2child,
                child_points,
                base_points,
                correlation_values,
                pts_in_block
            );

            SAFE_PRINTF(512, "Found %d matched features in window\n", num_matched_features);

            // Process matched features if enough were found
            if(num_matched_features > parameters.sliding.min_n_features) {
                // Estimate local homography for the matched features
                double patch_homo[3][3];
                getHomographyFromPoints_RANSAC_frame(child_points, base_points, num_matched_features, patch_homo, 3);

                // Process each matched feature
                for(int32_t feature_index = 0; feature_index < num_matched_features; ++feature_index) {
                    // Check if feature is an inlier by computing reprojection error
                    double reprojection_err[2];
                    homographyTransfer33(patch_homo, 
                                        (int32_t)child_points[feature_index * 2], 
                                        (int32_t)child_points[feature_index * 2 + 1], 
                                        reprojection_err);
                    reprojection_err[0] -= base_points[feature_index * 2];
                    reprojection_err[1] -= base_points[feature_index * 2 + 1];
                    double mag_reprojection = sqrt(reprojection_err[0] * reprojection_err[0] + 
                                                    reprojection_err[1] * reprojection_err[1]);

                    // Process inlier features
                    if (mag_reprojection < parameters.sliding.reprojection_threshold) {
                        process_matched_feature_2d(
                            child_points[feature_index * 2],
                            child_points[feature_index * 2 + 1],
                            base_points[feature_index * 2],
                            base_points[feature_index * 2 + 1],
                            correlation_values[feature_index],
                            results,
                            weights,
                            *child_image_num_cols,
                            *child_image_num_rows,
                            parameters.sliding.feature_influence_window
                        );
                    }
                }
            }

            // Clean up memory for this block
            if(child_points != NULL) free(child_points);
            if(base_points != NULL) free(base_points); 
        }
    }

    // Normalize correlation results by accumulated weights
    for (int32_t i = 0; i < child_image_num_pixels; ++i) {
        if (!isnan(weights[i])) {
            results->delta_y[i] = results->delta_y[i] / weights[i];
            results->delta_x[i] = results->delta_x[i] / weights[i];
            results->delta_z[i] = results->delta_z[i] / weights[i];
            results->correlation[i] = results->correlation[i] / weights[i];
        } else {
            results->delta_y[i] = NAN;
            results->delta_x[i] = NAN;
            results->delta_z[i] = NAN;
            results->correlation[i] = NAN;
        }
    }

    // Filter out outliers (points with large displacements)
    for (int32_t i = 0; i < child_image_num_pixels; ++i) {
        if (fabs(results->delta_y[i]) > parameters.sliding.max_delta_map) results->delta_y[i] = NAN;
        if (fabs(results->delta_x[i]) > parameters.sliding.max_delta_map) results->delta_x[i] = NAN;
        if (fabs(results->delta_z[i]) > parameters.sliding.max_delta_map) results->delta_z[i] = NAN;
    }

    // Clean up
    free(weights);

    return true;
} 