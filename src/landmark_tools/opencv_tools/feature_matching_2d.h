/**
 * \file feature_matching_2d.h
 * \author Tu-Hoa Pham
 * \date 2024
 * 
 * \brief 2D feature matching functions for image comparison
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

#ifndef FEATURE_MATCHING_2D_H
#define FEATURE_MATCHING_2D_H

#include <stdbool.h>
#include <stdint.h>

#include "landmark_tools/feature_tracking/parameters.h"
#include "landmark_tools/feature_tracking/correlation_results.h"

typedef enum {
    WarpingMethod_IMAGE,
    WarpingMethod_TEMPLATE,
    WarpingMethod_UNDEFINED
} WarpingMethod;

WarpingMethod StrToWarpingMethod(const char* str);

void free_image(uint8_t **p_image);

/**
 * \brief Process a matched feature point in 2D image space
 * 
 * This function processes a single matched feature point between two images, computing the
 * displacement between corresponding points and updating the correlation results with
 * Gaussian-weighted contributions in a local window around the feature.
 * 
 * \param[in] child_col Column coordinate in child image
 * \param[in] child_row Row coordinate in child image
 * \param[in] base_col Column coordinate in base image
 * \param[in] base_row Row coordinate in base image
 * \param[in] correlation Correlation value for the match
 * \param[in,out] results Correlation results structure to update
 * \param[in,out] weights Weight matrix for normalization
 * \param[in] num_cols Number of columns in the image
 * \param[in] num_rows Number of rows in the image
 * \param[in] feature_influence_window Size of influence window around feature
 * \return true if feature was processed successfully, false otherwise
 */
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
);

/**
 * \brief Match features between two images with local distortion handling in 2D
 * 
 * This function performs feature matching between a base and child image while handling
 * local distortions. It can optionally warp the base image onto the child image or
 * handle warping during template matching.
 * 
 * \param[in] parameters Feature tracking parameters
 * \param[in,out] base_image Base image data
 * \param[in,out] base_nan_mask Base image NaN mask
 * \param[in,out] base_image_num_rows Number of rows in base image
 * \param[in,out] base_image_num_cols Number of columns in base image
 * \param[in,out] child_image Child image data
 * \param[in,out] child_nan_mask Child image NaN mask
 * \param[in,out] child_image_num_rows Number of rows in child image
 * \param[in,out] child_image_num_cols Number of columns in child image
 * \param[out] results Correlation results structure
 * \param[in] warp_image_or_template Warping method to use
 * \param[in] output_dir Output directory for debug files
 * \param[in] homography_max_dist_between_matching_keypoints Maximum distance between matching keypoints
 * \param[in] child_nan_max_count Maximum number of NaN pixels allowed in child image window
 * \param[in] base_nan_max_count Maximum number of NaN pixels allowed in base image window
 * \return true if matching was successful, false otherwise
 */
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
);

#endif // FEATURE_MATCHING_2D_H 