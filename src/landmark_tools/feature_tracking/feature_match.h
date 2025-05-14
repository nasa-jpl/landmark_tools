/**
 * \file   feature_match.h
 * \author Yang Cheng
 * \brief  Dense patch-based correlation matching
 * 
 * \section updates Update History
 * - Created: March 2000
 * - Updated: 
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

#ifndef _LANDMARK_TOOLS_FEATURE_MATCH_H_
#define _LANDMARK_TOOLS_FEATURE_MATCH_H_

#include <stddef.h>                               // for size_t
#include <stdint.h>                               // for uint8_t, int32_t
#include <stdint.h>                               // for uint8_t, int32_t

#include "landmark_tools/feature_tracking/parameters.h"  // for FTP
#include "landmark_tools/landmark_util/landmark.h"      // for LMK
#include "landmark_tools/feature_tracking/correlation_results.h"  // for CorrelationResults

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \brief Apply sliding window correlation matcher for each pixel in template_image to find closest match in search_image.
 * 
 *  Any pixel which has a non-data neighbor will be skipped.
 * 
 * \param[in] parameters configuration settings
 * \param[in] template_image First image to extract templates from
 * \param[in] template_mask No-data mask for template_image. 0 if coordinate contains data, 1 otherwise
 * \param[in] template_cols Width of template_image
 * \param[in] template_rows Height of template_image
 * \param[in] search_image Second image to search for matches in
 * \param[in] search_mask No-data mask for search_image. 0 if coordinate contains data, 1 otherwise
 * \param[in] search_cols Width of search_image
 * \param[in] search_rows Height of search_image
 * \param[in] initial_homography Homography between images based on lat,lon coordinates of corner points
 * \param[in] template_points Row, col coordinates from template_image to use for feature matching
 * \param[out] matched_points Best matches in search_image for template_points
 * \param[out] correlation_values Correlation values for each match
 * \param[in] num_points Number of points in template_points
 */
int32_t MatchFeaturesOnly(
    Parameters parameters,
    uint8_t *template_image,
    uint8_t *template_mask,
    size_t template_cols,
    size_t template_rows,
    uint8_t *search_image,
    uint8_t *search_mask,
    size_t search_cols,
    size_t search_rows,
    double initial_homography[3][3],
    double *template_points,
    double *matched_points,
    double *correlation_values,
    int32_t num_points
);

/**
 * \brief Apply sliding window correlation matcher with configurable NaN handling
 * 
 * This function performs the same correlation matching as MatchFeaturesOnly but allows
 * configuration of how many NaN/masked pixels are allowed in both the template and search windows.
 * 
 * \param[in] parameters configuration settings
 * \param[in] template_image First image to extract templates from
 * \param[in] template_mask No-data mask for template_image. 0 if coordinate contains data, 1 otherwise 
 * \param[in] template_cols Width of template_image
 * \param[in] template_rows Height of template_image
 * \param[in] max_nan_count_template Maximum allowed masked points in template window (negative to ignore)
 * \param[in] search_image Second image to search for matches in
 * \param[in] search_mask No-data mask for search_image. 0 if coordinate contains data, 1 otherwise
 * \param[in] search_cols Width of search_image
 * \param[in] search_rows Height of search_image
 * \param[in] max_nan_count_search Maximum allowed masked points in search window (negative to ignore)
 * \param[in] initial_homography Homography between images based on lat,lon coordinates of corner points
 * \param[in] template_points Row, col coordinates from template_image to use for feature matching
 * \param[out] matched_points Best matches in search_image for template_points
 * \param[out] correlation_values Correlation values for each match
 * \param[in] num_points Number of points in template_points
 */
int32_t MatchFeaturesWithNaNHandling(
    Parameters parameters,
    uint8_t *template_image,
    uint8_t *template_mask,
    size_t template_cols,
    size_t template_rows,
    int32_t max_nan_count_template,
    uint8_t *search_image,
    uint8_t *search_mask,
    size_t search_cols,
    size_t search_rows,
    int32_t max_nan_count_search,
    double initial_homography[3][3],
    double *template_points,
    double *matched_points,
    double *correlation_values,
    int32_t num_points
);

/**
 * \brief Match features between two landmarks with local distortion handling
 * 
 * This function performs dense patch-based correlation matching between two landmarks,
 * taking into account local distortions. It uses a sliding window approach where
 * each window is processed independently to handle local variations.
 * 
 * \param[in] parameters Configuration parameters for feature matching
 * \param[in] base_landmark Base landmark to compare against
 * \param[in] child_landmark Child landmark being compared
 * \param[out] results Structure to store correlation results
 * \param[in] max_nan_count_base Maximum allowed NaN values in base landmark window
 * \param[in] max_nan_count_child Maximum allowed NaN values in child landmark window
 * \return true if matching successful, false otherwise
 */
bool MatchFeaturesWithLocalDistortion(
    Parameters parameters,
    LMK *base_landmark,
    LMK *child_landmark,
    CorrelationResults *results,
    int32_t max_nan_count_base,
    int32_t max_nan_count_child
);

/**
 * \brief Process a matched feature point and update the correlation results
 * 
 * This function takes a matched feature point and updates the correlation results
 * by distributing the delta values and correlation values within the feature's
 * influence window using a Gaussian weight function.
 *
 * \param[in] child_landmark The child landmark containing the feature point
 * \param[in] base_landmark The base landmark containing the matched point
 * \param[in] child_col Column coordinate in child landmark
 * \param[in] child_row Row coordinate in child landmark
 * \param[in] base_col Column coordinate in base landmark
 * \param[in] base_row Row coordinate in base landmark
 * \param[in] covariance Correlation value for the matched feature
 * \param[out] results Correlation results structure to update
 * \param[out] weights Array of weights for each pixel
 * \param[in] num_cols Number of columns in the landmark
 * \param[in] num_rows Number of rows in the landmark
 * \param[in] feature_influence_window Size of the window around the feature point for distributing values
 * \return true if successful, false otherwise
 */
bool process_matched_feature(
    const LMK* child_landmark,
    const LMK* base_landmark,
    double child_col,
    double child_row,
    double base_col,
    double base_row,
    double covariance,
    CorrelationResults* results,
    float* weights,
    int32_t num_cols,
    int32_t num_rows,
    int32_t feature_influence_window
);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LANDMARK_TOOLS_FEATURE_MATCH_H_
