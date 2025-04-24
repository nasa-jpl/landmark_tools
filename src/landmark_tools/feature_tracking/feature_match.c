/**
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "landmark_tools/data_interpolation/interpolate_data.h"
#include "landmark_tools/feature_tracking/corr_image_long.h"
#include "landmark_tools/feature_tracking/feature_match.h"
#include "landmark_tools/feature_tracking/parameters.h"
#include "landmark_tools/math/homography_util.h"
#include "landmark_tools/landmark_util/landmark.h"
#include "landmark_tools/landmark_util/estimate_homography.h"

#include "img/utils/imgutils.h"
#include "math/mat3/mat3.h"

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
)
{
    // Default settings for NaN handling:
    // - Reject any masked-out points in template image
    // - Ignore mask for search image
    int32_t max_nan_count_template = 0;
    int32_t max_nan_count_search = -1;
    
    return MatchFeaturesWithNaNHandling(
        parameters,
        template_image,
        template_mask,
        template_cols,
        template_rows,
        max_nan_count_template,
        search_image,
        search_mask,
        search_cols,
        search_rows,
        max_nan_count_search,
        initial_homography,
        template_points,
        matched_points,
        correlation_values,
        num_points
    );
}

/**
 * \brief Extract template window from image with NaN handling
 * 
 * \param[in] image Source image
 * \param[in] mask Image mask (0 for valid data, 1 for NaN)
 * \param[in] cols Image width
 * \param[in] rows Image height
 * \param[in] center_x Center x coordinate
 * \param[in] center_y Center y coordinate
 * \param[in] template_size Size of template window
 * \param[out] template_buffer Buffer to store template window
 * \param[out] nan_count Number of NaN pixels found
 * \return true if template extraction successful, false otherwise
 */
static bool extract_template_window(
    uint8_t *image,
    uint8_t *mask,
    size_t cols,
    size_t rows,
    int32_t center_x,
    int32_t center_y,
    int32_t template_size,
    uint8_t *template_buffer,
    int32_t *nan_count
) {
    int32_t half_template = template_size / 2;
    *nan_count = 0;
    
    for(int32_t row = -half_template, buf_idx = 0; row <= half_template; ++row) {
        for(int32_t col = -half_template; col <= half_template; ++col) {
            int32_t img_x = center_x + col;
            int32_t img_y = center_y + row;
            
            if(img_x >= 0 && img_x < cols && img_y >= 0 && img_y < rows) {
                size_t img_idx = img_y * cols + img_x;
                if(mask != NULL && mask[img_idx] != 0) {
                    (*nan_count)++;
                }
                template_buffer[buf_idx] = image[img_idx];
            } else {
                (*nan_count)++;
                template_buffer[buf_idx] = 0;
            }
            buf_idx++;
        }
    }
    return true;
}

/**
 * \brief Check for NaN pixels in search window
 * 
 * \param[in] mask Image mask
 * \param[in] cols Image width
 * \param[in] left Left boundary of search window
 * \param[in] top Top boundary of search window
 * \param[in] width Window width
 * \param[in] height Window height
 * \return Number of NaN pixels found
 */
static int32_t count_nan_in_search_window(
    uint8_t *mask,
    size_t cols,
    int32_t left,
    int32_t top,
    size_t width,
    size_t height
) {
    if(mask == NULL) return 0;
    
    int32_t nan_count = 0;
    for(int32_t row = top; row < top + height; ++row) {
        for(int32_t col = left; col < left + width; ++col) {
            size_t idx = row * cols + col;
            if(mask[idx] != 0) nan_count++;
        }
    }
    return nan_count;
}

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
)
{
    // Extract correlation parameters
    int32_t template_size = parameters.matching.correlation_window_size;
    int32_t search_win_size = parameters.matching.search_window_size;
    int32_t half_search_win = search_win_size / 2;
    
    // Allocate memory for feature matching
    uint8_t *template_buffer = (uint8_t *)malloc(sizeof(uint8_t) * template_size * template_size);
    
    if(template_buffer == NULL) {
        printf("MatchFeaturesWithNaNHandling(): memory allocation error\n");
        return 0;
    }
    
    // Compute inverse homography for coordinate transformation
    double inv_homography[3][3];
    inverseHomography33(initial_homography, inv_homography);
    
    // Process each feature point
    int32_t num_matches = 0;
    for(int32_t point_idx = 0; point_idx < num_points; ++point_idx) {
        // Transform point coordinates using initial homography
        double transformed_coords[2];
        homographyTransfer33D(initial_homography, &template_points[point_idx*2], transformed_coords);
        
        // Convert to integer coordinates with sub-pixel offset
        int32_t center_x = (int32_t)(transformed_coords[0] + 0.5);
        int32_t center_y = (int32_t)(transformed_coords[1] + 0.5);
        double subpixel_x = transformed_coords[0] - center_x;
        double subpixel_y = transformed_coords[1] - center_y;
        
        // Extract template window
        int32_t nan_count;
        if(!extract_template_window(
            template_image,
            template_mask,
            template_cols,
            template_rows,
            center_x,
            center_y,
            template_size,
            template_buffer,
            &nan_count
        )) {
            continue;
        }
        
        // Skip if too many NaN pixels in template
        if((max_nan_count_template >= 0) && (nan_count > max_nan_count_template)) {
            continue;
        }
        
        // Define search window bounds in search image
        int32_t search_left = center_x - half_search_win;
        if(search_left < 0) search_left = 0;
        
        int32_t search_top = center_y - half_search_win;
        if(search_top < 0) search_top = 0;
        
        size_t search_width = search_win_size;
        size_t search_height = search_win_size;
        if(search_left + search_win_size > search_cols) search_width = search_cols - search_left;
        if(search_top + search_win_size > search_rows) search_height = search_rows - search_top;
        
        // Check for masked pixels in search window
        if((search_mask != NULL) && (max_nan_count_search >= 0)) {
            int32_t search_nan_count = count_nan_in_search_window(
                search_mask,
                search_cols,
                search_left,
                search_top,
                search_width,
                search_height
            );
            if(search_nan_count > max_nan_count_search) {
                continue;
            }
        }
        
        // Perform correlation matching
        double best_correlation, best_row, best_col, correlation_cov[3];
        if(corimg_long(
            template_buffer,
            template_size,
            0, 0,
            template_size, template_size,
            search_image,
            search_cols,
            search_left, search_top,
            search_width, search_height,
            &best_row, &best_col, &best_correlation, correlation_cov
        )) {
            // Check if correlation is above threshold
            if(best_correlation > parameters.matching.min_correlation) {
                // Store matched point coordinates and correlation value
                template_points[num_matches*2] = template_points[point_idx*2];
                template_points[num_matches*2+1] = template_points[point_idx*2+1];
                
                matched_points[num_matches*2] = best_col + subpixel_x;
                matched_points[num_matches*2+1] = best_row + subpixel_y;
                correlation_values[num_matches] = best_correlation;
                num_matches++;
            }
        }
    }
    
    // Cleanup
    free(template_buffer);
    
    return num_matches;
}

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
) {
    // Convert points to world coordinates and compute delta in map frame
    double child_world[3], base_world[3], delta_world[3], delta_map[3];
    
    if (!LMK_Col_Row2World(child_landmark, child_col, child_row, child_world) ||
        !LMK_Col_Row2World(base_landmark, base_col, base_row, base_world)) {
        return false;
    }
    
    // Compute delta in world coordinates and convert to map frame
    sub3(child_world, base_world, delta_world);
    mult331(child_landmark->mapRworld, delta_world, delta_map);
    
    // Update maps with Gaussian-weighted contributions
    int32_t row = (int32_t)child_row;
    int32_t col = (int32_t)child_col;
    
    for (int32_t m = row - feature_influence_window; m <= row + feature_influence_window; ++m) {
        for (int32_t n = col - feature_influence_window; n <= col + feature_influence_window; ++n) {
            if (m >= 0 && m < num_rows && n >= 0 && n < num_cols) {
                // Compute Gaussian weight based on distance from feature
                double distance = sqrt(((double)m - row) * ((double)m - row) + 
                                     ((double)n - col) * ((double)n - col));
                float weight = (float)exp(-distance);
                
                // Update maps with weighted contributions
                int32_t index = m * num_cols + n;
                results->delta_x[index] += (float)(delta_map[0] * weight);
                results->delta_y[index] += (float)(delta_map[1] * weight);
                results->delta_z[index] += (float)(delta_map[2] * weight);
                results->correlation[index] += (float)(covariance * weight);
                
                // Update weights
                if (isnan(weights[index])) {
                    weights[index] = weight;
                } else {
                    weights[index] += weight;
                }
            }
        }
    }
    
    return true;
}

bool MatchFeaturesWithLocalDistortion(
    Parameters parameters,
    LMK *base_landmark,
    LMK *child_landmark,
    CorrelationResults *results,
    int32_t max_nan_count_base,
    int32_t max_nan_count_child
) {
    // Estimate initial homography between landmarks using corner points
    double base2child[3][3];
    estimateHomographyUsingCorners(base_landmark, child_landmark, base2child);
    
    // Allocate memory for weights and NaN masks
    float *weights = (float *)malloc(sizeof(float) * child_landmark->num_pixels);
    uint8_t* child_nan_mask = (uint8_t*)malloc(sizeof(uint8_t) * child_landmark->num_pixels);
    uint8_t* base_nan_mask = (uint8_t*)malloc(sizeof(uint8_t) * base_landmark->num_pixels);
    
    if (weights == NULL || child_nan_mask == NULL || base_nan_mask == NULL) {
        if (child_nan_mask) free(child_nan_mask);
        if (base_nan_mask) free(base_nan_mask);
        if (weights) free(weights);
        printf("MatchFeaturesWithLocalDistortion(): memory allocation error\n");
        return false;
    }
        
    // Initialize weights and create NaN masks
    for (int32_t i = 0; i < child_landmark->num_pixels; i++) {
        weights[i] = NAN;
        
        // Create NaN mask for child landmark
        if (isnan(child_landmark->ele[i])) {
            child_nan_mask[i] = 1;
        } else {
            child_nan_mask[i] = 0;
        }
        
        // Create NaN mask for base landmark
        if (isnan(base_landmark->ele[i])) {
            base_nan_mask[i] = 1;
        } else {
            base_nan_mask[i] = 0;
        }
    }
    
    // Process landmarks in blocks
    for (int32_t row_index = 0; row_index < child_landmark->num_rows; row_index += parameters.sliding.block_size) {
        printf("Processing row %d of %d\n", row_index, child_landmark->num_rows);
        
        for (int32_t col_index = 0; col_index < child_landmark->num_cols; col_index += parameters.sliding.block_size) {
            // Extract feature points at STEP_SIZE intervals within the current block
            int32_t size = ((parameters.sliding.block_size / parameters.sliding.step_size) + 1) * ((parameters.sliding.block_size / parameters.sliding.step_size) + 1);
            double *child_points = (double *)malloc(sizeof(double) * size * 2);
            double *base_points = (double *)malloc(sizeof(double) * size * 2);
            
            if (child_points == NULL || base_points == NULL) {
                if (child_points != NULL) free(child_points);
                if (base_points != NULL) free(base_points);
                printf("MatchFeaturesWithLocalDistortion(): memory allocation error\n");
                return false;
            }
            
            // Fill arrays with coordinates at STEP_SIZE intervals
            int32_t num_points = 0;
            for (int32_t m = row_index; m <= row_index + parameters.sliding.block_size; m += parameters.sliding.step_size) {
                for (int32_t n = col_index; n <= col_index + parameters.sliding.block_size; n += parameters.sliding.step_size) {
                    child_points[num_points * 2] = n;
                    child_points[num_points * 2 + 1] = m;
                    num_points++;
                }
            }
            
            double covariances[num_points];
            int32_t num_matched_features = MatchFeaturesWithNaNHandling(
                parameters,
                child_landmark->srm,
                child_nan_mask,
                child_landmark->num_cols,
                child_landmark->num_rows,
                max_nan_count_child,
                base_landmark->srm,
                base_nan_mask,
                base_landmark->num_cols,
                base_landmark->num_rows,
                max_nan_count_base,
                base2child,
                child_points,
                base_points,
                covariances,
                num_points
            );
            
            printf("Found %d matched features in window\n", num_matched_features);
            
            if (num_matched_features > parameters.sliding.min_n_features) {
                // Compute local homography for matched features
                double local_homography[3][3];
                getHomographyFromPoints_RANSAC_frame(child_points, base_points, num_matched_features, local_homography, 3);
                
                // Process each matched feature
                for (int32_t feature_index = 0; feature_index < num_matched_features; ++feature_index) {
                    // Check reprojection error
                    double reprojection_error[2];
                    homographyTransfer33(local_homography, 
                                        (int32_t)child_points[feature_index * 2], 
                                        (int32_t)child_points[feature_index * 2 + 1], 
                                        reprojection_error);
                    
                    reprojection_error[0] -= base_points[feature_index * 2];
                    reprojection_error[1] -= base_points[feature_index * 2 + 1];
                    double error_magnitude = sqrt(reprojection_error[0] * reprojection_error[0] + 
                                                reprojection_error[1] * reprojection_error[1]);
                    
                    // Process inlier features
                    if (error_magnitude < parameters.sliding.reprojection_threshold) {
                        process_matched_feature(
                            child_landmark,
                            base_landmark,
                            child_points[feature_index * 2],
                            child_points[feature_index * 2 + 1],
                            base_points[feature_index * 2],
                            base_points[feature_index * 2 + 1],
                            covariances[feature_index],
                            results,
                            weights,
                            child_landmark->num_cols,
                            child_landmark->num_rows,
                            parameters.sliding.feature_influence_window
                        );
                    }
                }
            }
            
            
            free(child_points);
            free(base_points);
        }
    }
    
    // Normalize results by weights
    for (int32_t i = 0; i < child_landmark->num_pixels; ++i) {
        if (!isnan(weights[i])) {
            results->delta_x[i] /= weights[i];
            results->delta_y[i] /= weights[i];
            results->delta_z[i] /= weights[i];
            results->correlation[i] /= weights[i];
        } else {
            results->delta_x[i] = NAN;
            results->delta_y[i] = NAN;
            results->delta_z[i] = NAN;
            results->correlation[i] = NAN;
        }
    }
    
    // Filter outliers (points with large deltas)
    for (int32_t i = 0; i < child_landmark->num_pixels; ++i) {
        if (fabs(results->delta_y[i]) > parameters.sliding.max_delta_map) results->delta_y[i] = NAN;
        if (fabs(results->delta_x[i]) > parameters.sliding.max_delta_map) results->delta_x[i] = NAN;
        if (fabs(results->delta_z[i]) > parameters.sliding.max_delta_map) results->delta_z[i] = NAN;
    }
    
    // Cleanup
    free(weights);
    free(child_nan_mask);
    free(base_nan_mask);
    
    return true;
}
