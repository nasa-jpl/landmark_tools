/** 
 * \file landmark_registration.c
 * \author Yang Cheng
 * 
 * \brief Implementation of landmark registration functions
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

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "landmark_tools/landmark_registration/landmark_registration.h"
#include "landmark_tools/feature_selection/int_forstner_extended.h"
#include "landmark_tools/feature_tracking/corr_image_long.h"
#include "landmark_tools/image_io/image_utils.h"
#include "landmark_tools/image_io/imagedraw.h"
#include "landmark_tools/math/homography_util.h"
#include "landmark_tools/landmark_util/landmark.h"
#include "landmark_tools/landmark_util/estimate_homography.h"
#include "landmark_tools/math/math_utils.h"
#include "landmark_tools/math/point_line_plane_util.h"
#include "math/mat3/mat3.h"

// Constants for landmark registration
#define GRID_DIVISION_FACTOR 20
#define STRBUF_SIZE 256
#define DEBUG_BUF_SIZE 128

/**
 * \brief Visualizes the warped landmark data for debugging purposes
 * 
 * This function performs the following operations when DEBUG is enabled:
 * 1. Estimates homography between base and child landmarks
 * 2. Warps the base landmark's surface reflectance map (SRM) and elevation data
 * 3. Saves the warped SRM as a PNG image
 * 4. Saves the warped elevation data as a raw binary file
 * 
 * \param base_landmark Pointer to base landmark structure
 * \param child_landmark Pointer to child landmark structure
 * \param visualization_buffer Pointer to temporary image buffer for visualization
 */
static void visualize_warped_landmark(const LMK *base_landmark, const LMK *child_landmark, uint8_t *visualization_buffer)
{
    // Estimate homography between landmarks
    double base_to_child_homography[3][3], child_to_base_homography[3][3];
    estimateHomographyUsingCorners(base_landmark, child_landmark, base_to_child_homography);
    inverseHomography33(base_to_child_homography, child_to_base_homography);

    // Allocate memory for warped elevation data
    float *warped_elevation = (float *)malloc(sizeof(float)*base_landmark->num_pixels);
    if (warped_elevation == NULL) {
        printf("visualize_warped_landmark(): Failed to allocate memory for warped elevation data\n");
        return;
    }

    // Initialize buffers
    memset(visualization_buffer, 0, sizeof(uint8_t)*base_landmark->num_pixels);
    memset(warped_elevation, 0, sizeof(float)*base_landmark->num_pixels);
    
    // Warp base landmark data using estimated homography
    for(int row = 0; row < base_landmark->num_rows; row++) {
        for(int col = 0; col < base_landmark->num_cols; col++) {
            // Calculate warped coordinates
            int warped_row = (child_to_base_homography[1][0]*col + child_to_base_homography[1][1]*row + child_to_base_homography[1][2]) / 
                           (child_to_base_homography[2][0]*col + child_to_base_homography[2][1]*row + child_to_base_homography[2][2]);
            int warped_col = (child_to_base_homography[0][0]*col + child_to_base_homography[0][1]*row + child_to_base_homography[0][2]) / 
                           (child_to_base_homography[2][0]*col + child_to_base_homography[2][1]*row + child_to_base_homography[2][2]);
            
            // Check if warped coordinates are within bounds
            if(warped_row >= 0 && warped_row < base_landmark->num_rows && 
               warped_col >= 0 && warped_col < base_landmark->num_cols) {
                int source_idx = row * base_landmark->num_cols + col;
                int target_idx = warped_row * base_landmark->num_cols + warped_col;
                visualization_buffer[target_idx] = base_landmark->srm[source_idx];
                warped_elevation[target_idx] = base_landmark->ele[source_idx];
            }
        }
    }

    // Save warped surface reflectance map
    write_channel_separated_image("warped_srm.png", visualization_buffer, 
                                 base_landmark->num_cols, base_landmark->num_rows, 1);

    // Save warped elevation data
    char elevation_filename[DEBUG_BUF_SIZE];
    snprintf(elevation_filename, DEBUG_BUF_SIZE, "warped_ele_float_%dby%d.raw", 
             base_landmark->num_cols, base_landmark->num_rows);
    
    FILE *elevation_file = fopen(elevation_filename, "wb");
    if (elevation_file != NULL) {
        fwrite(warped_elevation, sizeof(float), base_landmark->num_cols*base_landmark->num_rows, elevation_file);
        fclose(elevation_file);
    } else {
        printf("visualize_warped_landmark(): Failed to open elevation file for writing\n");
    }

    free(warped_elevation);
}

/**
 * \brief Helper function to free all allocated memory
 * 
 * \param lmk_child Pointer to child landmark structure
 * \param lmk_base Pointer to base landmark structure
 * \param match_pair_base Pointer to base match pairs array
 * \param match_pair_child Pointer to child match pairs array
 * \param template Pointer to template array
 * \param feature_coord Pointer to feature coordinates array
 * \param feature_strength Pointer to feature strength array
 * \param tmpimg Pointer to temporary image buffer
 * \param pts_3d_child Pointer to 3D child points array
 * \param pts_3d_base Pointer to 3D base points array
 */
static void cleanup_memory(LMK *lmk_child, LMK *lmk_base,
                         double *match_pair_base, double *match_pair_child,
                         uint8_t *template, int64_t (*feature_coord)[2],
                         float *feature_strength, uint8_t *tmpimg,
                         double *pts_3d_child, double *pts_3d_base)
{
    free_lmk(lmk_child);
    free_lmk(lmk_base);
    if(match_pair_base != NULL) free(match_pair_base);
    if(match_pair_child != NULL) free(match_pair_child);
    if(template != NULL) free(template);
    if(feature_coord != NULL) free(feature_coord);
    if(feature_strength != NULL) free(feature_strength);
    if(tmpimg != NULL) free(tmpimg);
    if(pts_3d_child != NULL) free(pts_3d_child);
    if(pts_3d_base != NULL) free(pts_3d_base);
}

int32_t RegisterLandmarks(Parameters parameters, const char *base_landmark_filename, const char *child_landmark_filename)
{
    // Initialize landmark structures
    LMK lmk_child = {0};
    LMK lmk_base = {0};
    Read_LMK(child_landmark_filename, &lmk_child);
    Read_LMK(base_landmark_filename, &lmk_base);
    
    // Extract matching parameters
    int32_t correlation_window_size = parameters.matching.correlation_window_size;
    int32_t half_correlation_window = parameters.matching.correlation_window_size / 2;
    int32_t search_window_size = parameters.matching.search_window_size;
    int32_t half_search_window = parameters.matching.search_window_size / 2;
    
    // Calculate grid dimensions for feature matching
    int32_t grid_cols = lmk_child.num_cols / GRID_DIVISION_FACTOR;
    int32_t grid_rows = lmk_child.num_rows / GRID_DIVISION_FACTOR;
    
    // Allocate memory for feature matching
    double *base_feature_coords = (double *)malloc(sizeof(double)*grid_cols*grid_rows);
    double *child_feature_coords = (double *)malloc(sizeof(double)*grid_cols*grid_rows);
    uint8_t *correlation_template = (uint8_t *)malloc(sizeof(uint8_t)*correlation_window_size*correlation_window_size);
    
    if(base_feature_coords==NULL || child_feature_coords == NULL || correlation_template == NULL){
        printf("RegisterLandmarks(): memory allocation error\n");
        cleanup_memory(&lmk_child, &lmk_base, base_feature_coords, child_feature_coords, 
                      correlation_template, NULL, NULL, NULL, NULL, NULL);
        return 0;
    }
    
    // Calculate initial transformation between landmarks
    double base_to_child_transform[3][3] = {0};
    double child_to_base_transform[3][3] = {0};
    mult333(lmk_child.mapRworld, lmk_base.worldRmap, child_to_base_transform);
    inverseHomography33(child_to_base_transform, base_to_child_transform); 
    
    // Detect Forstner features in child landmark
    int32_t num_detected_features = 0;
    int64_t (*feature_pixel_coords)[2] = malloc(sizeof(int64_t[parameters.detector.num_features * 2]));
    float *feature_quality_scores = malloc(sizeof(float) * parameters.detector.num_features);
    int_forstner_nbest_even_distribution(lmk_child.srm, lmk_child.num_cols, lmk_child.num_rows, 
                                        10, 10, lmk_child.num_cols-20, lmk_child.num_rows-20, 
                                        parameters.detector.window_size, parameters.detector.num_features, 
                                        &num_detected_features, feature_pixel_coords, feature_quality_scores, 
                                        (int32_t)parameters.detector.min_dist_feature);
    
    uint8_t *visualization_buffer = NULL;
    #ifdef DEBUG
    // Allocate temporary image for visualization
    visualization_buffer = (uint8_t *)malloc(sizeof(uint8_t)*lmk_base.num_pixels);
    if(visualization_buffer == NULL){
        printf("RegisterLandmarks(): memory allocation error\n");
        cleanup_memory(&lmk_child, &lmk_base, base_feature_coords, child_feature_coords, 
                      correlation_template, feature_pixel_coords, feature_quality_scores, NULL, NULL, NULL);
        return 0;
    }
    memcpy(visualization_buffer, lmk_base.srm, sizeof(uint8_t)*lmk_base.num_pixels);
    #endif

    // Match features between landmarks
    int32_t num_matched_pairs = 0;
    for (int32_t feature_idx = 0; feature_idx < num_detected_features; ++feature_idx)
    {
        int64_t *child_feature_pixel = feature_pixel_coords[feature_idx];
        // Skip child features on NaN elevation
        if (isnan(lmk_child.ele[child_feature_pixel[1] * lmk_child.num_cols + child_feature_pixel[0]]))
        {
            continue;
        }
        
        // Transform child feature coordinates to base landmark frame
        double base_feature_coord[2];
        homographyTransfer33(child_to_base_transform, (double)child_feature_pixel[0], (double)child_feature_pixel[1], base_feature_coord);
        
        // Check if transformed coordinates are within base landmark bounds
        if(base_feature_coord[0] > 0 && base_feature_coord[0] < lmk_base.num_cols && 
           base_feature_coord[1] > 0 && base_feature_coord[1] < lmk_base.num_rows)
        {
            int32_t center_col = (int32_t)base_feature_coord[0];
            int32_t center_row = (int32_t)base_feature_coord[1];
            double subpixel_offset_col = base_feature_coord[0] - center_col;
            double subpixel_offset_row = base_feature_coord[1] - center_row;
            
            // Extract correlation template around feature
            int32_t template_idx = 0;
            for (int32_t row_offset = -half_correlation_window; row_offset <= half_correlation_window; ++row_offset)
            {
                for (int32_t col_offset = -half_correlation_window; col_offset <= half_correlation_window; ++col_offset)
                {
                    double base_pt[2];
                    double child_pt[2];
                    base_pt[0] = center_col + col_offset;
                    base_pt[1] = center_row + row_offset;
                    homographyTransfer33D(child_to_base_transform, base_pt, child_pt);
                    double interpolated_value = Interpolate_LMK_SRM(&lmk_child, child_pt[0], child_pt[1]);
                    correlation_template[template_idx] = (int32_t)interpolated_value;
                    template_idx++;
                }
            }
            
            // Define search window bounds
            int64_t search_left = center_col - half_search_window;
            int64_t search_top = center_row - half_search_window;
            if (search_top < 0) search_top = 0;
            if (search_left < 0) search_left = 0;
            int64_t search_width = search_window_size;
            int64_t search_height = search_window_size;
            if (search_left + search_window_size > lmk_base.num_cols)  search_width = lmk_base.num_cols - search_left - 1;
            if (search_top + search_window_size > lmk_base.num_rows)  search_height = lmk_base.num_rows - search_top - 1;
            
            // Perform correlation matching
            double best_match_row = -1.0, best_match_col = -1.0, best_correlation_score = 0.0;
            double correlation_covariance[3] = {0};
            bool correlation_success = corimg_long(correlation_template,
                                correlation_window_size,
                                0, 0,
                                correlation_window_size, correlation_window_size,
                                lmk_base.srm,
                                lmk_base.num_cols,
                                search_left, search_top,
                                search_width, search_height,
                                &best_match_row, &best_match_col, &best_correlation_score, correlation_covariance);
            
            // If correlation successful and score above threshold, store match
            if (correlation_success && best_correlation_score > parameters.matching.min_correlation)
            {
                double base_center[2];
                double child_center[2];
                base_center[0] = base_feature_coord[0];
                base_center[1] = base_feature_coord[1];
                homographyTransfer33D(child_to_base_transform, base_center, child_center);
                
                // Store matched feature coordinates
                child_feature_coords[num_matched_pairs * 2] = child_center[0];
                child_feature_coords[num_matched_pairs * 2 + 1] = child_center[1];
                // Adjust coordinates for subpixel alignment
                base_feature_coords[num_matched_pairs * 2] = best_match_col + subpixel_offset_col;
                base_feature_coords[num_matched_pairs * 2 + 1] = best_match_row + subpixel_offset_row;
                
                #ifdef DEBUG
                // Draw match visualization
                DrawArrow(visualization_buffer, lmk_base.num_cols, lmk_base.num_rows,
                          base_feature_coord[0], base_feature_coord[1],
                          best_match_col,
                          best_match_row,
                          255, 3);
                #endif
                num_matched_pairs++;
            }
            else
            {
                #ifdef DEBUG
                // Draw unmatched feature
                DrawFeatureBlock(visualization_buffer, lmk_base.num_cols, lmk_base.num_rows, 
                               base_feature_coord[0], base_feature_coord[1], 255, 5);
                #endif
            }
        }
    }
    
#ifdef DEBUG
    write_channel_separated_image("matched_point.png", visualization_buffer, lmk_base.num_cols, lmk_base.num_rows, 1);
#endif
    
    // Calculate homography from matched feature pairs
    double estimated_homography[3][3] = {0};
    int32_t num_homography_inliers = getHomographyFromPoints_RANSAC_frame(child_feature_coords, base_feature_coords, 
                                                                         num_matched_pairs, estimated_homography, 
                                                                         parameters.sliding.reprojection_threshold);
    
    // Allocate memory for 3D point clouds
    double *child_3d_points = (double *)malloc(sizeof(double)*num_matched_pairs * 3);
    double *base_3d_points = (double *)malloc(sizeof(double)*num_matched_pairs * 3);
    if(child_3d_points == NULL || base_3d_points == NULL){
        printf("RegisterLandmarks(): memory allocation error\n");
        cleanup_memory(&lmk_child, &lmk_base, base_feature_coords, child_feature_coords, 
                      correlation_template, feature_pixel_coords, feature_quality_scores, visualization_buffer,
                      child_3d_points, base_3d_points);
        return 0;
    }
    
#ifdef DEBUG
    memcpy(visualization_buffer, lmk_base.srm, sizeof(uint8_t)*lmk_base.num_pixels);
#endif
    
    // Convert homography inliers to 3D point clouds
    int32_t num_3d_points = 0;
    for (int32_t pair_idx = 0; pair_idx < num_matched_pairs; ++pair_idx)
    {
        double child_pt[2] = {0}, projected_base_pt[2] = {0};
        child_pt[0] = child_feature_coords[pair_idx * 2 + 0];
        child_pt[1] = child_feature_coords[pair_idx * 2 + 1];
        homographyTransfer33(estimated_homography, child_pt[0], child_pt[1], projected_base_pt);
        
        double base_pt[2] = {0}, reprojection_error[2] = {0};
        base_pt[0] = base_feature_coords[pair_idx * 2 + 0];
        base_pt[1] = base_feature_coords[pair_idx * 2 + 1];
        reprojection_error[0] = projected_base_pt[0] - base_pt[0];
        reprojection_error[1] = projected_base_pt[1] - base_pt[1];
        double reprojection_error_magnitude = sqrt(reprojection_error[0] * reprojection_error[0] + 
                                                 reprojection_error[1] * reprojection_error[1]);
        
        if (reprojection_error_magnitude < parameters.sliding.reprojection_threshold)
        {
#ifdef DEBUG
            DrawArrow(visualization_buffer, lmk_base.num_cols, lmk_base.num_rows,
                      child_pt[0], child_pt[1],
                      base_pt[0], base_pt[1],
                      255, 3);
#endif
            
            if (LMK_Col_Row2World(&lmk_child, child_pt[0], child_pt[1], &child_3d_points[num_3d_points * 3]) &
                LMK_Col_Row2World(&lmk_base, base_pt[0], base_pt[1], &base_3d_points[num_3d_points * 3]))
            {
                ++num_3d_points;
            }
        }
    }

#ifdef DEBUG
    printf("# of RANSAC inliers %d\n", num_3d_points);
    write_channel_separated_image("RANSAC_inlier.png", visualization_buffer, lmk_base.num_cols, lmk_base.num_rows, 1);
#endif
    
    // Check for sufficient inliers, otherwise later call to Point_Clouds_rot_T_RANSAC will segfault
    if(num_3d_points == 0){
        printf("RegisterLandmarks(): no homography inliers within reprojection threshold\n");
        cleanup_memory(&lmk_child, &lmk_base, base_feature_coords, child_feature_coords, 
                      correlation_template, feature_pixel_coords, feature_quality_scores, visualization_buffer,
                      child_3d_points, base_3d_points);
        return 0;
    }
    
    // Refine transformation using 3D point clouds
    double refined_rotation[3][3] = {0};
    double refined_translation[3] = {0};

    if (!Point_Clouds_rot_T_RANSAC(child_3d_points, base_3d_points, num_3d_points, 
                                                         refined_rotation, refined_translation, 
                                                         parameters.sliding.min_n_features))
    {
        printf("RegisterLandmarks(): Point_Clouds_rot_T_RANSAC did not find enough inliers\n");
        cleanup_memory(&lmk_child, &lmk_base, base_feature_coords, child_feature_coords, 
                      correlation_template, feature_pixel_coords, feature_quality_scores, visualization_buffer,
                      child_3d_points, base_3d_points);
        return 0;
    }
    prt33(refined_rotation);
    prt3(refined_translation);
    
    // Update landmark structure with refined transformation
    double world_to_map_rotation[3][3] = {0}, transformed_point[3] = {0};
    mult333(refined_rotation, lmk_child.worldRmap, world_to_map_rotation);
    mult331(refined_rotation, lmk_child.anchor_point, transformed_point);
    add3(transformed_point, refined_translation, lmk_child.anchor_point);
    
    mult331(refined_rotation, lmk_child.map_normal_vector, transformed_point);
    copy3(transformed_point, lmk_child.map_normal_vector);
    copy33(world_to_map_rotation, lmk_child.worldRmap);
    trans33(world_to_map_rotation, lmk_child.mapRworld);
    normalpoint2plane(lmk_child.map_normal_vector, lmk_child.anchor_point, lmk_child.map_plane_params);
    
    // Save registered landmark
    char output_filename[STRBUF_SIZE];
    snprintf(output_filename, STRBUF_SIZE, "%.200s_registered.lmk", child_landmark_filename);
    Write_LMK(output_filename, &lmk_child);
    
    #ifdef DEBUG
    visualize_warped_landmark(&lmk_base, &lmk_child, visualization_buffer);
    #endif
    
    // Clean up allocated memory
    cleanup_memory(&lmk_child, &lmk_base, base_feature_coords, child_feature_coords, 
                  correlation_template, feature_pixel_coords, feature_quality_scores, visualization_buffer,
                  child_3d_points, base_3d_points);
    return 1;
} 