/************************************************
 * \file opencv_feature_matching.html
 * \author Tu-Hoa Pham
 * \brief Calculate homography using opencv feature matching
 *
 *  \copyright Copyright 2024 California Institute of Technology
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
 * 
 ***********************************************/

#ifndef _OPENCV_FEATURE_MATCHING_H_
#define _OPENCV_FEATURE_MATCHING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "landmark_tools/opencv_tools/homography_match_method.h"

/**
 \brief TODO
 
 \param[] base_image 
 \param[] base_nan_mask 
 \param[] base_image_num_rows 
 \param[] base_image_num_cols 
 \param[] child_image 
 \param[] child_nan_mask 
 \param[] child_image_num_rows 
 \param[] child_image_num_cols 
 \param[] homography_arr 
 \param[] homography_match_method 
 \param[] homography_min_inlier_count 
 \param[] p_homography_found_inlier_count 
 \param[] do_draw_homography_image 
 \param[] path_draw_match_image 
 \param[] path_draw_inlier_image 
 \param[] homography_max_dist_between_matching_keypoints 
 \return true 
 \return false 
*/
bool calc_homography_from_feature_matching(
    uint8_t *base_image,
    uint8_t *base_nan_mask,
    int base_image_num_rows,
    int base_image_num_cols,
    uint8_t *child_image,
    uint8_t *child_nan_mask,
    int child_image_num_rows,
    int child_image_num_cols,
    double homography_arr[3][3],
    HomographyMatchMethod homography_match_method,
    uint32_t homography_min_inlier_count,
    uint32_t *p_homography_found_inlier_count,
    bool do_draw_homography_image,
    char *path_draw_match_image,
    char *path_draw_inlier_image,
    double homography_max_dist_between_matching_keypoints
);

#ifdef __cplusplus
}
#endif

#endif //_OPENCV_FEATURE_MATCHING_H_
