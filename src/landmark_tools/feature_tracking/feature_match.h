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
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \brief Apply sliding window correlation matcher for each pixel in img1 to find closest match in img2.
 * 
 *  Any pixel which has a non-data neighbor will be skipped.
 * 
 * \param[in] parameters configuration settings
 * \param[in] img1
 * \param[in] mask1
 * \param[in] cols1 width of img1
 * \param[in] rows1 height of img1
 * \param[in] img2
 * \param[in] mask2 unused
 * \param[in] cols2 width of img2
 * \param[in] rows2 height of img2
 * \param[in] init_homo homography between img1 and img2 based on lat,lon coordinates of corner points
 * \param[in] pts_img1 row, col coordinates from img1 to use for feature matching
 * \param[out] pts_img2 Best match in img2 for `pts_img1`
 * \param[out] covs  Covariance of matches at `pts_img1`
 * \param[in] np  Number of points in `pts_img1`
 */
int32_t MatchFeaturesOnly(Parameters parameters, uint8_t *img1, uint8_t *mask1, size_t cols1, size_t rows1,
                          uint8_t *img2, uint8_t *mask2, size_t cols2, size_t rows2,
                          double init_homo[3][3], double *pts_img1, double *pts_img2,
                          double *covs, int32_t np);

/**
 * \brief Apply sliding window correlation matcher for each pixel in img1 to find closest match in img2
 * \param[in] parameters configuration settings
 * \param[in] img1
 * \param[in] mask1 no data mask for `img1`. 0 if coordinate contains data in `img1`, 1 otherwise 
 * \param[in] cols1 width of `img1`
 * \param[in] rows1 height of `img1`
 * \param[in] no_data_max1 a pixel will be skipped by the matcher if the surrounding patch in `img1` contains greater than `no_data_max1` masked points. (set to negative to ignore)
 * \param[in] img2
 * \param[in] mask2 no data mask for `img2`.  0 if coordinate contains data in `img1`, 1 otherwise
 * \param[in] cols2 width of `img2`
 * \param[in] rows2 height of `img2`
 * \param[in] no_data_max2 a pixel will be skipped by the matcher if the surrounding patch in `img2` contains greater than `no_data_max2` masked points. (set to negative to ignore)
 * \param[in] init_homo homography between `img1` and `img2` based on lat,lon coordinates of corner points
 * \param[in] pts_img1 row, col coordinates from `img1` to use for feature matching
 * \param[out] pts_img2 Best match in `img2` for `pts_img1`
 * \param[out] covs  Covariance of matches at `pts_img1`
 * \param[in] np  Number of points in `pts_img1`
 */
int32_t MatchFeaturesOnlyExtended(
    Parameters parameters,
    uint8_t *img1,
    uint8_t *mask1,
    size_t cols1,
    size_t rows1,
    int32_t no_data_max1,
    uint8_t *img2,
    uint8_t *mask2,
    size_t cols2,
    size_t rows2,
    int32_t no_data_max2,
    double init_homo[3][3],
    double *pts_img1,
    double *pts_img2,
    double *covs,
    int32_t np
);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LANDMARK_TOOLS_FEATURE_MATCH_H_
