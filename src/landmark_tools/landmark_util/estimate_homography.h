/**
 * \file   `estimate_homography.h`
 * \author Yang Cheng
 * \brief  TODO
 *
 * \section updates Update History
 * - Created: Unknown
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
#ifndef _LANDMARK_TOOLS_ESTIMATE_HOMOGRAPHY_H_
#define _LANDMARK_TOOLS_ESTIMATE_HOMOGRAPHY_H_

#include "landmark_tools/landmark_util/landmark.h"

/**
 \brief Align the two maps using a homography
 
 For four corner points in `lmk_child`, find the corresponding point in `lmk_base` and compute a homography.
 \param[in] lmk_base
 \param[in] lmk_child
 \param[out] base2child homography matrix
 */
void estimateHomographyUsingCorners(LMK* lmk_base, LMK* lmk_child, double base2child[3][3]);

#endif /* _LANDMARK_TOOLS_ESTIMATE_HOMOGRAPHY_H_ */
