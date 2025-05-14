/** 
 * \file landmark_registration.h
 * \author Yang Cheng
 * 
 * \brief Functions for registering landmarks using feature matching
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

#ifndef LANDMARK_REGISTRATION_H
#define LANDMARK_REGISTRATION_H

#include <stdint.h>
#include "landmark_tools/landmark_util/landmark.h"
#include "landmark_tools/feature_tracking/parameters.h"

/**
 * \brief Registers two landmarks by finding corresponding features and estimating the transformation
 * 
 * This function performs the following operations:
 * 1. Detects features in both landmarks
 * 2. Matches features between landmarks
 * 3. Estimates homography from matched features
 * 4. Refines transformation using 3D point clouds
 * 5. Updates the child landmark with the refined transformation
 * 
 * \param parameters Matching parameters for feature detection and matching
 * \param base_landmark_filename Filename of the base landmark
 * \param child_landmark_filename Filename of the child landmark
 * \return 1 on success, 0 on failure
 */
int32_t RegisterLandmarks(Parameters parameters, 
                         const char *base_landmark_filename, 
                         const char *child_landmark_filename);

#endif // LANDMARK_REGISTRATION_H 