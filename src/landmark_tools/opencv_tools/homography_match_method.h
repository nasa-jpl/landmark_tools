/**
 * \file homography_match_method.h
 * \author Tu-Hoa Pham
 * \date 2024
 * 
 * \brief Feature matching methods for homography estimation
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

#ifndef HOMOGRAPHY_MATCH_METHOD_H
#define HOMOGRAPHY_MATCH_METHOD_H

/**
 * \brief Supported feature descriptors for homography matching
 */
typedef enum {
    SIFT,  ///< Scale-Invariant Feature Transform
    ORB    ///< Oriented FAST and Rotated BRIEF
} HomographyMatchMethod;

/**
 * \brief Convert HomographyMatchMethod enum to string representation
 * 
 * \param[in] method The feature matching method to convert
 * \return const char* String representation of the method
 */
const char* HomographyMatchMethodToStr(HomographyMatchMethod method);

#endif // HOMOGRAPHY_MATCH_METHOD_H 