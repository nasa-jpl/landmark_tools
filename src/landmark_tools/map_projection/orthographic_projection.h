/**
 * \file   orthographic_projection.h
 * \author Yang Cheng
 * \brief  Projection functions for Orthographic Projection
 * 
 * \section updates Update History
 * - Created: 1/2/19
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



#ifndef _LANDMARK_TOOLS_ORTHOGRAPHIC_PROJECTION_H_
#define _LANDMARK_TOOLS_ORTHOGRAPHIC_PROJECTION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief Project latitude and longitude into orthographic coordinates (x,y)
 
 \param[in] lat 
 \param[in] lg 
 \param[in] lat0 
 \param[in] lg0 
 \param[in] R 
 \param[out] x 
 \param[out] y 
*/
void orthographic_map_projection(double lat, double lg, double lat0, double lg0, double R, double *x, double*y);

/**
 \brief Project orthographic coordinates (x,y) to latitude and longitude
 
 \param[in] x 
 \param[in] y 
 \param[in] lat0 
 \param[in] lg0 
 \param[in] R 
 \param[out] lat 
 \param[out] lg 
*/
void inverse_orthographic_map_projection(double x, double y, double lat0, double lg0, double R, double *lat, double *lg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_ORTHOGRAPHIC_PROJECTION_H_ */