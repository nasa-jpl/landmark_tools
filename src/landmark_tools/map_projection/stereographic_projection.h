/**
 * \file   stereographic_projection.h
 * \author Yang Cheng
 * \brief  Projection functions for Stereographic Projection
 *
 * \section updates Update History
 * - Created: 2019-11-05
 * - Updated: 2024-04-12 by Cecilia Mauceri Rewrote projections based on Snyder 1987
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

#ifndef _LANDMARK_TOOLS_STEREOGRAPHIC_PROJECTION_H_
#define _LANDMARK_TOOLS_STEREOGRAPHIC_PROJECTION_H_

#include <stdint.h>                                          // for int32_t

#include "landmark_tools/map_projection/datum_conversion.h"  // for Planet

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief Takes a latitude and longitude and converts it to x,y coordinates on a stereographic projection
 
 \param[in] lat input latitude in degrees
 \param[in] lg input longitude in degrees
 \param[in] lat0 Latitude of standard parallel in degrees
 \param[in] long0 Longitude of origin in degrees
 \param[in] R radius of planetary body
 \param[out] x
 \param[out] y
 */
void LatLong2StereographicProjection(double lat, double lg, double lat0, double long0, enum Planet body,   double *x, double *y);

/**
 \brief Takes stereographic coordinates (x,y) and converts them to a latitude and longitude
 
 \param[in] x 
 \param[in] y 
 \param[in] lat0 
 \param[in] long0 
 \param[in] R 
 \param[out] lat 
 \param[out] lg 
*/
void StereographicProjection2LatLong( double x, double y, double lat0, double long0, double R,  double *lat, double *lg );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_STEREOGRAPHIC_PROJECTION_H_ */
