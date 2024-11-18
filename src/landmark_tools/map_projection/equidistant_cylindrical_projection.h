/**
 * \file   equidistant_cylindrical_projection.h
 * \author Cecilia Mauceri
 * \brief  Projection functions for Equidistant Cylindrical Projection
 *
 * \section updates Update History
 * - Created: 2024-11-05 Projections based on Snyder 1987
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


#ifndef _LANDMARK_TOOLS_EQUIDISTANT_CYLINDRICAL_PROJECTION_H_
#define _LANDMARK_TOOLS_EQUIDISTANT_CYLINDRICAL_PROJECTION_H_

#include "landmark_tools/map_projection/datum_conversion.h"

/**
 \brief Takes a latitude and longitude and converts it to x,y coordinates on a equidistant cylindrical projection
 
 \param[in] latitude input latitude in degrees
 \param[in] longitude input longitude in degrees
 \param[in] standard_parallel Latitude of standard parallel in degrees
 \param[in] central_meridian Longitude of origin in degrees
 \param[in] body planetary body
 \param[out] x
 \param[out] y
 */
void LatLong2EquidistantCylindricalProjection(double latitude, double longitude, double standard_parallel, double central_meridian, enum Planet body, double *x, double *y);

/**
 \brief Takes equidistant cylindrical coordinates and converts it to a latitude and longitude
 
 \param[in] x 
 \param[in] y 
 \param[in] standard_parallel 
 \param[in] central_meridian 
 \param[in] body 
 \param[out] latitude 
 \param[out] longitude 
*/
void EquidistantCylindricalProjection2LatLong( double x, double y, double standard_parallel, double central_meridian, enum Planet body, double *latitude, double *longitude );

#endif /* _LANDMARK_TOOLS_EQUIDISTANT_CYLINDRICAL_PROJECTION_H_ */
