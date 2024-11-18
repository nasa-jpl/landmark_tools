/**
 * \file   utm.h
 * \brief  Projection functions for Universal Transverse Mercator (UTM) projection
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

#ifndef _LANDMARK_TOOLS_UTM_H_
#define _LANDMARK_TOOLS_UTM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief Project latitude and longitude to UTM coordinates
 
 \param[in] lat 
 \param[in] lg 
 \param[in] lg0 
 \param[out] x 
 \param[out] y 
*/
void latlong2utm(double lat, double lg, double lg0, double *x, double *y);

/**
 \brief Project UTM coordinates to latitude and longitude 
 
 \param[in] x 
 \param[in] y 
 \param[in] lg0 
 \param[out] lat 
 \param[out] lg 
*/
void utm2latlong(double x, double y, double lg0, double *lat, double *lg);

/**
 \brief TODO
 
 \param[] lat 
 \param[] lg 
 \param[] lg0 
 \return double 
*/
double UTMScale(double lat, double lg, double lg0);

/**
 \brief TODO
 
 \param[] lat 
 \param[] lg 
 \param[] lg0 
 \return double 
*/
double UTMbearings(double lat, double lg, double lg0);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // UTM_H
