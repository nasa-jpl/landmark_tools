#ifndef _LANDMARK_TOOLS_LAMBERT_H_
#define _LANDMARK_TOOLS_LAMBERT_H_

/**
 * \file   lambert.h
 * \brief  Projection functions for Lambert conformal conic projection
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


#include <inttypes.h>
#include <stdint.h>  // for int32_t

typedef struct _LAMBERT
{
	double lat1;
	double lat2;
	double lat0;
	double long0;
	double n;
	double m1;
	double m2;
	double m0;
	double p0;
	double t0;
	double f;
}LAMBERT;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief TODO
 
 \param[] lat1 
 \param[] lat2 
 \param[] lat0 
 \param[] long0 
 \param[] lambert 
*/
void initial_lambert( double lat1, double lat2, double lat0, double long0, LAMBERT *lambert);

/**
 \brief TODO
 
 \param[] lambert 
 \param[] lat 
 \param[] lg 
 \param[] x 
 \param[] y 
*/
void latlong2lambert(LAMBERT lambert, double lat, double lg,  double *x, double *y);

/**
 \brief TODO
 
 \param[] lat1 
 \param[] lat2 
 \param[] lat0 
 \param[] long0 
 \param[] radius 
 \param[] lambert 
*/
void initial_lambert_sphere( double lat1, double lat2, double lat0, double long0, double radius, LAMBERT *lambert);

/**
 \brief TODO
 
 \param[] lambert 
 \param[] lat 
 \param[] lg 
 \param[] radius 
 \param[] x 
 \param[] y 
*/
void latlong2lambert_sphere(LAMBERT lambert, double lat, double lg, double radius,  double *x, double *y);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LANDMARK_TOOLS_LAMBERT_H_
