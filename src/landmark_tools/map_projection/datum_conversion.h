#ifndef _LANDMARK_TOOLS_DATUM_CONVERSION_H_
#define _LANDMARK_TOOLS_DATUM_CONVERSION_H_
    
/**
 * \file   datum_conversion.h
 * \brief  Map reprojection algorithms
 * 
 * \section updates Update History
 * - Created: Unknown
 * - Updated: January 29, 2024 by Cecilia Mauceri
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

#include <stddef.h>

/**
 \brief Supported projections
*/
enum Projection {UTM, STEREO, EQUIDISTANT_CYLINDRICAL, GEOGRAPHIC, ORTHOGRAPHIC, Projection_UNDEFINED};

/**
 \brief Planetary bodies with defined ellipsoids
*/
enum Planet {Earth, Moon, Mars, Planet_UNDEFINED};
//TODO extend ellipsoid model to Mercury, Venus, Jupiter, Saturn, Uranus, Neptune, Europa

/**
 \brief Struct to contain ellipsoid parameters
*/
typedef struct _ELLIPSOID{
	double a;
	double b;
    double e2;
	double e2B;
	double e;
	double f;
}ELLIPSOID;

/**
 \brief Ellipsoid parameters for Planets
*/
static ELLIPSOID ellipsoids[]={
	{6378137.00,  6356752.3141, 0.00669437999013, 0.00673949678826, 0.08181919084255, 0.00335281066474}, //Earth
	{1737400.0 , 1737400.0 , 0.0, 0.0, 0.0, 0.0},  //Moon
	{3396190.0,  3376200.0, 0.0117373700, 0.011876772094, 0.10833914343394, 0.0058860075}, //Mars
};

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief Convert latitude, longitude and height to Body-Centric-Body-Fixed coordinates using an ellipsoid model of the Body. 
 
 \param[in] latitude_degrees latitude in degrees
 \param[in] longitude_degrees longitude in degrees
 \param[in] elevation_meters elevation in meters
 \param[out] p x,y,z coordinate in body-centric-body-fixed frame
 \param[in] body Plantary model
 */
void LatLongHeight_to_ECEF(double latitude_degrees, double longitude_degrees, double ele, double p[3], enum Planet body);

/**
 \brief Convert Body-Centric-Body-Fixed coordinates to latitude, longitude and height using an ellipsoid model of the Body. 
 
 \param[in] xyz point to convert
 \param[out] latitude_degrees latitude in degrees
 \param[out] longitude_degrees longitude in degrees
 \param[out] elevation_meters elevation in meters
 \param[in] body Plantary model
 */
void ECEF_to_LatLongHeight(double p[3], double *latitude_degrees, double *longitude_degrees, double *ele, enum Planet body);

/**
 \brief Convert latitude, longitude and height to Body-Centric-Body-Fixed coordinates using a spherical model of the body. 
 
 \param[in] latitude_degrees latitude in degrees
 \param[in] longitude_degrees longitude in degrees
 \param[in] elevation_meters elevation in meters
 \param[out] p (x,y,z) coordinate in body-centric-body-fixed frame
 \param[in] radius_meters body radius_meters in meters
 */
void LatLongHeight_to_ECEF_sphere(double latitude_degrees, double longitude_degrees, double ele, double p[3], double radius_meters);

/**
 \brief Convert Body-Centric-Body-Fixed coordinates to latitude, longitude and height using a spherical model of the body. 
 
 \param[in] xyz point to convert
 \param[out] latitude_degrees latitude in degrees
 \param[out] longitude_degrees longitude in degrees
 \param[out] elevation_meters elevation in meters
 \param[in] radius_meters Radius of planetary body in meters
 */
void ECEF_to_LatLongHeight_sphere(double p[3], double *latitude_degrees, double *longitude_degrees, double *ele, double radius_meters);

/**
 \brief Calculate the rotation between the local map frame and the Body-Centric-Body-Fixed frame with an ellipsoid Planet model
  the rotation is from east north up to ecef frame

 \param[in] lat 
 \param[in] lg 
 \param[in] elv 
 \param[out] rot_l_b 
 \param[in] body 
*/
void localmap2ECEF_rot(double lat, double lg, double elv, double rot_l_b[3][3], enum Planet body);

/**
 \brief Calculate the rotation between the local map frame and the Body-Centric-Body-Fixed frame with a spherical Planet model
  the rotation is from east north up to ecef frame

 \param[in] lat 
 \param[in] lg 
 \param[in] elv 
 \param[out] rot_l_b 
 \param[in] radius 
*/
void localmap2ECEF_rot_sphere(double lat, double lg, double elv, double rot_l_b[3][3], double radius);

/**
 \brief Convert string to Planet Enum
 
 \param[in] str 
 \return enum Planet 
*/
enum Planet strToPlanet(char *str);

/**
 \brief Convert string to Projection Enum
 
 \param[in] str 
 \return enum Projection 
*/
enum Projection strToProjection(char *str);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _LANDMARK_TOOLS_DATUM_CONVERSION_H_
