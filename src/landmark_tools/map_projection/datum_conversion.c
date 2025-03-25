/** 
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

#include <math.h>
#include <stdio.h>                               // for printf, NULL
#include <string.h>                              // for strncmp

#include "landmark_tools/map_projection/datum_conversion.h"
#include "landmark_tools/math/math_constants.h"  // for DEG2RAD, RAD2DEG
#include "math/mat3/mat3.h"                      // for copy3, sub3, unit3


void LatLongHeight_to_ECEF_sphere(double latitude_degrees, double longitude_degrees, double elevation_meters, double p[3], double radius_meters)
{
   
   double cs, si, csl, sil;
   double A;
    
   A = radius_meters + elevation_meters;
   cs = cos(latitude_degrees*DEG2RAD);
   si = sin(latitude_degrees*DEG2RAD);
   csl = cos(longitude_degrees*DEG2RAD);
   sil = sin(longitude_degrees*DEG2RAD);
   p[0] = A*cs*csl;
   p[1] = A*cs*sil;
   p[2] = A*si;
   return;
}


void LatLongHeight_to_ECEF(double latitude_degrees, double longitude_degrees, double elevation_meters, double p[3], enum Planet body)
{
   double N;
   double cs, si, csl, sil;
   double A = ellipsoids[body].a;
   double e2 = ellipsoids[body].e2;

   cs = cos(latitude_degrees*DEG2RAD);
   si = sin(latitude_degrees*DEG2RAD);
   csl = cos(longitude_degrees*DEG2RAD);
   sil = sin(longitude_degrees*DEG2RAD);
   N = A/sqrt(1-e2*si*si);
   p[0] = (N + elevation_meters)*cs*csl;
   p[1] = (N + elevation_meters)*cs*sil;
   p[2] = (N*(1 - e2) + elevation_meters)*si;
   return;
}


void ECEF_to_LatLongHeight_sphere(double p[3], double *latitude_degrees, double *longitude_degrees, double *elevation_meters, double radius_meters )
{
	double d = sqrt(p[0]*p[0] + p[1]*p[1]);
    
    *latitude_degrees = atan(p[2]  /d)*RAD2DEG;
    *longitude_degrees = atan2(p[1], p[0])*RAD2DEG;
	 
	*elevation_meters = mag3(p) - radius_meters;
	return ;
}


void ECEF_to_LatLongHeight(double p[3], double *latitude_degrees, double *longitude_degrees, double *elevation_meters, enum Planet body)
{
	double d;
	double ph;
	double cs, si;
    double N;
    double A = ellipsoids[body].a;
    double e2 = ellipsoids[body].e2;
    double B = ellipsoids[body].b;
	double e2H = ellipsoids[body].e2B;

	d = sqrt(p[0]*p[0] + p[1]*p[1]);
    ph = atan(p[2]*A/d/B);
    cs = cos(ph);
	si = sin(ph);
    double latitude_radians = atan((p[2] + e2H*B*si*si*si)/(d-e2*A*cs*cs*cs));
    double longitude_radians = atan2(p[1], p[0]);
	si = sin(latitude_radians);
    N = A/sqrt(1-e2*si*si);
	*elevation_meters = d/cos(latitude_radians) - N;

    *latitude_degrees = (latitude_radians)*RAD2DEG;
    *longitude_degrees= (longitude_radians)*RAD2DEG;
	return ;
}


void localmap2ECEF_rot_sphere(double lat, double lg, double elv, double rot_l_b[3][3], double radius)
{
    double p1[3] = {0};
    double p2[3] = {0};
    double dp_lat[3]= {0};
    double dp_long[3]= {0};
    double dx[3] = {0};
    double dy[3] = {0};
    double dz[3] = {0};

    if(lat == -90 || lat == 90){
        dx[1] = 1;
        if(lat == 90){
            dz[2] = 1;
        }else{
            dz[2] = -1;
        }
        
        cross3(dz, dx, dy);
    }else{
        LatLongHeight_to_ECEF_sphere(lat+0.001, lg, elv, p1, radius);
        LatLongHeight_to_ECEF_sphere(lat-0.001, lg, elv, p2, radius);
        sub3(p1, p2, dp_lat);
        
        LatLongHeight_to_ECEF_sphere(lat, lg+0.001, elv, p1, radius);
        LatLongHeight_to_ECEF_sphere(lat, lg-0.001, elv, p2, radius);
        sub3(p1, p2, dp_long);
        
        unit3(dp_lat, dy);
        
        unit3(dp_long, dx);
        cross3(dx, dy, dz);
    }
    
    copy3(dx, rot_l_b[0]);
    copy3(dy, rot_l_b[1]);
    copy3(dz, rot_l_b[2]);
}


void localmap2ECEF_rot(double lat, double lg, double elv, double rot_l_b[3][3], enum Planet body)
{
    double p1[3] = {0};
    double p2[3] = {0};
    double dp_lat[3]= {0};
    double dp_long[3]= {0};
    double dx[3] = {0};
    double dy[3] = {0};
    double dz[3] = {0};

    if(lat == -90 || lat == 90){
        dx[1] = 1;
        if(lat == 90){
            dz[2] = 1;
        }else{
            dz[2] = -1;
        }
        
        cross3(dz, dx, dy);
    }else{
        LatLongHeight_to_ECEF(lat+0.001, lg, elv, p1, body);
        LatLongHeight_to_ECEF(lat-0.001, lg, elv, p2, body);
        sub3(p1, p2, dp_lat);
        
        LatLongHeight_to_ECEF(lat, lg+0.001, elv, p1, body);
        LatLongHeight_to_ECEF(lat, lg-0.001, elv, p2, body);
        sub3(p1, p2, dp_long);
        
        unit3(dp_lat, dy);
        
        unit3(dp_long, dx);
        cross3(dx, dy, dz);
    }
    copy3(dx, rot_l_b[0]);
    copy3(dy, rot_l_b[1]);
    copy3(dz, rot_l_b[2]);
}

enum Planet strToPlanet(char *str){
    enum Planet planet = Moon;
    if(str != NULL){
        if(strncmp(str, "Moon", strlen(str))==0){
            planet = Moon;
        }else if(strncmp(str, "Earth", strlen(str))==0){
            planet = Earth;
        }else if(strncmp(str, "Mars", strlen(str))==0){
            planet = Mars;
        }else{
            printf("Value of str must be \"Moon\" or \"Earth\" or \"Mars\"");
            planet = Planet_UNDEFINED;
        }
    }
    return planet;
}

enum Projection strToProjection(char *str){
    if(str != NULL){
        if(strncmp(str, "EQ_CYLINDERICAL", strlen(str))==0){
            return EQUIDISTANT_CYLINDRICAL;
        }else if(strncmp(str, "UTM", strlen(str))==0){
            return UTM;
        }else if(strncmp(str, "STEREO", strlen(str))==0){
            return STEREO;
        }else if(strncmp(str, "GEOGRAPHIC", strlen(str))==0){
            return GEOGRAPHIC;
        }else{
            printf("Value of str must be \"EQ_CYLINDERICAL\" or \"UTM\" or \"STEREO\"");
            return Projection_UNDEFINED;
        }
    }
    
    return Projection_UNDEFINED;
}
