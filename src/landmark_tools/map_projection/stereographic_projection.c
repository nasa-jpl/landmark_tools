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

#include "landmark_tools/map_projection/datum_conversion.h"
#include "landmark_tools/map_projection/stereographic_projection.h"
#include "landmark_tools/math/math_constants.h"

void LatLong2StereographicProjection(double lat, double lg, double lat0, double long0, enum Planet body,   double *x, double *y)
{
    double lat0_rad = lat0*DEG2RAD;
    double long0_rad = long0*DEG2RAD;
    double lat_rad = lat*DEG2RAD;
    double lg_rad = lg*DEG2RAD;

    //TODO is there a way to use the other ellipse parameters?
    double R = ellipsoids[body].a;
    
    double k0 = 1.0; //Scale factor
    double k = 2*k0/(1+sin(lat0_rad)*sin(lat_rad)+cos(lat0_rad)*cos(lat_rad)*cos(lg_rad-long0_rad));
    *x = R*k*cos(lat_rad)*sin(lg_rad-long0_rad);
    *y = R*k*(cos(lat0_rad)*sin(lat_rad)-sin(lat0_rad)*cos(lat_rad)*cos(lg_rad-long0_rad));
}


void StereographicProjection2LatLong( double x, double y, double lat0, double long0, double R,   double *lat, double *lg )
{
    double lat0_rad = lat0*DEG2RAD;
    double long0_rad = long0*DEG2RAD;
    
    double k0 = 1.0; //Scale factor
    double rho = sqrt(x*x + y*y);
    double c = 2*atan(rho/(2*R*k0));
    
    *lat = asin(cos(c)*sin(lat0_rad)+(y*sin(c)*cos(lat0_rad)/rho));
    if(lat0 == 90){
        *lg = long0 + (atan(x/-y))*RAD2DEG;
    }else if(lat0 == -90){
        *lg = long0 + (atan(x/y))*RAD2DEG;
    }else{
        *lg = long0 + (atan(x* sin(c) /(rho*cos(lat0_rad)*cos(c)-y*sin(lat0_rad)*sin(c))))*RAD2DEG;
    }
}
