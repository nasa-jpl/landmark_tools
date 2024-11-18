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
#include "landmark_tools/map_projection/equidistant_cylindrical_projection.h"
#include "landmark_tools/math/math_constants.h"


void LatLong2EquidistantCylindricalProjection(double latitude, double longitude, double standard_parallel, double central_meridian, enum Planet body, double *x, double *y)
{
    double central_meridian_rad = central_meridian*DEG2RAD;
    double standard_parallel_rad = standard_parallel*DEG2RAD;
    double latitude_rad = latitude*DEG2RAD;
    double longitude_rad = longitude*DEG2RAD;

    double delta_longitude = longitude_rad-standard_parallel_rad;
    if(delta_longitude > PI){
        delta_longitude -= 2*PI;
    }if(delta_longitude < -PI){
        delta_longitude += 2*PI;
    }
    
    //TODO is there a way to use the other ellipse parameters?
    double R = ellipsoids[body].a;
    
    *x = R*cos(central_meridian_rad)*delta_longitude;
    *y = R*latitude_rad;
}


void EquidistantCylindricalProjection2LatLong( double x, double y, double standard_parallel, double central_meridian, enum Planet body, double *latitude, double *longitude )
{
    double standard_parallel_rad = standard_parallel*DEG2RAD;
    double central_meridian_rad = central_meridian*DEG2RAD;
    double R = ellipsoids[body].a;
    
    *latitude = y / R;
    *longitude = central_meridian_rad + x/(R *cos(standard_parallel_rad));
}
