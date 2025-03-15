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
#include "landmark_tools/math/math_constants.h"
#include "landmark_tools/map_projection/orthographic_projection.h"

void orthographic_map_projection(double lat, double lg, double lat0, double lg0, enum Planet body, double *x, double *y)
{
    //TODO is there a way to use the other ellipse parameters?
    double R = ellipsoids[body].a;

    *x = R*cos(lat*DEG2RAD)*sin((lg-lg0)*DEG2RAD);
    *y = R*(cos(lat0*DEG2RAD)*sin(lat*DEG2RAD)-sin(lat0*DEG2RAD)*cos(lat*DEG2RAD)*cos((lg-lg0)*DEG2RAD));
}

void inverse_orthographic_map_projection(double x, double y, double lat0, double lg0, double R, double *lat, double *lg)
{
    double cosc;
    double sinc;
    double rou;
    double c;
    rou = sqrt(x*x + y*y);
    if(rou < 0.000001)
    {
        *lat = lat0;
        *lg = lg0;
    }
    c = asin(rou/R);
    cosc= cos(c);
    sinc = sin(c);
    *lat = asin(cosc*sin(lat0*DEG2RAD) + y*sinc*cos(lat0*DEG2RAD)/rou)*RAD2DEG;
    *lg = lg0 + atan(x*sinc/(rou*cosc*cos(lat0*DEG2RAD)-y*sinc*sin(lat0*DEG2RAD)))*RAD2DEG;
}

