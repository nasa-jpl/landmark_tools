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

#include "landmark_tools/map_projection/lambert.h"
#include "landmark_tools/math/math_constants.h"

void initial_lambert( double lat1, double lat2, double lat0, double long0, LAMBERT *lambert)
{
	double E;
	double d;
	double t1, t2;

	E = sqrt(E2);
	
	lambert->lat1 = lat1;
	lambert->lat2 = lat2;
	lambert->lat0 = lat0;
	lambert->long0 = long0;

	lat1 = lat1*DEG2RAD;
	lambert->m1 = cos(lat1)/sqrt(1.0-E2*sin(lat1)*sin(lat1));
	d = (1-E*sin(lat1))/(1.0+E*sin(lat1));
	d = pow(d, E/2.0);
	t1 = tan(PI/4.0 - lat1/2.0)/d;

	lat2 = lat2*DEG2RAD;
	lambert->m2 = cos(lat2)/sqrt(1.0-E2*sin(lat2)*sin(lat2));
	d = (1-E*sin(lat2))/(1.0+E*sin(lat2));
	d = pow(d, E/2.0);
	t2 = tan(PI/4.0 - lat2/2.0)/d;

	lat0 = lat0*DEG2RAD;
	lambert->m0 = cos(lat0)/sqrt(1.0-E2*sin(lat0)*sin(lat0));
	d = (1-E*sin(lat0))/(1.0+E*sin(lat0));
	d = pow(d, E/2.0);
	lambert->t0 = tan(PI/4.0 - lat0/2.0)/d;


	d = (1-E*sin(lat1))/(1.0+E*sin(lat1));
	d = pow(d, E/2.0);
	t1 = tan(PI/4.0 - lat1/2.0)/d;

	lambert->n = (log(lambert->m1)-log(lambert->m2))/(log(t1) - log(t2));
	lambert->f = lambert->m1/lambert->n/pow(t1, lambert->n);

	lambert->p0 = Ae*lambert->f*pow(lambert->t0, lambert->n);
}


void latlong2lambert(LAMBERT lambert, double lat, double lg,  double *x, double *y)
{
	double t, d;
	double E;
	double theta;
	double P;
	E = sqrt(E2);
	
	theta = lambert.n*(lg-lambert.long0)*DEG2RAD;
	lat = lat*DEG2RAD;

	d = (1-E*sin(lat))/(1.0+E*sin(lat));
	d = pow(d, E/2.0);
	t = tan(PI/4.0 - lat/2.0)/d;

	P = Ae*lambert.f*pow(t, lambert.n);
	*x = P*sin(theta) + 2000000.0;
	*y = lambert.p0 - P*cos(theta) + 500000.0;
}



void initial_lambert_sphere( double lat1, double lat2, double lat0, double long0, double radius, LAMBERT *lambert)
{
    double E;
    double d;
    double t1, t2;
    
    E = sqrt(E2);
    E = 0.0;
    
    lambert->lat1 = lat1;
    lambert->lat2 = lat2;
    lambert->lat0 = lat0;
    lambert->long0 = long0;
    
    lat1 = lat1*DEG2RAD;
    lambert->m1 = cos(lat1) ;
    d = (1-E*sin(lat1))/(1.0+E*sin(lat1));
    d = pow(d, E/2.0);
    t1 = tan(PI/4.0 - lat1/2.0)/d;
    
    lat2 = lat2*DEG2RAD;
    lambert->m2 = cos(lat2) ;
    d = (1-E*sin(lat2))/(1.0+E*sin(lat2));
    d = pow(d, E/2.0);
    t2 = tan(PI/4.0 - lat2/2.0)/d;
    
    lat0 = lat0*DEG2RAD;
    lambert->m0 = cos(lat0) ;
    d = (1-E*sin(lat0))/(1.0+E*sin(lat0));
    d = pow(d, E/2.0);
    lambert->t0 = tan(PI/4.0 - lat0/2.0)/d;
    
    
    d = (1-E*sin(lat1))/(1.0+E*sin(lat1));
    d = pow(d, E/2.0);
    t1 = tan(PI/4.0 - lat1/2.0)/d;
    if(lat1 == lat2)
    {
        lambert->n = sin(lat1);
    }
    else
    {
        lambert->n = (log(lambert->m1)-log(lambert->m2))/(log(t1) - log(t2));
    }
    lambert->f = lambert->m1/lambert->n/pow(t1, lambert->n);
    
    lambert->p0 = radius*lambert->f*pow(lambert->t0, lambert->n);
}


void latlong2lambert_sphere(LAMBERT lambert, double lat, double lg, double radius,  double *x, double *y)
{
    double t, d;
    double E;
    double theta;
    double P;
    
    E = 0.0;
    
    theta = lambert.n*(lg-lambert.long0)*DEG2RAD;
    lat = lat*DEG2RAD;
    
    d = (1-E*sin(lat))/(1.0+E*sin(lat));
    d = pow(d, E/2.0);
    t = tan(PI/4.0 - lat/2.0)/d;
    
    P = radius*lambert.f*pow(t, lambert.n);
    *x = P*sin(theta) ;
    *y = lambert.p0 - P*cos(theta) ;
}
