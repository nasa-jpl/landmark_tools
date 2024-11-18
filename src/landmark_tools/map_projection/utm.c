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

#include "landmark_tools/map_projection/utm.h"
#include "landmark_tools/math/math_constants.h"  // for E2, Ae, DEG2RAD

void latlong2utm(double lat, double lg, double lg0, double *x, double *y)
{
  double E2b;
  double lat_r, lg_r, lg_r0;
  double si, cs;
  double k0,  N, T, C, A, M;
  //double Ae;
  //double E2;
  //Ae = 6378137.0;
  //E2 = 0.0066945;
  lat_r = lat* DEG2RAD;
  lg_r = lg*DEG2RAD;
  lg_r0 = lg0*DEG2RAD;
  E2b = E2/(1-E2);
  si = sin(lat_r);
  cs = cos(lat_r);
  N = Ae/sqrt(1-E2*si*si);
  T = tan(lat_r)*tan(lat_r);
  C = E2b*cs*cs;
  A = (lg_r - lg_r0)*cs;
  M = Ae*((1.0-E2/4.0 - 3.0*E2*E2/64 - 5*E2*E2*E2/256)*lat_r - (3.0*E2/8.0 + 3*E2*E2/32.0 + 45*E2*E2*E2/1024)*sin(lat_r*2)
	  + (15.0*E2*E2/256 +45*E2*E2*E2/1024.0)*sin(lat_r*4) - (35.0*E2*E2*E2/3072*sin(lat_r*6)));
 
  k0 = 0.9996;
  *x = k0*N*(A +(1-T+C)*A*A*A/6.0 + (5-18.0*T + T*T +72*C -58*E2b)*A*A*A*A*A/120.0)+ 500000;
  *y = k0*(M + N*tan(lat_r)*(A*A/2 + (5-T + 9*C + 4*C*C)*A*A*A*A/24.0 +
	  (61-58*T +T*T +600*C -330*E2b)*A*A*A*A*A*A/720));
}

void utm2latlong(double x, double y, double lg0, double *lat, double *lg)
{
	double k0;
	double M;
	double mu, e1;
    double eb2, c1, t1, n1, r1, d;
	double ph1;
	double ph, lamd;
	k0 = 0.9996;
    
	M = y/k0;
	x = x-500000;
    mu = M/(Ae*(1-E2/4.0 - 3.0*E2*E2/64.0 - 5.0*E2*E2*E2/256));
	e1 = (1-sqrt(1-E2))/(1+sqrt(1-E2));
    ph1 = mu +(3.0*e1/2.0 - 27.0*e1*e1*e1/32.0)*sin(2*mu) + 
		(21.0*e1*e1/16.0 - 55.0*e1*e1*e1*e1/32.0)*sin(4.0*mu) + 
		(151*e1*e1*e1/96)*sin(6.0*mu) + 
		(1097.0*e1*e1*e1*e1/512.0)*sin(8.0*mu);
	eb2 = E2/(1-E2);
	c1 = eb2*cos(ph1)*cos(ph1);
	t1 = tan(ph1)*tan(ph1);
	n1 = Ae/sqrt(1-E2*sin(ph1)*sin(ph1));
	r1 = Ae*(1-E2)/(1-E2*sin(ph1)*sin(ph1))/sqrt(1-E2*sin(ph1)*sin(ph1));
	d = x/(n1*k0);
	ph = (d*d/2 - (5.0 + 3.0*t1 + 10.0*c1 - 4.0*c1*c1 - 9.0*eb2)*d*d*d*d/24.0
		+(61.0 + 90.0*t1 + 298*c1 + 45 *t1*t1 - 253*eb2 - 3*c1*c1)*d*d*d*d*d*d/720);
	ph = ph*(n1*tan(ph1)/r1);
	ph = ph1 - ph;
	lamd = (d-(1+2.0*t1 + c1)*d*d*d/6.0 + 
		(5.0 - 2.0*c1 + 28*t1 - 3.0*c1*c1 + 8*eb2 + 24.0*t1*t1)*d*d*d*d*d/120.0)/cos(ph1);
	ph = ph*180.0/3.1415926;
	lamd = lamd*180.0/3.1415926;
	*lat = ph;
	*lg = lamd + lg0;
}

double UTMScale(double lat, double lg, double lg0)
{
	double k0;
	double l, b;
	 
	double scale;
	double cs, si;
	double E2b, T, C, A;
	 E2b = E2/(1-E2);
	l = (lg- lg0)*3.1415926/180.0;
    b = lat*3.1415926/180.0;
	cs = cos(b);
	si = sin(b);
   
//    N = Ae/sqrt(1-E2*si*si);
    T = tan(b)*tan(b);
    C = E2b*cs*cs;
    A = l*cs;
	
   
    k0 = 0.9996;
    //scale = 1 + l*l/2*cs*cs*(1+mu2) + l*l*l*l/24.0*cs*cs*cs*cs*(5.0-4.0*tan(b)*tan(b));
	scale = 1.0 + (1.0 + C)*A*A/2 + (5 - 4.0*T + 42.0*C + 13.0*C*C - 28.0*E2b)*A*A*A*A/24 + (61.0-148*T + 16.0*T*T)*A*A*A*A*A*A/720;
	return scale*k0;
}

double UTMbearings(double lat, double lg, double lg0)
{
   
	double l, b;
	double mu2;
	double cs, si;
	double gama;
	l = (lg- lg0)*3.1415926/180.0;
    b = lat*3.1415926/180.0;
	cs = cos(b);
	si = sin(b);
	mu2 = (E2-E2*si*si)/(1.0-E2)-1.0;
	gama = l*si + l*l*l/3*si*cs*cs*(1+3.0*mu2 + 2.0*mu2*mu2)
		+ l*l*l*l*l/15.0 *si *cs*cs*cs*cs*(2.0 - tan(b)*tan(b));
	return gama;
}
