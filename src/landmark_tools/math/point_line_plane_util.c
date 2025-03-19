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

#define _CRTDBG_MAP_ALLOC

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "landmark_tools/math/math_constants.h"
#include "landmark_tools/math/math_utils.h"
#include "landmark_tools/math/point_line_plane_util.h"
#include "landmark_tools/math/double_matrix.h"
#include "landmark_tools/math/math_utils.h"
#include "math/mat3/mat3.h"

double Point2PlaneDist(double P[3], double plane[4])
{
    double d;
    d = dot3(P, plane) + plane[3];
    return d;
}

double XY2LineDist2D(double x, double y, double line[3])
{
    double d;
    d = line[0]*x + line[1]*y + line[2];
    return d;
}

double Point2LineDist2D(double pt[2], double line[3])
{
    return XY2LineDist2D(pt[0], pt[1], line);
}

int32_t PointProject2Line3D(double vec[3], double p0[3], double pin[3], double pout[3])
{
    double r, dp[3];
    sub3(p0, pin, dp);
    r = -dot3(dp, vec);
    scale3(r, vec, pout);
    add3(p0, pout, pout);
    return 1;
}

void PointProject2Line2D(double vec[2], double p0[2], double pin[2], double pout[2])
{
    double dp[2], r;
    dp[0] = p0[0] - pin[0];
    dp[1] = p0[1] - pin[1];
    r = -(vec[0]*dp[0] + vec[1]*dp[1]);
    pout[0] = p0[0] + r*vec[0];
    pout[1] = p0[1] + r*vec[1];
}


double PointsDist3D(double p1[3], double p2[3])
{
    double dp[3];
    sub3(p1, p2, dp);
    return mag3(dp);
}


int32_t PointRayIntersection2Plane(double p[3], double ray[3], double plane[4], double p_out[3])
{
    double r;
    double rays[3];
    r = (-plane[3] - dot3(plane, p))/dot3(ray, plane);
    scale3(r, ray, rays);
    add3(p, rays, p_out);
    return 1;
}


int32_t normalpoint2plane(double vec[3], double p[3], double plane[4])
{
    copy3(vec, plane);
    plane[3] = -dot3(vec, p);
    return 1;
}


int32_t  Point_Clouds_rot_T(double *ptsA, double *ptsB, int32_t num_pts, double bRa[3][3], double T[3])
{
    double Pam[3];
    double Pbm[3];
    double pa[3], pb[3];
    double A[3][3];
    double Aab[3][3];
    
    int32_t i;
    if(num_pts < 3)
    {
        printf("too few points for estimating the rotation and translation\n");
        return 0;
    }
    zero3(Pam);
    zero3(Pbm);
    for(i = 0; i < num_pts; ++i)
    {
        add3(Pam, &ptsA[i*3], Pam);
        add3(Pbm, &ptsB[i*3], Pbm);
    }
    scale3(1.0/(double)num_pts, Pam, Pam);
    scale3(1.0/(double)num_pts, Pbm, Pbm);
    zero33(A);
    for(i = 0; i < num_pts; ++i)
    {
        sub3(&ptsB[i*3], Pbm, pb);
        sub3(&ptsA[i*3], Pam, pa);
        mult313(pb, pa, Aab);
        add33(Aab, A, A);
    }
    // prt33(A);
    normalizeRotation(A);
    copy33(A, bRa);
    mult331(bRa, Pam, pb);
    sub3(Pbm, pb, T);
    return 1;
}

#define PC_RANSAC_MAX_ITERATIONS 30
int32_t  Point_Clouds_rot_T_RANSAC(double *ptsA, double *ptsB, int32_t num_pts, double bRa[3][3], double T[3], double tol)
{
    double Pam[3];
    double Pbm[3];
    double pa[3], pb[3];
    double A[3][3];
    double Aab[3][3];
    double  Am[9], Bm[9], s;
    double bRa_tmp[3][3], Ttmp[3];
    double p[3];
    double  n[3], bestT[3],  bestR[3][3];
    double *ptsA_tmp;
    double *ptsB_tmp;
    
    ptsA_tmp = (double *)malloc(sizeof(double)*num_pts*3);
if (ptsA_tmp == NULL)
	{
		printf("Point_Clouds_rot_T_RANSAC() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
		return 0;
	}

    ptsB_tmp = (double *)malloc(sizeof(double)*num_pts*3);
if (ptsB_tmp == NULL)
	{
		printf("Point_Clouds_rot_T_RANSAC() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
		return 0;
	}
    
    int32_t index[3], flag, bestk, nm ;
    int32_t i, j, p1;
    int32_t k;
    n[0] = 1.0;
    n[1] = 1.0;
    n[2] = 1.0;
    bestk = 0;
    for(int32_t iter = 0; iter <  PC_RANSAC_MAX_ITERATIONS; ++iter)
    {
        printf("iter = %d\n", iter);
        nm = 0;
        while(nm < 3)
        {
            p1 = (int32_t)(rand()%num_pts);
            if(p1 < num_pts && p1 >= 0)
            {
                flag = 1;
                
                for(i = 0; i < nm; ++i)
                {
                    if(index[i] == p1)
                        flag = 0;
                }
                
                if(flag == 1)
                {
                    //printf("p1 = %d\n", p1);
                    copy3(&ptsA[3*p1], &Am[nm*3]);
                    copy3(&ptsB[3*p1], &Bm[nm*3]);
                    index[nm] = p1;
                    nm++;
                }
            }
        }
        Point_Clouds_rot_T(Am, Bm,3, bRa_tmp, Ttmp);
        //  prt33(bRa_tmp);
        //  prt3(Ttmp);
        //Pb = RPa + T
        for(i = 0, k = 0; i < num_pts; ++i)
        {
            mult331(bRa_tmp,&ptsA[3*i], p );
            add3(p, Ttmp, p);
            sub3(p, &ptsB[3*i], p);
            // prt3(p);
            // printf("tol %f mag3 %f\n", tol, mag3(p));
            if(mag3(p) < tol)
            {
                k++;
                // printf("hihihi tol %f mag3 %f k = %d\n", tol, mag3(p));
            }
        }
        if(k > bestk)
        {
            bestk = k;
            copy33(bRa_tmp, bestR);
            copy3(Ttmp, bestT);
        }
    }
    printf("bestk %d\n", bestk);
    for(i = 0, k = 0; i < num_pts; ++i)
    {
        mult331(bestR,&ptsA[3*i], p );
        add3(p, bestT, p);
        sub3(p, &ptsB[3*i], p);
        if(mag3(p) < tol)
        {
            copy3(&ptsA[3*i], &ptsA_tmp[3*k]);
            copy3(&ptsB[3*i], &ptsB_tmp[3*k]);
            k++;
        }
    }
    
    if(k > 6)
    {
        // printf("k = %d\n", k);
        Point_Clouds_rot_T(ptsA_tmp, ptsB_tmp,k, bRa, T);
    }
    else
    {
      // [THP 2024/10/21] Return 0 if RANSAC failed
      free( ptsA_tmp );
      free( ptsB_tmp );
      return 0;
    }
    prt3(T);
    prt33(bRa);
    
    free( ptsA_tmp );
    free( ptsB_tmp );
    return 1;
}
