/************************************************
 * \file homography_util.h
 * \brief Calcuate homographies from point correspondances
 *
 *  \copyright Copyright 2024 California Institute of Technology
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
 * 
 ***********************************************/

#ifndef _LANDMARK_TOOLS_HOMOGRAPHY_UTIL_H_
#define _LANDMARK_TOOLS_HOMOGRAPHY_UTIL_H_

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


//TODO Document functions!!
int32_t convertTo33(double h[9], double h_out[3][3]);
int32_t convertTo19( double h[3][3], double h_out[9]);
int32_t convertToHomo(double ip[2], double op[3]);
int32_t convertToImage(double ip[3], double op[2]);

int32_t homographyTransfer33D(double m[3][3], double ip[2], double op[2]);
int32_t homographyTransfer33(double m[3][3], int32_t x, int32_t y,  double op[2]);
    
int32_t inverseHomography19(double *m, double *invm);
int32_t inverseHomography33(double m[3][3], double invm[3][3]);

int32_t  Convert2ImageCoordinate33(double homo[3][3], double homo_out[3][3],  double inm[3][3]);

int32_t  getHomographyFromPointsNormalize(double *points2d1, 
					   double *points2d2, int32_t num_pts_plane, double homo[3][3]);
int32_t  getHomographyFromPoints(double *points2d1, 
					   double *points2d2, int32_t num_pts_plane,  double homo[3][3]);
int32_t  getHomographyFromPoints_RANSAC(double *points2d1, 
					   double *points2d2, int32_t num_pts_plane, double int32_trisicM[3][3],double homo[3][3]);
int32_t  getHomographyFromPoints_RANSAC_frame(double *points2d1, 
					   double *points2d2, int32_t num_pts_plane, double homo[3][3], double tol);

/**
 \brief Calculate the homography from point correspondences using the method of 
 Homography construction from Juyang Weng's paper IEEE SP Vol. 39 Np 12 Dec 1991
 "Motion and struture from point correspondences with error estimation:planar surfaces"

 \param[in] prefeatures Initial feature location in pixel (x,y) coordinates
 \param[in] curfeatures Current feature location  in pixel (x,y) coordinates
 \param[in] num_features Number of features
 \param[in] intrisicM Camera Intrinsic matrix
 \param[out] h homography
 \return true on success
 \return false on error 
*/
bool getHomographyFromPoints_Eigenvalue(double *prefeatures, double *curfeatures, int32_t num_features, double intrisicM[3][3], double h[3][3]);


int32_t ShiftHomographyOrigin(double homo[3][3], double p10[2], double p20[2]);

int32_t transferImage(double homo[3][3], uint8_t *in_img, int32_t cols, int32_t rows, uint8_t *outimg, int32_t cols2, int32_t rows2);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // end include guard _LANDMARK_TOOLS_HOMOGRAPHY_UTIL_H_
