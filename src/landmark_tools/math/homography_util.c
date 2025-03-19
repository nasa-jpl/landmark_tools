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

#include <math.h>                                                // for fabs
#include <stdio.h>                                               // for printf
#include <stdlib.h>                                              // for free

#include "landmark_tools/data_interpolation/interpolate_data.h"  // for inte...
#include "landmark_tools/math/homography_util.h"
#include "landmark_tools/math/double_matrix.h"                   // for Line...
#include "landmark_tools/math/math_utils.h"                      // for prt3
#include "math/mat3/mat3.h"                                      // for mult331

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#define RANSAC_MAX_ITERATIONS 200

int32_t convertTo33(double h[9], double h_out[3][3])
{
    h_out[0][0] = h[0];
    h_out[0][1] = h[1];
    h_out[0][2] = h[2];
    
    h_out[1][0] = h[3];
    h_out[1][1] = h[4];
    h_out[1][2] = h[5];
    
    h_out[2][0] = h[6];
    h_out[2][1] = h[7];
    h_out[2][2] = 1.0;
    return 1;
}

int32_t convertTo19( double h[3][3], double h_out[9])
{
    h_out[0] = h[0][0];
    h_out[1] = h[0][1];
    h_out[2] = h[0][2];
    
    h_out[3] = h[1][0];
    h_out[4] = h[1][1];
    h_out[5] = h[1][2];
    
    h_out[6] = h[2][0];
    h_out[7] = h[2][1];
    h_out[8] = 1.0;
    return 1;
}

int32_t convertToHomo(double ip[2], double op[3])
{
    op[0] = ip[0];
    op[1] = ip[1];
    op[2] = 1.0;
    return 1;
}

int32_t convertToImage(double ip[3], double op[2])
{
    op[0] = ip[0]/ip[2];
    op[1] = ip[1]/ip[2];
    return 1;
}


int32_t homographyTransfer33D(double h[3][3], double ip[2], double op[2])
{
    double  p[3];
    double hp[3];
    convertToHomo(ip, p);
    mult331(h, p, hp);
    convertToImage(hp, op);
    return 1;
}


int32_t homographyTransfer33(double h[3][3], int32_t x, int32_t y,  double op[2])
{
    double  p[3];
    double hp[3];
    p[0] = x;
    p[1] = y;
    p[2] = 1.0;
    mult331(h, p, hp);
    convertToImage(hp, op);
    return 1;
}

int32_t inverseHomography19(double *m, double *invm)
{
    double d;
    d = (m[0]*m[4] - m[1]*m[3]);
    invm[0] = (m[4] - m[5]*m[7])/d;
    invm[1] = (m[2]*m[7] - m[1])/d;
    invm[2] = (m[1]*m[5] - m[2]*m[4])/d;
    invm[3] = (m[5]*m[6] - m[3])/d;
    invm[4] = (m[0] - m[2]*m[6])/d;
    invm[5] = (m[2]*m[3] - m[0]*m[5])/d;
    invm[6] = (m[3]*m[7] - m[6]*m[4])/d;
    invm[7] = (m[1]*m[6] - m[0]*m[7])/d;
    invm[8] = 1.0/m[8];
    
    return 1;
}

int32_t inverseHomography33(double h[3][3], double invh[3][3])
{
    double m[9], invm[9];
    
    convertTo19(h, m);
    inverseHomography19(m, invm);
    convertTo33(invm, invh);

    return 1;
}
//homo is from out to in
int32_t transferImage(double homo[3][3], uint8_t *in_img, int32_t cols, int32_t rows, uint8_t *outimg, int32_t cols2, int32_t rows2)
{
    int32_t i, j;
    double op[2];
    for(i = 0; i < rows2; ++i)
    {
        for(j = 0; j < cols2; ++j)
        {
            homographyTransfer33(homo, j, i,  op);
            if(op[0] > 0 && op[0] < cols-1 && op[1] > 0 && op[1] < rows - 1)
            {
                uint8_t val = 0;
                if(inter_uint8_matrix(in_img, cols, rows, op[0], op[1], &val)){
                    outimg[i*cols2 + j] = (int32_t)val;
                }
            }
            else
            {
                outimg[i*cols2 + j] = 0;
            }
        }
    }
    return 1;
}


int32_t  Convert2ImageCoordinate33(double homo[3][3], double homo_out[3][3],  double inm[3][3])
{
    double inv_inm[3][3];
    double h[3][3];
    double k1[3], k2[3], k3[3], k[3];
    double newh[3][3];
    inv33_stable(inm, inv_inm);
    copy33(homo, h);
    
    scale3(inm[0][0], h[0], k1);
    scale3(inm[0][1], h[1], k2);
    scale3(inm[0][2], h[2], k3);
    add3(k1, k2, k);
    add3(k, k3, k);
    scale3(k[0], inv_inm[0], k1);
    scale3(k[1], inv_inm[1], k2);
    scale3(k[2], inv_inm[2], k3);
    add3(k1, k2, k);
    add3(k, k3, newh[0]);
    
    scale3(inm[1][0], h[0], k1);
    scale3(inm[1][1], h[1], k2);
    scale3(inm[1][2], h[2], k3);
    add3(k1, k2, k);
    add3(k, k3, k);
    scale3(k[0], inv_inm[0], k1);
    scale3(k[1], inv_inm[1], k2);
    scale3(k[2], inv_inm[2], k3);
    add3(k1, k2, k);
    add3(k, k3, newh[1]);
    
    scale3(h[2][0], inv_inm[0], k1);
    scale3(h[2][1], inv_inm[1], k2);
    scale3(h[2][2], inv_inm[2], k3);
    add3(k1, k2, k);
    add3(k, k3, newh[2]);
    scale33(1/newh[2][2], newh, newh);
    copy33(newh, homo_out);
    
    return 1;
}

bool getHomographyFromPoints_Eigenvalue(double *prefeatures, double *curfeatures, int32_t num_features, double intrisicM[3][3], double h[3][3])
{
    // Copy intrinsic matrix to gsl_matrix
    gsl_matrix *M = gsl_matrix_alloc(3, 3);
    if (M == NULL) {
        fprintf(stderr, "getHomographyFromPoints_Eigenvalue -> Could not allocate memory for matrix M: %.256s, %d\n", __FILE__, __LINE__);
        return false;
    }

    for (size_t i = 0; i < 3; i++) {
       for (size_t j = 0; j < 3; j++) {
           gsl_matrix_set(M, i, j, intrisicM[i][j]);
       }
    }

    // Calculate inverse 
    gsl_matrix *invM = gsl_matrix_inverse(M);
    if(invM == NULL){
        fprintf(stderr, "getHomographyFromPoints_Eigenvalue() ==>> Could calculate inverse of intrinsics matrix: %.256s, %d\n", __FILE__, __LINE__);
        if(M != NULL){
            gsl_matrix_free(M);
        }
        return false;
    }
    
    // Allocate gsl_matrices
    gsl_matrix *ATA = gsl_matrix_alloc(9, 9);
    gsl_matrix *A = gsl_matrix_alloc(9,1);
    gsl_matrix *temp_ATA = gsl_matrix_alloc(9, 9);
    gsl_matrix *point = gsl_matrix_alloc(3,1);
    gsl_matrix *curpoint_p = gsl_matrix_alloc(3,1);
    gsl_matrix *prepoint_p = gsl_matrix_alloc(3,1);

    if (ATA == NULL || A == NULL || temp_ATA == NULL || point == NULL || curpoint_p == NULL || prepoint_p == NULL) {
        fprintf(stderr, "getHomographyFromPoints_Eigenvalue() ==>> Could not allocate memory for gsl matrix or vector: %.256s, %d\n", __FILE__, __LINE__);
        if(M != NULL){
            gsl_matrix_free(M);
        }
        if(invM != NULL){
            gsl_matrix_free(invM);
        }
        if(ATA != NULL){
            gsl_matrix_free(ATA);
        }
        if(A != NULL){
            gsl_matrix_free(A);
        }
        if(temp_ATA != NULL){
            gsl_matrix_free(temp_ATA);
        }
        if(point != NULL){
            gsl_matrix_free(point);
        }
        if(curpoint_p != NULL){
            gsl_matrix_free(curpoint_p);
        }
        if(prepoint_p != NULL){
            gsl_matrix_free(prepoint_p);
        }
        return false;
    }

    for(int32_t i = 0; i < num_features; ++i)
    {
        gsl_matrix_set(point, 0, 0, prefeatures[i*2]);
        gsl_matrix_set(point, 0, 1, prefeatures[i*2 +1]);
        gsl_matrix_set(point, 0, 2, 1.0);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, invM, point, 0.0, prepoint_p);
        
        gsl_matrix_set(point, 0, 0, curfeatures[i*2]);
        gsl_matrix_set(point, 0, 1, curfeatures[i*2 +1]);
        gsl_matrix_set(point, 0, 2, 1.0);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, invM, point, 0.0, curpoint_p);

        gsl_matrix_set(A, 0, 0, gsl_matrix_get(prepoint_p, 0, 0));
        gsl_matrix_set(A, 1, 0, gsl_matrix_get(prepoint_p, 1, 0));
        gsl_matrix_set(A, 2, 0, 1.0);
        gsl_matrix_set(A, 3, 0, 0.0);
        gsl_matrix_set(A, 4, 0, 0.0);
        gsl_matrix_set(A, 5, 0, 0.0);
        gsl_matrix_set(A, 6, 0, -1*gsl_matrix_get(curpoint_p, 0, 0)*gsl_matrix_get(prepoint_p, 0, 0));
        gsl_matrix_set(A, 7, 0, -1*gsl_matrix_get(curpoint_p, 0, 0)*gsl_matrix_get(prepoint_p, 1, 0));
        gsl_matrix_set(A, 8, 0, -1*gsl_matrix_get(curpoint_p, 0, 0));

        gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, A, A, 0.0, temp_ATA);
        gsl_matrix_add(temp_ATA, ATA);


        gsl_matrix_set(A, 0, 0, 0.0);
        gsl_matrix_set(A, 1, 0, 0.0);
        gsl_matrix_set(A, 2, 0, 0.0);
        gsl_matrix_set(A, 3, 0, gsl_matrix_get(prepoint_p, 0, 0));
        gsl_matrix_set(A, 4, 0, gsl_matrix_get(prepoint_p, 1, 0));
        gsl_matrix_set(A, 5, 0, 1.0);
        gsl_matrix_set(A, 6, 0, -1*gsl_matrix_get(curpoint_p, 0, 0)*gsl_matrix_get(prepoint_p, 0, 0));
        gsl_matrix_set(A, 7, 0, -1*gsl_matrix_get(curpoint_p, 0, 0)*gsl_matrix_get(prepoint_p, 1, 0));
        gsl_matrix_set(A, 8, 0, -1*gsl_matrix_get(curpoint_p, 0, 0));

        gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, A, A, 0.0, temp_ATA);
        gsl_matrix_add(temp_ATA, ATA);
        
    }

    if(temp_ATA != NULL){
        gsl_matrix_free(temp_ATA);
    }
    if(point != NULL){
        gsl_matrix_free(point);
    }
    if(curpoint_p != NULL){
        gsl_matrix_free(curpoint_p);
    }
    if(prepoint_p != NULL){
        gsl_matrix_free(prepoint_p);
    }
  
    //Calculate eigen decomposition
    uint32_t iter = 0;
    gsl_matrix *eigenvectors = gsl_matrix_alloc(9, 9);
    gsl_vector *eigenvalues = gsl_vector_alloc(9);
    if (eigenvectors == NULL || eigenvalues == NULL) {
        fprintf(stderr, "getHomographyFromPoints_Eigenvalue() ==>> Could not allocate memory for gsl matrix or vector: %.256s, %d\n", __FILE__, __LINE__);
        if(M != NULL){
            gsl_matrix_free(M);
        }
        if(invM != NULL){
            gsl_matrix_free(invM);
        }
        if(ATA != NULL){
            gsl_matrix_free(ATA);
        }
        if(A != NULL){
            gsl_matrix_free(A);
        }
        if(eigenvectors != NULL){
            gsl_matrix_free(eigenvectors);
        }
        if(eigenvalues != NULL){
            gsl_vector_free(eigenvalues);
        }
        return false;
    }

    if (!gsl_eigen_jacobi(ATA, eigenvalues, eigenvectors, 100, &iter))
    {
        fprintf(stderr, "getHomographyFromPoints_Eigenvalue() ==>> gsl_eigen_jacobi() failed, %.256s, %d\n", __FILE__, __LINE__);
        if(M != NULL){
            gsl_matrix_free(M);
        }
        if(invM != NULL){
            gsl_matrix_free(invM);
        }
        if(ATA != NULL){
            gsl_matrix_free(ATA);
        }
        if(A != NULL){
            gsl_matrix_free(A);
        }
        if(eigenvectors != NULL){
            gsl_matrix_free(eigenvectors);
        }
        if(eigenvalues != NULL){
            gsl_vector_free(eigenvalues);
        }
        return false;
    }

    //Find the smallest eigen values
    double smallest_eigenvalue = DBL_MAX; 
    size_t smallest_index = 0;
    for(size_t i = 0; i < 9; ++i)
    {
        double eigenvalue = fabs(gsl_vector_get(eigenvalues, i));
        if(eigenvalue < smallest_eigenvalue)
        {
            smallest_index = i;
            smallest_eigenvalue = eigenvalue;
        }
    }

    double hi[3][3];
    hi[0][0] = gsl_matrix_get(eigenvectors, smallest_index, 0); //[0*9+ smallest_index];
    hi[0][1] = gsl_matrix_get(eigenvectors, smallest_index, 1); //[1*9+ smallest_index];
    hi[0][2] = gsl_matrix_get(eigenvectors, smallest_index, 2); //[2*9+ smallest_index];

    hi[1][0] = gsl_matrix_get(eigenvectors, smallest_index, 3); //[3*9+ smallest_index];
    hi[1][1] = gsl_matrix_get(eigenvectors, smallest_index, 4); //[4*9+ smallest_index];
    hi[1][2] = gsl_matrix_get(eigenvectors, smallest_index, 5); //[5*9+ smallest_index];
    
    hi[2][0] = gsl_matrix_get(eigenvectors, smallest_index, 6); //[6*9+ smallest_index];
    hi[2][1] = gsl_matrix_get(eigenvectors, smallest_index, 7); //[7*9+ smallest_index];
    hi[2][2] = gsl_matrix_get(eigenvectors, smallest_index, 8); //[8*9+ smallest_index];
    
    smallest_eigenvalue = 1.0/hi[2][2];
    scale33(smallest_eigenvalue, hi, hi);
    
    Convert2ImageCoordinate33(hi, h,  intrisicM);
    return true;
}

int32_t getHomographyFromPoints_RANSAC(double *prefeature, double *curfeature, int32_t num_features, double intrisicM[3][3], double h[3][3])
{
    double d[3], d3[3];
    double  s;
    int32_t i, j, k, bestk, nm, index[9], p1, flag;
    double besthomo[3][3];
    double *features1, *features2;
    double homo_loc[3][3];
    double min_offset = 0.4;
    //normalize the data
    //num_features = 9;
    //convert to image coordinate system
    if(num_features < 8)
    {
        printf("too few points to be used for ransac method here\n");
        return 0;
    }
    
    features1 = (double *)malloc(sizeof(double)*num_features*2);
    if (features1 == NULL)
    {
        printf("getHomographyFromPoints_RANSAC() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        return 0;
    }
    
    features2 = (double *)malloc(sizeof(double)*num_features*2);
    if (features2 == NULL)
    {
        printf("getHomographyFromPoints_RANSAC() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        free(features1);
        return 0;
    }
    
    
    
    bestk = 0;
    for(j = 0; j < RANSAC_MAX_ITERATIONS; ++j)
    {
        nm = 0;
        while(nm < 4)
        {
            p1 = (int32_t)(rand()*num_features/RAND_MAX);
            flag = 1;
            
            for(i = 0; i < nm; ++i)
            {
                if(index[i] == p1)
                    flag = 0;
            }
            
            if(flag == 1)
            {
                //printf("p1 = %d\n", p1);
                features1[nm*2+0] = prefeature[p1*2+0];
                features1[nm*2+1] = prefeature[p1*2+1];
                features2[nm*2+0] = curfeature[p1*2+0];
                features2[nm*2+1] = curfeature[p1*2+1];
                index[nm] = p1;
                nm++;
            }
        }
        //pfc=0
        getHomographyFromPoints_Eigenvalue(features1, features2, 4, intrisicM, homo_loc);
        
        for(i = 0, k= 0; i < num_features; ++i)
        {
            d[0] = prefeature[i*2+0];
            d[1] = prefeature[i*2+1];
            d[2] = 1.0;
            mult331(homo_loc, d, d3);
            d3[0] = d3[0]/d3[2];
            d3[1] = d3[1]/d3[2];
            d[0] = curfeature[i*2+0];
            d[1] = curfeature[i*2+1];
            d[2] = 1.0;
            d3[0] -=d[0];
            d3[1] -=d[1];
            s = sqrt(d3[0]*d3[0] + d3[1] *d3[1]);
            if(s < min_offset)
            {
                k++;
            }
        }
        //	printf("k = %d best k %d\n", k, bestk);
        if(k > bestk)
        {
            copy33(homo_loc, besthomo);
            bestk = k;
        }
    }
    for(i = 0, k= 0; i < num_features; ++i)
    {
        d[0] = prefeature[i*2+0];
        d[1] = prefeature[i*2+1];
        d[2] = 1.0;
        mult331(besthomo, d, d3);
        d3[0] = d3[0]/d3[2];
        d3[1] = d3[1]/d3[2];
        d[0] = curfeature[i*2+0];
        d[1] = curfeature[i*2+1];
        d[2] = 1.0;
        d3[0] -=d[0];
        d3[1] -=d[1];
        s = sqrt(d3[0]*d3[0] + d3[1] *d3[1]);
        if(s < min_offset)
        {
            features1[k*2+0] = prefeature[i*2+0];
            features1[k*2+1] = prefeature[i*2+1];
            features2[k*2+0] = curfeature[i*2+0];
            features2[k*2+1] = curfeature[i*2+1];
            //printf("i = %d pre %f %f cur %f %f diff %f\n", i, feature_41[k][0], feature_41[k][1],
            //feature_42[k][0], feature_42[k][1], s);
            k++;
        }
    }
    // to compute the epipole on both images
    if(k > 4)
    {
        printf("total points used in homography are %d\n", k);
        getHomographyFromPoints_Eigenvalue(features1, features2, k, intrisicM, h);
        
    }
    
    free(features1);
    free(features2);
    return 1;
}

int32_t getHomographyFromPoints(double *points2d1,
                                double *points2d2, int32_t num_pts_plane, double homo[3][3])
{
    //double *r, *rt, *l;
    double  a[64], inva[64], b[8], m[8];
    double ipoint[2];
    double ipoint1[2];
    double r[8];
    double rtr[64];
    int32_t i, k, j;
    
    for(j = 0; j < 64; ++j)
    {
        a[j] = 0.0;
    }
    for(j = 0; j < 8; ++j)
    {
        b[j] = 0.0;
    }
    for(i = 0,k = 0; i < num_pts_plane; ++i)
    {
        ipoint[0] = points2d1[i*2+0];
        ipoint[1] = points2d1[i*2+1];
        ipoint1[0] = points2d2[i*2+0];
        ipoint1[1] = points2d2[i*2+1];
        //printf("i = %d point from %f %f to  %f %f\n", i, ipoint[0], ipoint[1], ipoint1[0], ipoint1[1]);
        r[  0] = (double)ipoint[0];
        r[  1] = (double)ipoint[1];
        r[  2] = 1.0;
        r[  3] = 0.0;
        r[  4] = 0.0;
        r[  5] = 0.0;
        r[  6] = (double)(-ipoint[0]*ipoint1[0]);
        r[  7] = (double)(-ipoint1[0]*ipoint[1]);
        LinearTransformD(r, r, rtr, 8, 1, 8);
        for(j = 0; j < 64; ++j)
        {
            a[j] +=rtr[j];
        }
        
        for(j = 0; j < 8; ++j)
        {
            b[j] +=r[j]*(double)ipoint1[0];
        }
        
        
        r[0] = 0.0;
        r[1] = 0.0;
        r[2] = 0.0;
        r[3] = (double)ipoint[0];
        r[4] = (double)ipoint[1];
        r[5] = 1.0;
        r[6] = (double)(-ipoint[0]*ipoint1[1]);
        r[7] = (double)(-ipoint[1]*ipoint1[1]);
        LinearTransformD(r, r, rtr, 8, 1, 8);
        for(j = 0; j < 64; ++j)
        {
            a[j] +=rtr[j];
        }
        
        for(j = 0; j < 8; ++j)
        {
            b[j] +=r[j]*(double)ipoint1[1];
        }
        
    }
    
    
    if(InvertMatrixD(a, inva, 8, 8) == 0)
    {
        
        return (0);
    }
    
    LinearTransformD(inva, b, m, 8, 8, 1);
    homo[0][0] = m[0];
    homo[0][1] = m[1];
    homo[0][2] = m[2];
    
    homo[1][0] = m[3];
    homo[1][1] = m[4];
    homo[1][2] = m[5];
    
    homo[2][0] = m[6];
    homo[2][1] = m[7];
    homo[2][2] = 1.0;
    //printf("homo %f %f %f %f %f %f %f %f\n", m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
    
    
    return 1;
}


int32_t getHomographyFromPointsNormalize(double *points2d1,
                                         double *points2d2, int32_t num_pts_plane, double homo[3][3])
{
    double *r, *rt, *l;
    double  a[64], inva[64], b[8], m[8];
    double ipoint[2];
    double ipoint1[2];
    double p10[2], p20[2];
    int32_t i, k;
    
    r = (double *)malloc(sizeof(double)*num_pts_plane*16);
    if (r == NULL)
    {
        printf("getHomographyFromPointsNormalize() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        return 0;
    }
    
    rt = (double *)malloc(sizeof(double)*num_pts_plane*16);
    if (rt == NULL)
    {
        free(r);
        printf("getHomographyFromPointsNormalize() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        return 0;
    }
    
    l = (double *)malloc(sizeof(double)*num_pts_plane*2);
    if (l == NULL)
    {
        free(r);
        free(rt);
        printf("getHomographyFromPointsNormalize() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        return 0;
    }
    
    p10[0] = 0;
    p10[1] = 0;
    p20[0] = 0;
    p20[1] = 0;
    for(i = 0; i < num_pts_plane; ++i)
    {
        p10[0] += points2d1[i*2+0];
        p10[1] += points2d1[i*2+1];
        p20[0] += points2d2[i*2+0];
        p20[1] += points2d2[i*2+1];
    }
    p10[0] = p10[0]/num_pts_plane;
    p10[1] = p10[1]/num_pts_plane;
    p20[0] = p20[0]/num_pts_plane;
    p20[1] = p20[1]/num_pts_plane;
  
    for(i = 0,k = 0; i < num_pts_plane; ++i)
    {
        ipoint[0] = points2d1[i*2+0] - p10[0];
        ipoint[1] = points2d1[i*2+1] - p10[1];
        ipoint1[0] = points2d2[i*2+0] - p20[0];
        ipoint1[1] = points2d2[i*2+1] - p20[1];
        //printf("i = %d point from %f %f to  %f %f\n", i, ipoint[0], ipoint[1], ipoint1[0], ipoint1[1]);
        r[k*16 + 0] = (double)ipoint[0];
        r[k*16 + 1] = (double)ipoint[1];
        r[k*16 + 2] = 1.0;
        r[k*16 + 3] = 0.0;
        r[k*16 + 4] = 0.0;
        r[k*16 + 5] = 0.0;
        r[k*16 + 6] = (double)(-ipoint[0]*ipoint1[0]);
        r[k*16 + 7] = (double)(-ipoint1[0]*ipoint[1]);
        l[i*2 +0] = (double)ipoint1[0];
        r[k*16 + 8] = 0.0;
        r[k*16 + 9] = 0.0;
        r[k*16 + 10] = 0.0;
        r[k*16 + 11] = (double)ipoint[0];
        r[k*16 + 12] = (double)ipoint[1];
        r[k*16 + 13] = 1.0;
        r[k*16 + 14] = (double)(-ipoint[0]*ipoint1[1]);
        r[k*16 + 15] = (double)(-ipoint[1]*ipoint1[1]);
        l[i*2 +1] = (double)ipoint1[1];
        k++;
    }
    TransposeMatrixD(r, rt, num_pts_plane*2, 8);
    LinearTransformD(rt, r, a, 8, num_pts_plane*2, 8);
    LinearTransformD(rt, l, b, 8, num_pts_plane*2, 1);
    
    if(InvertMatrixD(a, inva, 8, 8) == 0)
    {
        free(r);
        free(rt);
        free(l);
        return (0);
    }
    
    LinearTransformD(inva, b, m, 8, 8, 1);
    homo[0][0] = m[0];
    homo[0][1] = m[1];
    homo[0][2] = m[2];
    
    homo[1][0] = m[3];
    homo[1][1] = m[4];
    homo[1][2] = m[5];
    
    homo[2][0] = m[6];
    homo[2][1] = m[7];
    homo[2][2] = 1.0;
    
    
    p10[0] = -p10[0];
    p10[1] = -p10[1];
    p20[0] = -p20[0];
    p20[1] = -p20[1];
    ShiftHomographyOrigin(homo, p10, p20);
    //printf("homo %f %f %f %f %f %f %f %f\n", m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
    
    free(r);
    free(rt);
    free(l);
    return 1;
}


int32_t getHomographyFromPoints_RANSAC_frame(double *prefeature, double *curfeature, int32_t num_features, double h[3][3], double tol)
{
    double d[3], d3[3];
    double  s;
    int32_t i, j, k, bestk, nm, index[9], p1, flag;
    double besthomo[3][3];
    double *features1, *features2;
    double homo_loc[3][3];
    double min_offset;
    double px_norm, py_norm, norm_ratio; //this is for test the correcness of homography;
    int32_t iter;
    min_offset = tol;
    //int32_t heap_ok;
    //normalize the data
    //num_features = 9;
    //convert to image coordinate system
    if(num_features < 5)
    {
        printf("getHomographyFromPoints_RANSAC_frame() ==>> too few points to be used for ransac method here, , %.256s, %d\n", __FILE__, __LINE__);
        return -1;
    }
    
    //heap_ok = check_for_heap_integrity(__FILE__, __LINE__);
    
    //add checking for return NULL
    features1 = (double *)malloc(sizeof(double)*num_features*2);
    if (features1 == NULL)
    {
        printf("getHomographyFromPoints_RANSAC_frame() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        return -1;
    }
    
    features2 = (double *)malloc(sizeof(double)*num_features*2);
    if (features2 == NULL)
    {
        free(features1);
        printf("getHomographyFromPoints_RANSAC_frame() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        return -1;
    }
    
    
    bestk = 0;
    for(j = 0; j < RANSAC_MAX_ITERATIONS; ++j)
    {
        nm = 0;
        while(nm < 4)
        {
            p1 = (int32_t)(rand()%num_features);
            flag = 1;
            
            for(i = 0; i < nm; ++i)
            {
                if(index[i] == p1)
                    flag = 0;
            }
            
            if(flag == 1)
            {
                //printf("p1 = %d\n", p1);
                features1[nm*2+0] = prefeature[p1*2+0];
                features1[nm*2+1] = prefeature[p1*2+1];
                features2[nm*2+0] = curfeature[p1*2+0];
                features2[nm*2+1] = curfeature[p1*2+1];
                index[nm] = p1;
                nm++;
            }
        }
        //pfc=0
        if(getHomographyFromPointsNormalize(features1, features2, 4,  homo_loc) != 0)
        {
            px_norm = sqrt(homo_loc[0][0]*homo_loc[0][0] + homo_loc[0][1]*homo_loc[0][1]);
            py_norm = sqrt(homo_loc[1][0]*homo_loc[1][0] + homo_loc[1][1]*homo_loc[1][1]);
            if(px_norm > py_norm)
            {
                norm_ratio = py_norm/px_norm;
            }
            else
            {
                norm_ratio = px_norm/py_norm;
            }
            if(norm_ratio > 0.3)
            {
                for(i = 0, k= 0; i < num_features; ++i)
                {
                    d[0] = prefeature[i*2+0];
                    d[1] = prefeature[i*2+1];
                    d[2] = 1.0;
                    mult331(homo_loc, d, d3);
                    d3[0] = d3[0]/d3[2];
                    d3[1] = d3[1]/d3[2];
                    d[0] = curfeature[i*2+0];
                    d[1] = curfeature[i*2+1];
                    d[2] = 1.0;
                    d3[0] -=d[0];
                    d3[1] -=d[1];
                    s = sqrt(d3[0]*d3[0] + d3[1] *d3[1]);
                    if(s < min_offset)
                    {
                        k++;
                    }
                }
                
                if(k > bestk)
                {
                    copy33(homo_loc, besthomo);
                    //printf("besthomo\n");
                    // prt33(besthomo);
                    bestk = k;
                }
            }
        }
        else
        {
            free(features1);
            free(features2);
            printf("getHomographyFromPoints_RANSAC_frame() ==>> getHomographyFromPointsNormalize() failed, %.256s, %d\n", __FILE__, __LINE__);
            return -1;
        }
    }
    
    //heap_ok = check_for_heap_integrity(__FILE__, __LINE__);
    
    //	printf("best k = %d num_features %d\n", bestk, num_features);
    if(bestk > 10)
    {
        for(iter = 0; iter < 3; ++iter)
        {
            for(i = 0, k= 0; i < num_features; ++i)
            {
                d[0] = prefeature[i*2+0];
                d[1] = prefeature[i*2+1];
                d[2] = 1.0;
                mult331(besthomo, d, d3);
                d3[0] = d3[0]/d3[2];
                d3[1] = d3[1]/d3[2];
                d[0] = curfeature[i*2+0];
                d[1] = curfeature[i*2+1];
                d[2] = 1.0;
                d3[0] -=d[0];
                d3[1] -=d[1];
                s = sqrt(d3[0]*d3[0] + d3[1] *d3[1]);
                if(s < min_offset)
                {
                    features1[k*2+0] = prefeature[i*2+0];
                    features1[k*2+1] = prefeature[i*2+1];
                    features2[k*2+0] = curfeature[i*2+0];
                    features2[k*2+1] = curfeature[i*2+1];
                    // printf("i = %d pre %f %f cur %f %f diff %f\n", i, features1[k*2+0], features1[k*2+1],
                    // features2[k*2+0], features2[k*2+1], s);
                    k++;
                }
            }
            if(getHomographyFromPointsNormalize(features1, features2, k,  h) == 0)
            {
                free(features1);
                free(features2);
                printf("getHomographyFromPoints_RANSAC_frame() ==>> getHomographyFromPointsNormalize() failed, %.256s, %d\n", __FILE__, __LINE__);
                return -1;
            }
            
            copy33(h, besthomo);
        }
    }
    
    //heap_ok = check_for_heap_integrity(__FILE__, __LINE__);
    
    // to compute the epipole on both images
    if(k >= 4)
    {
        free(features1);
        free(features2);
        printf("best feat for homography %d\n", k);
        return k;
    }
    else
    {
        free(features1);
        free(features2);
        return -1;
    }
}




///
/// in this case x' = x + x0 y' = x + y0
int32_t ShiftHomographyOrigin(double homo[3][3], double p10[2], double p20[2])
{
    double s;
    double t[3];
    homo[0][2] = homo[0][2] + p10[0]*homo[0][0] + p10[1]*homo[0][1];
    homo[1][2] = homo[1][2] + p10[0]*homo[1][0] + p10[1]*homo[1][1];
    homo[2][2] = homo[2][2] + p10[0]*homo[2][0] + p10[1]*homo[2][1];
    s = 1.0/homo[2][2];
    scale33(s, homo, homo);
    scale3(-p20[0], homo[2], t);
    add3(t, homo[0], homo[0]);
    scale3(-p20[1], homo[2], t);
    add3(t, homo[1], homo[1]);
    return 1;
}