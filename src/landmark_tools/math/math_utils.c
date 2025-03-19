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


#include <stdio.h>                               // for printf, NULL

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "landmark_tools/math/math_constants.h"  // for PI
#include "landmark_tools/math/math_utils.h"
#include "math/mat3/mat3.h"                      // for mult333, zero33, copy33

int32_t skew(double v[3], double sk[3][3])
{
    zero33(sk);
    sk[0][1] = -v[2];
    sk[0][2] = v[1];
    sk[1][0] = v[2];
    sk[1][2] = -v[0];
    sk[2][0] = -v[1];
    sk[2][1] = v[0];
    return 1;
}

void normalizeRotation_quaternion_method(double R[3][3])
{
    double q[4];
    quatr(R, q);
    rotq(q, R);
    // printf("detr %f\n", det33(R));
}

 
void normalizeRotation(double R[3][3])
{
       double v[3], w[3][3], u[3][3], wt[3][3];
       copy33(R, u);
       svd33(u, v, w);
       trans33(w, wt);
       mult333(u, wt, R);

       if(det33(R) < 0.0)
       {
             scale33(-1, wt, wt);
             mult333(u, wt, R);
       }
}

double frobenius_norm(double a[3][3])
{
    double f;
    int32_t i, j;
    f = 0;
    for(i = 0; i < 3; ++i)
    {
        for(j = 0; j < 3; ++j)
        {
            f += a[i][j]*a[i][j];
        }
    }
  return f;
}

double trace33(double a[3][3])
{
    double f;
    f = a[0][0] + a[1][1] + a[2][2];
    return f;
}

bool svd33(double a[3][3], double s[3], double v[3][3])
{
    // Allocate gsl_matrix for input matrix `A`
    gsl_matrix *A = gsl_matrix_alloc(3, 3);
    if (A == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for matrix A\n");
        return false;
    }
    for (size_t i = 0; i < 3; i++) {
       for (size_t j = 0; j < 3; j++) {
           gsl_matrix_set(A, i, j, a[i][j]);
       }
    }

    // Allocate memory for the singular value decomposition components
    gsl_matrix *V = gsl_matrix_alloc(3, 3);
    gsl_vector *S = gsl_vector_alloc(3);
    gsl_vector *work = gsl_vector_alloc(3);
    if (V == NULL || S == NULL || work == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for gsl matrix\n");
        if(A != NULL)
            gsl_matrix_free(A);
        if(V != NULL)
            gsl_matrix_free(V);
        if(S != NULL)
            gsl_vector_free(S);
        if(work != NULL)
            gsl_vector_free(work);
        return false;
    }

    // Perform the Singular Value Decomposition (SVD)
    int status = gsl_linalg_SV_decomp(A, V, S, work);
    if (status != GSL_SUCCESS) {
        fprintf(stderr, "Error: SVD decomposition failed\n");
        if(A != NULL)
            gsl_matrix_free(A);
        if(V != NULL)
            gsl_matrix_free(V);
        if(S != NULL)
            gsl_vector_free(S);
        if(work != NULL)
            gsl_vector_free(work);
        return false;
    }

    // Extract the singular values
    for (size_t i = 0; i < 3; i++) {
        s[i] = gsl_vector_get(S, i);
    }

    // Extract the left singular vectors (U stored in `A`) and right singular vectors (`V`)
    for (size_t i = 0; i < 3; i++) {
       for (size_t j = 0; j < 3; j++) {
           a[i][j] = gsl_matrix_get(A, i, j);
           v[i][j] = gsl_matrix_get(V, i, j);
       }
    }

    // Free allocated memory
    if(A != NULL)
        gsl_matrix_free(A);
    if(V != NULL)
        gsl_matrix_free(V);
    if(S != NULL)
        gsl_vector_free(S);
    if(work != NULL)
        gsl_vector_free(work);
    
    return true;
}


bool jacobi33(double a[3][3], double w[3], double v[3][3])
{
    // Allocate gsl_matrix for the input matrix `a`
    gsl_matrix *A_gsl = gsl_matrix_alloc(3, 3);
    if (A_gsl == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for matrix A_gsl\n");
        return false;
    }
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            gsl_matrix_set(A_gsl, i, j, a[i][j]);
        }
    }

    // Allocate gsl_vector for the eigenvalues
    gsl_vector *eigenvalues = gsl_vector_alloc(3);
    gsl_matrix *eigenvectors = gsl_matrix_alloc(3, 3);
    gsl_eigen_symmv_workspace *workspace = gsl_eigen_symmv_alloc(3);
    
    if (eigenvalues == NULL || eigenvectors == NULL || workspace == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for gsl operation\n");
        if(workspace != NULL)
            gsl_eigen_symmv_free(workspace);
        if(A_gsl != NULL)
            gsl_matrix_free(A_gsl);
        if(eigenvalues != NULL)
            gsl_vector_free(eigenvalues);
        if(eigenvectors != NULL)
            gsl_matrix_free(eigenvectors);
        return false;
    }

    // Perform eigenvalue and eigenvector computation
    int status = gsl_eigen_symmv(A_gsl, eigenvalues, eigenvectors, workspace);
    if (status != GSL_SUCCESS) {
        fprintf(stderr, "Error: Eigenvalue/eigenvector computation failed\n");
        if(workspace != NULL)
            gsl_eigen_symmv_free(workspace);
        if(A_gsl != NULL)
            gsl_matrix_free(A_gsl);
        if(eigenvalues != NULL)
            gsl_vector_free(eigenvalues);
        if(eigenvectors != NULL)
            gsl_matrix_free(eigenvectors);
        return false;
    }

    // Optionally sort the eigenvalues and eigenvectors
    // gsl_eigen_symmv_sort(eigenvalues, eigenvectors, GSL_EIGEN_SORT_ABS_ASC);

    // Copy the eigenvalues to `w`
    for (size_t i = 0; i < 3; i++) {
        w[i] = gsl_vector_get(eigenvalues, i);
    }

    // Copy the eigenvectors to `v`
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            v[i][j] = gsl_matrix_get(eigenvectors, i, j);
        }
    }

    // Free allocated memory
    if(workspace != NULL)
        gsl_eigen_symmv_free(workspace);
    if(A_gsl != NULL)
        gsl_matrix_free(A_gsl);
    if(eigenvalues != NULL)
        gsl_vector_free(eigenvalues);
    if(eigenvectors != NULL)
        gsl_matrix_free(eigenvectors);
    return true;
}


void prt3(double a[3])
{
    printf("%18.16f %18.16f %18.16f\n", a[0], a[1], a[2]);
}
void prt33(double a[3][3])
{
    printf("%18.15f %18.15f %18.15f\n", a[0][0], a[0][1], a[0][2]);
    printf("%18.15f %18.15f %18.15f\n", a[1][0], a[1][1], a[1][2]);
    printf("%18.15f %18.15f %18.15f\n", a[2][0], a[2][1], a[2][2]);
}


double ((*inv33_stable(double a[3][3], double b[3][3]))[3])
{
    /* Check for NULL inputs */
    if ((a == NULL) || (b == NULL)){
        fprintf(stderr, "ERROR: Input to inv33_stable is NULL\n");
        return NULL;
    }

    /* Check for non-distinct output */
    if (a == b){
        fprintf(stderr, "ERROR: Input to inv33_stable is non-distinct\n");
        return NULL;
    }
    
    gsl_matrix *A_gsl = gsl_matrix_alloc(3, 3);
    if(A_gsl == NULL){
        fprintf(stderr, "Error: Could not allocate memory for matrix A_gsl\n");
        return NULL;
    }
    for(size_t i = 0; i<3; i++){
        for(size_t j = 0; j<3; j++){
            gsl_matrix_set(A_gsl, i, j, a[i][j]);
        }
    }

    // Invert matrix A_gsl
    gsl_matrix *inverse = gsl_matrix_inverse(A_gsl);
    if(inverse==NULL){
        fprintf(stderr, "Error: Could not allocate memory for inverse\n");
        if(A_gsl != NULL){
            gsl_matrix_free(A_gsl);
        }
        return NULL;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            b[i][j] = gsl_matrix_get(inverse, i, j);
        }
    }

    // Free memory
    if(A_gsl != NULL){
        gsl_matrix_free(A_gsl);
    }
    if(inverse != NULL){
        gsl_matrix_free(inverse);
    }
    return b;
    
}


gsl_matrix* gsl_matrix_inverse(gsl_matrix *A_gsl)
{
    /* Check for NULL inputs */
    if (A_gsl == NULL){
        fprintf(stderr, "ERROR: Input to gsl_matrix_inverse is NULL\n");
        return NULL;
    }

    // Permutation for LU decomposition
    gsl_permutation *p = gsl_permutation_alloc(3);
    if(p==NULL){
        fprintf(stderr, "Error: Could not allocate memory for gsl_permutation\n");
        return NULL;
    }

    // LU decomposition of A
    int signum;
    int status = gsl_linalg_LU_decomp(A_gsl, p, &signum);
    if (status != GSL_SUCCESS) {
            fprintf(stderr, "Error: LU decomposition failed\n");
            gsl_permutation_free(p);
            return NULL;
        }
    
    // Check for a singular matrix (determinant == 0)
    double det = gsl_linalg_LU_det(A_gsl, signum);
    if (det == 0) {
        fprintf(stderr, "Error: Matrix is singular and cannot be inverted\n");
        gsl_permutation_free(p);
        return NULL;
    }

    // Invert matrix A_gsl
    gsl_matrix *inverse = gsl_matrix_alloc(A_gsl->size1, A_gsl->size1);
    if(inverse==NULL){
        fprintf(stderr, "Error: Could not allocate memory for inverse\n");
        if(p != NULL){
            gsl_permutation_free(p);
        }
        return NULL;
    }
    status = gsl_linalg_LU_invert(A_gsl, p, inverse);
    if (status != GSL_SUCCESS) {
        fprintf(stderr, "Error: Matrix inversion failed\n");
        gsl_matrix_free(inverse);
        gsl_permutation_free(p);
        return NULL;
    }

    // Free memory
    if(p != NULL){
        gsl_permutation_free(p);
    }
    
    return inverse;
}
