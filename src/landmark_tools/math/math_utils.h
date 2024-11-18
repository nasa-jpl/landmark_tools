/************************************************
 * \file math_utils.h
 * \brief Math functions for 3x3 matrices.
 * 
 * \section updates Update History
 * - Updated: September 26, 2024 Cecilia Mauceri : Used GSL for svd, jacobi, and inv
 *
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
 ***********************************************/


#ifndef _LANDMARK_TOOLS_MATH_UTILS_H_
#define _LANDMARK_TOOLS_MATH_UTILS_H_

#include <inttypes.h>
#include <math.h>    // for fabs
#include <stdint.h>  // for int32_t, int64_t
#include <stdio.h>
#include <stdbool.h>

#include <gsl/gsl_matrix.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAX(a,b) ((a>=b)? a : b)
#define MIN(a,b) ((a<=b)? a : b)

#define SQR(x)        ((x)*(x))
#define SIGN(a,b)    ((b) >= 0.0 ? fabs(a) : -fabs(a))

/**
 \brief TODO
 
 \param[] v 
 \param[] a 
 \return int32_t 
*/
int32_t skew(double v[3], double a[3][3]);

/**
 \brief TODO
 
 \param[] R 
*/
void normalizeRotation(double R[3][3]);

/**
 \brief TODO
 
 \param[] R 
*/
void normalizeRotation_quaternion_method(double R[3][3]);

/**
 \brief TODO
 
 \param[] a 
 \return double 
*/
double frobenius_norm(double a[3][3]);

/**
 \brief TODO
 
 \param[] a 
 \return double 
*/
double trace33(double a[3][3]);

/**
 \brief
 \param[in] a
 \param[out] s singluar values
 \param[out] v right singular values
 \return true on success
 \return false on error
 */
bool svd33(double a[3][3], double s[3], double v[3][3]);

/**
 \brief
 \param[in] a
 \param[out] w eigenvalues
 \param[out] v eigenvectors
 \return true on success
 \return false on error
 */
bool jacobi33(double a[3][3], double w[3], double v[3][3]);

/**
 \brief Print a length 3 vector
 
 \param[in] a 
*/
void prt3(double a[3]);

/**
 \brief Print a 3x3 matrix
 
 \param[in] a 
*/
void prt33(double a[3][3]);

/**
 \brief TODO
 \param[in] a input matrix
 \param[out] b output matrix
 \return b
 */
double ((*inv33_stable(double a[3][3], double b[3][3]))[3]);

/**
 \brief Invert `A_gsl`
 
 \param[in] A_gsl 
 \return gsl_matrix* 
*/
gsl_matrix* gsl_matrix_inverse(gsl_matrix *A_gsl);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_MATH_UTILS_H_ */
