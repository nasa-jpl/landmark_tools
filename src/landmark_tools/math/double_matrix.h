/************************************************
 * \file double_matrix.h
 * \brief Generic Matrix procedures
 * \section updates Update History
 * - Updated: September 26, 2024 Cecilia Mauceri : Use GSL 
  
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

#ifndef _LANDMARK_TOOLS_DOUBLE_MATRIX_H_
#define _LANDMARK_TOOLS_DOUBLE_MATRIX_H_

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief ZeroMatrixD() sets the given (nRows x nCols) matrix to a zero matrix.
 
 \param[] m 
 \param[] nRows 
 \param[] nCols 
*/
void ZeroMatrixD(double *m, int64_t nRows, register int64_t nCols);

/**
 \brief TransposeMatrixD() copies the transpose of matrix M into matrix N
 *
 * This works for rectangular matrices as well, transposing a (nRows x nCols) matrix into a (nCols x nRows) matrix.
 *
 * IN PLACE SUPPORTED
 
 \param[] from 
 \param[] to 
 \param[] nRows 
 \param[] nCols 
*/
void TransposeMatrixD(register const double *from, register double *to,
                      int64_t nRows, register int64_t nCols);

/**
 * \brief InvertMatrixD() inverts square matrices
 *
 *	With tall matrices, invert upper part and transform the bottom rows as would be expected if embedded into a larger matrix.
 *
 *	Undefined for wide matrices.
 *
 * IN PLACE SUPPORTED, no performance difference
 *
 * \param[in] M matrix to invert
 * \param[out] Minv the inverted matrix
 * \param[in] nRows number of rows in M
 * \param[in] n number of cols in M?
 * \return 1 is returned if the matrix was non-singular and the inversion was successful; 0 is returned if the matrix was singular and the inversion failed.
*/
int64_t InvertMatrixD(const double *M, double *Minv, int64_t nRows,
                      register int64_t n);

/**
 \brief LUDecompose() decomposes the coefficient matrix A into upper and lower
 * triangular matrices, the composite being the LU matrix.
 *
 * This is then followed by multiple applications of LUSolveD(),
 * to solve several problems with the same system matrix.
 * 
 * \param[in] a the (n x n) coefficient matrix
 * \param[out] lu the (n x n) lu matrix augmented by an (n x 1) pivot sequence
 * \param[in] n the order of the matrix
 * \return 1 is returned if the matrix is non-singular and the decomposition was successful; 0 is returned if the matrix is singular and the decomposition failed.
*/
int64_t LUDecomposeD(
    register const double *a,
    register double
        *lu,
    register int64_t n
);

/**
 \brief LUSolveD() solves the linear equation (xA = b) after the matrix A has
 * been decomposed with LUDecompose()
 *
 * (xA = b)  is equivalent to (xUL = b).
 
 \param[in] lu the decomposed LU matrix
 \param[in] b the constant vector
 \param[out] x the solution vector
 \param[in] n the order of the equation
*/ 
void LUSolveD(register const double *lu, 
              register const double *b,  
              register double *x,       
              register int64_t n       
);

/********************************************************************************
 * \brief Linear transformations, for transforming vectors and matrices.
 *
 * This works for row vectors and column vectors alike.
 *	L[nRows][lCol]	- input (left) matrix
 *	rg[lCol][rCol]	- transformation (right) matrix
 *	P[nRows][rCol]	- output (product) matrix
 *
 * Examples:
 * v[3] * M[3][3] -> w[3] :			MLLinearTransform(&v[0], &M[0][0], &w[0], 1, 3, 3);
 * M[3][3] * v[3] -> w[3] :			MLLinearTransform(&M[0][0], &v[0], &w[0], 3, 3, 1);
 * M[4][4] * N[4][4] -> P[4][4]:	MLLinearTransform(&M[0][0], &N[0][0], &P[0][0], 4, 4, 4);
 * v[4] * M[4][3] -> w[3]:			MLLinearTransform(&v[0], &M[0][0], &w[0], 1, 4, 3);
 * v[3] tensor w[3] -> T[3][3]:		MLLinearTransform(&v[0], &w[0], T[3][3], 3, 1, 3);
 *
 * This can be used In Place, i.e., to transform the left matrix
 * by the right matrix, placing the result back in the left.  By its nature,
 * then, this can only be used for transforming row vectors or concatenating
 * matrices from the right.
 * 
 * \param[in] L The left matrix
 * \param[in] R The right matrix
 * \param[out] P The resultant matrix
 * \param[in] nRows The number of rows of the left and resultant matrices
 * \param[in] lCol The number of columns in the left matrix
 * \param[in] rCol The number of columns in the resultant matrix
 ********************************************************************************/
void LinearTransformD(
    const double *L,    
    const double *R,   
    register double *P, 
    int64_t nRows,
    int64_t lCol,  
    int64_t rCol 
);

void SubtractMatrixD(double *A, double *B, double *C, int64_t m, int64_t n);

void LinearAddMatrixD(double A, double *B, double *C, double *D, int64_t m,
                      int64_t n);

void AddMatrixD(double *A, double *B, double *result, int64_t m, int64_t n);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LANDMARK_TOOLS_DOUBLE_MATRIX_H_
