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

#include <math.h>                            // for fabs, sqrt
#include <stdio.h>                           // for printf, fprintf, stdout
#include <stdlib.h>                          // for free, malloc, NULL

#include "landmark_tools/math/double_matrix.h"
#include "landmark_tools/math/math_utils.h"  // for SIGN, SQR

//TODO replace all matrix operations with GSL
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#define DB_MATRIX_MAXDIM	32			//!< The maximum dimension of a matrix

// local typedef for ease of displaying code---should not impact efficiency
typedef double vec2[2], vec3[3], vec4[4];

void ZeroMatrixD (double *m, int64_t nRows, register int64_t nCols)
{
	register double *p;
	register int64_t i;
	
	for (i = nRows * nCols, p = m; i--; *p++ = 0.0F) ;	/* Clear matrix */
}


void TransposeMatrixD(register const double *from, register double *to, int64_t nRows, register int64_t nCols)
{
	register int64_t i, j;
	register const double *p;
	double temp[DB_MATRIX_MAXDIM][DB_MATRIX_MAXDIM];

	if (from == to) {
		for (i = 0, p = from; i < nRows; i++)	/* Copy matrix to temp */
			for (j = 0; j < nCols; j++)
				temp[i][j] = *p++;

		for (i = 0, p = from; i < nCols; i++)	/* Transpose and copy back */
			for (j = 0; j < nRows; j++)
				*to++ = temp[j][i];

	} else {
		for (i = nCols; i--; from++)		/* For each column */
			for (j = nRows, p = from; j--; p += nCols)
				*to++ = *p;			/* Copy columns to rows */
	}
}


int64_t InvertMatrixD (const double *M, double *Minv, int64_t nRows, register int64_t n)
{
	double *m;
	int64_t tallerBy = nRows - n;		/* Excess of rows over columns */
	register int64_t j, i;
	double b[DB_MATRIX_MAXDIM];
	double lu[DB_MATRIX_MAXDIM*DB_MATRIX_MAXDIM+DB_MATRIX_MAXDIM];

	/* Decompose matrix into L and U triangular matrices */
// was	if ((tallerBy < 0) || (MLLUdecompose(M, lu, n) == 0)) {
	if ((tallerBy < 0) || (LUDecomposeD(M, lu, n) == 0)) {
		return(0);		/* Singular */
	}

	/* Invert matrix by solving n simultaneous equations n times */
	for (i = 0, m = Minv; i < n; i++, m += n) {
		for(j = 0; j < n; j++)
			b[j] = 0;
		b[i] = 1;
	// was	MLLUsolve(lu, m, b, n);	/* Into a row of m */

		LUSolveD(lu, b, m, n);	/* Into a row of m */
	}
	
	/* Special post-processing for affine transformations (e.g. 4x3) */
	if (tallerBy) {			/* Affine transformation */
		register double *t = Minv+n*n;			/* Translation vector */
		m = Minv;			/* Reset m */
	// was	MLLinearTransformInPlace(t, m, tallerBy, n);	/* Invert translation */
		LinearTransformD(t, m, t, tallerBy, n, n);	/* Invert translation */
		for (j = tallerBy * n; n--; t++)
			*t = -*t;				/* Negate translation vector */
	}

	return(1);
}

#define luel(i, j)  lu[(i)*n+(j)]
#define ael(i, j)	a[(i)*n+(j)]
#define A(i,j)		a[(i)*n+(j)]

int64_t LUDecomposeD(
	register const double	*a,		
	register double		*lu, 	
	register int64_t			n		
)
{
	register int64_t i, j, k;
	int16_t pivotindex;
	double pivot, biggest, mult, tempf;
	register int64_t *ps;
	double scales[DB_MATRIX_MAXDIM];

	ps = (int64_t *)(&lu[n*n]); /* Memory for ps[] comes after LU[][] */

	for (i = 0; i < n; i++) {	/* For each row */
		/* Find the largest element in each row for row equilibration */
		biggest = 0.0;
		for (j = 0; j < n; j++)
			if (biggest < (tempf = fabs(luel(i,j) = ael(j,i)))) /* A transposed for row vectors */
				biggest = tempf;
		if (biggest != 0.0)
			scales[i] = 1.0 / biggest;
		else {
			scales[i] = 0.0;
			return(0);	/* Zero row: singular matrix */
		}

		ps[i] = i;		/* Initialize pivot sequence */
	}

	for (k = 0; k < n-1; k++) { /* For each column */
		/* Find the largest element in each column to pivot around */
		biggest = 0.0;
		for (i = k; i < n; i++) {
			if (biggest < (tempf = fabs(luel(ps[i],k)) * scales[ps[i]])) {
				biggest = tempf;
				pivotindex = (int16_t)i;
			}
		}
		if (biggest == 0.0)
			return(0);	/* Zero column: singular matrix */
		if (pivotindex != k) {	/* Update pivot sequence */
			j = ps[k];
			ps[k] = ps[pivotindex];
			ps[pivotindex] = j;
		}

		/* Pivot, eliminating an extra variable each time */
		pivot = luel(ps[k],k);
		for (i = k+1; i < n; i++) {
			luel(ps[i],k) = mult = luel(ps[i],k) / pivot;
			if (mult != 0.0) {
				for (j = k+1; j < n; j++)
					luel(ps[i],j) -= mult * luel(ps[k],j);
			}
		}
	}
	return(luel(ps[n-1],n-1) != 0.0);	/* 0 if singular, 1 if not */
}	


void LUSolveD(
	register const double	*lu,	
	register const double	*b,		
	register double			*x,		
	register int64_t			n		
)
{
	register int64_t i, j;
	double dot;
	register const int64_t *ps;
	
	ps = (const int64_t *)(&lu[n*n]); /* Memory for ps[] comes after LU[][] */

	/* Vector reduction using U triangular matrix */
	for (i = 0; i < n; i++) {
		dot = 0.0;
		for (j = 0; j < i; j++)
			dot += luel(ps[i],j) * x[j];
		x[i] = b[ps[i]] - dot;
	}

	/* Back substitution, in L triangular matrix */
	for (i = n-1; i >= 0; i--) {
		dot = 0.0;
		for (j = i+1; j < n; j++)
			dot += luel(ps[i],j) * x[j];
		x[i] = (x[i] - dot) / luel(ps[i],i);
	}
}	/* LUSolve */


void
LinearTransformD(
	const double		*L,		/* The left matrix */
	const double		*R,		/* The right matrix */
	register double	*P,		/* The resultant matrix */
	int64_t			nRows,	/* The number of rows of the left and resultant matrices */
	int64_t			lCol,	/* The number of columns in the left matrix */
	int64_t			rCol	/* The number of columns in the resultant matrix */
)
{
	register const double *lp;		/* Left matrix pointer for dot product */
	register const int8_t *rp;		/* Right matrix pointer for dot product */
	register int64_t k;				/* Loop counter */
	register double sum;			/* Extended precision for intermediate results */
	register int64_t rowBytes = lCol * sizeof(double);
	register int64_t rRowBytes = rCol * sizeof(double);
	register int64_t j, i;				/* Loop counters */
	register int64_t lRowBytes = lCol * sizeof(double);
	const int8_t *lb = (const int8_t*)L;
	double temp[DB_MATRIX_MAXDIM*DB_MATRIX_MAXDIM]; // Temporary storage for in-place transformations 
	register double *tp;
	
	if (P == L) {  // IN PLACE
		double *op = P;				/* Output geometry */
		for (i = nRows; i--; lb += rowBytes) {	/* Each row in L */
			{	
				for (k = lCol, lp = (double*)lb, tp = &temp[0]; k--; )
					*tp++ = *lp++;			/* Copy one input vector to temp storage */
			}
			for (j = 0; j < lCol; j++) {		/* Each column in R */
				lp = &temp[0];				/* Left of ith row of L */
				rp = (const int8_t *)(R + j);	/* Top of jth column of R */
				sum = 0;
				for (k = lCol; k--; rp += rowBytes)
					sum += *lp++ * (*((const double*)rp));	/* *P += L[i'][k'] * R[k'][j] */
				*op++ = sum;
			}
		}
	} else if (P != R) {
		for (i = nRows; i--; lb += lRowBytes) {	/* Each row in L */
			for (j = 0; j < rCol; j++) {	/* Each column in R */
				lp = (const double *)lb;		/* Left of ith row of L */
				rp = (const int8_t *)(R + j);	/* Top of jth column of R */
				sum = 0;
				for (k = lCol; k--; rp += rRowBytes)
					sum += *lp++ * (*((const double*)rp));	/* *P += L[i'][k'] * R[k'][j] */
				*P++ = sum;
			}
		}
	} else { // P == R
		for (tp = temp, i = lCol * rCol; i--; ) *tp++ = *R++;  // copy R
		for (i = nRows; i--; lb += lRowBytes) {	/* Each row in L */
			for (j = 0; j < rCol; j++) {	/* Each column in R */
				lp = (const double *)lb;		/* Left of ith row of L */
				rp = (const int8_t *)(temp + j);	/* Top of jth column of R (now in temp) */
				sum = 0;
				for (k = lCol; k--; rp += rRowBytes)
					sum += *lp++ * (*((const double*)rp));	/* *P += L[i'][k'] * R[k'][j] */
				*P++ = sum;
			}
		}
	} 
}

void
SubtractMatrixD (double *A, double *B, double *C, int64_t m, int64_t n)
{
	register int64_t i, index = m * n;
	for (i = 0; i < index; i++)
		C[i] = A[i] - B[i];
}

/* A * B + C = D */

void
LinearAddMatrixD (double A, double *B, double *C, double *D, int64_t m, int64_t n)
{
	register int64_t i, index = m * n;
	for (i = 0; i < index; i++)
		D[i] = A * B[i] + C[i];
}

void
AddMatrixD (double *A, double *B, double *result, int64_t m, int64_t n)
{
	int64_t i;
	for (i = m * n; i --; A++, B++, result ++)
		*result = (*A) + (*B);
}

#undef luel
#undef ael
#undef A
