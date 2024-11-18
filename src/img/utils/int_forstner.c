/******************************************************************************
*                                                                             *
*                           I N T _ F O R S T N E R                           *
*                                                                             *
*                                       Todd Litwin                           *
*                                       Written: 28 Aug 1995                  *
*                                       Updated: 24 Sep 2006                  *
*                                                                             *
*                                                                             *
*******************************************************************************


	This file contains a function to compute an interest image from an
	image. */

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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "imgutils.h"

static int nx_max;
static int n_max;
static int *m00_sums;
static int *m10_sums;
static int *m11_sums;


/******************************************************************************
********************************   INT_FORSTNER   *****************************
*******************************************************************************

    This function uses the interest operator developed by Forstner (?) to
    compute an interest image from an input image. The value at each point
    is computed from an NxN neighborhood around each pixel, where N is an
    odd number. If the neighborhood would overlap the image bounds, then the
    processing rectangle is shrunk so that this won't happen.

    Proper interest values are always non-negative, with smaller values being
    more interesting. The output image may have negative values in locations
    where there was some problem computing the interest; these points should
    be ignored.

    The output interest values are actually the largest eigenvalue of the
    covariance matrix. As mentioned above, these values are more interesting
    as they get smaller. In addition they are not distributed in a linear
    fashion with interest. To convert such a value to something more
    recognizable as an interest value, one could take each value output, I,
    and compute from it log(1.0/I).

    */

int int_forstner(
    const unsigned char *image,	/* input intensity image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int n,			/* input size of NxN interest neighborhood */
    float *interest)		/* output interest image */
{
    int ix, iy, ix_start, iy_start, ix_stop, iy_stop, row_offset, start_offset;
    int i, j, w, d0, d1, dd, first_row, first_col;
    const unsigned char *img;
    double a, b, d, det, m00 = 0, m01 = 0, m10 = 0, m11 = 0;
    int *m00sums, *m10sums, *m11sums, *m00p = 0, *m10p = 0, *m11p = 0, alloc;

    /* Check that N is odd */
    if ((n & 1) == 0) {
	printf("int_forstner(): N must be odd: %d\n", n);
	return FAILURE;
	}

    /* Use previously allocated buffers if available and appropriate */
    if (((nx + n) <= (nx_max + n_max)) &&
	(m00_sums != NULL) && (m10_sums != NULL) && (m11_sums != NULL)) {
	alloc = FALSE;
	m00sums = m00_sums;
	m10sums = m10_sums;
	m11sums = m11_sums;
	}

    /* Otherwise allocate summation buffers */
    else {
	alloc = TRUE;
	if ((m00sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner(): memory allocation error\n");
	    return FAILURE;
	    }
	if ((m10sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner(): memory allocation error\n");
	    free((char *)m00sums);
	    return FAILURE;
	    }
	if ((m11sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner(): memory allocation error\n");
	    free((char *)m00sums);
	    free((char *)m10sums);
	    return FAILURE;
	    }
	}

    /* Adjust the processed rectangle so that we stay within the image */
    w = n / 2;
    ix_start = w - x0;
    iy_start = w - y0;
    ix_stop  = xdim - w - x0 - 1;
    iy_stop  = ydim - w - y0 - 1;

    /* Compute how far we advance between rows */
    row_offset = xdim - nx;

    /* Move to the beginning of the first row to process */
    start_offset = (xdim * y0) + x0;
    image    += start_offset;
    interest += start_offset;

    /* Process each row */
    first_row = TRUE;
    for (iy=0; iy<ny; iy++) {

	/* Process each column in the row */
	first_col = TRUE;
	for (ix=0; ix<nx; ix++,image++,interest++) {

	    /* Make sure that we are within the correct processing region */
	    if ((ix < ix_start) || (ix > ix_stop) ||
		(iy < iy_start) || (iy > iy_stop)) {
		*interest = -1.0;
		continue;
		}

	    /* Compute the sums for the first row */
	    if (first_row) {

		/* For first column, do all column sums in NxN neighborhood */
		if (first_col) {
		    first_col = FALSE;

		    /* Zero out summations */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    for (j=1-w; j<w; j++,m00p++,m10p++,m11p++)
			*m00p = *m10p = *m11p = 0;
		    m00 = m10 = m11 = 0;

		    /* Sum entire NxN neighborhood, recording column sums */
		    for (i=1-w; i<w; i++) {
			m00p = m00sums;
			m10p = m10sums;
			m11p = m11sums;
			img = image + (i * xdim) + 1 - w;
			for (j=1-w; j<w; j++,img++,m00p++,m10p++,m11p++) {
			    d0 = img[1]    - img[-1];
			    d1 = img[xdim] - img[-xdim];
			    m00   += dd = (d0 * d0);	/* / 4.0 */
			    *m00p += dd;
			    m11   += dd = (d1 * d1);	/* / 4.0 */
			    *m11p += dd;
			    m10   += dd = (d1 * d0);	/* / 4.0 */
			    *m10p += dd;
			    }
			}
		    }

		/* For following columns, merely compute new column sum */
		else {

		    /* Compute sum for newest column */
		    *m00p = *m10p = *m11p = 0;
		    img = image + ((1-w) * xdim) + w - 1;
		    for (i=1-w; i<w; i++,img+=xdim) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p += (d0 * d0);	/* / 4.0 */
			*m11p += (d1 * d1);	/* / 4.0 */
			*m10p += (d1 * d0);	/* / 4.0 */
			}

		    /* Update sums by removing oldest and adding newest col */
		    i = 1 - (w << 1);
		    m00 += (m00p[0] - m00p[i]);
		    m10 += (m10p[0] - m10p[i]);
		    m11 += (m11p[0] - m11p[i]);
		    m00p++;
		    m10p++;
		    m11p++;
		    }
		}

	    /* For following rows, merely adjust the previous summations */
	    else {

		/* For first column, adjust all of the column sums */
		if (first_col) {
		    first_col = FALSE;

		    /* Remove old top terms from each column sum */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    img = image + (-w * xdim) + 1 - w;
		    for (i=1-w; i<w; i++,img++,m00p++,m10p++,m11p++) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p -= (d0 * d0);	/* / 4.0 */
			*m11p -= (d1 * d1);	/* / 4.0 */
			*m10p -= (d1 * d0);	/* / 4.0 */
			}

		    /* Add new bottom terms to each column sum */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    img = image + ((w-1) * xdim) + 1 - w;
		    for (i=1-w; i<w; i++,img++,m00p++,m10p++,m11p++) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p += (d0 * d0);	/* / 4.0 */
			*m11p += (d1 * d1);	/* / 4.0 */
			*m10p += (d1 * d0);	/* / 4.0 */
			}

		    /* Add up all column sums to get new summations */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    m00 = m10 = m11 = 0;
		    for (i=1-w; i<w; i++,m00p++,m10p++,m11p++) {
			m00 += *m00p;
			m10 += *m10p;
			m11 += *m11p;
			}
		    }

		/* For following columns, merely adjust new column sums */
		else {

		    /* Remove old top term from column sum */
		    img = image + (-w * xdim) + w - 1;
		    d0 = img[1]    - img[-1];
		    d1 = img[xdim] - img[-xdim];
		    *m00p -= (d0 * d0);	/* / 4.0 */
		    *m11p -= (d1 * d1);	/* / 4.0 */
		    *m10p -= (d1 * d0);	/* / 4.0 */

		    /* Add new bottom term to column sum */
		    img = image + ((w-1) * xdim) + w - 1;
		    d0 = img[1]    - img[-1];
		    d1 = img[xdim] - img[-xdim];
		    *m00p += (d0 * d0);	/* / 4.0 */
		    *m11p += (d1 * d1);	/* / 4.0 */
		    *m10p += (d1 * d0);	/* / 4.0 */

		    /* Update sums by removing oldest and adding newest col */
		    i = 1 - (w << 1);
		    m00 += (m00p[0] - m00p[i]);
		    m10 += (m10p[0] - m10p[i]);
		    m11 += (m11p[0] - m11p[i]);
		    m00p++;
		    m10p++;
		    m11p++;
		    }
		}

	    /* Invert the matrix to get covariance */
	    m01 = m10;
	    det = (m00 * m11) - (m01 * m10);
	    det /= 4.0;		/* to compensate for not dividing above */
	    if (det < 0.00001) {
		*interest = -2.0;
	        continue;
		}
	    a =  m11 / det;
	    d =  m00 / det;
	    b = -m01 / det;
	    *interest = ((a + d) + sqrt((a-d)*(a-d) + 4*b*b)) / 2.0;
	    }

	/* Move to the start of the next row */
	image    += row_offset;
	interest += row_offset;
	if (iy >= iy_start)
	    first_row = FALSE;
	}

    /* Free summation buffers */
    if (alloc) {
	free((char *)m00sums);
	free((char *)m10sums);
	free((char *)m11sums);
	}

    return SUCCESS;
    }


/******************************************************************************
********************************   INT_FORSTNER_ALLOC   ***********************
*******************************************************************************

    This function allocates internal buffers which are used instead of having
    the individual interest-generating functions allocate and free buffers
    with each call. If used, however, the functions become non-reentrant. */

int int_forstner_alloc(
    int nxmax,			/* input max number of columns */
    int nmax)			/* input max size of NxN int. neighborhood */
{
    /* Free any previously allocated buffers */
    int_forstner_free();

    /* Record the sizes we are allocating for */
    nx_max = nxmax;
    n_max  = nmax;

    if ((m00_sums = (int *)malloc((sizeof(int)) * (nxmax + nmax))) == NULL) {
	printf("int_forstner(): memory allocation error\n");
	return FAILURE;
	}
    if ((m10_sums = (int *)malloc((sizeof(int)) * (nxmax + nmax))) == NULL) {
	printf("int_forstner(): memory allocation error\n");
	free((char *)m00_sums);
	return FAILURE;
	}
    if ((m11_sums = (int *)malloc((sizeof(int)) * (nxmax + nmax))) == NULL) {
	printf("int_forstner(): memory allocation error\n");
	free((char *)m00_sums);
	free((char *)m10_sums);
	return FAILURE;
	}
    return SUCCESS;
    }


/******************************************************************************
********************************   INT_FORSTNER_BEST   ************************
*******************************************************************************

    This function uses the interest operator developed by Forstner (?) to
    find the most interesting point in an image. */

int int_forstner_best(
    const unsigned char *image,	/* input intensity image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int n,			/* input size of NxN neighborhood around pt */
    int *x,			/* output X coordinate of best point */
    int *y,			/* output Y coordinate of best point */
    double *intr)		/* output value of interest operation */
{
    int ix, iy, ix_start, iy_start, ix_stop, iy_stop, row_offset, start_offset;
    int i, j, w, d0, d1, bx = 0, by = 0, dd, first_row, first_col;
    const unsigned char *img;
    double a, b, d, det, m00 = 0, m01 = 0, m10 = 0, m11 = 0, val, bval;
    int *m00sums, *m10sums, *m11sums, *m00p = 0, *m10p = 0, *m11p = 0, alloc;

    /* Check that N is odd */
    if ((n & 1) == 0) {
	printf("int_forstner_best(): N must be odd: %d\n", n);
	return FAILURE;
	}

    /* Use previously allocated buffers if available and appropriate */
    if (((nx + n) <= (nx_max + n_max)) &&
	(m00_sums != NULL) && (m10_sums != NULL) && (m11_sums != NULL)) {
	alloc = FALSE;
	m00sums = m00_sums;
	m10sums = m10_sums;
	m11sums = m11_sums;
	}

    /* Otherwise allocate summation buffers */
    else {
	alloc = TRUE;
	if ((m00sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_best(): memory allocation error\n");
	    return FAILURE;
	    }
	if ((m10sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_best(): memory allocation error\n");
	    free((char *)m00sums);
	    return FAILURE;
	    }
	if ((m11sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_best(): memory allocation error\n");
	    free((char *)m00sums);
	    free((char *)m10sums);
	    return FAILURE;
	    }
	}

    /* Adjust the processed rectangle so that we stay within the image */
    w = n / 2;
    ix_start = w - x0;
    iy_start = w - y0;
    ix_stop  = xdim - w - x0 - 1;
    iy_stop  = ydim - w - y0 - 1;

    /* Compute how far we advance between rows */
    row_offset = xdim - nx;

    /* Move to the beginning of the first row to process */
    start_offset = (xdim * y0) + x0;
    image += start_offset;

    /* Process each row */
    bval = -1;
    first_row = TRUE;
    for (iy=0; iy<ny; iy++) {

	/* Process each column in the row */
	first_col = TRUE;
	for (ix=0; ix<nx; ix++,image++) {

	    /* Make sure that we are within the correct processing region */
	    if ((ix < ix_start) || (ix > ix_stop) ||
		(iy < iy_start) || (iy > iy_stop))
		continue;

	    /* Compute the sums for the first row */
	    if (first_row) {

		/* For first column, do all column sums in NxN neighborhood */
		if (first_col) {
		    first_col = FALSE;

		    /* Zero out summations */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    for (j=1-w; j<w; j++,m00p++,m10p++,m11p++)
			*m00p = *m10p = *m11p = 0;
		    m00 = m10 = m11 = 0;

		    /* Sum entire NxN neighborhood, recording column sums */
		    for (i=1-w; i<w; i++) {
			m00p = m00sums;
			m10p = m10sums;
			m11p = m11sums;
			img = image + (i * xdim) + 1 - w;
			for (j=1-w; j<w; j++,img++,m00p++,m10p++,m11p++) {
			    d0 = img[1]    - img[-1];
			    d1 = img[xdim] - img[-xdim];
			    m00   += dd = (d0 * d0);	/* / 4.0 */
			    *m00p += dd;
			    m11   += dd = (d1 * d1);	/* / 4.0 */
			    *m11p += dd;
			    m10   += dd = (d1 * d0);	/* / 4.0 */
			    *m10p += dd;
			    }
			}
		    }

		/* For following columns, merely compute new column sum */
		else {

		    /* Compute sum for newest column */
		    *m00p = *m10p = *m11p = 0;
		    img = image + ((1-w) * xdim) + w - 1;
		    for (i=1-w; i<w; i++,img+=xdim) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p += (d0 * d0);	/* / 4.0 */
			*m11p += (d1 * d1);	/* / 4.0 */
			*m10p += (d1 * d0);	/* / 4.0 */
			}

		    /* Update sums by removing oldest and adding newest col */
		    i = 1 - (w << 1);
		    m00 += (m00p[0] - m00p[i]);
		    m10 += (m10p[0] - m10p[i]);
		    m11 += (m11p[0] - m11p[i]);
		    m00p++;
		    m10p++;
		    m11p++;
		    }
		}

	    /* For following rows, merely adjust the previous summations */
	    else {

		/* For first column, adjust all of the column sums */
		if (first_col) {
		    first_col = FALSE;

		    /* Remove old top terms from each column sum */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    img = image + (-w * xdim) + 1 - w;
		    for (i=1-w; i<w; i++,img++,m00p++,m10p++,m11p++) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p -= (d0 * d0);	/* / 4.0 */
			*m11p -= (d1 * d1);	/* / 4.0 */
			*m10p -= (d1 * d0);	/* / 4.0 */
			}

		    /* Add new bottom terms to each column sum */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    img = image + ((w-1) * xdim) + 1 - w;
		    for (i=1-w; i<w; i++,img++,m00p++,m10p++,m11p++) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p += (d0 * d0);	/* / 4.0 */
			*m11p += (d1 * d1);	/* / 4.0 */
			*m10p += (d1 * d0);	/* / 4.0 */
			}

		    /* Add up all column sums to get new summations */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    m00 = m10 = m11 = 0;
		    for (i=1-w; i<w; i++,m00p++,m10p++,m11p++) {
			m00 += *m00p;
			m10 += *m10p;
			m11 += *m11p;
			}
		    }

		/* For following columns, merely adjust new column sums */
		else {

		    /* Remove old top term from column sum */
		    img = image + (-w * xdim) + w - 1;
		    d0 = img[1]    - img[-1];
		    d1 = img[xdim] - img[-xdim];
		    *m00p -= (d0 * d0);	/* / 4.0 */
		    *m11p -= (d1 * d1);	/* / 4.0 */
		    *m10p -= (d1 * d0);	/* / 4.0 */

		    /* Add new bottom term to column sum */
		    img = image + ((w-1) * xdim) + w - 1;
		    d0 = img[1]    - img[-1];
		    d1 = img[xdim] - img[-xdim];
		    *m00p += (d0 * d0);	/* / 4.0 */
		    *m11p += (d1 * d1);	/* / 4.0 */
		    *m10p += (d1 * d0);	/* / 4.0 */

		    /* Update sums by removing oldest and adding newest col */
		    i = 1 - (w << 1);
		    m00 += (m00p[0] - m00p[i]);
		    m10 += (m10p[0] - m10p[i]);
		    m11 += (m11p[0] - m11p[i]);
		    m00p++;
		    m10p++;
		    m11p++;
		    }
		}

	    /* Invert the matrix to get covariance */
	    m01 = m10;
	    det = (m00 * m11) - (m01 * m10);
	    det /= 4.0;		/* to compensate for not dividing above */
	    if (det < 0.00001)
	        continue;
	    a =  m11 / det;
	    d =  m00 / det;
	    b = -m01 / det;
	    val = ((a + d) + sqrt((a-d)*(a-d) + 4*b*b)) / 2.0;

	    /* See if this value is better than all previous ones */
	    if ((bval < 0) || (bval > val)) {
		bval = val;
		bx = ix;
		by = iy;
		}
	    }

	/* Move to the start of the next row */
	image += row_offset;
	if (iy >= iy_start)
	    first_row = FALSE;
	}

    /* Free summation buffers */
    if (alloc) {
	free((char *)m00sums);
	free((char *)m10sums);
	free((char *)m11sums);
	}

    /* Return the best point */
    *x = bx + x0;
    *y = by + y0;
    *intr = bval;

    return SUCCESS;
    }


/******************************************************************************
********************************   INT_FORSTNER_COV   *************************
*******************************************************************************

    This function produces the 2x2 covariance matrix at each point, as used
    by the interest operator developed by Forstner (?) to find the most
    interesting point in an image. Note that the output (0,1) elements of
    the covariance matrix are identical with the (1,0) elements. In any
    location where there was a problem computing the covariance, all three
    outputs will be set to identical negative values (depending on the
    problem). */

int int_forstner_cov(
    const unsigned char *image,	/* input intensity image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int n,			/* input size of NxN neighborhood around pt */
    float *c00,			/* output image of (0,0) elements of covar */
    float *c01,			/* output image of (0,1) elements of covar */
    float *c11)			/* output image of (1,1) elements of covar */
{
    int ix, iy, ix_start, iy_start, ix_stop, iy_stop, row_offset, start_offset;
    int i, j, w, d0, d1, dd, first_row, first_col;
    const unsigned char *img;
    double a, b, d, det, m00 = 0, m01 = 0, m10 = 0, m11 = 0;
    int *m00sums, *m10sums, *m11sums, *m00p = 0, *m10p = 0, *m11p = 0, alloc;

    /* Check that N is odd */
    if ((n & 1) == 0) {
	printf("int_forstner_cov(): N must be odd: %d\n", n);
	return FAILURE;
	}

    /* Use previously allocated buffers if available and appropriate */
    if (((nx + n) <= (nx_max + n_max)) &&
	(m00_sums != NULL) && (m10_sums != NULL) && (m11_sums != NULL)) {
	alloc = FALSE;
	m00sums = m00_sums;
	m10sums = m10_sums;
	m11sums = m11_sums;
	}

    /* Otherwise allocate summation buffers */
    else {
	alloc = TRUE;
	if ((m00sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_cov(): memory allocation error\n");
	    return FAILURE;
	    }
	if ((m10sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_cov(): memory allocation error\n");
	    free((char *)m00sums);
	    return FAILURE;
	    }
	if ((m11sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_cov(): memory allocation error\n");
	    free((char *)m00sums);
	    free((char *)m10sums);
	    return FAILURE;
	    }
	}

    /* Adjust the processed rectangle so that we stay within the image */
    w = n / 2;
    ix_start = w - x0;
    iy_start = w - y0;
    ix_stop  = xdim - w - x0 - 1;
    iy_stop  = ydim - w - y0 - 1;

    /* Compute how far we advance between rows */
    row_offset = xdim - nx;

    /* Move to the beginning of the first row to process */
    start_offset = (xdim * y0) + x0;
    image += start_offset;
    c00   += start_offset;
    c01   += start_offset;
    c11   += start_offset;

    /* Process each row */
    first_row = TRUE;
    for (iy=0; iy<ny; iy++) {

	/* Process each column in the row */
	first_col = TRUE;
	for (ix=0; ix<nx; ix++,image++,c00++,c01++,c11++) {

	    /* Make sure that we are within the correct processing region */
	    if ((ix < ix_start) || (ix > ix_stop) ||
		(iy < iy_start) || (iy > iy_stop)) {
		*c00 = *c01 = *c11 = -1.0;
		continue;
		}

	    /* Compute the sums for the first row */
	    if (first_row) {

		/* For first column, do all column sums in NxN neighborhood */
		if (first_col) {
		    first_col = FALSE;

		    /* Zero out summations */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    for (j=1-w; j<w; j++,m00p++,m10p++,m11p++)
			*m00p = *m10p = *m11p = 0;
		    m00 = m10 = m11 = 0;

		    /* Sum entire NxN neighborhood, recording column sums */
		    for (i=1-w; i<w; i++) {
			m00p = m00sums;
			m10p = m10sums;
			m11p = m11sums;
			img = image + (i * xdim) + 1 - w;
			for (j=1-w; j<w; j++,img++,m00p++,m10p++,m11p++) {
			    d0 = img[1]    - img[-1];
			    d1 = img[xdim] - img[-xdim];
			    m00   += dd = (d0 * d0);	/* / 4.0 */
			    *m00p += dd;
			    m11   += dd = (d1 * d1);	/* / 4.0 */
			    *m11p += dd;
			    m10   += dd = (d1 * d0);	/* / 4.0 */
			    *m10p += dd;
			    }
			}
		    }

		/* For following columns, merely compute new column sum */
		else {

		    /* Compute sum for newest column */
		    *m00p = *m10p = *m11p = 0;
		    img = image + ((1-w) * xdim) + w - 1;
		    for (i=1-w; i<w; i++,img+=xdim) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p += (d0 * d0);	/* / 4.0 */
			*m11p += (d1 * d1);	/* / 4.0 */
			*m10p += (d1 * d0);	/* / 4.0 */
			}

		    /* Update sums by removing oldest and adding newest col */
		    i = 1 - (w << 1);
		    m00 += (m00p[0] - m00p[i]);
		    m10 += (m10p[0] - m10p[i]);
		    m11 += (m11p[0] - m11p[i]);
		    m00p++;
		    m10p++;
		    m11p++;
		    }
		}

	    /* For following rows, merely adjust the previous summations */
	    else {

		/* For first column, adjust all of the column sums */
		if (first_col) {
		    first_col = FALSE;

		    /* Remove old top terms from each column sum */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    img = image + (-w * xdim) + 1 - w;
		    for (i=1-w; i<w; i++,img++,m00p++,m10p++,m11p++) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p -= (d0 * d0);	/* / 4.0 */
			*m11p -= (d1 * d1);	/* / 4.0 */
			*m10p -= (d1 * d0);	/* / 4.0 */
			}

		    /* Add new bottom terms to each column sum */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    img = image + ((w-1) * xdim) + 1 - w;
		    for (i=1-w; i<w; i++,img++,m00p++,m10p++,m11p++) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p += (d0 * d0);	/* / 4.0 */
			*m11p += (d1 * d1);	/* / 4.0 */
			*m10p += (d1 * d0);	/* / 4.0 */
			}

		    /* Add up all column sums to get new summations */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    m00 = m10 = m11 = 0;
		    for (i=1-w; i<w; i++,m00p++,m10p++,m11p++) {
			m00 += *m00p;
			m10 += *m10p;
			m11 += *m11p;
			}
		    }

		/* For following columns, merely adjust new column sums */
		else {

		    /* Remove old top term from column sum */
		    img = image + (-w * xdim) + w - 1;
		    d0 = img[1]    - img[-1];
		    d1 = img[xdim] - img[-xdim];
		    *m00p -= (d0 * d0);	/* / 4.0 */
		    *m11p -= (d1 * d1);	/* / 4.0 */
		    *m10p -= (d1 * d0);	/* / 4.0 */

		    /* Add new bottom term to column sum */
		    img = image + ((w-1) * xdim) + w - 1;
		    d0 = img[1]    - img[-1];
		    d1 = img[xdim] - img[-xdim];
		    *m00p += (d0 * d0);	/* / 4.0 */
		    *m11p += (d1 * d1);	/* / 4.0 */
		    *m10p += (d1 * d0);	/* / 4.0 */

		    /* Update sums by removing oldest and adding newest col */
		    i = 1 - (w << 1);
		    m00 += (m00p[0] - m00p[i]);
		    m10 += (m10p[0] - m10p[i]);
		    m11 += (m11p[0] - m11p[i]);
		    m00p++;
		    m10p++;
		    m11p++;
		    }
		}

	    /* Invert the matrix to get covariance */
	    m01 = m10;
	    det = (m00 * m11) - (m01 * m10);
	    det /= 4.0;		/* to compensate for not dividing above */
	    if (det < 0.00001) {
		*c00 = *c01 = *c11 = -2.0;
	        continue;
		}
	    a =  m11 / det;
	    d =  m00 / det;
	    b = -m01 / det;
	    *c00 = a;
	    *c01 = b;
	    *c11 = d;
	    }

	/* Move to the start of the next row */
	image += row_offset;
	c00   += row_offset;
	c01   += row_offset;
	c11   += row_offset;
	if (iy >= iy_start)
	    first_row = FALSE;
	}

    /* Free summation buffers */
    if (alloc) {
	free((char *)m00sums);
	free((char *)m10sums);
	free((char *)m11sums);
	}

    return SUCCESS;
    }


/******************************************************************************
********************************   INT_FORSTNER_FREE   ************************
*******************************************************************************

    This function frees the buffers allocated by int_forstner_alloc(). */

void int_forstner_free(void)
{
    if (m00_sums != NULL)
	free((char *)m00_sums);
    if (m10_sums != NULL)
	free((char *)m10_sums);
    if (m11_sums != NULL)
	free((char *)m11_sums);
    m00_sums = NULL;
    m10_sums = NULL;
    m11_sums = NULL;
    }


/******************************************************************************
********************************   INT_FORSTNER_NBEST   ***********************
*******************************************************************************

    This function uses the interest operator developed by Forstner (?)  to
    find the N most interesting points in an image. The output points are
    in no special order. */

int int_forstner_nbest(
    const unsigned char *image,	/* input intensity image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int n,			/* input size of NxN neighborhood around pt */
    int max,			/* input maximum length of output lists */
    int *num,			/* output number of points in output lists */
    int pos2[][2],		/* output coordinates of best point */
    float intr[])		/* output values of interest operation */
{
    int ix, iy, ix_start, iy_start, ix_stop, iy_stop, row_offset, start_offset;
    int i, j, w, d0, d1, dd, first_row, first_col, len, worst = 0;
    const unsigned char *img;
    double a, b, d, det, m00 = 0, m01 = 0, m10 = 0, m11 = 0, val, wval = 0;
    int *m00sums, *m10sums, *m11sums, *m00p = 0, *m10p = 0, *m11p = 0, alloc;

    /* Check that N is odd */
    if ((n & 1) == 0) {
	printf("int_forstner_best(): N must be odd: %d\n", n);
	return FAILURE;
	}

    /* Use previously allocated buffers if available and appropriate */
    if (((nx + n) <= (nx_max + n_max)) &&
	(m00_sums != NULL) && (m10_sums != NULL) && (m11_sums != NULL)) {
	alloc = FALSE;
	m00sums = m00_sums;
	m10sums = m10_sums;
	m11sums = m11_sums;
	}

    /* Otherwise allocate summation buffers */
    else {
	alloc = TRUE;
	if ((m00sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_best(): memory allocation error\n");
	    return FAILURE;
	    }
	if ((m10sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_best(): memory allocation error\n");
	    free((char *)m00sums);
	    return FAILURE;
	    }
	if ((m11sums = (int *)malloc((sizeof(int)) * (nx + n))) == NULL) {
	    printf("int_forstner_best(): memory allocation error\n");
	    free((char *)m00sums);
	    free((char *)m10sums);
	    return FAILURE;
	    }
	}

    /* Adjust the processed rectangle so that we stay within the image */
    w = n / 2;
    ix_start = w - x0;
    iy_start = w - y0;
    ix_stop  = xdim - w - x0 - 1;
    iy_stop  = ydim - w - y0 - 1;

    /* Compute how far we advance between rows */
    row_offset = xdim - nx;

    /* Move to the beginning of the first row to process */
    start_offset = (xdim * y0) + x0;
    image += start_offset;

    /* Process each row */
    *num = len = 0;
    first_row = TRUE;
    for (iy=0; iy<ny; iy++) {

	/* Process each column in the row */
	first_col = TRUE;
	for (ix=0; ix<nx; ix++,image++) {

	    /* Make sure that we are within the correct processing region */
	    if ((ix < ix_start) || (ix > ix_stop) ||
		(iy < iy_start) || (iy > iy_stop))
		continue;

	    /* Compute the sums for the first row */
	    if (first_row) {

		/* For first column, do all column sums in NxN neighborhood */
		if (first_col) {
		    first_col = FALSE;

		    /* Zero out summations */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    for (j=1-w; j<w; j++,m00p++,m10p++,m11p++)
			*m00p = *m10p = *m11p = 0;
		    m00 = m10 = m11 = 0;

		    /* Sum entire NxN neighborhood, recording column sums */
		    for (i=1-w; i<w; i++) {
			m00p = m00sums;
			m10p = m10sums;
			m11p = m11sums;
			img = image + (i * xdim) + 1 - w;
			for (j=1-w; j<w; j++,img++,m00p++,m10p++,m11p++) {
			    d0 = img[1]    - img[-1];
			    d1 = img[xdim] - img[-xdim];
			    m00   += dd = (d0 * d0);	/* / 4.0 */
			    *m00p += dd;
			    m11   += dd = (d1 * d1);	/* / 4.0 */
			    *m11p += dd;
			    m10   += dd = (d1 * d0);	/* / 4.0 */
			    *m10p += dd;
			    }
			}
		    }

		/* For following columns, merely compute new column sum */
		else {

		    /* Compute sum for newest column */
		    *m00p = *m10p = *m11p = 0;
		    img = image + ((1-w) * xdim) + w - 1;
		    for (i=1-w; i<w; i++,img+=xdim) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p += (d0 * d0);	/* / 4.0 */
			*m11p += (d1 * d1);	/* / 4.0 */
			*m10p += (d1 * d0);	/* / 4.0 */
			}

		    /* Update sums by removing oldest and adding newest col */
		    i = 1 - (w << 1);
		    m00 += (m00p[0] - m00p[i]);
		    m10 += (m10p[0] - m10p[i]);
		    m11 += (m11p[0] - m11p[i]);
		    m00p++;
		    m10p++;
		    m11p++;
		    }
		}

	    /* For following rows, merely adjust the previous summations */
	    else {

		/* For first column, adjust all of the column sums */
		if (first_col) {
		    first_col = FALSE;

		    /* Remove old top terms from each column sum */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    img = image + (-w * xdim) + 1 - w;
		    for (i=1-w; i<w; i++,img++,m00p++,m10p++,m11p++) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p -= (d0 * d0);	/* / 4.0 */
			*m11p -= (d1 * d1);	/* / 4.0 */
			*m10p -= (d1 * d0);	/* / 4.0 */
			}

		    /* Add new bottom terms to each column sum */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    img = image + ((w-1) * xdim) + 1 - w;
		    for (i=1-w; i<w; i++,img++,m00p++,m10p++,m11p++) {
			d0 = img[1]    - img[-1];
			d1 = img[xdim] - img[-xdim];
			*m00p += (d0 * d0);	/* / 4.0 */
			*m11p += (d1 * d1);	/* / 4.0 */
			*m10p += (d1 * d0);	/* / 4.0 */
			}

		    /* Add up all column sums to get new summations */
		    m00p = m00sums;
		    m10p = m10sums;
		    m11p = m11sums;
		    m00 = m10 = m11 = 0;
		    for (i=1-w; i<w; i++,m00p++,m10p++,m11p++) {
			m00 += *m00p;
			m10 += *m10p;
			m11 += *m11p;
			}
		    }

		/* For following columns, merely adjust new column sums */
		else {

		    /* Remove old top term from column sum */
		    img = image + (-w * xdim) + w - 1;
		    d0 = img[1]    - img[-1];
		    d1 = img[xdim] - img[-xdim];
		    *m00p -= (d0 * d0);	/* / 4.0 */
		    *m11p -= (d1 * d1);	/* / 4.0 */
		    *m10p -= (d1 * d0);	/* / 4.0 */

		    /* Add new bottom term to column sum */
		    img = image + ((w-1) * xdim) + w - 1;
		    d0 = img[1]    - img[-1];
		    d1 = img[xdim] - img[-xdim];
		    *m00p += (d0 * d0);	/* / 4.0 */
		    *m11p += (d1 * d1);	/* / 4.0 */
		    *m10p += (d1 * d0);	/* / 4.0 */

		    /* Update sums by removing oldest and adding newest col */
		    i = 1 - (w << 1);
		    m00 += (m00p[0] - m00p[i]);
		    m10 += (m10p[0] - m10p[i]);
		    m11 += (m11p[0] - m11p[i]);
		    m00p++;
		    m10p++;
		    m11p++;
		    }
		}

	    /* Invert the matrix to get covariance */
	    m01 = m10;
	    det = (m00 * m11) - (m01 * m10);
	    det /= 4.0;		/* to compensate for not dividing above */
	    if (det < 0.00001)
	        continue;
	    a =  m11 / det;
	    d =  m00 / det;
	    b = -m01 / det;
	    val = ((a + d) + sqrt((a-d)*(a-d) + 4*b*b)) / 2.0;

	    /* If the list is not yet full, then add new point */
	    if (len < max) {
		pos2[len][0] = ix + x0;
		pos2[len][1] = iy + y0;
		intr[len]    = val;
		if ((len == 0) || (val > wval)) {
		    worst = len;
		    wval  = val;
		    }
		len++;
		continue;
		}

	    /* Skip any point which is worse than the worst in the list */
	    if (val > wval)
		continue;

	    /* Add new point to list, and update pointer to worst entry */
	    pos2[worst][0] = ix + x0;
	    pos2[worst][1] = iy + y0;
	    intr[worst]    = val;
	    worst = 0;
	    wval  = intr[0];
	    for (i=1; i<max; i++) {
		if (intr[i] > wval) {
		    worst = i;
		    wval  = intr[i];
		    }
		}
	    }

	/* Move to the start of the next row */
	image += row_offset;
	if (iy >= iy_start)
	    first_row = FALSE;
	}

    /* Free summation buffers */
    if (alloc) {
	free((char *)m00sums);
	free((char *)m10sums);
	free((char *)m11sums);
	}

    /* Finish up */
    *num = len;

    return SUCCESS;
    }
