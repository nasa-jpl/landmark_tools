/******************************************************************************
*                                                                             *
*                                   IMGUTILS                                  *
*                                Image Utilities                              *
*                                                                             *
*                                       Todd Litwin                           *
*                                       Written: 19 Aug 2002                  *
*                                       Updated: 22 Sep 2006                  *
*                                                                             *
*                                                                             *
*******************************************************************************

	This header file contains declarations for the image utilities. */

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

#ifndef _IMGUTILS_H
#define _IMGUTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FAILURE
#define FAILURE (-1)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

extern int match_pyr_point_deepest;

#ifdef __STDC__

int alignstat_begin(
    int n,			/* input number of image locations */
    int count[],		/* output list of iteration counters */
    double xsum[],		/* output list of sums of X coordinates */
    double ysum[],		/* output list of sums of Y coordinates */
    double x2sum[],		/* output list of sums of X sqaured */
    double xysum[],		/* output list of sums of X * Y */
    double y2sum[]);		/* output list of sums of Y sqaured */

int alignstat_end(
    int n,			/* input number of image locations */
    const int count[],		/* input list of iteration counters */
    const double xsum[],	/* input list of sums of X coordinates */
    const double ysum[],	/* input list of sums of Y coordinates */
    const double x2sum[],	/* input list of sums of X sqaured */
    const double xysum[],	/* input list of sums of X * Y */
    const double y2sum[],	/* input list of sums of Y sqaured */
    double xmean[],		/* output list of means of X coordinates */
    double ymean[],		/* output list of means of X coordinates */
    double cov[][2][2]);	/* output list of covariances of X and Y */

int alignstat_next(
    const unsigned char *image1,/* input image of targets */
    const unsigned char *image2,/* input image for alignment searches */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int tsize,			/* input size of target neighborhood */
    int x0,			/* input left-most X target-relative coord */
    int y0,			/* input left-most Y target-relative coord */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    double thresh,		/* input threshold for the correlation value */
    int n,			/* input number of image locations */
    const int xlist[],		/* input list of X coordinates in image */
    const int ylist[],		/* input list of Y coordinates in image */
    int count[],		/* input/output list of iteration counters */
    double xsum[],		/* input/output list of sums of X coordinates */
    double ysum[],		/* input/output list of sums of Y coordinates */
    double x2sum[],		/* input/output list of sums of X sqaured */
    double xysum[],		/* input/output list of sums of X * Y */
    double y2sum[]);		/* input/output list of sums of Y sqaured */

int alignstat_select(
    unsigned char *target,	/* input target image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int mx,			/* input number of sub-blocks in X dimension */
    int my,			/* input number of sub-blocks in Y dimension */
    int isize,			/* input size of interest neighborhood */
    int xlist[],		/* output list of X image coords, mx*my long */
    int ylist[]);		/* output list of Y image coords, mx*my long */

int corr_image_alloc(
    int t_maxx,			/* input maximum number of columns in target */
    int t_maxy,			/* input maximum number of rows    in target */
    int s_maxx,			/* input maximum number of columns in search */
    int s_maxy);		/* input maximum number of rows    in search */

void corr_image_free(void);

int corr_image(
    const unsigned char *target,/* input target image */
    int t_xdim,			/* input size of X dimension */
    int t_ydim,			/* input size of Y dimension */
    int t_x0,			/* input left-most X coordinate */
    int t_y0,			/* input upper-most Y coordinate */
    int t_nx,			/* input number of columns */
    int t_ny,			/* input number of rows */
    const unsigned char *search,/* input search image */
    int s_xdim,			/* input size of X dimension */
    int s_ydim,			/* input size of Y dimension */
    int s_x0,			/* input left-most X coordinate */
    int s_y0,			/* input upper-most Y coordinate */
    int s_nx,			/* input number of columns */
    int s_ny,			/* input number of rows */
    double *x,			/* output X coordinate of best match */
    double *y,			/* output Y coordinate of best match */
    double *val);		/* output correlation value of best match */

int corr_image2(
    const unsigned char *target,/* input target image */
    int t_xdim,			/* input size of X dimension */
    int t_ydim,			/* input size of Y dimension */
    int t_x0,			/* input left-most X coordinate */
    int t_y0,			/* input upper-most Y coordinate */
    int t_nx,			/* input number of columns */
    int t_ny,			/* input number of rows */
    const unsigned char *search,/* input search image */
    int s_xdim,			/* input size of X dimension */
    int s_ydim,			/* input size of Y dimension */
    int s_x0,			/* input left-most X coordinate */
    int s_y0,			/* input upper-most Y coordinate */
    int s_nx,			/* input number of columns */
    int s_ny,			/* input number of rows */
    double *x,			/* output X coordinate of best match */
    double *y,			/* output Y coordinate of best match */
    double *val,		/* output correlation value of best match */
    double cov[3]);		/* output best-match covariances: 00, 01, 11 */

int corr_image_diag(
    int setting);		/* input setting: TRUE or FALSE */

int int_forstner(
    const unsigned char *image,	/* input intensity image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int n,			/* input size of NxN interest neighborhood */
    float *interest);		/* output interest image */

int int_forstner_alloc(
    int nxmax,			/* input max number of columns */
    int nmax);			/* input max size of NxN int. neighborhood */

int int_forstner_best(
    const unsigned char *image,	/* input intensity image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int n,			/* input size of NxN neighborhood around pix */
    int *x,			/* output X coordinate of best point */
    int *y,			/* output Y coordinate of best point */
    double *intr);		/* output value of interest operation */

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
    float *c11);		/* output image of (1,1) elements of covar */

void int_forstner_free(void);

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
    float intr[]);		/* output values of interest operation */

int int_moravec(
    const unsigned char *image,	/* input intensity image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int n,			/* input size of NxN neighborhood around pt */
    int *interest);		/* output interest image */

int int_moravec_best(
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
    int *intr);			/* output value of interest operation */

int int_extrema_matrix(
    const float *interest,	/* input interest image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int mx,			/* input number of matrix cells across cols */
    int my,			/* input number of matrix cells down rows */
    int extrem,			/* input extremity: 0=minima, 1=maxima */
    int max,			/* input maximum length of output lists */
    int *num,			/* output number of points in output lists */
    int pos2[][2],		/* output coordinates of best point */
    float intr[]);		/* output values of interest operation */

int int_maxima_local(
    const float *interest,	/* input interest image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int max,			/* input maximum length of output lists */
    int *num,			/* output number of points in output lists */
    int pos2[][2],		/* output coordinates of best point */
    float intr[]);		/* output values of interest operation */

int int_minima_local(
    const float *interest,	/* input interest image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int max,			/* input maximum length of output lists */
    int *num,			/* output number of points in output lists */
    int pos2[][2],		/* output coordinates of best point */
    float intr[]);		/* output values of interest operation */

int los_next(
    int *xout,			/* output X coordinate of first point found */
    int *yout);			/* output Y coordinate of first point found */

int los_start(
    const unsigned char *image,	/* input image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int x,			/* input central X coordinate */
    int y,			/* input central Y coordinate */
    int lower,			/* input lower intensity value */
    int upper,			/* input upper intensity value */
    int *xout,			/* output X coordinate of first point found */
    int *yout);			/* output Y coordinate of first point found */

void *match_pyr_alloc(
    int t_maxx,			/* input maximum number of columns in target */
    int t_maxy,			/* input maximum number of rows    in search */
    int s_maxx,			/* input maximum number of columns in target */
    int s_maxy);		/* input maximum number of rows    in search */

int match_pyr_init(
    const void *pyr,		/* input pyramid from match_pyr_alloc() */
    int n,			/* input depth of pyramid for search */
    const unsigned char *target,/* input target image */
    int t_xdim,			/* input size of X dimension */
    int t_ydim,			/* input size of Y dimension */
    int t_x0,			/* input left-most X coordinate */
    int t_y0,			/* input upper-most Y coordinate */
    int t_nx,			/* input number of columns */
    int t_ny,			/* input number of rows */
    const unsigned char *search,/* input search image */
    int s_xdim,			/* input size of X dimension */
    int s_ydim,			/* input size of Y dimension */
    int s_x0,			/* input left-most X coordinate */
    int s_y0,			/* input upper-most Y coordinate */
    int s_nx,			/* input number of columns */
    int s_ny);			/* input number of rows */

int match_pyr_point(
    const void *pyr,		/* input pyramid from match_pyr_setup() */
    int t_x,			/* input target X coordinate */
    int t_y,			/* input target Y coordinate */
    int t_rx,			/* input radius in columns */
    int t_ry,			/* input radius in rows */
    int s_x0,			/* input left-most  X search coordinate */
    int s_y0,			/* input upper-most Y search coordinate */
    int s_nx,			/* input number of search columns */
    int s_ny,			/* input number of search rows */
    double *x,			/* output X coordinate of best match */
    double *y,			/* output Y coordinate of best match */
    double *val,		/* output correlation value of best match */
    double cov[3]);		/* output best-match covariances: 00, 01, 11 */

void *match_pyr_setup(
    int n,			/* input depth of pyramid for search */
    const unsigned char *target,/* input target image */
    int t_xdim,			/* input size of X dimension */
    int t_ydim,			/* input size of Y dimension */
    int t_x0,			/* input left-most X coordinate */
    int t_y0,			/* input upper-most Y coordinate */
    int t_nx,			/* input number of columns */
    int t_ny,			/* input number of rows */
    const unsigned char *search,/* input search image */
    int s_xdim,			/* input size of X dimension */
    int s_ydim,			/* input size of Y dimension */
    int s_x0,			/* input left-most X coordinate */
    int s_y0,			/* input upper-most Y coordinate */
    int s_nx,			/* input number of columns */
    int s_ny);			/* input number of rows */

int region_find(
    const unsigned char *image,	/* input image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int lower,			/* input lower region value */
    int upper,			/* input upper region value */
    short *region,		/* output image of region IDs */
    int *nreg);			/* output number of regions */

int region_find8(
    const unsigned char *image,	/* input image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int lower,			/* input lower region value */
    int upper,			/* input upper region value */
    short *region,		/* output image of region IDs */
    int *nreg);			/* output number of regions */

int region_maxsize(
    short *region,		/* input region block */
    int nreg,			/* input number of regions */
    int xdim,			/* input number if image columns */
    int ydim,			/* input number of image rows */
    int x0,			/* input starting column of bounding box */
    int y0,			/* input starting row    of bounding box */
    int nx,			/* input width  of bounding box */
    int ny,			/* input height of bounding box */
    int *regid,			/* output id of largest region */
    int *regsize);		/* output size of largest region */

int region_start(
    int max_regions);		/* input maximum number of regions handled */

int region_stat(
    const short *region,	/* input region block */
    int regid,			/* input ID of selected region */
    int xdim,			/* input number if image columns */
    int ydim,			/* input number of image rows */
    int *x0,			/* input/output starting col of bounding box */
    int *y0,			/* input/output starting row of bounding box */
    int *nx,			/* input/output width  of bounding box */
    int *ny,			/* input/output height of bounding box */
    double *x,			/* output X coordinate of centroid */
    double *y,			/* output Y coordinate of centroid */
    int *regsize);		/* output size of region */

int region_stats(
    const short *region,	/* input region block */
    int nreg,			/* input number of regions */
    int xdim,			/* input number if image columns */
    int ydim,			/* input number of image rows */
    int x0[],			/* input/output starting col of bounding box */
    int y0[],			/* input/output starting row of bounding box */
    int nx[],			/* input/output width  of bounding box */
    int ny[],			/* input/output height of bounding box */
    double x[],			/* output X coordinate of centroid */
    double y[],			/* output Y coordinate of centroid */
    int regsize[]);		/* output size of region */

int region_stop(void);

int sens_align(
    const unsigned char *image,	/* input intensity image */
    int xdim,			/* input size of X dimension */
    int ydim,			/* input size of Y dimension */
    int x0,			/* input left-most X coordinate */
    int y0,			/* input upper-most Y coordinate */
    int nx,			/* input number of columns */
    int ny,			/* input number of rows */
    int n,			/* input size of NxN neighborhood around pix */
    float *interest);		/* output interest image */

#else

int alignstat_begin();
int alignstat_end();
int alignstat_next();
int alignstat_select();

int corr_image_alloc();
void corr_image_free();
int corr_image();
int corr_image2();
int corr_image_diag();

int int_forstner();
int int_forstner_best();
int int_forstner_cov();
int int_forstner_nbest();

int int_moravec();
int int_moravec_best();

int int_extrema_matrix();
int int_maxima_local();
int int_minima_local();

int los_next();
int los_start();

void *match_pyr_alloc();
int match_pyr_init();
int match_pyr_point();
void *match_pyr_setup();

int region_find();
int region_find8();
int region_maxsize();
int region_start();
int region_stat();
int region_stats();
int region_stop();

int sens_align();

#endif

#ifdef	__cplusplus
}
#endif

#endif
