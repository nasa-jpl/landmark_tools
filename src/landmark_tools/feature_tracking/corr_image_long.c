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
#include <stdio.h>
#include <stdlib.h>

#include "landmark_tools/feature_tracking/corr_image_long.h"

/* Precomputed normal equations for the least squares fit of the correlation
 * scores.
 */
static double fitmat[6][9] = {
    {1.0/6, -2.0/6,  1.0/6,  1.0/6, -2.0/6,  1.0/6,  1.0/6, -2.0/6,  1.0/6 },
    {1.0/6,  1.0/6,  1.0/6, -2.0/6, -2.0/6, -2.0/6,  1.0/6,  1.0/6,  1.0/6},
    { 1.0/4,  0.0,   -1.0/4,  0.0,    0.0,    0.0,   -1.0/4,  0.0,    1.0/4},
    {-1.0/6,  0.0,    1.0/6, -1.0/6,  0.0,    1.0/6, -1.0/6,  0.0,    1.0/6},
    {-1.0/6, -1.0/6, -1.0/6,  0.0,    0.0,    0.0,    1.0/6,  1.0/6,  1.0/6},
    {-1.0/9,  2.0/9, -1.0/9,  2.0/9,  5.0/9,  2.0/9, -1.0/9,  2.0/9, -1.0/9},
};

bool corimg_long_with_input_check(
                                     unsigned char *img1,
                                     int32_t rowBytes1,
                                     int32_t left1,
                                     int32_t top1,
                                     int32_t cols1,
                                     int32_t rows1,
                                     unsigned char *img2,
                                     int32_t rowBytes2,
                                     int32_t left2,
                                     int32_t top2,
                                     int32_t cols2,
                                     int32_t rows2,
                                     double *bestrow,
                                     double *bestcol,
                                     double *bestval,
                                     double *covar)
{
    if (cols2 < cols1 || rows2 < rows1) return false;
    
    if(left2 < 0){
        int32_t diff = -left2;
        left2 += diff;
        cols2 += diff;
        if(cols2 < 0) return false;
    }
    if(top2 < 0){
        int32_t diff = -top2;
        top2 += diff;
        rows2 += diff;
        if(rows2 < 0) return false;
    }
    return corimg_long(img1, rowBytes1, left1, top1, cols1, rows1,
                img2, rowBytes2, left2, top2, cols2, rows2,
                bestrow, bestcol, bestval, covar);
}

bool corimg_long (
        unsigned char *img1,
        size_t rowBytes1,
        size_t left1,
        size_t top1,
        size_t cols1,
        size_t rows1,
        unsigned char *img2,
        size_t rowBytes2,
        size_t left2,
        size_t top2,
        size_t cols2,
        size_t rows2,
        double *bestrow,
        double *bestcol,
        double *bestval,
        double *covar)
{
    size_t r, c;                /* row and col within window */
    double sumabsq;                        /* sum((a+b)**2) */
    long normsumab;              	      /* 2*sum((a-abar)(b-bbar)) */
    double suma, sumasq, normsumasq;  /* sum(a), sum(a**2), sum((a-abar)**2) */
    double sumb, sumbsq, normsumbsq;  /* sum(b), sum(b**2), sum((b-bbar)**2) */
    long row, col;                    /* row and col offsets for window */
    long n;                            /* number of pixels in a window */
    long bestr, bestc;
    unsigned char *src, *dst;
    register unsigned char *s, *d, pixel;
    register unsigned long *first, *last, *firstsq, *lastsq, longPixel;  //add unsigned
    double coeff, *coeffs;                    /* correlation for this window place */
    unsigned long *colsum;   /* running total for sum(b) */
    unsigned long *colsq;    /* running total for sum(b**2) */
    double *cbuff;
    
    if (cols2 < cols1 || rows2 < rows1) return false;

    n = cols1 * rows1;

    *bestval = -2.0;

    /* calculate only once per call */

    suma = sumasq = 0;
    src = img1 + top1 * rowBytes1 + left1;
    for (r = rows1; r--; src += rowBytes1)
    {
        for (c = cols1, s = src; c--; s++) {
            pixel = *s;
            suma += pixel;
            sumasq += pixel * pixel;
        }
    }
    normsumasq = sumasq - suma * suma / n;
    //printf("%f %f %f\n", sumasq, suma, normsumasq);
    if (normsumasq == 0) {
        //fprintf (stderr, "Uniform pixels in target window");
        return false;
    }

    colsum = (unsigned long *) malloc (cols2 * 2 * sizeof(long) +
        (cols2 - cols1 + 1) * (rows2 - rows1 + 1) * sizeof (double));
    colsq  = colsum + cols2;
    cbuff = (double *) (colsq + cols2);

    dst = img2 + (top2 * rowBytes2) + left2;
    for (r = rows1; r--; dst += rowBytes2) {
        if (r == (rows1 - 1)) {
            for (c = 0, d = dst; c < cols2; c++, d++) {
                pixel = *d;
                    colsum[c] = pixel;
                    colsq[c] = pixel * pixel;
            }
        } else {
            for (c = 0, d = dst; c < cols2; c++, d++) {
                pixel = *d;
                    colsum[c] += pixel;
                    colsq[c] += pixel * pixel;
            }
        }
    }

    /* now do the convolution */

    coeffs = cbuff;
    for (row = 0; row <= rows2 - rows1; row++) {

        /* initialize sum */
        sumb = sumbsq = 0;
        first = last = colsum;
        firstsq = lastsq = colsq;
        for (col = cols1; col--; last++, lastsq ++) {
                sumbsq += *lastsq;
                sumb += *last;
        }

        for (col = 0; col <= cols2 - cols1; col++, coeffs++) {
                
                // out of magic: have to do it the naive way!
                sumabsq = 0;
                src = img1 + top1 * rowBytes1 + left1;
                dst = img2 + ((top2 + row) * rowBytes2) + left2 + col;
            for (r = rows1; r--; src += rowBytes1, dst += rowBytes2) {
                for (c = cols1, s = src, d = dst; c--; s++, d++) {
                    longPixel = *s + *d;
                    sumabsq += longPixel * longPixel;
                }
            }
            //2 * sum(img1*img2) = sum((img1+img2)**2) - sum(img1**2) - sum(img2**2)
            normsumbsq = sumbsq - sumb * sumb / n;
            normsumab = (long)(sumabsq - sumasq - sumbsq - 2 * suma * sumb / n);
            /*normsumbsq = sumbsq  ;
            normsumab = (long)(sumabsq - sumasq - sumbsq  );*/

            *coeffs = coeff = ((double) normsumab) / ((double) (normsumasq + normsumbsq));
            /*if(coeff < 0.0)
            {
                printf("negative correlation \n");
            } */
            if (coeff > *bestval) {
                *bestval = coeff;
                bestr= row;
                bestc = col;
            }
            
            sumb -= *first++;
            sumb += *last++;
            sumbsq -= *firstsq++;
            sumbsq += *lastsq++;
        }

        /* update colsums */
        if (row == (rows2 - rows1)) continue;
        dst = img2 + ((top2 + row) * rowBytes2) + left2;
        for (c = 0; c < cols2; c++, dst++) {
            pixel = *dst;
            colsq[c] -= pixel * pixel;
            colsum[c] -= pixel;
        }
        dst = img2 + ((top2 + row + rows1) * rowBytes2) + left2;
        for (c = 0; c < cols2; c++, dst++) {
            pixel = *dst;
            colsq[c] += pixel * pixel;
            colsum[c] += pixel;
        }
    }

    /*...
    printf ("Pixel-resolution match coordinate: %d %d\n",
        bestr + top2 + height1/2, bestc + left2 + width1/2);
    ...*/
    if(covar != NULL)
    {
        if (subpixel_long (bestr, bestc, rows2 - rows1 + 1, cols2 - cols1 + 1, cbuff,
              bestrow, bestcol, bestval, covar) != true) {
                  //printf("subpixel error\n");
            free (colsum);
            return false;
        }
    }
    else
    {
        *bestval= cbuff[bestr*(cols2 - cols1 + 1)+bestc];
        *bestrow = bestr;
        *bestcol = bestc;
    }
/*
    *bestrow = bestr;
    *bestcol = bestc; */
    free (colsum);
    /* Offset to upper-left corner */
    *bestrow += top2 + (rows1 -1) * 0.5;
    *bestcol += left2 + (cols1 -1) * 0.5;

    return true;
}

bool subpixel_long (size_t bestr, size_t bestc, size_t rows, size_t cols,
        double (*cbuff), double *bestrow, double *bestcol, double *bestval,
        double *covar)
{
    double  A[6];        /* Coefficients a,b,c,d,e,f    in order */
    double  Q[9];        /* Correlation scores, by row */
    double  subr, subc,    interp, denom;
    long   r, c;
 

    *bestrow = bestr;
    *bestcol = bestc;

    /* Print correlation array as a diagnostic */
    /*...
    printf ("Pixel-resolution maximum: %d %d\n", bestr, bestc);
    for (r=0; r<rows; r++) {
    for (c=0; c<cols; c++) {
        printf ("%5.2f ", cbuff[r*cols+c]);
    }
    printf ("\n");
    }
    ...*/

    /* Check that pixel-resolution maximum is interior */
    if (bestr == 0 || bestc == 0 || bestr ==  rows-1  ||
        bestc ==  cols-1 ) {
#ifdef PRINT_ERR
         fprintf (stderr, "Pixel-resolution maximum not interior bestr %d rows %d\n", bestr, rows);
#endif
        return false;

    }

    /* Make sure that pixel-resolution maximum is a strong maximum.
     * Also string out 3x3 patch of correlation scores around the maximum
     * in row order
     */
    for (r=bestr-1; r<=bestr+1; r++) {
        for (c=bestc-1; c<=bestc+1; c++) {
                Q[3*(r-(bestr-1)) + (c-(bestc-1))] = cbuff[r*cols+c];
            if (r == bestr && c == bestc) continue;
            if (cbuff[r*cols+c] >= cbuff[bestr*cols+bestc]) {
#ifdef PRINT_ERR
                 fprintf (stderr, "Not a strong maximum\n");
#endif
                return false;
            }
        }
    }

    /* Compute coefficients of biquadratic fit */
    for (r=0; r<6; r++) {
        A[r] = 0.0;
        for (c=0; c<9; c++) {
                A[r] += fitmat[r][c] * Q[c];
        }
        /*printf ("A[%d] = %f\n", r, A[r]);*/
    }

    denom = 4*A[0]*A[1] - A[2]*A[2];
    if (fabs(denom) < 1.0e-6) {
        fprintf (stderr, "Ill conditioned peak\n");
        return false;
    }

    /* Relative Covariance as Curvature matrix
       The negative signs are caused by maximum point */
    //covar[0] is the curvature along x ?
    //cover[1] is the curvature along y ?
    if (covar != NULL) {
        covar[0] = -2.0 * A[1]/denom;
        covar[1] =  A[2] /denom;
        covar[2] = - 2.0 * A[0]/denom;
        //covar[0] = - 2.0 * A[1] / (4 * A[0] * A[1] - A[2] * A[2]);
        //covar[1] =  A[2] / (4 * A[0] * A[1] - A[2] * A[2]);
        //covar[2] = - 2.0 * A[0] / (4 * A[0] * A[1] - A[2] * A[2]);
    }

    /* Compute subpixel offset */
    subc = (-2*A[1]*A[3] + A[2]*A[4]) / denom;
    subr = (-2*A[0]*A[4] + A[2]*A[3]) / denom;

    /*printf ("subpixel offset: %f %f\n", subr, subc);*/

    /* Verify that subpixel offset is within +/- 1 */
    if (fabs(subc) >= 1.0 || fabs(subr) >= 1.0) {
        // printf ( "Subpixel offset > 1 %f %f \n", subc, subr);
        // printf(" Q = %f %f %f\n Q =   %f %f %f\n Q =   %f %f %f \n", Q[0], Q[1], Q[2], Q[3], Q[4], Q[5], Q[6], Q[7], Q[8]);
         
#ifdef PRINT_ERR
       fprintf (stderr, "Subpixel offset > 1 subc %f subr %f \n", subc, subr);
#endif
        return false;
    }
    *bestrow = bestr + subr;
    *bestcol = bestc + subc;

    /* Interpolate the correlation score */
    interp = A[0]*subc*subc + A[1]*subr*subr + A[2]*subc*subr +
         A[3]*subc + A[4]*subr + A[5];
    /*printf ("Coeff %f, interpolated %f\n", *bestval, interp);*/
    if (bestval)
        *bestval = interp;
     

    return true;
}
