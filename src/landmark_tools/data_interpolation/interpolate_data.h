/**
 * \file   interpolate_data.h
 * \brief  Bilinear interpolation methods for matrices
 *
 * \section updates Update History
 * - Created: Unknown
 * - Updated: 
 *   - Aug 2023, Cecilia Mauceri 
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
 */

#ifndef _LANDMARK_TOOLS_INTERPOLATE_DATA_H_
#define _LANDMARK_TOOLS_INTERPOLATE_DATA_H_

#include <inttypes.h>
#include <stdint.h>  // for int16_t, int32_t, uint16_t, uint8_t
#include <stdio.h>   // for size_t
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief Bilinear interpolation of matrix at coordinate (x, y)
 
 If coordinate is at the edge of the matrix, `x==0 || x==xsize-1` or `y==0 || y==ysize-1`, the edge value will be returned without interpolation.
 
 If one of the neighbors of (x,y) is NAN, NAN will be returned.
 
 \param[in] img matrix
 \param[in] xsize width of matrix
 \param[in] ysize height of matrix
 \param[in] x coordinate
 \param[in] y coordinate
 \return interpolated value if in bounds, NAN if out of bounds or one of the neighbors is NAN
 */
double  inter_double_matrix(double *img, size_t xsize, size_t ysize, double  x, double  y);

/**
 \brief Bilinear interpolation of matrix at coordinate (x, y)
 
 If coordinate is at the edge of the matrix, `x==0 || x==xsize-1` or `y==0 || y==ysize-1`, the edge value will be returned without interpolation.
 
 If one of the neighbors of (x,y) is NAN, NAN will be returned.
 
 \param[in] img matrix
 \param[in] xsize width of matrix
 \param[in] ysize height of matrix
 \param[in] x coordinate
 \param[in] y coordinate
 \return interpolated value if in bounds, NAN if out of bounds or one of the neighbors is NAN
 */
double  inter_float_matrix(float *img, size_t xsize, size_t ysize, double  x, double  y);

/**
 \brief Bilinear interpolation of matrix at coordinate (x, y)
 
 If coordinate is at the edge of the matrix, `x==0 || x==xsize-1` or `y==0 || y==ysize-1`, the edge value will be returned without interpolation.
 
 If one of the neighbors of (x,y) is NAN, NAN will be returned.
 
 \param[in] img matrix
 \param[in] xsize width of matrix
 \param[in] ysize height of matrix
 \param[in] x coordinate
 \param[in] y coordinate
 \param[out] val interpolated value
 \return false if out of bounds
 \return true if in bounds
 */
bool  inter_uint8_matrix(uint8_t *img, size_t xsize, size_t ysize, double  x, double  y, uint8_t *val);

/**
 \brief Bilinear interpolation of matrix at coordinate (x, y)
 
 If coordinate is at the edge of the matrix, `x==0 || x==xsize-1` or `y==0 || y==ysize-1`, the edge value will be returned without interpolation.
 
 If one of the neighbors of (x,y) is NAN, NAN will be returned.
 
 \param[in] img matrix
 \param[in] xsize width of matrix
 \param[in] ysize height of matrix
 \param[in] x coordinate
 \param[in] y coordinate
 \return interpolated value if in bounds, NAN if out of bounds or one of the neighbors is NAN
 */
double  inter_short_elevation(short *img, size_t xsize, size_t ysize, double  x, double  y);

/**
 \brief Bilinear interpolation of matrix at coordinate (x, y)
 
 \param[in] img matrix
 \param[in] xsize width of matrix
 \param[in] ysize height of matrix
 \param[in] x coordinate
 \param[in] y coordinate
 \param[out] bv interpolated value
 \return 0 if out of bounds, 1 otherwise
 */
int32_t inter_unsigned_short_image(uint16_t *img, size_t xsize, size_t ysize, double  x, double  y, double *bv);

/**
 \brief Reverse the byte order of a short

 \param[in,out] longone 
*/
void rev_short(int16_t *longone);

/**
 \brief Reverse the byte order of a float
 
 \param[in,out] longone 
*/
void rev_float(float *longone);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif // _LANDMARK_TOOLS_INTERPOLATE_DATA_H_
