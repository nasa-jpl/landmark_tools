/**
 * \file corr_image_long.h
 * \author Todd Litwin
 * 
 * \brief Image correlation
 * 
 * These are the same equations as corr_image.c with higher capacity integer types. corr_image.c can suffer from overflow errors.
 * This file contains an interface to the CMU funtion corimg(), inherited from Larry Matthies.
 * 
 * \section updates Update History
 * - Contributers: Larry Matthies, Todd Litwin 
 * - Updated: Yang Cheng
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

#ifndef _LANDMARK_TOOLS_CORR_IMAGE_LONG_H_
#define _LANDMARK_TOOLS_CORR_IMAGE_LONG_H_

#include <stdint.h>  // for int32_t
#include <stdbool.h>  // for bool
#include <stdio.h>   // for size_t

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief TODO
 
 \param[] img1 
 \param[] rowBytes1 
 \param[] left1 
 \param[] top1 
 \param[] cols1 
 \param[] rows1 
 \param[] img2 
 \param[] rowBytes2 
 \param[] left2 
 \param[] top2 
 \param[] cols2 
 \param[] rows2 
 \param[] bestrow 
 \param[] bestcol 
 \param[] bestval 
 \param[] covar 
 \return true 
 \return false 
*/
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
                                     double *covar);

/**
 \brief TODO
 
 \param[] img1 
 \param[] rowBytes1 
 \param[] left1 
 \param[] top1 
 \param[] cols1 
 \param[] rows1 
 \param[] img2 
 \param[] rowBytes2 
 \param[] left2 
 \param[] top2 
 \param[] cols2 
 \param[] rows2 
 \param[] bestrow 
 \param[] bestcol 
 \param[] bestval 
 \param[] covar 
 \return true 
 \return false 
*/
bool corimg_long (unsigned char *img1, size_t rowBytes1,
                  size_t left1, size_t top1, size_t cols1, size_t rows1,
        unsigned char *img2, size_t rowBytes2,
                  size_t left2, size_t top2, size_t cols2, size_t rows2,
             double *bestrow, double *bestcol, double *bestval, double *covar);

/**
 \brief TODO
 
 \param[] bestr 
 \param[] bestc 
 \param[] rows 
 \param[] cols 
 \param[] bestrow 
 \param[] bestcol 
 \param[] bestval 
 \param[] covar 
 \return true 
 \return false 
*/
bool subpixel_long (size_t bestr, size_t bestc, size_t rows, size_t cols,
        double (*cbuff), double *bestrow, double *bestcol, double *bestval,
              double *covar);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_CORR_IMAGE_LONG_H_ */
