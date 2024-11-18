/**
 * \file   imagedraw.h
 * \brief  Draw annotations on image array
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


#ifndef _LANDMARK_TOOLS_IMAGEDRAW_H_
#define _LANDMARK_TOOLS_IMAGEDRAW_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <inttypes.h>
#include <stdint.h>  // for int32_t, uint8_t
#include <stdio.h>
#include <stdlib.h>

int32_t DrawFeatureBlock (uint8_t *pixel, int32_t cols, int32_t rows, double x, double y, uint8_t v, int32_t size);
int32_t DrawFeatureCircle (uint8_t *pixel, int32_t cols, int32_t rows, double x, double y, uint8_t v, int32_t size);
int32_t DrawFeatureCross (uint8_t *pixel, int32_t cols, int32_t rows, double x, double y, uint8_t v, int32_t size);
int32_t DrawArrow(uint8_t *greyscale, int32_t cols, int32_t rows, double x0, double y0, double  x1, double  y1, uint8_t c, int32_t size);
int32_t DrawEllipse(uint8_t *greyscale, int32_t cols,int32_t rows,  double x0, double y0, double a, double b,
                      double theta, uint8_t c);
int32_t FillEllipse(uint8_t *greyscale, int32_t cols,int32_t rows,  double x0, double y0, double a, double b,
                      double theta, uint8_t c);
int32_t DrawCircle(uint8_t *greyscale, int32_t cols,int32_t rows,  double x0, double y0, double a, 
                       uint8_t c);
int32_t DrawBox(uint8_t *greyscale, int32_t cols,int32_t rows,  double x0, double y0, int32_t boxsize,uint8_t c, int32_t linesize);
int32_t DrawLine(uint8_t *greyscale, int32_t cols, int32_t rows, double x0, double y0, double  x1, double  y1, uint8_t c, int32_t size);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _LANDMARK_TOOLS_IMAGEDRAW_H_

