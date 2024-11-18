/**
 * \file int_forstner_extended.h
 * \author Yang Cheng
 * \brief Helper functions for working with Forstner interest operator
 * 
 * \section updates Update History
 * - Created: Unknown
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

#ifndef _LANDMARK_TOOLS_INT_FORSTNER_EXTENDED_H_
#define _LANDMARK_TOOLS_INT_FORSTNER_EXTENDED_H_

#include <stdint.h>  // for int32_t, int64_t, uint8_t
#include <stdint.h>  // for int32_t, int64_t, uint8_t
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
\brief TODO
\param[in] image intensity image
\param[in] xdim size of X dimension
\param[in] ydim size of Y dimension
\param [in] x0 left-most X coordinate
\param[in] y0 upper-most Y coordinate
\param[in] nx number of columns
\param[in] ny number of rows
\param[in] n size of NxN neighborhood around pixel
\param[in] max maximum length of output lists
\param[out] num number of points in output lists
\param[out] pos2 coordinates of best point
\param[out] intr values of interest operation
\param[in] minDist minimum distance between feature points
 */
int32_t int_forstner_nbest_even_distribution(uint8_t *image, int32_t xdim, int32_t ydim, int32_t x0, int32_t y0,
                                         int32_t nx, int32_t ny, int32_t n, int32_t max, int32_t *num, int64_t (*pos2)[2], float *int32_tr, int32_t minDist);

/**
 \brief TODO
 
 \param[] n 
 \param[] ra 
 \param[] rb 
*/
static void sort_features_descent(int32_t n, float ra[], int64_t rb[]);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_INT_FORSTNER_EXTENDED_H_ */
