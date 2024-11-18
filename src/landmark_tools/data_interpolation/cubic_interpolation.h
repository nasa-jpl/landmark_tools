/**
 * \file cubic_interpolation.h
 * \author Yang Cheng
 * \brief  Implementation of cubic sub-pixel interpolation method from 
 * 
 * Lv, Yong & Feng, Qibo & Qi, Liangyu. (2008). A study of sub-pixel interpolation algorithm in digital speckle correlation method. 10.1117/12.807128. 
 * 
 * See discussions, stats, and author profiles for this publication at: https://www.researchgate.net/publication/241293868
 * 
 * \section updates Update History
 * - Created: 6/12/22
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

#ifndef _LANDMARK_TOOLS_CUBIC_INTERPOLATION_H_
#define _LANDMARK_TOOLS_CUBIC_INTERPOLATION_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//!\brief Interpolate pixel value at (x,y)
//!
//!\param[in] img 
//!\param[in] cols 
//!\param[in] rows 
//!\param[in] x 
//!\param[in] y 
//!\param[out] bv interpolated value 
//!\return true if (x,y) in bounds
//!\return false if (x,y) out of bounds  
bool cubic_interpolation(uint16_t *img, int32_t cols, int32_t rows, double x, double y, double *bv);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_CUBIC_INTERPOLATION_H_ */
