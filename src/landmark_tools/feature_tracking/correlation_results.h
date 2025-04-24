/**
 * \file   correlation_results.h
 * \author Yang Cheng
 * \brief  Structures and functions for managing correlation results
 * 
 * \section updates Update History
 * - Created: March 2024
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

#ifndef _LANDMARK_TOOLS_CORRELATION_RESULTS_H_
#define _LANDMARK_TOOLS_CORRELATION_RESULTS_H_

#include <stdint.h>  // for int32_t
#include <stdbool.h> // for bool

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \brief Structure to store correlation results
 * 
 * This structure holds the delta maps (x, y, z) and correlation values
 * for each pixel in the landmark. The deltas represent the differences
 * between the child and base landmarks in the local reference frame.
 */
typedef struct {
    float* delta_x;        /**< Delta in x direction (east) */
    float* delta_y;        /**< Delta in y direction (north) */
    float* delta_z;        /**< Delta in z direction (up) */
    float* correlation;    /**< Correlation value for each point */
} CorrelationResults;

/**
 * \brief Allocate memory for correlation results structure
 * 
 * Allocates memory for the delta maps and correlation values in the
 * CorrelationResults structure.
 * 
 * \param[in,out] corr_struct Structure to allocate memory for
 * \param[in] num_pixels Number of pixels to allocate memory for
 * \return true if allocation successful, false otherwise
 */
bool allocate_correlation_results(CorrelationResults* corr_struct, size_t num_pixels);

/**
 * \brief Free memory allocated for correlation results structure
 * 
 * Frees all memory allocated for the delta maps and correlation values
 * in the CorrelationResults structure.
 * 
 * \param[in,out] corr_struct Structure to free memory for
 */
void destroy_correlation_results(CorrelationResults* corr_struct);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LANDMARK_TOOLS_CORRELATION_RESULTS_H_ 