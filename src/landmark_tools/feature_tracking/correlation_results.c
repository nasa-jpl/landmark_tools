/**
 * \file   correlation_results.c
 * \author Yang Cheng
 * \brief  Implementation of correlation results management functions
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

#include <math.h> // for NAN
#include <stdlib.h>  // for malloc, free
#include <stdbool.h> // for bool, false
#include "landmark_tools/feature_tracking/correlation_results.h"

bool allocate_correlation_results(CorrelationResults* corr_struct, size_t num_pixels) {
    // Allocate memory for each array
    corr_struct->delta_x = (float*)malloc(sizeof(float) * num_pixels);
    corr_struct->delta_y = (float*)malloc(sizeof(float) * num_pixels);
    corr_struct->delta_z = (float*)malloc(sizeof(float) * num_pixels);
    corr_struct->correlation = (float*)malloc(sizeof(float) * num_pixels);
    
    // Check if any allocation failed
    if (corr_struct->delta_x == NULL || corr_struct->delta_y == NULL ||
        corr_struct->delta_z == NULL || corr_struct->correlation == NULL) {
        destroy_correlation_results(corr_struct);
        return false;
    }
    
    // Initialize all values to NAN
    for (size_t i = 0; i < num_pixels; ++i) {
        corr_struct->delta_x[i] = NAN;
        corr_struct->delta_y[i] = NAN;
        corr_struct->delta_z[i] = NAN;
        corr_struct->correlation[i] = NAN;
    }
    
    return true;
}

void destroy_correlation_results(CorrelationResults* corr_struct) {
    if (corr_struct->delta_x != NULL) {
        free(corr_struct->delta_x);
        corr_struct->delta_x = NULL;
    }
    if (corr_struct->delta_y != NULL) {
        free(corr_struct->delta_y);
        corr_struct->delta_y = NULL;
    }
    if (corr_struct->delta_z != NULL) {
        free(corr_struct->delta_z);
        corr_struct->delta_z = NULL;
    }
    if (corr_struct->correlation != NULL) {
        free(corr_struct->correlation);
        corr_struct->correlation = NULL;
    }
} 
