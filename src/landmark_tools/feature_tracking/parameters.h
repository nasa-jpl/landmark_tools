/**
 * \file   parameters.h
 * \author Yang Cheng
 * \brief  Read and write user configurable parameters
 *
 * \section updates Update History
 * - Created: March 2020
 * - Updated: 2023-08-17 by Cecilia Mauceri
 * - Updated: 2024-08-26 by Cecilia Mauceri
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

#ifndef _LANDMARK_TOOLS_PARAMETERS_H_
#define _LANDMARK_TOOLS_PARAMETERS_H_

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>  // for int32_t, int8_t

#define DEFAULT_CORRELATION_WINDOW_SIZE       25            /*!< \brief Default value for `Parameters.correlation_window_size`*/
#define DEFAULT_SEARCH_WINDOW_SIZE     36           /*!< \brief Default value for `Parameters.search_window_size`*/
#define DEFAULT_FORSTNER_FEATURE_WINDOW_SIZE     9             /*!< \brief Default value for `Parameters.forstner_feature_window_size`*/
#define DEFAULT_MIN_CORRELATION        0.3           /*!< \brief Default value for `Parameters.min_correlation`*/
#define DEFAULT_NUM_FEATURES           600           /*!< \brief Default value for `Parameters.num_features`*/
#define DEFAULT_MIN_DIST_FEATURE       5.0             /*!< \brief Default value for `Parameters.min_dist_feature`*/

typedef struct {
    /**
     * \brief Size of the template used for correlation matching.
     *
     * This parameter defines the dimensions of the template extracted
     * from the first image, which will be used for correlation matching
     * against the search window in the second image. The template is slid
     * over the search window to find the best matching region. A smaller template size is more accurate. Larger template is more robust to noise.
     *
     * @note The template size should be smaller than or equal to the
     * search window size to ensure effective matching. Recommended value range from [1, 50] pixels . */
    int32_t correlation_window_size;
    
    /**
     * \brief Size of the search window in the second image.
     *
     * This parameter defines the dimensions of the window in the second image
     * where the template from the first image will be slid over to perform
     * the correlation matching. A larger search window allows for a more
     * extensive search area but increases computational cost quadratically.
     * Increase to compensate for large map tie error or distortion. Decrease to reduce outliers and improve speed.
     *
     * @note The search window size should be greater than or equal to
     * the template size to ensure proper matching.
     */
    int32_t search_window_size;
    
    /**
     * \brief Neighborhood size for Foestner (1987) interest operator.
     *
     * Foestner interest is used to select features points for `landmark_registration`. This value is not too sensitive.
     *
     * @note Must be an odd number. Recommended value range from [5, 21] pixels  */
    int32_t forstner_feature_window_size;
    
    /**
     * \brief Distance between closest pair of feature points in meters.
     *
     * //TODO (Cecilia) check that this is really meters.
     * This threshold forces the features to be distributed across image. A good distribution of feature points makes the homography estimate more accurate.
     *
     * @note Recommended value range from [0.0, 5.0] meters */
    float min_dist_feature;
    
    /**
     * \brief Maximum number of forstner features to consider for homography
     *
     * @note Recommended value range from [0, 600] features  */
    int32_t num_features;
    
    /**
     * \brief Minimum normalized correlation score for selection for TODO
     *
     * @note Required value range from [0.0, 1.0]*/
    float min_correlation;
} Parameters;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \brief Read values from `filename` into Parameters struct.
 *
 * Each line in the file should be in the format "`Parameter <variable name> <value>`". For example, "Parameter `corr_window_rows` 10".
 * Missing variable names will not be assigned values.
 *
 * \param[in] filename cstring filename
 * \param[out] parameters Parameters struct pointer to be populated with values from file
 * \return false if file cannot be opened to read
 * \return true on success
 */
bool read_parameterfile(char *filename, Parameters *parameters);

/**
 * \brief Uses the default values defined in the Parameters header file to populate an Parameters struct
 * \param[out] Parameters struct pointer to populate
 */
void load_default_parameters (Parameters *parameters);

/**
 \brief Print parameters to stdout
 
 \param[in] parameters 
*/
void print_parameters(Parameters parameters);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LANDMARK_TOOLS_PARAMETERS_H_
