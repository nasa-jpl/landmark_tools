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
#define DEFAULT_BLOCK_SIZE             200            /*!< \brief Default value for `Parameters.block_size`*/
#define DEFAULT_STEP_SIZE              4              /*!< \brief Default value for `Parameters.step_size`*/
#define DEFAULT_MIN_N_FEATURES         20             /*!< \brief Default value for `Parameters.min_n_features`*/
#define DEFAULT_FEATURE_INFLUENCE_WINDOW 7            /*!< \brief Default value for `Parameters.feature_influence_window`*/
#define DEFAULT_REPROJECTION_THRESHOLD 5.0            /*!< \brief Default value for `Parameters.reprojection_threshold`*/
#define DEFAULT_MAX_DELTA_MAP          500.0          /*!< \brief Default value for `Parameters.max_delta_map`*/

/**
 * \brief Parameters for correlation-based feature matching
 */
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
     * \brief Minimum normalized correlation score for selection
     *
     * @note Required value range from [0.0, 1.0]*/
    float min_correlation;
} MatchingParameters;

/**
 * \brief Parameters for sliding window processing
 */
typedef struct {
    /**
     * \brief Size of the sliding window block for processing
     *
     * This parameter defines the size of the window used when processing landmarks
     * in a sliding window fashion. Larger blocks can capture more context but
     * increase memory usage and computation time.
     *
     * @note Recommended value range from [100, 500] pixels */
    int32_t block_size;
    
    /**
     * \brief Step size for downsampling points within a block
     *
     * This parameter controls the sampling density of points processed within
     * each block. Instead of processing every pixel in the block, points are
     * sampled at regular intervals defined by this step size. For example, a
     * step size of 4 means every 4th point in both row and column directions
     * is processed. This downsampling reduces computation time while still
     * maintaining sufficient coverage for feature matching.
     *
     * @note Must be at least 1 (no downsampling). Recommended value range from [1, 10] pixels.
     *       Larger values reduce computation time but may miss important features. */
    int32_t step_size;
    
    /**
     * \brief Minimum number of features required for a valid match
     *
     * This parameter sets the minimum number of matched features required
     * to consider a block match valid. Higher values increase robustness
     * but may reject valid matches in areas with fewer features.
     *
     * @note Recommended value range from [5, 20] features */
    int32_t min_n_features;
    
    /**
     * \brief Size of the influence window around each matched feature point
     *
     * This parameter defines the radius of influence around each matched feature point.
     * Within this window, the feature's delta values (x, y, z) and correlation values
     * are distributed using a Gaussian weight function (exp(-distance)). The influence
     * smoothly decreases with distance from the feature point, with values at the window
     * boundary having minimal impact.
     *
     * @note Must be an odd number. Recommended value range from [3, 15] pixels.
     *       Larger values create smoother transitions but increase computation time. */
    int32_t feature_influence_window;
    
    /**
     * \brief Maximum allowed reprojection error for a feature to be considered an inlier
     *
     * This parameter sets the threshold for reprojection error when determining
     * if a feature match is valid. Higher values allow more matches but may
     * include more outliers.
     *
     * @note Recommended value range from [1.0, 10.0] pixels */
    float reprojection_threshold;
    
    /**
     * \brief Maximum allowed delta values in map frame
     *
     * This parameter sets the maximum allowed delta values (x, y, z) in the
     * map frame. Values exceeding this threshold are considered outliers and
     * filtered out.
     *
     * @note Recommended value range from [100.0, 1000.0] meters */
    float max_delta_map;
} SlidingWindowParameters;

/**
 * \brief Parameters for Forstner feature detector
 */
typedef struct {
    /**
     * \brief Neighborhood size for Foestner (1987) interest operator.
     *
     * Foestner interest is used to select features points for `landmark_registration`. This value is not too sensitive.
     *
     * @note Must be an odd number. Recommended value range from [5, 21] pixels  */
    int32_t window_size;
    
    /**
     * \brief Distance between closest pair of feature points in meters.
     *
     * This threshold forces the features to be distributed across image. A good distribution of feature points makes the homography estimate more accurate.
     *
     * @note Recommended value range from [0.0, 5.0] meters */
    float min_dist_feature;
    
    /**
     * \brief Maximum number of forstner features to consider for homography
     *
     * @note Recommended value range from [0, 600] features  */
    int32_t num_features;
} FeatureDetectorParameters;

/**
 * \brief All parameters for feature tracking and matching
 */
typedef struct {
    MatchingParameters matching;          /**< Parameters for correlation matching */
    SlidingWindowParameters sliding;      /**< Parameters for sliding window processing */
    FeatureDetectorParameters detector;   /**< Parameters for feature detection */
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
