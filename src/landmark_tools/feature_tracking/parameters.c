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

#include "landmark_tools/feature_tracking/parameters.h"

#include <stdio.h>          // for sscanf, fprintf, fopen, fgets, printf, FILE
#include <string.h>         // for strstr
#include <yaml.h>

#include "landmark_tools/utils/two_level_yaml_parser.h"

void load_default_parameters (Parameters *ftParms)
{
    // Matching parameters
    ftParms->matching.correlation_window_size = DEFAULT_CORRELATION_WINDOW_SIZE;
    ftParms->matching.search_window_size      = DEFAULT_SEARCH_WINDOW_SIZE;
    ftParms->matching.min_correlation         = (float)DEFAULT_MIN_CORRELATION;
    
    // Sliding window parameters
    ftParms->sliding.block_size               = DEFAULT_BLOCK_SIZE;
    ftParms->sliding.step_size                = DEFAULT_STEP_SIZE;
    ftParms->sliding.min_n_features           = DEFAULT_MIN_N_FEATURES;
    ftParms->sliding.feature_influence_window = DEFAULT_FEATURE_INFLUENCE_WINDOW;
    ftParms->sliding.reprojection_threshold   = (float)DEFAULT_REPROJECTION_THRESHOLD;
    ftParms->sliding.max_delta_map            = (float)DEFAULT_MAX_DELTA_MAP;
    
    // Feature detector parameters
    ftParms->detector.window_size             = DEFAULT_FORSTNER_FEATURE_WINDOW_SIZE;
    ftParms->detector.min_dist_feature        = (float)DEFAULT_MIN_DIST_FEATURE;
    ftParms->detector.num_features            = DEFAULT_NUM_FEATURES;
    
    return;
} 

bool read_parameterfile(char *filename, Parameters *ftParms)
{
    const char* parent_keys[] = {"feature_match", "forstner_feature_detector", "sliding_window"};
    const char* child_keys[] = {
        // Feature match parameters
        "correlation_window_size", "search_window_size", "min_correlation",
        // Forstner detector parameters
        "window_size", "min_dist_feature", "num_features",
        // Sliding window parameters
        "block_size", "step_size", "min_n_features", "feature_influence_window",
        "reprojection_threshold", "max_delta_map"
    };
    size_t num_child_keys[] = {3, 3, 6};
    const char* values[12] = {""};
    
    if(!parseYaml(filename,
                  parent_keys,
                  3,
                  child_keys,
                  num_child_keys,
                  false,
                  values)){
        return false;
    }
    
    // Feature match parameters
    if(strncmp(values[0], "", strlen(values[0])) != 0)
        ftParms->matching.correlation_window_size = atoi(values[0]);
    if(strncmp(values[1], "", strlen(values[1])) != 0)
        ftParms->matching.search_window_size = atoi(values[1]);
    if(strncmp(values[2], "", strlen(values[2])) != 0)
        ftParms->matching.min_correlation = atof(values[2]);
    
    // Forstner feature detector parameters
    if(strncmp(values[3], "", strlen(values[3])) != 0)
        ftParms->detector.window_size = atoi(values[3]);
    if(strncmp(values[4], "", strlen(values[4])) != 0)
        ftParms->detector.min_dist_feature = atof(values[4]);
    if(strncmp(values[5], "", strlen(values[5])) != 0)
        ftParms->detector.num_features = atoi(values[5]);
    
    // Sliding window parameters
    if(strncmp(values[6], "", strlen(values[6])) != 0)
        ftParms->sliding.block_size = atoi(values[6]);
    if(strncmp(values[7], "", strlen(values[7])) != 0)
        ftParms->sliding.step_size = atoi(values[7]);
    if(strncmp(values[8], "", strlen(values[8])) != 0)
        ftParms->sliding.min_n_features = atoi(values[8]);
    if(strncmp(values[9], "", strlen(values[9])) != 0)
        ftParms->sliding.feature_influence_window = atoi(values[9]);
    if(strncmp(values[10], "", strlen(values[10])) != 0)
        ftParms->sliding.reprojection_threshold = atof(values[10]);
    if(strncmp(values[11], "", strlen(values[11])) != 0)
        ftParms->sliding.max_delta_map = atof(values[11]);
    
    // Validate and adjust parameters
    if(ftParms->matching.correlation_window_size%2 == 0) 
        ftParms->matching.correlation_window_size +=1;
    if(ftParms->matching.search_window_size %2 == 0) 
        ftParms->matching.search_window_size +=1;
    while(ftParms->matching.search_window_size <= ftParms->matching.correlation_window_size)
    {
        ftParms->matching.search_window_size +=2;
    }
    
    // Ensure step_size is at least 1
    if(ftParms->sliding.step_size < 1) 
        ftParms->sliding.step_size = 1;
    
    // Ensure feature_influence_window is odd
    if(ftParms->sliding.feature_influence_window%2 == 0) 
        ftParms->sliding.feature_influence_window +=1;
    
    // Ensure detector window size is odd
    if(ftParms->detector.window_size%2 == 0) 
        ftParms->detector.window_size +=1;
    
    return true;
}

void print_parameters(Parameters parameters)
{
    printf("feature_match: \n");
    printf("  correlation_window_size: %d\n", parameters.matching.correlation_window_size);
    printf("  search_window_size: %d\n", parameters.matching.search_window_size);
    printf("  min_correlation: %f\n", parameters.matching.min_correlation);
    
    printf("forstner_feature_detector: \n");
    printf("  window_size: %d\n", parameters.detector.window_size);
    printf("  min_dist_feature: %f\n", parameters.detector.min_dist_feature);
    printf("  num_features: %d\n", parameters.detector.num_features);
    
    printf("sliding_window: \n");
    printf("  block_size: %d\n", parameters.sliding.block_size);
    printf("  step_size: %d\n", parameters.sliding.step_size);
    printf("  min_n_features: %d\n", parameters.sliding.min_n_features);
    printf("  feature_influence_window: %d\n", parameters.sliding.feature_influence_window);
    printf("  reprojection_threshold: %f\n", parameters.sliding.reprojection_threshold);
    printf("  max_delta_map: %f\n", parameters.sliding.max_delta_map);
}
