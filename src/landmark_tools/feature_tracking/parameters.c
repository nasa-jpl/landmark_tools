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
    ftParms->correlation_window_size          = DEFAULT_CORRELATION_WINDOW_SIZE;
    ftParms->search_window_size               = DEFAULT_SEARCH_WINDOW_SIZE;
    ftParms->forstner_feature_window_size     = DEFAULT_FORSTNER_FEATURE_WINDOW_SIZE;
    ftParms->min_dist_feature       = (float)DEFAULT_MIN_DIST_FEATURE;
    ftParms->num_features           = DEFAULT_NUM_FEATURES;
    ftParms->min_correlation        = (float)DEFAULT_MIN_CORRELATION;
    return;
} 

bool read_parameterfile(char *filename, Parameters *ftParms)
{
    const char* parent_keys[] = {"feature_match", "forstner_feature_detector"};
    const char* child_keys[] = {"correlation_window_size", "search_window_size", "min_correlation",
        "num_features", "min_dist_feature", "forstner_feature_window_size"};
    size_t num_child_keys[] = {3, 3};
    const char* values[6] = {""};
    
    if(!parseYaml(filename,
                  parent_keys,
                  2,
                  child_keys,
                  num_child_keys,
                  false,
                  values)){
        return false;
    }
    
    if(strncmp(values[0], "", strlen(values[0])) != 0)
        ftParms->correlation_window_size = atoi(values[0]);
    if(strncmp(values[1], "", strlen(values[1])) != 0)
        ftParms->search_window_size = atoi(values[1]);
    if(strncmp(values[2], "", strlen(values[2])) != 0)
        ftParms->min_correlation = atof(values[2]);
    
    if(strncmp(values[3], "", strlen(values[3])) != 0)
        ftParms->num_features = atoi(values[3]);
    if(strncmp(values[4], "", strlen(values[4])) != 0)
        ftParms->min_dist_feature = atof(values[4]);
    if(strncmp(values[5], "", strlen(values[5])) != 0)
        ftParms->forstner_feature_window_size = atoi(values[5]);
    
    if(ftParms->correlation_window_size%2 == 0) ftParms->correlation_window_size +=1;
    if(ftParms->search_window_size %2 == 0) ftParms->search_window_size +=1;
    while(ftParms->search_window_size <= ftParms->correlation_window_size)
    {
        ftParms->search_window_size +=2;
    }
    return true;
}

void print_parameters(Parameters parameters)
{
    printf("feature_match: \n");
    printf("  correlation_window_size: %d\n", parameters.correlation_window_size);
    printf("  search_window_size: %d\n", parameters.search_window_size);
    printf("  min_correlation: %f\n", parameters.min_correlation);
    printf("forstner_feature_detector: \n");
    printf("  min_dist_feature: %f\n", parameters.min_dist_feature);
    printf("  num_features: %d\n", parameters.num_features);
    printf("  forstner_feature_window_size: %d\n", parameters.forstner_feature_window_size);
}
