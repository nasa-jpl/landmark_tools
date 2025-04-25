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

#include "landmark_tools/utils/two_level_yaml_parser.h"
#include <yaml.h>
#include "landmark_tools/utils/safe_string.h"


bool parseYaml(const char *yaml_filename,
                  const char** parent_keys,
                  size_t num_parent_keys,
                  const char** child_keys,
                  size_t* num_child_keys,
                  bool must_include_all,
                  const char ** values){
    FILE *fp = fopen(yaml_filename, "r");
    yaml_parser_t parser;
    yaml_event_t event;
    event.type = YAML_NO_EVENT;

    /* Initialize parser */
    if(!yaml_parser_initialize(&parser)){
        SAFE_FPRINTF(stderr, 512, "Failed to initialize parser: %.256s\n", yaml_filename);
        return false;
    }
    if(fp == NULL){
        SAFE_FPRINTF(stderr, 512, "Failed to open file: %.256s\n", yaml_filename);
        return false;
    }

    /* Set input file */
    yaml_parser_set_input_file(&parser, fp);

    // Calculate the start position of each parent block
    size_t total_child_keys = 0;
    size_t start_index_child[num_parent_keys];
    start_index_child[0] = 0;
    for(size_t i = 0 ; i< num_parent_keys; i++){
        start_index_child[i] = total_child_keys;
        total_child_keys += num_child_keys[i];
    }
    
    bool child_observed[total_child_keys];
    for(size_t i =0; i< total_child_keys; i ++){
        child_observed[i] = false;
    }
    
    size_t current_key_size = 50;
    char current_key[current_key_size];
    current_key[0] = '\0';
    char* current_value = NULL;
    int8_t current_parent = -1;
    while (event.type != YAML_STREAM_END_EVENT) {
        if (!yaml_parser_parse(&parser, &event)) {
            fprintf(stderr, "Error parsing YAML.\n");
            yaml_parser_delete(&parser);
            fclose(fp);
            return false;
        }
        
        if (event.type == YAML_SCALAR_EVENT) {
            if (current_key[0] == '\0') {
                // Save the current key
                strncpy(current_key, (char*)event.data.scalar.value, current_key_size);
                
                // Determine if we're parsing parent or child
                if(current_parent == -1){
                    bool isValid = false;
                    for(size_t i = 0 ; i< num_parent_keys; i++){
                        if (strncmp(current_key, parent_keys[i], current_key_size) == 0){
                            current_parent = i;
                            isValid = true;
                            break;
                        }
                    }
                    
                    if(!isValid){
                        SAFE_FPRINTF(stderr, 512, "ERROR: Yaml contains unexpected key: %.256s.\n.", current_key);
                        yaml_parser_delete(&parser);
                        fclose(fp);
                        return false;
                    }
                }
                
            }else if(current_value == NULL){
                //Parsing key and value pair
                current_value = (char*)event.data.scalar.value;
                
                bool isValid = false;
                size_t child_index_root = start_index_child[current_parent];
                size_t num_child = num_child_keys[current_parent];
                for(size_t i=child_index_root; i<child_index_root+num_child; i++){
                    if (strncmp(current_key, child_keys[i], current_key_size) == 0){
                        if(child_observed[i]){
                            SAFE_FPRINTF(stderr, 512, "WARNING: Yaml contains two of the same key: %.256s\n. Second instance will be used.", current_key);
                        }
                        values[i] = current_value;
                        child_observed[i] = true;
                        isValid = true;
                        break;
                    }
                }
                    
                if(!isValid){
                    SAFE_FPRINTF(stderr, 512, "ERROR: Yaml contains unexpected key: %.256s.\n.", current_key);
                    yaml_parser_delete(&parser);
                    fclose(fp);
                    return false;
                }
                
                current_key[0] = '\0';  // Reset the key for the next value
                current_value = NULL;
            }
        } else if (event.type == YAML_MAPPING_START_EVENT) {
            current_key[0] = '\0';  // Reset current key when starting a new map
            current_value = NULL;
        } else if (event.type == YAML_MAPPING_END_EVENT) {
            current_parent = -1; // Reset when exiting block
        } else if (event.type == YAML_STREAM_END_EVENT) {
            break;
        }
        
    }

    /* Cleanup */
    yaml_parser_delete(&parser);
    fclose(fp);
    
    if(must_include_all){
        for(size_t i=0; i<total_child_keys; i++){
            if (!child_observed[i]) {
                SAFE_FPRINTF(stderr, 512, "ERROR: Yaml file %.256s is missing required key: %.256s\n", yaml_filename, child_keys[i]);
                return false;
            }
        }
    }
    
    return true;
}
