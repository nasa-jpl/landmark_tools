/************************************************
 * \file parse_args.h
 * \author Cecilia Mauceri
 * \brief Parse nested yaml files
 *
 *  \copyright Copyright 2024 California Institute of Technology
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
 * 
 ***********************************************/

#ifndef two_level_yaml_parser_h
#define two_level_yaml_parser_h

#include <stdio.h>
#include <stdbool.h>

/**
 \brief Parses Yaml with one level of nesting
 
 \param[in] yaml_filename file to parse
 \param[in] parent_keys array of cstrings containing the keys for the outer level of nesting
 \param[in] num_parent_keys
 \param[in] child_keys flattened array of cstrings containing the keys for the inner level of nesting
 \param[in] num_child_keys array of number of child keys for each parent key
 \param[in] must_include_all flag to make all keys required
 \param[out] values array of cstrings of the corresponding values for each child key
 \return true if file is successfully parsed
 \return false if file parsing fails or if required key is missing
 */
bool parseYaml(const char *yaml_filename,
                  const char** parent_keys,
                  size_t num_parent_keys,
                  const char** child_keys,
                  size_t* num_child_keys,
                  bool must_include_all,
               const char ** values);

#endif /* two_level_yaml_parser_h */
