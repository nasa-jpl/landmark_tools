/************************************************
 * \file parse_args.h
 * \author Hrand Aghazarian
 * \brief Parse command line arguments
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

#ifndef _LANDMARK_TOOLS_PARSE_ARGS_H_
#define _LANDMARK_TOOLS_PARSE_ARGS_H_

#include <stdint.h>  // for int32_t

enum ArgType{
    CFO_STRING,
    CFO_INT,
    CFO_DOUBLE,
    CFO_FLOAT,
    CFO_HEX
};

#define MAX_NUM_PARS    (32)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
* @brief parses parameters from a command line or a file
* @param[in]  argv  List of arguments
* @param[in]  argname  Name of argument to try to match
* @param[out]  argvarptr  Ptr to the location of the arg
* @param[in]  argtype     Type of the arg variable
* @return 1 for success
* @return 0 for failed
*
*/
int32_t m_getarg(char **argv, char *argname, void *argvarptr, int32_t argtype);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_PARSE_ARGS_H_ */
