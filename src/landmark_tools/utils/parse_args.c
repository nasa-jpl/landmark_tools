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

#include "landmark_tools/utils/parse_args.h"

#include <stdio.h>   // for sscanf
#include <string.h>  // for strncmp
#include "landmark_tools/utils/safe_string.h"


int32_t m_getarg(char **argv, char *argname, void *argvarptr, int32_t argtype)
{
    float dummy;

    if (strncmp(argv[0], argname, strlen(argv[0])) == 0 && strlen(argv[0]) == strlen(argname))
    {
        SAFE_PRINTF(512, "%s = %s\n", argname, argv[1]);
        if (argtype == CFO_STRING)
            *((char **)argvarptr) = argv[1];

        else if (argtype == CFO_DOUBLE)
        {
            if (sscanf(argv[1], "%f", &dummy) != 1)
            {
                SAFE_PRINTF(512, "m_getarg() ==>> Error reading %s value: %s\n", argname, argv[1]);
                return -1;
            }
            else
                *((double *)argvarptr) = (double)dummy;
        }

        else if (argtype == CFO_FLOAT)
        {
            if (sscanf(argv[1], "%f", &dummy) != 1)
            {
                SAFE_PRINTF(512, "m_getarg() ==>> Error reading %s value: %s\n", argname, argv[1]);
                return -1;
            }
            else
                *((float *)argvarptr) = dummy;
        }

        else if (argtype == CFO_INT)
        {
            if (sscanf(argv[1], "%d", ((int *)argvarptr)) != 1)
            {
                SAFE_PRINTF(512, "m_getarg() ==>> Error reading %s value: %s\n", argname, argv[1]);
                return -1;
            }
        }

        return 1;
    }
    else
        return -1;
}
