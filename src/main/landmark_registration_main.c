/** 
 * \file landmark_registration_main.c
 * \author Yang Cheng
 * 
 * \brief Register two landmark files using Forstner interest features
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

/*-----------------------------------------------------------*/
/*------------------------ Includes -------------------------*/
/*-----------------------------------------------------------*/
#include <math.h>
#include <stdint.h>
#include <stdio.h>                  // for fprintf, printf, fclose, fopen
#include <stdlib.h>                 // for malloc, exit, free
#include <string.h>                 // for strcmp, memcpy
#include <assert.h>                                         // for assert

#include "landmark_tools/feature_selection/int_forstner_extended.h"           // for int_forstner_nbest_even_distribution
#include "landmark_tools/feature_tracking/corr_image_long.h"
#include "landmark_tools/feature_tracking/parameters.h"                     // for Parameters
#include "landmark_tools/image_io/image_utils.h"
#include "landmark_tools/image_io/imagedraw.h"              // for DrawArrow, DrawCircle, DrawFeatur...
#include "landmark_tools/math/homography_util.h"        // for inverseHomography33, homographyTr...
#include "landmark_tools/landmark_util/landmark.h"    // for LMK_Col_Row2World, Read_L...
#include "landmark_tools/landmark_util/estimate_homography.h"
#include "landmark_tools/math/math_utils.h"
#include "landmark_tools/math/point_line_plane_util.h"  // for Point_Clouds_rot_T_RANSAC, normal...
#include "landmark_tools/utils/parse_args.h"
#include "math/mat3/mat3.h"                   // for mult331, mult333, sub3, zero3, copy3
#include "landmark_tools/landmark_registration/landmark_registration.h"
#include "landmark_tools/utils/safe_string.h"

void show_usage_and_exit()
{
    printf("Reregister landmarks. The child landmark will be reprojected into the base landmark's reference frame.\n");
    printf("Usage for landmark_register: \n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -base   <filename> - base landmark\n");
    printf("    -child   <filename> - child landmark\n");
    printf("    -parameters   <filename> - parameter file\n");
    exit(EXIT_FAILURE);
}

int32_t main(int32_t argc, char **argv)
{
    char *baselmkfile = NULL;
    char *childlmkfile  = NULL;
    char *parametersfile = NULL;
    
    argc--;
    argv++;
    
    if (argc==0) show_usage_and_exit();
    
    while(argc>0)
    {
        if (argc==1) show_usage_and_exit();
        if((m_getarg(argv, "-base",   &baselmkfile, CFO_STRING)!=1) &&
           (m_getarg(argv, "-child",   &childlmkfile,  CFO_STRING)!=1) &&
           (m_getarg(argv, "-parameters",   &parametersfile,  CFO_STRING)!=1))
            if(argc == 2) break;
        argc-=2;
        argv+=2;
    }
    
    Parameters parameters;
    load_default_parameters(&parameters);
    if(parametersfile==NULL){
        printf("No parameter file provided. Using defaults.\n");
    }else{
        int32_t success = read_parameterfile(parametersfile, &parameters);
        if(!success){
            SAFE_PRINTF(256, "Cannot load %s\n", parametersfile);
            return EXIT_FAILURE;
        }
    }
    print_parameters(parameters);
    
    if(RegisterLandmarks(parameters, baselmkfile, childlmkfile)){
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}