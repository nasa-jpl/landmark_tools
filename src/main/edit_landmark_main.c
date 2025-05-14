/** 
 * \file edit_landmark_main.c
 * \author Cecilia Mauceri
 * \date 2024-03-24
 * 
 * \brief Crop or scale a landmark file
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

#include <stdbool.h>                                // for bool, true
#include <stdint.h>                                 // for int32_t
#include <stdio.h>                                  // for printf, sscanf
#include <stdlib.h>                                 // for NULL, EXIT_FAILURE
#include <string.h>                                 // for strncmp

#include "landmark_tools/landmark_util/landmark.h"  // for free_lmk, Crop_In...
#include "landmark_tools/utils/parse_args.h"        // for m_getarg, CFO_STRING
#include "landmark_tools/utils/safe_string.h"

void  show_usage_and_exit()
{
    printf("Crop or scale a landmark file\n\n");
    printf("SUBSET uses the same tangent plane and rastor grid as the original DEM\n"); 
    printf("CROP defines a new tangent plane at the center of the cropped area and performs interpolation\n");
    printf("Usage for render_landmark:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -input   <filename> - input landmark filepath\n");
    printf("    -output   <filename> - output landmark filepath\n");
    printf("    -operation   <CROP|RESCALE|SUBSET> - what operation to perform\n");
    printf("  Optional arguments:\n");
    printf("    -scale   <double> - scale for RESCALE operation\n");
    printf("    -roi   <left> <top> <width> <height> - roi for crop and subset operations\n");
    exit(EXIT_FAILURE);
}

int32_t main (int32_t argc, char **argv)
{
    char *infile=NULL;
    char *outfile=NULL;
    char *operation=NULL;
    double scale=1.0;
    int32_t roi_left = -1;
    int32_t roi_top = -1;
    int32_t roi_width = -1;
    int32_t roi_height = -1;
    
    argc--;
    argv++;
    
    if (argc==0) show_usage_and_exit();
    
    for (int32_t i = 1; i < argc; i+=2) {
        //Arguments with one value
        if ((m_getarg(argv, "-input", &infile,  CFO_STRING) == 1) ||
            (m_getarg(argv, "-output", &outfile,  CFO_STRING) == 1) ||
            (m_getarg(argv, "-operation", &operation, CFO_STRING) == 1) ||
            (m_getarg(argv, "-scale", &scale, CFO_DOUBLE) == 1))
        {
            argv+=2;
        }else if (m_getarg(argv, "-roi", &roi_left, CFO_INT) == 1){
            // roi has four values
            if( (sscanf(argv[2], "%d", (&roi_top)) == 1) &&
               ( sscanf(argv[3], "%d", (&roi_width)) == 1) &&
               ( sscanf(argv[4], "%d", (&roi_height)) == 1) ){
                argv +=5;
                i += 3;
            }else{
                printf("Error reading roi value.\n");
                show_usage_and_exit();
            }
        }else{
            //Undefined argument
            show_usage_and_exit();
        }
    }
    
    // Required arguments
    if(infile==NULL | outfile==NULL | operation==NULL){
        show_usage_and_exit();
    }
    
    LMK lmk = {0};
    LMK lmk_out = {0};
    if(!Read_LMK(infile, &lmk)){
        SAFE_PRINTF(256, "Failed to read landmark file: %s\n", infile);
        return EXIT_FAILURE;
    }
    
    bool success = true;
    if(strncmp(operation, "RESCALE", strlen(operation))==0){
        if(scale == 1.0){
            printf("Failed to parse scale factor\n");
            show_usage_and_exit();
        }
        
        success &= ResampleLMK(&lmk, &lmk_out, scale);
        
    }else if(strncmp(operation, "CROP", strlen(operation))==0){
        if(roi_left == -1 || roi_top == -1 || roi_height == -1 || roi_width == -1){
            printf("Failed to parse roi factor\n");
            show_usage_and_exit();
        }
        
        success &= Crop_IntepolateLMK(&lmk, &lmk_out, roi_left, roi_top, roi_width, roi_height);
    }else if(strncmp(operation, "SUBSET", strlen(operation))==0){
        if(roi_left == -1 || roi_top == -1 || roi_height == -1 || roi_width == -1){
            printf("Failed to parse roi factor\n");
            show_usage_and_exit();
        }
        
        success &= SubsetLMK(&lmk, &lmk_out, roi_left, roi_top, roi_width, roi_height);
    }else{
        show_usage_and_exit();
    }
    
    if(success){
        success &= Write_LMK(outfile, &lmk_out);
    }
    
    free_lmk(&lmk);
    free_lmk(&lmk_out);
    
    if(success){
        SAFE_PRINTF(256, "Landmark file written to: %s\n", outfile);
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}
