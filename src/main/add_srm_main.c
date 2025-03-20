/** 
 * \file add_srm_main.c
 * \author Cecilia Mauceri
 * \date 2024-07-09
 * 
 * \brief Adds a surface image to an existing landmark file. The image must be the same dimensions and resolution as landmark structure. It must also be in an orthographic projection.
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

#include <stdbool.h>                                // for bool, true
#include <stdint.h>                                 // for int32_t
#include <stdio.h>                                  // for printf
#include <stdlib.h>                                 // for NULL, EXIT_FAILURE
#include <string.h>                                 // for strncmp

#include "landmark_tools/landmark_util/landmark.h"  // for free_lmk, Crop_In...
#include "landmark_tools/utils/parse_args.h"        // for m_getarg, CFO_STRING
#include "landmark_tools/image_io/image_utils.h"             // for load_cha...

void  show_usage_and_exit()
{
    printf("Usage for add_srm:\n");
    printf("Adds a surface image to an existing landmark file. The image must be the same dimensions and resolution as landmark structure. It must also be in an orthographic projection.\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -input   <filename> - input landmark filepath\n");
    printf("    -output   <filename> - output landmark filepath\n");
    printf("    -srm   <filename> - input surface image\n");
    exit(EXIT_FAILURE);
}

int32_t main (int32_t argc, char **argv)
{
    char *infile=NULL;
    char *outfile=NULL;
    char *srmfile=NULL;
    
    argc--;
    argv++;
    
    if (argc==0) show_usage_and_exit();
    
    for (int32_t i = 1; i < argc; i+=2) {
        //Arguments with one value
        if ((m_getarg(argv, "-input", &infile,  CFO_STRING) == 1) ||
            (m_getarg(argv, "-output", &outfile,  CFO_STRING) == 1) ||
            (m_getarg(argv, "-srm", &srmfile, CFO_STRING) == 1))
        {
            argv+=2;
        }else{
            //Undefined argument
            show_usage_and_exit();
        }
    }
    
    // Required arguments
    if(infile==NULL | outfile==NULL | srmfile==NULL){
        show_usage_and_exit();
    }
    
    LMK lmk = {0};
    if(!Read_LMK(infile, &lmk)){
        printf("Failed to read landmark file: %.256s\n", infile);
        return EXIT_FAILURE;
    }
    
    int32_t icols, irows;
    uint8_t *srm_img = load_channel_separated_image(srmfile, &icols, &irows);
    
    if (srm_img == NULL) {
        printf("Failure to load surface reflectance map from %.256s\n", srmfile);
        return EXIT_FAILURE;
    }
    
    if(icols!= lmk.num_cols || irows!=lmk.num_rows){
        printf("SRM dimensions (%dx%d) differ from landmark dimensions (%dx%d)\n", icols, irows, lmk.num_cols, lmk.num_rows);
        return EXIT_FAILURE;
    }
    
    lmk.srm = srm_img;
    bool success = Write_LMK(outfile, &lmk);
    
    free_lmk(&lmk);
    
    if(success){
        printf("Landmark file written to: %.256s\n", outfile);
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}
