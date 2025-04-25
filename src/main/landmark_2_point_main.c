/** 
 * \file landmark_2_point_main.c
 * \author Cecilia Mauceri
 * \date 2024
 * 
 * \brief Make a PLY pointcloud or mesh file from a landmark file
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

#include <stdbool.h>                                     // for bool
#include <stdint.h>                                      // for int32_t
#include <stdio.h>                                       // for printf
#include <stdlib.h>                                      // for NULL, EXIT_F...
#include <string.h>

#include "landmark_tools/landmark_util/landmark.h"       // for Read_LMK
#include "landmark_tools/landmark_util/point_cloud2grid.h"  // for PointStructure
#include "landmark_tools/utils/parse_args.h"             // for m_getarg
#include "rply.h"                                        // for e_ply_storag...
#include "landmark_tools/utils/safe_string.h"
void show_usage_and_exit()
{
    printf("Write a landmark to a ply mesh or pointcloud.\n");
    printf("Usage for landmark_2_point:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -landmark   <filename> - input lmkfile\n");
    printf("    -ply  <filename> - output PLY filepath\n");
    printf("  Optional arguments:\n");
    printf("    -filetype <PLY_ASCII|PLY_LITTLE_ENDIAN|PLY_BIG_ENDIAN> - (default arch endian)\n");
    printf("    -structure <POINTCLOUD|MESH> - (default MESH)\n");
    printf("    -frame <WORLD|LOCAL|RASTER> - (default WORLD)\n");
    exit(EXIT_FAILURE);
}


int32_t main(int32_t argc, char **argv)
{
    char *pointfile= NULL;
    char *lmkfile = NULL;
    char *filetype_str = NULL;
    char *structure_str = NULL;
    char *frame_str = NULL;
    
    argc--;
    argv++;
    
    if (argc==0) show_usage_and_exit();
    
    while (argc>0)
    {
        if (argc==1) show_usage_and_exit();
        if ((m_getarg(argv, "-ply",    &pointfile,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-landmark",   &lmkfile,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-filetype",   &filetype_str,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-structure",   &structure_str,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-frame",   &frame_str,        CFO_STRING)!=1))
            show_usage_and_exit();
        
        argc-=2;
        argv+=2;
    }
    
    enum e_ply_storage_mode_ filetype = strToPlyFileType(filetype_str);
    enum PointStructure structure = strToStructure(structure_str);
    enum PointFrame frame = strToFrame(frame_str);
    
    LMK lmk = {0};
    bool success = Read_LMK(lmkfile, &lmk);
    if(!success){
        SAFE_PRINTF(256, "Failed to read landmark file at %s\n", lmkfile);
        return EXIT_FAILURE;
    }
    
    if(structure == POINTCLOUD){
        success = Write_LMK_PLY_Points(pointfile, &lmk, filetype, frame);
    }else{
        success = Write_LMK_PLY_Facet(pointfile, &lmk, filetype, frame);
    }
 
    free_lmk(&lmk);
    if(success){
        SAFE_PRINTF(256, "Landmark file saved at %s\n", pointfile);
        return EXIT_SUCCESS;
    }else{
        SAFE_PRINTF(256, "Failed to save landmark file at %s\n", pointfile);
        return EXIT_FAILURE;
    }
}
