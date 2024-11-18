/** 
 * \file point_2_landmark_main.c
 * \author Cecilia Mauceri
 * \date 2024
 * 
 * \brief Make a landmark file from a PLY point cloud
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

#include <stdbool.h>                                         // for bool, false
#include <stdint.h>                                          // for int32_t
#include <stdio.h>                                           // for printf
#include <stdlib.h>                                          // for free
#include <string.h>                                          // for strncpy

#include "landmark_tools/landmark_util/landmark.h"           // for free_lmk
#include "landmark_tools/map_projection/datum_conversion.h"  // for Planet
#include "landmark_tools/landmark_util/point_cloud2grid.h"
#include "landmark_tools/utils/parse_args.h"                 // for m_getarg

void show_usage_and_exit()
{
    printf("Convert point cloud to landmark format.\n");
	printf("Usage for point_2_landmark:\n");
	printf("------------------\n");
	printf("  Required arguments:\n");
	printf("    -p   <filename> - input point cloud file.\n");
    printf("    -l   <filename> - output lmkfile\n");
	printf("    -d   <float> - resolution in meters per pixel\n");
    printf("    -lt   <float> - latitude of center anchor point\n");
    printf("    -lg   <float> - longitude of center anchor point\n");
    printf("    -ele   <float> - elevation of center anchor point in meters\n");
    printf("    -s   <float> - map width in meters\n");
    printf("    -sy   <float> - map height in meters\n");
    printf("    -planet <Moon|Earth|Mars> - (default Moon)\n");
    printf("    -filetype <POINT|PLY> - file format of input file (default POINT)\n");
    printf("    -frame <WORLD|LOCAL|RASTER> - reference frame of the input pointcloud (default WORLD)\n");
	exit(EXIT_FAILURE);
}


int32_t main(int32_t argc, char **argv)
{
    char *pointfile= NULL;
    char *lmkfile = NULL;
    char *filetype_str = NULL;
    char *planet_str = NULL;
    char *frame_str = NULL;
    float res;
    float lat;
    float lg;
    float size;
    float sizey;
    float ele = 0.;
    
    argc--;
    argv++;
    
    if (argc==0) show_usage_and_exit();
    
    while (argc>0)
    {
        if (argc==1) show_usage_and_exit();
        //TODO make planet and filetype optional
        if ((m_getarg(argv, "-p",    &pointfile,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-l",   &lmkfile,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-d",   &res,        CFO_FLOAT)!=1) &&
            (m_getarg(argv, "-lt",  &lat,        CFO_FLOAT)!=1) &&
            (m_getarg(argv, "-lg",  &lg,        CFO_FLOAT)!=1) &&
            (m_getarg(argv, "-ele",  &ele,        CFO_FLOAT)!=1) &&
            (m_getarg(argv, "-s",   &size,        CFO_FLOAT)!=1) &&
            (m_getarg(argv, "-sy",   &sizey,        CFO_FLOAT)!=1) &&
            (m_getarg(argv, "-planet",    &planet_str,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-filetype",   &filetype_str,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-frame",   &frame_str,        CFO_STRING)!=1))
            show_usage_and_exit();
        
        argc-=2;
        argv+=2;
    }
    
    enum Planet planet = strToPlanet(planet_str);
    if(planet == Planet_UNDEFINED){
        show_usage_and_exit();
    }
    
    enum PointFileType filetype = strToPointFileType(filetype_str);
    if(filetype == UNDEFINED){
        show_usage_and_exit();
    }
    
    enum PointFrame frame = strToFrame(frame_str);
    
    // Read Point Cloud
    bool read_success = false;
    
    double *pts;
    uint8_t *bv;
    size_t num_pts;
    
    if(filetype == POINT){
        read_success = readinpoints_ascii(pointfile, &pts, &bv, &num_pts);
    }else if(filetype == PLY){
        read_success = readinply(pointfile, &pts, &bv, &num_pts);
    }
    
    if(!read_success){
        printf("Unable to read %.256s\n", pointfile);
        return EXIT_FAILURE;
    }
    
    // Generate lmk header
    LMK lmk = {0};
    lmk.BODY = planet;
    lmk.num_cols = (int32_t)((double)size / res);
    lmk.num_rows = (int32_t)((double)sizey / res);
    lmk.num_pixels = lmk.num_cols * lmk.num_rows;
    lmk.anchor_col = (float)lmk.num_cols / 2.0;
    lmk.anchor_row = (float)lmk.num_rows / 2.0;
    lmk.resolution = res;
    
    strncpy(lmk.filename, lmkfile, LMK_FILENAME_SIZE);
    strncpy(lmk.lmk_id, "0", LMK_ID_SIZE); //TODO Described in the D_101723_LVS_Ref_Map_Product_ICD document
    
    if(!allocate_lmk_arrays(&lmk, lmk.num_cols, lmk.num_rows)){
        free_lmk(&lmk);
        free(pts);
        free(bv);
        printf("Failed to allocate landmark memory\n");
        return EXIT_FAILURE;
    }
    
    //Calculate map transform matrices
    // [THP 2024/08/05] Last argument should be elevation, not radius of the planet
    // Radius is added internally within calculateAnchorRotation based on planet parameters
    calculateAnchorRotation(&lmk, lat, lg, ele);
    calculateDerivedValuesVectors(&lmk);
    
    // Transform points to landmark coordinate frame
    bool success = point2lmk(pts, bv, num_pts, &lmk, frame);
    free(pts);
    free(bv);
    if(!success){
        free_lmk(&lmk);
        printf("Failed to convert points to landmark coordinate frame\n");
        return EXIT_FAILURE;
    }
    
    success = Write_LMK(lmkfile, &lmk);
    free_lmk(&lmk);
    if(success){
        printf("Landmark file saved at %.256s\n", lmkfile);
        return EXIT_SUCCESS;
    }else{
        printf("Failed to save landmark file at %.256s\n", lmkfile);
        return EXIT_FAILURE;
    }
}
