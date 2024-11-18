/** 
 * \file create_landmark_from_img_main.c
 * \author Yumi Iwashita
 * \date 2024
 * 
 * \brief Create a landmark file from a PDS4 DEM that has a seperate PDS4 surface reflectance image file 
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

#include <math.h>                                            // for NAN, isnan
#include <stdbool.h>                                         // for bool, false
#include <stdint.h>                                          // for int32_t
#include <stdio.h>                                           // for printf
#include <stdlib.h>                                          // for EXIT_FAI...
#include <string.h>                                          // for strncmp
#include <float.h>

#include "landmark_tools/image_io/image_utils.h"             // for load_cha...
#include "landmark_tools/landmark_util/create_landmark.h"    // for CreateLa...
#include "landmark_tools/landmark_util/landmark.h"           // for free_lmk
#include "landmark_tools/map_projection/datum_conversion.h"  // for Projection
#include "landmark_tools/utils/parse_args.h"                 // for m_getarg
#include "landmark_tools/data_interpolation/interpolate_data.h" // rev_short

#include "landmark_tools/image_io/geotiff_struct.h"

#ifdef USE_GEOTIFF
#include "landmark_tools/image_io/geotiff_interface.h"       // for st_geoti...
#endif //USE_GEOTIFF

static void show_usage(void)
{
    printf("Usage for create_landmark_from_img:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -ele_lbl_file   <filename> - input dem lbl file name\n");
    printf("    -lmk_file   <filename> - output lmk file name\n");
    printf("    -lmk_width_meters   <filename> - lmk col size\n");
    printf("    -lmk_height_meters   <filename> - lmk row size\n");
    printf("    -lmk_res   <filename> - lmk resolution\n");
    printf("    -lmk_center_lat   <filename> - lmk center lat\n");
    printf("    -lmk_center_long   <filename> - lmk center long\n");
    printf("  Optional arguments:\n");
    printf("    -planet <Moon or Earth> - (default Moon)\n");
    printf("    -data_depth_bits <16 or 32> - (default 16)\n");
    printf("    -set_anchor_point_ele <float> - (default NAN, use ele based on a point at anchor lat long)\n");
    printf("    -srm_file <filename> - png image file containing surface reflectance map\n");
    printf("    -srm_lbl_file   <filename> - input srm lbl file name\n");
    printf("    -depth_scaling   <float> - depth scaling (0~1)\n");
}

/////////////////////////////////////////////////////////////////////////
int32_t main(int32_t argc, char** argv)
{
    float lmkcols=0.0, lmkrows = 0.0;
    float lmkres = 0.0;
    char* input_ele_lbl_file_name=0;
    char* input_srm_lbl_file_name=0;
    char* projection_type=0;
    
    char* output_lmk_file_name=0;
    char* planet_str=0;
    char* srm_file_name=0;
    int data_depth_bits = 16;
    float set_anchor_point_ele = NAN;
    double lat0 = 0.0, long0 = 0.0;
    double nodata_value = NAN;
    double depth_scaling = 1.0;
    
    argc--;
    argv++;
    
    if (argc == 0)
    {
        show_usage();
        printf("main() ==>> failed, missing all parameters\n");
        return 1;
    }
    
    int32_t args_found = 0;
    int32_t num_required_args = 7;
    for (int32_t i = 1; i < argc; i+=2) {
        //Required Arguments
        if (
            (m_getarg(argv, "-ele_lbl_file", &input_ele_lbl_file_name,  CFO_STRING) == 1) ||
            (m_getarg(argv, "-lmk_file", &output_lmk_file_name, CFO_STRING) == 1) ||
            (m_getarg(argv, "-lmk_width_meters", &lmkcols,   CFO_FLOAT) == 1) ||
            (m_getarg(argv, "-lmk_height_meters", &lmkrows, CFO_FLOAT) == 1) ||
            (m_getarg(argv, "-lmk_res", &lmkres,     CFO_FLOAT) == 1) ||
            (m_getarg(argv, "-lmk_center_lat", &lat0, CFO_DOUBLE) == 1) ||
            (m_getarg(argv, "-lmk_center_long", &long0, CFO_DOUBLE) == 1) )
        {
            args_found ++;
        }else{
            //Optional arguments
            m_getarg(argv, "-data_depth_bits", &data_depth_bits,  CFO_INT);
            m_getarg(argv, "-planet", &planet_str, CFO_STRING);
            m_getarg(argv, "-set_anchor_point_ele", &set_anchor_point_ele, CFO_FLOAT); // if not specified, use ele based on a point at anchor lat long
            m_getarg(argv, "-srm_file", &srm_file_name, CFO_STRING);
            m_getarg(argv, "-srm_lbl_file", &input_srm_lbl_file_name,  CFO_STRING); // if this is defined, define srm_img_file as well
            m_getarg(argv, "-depth_scaling", &depth_scaling, CFO_FLOAT);
        }
        argv+=2;
    }
    
    enum Planet planet = strToPlanet(planet_str);
    if(planet == Planet_UNDEFINED){
        show_usage();
        return EXIT_FAILURE;
    }
    
    double anchor_longitude_degrees = long0;
    double anchor_latitude_degrees = lat0;
    
    // Use the arguments and geotiff metadata
    LMK lmk = {0};
    lmk.BODY = planet;
    lmk.num_cols = (int32_t)(lmkcols / lmkres);
    lmk.num_rows = (int32_t)(lmkrows / lmkres);
    lmk.num_pixels = lmk.num_cols * lmk.num_rows;
    lmk.anchor_col = (float)lmk.num_cols / 2.0;
    lmk.anchor_row = (float)lmk.num_rows / 2.0;
    lmk.resolution = lmkres;
    anchor_longitude_degrees = long0;
    anchor_latitude_degrees = lat0;
    
    strncpy(lmk.filename, output_lmk_file_name, LMK_FILENAME_SIZE);
    strncpy(lmk.lmk_id, "0", LMK_ID_SIZE);
    
    if(!allocate_lmk_arrays(&lmk, lmk.num_cols, lmk.num_rows)){
        free_lmk(&lmk);
        return EXIT_FAILURE;
    }
    
    GeoTiffData info_ele = {0};
    GeoTiffData info_srm = {0};
    if(!readGeoTiff(input_ele_lbl_file_name, &info_ele)){
        printf("Failed to read: %.256s\n", input_ele_lbl_file_name);
        free_lmk(&lmk);
        return EXIT_FAILURE;
    }
    if(input_srm_lbl_file_name != NULL && !readGeoTiff(input_srm_lbl_file_name, &info_srm)){
        printf("Failed to read: %.256s\n", input_srm_lbl_file_name);
        free_lmk(&lmk);
        return EXIT_FAILURE;
    }
    
    bool ok = false;
    if(input_srm_lbl_file_name == NULL && srm_file_name == NULL){
        printf("Creating landmark with empty surface reflectance map.\n");
        ok = CreateLandmark_dem_only(&info_ele, anchor_latitude_degrees, anchor_longitude_degrees, info_ele.projection, &lmk, set_anchor_point_ele);
    }else{
        //Load the surface reflectance map
        int32_t icols, irows;
        uint8_t *srm_img;
        if(srm_file_name != NULL)
            srm_img = load_channel_seperated_image(srm_file_name, &icols, &irows);
        else{
            //check if the size matches with ele
            if((int)info_ele.imageSize[0] != info_srm.imageSize[0] || (int)info_ele.imageSize[1] != info_srm.imageSize[1] || info_ele.projection != info_srm.projection){
                printf("Loaded srm img and ele img do not match in size-wise\n");
                return EXIT_FAILURE;
            }
            
            icols = info_srm.imageSize[0];
            irows = info_srm.imageSize[1];
            srm_img = malloc(icols*irows*sizeof(uint8_t));
            if(srm_img == NULL){
                fprintf(stderr, "Failure to allocate memory.\n");
                return EXIT_FAILURE;
            }
            
            //Normalize srm image
            float max = 0;
            float min = FLT_MAX;
            for(size_t i=0; i<irows*icols; i++){
                if(info_srm.demValues[i] < min){
                    min = info_srm.demValues[i];
                }
                
                if(info_srm.demValues[i]> max){
                    max = info_srm.demValues[i];
                }
            }
            
            for(size_t i=0; i<irows*icols; i++){
                srm_img[i] = (uint8_t) (255*(info_srm.demValues[i] - min)/(max-min));
            }
            
#ifdef DEBUG
            write_channel_seperated_image("srm.png", srm_img, icols, irows, 1);
#endif
        }
        
#ifdef DEBUG
        uint8_t *ele_img = malloc(icols*irows*sizeof(uint8_t));
        float max = 0;
        float min = FLT_MAX;
        for(size_t i=0; i<irows*icols; i++){
            if(info_ele.demValues[i] < min){
                min = info_ele.demValues[i];
            }
            
            if(info_ele.demValues[i]> max){
                max = info_ele.demValues[i];
            }
        }
        for(size_t i=0; i<irows*icols; i++){
            ele_img[i] = (uint8_t) (255*(info_ele.demValues[i] - min)/(max-min));
        }
        write_channel_seperated_image("ele.png", ele_img, icols, irows, 1);
#endif
        
        if (srm_img == NULL) {
            printf("Failure to load surface reflectance map from %.256s\n", srm_file_name);
            return EXIT_FAILURE;
        }
        
        ok = CreateLandmark(&info_ele, srm_img, icols, irows, anchor_latitude_degrees, anchor_longitude_degrees, info_ele.projection, &lmk, set_anchor_point_ele);
        write_channel_seperated_image("lmk.srm.png", lmk.srm, lmk.num_cols, lmk.num_rows, 1);
        
        if(srm_img) free(srm_img);
    }
    
    if (ok){
        ok = Write_LMK(lmk.filename, &lmk);
    }
    
    free_lmk(&lmk);
    free(info_ele.demValues);
    free(info_srm.demValues);
    
    if(ok){
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
    
}
