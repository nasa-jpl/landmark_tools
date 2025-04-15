/** 
 * \file create_landmark_main.c
 * \author Yang Cheng
 * 
 * \brief Create landmark file from DEM
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

#include "landmark_tools/image_io/geotiff_struct.h"

#ifdef USE_GEOTIFF
#include "landmark_tools/image_io/geotiff_interface.h"       // for st_geoti...
#endif //USE_GEOTIFF

static void show_usage(void)
{
    printf("Usage for create_landmark:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -projection <EQ_CYLINDERICAL or UTM or STEREO or GEOGRAPHIC>\n");
    printf("    -config_file <filename> - configuration file\n");
    printf("  OR\n");
    printf("    -geotif_file   <filename> - input dem tif file name\n");
    printf("    -lmk_file   <filename> - output lmk file name\n");
    printf("    -lmk_width_meters   <filename> - lmk col size\n");
    printf("    -lmk_height_meters   <filename> - lmk row size\n");
    printf("    -lmk_res   <filename> - lmk resolution\n");
    printf("    -lmk_center_lat   <filename> - lmk center lat\n");
    printf("    -lmk_center_long   <filename> - lmk center long\n");
    printf("  Optional arguments:\n");
    printf("    -planet <Moon or Earth> - (default Moon)\n");
    printf("    -nodata_value <int> - (default NaN)\n");
    printf("    -srm_file <filename> - png image file containing surface reflectance map\n");
    printf("    -set_anchor_point_ele <float> - (default NAN, use ele based on a point at anchor lat long)\n");
}

/////////////////////////////////////////////////////////////////////////
int32_t main(int32_t argc, char** argv)
{
    float lmkcols=0.0, lmkrows = 0.0;
    float lmkres = 0.0;
    char* projection_type=NULL;
    char* config_file_name=NULL;
    char* input_geotif_file_name=NULL;
    char* output_lmk_file_name=NULL;
    char* planet_str=NULL;
    char* srm_file_name=NULL;
    double lat0 = 0.0, long0 = 0.0;
    double nodata_value = NAN;
    float set_anchor_point_ele = NAN;

    
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
            (m_getarg(argv, "-geotif_file", &input_geotif_file_name,  CFO_STRING) == 1) ||
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
            m_getarg(argv, "-projection", &projection_type,  CFO_STRING);
            m_getarg(argv, "-config_file", &config_file_name, CFO_STRING);
            m_getarg(argv, "-planet", &planet_str, CFO_STRING);
            m_getarg(argv, "-nodata_value", &nodata_value, CFO_DOUBLE);
            m_getarg(argv, "-srm_file", &srm_file_name, CFO_STRING);
            m_getarg(argv, "-set_anchor_point_ele", &set_anchor_point_ele, CFO_FLOAT); // if not specified, use ele based on a point at anchor point
        }
        argv+=2;
    }
    
    LMK lmk = {0};
    GeoTiffData geotiff_info = {0};
    
    enum Planet planet = strToPlanet(planet_str);
    if(planet == Planet_UNDEFINED){
        show_usage();
        return EXIT_FAILURE;
    }

    double anchor_longitude_degrees = long0;
    double anchor_latitude_degrees = lat0;

    if(config_file_name !=NULL && projection_type !=NULL){
        
        geotiff_info.projection = strToProjection(projection_type);
        if(geotiff_info.projection == Projection_UNDEFINED){
            show_usage();
            return EXIT_FAILURE;
        }
        
        geotiff_info.noDataValue = nodata_value;
        
        //Use config file
        size_t demname_size = 256;
        char demname[demname_size];
        memset(&demname, 0, demname_size);
        
        bool success = readCreateLandmarkConfiguration(config_file_name, geotiff_info.projection, planet, &lmk, &geotiff_info,
            demname, demname_size, &anchor_latitude_degrees, &anchor_longitude_degrees);
        if(!success){
            printf("Failed to read configuration file:%.256s\n", config_file_name);
            return EXIT_FAILURE;
        }
        
        // Load DEM and srm image
        geotiff_info.demValues = (float *)malloc(sizeof(float)* geotiff_info.imageSize[0]*geotiff_info.imageSize[1]);
        if(geotiff_info.demValues == NULL){
            free_lmk(&lmk);
            printf("Failed to allocate memory for DEM: %.256s\n", demname);
            return EXIT_FAILURE;
        }
        
        FILE* fp = fopen(demname, "rb");
        if(fp == NULL)
        {
           free_lmk(&lmk);
           printf("Failed to open DEM file: %.256s\n", demname);
           return EXIT_FAILURE;
        }
        
//        if(geotiff_info.bits_per_sample == 16)
//        {
//           int16_t* dem = (int16_t *)malloc(sizeof(int16_t)*geotiff_info.imageSize[0]*geotiff_info.imageSize[1]);
//           for(int32_t i = 0; i < geotiff_info.imageSize[1]; ++i)
//           {
//               fread(&dem[i*geotiff_info.imageSize[0]], sizeof(int16_t), geotiff_info.imageSize[0], fp);
//           }
//           for(int32_t i = 0; i < geotiff_info.imageSize[0]*geotiff_info.imageSize[1]; ++i)
//           {
//               fdem[i] = dem[i];
//           }
//           free(dem);
//        }
        if(geotiff_info.bits_per_sample == 32)
        {
           for(size_t i = 0; i < geotiff_info.imageSize[1]; ++i)
           {
               fread(&(geotiff_info.demValues)[i*geotiff_info.imageSize[0]], sizeof(float), geotiff_info.imageSize[0], fp);
           }
        }else{
            fprintf(stderr, "Bit depth not supported: %d\n", geotiff_info.bits_per_sample);
            fclose(fp);
            return EXIT_FAILURE;
        }
        fclose(fp);
        
        if(!isnan(geotiff_info.noDataValue)){
            for(int32_t i = 0; i < geotiff_info.imageSize[0]*geotiff_info.imageSize[1]; ++i)
            {
                if(geotiff_info.demValues[i]==geotiff_info.noDataValue){
                    geotiff_info.demValues[i] = NAN;
                }
            }
        }
    } else if (args_found<num_required_args){
        show_usage();
            printf("create_landmark_main.main() ==>> failed, missing parameter(s)\n");
            return EXIT_FAILURE;
    } else {
        #ifdef USE_GEOTIFF
        
        // Use the arguments and geotiff metadata
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
        
        //TODO lmk_id described in the D_101723_LVS_Ref_Map_Product_ICD document
        strncpy(lmk.lmk_id, "0", LMK_ID_SIZE);
        
        if(!allocate_lmk_arrays(&lmk, lmk.num_cols, lmk.num_rows)){
            free_lmk(&lmk);
            return EXIT_FAILURE;
        }
            
        bool ok = readGeoTiff(input_geotif_file_name, &geotiff_info);
        if (!ok)
        {
            free_lmk(&lmk);
            printf("main() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
            return EXIT_FAILURE;
        }
        
#ifdef DEBUG
        uint8_t *ele_img = malloc(geotiff_info.imageSize[0]*geotiff_info.imageSize[1]*sizeof(uint8_t));
        float max = 0;
        float min = FLT_MAX;
        for(size_t i=0; i<geotiff_info.imageSize[0]*geotiff_info.imageSize[1]; i++){
            if(geotiff_info.demValues[i] < min){
                min = geotiff_info.demValues[i];
            }
            
            if(geotiff_info.demValues[i]> max){
                max = geotiff_info.demValues[i];
            }
        }
        for(size_t i=0; i<geotiff_info.imageSize[0]*geotiff_info.imageSize[1]; i++){
            ele_img[i] = (uint8_t) (255*(geotiff_info.demValues[i] - min)/(max-min));
        }
        write_channel_separated_image("ele.png", ele_img, geotiff_info.imageSize[0], geotiff_info.imageSize[1], 1);
#endif
        
        #else
        printf("create_landmark not built with GeoTiff support. Please use config file and binary DEM option");
        return EXIT_FAILURE;
        #endif //USE_GEOTIFF
    }

    bool ok = false;
    if(srm_file_name == NULL){
        printf("Creating landmark with empty surface reflectance map.\n");
        ok = CreateLandmark_dem_only(&geotiff_info, anchor_latitude_degrees, anchor_longitude_degrees, geotiff_info.projection, &lmk, set_anchor_point_ele);
    }else{
        //Load the surface reflectance map
        int32_t icols, irows;
        uint8_t *srm_img = load_channel_separated_image(srm_file_name, &icols, &irows);
        
        if (srm_img == NULL) {
            printf("Failure to load surface reflectance map from %.256s\n", srm_file_name);
            return EXIT_FAILURE;
        }
        
        ok = CreateLandmark(&geotiff_info, srm_img, icols, irows, anchor_latitude_degrees, anchor_longitude_degrees, geotiff_info.projection, &lmk, set_anchor_point_ele);
        if(srm_img) free(srm_img);
    }
        
    if (ok){
        ok = Write_LMK(lmk.filename, &lmk);
    }

    free_lmk(&lmk);
    free(geotiff_info.demValues);
    
    if(ok){
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}
