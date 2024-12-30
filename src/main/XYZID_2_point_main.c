/**
 * \file `XYZID_2_point_main.c`
 * \author Cecilia Mauceri
 * \date 2024
 *
 * \brief Make a PLY point cloud from file where file format is binary double precision, 4 columns: X,Y,Z,ID where X,Y are the stereographic coordinates (in meters), Z is the surface height (in meters) and ID is the LOLA RDR ID.
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
#include "landmark_tools/map_projection/stereographic_projection.h"

int is_little_endian() {
    uint16_t x = 1;
    return *((uint8_t *)&x);
}

#if defined(MAC_OS)

#include <arpa/inet.h>   // for htonl, ntohl, htons, ntohs

#include <libkern/OSByteOrder.h>
#define letohll(x) OSSwapLittleToHostInt64(x)

#endif

#if defined(LINUX_OS)

#include <endian.h>
#define letohll(x) le64toh(x)

#endif

#ifdef WINDOWS_OS

#include <winsock2.h>
#define letohll(x) (is_little_endian() ? (x) : _byteswap_uint64(x))

#endif

void show_usage_and_exit()
{
    printf("Make a PLY point cloud from file. \n");
    printf("Input file format is binary double precision, 4 columns: X,Y,Z,ID where X,Y are the stereographic coordinates (in meters), Z is the surface height (in meters) and ID is the LOLA RDR ID.\n");
    printf("Usage for XYZID_2_point:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -input   <filename> - input point file.\n");
    printf("    -output   <filename> - output ply point file \n");
    printf("    -filetype <PLY_ASCII|PLY_LITTLE_ENDIAN|PLY_BIG_ENDIAN> \n");
    printf("    -projection_latitude <double> - Latitude of natural origin\n");
    printf("    -projection_longitude <double> - Reference_Meridian\n");
    printf("    -planet <Moon|Earth|Mars> \n");
    exit(EXIT_FAILURE);
}


int32_t main(int32_t argc, char **argv)
{
    char *infile= NULL;
    char *outfile = NULL;
    char *filetype_str = NULL;
    double lat0;
    double long0;
    char * planet_str = NULL;

    argc--;
    argv++;
    
    if (argc==0) show_usage_and_exit();
    
    while (argc>0)
    {
        if (argc==1) show_usage_and_exit();
        //TODO make planet and filetype optional
        if ((m_getarg(argv, "-input",    &infile,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-output",   &outfile,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-filetype",   &filetype_str,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-projection_latitude", &lat0, CFO_DOUBLE) != 1) &&
            (m_getarg(argv, "-projection_longitude", &long0, CFO_DOUBLE) != 1) &&
            (m_getarg(argv, "-planet", &planet_str, CFO_STRING) != 1))
            show_usage_and_exit();
        
        argc-=2;
        argv+=2;
    }
    
    if(filetype_str == NULL || infile == NULL || outfile == NULL || planet_str == NULL){
        show_usage_and_exit();
    }
    enum e_ply_storage_mode_ filetype = strToPlyFileType(filetype_str);
    
    enum Planet planet = strToPlanet(planet_str);
    if(planet == Planet_UNDEFINED){
        show_usage_and_exit();
    }
    
    // Read Point Cloud
    bool read_success = false;
    
    double *pts_stereographic;
    size_t num_pts;
    
    if(filetype == PLY_ASCII){
        read_success =  readinpoints_xyzid_ascci(infile, &pts_stereographic, &num_pts);
    }else if(filetype == PLY_BIG_ENDIAN || filetype == PLY_LITTLE_ENDIAN){
        FILE *fp = fopen(infile, 'rb');
        if(fp == NULL)
        {
            printf("Cannot open file %.256s to read\n", infile);
            return false;
        }
        
        // Count points
        uint64_t raw;
        while (fread(&raw, sizeof(uint64_t), 1, fp) == 1) {
            num_pts ++;
        }
        num_pts = num_pts/4;
        pts_stereographic = malloc(num_pts * 3 * sizeof(double));
        if(pts_stereographic == NULL){
            printf("Failure to allocate memory\n");
            return EXIT_FAILURE;
        }
        
        rewind(fp);
        
        // Read points
        size_t i = 0;
        size_t point_index = 0;
        while (fread(&raw, sizeof(uint64_t), 1, fp) == 1) {
            if(point_index<num_pts){
                printf("To many points %ld > %ld\n", point_index, num_pts);
                if(pts_stereographic!=NULL) free(pts_stereographic);
                return EXIT_FAILURE;
            }
            
            if(i%4!=3){
                
                if(filetype == PLY_BIG_ENDIAN){
                    raw = ntohl(raw);  // Convert network to host byte order
                }else if(filetype == PLY_LITTLE_ENDIAN){
                    raw = letohll(raw); // Convert little endian to host byte order
                }
                memcpy(&(pts_stereographic[point_index]), &raw, sizeof(double));
                point_index ++;
            }
            i++;
        }
    }
    
    if(!read_success){
        printf("Unable to read %.256s\n", infile);
        if(pts_stereographic!=NULL) free(pts_stereographic);
        return EXIT_FAILURE;
    }
    
    // Convert from stereographic to ECEF coordinates
    double* pts_ecef = malloc(num_pts * 3 * sizeof(double));
    if(pts_ecef == NULL){
        printf("Failure to allocate memory\n");
        if(pts_stereographic!=NULL) free(pts_stereographic);
        return EXIT_FAILURE;
    }
    
    for(size_t i = 0; i<num_pts; i++){
        double point_latitude;
        double point_longitude;
        StereographicProjection2LatLong( pts_stereographic[i*3], pts_stereographic[i*3+1], lat0, long0, ellipsoids[planet].a, &point_latitude, &point_longitude );
        LatLongHeight_to_ECEF(point_latitude, point_longitude, pts_stereographic[i*3+2], &(pts_ecef[i*3]), planet);
    }
    
    
    // Write point cloud
    if(!Write_PLY_Points(outfile, pts_ecef, num_pts, filetype)){
        printf("Unable to write %.256s\n", outfile);
        if(pts_stereographic!=NULL) free(pts_stereographic);
        if(pts_ecef!=NULL) free(pts_ecef);
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}
