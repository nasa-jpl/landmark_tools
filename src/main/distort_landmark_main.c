/** 
 * \file distort_landmark_main.c
 * \author Cecilia Mauceri
 * \date 2024-03-24
 * 
 * \brief Simulate map error with simple distortions added to landmark file
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
#include <math.h>
#include <time.h>

#include "landmark_tools/landmark_util/landmark.h"  // for free_lmk, Crop_In...
#include "landmark_tools/math/math_constants.h"
#include "landmark_tools/utils/parse_args.h"        // for m_getarg, CFO_STRING
#include "math/mat3/mat3.h"

void  show_usage_and_exit()
{
    printf("Simulate map error\n");
    printf("Usage for distort_landmark:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -input   <filename> - input landmark filepath\n");
    printf("    -output  <filename> - output landmark filepath\n");
    printf("  Optional arguments:\n");
    printf("    -translate <x meters> <y meters> <z meters> - simulates map tie error \n");
    printf("    -rotate <in-plane rotation degrees> - simulates map orientation error\n");
    printf("    -random_displace <mean> <stddev> - simulates correlation noise with gaussian elevation displacement\n");
//    printf("    -sine_wave <amplitude> <frequency> <azimuth> - simulates image jitter with elevation displacement\n");
//    printf("    -cubic  <a> <b> <c> <d> - simulates camera model error with elevation displacement\n");
    
    exit(EXIT_FAILURE);
}

void rotateLandmark(LMK* lmk, float rot_z_degrees){
    
    float rot_z = rot_z_degrees*DEG2RAD;
    
//    float3 Rx[3][3] = {0}, Ry[3][3] = {0};
    float3 Rz[3][3]= {0};
    
//    Rx[0][0] = 1;
//    Rx[1][1] = cos(rot_x);
//    Rx[1][2] = -1*sin(rot_x);
//    Rx[2][1] = sin(rot_x);
//    Rx[2][2] = cos(rot_x);
//    
//    Ry[0][0] = cos(rot_y);
//    Ry[0][2] = sin(rot_y);
//    Ry[1][1] = 1;
//    Ry[2][0] = -1*sin(rot_y);
//    Ry[2][2] = cos(rot_y);
    
    Rz[0][0] = cos(rot_z);
    Rz[0][1] = -1*sin(rot_z);
    Rz[1][0] = sin(rot_z);
    Rz[1][1] = cos(rot_z);
    Rz[2][2] = 1;
    
//    float3 Rxy[3][3]={0};
//    mult333(Rx, Ry, Rxy);
//    float3 Rxyz[3][3]={0};
//    mult333(Rxy, Rz, Rxyz);
    
    float3 R[3][3]={0};
    mult333(Rz, lmk->mapRworld, R);
    copy33(R, lmk->mapRworld);
}

void translateLandmark(LMK* lmk, float translate_x, float translate_y, float translate_z){
    double translateMap[3] = {translate_x, translate_y, translate_z};
    double translateWorld[3];
    
    mult331(lmk->worldRmap, translateMap, translateWorld);
    
    lmk->anchor_point[0] += translateWorld[0];
    lmk->anchor_point[1] += translateWorld[1];
    lmk->anchor_point[2] += translateWorld[2];
}

//Based on Wikipedia C++ implementation
double sampleBoxMuller(double mean, double stddev) {
    static double x1=0, x2=0;
    static bool call = false;

    if (call) {
        call = !call;
        return x2;
    }

    double u1, u2;
    do {
        u1 = (double) rand() / RAND_MAX;
    } while (u1 == 0);
    u2 = (double) rand() / RAND_MAX;
    
    double two_pi = 2.0 * PI;
    double mag = stddev * sqrt(-2.0 * log(u1));
    x1  = mag * cos(two_pi * u2) + mean;
    x2  = mag * sin(two_pi * u2) + mean;

    call = !call;

    return x1;
}

//TODO do we want to displace the elevation or the 3d point?
void displaceElevationGaussian(LMK* lmk, float mean, float stddev){
    for(size_t i =0; i<lmk->num_pixels; i++){
        lmk->ele[i] += sampleBoxMuller(mean, stddev);
    }
}

bool displaceElevationCubic(LMK* lmk, float cubic_a, float cubic_b, float cubic_c, float cubic_d){
    return false;
}

void displaceElevationSine(LMK* lmk, float sine_amplitude, float sine_frequency, float sine_azimuth){
    float sine_azimuth_rad = sine_azimuth * PI / 180.0;
    for(size_t x =0; x<lmk->num_cols; x++){
        for(size_t y =0; y<lmk->num_rows; y++){
            lmk->ele[y * lmk->num_cols + x] += sine_amplitude
            * sin(2.0 * PI * sine_frequency * (float)x * cos(sine_azimuth_rad) + (float)y * cos(sine_azimuth_rad));
        }
    }
}

int32_t main (int32_t argc, char **argv)
{
    char *infile=NULL;
    char *outfile=NULL;
    
    float translate_x = 0;
    float translate_y = 0;
    float translate_z = 0;
//    float rot_x = 0;
//    float rot_y = 0;
    float rot_z = 0;
    float mean = 0;
    float stddev = -1;
    float cubic_a = 0;
    float cubic_b = 0;
    float cubic_c = 1;
    float cubic_d = 0;
    float sine_amplitude = 0;
    float sine_frequency = 0;
    float sine_azimuth = 0;
    
    argc--;
    argv++;
    
    if (argc==0) show_usage_and_exit();
    
    for (int32_t i = 1; i < argc; i+=2) {
        //Arguments with one value
        if ((m_getarg(argv, "-input", &infile,  CFO_STRING) == 1) ||
            (m_getarg(argv, "-output", &outfile,  CFO_STRING) == 1) ||
            (m_getarg(argv, "-rotate", &rot_z, CFO_FLOAT) == 1))
        {
            argv+=2;
        }else if (m_getarg(argv, "-translate", &translate_x, CFO_FLOAT) == 1){
            // translate has three values
            if( (sscanf(argv[2], "%f", (&translate_y)) == 1) &&
               (sscanf(argv[3], "%f", (&translate_z)) == 1)){
                argv +=4;
                i += 3;
            }else{
                printf("Error reading -translate value.\n");
                show_usage_and_exit();
            }
        }
        else if (m_getarg(argv, "-random_displace", &mean, CFO_FLOAT) == 1){
            // random_displace has two values
            if( (sscanf(argv[2], "%f", (&stddev)) == 1)){
                argv +=3;
                i += 2;
            }else{
                printf("Error reading -random_displace value.\n");
                show_usage_and_exit();
            }
        }else if (m_getarg(argv, "-sine_wave", &sine_amplitude, CFO_FLOAT) == 1){
            // sine_wave has three values
            if( (sscanf(argv[2], "%f", (&sine_frequency)) == 1) &&
               (sscanf(argv[3], "%f", (&sine_azimuth)) == 1)){
                argv +=4;
                i += 3;
            }else{
                printf("Error reading -sine_wave value.\n");
                show_usage_and_exit();
            }
        }else if (m_getarg(argv, "-cubic", &cubic_a, CFO_FLOAT) == 1){
            // cubic has four values
            if( (sscanf(argv[2], "%f", (&cubic_b)) == 1) &&
               (sscanf(argv[3], "%f", (&cubic_c)) == 1) &&
               (sscanf(argv[4], "%f", (&cubic_d)) == 1)){
                argv +=5;
                i += 4;
            }else{
                printf("Error reading -cubic value.\n");
                show_usage_and_exit();
            }
        }else{
            //Undefined argument
            show_usage_and_exit();
        }
    }
    
    // Required arguments
    if(infile==NULL | outfile==NULL ){
        show_usage_and_exit();
    }
    
    LMK lmk = {0};
    if(!Read_LMK(infile, &lmk)){
        printf("Failed to read landmark file: %.256s\n", infile);
        free_lmk(&lmk);
        return EXIT_FAILURE;
    }
    
    if(rot_z != 0){
        printf("Rotating landmark in plane by %f degrees...", rot_z);
        rotateLandmark(&lmk, rot_z);
        printf("done.\n");
    }
    
    if(translate_x != 0 || translate_y != 0 || translate_z != 0){
        printf("Translating landmark by (%f, %f, %f)...", translate_x, translate_y, translate_z);
        translateLandmark(&lmk, translate_x, translate_y, translate_z);
        printf("done.\n");
    }
    
    if(stddev != -1){
        printf("Applying random displacement to landmark with mu=%f, sigma=%f ...", mean, stddev);
        srand(time(NULL)); //seed random number generator
        displaceElevationGaussian(&lmk, mean, stddev);
        printf("done.\n");
    }
    
    if(cubic_a !=0 || cubic_b != 0 || cubic_c != 1 || cubic_d != 0 ){
        printf("Applying cubic displacement to landmark: f(z) = %fz^3 + %fz^2 + %fz + %f ...", cubic_a, cubic_b, cubic_c, cubic_d);
        if(!displaceElevationCubic(&lmk, cubic_a, cubic_b, cubic_c, cubic_d)){
            printf("Failed to displace landmark\n");
            free_lmk(&lmk);
            return EXIT_FAILURE;
        }
        printf("done.\n");
    }
    
    if(sine_amplitude !=0 || sine_frequency != 0 || sine_azimuth != 0 ){
        printf("Applying sine displacement to landmark: z(x,y) = %fsin(2PI*%fx*cos(%f) +y*cos(%f))...", sine_amplitude, sine_frequency, sine_azimuth, sine_azimuth);
        displaceElevationSine(&lmk, sine_amplitude, sine_frequency, sine_azimuth);
        printf("done.\n");
    }
    
    bool success = Write_LMK(outfile, &lmk);
    
    free_lmk(&lmk);
    
    if(success){
        printf("Landmark file written to: %.256s\n", outfile);
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}
