/** 
 * \file landmark_comparison_main.c
 * \author Yang Cheng
 * 
 * \brief Compare two landmark files using dense patch-based correlation matching
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
#include <assert.h>                                         // for assert
#include <math.h>                                           // for fabs, sqrt
#include <stdbool.h>                                        // for false, bool
#include <stdint.h>                                         // for int32_t
#include <stdio.h>                                          // for printf, NULL
#include <stdlib.h>                                         // for free, malloc

#include "landmark_tools/image_io/image_utils.h"             // for load_cha...
#include "landmark_tools/feature_tracking/feature_match.h"  // for MatchFeat...
#include "landmark_tools/feature_tracking/parameters.h"            // for Parameters, Read...
#include "landmark_tools/math/homography_util.h"
#include "landmark_tools/landmark_util/landmark.h"          // for free_lmk
#include "landmark_tools/landmark_util/estimate_homography.h"
#include "landmark_tools/utils/parse_args.h"                // for m_getarg
#include "math/mat3/mat3.h"                                 // for mult331

/*-----------------------------------------------------------*/
/*----------------------- Variables -------------------------*/
/*-----------------------------------------------------------*/

#define BLOCK_SIZE 200
#define STEP_SIZE 4
#define MIN_N_FEATURES 20
// TODO Hrand's code uses a FEATURE_WINDOW of 5
#define FEATURE_WINDOW 7
#define REPROJECTION_THRESHOLD 5.0

typedef struct {
    float* deltax;
    float* deltay;
    float* deltaz;
    float* corr_m;
} CORR_STRUCT;

uint8_t allocate_corr(CORR_STRUCT* corr_struct, size_t num_pixels);
void destroy_corr(CORR_STRUCT* corr_struct);

uint8_t allocate_corr(CORR_STRUCT* corr_struct, size_t num_pixels){
    corr_struct->deltax = (float *)malloc(sizeof(float)*num_pixels);
    corr_struct->deltay = (float *)malloc(sizeof(float)*num_pixels);
    corr_struct->deltaz = (float *)malloc(sizeof(float)*num_pixels);
    corr_struct->corr_m = (float *)malloc(sizeof(float)*num_pixels);
    
    if(corr_struct->deltax == NULL || corr_struct->deltay == NULL ||
       corr_struct->deltaz == NULL || corr_struct->corr_m == NULL){
        destroy_corr(corr_struct);
        return false;
    }
    
    for(size_t i = 0; i < num_pixels; ++i)
    {
        corr_struct->deltax[i] = 0;
        corr_struct->deltay[i] = 0;
        corr_struct->deltaz[i] = 0;
        corr_struct->corr_m[i] = 0;
    }

    return true;
}

void destroy_corr(CORR_STRUCT* corr_struct) {
    if(corr_struct->deltax != NULL){
        free(corr_struct->deltax);
    }
    if(corr_struct->deltay != NULL){
        free(corr_struct->deltay);
    }
    if(corr_struct->deltaz != NULL){
        free(corr_struct->deltaz);
    }
    if(corr_struct->corr_m != NULL){
        free(corr_struct->corr_m);
    }
}


void SimpleScaleNormalizeUInt8Image(unsigned char *img, unsigned char *outimg, int cols, int rows, int win_size)
{
  double *vr, *mean, *sqimg;
  int i, j;
  int m, n;
  int h_win;
  int top, bottom, left, right;
  double *vrpt, *mpt, *sqpt;
  int imagesize;
  int d;
  unsigned char *pt, *opt;
  double win_size2;
  double maxv, minv, dstep;;
  win_size2 = win_size*win_size;
  imagesize = cols*rows;
  vr = (double *)malloc(imagesize*sizeof(double));
  mean = (double *)malloc(imagesize*sizeof(double));
  sqimg = (double *)malloc(imagesize*sizeof(double));
  h_win = (win_size-1)/2;
  //clrad=(clsz-1)/2;
  top = h_win;
  left = h_win;
  bottom = rows- h_win;
  right = cols - h_win;
  for (i=0; i<imagesize; i++)
  {
    mean[i]=vr[i]=0;
      //outimg[i] = 0;
  }
  for(i = top; i < bottom; ++i)
  {
      pt = &img[i*cols + left];
      for(j = left; j < right; ++j)
      {
         for(m = i - h_win; m <= i + h_win; ++m)
         {
             mpt = &mean[m*cols + j-h_win];
             for(n = j- h_win; n <= j + h_win; ++n)
             {
                 *mpt++ += *pt;
             }
         }
         pt++;
      }
  }
  
  
  
    //win_size2 *=4000;
    win_size2 *= 255;
    mpt = mean;
    pt =  img ;
    opt = outimg;
    for(i = top; i < bottom; ++i)
    {
        pt = &img[i*cols + left];
        mpt = &mean[i*cols + left];
        opt = &outimg[i*cols + left];
        for(j = left; j < right; ++j)
        {
            if(*mpt > 0)
            {
                d = win_size2 * *pt / *mpt ;
                //d = *pt / *mpt * win_size2 * 128 + 128;
                if(d < 256)
                {
                    *opt = d;
                }
                else
                {
                    *opt =*pt ;
                }
            }
            else
            {
                *opt =*pt ;
            }
            opt++;
            mpt++;
            pt++;
        }
    }
  free(vr);
  free(mean);
  free(sqimg);
}

/*-----------------------------------------------------------*/
/*----------------------- Functions -------------------------*/
/*-----------------------------------------------------------*/

bool MatchFeatures_local_distortion(
    Parameters parameters,
    LMK *lmk_base,
    LMK *lmk_child,
    CORR_STRUCT *corr_struct,
    int32_t nan_max_count_base,
    int32_t nan_max_count_child
);

void show_usage_and_exit()
{
    printf("Compare landmark files using a dense patch-based correlation matcher\n");
    printf("Usage for landmark_compare:\n");
    printf("------------------\n");
    printf("  Required arguments:\n");
    printf("    -l1   <lmk_filepath> \n");
    printf("    -l2   <lmk_filepath> \n");
    printf("    -o    <output_prefix> \n");
    printf("    -c    <parameters_config_filepath> \n");
    printf("    -nan_max_count1     <-1 to ignore, 0 or greater to filter> \n");
    printf("    -nan_max_count2     <-1 to ignore, 0 or greater to filter> \n");
    exit(EXIT_FAILURE);
}


int32_t main (int32_t argc, char **argv)
{
    char *lmkfile1 = NULL;
    char *lmkfile2 = NULL;
    char *outfile = NULL;
    char *parametersfile = NULL;
    char *nan_max_count1_str = NULL;
    char *nan_max_count2_str = NULL;
    argc--;
    argv++;
    
    if (argc==0) show_usage_and_exit( );
    
    while (argc>0)
    {
        if (argc==1) show_usage_and_exit( );
        if ((m_getarg(argv, "-l1",    &lmkfile1,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-l2",   &lmkfile2,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-o",   &outfile,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-c",   &parametersfile,        CFO_STRING)!=1) &&
            (m_getarg(argv, "-nan_max_count1", &nan_max_count1_str, CFO_STRING)!=1) &&
            (m_getarg(argv, "-nan_max_count2", &nan_max_count2_str, CFO_STRING)!=1))
            show_usage_and_exit( );
        
        argc-=2;
        argv+=2;
    }
    
    Parameters parameters;
    load_default_parameters (&parameters);
    if(parametersfile==NULL){
        printf("No parameter file provided. Using defaults.\n");
    }else{
        int32_t success = read_parameterfile(parametersfile, &parameters);
        if(!success){
            printf("Cannot load %.256s\n", parametersfile);
            return EXIT_FAILURE;
        }
    }
    print_parameters(parameters);
    
    LMK lmk_child = {0};
    LMK lmk_base = {0};
    int success = Read_LMK(lmkfile1, &lmk_child);
    success &= Read_LMK(lmkfile2, &lmk_base);
    if(!success){
        free_lmk(&lmk_child);
        free_lmk(&lmk_base);
        return EXIT_FAILURE;
    }

    int32_t nan_max_count_child = -1; // default behavior: do not check for nan in child lmk
    int32_t nan_max_count_base = 0; // default behavior: do not allow any nan in base lmk
    if (nan_max_count1_str != NULL)
    {
        nan_max_count_child = atoi(nan_max_count1_str);
    }
    if (nan_max_count2_str != NULL)
    {
        nan_max_count_base = atoi(nan_max_count2_str);
    }
    
#ifdef DEBUG
    write_channel_separated_image("basemap.png", lmk_base.srm, lmk_base.num_cols, lmk_base.num_rows, 1);
    write_channel_separated_image("childmap.png", lmk_child.srm, lmk_child.num_cols, lmk_child.num_rows, 1);
#endif

    write_channel_separated_image("lmk_base.srm.png", lmk_base.srm, lmk_base.num_cols, lmk_base.num_rows, 1);
    write_channel_separated_image("lmk_child.srm.png", lmk_child.srm, lmk_child.num_cols, lmk_child.num_rows, 1);

    CORR_STRUCT corr_struct;
    allocate_corr(&corr_struct, lmk_child.num_pixels);
    success &= MatchFeatures_local_distortion(
        parameters,
        &lmk_child,
        &lmk_base,
        &corr_struct,
        nan_max_count_child,
        nan_max_count_base
    );
    if(!success){
        printf("Failed to match features. Exiting without output.\n");
        free_lmk(&lmk_child);
        free_lmk(&lmk_base);
        destroy_corr(&corr_struct);
        return EXIT_FAILURE;
    }
    
    printf("Saving results to %.256s\n", outfile);
    
    FILE *fp;
    size_t buf_size = 256;
    char buf[buf_size];
    snprintf(buf, buf_size, "%.256s_delta_x_%dby%d.raw", outfile, lmk_child.num_cols, lmk_child.num_rows);
    fp = fopen(buf, "wb");
    fwrite(corr_struct.deltax, sizeof(float), lmk_child.num_pixels, fp);
    fclose(fp);
    snprintf(buf, buf_size, "%.256s_delta_y_%dby%d.raw", outfile, lmk_child.num_cols, lmk_child.num_rows);
    fp = fopen(buf, "wb");
    fwrite(corr_struct.deltay, sizeof(float), lmk_child.num_pixels, fp);
    fclose(fp);
    
    snprintf(buf, buf_size, "%.256s_delta_z_%dby%d.raw", outfile, lmk_child.num_cols, lmk_child.num_rows);
    fp = fopen(buf, "wb");
    fwrite(corr_struct.deltaz, sizeof(float), lmk_child.num_pixels, fp);
    fclose(fp);
    
    snprintf(buf, buf_size, "%.256s_corr_%dby%d.raw", outfile, lmk_child.num_cols, lmk_child.num_rows);
    fp = fopen(buf, "wb");
    fwrite(corr_struct.corr_m, sizeof(float), lmk_child.num_pixels, fp);
    fclose(fp);
    
    free_lmk(&lmk_child);
    free_lmk(&lmk_base);
    destroy_corr(&corr_struct);
    if(success){
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}

bool MatchFeatures_local_distortion(
    Parameters parameters,
    LMK *lmk_base,
    LMK *lmk_child,
    CORR_STRUCT* corr_struct,
    int32_t nan_max_count_base,
    int32_t nan_max_count_child
){
    double base2child[3][3];
    estimateHomographyUsingCorners(lmk_base, lmk_child, base2child);
    
    float *wt = (float *)malloc(sizeof(float)*lmk_child->num_pixels);
    uint8_t* child_nan_mask = (uint8_t*)malloc(sizeof(uint8_t*)*lmk_child->num_pixels);
    uint8_t* base_nan_mask = (uint8_t*)malloc(sizeof(uint8_t*)*lmk_base->num_pixels);
    if(wt == NULL || child_nan_mask == NULL || base_nan_mask == NULL){
        if(child_nan_mask)
            free(child_nan_mask);
        if(base_nan_mask)
            free(base_nan_mask);
        if(wt)
            free(wt);
        printf("MatchFeatures_local_distortion(): memory allocation error\n");
        return false;
    }
        
    for(int32_t i = 0; i<lmk_child->num_pixels; i++){
        wt[i] = NAN;
        
        if(isnan(lmk_child->ele[i])){
            child_nan_mask[i] = 1;
        }else{
            child_nan_mask[i] = 0;
        }
        
        if(isnan(lmk_base->ele[i])){
            base_nan_mask[i] = 1;
        }else{
            base_nan_mask[i] = 0;
        }
    }
    
    //Sliding window with step size of BLOCK_SIZE
    for(int32_t row_index = 0; row_index < lmk_child->num_rows; row_index +=BLOCK_SIZE)
    {
        printf("line = %d\n", row_index);
        for(int32_t col_index = 0; col_index < lmk_child->num_cols ; col_index+=BLOCK_SIZE)
        {
            //Copy the row, col coordinates of the patch into pts1
            //Patch is downsampled by STEP_SIZE
            int32_t size = ((BLOCK_SIZE/STEP_SIZE)+1)*((BLOCK_SIZE/STEP_SIZE)+1);
            double *pts_child_patch = (double *)malloc(sizeof(double)*size*2);
            double *pts_base_patch = (double *)malloc(sizeof(double)*size*2);
            
            if(pts_child_patch == NULL || pts_base_patch == NULL){
                if(pts_child_patch != NULL) free(pts_child_patch);
                if(wt != NULL) free(wt);
                printf("MatchFeatures_local_distortion(): memory allocation error\n");
                return false;
            }
            
            int32_t pts_in_block = 0;
            for(int32_t m = row_index; m <= row_index+BLOCK_SIZE; m+=STEP_SIZE)
            {
                for(int32_t n = col_index; n <= col_index+BLOCK_SIZE; n+=STEP_SIZE)
                {
                    
                    pts_child_patch[pts_in_block*2] = n;
                    pts_child_patch[pts_in_block*2+1] = m;
                    pts_in_block++;
                    
                }
            }
            
            assert(pts_in_block <= size);
            
            // If the patch has at least 40 valid coordinates, find matching features in base map
            // TODO why 40?
            if(pts_in_block > 40)
            {
                double covs[pts_in_block];
                int32_t num_matched_features = MatchFeaturesOnlyExtended(
                    parameters,
                    lmk_child->srm,
                    child_nan_mask,
                    lmk_child->num_cols,
                    lmk_child->num_rows,
                    nan_max_count_child,
                    lmk_base->srm,
                    base_nan_mask,
                    lmk_base->num_cols,
                    lmk_base->num_rows,
                    nan_max_count_base,
                    base2child,
                    pts_child_patch,
                    pts_base_patch,
                    covs,
                    pts_in_block
                );
                
                printf("num_matched_features %d\n", num_matched_features);
                
                if(num_matched_features > MIN_N_FEATURES)
                {
                    printf("i = %d j = %d\n", row_index, col_index);
                    double patch_homo[3][3];
                    getHomographyFromPoints_RANSAC_frame(pts_child_patch, pts_base_patch, num_matched_features, patch_homo, 3);
                    
                    for(int32_t feature_index = 0; feature_index < num_matched_features; ++feature_index)
                    {
                        double reprojection_err[2];
                        homographyTransfer33(patch_homo, (int32_t)pts_child_patch[feature_index * 2], (int32_t)pts_child_patch[feature_index * 2 + 1], reprojection_err);
                        reprojection_err[0] -= pts_base_patch[feature_index * 2];
                        reprojection_err[1] -= pts_base_patch[feature_index * 2 + 1];
                        double mag_reprojection = sqrt(reprojection_err[0] * reprojection_err[0] + reprojection_err[1] * reprojection_err[1]);
                        
                        //If the feature is a homography inlier, calculate the delta between the feature points
                        if (mag_reprojection < REPROJECTION_THRESHOLD)
                        {
                            double p_child[3], p_base[3], p_delta_map[3], p_delta_world[3];
                            LMK_Col_Row2World(lmk_child,  pts_child_patch[feature_index*2], pts_child_patch[feature_index*2+1], p_child);
                            LMK_Col_Row2World(lmk_base,   pts_base_patch[feature_index*2], pts_base_patch[feature_index*2+1], p_base);
                            
                            int32_t m = (int32_t) pts_child_patch[feature_index * 2 + 1];
                            int32_t n = (int32_t) pts_child_patch[feature_index * 2];
                            sub3(p_child, p_base, p_delta_world);
                            
                            // Rotate p_delta so that it is in local reference frame
                            mult331(lmk_child->mapRworld, p_delta_world, p_delta_map);
                            
                            //Fill the delta and corr maps for around the feature pt using a distance weighted contribution from the feature
                            for(int32_t mm = m - FEATURE_WINDOW; mm  <= m+FEATURE_WINDOW; ++mm)
                            {
                                for(int32_t nn = n - FEATURE_WINDOW; nn <= n + FEATURE_WINDOW; ++nn)
                                {
                                    double s = sqrt(((double)mm - pts_child_patch[feature_index * 2 + 1])*((double)mm - pts_child_patch[feature_index * 2 + 1]) + ((double)nn - pts_child_patch[feature_index * 2])*((double)nn - pts_child_patch[feature_index * 2]));
                                    float w = (float)exp(-s);
                                    if (mm > 0 && mm < lmk_child->num_rows && nn > 0 && nn < lmk_child->num_cols)
                                    {
                                        corr_struct->deltax[mm*lmk_child->num_cols + nn] += (float)(p_delta_map[0] * (double)w);
                                        corr_struct->deltay[mm*lmk_child->num_cols + nn] += (float)(p_delta_map[1] * (double)w);
                                        corr_struct->deltaz[mm*lmk_child->num_cols + nn] += (float)(p_delta_map[2] * (double)w);
                                        corr_struct->corr_m[mm*lmk_child->num_cols + nn] += (float)(covs[feature_index] * (double)w);
                                        if(isnan(wt[mm*lmk_child->num_cols + nn]))
                                            wt[mm*lmk_child->num_cols + nn] = w;
                                        else
                                            wt[mm*lmk_child->num_cols + nn] += w;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            
            if(pts_child_patch != NULL) free(pts_child_patch);
            if(pts_base_patch != NULL) free(pts_base_patch);
        }
    }
    
    for (int32_t i = 0; i < lmk_child->num_pixels; ++i)
    {
        if (!isnan(wt[i]))
        {
            corr_struct->deltay[i] = corr_struct->deltay[i] / wt[i];
            corr_struct->deltax[i] = corr_struct->deltax[i] / wt[i];
            corr_struct->deltaz[i] = corr_struct->deltaz[i] / wt[i];
            corr_struct->corr_m[i] = corr_struct->corr_m[i] / wt[i];
        }else{
            corr_struct->deltay[i] = NAN;
            corr_struct->deltax[i] = NAN;
            corr_struct->deltaz[i] = NAN;
            corr_struct->corr_m[i] = NAN;
        }
    }

    //TODO this looks like an outlier filter. How was 300 choosen?
    for (int32_t i = 0; i < lmk_child->num_pixels; ++i)
    {
        
        if (fabs(corr_struct->deltay[i]) > 500)  corr_struct->deltay[i] = NAN;
        if (fabs(corr_struct->deltax[i]) > 500)  corr_struct->deltax[i] = NAN;
        if (fabs(corr_struct->deltaz[i]) > 500)  corr_struct->deltaz[i] = NAN;
        
    }
    
    free(wt);
    if(child_nan_mask)
        free(child_nan_mask);
    if(base_nan_mask)
        free(base_nan_mask);
    return true;
}
