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

/*-----------------------------------------------------------*/
/*----------------------- Variables -------------------------*/
/*-----------------------------------------------------------*/

#define REPROJECTION_THRESHOLD 7
#define DEBUG

/*-----------------------------------------------------------*/
/*----------------------- Functions -------------------------*/
/*-----------------------------------------------------------*/
int32_t MatchFeatures_register_two_landmarks(Parameters parameters, const char *lmkname_base, const char *lmkname_child);

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

int32_t main (int32_t argc, char **argv)
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
    
    if(MatchFeatures_register_two_landmarks( parameters,baselmkfile, childlmkfile)){
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}

int32_t MatchFeatures_register_two_landmarks(Parameters parameters, const char *lmkname_base, const char *lmkname_child)
{
    LMK lmk_child = {0};
    LMK lmk_base = {0};
    Read_LMK(lmkname_child, &lmk_child);
    Read_LMK(lmkname_base, &lmk_base);
    
    int32_t template_size = parameters.correlation_window_size;
    int32_t half_template = parameters.correlation_window_size / 2;
    int32_t search_win_size = parameters.search_window_size;
    int32_t half_search_win_size = parameters.search_window_size / 2;
    
    int32_t pc = lmk_child.num_cols / 20;
    int32_t pr = lmk_child.num_rows / 20;
    
    double *match_pair_base = (double *)malloc(sizeof(double)*pc*pr);
    double *match_pair_child = (double *)malloc(sizeof(double)*pc*pr);
    uint8_t *template = (uint8_t *)malloc(sizeof(uint8_t)*template_size*template_size);
    
    if(match_pair_base==NULL || match_pair_child == NULL || template == NULL){
        printf("MatchFeatures_register_two_landmarks(): memory allocation error\n");
        free_lmk(&lmk_child);
        free_lmk(&lmk_base);
        
        if(match_pair_base != NULL)
            free(match_pair_base);
        if(match_pair_child != NULL)
            free(match_pair_child);
        if(template != NULL)
            free(template);
        return 0;
    }
    
    // Get transformation from child to base
    double base2child_init[3][3] = {0};
    double child2base_init[3][3] = {0};
    mult333(lmk_child.mapRworld, lmk_base.worldRmap, child2base_init);
    inverseHomography33(child2base_init, base2child_init); // base to child
    
    // Find forstner features
    int32_t num_features = 0;
    int64_t (*feature_coord)[2] = malloc(sizeof(int64_t[parameters.num_features * 2]));
    float *feature_strength = malloc(sizeof(float) * parameters.num_features);
    int_forstner_nbest_even_distribution(lmk_child.srm, lmk_child.num_cols, lmk_child.num_rows, 10, 10  , lmk_child.num_cols-20, lmk_child.num_rows -20, parameters.forstner_feature_window_size, parameters.num_features, &num_features, feature_coord, feature_strength, (int32_t)parameters.min_dist_feature);
    
    uint8_t *tmpimg = (uint8_t *)malloc(sizeof(uint8_t)*lmk_base.num_pixels);
    if(tmpimg == NULL){
        printf("MatchFeatures_register_two_landmarks(): memory allocation error\n");
        free(feature_coord);
        free(feature_strength);
        return 0;
    }
    memcpy(tmpimg, lmk_base.srm, sizeof(uint8_t)*lmk_base.num_pixels);
    
    //For each feature, find the best match in the other image
    int32_t num_pairs = 0;
    int32_t tp = 0;
    for (int32_t i = 0; i < num_features; ++i)
    {
        int64_t *child_feature = feature_coord[i];
        // [THP 2024-09-25] Skip child features on NaN elevation
        if (isnan(lmk_child.ele[child_feature[1] * lmk_child.num_cols + child_feature[0]]))
        {
            continue;
        }
        double coord_in_base[2];
        homographyTransfer33(child2base_init, (double)child_feature[0], (double)child_feature[1], coord_in_base);
        if(coord_in_base[0] > 0 && coord_in_base[0] < lmk_base.num_cols && coord_in_base[1] > 0 && coord_in_base[1] < lmk_base.num_rows)
        {
            int32_t center_x = (int32_t)coord_in_base[0];
            int32_t center_y = (int32_t)coord_in_base[1];
            double dx = coord_in_base[0] - center_x;
            double dy = coord_in_base[1] - center_y;
            
            int32_t k = 0;
            for (int32_t m = -half_template; m <= half_template; ++m)
            {
                for (int32_t n = -half_template; n <= half_template; ++n)
                {
                    double base_pt[2];
                    double child_pt[2];
                    base_pt[0] = center_x + n;
                    base_pt[1] = center_y + m;
                    homographyTransfer33D(base2child_init, base_pt, child_pt);
                    double value = Interpolate_LMK_SRM(&lmk_child , child_pt[0], child_pt[1] );
                    template[k] = (int32_t)value;
                    k++;
                }
            }
            
            int64_t left2 = center_x - half_search_win_size;
            int64_t top2 = center_y - half_search_win_size;
            if (top2 < 0) top2 = 0;
            if (left2 < 0) left2 = 0;
            int64_t cols_win2 = search_win_size;
            int64_t rows_win2 = search_win_size;
            if (left2 + search_win_size > lmk_base.num_cols)  cols_win2 = lmk_base.num_cols - left2 - 1;
            if (top2 + search_win_size > lmk_base.num_rows)  rows_win2 = lmk_base.num_rows - top2 - 1;
            
            double bestr= -1.0, bestc= -1.0, bestv= 0.0;
            double cov[3] = {0};
            bool status = corimg_long(template,
                                template_size,
                                0, 0,
                                template_size, template_size,
                                lmk_base.srm,
                                lmk_base.num_cols,
                                left2, top2,
                                cols_win2, rows_win2,
                                &bestr, &bestc, &bestv, cov);
            if (status && bestv > parameters.min_correlation)
            {
                double center_base[2];
                double center_child[2];
                center_base[0] = coord_in_base[0];
                center_base[1] = coord_in_base[1];
                homographyTransfer33D(base2child_init, center_base, center_child);
                
                match_pair_child[num_pairs * 2] = center_child[0];
                match_pair_child[num_pairs * 2 + 1] = center_child[1];
                //Adjust coordinate for subpixel alignment
                match_pair_base[num_pairs * 2] = bestc + dx;
                match_pair_base[num_pairs * 2 + 1] = bestr + dy;
                
                
                DrawArrow(tmpimg, lmk_base.num_cols, lmk_base.num_rows,
                          coord_in_base[0], coord_in_base[1],
                          //coord_in_base[0] + (match_pair_child[num_pairs * 2] - coord_in_base[0]),
                          //coord_in_base[1] + (match_pair_child[num_pairs * 2 + 1] - coord_in_base[1]),
                          bestc,
                          bestr,
                          255, 3);
                num_pairs++;
            }
            else
            {
                DrawFeatureBlock(tmpimg, lmk_base.num_cols, lmk_base.num_rows, coord_in_base[0], coord_in_base[1], 255, 5);
                
            }
        }
        
    }
    
#ifdef DEBUG
    write_channel_separated_image("matched_point.png", tmpimg, lmk_base.num_cols, lmk_base.num_rows, 1);
#endif
    
    //Calculate a homography from the feature pairs
    double homo[3][3]= {0};
    int32_t num_inliers = getHomographyFromPoints_RANSAC_frame(match_pair_child, match_pair_base, num_pairs, homo, REPROJECTION_THRESHOLD);
    
    double *pts_3d_child = (double *)malloc(sizeof(double)*num_pairs * 3);
    double *pts_3d_base = (double *)malloc(sizeof(double)*num_pairs * 3);
    if(pts_3d_child == NULL || pts_3d_base == NULL){
        printf("MatchFeatures_register_two_landmarks(): memory allocation error\n");
        free_lmk(&lmk_child);
        free_lmk(&lmk_base);
        free(tmpimg);
        free(match_pair_base);
        free(match_pair_child);
        free(template);
        free(feature_coord);
        free(feature_strength);
        
        if(pts_3d_child != NULL)
            free(pts_3d_child);
        if(pts_3d_base != NULL)
            free(pts_3d_base);
        return 0;
    }
    
#ifdef DEBUG
    memcpy(tmpimg, lmk_base.srm, sizeof(uint8_t)*lmk_base.num_pixels);
#endif

    
    // Copy all homography inliers to point clouds, pts_3d_child and pts_3d_base
    int32_t j = 0;
    for (int32_t i = 0; i < num_pairs; ++i)
    {
        double d[2] = {0}, d_base[2] = {0};
        d[0] = match_pair_child[i * 2 + 0];
        d[1] = match_pair_child[i * 2 + 1];
        homographyTransfer33(homo, d[0], d[1], d_base);
        
        double d_pair[2]= {0}, d_diff[2]= {0};
        d_pair[0] = match_pair_base[i * 2 + 0];
        d_pair[1] = match_pair_base[i * 2 + 1];
        d_diff[0] = d_base[0] - d_pair[0];
        d_diff[1] = d_base[1] - d_pair[1];
        double s = sqrt(d_diff[0] * d_diff[0] + d_diff[1] * d_diff[1]);
        if (s < REPROJECTION_THRESHOLD)
        {
            
#ifdef DEBUG
            DrawArrow(tmpimg, lmk_base.num_cols, lmk_base.num_rows,
                      d[0], d[1],
                      d_pair[0], d_pair[1],
                      255, 3);
#endif
            
            if (LMK_Col_Row2World(&lmk_child, d[0], d[1], &pts_3d_child[j * 3]) &
                LMK_Col_Row2World(&lmk_base, d_pair[0], d_pair[1], &pts_3d_base[j * 3]))
            {
                ++j;
            }
        }
    }

#ifdef DEBUG
    printf("# of RANSAC inliers %d\n", j);
    write_channel_separated_image("RANSAC_inlier.png", tmpimg, lmk_base.num_cols, lmk_base.num_rows, 1);
#endif
    
    // [THP 2024-09-24] Check that inlier count is non zero, otherwise later call to Point_Clouds_rot_T_RANSAC will segfault
    if(j == 0){
        printf("MatchFeatures_register_two_landmarks(): no homography inliers within reprojection threshold\n");
        free_lmk(&lmk_child);
        free_lmk(&lmk_base);
        free(tmpimg);
        free(match_pair_base);
        free(match_pair_child);
        free(template);
        free(feature_coord);
        free(feature_strength);

        if(pts_3d_child != NULL)
            free(pts_3d_child);
        if(pts_3d_base != NULL)
            free(pts_3d_base);
        return 0;
    }
    
    // Rotation and translation refinement on point clouds
    double baseRchild[3][3]= {0};
    double T[3]= {0};

    // [THP 2024/10/21] Point_Clouds_rot_T_RANSAC does not update baseRchild and T if it cannot find enough inliers
    int32_t success_homography_to_rot_T = Point_Clouds_rot_T_RANSAC(pts_3d_child, pts_3d_base, j, baseRchild, T, 30);
    if (!success_homography_to_rot_T)
    {
        printf("MatchFeatures_register_two_landmarks(): Point_Clouds_rot_T_RANSAC did not find enough inliers\n");
        free_lmk(&lmk_child);
        free_lmk(&lmk_base);
        free(tmpimg);
        free(match_pair_base);
        free(match_pair_child);
        free(template);
        free(feature_coord);
        free(feature_strength);

        if(pts_3d_child != NULL)
            free(pts_3d_child);
        if(pts_3d_base != NULL)
            free(pts_3d_base);
        return 0;
    }
    prt33(baseRchild);
    prt3(T);
    
//    logResult();
    
    // Update landmark structure with new anchor_point and transformations
    double R[3][3]= {0}, p[3]= {0};
    mult333(baseRchild, lmk_child.worldRmap, R);
    mult331(baseRchild, lmk_child.anchor_point, p);
    add3(p, T, lmk_child.anchor_point);
    
    mult331(baseRchild, lmk_child.map_normal_vector, p);
    copy3(p, lmk_child.map_normal_vector);
    copy33(R, lmk_child.worldRmap);
    trans33(R, lmk_child.mapRworld);
    normalpoint2plane(lmk_child.map_normal_vector, lmk_child.anchor_point, lmk_child.map_plane_params);
    
    size_t buff_size = 256;
    char strbuf[buff_size];
    snprintf(strbuf, buff_size, "%.256s_registered.lmk", lmkname_child);
    Write_LMK(strbuf, &lmk_child);
    
    
    
#ifdef DEBUG
    double base2child[3][3], child2base[3][3];
    estimateHomographyUsingCorners(&lmk_base, &lmk_child, base2child);
    inverseHomography33(base2child, child2base);

    // warp
    char buf[128];
    float *tmpele = (float *)malloc(sizeof(float)*lmk_base.num_pixels);;
    memset(tmpimg, 0, sizeof(uint8_t)*lmk_base.num_pixels);
    memset(tmpele, 0, sizeof(float)*lmk_base.num_pixels);
    for(int r=0; r<lmk_base.num_rows; r++){
        for(int c=0; c<lmk_base.num_cols; c++){
            int dst_r = (child2base[1][0]*c + child2base[1][1]*r + child2base[1][2]) / (child2base[2][0]*c + child2base[2][1]*r + child2base[2][2]);
            int dst_c = (child2base[0][0]*c + child2base[0][1]*r + child2base[0][2]) / (child2base[2][0]*c + child2base[2][1]*r + child2base[2][2]);
            if(dst_r >= 0 && dst_r < lmk_base.num_rows && dst_c >=0 && dst_c < lmk_base.num_cols){
                int tmp = r * lmk_base.num_cols + c;
                int dst_tmp = dst_r * lmk_base.num_cols + dst_c;
                tmpimg[dst_tmp] = lmk_base.srm[tmp];
                tmpele[dst_tmp] = lmk_base.ele[tmp];
            }
        }
    }
    write_channel_separated_image("warped_srm.png", tmpimg, lmk_base.num_cols, lmk_base.num_rows, 1);
    sprintf(buf, "warped_ele_float_%dby%d.raw", lmk_base.num_cols, lmk_base.num_rows);
    FILE *fp = fopen(buf, "wb");
    fwrite(tmpele, sizeof(float), lmk_base.num_cols*lmk_base.num_rows, fp);
    fclose(fp);
#endif
    
    free_lmk(&lmk_child);
    free_lmk(&lmk_base);
    free(tmpimg);
    free(match_pair_child);
    free(match_pair_base);
    free(template);
    free(feature_coord);
    free(feature_strength);
    free(pts_3d_child);
    free(pts_3d_base);
    return 1;
}
