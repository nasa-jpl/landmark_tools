/**
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

#include <stdio.h>
#include <stdlib.h>

#include "landmark_tools/data_interpolation/interpolate_data.h"
#include "landmark_tools/feature_tracking/corr_image_long.h"
#include "landmark_tools/feature_tracking/feature_match.h"
#include "landmark_tools/feature_tracking/parameters.h"
#include "landmark_tools/math/homography_util.h"

#include "img/utils/imgutils.h"
#include "math/mat3/mat3.h"

int32_t MatchFeaturesOnly(Parameters parameters, uint8_t *img1, uint8_t *mask1, size_t cols1, size_t rows1,
                          uint8_t *img2, uint8_t *mask2, size_t cols2, size_t rows2,
                          double init_homo[3][3], double *pts_img1, double *pts_img2,
                          double *covs, int32_t np)
{
    int32_t no_data_max1 = 0; // default: reject any masked-out point
    int32_t no_data_max2 = -1; // default: mask2 is not used
    return MatchFeaturesOnlyExtended(
        parameters,
        img1,
        mask1,
        cols1,
        rows1,
        no_data_max1,
        img2,
        mask2,
        cols2,
        rows2,
        no_data_max2,
        init_homo,
        pts_img1,
        pts_img2,
        covs,
        np
    );
}

int32_t MatchFeaturesOnlyExtended(
    Parameters parameters,
    uint8_t *img1,
    uint8_t *mask1,
    size_t cols1,
    size_t rows1,
    int32_t no_data_max1,
    uint8_t *img2,
    uint8_t *mask2,
    size_t cols2,
    size_t rows2,
    int32_t no_data_max2,
    double init_homo[3][3],
    double *pts_img1,
    double *pts_img2,
    double *covs,
    int32_t np
)
{
    
    int32_t template_size = parameters.correlation_window_size;
    int32_t half_template = parameters.correlation_window_size/2;
    int32_t search_win_size = parameters.search_window_size;
    int32_t half_search_win_size = parameters.search_window_size/2;
    
    double *pts1 = (double *)malloc(sizeof(double)*np*2);
    double *pts2 = (double *)malloc(sizeof(double)*np*2);
    uint8_t *buf = (uint8_t *)malloc(sizeof(char)*template_size*template_size);
    int32_t *matchid = (int32_t *)malloc(sizeof(int32_t)*np);
    
    if(pts1 == NULL || pts2 == NULL || buf == NULL || matchid == NULL){
        printf("MatchFeaturesOnly(): memory allocation error\n");
        if(pts1 != NULL)
            free(pts1);
        if(pts2 != NULL)
            free(pts2);
        if(buf != NULL)
            free(buf);
        if(matchid != NULL)
            free(matchid);
        return 0;
    }
    
    double inv_homo[3][3];
    inverseHomography33(init_homo, inv_homo);
    
    //For each point in pts_img1
    int32_t numm = 0;
    for(int32_t i = 0; i < np; ++i)
    {
        //Approximate the location of pts_img1 in img2 using the init_homo
        double op[2];
        homographyTransfer33D(init_homo, &pts_img1[i*2+0], op);
        if(op[0] > half_template && op[0] < cols2 - half_template &&
           op[1] > half_template && op[1] < rows2 - half_template)
        {
            int32_t ix0 = (int32_t)(op[0]+0.5);
            int32_t iy0 = (int32_t)(op[1]+0.5);
            double dx  = op[0] - ix0;
            double dy =  op[1] - iy0;
            
            //Copy the image values from the template centered at ix0, iy0 to buf
            //Do bilinear interpolation if template is not centered on pixels
            int32_t no_data1 = 0;
            for(int32_t m = -half_template, k  = 0; m <= half_template; ++m)
            {
                for(int32_t n = -half_template; n <= half_template; ++n)
                {
                    double ip2[2], ip1[2];
                    ip2[0] = ix0 + n;
                    ip2[1] = iy0 + m;
                    
                    homographyTransfer33D(inv_homo, ip2, ip1);
                    
                    uint8_t bv = 0;
                    if(inter_uint8_matrix(img1,  cols1, rows1, ip1[0], ip1[1], &bv)){
                        buf[k] = (int32_t)bv;
                    }
                    
                    uint8_t mask_v = 0;
                    inter_uint8_matrix(mask1, cols1, rows1, ip1[0], ip1[1], &mask_v);
                    if(mask1 != NULL && mask_v!=0){
                        no_data1++;
                    }
                    k++;
                }
            }
            
            if((no_data_max1 >= 0) && (no_data1 > no_data_max1)){
                continue;
            }

            // Define search range in img2 not to exceed size of img2
            int32_t left2 = op[0] - half_search_win_size;
            if(left2 < 0 ) left2 = 0;
            
            int32_t top2 = op[1] - half_search_win_size;
            if(top2 < 0) top2 = 0;
            
            size_t cols_win2 = search_win_size;
            size_t rows_win2 = search_win_size;
            if(left2 + search_win_size > cols2)  cols_win2 = cols2 - left2 - 1;
            if(top2 + search_win_size > rows2)  rows_win2 = rows2 - top2 - 1;
            
            // Check for nans in mask2
            if ((mask2 != NULL) && (no_data_max2 >= 0))
            {
                int32_t no_data2 = 0;
                for (int32_t mask_row = top2; mask_row < top2 + rows_win2; ++mask_row)
                {
                    for (int32_t mask_col = left2; mask_col < left2 + cols_win2; ++mask_col)
                    {
                        size_t mask_pixel = mask_row * cols2 + mask_col;
                        if (mask2[mask_pixel] != 0)
                        {
                            ++no_data2;
                        }
                    }
                }
                if (no_data2 > no_data_max2)
                {
                    continue;
                }
            }

            double bestv = 0.0;
            double bestr, bestc, cov[3];
            // TODO make a regression test and replace with corr_image2
            if(corimg_long (buf,
                           template_size , // template_size ,
                           0, 0,
                           template_size , template_size ,
                           img2,
                           cols2, // rows2,
                           left2, top2,
                           cols_win2, rows_win2,
                           &bestr, &bestc, &bestv, cov))
            {
                if(bestv > parameters.min_correlation )
                {
                    pts_img1[numm*2] = pts_img1[i*2+0];
                    pts_img1[numm*2+1] = pts_img1[i*2+1];
                    
                    pts_img2[numm*2] = bestc + dx;
                    pts_img2[numm*2+1] = bestr + dy;
                    covs[numm]= bestv;
                    numm++;
                }
            }
        }
    }
    
    free(pts1);
    free(pts2);
    free(buf);
    free(matchid);
    return numm;
}
