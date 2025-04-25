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

#include <math.h>                   // for exp, fabs, sqrt
#include <stdbool.h>
#include <stdio.h>                  // for fopen, snprintf, fclose, FILE, fgets
#include <stdlib.h>                 // for malloc
#include <string.h>

#include "landmark_tools/landmark_util/landmark.h"    // for Write_LMK_PLY_Facet_Window
#include "landmark_tools/landmark_util/point_cloud2grid.h"
#include "landmark_tools/utils/safe_string.h"
#include "rply.h"
#include "math/mat3/mat3.h"

static int32_t vertex_index = 0;
static double *pts_array = 0;
static uint8_t *bv_array = 0;


bool point2lmk( double *pts, uint8_t *bv, size_t num_pts, LMK *lmk, enum PointFrame frame)
{
    float* weight_map = (float *)malloc(sizeof(float)*lmk->num_cols*lmk->num_rows);
    float* srm1 = (float *)malloc(sizeof(float)*lmk->num_cols*lmk->num_rows);
    float* ele1 = (float *)malloc(sizeof(float)*lmk->num_cols*lmk->num_rows);
    
    if(weight_map == NULL || srm1 == NULL || ele1 == NULL){
        if(weight_map != NULL) free(weight_map);
        if(srm1 != NULL) free(srm1);
        if(ele1 != NULL) free(ele1);
        return false;
    }
    
    memset(weight_map, 0, sizeof(float)*lmk->num_cols*lmk->num_rows);
    memset(srm1, 0, sizeof(float)*lmk->num_cols*lmk->num_rows);
    memset(ele1, 0, sizeof(float)*lmk->num_cols*lmk->num_rows);
    
    for(int32_t i = 0; i < num_pts; ++i)
    {
        double x, y, ele;
        if (frame == WORLD){
            World2LMK_Col_Row_Ele(lmk, &pts[i*3], &x, &y, &ele);
        }else if(frame == LOCAL){
            double pw[3], pm[3]; 
            sub3(&pts[i*3], lmk->anchor_point, pw);
            mult331(lmk->mapRworld, pw, pm);
            x = pm[0];
            y = pm[1];
            ele = pm[2];
        }else{
            x = pts[i*3];
            y = pts[i*3+1];
            ele = pts[i*3+2] * lmk->resolution;
        }


        int32_t ix = round(x);
        int32_t iy = round(y);
        uint8_t bb = bv[i];
        
        if(ix >= 0 && iy >= 0 && ix < lmk->num_cols && iy < lmk->num_rows)
        {
            int32_t cmin = ix - 4;
            int32_t  cmax = ix + 4;
            if(cmin < 0) cmin = 0;
            if(cmax >= lmk->num_cols) cmax =lmk->num_cols-1;
            
            int32_t rmin = iy - 4;
            int32_t rmax = iy + 4;
            if(rmin < 0) rmin = 0;
            if(rmax >= lmk->num_rows) rmax = lmk->num_rows-1;
            for(int32_t m = rmin; m <= rmax; ++m)
            {
                for(int32_t n = cmin; n <= cmax; ++n)
                {
                    double d = 2.0*sqrt((m- y)*(m-y) + (n- x)*(n-x));
                    double wt = exp(-d);
                    ele1[m*lmk->num_cols + n] += wt*ele;
                    srm1[m*lmk->num_cols + n] +=wt*(double)bb;
                    weight_map[m*lmk->num_cols + n] +=wt;
                }
            }
            
        }
        
    }
    
    for(int32_t i = 0; i <lmk->num_pixels; ++i)
    {
        if(weight_map[i] > 0.0)
        {
            lmk->ele[i] = ele1[i]/weight_map[i];
            double srm = srm1[i]/weight_map[i];
            if(srm > 255) srm = 255;
            if(srm <0 ) srm = 0;
            lmk->srm[i] = srm;
        }
        else
        {
            lmk->ele[i] = NAN;
            lmk->srm[i] = 0;
        }
    }
    
    if(weight_map != NULL) free(weight_map);
    if(srm1 != NULL) free(srm1);
    if(ele1 != NULL) free(ele1);
    return true;
}

static int vertex_cb(p_ply_argument argument) {
    if(pts_array == NULL) return 0;
    long coord_offset;
    ply_get_argument_user_data(argument, NULL, &coord_offset);
    double value = ply_get_argument_value(argument);
    pts_array[vertex_index*3+coord_offset] = value;
    return 1;
}

static int intensity_cb(p_ply_argument argument) {
    if(bv_array == NULL) return 0;
    ply_get_argument_user_data(argument, NULL, NULL);
    double value = ply_get_argument_value(argument);
    bv_array[vertex_index] = value;
    vertex_index ++;
    return 1;
}


bool readinply(char * plyname, double **pts, uint8_t **bv, size_t *num_pts)
{
    if(bv_array != NULL) free(bv_array);
    if(pts_array != NULL) free(pts_array);
    vertex_index = 0;
    
    p_ply ply = ply_open(plyname, NULL, 0, NULL);
    if (!ply) {
        return false;
    }
    
    if (!ply_read_header(ply)) {
        return false;
    }
    
    size_t n = ply_set_read_cb(ply, "vertex", "x", vertex_cb, pts, 0);
    ply_set_read_cb(ply, "vertex", "y", vertex_cb, pts, 1);
    ply_set_read_cb(ply, "vertex", "z", vertex_cb, pts, 2);
    ply_set_read_cb(ply, "vertex", "intensity", intensity_cb, bv, 0);
    
    pts_array = malloc(n * 3 * sizeof(double));
    bv_array = malloc(n *sizeof(uint8_t));
    if(pts_array == NULL || bv_array == NULL){
        printf("Failure to allocate memory\n");
        if(pts_array!=NULL) free(pts_array);
        if(bv_array!=NULL) free(bv_array);
        return false;
    }
    
    *pts = pts_array;
    *bv = bv_array;
    *num_pts = n;
    
    if (!ply_read(ply)) {
        return false;
    }
    ply_close(ply);
    return true;
}


bool readinpoints_ascii(char * plyname, double **pts, uint8_t **bv, size_t *num_pts)
{
    FILE *fp = fopen(plyname, "r");
    if(fp == NULL)
    {
        return false;
    }
    
    //Count the number of lines so that we know how much memory to allocate
    int buf_size = 128;
    char buf[buf_size];
    memset(&buf, 0, buf_size);
    
    int32_t line_count;
    while (fgets(buf, buf_size, fp) != NULL) {
        line_count++;
    }
    
    double *pts_array = malloc(line_count * 3 * sizeof(double));
    uint8_t *bv_array = malloc(line_count *sizeof(uint8_t));
    if(pts_array == NULL || bv_array == NULL){
        printf("Failure to allocate memory\n");
        if(pts_array!=NULL) free(pts_array);
        if(bv_array!=NULL) free(bv_array);
        return false;
    }
    
    *pts = pts_array;
    *bv = bv_array;
    
    rewind(fp);
    
    // Read each line
    *num_pts = 0;
    for(int32_t i = 0; i< line_count; i++){
        if(fgets(buf, buf_size, fp)){
            // Note that bv_array is type uint8_t but the value should be formated as int64_t.
            if(sscanf(buf, "%lf %lf %lf %ld", &pts_array[*num_pts*3], &pts_array[*num_pts*3+1], &pts_array[*num_pts*3+2] , &bv_array[*num_pts]) == 4){
                 *num_pts += 1;
            }else{
                SAFE_PRINTF(512, "Failure to scan point values from line %s\n", buf);
                printf("Ignoring line and continuing\n");
            }
        }
    }

    fclose(fp);
    
    return true;
}


enum PointFileType strToPointFileType(char *str){
    enum PointFileType filetype = POINT;
    if(str != NULL){
        if(strncmp(str, "POINT", strlen(str))==0){
            filetype = POINT;
        }else if(strncmp(str, "PLY", strlen(str))==0){
            filetype = PLY;
        }else{
            printf("Value of str must be \"POINT\" or \"PLY\"\n");
            filetype = UNDEFINED;
        }
    }
    return filetype;
}

enum e_ply_storage_mode_ strToPlyFileType(char *str){
    enum e_ply_storage_mode_ filetype = PLY_DEFAULT;
    if(str != NULL){
        if(strncmp(str, "PLY_ASCII", strlen(str))==0){
            filetype = PLY_ASCII;
        }else if(strncmp(str, "PLY_BIG_ENDIAN", strlen(str))==0){
            filetype = PLY_BIG_ENDIAN;
        }else if(strncmp(str, "PLY_LITTLE_ENDIAN", strlen(str))==0){
            filetype = PLY_BIG_ENDIAN;
        }else{
            printf("Value of str must be \"PLY_BIG_ENDIAN\" or \"PLY_LITTLE_ENDIAN\" or \"PLY_ASCII\"\n");
            filetype = PLY_DEFAULT;
        }
    }
    return filetype;
}

enum PointFrame strToFrame(char *str){
    enum PointFrame frame = WORLD;
    if(str != NULL){
        if(strncmp(str, "WORLD", strlen(str))==0){
            frame = WORLD;
        }else if(strncmp(str, "LOCAL", strlen(str))==0){
            frame = LOCAL;
        }else if(strncmp(str, "RASTER", strlen(str))==0){
            frame = RASTER;
        }else{
            printf("Value of str must be \"WORLD\" or \"LOCAL\" or \"RASTER\"\n");
            printf("Defaulting to WORLD\n");
            frame = WORLD;
        }
    }else{
        printf("Defaulting to WORLD\n");
    }
    return frame;
}

enum PointStructure strToStructure(char *str){
    enum PointStructure structure = MESH;
    if(str != NULL){
        if(strncmp(str, "MESH", strlen(str))==0){
            structure = MESH;
        }else if(strncmp(str, "POINTCLOUD", strlen(str))==0){
            structure = POINTCLOUD;
        }else{
            printf("Value of str must be \"POINTCLOUD\" or \"MESH\". Defaulting to MESH\n");
            structure = PLY_DEFAULT;
        }
    }
    return structure;
}

bool writePoints(LMK *lmk,  p_ply oply, enum PointFrame frame){
    for(int32_t i = 0; i <lmk->num_rows; i++)
    {
        for(int32_t j = 0; j < lmk->num_cols; j++)
        {
            double ele = Interpolate_LMK_ELE(lmk, j,  i);
            if(!isnan(ele)){
                double dp[3];
                if(frame == WORLD){
                    LMK_Col_Row2World( lmk,  (double)j , (double)i , dp);
                }else if(frame == LOCAL){
                    double draster[3] = {j, i, 1};
                    dp[0] = dot3(lmk->col_row2mapxy[0], draster);
                    dp[1] = dot3(lmk->col_row2mapxy[1], draster);
                    dp[2] = ele;
                }else{
                    dp[0] = j;
                    dp[1] = i;
                    dp[2] = ele / lmk->resolution;
                }

                uint8_t uc = lmk->srm[i *lmk->num_cols + j ];
                if (!ply_write(oply, dp[0])) return false;
                if (!ply_write(oply, dp[1])) return false;
                if (!ply_write(oply, dp[2])) return false;
                if (!ply_write(oply, uc)) return false;
            }
        }
    }

    return true;
}


bool Write_LMK_PLY_Facet_Window(const char *filename, LMK *lmk, int32_t x0, int32_t y0, int32_t c, int32_t r,
                                enum e_ply_storage_mode_ filetype, enum PointFrame frame)
{
    int32_t min_i = y0-r/2;
    int32_t max_i = y0+r/2;
    if (r % 2 == 1)
    {
        max_i++;
    }
    int32_t min_j = x0-c/2;
    int32_t max_j = x0+c/2;
    if (c % 2 == 1)
    {
        max_j++;
    }
    if(min_i<0) min_i = 0;
    if(min_j<0) min_j = 0;
    if(max_i>lmk->num_rows) max_i = lmk->num_rows;
    if(max_j>lmk->num_cols) max_j = lmk->num_cols;
    if(max_i <= min_i) return false;
    if(max_j <= min_j) return false;
    
    p_ply oply = ply_create(filename, filetype, NULL, 0, NULL);
    if (!oply) return false;
    
    p_ply_element element = NULL;
    
    //Find number of points with data values
    //Map index of each vertex in the file to index in lmk elevation array
    int32_t* vertex_indices = (int32_t*)malloc(sizeof(int32_t)*c*r);
    if(vertex_indices == NULL){
        printf("Failure to allocate memory for vertex_indices array\n");
        return false;
    }
    memset(vertex_indices, -1, sizeof(int32_t)*c*r);
    
    int32_t numpts = 0;
    int32_t num_faces = 0;
    for(int32_t i = min_i; i < max_i; i++)
    {
        for(int32_t j = min_j; j < max_j; j++)
        {
            bool ul_valid = !isnan( lmk->ele[i *lmk->num_cols + j] );
            if(ul_valid){
                vertex_indices[i *lmk->num_cols + j] = numpts;
                numpts ++;
            }
             
            if(i<max_i-1 && j<max_j-1){
                bool ur_valid = !isnan( lmk->ele[i *lmk->num_cols + j+1] );
                bool lr_valid = !isnan( lmk->ele[(i+1)*lmk->num_cols + j+1] );
                bool ll_valid = !isnan( lmk->ele[(i+1)*lmk->num_cols + j] );
                
                if(ul_valid && ur_valid && lr_valid){
                    num_faces ++;
                }
                
                if(ul_valid && lr_valid && ll_valid){
                    num_faces ++;
                }
            }
        }
    }
    
    if (!ply_add_element(oply, "vertex", numpts)) return false;
    if (!ply_add_scalar_property(oply, "x", PLY_FLOAT)) return false;
    if (!ply_add_scalar_property(oply, "y", PLY_FLOAT)) return false;
    if (!ply_add_scalar_property(oply, "z", PLY_FLOAT)) return false;
    if (!ply_add_scalar_property(oply, "intensity", PLY_UINT8)) return false;
    if (!ply_add_element(oply, "face", num_faces)) return false;
    if (!ply_add_list_property(oply, "vertex_indices", PLY_INT32, PLY_INT32)) return false;
    if (!ply_write_header(oply)) return false;
    
    // Write point values
    if(!writePoints(lmk, oply, frame)){
        return false;
    }
    
    // Write face values
    int32_t verify_faces = 0;
    for(int32_t i = min_i; i < max_i-1; i++)
    {
        for(int32_t j = min_j; j < max_j-1; j++)
        {
            if(verify_faces > num_faces){
                printf("WARNING: Num_faces is invalid.\n");
            }
            
            double dp[3];
            
            int32_t ul_index = vertex_indices[i*lmk->num_cols + j];
            int32_t ur_index = vertex_indices[i*lmk->num_cols + (j+1)];
            int32_t ll_index = vertex_indices[(i+1)*lmk->num_cols + j];
            int32_t lr_index = vertex_indices[(i+1)*lmk->num_cols + (j+1)];
            
//            if(ul_index == 0 || ur_index == 0 || ll_index == 0 || lr_index == 0){
//                printf("Debug");
//            }
            
            if(ul_index>=numpts || ur_index>=numpts || ll_index>=numpts || lr_index>=numpts){
                printf("WARNING: Vertices are invalid.\n");
            }
            
            if(ul_index>=0 && ur_index>=0 && lr_index>=0){
                if (!ply_write(oply, 3)) return false;
                if (!ply_write(oply, ul_index)) return false;
                if (!ply_write(oply, ur_index)) return false;
                if (!ply_write(oply, lr_index)) return false;
                verify_faces ++;
            }
            
            if(ul_index>=0 && lr_index>=0 && ll_index>=0){
                if (!ply_write(oply, 3)) return false;
                if (!ply_write(oply, ul_index)) return false;
                if (!ply_write(oply, lr_index)) return false;
                if (!ply_write(oply, ll_index)) return false;
                verify_faces ++;
            }
        }
    }
    
    if (!ply_close(oply)) return false;
    
    return true;
}


bool Write_LMK_PLY_Facet(const char *filename, LMK *lmk, enum e_ply_storage_mode_ filetype, enum PointFrame frame)
{
    return Write_LMK_PLY_Facet_Window(filename, lmk, lmk->anchor_col, lmk->anchor_row, lmk->num_cols, lmk->num_rows, filetype, frame);
}


bool Write_LMK_PLY_Points(const char *filename, LMK *lmk, enum e_ply_storage_mode_ filetype, enum PointFrame frame)
{
    p_ply oply = ply_create(filename, filetype, NULL, 0, NULL);
    if (!oply) return false;
    
    p_ply_element element = NULL;
    
    // Find number of points with data values
    int32_t numpts = 0;
    for(int32_t i = 0; i <lmk->num_rows; i++)
    {
        for(int32_t j = 0; j < lmk->num_cols; j++)
        {
            double dp[3];
            if(LMK_Col_Row2World( lmk,  (double)j , (double)i , dp)){
                numpts ++;
            }
        }
    }
    
    if (!ply_add_element(oply, "vertex", numpts)) return false;
    if (!ply_add_scalar_property(oply, "x", PLY_FLOAT)) return false;
    if (!ply_add_scalar_property(oply, "y", PLY_FLOAT)) return false;
    if (!ply_add_scalar_property(oply, "z", PLY_FLOAT)) return false;
    if (!ply_add_scalar_property(oply, "intensity", PLY_UINT8)) return false;
    if (!ply_write_header(oply)) return false;
    
    // Write point values
    if(!writePoints(lmk, oply, frame)){
        return false;
    }
    
    if (!ply_close(oply)) return false;
    return true;
}
