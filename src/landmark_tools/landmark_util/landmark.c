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

#include <float.h>
#include <math.h>                                                   // for fabs
#include <stdio.h>                  // for fprintf, fread, fwrite, fclose
#include <stdlib.h>                                                 // for free
#include <string.h>

#include "landmark_tools/data_interpolation/interpolate_data.h"
#include "landmark_tools/landmark_util/landmark.h"
#include "landmark_tools/math/math_utils.h"
#include "landmark_tools/math/point_line_plane_util.h"  // for normalpoint2plane, PointRayInters...
#include "landmark_tools/utils/endian_read_write.h"
#include "math/mat3/mat3.h"                                         // for dot3

#define INTERSECTION_MAX_ITERATIONS 100

bool allocate_lmk_arrays(LMK* lmk, int32_t num_cols, int32_t num_rows) {
    free_lmk(lmk); //Clear out any previously allocated memory
    
    lmk->srm = (uint8_t *)malloc(sizeof(uint8_t)*lmk->num_cols*lmk->num_rows);

    if (lmk->srm == NULL)
    {
        printf("allocate_lmk_arrays() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        return false;
    }else{
        for(int32_t i = 0; i < lmk->num_cols*lmk->num_rows; ++i)
        {
            lmk->srm[i] = SRM_DEFAULT;
        }
    }
    
    lmk->ele = (float *)malloc(sizeof(float)*lmk->num_cols*lmk->num_rows);
    if (lmk->ele == NULL)
    {
        printf("allocate_lmk_arrays() ==>> malloc() failed, %.256s, %d\n", __FILE__, __LINE__);
        free(lmk->srm);
        return false;
    }else{
        for(int32_t i = 0; i < lmk->num_cols*lmk->num_rows; ++i)
        {
            lmk->ele[i] = NAN;
        }
    }

    return true;
}


void free_lmk(LMK* lmk) {
    if(lmk->srm != NULL){
        free(lmk->srm);
    }
    if(lmk->ele != NULL){
        free(lmk->ele);
    }
}


void calculateAnchorRotation(LMK* lmk, double anchor_latitude_degrees, double anchor_longitude_degrees, double ele0){
    double lRw[3][3], wRl[3][3];
    LatLongHeight_to_ECEF(anchor_latitude_degrees, anchor_longitude_degrees,
                          ele0, lmk->anchor_point, lmk->BODY);
    localmap2ECEF_rot( anchor_latitude_degrees, anchor_longitude_degrees,
                      ele0, lRw, lmk->BODY);
    
    prt33(lRw);
    trans33(lRw, wRl);
    
    copy33(lRw, lmk->mapRworld);
    return;
}

void calculateDerivedValuesVectors(LMK* lmk){
    lmk->num_pixels = lmk->num_rows*lmk->num_cols;
    
    lmk->col_row2mapxy[0][0] = lmk->resolution;
    lmk->col_row2mapxy[0][1] = 0.0;
    lmk->col_row2mapxy[0][2] = -lmk->resolution*lmk->anchor_col;
    lmk->col_row2mapxy[1][0] = 0.0;
    lmk->col_row2mapxy[1][1] = -lmk->resolution;
    lmk->col_row2mapxy[1][2] = lmk->resolution*lmk->anchor_row;

    lmk->mapxy2col_row[0][0] = 1/lmk->resolution;
    lmk->mapxy2col_row[0][1] = 0;
    lmk->mapxy2col_row[0][2] = lmk->anchor_col;
    lmk->mapxy2col_row[1][0] = 0;
    lmk->mapxy2col_row[1][1] = -1/lmk->resolution;
    lmk->mapxy2col_row[1][2] = lmk->anchor_row;
    
    trans33(lmk->mapRworld, lmk->worldRmap);
    
    lmk->map_normal_vector[0] = lmk->worldRmap[0][2];
    lmk->map_normal_vector[1] = lmk->worldRmap[1][2];
    lmk->map_normal_vector[2] = lmk->worldRmap[2][2];

    copy3(lmk->map_normal_vector, lmk->map_plane_params);
    lmk->map_plane_params[3] = -dot3(lmk->map_normal_vector, lmk->anchor_point);
    
    return;
}

bool Copy_LMK(LMK *from, LMK *to)
{
    Copy_LMK_Header(from, to);
    if(allocate_lmk_arrays(to, to->num_cols, to->num_rows)){
        for(int32_t i =0; i< to->num_pixels; i++){
            to->srm[i] = from->srm[i];
            to->ele[i] = from->ele[i];
        }
        return true;
    }
    return false;
}

void Copy_LMK_Header(LMK *from, LMK *to){
    to->BODY = from->BODY;
    strncpy(to->lmk_id, from->lmk_id, LMK_ID_SIZE);
    to->num_cols = from->num_cols;
    to->num_rows = from->num_rows;
    to->anchor_col = from->anchor_col;
    to->anchor_row = from->anchor_row;
    to->resolution = from ->resolution;
    copy3(from->anchor_point, to->anchor_point);
    copy33(from->mapRworld, to->mapRworld);
    
    to->num_pixels = from->num_pixels;
    copy33(from->worldRmap, to->worldRmap);
    
    copy3(from->col_row2mapxy[0], to->col_row2mapxy[0]);
    copy3(from->col_row2mapxy[1], to->col_row2mapxy[1]);
    copy3(from->mapxy2col_row[0], to->mapxy2col_row[0]);
    copy3(from->mapxy2col_row[1], to->mapxy2col_row[1]);
    copy3(from->map_normal_vector, to->map_normal_vector);
    copy3(from->map_plane_params, to->map_plane_params);
    to->map_plane_params[3] = from->map_plane_params[3];
    
}

bool Write_LMK(const char *filename, LMK *lmk)
{
    // Write landmark file
    FILE *fp;
    fp = fopen(filename, "wb");
    if(fp == NULL)
    {
        printf("Write_LMK() ==>> cannot open file %.256s to write\n", filename);
        return false;
    }

    size_t version_size = 32;
    char version[version_size];
    strncpy(version, "#! LVS Map v3.0", version_size);
    if(fwrite(version, sizeof(uint8_t), version_size, fp) != version_size) return false;

     if(fwrite(&lmk->lmk_id, sizeof(uint8_t), LMK_ID_SIZE, fp) != LMK_ID_SIZE) return false;

    uint32_t big_endian_32 = htonl(lmk->BODY);    
    if(fwrite(&big_endian_32, sizeof(uint32_t), 1, fp)  != 1) return false;
    big_endian_32 = htonl(lmk->num_cols);
    if(fwrite(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) return false;
    big_endian_32 = htonl(lmk->num_rows);
    if(fwrite(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) return false;

    if(!write_double_big_endian(fp, lmk->anchor_col)) return false;
    if(!write_double_big_endian(fp, lmk->anchor_row)) return false;
    if(!write_double_big_endian(fp, lmk->resolution)) return false;

    if(write_big_endian_array(lmk->anchor_point, 64, true, 3, fp)!= 3) return false;
    if(write_big_endian_array(lmk->mapRworld[0], 64, true, 3, fp)!= 3) return false;
    if(write_big_endian_array(lmk->mapRworld[1], 64, true, 3, fp)!= 3) return false;
    if(write_big_endian_array(lmk->mapRworld[2], 64, true, 3, fp)!= 3) return false;
    
    if(fwrite(lmk->srm, sizeof(uint8_t), lmk->num_pixels, fp)!= lmk->num_pixels) return false;
    if(write_big_endian_array(lmk->ele, 32, true, lmk->num_pixels, fp)!= lmk->num_pixels) return false;
    
    fclose(fp);
    
    //Write ascii header file
    size_t buf_size = 256;
    char buf[buf_size];
    snprintf(buf, buf_size, "%.256s.txt", filename);
    fp = fopen(buf, "w");
    if(fp == NULL)
    {
        printf("Write_LMK() ==>> cannot open the file %.256s to write\n", buf);
        return false;
        
    }
    
    fprintf(fp, "LMK_BODY %d \n", lmk->BODY );
    fprintf(fp, "LMK_ID %.32s\n", lmk->lmk_id );
    fprintf(fp, "LMK_SIZE %d %d\n",  lmk->num_cols, lmk->num_rows );
    
    fprintf(fp, "LMK_RESOLUTION %f \n", lmk->resolution );
    
    fprintf(fp, "LMK_ANCHOR_POINT %f %f %f \n", lmk->anchor_point[0],lmk->anchor_point[1],lmk->anchor_point[2]  );
    fprintf(fp, "LMK_ANCHOR_PIXEL %f %f \n", lmk->anchor_col, lmk->anchor_row );
    
    fprintf(fp, "LMK_WORLD_2_MAP_ROT %f %f %f \n", lmk->mapRworld[0][0], lmk->mapRworld[0][1], lmk->mapRworld[0][2]  );
    fprintf(fp, "LMK_WORLD_2_MAP_ROT %f %f %f \n", lmk->mapRworld[1][0], lmk->mapRworld[1][1], lmk->mapRworld[1][2]  );
    fprintf(fp, "LMK_WORLD_2_MAP_ROT %f %f %f \n", lmk->mapRworld[2][0], lmk->mapRworld[2][1], lmk->mapRworld[2][2]  );
    
    fclose(fp);
    
    return true;
}

bool Read_LMK(const char *filename, LMK *lmk)
{
    FILE *fp;
    fp = fopen(filename, "rb");
    if(fp == NULL)
    {
        printf("Read_LMK() ==>> cannot open file %.256s to read\n", filename);
        return false;
    }
    
    strncpy(lmk->filename, filename, LMK_FILENAME_SIZE);
    uint32_t big_endian_32 = 0;
    uint64_t big_endian_64 = 0;

    // Version comment
    size_t char_array_size = 32;
    char char_array[char_array_size];
    if(fread(char_array, sizeof(char), char_array_size, fp) != char_array_size) return false;
    printf("%32s\n", char_array);

    if(fread(lmk->lmk_id, sizeof(char), LMK_ID_SIZE, fp) != LMK_ID_SIZE) return false;

    if(fread(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) return false;
    lmk->BODY = ntohl(big_endian_32);
    if(fread(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) return false;
    lmk->num_cols = ntohl(big_endian_32);
    if(fread(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) return false;
    lmk->num_rows = ntohl(big_endian_32);
    
    if(!read_double_big_endian(fp, &lmk->anchor_col)) return false;
    if(!read_double_big_endian(fp, &lmk->anchor_row)) return false;
    if(!read_double_big_endian(fp, &lmk->resolution)) return false;
    
    if(read_big_endian_array(lmk->anchor_point, 64, true, 3, fp) != 3) return false;
    if(read_big_endian_array(lmk->mapRworld[0], 64, true, 3, fp) != 3) return false;
    if(read_big_endian_array(lmk->mapRworld[1], 64, true, 3, fp) != 3) return false;
    if(read_big_endian_array(lmk->mapRworld[2], 64, true, 3, fp) != 3) return false;
    
    calculateDerivedValuesVectors(lmk);
    
    if(allocate_lmk_arrays(lmk, lmk->num_cols, lmk->num_rows)){
        if(fread(lmk->srm, sizeof(uint8_t), lmk->num_pixels, fp) != lmk->num_pixels) return false;
        if(read_big_endian_array(lmk->ele, 32, true, lmk->num_pixels, fp) != lmk->num_pixels) return false;
        fclose(fp);
        return true;
    }else
    {
        fclose(fp);
        free_lmk(lmk);
        return false;
    }
}

void LMK_Col_Row_Elevation2World(LMK *lmk,  double col, double row, double ele, double p[3])
{
    double pm[3];
    double pim[3];
    double p1[3];
    
    pim[0] = col;
    pim[1] = row;
    pim[2] = 1;
    
    pm[0] = dot3(lmk->col_row2mapxy[0], pim);
    pm[1] = dot3(lmk->col_row2mapxy[1], pim);
    pm[2] = ele;
    mult331(lmk->worldRmap, pm, p1);
    add3(p1, lmk->anchor_point, p);
}


bool LMK_Col_Row2World(LMK *lmk,  double col, double row,  double p[3])
{
    double pm[3];
    double pim[3];
    pim[0] = col;
    pim[1] = row;
    pim[2] = 1;
    
    //mult331(lmk->col_row2mapxy, pim, pm);
    pm[0] = dot3(lmk->col_row2mapxy[0], pim);
    pm[1] = dot3(lmk->col_row2mapxy[1], pim);
    pm[2] = Interpolate_LMK_ELE(lmk, col,  row);
    
    bool elevation_is_nan = isnan(pm[2]);
    if(elevation_is_nan)
    {
       pm[2] = 0;
    }
    
    //sub3(pm, lmk.v, pm);
    mult331(lmk->worldRmap, pm, p);
    add3(p, lmk->anchor_point, p);
    return !elevation_is_nan;
}


double  Interpolate_LMK_ELE(LMK *lmk,  double  col, double  row)
{
    return inter_float_matrix(lmk->ele, lmk->num_cols , lmk->num_rows, col,   row);
}


double  Interpolate_LMK_SRM(LMK *lmk, double  col, double  row)
{
    uint8_t val = 0;
    if(inter_uint8_matrix(lmk->srm, lmk->num_cols, lmk->num_rows,  col, row, &val)){
        return (double)val;
    }else{
        return NAN;
    }
}


void World2LMK_Col_Row_Ele(LMK *lmk, double p[3], double *col, double *row, double *ele)
{
    double pm[3];
    double pi[3];
    double pw[3];
    sub3(p, lmk->anchor_point, pw);
    mult331(lmk->mapRworld, pw, pm);
    *ele = pm[2];
    pm[2] = 1;
    pi[0] = dot3(lmk->mapxy2col_row[0], pm );
    pi[1] = dot3(lmk->mapxy2col_row[1], pm );
    *col = pi[0];
    *row = pi[1];
}


bool Intersect_LMK_map_plane_params_World(LMK *lmk, double c[3], double ray[3],  double point3d[3])
{
    double r;
    double cp, rp;
    double tmp[3];
    cp = dot3(c, lmk->map_plane_params);
    rp = dot3(ray, lmk->map_plane_params);
    if(rp == 0.0){
        return false;
    }
    r = (-lmk->map_plane_params[3] - cp)/rp;
    scale3(r, ray, tmp);
    add3(c,tmp, point3d);
    return true;
}


bool Intersect_LMK_ELE(LMK *lmk, double c[3], double ray[3],  double point3d[3], double tol)
{
    int32_t cols = lmk->num_cols;
    int32_t rows = lmk->num_rows;
    //compute the elevation of ray to the LMK plane
    double dot_n_ray = dot3(ray, lmk->map_normal_vector);
    //initialize p to be the intersection between ray and the map plane
    double  p[3] ;
    if(!Intersect_LMK_map_plane_params_World(lmk, c, ray,  p)){
        //ray is parallel to landmark plane
        return false;
    }
    
    double dist = DBL_MAX;
    for(int32_t i=0; i<INTERSECTION_MAX_ITERATIONS && dist>tol; i++)
    {
        double dx, dy, ele0;
        World2LMK_Col_Row_Ele(lmk, p, &dx, &dy, &ele0);
        if(dx < 2 || dx > cols-2 || dy < 2 || dy > rows-2)
        {
            //ray interstection is outside landmark area
            return false;
        }
        double ele_dem = Interpolate_LMK_ELE(lmk, dx, dy);
        // Follow ray proportionally to the amount that ele0 differs from dem elevation
        // Elevation is used a normalization factor because rays hitting at a shallow angle travel
        // through more terrain for the same vertical change than rays hitting perpendicularly
        double cut_ray = (ele_dem-ele0)/dot_n_ray;
        double sray[3];
        scale3(cut_ray, ray, sray);
        add3(p, sray, p);
        dist = fabs(ele_dem - ele0);
    }

    if(dist < tol )
    {
        copy3(p, point3d);
        return true;
    }
    else
    {
        zero3(point3d);
        return false;
    }
}


bool Intersect_LMK_ELE_low_slant_angle(LMK *lmk, double c[3], double ray[3], double max_range,  double point3d[3], double mine, double maxe)
{
//    // Find plane which contains the ray. The intersection should be on this plane
//    double crossv[3] = {0};
//    double plane[4] = {0};
//    cross3(ray, lmk->map_normal_vector, crossv);
//    unit3(crossv, crossv);
//    normalpoint2plane(crossv, c, plane);

    // Find distance between ray endpoint and map plane
    double camh = Point2PlaneDist(c, lmk->map_plane_params);
    if(maxe > camh) maxe = camh;  //we only check any point pointinf downward
    
    // Define two planes parallel with the map plane at distances mine and maxe
    double high_plane[4];
    double  p[3] = {0};
    scale3(maxe, lmk->map_normal_vector, p);
    add3(p, lmk->anchor_point, p);
    normalpoint2plane(lmk->map_normal_vector, p, high_plane);
    
    double low_plane[4];
    scale3(mine, lmk->map_normal_vector, p);
    add3(p, lmk->anchor_point, p);
    normalpoint2plane(lmk->map_normal_vector, p, low_plane);

    // Find intersection of ray with high and low planes
    double ele1, x1, y1;
    PointRayIntersection2Plane(c, ray, high_plane, p);
    World2LMK_Col_Row_Ele(lmk, p, &x1, &y1, &ele1);

    double ele2, x2, y2;
    PointRayIntersection2Plane(c, ray, low_plane, p);
    World2LMK_Col_Row_Ele(lmk, p, &x2, &y2, &ele2);

    // Define the maximum number of iterations to be twice the Chebyshev distance between the planes
    double delta_x = fabs(x1 - x2);
    double delta_y = fabs(y1 - y2);
    int32_t steps;
    if(delta_x>delta_y){
        steps = round(delta_x*2);
    }else{
        steps = round(delta_y*2);
    }
    double dh = -(maxe-mine)/(double)steps; // The step size
    double dx = -(x1 - x2)/(double)steps;
    double dy = -(y1 - y2)/(double)steps;

    // Clamp the number of iterations
    int32_t maxi = max_range/lmk->resolution;
    if(maxi < steps)
    {
        steps = maxi;
    }

    double dhp = 1.0;
    for(int32_t i =0; i<steps; i++) {
        double x = x1 + dx*i;
        double y = y1 + dy*i;
        if(x < 1 || x > lmk->num_cols-1 || y < 1 || i > lmk->num_rows-1)
        {
            return false;
        }
        
        double h_dem = Interpolate_LMK_ELE(lmk, x, y);
        if(isnan(h_dem)){
            //A no-data hole
            continue;
        }
        double h = h_dem-(maxe + dh*i);
        
        if(h >= 0.0 && dhp < 0.0)  //In the last iteration, we passed the intersection.
        {
            //Interpolate the location of the intersection
            double pts[27] = {0};
            // Find four adjacent points
            if( LMK_Col_Row2World(lmk,  x-0.5, y-0.5, &pts[0]) &&
               LMK_Col_Row2World(lmk,  x+0.5, y+0.5, &pts[6]) &&
               LMK_Col_Row2World(lmk,  x+0.5, y-0.5, &pts[9]) &&
               LMK_Col_Row2World(lmk,  x-0.5, y+0.5, &pts[3]) ){
                
                //Calculate a plane containing the four points
                double tx[3], ty[3], dp[3];
                sub3(&pts[9], &pts[0], tx);
                sub3(&pts[6], &pts[3],dp);
                add3(dp, tx, tx);
                sub3(&pts[0], &pts[3], ty);
                sub3(&pts[9], &pts[6], dp);
                add3(dp, ty, ty);
                
                double norm[3];
                cross3(tx, ty, norm);
                unit3(norm, norm);
                LMK_Col_Row2World(lmk,  x , y , p);
                
                //Intersect ray with local plane
                double local_plane[4];
                normalpoint2plane(norm, p, local_plane);
                PointRayIntersection2Plane(c, ray, local_plane, point3d);
                return true;
            }else{
                // One of the adjacent points was a no-data hole.
                return false;
            }
        }
        dhp = h;
    }
    
    return false;
}


bool SubsetLMK(LMK *lmk, LMK *lmk_sub, int32_t left, int32_t top, int32_t ncols, int32_t nrows)
{
    Copy_LMK_Header(lmk, lmk_sub);
    lmk_sub->num_cols = ncols;
    lmk_sub->num_rows = nrows;

    lmk_sub->anchor_col = (double)ncols/2.0 ;
    lmk_sub->anchor_row = (double)nrows/2.0 ;
    lmk_sub->num_pixels = ncols*nrows;
    
    LMK_Col_Row_Elevation2World( lmk,  left + lmk_sub->anchor_col, top + lmk_sub->anchor_row, 0.0, lmk_sub->anchor_point);
    normalpoint2plane(lmk_sub->map_normal_vector, lmk_sub->anchor_point, lmk_sub->map_plane_params);
    double center_ele = 0.0;
    
    uint8_t success = allocate_lmk_arrays(lmk_sub, lmk_sub->num_cols, lmk_sub->num_rows);
    if(!success){
        return success;
    }
    
	for(int32_t i = top, m = 0; i < top+nrows; ++i, ++m)
	{
		for(int32_t j = left, n = 0 ; j < left+ncols; ++j, n++)
		{
			lmk_sub->ele[m*ncols + n] = lmk->ele[i*lmk->num_cols + j] ;
			lmk_sub->srm[m*ncols + n] = lmk->srm[i*lmk->num_cols + j];
			 
		}
	}
    
    return true;
}


bool RescaleLMK(LMK *lmk, LMK *lmk_out, double out_lmk_res)
{
    double scale = out_lmk_res/lmk->resolution;
	return ResampleLMK(lmk, lmk_out,  scale);
}


bool  ResampleLMK(LMK *lmk, LMK *lmk_sub,  double scale)
{
	Copy_LMK_Header( lmk, lmk_sub);
	lmk_sub->num_cols = (int32_t)(lmk_sub->num_cols/scale);
	lmk_sub->num_rows = (int32_t)(lmk_sub->num_rows/scale);
	lmk_sub->resolution = lmk->resolution*scale;
	scale3(scale, lmk->col_row2mapxy[0], lmk_sub->col_row2mapxy[0]);
	scale3(scale, lmk->col_row2mapxy[1], lmk_sub->col_row2mapxy[1]);

	scale3(1/scale, lmk->mapxy2col_row[0], lmk_sub->mapxy2col_row[0]);
	scale3(1/scale, lmk->mapxy2col_row[1], lmk_sub->mapxy2col_row[1]);

	lmk_sub->anchor_col = (float)lmk_sub->num_cols/2.0;
	lmk_sub->anchor_row = (float)lmk_sub->num_rows/2.0  ;
	lmk_sub->num_pixels = lmk_sub->num_cols*lmk_sub->num_rows;

	double x =  lmk_sub->anchor_col*scale ;
	double y =   lmk_sub->anchor_row*scale;
    LMK_Col_Row2World(lmk,  x,  y, lmk_sub->anchor_point);
    bool success = allocate_lmk_arrays(lmk_sub, lmk_sub->num_cols, lmk_sub->num_rows);
    if(!success){
        return false;
    }

	for(int32_t i = 0, k = 0; i < lmk_sub->num_rows; ++i)
	{
		for(int32_t j =0 ; j <  lmk_sub->num_cols; ++j)
		{
            double p[3], ele;
            x = j*scale;
            y = i*scale;
			double d = Interpolate_LMK_ELE(lmk ,  x,  y);
			lmk_sub->ele[k] = (float)d;
			lmk_sub->srm[k] = (uint8_t)Interpolate_LMK_SRM(lmk, x, y);
			k++;
		}
	}
    
    return true;
}


bool Crop_IntepolateLMK(LMK *lmk, LMK *lmk_sub, int32_t left, int32_t top, int32_t ncols, int32_t nrows)
{
    Copy_LMK_Header(lmk, lmk_sub);
    lmk_sub->num_cols = ncols;
    lmk_sub->num_rows = nrows;

    lmk_sub->anchor_col = (double)ncols/2.0 ;
    lmk_sub->anchor_row = (double)nrows/2.0 ;
    lmk_sub->num_pixels = ncols*nrows;
    
    double center_ele = (float)Interpolate_LMK_ELE(lmk , left + lmk_sub->anchor_col, top + lmk_sub->anchor_row);
    LMK_Col_Row_Elevation2World( lmk,  left + lmk_sub->anchor_col, top + lmk_sub->anchor_row, center_ele, lmk_sub->anchor_point);
    double anchor_latitude_degrees, anchor_longitude_degrees;
    ECEF_to_LatLongHeight(lmk_sub->anchor_point, &anchor_latitude_degrees, &anchor_longitude_degrees, &center_ele, lmk_sub->BODY);
    calculateAnchorRotation(lmk, anchor_latitude_degrees, anchor_longitude_degrees, center_ele);
    calculateDerivedValuesVectors(lmk);
    
    uint8_t success = allocate_lmk_arrays(lmk_sub, lmk_sub->num_cols, lmk_sub->num_rows);
    if(!success){
        return false;
    }
    
    for(int32_t i = 0, k = 0; i < lmk_sub->num_rows; ++i)
    {
        for(int32_t j =0 ; j <  lmk_sub->num_cols; ++j)
        {
            double p[3], x, y, ele;
            LMK_Col_Row_Elevation2World(lmk_sub,  (double)j,  (double)i, 0.0,   p);
            World2LMK_Col_Row_Ele(lmk, p, &x, &y, &ele);
            lmk_sub->ele[k] = Interpolate_LMK_ELE(lmk , x, y);
            lmk_sub->srm[k] = Interpolate_LMK_SRM(lmk, x, y);
            k++;
        }
    }
    
    return true;
}
