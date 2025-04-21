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

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "landmark_tools/data_interpolation/interpolate_data.h"

double  inter_double_matrix(double *img, size_t xsize, size_t ysize, double  x, double  y)
{
    
    register double bv;
    register double p00, p01, p11, p10;
    register int64_t ix, iy;
    register double dx, dy, dx0, dy0;
    
    if(x<0 || y<0 || x>=xsize || y>=ysize){
        //Out of bounds
        return NAN;
    }
    else if(x >= 1 && x < xsize-1 && y >= 1 &&  y < ysize - 1)
    {
        // Floor of x and y
        ix = (int64_t)x;
        iy = (int64_t)y;
        
        // Fractional part of x and y
        dx = (x - ix);
        dy = (y - iy);
        dx0 = 1.0 - dx;
        dy0 = 1.0 - dy;
        
        //Four neighboring elements
        // img[floor(x), floor(y)]
        p00 = img[iy*xsize + ix];
        // img[floor(x)+1, floor(y)]
        p01 = img[iy*xsize + ix+1];
        // img[floor(x)+1, floor(y)+1]
        p11 = img[(iy+1)*xsize + ix+1];
        // img[floor(x), floor(y)+1]
        p10 = img[(iy+1)*xsize + ix];
        
        if(p00 <= NAN || p01 <= NAN || p11 <= NAN || p10 <= NAN)
        {
            return NAN;
        }
        
        //Bilinear interpolation
        bv = dy0 * (dx0 * p00 + dx * p01) + dy * (dx0 * p10 + dx * p11);
        
        return bv;
    }
    else
    {
        return img[(int32_t)y*xsize + (int32_t)x];
        
    }
}

double  inter_float_matrix(float *img, size_t xsize, size_t ysize, double  x, double  y)
{
	 
    register double bv;
    register float p00, p01, p11, p10;
    register int64_t ix, iy;
    register double dx, dy, dx0, dy0;
    
    double round_x = round(x);
    double round_y = round(y);
    if(round_x<0 || round_y<0 || round_x>=xsize || round_y>=ysize){
        //Out of bounds
        return NAN;
    }
    else if(x == round_x && y == round_y){
        //Indices are round
        return img[(int32_t)y*xsize + (int32_t)x];
    }
    
    // If close to edge, don't interpolate along that axis
    if(x > xsize-1){
        x = round_x;
    }
    
    if(y > ysize - 1){
        y = round_y;
    }
    
    // Floor of x and y
    ix = (int64_t)x;
    iy = (int64_t)y;
    
    // Fractional part of x and y
    dx = (x - ix);
    dy = (y - iy);
    dx0 = 1.0 - dx;
    dy0 = 1.0 - dy;
    
    //Four neighboring elements
    // img[floor(x), floor(y)]
    p00 = img[iy*xsize + ix];
    // img[floor(x)+1, floor(y)]
    p01 = img[iy*xsize + ix+1];
    // img[floor(x)+1, floor(y)+1]
    p11 = img[(iy+1)*xsize + ix+1];
    // img[floor(x), floor(y)+1]
    p10 = img[(iy+1)*xsize + ix];
    
    if(isnan(p00) || isnan(p01) || isnan(p11) || isnan(p10))
    {
        return NAN;
    }
    
    //Bilinear interpolation
    bv = dy0 * (dx0 * p00 + dx * p01) + dy * (dx0 * p10 + dx * p11);
    
    return bv;

}
 
bool  inter_uint8_matrix(uint8_t *img, size_t xsize, size_t ysize, double  x, double  y, uint8_t* val)
{
    register double bv;
    register double dx, dy, dx0, dy0;
    register int64_t p00, p01, p11, p10;
    register int64_t ix, iy;
    
    if(x<0 || y<0 || x>=xsize || y>=ysize){
        //Out of bounds
        return false;
    }
    else if(x >= 1 && x < xsize-1 && y >= 1 &&  y < ysize - 1)
    {
        // Floor of x and y
        ix = (int64_t)x;
        iy = (int64_t)y;
        
        // Fractional part of x and y
        dx = (x - ix);
        dy = (y - iy);
        dx0 = 1.0 - dx;
        dy0 = 1.0 - dy;
        
        //Four neighboring elements
        // img[floor(x), floor(y)]
        p00 = img[iy*xsize + ix];
        // img[floor(x)+1, floor(y)]
        p01 = img[iy*xsize + ix+1];
        // img[floor(x)+1, floor(y)+1]
        p11 = img[(iy+1)*xsize + ix+1];
        // img[floor(x), floor(y)+1]
        p10 = img[(iy+1)*xsize + ix];
        
        //Bilinear interpolation
        bv = dy0 * (dx0 * p00 + dx * p01) + dy * (dx0 * p10 + dx * p11);
        
        *val = (uint8_t)round(bv);
        return true;
    }
    else
    {
        *val = img[(int32_t)y*xsize + (int32_t)x];
        return true;
    }
}

double  inter_short_elevation(int16_t *img, size_t xsize, size_t ysize, double  x, double  y)
{
    register double bv;
    register double dx, dy, dx0, dy0;
    register int64_t p00, p01, p11, p10;
    register int64_t ix, iy;
    
    if(x<0 || y<0 || x>=xsize || y>=ysize){
        //Out of bounds
        return NAN;
    }
    else if(x >= 1 && x < xsize-1 && y >= 1 &&  y < ysize - 1)
    {
        // Floor of x and y
        ix = (int64_t)x;
        iy = (int64_t)y;
        
        // Fractional part of x and y
        dx = (x - ix);
        dy = (y - iy);
        dx0 = 1.0 - dx;
        dy0 = 1.0 - dy;
        
        //Four neighboring elements
        // img[floor(x), floor(y)]
        p00 = img[iy*xsize + ix];
        // img[floor(x)+1, floor(y)]
        p01 = img[iy*xsize + ix+1];
        // img[floor(x)+1, floor(y)+1]
        p11 = img[(iy+1)*xsize + ix+1];
        // img[floor(x), floor(y)+1]
        p10 = img[(iy+1)*xsize + ix];
        
        if(p00 <= NAN || p01 <= NAN || p11 <= NAN || p10 <= NAN)
        {
            return NAN;
        }
        
        //Bilinear interpolation
        bv = dy0 * (dx0 * p00 + dx * p01) + dy * (dx0 * p10 + dx * p11);
        
        return bv;
    }
    else
    {
        return img[(int32_t)y*xsize + (int32_t)x];
        
    }
}

int32_t inter_unsigned_short_image(uint16_t *img, size_t xsize, size_t ysize, double  x, double  y, double *bv)
{
    register double dx, dy, dx0, dy0;
    register int64_t p00, p01, p11, p10;
    register int64_t ix, iy;
    
    if(x<0 || y<0 || x>=xsize || y>=ysize){
        //Out of bounds
        *bv = NAN;
        return 0;
    }
    else if(x >= 1 && x < xsize-1 && y >= 1 &&  y < ysize - 1)
    {
        // Floor of x and y
        ix = (int64_t)x;
        iy = (int64_t)y;
        
        // Fractional part of x and y
        dx = (x - ix);
        dy = (y - iy);
        dx0 = 1.0 - dx;
        dy0 = 1.0 - dy;
        
        //Four neighboring elements
        // img[floor(x), floor(y)]
        p00 = img[iy*xsize + ix];
        // img[floor(x)+1, floor(y)]
        p01 = img[iy*xsize + ix+1];
        // img[floor(x)+1, floor(y)+1]
        p11 = img[(iy+1)*xsize + ix+1];
        // img[floor(x), floor(y)+1]
        p10 = img[(iy+1)*xsize + ix];
        
        if(p00 <= NAN || p01 <= NAN || p11 <= NAN || p10 <= NAN)
        {
            *bv = NAN;
            return 0;
        }
        
        //Bilinear interpolation
        *bv = dy0 * (dx0 * p00 + dx * p01) + dy * (dx0 * p10 + dx * p11);
        
        return 1;
    }
    else
    {
        *bv = img[(int32_t)y*xsize + (int32_t)x];
        return 1;
        
    }
}

void rev_short(int16_t *longone)
{
    struct long_bytes {
        int8_t byte1;
        int8_t byte2;
    } *longptr;
    uint8_t temp;

    longptr = (struct long_bytes *) longone;
    temp = longptr->byte1;
    longptr->byte1 = longptr->byte2;
    longptr->byte2 = temp;
}

void rev_float(float *longone)
{
    struct long_bytes {
        int8_t byte1;
        int8_t byte2;
        int8_t byte3;
        int8_t byte4;
    } *longptr;
    uint8_t temp;

    longptr = (struct long_bytes *) longone;
    temp = longptr->byte1;
    longptr->byte1 = longptr->byte4;
    longptr->byte4 = temp;
    temp = longptr->byte2;
    longptr->byte2 = longptr->byte3;
    longptr->byte3 = temp;
}
