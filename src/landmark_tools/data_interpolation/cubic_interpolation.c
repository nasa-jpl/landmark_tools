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

#include "landmark_tools/data_interpolation/cubic_interpolation.h"

bool cubic_interpolation(uint16_t *img, int32_t cols, int32_t rows, double x, double y, double *bv)
{
    int32_t ix, iy;
    //int i, j;
    double dx, dy;
    double hx[4];
    double hy[4];
    if(x < 2 || x > cols - 3 || y < 2 || y > rows -2)
    {
        return false;
    }
    ix = (int32_t)x;
    iy = (int32_t)y;
    dx = x- ix;
    hx[1] = 1.5*dx*dx*dx - 2.5*dx*dx+1;
    dx = dx+1;
    hx[0] = -0.5*dx*dx*dx +2.5*dx*dx - 4.0*dx +2;
    dx = 1.0-(x- ix);
    hx[2] = 1.5*dx*dx*dx - 2.5*dx*dx+1;
    dx = dx+1;
    hx[3] = -0.5*dx*dx*dx +2.5*dx*dx - 4.0*dx +2;
    
    dy = y- iy;
    hy[1] = 1.5*dy*dy*dy - 2.5*dy*dy+1;
    dy = dy+1;
    hy[0] = -0.5*dy*dy*dy +2.5*dy*dy - 4.0*dy +2;
    dy = 1.0-(y- iy);
    hy[2] = 1.5*dy*dy*dy - 2.5*dy*dy+1;
    dy = dy+1;
    hy[3] = -0.5*dy*dy*dy +2.5*dy*dy - 4.0*dy +2;
    ix = ix -1; //move to the top corner
    iy = iy -1;
    *bv = 0;
    for(int32_t m = iy,i = 0; i <   4; m++, i++)
    {
        for(int32_t n = ix,j = 0; j <   4; n++, j++)
        {
            *bv += img[m*cols + n]*hx[j]*hy[i];
        }
    }
    return true;
}
