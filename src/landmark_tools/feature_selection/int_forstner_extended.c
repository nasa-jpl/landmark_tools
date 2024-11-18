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

#include <math.h>                // for fabs
#include <stdio.h>               // for NULL, printf
#include <stdlib.h>
#include <stdbool.h>
#include <float.h>

#include "img/imgutils.h"  // for FAILURE, int_forstner, SUCCESS
#include "landmark_tools/feature_selection/int_forstner_extended.h"

int32_t int_forstner_nbest_even_distribution(uint8_t *image, int32_t xdim, int32_t ydim, int32_t x0, int32_t y0,
                                        int32_t nx, int32_t ny, int32_t n, int32_t max, int32_t *num, int64_t (*pos2)[2], float *intr, int32_t minDist)
{
    
    float *interest = (float *)malloc(sizeof(float)*xdim*ydim);
    if(interest == NULL){
        return FAILURE;
    }
    for(size_t i = 0; i < xdim*ydim; ++i)
    {
        interest[i] = FLT_MAX;
    }
    int_forstner(image, xdim, ydim, 0, 0, xdim,  ydim, n, interest);

    int32_t grid_size = minDist-1;
    
    int32_t length = 20000;
    while(length > 10000)
    {
       grid_size++;
       length = nx*ny/grid_size/grid_size;
    }
    int32_t grid_cols = nx/grid_size;
    int32_t grid_rows = ny/grid_size;
    int32_t minDist_xy = (int32_t)(minDist/1.414);
    
    /* Check that N is odd */
    length = (grid_cols+1)*(grid_rows+1);
    if ((n & 1) == 0) {
       n++;
    }
    
    int64_t *index = (int64_t *) malloc (length * sizeof (int64_t));
    if (index == NULL)
    {
        free(interest);
        return FAILURE;
    }

    float *subsetValues;
    subsetValues = (float *) malloc (length * sizeof (float));
    if (subsetValues == NULL)
    {
        free(interest);
        free(index);
        return FAILURE;
    }
    
    for(size_t i = 0; i < length; ++i)
    {
        index[i] = 0;
        subsetValues[i] = 0.0;
    }
    
    size_t k = 0;
    for(size_t i = 0; i < grid_rows; ++i)
    {
        size_t iy_start =y0+i*grid_size;
        for(size_t j = 0; j < grid_cols; ++j)
        {
            size_t ix_start = x0+j*grid_size;
             for(size_t p = iy_start; p < iy_start+grid_size; ++p)
             {
                 for(size_t q = ix_start; q< ix_start + grid_size; ++q)
                 {
                     if(subsetValues[k] < interest[p*xdim + q] && interest[p*xdim + q] >0.0)  //800 is hyper
                     {
                           subsetValues[k] =  interest[p*xdim + q];
                           index[k] = p*xdim +q;
                     }
                 }
             }
             k++;
        }
    }
    sort_features_descent (length, subsetValues-1, index - 1); /* matrix in RECIPES has index starting 1 */

    int32_t count = 0;
    for (size_t i = 0; i < length; i++)
    {
        int64_t row = index[i] / xdim;
        int64_t col = index[i] - row * xdim;
      
       if (subsetValues[i] < 0.0) continue;
       
       if (row >= y0 && row < (y0 + ny) && col >= x0 && col < (x0+nx))
       {
            bool too_close = false;
            for (size_t j = 0; j < count; j++)
            {
                if (llabs(row - pos2[j][1]) < minDist_xy &&
                      llabs(col - pos2[j][0]) < minDist_xy)
                {
                    too_close = true;
                    break;
                }
            }
            if (!too_close)
            {
                pos2[count][0] = col;
                pos2[count][1] = row;
                intr[count] = subsetValues[i];
                count ++;
                if (count == max)
                {
                    printf("At feature i = %ld done length %d\n", i, length);
                    break;
                }
            }
         }
     }

    /* Finish up */
    *num = count;
    free(interest);
    free(subsetValues);
    free(index);
    return SUCCESS;
}


static void sort_features_descent(int32_t n, float ra[], int64_t rb[])
{
        int64_t l,j,ir,i;
        float rra;
        int64_t rrb;

        l=(n >> 1)+1;
        ir=n;
        for (;;) {
                if (l > 1) {
                        rra=ra[--l];
                        rrb=rb[l];
                } else {
                        rra=ra[ir];
                        rrb=rb[ir];
                        ra[ir]=ra[1];
                        rb[ir]=rb[1];
                        if (--ir == 1) {
                                ra[1]=rra;
                                rb[1]=rrb;
                                return;
                        }
                }
                i=l;
                j=l << 1;
                while (j <= ir) {
                        if (j < ir && ra[j] > ra[j+1]) ++j;
                        if (rra > ra[j]) {
                                ra[i]=ra[j];
                                rb[i]=rb[j];
                                j += (i=j);
                        }
                        else j=ir+1;
                }
                ra[i]=rra;
                rb[i]=rrb;
        }
}
