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

#include "landmark_tools/landmark_util/estimate_homography.h"
#include "landmark_tools/math/homography_util.h"
#include <assert.h>

void estimateHomographyUsingCorners(const LMK* lmk_base, const LMK* lmk_child, double base2child[3][3]){
    double pts1[8], pts2[8];
    
    //Put the row,col coordinates from lmk_child in pts1
    //Put the row,col coordinates from lmk_base in pts2
    int32_t k = 0;
    for(int32_t i = 0; i <= lmk_child->num_rows; i += lmk_child->num_rows- 1)
    {
        for (int32_t j = 0; j <= lmk_child->num_cols; j += lmk_child->num_cols - 1)
        {
            double p[3], p1[3];
            LMK_Col_Row_Elevation2World(lmk_child, (double)j, (double)i, 0, p);
            World2LMK_Col_Row_Ele(lmk_base, p, &p1[0], &p1[1], &p1[2]);
            pts1[k * 2] = j;
            pts1[k * 2 + 1] = i;
            pts2[k * 2] = p1[0];
            pts2[k * 2 + 1] = p1[1];
            k++;
        }
    }
    
    assert(k==4);
    
    getHomographyFromPoints(pts1, pts2, 4, base2child);  //child to base
}
