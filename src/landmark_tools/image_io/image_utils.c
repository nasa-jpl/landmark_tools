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

#include "landmark_tools/image_io/image_utils.h"
#include "landmark_tools/utils/safe_string.h"

#include <stdlib.h>               // for malloc, free, EXIT_FAILURE

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"  // for stbi_write_png


void interleave_rgb(unsigned char *planar, size_t width, size_t height,
                    unsigned char *interleaved){
    for (int32_t i = 0, j = 0; i < width * height * 3; i += 3, j++) {
            interleaved[i] = planar[j];    // Red component
            interleaved[i+1] = planar[j+width*height];  // Green component
            interleaved[i+2] = planar[j+2*width*height];  // Blue component
        }
    
}

void channel_separated_rgb(unsigned char *interleaved, size_t width, size_t height,
                      unsigned char *planar){
    for (int32_t i = 0, j = 0; i < width * height * 3; i += 3, j++) {
            planar[j]   = interleaved[i];    // Red component
            planar[j+width*height] = interleaved[i+1];  // Green component
            planar[j+2*width*height]  = interleaved[i+2];  // Blue component
        }
}

uint8_t* load_channel_separated_image(const char* filename, int32_t *icols, int32_t *irows){
    int32_t ichannels;
    uint8_t *img_interleaved = stbi_load(filename, icols, irows, &ichannels, STBI_default);
    if (img_interleaved == NULL) {
        SAFE_FPRINTF(stderr, 512, "Failure to load surface reflectance map from %s\n", filename);
        return NULL;
    }
    
    uint8_t *img = NULL;
    
    if(ichannels == 3){
        img = (uint8_t*)malloc(sizeof(uint8_t)*(*icols)*(*irows)*ichannels);
        if(img == NULL){
            printf("Failure to allocate memory for img\n");
            stbi_image_free(img_interleaved);
            return NULL;
        }
        channel_separated_rgb(img_interleaved, *icols, *irows,
                              img);
        stbi_image_free(img_interleaved);
    }else if(ichannels == 1){
        img = img_interleaved;
    }else{
        printf("Surface reflectance map must be monochrome or RGB\n");
        stbi_image_free(img_interleaved);
        return NULL;
    }
    
    return img;
}

bool write_channel_separated_image(const char* filename, uint8_t *img, int32_t cols, int32_t rows, int32_t channels){
    uint8_t *tmpImgInterleaved;
    
    if(channels == 3){
        tmpImgInterleaved = (uint8_t *)malloc(sizeof(uint8_t)*cols*rows*channels);
        if (tmpImgInterleaved == NULL)
        {
            SAFE_FPRINTF(stderr, 512, "write_channel_separated_image() ==>> malloc() failed, %s, %d\n", __FILE__, __LINE__);
            return false;
        }
        
        interleave_rgb(img, cols, rows,
                       tmpImgInterleaved);
    }else if(channels == 1){
        tmpImgInterleaved = img;
    }else{
        SAFE_FPRINTF(stderr, 512, "write_channel_separated_image() not supported for %d channels: %s, %d\n", channels, __FILE__, __LINE__);
        return false;
    }

    int ret = stbi_write_png(filename, cols, rows, channels, tmpImgInterleaved, channels*cols*sizeof(uint8_t));
    
    if(channels == 3)
        free(tmpImgInterleaved);
    
    return ret==1;
}
