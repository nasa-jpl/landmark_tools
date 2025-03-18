/**
 * \file geotiff_struct.h
 * \author Cecilia Mauceri
 * \brief  Read and write image files with stb
 *
 * \section updates Update History
 * - Created: 2024-6-4
 *  
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

#ifndef image_utils_h
#define image_utils_h

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/**
 \brief Turn a channel separated image array into an interleaved image array
 
 \param[in] planar 
 \param[in] width 
 \param[in] height 
 \param[out] interleaved 
*/
void interleave_rgb(unsigned char *planar, size_t width, size_t height,
                    unsigned char *interleaved);

/**
 \brief Turn a channel interleaved image array into a channel separated image array 
 
 \param[in] interleaved 
 \param[in] width 
 \param[in] height 
 \param[out] planar 
*/
void channel_separated_rgb(unsigned char *interleaved, size_t width, size_t height,
                           unsigned char *planar);

/**
 \brief Read an image from `filename` into a channel separated array
 
 \param[in] filename 
 \param[in] icols 
 \param[in] irows 
 \return uint8_t* array 
*/
uint8_t* load_channel_separated_image(const char* filename, int32_t *icols, int32_t *irows);

/**
 \brief Write a channel separated image to `filename`
 
 \param[in] filename 
 \param[in] img 
 \param[in] cols 
 \param[in] rows 
 \param[in] channels 
 \return true on success  
 \return false on IO error
*/
bool write_channel_separated_image(const char* filename, uint8_t *img, int32_t cols, int32_t rows, int32_t channels);

#endif /* image_utils_h */
