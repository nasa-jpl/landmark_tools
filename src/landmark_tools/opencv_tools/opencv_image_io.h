/************************************************
 * \file opencv_image_io.hpp
 * \author Cecilia Mauceri
 * \brief Wrappers for opencv image io functions
 *
 *  \copyright Copyright 2024 California Institute of Technology
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
 * 
 ***********************************************/

#ifndef LANDMARK_TOOLS_OPENCV_IMAGE_IO_HPP
#define LANDMARK_TOOLS_OPENCV_IMAGE_IO_HPP

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 \brief Read a PGM file into a uint8_t array
 
 \param[in] filename 
 \param[out] width 
 \param[out] height 
 \return uint8_t* image array 
*/
uint8_t* readPGMToArray(const char* filename, int* width, int* height);

/**
 \brief Write a PGM file from a uint8_t array
 
 \param[in] filename 
 \param[in] array 
 \param[in] width 
 \param[in] height 
 \return true success 
 \return false IO error
*/
bool writePGMFromArray(const char* filename, const uint8_t* array, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* LANDMARK_TOOLS_OPENCV_IMAGE_IO_HPP */
