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

#include "opencv_image_io.h"

#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

uint8_t* readPGMToArray(const char* filename, int* width, int* height) {
    // Load the image in grayscale mode (CV_8U means 8-bit unsigned single-channel)
    cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    if (image.data == NULL || image.empty()) {
        fprintf(stderr, "Error: Could not open or find the image.\n");
        return NULL;
    }

    // Get the dimensions of the image
    *width = image.cols;
    *height = image.rows;

    // Allocate memory for the uint8_t array
    uint8_t* array = (uint8_t*)malloc(*width * *height * sizeof(uint8_t));
    if (array == NULL) {
        fprintf(stderr, "Error: Memory allocation failed.\n");
        return NULL;
    }

    // Copy image data to the uint8_t array
    memcpy(array, image.data, *width * *height * sizeof(uint8_t));

    return array;
}

bool writePGMFromArray(const char* filename, const uint8_t* array, int width, int height) {
    // Create a cv::Mat from the array data with the specified width and height
    cv::Mat image(height, width, CV_8UC1, (void*)array);

    // Write the image to a PGM file
    if (!cv::imwrite(filename, image)) {
        fprintf(stderr, "Error: Could not write the image.\n");
        return false; // Return error code
    }

    return true; // Success
}

