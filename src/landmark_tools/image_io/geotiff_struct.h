/**
 * \file geotiff_struct.h
 * \author Cecilia Mauceri
 * \brief  Struct for storing metadata for DEM
 *
 * \section updates Update History
 * - Created: 2024-6-18
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

#include <stdint.h>
#include "landmark_tools/map_projection/datum_conversion.h"

#ifndef geotiff_struct_h
#define geotiff_struct_h

typedef struct {
    enum Projection projection; 
    double origin[2];        // Origin (top left x, top left y)
    double pixelSize[2];     // Pixel size (x, y)
    int32_t imageSize[2];        // Image size (cols, rows)
    double noDataValue;      // No data value
    float* demValues;       // DEM values
    double natOrigin[2];     // Natural origin latitude and longitude
    double falseEasting;     // False easting
    double falseNorthing;    // False northing
    uint8_t bits_per_sample; //bit depth of image
} GeoTiffData;

#endif /* geotiff_struct_h */
