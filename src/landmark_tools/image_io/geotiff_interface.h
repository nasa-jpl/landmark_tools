/**
 * \file   `geotiff_interface.h`
 * \author Hrand Aghazarian
 * \brief  GDAL wrapper for reading GeoTiff and PDS4 Files
 *
 * \section updates Update History
 * - Created: 2023
 * - Updated: 2024-05-24 by Cecilia Mauceri
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

#ifndef _LANDMARK_TOOLS_GEOTIFF_INTERFACE_H_
#define _LANDMARK_TOOLS_GEOTIFF_INTERFACE_H_

#include <stdint.h>                                          // for int32_t
#include <stdbool.h>

#include "landmark_tools/image_io/geotiff_struct.h"
#include "landmark_tools/map_projection/datum_conversion.h"  // for Projection

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief Read a GeoTiff or PDS4 file into a GeoTiffData struct
 
 \param[in] fileName 
 \param[out] data 
 \return true if successful 
 \return false if ioerror
*/
bool readGeoTiff(const char* fileName, GeoTiffData* data);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LANDMARK_TOOLS_GEOTIFF_INTERFACE_H_
