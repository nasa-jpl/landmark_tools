/**
 * \file   `create_landmark.h`
 * \author Yang Cheng
 * \brief  TODO
 *
 * \section updates Update History
 * - Created: 2018-11-21
 * - Updated: 2023-08-17 by Cecilia Mauceri
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

#ifndef _LANDMARK_TOOLS_CREATE_LANDMARK_H_
#define _LANDMARK_TOOLS_CREATE_LANDMARK_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "landmark_tools/image_io/geotiff_interface.h"
#include "landmark_tools/landmark_util/landmark.h"
#include "landmark_tools/map_projection/datum_conversion.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief TODO
 
 \param[] filename 
 \param[] projection 
 \param[] planet 
 \param[] lmk 
 \param[] geotiff_info 
 \param[] demname 
 \param[] demname_size 
 \param[] anchor_latitude_degrees 
 \param[] anchor_longitude_degrees 
 \return true 
 \return false 
*/
bool readCreateLandmarkConfiguration(const char *filename, enum Projection projection, enum Planet planet, LMK *lmk, GeoTiffData* geotiff_info,
    char* demname, size_t demname_size,
    double* anchor_latitude_degrees, double* anchor_longitude_degrees);

/**
 * \brief Given the x,y coordinates in map projection frame of a point. Return the elevation at that point.
 *
 *  The map projection frame must be shared between geotiff\_info and the (x,y) point. As long as the frame is shared, any frame can be used.
 *
 *  \param[in] geotiff_info
 *  \param[in] lmk
 *  \param[in] x coordinate in map projection frame
 *  \param[in] y coordinate in map projection frame
 * @return elevation (might be NAN if data is missing)
*/    
double getCenterElevation(GeoTiffData* geotiff_info, LMK* lmk, double x, double y);

/**
 \brief TODO
 
 \param[] proj 
 \param[] lmk 
 \param[] geotiff_info 
 \param[] lat 
 \param[] lon 
 \param[] x 
 \param[] y 
 \return true 
 \return false 
*/
bool ProjectLatLong(enum Projection proj, LMK* lmk, GeoTiffData* geotiff_info, double lat, double lon, double* x, double* y);

/**
 \brief Create a lmk structure from an equal rectangular projection 
 using a default value for the surface reflectance model
 
 \param[in] geotiff_info 
 \param[in] anchor_latitude_degrees 
 \param[in] anchor_longitude_degrees 
 \param[in] proj 
 \param[out] lmk 
 \param[in] set_anchor_point_ele 
 \return true 
 \return false 
*/
bool CreateLandmark_dem_only(GeoTiffData* geotiff_info,
            double anchor_latitude_degrees, double anchor_longitude_degrees,
            enum Projection proj,
            LMK* lmk,
            float set_anchor_point_ele);

/**
 \brief Create a lmk structure from an equal rectangular projection 
 using srm_img for the surface reflectance model
 
 \param[in] geotiff_info 
 \param[in] srm_img surface reflectance model scaled to uint8 image. Must be coaligned with DEM.
 \param[in] icols width of srm_img
 \param[in] irows height of srm_img
 \param[in] anchor_latitude_degrees 
 \param[in] anchor_longitude_degrees 
 \param[in] proj 
 \param[out] lmk 
 \param[in] set_anchor_point_ele 
 \return true 
 \return false 
*/
bool CreateLandmark(GeoTiffData* geotiff_info,
            uint8_t *srm_img, int32_t icols, int32_t irows,
            double anchor_latitude_degrees, double anchor_longitude_degrees,
            enum Projection proj,  
            LMK* lmk,
            float set_anchor_point_ele);


// /**
//  * @brief Create a lmk structure from a lambert projection 
//  * using a default value for the surface reflectance model
// */
//int32_t CreateLandmark_Lambert_Projection(GeoTiffData* geotiff_info,
//            double lat1, double lat2, double lat_org, double lg_cen,
//            uint8_t *srm_img, int32_t icols, int32_t irows, LMK* lmk);
//int32_t CreateLandmark_Lambert_Projection_dem_only(GeoTiffData* geotiff_info, 
//            double lat1, double lat2, double lat_org, double lg_cen, 
//            LMK* lmk);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_CREATE_LANDMARK_H_ */
