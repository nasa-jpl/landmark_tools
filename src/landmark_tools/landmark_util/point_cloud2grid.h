/**
 * \file   `point_cloud2grid.c`
 * \author Yang Cheng
 * \brief  Convert csv and ply pointclouds to landmark files
 *
 * \section updates Update History
 * - Created: 2021-12-10
 * - Updated: 2023-08-17 by Cecilia Mauceri
 * - Updated: 2024-04-23 by Cecilia Mauceri
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

#ifndef _LANDMARK_TOOLS_POINT_CLOUD2GRID_H_
#define _LANDMARK_TOOLS_POINT_CLOUD2GRID_H_

#include <stdbool.h>                                // for bool
#include <stddef.h>                                 // for size_t
#include <stddef.h>                                 // for size_t
#include <stdint.h>                                 // for int32_t, uint8_t

#include "landmark_tools/landmark_util/landmark.h"  // for LMK
#include "landmark_tools/map_projection/datum_conversion.h"
#include "rply.h"                                   // for e_ply_storage_mode_
#include "rply.h"                                   // for e_ply_storage_mode_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

enum PointFileType{POINT, PLY, UNDEFINED};
enum PointStructure{POINTCLOUD, MESH};
enum PointFrame{WORLD, LOCAL, RASTER};

/** 
 \brief Projects points in pts to landmark coordinates and uses inverse distance weighting to calculate elevations on a grid.
 \param[in] pts array of (X,Y,Z) coordinates in ECEF reference frame
 \param[in] bv array of point intensities for each point in `pts`
 \param[in] num_pts number of points in pts
 \param[in,out] lmk should have complete header at the time of input
 */
bool point2lmk( double *pts, uint8_t *bv, size_t num_pts, LMK *lmk, enum PointFrame frame);

/** \brief Open a .ply file and read the points into an array
 * \param[in] plyname filename
 * \param[out] pts coordinate array
 * \param[out] bv intensity array
 * \param[out] num_pts number of points in file
 * \return true if success
 * \return false if failed to open file
 */
bool readinply(char * plyname, double **pts, uint8_t **bv, size_t *num_pts);

/** \brief Open an ascii file containing one point on every line in the form: `X Y Z intensity`
 * \param[in] plyname filename
 * \param[out] pts coordinate array
 * \param[out] bv intensity array
 * \param[out] num_pts number of points in file
 * \return true if success
 * \return false if failed to open file
 */
bool readinpoints_ascii(char * plyname, double **pts, uint8_t **bv, size_t *num_pts);

/**
 \brief TODO
 
 \param[] filetype_str 
 \return enum PointFileType 
*/
enum PointFileType strToPointFileType(char *filetype_str);

/**
 \brief TODO
 
 \param[] str 
 \return enum e_ply_storage_mode_ 
*/
enum e_ply_storage_mode_ strToPlyFileType(char *str);

/**
 \brief TODO
 
 \param[] structure_str 
 \return enum PointStructure 
*/
enum PointStructure strToStructure(char *structure_str);

/**
 \brief TODO
 
 \param[] frame_str 
 \return enum PointFrame 
*/
enum PointFrame strToFrame(char *frame_str);

/**
 * \brief Write a small patch of landmark to a ply facet
 *
 * This subroutine is used to validating the spatial geometry accuracy of landmark after data manupliation
 * \param[in] filename filename for ply facet
 * \param[in] lmk landmark structure
 * \param[in] x0 center col coordinate of landmark
 * \param[in] y0 center row coordinate of landmark
 * \param[in] c number of cols
 * \param[in] r number of rows
 * \return true on success
 * \return false if fopen cannot open file for writing
 */
bool Write_LMK_PLY_Facet_Window(const char *filename, LMK *lmk, int32_t x0, int32_t y0, int32_t c, int32_t r,
     enum e_ply_storage_mode_ filetype, enum PointFrame frame);

/**
 * \brief Write entire landmark to a ply facet
 * \param[in] filename filename for ply facet
 * \param[in] lmk landmark structure
 * \return true on success
 * \return false if fopen cannot open file for writing
 */
bool Write_LMK_PLY_Facet(const char *filename, LMK *lmk, enum e_ply_storage_mode_ filetype, enum PointFrame frame);

/**
 * \brief Write landmark into a point cloud ply file format
 * \param[in] filename filename for ply facet
 * \param[in] lmk landmark structure
 * \return true on success
 * \return false if fopen cannot open file for writing
 */
bool Write_LMK_PLY_Points(const char *filename, LMK *lmk, enum e_ply_storage_mode_ filetype, enum PointFrame frame);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LANDMARK_TOOLS_POINT_CLOUD2GRID_H_ */
