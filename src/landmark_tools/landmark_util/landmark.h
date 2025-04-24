/**
 * \file `landmark.h`
 * \author Yang Cheng
 * \brief  TODO
 *
 *
 * \section updates Update History
 * - Created: 2014
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

#ifndef _LANDMARK_TOOLS_LANDMARK_H_
#define _LANDMARK_TOOLS_LANDMARK_H_

#include <inttypes.h>
#include <stdbool.h>                                         // for bool
#include <stdint.h>                                          // for int32_t
#include <stdio.h>
#include <stdlib.h>

#include "landmark_tools/map_projection/datum_conversion.h"  // for Planet

#define SRM_DEFAULT 100 //pixel value when no known surface reflectance model
#define LMK_FILENAME_SIZE 256
#define LMK_ID_SIZE 32

typedef struct {

  char filename[LMK_FILENAME_SIZE];
  //Fields in file format
  enum Planet BODY;
  char lmk_id[LMK_ID_SIZE];    //!< Data source date of creation, resolution etc
  int32_t num_cols;  //!< width of landmark
  int32_t num_rows;  //!< height of landmark
  double anchor_col; //!< pixel column coordinate of anchor point
  double anchor_row; //!< pixel row coordinates of anchor point
  double resolution; //!< meters/pixel horizontally
  double anchor_point[3]; //!< Vector from world frame (P) origin to local map (Lm) frame origin in meters. 
  double mapRworld[3][3]; //!< Rotation from world frame (P) to local map (Lm)
  uint8_t *srm;           //!< Surface reflectance matrix
  float *ele;             //!< Elevation matrix in meters

  //Derived fields 
  int64_t num_pixels;                  //!< Number of map pixels (derived from num_cols*num_rows)
  double worldRmap[3][3]; //!< Rotation from local map (Lm) to world frame (P)
  double col_row2mapxy[2][3]; //!< Transform from pixel (col,row) coordinates to local map (Lm) (x,y) coordinates 
  double mapxy2col_row[2][3]; //!< Inverse transform of col_row2mapxy
  double map_normal_vector[3]; //!< Lm frame z-direction expressed in P frame. Equal to last column of worldRmap
  double map_plane_params[4]; //!< The map plane parameters describe the map plane in P frame coordinates
} LMK;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \brief Allocate memory for elevation and surface reflectance maps in lmk structure
 *
 * \param[in,out] lmk
 * \param[in] num_cols in allocated array
 * \param[in] num_rows in allocated array
 * \return true if allocation is successful
 * \return false otherwise
 */
bool allocate_lmk_arrays(LMK* lmk, int32_t num_cols, int32_t num_rows);

/**
 \brief 
 
 \param[] lmk 
*/
void free_lmk(LMK* lmk);

/**
 \brief Go from latitude and longitude to the local map anchor point and the local map to world rotation matrix
 
 \param[] lmk 
 \param[] anchor_latitude_degrees 
 \param[] anchor_longitude_degrees 
 \param[] ele0 
*/
void calculateAnchorRotation(LMK* lmk, double anchor_latitude_degrees, double anchor_longitude_degrees, double ele0);

/**
 \brief Fill the lmk structure values which are derived from other values
 such as the normal vector, transforms between row/col and lat/long space, etc.
 
 \param[] lmk 
*/
void calculateDerivedValuesVectors(LMK* lmk);

/**
 * \brief Make a deep copy of a landmark structure
 *  \param[in] from original landmark structure
 *  \param[out] to new copy
 *  \return true if success
 * \return false if memory allocation fails
 */
bool Copy_LMK(const LMK *from, LMK *to);

/**
 * \brief Copy the header values of a landmark structure
 *  \param[in] from original landmark structure
 *  \param[out] to new copy
 */
void Copy_LMK_Header(const LMK *from, LMK *to);

/**
 * \brief Write a landmark file to disk and also write an ascii file of landmark header to "filename".txt
 * \param[in] filename cstring containing filename
 * \param[in] lmk landmark
 * \return true on success
 * \return false if fopen cannot open file to write
 */
bool Write_LMK(const char *filename, const LMK *lmk);

/**
 * \brief Read landmark file into landmark structure
 * \param[in] filename location of landmark file
 * \param[in] lmk landmark structure
 * \return true on success
 * \return false if fopen cannot open file for writing
 */
bool Read_LMK(const char *filename, LMK *lmk);

/**
 \brief Given landmark column, row; and elevation calculate the corresponding position in the world frame
 
 The world frame is in a planetary coordinate system (body-centered body-fixed).
 \param[in] lmk
 \param[in] col column coordinate
 \param[in] row row coordinate
 \param[in] ele elevation
 \param[out] p world position
 */
void LMK_Col_Row_Elevation2World(const LMK *lmk, double x, double y, double ele,
                                    double p[3]);

/**
 \brief Given landmark column and row; interpolate the elevation calculate the corresponding position in the world frame
 
 The world frame is in a planetary coordinate system (body-centered body-fixed).
 \param[in] lmk
 \param[in] col column coordinate
 \param[in] row row coordinate
 \param[out] p world position
 \return true if the map coordinate had elevation data
 \return false if the map coordiante was a non-data value and 0 elevation was used as a stand-in.
 */
bool LMK_Col_Row2World(const LMK *lmk, double x, double y, double p[3]);

/**
 \brief Interpolate the elevation at pixel location (col, row)
 
Uses bilinear interpolation
 
 \param[in] lmk
 \param[in] col column coordinate
 \param[in] row row coordinate
 \return elevation value
 */
double Interpolate_LMK_ELE(const LMK *lmk, double x, double y);

/**
 \brief Interpolate the surface reflectance map at pixel location (col, row)
 
 Uses bilinear interpolation
 
 \param[in] lmk
 \param[in] col column coordinate
 \param[in] row row coordinate
 \return surface reflectance value
 */
double Interpolate_LMK_SRM(const LMK *lmk, double x, double y);

/**
 \brief Give a 3d point in world frame to caculate the col, row and altitude in landmark frame
 
 The world frame is in a planetary coordinate system (body-centered body-fixed).
 
 \param[in] lmk
 \param[in] p point in world coordinates
 \param[out] col column coordinate in landmark frame
 \param[out] row row coordinate in landmark frame
 \param[out] ele elevation in landmark frame
 */
void World2LMK_Col_Row_Ele(const LMK *lmk, double p[3], double *x, double *y,
                              double *ele);

/**
 \brief Find the intersection between a ray in the world frame and the landmark tangent plane
 
 This function is useful for finding the intersection of a camera ray and the tangent plane
 
 \param[in] lmk
 \param[in] c 3d ray endpoint in world frame
 \param[in] ray ray vector in world frame
 \param[out] point3d intersection point in landmark coordinates
 \return false if ray is parallel with plane
 \return true if success
 */
bool Intersect_LMK_map_plane_params_World(const LMK *lmk, double c[3],
                                             double ray[3], double point3d[3]);

/**
 Find the intersection between a ray in the world frame and the landmark elevation map
 
 This function is useful for finding the intersection of a camera ray and the map.
 It works by finding the intersection between the ray and the plane as a first estimate, and then iteratively refining the elevation estimate.
 For low elevation angles, it might overlook occluding terrain between the camera and the map plane that is too far from the first estimate.
 If you want to find the intersection for low elevation angles (`<10` degrees)>, use `Intersect_LMK_ELE_low_slant_angle`
 
 TODO implement a standard algorithim for ray-tracing height-fields
 \param[in] lmk
 \param[in] c 3d ray endpoint in world frame
 \param[in] ray ray vector in world frame
 \param[out] point3d intersection point in landmark coordinates
 \param[in] tol tolerance threshold TODO How to choose?
 \return true if success
 \return false if no intersection found
 */
bool Intersect_LMK_ELE(const LMK *lmk, double c[3], double ray[3],
                          double point3d[3], double tol);

/**
 \brief Find the intersection between a ray in the world frame and the landmark elevation map when the ray and the surface have a low slant angle (`<10` degrees).
 
 This is a forward projection algorithm which iterates along the ray in small steps until an intersection is found.
 To limit the search range, [`mine`, `maxe`] define maximum and minimum bounds on the elevation, and `max_range` defines a maximum distance to traverse between `mine` and `maxe`
 
 This code is slow. Avoid using it unless it is necessary
 
 \param[in] lmk
 \param[in] c 3d ray endpoint in world frame
 \param[in] ray ray vector in world frame
 \param[in] max_range maximum distance to traverse ray in meters
 \param[out] point3d intersection point in landmark coordinates
 \param[in] mine minimum elevation of potential intersection
 \param[in] maxe maximum elevation of potential intersection
 \return true if success
 \return false if intersection is not found (for example out-of-bounds, no data array, ray is parallel to plane, etc)
*/
bool Intersect_LMK_ELE_low_slant_angle(const LMK *lmk, double c[3], double ray[3],
                                          double max_range, double point3d[3],
                                          double mine, double maxe);

/**
 * \brief Copy values from region of interest (roi) of `lmk` into `lmk_sub`
 *
 * This function moves the anchor point of the landmark to the center of the region of interest, but keeps the same normal vector and elevation values
 * As a result, the anchor point will not have a zero value in the `lmk_sub.ele` array.
 *
 * \param[in] lmk
 * \param[out] lmk_sub
 * \param[in] left col index for start of roi
 * \param[in] top row index for start of roi
 * \param[in] ncols width of roi
 * \param[in] nrows height of roi
 * \return true on success
 * \return false otherwise
 */                                          
bool SubsetLMK(const LMK *lmk, LMK *lmk_sub, int32_t left, int32_t top,
                  int32_t ncols, int32_t nrows);

/**
 * \brief Resample the landmark struct such that the new resolution is `scale*old_resolution`
 *
 * \param[in] lmk input landmark
 * \param[out] lmk_sub rescaled landmark
 * \param[in] scale
 * \return true on success
 * \return false otherwise
 */
bool ResampleLMK(const LMK *lmk_from, LMK *lmk_to, double scale);

/** \brief Crop a landmark struct to a region of interest (ROI)
 *
 * This function moves the anchor point to the center of the roi and recalculates the tangent plane relative to the new anchor point.
 * Therefore, `lmk_sub.ele` values will differ compaired to the corresponding element in `lmk.ele`.
 * \param[in] lmk input landmark
 * \param[out] lmk_sub cropped landmark
 * \param[in] left col index for start of roi
 * \param[in] top row index for start of roi
 * \param[in] ncols width of roi
 * \param[in] nrows height of roi
 */
bool Crop_IntepolateLMK(const LMK *lmk, LMK *lmk_sub, int32_t left, int32_t top,
                        int32_t ncols, int32_t nrows);

/**
 * \brief Rescale the resolution of a landmark struct
 *
 * \param[in] lmk input landmark
 * \param[out] lmk_out rescaled landmark
 * \param[in] out_lmk_res new resolution
 * \return true on success
 * \return false otherwise
 */
bool RescaleLMK(const LMK *lmk_from, LMK *lmk_to,
                   double out_lmk_res);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _LANDMARK_TOOLS_LANDMARK_H_
