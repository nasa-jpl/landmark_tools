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

#include "landmark_tools/landmark_util/create_landmark.h"

#include <math.h>                   // for fabs, NAN
#include <string.h>

#include "landmark_tools/map_projection/datum_conversion.h"        // for LatLongHeight_ECEF_xyz, Planet
#include "landmark_tools/landmark_util/landmark.h"    // for LMK_Col_Row_Elevation2World
#include "landmark_tools/data_interpolation/interpolate_data.h"   // for inter_short_elevation, inter_uint8_matrix
#include "landmark_tools/map_projection/equidistant_cylindrical_projection.h"
#include "landmark_tools/map_projection/utm.h"                    // for latlong2utm
#include "landmark_tools/map_projection/stereographic_projection.h"  // for LatLong2StereographicProjection
#include "landmark_tools/utils/two_level_yaml_parser.h"
#include "landmark_tools/map_projection/orthographic_projection.h"

#define ELEVATION_TOLERANCE 0.01

/**
 
 */
bool readCreateLandmarkConfiguration(const char *yaml_config_filename,
                enum Projection projection,
                enum Planet planet, LMK *lmk, GeoTiffData* geotiff_info,
                char* demname, size_t demname_size,
                double* anchor_latitude_degrees, double* anchor_longitude_degrees)
{
    //TODO (Cecilia) find a clean solution that doesn't require code duplication for optional keys
    const char* parent_keys[] = {"input", "output"};
    if(projection == GEOGRAPHIC){
        const char* child_keys[] = {"filename", "width_px", "height_px", "bit_depth", "upper_left_x_projection_unit", "upper_left_y_projection_unit","pixel_resolution_projection_unit",
            "filename", "width_px", "height_px", "pixel_resolution_meters", "center_latitude", "center_longitude"};
        size_t num_child_keys[] = {7, 6};
        const char* values[15] = {""};
        if(!parseYaml(yaml_config_filename,
                      parent_keys,
                      2,
                      child_keys,
                      num_child_keys,
                      true,
                      values)){
            return false;
        }
        
        
        strncpy(demname, values[0], demname_size);
        geotiff_info->imageSize[0]  = atoi(values[1]);
        geotiff_info->imageSize[1] = atoi(values[2]);
        geotiff_info->bits_per_sample = atoi(values[3]);
        geotiff_info->origin[0] = atof(values[4]);
        geotiff_info->origin[1] = atof(values[5]);
        geotiff_info->pixelSize[0] = atof(values[6]);
        geotiff_info->pixelSize[1] = atof(values[6]);
        strncpy(lmk->filename, values[7], LMK_FILENAME_SIZE);
        lmk->num_cols  = atoi(values[8]);
        lmk->num_rows = atoi(values[9]);
        lmk->resolution = atoi(values[10]);
        *anchor_latitude_degrees = atof(values[11]);
        *anchor_longitude_degrees = atof(values[12]);
        
    }else{
        const char* child_keys[] = {"filename", "width_px", "height_px", "bit_depth", "upper_left_x_projection_unit", "upper_left_y_projection_unit","pixel_resolution_projection_unit", "latitude_standard_parallel", "longitude_natural_origin",
            "filename", "width_px", "height_px", "pixel_resolution_meters", "center_latitude", "center_longitude"};
        size_t num_child_keys[] = {9, 6};
        const char* values[15] = {""};
        if(!parseYaml(yaml_config_filename,
                      parent_keys,
                      2,
                      child_keys,
                      num_child_keys,
                      true,
                      values)){
            return false;
        }
        
        
        strncpy(demname, values[0], demname_size);
        geotiff_info->imageSize[0]  = atoi(values[1]);
        geotiff_info->imageSize[1] = atoi(values[2]);
        geotiff_info->bits_per_sample = atoi(values[3]);
        geotiff_info->origin[0] = atof(values[4]);
        geotiff_info->origin[1] = atof(values[5]);
        geotiff_info->pixelSize[0] = atof(values[6]);
        geotiff_info->pixelSize[1] = atof(values[6]);
        geotiff_info->natOrigin[0] = atof(values[7]);
        geotiff_info->natOrigin[1] = atof(values[8]);
        strncpy(lmk->filename, values[9], LMK_FILENAME_SIZE);
        lmk->num_cols  = atoi(values[10]);
        lmk->num_rows = atoi(values[11]);
        lmk->resolution = atoi(values[12]);
        *anchor_latitude_degrees = atof(values[13]);
        *anchor_longitude_degrees = atof(values[14]);
    }
      
    // Fill LMK structure with derived values
    lmk->BODY = planet;
    lmk->num_pixels = lmk->num_cols*lmk->num_rows;
    lmk->anchor_col = (float)lmk->num_cols/2.0;
    lmk->anchor_row = (float)lmk->num_rows/2.0;
    
    // Allocate memory for surface reflectance map and elevation map
    if(allocate_lmk_arrays(lmk, lmk->num_cols, lmk->num_rows)){
        return true;
    }else{
        return false;
    }
}

double getCenterElevation(GeoTiffData* geotiff_info, LMK* lmk, double x, double y){
    // Tie points in map projection space
    double dx0 = geotiff_info->origin[0]; // upper left x
    double dy0 = geotiff_info->origin[1]; // upper left y
    double dres = geotiff_info->pixelSize[0]; // resolution
    
    // Distance in raster space between tie points and x,y
    double dx = (x - dx0)/dres;
    double dy = (dy0 -y)/dres;
    
    // Interpolate the elevation at x,y
    return inter_float_matrix(geotiff_info->demValues, geotiff_info->imageSize[0], geotiff_info->imageSize[1], dx, dy);
}

bool ProjectLatLong(enum Projection proj, LMK* lmk, GeoTiffData* geotiff_info, double lat, double lon, double* x, double* y){
    if(proj==UTM){
        latlong2utm( lat, lon, geotiff_info->natOrigin[1], x, y);
    }else if(proj==GEOGRAPHIC){
        *x = lon;
        *y = lat;
    }else if(proj==EQUIDISTANT_CYLINDRICAL){
        LatLong2EquidistantCylindricalProjection(lat, lon,
                                 geotiff_info->natOrigin[1], geotiff_info->natOrigin[0],
                                 lmk->BODY , x, y);
    }else if(proj==STEREO){
        LatLong2StereographicProjection( lat, lon,
                                        geotiff_info->natOrigin[0], geotiff_info->natOrigin[1],
                                        lmk->BODY , x, y);
        //TODO Test LAMBERT Projection
//    }else if(proj==LAMBERT){
//        if(lg < 0 ) lg = 360+lg;
//        latlong2lambert_sphere(lambert, lat, lg, lmk->reference_body_radius_meters, &x, &y);
    }else if(proj==ORTHOGRAPHIC){
        orthographic_map_projection( lat, lon,
                                    geotiff_info->natOrigin[0], geotiff_info->natOrigin[1],
                                    lmk->BODY , x, y);
    }else{
        return false;
    }
    return true;
}


bool CreateLandmark_dem_only(GeoTiffData* geotiff_info,
            double anchor_latitude_degrees, double anchor_longitude_degrees,
            enum Projection proj,
            LMK* lmk,
            float set_anchor_point_ele)
{
   return CreateLandmark(geotiff_info,
            NULL, 0, 0, anchor_latitude_degrees, anchor_longitude_degrees, proj, lmk, set_anchor_point_ele);
}


bool CreateLandmark(GeoTiffData* geotiff_info,
            uint8_t *srm_img, int32_t srm_width, int32_t srm_height,
            double anchor_latitude_degrees, double anchor_longitude_degrees,
            enum Projection proj,
            LMK* lmk,
            float set_anchor_point_ele)
{
    bool success = true;
    
    double dem_origin_x = geotiff_info->origin[0]; //X
    double dem_origin_y = geotiff_info->origin[1]; //Y
    double dem_resolution = geotiff_info->pixelSize[0]; //res
    
    // Anchor point in DEM projection coordinates
    double x_anchor, y_anchor;
    success &= ProjectLatLong(proj, lmk, geotiff_info,  anchor_latitude_degrees, anchor_longitude_degrees, &x_anchor, &y_anchor);
    if(!success){
        return success;
    }
    
    //Calculate map normal, map to col/row transforms etc.
    double ele0 = getCenterElevation(geotiff_info, lmk, x_anchor, y_anchor);
    if(isnan(ele0)){
        //printf("Error: Center of DEM is non-data value. Cannot convert to landmark\n");
        //return false;
        printf("Center of DEM is non-data value, so assign 0\n");
        ele0 = 0;
    }
    if(!isnan(set_anchor_point_ele))
        ele0 = set_anchor_point_ele;

    calculateAnchorRotation(lmk, anchor_latitude_degrees, anchor_longitude_degrees, ele0);
    calculateDerivedValuesVectors(lmk);
    
    for(int32_t lmk_y = 0; lmk_y < lmk->num_rows; ++lmk_y)
    {
        for(int32_t lmk_x = 0; lmk_x < lmk->num_cols; ++lmk_x)
        {
            int32_t loop = 0;
            double last_elevation_estimate = 0.0;
            double elevation_estimate = 0.0;
            double dem_x = -1;
            double dem_y = -1;
            do{
                double world_p[3];
                //to compute the patch position in ecef
                LMK_Col_Row_Elevation2World(lmk,  (double)lmk_x, (double)lmk_y, elevation_estimate, world_p);
                
                //compute patch position in DEM projection coordinates
                double map_projection_x, map_projection_y, latitude, longitude;
                double temp_elevation_estimate;
                ECEF_to_LatLongHeight(world_p,
                    &latitude, &longitude, &temp_elevation_estimate,
                    lmk->BODY);
                success &= ProjectLatLong(proj, lmk, geotiff_info, latitude, longitude, &map_projection_x, &map_projection_y);
                
                //compute patch position in DEM pixel coordinates
                dem_x = (map_projection_x - dem_origin_x)/dem_resolution;
                dem_y = (dem_origin_y - map_projection_y)/dem_resolution;
                
                //TODO This iterative refinement method is time consuming.
                if(dem_x > 0 && dem_x < geotiff_info->imageSize[0] && dem_y > 0 && dem_y < geotiff_info->imageSize[1])
                {
                    // refine the elevation estimate by retrieving value from dem and recalculating elevation position
                    double elevation_refined = inter_float_matrix(geotiff_info->demValues, geotiff_info->imageSize[0], geotiff_info->imageSize[1], dem_x, dem_y)  ;
                    if(!isnan(elevation_refined)){
                        last_elevation_estimate = elevation_estimate;
                        
                        LatLongHeight_to_ECEF(latitude, longitude, elevation_refined, world_p, lmk->BODY);
                        double temp_lmk_x, temp_lmk_y;
                        World2LMK_Col_Row_Ele(lmk, world_p, &temp_lmk_x, &temp_lmk_y, &elevation_estimate);
                    }else{
                        // dem has non-data value at location
                        elevation_estimate = NAN;
                    }
                } else {
                    // location is outside dem
                    elevation_estimate = NAN;
                }
                loop ++;
            }while(loop<10 && !isnan(elevation_estimate) && fabs(last_elevation_estimate-elevation_estimate)>ELEVATION_TOLERANCE);

            if(srm_img != NULL && dem_x > 0 && dem_x < srm_width && dem_y > 0 && dem_y < srm_height){
                //TODO SRM image must have same resolution and anchor as DEM
                uint8_t val = 0;
                if(inter_uint8_matrix(srm_img, srm_width, srm_height, dem_x, dem_y, &val))
                    lmk->srm[lmk_y*lmk->num_cols + lmk_x] = (int32_t)(val);
            }else{
                lmk->srm[lmk_y*lmk->num_cols + lmk_x] = SRM_DEFAULT;
            }

            lmk->ele[lmk_y*lmk->num_cols + lmk_x] = elevation_estimate;
            
        }
    }
    return true;
}

//TODO Test LAMBERT Projection
//int32_t CreateLandmark_Lambert_Projection_dem_only(st_geotif_info* geotiff_info, 
//            double lat1, double lat2, double lat_org, double lg_cen, 
//            LMK* lmk)
//{
//   return CreateLandmark_Lambert_Projection(geotiff_info, 
//            lat1, lat2, lat_org, lg_cen, 
//            NULL, 0, 0, lmk);
//}
//
//int32_t CreateLandmark_Lambert_Projection(st_geotif_info* geotiff_info, 
//            double lat1, double lat2, double lat_org, double lg_cen,
//            uint8_t *srm_img, int32_t icols, int32_t irows, LMK* lmk)
//{
//    int8_t success = true;
//
//    LAMBERT lambert;
//    
//    //TODO need lat1, lat2, lat_org, lg_cen from geotiff
//    initial_lambert_sphere(lat1, lat2, lat_org, lg_cen, lmk->reference_body_radius_meters, &lambert);
//   
//    return CreateLandmark(geotiff_info,
//             NULL, 0, 0, lmk, LAMBERT, utm_zone);
//}


