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

#include "landmark_tools/image_io/geotiff_interface.h"

#include <gdal.h>
#include <ogr_srs_api.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <cpl_conv.h> // for CPLMalloc(), CPLFree()
#include "landmark_tools/utils/safe_string.h"

bool readGeoTiff(const char* fileName, GeoTiffData* data) {
    GDALAllRegister();
    GDALDatasetH hDataset = GDALOpen(fileName, GA_ReadOnly);
    if (hDataset == NULL) {
        SAFE_FPRINTF(stderr, 512, "Failed to open file: %s\n", fileName);
        return false;
    }
    
    data->imageSize[0] = GDALGetRasterXSize(hDataset);
    data->imageSize[1] = GDALGetRasterYSize(hDataset);
    double adfGeoTransform[6];
    if (GDALGetGeoTransform(hDataset, adfGeoTransform) == CE_None) {
        data->origin[0] = adfGeoTransform[0];
        data->origin[1] = adfGeoTransform[3];
        data->pixelSize[0] = adfGeoTransform[1];
        data->pixelSize[1] = adfGeoTransform[5];
    }
    
    char *pszWKT = (char *)GDALGetProjectionRef(hDataset);
    if (pszWKT && strlen(pszWKT) > 0) {
        const char* projection = strdup(pszWKT);
        OGRSpatialReferenceH hSRS = OSRNewSpatialReference(projection);
        if (OSRIsProjected(hSRS)) {
            
            const char *pszProjectionType = OSRGetAttrValue(hSRS, "PROJECTION", 0);
            if(strncmp(pszProjectionType, "Transverse_Mercator", 20)==0){
                printf("UTM Projection Detected\n");
                data->projection = UTM;
            }else if(strncmp(pszProjectionType, "Polar_Stereographic", 20)==0){
                printf("Polar Stereographic Projection Detected\n");
                data->projection = STEREO;
            }else if(strncmp(pszProjectionType, "Stereographic", 20)==0){
                printf("Stereographic Projection Detected\n");
                data->projection = STEREO;
            }else if(strncmp(pszProjectionType, "Oblique_Stereographic", 20)==0){
                printf("Oblique Stereographic Projection Detected\n");
                data->projection = STEREO;
            }else if(strncmp(pszProjectionType, "Equirectangular", 15)==0){
                printf("Equirectangular Projection Detected\n");
                data->projection = EQUIDISTANT_CYLINDRICAL;
                //            }else if(strncmp(pszProjectionType, "Lambert_Conformal_Conic_2SP", 28)==0){
                //                data->projection = LAMBERT;
            }else if(strncmp(pszProjectionType, "Orthographic", 12)==0){
                printf("Orthographic Projection Detected\n");
                data->projection = ORTHOGRAPHIC;
                //            }else if(strncmp(pszProjectionType, "Lambert_Conformal_Conic_2SP", 28)==0){
                //                data->projection = LAMBERT;
            }else{
                SAFE_FPRINTF(stderr, 512, "Projection type %s is not supported", pszProjectionType);
                OSRDestroySpatialReference(hSRS);
                GDALClose(hDataset);
                return false;
            }
            
            data->falseEasting = OSRGetProjParm(hSRS, SRS_PP_FALSE_EASTING, 0, NULL);
            data->falseNorthing = OSRGetProjParm(hSRS, SRS_PP_FALSE_NORTHING, 0, NULL);
            data->natOrigin[0] = OSRGetProjParm(hSRS, SRS_PP_LATITUDE_OF_ORIGIN, 0, NULL);
            data->natOrigin[1] = OSRGetProjParm(hSRS, SRS_PP_CENTRAL_MERIDIAN, 0, NULL);
        }else if(OSRIsGeographic(hSRS)){
            printf("Geographic coordinate system detected\n");
            data->projection = GEOGRAPHIC;
        }else{
            fprintf(stderr, "Failed to find projection.\n");
            GDALClose(hDataset);
            return false;
        }
        
        OSRDestroySpatialReference(hSRS);
    }else{
        fprintf(stderr, "GDALGetProjectionRef failed to read metadata.\n");
        GDALClose(hDataset);
        return false;
    }
    
    GDALRasterBandH hBand = GDALGetRasterBand(hDataset, 1); // Assume band 1 is DEM
    if (hBand == NULL) {
        fprintf(stderr, "Raster band not found.\n");
        GDALClose(hDataset);
        return false;
    }
    
    int bGotNoData;
    double noDataValue = GDALGetRasterNoDataValue(hBand, &bGotNoData);
    data->noDataValue = bGotNoData ? noDataValue : NAN; // Default NoData value if not found
    
    data->demValues = (float *) malloc(sizeof(float) * data->imageSize[0] * data->imageSize[1]);
    if(data->demValues == NULL){
        fprintf(stderr, "Failure to allocate memory.\n");
        GDALClose(hDataset);
        return false;
    }
    
    int32_t dataType = GDALGetRasterDataType(hBand);
    
    if(dataType == GDT_Float32){
        if (GDALRasterIO(hBand, GF_Read, 0, 0, data->imageSize[0], data->imageSize[1], data->demValues,
                         data->imageSize[0], data->imageSize[1], GDT_Float32, 0, 0) != CE_None) {
            fprintf(stderr, "Failed to read raster data.\n");
            GDALClose(hDataset);
            CPLFree(data->demValues);
            return false;
        }
    } else if (dataType == GDT_Float64){
        double *demValues_float64 = (double *) malloc(sizeof(double) * data->imageSize[0] * data->imageSize[1]);
        if(demValues_float64 == NULL){
            fprintf(stderr, "Failure to allocate memory.\n");
            GDALClose(hDataset);
            return false;
        }
        
        if (GDALRasterIO(hBand, GF_Read, 0, 0, data->imageSize[0], data->imageSize[1], demValues_float64,
                            data->imageSize[0], data->imageSize[1], GDT_Float64, 0, 0) != CE_None) {
            fprintf(stderr, "Failed to read raster data.\n");
            GDALClose(hDataset);
            CPLFree(data->demValues);
            return false;
        }
        
        for(int64_t i=0; i<((int64_t)data->imageSize[0] * (int64_t)data->imageSize[1]); i++)
            data->demValues[i] = (float)demValues_float64[i]; // Lose precision
        
        free(demValues_float64);
    }else if(dataType == GDT_Int16){
        int16_t *demValues_int16 = (int16_t *) malloc(sizeof(int16_t) * data->imageSize[0] * data->imageSize[1]);
        if(demValues_int16 == NULL){
            fprintf(stderr, "Failure to allocate memory.\n");
            GDALClose(hDataset);
            return false;
        }
        
        if (GDALRasterIO(hBand, GF_Read, 0, 0, data->imageSize[0], data->imageSize[1], demValues_int16,
                         data->imageSize[0], data->imageSize[1], GDT_Int16, 0, 0) != CE_None) {
            fprintf(stderr, "Failed to read raster data.\n");
            GDALClose(hDataset);
            CPLFree(data->demValues);
            return false;
        }
        
        for(int64_t i=0; i<((int64_t)data->imageSize[0] * (int64_t)data->imageSize[1]); i++)
            data->demValues[i] = (float)demValues_int16[i];
        
        free(demValues_int16);
    }else if(dataType == GDT_UInt16){
        uint16_t *demValues_uint16 = (uint16_t *) malloc(sizeof(uint16_t) * data->imageSize[0] * data->imageSize[1]);
        if(demValues_uint16 == NULL){
            fprintf(stderr, "Failure to allocate memory.\n");
            GDALClose(hDataset);
            return false;
        }
        
        if (GDALRasterIO(hBand, GF_Read, 0, 0, data->imageSize[0], data->imageSize[1], demValues_uint16,
                         data->imageSize[0], data->imageSize[1], GDT_UInt16, 0, 0) != CE_None) {
            fprintf(stderr, "Failed to read raster data.\n");
            GDALClose(hDataset);
            CPLFree(data->demValues);
            return false;
        }
        
        for(int64_t i=0; i<((int64_t)data->imageSize[0] * (int64_t)data->imageSize[1]); i++)
            data->demValues[i] = (float)demValues_uint16[i];
        
        free(demValues_uint16);
    }else{
        free(data->demValues);
        fprintf(stderr, "Only Float32 / Int16 / UInt16/ Geotiffs are currently supported.\n");
        GDALClose(hDataset);
        return false;
    }
    
    double offset = GDALGetRasterOffset(hBand, NULL);
    double scale = GDALGetRasterScale(hBand, NULL);
    if(offset!=0 || scale !=1){
        for(int64_t i=0; i<((int64_t)data->imageSize[0] * (int64_t)data->imageSize[1]); i++)
            data->demValues[i] = (data->demValues[i]*scale) + offset;
    }

    GDALClose(hDataset);
    return true;
}
