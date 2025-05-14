/************************************************
 * \file endian_read_write.h
 * \author Cecilia Mauceri
 * \brief Read and write binary landmark files
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

#ifndef _LANDMARK_TOOLS_ENDIAN_READ_WRITE_H_
#define _LANDMARK_TOOLS_ENDIAN_READ_WRITE_H_

#include <stdbool.h>  // for bool
#include <stdint.h>   // for int64_t, uint8_t
#include <stdio.h>    // for FILE

/**
 \brief Read a big endian byte array from a file 
 Supports floating point and integer types
 
 \param[out] array must be pre-allocated
 \param[in] byte_width the byte width of an array element
 \param[in] isfloat if true, interprets bytes as floating point. otherwise interprets bytes as integer type
 \param[in] size the total number of elements in the array
 \param[in] fp file pointer
 \return int64_t number of elements sucessfully read
*/
int64_t read_big_endian_array(void* array, uint8_t byte_width, bool isfloat, int64_t size, FILE *fp);

/**
 \brief Read a double stored in big endian byte order from a file 
 
 \param[in] fp file pointer
 \param[out] val 
 \return true on success
 \return false on io error
*/
bool read_double_big_endian(FILE *fp, double *val);

/**
 \brief Read a float stored in big endian byte order from a file 
 
 \param[in] fp file pointer
 \param[out] val 
 \return true on success
 \return false on io error
*/
bool read_float_big_endian(FILE *fp, float *val);

/**
 \brief Write an array to a file in big endian byte order
 Supports floating point and integer types
 
 \param[in] array values to write
 \param[in] byte_width the byte width of an array element
 \param[in] isfloat true if array is floating point type, false otherwise
 \param[in] size the total number of elements in the array
 \param[in] fp file pointer
 \return int64_t number of points successfully written
*/
int64_t write_big_endian_array(const void* array, uint8_t byte_width, bool isfloat, int64_t size, FILE *fp);

/**
 \brief Write a double to a file in big endian byte order
 
 \param[in] fp file pointer
 \param[in] val value
 \return true on success
 \return false on io error
*/
bool write_double_big_endian(FILE *fp, double val);

/**
 \brief Write a float to a file in big endian byte order
 
 \param[in] fp file pointer
 \param[in] val 
 \return true on sucess
 \return false on io error
*/
bool write_float_big_endian(FILE *fp, float val);

#endif /* _LANDMARK_TOOLS_ENDIAN_READ_WRITE_H_ */
