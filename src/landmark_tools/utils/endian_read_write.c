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

#include "landmark_tools/utils/endian_read_write.h"

#if defined(LINUX_OS) || defined(MAC_OS)

#include <arpa/inet.h>   // for htonl, ntohl, htons, ntohs

#endif

#if defined(LINUX_OS)

#include <endian.h>

#define htonll(x) htobe64(x)
#define ntohll(x) be64toh(x)

#endif

#ifdef WINDOWS_OS

#include <winsock2.h>

#endif

int64_t read_big_endian_array(void* array, uint8_t byte_width, bool isfloat, int64_t size, FILE *fp){
    if(array == NULL){
        printf("array must be pre-allocated\n");
        return 0;
    }
    
    for(int32_t i = 0; i< size; i++){
        if(isfloat){
            if(byte_width == 32){
                float* ptr = (float*)array;
                if(!read_float_big_endian(fp, &ptr[i])) return i;
            }else if(byte_width == 64){
                double* ptr = (double*)array;
                if(!read_double_big_endian(fp, &ptr[i])) return i;
            }else{
                printf("read_big_endian_array: Unsupported float numerical type %lld\n", size);
                return 0;
            }
        }else{
            if(byte_width == 32){
                uint32_t big_endian_32;
                if(fread(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) return i;
                uint32_t* ptr = (uint32_t*)array;
                ptr[i] = ntohl(big_endian_32);
            }else if(byte_width == 64){
                uint64_t big_endian_64;
                if(fread(&big_endian_64, sizeof(uint64_t), 1, fp) != 1) return i;
                uint64_t* ptr = (uint64_t*)array;
                ptr[i] = ntohll(big_endian_64);
            }else if(byte_width == 16){
                uint16_t big_endian_16;
                if(fread(&big_endian_16, sizeof(uint16_t), 1, fp) != 1) return i;
                uint16_t* ptr = (uint16_t*)array;
                ptr[i] = ntohs(big_endian_16);
            }else{
                printf("read_big_endian_array: Unsupported int numerical type %lld\n", size);
                return 0;
            }
            
        }
    }
    return size;
}

bool read_double_big_endian(FILE *fp, double *val) {
    uint64_t big_endian_64 = 0;
    if (fread(&big_endian_64, sizeof(uint64_t), 1, fp) != 1) {
        return false;
    }

    uint64_t host_order_64 = ntohll(big_endian_64);

    union {
        uint64_t u64;
        double d;
    } u;
    u.u64 = host_order_64;
    *val = u.d;
    return true;
}

bool read_float_big_endian(FILE *fp, float *val) {
    uint32_t big_endian_32 = 0;
    if (fread(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) {
        return false;
    }

    uint32_t host_order_32 = ntohl(big_endian_32);

    union {
        uint32_t u32;
        float f;
    } u;
    u.u32 = host_order_32;
    *val = u.f;
    return true;
}


int64_t write_big_endian_array(void* array, uint8_t byte_width, bool isfloat, int64_t size, FILE *fp){
    for(int32_t i = 0; i< size; i++){
        if(isfloat){
            if(byte_width == 32){
                float* ptr = (float*)array;
                if(!write_float_big_endian(fp, ptr[i])) return i;
            }else if(byte_width == 64){
                double* ptr = (double*)array;
                if(!write_double_big_endian(fp, ptr[i])) return i;
            }else{
                printf("write_big_endian_array: Unsupported float numerical type %lld\n", size);
                return 0;
            }
            
        }else{
            if(byte_width == 32){
                uint32_t* ptr = (uint32_t*)array;
                uint32_t big_endian_32 = htonl(ptr[i]);
                if(fwrite(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) return i;
            }else if(byte_width == 64){
                uint64_t* ptr = (uint64_t*)array;
                uint64_t big_endian_64 = htonll(ptr[i]);
                if(fwrite(&big_endian_64, sizeof(uint64_t), 1, fp)!= 1) return i;
            }else if(byte_width == 16){
                uint16_t* ptr = (uint16_t*)array;
                uint16_t big_endian_16 = htons(ptr[i]);
                if(fwrite(&big_endian_16, sizeof(uint16_t), 1, fp)!= 1) return i;
            }else{
                printf("write_big_endian_array: Unsupported int numerical type %lld\n", size);
                return 0;
            }
        }
    }
    return size;
}

bool write_double_big_endian(FILE *fp, double val) {
    union {
        uint64_t u64;
        double d;
    } u;
    u.d = val;
    uint64_t big_endian_64 = htonll(u.u64);
    
    if (fwrite(&big_endian_64, sizeof(uint64_t), 1, fp) != 1) {
        return false;
    }
    
    return true;
}

bool write_float_big_endian(FILE *fp, float val) {
    union {
        uint32_t u32;
        float f;
    } u;
    u.f = val;
    uint32_t big_endian_32 = htonl(u.u32);
    
    if (fwrite(&big_endian_32, sizeof(uint32_t), 1, fp) != 1) {
        return false;
    }
    
    return true;
}
