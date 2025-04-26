
# #
#  \file   `landmark.py`
#  \author Cecilia Mauceri
#  \brief  Load and save landmark files in python
#   
#  \copyright Copyright 2024 California Institute of Technology
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#  
#  \section updates Update History
#  - Created: 2023-08-22
#  

import numpy as np
import struct

## \brief Unpack a big endian binary matrix into a numpy array
#
# Element types supported:
# d = double
# f = float
# B = uint8
def unpack_matrix(type, size, buffer, little_endian=False):
    if type == 'd':
        b_size = 8
    elif type == 'f':
        b_size = 4
    elif type == 'B':
        b_size = 1
    else:
        raise ValueError("Type not supported");

    endian_flag = '>'
    if little_endian:
        endian_flag = '<'

    list_m = []
    bytes_unpacked = 0
    for ii in range(size[1]):
        nbytes = b_size*size[0]
        list_m.append(struct.unpack('>'+type*size[0], buffer[bytes_unpacked:bytes_unpacked+nbytes]))
        bytes_unpacked += nbytes

    return np.array(list_m)

## \brief Pack a big endian numpy array into a bytearray
#
# Works for 1D and 2D matrices
#
# Element types supported:
# d = double
# f = float
# B = uint8
def pack_matrix(type, mat, buffer, offset=0, little_endian=False):
    if type == 'd':
        b_size = 8
    elif type == 'f':
        b_size = 4
    elif type == 'B':
        b_size = 1
    else:
        raise ValueError("Type not supported");

    endian_flag = '>'
    if little_endian:
        endian_flag = '<'

    bytes_packed = offset
    if len(mat.shape)==1:
       for jj in range(mat.shape[0]):
            struct.pack_into(endian_flag+type, buffer, bytes_packed, mat[jj])
            bytes_packed += b_size
    else:
        for ii in range(mat.shape[0]):
            for jj in range(mat.shape[1]):
                struct.pack_into(endian_flag+type, buffer, bytes_packed, mat[ii, jj])
                bytes_packed += b_size

    return

class Landmark:

    def __init__(self, lmk_file):
        with open(lmk_file, 'rb') as fp:
            file_data = fp.read()

        # Comment field not read into memory 
        # Skip first 32 chars
        self.lmk_id =  file_data[32:64]
        bytes_unpacked = 64
        
        [self.BODY, self.num_cols, self.num_rows] = struct.unpack('>iii', file_data[bytes_unpacked:bytes_unpacked+3*4])
        bytes_unpacked += 3*4

        self.num_pixels = self.num_cols*self.num_rows

        [self.anchor_col, self.anchor_row, self.resolution] = struct.unpack('>ddd', file_data[bytes_unpacked:bytes_unpacked+3*8])
        bytes_unpacked += 3*8

        self.anchor_point = np.array(struct.unpack('>ddd', file_data[bytes_unpacked:bytes_unpacked+3*8]))
        bytes_unpacked += 3*8
        
        self.mapRworld = unpack_matrix('d', [3, 3], file_data[bytes_unpacked:])
        bytes_unpacked += (3*3)*8

        self.srm = unpack_matrix('B', [self.num_cols, self.num_rows], file_data[bytes_unpacked:])
        bytes_unpacked += (self.num_pixels)*1

        self.ele = unpack_matrix('f', [self.num_cols, self.num_rows], file_data[bytes_unpacked:])
        bytes_unpacked += (self.num_pixels)*4

    def save(self, lmk_file):
        version = b'#! LVS Map v3.0'
        version += b'0'*(32-len(version))

        size = 196 + (self.num_pixels)*1 + (self.num_pixels)*4
        file_data = bytearray(size)
        
        file_data[0:32] = version
        file_data[32:64] = self.lmk_id

        bytes_packed = 64
        struct.pack_into('>iii', file_data, bytes_packed, self.BODY, self.num_cols, self.num_rows)
        bytes_packed += 3*4

        struct.pack_into('>ddd', file_data, bytes_packed, self.anchor_col, self.anchor_row, self.resolution)
        bytes_packed += 3*8

        pack_matrix('d', self.anchor_point, file_data, bytes_packed)
        bytes_packed += 3*8

        pack_matrix('d', self.mapRworld, file_data, bytes_packed)
        bytes_packed += (3*3)*8

        pack_matrix('B', self.srm, file_data, bytes_packed)
        bytes_packed += (self.num_pixels)*1

        pack_matrix('f', self.ele, file_data, bytes_packed)
        bytes_packed += (self.num_pixels)*4

        with open(lmk_file, 'wb') as fp:
            fp.write(file_data)

    def save_legacy_little_endian(self, lmk_file, lat=0.0, lon=0.0, radius=0.0):
        size = (4*4) + (6*8) + (3*8) + (3*2*8) + (3*2*8) + (3*3*8) + (3*8) + (4*8) + (self.num_pixels)*1 + (self.num_pixels)*4
        file_data = bytearray(size)

        bytes_packed = 0
        # BODY, lmk_id (integer), num_cols, num_rows
        struct.pack_into('<iiii', file_data, bytes_packed, self.BODY, 1, self.num_cols, self.num_rows)
        bytes_packed += 4*4

        # anchor_col, anchor_row, lat, lon, radius, resolution
        struct.pack_into('<dddddd', file_data, bytes_packed, self.anchor_col, self.anchor_row, lat, lon, radius, self.resolution)
        bytes_packed += 6*8

        # anchor_point (x, y, z)
        struct.pack_into('<ddd', file_data, bytes_packed, *self.anchor_point)
        bytes_packed += 3*8

        # Derived matrix: col_row2mapxy (3x2 matrix)
        col_row2mapxy = np.array([
            [self.resolution, 0.0, -self.resolution * self.anchor_col],
            [0.0, -self.resolution, self.resolution * self.anchor_row]
        ])
        struct.pack_into('<' + 'd'*6, file_data, bytes_packed, *col_row2mapxy.flatten())
        bytes_packed += (3*2)*8

        # Derived matrix: mapxy2col_row (3x2 matrix)
        mapxy2col_row = np.array([
            [1.0/self.resolution, 0.0, self.anchor_col],
            [0.0, -1.0/self.resolution, self.anchor_row]
        ])
        struct.pack_into('<' + 'd'*6, file_data, bytes_packed, *mapxy2col_row.flatten())
        bytes_packed += (3*2)*8

        # mapRworld (3x3 matrix)
        pack_matrix('d', self.mapRworld, file_data, bytes_packed, little_endian=True)
        bytes_packed += (3*3)*8

        # Derived vector: map_normal_vector (3 doubles)
        map_normal_vector = np.array([
            self.mapRworld[0,2],
            self.mapRworld[1,2],
            self.mapRworld[2,2]
        ])
        struct.pack_into('<' + 'd'*3, file_data, bytes_packed, *map_normal_vector)
        bytes_packed += 3*8

        # Derived vector: map_plane_params (4 doubles)
        dot_product = np.dot(map_normal_vector, self.anchor_point)
        map_plane_params = np.zeros(4)
        map_plane_params[:3] = map_normal_vector
        map_plane_params[3] = -dot_product
        struct.pack_into('<' + 'd'*4, file_data, bytes_packed, *map_plane_params)
        bytes_packed += 4*8

        # srm matrix (B uint8)
        pack_matrix('B', self.srm, file_data, bytes_packed, little_endian=True)
        bytes_packed += (self.num_pixels)*1

        # ele matrix (f float)
        pack_matrix('f', self.ele, file_data, bytes_packed, little_endian=True)
        bytes_packed += (self.num_pixels)*4

        with open(lmk_file, 'wb') as fp:
            fp.write(file_data)


    def __eq__(self, other):
        if isinstance(other, Landmark):
            return self.BODY == other.BODY and \
                self.lmk_id == other.lmk_id and \
                self.num_cols == other.num_cols and \
                self.num_rows == other.num_rows and \
                self.anchor_col == other.anchor_col and \
                self.anchor_row == other.anchor_row and \
                self.resolution == other.resolution and \
                np.allclose(self.anchor_point, other.anchor_point) and \
                np.allclose(self.mapRworld, other.mapRworld, rtol=0, atol=1e-4) and \
                np.allclose(self.srm, other.srm) and \
                np.allclose(self.ele, other.ele)
        return NotImplemented

    def approx_equal(self, other, elevation_tolerance):
        if isinstance(other, Landmark):
            return self.BODY == other.BODY and \
                self.lmk_id == other.lmk_id and \
                self.num_cols == other.num_cols and \
                self.num_rows == other.num_rows and \
                self.anchor_col == other.anchor_col and \
                self.anchor_row == other.anchor_row and \
                self.resolution == other.resolution and \
                np.allclose(self.anchor_point, other.anchor_point) and \
                np.allclose(self.mapRworld, other.mapRworld, rtol=0, atol=1e-4) and \
                np.allclose(self.srm, other.srm) and \
                np.allclose(self.ele, other.ele, rtol=0, atol=elevation_tolerance)
        return NotImplemented

    def assess_equality(self, other):
        if self == other :
            print("Objects are equal")
        elif isinstance(other, Landmark):
            if(self.BODY != other.BODY):
                print("self.BODY = {} other.BODY = {}", self.BODY, other.BODY)
            if(self.lmk_id != other.lmk_id):
                print("self.lmk_id = {} other.lmk_id = {}", self.lmk_id, other.lmk_id)
            if(self.num_cols != other.num_cols):
                print("self.num_cols = {} other.num_cols = {}", self.num_cols, other.num_cols)
            if(self.num_rows != other.num_rows):
                print("self.num_rows = {} other.num_rows = {}", self.num_rows, other.num_rows)
            if(self.anchor_col != other.anchor_col):
                print("self.anchor_col = {} other.anchor_col = {}", self.anchor_col, other.anchor_col)
            if(self.anchor_row != other.anchor_row):
                print("self.anchor_row = {} other.anchor_row = {}", self.anchor_row, other.anchor_row)
            if(self.resolution != other.resolution):
                print("self.resolution = {} other.resolution = {}", self.resolution, other.resolution)
        
            if(not np.allclose(self.anchor_point, other.anchor_point)):
                print("self.anchor_point != other.anchor_point")
            if(not np.allclose(self.mapRworld, other.mapRworld, rtol=0, atol=1e-4)):
                print("self.mapRworld != other.mapRworld")
            if(not np.allclose(self.srm, other.srm)):
                print("self.srm != other.srm")
            if(not np.allclose(self.ele, other.ele)):
                print("self.ele != other.ele")
        else:
            return NotImplemented
    
    
    def __str__(self):
        str = ("LMK_BODY {}".format(self.BODY) + "\n" +
            "LMK_ID {}".format(self.lmk_id) + "\n" +
            "LMK_SIZE {} {}".format(self.num_cols, self.num_rows) + "\n" +
            "LMK_RESOLUTION {}".format(self.resolution) + "\n" +
            "LMK_ANCHOR_POINT {} {} {}".format(self.anchor_point[0], self.anchor_point[1], self.anchor_point[2]) + "\n" +
            "LMK_ANCHOR_PIXEL {} {}".format(self.anchor_col, self.anchor_row) + "\n" +
            "LMK_WORLD_2_MAP_ROT {} {} {}".format(self.mapRworld[0][0], self.mapRworld[0][1], self.mapRworld[0][2]) + "\n" +
            "LMK_WORLD_2_MAP_ROT {} {} {}".format(self.mapRworld[1][0], self.mapRworld[1][1], self.mapRworld[1][2]) + "\n" +
            "LMK_WORLD_2_MAP_ROT {} {} {}".format(self.mapRworld[2][0], self.mapRworld[2][1], self.mapRworld[2][2]) + "\n" )       
        return str
    

## Test for read and write. 
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    gt = Landmark("tests/gold_standard_data/equal_rectangular_WY.lmk")
    gt.save('tests/python_save.lmk')
    test = Landmark('tests/python_save.lmk')

    assert(gt==test)
    print("Success")