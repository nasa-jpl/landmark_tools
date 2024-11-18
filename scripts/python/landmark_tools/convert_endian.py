
# #
#  \file   `convert_endian.py`
#  \author Cecilia Mauceri
#  \brief  Load little endian and save big endian landmark files in python
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
from landmark import Landmark

body_conversion = {'Earth':0, 'Moon':1, 'Mars':0};

## \brief Unpack a big endian binary matrix into a numpy array
#
# Element types supported:
# d = double
# f = float
# B = uint8
def unpack_little_endian_matrix(type, size, buffer):
    if type == 'd':
        b_size = 8
    elif type == 'f':
        b_size = 4
    elif type == 'B':
        b_size = 1
    else:
        raise ValueError("Type not supported");

    list_m = []
    bytes_unpacked = 0
    for ii in range(size[1]):
        nbytes = b_size*size[0]
        list_m.append(struct.unpack('<'+type*size[0], buffer[bytes_unpacked:bytes_unpacked+nbytes]))
        bytes_unpacked += nbytes

    return np.array(list_m)

class LegacyLittleEndianLandmark(Landmark):

    def __init__(self, lmk_file, body=None):
        with open(lmk_file, 'rb') as fp:
            file_data = fp.read()

        bytes_unpacked = 0

        [self.BODY, self.lmk_id, self.num_cols, self.num_rows] = struct.unpack('<iiii', file_data[bytes_unpacked:bytes_unpacked+4*4])
        bytes_unpacked += 4*4

        id = bytearray('{}'.format(self.lmk_id).encode('utf-8'))
        id += b'0'*(32-len(id))
        self.lmk_id = id

        if body is not None:
            self.BODY = body_conversion[body]

        self.num_pixels = self.num_cols*self.num_rows

        [self.anchor_col, self.anchor_row, lat, long, radius, self.resolution] = struct.unpack('<dddddd', file_data[bytes_unpacked:bytes_unpacked+6*8])
        bytes_unpacked += 6*8

        self.anchor_point = np.array(struct.unpack('<ddd', file_data[bytes_unpacked:bytes_unpacked+3*8]))
        bytes_unpacked += 3*8

        #skipped derived matrices
        bytes_unpacked += (3*2)*8
        bytes_unpacked += (3*2)*8
        
        self.mapRworld = unpack_little_endian_matrix('d', [3, 3], file_data[bytes_unpacked:])
        bytes_unpacked += (3*3)*8

        #skipped derived matrices
        bytes_unpacked += (3)*8
        bytes_unpacked += (4)*8

        self.srm = unpack_little_endian_matrix('B', [self.num_cols, self.num_rows], file_data[bytes_unpacked:])
        bytes_unpacked += (self.num_pixels)*1

        self.ele = unpack_little_endian_matrix('f', [self.num_cols, self.num_rows], file_data[bytes_unpacked:])
        bytes_unpacked += (self.num_pixels)*4
    

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert a legacy little endian landmark file to big endian")
    parser.add_argument("infile", help="input filepath")
    parser.add_argument("outfile", type=str, help="output filepath")
    parser.add_argument("planet", type=str, help="<Earth, Moon, Mars>")
    args = parser.parse_args()

    L = LegacyLittleEndianLandmark(args.infile, args.planet)
    L.save(args.outfile)

    print("Success")