%  Copyright 2024 California Institute of Technology
%
%  Licensed under the Apache License, Version 2.0 (the "License");
%  you may not use this file except in compliance with the License.
%  You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
%  Unless required by applicable law or agreed to in writing, software
%  distributed under the License is distributed on an "AS IS" BASIS,
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%  See the License for the specific language governing permissions and
%  limitations under the License.

function writeLMK(lmk_file, lmk)
% writeLMK    Write a lmk struct to lmk_file

fileID = fopen(lmk_file,'w');
version = zeros(32, 1, 'uint8');
version(1:15) = '#! LVS Map v3.0';
fwrite(fileID, version, 'uint8');
fwrite(fileID, lmk.lmk_id, 'uint8');
fwrite(fileID, lmk.BODY, 'int', 0, 'b');
fwrite(fileID, lmk.num_cols, 'int', 0, 'b');
fwrite(fileID, lmk.num_rows, 'int', 0, 'b');

fwrite(fileID, lmk.anchor_col, 'double', 0, 'b');
fwrite(fileID, lmk.anchor_row, 'double', 0, 'b');
fwrite(fileID, lmk.resolution, 'double', 0, 'b');

fwrite(fileID, lmk.anchor_point, 'double', 0, 'b');
fwrite(fileID, lmk.mapRworld', 'double', 0, 'b');

fwrite(fileID, lmk.srm', "uint8");
fwrite(fileID, lmk.ele', "float", 0, 'b');

fclose(fileID);

end