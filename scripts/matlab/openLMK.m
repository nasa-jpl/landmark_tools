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

function lmk = openLMK(lmk_file)
% [lmk] = openLMK    Read lmk_file into a struct
lmk = struct();

fileID = fopen(lmk_file,'r');
comment = fread(fileID, 32, 'uint8');
lmk.lmk_id = fread(fileID, 32, 'uint8');

lmk.BODY = fread(fileID, 1, 'int', 0, 'b');
lmk.num_cols = fread(fileID, 1, 'int', 0, 'b');
lmk.num_rows = fread(fileID, 1, 'int', 0, 'b');

lmk.anchor_col = fread(fileID, 1, 'double', 0, 'b');
lmk.anchor_row = fread(fileID, 1, 'double', 0, 'b');
lmk.resolution = fread(fileID, 1, 'double', 0, 'b');

lmk.anchor_point = fread(fileID, 3, 'double', 0, 'b');
lmk.mapRworld = fread(fileID, [3, 3], 'double', 0, 'b')';

lmk.srm = fread(fileID, [lmk.num_cols, lmk.num_rows], "uint8")';
lmk.ele = fread(fileID, [lmk.num_cols, lmk.num_rows], "float", 0, 'b')';

fclose(fileID);

end