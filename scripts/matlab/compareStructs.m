 
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
 

function isEqual = compareStructs(s1, s2)
% [isEqual] = compareStructs   check the equality of two structs by comparing all their member variables  
  if isstruct(s1) && isstruct(s2)
    f1 = fieldnames(s1);
    f2 = fieldnames(s2);

    if length(f1) ~= length(f2)
      isEqual = false;
      return;
    end

    for i = 1:length(f1)
      if ~isfield(s2, f1{i})
        isEqual = false;
        fprintf("No field %s\n", f1{i});
        return;
      end

      if ~compareStructs(s1.(f1{i}), s2.(f1{i}))
        isEqual = false;
        fprintf("Field %s not equal \n", f1{i});
        return;
      end
    end

    isEqual = true;
  else
    isEqual = isequal(s1, s2);
  end
end