#include <stdio.h>
#include <stdlib.h>
#include "landmark_tools/utils/safe_string.h"
#include "landmark_tools/utils/write_array.h"

int write_data_to_file(const char* filename, const void* data, size_t element_size, size_t num_elements) {
    FILE* fp = fopen(filename, "wb");
    if (!fp) {
        SAFE_FPRINTF(stderr, 512, "Error: Could not open file '%s' for writing\n", filename);
        return -1;
    }
    
    size_t written = fwrite(data, element_size, num_elements, fp);
    fclose(fp);
    
    if (written != num_elements) {
        SAFE_FPRINTF(stderr, 512, "Error: Failed to write all data to file '%s'\n", filename);
        return -1;
    }
    
    return 0;
}