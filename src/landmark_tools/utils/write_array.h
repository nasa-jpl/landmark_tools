#include <stddef.h>

#ifndef WRITE_ARRAY_H
#define WRITE_ARRAY_H

/**
 * \brief Helper function to write data to a file with error checking
 * 
 * \param filename The name of the file to write
 * \param data Pointer to the data to write
 * \param element_size Size of each element in bytes
 * \param num_elements Number of elements to write
 * \return 0 on success, -1 on failure
 */
int write_data_to_file(const char* filename, const void* data, size_t element_size, size_t num_elements);

#endif /* WRITE_ARRAY_H */ 