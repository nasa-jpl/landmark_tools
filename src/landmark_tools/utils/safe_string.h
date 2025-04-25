#ifndef SAFE_STRING_H
#define SAFE_STRING_H

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>

// Safe printf wrapper that ensures format string width matches buffer size
#define SAFE_PRINTF(size, fmt, ...) \
    do { \
        char temp[size]; \
        snprintf(temp, size, fmt __VA_OPT__(,) __VA_ARGS__); \
        printf("%.*s", (int)size, temp); \
    } while(0)


#define SAFE_FPRINTF(fp, size, fmt, ...) \
    do { \
        char temp[size]; \
        snprintf(temp, size, fmt __VA_OPT__(,) __VA_ARGS__); \
        fprintf(fp, "%.*s", (int)size, temp); \
    } while(0)

#endif // SAFE_STRING_H 