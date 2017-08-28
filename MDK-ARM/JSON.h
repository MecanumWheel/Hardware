#pragma once

#include <stdint.h>

#include "JSON.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// pStr must be a pointer to a dynamically allocated
// array or a pointer to a null
                          
void jsonStart(char **pStr, size_t *pLen);

void jsonEnd(char **pStr, size_t *pLen);

void jsonAddString(char **pStr, size_t *pLen, char key[], char value[]);

void jsonAddInt(char **pStr, size_t *pLen, char key[], int32_t value);

void jsonAddFloat(char **pStr, size_t *pLen, char key[], float value);

void jsonAddStringArray(char **pStr, size_t *pLen,
                        char *key,
                        const char **strArr, size_t qty);

void jsonAddIntArray(char **pStr, size_t *pLen,
                     char *key,
                     const int32_t *intArr, size_t qty);
					 
void jsonAddFloatArray(char **pStr, size_t *pLen,
                       char *key,
                       const float *floatArr, size_t qty);
