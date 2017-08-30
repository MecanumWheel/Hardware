#include "JSON.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// pStr must be a pointer to a dynamically allocated
// array or to a null
static void stringAppend(char **pStr, size_t *pLen,
						 const char toAppend[])
{
	*pLen += strlen(toAppend);
	if (!*pStr)
		*pStr = (char*)calloc(*pLen + 1, sizeof(char)); // +1 for the null-terminator
	else
		*pStr = (char*)realloc(*pStr, (*pLen + 1) * sizeof(char));
	strcat(*pStr, toAppend);
}

//static void stringAppendNull(char **pStr, size_t *pLen,
//                             const size_t num)
//{
//	*pLen += num;
//	if (!*pStr)
//		*pStr = (char*)calloc(*pLen + 1, sizeof(char)); // +1 for the null-terminator
//	else
//		*pStr = (char*)realloc(*pStr, (*pLen + 1) * sizeof(char));
//}

static void jsonAddKey(char **pStr, size_t *pLen,
                       const char *key)
{
    stringAppend(pStr, pLen, "\"");
	stringAppend(pStr, pLen, key);
	stringAppend(pStr, pLen, "\":");
}

static void jsonAddStringValue(char **pStr, size_t *pLen,
                               const char *str)
{
    stringAppend(pStr, pLen, "\"");
	stringAppend(pStr, pLen, str);
	stringAppend(pStr, pLen, "\",");
}

static void jsonAddIntValue(char **pStr, size_t *pLen,
                            const int32_t value)
{
    char num[12] = {0}; // The longest int: approx. -2 bln (11 chars) + '\0'
	sprintf(num, "%d", value);
	stringAppend(pStr, pLen, num);
	stringAppend(pStr, pLen, ",");
}

static void jsonAddFloatValue(char **pStr, size_t *pLen,
                              const float value)
{
    char num[18] = {0};
	const float expThresh = 10e6;
	if (abs(value) > expThresh)
		sprintf(num, "%.6e", value);
	else
		sprintf(num, "%.6f", value);
	stringAppend(pStr, pLen, num);
	stringAppend(pStr, pLen, ",");
}

static void jsonBeginArray(char **pStr, size_t *pLen)
{
    stringAppend(pStr, pLen, "[");
}

static void jsonEndArray(char **pStr, size_t *pLen)
{
    (*pStr)[*pLen - 1] = ']';
    stringAppend(pStr, pLen, ",");
}
                          
void jsonStart(char **pStr, size_t *pLen)
{
	stringAppend(pStr, pLen, "{");
}

void jsonEnd(char **pStr, size_t *pLen)
{
    (*pStr)[*pLen - 1] = '}';
    stringAppend(pStr, pLen, ",");
}

void jsonAddString(char **pStr, size_t *pLen, char key[], char value[])
{
	jsonAddKey(pStr, pLen, key);
    jsonAddStringValue(pStr, pLen, value);
}

void jsonAddInt(char **pStr, size_t *pLen, char key[], int32_t value)
{
	jsonAddKey(pStr, pLen, key);
    jsonAddIntValue(pStr, pLen, value);
}

void jsonAddFloat(char **pStr, size_t *pLen, char key[], float value)
{
	jsonAddKey(pStr, pLen, key);
    jsonAddFloatValue(pStr, pLen, value);
}

void jsonAddStringArray(char **pStr, size_t *pLen,
                        char *key,
                        const char **strArr, size_t qty)
{
    jsonAddKey(pStr, pLen, key);
    jsonBeginArray(pStr, pLen);
    for (size_t i = 0; i < qty; i++)
        jsonAddStringValue(pStr, pLen, strArr[i]);
    jsonEndArray(pStr, pLen);
}

void jsonAddIntArray(char **pStr, size_t *pLen,
                     char *key,
                     const int32_t *intArr, size_t qty)
{
    jsonAddKey(pStr, pLen, key);
    jsonBeginArray(pStr, pLen);
    for (size_t i = 0; i < qty; i++)
        jsonAddIntValue(pStr, pLen, intArr[i]);
    jsonEndArray(pStr, pLen);
}

void jsonAddFloatArray(char **pStr, size_t *pLen,
                       char *key,
                       const float *floatArr, size_t qty)
{
    jsonAddKey(pStr, pLen, key);
    jsonBeginArray(pStr, pLen);
    for (size_t i = 0; i < qty; i++)
        jsonAddFloatValue(pStr, pLen, floatArr[i]);
    jsonEndArray(pStr, pLen);
}
