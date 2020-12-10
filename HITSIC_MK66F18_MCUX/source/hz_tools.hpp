
/*
 * hz_tools.h
 *
 *  Created on: 2020年11月22日
 *      Author: Skywalker
 */

#ifndef HZ_TOOLS_H_
#define HZ_TOOLS_H_

#include <stdio.h>
#include <stdlib.h>
//#include "cv.h"
//#include "highgui.h"
#include <math.h>
#include"my_control.hpp"

#define GrayScale 256


void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
void fxyk(uint8_t* y, uint8_t x_Start, uint8_t x_End, float* k, float* b);
int Max(int x, int y);
int Min(int x, int y);
float f_Max(float x, float y);
float f_Min(float x, float y);
float abs_float(float n);
uint8_t myOtsu(uint8_t* image);

#endif /* HZ_TOOLS_H_ */
