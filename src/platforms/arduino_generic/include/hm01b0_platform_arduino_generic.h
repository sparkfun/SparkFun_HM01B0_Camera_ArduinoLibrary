/*
Copyright (c) 2019 SparkFun Electronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _HM01B0_PLATFORM_ARDUINO_GENERIC_C_H_
#define _HM01B0_PLATFORM_ARDUINO_GENERIC_C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "hm01b0.h"

// todo: implement real macros for generic Arduino imlementation
#define HM01B0_READ_VSYNC     (0)
#define HM01B0_READ_HSYNC     (0)
#define HM01B0_READ_PCLK      (0)
#define HM01B0_READ_BYTE      (0)

// todo: implement an interface using generic Arduino functions

#ifdef __cplusplus
}
#endif

#endif // _HM01B0_PLATFORM_ARDUINO_GENERIC_C_H_