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

#ifndef _HM01B0_PLATFORM_H_

// Conditionally Include Platforms
#if defined(AM_PART_APOLLO3) &&   \
    (defined(ARDUINO_SFE_EDGE) || \
     defined(ARDUINO_AM_AP3_SFE_BB_ARTEMIS_MICROMOD))
#include "platforms/apollo3/include/hm01b0_platform_apollo3.h"
#else
//  #warning "No platform implementation - falling back to generic Arduino interface. Performance not guaranteed."
//#include "platforms/arduino_generic/include/hm01b0_platform_arduino_generic.h"
#include "platforms/apollo3/include/hm01b0_platform_apollo3.h"
#endif // platform inclusion

#ifndef HM01B0_READ_HSYNC
#warning HM01B0_READ_HSYNC undefined!
#define HM01B0_READ_HSYNC 0
#endif

#ifndef HM01B0_READ_PCLK
#warning HM01B0_READ_PCLK undefined!
#define HM01B0_READ_PCLK 0
#endif

#ifndef HM01B0_READ_BYTE
#warning HM01B0_READ_BYTE undefined!
#define HM01B0_READ_BYTE 0
#endif

#endif // _HM01B0_PLATFORM_H_