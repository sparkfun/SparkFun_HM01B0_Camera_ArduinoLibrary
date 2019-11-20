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

#ifndef _HM01B0_H_
#define _HM01B0_H_

#include "Arduino.h"
#include "hm01b0_c/include/hm01b0_c.h"

// #define 

extern hm01b0_cfg_t hm01b0_cfg; // This must be provided by the platform

class HM01B0 {
private:

public:
protected:
  hm01b0_cfg_t    cfg = {0};
  
public:
  hm01b0_status_e       status = HM01B0_ERR_OK;
  hm01b0_status_e       aeConvergenceStatus = HM01B0_ERR_OK;
  static const size_t   frameBufferSize = (HM01B0_PIXEL_X_NUM * HM01B0_PIXEL_Y_NUM);
  uint8_t               frameBuffer[frameBufferSize] = {0};
  hm01b0_ae_cfg_t aecfg = {0};

  HM01B0(hm01b0_cfg_t _cfg = hm01b0_cfg);

  hm01b0_status_e begin( void );
  hm01b0_status_e calibrateAutoExposure( void );
  hm01b0_status_e enableTestMode( void );
  uint32_t        countTestMismatches( void );
  hm01b0_status_e capture( void );
  void            getAutoExposureStatus( void );
  hm01b0_status_e end( void );
};



#endif // _HM01B0_H_
