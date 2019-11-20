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

#include "HM01B0.h"
#include "hm01b0_c/include/hm01b0_raw8_qvga_8bits_lsb_5fps.h"



void framebuffer_dump(uint8_t *pui8Buffer, uint32_t ui32BufferLen);

HM01B0::HM01B0(hm01b0_cfg_t _cfg){
  cfg = _cfg;
}

hm01b0_status_e HM01B0::begin( void ){
  uint16_t    ui16ModelId = 0x0000;

  status = hm01b0_mclk_enable(&cfg);
  if(status != HM01B0_ERR_OK){ goto fail; }

  status = hm01b0_init_if(&cfg);
  if(status != HM01B0_ERR_OK){ goto fail; }

  status = hm01b0_get_modelid(&cfg, &ui16ModelId);
  if( ui16ModelId != (uint16_t)HM01B0_MODEL_ID ){ status = HM01B0_ERR; }
  if(status != HM01B0_ERR_OK){ goto deinit; }

  hm01b0_init_system(&cfg, (hm_script_t *)sHM01B0InitScript, sizeof(sHM01B0InitScript)/sizeof(hm_script_t));

  return status;

deinit:
  hm01b0_deinit_if(&cfg);

fail:
  return status;
}

hm01b0_status_e HM01B0::calibrateAutoExposure( void ){
  status = hm01b0_cal_ae(&cfg, 10, frameBuffer, sizeof(frameBuffer), &aecfg);
  return status;
}

hm01b0_status_e HM01B0::enableTestMode( void ){
  status = hm01b0_test_walking1s(&cfg);
  return status;
}

uint32_t HM01B0::countTestMismatches( void ){
  uint32_t    mismatches  = 0;
  mismatches = hm01b0_test_walking1s_check_data_sanity(frameBuffer, sizeof(frameBuffer));
  return mismatches;
}

hm01b0_status_e HM01B0::capture( void ){
  hm01b0_status_e retval = HM01B0_ERR_OK;
  uint8_t     ui8Mode     = 0xFF;
  uint32_t    ui32Err     = HM01B0_ERR_OK;

  getAutoExposureStatus();
  hm01b0_get_mode(&cfg, &ui8Mode);
  // Serial.printf("HM01B0 current mode %d\n", ui8Mode);

  hm01b0_cmd_update(&cfg);
  hm01b0_set_mode(&cfg, HM01B0_REG_MODE_SELECT_STREAMING_NFRAMES, 1);
  hm01b0_blocking_read_oneframe(&cfg, frameBuffer, sizeof(frameBuffer));

  return retval;
}

void HM01B0::getAutoExposureStatus( void ){
  aeConvergenceStatus = hm01b0_get_ae(&cfg, &aecfg);
}

hm01b0_status_e HM01B0::end( void ){
  hm01b0_status_e retval = HM01B0_ERR_OK;

  hm01b0_deinit_if(&cfg);
  hm01b0_mclk_disable(&cfg);

  return retval;
}
