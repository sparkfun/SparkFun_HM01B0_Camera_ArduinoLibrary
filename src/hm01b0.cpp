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

HM01B0::HM01B0(hm01b0_cfg_t _cfg){
  cfg = _cfg;
}

hm01b0_status_e HM01B0::begin( void ){
  hm01b0_status_e retval = HM01B0_ERR_OK;

  uint32_t    ui32Err     = HM01B0_ERR_OK;
  uint16_t    ui16ModelId = 0x0000;
  uint8_t     ui8Mode     = 0xFF;

  // hm01b0_power_up(&cfg);
  delay(1);
  hm01b0_mclk_enable(&cfg);
  delay(1);
  hm01b0_init_if(&cfg);

  hm01b0_get_modelid(&cfg, &ui16ModelId);
  Serial.printf("HM01B0 Model ID 0x%04X\n", ui16ModelId);

  hm01b0_init_system(&cfg, (hm_script_t *)sHM01B0InitScript, sizeof(sHM01B0InitScript)/sizeof(hm_script_t));

#ifdef DEMO_HM01B0_TEST_MODE_ENABLE
  am_util_stdio_printf("HM01B0 Enable walking 1s test mode\n");
  hm01b0_test_walking1s(&cfg);
#else
  hm01b0_cal_ae(&cfg, 10, frameBuffer, sizeof(frameBuffer), &aecfg);
#endif

  return retval;
}

hm01b0_status_e HM01B0::end( void ){
  hm01b0_status_e retval = HM01B0_ERR_OK;
  
  hm01b0_deinit_if(&cfg);
  hm01b0_mclk_disable(&cfg);
  // hm01b0_power_down(&cfg);

  return retval;
}


hm01b0_status_e HM01B0::capture( void ){
  hm01b0_status_e retval = HM01B0_ERR_OK;

  Serial.print("Say cheese!\n");

  return retval;
}