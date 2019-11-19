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

// #define DEMO_HM01B0_TEST_MODE_ENABLE
#define DEMO_HM01B0_FRAMEBUFFER_DUMP_ENABLE

void framebuffer_dump(uint8_t *pui8Buffer, uint32_t ui32BufferLen);

HM01B0::HM01B0(hm01b0_cfg_t _cfg){
  cfg = _cfg;
}

hm01b0_status_e HM01B0::begin( void ){
  hm01b0_status_e retval = HM01B0_ERR_OK;

  uint16_t    ui16ModelId = 0x0000;

  // hm01b0_power_up(&cfg);
  delay(1);
  hm01b0_mclk_enable(&cfg);
  delay(1);
  hm01b0_init_if(&cfg);

  hm01b0_get_modelid(&cfg, &ui16ModelId);
  Serial.printf("HM01B0 Model ID 0x%04X\n", ui16ModelId);

  hm01b0_init_system(&cfg, (hm_script_t *)sHM01B0InitScript, sizeof(sHM01B0InitScript)/sizeof(hm_script_t));

  Serial.printf("System initialized\n");
  Serial.printf("Calibrating Auto Exposure\n");

#ifdef DEMO_HM01B0_TEST_MODE_ENABLE
  am_util_stdio_printf("HM01B0 Enable walking 1s test mode\n");
  hm01b0_test_walking1s(&cfg);
#else
  hm01b0_cal_ae(&cfg, 10, frameBuffer, sizeof(frameBuffer), &aecfg);
#endif

Serial.printf("AE Calibrated\n");

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
  uint8_t     ui8Mode     = 0xFF;
  uint32_t    ui32Err     = HM01B0_ERR_OK;

  Serial.print("Say cheese!\n");

  hm01b0_ae_cfg_t sAECfg;
  hm01b0_get_mode(&cfg, &ui8Mode);
  Serial.printf("HM01B0 current mode %d\n", ui8Mode);

  ui32Err = hm01b0_get_ae(&cfg, &sAECfg);
  hm01b0_cmd_update(&cfg);
  hm01b0_set_mode(&cfg, HM01B0_REG_MODE_SELECT_STREAMING_NFRAMES, 1);
  hm01b0_blocking_read_oneframe(&cfg, frameBuffer, sizeof(frameBuffer));

  Serial.printf("AE convergance(0x%02X) TargetMean 0x%02X, ConvergeInTh 0x%02X, AEMean 0x%02X\n", ui32Err, sAECfg.ui8AETargetMean, sAECfg.ui8ConvergeInTh, sAECfg.ui8AEMean);

#ifdef DEMO_HM01B0_TEST_MODE_ENABLE
  uint32_t    mismatches  = 0;
  mismatches = hm01b0_test_walking1s_check_data_sanity(frameBuffer, sizeof(frameBuffer));
  Serial.printf("Self-test mismatches: 0x%08X\n", mismatches);
#endif

#ifdef DEMO_HM01B0_FRAMEBUFFER_DUMP_ENABLE
  framebuffer_dump(frameBuffer, sizeof(frameBuffer));
#endif
  memset(frameBuffer, 0x00, sizeof(frameBuffer));


  return retval;
}





// frame buffer dump
void framebuffer_dump(uint8_t *pui8Buffer, uint32_t ui32BufferLen)
{
    Serial.printf("+++ frame +++");

    for (uint32_t ui32Idx = 0; ui32Idx < ui32BufferLen; ui32Idx++)
    {
        if ((ui32Idx & 0xF) == 0x00)
        {
            Serial.printf("\n0x%08X ", ui32Idx);
            // // this delay is to let itm have time to flush out data.
            // am_util_delay_ms(1);
        }

        Serial.printf("%02X ", *(pui8Buffer + ui32Idx));
    }

    Serial.printf("\n--- frame ---\n");
    // am_util_delay_ms(1);
}