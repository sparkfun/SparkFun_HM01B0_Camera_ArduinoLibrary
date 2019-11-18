/*
  Saving camera stills over UART on SparkFun Edge board
  By: Owen Lyke
  SparkFun Electronics
  Date: November 18th 2019
  This example code is in the public domain

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15170

  This example shows how to use the HM01B0 camera. It will stream frames captured by the 
  camera over the UART. (You can choose the port and baud rate - Serial and 921600 by default)

  Record the stream of data (capture as many frames as you like) and then use the 'raw2bmp.py'
  tool to convert the stream to a set of still images
*/

// Stanard includes
#include <stdio.h>
#include <string.h>
#include "HM01B0.h"

// Camera mode configuration
//  - you could use a different SW configuration created with the 'himax_script_converter.py' utility
#include "HM01B0_RAW8_QVGA_8bits_lsb_5fps.h"

// Camera interface configuration
//  - you can modify this file to use a custom board!
#include "platform.h"             


// Options
#define SERIAL_PORT Serial
#define BAUD_RATE 921600
//#define DEMO_HM01B0_TEST_MODE_ENABLE
#define DEMO_HM01B0_FRAMEBUFFER_DUMP_ENABLE

//
// Global variables
static hm01b0_cfg_t s_HM01B0Cfg = {0};
static uint8_t s_ui8FrameBuffer[HM01B0_PIXEL_X_NUM * HM01B0_PIXEL_Y_NUM];
hm01b0_ae_cfg_t sAECfg;
uint32_t    ui32Err     = HM01B0_ERR_OK;
uint16_t    ui16ModelId = 0x0000;
uint8_t     ui8Mode     = 0xFF;

//
// Forward declarations
void boost_mode_enable(bool bEnable);
void framebuffer_dump(uint8_t *pui8Buffer, uint32_t ui32BufferLen);


void setup() {
  SERIAL_PORT.begin(BAUD_RATE);

  //
  // Turn on the camera regulator
  //
  am_hal_gpio_pinconfig(AM_BSP_GPIO_CAMERA_HM01B0_DVDDEN, g_AM_HAL_GPIO_OUTPUT_12);
  am_hal_gpio_output_set(AM_BSP_GPIO_CAMERA_HM01B0_DVDDEN);

  init_camera_cfg(); // Setup the camera configuration structure

  //
  // Clear the terminal and print the banner.
  SERIAL_PORT.printf("HM01B0 Demo\n");
  SERIAL_PORT.printf(" SCL:\tPin %d\n",  s_HM01B0Cfg.ui8PinSCL);
  SERIAL_PORT.printf(" SDA:\tPin %d\n",  s_HM01B0Cfg.ui8PinSDA);
  SERIAL_PORT.printf(" MCLK:\tPin %d\n",  s_HM01B0Cfg.ui32CTimerOutputPin);
  SERIAL_PORT.printf(" VSYNC:\tPin %d\n", s_HM01B0Cfg.ui8PinVSYNC);
  SERIAL_PORT.printf(" HSYNC\tPin %d\n",  s_HM01B0Cfg.ui8PinHSYNC);
  SERIAL_PORT.printf(" PCLK:\tPin %d\n",  s_HM01B0Cfg.ui8PinPCLK);
  SERIAL_PORT.printf(" D0:\tPin %d\n",    s_HM01B0Cfg.ui8PinD0);
  SERIAL_PORT.printf(" D1:\tPin %d\n",    s_HM01B0Cfg.ui8PinD1);
  SERIAL_PORT.printf(" D2:\tPin %d\n",    s_HM01B0Cfg.ui8PinD2);
  SERIAL_PORT.printf(" D3:\tPin %d\n",    s_HM01B0Cfg.ui8PinD3);
  SERIAL_PORT.printf(" D4:\tPin %d\n",    s_HM01B0Cfg.ui8PinD4);
  SERIAL_PORT.printf(" D5:\tPin %d\n",    s_HM01B0Cfg.ui8PinD5);
  SERIAL_PORT.printf(" D6:\tPin %d\n",    s_HM01B0Cfg.ui8PinD6);
  SERIAL_PORT.printf(" D7:\tPin %d\n",    s_HM01B0Cfg.ui8PinD7);

  am_hal_interrupt_master_enable();

  boost_mode_enable(true);
  hm01b0_power_up(&s_HM01B0Cfg);
  am_util_delay_ms(1);

  hm01b0_mclk_enable(&s_HM01B0Cfg);
  am_util_delay_ms(1);

  hm01b0_init_if(&s_HM01B0Cfg);
  hm01b0_get_modelid(&s_HM01B0Cfg, &ui16ModelId);
  hm01b0_init_system(&s_HM01B0Cfg, (hm_script_t *)sHM01B0InitScript, sizeof(sHM01B0InitScript)/sizeof(hm_script_t));
  SERIAL_PORT.printf("HM01B0 Model ID 0x%04X\n", ui16ModelId);

#ifdef DEMO_HM01B0_TEST_MODE_ENABLE
  SERIAL_PORT.printf("HM01B0 Enable walking 1s test mode\n");
  hm01b0_test_walking1s(&s_HM01B0Cfg);
#else
  hm01b0_cal_ae(&s_HM01B0Cfg, 10, s_ui8FrameBuffer, sizeof(s_ui8FrameBuffer));
#endif

}

void loop() {
  hm01b0_get_mode(&s_HM01B0Cfg, &ui8Mode);
  SERIAL_PORT.printf("HM01B0 current mode %d\n", ui8Mode);

  ui32Err = hm01b0_get_ae(&s_HM01B0Cfg, &sAECfg);
  SERIAL_PORT.printf("AE convergance(0x%02X) TargetMean 0x%02X, ConvergeInTh 0x%02X, AEMean 0x%02X\n", ui32Err, sAECfg.ui8AETargetMean, sAECfg.ui8ConvergeInTh, sAECfg.ui8AEMean);

  hm01b0_cmd_update(&s_HM01B0Cfg);
  hm01b0_set_mode(&s_HM01B0Cfg, HM01B0_REG_MODE_SELECT_STREAMING_NFRAMES, 1);
  hm01b0_blocking_read_oneframe(&s_HM01B0Cfg, s_ui8FrameBuffer, sizeof(s_ui8FrameBuffer));

#ifdef DEMO_HM01B0_TEST_MODE_ENABLE
  hm01b0_test_walking1s_check_data_sanity(s_ui8FrameBuffer, sizeof(s_ui8FrameBuffer), 10);
  am_util_delay_ms(3000);
#endif

#ifdef DEMO_HM01B0_FRAMEBUFFER_DUMP_ENABLE
  framebuffer_dump(s_ui8FrameBuffer, sizeof(s_ui8FrameBuffer));
#endif
  memset(s_ui8FrameBuffer, 0x00, sizeof(s_ui8FrameBuffer));

  // give some time for user to stop the external itm logging.
  am_util_delay_ms(5000);
}






//
// init_camera_cfg
void init_camera_cfg( void ){
    // i2c settings
    s_HM01B0Cfg.ui16SlvAddr                = HM01B0_DEFAULT_ADDRESS;
    s_HM01B0Cfg.eIOMMode                   = HM01B0_IOM_MODE;
    s_HM01B0Cfg.ui32IOMModule              = HM01B0_IOM_MODULE;
    s_HM01B0Cfg.sIOMCfg.eInterfaceMode     = HM01B0_IOM_MODE;
    s_HM01B0Cfg.sIOMCfg.ui32ClockFreq      = HM01B0_I2C_CLOCK_FREQ;
    s_HM01B0Cfg.pIOMHandle                 = NULL;
    s_HM01B0Cfg.ui8PinSCL                  = HM01B0_PIN_SCL;
    s_HM01B0Cfg.ui8PinSDA                  = HM01B0_PIN_SDA;

    // MCLK settings
    s_HM01B0Cfg.ui32CTimerModule           = HM01B0_MCLK_GENERATOR_MOD;
    s_HM01B0Cfg.ui32CTimerSegment          = HM01B0_MCLK_GENERATOR_SEG;
    s_HM01B0Cfg.ui32CTimerOutputPin        = HM01B0_PIN_MCLK;

    // data interface
    s_HM01B0Cfg.ui8PinD0                   = HM01B0_PIN_D0;
    s_HM01B0Cfg.ui8PinD1                   = HM01B0_PIN_D1;
    s_HM01B0Cfg.ui8PinD2                   = HM01B0_PIN_D2;
    s_HM01B0Cfg.ui8PinD3                   = HM01B0_PIN_D3;
    s_HM01B0Cfg.ui8PinD4                   = HM01B0_PIN_D4;
    s_HM01B0Cfg.ui8PinD5                   = HM01B0_PIN_D5;
    s_HM01B0Cfg.ui8PinD6                   = HM01B0_PIN_D6;
    s_HM01B0Cfg.ui8PinD7                   = HM01B0_PIN_D7;
    s_HM01B0Cfg.ui8PinVSYNC                = HM01B0_PIN_VSYNC;
    s_HM01B0Cfg.ui8PinHSYNC                = HM01B0_PIN_HSYNC;
    s_HM01B0Cfg.ui8PinPCLK                 = HM01B0_PIN_PCLK;

#ifdef HM01B0_PIN_TRIG
    s_HM01B0Cfg.ui8PinTrig                 = HM01B0_PIN_TRIG;
#endif // HM01B0_PIN_TRIG

#ifdef HM01B0_PIN_INT
    s_HM01B0Cfg.ui8PinInt                  = HM01B0_PIN_INT;
#endif // HM01B0_PIN_INT

    s_HM01B0Cfg.pfnGpioIsr                 = NULL;
};

// frame buffer dump
void framebuffer_dump(uint8_t *pui8Buffer, uint32_t ui32BufferLen)
{
  SERIAL_PORT.printf("+++ frame +++");

  for (uint32_t ui32Idx = 0; ui32Idx < ui32BufferLen; ui32Idx++)
  {
    if ((ui32Idx & 0xF) == 0x00)
    {
      SERIAL_PORT.printf("\n0x%08X ", ui32Idx);
      // this delay is to let itm have time to flush out data.
      am_util_delay_ms(1);
    }

    SERIAL_PORT.printf("%02X ", *(pui8Buffer + ui32Idx));
  }

  SERIAL_PORT.printf("\n--- frame ---\n");
  am_util_delay_ms(1);
}

// burst mode enable
void boost_mode_enable(bool bEnable){
  am_hal_burst_avail_e          eBurstModeAvailable;
  am_hal_burst_mode_e           eBurstMode;

  // Check that the Burst Feature is available.
  if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_initialize(&eBurstModeAvailable)){
    if (AM_HAL_BURST_AVAIL == eBurstModeAvailable){
      SERIAL_PORT.printf("Apollo3 Burst Mode is Available\n");
    }
    else{
      SERIAL_PORT.printf("Apollo3 Burst Mode is Not Available\n");
      while(1){};
    }
  }
  else{
    SERIAL_PORT.printf("Failed to Initialize for Burst Mode operation\n");
  }

  // Make sure we are in "Normal" mode.
  if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_disable(&eBurstMode)){
    if (AM_HAL_NORMAL_MODE == eBurstMode){
      SERIAL_PORT.printf("Apollo3 operating in Normal Mode (48MHz)\n");
    }
  }
  else{
    SERIAL_PORT.printf("Failed to Disable Burst Mode operation\n");
  }

  // Put the MCU into "Burst" mode.
  if (bEnable)
  {
    if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_enable(&eBurstMode)){
      if (AM_HAL_BURST_MODE == eBurstMode){
        SERIAL_PORT.printf("Apollo3 operating in Burst Mode (96MHz)\n");
      }
    }
    else{
      SERIAL_PORT.printf("Failed to Enable Burst Mode operation\n");
    }
  }
}