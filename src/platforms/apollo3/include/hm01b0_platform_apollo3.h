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

#ifndef _HM01B0_PLATFORM_APOLLO3_H_
#define _HM01B0_PLATFORM_APOLLO3_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "hm01b0.h"

#if defined(ARDUINO_SFE_EDGE)

// Mapping BSP -> HM01B0
#define HM01B0_PIN_D0 AM_BSP_GPIO_CAMERA_HM01B0_D0
#define HM01B0_PIN_D1 AM_BSP_GPIO_CAMERA_HM01B0_D1
#define HM01B0_PIN_D2 AM_BSP_GPIO_CAMERA_HM01B0_D2
#define HM01B0_PIN_D3 AM_BSP_GPIO_CAMERA_HM01B0_D3
#define HM01B0_PIN_D4 AM_BSP_GPIO_CAMERA_HM01B0_D4
#define HM01B0_PIN_D5 AM_BSP_GPIO_CAMERA_HM01B0_D5
#define HM01B0_PIN_D6 AM_BSP_GPIO_CAMERA_HM01B0_D6
#define HM01B0_PIN_D7 AM_BSP_GPIO_CAMERA_HM01B0_D7
#define HM01B0_PIN_VSYNC AM_BSP_GPIO_CAMERA_HM01B0_VSYNC
#define HM01B0_PIN_HSYNC AM_BSP_GPIO_CAMERA_HM01B0_HSYNC
#define HM01B0_PIN_PCLK AM_BSP_GPIO_CAMERA_HM01B0_PCLK
#define HM01B0_PIN_SCL AM_BSP_CAMERA_HM01B0_I2C_SCL_PIN
#define HM01B0_PIN_SDA AM_BSP_CAMERA_HM01B0_I2C_SDA_PIN

// Some boards do not support TRIG or INT pins
#ifdef AM_BSP_GPIO_CAMERA_HM01B0_TRIG
#define HM01B0_PIN_TRIG AM_BSP_GPIO_CAMERA_HM01B0_TRIG
#endif // AM_BSP_GPIO_CAMERA_HM01B0_TRIG

#ifdef AM_BSP_GPIO_CAMERA_HM01B0_INT
#define HM01B0_PIN_INT AM_BSP_GPIO_CAMERA_HM01B0_INT
#endif // AM_BSP_GPIO_CAMERA_HM01B0_INT

// Define AP3B's CTIMER and output pin for HM01B0 MCLK generation
#define HM01B0_MCLK_GENERATOR_MOD AM_BSP_CAMERA_HM01B0_MCLK_GEN_MOD
#define HM01B0_MCLK_GENERATOR_SEG AM_BSP_CAMERA_HM01B0_MCLK_GEN_SEG
#define HM01B0_PIN_MCLK AM_BSP_CAMERA_HM01B0_MCLK_PIN

// Define I2C controller and SCL(pin8)/SDA(pin9) are configured automatically.
#define HM01B0_IOM_MODE AM_HAL_IOM_I2C_MODE
#define HM01B0_IOM_MODULE AM_BSP_CAMERA_HM01B0_I2C_IOM
#define HM01B0_I2C_CLOCK_FREQ AM_HAL_IOM_100KHZ

#elif defined(ARDUINO_AM_AP3_SFE_BB_ARTEMIS_MICROMOD)

//These will be in the MM variant
#define G0 16
#define G1 33
#define G2 34
#define G3 27
#define G4 28
#define G5 29
#define G6 14
#define G7 15
#define CAM_MCLK 0
#define CAM_PCLK 11
#define CAM_HSYNC 12
#define CAM_VSYNC 13
#define I2C_INT 18
#define D1 2

//Define the connections between camera and MM
#define HM01B0_PIN_D0 G0
#define HM01B0_PIN_D1 G1
#define HM01B0_PIN_D2 G2
#define HM01B0_PIN_D3 G3
#define HM01B0_PIN_D4 G4
#define HM01B0_PIN_D5 G5
#define HM01B0_PIN_D6 G6
#define HM01B0_PIN_D7 G7
#define HM01B0_PIN_VSYNC CAM_VSYNC
#define HM01B0_PIN_HSYNC CAM_HSYNC
#define HM01B0_PIN_PCLK CAM_PCLK
#define HM01B0_PIN_SCL 39
#define HM01B0_PIN_SDA 40

#define HM01B0_PIN_TRIG D1
#define HM01B0_PIN_INT I2C_INT

// Define AP3B's CTIMER and output pin for HM01B0 MCLK generation
#define HM01B0_MCLK_GENERATOR_MOD 0
#define HM01B0_MCLK_GENERATOR_SEG AM_HAL_CTIMER_TIMERB
#define HM01B0_PIN_MCLK CAM_MCLK

// Define I2C controller and SCL(pin39)/SDA(pin40) are configured automatically.
#define HM01B0_IOM_MODE AM_HAL_IOM_I2C_MODE
#define HM01B0_IOM_MODULE 4
#define HM01B0_I2C_CLOCK_FREQ 100000

#endif

// Fast-access macros
#define HM01B0_READ_VSYNC (AM_REGVAL(AM_REGADDR(GPIO, RDA)) & (1 << HM01B0_PIN_VSYNC))
#define HM01B0_READ_HSYNC (AM_REGVAL(AM_REGADDR(GPIO, RDA)) & (1 << HM01B0_PIN_HSYNC))
#define HM01B0_READ_PCLK (AM_REGVAL(AM_REGADDR(GPIO, RDA)) & (1 << HM01B0_PIN_PCLK))
#define HM01B0_READ_BYTE (APBDMA->BBINPUT)

  // Type Definitions
  typedef struct _hm01b0_platform_apollo3_arg_t
  {
    uint16_t ui16SlvAddr;
    am_hal_iom_mode_e eIOMMode;
    uint32_t ui32IOMModule;
    am_hal_iom_config_t sIOMCfg;
    void *pIOMHandle;

    uint32_t ui32CTimerModule;
    uint32_t ui32CTimerSegment;
    uint32_t ui32CTimerOutputPin;

    uint8_t ui8PinSCL;
    uint8_t ui8PinSDA;
    uint8_t ui8PinD0;
    uint8_t ui8PinD1;
    uint8_t ui8PinD2;
    uint8_t ui8PinD3;
    uint8_t ui8PinD4;
    uint8_t ui8PinD5;
    uint8_t ui8PinD6;
    uint8_t ui8PinD7;
    uint8_t ui8PinVSYNC;
    uint8_t ui8PinHSYNC;
    uint8_t ui8PinPCLK;

    uint8_t ui8PinTrig;
    uint8_t ui8PinInt;
    void (*pfnGpioIsr)(void);
  } hm01b0_platform_apollo3_arg_t;
  extern hm01b0_platform_apollo3_arg_t hm01b0_platform_apollo3_args;

  // Apollo3 HM01B0 Configuration (provided to hm01b0_arduino.h)
  extern hm01b0_cfg_t hm01b0_cfg;

  // Interface Function Declarations
  hm01b0_status_e hm01b0_platform_apollo3_init(hm01b0_cfg_t *psCfg, void *arg);                                                               // any initialization code needed
  hm01b0_status_e hm01b0_platform_apollo3_write(hm01b0_cfg_t *psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes, void *arg); // write to registers over I2C
  hm01b0_status_e hm01b0_platform_apollo3_read(hm01b0_cfg_t *psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes, void *arg);  // read from registers over I2C
  hm01b0_status_e hm01b0_platform_apollo3_mclk(hm01b0_cfg_t *psCfg, bool enable, void *arg);                                                  // enable/disable the clock generation hardware
  hm01b0_status_e hm01b0_platform_apollo3_trig(hm01b0_cfg_t *psCfg, bool enable, void *arg);                                                  // enable/disabe the trigger pin
  hm01b0_status_e hm01b0_platform_apollo3_deinit(hm01b0_cfg_t *psCfg, void *arg);                                                             // any deinitialization code needed

#ifdef __cplusplus
}
#endif

#endif // _HM01B0_PLATFORM_APOLLO3_H_