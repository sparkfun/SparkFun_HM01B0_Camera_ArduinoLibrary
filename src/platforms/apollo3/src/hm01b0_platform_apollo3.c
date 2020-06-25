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

#if defined(AM_PART_APOLLO3) &&   \
    (defined(ARDUINO_SFE_EDGE) || \
     defined(ARDUINO_AM_AP3_SFE_BB_ARTEMIS_MICROMOD))

#include "platforms/apollo3/include/hm01b0_platform_apollo3.h"

#define MCLK_UI64PATTERN 0x55555555
#define MCLK_UI64PATTERNLEN 31

// Forward Declarations
hm01b0_status_e burst_mode_enable(bool bEnable);

const am_hal_gpio_pincfg_t g_HM01B0_pin_int =
    {
        .uFuncSel = 3,
        .eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
        .eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN};

hm01b0_platform_apollo3_arg_t hm01b0_platform_apollo3_args = {
    // I2C settings
    .ui16SlvAddr = HM01B0_DEFAULT_ADDRESS,
    .eIOMMode = HM01B0_IOM_MODE,
    .ui32IOMModule = HM01B0_IOM_MODULE,
    .sIOMCfg = {
        .eInterfaceMode = HM01B0_IOM_MODE,
        .ui32ClockFreq = HM01B0_I2C_CLOCK_FREQ,
    },
    .pIOMHandle = NULL,
    .ui8PinSCL = HM01B0_PIN_SCL,
    .ui8PinSDA = HM01B0_PIN_SDA,

    // MCLK settings
    .ui32CTimerModule = HM01B0_MCLK_GENERATOR_MOD,
    .ui32CTimerSegment = HM01B0_MCLK_GENERATOR_SEG,
    .ui32CTimerOutputPin = HM01B0_PIN_MCLK,

    // data interface
    .ui8PinD0 = HM01B0_PIN_D0,
    .ui8PinD1 = HM01B0_PIN_D1,
    .ui8PinD2 = HM01B0_PIN_D2,
    .ui8PinD3 = HM01B0_PIN_D3,
    .ui8PinD4 = HM01B0_PIN_D4,
    .ui8PinD5 = HM01B0_PIN_D5,
    .ui8PinD6 = HM01B0_PIN_D6,
    .ui8PinD7 = HM01B0_PIN_D7,
    .ui8PinVSYNC = HM01B0_PIN_VSYNC,
    .ui8PinHSYNC = HM01B0_PIN_HSYNC,
    .ui8PinPCLK = HM01B0_PIN_PCLK,

#ifdef HM01B0_PIN_TRIG
    .ui8PinTrig = HM01B0_PIN_TRIG,
#endif // HM01B0_PIN_TRIG

#ifdef HM01B0_PIN_INT
    .ui8PinInt = HM01B0_PIN_INT,
#endif // HM01B0_PIN_INT

    .pfnGpioIsr = NULL,
};

//
// Interface
hm01b0_if_t hm01b0_platform_apollo3_if = {
    .init = hm01b0_platform_apollo3_init,
    .write = hm01b0_platform_apollo3_write,
    .read = hm01b0_platform_apollo3_read,
    .mclk = hm01b0_platform_apollo3_mclk,
    .trig = hm01b0_platform_apollo3_trig,
    .deinit = hm01b0_platform_apollo3_deinit,
    .arg = (void *)(&hm01b0_platform_apollo3_args),
};

//
// Apollo3 Configuration
hm01b0_cfg_t hm01b0_cfg = {

    .interface = &hm01b0_platform_apollo3_if,
};

// Interface Function Definitions
hm01b0_status_e hm01b0_platform_apollo3_init(hm01b0_cfg_t *psCfg, void *arg)
{
  hm01b0_status_e retval = HM01B0_ERR_OK;
  hm01b0_platform_apollo3_arg_t *args = (hm01b0_platform_apollo3_arg_t *)(arg);
  void *pIOMHandle = NULL;

  if (args->ui32IOMModule > AM_REG_IOM_NUM_MODULES)
  {
    return HM01B0_ERR_I2C;
  }

  //
  // Enable fault detection.
#if AM_APOLLO3_MCUCTRL
  am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else  // AM_APOLLO3_MCUCTRL
  am_hal_mcuctrl_fault_capture_enable();
#endif // AM_APOLLO3_MCUCTRL

  //
  // Initialize the IOM instance.
  // Enable power to the IOM instance.
  // Configure the IOM for Serial operation during initialization.
  // Enable the IOM.
  //
  if (am_hal_iom_initialize(args->ui32IOMModule, &pIOMHandle) || am_hal_iom_power_ctrl(pIOMHandle, AM_HAL_SYSCTRL_WAKE, false) || am_hal_iom_configure(pIOMHandle, &(args->sIOMCfg)) || am_hal_iom_enable(pIOMHandle))
  {
    return HM01B0_ERR_I2C;
  }
  else
  {
    // Configure the IOM pins.
    am_bsp_iom_pins_enable(args->ui32IOMModule, args->eIOMMode);
    args->pIOMHandle = pIOMHandle;
  }

  // initialize pins for camera parallel interface.
  am_hal_gpio_fastgpio_disable(args->ui8PinD0);
  am_hal_gpio_fastgpio_disable(args->ui8PinD1);
  am_hal_gpio_fastgpio_disable(args->ui8PinD2);
  am_hal_gpio_fastgpio_disable(args->ui8PinD3);
  am_hal_gpio_fastgpio_disable(args->ui8PinD4);
  am_hal_gpio_fastgpio_disable(args->ui8PinD5);
  am_hal_gpio_fastgpio_disable(args->ui8PinD6);
  am_hal_gpio_fastgpio_disable(args->ui8PinD7);

  am_hal_gpio_fastgpio_clr(args->ui8PinD0);
  am_hal_gpio_fastgpio_clr(args->ui8PinD1);
  am_hal_gpio_fastgpio_clr(args->ui8PinD2);
  am_hal_gpio_fastgpio_clr(args->ui8PinD3);
  am_hal_gpio_fastgpio_clr(args->ui8PinD4);
  am_hal_gpio_fastgpio_clr(args->ui8PinD5);
  am_hal_gpio_fastgpio_clr(args->ui8PinD6);
  am_hal_gpio_fastgpio_clr(args->ui8PinD7);

  am_hal_gpio_fast_pinconfig((uint64_t)0x1 << args->ui8PinD0 |
                                 (uint64_t)0x1 << args->ui8PinD1 |
                                 (uint64_t)0x1 << args->ui8PinD2 |
                                 (uint64_t)0x1 << args->ui8PinD3 |
                                 (uint64_t)0x1 << args->ui8PinD4 |
                                 (uint64_t)0x1 << args->ui8PinD5 |
                                 (uint64_t)0x1 << args->ui8PinD6 |
                                 (uint64_t)0x1 << args->ui8PinD7,
                             g_AM_HAL_GPIO_INPUT, 0);

  am_hal_gpio_pinconfig(args->ui8PinVSYNC, g_AM_HAL_GPIO_INPUT);
  am_hal_gpio_pinconfig(args->ui8PinHSYNC, g_AM_HAL_GPIO_INPUT);
  am_hal_gpio_pinconfig(args->ui8PinPCLK, g_AM_HAL_GPIO_INPUT);

  am_hal_gpio_pinconfig(args->ui8PinTrig, g_AM_HAL_GPIO_OUTPUT);

  am_hal_gpio_pinconfig(args->ui8PinInt, g_AM_HAL_GPIO_DISABLE);
  // am_hal_gpio_pinconfig(args->ui8PinInt,     g_HM01B0_pin_int);
  // am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(args->ui8PinInt));
  // am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(args->ui8PinInt));
  // NVIC_EnableIRQ(GPIO_IRQn);

  retval |= burst_mode_enable(true); // turn on burst mode

  return retval;
}

hm01b0_status_e hm01b0_platform_apollo3_write(hm01b0_cfg_t *psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes, void *arg)
{
  hm01b0_status_e retval = HM01B0_ERR_OK;
  am_hal_iom_transfer_t Transaction;
  hm01b0_platform_apollo3_arg_t *args = (hm01b0_platform_apollo3_arg_t *)(arg);

  //
  // Create the transaction.
  Transaction.ui32InstrLen = sizeof(uint16_t);
  Transaction.ui32Instr = (ui16Reg & 0x0000FFFF);
  Transaction.eDirection = AM_HAL_IOM_TX;
  Transaction.ui32NumBytes = ui32NumBytes;
  Transaction.pui32TxBuffer = (uint32_t *)pui8Value;
  Transaction.uPeerInfo.ui32I2CDevAddr = (uint32_t)args->ui16SlvAddr;
  Transaction.bContinue = false;
  Transaction.ui8RepeatCount = 0;
  Transaction.ui32PauseCondition = 0;
  Transaction.ui32StatusSetClr = 0;

  //
  // Execute the transction over IOM.
  if (am_hal_iom_blocking_transfer(args->pIOMHandle, &Transaction))
  {
    return HM01B0_ERR_I2C;
  }
  return retval;
}

hm01b0_status_e hm01b0_platform_apollo3_read(hm01b0_cfg_t *psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes, void *arg)
{
  hm01b0_status_e retval = HM01B0_ERR_OK;
  am_hal_iom_transfer_t Transaction;
  hm01b0_platform_apollo3_arg_t *args = (hm01b0_platform_apollo3_arg_t *)(arg);

  //
  // Create the transaction.
  Transaction.ui32InstrLen = sizeof(uint16_t);
  Transaction.ui32Instr = (ui16Reg & 0x0000FFFF);
  Transaction.eDirection = AM_HAL_IOM_RX;
  Transaction.ui32NumBytes = ui32NumBytes;
  Transaction.pui32RxBuffer = (uint32_t *)pui8Value;
  ;
  Transaction.uPeerInfo.ui32I2CDevAddr = (uint32_t)args->ui16SlvAddr;
  Transaction.bContinue = false;
  Transaction.ui8RepeatCount = 0;
  Transaction.ui32PauseCondition = 0;
  Transaction.ui32StatusSetClr = 0;

  //
  // Execute the transction over IOM.
  if (am_hal_iom_blocking_transfer(args->pIOMHandle, &Transaction))
  {
    return HM01B0_ERR_I2C;
  }
  return retval;
}

hm01b0_status_e hm01b0_platform_apollo3_mclk(hm01b0_cfg_t *psCfg, bool enable, void *arg)
{
  hm01b0_status_e retval = HM01B0_ERR_OK;
  hm01b0_platform_apollo3_arg_t *args = (hm01b0_platform_apollo3_arg_t *)(arg);
  static bool mclk_initialized = false;
  if (enable)
  {
    if (!mclk_initialized)
    {
      am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

      // Set up timer.
      am_hal_ctimer_clear(args->ui32CTimerModule, args->ui32CTimerSegment);
      am_hal_ctimer_config_single(args->ui32CTimerModule, args->ui32CTimerSegment, (AM_HAL_CTIMER_FN_PTN_REPEAT | AM_HAL_CTIMER_HFRC_12MHZ));

      // Set the pattern in the CMPR registers.
      am_hal_ctimer_compare_set(args->ui32CTimerModule, args->ui32CTimerSegment, 0, (uint32_t)(MCLK_UI64PATTERN & 0xFFFF));
      am_hal_ctimer_compare_set(args->ui32CTimerModule, args->ui32CTimerSegment, 1, (uint32_t)((MCLK_UI64PATTERN >> 16) & 0xFFFF));

      // Set the timer trigger and pattern length.
      am_hal_ctimer_config_trigger(args->ui32CTimerModule, args->ui32CTimerSegment, ((MCLK_UI64PATTERNLEN << CTIMER_AUX0_TMRA0LMT_Pos) | (CTIMER_AUX0_TMRB0TRIG_DIS << CTIMER_AUX0_TMRA0TRIG_Pos)));

      // Configure timer output pin.
      am_hal_ctimer_output_config(args->ui32CTimerModule, args->ui32CTimerSegment, args->ui32CTimerOutputPin, AM_HAL_CTIMER_OUTPUT_NORMAL, AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA);

      // Start the timer.
      am_hal_ctimer_start(args->ui32CTimerModule, args->ui32CTimerSegment);
    }
  }
  else
  {
    if (mclk_initialized)
    {
      //
      // Stop the timer.
      am_hal_ctimer_stop(args->ui32CTimerModule, args->ui32CTimerSegment);
      am_hal_gpio_pinconfig(args->ui32CTimerOutputPin, g_AM_HAL_GPIO_DISABLE);
    }
  }

  return retval;
}

hm01b0_status_e hm01b0_platform_apollo3_trig(hm01b0_cfg_t *psCfg, bool enable, void *arg)
{
  hm01b0_status_e retval = HM01B0_ERR_OK;
  hm01b0_platform_apollo3_arg_t *args = (hm01b0_platform_apollo3_arg_t *)(arg);

  if (enable)
  {
    am_hal_gpio_output_set(args->ui8PinTrig);
  }
  else
  {
    am_hal_gpio_output_clear(args->ui8PinTrig);
  }

  return retval;
}

hm01b0_status_e hm01b0_platform_apollo3_deinit(hm01b0_cfg_t *psCfg, void *arg)
{
  hm01b0_status_e retval = HM01B0_ERR_OK;
  hm01b0_platform_apollo3_arg_t *args = (hm01b0_platform_apollo3_arg_t *)(arg);

  am_hal_iom_disable(args->pIOMHandle);
  am_hal_iom_uninitialize(args->pIOMHandle);

  am_hal_gpio_pinconfig(args->ui8PinSCL, g_AM_HAL_GPIO_DISABLE);
  am_hal_gpio_pinconfig(args->ui8PinSDA, g_AM_HAL_GPIO_DISABLE);

  // initialize pins for camera parallel interface.
  am_hal_gpio_fastgpio_disable(args->ui8PinD0);
  am_hal_gpio_fastgpio_disable(args->ui8PinD1);
  am_hal_gpio_fastgpio_disable(args->ui8PinD2);
  am_hal_gpio_fastgpio_disable(args->ui8PinD3);
  am_hal_gpio_fastgpio_disable(args->ui8PinD4);
  am_hal_gpio_fastgpio_disable(args->ui8PinD5);
  am_hal_gpio_fastgpio_disable(args->ui8PinD6);
  am_hal_gpio_fastgpio_disable(args->ui8PinD7);

  am_hal_gpio_fastgpio_clr(args->ui8PinD0);
  am_hal_gpio_fastgpio_clr(args->ui8PinD1);
  am_hal_gpio_fastgpio_clr(args->ui8PinD2);
  am_hal_gpio_fastgpio_clr(args->ui8PinD3);
  am_hal_gpio_fastgpio_clr(args->ui8PinD4);
  am_hal_gpio_fastgpio_clr(args->ui8PinD5);
  am_hal_gpio_fastgpio_clr(args->ui8PinD6);
  am_hal_gpio_fastgpio_clr(args->ui8PinD7);

  am_hal_gpio_pinconfig(args->ui8PinVSYNC, g_AM_HAL_GPIO_DISABLE);
  am_hal_gpio_pinconfig(args->ui8PinHSYNC, g_AM_HAL_GPIO_DISABLE);
  am_hal_gpio_pinconfig(args->ui8PinPCLK, g_AM_HAL_GPIO_DISABLE);
  am_hal_gpio_pinconfig(args->ui8PinTrig, g_AM_HAL_GPIO_DISABLE);
  am_hal_gpio_pinconfig(args->ui8PinInt, g_AM_HAL_GPIO_DISABLE);

  retval |= burst_mode_enable(false); // turn off burst mode

  return retval;
}

// burst mode enable
hm01b0_status_e burst_mode_enable(bool bEnable)
{
  am_hal_burst_avail_e eBurstModeAvailable;
  am_hal_burst_mode_e eBurstMode;

  // Check that the Burst Feature is available.
  if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_initialize(&eBurstModeAvailable))
  {
    if (AM_HAL_BURST_AVAIL == eBurstModeAvailable)
    {
      // am_util_stdio_printf("Apollo3 Burst Mode is Available\n");
    }
    else
    {
      // am_util_stdio_printf("Apollo3 Burst Mode is Not Available\n");
      return HM01B0_ERR;
    }
  }
  else
  {
    // am_util_stdio_printf("Failed to Initialize for Burst Mode operation\n");
    return HM01B0_ERR;
  }

  // Make sure we are in "Normal" mode.
  if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_disable(&eBurstMode))
  {
    if (AM_HAL_NORMAL_MODE == eBurstMode)
    {
      // am_util_stdio_printf("Apollo3 operating in Normal Mode (48MHz)\n");
    }
  }
  else
  {
    // am_util_stdio_printf("Failed to Disable Burst Mode operation\n");
    return HM01B0_ERR;
  }

  // Put the MCU into "Burst" mode.
  if (bEnable)
  {
    if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_enable(&eBurstMode))
    {
      if (AM_HAL_BURST_MODE == eBurstMode)
      {
        // am_util_stdio_printf("Apollo3 operating in Burst Mode (96MHz)\n");
      }
    }
    else
    {
      // am_util_stdio_printf("Failed to Enable Burst Mode operation\n");
      return HM01B0_ERR;
    }
  }

  return HM01B0_ERR_OK;
}

#endif // platform exclusion