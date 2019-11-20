HM01B0 Platforms
================

Platforms allow for specialization of interface code for the camera module. There are two main domains for the interface:

* Low-speed: Abstraction takes place with a virtual function table
* High-speed: abstraction takes place with macros (preprocessor text replacement)

A typical case will combine these files to implement a patform:
* ```src/platforms/include/hm01b0_platform_{your_platform_name}.h``` : the 'platform header file' (included by ```src\hm01b0_c\src\hm01b0_c.c```)
* ```src/platforms/src/hm01b0_platform_{your_platform_name}.cpp``` : the 'platform implementation file' (C++)
* ```src/platforms/src/hm01b0_platform_{your_platform_name}.c``` : the 'platform implementation file' (C)

Each platform must be gated by preprocessor defines that are unique to that architecture so as to avoid conflicts. For example:
** In platform implementation files (C/C++) **
``` c
#if defined (PLATFORM_UNIQUE_MACRO)
... 
... // implementation code here
...
#endif
```

** In hm01b0_c.c **
``` c
...
... // platform includes
...
#elif defined (PLATFORM_UNIQUE_MACRO)
#include "src/platforms/include/hm01b0_platform_{your_platform_name}.h"
...
... // more platform includes
... 
```

## Low-speed Interface
The low-speed interface is used to handle I2C configuration, MCLK generation, and slow pin access like the trigger pin.

The interface is abstracted through this construct:
``` c
typedef hm01b0_status_e (*hm01b0_if_fn_t)(hm01b0_cfg_t* psCfg, void* arg);
typedef hm01b0_status_e (*hm01b0_if_i2c_fn_t)(hm01b0_cfg_t* psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes, void* arg);
typedef hm01b0_status_e (*hm01b0_if_on_off_fn_t)(hm01b0_cfg_t* psCfg, bool enable, void* arg);
typedef struct {
  hm01b0_if_fn_t        init;     // any initialization code needed
  hm01b0_if_i2c_fn_t    write;    // write to registers over I2C
  hm01b0_if_i2c_fn_t    read;     // read from registers over I2C
  hm01b0_if_on_off_fn_t mclk;     // enable/disable the clock generation hardware
  hm01b0_if_on_off_fn_t trig;     // enable/disabe the trigger pin
  hm01b0_if_fn_t        deinit;   // any deinitialization code needed
  void*                 arg;      // argument for the user available in interface functions
} hm01b0_if_t;  // abstracts the interface for the HM01B0
```

When porting a new platform create an ```hm01b0_if_t``` and fill it with pointers to appropriate functions. This structure is then referenced by the ```hm01b0_cfg_t``` structure called ```hm01b0_cfg``` which every platform must provide. 




## High-speed Interface
The high-speed interface is used in ```hm01b0_c.c``` to read data from the camera interface. You must implement the following macros:

``` c
HM01B0_READ_VSYNC // (currently unused)
HM01B0_READ_HSYNC // Evaluate to either 0 or 1 indicating the level of the HSYNC pin
HM01B0_READ_PCLK  // Evaluate to either 0 or 1 indicating the level of the PCLK pin
HM01B0_READ_BYTE  // Evaluate to an 8-bit quantity describing logic levels on data lines D0-7 
```

These macros can be defined in the platform header file
