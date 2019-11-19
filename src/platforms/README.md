HM01B0 Platforms
================

Platforms allow for specialization of interface code for the camera module. There are two main domains for the interface:

* Low-speed: Abstraction takes place with a virtual function table
* High-speed: abstraction takes place with macros (preprocessor text replacement)

A typical case will combine these files to implement a patform:
* ```src/platforms/include/hm01b0_platform_{your_platform_name}.h``` : the 'platform header file' (included by ```src\hm01b0_c\src\hm01b0_c.c```)
* ```src/platforms/src/hm01b0_platform_{your_platform_name}.cpp``` : the 'platform implementation file' (C++)
* ```src/platforms/src/hm01b0_platform_{your_platform_name}.c``` : the 'platform implementation file' (C)

## Low-speed Interface



## High-speed Interface
The high-speed interface is used in ```hm01b0_c.c``` to read data from the camera interface. You must implement the following macros:

* **HM01B0_READ_VSYNC** (currently unused)
* **HM01B0_READ_HSYNC** Evaluate to either 0 or 1 indicating the level of the HSYNC pin
* **HM01B0_READ_PCLK**  Evaluate to either 0 or 1 indicating the level of the PCLK pin
* **HM01B0_READ_BYTE**  Evaluate to an 8-bit quantity describing logic levels on data lines D0-7 

These macros can be defined in the platform header file
