Concerning Library Structure
============================

Ideally this C driver would exist as a standalone resource and would be included 'symbolically' (for example with git submodules) however the limitations of Arduino prevent this -- only the ```Library/src``` directory is added to the include path which makes it impossible for third-party sources to access their header files without modification or relocation. [Arduino Issue #9449](https://github.com/arduino/Arduino/issues/9449)