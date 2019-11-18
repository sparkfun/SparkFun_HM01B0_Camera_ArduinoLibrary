    uint16_t                ui16SlvAddr;
    am_hal_iom_mode_e       eIOMMode;
    uint32_t                ui32IOMModule;
    am_hal_iom_config_t     sIOMCfg;
    void                    *pIOMHandle;

    uint32_t                ui32CTimerModule;
    uint32_t                ui32CTimerSegment;
    uint32_t                ui32CTimerOutputPin;

    uint8_t                 ui8PinSCL;
    uint8_t                 ui8PinSDA;
    uint8_t                 ui8PinD0;
    uint8_t                 ui8PinD1;
    uint8_t                 ui8PinD2;
    uint8_t                 ui8PinD3;
    uint8_t                 ui8PinD4;
    uint8_t                 ui8PinD5;
    uint8_t                 ui8PinD6;
    uint8_t                 ui8PinD7;
    uint8_t                 ui8PinVSYNC;
    uint8_t                 ui8PinHSYNC;
    uint8_t                 ui8PinPCLK;

    uint8_t                 ui8PinTrig;
    uint8_t                 ui8PinInt;
    void                    (*pfnGpioIsr)(void);