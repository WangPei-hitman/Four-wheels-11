/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v8.0
processor: MK66FX1M0xxx18
package_id: MK66FX1M0VLQ18
mcu_data: ksdk2_0
processor_version: 8.0.1
functionalGroups:
- name: RTEPIP_Basic
  UUID: a4a4647f-1145-476a-aea5-a573ead41735
  selectedCore: core0
- name: RTEPIP_Device
  UUID: 3d45962e-c4f0-4bb8-bc02-d1abb2812c8d
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * RTEPIP_Basic functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DMA initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'DMA'
- type: 'edma'
- mode: 'basic'
- custom_name_enabled: 'false'
- type_id: 'edma_6d0dd4e17e2f179c7d42d98767129ede'
- functional_group: 'RTEPIP_Basic'
- peripheral: 'DMA'
- config_sets:
  - fsl_edma:
    - common_settings:
      - enableContinuousLinkMode: 'false'
      - enableHaltOnError: 'true'
      - enableRoundRobinArbitration: 'false'
      - enableDebugMode: 'false'
    - dma_table: []
    - edma_channels: []
    - errInterruptConfig:
      - enableErrInterrupt: 'false'
      - errorInterrupt:
        - IRQn: 'DMA_Error_IRQn'
        - enable_interrrupt: 'enabled'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
    - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const edma_config_t DMA_config = {
  .enableContinuousLinkMode = false,
  .enableHaltOnError = true,
  .enableRoundRobinArbitration = false,
  .enableDebugMode = false
};

static void DMA_init(void) {
}

/***********************************************************************************************************************
 * GPIOA initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOA'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'gpio_5920c5e026e8e974e6dc54fbd5e22ad7'
- functional_group: 'RTEPIP_Basic'
- peripheral: 'GPIOA'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTA_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '4'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOA_init(void) {
  /* Make sure, the clock gate for port A is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTA_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTA_IRQn, GPIOA_IRQ_PRIORITY);
  /* Enable interrupt PORTA_IRQn request in the NVIC */
  EnableIRQ(PORTA_IRQn);
}

/***********************************************************************************************************************
 * GPIOB initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOB'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'gpio_5920c5e026e8e974e6dc54fbd5e22ad7'
- functional_group: 'RTEPIP_Basic'
- peripheral: 'GPIOB'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTB_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '4'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOB_init(void) {
  /* Make sure, the clock gate for port B is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTB_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTB_IRQn, GPIOB_IRQ_PRIORITY);
  /* Enable interrupt PORTB_IRQn request in the NVIC */
  EnableIRQ(PORTB_IRQn);
}

/***********************************************************************************************************************
 * GPIOC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOC'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'gpio_5920c5e026e8e974e6dc54fbd5e22ad7'
- functional_group: 'RTEPIP_Basic'
- peripheral: 'GPIOC'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTC_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '4'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOC_init(void) {
  /* Make sure, the clock gate for port C is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTC_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTC_IRQn, GPIOC_IRQ_PRIORITY);
  /* Enable interrupt PORTC_IRQn request in the NVIC */
  EnableIRQ(PORTC_IRQn);
}

/***********************************************************************************************************************
 * GPIOD initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOD'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'gpio_5920c5e026e8e974e6dc54fbd5e22ad7'
- functional_group: 'RTEPIP_Basic'
- peripheral: 'GPIOD'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTD_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '4'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOD_init(void) {
  /* Make sure, the clock gate for port D is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTD_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTD_IRQn, GPIOD_IRQ_PRIORITY);
  /* Enable interrupt PORTD_IRQn request in the NVIC */
  EnableIRQ(PORTD_IRQn);
}

/***********************************************************************************************************************
 * GPIOE initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOE'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'gpio_5920c5e026e8e974e6dc54fbd5e22ad7'
- functional_group: 'RTEPIP_Basic'
- peripheral: 'GPIOE'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTE_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '4'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOE_init(void) {
  /* Make sure, the clock gate for port E is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTE_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTE_IRQn, GPIOE_IRQ_PRIORITY);
  /* Enable interrupt PORTE_IRQn request in the NVIC */
  EnableIRQ(PORTE_IRQn);
}

/***********************************************************************************************************************
 * LPTMR0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPTMR0'
- type: 'lptmr'
- mode: 'LPTMR_GENERAL'
- custom_name_enabled: 'false'
- type_id: 'lptmr_48552e76e8733b28a9c768b6d8d4fefa'
- functional_group: 'RTEPIP_Basic'
- peripheral: 'LPTMR0'
- config_sets:
  - fsl_lptmr:
    - lptmr_config:
      - timerMode: 'kLPTMR_TimerModeTimeCounter'
      - pinSelect: 'ALT.0'
      - pinPolarity: 'kLPTMR_PinPolarityActiveHigh'
      - enableFreeRunning: 'false'
      - bypassPrescaler: 'true'
      - prescalerClockSource: 'kLPTMR_PrescalerClock_3'
      - clockSource: 'RTECLK_HsRun_180MHz'
      - value: 'kLPTMR_Prescale_Glitch_0'
      - timerPeriod: '1ms'
    - enableInterrupt: 'true'
    - interrupt:
      - IRQn: 'LPTMR0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '5'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lptmr_config_t LPTMR0_config = {
  .timerMode = kLPTMR_TimerModeTimeCounter,
  .pinSelect = kLPTMR_PinSelectInput_0,
  .pinPolarity = kLPTMR_PinPolarityActiveHigh,
  .enableFreeRunning = false,
  .bypassPrescaler = true,
  .prescalerClockSource = kLPTMR_PrescalerClock_3,
  .value = kLPTMR_Prescale_Glitch_0
};

static void LPTMR0_init(void) {
  /* Initialize the LPTMR */
  LPTMR_Init(LPTMR0_PERIPHERAL, &LPTMR0_config);
  /* Set LPTMR period */
  LPTMR_SetTimerPeriod(LPTMR0_PERIPHERAL, LPTMR0_TICKS);
  /* Configure timer interrupt */
  LPTMR_EnableInterrupts(LPTMR0_PERIPHERAL, kLPTMR_TimerInterruptEnable);
  /* Interrupt vector LPTMR0_IRQn priority settings in the NVIC */
  NVIC_SetPriority(LPTMR0_IRQn, LPTMR0_IRQ_PRIORITY);
  /* Enable interrupt LPTMR0_IRQn request in the NVIC */
  EnableIRQ(LPTMR0_IRQn);
}

/***********************************************************************************************************************
 * PIT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PIT'
- type: 'pit'
- mode: 'LPTMR_GENERAL'
- custom_name_enabled: 'false'
- type_id: 'pit_a4782ba5223c8a2527ba91aeb2bc4159'
- functional_group: 'RTEPIP_Basic'
- peripheral: 'PIT'
- config_sets:
  - fsl_pit:
    - enableRunInDebug: 'false'
    - timingConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
    - channels: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const pit_config_t PIT_config = {
  .enableRunInDebug = false
};

static void PIT_init(void) {
  /* Initialize the PIT. */
  PIT_Init(PIT_PERIPHERAL, &PIT_config);
}

/***********************************************************************************************************************
 * RTEPIP_Device functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * CAM_UART initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'CAM_UART'
- type: 'uart'
- mode: 'polling'
- custom_name_enabled: 'true'
- type_id: 'uart_88ab1eca0cddb7ee407685775de016d5'
- functional_group: 'RTEPIP_Device'
- peripheral: 'UART3'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '9600'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t CAM_UART_config = {
  .baudRate_Bps = 9600,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void CAM_UART_init(void) {
  UART_Init(CAM_UART_PERIPHERAL, &CAM_UART_config, CAM_UART_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * DBG_LPUART initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'DBG_LPUART'
- type: 'lpuart'
- mode: 'polling'
- custom_name_enabled: 'true'
- type_id: 'lpuart_54a65a580e3462acdbacefd5299e0cac'
- functional_group: 'RTEPIP_Device'
- peripheral: 'LPUART0'
- config_sets:
  - lpuartConfig_t:
    - lpuartConfig:
      - clockSource: 'LpuartClock'
      - lpuartSrcClkFreq: 'RTECLK_HsRun_180MHz'
      - baudRate_Bps: '921600'
      - parityMode: 'kLPUART_ParityDisabled'
      - dataBitsCount: 'kLPUART_EightDataBits'
      - isMsb: 'false'
      - stopBitCount: 'kLPUART_OneStopBit'
      - enableRxRTS: 'false'
      - enableTxCTS: 'false'
      - txCtsSource: 'kLPUART_CtsSourcePin'
      - txCtsConfig: 'kLPUART_CtsSampleAtStart'
      - rxIdleType: 'kLPUART_IdleTypeStartBit'
      - rxIdleConfig: 'kLPUART_IdleCharacter1'
      - enableTx: 'true'
      - enableRx: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpuart_config_t DBG_LPUART_config = {
  .baudRate_Bps = 921600,
  .parityMode = kLPUART_ParityDisabled,
  .dataBitsCount = kLPUART_EightDataBits,
  .isMsb = false,
  .stopBitCount = kLPUART_OneStopBit,
  .enableRxRTS = false,
  .enableTxCTS = false,
  .txCtsSource = kLPUART_CtsSourcePin,
  .txCtsConfig = kLPUART_CtsSampleAtStart,
  .rxIdleType = kLPUART_IdleTypeStartBit,
  .rxIdleConfig = kLPUART_IdleCharacter1,
  .enableTx = true,
  .enableRx = true
};

static void DBG_LPUART_init(void) {
  LPUART_Init(DBG_LPUART_PERIPHERAL, &DBG_LPUART_config, DBG_LPUART_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * EMAG initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'EMAG'
- type: 'adc16'
- mode: 'ADC'
- custom_name_enabled: 'true'
- type_id: 'adc16_7a29cdeb84266e77f0c7ceac1b49fe2d'
- functional_group: 'RTEPIP_Device'
- peripheral: 'ADC0'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAlt0'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider1'
      - resolution: 'kADC16_Resolution8or9Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - enableHighSpeed: 'true'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'true'
    - trigger: 'false'
    - hardwareAverageConfiguration: 'kADC16_HardwareAverageCount8'
    - enable_dma: 'false'
    - enable_irq: 'false'
    - adc_interrupt:
      - IRQn: 'ADC0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - adc16_channels_config:
      - 0:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.16'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'false'
      - 1:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.23'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'false'
      - 2:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.17'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'false'
      - 3:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.18'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'false'
      - 4:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.10'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'false'
      - 5:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.11'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'false'
      - 6:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.12'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'false'
      - 7:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.13'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
adc16_channel_config_t EMAG_channelsConfig[8] = {
  {
    .channelNumber = 16U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  },
  {
    .channelNumber = 23U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  },
  {
    .channelNumber = 17U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  },
  {
    .channelNumber = 18U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  },
  {
    .channelNumber = 10U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  },
  {
    .channelNumber = 11U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  },
  {
    .channelNumber = 12U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  },
  {
    .channelNumber = 13U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  }
};
const adc16_config_t EMAG_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAlt0,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider1,
  .resolution = kADC16_Resolution8or9Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .enableHighSpeed = true,
  .enableLowPower = false,
  .enableContinuousConversion = false
};
const adc16_channel_mux_mode_t EMAG_muxMode = kADC16_ChannelMuxA;
const adc16_hardware_average_mode_t EMAG_hardwareAverageMode = kADC16_HardwareAverageCount8;

static void EMAG_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(EMAG_PERIPHERAL, &EMAG_config);
  /* Make sure, that software trigger is used */
  ADC16_EnableHardwareTrigger(EMAG_PERIPHERAL, false);
  /* Configure hardware average mode */
  ADC16_SetHardwareAverage(EMAG_PERIPHERAL, EMAG_hardwareAverageMode);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(EMAG_PERIPHERAL, EMAG_muxMode);
  /* Perform auto calibration */
  ADC16_DoAutoCalibration(EMAG_PERIPHERAL);
}

/***********************************************************************************************************************
 * ENCO_L initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ENCO_L'
- type: 'ftm'
- mode: 'QuadratureDecoder'
- custom_name_enabled: 'true'
- type_id: 'ftm_04a15ae4af2b404bf2ae403c3dbe98b3'
- functional_group: 'RTEPIP_Device'
- peripheral: 'FTM2'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - prescale: 'kFTM_Prescale_Divide_1'
      - bdmMode: 'kFTM_BdmMode_3'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: 'kFTM_CntMax kFTM_CntMin'
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM2_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_quadrature_decoder_mode:
    - timerModuloVal: '65535'
    - timerInitVal: '0'
    - ftm_quad_decoder_mode: 'kFTM_QuadCountAndDir'
    - ftm_phase_a_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
    - ftm_phase_b_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t ENCO_L_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_3,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = kFTM_CntMax | kFTM_CntMin,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_phase_params_t ENCO_L_phaseAParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};
const ftm_phase_params_t ENCO_L_phaseBParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};

static void ENCO_L_init(void) {
  FTM_Init(ENCO_L_PERIPHERAL, &ENCO_L_config);
/* Initialization of the timer initial value and modulo value */
  FTM_SetQuadDecoderModuloValue(ENCO_L_PERIPHERAL, 0,65535);
  FTM_SetupQuadDecode(ENCO_L_PERIPHERAL, &ENCO_L_phaseAParams, &ENCO_L_phaseBParams, kFTM_QuadCountAndDir);
  FTM_StartTimer(ENCO_L_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * ENCO_R initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ENCO_R'
- type: 'ftm'
- mode: 'QuadratureDecoder'
- custom_name_enabled: 'true'
- type_id: 'ftm_04a15ae4af2b404bf2ae403c3dbe98b3'
- functional_group: 'RTEPIP_Device'
- peripheral: 'FTM1'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - prescale: 'kFTM_Prescale_Divide_1'
      - bdmMode: 'kFTM_BdmMode_3'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM1_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_quadrature_decoder_mode:
    - timerModuloVal: '65535'
    - timerInitVal: '0'
    - ftm_quad_decoder_mode: 'kFTM_QuadCountAndDir'
    - ftm_phase_a_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
    - ftm_phase_b_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t ENCO_R_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_3,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_phase_params_t ENCO_R_phaseAParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};
const ftm_phase_params_t ENCO_R_phaseBParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};

static void ENCO_R_init(void) {
  FTM_Init(ENCO_R_PERIPHERAL, &ENCO_R_config);
/* Initialization of the timer initial value and modulo value */
  FTM_SetQuadDecoderModuloValue(ENCO_R_PERIPHERAL, 0,65535);
  FTM_SetupQuadDecode(ENCO_R_PERIPHERAL, &ENCO_R_phaseAParams, &ENCO_R_phaseBParams, kFTM_QuadCountAndDir);
  FTM_StartTimer(ENCO_R_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * IMU_I2C initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'IMU_I2C'
- type: 'i2c'
- mode: 'I2C_Polling'
- custom_name_enabled: 'true'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'RTEPIP_Device'
- peripheral: 'I2C0'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'false'
      - baudRate_Bps: '400000'
      - glitchFilterWidth: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t IMU_I2C_config = {
  .enableMaster = true,
  .enableStopHold = false,
  .baudRate_Bps = 400000,
  .glitchFilterWidth = 0
};

static void IMU_I2C_init(void) {
  /* Initialization function */
  I2C_MasterInit(IMU_I2C_PERIPHERAL, &IMU_I2C_config, IMU_I2C_CLK_FREQ);
}

/***********************************************************************************************************************
 * MOTOR initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'MOTOR'
- type: 'ftm'
- mode: 'CenterAligned'
- custom_name_enabled: 'true'
- type_id: 'ftm_04a15ae4af2b404bf2ae403c3dbe98b3'
- functional_group: 'RTEPIP_Device'
- peripheral: 'FTM0'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - clockSourceFreq: 'GetFreq'
      - prescale: 'kFTM_Prescale_Divide_1'
      - timerFrequency: '20000'
      - bdmMode: 'kFTM_BdmMode_3'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: 'kFTM_CntMax kFTM_CntMin'
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_center_aligned_mode:
    - ftm_center_aligned_channels_config:
      - 0:
        - chnlNumber: 'kFTM_Chnl_0'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
      - 1:
        - chnlNumber: 'kFTM_Chnl_1'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
      - 2:
        - chnlNumber: 'kFTM_Chnl_2'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
      - 3:
        - chnlNumber: 'kFTM_Chnl_3'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t MOTOR_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_3,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = kFTM_CntMax | kFTM_CntMin,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_chnl_pwm_signal_param_t MOTOR_centerPwmSignalParams[] = { 
  {
    .chnlNumber = kFTM_Chnl_0,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  },
  {
    .chnlNumber = kFTM_Chnl_1,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  },
  {
    .chnlNumber = kFTM_Chnl_2,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  },
  {
    .chnlNumber = kFTM_Chnl_3,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  }
};

static void MOTOR_init(void) {
  FTM_Init(MOTOR_PERIPHERAL, &MOTOR_config);
  FTM_SetupPwm(MOTOR_PERIPHERAL, MOTOR_centerPwmSignalParams, sizeof(MOTOR_centerPwmSignalParams) / sizeof(ftm_chnl_pwm_signal_param_t), kFTM_CenterAlignedPwm, 20000U, MOTOR_CLOCK_SOURCE);
  FTM_StartTimer(MOTOR_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * OLED_SPI initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'OLED_SPI'
- type: 'dspi'
- mode: 'DSPI_Polling'
- custom_name_enabled: 'true'
- type_id: 'dspi_305e5b03c593d065f61ded8061d15797'
- functional_group: 'RTEPIP_Device'
- peripheral: 'SPI2'
- config_sets:
  - fsl_dspi:
    - dspi_mode: 'kDSPI_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - dspi_master_config:
      - whichCtar: 'kDSPI_Ctar0'
      - ctarConfig:
        - baudRate: '10000000'
        - bitsPerFrame: '8'
        - cpol: 'kDSPI_ClockPolarityActiveHigh'
        - cpha: 'kDSPI_ClockPhaseFirstEdge'
        - direction: 'kDSPI_MsbFirst'
        - pcsToSckDelayInNanoSec: '1000'
        - lastSckToPcsDelayInNanoSec: '1000'
        - betweenTransferDelayInNanoSec: '1000'
      - whichPcs: 'PCS0_SS'
      - pcsActiveHighOrLow: 'kDSPI_PcsActiveLow'
      - enableContinuousSCK: 'false'
      - enableRxFifoOverWrite: 'false'
      - enableModifiedTimingFormat: 'false'
      - samplePoint: 'kDSPI_SckToSin0Clock'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const dspi_master_config_t OLED_SPI_config = {
  .whichCtar = kDSPI_Ctar0,
  .ctarConfig = {
    .baudRate = 10000000,
    .bitsPerFrame = 8,
    .cpol = kDSPI_ClockPolarityActiveHigh,
    .cpha = kDSPI_ClockPhaseFirstEdge,
    .direction = kDSPI_MsbFirst,
    .pcsToSckDelayInNanoSec = 1000,
    .lastSckToPcsDelayInNanoSec = 1000,
    .betweenTransferDelayInNanoSec = 1000
  },
  .whichPcs = kDSPI_Pcs0,
  .pcsActiveHighOrLow = kDSPI_PcsActiveLow,
  .enableContinuousSCK = false,
  .enableRxFifoOverWrite = false,
  .enableModifiedTimingFormat = false,
  .samplePoint = kDSPI_SckToSin0Clock
};

static void OLED_SPI_init(void) {
  /* Initialization function */
  DSPI_MasterInit(OLED_SPI_PERIPHERAL, &OLED_SPI_config, OLED_SPI_CLK_FREQ);
}

/***********************************************************************************************************************
 * SERVO initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SERVO'
- type: 'ftm'
- mode: 'CenterAligned'
- custom_name_enabled: 'true'
- type_id: 'ftm_04a15ae4af2b404bf2ae403c3dbe98b3'
- functional_group: 'RTEPIP_Device'
- peripheral: 'FTM3'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - clockSourceFreq: 'GetFreq'
      - prescale: 'kFTM_Prescale_Divide_16'
      - timerFrequency: '50'
      - bdmMode: 'kFTM_BdmMode_3'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: 'kFTM_CntMin'
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM3_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_center_aligned_mode:
    - ftm_center_aligned_channels_config:
      - 0:
        - chnlNumber: 'kFTM_Chnl_7'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '30'
        - enable_chan_irq: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t SERVO_config = {
  .prescale = kFTM_Prescale_Divide_16,
  .bdmMode = kFTM_BdmMode_3,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = kFTM_CntMin,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_chnl_pwm_signal_param_t SERVO_centerPwmSignalParams[] = { 
  {
    .chnlNumber = kFTM_Chnl_7,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 30
  }
};

static void SERVO_init(void) {
  FTM_Init(SERVO_PERIPHERAL, &SERVO_config);
  FTM_SetupPwm(SERVO_PERIPHERAL, SERVO_centerPwmSignalParams, sizeof(SERVO_centerPwmSignalParams) / sizeof(ftm_chnl_pwm_signal_param_t), kFTM_CenterAlignedPwm, 50U, SERVO_CLOCK_SOURCE);
  FTM_StartTimer(SERVO_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * WLAN_UART initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'WLAN_UART'
- type: 'uart'
- mode: 'polling'
- custom_name_enabled: 'true'
- type_id: 'uart_88ab1eca0cddb7ee407685775de016d5'
- functional_group: 'RTEPIP_Device'
- peripheral: 'UART0'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '921600'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t WLAN_UART_config = {
  .baudRate_Bps = 921600,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void WLAN_UART_init(void) {
  UART_Init(WLAN_UART_PERIPHERAL, &WLAN_UART_config, WLAN_UART_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void RTEPIP_Basic(void)
{
  /* Global initialization */
  DMAMUX_Init(DMA_DMAMUX_BASEADDR);
  EDMA_Init(DMA_DMA_BASEADDR, &DMA_config);

  /* Initialize components */
  DMA_init();
  GPIOA_init();
  GPIOB_init();
  GPIOC_init();
  GPIOD_init();
  GPIOE_init();
  LPTMR0_init();
  PIT_init();
}

void RTEPIP_Device(void)
{
  /* Initialize components */
  CAM_UART_init();
  DBG_LPUART_init();
  EMAG_init();
  ENCO_L_init();
  ENCO_R_init();
  IMU_I2C_init();
  MOTOR_init();
  OLED_SPI_init();
  SERVO_init();
  WLAN_UART_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
}
