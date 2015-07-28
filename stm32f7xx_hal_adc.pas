(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_adc.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of ADC HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
   *)

unit stm32f7xx_hal_adc;

interface

uses
  stm32f7xx_hal, stm32f7xx_defs, stm32f7xx_hal_dma;

(**
  * @brief  HAL State structures definition
   *)

const
  HAL_ADC_STATE_RESET = $00;  (*!< ADC not yet initialized or disabled  *)
  HAL_ADC_STATE_READY = $01;  (*!< ADC peripheral ready for use  *)
  HAL_ADC_STATE_BUSY = $02;  (*!< An internal process is ongoing  *)
  HAL_ADC_STATE_BUSY_REG = $12;  (*!< Regular conversion is ongoing  *)
  HAL_ADC_STATE_BUSY_INJ = $22;  (*!< Injected conversion is ongoing  *)
  HAL_ADC_STATE_BUSY_INJ_REG = $32;  (*!< Injected and regular conversion are ongoing  *)
  HAL_ADC_STATE_TIMEOUT = $03;  (*!< Timeout state  *)
  HAL_ADC_STATE_ERROR = $04;  (*!< ADC state error  *)
  HAL_ADC_STATE_EOC = $05;  (*!< Conversion is completed  *)
  HAL_ADC_STATE_EOC_REG = $15;  (*!< Regular conversion is completed  *)
  HAL_ADC_STATE_EOC_INJ = $25;  (*!< Injected conversion is completed  *)
  HAL_ADC_STATE_EOC_INJ_REG = $35;  (*!< Injected and regular conversion are completed  *)
  HAL_ADC_STATE_AWD = $06;  (*!< ADC state analog watchdog  *)

type
  HAL_ADC_StateTypeDef = integer;

  (**
  * @brief   ADC Init structure definition
   *)

  ADC_InitTypeDef = record
    ClockPrescaler: longword;  (*!< Select the frequency of the clock to the ADC. The clock is common for
                                       all the ADCs.
                                       This parameter can be a value of @ref ADC_ClockPrescaler  *)
    Resolution: longword;  (*!< Configures the ADC resolution dual mode.
                                       This parameter can be a value of @ref ADC_Resolution  *)
    DataAlign: longword;  (*!< Specifies whether the ADC data  alignment is left or right.
                                       This parameter can be a value of @ref ADC_data_align  *)
    ScanConvMode: longword;  (*!< Specifies whether the conversion is performed in Scan (multi channels) or
                                       Single (one channel) mode.
                                       This parameter can be set to ENABLE or DISABLE  *)
    EOCSelection: longword;  (*!< Specifies whether the EOC flag is set
                                       at the end of single channel conversion or at the end of all conversions.
                                       This parameter can be a value of @ref ADC_EOCSelection  *)
    ContinuousConvMode: longword;  (*!< Specifies whether the conversion is performed in Continuous or Single mode.
                                       This parameter can be set to ENABLE or DISABLE.  *)
    DMAContinuousRequests: longword;  (*!< Specifies whether the DMA requests is performed in Continuous or in Single mode.
                                       This parameter can be set to ENABLE or DISABLE.  *)
    NbrOfConversion: longword;  (*!< Specifies the number of ADC conversions that will be done using the sequencer for
                                       regular channel group.
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 16.  *)
    DiscontinuousConvMode: longword;  (*!< Specifies whether the conversion is performed in Discontinuous or not
                                       for regular channels.
                                       This parameter can be set to ENABLE or DISABLE.  *)
    NbrOfDiscConversion: longword;  (*!< Specifies the number of ADC discontinuous conversions that will be done
                                       using the sequencer for regular channel group.
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 8.  *)
    ExternalTrigConv: longword;  (*!< Selects the external event used to trigger the conversion start of regular group.
                                       If set to ADC_SOFTWARE_START, external triggers are disabled.
                                       This parameter can be a value of @ref ADC_External_trigger_Source_Regular
                                       Note: This parameter can be modified only if there is no conversion is ongoing.  *)
    ExternalTrigConvEdge: longword;  (*!< Selects the external trigger edge of regular group.
                                       If trigger is set to ADC_SOFTWARE_START, this parameter is discarded.
                                       This parameter can be a value of @ref ADC_External_trigger_edge_Regular
                                       Note: This parameter can be modified only if there is no conversion is ongoing.  *)
  end;

  (**
  * @brief  ADC handle Structure definition
   *)

  PADC_HandleTypeDef = ^ADC_HandleTypeDef;

  ADC_HandleTypeDef = record
    Instance: ^TADC1_Registers;  (*!< Register base address  *)
    Init: ADC_InitTypeDef;  (*!< ADC required parameters  *)
    NbrOfCurrentConversionRank: longword;  (*!< ADC number of current conversion rank  *)
    DMA_Handle: ^__DMA_HandleTypeDef;  (*!< Pointer DMA Handler  *)
    Lock: HAL_LockTypeDef;  (*!< ADC locking object  *)
    State: HAL_ADC_StateTypeDef;  (*!< ADC communication state  *)
    ErrorCode: longword;  (*!< ADC Error code  *)
  end;

  (**
  * @brief   ADC Configuration regular Channel structure definition
   *)

  ADC_ChannelConfTypeDef = record
    Channel: longword;  (*!< The ADC channel to configure.
                                This parameter can be a value of @ref ADC_channels  *)
    Rank: longword;  (*!< The rank in the regular group sequencer.
                                This parameter must be a number between Min_Data = 1 and Max_Data = 16  *)
    SamplingTime: longword;  (*!< The sample time value to be set for the selected channel.
                                This parameter can be a value of @ref ADC_sampling_times  *)
    Offset: longword;  (*!< Reserved for future use, can be set to 0  *)
  end;

  (**
  * @brief   ADC Configuration multi-mode structure definition
   *)

  ADC_AnalogWDGConfTypeDef = record
    WatchdogMode: longword;  (*!< Configures the ADC analog watchdog mode.
                                   This parameter can be a value of @ref ADC_analog_watchdog_selection  *)
    HighThreshold: longword;  (*!< Configures the ADC analog watchdog High threshold value.
                                   This parameter must be a 12-bit value.  *)
    LowThreshold: longword;  (*!< Configures the ADC analog watchdog High threshold value.
                                   This parameter must be a 12-bit value.  *)
    Channel: longword;  (*!< Configures ADC channel for the analog watchdog.
                                   This parameter has an effect only if watchdog mode is configured on single channel
                                   This parameter can be a value of @ref ADC_channels  *)
    ITMode: longword;  (*!< Specifies whether the analog watchdog is configured
                                   is interrupt mode or in polling mode.
                                   This parameter can be set to ENABLE or DISABLE  *)
    WatchdogNumber: longword;  (*!< Reserved for future use, can be set to 0  *)
  end;

  (**
  * @end
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup ADC_Exported_Constants ADC Exported Constants
  * @begin
   *)


  (** @defgroup ADC_Error_Code ADC Error Code
  * @begin
   *)

const
  HAL_ADC_ERROR_NONE = ($00);  (*!< No error              *)
  HAL_ADC_ERROR_OVR = ($01);  (*!< OVR error             *)
  HAL_ADC_ERROR_DMA = ($02);  (*!< DMA transfer error    *)
  (**
  * @end
   *)


  (** @defgroup ADC_ClockPrescaler ADC Clock Prescaler
  * @begin
   *)

  ADC_CLOCK_SYNC_PCLK_DIV2 = ($00000000);
  ADC_CLOCK_SYNC_PCLK_DIV4 = (ADC_CCR_ADCPRE_0);
  ADC_CLOCK_SYNC_PCLK_DIV6 = (ADC_CCR_ADCPRE_1);
  ADC_CLOCK_SYNC_PCLK_DIV8 = (ADC_CCR_ADCPRE);
  (**
  * @end
   *)

  (** @defgroup ADC_delay_between_2_sampling_phases ADC Delay Between 2 Sampling Phases
  * @begin
   *)

  ADC_TWOSAMPLINGDELAY_5CYCLES = ($00000000);
  ADC_TWOSAMPLINGDELAY_6CYCLES = (ADC_CCR_DELAY_0);
  ADC_TWOSAMPLINGDELAY_7CYCLES = (ADC_CCR_DELAY_1);
  ADC_TWOSAMPLINGDELAY_8CYCLES = ((ADC_CCR_DELAY_1 or ADC_CCR_DELAY_0));
  ADC_TWOSAMPLINGDELAY_9CYCLES = (ADC_CCR_DELAY_2);
  ADC_TWOSAMPLINGDELAY_10CYCLES = ((ADC_CCR_DELAY_2 or ADC_CCR_DELAY_0));
  ADC_TWOSAMPLINGDELAY_11CYCLES = ((ADC_CCR_DELAY_2 or ADC_CCR_DELAY_1));
  ADC_TWOSAMPLINGDELAY_12CYCLES = ((ADC_CCR_DELAY_2 or ADC_CCR_DELAY_1 or ADC_CCR_DELAY_0));
  ADC_TWOSAMPLINGDELAY_13CYCLES = (ADC_CCR_DELAY_3);
  ADC_TWOSAMPLINGDELAY_14CYCLES = ((ADC_CCR_DELAY_3 or ADC_CCR_DELAY_0));
  ADC_TWOSAMPLINGDELAY_15CYCLES = ((ADC_CCR_DELAY_3 or ADC_CCR_DELAY_1));
  ADC_TWOSAMPLINGDELAY_16CYCLES = ((ADC_CCR_DELAY_3 or ADC_CCR_DELAY_1 or ADC_CCR_DELAY_0));
  ADC_TWOSAMPLINGDELAY_17CYCLES = ((ADC_CCR_DELAY_3 or ADC_CCR_DELAY_2));
  ADC_TWOSAMPLINGDELAY_18CYCLES = ((ADC_CCR_DELAY_3 or ADC_CCR_DELAY_2 or ADC_CCR_DELAY_0));
  ADC_TWOSAMPLINGDELAY_19CYCLES = ((ADC_CCR_DELAY_3 or ADC_CCR_DELAY_2 or ADC_CCR_DELAY_1));
  ADC_TWOSAMPLINGDELAY_20CYCLES = (ADC_CCR_DELAY);
  (**
  * @end
   *)

  (** @defgroup ADC_Resolution ADC Resolution
  * @begin
   *)

  ADC_RESOLUTION_12B = ($00000000);
  ADC_RESOLUTION_10B = (ADC_CR1_RES_0);
  ADC_RESOLUTION_8B = (ADC_CR1_RES_1);
  ADC_RESOLUTION_6B = (ADC_CR1_RES);
  (**
  * @end
   *)

  (** @defgroup ADC_External_trigger_edge_Regular ADC External Trigger Edge Regular
  * @begin
   *)

  ADC_EXTERNALTRIGCONVEDGE_NONE = ($00000000);
  ADC_EXTERNALTRIGCONVEDGE_RISING = (ADC_CR2_EXTEN_0);
  ADC_EXTERNALTRIGCONVEDGE_FALLING = (ADC_CR2_EXTEN_1);
  ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING = (ADC_CR2_EXTEN);
  (**
  * @end
   *)

  (** @defgroup ADC_External_trigger_Source_Regular ADC External Trigger Source Regular
  * @begin
   *)

  (* Note: Parameter ADC_SOFTWARE_START is a software parameter used for         *)

  (*       compatibility with other STM32 devices.                               *)

  ADC_EXTERNALTRIGCONV_T1_CC1 = ($00000000);
  ADC_EXTERNALTRIGCONV_T1_CC2 = (ADC_CR2_EXTSEL_0);
  ADC_EXTERNALTRIGCONV_T1_CC3 = (ADC_CR2_EXTSEL_1);
  ADC_EXTERNALTRIGCONV_T2_CC2 = ((ADC_CR2_EXTSEL_1 or ADC_CR2_EXTSEL_0));
  ADC_EXTERNALTRIGCONV_T5_TRGO = (ADC_CR2_EXTSEL_2);
  ADC_EXTERNALTRIGCONV_T4_CC4 = ((ADC_CR2_EXTSEL_2 or ADC_CR2_EXTSEL_0));
  ADC_EXTERNALTRIGCONV_T3_CC4 = ((ADC_CR2_EXTSEL_2 or ADC_CR2_EXTSEL_1));
  ADC_EXTERNALTRIGCONV_T8_TRGO = ((ADC_CR2_EXTSEL_2 or ADC_CR2_EXTSEL_1 or ADC_CR2_EXTSEL_0));
  ADC_EXTERNALTRIGCONV_T8_TRGO2 = (ADC_CR2_EXTSEL_3);
  ADC_EXTERNALTRIGCONV_T1_TRGO = ((ADC_CR2_EXTSEL_3 or ADC_CR2_EXTSEL_0));
  ADC_EXTERNALTRIGCONV_T1_TRGO2 = ((ADC_CR2_EXTSEL_3 or ADC_CR2_EXTSEL_1));
  ADC_EXTERNALTRIGCONV_T2_TRGO = ((ADC_CR2_EXTSEL_3 or ADC_CR2_EXTSEL_1 or ADC_CR2_EXTSEL_0));
  ADC_EXTERNALTRIGCONV_T4_TRGO = ((ADC_CR2_EXTSEL_3 or ADC_CR2_EXTSEL_2));
  ADC_EXTERNALTRIGCONV_T6_TRGO = ((ADC_CR2_EXTSEL_3 or ADC_CR2_EXTSEL_2 or ADC_CR2_EXTSEL_0));
  ADC_EXTERNALTRIGCONV_EXT_IT11 = (ADC_CR2_EXTSEL);
  ADC_SOFTWARE_START = (ADC_CR2_EXTSEL + 1);
  (**
  * @end
   *)

  (** @defgroup ADC_data_align ADC Data Align
  * @begin
   *)

  ADC_DATAALIGN_RIGHT = ($00000000);
  ADC_DATAALIGN_LEFT = (ADC_CR2_ALIGN);
  (**
  * @end
   *)

  (** @defgroup ADC_channels ADC Common Channels
  * @begin
   *)

  ADC_CHANNEL_0 = ($00000000);
  ADC_CHANNEL_1 = (ADC_CR1_AWDCH_0);
  ADC_CHANNEL_2 = (ADC_CR1_AWDCH_1);
  ADC_CHANNEL_3 = ((ADC_CR1_AWDCH_1 or ADC_CR1_AWDCH_0));
  ADC_CHANNEL_4 = (ADC_CR1_AWDCH_2);
  ADC_CHANNEL_5 = ((ADC_CR1_AWDCH_2 or ADC_CR1_AWDCH_0));
  ADC_CHANNEL_6 = ((ADC_CR1_AWDCH_2 or ADC_CR1_AWDCH_1));
  ADC_CHANNEL_7 = ((ADC_CR1_AWDCH_2 or ADC_CR1_AWDCH_1 or ADC_CR1_AWDCH_0));
  ADC_CHANNEL_8 = (ADC_CR1_AWDCH_3);
  ADC_CHANNEL_9 = ((ADC_CR1_AWDCH_3 or ADC_CR1_AWDCH_0));
  ADC_CHANNEL_10 = ((ADC_CR1_AWDCH_3 or ADC_CR1_AWDCH_1));
  ADC_CHANNEL_11 = ((ADC_CR1_AWDCH_3 or ADC_CR1_AWDCH_1 or ADC_CR1_AWDCH_0));
  ADC_CHANNEL_12 = ((ADC_CR1_AWDCH_3 or ADC_CR1_AWDCH_2));
  ADC_CHANNEL_13 = ((ADC_CR1_AWDCH_3 or ADC_CR1_AWDCH_2 or ADC_CR1_AWDCH_0));
  ADC_CHANNEL_14 = ((ADC_CR1_AWDCH_3 or ADC_CR1_AWDCH_2 or ADC_CR1_AWDCH_1));
  ADC_CHANNEL_15 = ((ADC_CR1_AWDCH_3 or ADC_CR1_AWDCH_2 or ADC_CR1_AWDCH_1 or ADC_CR1_AWDCH_0));
  ADC_CHANNEL_16 = (ADC_CR1_AWDCH_4);
  ADC_CHANNEL_17 = ((ADC_CR1_AWDCH_4 or ADC_CR1_AWDCH_0));
  ADC_CHANNEL_18 = ((ADC_CR1_AWDCH_4 or ADC_CR1_AWDCH_1));
  ADC_CHANNEL_VREFINT = (ADC_CHANNEL_17);
  ADC_CHANNEL_VBAT = (ADC_CHANNEL_18);
  (**
  * @end
   *)

  (** @defgroup ADC_sampling_times ADC Sampling Times
  * @begin
   *)

  ADC_SAMPLETIME_3CYCLES = ($00000000);
  ADC_SAMPLETIME_15CYCLES = (ADC_SMPR1_SMP10_0);
  ADC_SAMPLETIME_28CYCLES = (ADC_SMPR1_SMP10_1);
  ADC_SAMPLETIME_56CYCLES = ((ADC_SMPR1_SMP10_1 or ADC_SMPR1_SMP10_0));
  ADC_SAMPLETIME_84CYCLES = (ADC_SMPR1_SMP10_2);
  ADC_SAMPLETIME_112CYCLES = ((ADC_SMPR1_SMP10_2 or ADC_SMPR1_SMP10_0));
  ADC_SAMPLETIME_144CYCLES = ((ADC_SMPR1_SMP10_2 or ADC_SMPR1_SMP10_1));
  ADC_SAMPLETIME_480CYCLES = (ADC_SMPR1_SMP10);
  (**
  * @end
   *)

  (** @defgroup ADC_EOCSelection ADC EOC Selection
  * @begin
   *)

  ADC_EOC_SEQ_CONV = ($00000000);
  ADC_EOC_SINGLE_CONV = ($00000001);
  ADC_EOC_SINGLE_SEQ_CONV = ($00000002);  (*!< reserved for future use  *)
  (**
  * @end
   *)

  ADC_FLAG_AWD = (ADC_SR_AWD);
  ADC_FLAG_OVR = (ADC_SR_OVR);

  (** @defgroup ADC_Event_type ADC Event Type
  * @begin
   *)

  ADC_AWD_EVENT = (ADC_FLAG_AWD);
  ADC_OVR_EVENT = (ADC_FLAG_OVR);
  (**
  * @end
   *)

  (** @defgroup ADC_analog_watchdog_selection ADC Analog Watchdog Selection
  * @begin
   *)

  ADC_ANALOGWATCHDOG_SINGLE_REG = ((ADC_CR1_AWDSGL or ADC_CR1_AWDEN));
  ADC_ANALOGWATCHDOG_SINGLE_INJEC = ((ADC_CR1_AWDSGL or ADC_CR1_JAWDEN));
  ADC_ANALOGWATCHDOG_SINGLE_REGINJEC = ((ADC_CR1_AWDSGL or ADC_CR1_AWDEN or ADC_CR1_JAWDEN));
  ADC_ANALOGWATCHDOG_ALL_REG = (ADC_CR1_AWDEN);
  ADC_ANALOGWATCHDOG_ALL_INJEC = (ADC_CR1_JAWDEN);
  ADC_ANALOGWATCHDOG_ALL_REGINJEC = ((ADC_CR1_AWDEN or ADC_CR1_JAWDEN));
  ADC_ANALOGWATCHDOG_NONE = ($00000000);
  (**
  * @end
   *)

  (** @defgroup ADC_interrupts_definition ADC Interrupts Definition
  * @begin
   *)

  ADC_IT_EOC = (ADC_CR1_EOCIE);
  ADC_IT_AWD = (ADC_CR1_AWDIE);
  ADC_IT_JEOC = (ADC_CR1_JEOCIE);
  ADC_IT_OVR = (ADC_CR1_OVRIE);
  (**
  * @end
   *)

  (** @defgroup ADC_flags_definition ADC Flags Definition
  * @begin
   *)

  ADC_FLAG_EOC = (ADC_SR_EOC);
  ADC_FLAG_JEOC = (ADC_SR_JEOC);
  ADC_FLAG_JSTRT = (ADC_SR_JSTRT);
  ADC_FLAG_STRT = (ADC_SR_STRT);
  (**
  * @end
   *)

  (** @defgroup ADC_channels_type ADC Channels Type
  * @begin
   *)

  ADC_ALL_CHANNELS = ($00000001);
  ADC_REGULAR_CHANNELS = ($00000002);  (*!< reserved for future use  *)
  ADC_INJECTED_CHANNELS = ($00000003);  (*!< reserved for future use  *)

  (* Delay for ADC stabilization time.                                         *)
  (* Maximum delay is 1us (refer to device datasheet, parameter tSTAB).        *)
  (* Unit: us                                                                  *)
  ADC_STAB_DELAY_US = (3);
  (* Delay for temperature sensor stabilization time.                          *)
  (* Maximum delay is 10us (refer to device datasheet, parameter tSTART).      *)
  (* Unit: us                                                                  *)
  ADC_TEMPSENSOR_DELAY_US = (10);

function HAL_ADC_Init(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADC_DeInit(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_ADC_MspInit(var hadc: ADC_HandleTypeDef); external name 'HAL_ADC_MspInit';
procedure HAL_ADC_MspDeInit(var hadc: ADC_HandleTypeDef); external name 'HAL_ADC_MspDeInit';

(* I/O operation functions ***************************************************** *)
function HAL_ADC_Start(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADC_Stop(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADC_PollForConversion(var hadc: ADC_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
function HAL_ADC_PollForEvent(var hadc: ADC_HandleTypeDef; EventType, Timeout: longword): HAL_StatusTypeDef;
function HAL_ADC_Start_IT(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADC_Stop_IT(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_ADC_IRQHandler(var hadc: ADC_HandleTypeDef);
function HAL_ADC_Start_DMA(var hadc: ADC_HandleTypeDef; var pData: longword; Length: longword): HAL_StatusTypeDef;
function HAL_ADC_Stop_DMA(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADC_GetValue(var hadc: ADC_HandleTypeDef): longword;

procedure HAL_ADC_ConvCpltCallback(var hadc: ADC_HandleTypeDef); external name 'HAL_ADC_ConvCpltCallback';
procedure HAL_ADC_ConvHalfCpltCallback(var hadc: ADC_HandleTypeDef); external name 'HAL_ADC_ConvHalfCpltCallback';
procedure HAL_ADC_LevelOutOfWindowCallback(var hadc: ADC_HandleTypeDef); external name 'HAL_ADC_LevelOutOfWindowCallback';
procedure HAL_ADC_ErrorCallback(var hadc: ADC_HandleTypeDef); external name 'HAL_ADC_ErrorCallback';

(* Peripheral Control functions ************************************************ *)
function HAL_ADC_ConfigChannel(var hadc: ADC_HandleTypeDef; const sConfig: ADC_ChannelConfTypeDef): HAL_StatusTypeDef;
function HAL_ADC_AnalogWDGConfig(var hadc: ADC_HandleTypeDef; const AnalogWDGConfig: ADC_AnalogWDGConfTypeDef): HAL_StatusTypeDef;

(* Peripheral State functions ************************************************** *)
function HAL_ADC_GetState(var hadc: ADC_HandleTypeDef): HAL_ADC_StateTypeDef;
function HAL_ADC_GetError(var hadc: ADC_HandleTypeDef): longword;

function ADC_SQR1(_NbrOfConversion_: longword): longword;
function ADC_SMPR1(_SAMPLETIME_, _CHANNELNB_: longword): longword;
function ADC_SMPR2(_SAMPLETIME_, _CHANNELNB_: longword): longword;
function ADC_SQR3_RK(_CHANNELNB_, _RANKNB_: longword): longword;
function ADC_SQR2_RK(_CHANNELNB_, _RANKNB_: longword): longword;
function ADC_SQR1_RK(_CHANNELNB_, _RANKNB_: longword): longword;
function ADC_CR2_CONTINUOUS(_CONTINUOUS_MODE_: longword): longword;
function ADC_CR1_DISCONTINUOUS(_NBR_DISCONTINUOUSCONV_: longword): longword;
function ADC_CR1_SCANCONV(_SCANCONV_MODE_: longword): longword;
function ADC_CR2_EOCSelection(_EOCSelection_MODE_: longword): longword;
function ADC_CR2_DMAContReq(_DMAContReq_MODE_: longword): longword;
function ADC_GET_RESOLUTION(var __HANDLE__: ADC_HandleTypeDef): longword;

procedure __HAL_ADC_RESET_HANDLE_STATE(var __HANDLE__: ADC_HandleTypeDef);
procedure __HAL_ADC_ENABLE(var __HANDLE__: ADC_HandleTypeDef);
procedure __HAL_ADC_DISABLE(var __HANDLE__: ADC_HandleTypeDef);
procedure __HAL_ADC_ENABLE_IT(var __HANDLE__: ADC_HandleTypeDef; __INTERRUPT__: longword);
procedure __HAL_ADC_DISABLE_IT(var __HANDLE__: ADC_HandleTypeDef; __INTERRUPT__: longword);
function __HAL_ADC_GET_IT_SOURCE(var __HANDLE__: ADC_HandleTypeDef; __INTERRUPT__: longword): boolean;
procedure __HAL_ADC_CLEAR_FLAG(var __HANDLE__: ADC_HandleTypeDef; __FLAG__: longword);
function __HAL_ADC_GET_FLAG(var __HANDLE__: ADC_HandleTypeDef; __FLAG__: longword): boolean;

implementation

uses
  stm32f7xx_hal_adc_ex;

procedure __HAL_ADC_RESET_HANDLE_STATE(var __HANDLE__: ADC_HandleTypeDef);
begin
  (__HANDLE__).State := HAL_ADC_STATE_RESET;
end;

procedure __HAL_ADC_ENABLE(var __HANDLE__: ADC_HandleTypeDef);
begin
  (__HANDLE__).Instance^.CR2 := (__HANDLE__).Instance^.CR2 or ADC_CR2_ADON;
end;

procedure __HAL_ADC_DISABLE(var __HANDLE__: ADC_HandleTypeDef);
begin
  (__HANDLE__).Instance^.CR2 := (__HANDLE__).Instance^.CR2 and (not ADC_CR2_ADON);
end;

procedure __HAL_ADC_ENABLE_IT(var __HANDLE__: ADC_HandleTypeDef; __INTERRUPT__: longword);
begin
  ((__HANDLE__).Instance^.CR1) := ((__HANDLE__).Instance^.CR1) or (__INTERRUPT__);
end;

procedure __HAL_ADC_DISABLE_IT(var __HANDLE__: ADC_HandleTypeDef; __INTERRUPT__: longword);
begin
  ((__HANDLE__).Instance^.CR1) := ((__HANDLE__).Instance^.CR1) and (not __INTERRUPT__);
end;

function __HAL_ADC_GET_IT_SOURCE(var __HANDLE__: ADC_HandleTypeDef; __INTERRUPT__: longword): boolean;
begin
  exit(((__HANDLE__).Instance^.CR1 and (__INTERRUPT__)) = (__INTERRUPT__));
end;

procedure __HAL_ADC_CLEAR_FLAG(var __HANDLE__: ADC_HandleTypeDef; __FLAG__: longword);
begin
  ((__HANDLE__).Instance^.SR) := not __FLAG__;
end;

function __HAL_ADC_GET_FLAG(var __HANDLE__: ADC_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit((((__HANDLE__).Instance^.SR) and (__FLAG__)) = (__FLAG__));
end;

(**
  * @brief  Set ADC Regular channel sequence length.
  * @param  _NbrOfConversion_: Regular channel sequence length.
  * @retval None
  *)
function ADC_SQR1(_NbrOfConversion_: longword): longword;
begin
  exit(((_NbrOfConversion_) - 1) shl 20);
end;

(**
  * @brief  Set the ADC's sample time for channel numbers between 10 and 18.
  * @param  _SAMPLETIME_: Sample time parameter.
  * @param  _CHANNELNB_: Channel number.
  * @retval None
  *)
  function ADC_SMPR1(_SAMPLETIME_, _CHANNELNB_: longword): longword;
begin
  exit((_SAMPLETIME_) shl (3 * ((((_CHANNELNB_))) - 10)));
end;

(**
  * @brief  Set the ADC's sample time for channel numbers between 0 and 9.
  * @param  _SAMPLETIME_: Sample time parameter.
  * @param  _CHANNELNB_: Channel number.
  * @retval None
  *)
  function ADC_SMPR2(_SAMPLETIME_, _CHANNELNB_: longword): longword;
begin
  exit((_SAMPLETIME_) shl (3 * (((_CHANNELNB_)))));
end;

(**
  * @brief  Set the selected regular channel rank for rank between 1 and 6.
  * @param  _CHANNELNB_: Channel number.
  * @param  _RANKNB_: Rank number.
  * @retval None
  *)
  function ADC_SQR3_RK(_CHANNELNB_, _RANKNB_: longword): longword;
begin
  exit((((_CHANNELNB_))) shl (5 * ((_RANKNB_) - 1)));
end;

(**
  * @brief  Set the selected regular channel rank for rank between 7 and 12.
  * @param  _CHANNELNB_: Channel number.
  * @param  _RANKNB_: Rank number.
  * @retval None
  *)
  function ADC_SQR2_RK(_CHANNELNB_, _RANKNB_: longword): longword;
begin
  exit((((_CHANNELNB_))) shl (5 * ((_RANKNB_) - 7)));
end;

(**
  * @brief  Set the selected regular channel rank for rank between 13 and 16.
  * @param  _CHANNELNB_: Channel number.
  * @param  _RANKNB_: Rank number.
  * @retval None
  *)
  function ADC_SQR1_RK(_CHANNELNB_, _RANKNB_: longword): longword;
begin
  exit((((_CHANNELNB_))) shl (5 * ((_RANKNB_) - 13)));
end;

(**
  * @brief  Enable ADC continuous conversion mode.
  * @param  _CONTINUOUS_MODE_: Continuous mode.
  * @retval None
  *)
  function ADC_CR2_CONTINUOUS(_CONTINUOUS_MODE_: longword): longword;
begin
  exit((_CONTINUOUS_MODE_) shl 1);
end;

(**
  * @brief  Configures the number of discontinuous conversions for the regular group channels.
  * @param  _NBR_DISCONTINUOUSCONV_: Number of discontinuous conversions.
  * @retval None
  *)
  function ADC_CR1_DISCONTINUOUS(_NBR_DISCONTINUOUSCONV_: longword): longword;
begin
  exit(((_NBR_DISCONTINUOUSCONV_) - 1) shl BsfDWord(ADC_CR1_DISCNUM));
end;

(**
  * @brief  Enable ADC scan mode.
  * @param  _SCANCONV_MODE_: Scan conversion mode.
  * @retval None
  *)
  function ADC_CR1_SCANCONV(_SCANCONV_MODE_: longword): longword;
begin
  exit((_SCANCONV_MODE_) shl 8);
end;

(**
  * @brief  Enable the ADC end of conversion selection.
  * @param  _EOCSelection_MODE_: End of conversion selection mode.
  * @retval None
  *)
  function ADC_CR2_EOCSelection(_EOCSelection_MODE_: longword): longword;
begin
  exit((_EOCSelection_MODE_) shl 10);
end;

(**
  * @brief  Enable the ADC DMA continuous request.
  * @param  _DMAContReq_MODE_: DMA continuous request mode.
  * @retval None
  *)
  function ADC_CR2_DMAContReq(_DMAContReq_MODE_: longword): longword;
begin
  exit((_DMAContReq_MODE_) shl 9);
end;

(**
  * @brief Return resolution bits in CR1 register.
  * @param __HANDLE__: ADC handle
  * @retval None
  *)
  function ADC_GET_RESOLUTION(var __HANDLE__: ADC_HandleTypeDef): longword;
begin
  exit((__HANDLE__.Instance^.CR1) and ADC_CR1_RES);
end;

procedure ADC_Init(var hadc: ADC_HandleTypeDef);
begin
  (* Set ADC parameters *)
  (* Set the ADC clock prescaler *)
  C_ADC.CCR := C_ADC.CCR and (not (ADC_CCR_ADCPRE));
  C_ADC.CCR := C_ADC.CCR or hadc.Init.ClockPrescaler;

  (* Set ADC scan mode *)
  hadc.Instance^.CR1 := hadc.Instance^.CR1 and (not (ADC_CR1_SCAN));
  hadc.Instance^.CR1 := hadc.Instance^.CR1 or ADC_CR1_SCANCONV(hadc.Init.ScanConvMode);

  (* Set ADC resolution *)
  hadc.Instance^.CR1 := hadc.Instance^.CR1 and (not (ADC_CR1_RES));
  hadc.Instance^.CR1 := hadc.Instance^.CR1 or hadc.Init.Resolution;

  (* Set ADC data alignment *)
  hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_ALIGN));
  hadc.Instance^.CR2 := hadc.Instance^.CR2 or hadc.Init.DataAlign;

  (* Enable external trigger if trigger selection is different of software  *)
  (* start.                                                                 *)
  (* Note: This configuration keeps the hardware feature of parameter       *)
  (*       ExternalTrigConvEdge "trigger edge none" equivalent to           *)
  (*       software start.                                                  *)
  if(hadc.Init.ExternalTrigConv <> ADC_SOFTWARE_START) then
  begin
    (* Select external trigger to start conversion *)
    hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_EXTSEL));
    hadc.Instance^.CR2 := hadc.Instance^.CR2 or hadc.Init.ExternalTrigConv;

    (* Select external trigger polarity *)
    hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_EXTEN));
    hadc.Instance^.CR2 := hadc.Instance^.CR2 or hadc.Init.ExternalTrigConvEdge;
  end
  else
  begin
    (* Reset the external trigger *)
    hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_EXTSEL));
    hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_EXTEN));
  end;

  (* Enable or disable ADC continuous conversion mode *)
  hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_CONT));
  hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_CONTINUOUS(hadc.Init.ContinuousConvMode);

  if(hadc.Init.DiscontinuousConvMode <> 0)then
  begin
    (* Enable the selected ADC regular discontinuous mode *)
    hadc.Instance^.CR1 := hadc.Instance^.CR1 or ADC_CR1_DISCEN;

    (* Set the number of channels to be converted in discontinuous mode *)
    hadc.Instance^.CR1 := hadc.Instance^.CR1 and (not (ADC_CR1_DISCNUM));
    hadc.Instance^.CR1 := hadc.Instance^.CR1 or ADC_CR1_DISCONTINUOUS(hadc.Init.NbrOfDiscConversion);
  end
  else
  begin
    (* Disable the selected ADC regular discontinuous mode *)
    hadc.Instance^.CR1 := hadc.Instance^.CR1 and (not (ADC_CR1_DISCEN));
  end;

  (* Set ADC number of conversion *)
  hadc.Instance^.SQR1 := hadc.Instance^.SQR1 and (not (ADC_SQR1_L));
  hadc.Instance^.SQR1 := hadc.Instance^.SQR1 or ADC_SQR1(hadc.Init.NbrOfConversion);

  (* Enable or disable ADC DMA continuous request *)
  hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_DDS));
  hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_DMAContReq(hadc.Init.DMAContinuousRequests);

  (* Enable or disable ADC end of conversion selection *)
  hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_EOCS));
  hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_EOCSelection(hadc.Init.EOCSelection);
end;

function HAL_ADC_Init(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
begin
  if(hadc.State = HAL_ADC_STATE_RESET)then
  begin
    (* Allocate lock resource and initialize it *)
    hadc.Lock := HAL_UNLOCKED;
    (* Init the low level hardware *)
    HAL_ADC_MspInit(hadc);
  end;

  (* Initialize the ADC state *)
  hadc.State := HAL_ADC_STATE_BUSY;

  (* Set ADC parameters *)
  ADC_Init(hadc);

  (* Set ADC error code to none *)
  hadc.ErrorCode := HAL_ADC_ERROR_NONE;

  (* Initialize the ADC state *)
  hadc.State := HAL_ADC_STATE_READY;

  (* Release Lock *)
  __HAL_Unlock(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADC_DeInit(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_BUSY;

  (* DeInit the low level hardware *)
  HAL_ADC_MspDeInit(hadc);

  (* Set ADC error code to none *)
  hadc.ErrorCode := HAL_ADC_ERROR_NONE;

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_RESET;

  (* Return function status *)
  exit(HAL_OK);
end;

procedure HAL_ADC_MspInit_stub(var hadc: ADC_HandleTypeDef); assembler; nostackframe; public name 'HAL_ADC_MspInit';
  asm
    .weak HAL_ADC_MspInit
  end;

procedure HAL_ADC_MspDeInit_stub(var hadc: ADC_HandleTypeDef); assembler; nostackframe; public name 'HAL_ADC_MspDeInit';
  asm
    .weak HAL_ADC_MspDeInit
  end;

function HAL_ADC_Start(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
var
  counter: longword;
begin
  (* Process locked *)
  __HAL_LOCK(hadc.Lock);

  (* Check if an injected conversion is ongoing *)
  if hadc.State = HAL_ADC_STATE_BUSY_INJ then
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_BUSY_INJ_REG;
  end
  else
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_BUSY_REG;
  end;

    (* Check if ADC peripheral is disabled in order to enable it and wait during
    Tstab time the ADC's stabilization *)
  if (hadc.Instance^.CR2 and ADC_CR2_ADON) <> ADC_CR2_ADON then
  begin
    (* Enable the Peripheral *)
    __HAL_ADC_ENABLE(hadc);

    (* Delay for ADC stabilization time *)
    (* Compute number of CPU cycles to wait for *)
    counter := (ADC_STAB_DELAY_US * (SystemCoreClock div 1000000));
    while counter <> 0 do
    begin
      Dec(counter);
    end;
  end;

  (* Process unlocked *)
  __HAL_UNLOCK(hadc.lock);

  (* Check if Multimode enabled *)
  if HAL_IS_BIT_CLR(C_ADC.CCR, ADC_CCR_MULTI) then
  begin
    (* if no external trigger present enable software conversion of regular channels *)
    if (hadc.Instance^.CR2 and ADC_CR2_EXTEN) = 0 then
    begin
      (* Enable the selected ADC software conversion for regular group *)
      hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_SWSTART;
    end;
  end
  else
  begin
    (* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels *)
    if (hadc.Instance = @ADC1) and ((hadc.Instance^.CR2 and ADC_CR2_EXTEN) = 0) then
    begin
      (* Enable the selected ADC software conversion for regular group *)
      hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_SWSTART;
    end;
  end;

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADC_Stop(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Disable the Peripheral *)
  __HAL_ADC_DISABLE(hadc);

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_READY;

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADC_PollForConversion(var hadc: ADC_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  tickstart := 0;

  (* Verification that ADC configuration is compliant with polling for      *)
  (* each conversion:                                                       *)
  (* Particular case is ADC configured in DMA mode and ADC sequencer with   *)
  (* several ranks and polling for end of each conversion.                  *)
  (* For code simplicity sake, this particular case is generalized to       *)
  (* ADC configured in DMA mode and polling for end of each conversion.     *)
  if (HAL_IS_BIT_SET(hadc.Instance^.CR2, ADC_CR2_EOCS) and HAL_IS_BIT_SET(hadc.Instance^.CR2, ADC_CR2_DMA)) then
  begin
    (* Update ADC state machine to error *)
    hadc.State := HAL_ADC_STATE_ERROR;

    (* Process unlocked *)
    __HAL_UNLOCK(hadc.lock);

    exit(HAL_ERROR);
  end;

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Check End of conversion flag *)
  while (not (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) do
  begin
    (* Check for the Timeout *)
    if Timeout <> HAL_MAX_DELAY then
    begin
      if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
      begin
        hadc.State := HAL_ADC_STATE_TIMEOUT;
        (* Process unlocked *)
        __HAL_UNLOCK(hadc.lock);
        exit(HAL_TIMEOUT);
      end;
    end;
  end;

  (* Check if an injected conversion is ready *)
  if hadc.State = HAL_ADC_STATE_EOC_INJ then
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_EOC_INJ_REG;
  end
  else
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_EOC_REG;
  end;

  (* Return ADC state *)
  exit(HAL_OK);
end;

function HAL_ADC_PollForEvent(var hadc: ADC_HandleTypeDef; EventType, Timeout: longword): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  tickstart := 0;

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Check selected event flag *)
  while (not (__HAL_ADC_GET_FLAG(hadc, EventType))) do
  begin
    (* Check for the Timeout *)
    if Timeout <> HAL_MAX_DELAY then
    begin
      if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
      begin
        hadc.State := HAL_ADC_STATE_TIMEOUT;
        (* Process unlocked *)
        __HAL_UNLOCK(hadc.lock);
        exit(HAL_TIMEOUT);
      end;
    end;
  end;

  (* Check analog watchdog flag *)
  if EventType = ADC_AWD_EVENT then
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_AWD;

    (* Clear the ADCx's analog watchdog flag *)
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD);
  end
  else
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_ERROR;

    (* Clear the ADCx's Overrun flag *)
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
  end;

  (* Return ADC state *)
  exit(HAL_OK);
end;

function HAL_ADC_Start_IT(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
var
  counter: longword;
begin
  counter := 0;

  (* Process locked *)
  __HAL_LOCK(hadc.lock);

  (* Check if an injected conversion is ongoing *)
  if hadc.State = HAL_ADC_STATE_BUSY_INJ then
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_BUSY_INJ_REG;
  end
  else
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_BUSY_REG;
  end;

  (* Set ADC error code to none *)
  hadc.ErrorCode := HAL_ADC_ERROR_NONE;

  (* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization *)
  if (hadc.Instance^.CR2 and ADC_CR2_ADON) <> ADC_CR2_ADON then
  begin
    (* Enable the Peripheral *)
    __HAL_ADC_ENABLE(hadc);

    (* Delay for ADC stabilization time *)
    (* Compute number of CPU cycles to wait for *)
    counter := (ADC_STAB_DELAY_US * (SystemCoreClock div 1000000));
    while counter <> 0 do
    begin
      Dec(counter);
    end;
  end;

  (* Enable the ADC overrun interrupt *)
  __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

  (* Enable the ADC end of conversion interrupt for regular group *)
  __HAL_ADC_ENABLE_IT(hadc, ADC_IT_EOC);

  (* Process unlocked *)
  __HAL_UNLOCK(hadc.lock);

  (* Check if Multimode enabled *)
  if HAL_IS_BIT_CLR(C_ADC.CCR, ADC_CCR_MULTI) then
  begin
    (* if no external trigger present enable software conversion of regular channels *)
    if (hadc.Instance^.CR2 and ADC_CR2_EXTEN) = 0 then
    begin
      (* Enable the selected ADC software conversion for regular group *)
      hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_SWSTART;
    end;
  end
  else
  begin
    (* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels *)
    if ((hadc.Instance = pointer(ADC1_BASE)) and ((hadc.Instance^.CR2 and ADC_CR2_EXTEN) = 0)) then
    begin
      (* Enable the selected ADC software conversion for regular group *)
      hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_SWSTART;
    end;
  end;

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADC_Stop_IT(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Disable the ADC end of conversion interrupt for regular group *)
  __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);

  (* Disable the ADC end of conversion interrupt for injected group *)
  __HAL_ADC_DISABLE_IT(hadc, ADC_CR1_JEOCIE);

  (* Enable the Peripheral *)
  __HAL_ADC_DISABLE(hadc);

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_READY;

  (* Return function status *)
  exit(HAL_OK);
end;

procedure HAL_ADC_IRQHandler(var hadc: ADC_HandleTypeDef);
var
  tmp1, tmp2: boolean;
begin
  tmp1 := __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC);
  tmp2 := __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_EOC);
  (* Check End of conversion flag for regular channels *)
  if tmp1 and tmp2 then
  begin
    (* Check if an injected conversion is ready *)
    if hadc.State = HAL_ADC_STATE_EOC_INJ then
    begin
      (* Change ADC state *)
      hadc.State := HAL_ADC_STATE_EOC_INJ_REG;
    end
    else
    begin
      (* Change ADC state *)
      hadc.State := HAL_ADC_STATE_EOC_REG;
    end;

    if (hadc.Init.ContinuousConvMode = 0) and ((hadc.Instance^.CR2 and ADC_CR2_EXTEN) = 0) then
    begin
      if hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV then
      begin
        (* DISABLE the ADC end of conversion interrupt for regular group *)
        __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);

        (* DISABLE the ADC overrun interrupt *)
        __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);
      end
      else
      begin
        if (hadc.NbrOfCurrentConversionRank = 0) then
        begin
          hadc.NbrOfCurrentConversionRank := hadc.Init.NbrOfConversion;
        end;

        (* Decrement the number of conversion when an interrupt occurs *)
        Dec(hadc.NbrOfCurrentConversionRank);

        (* Check if all conversions are finished *)
        if hadc.NbrOfCurrentConversionRank = 0 then
        begin
          (* DISABLE the ADC end of conversion interrupt for regular group *)
          __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);

          (* DISABLE the ADC overrun interrupt *)
          __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);
        end;
      end;
    end;

    (* Conversion complete callback *)
    HAL_ADC_ConvCpltCallback(hadc);

    (* Clear the ADCx flag for regular end of conversion *)
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC);
  end;

  tmp1 := __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC);
  tmp2 := __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_JEOC);
  (* Check End of conversion flag for injected channels *)
  if tmp1 and tmp2 then
  begin
    (* Check if a regular conversion is ready *)
    if hadc.State = HAL_ADC_STATE_EOC_REG then
    begin
      (* Change ADC state *)
      hadc.State := HAL_ADC_STATE_EOC_INJ_REG;
    end
    else
    begin
      (* Change ADC state *)
      hadc.State := HAL_ADC_STATE_EOC_INJ;
    end;

    tmp1 := HAL_IS_BIT_CLR(hadc.Instance^.CR1, ADC_CR1_JAUTO);
    tmp2 := HAL_IS_BIT_CLR(hadc.Instance^.CR2, ADC_CR2_JEXTEN);
    if ((hadc.Init.ContinuousConvMode = 0) or tmp1) and tmp2 then
    begin
      (* DISABLE the ADC end of conversion interrupt for injected group *)
      __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
    end;

    (* Conversion complete callback *)
    HAL_ADCEx_InjectedConvCpltCallback(hadc);

    (* Clear the ADCx flag for injected end of conversion *)
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);
  end;

  tmp1 := __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_AWD);
  tmp2 := __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_AWD);
  (* Check Analog watchdog flag *)
  if tmp1 and tmp2 then
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_AWD;

    (* Clear the ADCx's Analog watchdog flag *)
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD);

    (* Level out of window callback *)
    HAL_ADC_LevelOutOfWindowCallback(hadc);
  end;

  tmp1 := __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_OVR);
  tmp2 := __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_OVR);
  (* Check Overrun flag *)
  if tmp1 and tmp2 then
  begin
    (* Change ADC state to overrun state *)
    hadc.State := HAL_ADC_STATE_ERROR;

    (* Set ADC error code to overrun *)
    hadc.ErrorCode := hadc.ErrorCode or HAL_ADC_ERROR_OVR;

    (* Clear the Overrun flag *)
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);

    (* Error callback *)
    HAL_ADC_ErrorCallback(hadc);
  end;
end;

procedure ADC_DMAConvCplt(var hdma: DMA_HandleTypeDef);
var
  hadc: PADC_HandleTypeDef;
begin
  hadc := PADC_HandleTypeDef(hdma.Parent);

  (* Check if an injected conversion is ready *)
  if (hadc^.State = HAL_ADC_STATE_EOC_INJ) then
  begin
    (* Change ADC state *)
    hadc^.State := HAL_ADC_STATE_EOC_INJ_REG;
  end
  else
  begin
    (* Change ADC state *)
    hadc^.State := HAL_ADC_STATE_EOC_REG;
  end;

  HAL_ADC_ConvCpltCallback(hadc^);
end;

procedure ADC_DMAHalfConvCplt(var hdma: DMA_HandleTypeDef);
var
  hadc: PADC_HandleTypeDef;
begin
  hadc := PADC_HandleTypeDef(hdma.Parent);

  HAL_ADC_ConvHalfCpltCallback(hadc^);
end;

procedure ADC_DMAError(var hdma: DMA_HandleTypeDef);
var
  hadc: PADC_HandleTypeDef;
begin
  hadc := PADC_HandleTypeDef(hdma.Parent);

  hadc^.State := HAL_ADC_STATE_ERROR;

  hadc^.ErrorCode := hadc^.ErrorCode or HAL_ADC_ERROR_DMA;

  HAL_ADC_ErrorCallback(hadc^);
end;

function HAL_ADC_Start_DMA(var hadc: ADC_HandleTypeDef; var pData: longword; Length: longword): HAL_StatusTypeDef;
var
  counter: longword;
begin
  counter := 0;

  (* Process locked *)
  __HAL_LOCK(hadc.lock);

  (* Enable ADC overrun interrupt *)
  __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

  (* Enable ADC DMA mode *)
  hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_DMA;

  (* Set the DMA transfer complete callback *)
  hadc.DMA_Handle^.XferCpltCallback := @ADC_DMAConvCplt;

  (* Set the DMA half transfer complete callback *)
  hadc.DMA_Handle^.XferHalfCpltCallback := @ADC_DMAHalfConvCplt;

  (* Set the DMA error callback *)
  hadc.DMA_Handle^.XferErrorCallback := @ADC_DMAError;

  (* Enable the DMA Stream *)
  HAL_DMA_Start_IT(hadc.DMA_Handle^, @hadc.Instance^.DR, @pData, Length);

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_BUSY_REG;

  (* Process unlocked *)
  __HAL_UNLOCK(hadc.lock);

  (* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization *)
  if (hadc.Instance^.CR2 and ADC_CR2_ADON) <> ADC_CR2_ADON then
  begin
    (* Enable the Peripheral *)
    __HAL_ADC_ENABLE(hadc);

    (* Delay for ADC stabilization time *)
    (* Compute number of CPU cycles to wait for *)
    counter := (ADC_STAB_DELAY_US * (SystemCoreClock div 1000000));
    while counter <> 0 do
    begin
      Dec(counter);
    end;
  end;

  (* if no external trigger present enable software conversion of regular channels *)
  if (hadc.Instance^.CR2 and ADC_CR2_EXTEN) = 0 then
  begin
    (* Enable the selected ADC software conversion for regular group *)
    hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_SWSTART;
  end;

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADC_Stop_DMA(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Disable the Peripheral *)
  __HAL_ADC_DISABLE(hadc);

  (* Disable ADC overrun interrupt *)
  __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

  (* Disable the selected ADC DMA mode *)
  hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not ADC_CR2_DMA);

  (* Disable the ADC DMA Stream *)
  HAL_DMA_Abort(hadc.DMA_Handle^);

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_READY;

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADC_GetValue(var hadc: ADC_HandleTypeDef): longword;
begin
  (* Return the selected ADC converted value *)
  exit(hadc.Instance^.DR);
end;

procedure HAL_ADC_ConvCpltCallback_stub(var hadc: ADC_HandleTypeDef); assembler; nostackframe; public name 'HAL_ADC_ConvCpltCallback';
  asm
    .weak HAL_ADC_ConvCpltCallback
  end;

procedure HAL_ADC_ConvHalfCpltCallback_stub(var hadc: ADC_HandleTypeDef); assembler; nostackframe; public name 'HAL_ADC_ConvHalfCpltCallback';
  asm
    .weak HAL_ADC_ConvHalfCpltCallback
  end;

procedure HAL_ADC_LevelOutOfWindowCallback_stub(var hadc: ADC_HandleTypeDef); assembler; nostackframe; public name 'HAL_ADC_LevelOutOfWindowCallback';
  asm
    .weak HAL_ADC_LevelOutOfWindowCallback
  end;

procedure HAL_ADC_ErrorCallback_stub(var hadc: ADC_HandleTypeDef); assembler; nostackframe; public name 'HAL_ADC_ErrorCallback';
  asm
    .weak HAL_ADC_ErrorCallback
  end;

function HAL_ADC_ConfigChannel(var hadc: ADC_HandleTypeDef; const sConfig: ADC_ChannelConfTypeDef): HAL_StatusTypeDef;
var
  counter: longword;
begin
  counter := 0;

  (* Process locked *)
  __HAL_LOCK(hadc.lock);

  (* if ADC_Channel_10 ... ADC_Channel_18 is selected *)
  if (sConfig.Channel > ADC_CHANNEL_9) then
  begin
    (* Clear the old sample time *)
    hadc.Instance^.SMPR1 := hadc.Instance^.SMPR1 and (not ADC_SMPR1(ADC_SMPR1_SMP10, sConfig.Channel));

    (* Set the new sample time *)
    hadc.Instance^.SMPR1 := hadc.Instance^.SMPR1 or ADC_SMPR1(sConfig.SamplingTime, sConfig.Channel);
  end
  else (* ADC_Channel include in ADC_Channel_[0..9] *)
  begin
    (* Clear the old sample time *)
    hadc.Instance^.SMPR2 := hadc.Instance^.SMPR2 and (not ADC_SMPR2(ADC_SMPR2_SMP0, sConfig.Channel));

    (* Set the new sample time *)
    hadc.Instance^.SMPR2 := hadc.Instance^.SMPR2 or ADC_SMPR2(sConfig.SamplingTime, sConfig.Channel);
  end;

  (* For Rank 1 to 6 *)
  if (sConfig.Rank < 7) then
  begin
    (* Clear the old SQx bits for the selected rank *)
    hadc.Instance^.SQR3 := hadc.Instance^.SQR3 and (not ADC_SQR3_RK(ADC_SQR3_SQ1, sConfig.Rank));

    (* Set the SQx bits for the selected rank *)
    hadc.Instance^.SQR3 := hadc.Instance^.SQR3 or ADC_SQR3_RK(sConfig.Channel, sConfig.Rank);
  end
  (* For Rank 7 to 12 *)
  else if (sConfig.Rank < 13) then
  begin
    (* Clear the old SQx bits for the selected rank *)
    hadc.Instance^.SQR2 := hadc.Instance^.SQR2 and (not ADC_SQR2_RK(ADC_SQR2_SQ7, sConfig.Rank));

    (* Set the SQx bits for the selected rank *)
    hadc.Instance^.SQR2 := hadc.Instance^.SQR2 or ADC_SQR2_RK(sConfig.Channel, sConfig.Rank);
  end
  (* For Rank 13 to 16 *)
  else
  begin
    (* Clear the old SQx bits for the selected rank *)
    hadc.Instance^.SQR1 := hadc.Instance^.SQR1 and (not ADC_SQR1_RK(ADC_SQR1_SQ13, sConfig.Rank));

    (* Set the SQx bits for the selected rank *)
    hadc.Instance^.SQR1 := hadc.Instance^.SQR1 or ADC_SQR1_RK(sConfig.Channel, sConfig.Rank);
  end;

  (* if ADC1 Channel_18 is selected enable VBAT Channel *)
  if ((hadc.Instance = @ADC1) and (sConfig.Channel = ADC_CHANNEL_VBAT)) then
  begin
    (* Enable the VBAT channel*)
    C_ADC.CCR := C_ADC.CCR or ADC_CCR_VBATE;
  end;

  (* if ADC1 Channel_16 or Channel_17 is selected enable TSVREFE Channel(Temperature sensor and VREFINT) *)
  if ((hadc.Instance = @ADC1) and ((sConfig.Channel = ADC_CHANNEL_TEMPSENSOR) or (sConfig.Channel = ADC_CHANNEL_VREFINT))) then
  begin
    (* Enable the TSVREFE channel*)
    C_ADC.CCR := C_ADC.CCR or ADC_CCR_TSVREFE;

    if (sConfig.Channel = ADC_CHANNEL_TEMPSENSOR) then
    begin
      (* Delay for temperature sensor stabilization time *)
      (* Compute number of CPU cycles to wait for *)
      counter := (ADC_TEMPSENSOR_DELAY_US * (SystemCoreClock div 1000000));
      while counter <> 0 do
      begin
        Dec(counter);
      end;
    end;
  end;

  (* Process unlocked *)
  __HAL_UNLOCK(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADC_AnalogWDGConfig(var hadc: ADC_HandleTypeDef; const AnalogWDGConfig: ADC_AnalogWDGConfTypeDef): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_LOCK(hadc.lock);

  if AnalogWDGConfig.ITMode <> 0 then
  begin
    (* Enable the ADC Analog watchdog interrupt *)
    __HAL_ADC_ENABLE_IT(hadc, ADC_IT_AWD);
  end
  else
  begin
    (* Disable the ADC Analog watchdog interrupt *)
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_AWD);
  end;

  (* Clear AWDEN, JAWDEN and AWDSGL bits *)
  hadc.Instance^.CR1 := hadc.Instance^.CR1 and (not (ADC_CR1_AWDSGL or ADC_CR1_JAWDEN or ADC_CR1_AWDEN));

  (* Set the analog watchdog enable mode *)
  hadc.Instance^.CR1 := hadc.Instance^.CR1 or AnalogWDGConfig.WatchdogMode;

  (* Set the high threshold *)
  hadc.Instance^.HTR := AnalogWDGConfig.HighThreshold;

  (* Set the low threshold *)
  hadc.Instance^.LTR := AnalogWDGConfig.LowThreshold;

  (* Clear the Analog watchdog channel select bits *)
  hadc.Instance^.CR1 := hadc.Instance^.CR1 and (not ADC_CR1_AWDCH);

  (* Set the Analog watchdog channel *)
  hadc.Instance^.CR1 := hadc.Instance^.CR1 or ((AnalogWDGConfig.Channel));

  (* Process unlocked *)
  __HAL_UNLOCK(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADC_GetState(var hadc: ADC_HandleTypeDef): HAL_ADC_StateTypeDef;
begin
  (* Return ADC state *)
  exit(hadc.State);
end;

function HAL_ADC_GetError(var hadc: ADC_HandleTypeDef): longword;
begin
  exit(hadc.ErrorCode);
end;

end.
