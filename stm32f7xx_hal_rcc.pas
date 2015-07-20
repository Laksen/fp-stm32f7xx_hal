(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_rcc.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of RCC HAL module.
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

unit stm32f7xx_hal_rcc;

interface

uses
  stm32f7xx_hal, stm32f7xx_hal_conf;

type
  RCC_PLLInitTypeDef = record
    PLLState: longword;  (*!< The new state of the PLL.
                            This parameter can be a value of @ref RCC_PLL_Config                       *)
    PLLSource: longword;  (*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCC_PLL_Clock_Source                *)
    PLLM: longword;  (*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data := 2 and Max_Data := 63     *)
    PLLN: longword;  (*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data := 192 and Max_Data := 432  *)
    PLLP: longword;  (*!< PLLP: Division factor for main system clock (SYSCLK).
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider              *)
    PLLQ: longword;  (*!< PLLQ: Division factor for OTG FS, SDMMC and RNG clocks.
                            This parameter must be a number between Min_Data := 2 and Max_Data := 15     *)
  end;

  (**
  * @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
   *)

  RCC_OscInitTypeDef = record
    OscillatorType: longword;  (*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                    *)
    HSEState: longword;  (*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                         *)
    LSEState: longword;  (*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                         *)
    HSIState: longword;  (*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                         *)
    HSICalibrationValue: longword;  (*!< The calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data := $00 and Max_Data := $1F  *)
    LSIState: longword;  (*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                         *)
    PLL: RCC_PLLInitTypeDef;
    (*!< PLL structure parameters                                                     *)
  end;

  (**
  * @brief  RCC System, AHB and APB busses clock configuration structure definition
   *)

  RCC_ClkInitTypeDef = record
    ClockType: longword;  (*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type  *)
    SYSCLKSource: longword;  (*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source     *)
    AHBCLKDivider: longword;  (*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source        *)
    APB1CLKDivider: longword;  (*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source  *)
    APB2CLKDivider: longword;  (*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source  *)
  end;

(** @defgroup RCC_Exported_Constants RCC Exported Constants
    * @begin
     *)

(** @defgroup RCC_Oscillator_Type Oscillator Type
    * @begin
     *)

const
  RCC_OSCILLATORTYPE_NONE = ($00000000);
  RCC_OSCILLATORTYPE_HSE = ($00000001);
  RCC_OSCILLATORTYPE_HSI = ($00000002);
  RCC_OSCILLATORTYPE_LSE = ($00000004);
  RCC_OSCILLATORTYPE_LSI = ($00000008);
  (**
    * @end;
     *)

  (** @defgroup RCC_HSE_Config RCC HSE Config
    * @begin
     *)

  RCC_HSE_OFF = ($00000000);
  RCC_HSE_ON = $00010000;
  RCC_HSE_BYPASS = (($00040000 or $00010000));
  (**
    * @end;
     *)

  (** @defgroup RCC_LSE_Config RCC LSE Config
    * @begin
     *)

  RCC_LSE_OFF = ($00000000);
  RCC_LSE_ON = $00000001;
  RCC_LSE_BYPASS = (($00000004 or $00000001));
  (**
    * @end;
     *)

  (** @defgroup RCC_HSI_Config RCC HSI Config
    * @begin
     *)

  RCC_HSI_OFF = ($00000000);
  RCC_HSI_ON = $00000001;
  (**
    * @end;
     *)

  (** @defgroup RCC_LSI_Config RCC LSI Config
    * @begin
     *)

  RCC_LSI_OFF = ($00000000);
  RCC_LSI_ON = $00000001;
  (**
    * @end;
     *)

  (** @defgroup RCC_PLL_Config RCC PLL Config
    * @begin
     *)

  RCC_PLL_NONE = ($00000000);
  RCC_PLL_OFF = ($00000001);
  RCC_PLL_ON = ($00000002);
  (**
    * @end;
     *)

  (** @defgroup RCC_PLLP_Clock_Divider PLLP Clock Divider
    * @begin
     *)

  RCC_PLLP_DIV2 = ($00000002);
  RCC_PLLP_DIV4 = ($00000004);
  RCC_PLLP_DIV6 = ($00000006);
  RCC_PLLP_DIV8 = ($00000008);
  (**
    * @end;
     *)

  (** @defgroup RCC_PLL_Clock_Source PLL Clock Source
    * @begin
     *)

  RCC_PLLSOURCE_HSI = $00000000;
  RCC_PLLSOURCE_HSE = $00400000;
  (**
    * @end;
     *)

  (** @defgroup RCC_System_Clock_Type RCC System Clock Type
    * @begin
     *)

  RCC_CLOCKTYPE_SYSCLK = ($00000001);
  RCC_CLOCKTYPE_HCLK = ($00000002);
  RCC_CLOCKTYPE_PCLK1 = ($00000004);
  RCC_CLOCKTYPE_PCLK2 = ($00000008);
  (**
    * @end;
     *)

  (** @defgroup RCC_System_Clock_Source RCC System Clock Source
    * @begin
     *)

  RCC_SYSCLKSOURCE_HSI = $00000000;
  RCC_SYSCLKSOURCE_HSE = $00000001;
  RCC_SYSCLKSOURCE_PLLCLK = $00000002;
  (**
    * @end;
     *)


  (** @defgroup RCC_System_Clock_Source_Status System Clock Source Status
    * @begin
     *)

  RCC_SYSCLKSOURCE_STATUS_HSI = $00000000;  (*!< HSI used as system clock  *)
  RCC_SYSCLKSOURCE_STATUS_HSE = $00000004;  (*!< HSE used as system clock  *)
  RCC_SYSCLKSOURCE_STATUS_PLLCLK = $00000008;  (*!< PLL used as system clock  *)
  (**
    * @end;
     *)

  (** @defgroup RCC_AHB_Clock_Source RCC AHB Clock Source
    * @begin
     *)

  RCC_SYSCLK_DIV1 = $00000000;
  RCC_SYSCLK_DIV2 = $00000080;
  RCC_SYSCLK_DIV4 = $00000090;
  RCC_SYSCLK_DIV8 = $000000A0;
  RCC_SYSCLK_DIV16 = $000000B0;
  RCC_SYSCLK_DIV64 = $000000C0;
  RCC_SYSCLK_DIV128 = $000000D0;
  RCC_SYSCLK_DIV256 = $000000E0;
  RCC_SYSCLK_DIV512 = $000000F0;
  (**
    * @end;
     *)

  (** @defgroup RCC_APB1_APB2_Clock_Source RCC APB1/APB2 Clock Source
    * @begin
     *)

  RCC_HCLK_DIV1 = $00000000;
  RCC_HCLK_DIV2 = $00001000;
  RCC_HCLK_DIV4 = $00001400;
  RCC_HCLK_DIV8 = $00001800;
  RCC_HCLK_DIV16 = $00001C00;
  (**
    * @end;
     *)

  (** @defgroup RCC_RTC_Clock_Source RCC RTC Clock Source
    * @begin
     *)

  RCC_RTCCLKSOURCE_LSE = ($00000100);
  RCC_RTCCLKSOURCE_LSI = ($00000200);
  RCC_RTCCLKSOURCE_HSE_DIV2 = ($00020300);
  RCC_RTCCLKSOURCE_HSE_DIV3 = ($00030300);
  RCC_RTCCLKSOURCE_HSE_DIV4 = ($00040300);
  RCC_RTCCLKSOURCE_HSE_DIV5 = ($00050300);
  RCC_RTCCLKSOURCE_HSE_DIV6 = ($00060300);
  RCC_RTCCLKSOURCE_HSE_DIV7 = ($00070300);
  RCC_RTCCLKSOURCE_HSE_DIV8 = ($00080300);
  RCC_RTCCLKSOURCE_HSE_DIV9 = ($00090300);
  RCC_RTCCLKSOURCE_HSE_DIV10 = ($000A0300);
  RCC_RTCCLKSOURCE_HSE_DIV11 = ($000B0300);
  RCC_RTCCLKSOURCE_HSE_DIV12 = ($000C0300);
  RCC_RTCCLKSOURCE_HSE_DIV13 = ($000D0300);
  RCC_RTCCLKSOURCE_HSE_DIV14 = ($000E0300);
  RCC_RTCCLKSOURCE_HSE_DIV15 = ($000F0300);
  RCC_RTCCLKSOURCE_HSE_DIV16 = ($00100300);
  RCC_RTCCLKSOURCE_HSE_DIV17 = ($00110300);
  RCC_RTCCLKSOURCE_HSE_DIV18 = ($00120300);
  RCC_RTCCLKSOURCE_HSE_DIV19 = ($00130300);
  RCC_RTCCLKSOURCE_HSE_DIV20 = ($00140300);
  RCC_RTCCLKSOURCE_HSE_DIV21 = ($00150300);
  RCC_RTCCLKSOURCE_HSE_DIV22 = ($00160300);
  RCC_RTCCLKSOURCE_HSE_DIV23 = ($00170300);
  RCC_RTCCLKSOURCE_HSE_DIV24 = ($00180300);
  RCC_RTCCLKSOURCE_HSE_DIV25 = ($00190300);
  RCC_RTCCLKSOURCE_HSE_DIV26 = ($001A0300);
  RCC_RTCCLKSOURCE_HSE_DIV27 = ($001B0300);
  RCC_RTCCLKSOURCE_HSE_DIV28 = ($001C0300);
  RCC_RTCCLKSOURCE_HSE_DIV29 = ($001D0300);
  RCC_RTCCLKSOURCE_HSE_DIV30 = ($001E0300);
  RCC_RTCCLKSOURCE_HSE_DIV31 = ($001F0300);
  (**
    * @end;
     *)


  (** @defgroup RCC_MCO_Index RCC MCO Index
    * @begin
     *)

  RCC_MCO1 = ($00000000);
  RCC_MCO2 = ($00000001);
  (**
    * @end;
     *)

  (** @defgroup RCC_MCO1_Clock_Source RCC MCO1 Clock Source
    * @begin
     *)

  RCC_MCO1SOURCE_HSI = ($00000000);
  RCC_MCO1SOURCE_LSE = $00200000;
  RCC_MCO1SOURCE_HSE = $00400000;
  RCC_MCO1SOURCE_PLLCLK = $00600000;
  (**
    * @end;
     *)

  (** @defgroup RCC_MCO2_Clock_Source RCC MCO2 Clock Source
    * @begin
     *)

  RCC_MCO2SOURCE_SYSCLK = ($00000000);
  RCC_MCO2SOURCE_PLLI2SCLK = $40000000;
  RCC_MCO2SOURCE_HSE = $80000000;
  RCC_MCO2SOURCE_PLLCLK = $C0000000;
  (**
    * @end;
     *)

  (** @defgroup RCC_MCOx_Clock_Prescaler RCC MCO1 Clock Prescaler
    * @begin
     *)

  RCC_MCODIV_1 = ($00000000);
  RCC_MCODIV_2 = $04000000;
  RCC_MCODIV_3 = ($01000000 or $04000000);
  RCC_MCODIV_4 = ($02000000 or $04000000);
  RCC_MCODIV_5 = $07000000;
  (**
    * @end;
     *)

  (** @defgroup RCC_Interrupt RCC Interrupt
    * @begin
     *)

  RCC_IT_LSIRDY = ($01);
  RCC_IT_LSERDY = ($02);
  RCC_IT_HSIRDY = ($04);
  RCC_IT_HSERDY = ($08);
  RCC_IT_PLLRDY = ($10);
  RCC_IT_PLLI2SRDY = ($20);
  RCC_IT_PLLSAIRDY = ($40);
  RCC_IT_CSS = ($80);
  (**
    * @end;
     *)

  (** @defgroup RCC_Flag RCC Flags
    *        Elements values convention: $XYYYYYb
    *           - YYYYY  : Flag position in the register
    *           - $X  : Register index
    *                 - 01: CR register
    *                 - 10: BDCR register
    *                 - 11: CSR register
    * @begin
     *)

  (* Flags in the CR register  *)

  RCC_FLAG_HSIRDY = ($21);
  RCC_FLAG_HSERDY = ($31);
  RCC_FLAG_PLLRDY = ($39);
  RCC_FLAG_PLLI2SRDY = ($3B);
  RCC_FLAG_PLLSAIRDY = ($3C);
  (* Flags in the BDCR register  *)

  RCC_FLAG_LSERDY = ($41);
  (* Flags in the CSR register  *)

  RCC_FLAG_LSIRDY = ($61);
  RCC_FLAG_BORRST = ($79);
  RCC_FLAG_PINRST = ($7A);
  RCC_FLAG_PORRST = ($7B);
  RCC_FLAG_SFTRST = ($7C);
  RCC_FLAG_IWDGRST = ($7D);
  RCC_FLAG_WWDGRST = ($7E);
  RCC_FLAG_LPWRRST = ($7F);
  (**
    * @end;
     *)

  (** @defgroup RCC_LSEDrive_Configuration RCC LSE Drive configurations
    * @begin
     *)

  RCC_LSEDRIVE_LOW = ($00000000);
  RCC_LSEDRIVE_MEDIUMLOW = $00000010;
  RCC_LSEDRIVE_MEDIUMHIGH = $00000008;
  RCC_LSEDRIVE_HIGH = $00000018;

  (**
    * @end;
     *)

procedure HAL_RCC_DeInit;
function HAL_RCC_OscConfig(const RCC_OscInitStruct: RCC_OscInitTypeDef): HAL_StatusTypeDef;
function HAL_RCC_ClockConfig(const RCC_ClkInitStruct: RCC_ClkInitTypeDef; FLatency: longword): HAL_StatusTypeDef;

procedure HAL_RCC_MCOConfig(RCC_MCOx, RCC_MCOSource, RCC_MCODiv: longword);
procedure HAL_RCC_EnableCSS;
procedure HAL_RCC_DisableCSS;
function HAL_RCC_GetSysClockFreq: longword;
function HAL_RCC_GetHCLKFreq: longword;
function HAL_RCC_GetPCLK1Freq: longword;
function HAL_RCC_GetPCLK2Freq: longword;
procedure HAL_RCC_GetOscConfig(var RCC_OscInitStruct: RCC_OscInitTypeDef);
procedure HAL_RCC_GetClockConfig(var RCC_ClkInitStruct: RCC_ClkInitTypeDef; var pFLatency: longword);

procedure __HAL_RCC_CRC_CLK_ENABLE();
procedure __HAL_RCC_DMA1_CLK_ENABLE();
procedure __HAL_RCC_CRC_CLK_DISABLE();
procedure __HAL_RCC_DMA1_CLK_DISABLE();
procedure __HAL_RCC_WWDG_CLK_ENABLE();
procedure __HAL_RCC_PWR_CLK_ENABLE();
procedure __HAL_RCC_WWDG_CLK_DISABLE();
procedure __HAL_RCC_PWR_CLK_DISABLE();
procedure __HAL_RCC_SYSCFG_CLK_ENABLE();
procedure __HAL_RCC_SYSCFG_CLK_DISABLE();
  function __HAL_RCC_CRC_IS_CLK_ENABLED(): boolean;
function __HAL_RCC_DMA1_IS_CLK_ENABLED(): boolean;
function __HAL_RCC_CRC_IS_CLK_DISABLED(): boolean;
function __HAL_RCC_DMA1_IS_CLK_DISABLED(): boolean;
  function __HAL_RCC_WWDG_IS_CLK_ENABLED(): boolean;
function __HAL_RCC_PWR_IS_CLK_ENABLED(): boolean;
function __HAL_RCC_WWDG_IS_CLK_DISABLED(): boolean;
function __HAL_RCC_PWR_IS_CLK_DISABLED(): boolean;
function __HAL_RCC_SYSCFG_IS_CLK_ENABLED(): boolean;
function __HAL_RCC_SYSCFG_IS_CLK_DISABLED(): boolean;
procedure __HAL_RCC_AHB1_FORCE_RESET();
procedure __HAL_RCC_CRC_FORCE_RESET();
procedure __HAL_RCC_DMA1_FORCE_RESET();
procedure __HAL_RCC_AHB1_RELEASE_RESET();
procedure __HAL_RCC_CRC_RELEASE_RESET();
procedure __HAL_RCC_DMA1_RELEASE_RESET();
procedure __HAL_RCC_APB1_FORCE_RESET();
procedure __HAL_RCC_WWDG_FORCE_RESET();
procedure __HAL_RCC_PWR_FORCE_RESET();
procedure __HAL_RCC_APB1_RELEASE_RESET();
procedure __HAL_RCC_WWDG_RELEASE_RESET();
procedure __HAL_RCC_PWR_RELEASE_RESET();
procedure __HAL_RCC_APB2_FORCE_RESET();
procedure __HAL_RCC_SYSCFG_FORCE_RESET();
procedure __HAL_RCC_APB2_RELEASE_RESET();
procedure __HAL_RCC_SYSCFG_RELEASE_RESET();
procedure __HAL_RCC_CRC_CLK_SLEEP_ENABLE();
procedure __HAL_RCC_DMA1_CLK_SLEEP_ENABLE();
procedure __HAL_RCC_CRC_CLK_SLEEP_DISABLE();
procedure __HAL_RCC_DMA1_CLK_SLEEP_DISABLE();
procedure __HAL_RCC_WWDG_CLK_SLEEP_ENABLE();
procedure __HAL_RCC_PWR_CLK_SLEEP_ENABLE();
procedure __HAL_RCC_WWDG_CLK_SLEEP_DISABLE();
procedure __HAL_RCC_PWR_CLK_SLEEP_DISABLE();
procedure __HAL_RCC_SYSCFG_CLK_SLEEP_ENABLE();
procedure __HAL_RCC_SYSCFG_CLK_SLEEP_DISABLE();
function __HAL_RCC_CRC_IS_CLK_SLEEP_ENABLED(): boolean;
function __HAL_RCC_DMA1_IS_CLK_SLEEP_ENABLED(): boolean;
function __HAL_RCC_CRC_IS_CLK_SLEEP_DISABLED(): boolean;
function __HAL_RCC_DMA1_IS_CLK_SLEEP_DISABLED(): boolean;
function __HAL_RCC_WWDG_IS_CLK_SLEEP_ENABLED(): boolean;
function __HAL_RCC_PWR_IS_CLK_SLEEP_ENABLED(): boolean;
function __HAL_RCC_WWDG_IS_CLK_SLEEP_DISABLED(): boolean;
function __HAL_RCC_PWR_IS_CLK_SLEEP_DISABLED(): boolean;
function __HAL_RCC_SYSCFG_IS_CLK_SLEEP_ENABLED(): boolean;
function __HAL_RCC_SYSCFG_IS_CLK_SLEEP_DISABLED(): boolean;
procedure __HAL_RCC_HSI_ENABLE();
procedure __HAL_RCC_HSI_DISABLE();
procedure __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(__HSICALIBRATIONVALUE__: longword);
procedure __HAL_RCC_LSI_ENABLE();
procedure __HAL_RCC_LSI_DISABLE();
procedure __HAL_RCC_HSE_CONFIG(__STATE__: longword);
procedure __HAL_RCC_LSE_CONFIG(__STATE__: longword);
procedure __HAL_RCC_RTC_ENABLE();
procedure __HAL_RCC_RTC_DISABLE();
procedure __HAL_RCC_RTC_CLKPRESCALER(__RTCCLKSource__: longword);
procedure __HAL_RCC_RTC_CONFIG(__RTCCLKSource__: longword);
procedure __HAL_RCC_BACKUPRESET_FORCE();
procedure __HAL_RCC_BACKUPRESET_RELEASE();
procedure __HAL_RCC_PLL_ENABLE();
procedure __HAL_RCC_PLL_DISABLE();
procedure __HAL_RCC_PLL_PLLSOURCE_CONFIG(__PLLSOURCE__: longword);
procedure __HAL_RCC_PLL_PLLM_CONFIG(__PLLM__: longword);
procedure __HAL_RCC_I2S_CONFIG(__SOURCE__: longword);
procedure __HAL_RCC_PLLI2S_ENABLE();
procedure __HAL_RCC_PLLI2S_DISABLE();
procedure __HAL_RCC_SYSCLK_CONFIG(__RCC_SYSCLKSOURCE__: longword);
function __HAL_RCC_GET_SYSCLK_SOURCE(): longword;
procedure __HAL_RCC_LSEDRIVE_CONFIG(__RCC_LSEDRIVE__: longword);
function __HAL_RCC_GET_PLL_OSCSOURCE: longword;
procedure __HAL_RCC_ENABLE_IT(__INTERRUPT__: longword);
procedure __HAL_RCC_DISABLE_IT(__INTERRUPT__: longword);
procedure __HAL_RCC_CLEAR_IT(__INTERRUPT__: longword);
function __HAL_RCC_GET_IT(__INTERRUPT__: longword): boolean;
procedure __HAL_RCC_CLEAR_RESET_FLAGS();
function __HAL_RCC_GET_FLAG(__FLAG__: longword): boolean;

const
  HSE_TIMEOUT_VALUE = HSE_STARTUP_TIMEOUT;
  HSI_TIMEOUT_VALUE = (100);  (* 100 ms  *)
  LSI_TIMEOUT_VALUE = (100);  (* 100 ms  *)
  PLL_TIMEOUT_VALUE = (100);  (* 100 ms  *)
  CLOCKSWITCH_TIMEOUT_VALUE = (5000);  (* 5 s     *)
  (** @defgroup RCC_BitAddress_Alias RCC BitAddress Alias
  * @brief RCC registers bit address alias
  * @{
   *)

  (* CIR register byte 2 (Bits[15:8]) base address  *)

  RCC_CIR_BYTE1_ADDRESS = ((RCC_BASE + $0C + $01));
  (* CIR register byte 3 (Bits[23:16]) base address  *)

  RCC_CIR_BYTE2_ADDRESS = ((RCC_BASE + $0C + $02));
  RCC_DBP_TIMEOUT_VALUE = (100);
  RCC_LSE_TIMEOUT_VALUE = (5000);

  RCC_FLAG_MASK = ($1F);

implementation

uses
  stm32f7xx_hal_flash, stm32f7xx_defs;

const
  APBAHBPrescTable: array[0..15] of byte =
    (0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9);

procedure __HAL_RCC_CRC_CLK_ENABLE();
var
  tmpreg: longword;
begin
  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_CRCEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_CRCEN);
end;

procedure __HAL_RCC_DMA1_CLK_ENABLE();
var
  tmpreg: longword;
begin
  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_DMA1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_DMA1EN);
end;

procedure __HAL_RCC_CRC_CLK_DISABLE();
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_CRCEN));
end;
procedure __HAL_RCC_DMA1_CLK_DISABLE();
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_DMA1EN));
end;

(**
  * @end
  *)

(** @defgroup RCC_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @begin
  *)
procedure __HAL_RCC_WWDG_CLK_ENABLE();
var
  tmpreg: longword;
begin
  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_WWDGEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_WWDGEN);
end;

procedure __HAL_RCC_PWR_CLK_ENABLE();
var
  tmpreg: longword;
begin
  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_PWREN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_PWREN);
end;

procedure __HAL_RCC_WWDG_CLK_DISABLE();
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_WWDGEN));
end;
procedure __HAL_RCC_PWR_CLK_DISABLE();
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_PWREN)) ;
end;
(**
  * @end
  *)

(** @defgroup RCC_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable
  * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @begin
  *)
procedure __HAL_RCC_SYSCFG_CLK_ENABLE();
var
  tmpreg: longword;
begin
  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_SYSCFGEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_SYSCFGEN);
end;

procedure __HAL_RCC_SYSCFG_CLK_DISABLE();
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_SYSCFGEN));
end;

(**
  * @end
  *)

(** @defgroup RCC_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @begin
  *)
  function __HAL_RCC_CRC_IS_CLK_ENABLED(): boolean;
begin
  exit((RCC.AHB1ENR and (RCC_AHB1ENR_CRCEN)) <> 0);
end;
function __HAL_RCC_DMA1_IS_CLK_ENABLED(): boolean;
begin
  exit((RCC.AHB1ENR and (RCC_AHB1ENR_DMA1EN)) <> 0);
end;

function __HAL_RCC_CRC_IS_CLK_DISABLED(): boolean;
begin
  exit((RCC.AHB1ENR and (RCC_AHB1ENR_CRCEN)) = 0);
end;
function __HAL_RCC_DMA1_IS_CLK_DISABLED(): boolean;
begin
  exit((RCC.AHB1ENR and (RCC_AHB1ENR_DMA1EN)) = 0);
end;
(**
  * @end
  *)

(** @defgroup RCC_APB1_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable  Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @begin
  *)
  function __HAL_RCC_WWDG_IS_CLK_ENABLED(): boolean;
begin
  exit((RCC.APB1ENR and (RCC_APB1ENR_WWDGEN)) <> 0);
end;
function __HAL_RCC_PWR_IS_CLK_ENABLED(): boolean;
begin
  exit((RCC.APB1ENR and (RCC_APB1ENR_PWREN)) <> 0);
end;

function __HAL_RCC_WWDG_IS_CLK_DISABLED(): boolean;
begin
  exit((RCC.APB1ENR and (RCC_APB1ENR_WWDGEN)) = 0);
end;
function __HAL_RCC_PWR_IS_CLK_DISABLED(): boolean;
begin
  exit((RCC.APB1ENR and (RCC_APB1ENR_PWREN)) = 0);
end;
(**
  * @end
  *)

(** @defgroup RCC_APB2_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  EGet the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @begin
  *)
function __HAL_RCC_SYSCFG_IS_CLK_ENABLED(): boolean;
begin
  exit((RCC.APB2ENR and (RCC_APB2ENR_SYSCFGEN)) <> 0);
end;
function __HAL_RCC_SYSCFG_IS_CLK_DISABLED(): boolean;
begin
  exit((RCC.APB2ENR and (RCC_APB2ENR_SYSCFGEN)) = 0);
end;
(**
  * @end
  *)

(** @defgroup RCC_Peripheral_Clock_Force_Release RCC Peripheral Clock Force Release
  * @brief  Force or release AHB peripheral reset.
  * @begin
  *)
procedure __HAL_RCC_AHB1_FORCE_RESET();
begin
  RCC.AHB1RSTR := $FFFFFFFF;
end;
procedure __HAL_RCC_CRC_FORCE_RESET();
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_CRCRST);
end;
procedure __HAL_RCC_DMA1_FORCE_RESET();
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_DMA1RST);
end;

procedure __HAL_RCC_AHB1_RELEASE_RESET();
begin
  RCC.AHB1RSTR := $00;
end;
procedure __HAL_RCC_CRC_RELEASE_RESET();
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_CRCRST));
end;
procedure __HAL_RCC_DMA1_RELEASE_RESET();
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_DMA1RST));
end;
(**
  * @end
  *)

(** @defgroup RCC_APB1_Force_Release_Reset APB1 Force Release Reset
  * @brief  Force or release APB1 peripheral reset.
  * @begin
  *)
procedure __HAL_RCC_APB1_FORCE_RESET();
begin
  RCC.APB1RSTR := $FFFFFFFF  ;
end;
procedure __HAL_RCC_WWDG_FORCE_RESET();
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_WWDGRST);
end;
procedure __HAL_RCC_PWR_FORCE_RESET();
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_PWRRST);
end;

procedure __HAL_RCC_APB1_RELEASE_RESET();
begin
  RCC.APB1RSTR := $00 ;
end;
procedure __HAL_RCC_WWDG_RELEASE_RESET();
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_WWDGRST));
end;
procedure __HAL_RCC_PWR_RELEASE_RESET();
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_PWRRST));
end;
(**
  * @end
  *)

(** @defgroup RCC_APB2_Force_Release_Reset APB2 Force Release Reset
  * @brief  Force or release APB2 peripheral reset.
  * @begin
  *)
procedure __HAL_RCC_APB2_FORCE_RESET();
begin
  RCC.APB2RSTR := $FFFFFFFF  ;
end;
procedure __HAL_RCC_SYSCFG_FORCE_RESET();
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SYSCFGRST);
end;

procedure __HAL_RCC_APB2_RELEASE_RESET();
begin
  RCC.APB2RSTR := $00;
end;
procedure __HAL_RCC_SYSCFG_RELEASE_RESET();
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_SYSCFGRST));
end;

(**
  * @end
  *)

(** @defgroup RCC_Peripheral_Clock_Sleep_Enable_Disable RCC Peripheral Clock Sleep Enable Disable
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @begin
  *)
procedure __HAL_RCC_CRC_CLK_SLEEP_ENABLE();
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_CRCLPEN);
end;
procedure __HAL_RCC_DMA1_CLK_SLEEP_ENABLE();
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_DMA1LPEN);
end;

procedure __HAL_RCC_CRC_CLK_SLEEP_DISABLE();
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_CRCLPEN));
end;
procedure __HAL_RCC_DMA1_CLK_SLEEP_DISABLE();
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_DMA1LPEN));
end;

(** @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
procedure __HAL_RCC_WWDG_CLK_SLEEP_ENABLE();
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_WWDGLPEN);
end;
procedure __HAL_RCC_PWR_CLK_SLEEP_ENABLE();
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_PWRLPEN);
end;

procedure __HAL_RCC_WWDG_CLK_SLEEP_DISABLE();
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_WWDGLPEN));
end;
procedure __HAL_RCC_PWR_CLK_SLEEP_DISABLE();
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_PWRLPEN));
end;

(** @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
procedure __HAL_RCC_SYSCFG_CLK_SLEEP_ENABLE();
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_SYSCFGLPEN);
end;
procedure __HAL_RCC_SYSCFG_CLK_SLEEP_DISABLE();
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_SYSCFGLPEN));
end;

(**
  * @end
  *)

(** @defgroup RCC_AHB1_Clock_Sleep_Enable_Disable_Status AHB1 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @begin
  *)
function __HAL_RCC_CRC_IS_CLK_SLEEP_ENABLED(): boolean;
begin
  exit((RCC.AHB1LPENR and (RCC_AHB1LPENR_CRCLPEN)) <> 0);
end;
function __HAL_RCC_DMA1_IS_CLK_SLEEP_ENABLED(): boolean;
begin
  exit((RCC.AHB1LPENR and (RCC_AHB1LPENR_DMA1LPEN)) <> 0);
end;

function __HAL_RCC_CRC_IS_CLK_SLEEP_DISABLED(): boolean;
begin
  exit((RCC.AHB1LPENR and (RCC_AHB1LPENR_CRCLPEN)) = 0);
end;
function __HAL_RCC_DMA1_IS_CLK_SLEEP_DISABLED(): boolean;
begin
  exit((RCC.AHB1LPENR and (RCC_AHB1LPENR_DMA1LPEN)) = 0);
end;
(**
  * @end
  *)

(** @defgroup RCC_APB1_Clock_Sleep_Enable_Disable_Status APB1 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @begin
  *)
function __HAL_RCC_WWDG_IS_CLK_SLEEP_ENABLED(): boolean;
begin
  exit((RCC.APB1LPENR and (RCC_APB1LPENR_WWDGLPEN))<> 0) ;
end;
function __HAL_RCC_PWR_IS_CLK_SLEEP_ENABLED(): boolean;
begin
  exit((RCC.APB1LPENR and (RCC_APB1LPENR_PWRLPEN))<> 0) ;
end;

function __HAL_RCC_WWDG_IS_CLK_SLEEP_DISABLED(): boolean;
begin
  exit((RCC.APB1LPENR and (RCC_APB1LPENR_WWDGLPEN)) = 0);
end;
function __HAL_RCC_PWR_IS_CLK_SLEEP_DISABLED(): boolean;
begin
  exit((RCC.APB1LPENR and (RCC_APB1LPENR_PWRLPEN)) = 0);
end;
(**
  * @end
  *)

(** @defgroup RCC_APB2_Clock_Sleep_Enable_Disable_Status APB2 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @begin
  *)
function __HAL_RCC_SYSCFG_IS_CLK_SLEEP_ENABLED(): boolean;
begin
  exit((RCC.APB2LPENR and (RCC_APB2LPENR_SYSCFGLPEN))<>0) ;
end;
function __HAL_RCC_SYSCFG_IS_CLK_SLEEP_DISABLED(): boolean;
begin
  exit((RCC.APB2LPENR and (RCC_APB2LPENR_SYSCFGLPEN)) = 0);
end;
(**
  * @end
  *)

(** @defgroup RCC_HSI_Configuration HSI Configuration
  * @begin
  *)

(** @brief  Macros to enable or disable the Internal High Speed oscillator (HSI).
  * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
  *         It is used (enabled by hardware) as system clock source after startup
  *         from Reset, wakeup from STOP and STANDBY mode, or in case of failure
  *         of the HSE used directly or indirectly as system clock (if the Clock
  *         Security System CSS is enabled).
  * @note   HSI can not be stopped if it is used as system clock source. In this case,
  *         you have to select another source of the system clock then stop the HSI.
  * @note   After enabling the HSI, the application software should wait on HSIRDY
  *         flag to be set indicating that HSI clock is stable and can be used as
  *         system clock source.
  * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
  *         clock cycles.
  *)
procedure __HAL_RCC_HSI_ENABLE();
begin
  RCC.CR := RCC.CR or (RCC_CR_HSION);
end;
procedure __HAL_RCC_HSI_DISABLE();
begin
  RCC.CR := RCC.CR and (not (RCC_CR_HSION));
end;

(** @brief  Macro to adjust the Internal High Speed oscillator (HSI) calibration value.
  * @note   The calibration is used to compensate for the variations in voltage
  *         and temperature that influence the frequency of the internal HSI RC.
  * @param  __HSICALIBRATIONVALUE__: specifies the calibration trimming value.
  *         This parameter must be a number between 0 and $1F.
  *)
procedure __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(__HSICALIBRATIONVALUE__: longword);
begin
  RCC.CR:=(RCC.CR and (not longword(RCC_CR_HSITRIM))) or ((__HSICALIBRATIONVALUE__) shl BsfDWord(RCC_CR_HSITRIM));
end;
(**
  * @end
  *)

(** @defgroup RCC_LSI_Configuration LSI Configuration
  * @begin
  *)

(** @brief  Macros to enable or disable the Internal Low Speed oscillator (LSI).
  * @note   After enabling the LSI, the application software should wait on
  *         LSIRDY flag to be set indicating that LSI clock is stable and can
  *         be used to clock the IWDG and/or the RTC.
  * @note   LSI can not be disabled if the IWDG is running.
  * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
  *         clock cycles.
  *)
procedure __HAL_RCC_LSI_ENABLE();
begin
  RCC.CSR := RCC.CSR or (RCC_CSR_LSION);
end;
procedure __HAL_RCC_LSI_DISABLE();
begin
  RCC.CSR := RCC.CSR and (not (RCC_CSR_LSION));
end;
(**
  * @end
  *)

(** @defgroup RCC_HSE_Configuration HSE Configuration
  * @begin
  *)
(**
  * @brief  Macro to configure the External High Speed oscillator (__HSE__).
  * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
  *         software should wait on HSERDY flag to be set indicating that HSE clock
  *         is stable and can be used to clock the PLL and/or system clock.
  * @note   HSE state can not be changed if it is used directly or through the
  *         PLL as system clock. In this case, you have to select another source
  *         of the system clock then change the HSE state (ex. disable it).
  * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
  * @note   This function reset the CSSON bit, so if the clock security system(CSS)
  *         was previously enabled you have to enable it again after calling this
  *         function.
  * @param  __STATE__: specifies the new state of the HSE.
  *         This parameter can be one of the following values:
  *            @arg RCC_HSE_OFF: turn OFF the HSE oscillator, HSERDY flag goes low after
  *                              6 HSE oscillator clock cycles.
  *            @arg RCC_HSE_ON: turn ON the HSE oscillator.
  *            @arg RCC_HSE_BYPASS: HSE oscillator bypassed with external clock.
  *)
procedure __HAL_RCC_HSE_CONFIG(__STATE__: longword);
begin
  RCC.CR := RCC.CR and (not longword(RCC_CR_HSEON));
  if((__STATE__) = RCC_HSE_ON)then
  begin
    RCC.CR := RCC.CR and (not longword(RCC_CR_HSEBYP));
    RCC.CR := RCC.CR or longword(RCC_CR_HSEON);
  end
  else if((__STATE__) = RCC_HSE_BYPASS) then
  begin
    RCC.CR := RCC.CR or longword(RCC_CR_HSEBYP);
    RCC.CR := RCC.CR or longword(RCC_CR_HSEON);
  end
  else
  begin
    RCC.CR := RCC.CR and (not longword(RCC_CR_HSEBYP));
    RCC.CR := RCC.CR and (not longword(RCC_CR_HSEON));
  end;
end;
(**
  * @end
  *)

(** @defgroup RCC_LSE_Configuration LSE Configuration
  * @begin
  *)

(**
  * @brief  Macro to configure the External Low Speed oscillator (LSE).
  * @note   Transition LSE Bypass to LSE On and LSE On to LSE Bypass are not supported by this macro.
  *         User should request a transition to LSE Off first and then LSE On or LSE Bypass.
  * @note   As the LSE is in the Backup domain and write access is denied to
  *         this domain after reset, you have to enable write access using
  *         HAL_PWR_EnableBkUpAccess() function before to configure the LSE
  *         (to be done once after reset).
  * @note   After enabling the LSE (RCC_LSE_ON or RCC_LSE_BYPASS), the application
  *         software should wait on LSERDY flag to be set indicating that LSE clock
  *         is stable and can be used to clock the RTC.
  * @param  __STATE__: specifies the new state of the LSE.
  *         This parameter can be one of the following values:
  *            @arg RCC_LSE_OFF: turn OFF the LSE oscillator, LSERDY flag goes low after
  *                              6 LSE oscillator clock cycles.
  *            @arg RCC_LSE_ON: turn ON the LSE oscillator.
  *            @arg RCC_LSE_BYPASS: LSE oscillator bypassed with external clock.
  *)
procedure __HAL_RCC_LSE_CONFIG(__STATE__: longword);
begin
  RCC.BDCR := RCC.BDCR and (not longword(RCC_BDCR_LSEON));
  if((__STATE__) = RCC_LSE_ON)then
  begin
    RCC.BDCR := RCC.BDCR and (not longword(RCC_BDCR_LSEBYP));
    RCC.BDCR := RCC.BDCR or longword(RCC_BDCR_LSEON);
  end
  else if((__STATE__) = RCC_LSE_BYPASS)then
  begin
    RCC.BDCR := RCC.BDCR or longword(RCC_BDCR_LSEBYP);
    RCC.BDCR := RCC.BDCR or longword(RCC_BDCR_LSEON);
  end
  else
  begin
    RCC.BDCR := RCC.BDCR and (not longword(RCC_BDCR_LSEBYP));
    RCC.BDCR := RCC.BDCR and (not longword(RCC_BDCR_LSEON));
  end;
end;
(**
  * @end
  *)

(** @defgroup RCC_Internal_RTC_Clock_Configuration RTC Clock Configuration
  * @begin
  *)

(** @brief  Macros to enable or disable the RTC clock.
  * @note   These macros must be used only after the RTC clock source was selected.
  *)
procedure __HAL_RCC_RTC_ENABLE();
begin
  RCC.BDCR := RCC.BDCR or (RCC_BDCR_RTCEN);
end;
procedure __HAL_RCC_RTC_DISABLE();
begin
  RCC.BDCR := RCC.BDCR and (not (RCC_BDCR_RTCEN));
end;

(** @brief  Macros to configure the RTC clock (RTCCLK).
  * @note   As the RTC clock configuration bits are in the Backup domain and write
  *         access is denied to this domain after reset, you have to enable write
  *         access using the Power Backup Access macro before to configure
  *         the RTC clock source (to be done once after reset).
  * @note   Once the RTC clock is configured it can't be changed unless the
  *         Backup domain is reset using __HAL_RCC_BackupReset_RELEASE() macro, or by
  *         a Power On Reset (POR).
  * @param  __RTCCLKSource__: specifies the RTC clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_RTCCLKSOURCE_LSE: LSE selected as RTC clock.
  *            @arg RCC_RTCCLKSOURCE_LSI: LSI selected as RTC clock.
  *            @arg RCC_RTCCLKSOURCE_HSE_DIVx: HSE clock divided by x selected
  *                                            as RTC clock, where x:[2,31]
  * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
  *         work in STOP and STANDBY modes, and can be used as wakeup source.
  *         However, when the HSE clock is used as RTC clock source, the RTC
  *         cannot be used in STOP and STANDBY modes.
  * @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
  *         RTC clock source).
  *)
procedure __HAL_RCC_RTC_CLKPRESCALER(__RTCCLKSource__: longword);
begin
  if (((__RTCCLKSource__) and RCC_BDCR_RTCSEL) = RCC_BDCR_RTCSEL)  then
    RCC.CFGR := (RCC.CFGR and (not longword(RCC_CFGR_RTCPRE))) or ((__RTCCLKSource__) and $FFFFCFF)
  else
    RCC.CFGR := RCC.CFGR and (not longword(RCC_CFGR_RTCPRE));
end;


procedure __HAL_RCC_RTC_CONFIG(__RTCCLKSource__: longword);
begin
  __HAL_RCC_RTC_CLKPRESCALER(__RTCCLKSource__);
  RCC.BDCR := RCC.BDCR or ((__RTCCLKSource__) and $00000FFF);
end;

(** @brief  Macros to force or release the Backup domain reset.
  * @note   This function resets the RTC peripheral (including the backup registers)
  *         and the RTC clock source selection in RCC_CSR register.
  * @note   The BKPSRAM is not affected by this reset.
  *)
procedure __HAL_RCC_BACKUPRESET_FORCE();
begin
  RCC.BDCR := RCC.BDCR or (RCC_BDCR_BDRST);
end;
procedure __HAL_RCC_BACKUPRESET_RELEASE();
begin
  RCC.BDCR := RCC.BDCR and (not (RCC_BDCR_BDRST));
end;
(**
  * @end
  *)

(** @defgroup RCC_PLL_Configuration PLL Configuration
  * @begin
  *)

(** @brief  Macros to enable or disable the main PLL.
  * @note   After enabling the main PLL, the application software should wait on
  *         PLLRDY flag to be set indicating that PLL clock is stable and can
  *         be used as system clock source.
  * @note   The main PLL can not be disabled if it is used as system clock source
  * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
  *)
procedure __HAL_RCC_PLL_ENABLE();
begin
  RCC.CR := RCC.CR or longword(RCC_CR_PLLON);
end;
procedure __HAL_RCC_PLL_DISABLE();
begin
  RCC.CR := RCC.CR and (not longword(RCC_CR_PLLON));
end;

(** @brief  Macro to configure the PLL clock source.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __PLLSOURCE__: specifies the PLL entry clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_PLLSOURCE_HSI: HSI oscillator clock selected as PLL clock entry
  *            @arg RCC_PLLSOURCE_HSE: HSE oscillator clock selected as PLL clock entry
  *
  *)
procedure __HAL_RCC_PLL_PLLSOURCE_CONFIG(__PLLSOURCE__: longword);
begin
  RCC.PLLCFGR := (RCC.PLLCFGR and (not longword(RCC_PLLCFGR_PLLSRC))) or (__PLLSOURCE__);
end;

(** @brief  Macro to configure the PLL multiplication factor.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __PLLM__: specifies the division factor for PLL VCO input clock
  *         This parameter must be a number between Min_Data := 2 and Max_Data := 63.
  * @note   You have to set the PLLM parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
  *         of 2 MHz to limit PLL jitter.
  *
  *)
procedure __HAL_RCC_PLL_PLLM_CONFIG(__PLLM__: longword);
begin
  RCC.PLLCFGR := (RCC.PLLCFGR and (not longword(RCC_PLLCFGR_PLLM))) or (__PLLM__);
end;
(**
  * @end
  *)

(** @defgroup RCC_PLL_I2S_Configuration PLL I2S Configuration
  * @begin
  *)

(** @brief  Macro to configure the I2S clock source (I2SCLK).
  * @note   This function must be called before enabling the I2S APB clock.
  * @param  __SOURCE__: specifies the I2S clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_I2SCLKSOURCE_PLLI2S: PLLI2S clock used as I2S clock source.
  *            @arg RCC_I2SCLKSOURCE_EXT: External clock mapped on the I2S_CKIN pin
  *                                       used as I2S clock source.
  *)
procedure __HAL_RCC_I2S_CONFIG(__SOURCE__: longword);
begin
  RCC.CFGR := RCC.CFGR and (not (RCC_CFGR_I2SSRC));
  RCC.CFGR := RCC.CFGR or (__SOURCE__);
end;

(** @brief Macros to enable or disable the PLLI2S.
  * @note  The PLLI2S is disabled by hardware when entering STOP and STANDBY modes.
  *)
procedure __HAL_RCC_PLLI2S_ENABLE();
begin
  RCC.CR := RCC.CR or (RCC_CR_PLLI2SON);
end;
procedure __HAL_RCC_PLLI2S_DISABLE();
begin
  RCC.CR := RCC.CR and (not (RCC_CR_PLLI2SON));
end;
(**
  * @end
  *)

(** @defgroup RCC_Get_Clock_source Get Clock source
  * @begin
  *)
(**
  * @brief Macro to configure the system clock source.
  * @param __RCC_SYSCLKSOURCE__: specifies the system clock source.
  * This parameter can be one of the following values:
  *              - RCC_SYSCLKSOURCE_HSI: HSI oscillator is used as system clock source.
  *              - RCC_SYSCLKSOURCE_HSE: HSE oscillator is used as system clock source.
  *              - RCC_SYSCLKSOURCE_PLLCLK: PLL output is used as system clock source.
  *)
procedure __HAL_RCC_SYSCLK_CONFIG(__RCC_SYSCLKSOURCE__: longword);
begin
  RCC.CFGR := (RCC.CFGR and (not longword(RCC_CFGR_SW))) or (__RCC_SYSCLKSOURCE__);
end;

(** @brief  Macro to get the clock source used as system clock.
  * @retval The clock source used as system clock. The returned value can be one
  *         of the following:
  *              - RCC_SYSCLKSOURCE_STATUS_HSI: HSI used as system clock.
  *              - RCC_SYSCLKSOURCE_STATUS_HSE: HSE used as system clock.
  *              - RCC_SYSCLKSOURCE_STATUS_PLLCLK: PLL used as system clock.
  *)
function __HAL_RCC_GET_SYSCLK_SOURCE(): longword;
begin
  exit((RCC.CFGR and RCC_CFGR_SWS));
end;

(**
  * @brief  Macro to configures the External Low Speed oscillator (LSE) drive capability.
  * @note   As the LSE is in the Backup domain and write access is denied to
  *         this domain after reset, you have to enable write access using
  *         HAL_PWR_EnableBkUpAccess() function before to configure the LSE
  *         (to be done once after reset).
  * @param  __RCC_LSEDRIVE__: specifies the new state of the LSE drive capability.
  *          This parameter can be one of the following values:
  *            @arg RCC_LSEDRIVE_LOW: LSE oscillator low drive capability.
  *            @arg RCC_LSEDRIVE_MEDIUMLOW: LSE oscillator medium low drive capability.
  *            @arg RCC_LSEDRIVE_MEDIUMHIGH: LSE oscillator medium high drive capability.
  *            @arg RCC_LSEDRIVE_HIGH: LSE oscillator high drive capability.
  * @retval None
  *)
procedure __HAL_RCC_LSEDRIVE_CONFIG(__RCC_LSEDRIVE__: longword);
begin
  RCC.BDCR := (RCC.BDCR and (not longword(RCC_BDCR_LSEDRV))) or (__RCC_LSEDRIVE__) ;
end;

(** @brief  Macro to get the oscillator used as PLL clock source.
  * @retval The oscillator used as PLL clock source. The returned value can be one
  *         of the following:
  *              - RCC_PLLSOURCE_HSI: HSI oscillator is used as PLL clock source.
  *              - RCC_PLLSOURCE_HSE: HSE oscillator is used as PLL clock source.
  *)
function __HAL_RCC_GET_PLL_OSCSOURCE: longword;
begin
  exit((RCC.PLLCFGR and RCC_PLLCFGR_PLLSRC));
end;
(**
  * @end
  *)

(** @defgroup RCC_Flags_Interrupts_Management Flags Interrupts Management
  * @brief macros to manage the specified RCC Flags and interrupts.
  * @begin
  *)

(** @brief  Enable RCC interrupt (Perform Byte access to RCC_CIR[14:8] bits to enable
  *         the selected interrupts).
  * @param  __INTERRUPT__: specifies the RCC interrupt sources to be enabled.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  *)
procedure __HAL_RCC_ENABLE_IT(__INTERRUPT__: longword);
begin
  pbyte(RCC_CIR_BYTE1_ADDRESS)^ := pbyte(RCC_CIR_BYTE1_ADDRESS)^ or (__INTERRUPT__);
end;

(** @brief Disable RCC interrupt (Perform Byte access to RCC_CIR[14:8] bits to disable
  *        the selected interrupts).
  * @param  __INTERRUPT__: specifies the RCC interrupt sources to be disabled.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  *)
procedure __HAL_RCC_DISABLE_IT(__INTERRUPT__: longword);
begin
  pbyte(RCC_CIR_BYTE1_ADDRESS)^ := pbyte(RCC_CIR_BYTE1_ADDRESS)^ and (not (__INTERRUPT__));
end;

(** @brief  Clear the RCC's interrupt pending bits (Perform Byte access to RCC_CIR[23:16]
  *         bits to clear the selected interrupt pending bits.
  * @param  __INTERRUPT__: specifies the interrupt pending bit to clear.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  *            @arg RCC_IT_CSS: Clock Security System interrupt
  *)
procedure __HAL_RCC_CLEAR_IT(__INTERRUPT__: longword);
begin
  pbyte(RCC_CIR_BYTE2_ADDRESS)^ := __INTERRUPT__;
end;

(** @brief  Check the RCC's interrupt has occurred or not.
  * @param  __INTERRUPT__: specifies the RCC interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  *            @arg RCC_IT_CSS: Clock Security System interrupt
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  *)
function __HAL_RCC_GET_IT(__INTERRUPT__: longword): boolean;
begin
  exit((RCC.CIR and (__INTERRUPT__)) = (__INTERRUPT__));
end;

(** @brief Set RMVF bit to clear the reset flags: RCC_FLAG_PINRST, RCC_FLAG_PORRST,
  *        RCC_FLAG_SFTRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST and RCC_FLAG_LPWRRST.
  *)
procedure __HAL_RCC_CLEAR_RESET_FLAGS();
begin
  RCC.CSR := RCC.CSR or RCC_CSR_RMVF;
end;

function __HAL_RCC_GET_FLAG(__FLAG__: longword): boolean;
var
  t: longword;
begin
  if (__FLAG__ shr 5) = 1 then
    t := RCC.CR
  else if (__FLAG__ shr 5) = 2 then
    t := RCC.BDCR
  else if (__FLAG__ shr 5) = 3 then
    t := RCC.CSR
  else
    t := RCC.CIR;

  exit((t and (1 shl ((__FLAG__) and RCC_FLAG_MASK))) <> 0);
end;

procedure SET_BIT(var a: longword; b: longword);
begin
  a := a or b;
end;

procedure CLEAR_BIT(var a: longword; b: longword);
begin
  a := a and (not b);
end;

procedure CLEAR_REG(var a: longword);
begin
  a := 0;
end;

procedure MODIFY_REG(var a: longword; b, c: longword);
begin
  a := (a and (not b)) or c;
end;

function READ_BIT(var a: longword; b: longword): longword;
begin
  READ_BIT := a and b;
end;

procedure HAL_RCC_DeInit;
begin
  (* Set HSION bit *)
  SET_BIT(RCC.CR, RCC_CR_HSION or RCC_CR_HSITRIM_4);

  (* Reset CFGR register *)
  CLEAR_REG(RCC.CFGR);

  (* Reset HSEON, CSSON, PLLON, PLLI2S *)
  CLEAR_BIT(RCC.CR, RCC_CR_HSEON or RCC_CR_CSSON or RCC_CR_PLLON or RCC_CR_PLLI2SON);

  (* Reset PLLCFGR register *)
  CLEAR_REG(RCC.PLLCFGR);
  SET_BIT(RCC.PLLCFGR, RCC_PLLCFGR_PLLM_4 or RCC_PLLCFGR_PLLN_6 or RCC_PLLCFGR_PLLN_7 or RCC_PLLCFGR_PLLQ_2);

  (* Reset PLLI2SCFGR register *)
  CLEAR_REG(RCC.PLLI2SCFGR);
  SET_BIT(RCC.PLLI2SCFGR, RCC_PLLI2SCFGR_PLLI2SN_6 or RCC_PLLI2SCFGR_PLLI2SN_7 or RCC_PLLI2SCFGR_PLLI2SR_1);

  (* Reset HSEBYP bit *)
  CLEAR_BIT(RCC.CR, RCC_CR_HSEBYP);

  (* Disable all interrupts *)
  CLEAR_REG(RCC.CIR);
end;

procedure __HAL_RCC_PLL_CONFIG(__RCC_PLLSource__, __PLLM__, __PLLN__, __PLLP__, __PLLQ__: longword);
begin
  RCC.PLLCFGR := ($20000000 or (__PLLM__) or ((__PLLN__) shl BsfDWord(RCC_PLLCFGR_PLLN)) or ((((__PLLP__) shr 1) - 1) shl BsfDWord(RCC_PLLCFGR_PLLP)) or (__RCC_PLLSource__) or ((__PLLQ__) shl BsfDWord(RCC_PLLCFGR_PLLQ)));
end;

function HAL_RCC_OscConfig(const RCC_OscInitStruct: RCC_OscInitTypeDef): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  tickstart := 0;

  (*------------------------------- HSE Configuration ------------------------*)
  if (((RCC_OscInitStruct.OscillatorType) and RCC_OSCILLATORTYPE_HSE) = RCC_OSCILLATORTYPE_HSE) then
  begin
    (* When the HSE is used as system clock or clock source for PLL, It can not be disabled *)
    if ((__HAL_RCC_GET_SYSCLK_SOURCE() = RCC_SYSCLKSOURCE_STATUS_HSE) or ((__HAL_RCC_GET_SYSCLK_SOURCE() = RCC_SYSCLKSOURCE_STATUS_PLLCLK) and ((RCC.PLLCFGR and RCC_PLLCFGR_PLLSRC) = RCC_PLLCFGR_PLLSRC_HSE))) then
    begin
      if ((__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY)) and (RCC_OscInitStruct.HSEState = RCC_HSE_OFF)) then
      begin
        exit(HAL_ERROR);
      end;
    end
    else
    begin
      (* Reset HSEON and HSEBYP bits before configuring the HSE --------------*)
      __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);

      (* Get Start Tick*)
      tickstart := HAL_GetTick();

      (* Wait till HSE is disabled *)
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY)) do
      begin
        if ((HAL_GetTick() - tickstart) > HSE_TIMEOUT_VALUE) then
        begin
          exit(HAL_TIMEOUT);
        end;
      end;

      (* Set the new HSE configuration ---------------------------------------*)
      __HAL_RCC_HSE_CONFIG(RCC_OscInitStruct.HSEState);

      (* Check the HSE State *)
      if (RCC_OscInitStruct.HSEState <> RCC_HSE_OFF) then
      begin
        (* Get Start Tick*)
        tickstart := HAL_GetTick();

        (* Wait till HSE is ready *)
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) = False) do
        begin
          if ((HAL_GetTick() - tickstart) > HSE_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end
      else
      begin
        (* Get Start Tick*)
        tickstart := HAL_GetTick();

        (* Wait till HSE is bypassed or disabled *)
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY)) do
        begin
          if ((HAL_GetTick() - tickstart) > HSE_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end;
    end;
  end;
  (*----------------------------- HSI Configuration --------------------------*)
  if (((RCC_OscInitStruct.OscillatorType) and RCC_OSCILLATORTYPE_HSI) = RCC_OSCILLATORTYPE_HSI) then
  begin
    (* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock *)
    if ((__HAL_RCC_GET_SYSCLK_SOURCE() = RCC_SYSCLKSOURCE_STATUS_HSI) or ((__HAL_RCC_GET_SYSCLK_SOURCE() = RCC_SYSCLKSOURCE_STATUS_PLLCLK) and ((RCC.PLLCFGR and RCC_PLLCFGR_PLLSRC) = RCC_PLLCFGR_PLLSRC_HSI))) then
    begin
      (* When HSI is used as system clock it will not disabled *)
      if ((__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY)) and (RCC_OscInitStruct.HSIState <> RCC_HSI_ON)) then
      begin
        exit(HAL_ERROR);
      end
      (* Otherwise, just the calibration is allowed *)
      else
      begin
        (* Adjusts the Internal High Speed oscillator (HSI) calibration value.*)
        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct.HSICalibrationValue);
      end;
    end
    else
    begin
      (* Check the HSI State *)
      if ((RCC_OscInitStruct.HSIState) <> RCC_HSI_OFF) then
      begin
        (* Enable the Internal High Speed oscillator (HSI). *)
        __HAL_RCC_HSI_ENABLE();

        (* Get Start Tick*)
        tickstart := HAL_GetTick();

        (* Wait till HSI is ready *)
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) = False) do
        begin
          if ((HAL_GetTick() - tickstart) > HSI_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;

        (* Adjusts the Internal High Speed oscillator (HSI) calibration value.*)
        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct.HSICalibrationValue);
      end
      else
      begin
        (* Disable the Internal High Speed oscillator (HSI). *)
        __HAL_RCC_HSI_DISABLE();

        (* Get Start Tick*)
        tickstart := HAL_GetTick();

        (* Wait till HSI is ready *)
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY)) do
        begin
          if ((HAL_GetTick() - tickstart) > HSI_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end;
    end;
  end;
  (*------------------------------ LSI Configuration -------------------------*)
  if (((RCC_OscInitStruct.OscillatorType) and RCC_OSCILLATORTYPE_LSI) = RCC_OSCILLATORTYPE_LSI) then
  begin
    (* Check the LSI State *)
    if ((RCC_OscInitStruct.LSIState) <> RCC_LSI_OFF) then
    begin
      (* Enable the Internal Low Speed oscillator (LSI). *)
      __HAL_RCC_LSI_ENABLE();

      (* Get Start Tick*)
      tickstart := HAL_GetTick();

      (* Wait till LSI is ready *)
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) = False) do
      begin
        if ((HAL_GetTick() - tickstart) > LSI_TIMEOUT_VALUE) then
        begin
          exit(HAL_TIMEOUT);
        end;
      end;
    end
    else
    begin
      (* Disable the Internal Low Speed oscillator (LSI). *)
      __HAL_RCC_LSI_DISABLE();

      (* Get Start Tick*)
      tickstart := HAL_GetTick();

      (* Wait till LSI is ready *)
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)) do
      begin
        if ((HAL_GetTick() - tickstart) > LSI_TIMEOUT_VALUE) then
        begin
          exit(HAL_TIMEOUT);
        end;
      end;
    end;
  end;
  (*------------------------------ LSE Configuration -------------------------*)
  if (((RCC_OscInitStruct.OscillatorType) and RCC_OSCILLATORTYPE_LSE) = RCC_OSCILLATORTYPE_LSE) then
  begin
    (* Enable Power Clock*)
    __HAL_RCC_PWR_CLK_ENABLE();

    (* Enable write access to Backup domain *)
    PWR.CR1 := PWR.CR1 or PWR_CR1_DBP;

    (* Wait for Backup domain Write protection disable *)
    tickstart := HAL_GetTick();

    while ((PWR.CR1 and PWR_CR1_DBP) = 0) do
    begin
      if ((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE) then
      begin
        exit(HAL_TIMEOUT);
      end;
    end;

    (* Reset LSEON and LSEBYP bits before configuring the LSE ----------------*)
    __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);

    (* Get Start Tick*)
    tickstart := HAL_GetTick();

    (* Wait till LSE is ready *)
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) do
    begin
      if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) then
      begin
        exit(HAL_TIMEOUT);
      end;
    end;

    (* Set the new LSE configuration -----------------------------------------*)
    __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct.LSEState);
    (* Check the LSE State *)
    if ((RCC_OscInitStruct.LSEState) <> RCC_LSE_OFF) then
    begin
      (* Get Start Tick*)
      tickstart := HAL_GetTick();

      (* Wait till LSE is ready *)
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) = False) do
      begin
        if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) then
        begin
          exit(HAL_TIMEOUT);
        end;
      end;
    end
    else
    begin
      (* Get Start Tick*)
      tickstart := HAL_GetTick();

      (* Wait till LSE is ready *)
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) do
      begin
        if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) then
        begin
          exit(HAL_TIMEOUT);
        end;
      end;
    end;
  end;
  (*-------------------------------- PLL Configuration -----------------------*)
  if ((RCC_OscInitStruct.PLL.PLLState) <> RCC_PLL_NONE) then
  begin
    (* Check if the PLL is used as system clock or not *)
    if (__HAL_RCC_GET_SYSCLK_SOURCE() <> RCC_SYSCLKSOURCE_STATUS_PLLCLK) then
    begin
      if ((RCC_OscInitStruct.PLL.PLLState) = RCC_PLL_ON) then
      begin
        (* Disable the main PLL. *)
        __HAL_RCC_PLL_DISABLE();

        (* Get Start Tick*)
        tickstart := HAL_GetTick();

        (* Wait till PLL is ready *)
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)) do
        begin
          if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;

        (* Configure the main PLL clock source, multiplication and division factors. *)
        __HAL_RCC_PLL_CONFIG(RCC_OscInitStruct.PLL.PLLSource,
          RCC_OscInitStruct.PLL.PLLM,
          RCC_OscInitStruct.PLL.PLLN,
          RCC_OscInitStruct.PLL.PLLP,
          RCC_OscInitStruct.PLL.PLLQ);
        (* Enable the main PLL. *)
        __HAL_RCC_PLL_ENABLE();

        (* Get Start Tick*)
        tickstart := HAL_GetTick();

        (* Wait till PLL is ready *)
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) = False) do
        begin
          if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end
      else
      begin
        (* Disable the main PLL. *)
        __HAL_RCC_PLL_DISABLE();

        (* Get Start Tick*)
        tickstart := HAL_GetTick();

        (* Wait till PLL is ready *)
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)) do
        begin
          if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end;
    end
    else
    begin
      exit(HAL_ERROR);
    end;
  end;
  exit(HAL_OK);
end;

function HAL_RCC_ClockConfig(const RCC_ClkInitStruct: RCC_ClkInitTypeDef; FLatency: longword): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  tickstart := 0;

  (* To correctly read data from FLASH memory, the number of wait states (LATENCY)
     must be correctly programmed according to the frequency of the CPU clock
     (HCLK) and the supply voltage of the device. *)

  (* Increasing the CPU frequency *)
  if (FLatency > (FLASH.ACR and FLASH_ACR_LATENCY)) then
  begin
    (* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register *)
    __HAL_FLASH_SET_LATENCY(FLatency);

    (* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register *)
    if ((FLASH.ACR and FLASH_ACR_LATENCY) <> FLatency) then
    begin
      exit(HAL_ERROR);
    end;

    (*-------------------------- HCLK Configuration --------------------------*)
    if (((RCC_ClkInitStruct.ClockType) and RCC_CLOCKTYPE_HCLK) = RCC_CLOCKTYPE_HCLK) then
    begin
      MODIFY_REG(RCC.CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct.AHBCLKDivider);
    end;

    (*------------------------- SYSCLK Configuration ---------------------------*)
    if (((RCC_ClkInitStruct.ClockType) and RCC_CLOCKTYPE_SYSCLK) = RCC_CLOCKTYPE_SYSCLK) then
    begin
      (* HSE is selected as System Clock Source *)
      if (RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE) then
      begin
        (* Check the HSE ready flag *)
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) = False) then
        begin
          exit(HAL_ERROR);
        end;
      end
      (* PLL is selected as System Clock Source *)
      else if (RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK) then
      begin
        (* Check the PLL ready flag *)
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) = False) then
        begin
          exit(HAL_ERROR);
        end;
      end
      (* HSI is selected as System Clock Source *)
      else
      begin
        (* Check the HSI ready flag *)
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) = False) then
        begin
          exit(HAL_ERROR);
        end;
      end;

      __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct.SYSCLKSource);
      (* Get Start Tick*)
      tickstart := HAL_GetTick();

      if (RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE) then
      begin
        while (__HAL_RCC_GET_SYSCLK_SOURCE() <> RCC_SYSCLKSOURCE_STATUS_HSE) do
        begin
          if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end
      else if (RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK) then
      begin
        while (__HAL_RCC_GET_SYSCLK_SOURCE() <> RCC_SYSCLKSOURCE_STATUS_PLLCLK) do
        begin
          if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end
      else
      begin
        while (__HAL_RCC_GET_SYSCLK_SOURCE() <> RCC_SYSCLKSOURCE_STATUS_HSI) do
        begin
          if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end;
    end;
  end
  (* Decreasing the CPU frequency *)
  else
  begin
    (*-------------------------- HCLK Configuration --------------------------*)
    if (((RCC_ClkInitStruct.ClockType) and RCC_CLOCKTYPE_HCLK) = RCC_CLOCKTYPE_HCLK) then
    begin
      MODIFY_REG(RCC.CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct.AHBCLKDivider);
    end;

    (*------------------------- SYSCLK Configuration -------------------------*)
    if (((RCC_ClkInitStruct.ClockType) and RCC_CLOCKTYPE_SYSCLK) = RCC_CLOCKTYPE_SYSCLK) then
    begin
      (* HSE is selected as System Clock Source *)
      if (RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE) then
      begin
        (* Check the HSE ready flag *)
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) = False) then
        begin
          exit(HAL_ERROR);
        end;
      end
      (* PLL is selected as System Clock Source *)
      else if (RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK) then
      begin
        (* Check the PLL ready flag *)
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) = False) then
        begin
          exit(HAL_ERROR);
        end;
      end
      (* HSI is selected as System Clock Source *)
      else
      begin
        (* Check the HSI ready flag *)
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) = False) then
        begin
          exit(HAL_ERROR);
        end;
      end;
      __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct.SYSCLKSource);
      (* Get Start Tick*)
      tickstart := HAL_GetTick();

      if (RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE) then
      begin
        while (__HAL_RCC_GET_SYSCLK_SOURCE() <> RCC_SYSCLKSOURCE_STATUS_HSE) do
        begin
          if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end
      else if (RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK) then
      begin
        while (__HAL_RCC_GET_SYSCLK_SOURCE() <> RCC_SYSCLKSOURCE_STATUS_PLLCLK) do
        begin
          if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end
      else
      begin
        while (__HAL_RCC_GET_SYSCLK_SOURCE() <> RCC_SYSCLKSOURCE_STATUS_HSI) do
        begin
          if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) then
          begin
            exit(HAL_TIMEOUT);
          end;
        end;
      end;
    end;

    (* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register *)
    __HAL_FLASH_SET_LATENCY(FLatency);

    (* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register *)
    if ((FLASH.ACR and FLASH_ACR_LATENCY) <> FLatency) then
    begin
      exit(HAL_ERROR);
    end;
  end;

  (*-------------------------- PCLK1 Configuration ---------------------------*)
  if (((RCC_ClkInitStruct.ClockType) and RCC_CLOCKTYPE_PCLK1) = RCC_CLOCKTYPE_PCLK1) then
  begin
    MODIFY_REG(RCC.CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct.APB1CLKDivider);
  end;

  (*-------------------------- PCLK2 Configuration ---------------------------*)
  if (((RCC_ClkInitStruct.ClockType) and RCC_CLOCKTYPE_PCLK2) = RCC_CLOCKTYPE_PCLK2) then
  begin
    MODIFY_REG(RCC.CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct.APB2CLKDivider) shl 3));
  end;

  (* Configure the source of time base considering new system clocks settings*)
  HAL_InitTick(TICK_INT_PRIORITY);

  exit(HAL_OK);
end;

procedure HAL_RCC_MCOConfig(RCC_MCOx, RCC_MCOSource, RCC_MCODiv: longword);
//var GPIO_InitStruct: GPIO_InitTypeDef;
begin
  {(* RCC_MCO1 *)
  if(RCC_MCOx = RCC_MCO1) then
  begin
    (* MCO1 Clock Enable *)
    MCO1_CLK_ENABLE();

    (* Configure the MCO1 pin in alternate function mode *)
    GPIO_InitStruct.Pin := MCO1_PIN;
    GPIO_InitStruct.Mode := GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed := GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull := GPIO_NOPULL;
    GPIO_InitStruct.Alternate := GPIO_AF0_MCO;
    HAL_GPIO_Init(MCO1_GPIO_PORT, andGPIO_InitStruct);

    (* Mask MCO1 and MCO1PRE[2:0] bits then Select MCO1 clock source and prescaler *)
    MODIFY_REG(RCC.CFGR, (RCC_CFGR_MCO1 or RCC_CFGR_MCO1PRE), (RCC_MCOSource or RCC_MCODiv));
  end
  else
  begin
    (* MCO2 Clock Enable *)
    MCO2_CLK_ENABLE();

    (* Configure the MCO2 pin in alternate function mode *)
    GPIO_InitStruct.Pin := MCO2_PIN;
    GPIO_InitStruct.Mode := GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed := GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull := GPIO_NOPULL;
    GPIO_InitStruct.Alternate := GPIO_AF0_MCO;
    HAL_GPIO_Init(MCO2_GPIO_PORT, andGPIO_InitStruct);

    (* Mask MCO2 and MCO2PRE[2:0] bits then Select MCO2 clock source and prescaler *)
    MODIFY_REG(RCC.CFGR, (RCC_CFGR_MCO2 or RCC_CFGR_MCO2PRE), (RCC_MCOSource or (RCC_MCODiv shl 3)));
  end;}
end;

procedure HAL_RCC_EnableCSS;
begin
  SET_BIT(RCC.CR, RCC_CR_CSSON);
end;

procedure HAL_RCC_DisableCSS;
begin
  CLEAR_BIT(RCC.CR, RCC_CR_CSSON);
end;

function HAL_RCC_GetSysClockFreq: longword;
var
  pllm, pllvco, pllp, sysclockfreq: longword;
begin
  pllm := 0;
  pllvco := 0;
  pllp := 0;
  sysclockfreq := 0;

  (* Get SYSCLK source -------------------------------------------------------*)
  case (RCC.CFGR and RCC_CFGR_SWS) of
    RCC_SYSCLKSOURCE_STATUS_HSI:  (* HSI used as system clock source *)
    begin
      sysclockfreq := HSI_VALUE;
    end;
    RCC_SYSCLKSOURCE_STATUS_HSE:  (* HSE used as system clock  source *)
    begin
      sysclockfreq := HSE_VALUE;
    end;
    RCC_SYSCLKSOURCE_STATUS_PLLCLK:  (* PLL used as system clock  source *)
    begin
      (* PLL_VCO := (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
      SYSCLK := PLL_VCO / PLLP *)
      pllm := RCC.PLLCFGR and RCC_PLLCFGR_PLLM;
      if (__HAL_RCC_GET_PLL_OSCSOURCE() <> RCC_PLLCFGR_PLLSRC_HSI) then
      begin
        (* HSE used as PLL clock source *)
        pllvco := ((HSE_VALUE div pllm) * ((RCC.PLLCFGR and RCC_PLLCFGR_PLLN) shr BsfDWord(RCC_PLLCFGR_PLLN)));
      end
      else
      begin
        (* HSI used as PLL clock source *)
        pllvco := ((HSI_VALUE div pllm) * ((RCC.PLLCFGR and RCC_PLLCFGR_PLLN) shr BsfDWord(RCC_PLLCFGR_PLLN)));
      end;
      pllp := ((((RCC.PLLCFGR and RCC_PLLCFGR_PLLP) shr BsfDWord(RCC_PLLCFGR_PLLP)) + 1) * 2);

      sysclockfreq := pllvco div pllp;
    end;
    else
    begin
      sysclockfreq := HSI_VALUE;
    end;
  end;
  exit(sysclockfreq);
end;

function HAL_RCC_GetHCLKFreq: longword;
begin
  SystemCoreClock := HAL_RCC_GetSysClockFreq() shr APBAHBPrescTable[(RCC.CFGR and RCC_CFGR_HPRE) shr BsfDWord(RCC_CFGR_HPRE)];
  exit(SystemCoreClock);
end;

function HAL_RCC_GetPCLK1Freq: longword;
begin
  (* Get HCLK source and Compute PCLK1 frequency ---------------------------*)
  exit(HAL_RCC_GetHCLKFreq() shr APBAHBPrescTable[(RCC.CFGR and RCC_CFGR_PPRE1) shr BsfDWord(RCC_CFGR_PPRE1)]);
end;

function HAL_RCC_GetPCLK2Freq: longword;
begin
  (* Get HCLK source and Compute PCLK2 frequency ---------------------------*)
  exit(HAL_RCC_GetHCLKFreq() shr APBAHBPrescTable[(RCC.CFGR and RCC_CFGR_PPRE2) shr BsfDWord(RCC_CFGR_PPRE2)]);
end;

procedure HAL_RCC_GetOscConfig(var RCC_OscInitStruct: RCC_OscInitTypeDef);
begin
  (* Set all possible values for the Oscillator type parameter ---------------*)
  RCC_OscInitStruct.OscillatorType :=
    RCC_OSCILLATORTYPE_HSE or RCC_OSCILLATORTYPE_HSI or RCC_OSCILLATORTYPE_LSE or RCC_OSCILLATORTYPE_LSI;

  (* Get the HSE configuration -----------------------------------------------*)
  if ((RCC.CR and RCC_CR_HSEBYP) = RCC_CR_HSEBYP) then
  begin
    RCC_OscInitStruct.HSEState := RCC_HSE_BYPASS;
  end
  else if ((RCC.CR and RCC_CR_HSEON) = RCC_CR_HSEON) then
  begin
    RCC_OscInitStruct.HSEState := RCC_HSE_ON;
  end
  else
  begin
    RCC_OscInitStruct.HSEState := RCC_HSE_OFF;
  end;

  (* Get the HSI configuration -----------------------------------------------*)
  if ((RCC.CR and RCC_CR_HSION) = RCC_CR_HSION) then
  begin
    RCC_OscInitStruct.HSIState := RCC_HSI_ON;
  end
  else
  begin
    RCC_OscInitStruct.HSIState := RCC_HSI_OFF;
  end;

  RCC_OscInitStruct.HSICalibrationValue :=
    ((RCC.CR and RCC_CR_HSITRIM) shr BsfDWord(RCC_CR_HSITRIM));

  (* Get the LSE configuration -----------------------------------------------*)
  if ((RCC.BDCR and RCC_BDCR_LSEBYP) = RCC_BDCR_LSEBYP) then
  begin
    RCC_OscInitStruct.LSEState := RCC_LSE_BYPASS;
  end
  else if ((RCC.BDCR and RCC_BDCR_LSEON) = RCC_BDCR_LSEON) then
  begin
    RCC_OscInitStruct.LSEState := RCC_LSE_ON;
  end
  else
  begin
    RCC_OscInitStruct.LSEState := RCC_LSE_OFF;
  end;

  (* Get the LSI configuration -----------------------------------------------*)
  if ((RCC.CSR and RCC_CSR_LSION) = RCC_CSR_LSION) then
  begin
    RCC_OscInitStruct.LSIState := RCC_LSI_ON;
  end
  else
  begin
    RCC_OscInitStruct.LSIState := RCC_LSI_OFF;
  end;

  (* Get the PLL configuration -----------------------------------------------*)
  if ((RCC.CR and RCC_CR_PLLON) = RCC_CR_PLLON) then
  begin
    RCC_OscInitStruct.PLL.PLLState := RCC_PLL_ON;
  end
  else
  begin
    RCC_OscInitStruct.PLL.PLLState := RCC_PLL_OFF;
  end;
  RCC_OscInitStruct.PLL.PLLSource := (RCC.PLLCFGR and RCC_PLLCFGR_PLLSRC);
  RCC_OscInitStruct.PLL.PLLM := (RCC.PLLCFGR and RCC_PLLCFGR_PLLM);
  RCC_OscInitStruct.PLL.PLLN :=
    ((RCC.PLLCFGR and RCC_PLLCFGR_PLLN) shr BsfDWord(RCC_PLLCFGR_PLLN));
  RCC_OscInitStruct.PLL.PLLP :=
    ((((RCC.PLLCFGR and RCC_PLLCFGR_PLLP) + RCC_PLLCFGR_PLLP_0) shl 1) shr BsfDWord(RCC_PLLCFGR_PLLP));
  RCC_OscInitStruct.PLL.PLLQ :=
    ((RCC.PLLCFGR and RCC_PLLCFGR_PLLQ) shr BsfDWord(RCC_PLLCFGR_PLLQ));
end;

procedure HAL_RCC_GetClockConfig(var RCC_ClkInitStruct: RCC_ClkInitTypeDef; var pFLatency: longword);
begin
  (* Set all possible values for the Clock type parameter --------------------*)
  RCC_ClkInitStruct.ClockType :=
    RCC_CLOCKTYPE_SYSCLK or RCC_CLOCKTYPE_HCLK or RCC_CLOCKTYPE_PCLK1 or RCC_CLOCKTYPE_PCLK2;

  (* Get the SYSCLK configuration --------------------------------------------*)
  RCC_ClkInitStruct.SYSCLKSource := (RCC.CFGR and RCC_CFGR_SW);

  (* Get the HCLK configuration ----------------------------------------------*)
  RCC_ClkInitStruct.AHBCLKDivider := (RCC.CFGR and RCC_CFGR_HPRE);

  (* Get the APB1 configuration ----------------------------------------------*)
  RCC_ClkInitStruct.APB1CLKDivider := (RCC.CFGR and RCC_CFGR_PPRE1);

  (* Get the APB2 configuration ----------------------------------------------*)
  RCC_ClkInitStruct.APB2CLKDivider := ((RCC.CFGR and RCC_CFGR_PPRE2) shr 3);

  (* Get the Flash Wait State (Latency) configuration ------------------------*)
  pFLatency := (FLASH.ACR and FLASH_ACR_LATENCY);
end;


end.
