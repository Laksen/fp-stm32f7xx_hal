(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_rcc_ex.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Extension RCC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities RCC extension peripheral:
  *           + Extended Peripheral Control functions
  *
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

unit stm32f7xx_hal_rcc_ex;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal;

type
  RCC_PLLI2SInitTypeDef = record
    PLLI2SN: longword;  (*!< Specifies the multiplication factor for PLLI2S VCO output clock.
                            This parameter must be a number between Min_Data = 49 and Max_Data = 432.
                            This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI  *)
    PLLI2SR: longword;  (*!< Specifies the division factor for I2S clock.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 7.
                            This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI  *)
    PLLI2SQ: longword;  (*!< Specifies the division factor for SAI1 clock.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15.
                            This parameter will be used only when PLLI2S is selected as Clock Source SAI  *)
    PLLI2SP: longword;  (*!< Specifies the division factor for SPDIF-RX clock.
                            This parameter must be a number between 0 and 3 for respective values 2, 4, 6 and 8.
                            This parameter will be used only when PLLI2S is selected as Clock Source SPDDIF-RX  *)
  end;

  (**
  * @brief  PLLSAI Clock structure definition
   *)

  RCC_PLLSAIInitTypeDef = record
    PLLSAIN: longword;  (*!< Specifies the multiplication factor for PLLI2S VCO output clock.
                            This parameter must be a number between Min_Data = 49 and Max_Data = 432.
                            This parameter will be used only when PLLSAI is selected as Clock Source SAI or LTDC  *)
    PLLSAIQ: longword;  (*!< Specifies the division factor for SAI1 clock.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15.
                            This parameter will be used only when PLLSAI is selected as Clock Source SAI or LTDC  *)
    PLLSAIR: longword;  (*!< specifies the division factor for LTDC clock
                            This parameter must be a number between Min_Data = 2 and Max_Data = 7.
                            This parameter will be used only when PLLSAI is selected as Clock Source LTDC  *)
    PLLSAIP: longword;  (*!< Specifies the division factor for 48MHz clock.
                            This parameter can be a value of @ref RCCEx_PLLSAIP_Clock_Divider
                            This parameter will be used only when PLLSAI is disabled  *)
  end;

  (**
  * @brief  RCC extended clocks structure definition
   *)

  RCC_PeriphCLKInitTypeDef = record
    PeriphClockSelection: longword;  (*!< The Extended Clock to be configured.
                                      This parameter can be a value of @ref RCCEx_Periph_Clock_Selection  *)
    PLLI2S: RCC_PLLI2SInitTypeDef;  (*!< PLL I2S structure parameters.
                                      This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI  *)
    PLLSAI: RCC_PLLSAIInitTypeDef;  (*!< PLL SAI structure parameters.
                                      This parameter will be used only when PLLI2S is selected as Clock Source SAI or LTDC  *)
    PLLI2SDivQ: longword;  (*!< Specifies the PLLI2S division factor for SAI1 clock.
                                      This parameter must be a number between Min_Data = 1 and Max_Data = 32
                                      This parameter will be used only when PLLI2S is selected as Clock Source SAI  *)
    PLLSAIDivQ: longword;  (*!< Specifies the PLLI2S division factor for SAI1 clock.
                                      This parameter must be a number between Min_Data = 1 and Max_Data = 32
                                      This parameter will be used only when PLLSAI is selected as Clock Source SAI  *)
    PLLSAIDivR: longword;  (*!< Specifies the PLLSAI division factor for LTDC clock.
                                      This parameter must be one value of @ref RCCEx_PLLSAI_DIVR  *)
    RTCClockSelection: longword;  (*!< Specifies RTC Clock source Selection.
                                        This parameter can be a value of @ref RCC_RTC_Clock_Source  *)
    I2sClockSelection: longword;  (*!< Specifies I2S Clock source Selection.
                                        This parameter can be a value of @ref RCCEx_I2S_Clock_Source  *)
    TIMPresSelection: longword;  (*!< Specifies TIM Clock Prescalers Selection.
                                       This parameter can be a value of @ref RCCEx_TIM_Prescaler_Selection  *)
    Sai1ClockSelection: longword;  (*!< Specifies SAI1 Clock Prescalers Selection
                                        This parameter can be a value of @ref RCCEx_SAI1_Clock_Source  *)
    Sai2ClockSelection: longword;  (*!< Specifies SAI2 Clock Prescalers Selection
                                        This parameter can be a value of @ref RCCEx_SAI2_Clock_Source  *)
    Usart1ClockSelection: longword;  (*!< USART1 clock source
                                      This parameter can be a value of @ref RCCEx_USART1_Clock_Source  *)
    Usart2ClockSelection: longword;  (*!< USART2 clock source
                                      This parameter can be a value of @ref RCCEx_USART2_Clock_Source  *)
    Usart3ClockSelection: longword;  (*!< USART3 clock source
                                      This parameter can be a value of @ref RCCEx_USART3_Clock_Source  *)
    Uart4ClockSelection: longword;  (*!< UART4 clock source
                                      This parameter can be a value of @ref RCCEx_UART4_Clock_Source  *)
    Uart5ClockSelection: longword;  (*!< UART5 clock source
                                      This parameter can be a value of @ref RCCEx_UART5_Clock_Source  *)
    Usart6ClockSelection: longword;  (*!< USART6 clock source
                                      This parameter can be a value of @ref RCCEx_USART6_Clock_Source  *)
    Uart7ClockSelection: longword;  (*!< UART7 clock source
                                      This parameter can be a value of @ref RCCEx_UART7_Clock_Source  *)
    Uart8ClockSelection: longword;  (*!< UART8 clock source
                                      This parameter can be a value of @ref RCCEx_UART8_Clock_Source  *)
    I2c1ClockSelection: longword;  (*!< I2C1 clock source
                                      This parameter can be a value of @ref RCCEx_I2C1_Clock_Source  *)
    I2c2ClockSelection: longword;  (*!< I2C2 clock source
                                      This parameter can be a value of @ref RCCEx_I2C2_Clock_Source  *)
    I2c3ClockSelection: longword;  (*!< I2C3 clock source
                                      This parameter can be a value of @ref RCCEx_I2C3_Clock_Source  *)
    I2c4ClockSelection: longword;  (*!< I2C4 clock source
                                      This parameter can be a value of @ref RCCEx_I2C4_Clock_Source  *)
    Lptim1ClockSelection: longword;  (*!< Specifies LPTIM1 clock source
                                        This parameter can be a value of @ref RCCEx_LPTIM1_Clock_Source  *)
    CecClockSelection: longword;  (*!< CEC clock source
                                        This parameter can be a value of @ref RCCEx_CEC_Clock_Source  *)
    Clk48ClockSelection: longword;  (*!< Specifies 48Mhz clock source used by USB OTG FS, RNG and SDMMC
                                        This parameter can be a value of @ref RCCEx_CLK48_Clock_Source  *)
    Sdmmc1ClockSelection: longword;  (*!< SDMMC1 clock source
                                        This parameter can be a value of @ref RCCEx_SDMMC1_Clock_Source  *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup RCCEx_Exported_Constants RCCEx Exported Constants
  * @{
   *)

  (** @defgroup RCCEx_Periph_Clock_Selection RCC Periph Clock Selection
  * @{
   *)

const
  RCC_PERIPHCLK_I2S = ($00000001);
  RCC_PERIPHCLK_LTDC = ($00000008);
  RCC_PERIPHCLK_TIM = ($00000010);
  RCC_PERIPHCLK_RTC = ($00000020);
  RCC_PERIPHCLK_USART1 = ($00000040);
  RCC_PERIPHCLK_USART2 = ($00000080);
  RCC_PERIPHCLK_USART3 = ($00000100);
  RCC_PERIPHCLK_UART4 = ($00000200);
  RCC_PERIPHCLK_UART5 = ($00000400);
  RCC_PERIPHCLK_USART6 = ($00000800);
  RCC_PERIPHCLK_UART7 = ($00001000);
  RCC_PERIPHCLK_UART8 = ($00002000);
  RCC_PERIPHCLK_I2C1 = ($00004000);
  RCC_PERIPHCLK_I2C2 = ($00008000);
  RCC_PERIPHCLK_I2C3 = ($00010000);
  RCC_PERIPHCLK_I2C4 = ($00020000);
  RCC_PERIPHCLK_LPTIM1 = ($00040000);
  RCC_PERIPHCLK_SAI1 = ($00080000);
  RCC_PERIPHCLK_SAI2 = ($00100000);
  RCC_PERIPHCLK_CLK48 = ($00200000);
  RCC_PERIPHCLK_CEC = ($00400000);
  RCC_PERIPHCLK_SDMMC1 = ($00800000);
  RCC_PERIPHCLK_SPDIFRX = ($01000000);
  RCC_PERIPHCLK_PLLI2S = ($02000000);
  (**
  * @}
   *)

  (** @defgroup RCCEx_PLLSAIP_Clock_Divider RCCEx PLLSAIP Clock Divider
  * @{
   *)

  RCC_PLLSAIP_DIV2 = ($00000000);
  RCC_PLLSAIP_DIV4 = ($00000001);
  RCC_PLLSAIP_DIV6 = ($00000002);
  RCC_PLLSAIP_DIV8 = ($00000003);
  (**
  * @}
   *)

  (** @defgroup RCCEx_PLLSAI_DIVR RCCEx PLLSAI DIVR
  * @{
   *)

  RCC_PLLSAIDIVR_2 = ($00000000);
  RCC_PLLSAIDIVR_4 = RCC_DCKCFGR1_PLLSAIDIVR_0;
  RCC_PLLSAIDIVR_8 = RCC_DCKCFGR1_PLLSAIDIVR_1;
  RCC_PLLSAIDIVR_16 = RCC_DCKCFGR1_PLLSAIDIVR;
  (**
  * @}
   *)

  (** @defgroup RCCEx_I2S_Clock_Source RCCEx I2S Clock Source
  * @{
   *)

  RCC_I2SCLKSOURCE_PLLI2S = ($00000000);
  RCC_I2SCLKSOURCE_EXT = RCC_CFGR_I2SSRC;
  (**
  * @}
   *)


  (** @defgroup RCCEx_SAI1_Clock_Source RCCEx SAI1 Clock Source
  * @{
   *)

  RCC_SAI1CLKSOURCE_PLLSAI = ($00000000);
  RCC_SAI1CLKSOURCE_PLLI2S = RCC_DCKCFGR1_SAI1SEL_0;
  RCC_SAI1CLKSOURCE_PIN = RCC_DCKCFGR1_SAI1SEL_1;
  (**
  * @}
   *)

  (** @defgroup RCCEx_SAI2_Clock_Source RCCEx SAI2 Clock Source
  * @{
   *)

  RCC_SAI2CLKSOURCE_PLLSAI = ($00000000);
  RCC_SAI2CLKSOURCE_PLLI2S = RCC_DCKCFGR1_SAI2SEL_0;
  RCC_SAI2CLKSOURCE_PIN = RCC_DCKCFGR1_SAI2SEL_1;
  (**
  * @}
   *)

  (** @defgroup RCCEx_SDMMC1_Clock_Source RCCEx SDMMC1 Clock Source
  * @{
   *)

  RCC_SDMMC1CLKSOURCE_CLK48 = ($00000000);
  RCC_SDMMC1CLKSOURCE_SYSCLK = RCC_DCKCFGR2_SDMMC1SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_CEC_Clock_Source RCCEx CEC Clock Source
  * @{
   *)

  RCC_CECCLKSOURCE_LSE = ($00000000);
  RCC_CECCLKSOURCE_HSI = RCC_DCKCFGR2_CECSEL;  (* CEC clock is HSI/488 *)
  (**
  * @}
   *)

  (** @defgroup RCCEx_USART1_Clock_Source RCCEx USART1 Clock Source
  * @{
   *)

  RCC_USART1CLKSOURCE_PCLK2 = ($00000000);
  RCC_USART1CLKSOURCE_SYSCLK = RCC_DCKCFGR2_USART1SEL_0;
  RCC_USART1CLKSOURCE_HSI = RCC_DCKCFGR2_USART1SEL_1;
  RCC_USART1CLKSOURCE_LSE = RCC_DCKCFGR2_USART1SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_USART2_Clock_Source RCCEx USART2 Clock Source
  * @{
   *)

  RCC_USART2CLKSOURCE_PCLK1 = ($00000000);
  RCC_USART2CLKSOURCE_SYSCLK = RCC_DCKCFGR2_USART2SEL_0;
  RCC_USART2CLKSOURCE_HSI = RCC_DCKCFGR2_USART2SEL_1;
  RCC_USART2CLKSOURCE_LSE = RCC_DCKCFGR2_USART2SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_USART3_Clock_Source RCCEx USART3 Clock Source
  * @{
   *)

  RCC_USART3CLKSOURCE_PCLK1 = ($00000000);
  RCC_USART3CLKSOURCE_SYSCLK = RCC_DCKCFGR2_USART3SEL_0;
  RCC_USART3CLKSOURCE_HSI = RCC_DCKCFGR2_USART3SEL_1;
  RCC_USART3CLKSOURCE_LSE = RCC_DCKCFGR2_USART3SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_UART4_Clock_Source RCCEx UART4 Clock Source
  * @{
   *)

  RCC_UART4CLKSOURCE_PCLK1 = ($00000000);
  RCC_UART4CLKSOURCE_SYSCLK = RCC_DCKCFGR2_UART4SEL_0;
  RCC_UART4CLKSOURCE_HSI = RCC_DCKCFGR2_UART4SEL_1;
  RCC_UART4CLKSOURCE_LSE = RCC_DCKCFGR2_UART4SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_UART5_Clock_Source RCCEx UART5 Clock Source
  * @{
   *)

  RCC_UART5CLKSOURCE_PCLK1 = ($00000000);
  RCC_UART5CLKSOURCE_SYSCLK = RCC_DCKCFGR2_UART5SEL_0;
  RCC_UART5CLKSOURCE_HSI = RCC_DCKCFGR2_UART5SEL_1;
  RCC_UART5CLKSOURCE_LSE = RCC_DCKCFGR2_UART5SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_USART6_Clock_Source RCCEx USART6 Clock Source
  * @{
   *)

  RCC_USART6CLKSOURCE_PCLK2 = ($00000000);
  RCC_USART6CLKSOURCE_SYSCLK = RCC_DCKCFGR2_USART6SEL_0;
  RCC_USART6CLKSOURCE_HSI = RCC_DCKCFGR2_USART6SEL_1;
  RCC_USART6CLKSOURCE_LSE = RCC_DCKCFGR2_USART6SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_UART7_Clock_Source RCCEx UART7 Clock Source
  * @{
   *)

  RCC_UART7CLKSOURCE_PCLK1 = ($00000000);
  RCC_UART7CLKSOURCE_SYSCLK = RCC_DCKCFGR2_UART7SEL_0;
  RCC_UART7CLKSOURCE_HSI = RCC_DCKCFGR2_UART7SEL_1;
  RCC_UART7CLKSOURCE_LSE = RCC_DCKCFGR2_UART7SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_UART8_Clock_Source RCCEx UART8 Clock Source
  * @{
   *)

  RCC_UART8CLKSOURCE_PCLK1 = ($00000000);
  RCC_UART8CLKSOURCE_SYSCLK = RCC_DCKCFGR2_UART8SEL_0;
  RCC_UART8CLKSOURCE_HSI = RCC_DCKCFGR2_UART8SEL_1;
  RCC_UART8CLKSOURCE_LSE = RCC_DCKCFGR2_UART8SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_I2C1_Clock_Source RCCEx I2C1 Clock Source
  * @{
   *)

  RCC_I2C1CLKSOURCE_PCLK1 = ($00000000);
  RCC_I2C1CLKSOURCE_SYSCLK = RCC_DCKCFGR2_I2C1SEL_0;
  RCC_I2C1CLKSOURCE_HSI = RCC_DCKCFGR2_I2C1SEL_1;
  (**
  * @}
   *)

  (** @defgroup RCCEx_I2C2_Clock_Source RCCEx I2C2 Clock Source
  * @{
   *)

  RCC_I2C2CLKSOURCE_PCLK1 = ($00000000);
  RCC_I2C2CLKSOURCE_SYSCLK = RCC_DCKCFGR2_I2C2SEL_0;
  RCC_I2C2CLKSOURCE_HSI = RCC_DCKCFGR2_I2C2SEL_1;
  (**
  * @}
   *)

  (** @defgroup RCCEx_I2C3_Clock_Source RCCEx I2C3 Clock Source
  * @{
   *)

  RCC_I2C3CLKSOURCE_PCLK1 = ($00000000);
  RCC_I2C3CLKSOURCE_SYSCLK = RCC_DCKCFGR2_I2C3SEL_0;
  RCC_I2C3CLKSOURCE_HSI = RCC_DCKCFGR2_I2C3SEL_1;
  (**
  * @}
   *)

  (** @defgroup RCCEx_I2C4_Clock_Source RCCEx I2C4 Clock Source
  * @{
   *)

  RCC_I2C4CLKSOURCE_PCLK1 = ($00000000);
  RCC_I2C4CLKSOURCE_SYSCLK = RCC_DCKCFGR2_I2C4SEL_0;
  RCC_I2C4CLKSOURCE_HSI = RCC_DCKCFGR2_I2C4SEL_1;
  (**
  * @}
   *)


  (** @defgroup RCCEx_LPTIM1_Clock_Source RCCEx LPTIM1 Clock Source
  * @{
   *)

  RCC_LPTIM1CLKSOURCE_PCLK = ($00000000);
  RCC_LPTIM1CLKSOURCE_LSI = RCC_DCKCFGR2_LPTIM1SEL_0;
  RCC_LPTIM1CLKSOURCE_HSI = RCC_DCKCFGR2_LPTIM1SEL_1;
  RCC_LPTIM1CLKSOURCE_LSE = RCC_DCKCFGR2_LPTIM1SEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_CLK48_Clock_Source RCCEx CLK48 Clock Source
  * @{
   *)

  RCC_CLK48SOURCE_PLL = ($00000000);
  RCC_CLK48SOURCE_PLLSAIP = RCC_DCKCFGR2_CK48MSEL;
  (**
  * @}
   *)

  (** @defgroup RCCEx_TIM_Prescaler_Selection RCCEx TIM Prescaler Selection
  * @{
   *)

  RCC_TIMPRES_DESACTIVATED = ($00000000);
  RCC_TIMPRES_ACTIVATED = RCC_DCKCFGR1_TIMPRE;

procedure __HAL_RCC_BKPSRAM_CLK_ENABLE;
procedure __HAL_RCC_DTCMRAMEN_CLK_ENABLE;
procedure __HAL_RCC_DMA2_CLK_ENABLE;
procedure __HAL_RCC_DMA2D_CLK_ENABLE;
procedure __HAL_RCC_USB_OTG_HS_CLK_ENABLE;
procedure __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE;
procedure __HAL_RCC_GPIOA_CLK_ENABLE;
procedure __HAL_RCC_GPIOB_CLK_ENABLE;
procedure __HAL_RCC_GPIOC_CLK_ENABLE;
procedure __HAL_RCC_GPIOD_CLK_ENABLE;
procedure __HAL_RCC_GPIOE_CLK_ENABLE;
procedure __HAL_RCC_GPIOF_CLK_ENABLE;
procedure __HAL_RCC_GPIOG_CLK_ENABLE;
procedure __HAL_RCC_GPIOH_CLK_ENABLE;
procedure __HAL_RCC_GPIOI_CLK_ENABLE;
procedure __HAL_RCC_GPIOJ_CLK_ENABLE;
procedure __HAL_RCC_GPIOK_CLK_ENABLE;
procedure __HAL_RCC_BKPSRAM_CLK_DISABLE;
procedure __HAL_RCC_DTCMRAMEN_CLK_DISABLE;
procedure __HAL_RCC_DMA2_CLK_DISABLE;
procedure __HAL_RCC_DMA2D_CLK_DISABLE;
procedure __HAL_RCC_USB_OTG_HS_CLK_DISABLE;
procedure __HAL_RCC_USB_OTG_HS_ULPI_CLK_DISABLE;
procedure __HAL_RCC_GPIOA_CLK_DISABLE;
procedure __HAL_RCC_GPIOB_CLK_DISABLE;
procedure __HAL_RCC_GPIOC_CLK_DISABLE;
procedure __HAL_RCC_GPIOD_CLK_DISABLE;
procedure __HAL_RCC_GPIOE_CLK_DISABLE;
procedure __HAL_RCC_GPIOF_CLK_DISABLE;
procedure __HAL_RCC_GPIOG_CLK_DISABLE;
procedure __HAL_RCC_GPIOH_CLK_DISABLE;
procedure __HAL_RCC_GPIOI_CLK_DISABLE;
procedure __HAL_RCC_GPIOJ_CLK_DISABLE;
procedure __HAL_RCC_GPIOK_CLK_DISABLE;
procedure __HAL_RCC_ETHMAC_CLK_ENABLE;
procedure __HAL_RCC_ETHMACTX_CLK_ENABLE;
procedure __HAL_RCC_ETHMACRX_CLK_ENABLE;
procedure __HAL_RCC_ETHMACPTP_CLK_ENABLE;
procedure __HAL_RCC_ETH_CLK_ENABLE;
procedure __HAL_RCC_ETHMAC_CLK_DISABLE;
procedure __HAL_RCC_ETHMACTX_CLK_DISABLE;
procedure __HAL_RCC_ETHMACRX_CLK_DISABLE;
procedure __HAL_RCC_ETHMACPTP_CLK_DISABLE;
procedure __HAL_RCC_ETH_CLK_DISABLE;
procedure __HAL_RCC_DCMI_CLK_ENABLE;
procedure __HAL_RCC_RNG_CLK_ENABLE;
procedure __HAL_RCC_USB_OTG_FS_CLK_ENABLE;
procedure __HAL_RCC_DCMI_CLK_DISABLE;
procedure __HAL_RCC_RNG_CLK_DISABLE;
procedure __HAL_RCC_USB_OTG_FS_CLK_DISABLE;
{$if defined(STM32F756xx)}
procedure __HAL_RCC_CRYP_CLK_ENABLE;
procedure __HAL_RCC_HASH_CLK_ENABLE;
procedure __HAL_RCC_CRYP_CLK_DISABLE;
procedure __HAL_RCC_HASH_CLK_DISABLE;
{$endif}
procedure __HAL_RCC_FMC_CLK_ENABLE;
procedure __HAL_RCC_QSPI_CLK_ENABLE;
procedure __HAL_RCC_FMC_CLK_DISABLE;
procedure __HAL_RCC_QSPI_CLK_DISABLE;
procedure __HAL_RCC_TIM2_CLK_ENABLE;
procedure __HAL_RCC_TIM3_CLK_ENABLE;
procedure __HAL_RCC_TIM4_CLK_ENABLE;
procedure __HAL_RCC_TIM5_CLK_ENABLE;
procedure __HAL_RCC_TIM6_CLK_ENABLE;
procedure __HAL_RCC_TIM7_CLK_ENABLE;
procedure __HAL_RCC_TIM12_CLK_ENABLE;
procedure __HAL_RCC_TIM13_CLK_ENABLE;
procedure __HAL_RCC_TIM14_CLK_ENABLE;
procedure __HAL_RCC_LPTIM1_CLK_ENABLE;
procedure __HAL_RCC_SPI2_CLK_ENABLE;
procedure __HAL_RCC_SPI3_CLK_ENABLE;
procedure __HAL_RCC_SPDIFRX_CLK_ENABLE;
procedure __HAL_RCC_USART2_CLK_ENABLE;
procedure __HAL_RCC_USART3_CLK_ENABLE;
procedure __HAL_RCC_UART4_CLK_ENABLE;
procedure __HAL_RCC_UART5_CLK_ENABLE;
procedure __HAL_RCC_I2C1_CLK_ENABLE;
procedure __HAL_RCC_I2C2_CLK_ENABLE;
procedure __HAL_RCC_I2C3_CLK_ENABLE;
procedure __HAL_RCC_I2C4_CLK_ENABLE;
procedure __HAL_RCC_CAN1_CLK_ENABLE;
procedure __HAL_RCC_CAN2_CLK_ENABLE;
procedure __HAL_RCC_CEC_CLK_ENABLE;
procedure __HAL_RCC_DAC_CLK_ENABLE;
procedure __HAL_RCC_UART7_CLK_ENABLE;
procedure __HAL_RCC_UART8_CLK_ENABLE;
procedure __HAL_RCC_TIM2_CLK_DISABLE;
procedure __HAL_RCC_TIM3_CLK_DISABLE;
procedure __HAL_RCC_TIM4_CLK_DISABLE;
procedure __HAL_RCC_TIM5_CLK_DISABLE;
procedure __HAL_RCC_TIM6_CLK_DISABLE;
procedure __HAL_RCC_TIM7_CLK_DISABLE;
procedure __HAL_RCC_TIM12_CLK_DISABLE;
procedure __HAL_RCC_TIM13_CLK_DISABLE;
procedure __HAL_RCC_TIM14_CLK_DISABLE;
procedure __HAL_RCC_LPTIM1_CLK_DISABLE;
procedure __HAL_RCC_SPI2_CLK_DISABLE;
procedure __HAL_RCC_SPI3_CLK_DISABLE;
procedure __HAL_RCC_SPDIFRX_CLK_DISABLE;
procedure __HAL_RCC_USART2_CLK_DISABLE;
procedure __HAL_RCC_USART3_CLK_DISABLE;
procedure __HAL_RCC_UART4_CLK_DISABLE;
procedure __HAL_RCC_UART5_CLK_DISABLE;
procedure __HAL_RCC_I2C1_CLK_DISABLE;
procedure __HAL_RCC_I2C2_CLK_DISABLE;
procedure __HAL_RCC_I2C3_CLK_DISABLE;
procedure __HAL_RCC_I2C4_CLK_DISABLE;
procedure __HAL_RCC_CAN1_CLK_DISABLE;
procedure __HAL_RCC_CAN2_CLK_DISABLE;
procedure __HAL_RCC_CEC_CLK_DISABLE;
procedure __HAL_RCC_DAC_CLK_DISABLE;
procedure __HAL_RCC_UART7_CLK_DISABLE;
procedure __HAL_RCC_UART8_CLK_DISABLE;
procedure __HAL_RCC_TIM1_CLK_ENABLE;
procedure __HAL_RCC_TIM8_CLK_ENABLE;
procedure __HAL_RCC_USART1_CLK_ENABLE;
procedure __HAL_RCC_USART6_CLK_ENABLE;
procedure __HAL_RCC_ADC1_CLK_ENABLE;
procedure __HAL_RCC_ADC2_CLK_ENABLE;
procedure __HAL_RCC_ADC3_CLK_ENABLE;
procedure __HAL_RCC_SDMMC1_CLK_ENABLE;
procedure __HAL_RCC_SPI1_CLK_ENABLE;
procedure __HAL_RCC_SPI4_CLK_ENABLE;
procedure __HAL_RCC_TIM9_CLK_ENABLE;
procedure __HAL_RCC_TIM10_CLK_ENABLE;
procedure __HAL_RCC_TIM11_CLK_ENABLE;
procedure __HAL_RCC_SPI5_CLK_ENABLE;
procedure __HAL_RCC_SPI6_CLK_ENABLE;
procedure __HAL_RCC_SAI1_CLK_ENABLE;
procedure __HAL_RCC_SAI2_CLK_ENABLE;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_CLK_ENABLE;
{$endif}
procedure __HAL_RCC_TIM1_CLK_DISABLE;
procedure __HAL_RCC_TIM8_CLK_DISABLE;
procedure __HAL_RCC_USART1_CLK_DISABLE;
procedure __HAL_RCC_USART6_CLK_DISABLE;
procedure __HAL_RCC_ADC1_CLK_DISABLE;
procedure __HAL_RCC_ADC2_CLK_DISABLE;
procedure __HAL_RCC_ADC3_CLK_DISABLE;
procedure __HAL_RCC_SDMMC1_CLK_DISABLE;
procedure __HAL_RCC_SPI1_CLK_DISABLE;
procedure __HAL_RCC_SPI4_CLK_DISABLE;
procedure __HAL_RCC_TIM9_CLK_DISABLE;
procedure __HAL_RCC_TIM10_CLK_DISABLE;
procedure __HAL_RCC_TIM11_CLK_DISABLE;
procedure __HAL_RCC_SPI5_CLK_DISABLE;
procedure __HAL_RCC_SPI6_CLK_DISABLE;
procedure __HAL_RCC_SAI1_CLK_DISABLE;
procedure __HAL_RCC_SAI2_CLK_DISABLE;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_CLK_DISABLE;
{$endif}
function __HAL_RCC_BKPSRAM_IS_CLK_ENABLED: boolean;
function __HAL_RCC_DTCMRAMEN_IS_CLK_ENABLED: boolean;
function __HAL_RCC_DMA2_IS_CLK_ENABLED: boolean;
function __HAL_RCC_DMA2D_IS_CLK_ENABLED: boolean;
function __HAL_RCC_USB_OTG_HS_IS_CLK_ENABLED: boolean;
function __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOA_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOB_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOC_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOD_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOE_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOF_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOG_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOH_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOI_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOJ_IS_CLK_ENABLED: boolean;
function __HAL_RCC_GPIOK_IS_CLK_ENABLED: boolean;
function __HAL_RCC_BKPSRAM_IS_CLK_DISABLED: boolean;
function __HAL_RCC_DTCMRAMEN_IS_CLK_DISABLED: boolean;
function __HAL_RCC_DMA2_IS_CLK_DISABLED: boolean;
function __HAL_RCC_DMA2D_IS_CLK_DISABLED: boolean;
function __HAL_RCC_USB_OTG_HS_IS_CLK_DISABLED: boolean;
function __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOA_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOB_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOC_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOD_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOE_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOF_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOG_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOH_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOI_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOJ_IS_CLK_DISABLED: boolean;
function __HAL_RCC_GPIOK_IS_CLK_DISABLED: boolean;
function __HAL_RCC_ETHMAC_IS_CLK_ENABLED: boolean;
function __HAL_RCC_ETHMACTX_IS_CLK_ENABLED: boolean;
function __HAL_RCC_ETHMACRX_IS_CLK_ENABLED: boolean;
function __HAL_RCC_ETHMACPTP_IS_CLK_ENABLED: boolean;
function __HAL_RCC_ETH_IS_CLK_ENABLED: boolean;
function __HAL_RCC_ETHMAC_IS_CLK_DISABLED: boolean;
function __HAL_RCC_ETHMACTX_IS_CLK_DISABLED: boolean;
function __HAL_RCC_ETHMACRX_IS_CLK_DISABLED: boolean;
function __HAL_RCC_ETHMACPTP_IS_CLK_DISABLED: boolean;
function __HAL_RCC_ETH_IS_CLK_DISABLED: boolean;
function __HAL_RCC_DCMI_IS_CLK_ENABLED: boolean;
function __HAL_RCC_RNG_IS_CLK_ENABLED: boolean;
function __HAL_RCC_USB_OTG_FS_IS_CLK_ENABLED: boolean;
function __HAL_RCC_DCMI_IS_CLK_DISABLED: boolean;
function __HAL_RCC_RNG_IS_CLK_DISABLED: boolean;
function __HAL_RCC_USB_IS_OTG_FS_CLK_DISABLED: boolean;
{$if defined(STM32F756xx)}
function __HAL_RCC_CRYP_IS_CLK_ENABLED: boolean;
function __HAL_RCC_HASH_IS_CLK_ENABLED: boolean;
function __HAL_RCC_CRYP_IS_CLK_DISABLED: boolean;
function __HAL_RCC_HASH_IS_CLK_DISABLED: boolean;
{$endif}
function __HAL_RCC_FMC_IS_CLK_ENABLED: boolean;
function __HAL_RCC_QSPI_IS_CLK_ENABLED: boolean;
function __HAL_RCC_FMC_IS_CLK_DISABLED: boolean;
function __HAL_RCC_QSPI_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM2_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM3_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM4_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM5_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM6_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM7_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM12_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM13_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM14_IS_CLK_ENABLED: boolean;
function __HAL_RCC_LPTIM1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SPI2_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SPI3_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SPDIFRX_IS_CLK_ENABLED: boolean;
function __HAL_RCC_USART2_IS_CLK_ENABLED: boolean;
function __HAL_RCC_USART3_IS_CLK_ENABLED: boolean;
function __HAL_RCC_UART4_IS_CLK_ENABLED: boolean;
function __HAL_RCC_UART5_IS_CLK_ENABLED: boolean;
function __HAL_RCC_I2C1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_I2C2_IS_CLK_ENABLED: boolean;
function __HAL_RCC_I2C3_IS_CLK_ENABLED: boolean;
function __HAL_RCC_I2C4_IS_CLK_ENABLED: boolean;
function __HAL_RCC_CAN1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_CAN2_IS_CLK_ENABLED: boolean;
function __HAL_RCC_CEC_IS_CLK_ENABLED: boolean;
function __HAL_RCC_DAC_IS_CLK_ENABLED: boolean;
function __HAL_RCC_UART7_IS_CLK_ENABLED: boolean;
function __HAL_RCC_UART8_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM2_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM3_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM4_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM5_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM6_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM7_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM12_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM13_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM14_IS_CLK_DISABLED: boolean;
function __HAL_RCC_LPTIM1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SPI2_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SPI3_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SPDIFRX_IS_CLK_DISABLED: boolean;
function __HAL_RCC_USART2_IS_CLK_DISABLED: boolean;
function __HAL_RCC_USART3_IS_CLK_DISABLED: boolean;
function __HAL_RCC_UART4_IS_CLK_DISABLED: boolean;
function __HAL_RCC_UART5_IS_CLK_DISABLED: boolean;
function __HAL_RCC_I2C1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_I2C2_IS_CLK_DISABLED: boolean;
function __HAL_RCC_I2C3_IS_CLK_DISABLED: boolean;
function __HAL_RCC_I2C4_IS_CLK_DISABLED: boolean;
function __HAL_RCC_CAN1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_CAN2_IS_CLK_DISABLED: boolean;
function __HAL_RCC_CEC_IS_CLK_DISABLED: boolean;
function __HAL_RCC_DAC_IS_CLK_DISABLED: boolean;
function __HAL_RCC_UART7_IS_CLK_DISABLED: boolean;
function __HAL_RCC_UART8_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM8_IS_CLK_ENABLED: boolean;
function __HAL_RCC_USART1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_USART6_IS_CLK_ENABLED: boolean;
function __HAL_RCC_ADC1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_ADC2_IS_CLK_ENABLED: boolean;
function __HAL_RCC_ADC3_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SDMMC1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SPI1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SPI4_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM9_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM10_IS_CLK_ENABLED: boolean;
function __HAL_RCC_TIM11_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SPI5_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SPI6_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SAI1_IS_CLK_ENABLED: boolean;
function __HAL_RCC_SAI2_IS_CLK_ENABLED: boolean;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
function __HAL_RCC_LTDC_IS_CLK_ENABLED: boolean;
{$endif}
function __HAL_RCC_TIM1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM8_IS_CLK_DISABLED: boolean;
function __HAL_RCC_USART1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_USART6_IS_CLK_DISABLED: boolean;
function __HAL_RCC_ADC1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_ADC2_IS_CLK_DISABLED: boolean;
function __HAL_RCC_ADC3_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SDMMC1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SPI1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SPI4_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM9_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM10_IS_CLK_DISABLED: boolean;
function __HAL_RCC_TIM11_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SPI5_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SPI6_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SAI1_IS_CLK_DISABLED: boolean;
function __HAL_RCC_SAI2_IS_CLK_DISABLED: boolean;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
function __HAL_RCC_LTDC_IS_CLK_DISABLED: boolean;
{$endif}
procedure __HAL_RCC_DMA2_FORCE_RESET;
procedure __HAL_RCC_DMA2D_FORCE_RESET;
procedure __HAL_RCC_ETHMAC_FORCE_RESET;
procedure __HAL_RCC_USB_OTG_HS_FORCE_RESET;
procedure __HAL_RCC_GPIOA_FORCE_RESET;
procedure __HAL_RCC_GPIOB_FORCE_RESET;
procedure __HAL_RCC_GPIOC_FORCE_RESET;
procedure __HAL_RCC_GPIOD_FORCE_RESET;
procedure __HAL_RCC_GPIOE_FORCE_RESET;
procedure __HAL_RCC_GPIOF_FORCE_RESET;
procedure __HAL_RCC_GPIOG_FORCE_RESET;
procedure __HAL_RCC_GPIOH_FORCE_RESET;
procedure __HAL_RCC_GPIOI_FORCE_RESET;
procedure __HAL_RCC_GPIOJ_FORCE_RESET;
procedure __HAL_RCC_GPIOK_FORCE_RESET;
procedure __HAL_RCC_DMA2_RELEASE_RESET;
procedure __HAL_RCC_DMA2D_RELEASE_RESET;
procedure __HAL_RCC_ETHMAC_RELEASE_RESET;
procedure __HAL_RCC_USB_OTG_HS_RELEASE_RESET;
procedure __HAL_RCC_GPIOA_RELEASE_RESET;
procedure __HAL_RCC_GPIOB_RELEASE_RESET;
procedure __HAL_RCC_GPIOC_RELEASE_RESET;
procedure __HAL_RCC_GPIOD_RELEASE_RESET;
procedure __HAL_RCC_GPIOE_RELEASE_RESET;
procedure __HAL_RCC_GPIOF_RELEASE_RESET;
procedure __HAL_RCC_GPIOG_RELEASE_RESET;
procedure __HAL_RCC_GPIOH_RELEASE_RESET;
procedure __HAL_RCC_GPIOI_RELEASE_RESET;
procedure __HAL_RCC_GPIOJ_RELEASE_RESET;
procedure __HAL_RCC_GPIOK_RELEASE_RESET;
procedure __HAL_RCC_AHB2_FORCE_RESET;
procedure __HAL_RCC_DCMI_FORCE_RESET;
procedure __HAL_RCC_RNG_FORCE_RESET;
procedure __HAL_RCC_USB_OTG_FS_FORCE_RESET;
procedure __HAL_RCC_AHB2_RELEASE_RESET;
procedure __HAL_RCC_DCMI_RELEASE_RESET;
procedure __HAL_RCC_RNG_RELEASE_RESET;
procedure __HAL_RCC_USB_OTG_FS_RELEASE_RESET;
{$if defined(STM32F756xx)}
procedure __HAL_RCC_CRYP_FORCE_RESET;
procedure __HAL_RCC_HASH_FORCE_RESET;
procedure __HAL_RCC_CRYP_RELEASE_RESET;
procedure __HAL_RCC_HASH_RELEASE_RESET;
{$endif}
procedure __HAL_RCC_AHB3_FORCE_RESET;
procedure __HAL_RCC_FMC_FORCE_RESET;
procedure __HAL_RCC_QSPI_FORCE_RESET;
procedure __HAL_RCC_AHB3_RELEASE_RESET;
procedure __HAL_RCC_FMC_RELEASE_RESET;
procedure __HAL_RCC_QSPI_RELEASE_RESET;
procedure __HAL_RCC_TIM2_FORCE_RESET;
procedure __HAL_RCC_TIM3_FORCE_RESET;
procedure __HAL_RCC_TIM4_FORCE_RESET;
procedure __HAL_RCC_TIM5_FORCE_RESET;
procedure __HAL_RCC_TIM6_FORCE_RESET;
procedure __HAL_RCC_TIM7_FORCE_RESET;
procedure __HAL_RCC_TIM12_FORCE_RESET;
procedure __HAL_RCC_TIM13_FORCE_RESET;
procedure __HAL_RCC_TIM14_FORCE_RESET;
procedure __HAL_RCC_LPTIM1_FORCE_RESET;
procedure __HAL_RCC_SPI2_FORCE_RESET;
procedure __HAL_RCC_SPI3_FORCE_RESET;
procedure __HAL_RCC_SPDIFRX_FORCE_RESET;
procedure __HAL_RCC_USART2_FORCE_RESET;
procedure __HAL_RCC_USART3_FORCE_RESET;
procedure __HAL_RCC_UART4_FORCE_RESET;
procedure __HAL_RCC_UART5_FORCE_RESET;
procedure __HAL_RCC_I2C1_FORCE_RESET;
procedure __HAL_RCC_I2C2_FORCE_RESET;
procedure __HAL_RCC_I2C3_FORCE_RESET;
procedure __HAL_RCC_I2C4_FORCE_RESET;
procedure __HAL_RCC_CAN1_FORCE_RESET;
procedure __HAL_RCC_CAN2_FORCE_RESET;
procedure __HAL_RCC_CEC_FORCE_RESET;
procedure __HAL_RCC_DAC_FORCE_RESET;
procedure __HAL_RCC_UART7_FORCE_RESET;
procedure __HAL_RCC_UART8_FORCE_RESET;
procedure __HAL_RCC_TIM2_RELEASE_RESET;
procedure __HAL_RCC_TIM3_RELEASE_RESET;
procedure __HAL_RCC_TIM4_RELEASE_RESET;
procedure __HAL_RCC_TIM5_RELEASE_RESET;
procedure __HAL_RCC_TIM6_RELEASE_RESET;
procedure __HAL_RCC_TIM7_RELEASE_RESET;
procedure __HAL_RCC_TIM12_RELEASE_RESET;
procedure __HAL_RCC_TIM13_RELEASE_RESET;
procedure __HAL_RCC_TIM14_RELEASE_RESET;
procedure __HAL_RCC_LPTIM1_RELEASE_RESET;
procedure __HAL_RCC_SPI2_RELEASE_RESET;
procedure __HAL_RCC_SPI3_RELEASE_RESET;
procedure __HAL_RCC_SPDIFRX_RELEASE_RESET;
procedure __HAL_RCC_USART2_RELEASE_RESET;
procedure __HAL_RCC_USART3_RELEASE_RESET;
procedure __HAL_RCC_UART4_RELEASE_RESET;
procedure __HAL_RCC_UART5_RELEASE_RESET;
procedure __HAL_RCC_I2C1_RELEASE_RESET;
procedure __HAL_RCC_I2C2_RELEASE_RESET;
procedure __HAL_RCC_I2C3_RELEASE_RESET;
procedure __HAL_RCC_I2C4_RELEASE_RESET;
procedure __HAL_RCC_CAN1_RELEASE_RESET;
procedure __HAL_RCC_CAN2_RELEASE_RESET;
procedure __HAL_RCC_CEC_RELEASE_RESET;
procedure __HAL_RCC_DAC_RELEASE_RESET;
procedure __HAL_RCC_UART7_RELEASE_RESET;
procedure __HAL_RCC_UART8_RELEASE_RESET;
procedure __HAL_RCC_TIM1_FORCE_RESET;
procedure __HAL_RCC_TIM8_FORCE_RESET;
procedure __HAL_RCC_USART1_FORCE_RESET;
procedure __HAL_RCC_USART6_FORCE_RESET;
procedure __HAL_RCC_ADC_FORCE_RESET;
procedure __HAL_RCC_SDMMC1_FORCE_RESET;
procedure __HAL_RCC_SPI1_FORCE_RESET;
procedure __HAL_RCC_SPI4_FORCE_RESET;
procedure __HAL_RCC_TIM9_FORCE_RESET;
procedure __HAL_RCC_TIM10_FORCE_RESET;
procedure __HAL_RCC_TIM11_FORCE_RESET;
procedure __HAL_RCC_SPI5_FORCE_RESET;
procedure __HAL_RCC_SPI6_FORCE_RESET;
procedure __HAL_RCC_SAI1_FORCE_RESET;
procedure __HAL_RCC_SAI2_FORCE_RESET;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_FORCE_RESET;
{$endif}
procedure __HAL_RCC_TIM1_RELEASE_RESET;
procedure __HAL_RCC_TIM8_RELEASE_RESET;
procedure __HAL_RCC_USART1_RELEASE_RESET;
procedure __HAL_RCC_USART6_RELEASE_RESET;
procedure __HAL_RCC_ADC_RELEASE_RESET;
procedure __HAL_RCC_SDMMC1_RELEASE_RESET;
procedure __HAL_RCC_SPI1_RELEASE_RESET;
procedure __HAL_RCC_SPI4_RELEASE_RESET;
procedure __HAL_RCC_TIM9_RELEASE_RESET;
procedure __HAL_RCC_TIM10_RELEASE_RESET;
procedure __HAL_RCC_TIM11_RELEASE_RESET;
procedure __HAL_RCC_SPI5_RELEASE_RESET;
procedure __HAL_RCC_SPI6_RELEASE_RESET;
procedure __HAL_RCC_SAI1_RELEASE_RESET;
procedure __HAL_RCC_SAI2_RELEASE_RESET;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_RELEASE_RESET;
{$endif}
procedure __HAL_RCC_FLITF_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_AXI_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SRAM1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SRAM2_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_BKPSRAM_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_DTCM_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_DMA2_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_DMA2D_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_ETHMAC_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_ETHMACTX_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_ETHMACRX_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_ETHMACPTP_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_USB_OTG_HS_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_USB_OTG_HS_ULPI_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOA_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOB_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOC_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOD_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOE_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOF_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOG_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOH_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOI_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOJ_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_GPIOK_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_FLITF_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_AXI_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SRAM1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SRAM2_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_BKPSRAM_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_DTCM_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_DMA2_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_DMA2D_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_ETHMAC_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_ETHMACTX_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_ETHMACRX_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_ETHMACPTP_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_USB_OTG_HS_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_USB_OTG_HS_ULPI_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOA_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOB_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOC_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOD_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOE_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOF_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOG_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOH_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOI_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOJ_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_GPIOK_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_DCMI_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_DCMI_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_RNG_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_RNG_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_USB_OTG_FS_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_USB_OTG_FS_CLK_SLEEP_DISABLE;
{$if defined(STM32F756xx)}
procedure __HAL_RCC_CRYP_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_HASH_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_CRYP_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_HASH_CLK_SLEEP_DISABLE;
{$endif}
procedure __HAL_RCC_FMC_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_FMC_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_QSPI_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_QSPI_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM2_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM3_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM4_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM5_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM6_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM7_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM12_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM13_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM14_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_LPTIM1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SPI2_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SPI3_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SPDIFRX_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_USART2_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_USART3_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_UART4_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_UART5_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_I2C1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_I2C2_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_I2C3_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_I2C4_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_CAN1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_CAN2_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_CEC_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_DAC_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_UART7_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_UART8_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM2_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM3_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM4_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM5_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM6_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM7_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM12_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM13_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM14_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_LPTIM1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SPI2_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SPI3_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SPDIFRX_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_USART2_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_USART3_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_UART4_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_UART5_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_I2C1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_I2C2_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_I2C3_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_I2C4_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_CAN1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_CAN2_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_CEC_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_DAC_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_UART7_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_UART8_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM8_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_USART1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_USART6_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_ADC1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_ADC2_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_ADC3_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SDMMC1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SPI1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SPI4_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM9_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM10_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_TIM11_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SPI5_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SPI6_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SAI1_CLK_SLEEP_ENABLE;
procedure __HAL_RCC_SAI2_CLK_SLEEP_ENABLE;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_CLK_SLEEP_ENABLE;
{$endif}
procedure __HAL_RCC_TIM1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM8_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_USART1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_USART6_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_ADC1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_ADC2_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_ADC3_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SDMMC1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SPI1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SPI4_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM9_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM10_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_TIM11_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SPI5_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SPI6_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SAI1_CLK_SLEEP_DISABLE;
procedure __HAL_RCC_SAI2_CLK_SLEEP_DISABLE;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_CLK_SLEEP_DISABLE;
{$endif}
function __HAL_RCC_FLITF_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_AXI_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SRAM1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SRAM2_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_BKPSRAM_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_DTCM_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_DMA2_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_DMA2D_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_ETHMAC_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_ETHMACTX_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_ETHMACRX_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_ETHMACPTP_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_USB_OTG_HS_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOA_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOB_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOC_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOD_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOE_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOF_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOG_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOH_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOI_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOJ_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_GPIOK_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_FLITF_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_AXI_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SRAM1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SRAM2_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_BKPSRAM_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_DTCM_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_DMA2_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_DMA2D_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_ETHMAC_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_ETHMACTX_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_ETHMACRX_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_ETHMACPTP_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_USB_OTG_HS_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOA_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOB_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOC_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOD_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOE_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOF_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOG_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOH_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOI_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOJ_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_GPIOK_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_DCMI_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_DCMI_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_RNG_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_RNG_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_USB_OTG_FS_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_USB_OTG_FS_IS_CLK_SLEEP_DISABLED: boolean;
{$if defined(STM32F756xx)}
function __HAL_RCC_CRYP_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_HASH_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_CRYP_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_HASH_IS_CLK_SLEEP_DISABLED: boolean;
{$endif}
function __HAL_RCC_FMC_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_FMC_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_QSPI_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_QSPI_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM2_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM3_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM4_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM5_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM6_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM7_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM12_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM13_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM14_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_LPTIM1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SPI2_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SPI3_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SPDIFRX_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_USART2_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_USART3_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_UART4_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_UART5_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_I2C1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_I2C2_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_I2C3_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_I2C4_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_CAN1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_CAN2_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_CEC_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_DAC_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_UART7_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_UART8_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM2_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM3_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM4_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM5_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM6_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM7_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM12_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM13_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM14_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_LPTIM1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SPI2_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SPI3_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SPDIFRX_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_USART2_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_USART3_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_UART4_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_UART5_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_I2C1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_I2C2_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_I2C3_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_I2C4_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_CAN1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_CAN2_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_CEC_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_DAC_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_UART7_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_UART8_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM8_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_USART1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_USART6_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_ADC1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_ADC2_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_ADC3_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SDMMC1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SPI1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SPI4_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM9_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM10_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_TIM11_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SPI5_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SPI6_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SAI1_IS_CLK_SLEEP_ENABLED: boolean;
function __HAL_RCC_SAI2_IS_CLK_SLEEP_ENABLED: boolean;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
function __HAL_RCC_LTDC_IS_CLK_SLEEP_ENABLED: boolean;
{$endif}
function __HAL_RCC_TIM1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM8_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_USART1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_USART6_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_ADC1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_ADC2_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_ADC3_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SDMMC1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SPI1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SPI4_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM9_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM10_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_TIM11_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SPI5_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SPI6_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SAI1_IS_CLK_SLEEP_DISABLED: boolean;
function __HAL_RCC_SAI2_IS_CLK_SLEEP_DISABLED: boolean;
{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
function __HAL_RCC_LTDC_IS_CLK_SLEEP_DISABLED: boolean;
{$endif}
procedure __HAL_RCC_TIMCLKPRESCALER(__PRESC__: longword);
procedure __HAL_RCC_PLLSAI_ENABLE;
procedure __HAL_RCC_PLLSAI_DISABLE;
procedure __HAL_RCC_PLLSAI_CONFIG(__PLLSAIN__, __PLLSAIP__, __PLLSAIQ__, __PLLSAIR__: longword);
procedure __HAL_RCC_PLLI2S_CONFIG(__PLLI2SN__, __PLLI2SP__, __PLLI2SQ__, __PLLI2SR__: longword);
procedure __HAL_RCC_PLLI2S_PLLSAICLKDIVQ_CONFIG(__PLLI2SDivQ__: longword);
procedure __HAL_RCC_PLLSAI_PLLSAICLKDIVQ_CONFIG(__PLLSAIDivQ__: longword);
procedure __HAL_RCC_PLLSAI_PLLSAICLKDIVR_CONFIG(__PLLSAIDivR__: longword);
procedure __HAL_RCC_SAI1_CONFIG(__SOURCE__: longword);
function __HAL_RCC_GET_SAI1_SOURCE: longword;
procedure __HAL_RCC_SAI2_CONFIG(__SOURCE__: longword);
function __HAL_RCC_GET_SAI2_SOURCE: longword;
procedure __HAL_RCC_PLLSAI_ENABLE_IT;
procedure __HAL_RCC_PLLSAI_DISABLE_IT;
procedure __HAL_RCC_PLLSAI_CLEAR_IT;
function __HAL_RCC_PLLSAI_GET_IT: boolean;
function __HAL_RCC_PLLSAI_GET_FLAG: boolean;
function __HAL_RCC_GET_I2SCLKSOURCE: longword;
procedure __HAL_RCC_I2C1_CONFIG(__I2C1_CLKSOURCE__: longword);
function __HAL_RCC_GET_I2C1_SOURCE: longword;
procedure __HAL_RCC_I2C2_CONFIG(__I2C2_CLKSOURCE__: longword);
function __HAL_RCC_GET_I2C2_SOURCE: longword;
procedure __HAL_RCC_I2C3_CONFIG(__I2C3_CLKSOURCE__: longword);
function __HAL_RCC_GET_I2C3_SOURCE: longword;
procedure __HAL_RCC_I2C4_CONFIG(__I2C4_CLKSOURCE__: longword);
function __HAL_RCC_GET_I2C4_SOURCE: longword;
procedure __HAL_RCC_USART1_CONFIG(__USART1_CLKSOURCE__: longword);
function __HAL_RCC_GET_USART1_SOURCE: longword;
procedure __HAL_RCC_USART2_CONFIG(__USART2_CLKSOURCE__: longword);
function __HAL_RCC_GET_USART2_SOURCE: longword;
procedure __HAL_RCC_USART3_CONFIG(__USART3_CLKSOURCE__: longword);
function __HAL_RCC_GET_USART3_SOURCE: longword;
procedure __HAL_RCC_UART4_CONFIG(__UART4_CLKSOURCE__: longword);
function __HAL_RCC_GET_UART4_SOURCE: longword;
procedure __HAL_RCC_UART5_CONFIG(__UART5_CLKSOURCE__: longword);
function __HAL_RCC_GET_UART5_SOURCE: longword;
procedure __HAL_RCC_USART6_CONFIG(__USART6_CLKSOURCE__: longword);
function __HAL_RCC_GET_USART6_SOURCE: longword;
procedure __HAL_RCC_UART7_CONFIG(__UART7_CLKSOURCE__: longword);
function __HAL_RCC_GET_UART7_SOURCE: longword;
procedure __HAL_RCC_UART8_CONFIG(__UART8_CLKSOURCE__: longword);
function __HAL_RCC_GET_UART8_SOURCE: longword;
procedure __HAL_RCC_LPTIM1_CONFIG(__LPTIM1_CLKSOURCE__: longword);
function __HAL_RCC_GET_LPTIM1_SOURCE: longword;
procedure __HAL_RCC_CEC_CONFIG(__CEC_CLKSOURCE__: longword);
function __HAL_RCC_GET_CEC_SOURCE: longword;
procedure __HAL_RCC_CLK48_CONFIG(__CLK48_SOURCE__: longword);
function __HAL_RCC_GET_CLK48_SOURCE: longword;
procedure __HAL_RCC_SDMMC1_CONFIG(__SDMMC1_CLKSOURCE__: longword);
function __HAL_RCC_GET_SDMMC1_SOURCE: longword;

function HAL_RCCEx_PeriphCLKConfig(const PeriphClkInit: RCC_PeriphCLKInitTypeDef): HAL_StatusTypeDef;
procedure HAL_RCCEx_GetPeriphCLKConfig(var PeriphClkInit: RCC_PeriphCLKInitTypeDef);
function HAL_RCCEx_GetPeriphCLKFreq(PeriphClk: longword): longword;

implementation

uses
  stm32f7xx_hal_conf,
  stm32f7xx_hal_rcc;

const
  PLLI2S_TIMEOUT_VALUE = 100;  (* Timeout value fixed to 100 ms   *)
  PLLSAI_TIMEOUT_VALUE = 100;  (* Timeout value fixed to 100 ms   *)

procedure __HAL_RCC_BKPSRAM_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_BKPSRAMEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_BKPSRAMEN);

end;

procedure __HAL_RCC_DTCMRAMEN_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_DTCMRAMEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_DTCMRAMEN);

end;

procedure __HAL_RCC_DMA2_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_DMA2EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_DMA2EN);

end;

procedure __HAL_RCC_DMA2D_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_DMA2DEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_DMA2DEN);

end;

procedure __HAL_RCC_USB_OTG_HS_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_OTGHSEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_OTGHSEN);

end;

procedure __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_OTGHSULPIEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_OTGHSULPIEN);

end;

procedure __HAL_RCC_GPIOA_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOAEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOAEN);

end;

procedure __HAL_RCC_GPIOB_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOBEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOBEN);

end;

procedure __HAL_RCC_GPIOC_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOCEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOCEN);

end;

procedure __HAL_RCC_GPIOD_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIODEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIODEN);

end;

procedure __HAL_RCC_GPIOE_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOEEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOEEN);

end;

procedure __HAL_RCC_GPIOF_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOFEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOFEN);

end;

procedure __HAL_RCC_GPIOG_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOGEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOGEN);

end;

procedure __HAL_RCC_GPIOH_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOHEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOHEN);

end;

procedure __HAL_RCC_GPIOI_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOIEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOIEN);

end;

procedure __HAL_RCC_GPIOJ_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOJEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOJEN);

end;

procedure __HAL_RCC_GPIOK_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_GPIOKEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_GPIOKEN);

end;

procedure __HAL_RCC_BKPSRAM_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_BKPSRAMEN));
end;

procedure __HAL_RCC_DTCMRAMEN_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_DTCMRAMEN));
end;

procedure __HAL_RCC_DMA2_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_DMA2EN));
end;

procedure __HAL_RCC_DMA2D_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_DMA2DEN));
end;

procedure __HAL_RCC_USB_OTG_HS_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_OTGHSEN));
end;

procedure __HAL_RCC_USB_OTG_HS_ULPI_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_OTGHSULPIEN));
end;

procedure __HAL_RCC_GPIOA_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOAEN));
end;

procedure __HAL_RCC_GPIOB_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOBEN));
end;

procedure __HAL_RCC_GPIOC_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOCEN));
end;

procedure __HAL_RCC_GPIOD_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIODEN));
end;

procedure __HAL_RCC_GPIOE_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOEEN));
end;

procedure __HAL_RCC_GPIOF_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOFEN));
end;

procedure __HAL_RCC_GPIOG_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOGEN));
end;

procedure __HAL_RCC_GPIOH_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOHEN));
end;

procedure __HAL_RCC_GPIOI_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOIEN));
end;

procedure __HAL_RCC_GPIOJ_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOJEN));
end;

procedure __HAL_RCC_GPIOK_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_GPIOKEN));
end;

(**
  * @brief  Enable ETHERNET clock.
  *)
procedure __HAL_RCC_ETHMAC_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_ETHMACEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_ETHMACEN);

end;

procedure __HAL_RCC_ETHMACTX_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_ETHMACTXEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_ETHMACTXEN);

end;

procedure __HAL_RCC_ETHMACRX_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_ETHMACRXEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_ETHMACRXEN);

end;

procedure __HAL_RCC_ETHMACPTP_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB1ENR := RCC.AHB1ENR or longword(RCC_AHB1ENR_ETHMACPTPEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB1ENR and RCC_AHB1ENR_ETHMACPTPEN);

end;

procedure __HAL_RCC_ETH_CLK_ENABLE;
begin

  __HAL_RCC_ETHMAC_CLK_ENABLE();
  __HAL_RCC_ETHMACTX_CLK_ENABLE();
  __HAL_RCC_ETHMACRX_CLK_ENABLE();

end;

(**
  * @brief  Disable ETHERNET clock.
  *)
procedure __HAL_RCC_ETHMAC_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_ETHMACEN));
end;

procedure __HAL_RCC_ETHMACTX_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_ETHMACTXEN));
end;

procedure __HAL_RCC_ETHMACRX_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_ETHMACRXEN));
end;

procedure __HAL_RCC_ETHMACPTP_CLK_DISABLE;
begin
  RCC.AHB1ENR := RCC.AHB1ENR and (not (RCC_AHB1ENR_ETHMACPTPEN));
end;

procedure __HAL_RCC_ETH_CLK_DISABLE;
begin

  __HAL_RCC_ETHMACTX_CLK_DISABLE();
  __HAL_RCC_ETHMACRX_CLK_DISABLE();
  __HAL_RCC_ETHMAC_CLK_DISABLE();

end;

(** @brief  Enable or disable the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
procedure __HAL_RCC_DCMI_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB2ENR := RCC.AHB2ENR or longword(RCC_AHB2ENR_DCMIEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB2ENR and RCC_AHB2ENR_DCMIEN);

end;

procedure __HAL_RCC_RNG_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB2ENR := RCC.AHB2ENR or longword(RCC_AHB2ENR_RNGEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB2ENR and RCC_AHB2ENR_RNGEN);

end;

procedure __HAL_RCC_USB_OTG_FS_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB2ENR := RCC.AHB2ENR or longword(RCC_AHB2ENR_OTGFSEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB2ENR and RCC_AHB2ENR_OTGFSEN);

  __HAL_RCC_SYSCFG_CLK_ENABLE();

end;

procedure __HAL_RCC_DCMI_CLK_DISABLE;
begin
  RCC.AHB2ENR := RCC.AHB2ENR and (not (RCC_AHB2ENR_DCMIEN));
end;

procedure __HAL_RCC_RNG_CLK_DISABLE;
begin
  RCC.AHB2ENR := RCC.AHB2ENR and (not (RCC_AHB2ENR_RNGEN));
end;

procedure __HAL_RCC_USB_OTG_FS_CLK_DISABLE;
begin
  RCC.AHB2ENR := RCC.AHB2ENR and (not (RCC_AHB2ENR_OTGFSEN));
  __HAL_RCC_SYSCFG_CLK_DISABLE();

end;

{$if defined(STM32F756xx)}
procedure __HAL_RCC_CRYP_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB2ENR := RCC.AHB2ENR or longword(RCC_AHB2ENR_CRYPEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB2ENR and RCC_AHB2ENR_CRYPEN);

end;

procedure __HAL_RCC_HASH_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB2ENR := RCC.AHB2ENR or longword(RCC_AHB2ENR_HASHEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB2ENR and RCC_AHB2ENR_HASHEN);

end;

procedure __HAL_RCC_CRYP_CLK_DISABLE;
begin
  RCC.AHB2ENR := RCC.AHB2ENR and (not (RCC_AHB2ENR_CRYPEN));
end;

procedure __HAL_RCC_HASH_CLK_DISABLE;
begin
  RCC.AHB2ENR := RCC.AHB2ENR and (not (RCC_AHB2ENR_HASHEN));
end;

{$endif}
(** @brief  Enables or disables the AHB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
procedure __HAL_RCC_FMC_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB3ENR := RCC.AHB3ENR or longword(RCC_AHB3ENR_FMCEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB3ENR and RCC_AHB3ENR_FMCEN);

end;

procedure __HAL_RCC_QSPI_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.AHB3ENR := RCC.AHB3ENR or longword(RCC_AHB3ENR_QSPIEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.AHB3ENR and RCC_AHB3ENR_QSPIEN);

end;

procedure __HAL_RCC_FMC_CLK_DISABLE;
begin
  RCC.AHB3ENR := RCC.AHB3ENR and (not (RCC_AHB3ENR_FMCEN));
end;

procedure __HAL_RCC_QSPI_CLK_DISABLE;
begin
  RCC.AHB3ENR := RCC.AHB3ENR and (not (RCC_AHB3ENR_QSPIEN));
end;

(** @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
procedure __HAL_RCC_TIM2_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM2EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM2EN);

end;

procedure __HAL_RCC_TIM3_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM3EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM3EN);

end;

procedure __HAL_RCC_TIM4_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM4EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM4EN);

end;

procedure __HAL_RCC_TIM5_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM5EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM5EN);

end;

procedure __HAL_RCC_TIM6_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM6EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM6EN);

end;

procedure __HAL_RCC_TIM7_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM7EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM7EN);

end;

procedure __HAL_RCC_TIM12_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM12EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM12EN);

end;

procedure __HAL_RCC_TIM13_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM13EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM13EN);

end;

procedure __HAL_RCC_TIM14_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_TIM14EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_TIM14EN);

end;

procedure __HAL_RCC_LPTIM1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_LPTIM1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_LPTIM1EN);

end;

procedure __HAL_RCC_SPI2_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_SPI2EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_SPI2EN);

end;

procedure __HAL_RCC_SPI3_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_SPI3EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_SPI3EN);

end;

procedure __HAL_RCC_SPDIFRX_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_SPDIFRXEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_SPDIFRXEN);

end;

procedure __HAL_RCC_USART2_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_USART2EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_USART2EN);

end;

procedure __HAL_RCC_USART3_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_USART3EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_USART3EN);

end;

procedure __HAL_RCC_UART4_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_UART4EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_UART4EN);

end;

procedure __HAL_RCC_UART5_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_UART5EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_UART5EN);

end;

procedure __HAL_RCC_I2C1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_I2C1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_I2C1EN);

end;

procedure __HAL_RCC_I2C2_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_I2C2EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_I2C2EN);

end;

procedure __HAL_RCC_I2C3_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_I2C3EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_I2C3EN);

end;

procedure __HAL_RCC_I2C4_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_I2C4EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_I2C4EN);

end;

procedure __HAL_RCC_CAN1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_CAN1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_CAN1EN);

end;

procedure __HAL_RCC_CAN2_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_CAN2EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_CAN2EN);

end;

procedure __HAL_RCC_CEC_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_CECEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_CECEN);

end;

procedure __HAL_RCC_DAC_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_DACEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_DACEN);

end;

procedure __HAL_RCC_UART7_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_UART7EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_UART7EN);

end;

procedure __HAL_RCC_UART8_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB1ENR := RCC.APB1ENR or longword(RCC_APB1ENR_UART8EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB1ENR and RCC_APB1ENR_UART8EN);

end;

procedure __HAL_RCC_TIM2_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM2EN));
end;

procedure __HAL_RCC_TIM3_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM3EN));
end;

procedure __HAL_RCC_TIM4_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM4EN));
end;

procedure __HAL_RCC_TIM5_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM5EN));
end;

procedure __HAL_RCC_TIM6_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM6EN));
end;

procedure __HAL_RCC_TIM7_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM7EN));
end;

procedure __HAL_RCC_TIM12_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM12EN));
end;

procedure __HAL_RCC_TIM13_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM13EN));
end;

procedure __HAL_RCC_TIM14_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_TIM14EN));
end;

procedure __HAL_RCC_LPTIM1_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_LPTIM1EN));
end;

procedure __HAL_RCC_SPI2_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_SPI2EN));
end;

procedure __HAL_RCC_SPI3_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_SPI3EN));
end;

procedure __HAL_RCC_SPDIFRX_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_SPDIFRXEN));
end;

procedure __HAL_RCC_USART2_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_USART2EN));
end;

procedure __HAL_RCC_USART3_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_USART3EN));
end;

procedure __HAL_RCC_UART4_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_UART4EN));
end;

procedure __HAL_RCC_UART5_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_UART5EN));
end;

procedure __HAL_RCC_I2C1_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_I2C1EN));
end;

procedure __HAL_RCC_I2C2_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_I2C2EN));
end;

procedure __HAL_RCC_I2C3_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_I2C3EN));
end;

procedure __HAL_RCC_I2C4_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_I2C4EN));
end;

procedure __HAL_RCC_CAN1_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_CAN1EN));
end;

procedure __HAL_RCC_CAN2_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_CAN2EN));
end;

procedure __HAL_RCC_CEC_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_CECEN));
end;

procedure __HAL_RCC_DAC_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_DACEN));
end;

procedure __HAL_RCC_UART7_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_UART7EN));
end;

procedure __HAL_RCC_UART8_CLK_DISABLE;
begin
  RCC.APB1ENR := RCC.APB1ENR and (not (RCC_APB1ENR_UART8EN));
end;

(** @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
procedure __HAL_RCC_TIM1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_TIM1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_TIM1EN);

end;

procedure __HAL_RCC_TIM8_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_TIM8EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_TIM8EN);

end;

procedure __HAL_RCC_USART1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_USART1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_USART1EN);

end;

procedure __HAL_RCC_USART6_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_USART6EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_USART6EN);

end;

procedure __HAL_RCC_ADC1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_ADC1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_ADC1EN);

end;

procedure __HAL_RCC_ADC2_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_ADC2EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_ADC2EN);

end;

procedure __HAL_RCC_ADC3_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_ADC3EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_ADC3EN);

end;

procedure __HAL_RCC_SDMMC1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_SDMMC1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_SDMMC1EN);

end;

procedure __HAL_RCC_SPI1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_SPI1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_SPI1EN);

end;

procedure __HAL_RCC_SPI4_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_SPI4EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_SPI4EN);

end;

procedure __HAL_RCC_TIM9_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_TIM9EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_TIM9EN);

end;

procedure __HAL_RCC_TIM10_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_TIM10EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_TIM10EN);

end;

procedure __HAL_RCC_TIM11_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_TIM11EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_TIM11EN);

end;

procedure __HAL_RCC_SPI5_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_SPI5EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_SPI5EN);

end;

procedure __HAL_RCC_SPI6_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_SPI6EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_SPI6EN);

end;

procedure __HAL_RCC_SAI1_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_SAI1EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_SAI1EN);

end;

procedure __HAL_RCC_SAI2_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_SAI2EN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_SAI2EN);

end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_CLK_ENABLE;

var
  tmpreg: longword;
begin

  RCC.APB2ENR := RCC.APB2ENR or longword(RCC_APB2ENR_LTDCEN);
  (* Delay after an RCC peripheral clock enabling *)
  tmpreg := (RCC.APB2ENR and RCC_APB2ENR_LTDCEN);

end;

{$endif}

procedure __HAL_RCC_TIM1_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_TIM1EN));
end;

procedure __HAL_RCC_TIM8_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_TIM8EN));
end;

procedure __HAL_RCC_USART1_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_USART1EN));
end;

procedure __HAL_RCC_USART6_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_USART6EN));
end;

procedure __HAL_RCC_ADC1_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_ADC1EN));
end;

procedure __HAL_RCC_ADC2_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_ADC2EN));
end;

procedure __HAL_RCC_ADC3_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_ADC3EN));
end;

procedure __HAL_RCC_SDMMC1_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_SDMMC1EN));
end;

procedure __HAL_RCC_SPI1_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_SPI1EN));
end;

procedure __HAL_RCC_SPI4_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_SPI4EN));
end;

procedure __HAL_RCC_TIM9_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_TIM9EN));
end;

procedure __HAL_RCC_TIM10_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_TIM10EN));
end;

procedure __HAL_RCC_TIM11_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_TIM11EN));
end;

procedure __HAL_RCC_SPI5_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_SPI5EN));
end;

procedure __HAL_RCC_SPI6_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_SPI6EN));
end;

procedure __HAL_RCC_SAI1_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_SAI1EN));
end;

procedure __HAL_RCC_SAI2_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_SAI2EN));
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_CLK_DISABLE;
begin
  RCC.APB2ENR := RCC.APB2ENR and (not (RCC_APB2ENR_LTDCEN));
end;

{$endif}
(**
  * @end
  *)


(** @defgroup RCCEx_Peripheral_Clock_Enable_Disable_Status Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB/APB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @begin
  *)

(** @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
function __HAL_RCC_BKPSRAM_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_BKPSRAMEN)) <> 0));
end;

function __HAL_RCC_DTCMRAMEN_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_DTCMRAMEN)) <> 0));
end;

function __HAL_RCC_DMA2_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_DMA2EN)) <> 0));
end;

function __HAL_RCC_DMA2D_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_DMA2DEN)) <> 0));
end;

function __HAL_RCC_USB_OTG_HS_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_OTGHSEN)) <> 0));
end;

function __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_OTGHSULPIEN)) <> 0));
end;

function __HAL_RCC_GPIOA_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOAEN)) <> 0));
end;

function __HAL_RCC_GPIOB_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOBEN)) <> 0));
end;

function __HAL_RCC_GPIOC_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOCEN)) <> 0));
end;

function __HAL_RCC_GPIOD_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIODEN)) <> 0));
end;

function __HAL_RCC_GPIOE_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOEEN)) <> 0));
end;

function __HAL_RCC_GPIOF_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOFEN)) <> 0));
end;

function __HAL_RCC_GPIOG_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOGEN)) <> 0));
end;

function __HAL_RCC_GPIOH_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOHEN)) <> 0));
end;

function __HAL_RCC_GPIOI_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOIEN)) <> 0));
end;

function __HAL_RCC_GPIOJ_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOJEN)) <> 0));
end;

function __HAL_RCC_GPIOK_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOKEN)) <> 0));
end;

function __HAL_RCC_BKPSRAM_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_BKPSRAMEN)) = 0));
end;

function __HAL_RCC_DTCMRAMEN_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_DTCMRAMEN)) = 0));
end;

function __HAL_RCC_DMA2_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_DMA2EN)) = 0));
end;

function __HAL_RCC_DMA2D_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_DMA2DEN)) = 0));
end;

function __HAL_RCC_USB_OTG_HS_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_OTGHSEN)) = 0));
end;

function __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_OTGHSULPIEN)) = 0));
end;

function __HAL_RCC_GPIOA_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOAEN)) = 0));
end;

function __HAL_RCC_GPIOB_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOBEN)) = 0));
end;

function __HAL_RCC_GPIOC_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOCEN)) = 0));
end;

function __HAL_RCC_GPIOD_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIODEN)) = 0));
end;

function __HAL_RCC_GPIOE_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOEEN)) = 0));
end;

function __HAL_RCC_GPIOF_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOFEN)) = 0));
end;

function __HAL_RCC_GPIOG_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOGEN)) = 0));
end;

function __HAL_RCC_GPIOH_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOHEN)) = 0));
end;

function __HAL_RCC_GPIOI_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOIEN)) = 0));
end;

function __HAL_RCC_GPIOJ_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOJEN)) = 0));
end;

function __HAL_RCC_GPIOK_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_GPIOKEN)) = 0));
end;

(**
  * @brief  Enable ETHERNET clock.
  *)
function __HAL_RCC_ETHMAC_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_ETHMACEN)) <> 0));
end;

function __HAL_RCC_ETHMACTX_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_ETHMACTXEN)) <> 0));
end;

function __HAL_RCC_ETHMACRX_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_ETHMACRXEN)) <> 0));
end;

function __HAL_RCC_ETHMACPTP_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_ETHMACPTPEN)) <> 0));
end;

function __HAL_RCC_ETH_IS_CLK_ENABLED: boolean;
begin
  exit((__HAL_RCC_ETHMAC_IS_CLK_ENABLED() and __HAL_RCC_ETHMACTX_IS_CLK_ENABLED() and __HAL_RCC_ETHMACRX_IS_CLK_ENABLED()));
end;

(**
  * @brief  Disable ETHERNET clock.
  *)
function __HAL_RCC_ETHMAC_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_ETHMACEN)) = 0));
end;

function __HAL_RCC_ETHMACTX_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_ETHMACTXEN)) = 0));
end;

function __HAL_RCC_ETHMACRX_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_ETHMACRXEN)) = 0));
end;

function __HAL_RCC_ETHMACPTP_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB1ENR and (RCC_AHB1ENR_ETHMACPTPEN)) = 0));
end;

function __HAL_RCC_ETH_IS_CLK_DISABLED: boolean;
begin
  exit((__HAL_RCC_ETHMAC_IS_CLK_DISABLED() and __HAL_RCC_ETHMACTX_IS_CLK_DISABLED() and __HAL_RCC_ETHMACRX_IS_CLK_DISABLED()));
end;

(** @brief  Get the enable or disable status of the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
function __HAL_RCC_DCMI_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_DCMIEN)) <> 0));
end;

function __HAL_RCC_RNG_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_RNGEN)) <> 0));
end;

function __HAL_RCC_USB_OTG_FS_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_OTGFSEN)) <> 0));
end;


function __HAL_RCC_DCMI_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_DCMIEN)) = 0));
end;

function __HAL_RCC_RNG_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_RNGEN)) = 0));
end;

function __HAL_RCC_USB_IS_OTG_FS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_OTGFSEN)) = 0));
end;

{$if defined(STM32F756xx)}
function __HAL_RCC_CRYP_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_CRYPEN)) <> 0));
end;

function __HAL_RCC_HASH_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_HASHEN)) <> 0));
end;

function __HAL_RCC_CRYP_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_CRYPEN)) = 0));
end;

function __HAL_RCC_HASH_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB2ENR and (RCC_AHB2ENR_HASHEN)) = 0));
end;

{$endif}

(** @brief  Get the enable or disable status of the AHB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
function __HAL_RCC_FMC_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB3ENR and (RCC_AHB3ENR_FMCEN)) <> 0));
end;

function __HAL_RCC_QSPI_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.AHB3ENR and (RCC_AHB3ENR_QSPIEN)) <> 0));
end;

function __HAL_RCC_FMC_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB3ENR and (RCC_AHB3ENR_FMCEN)) = 0));
end;

function __HAL_RCC_QSPI_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.AHB3ENR and (RCC_AHB3ENR_QSPIEN)) = 0));
end;

(** @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
function __HAL_RCC_TIM2_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM2EN)) <> 0));
end;

function __HAL_RCC_TIM3_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM3EN)) <> 0));
end;

function __HAL_RCC_TIM4_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM4EN)) <> 0));
end;

function __HAL_RCC_TIM5_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM5EN)) <> 0));
end;

function __HAL_RCC_TIM6_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM6EN)) <> 0));
end;

function __HAL_RCC_TIM7_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM7EN)) <> 0));
end;

function __HAL_RCC_TIM12_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM12EN)) <> 0));
end;

function __HAL_RCC_TIM13_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM13EN)) <> 0));
end;

function __HAL_RCC_TIM14_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM14EN)) <> 0));
end;

function __HAL_RCC_LPTIM1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_LPTIM1EN)) <> 0));
end;

function __HAL_RCC_SPI2_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_SPI2EN)) <> 0));
end;

function __HAL_RCC_SPI3_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_SPI3EN)) <> 0));
end;

function __HAL_RCC_SPDIFRX_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_SPDIFRXEN)) <> 0));
end;

function __HAL_RCC_USART2_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_USART2EN)) <> 0));
end;

function __HAL_RCC_USART3_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_USART3EN)) <> 0));
end;

function __HAL_RCC_UART4_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_UART4EN)) <> 0));
end;

function __HAL_RCC_UART5_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_UART5EN)) <> 0));
end;

function __HAL_RCC_I2C1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_I2C1EN)) <> 0));
end;

function __HAL_RCC_I2C2_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_I2C2EN)) <> 0));
end;

function __HAL_RCC_I2C3_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_I2C3EN)) <> 0));
end;

function __HAL_RCC_I2C4_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_I2C4EN)) <> 0));
end;

function __HAL_RCC_CAN1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_CAN1EN)) <> 0));
end;

function __HAL_RCC_CAN2_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_CAN2EN)) <> 0));
end;

function __HAL_RCC_CEC_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_CECEN)) <> 0));
end;

function __HAL_RCC_DAC_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_DACEN)) <> 0));
end;

function __HAL_RCC_UART7_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_UART7EN)) <> 0));
end;

function __HAL_RCC_UART8_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_UART8EN)) <> 0));
end;

function __HAL_RCC_TIM2_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM2EN)) = 0));
end;

function __HAL_RCC_TIM3_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM3EN)) = 0));
end;

function __HAL_RCC_TIM4_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM4EN)) = 0));
end;

function __HAL_RCC_TIM5_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM5EN)) = 0));
end;

function __HAL_RCC_TIM6_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM6EN)) = 0));
end;

function __HAL_RCC_TIM7_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM7EN)) = 0));
end;

function __HAL_RCC_TIM12_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM12EN)) = 0));
end;

function __HAL_RCC_TIM13_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM13EN)) = 0));
end;

function __HAL_RCC_TIM14_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_TIM14EN)) = 0));
end;

function __HAL_RCC_LPTIM1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_LPTIM1EN)) = 0));
end;

function __HAL_RCC_SPI2_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_SPI2EN)) = 0));
end;

function __HAL_RCC_SPI3_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_SPI3EN)) = 0));
end;

function __HAL_RCC_SPDIFRX_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_SPDIFRXEN)) = 0));
end;

function __HAL_RCC_USART2_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_USART2EN)) = 0));
end;

function __HAL_RCC_USART3_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_USART3EN)) = 0));
end;

function __HAL_RCC_UART4_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_UART4EN)) = 0));
end;

function __HAL_RCC_UART5_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_UART5EN)) = 0));
end;

function __HAL_RCC_I2C1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_I2C1EN)) = 0));
end;

function __HAL_RCC_I2C2_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_I2C2EN)) = 0));
end;

function __HAL_RCC_I2C3_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_I2C3EN)) = 0));
end;

function __HAL_RCC_I2C4_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_I2C4EN)) = 0));
end;

function __HAL_RCC_CAN1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_CAN1EN)) = 0));
end;

function __HAL_RCC_CAN2_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_CAN2EN)) = 0));
end;

function __HAL_RCC_CEC_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_CECEN)) = 0));
end;

function __HAL_RCC_DAC_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_DACEN)) = 0));
end;

function __HAL_RCC_UART7_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_UART7EN)) = 0));
end;

function __HAL_RCC_UART8_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB1ENR and (RCC_APB1ENR_UART8EN)) = 0));
end;

(** @brief  Get the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  *)
function __HAL_RCC_TIM1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM1EN)) <> 0));
end;

function __HAL_RCC_TIM8_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM8EN)) <> 0));
end;

function __HAL_RCC_USART1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_USART1EN)) <> 0));
end;

function __HAL_RCC_USART6_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_USART6EN)) <> 0));
end;

function __HAL_RCC_ADC1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_ADC1EN)) <> 0));
end;

function __HAL_RCC_ADC2_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_ADC2EN)) <> 0));
end;

function __HAL_RCC_ADC3_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_ADC3EN)) <> 0));
end;

function __HAL_RCC_SDMMC1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SDMMC1EN)) <> 0));
end;

function __HAL_RCC_SPI1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SPI1EN)) <> 0));
end;

function __HAL_RCC_SPI4_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SPI4EN)) <> 0));
end;

function __HAL_RCC_TIM9_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM9EN)) <> 0));
end;

function __HAL_RCC_TIM10_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM10EN)) <> 0));
end;

function __HAL_RCC_TIM11_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM11EN)) <> 0));
end;

function __HAL_RCC_SPI5_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SPI5EN)) <> 0));
end;

function __HAL_RCC_SPI6_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SPI6EN)) <> 0));
end;

function __HAL_RCC_SAI1_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SAI1EN)) <> 0));
end;

function __HAL_RCC_SAI2_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SAI2EN)) <> 0));
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
function __HAL_RCC_LTDC_IS_CLK_ENABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_LTDCEN)) <> 0));
end;

{$endif}
function __HAL_RCC_TIM1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM1EN)) = 0));
end;

function __HAL_RCC_TIM8_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM8EN)) = 0));
end;

function __HAL_RCC_USART1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_USART1EN)) = 0));
end;

function __HAL_RCC_USART6_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_USART6EN)) = 0));
end;

function __HAL_RCC_ADC1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_ADC1EN)) = 0));
end;

function __HAL_RCC_ADC2_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_ADC2EN)) = 0));
end;

function __HAL_RCC_ADC3_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_ADC3EN)) = 0));
end;

function __HAL_RCC_SDMMC1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SDMMC1EN)) = 0));
end;

function __HAL_RCC_SPI1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SPI1EN)) = 0));
end;

function __HAL_RCC_SPI4_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SPI4EN)) = 0));
end;

function __HAL_RCC_TIM9_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM9EN)) = 0));
end;

function __HAL_RCC_TIM10_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM10EN)) = 0));
end;

function __HAL_RCC_TIM11_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_TIM11EN)) = 0));
end;

function __HAL_RCC_SPI5_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SPI5EN)) = 0));
end;

function __HAL_RCC_SPI6_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SPI6EN)) = 0));
end;

function __HAL_RCC_SAI1_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SAI1EN)) = 0));
end;

function __HAL_RCC_SAI2_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_SAI2EN)) = 0));
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
function __HAL_RCC_LTDC_IS_CLK_DISABLED: boolean;
begin
  exit(((RCC.APB2ENR and (RCC_APB2ENR_LTDCEN)) = 0));
end;

{$endif}
(**
  * @end
  *)

(** @defgroup RCCEx_Force_Release_Peripheral_Reset RCCEx Force Release Peripheral Reset
  * @brief  Forces or releases AHB/APB peripheral reset.
  * @begin
  *)

(** @brief  Force or release AHB1 peripheral reset.
  *)
procedure __HAL_RCC_DMA2_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_DMA2RST);
end;

procedure __HAL_RCC_DMA2D_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_DMA2DRST);
end;

procedure __HAL_RCC_ETHMAC_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_ETHMACRST);
end;

procedure __HAL_RCC_USB_OTG_HS_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_OTGHRST);
end;

procedure __HAL_RCC_GPIOA_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOARST);
end;

procedure __HAL_RCC_GPIOB_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOBRST);
end;

procedure __HAL_RCC_GPIOC_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOCRST);
end;

procedure __HAL_RCC_GPIOD_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIODRST);
end;

procedure __HAL_RCC_GPIOE_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOERST);
end;

procedure __HAL_RCC_GPIOF_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOFRST);
end;

procedure __HAL_RCC_GPIOG_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOGRST);
end;

procedure __HAL_RCC_GPIOH_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOHRST);
end;

procedure __HAL_RCC_GPIOI_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOIRST);
end;

procedure __HAL_RCC_GPIOJ_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOJRST);
end;

procedure __HAL_RCC_GPIOK_FORCE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR or (RCC_AHB1RSTR_GPIOKRST);
end;

procedure __HAL_RCC_DMA2_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_DMA2RST));
end;

procedure __HAL_RCC_DMA2D_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_DMA2DRST));
end;

procedure __HAL_RCC_ETHMAC_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_ETHMACRST));
end;

procedure __HAL_RCC_USB_OTG_HS_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_OTGHRST));
end;

procedure __HAL_RCC_GPIOA_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOARST));
end;

procedure __HAL_RCC_GPIOB_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOBRST));
end;

procedure __HAL_RCC_GPIOC_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOCRST));
end;

procedure __HAL_RCC_GPIOD_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIODRST));
end;

procedure __HAL_RCC_GPIOE_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOERST));
end;

procedure __HAL_RCC_GPIOF_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOFRST));
end;

procedure __HAL_RCC_GPIOG_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOGRST));
end;

procedure __HAL_RCC_GPIOH_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOHRST));
end;

procedure __HAL_RCC_GPIOI_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOIRST));
end;

procedure __HAL_RCC_GPIOJ_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOJRST));
end;

procedure __HAL_RCC_GPIOK_RELEASE_RESET;
begin
  RCC.AHB1RSTR := RCC.AHB1RSTR and (not (RCC_AHB1RSTR_GPIOKRST));
end;

(** @brief  Force or release AHB2 peripheral reset.
  *)
procedure __HAL_RCC_AHB2_FORCE_RESET;
begin
  RCC.AHB2RSTR := $FFFFFFFF;
end;

procedure __HAL_RCC_DCMI_FORCE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR or (RCC_AHB2RSTR_DCMIRST);
end;

procedure __HAL_RCC_RNG_FORCE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR or (RCC_AHB2RSTR_RNGRST);
end;

procedure __HAL_RCC_USB_OTG_FS_FORCE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR or (RCC_AHB2RSTR_OTGFSRST);
end;

procedure __HAL_RCC_AHB2_RELEASE_RESET;
begin
  RCC.AHB2RSTR := $00;
end;

procedure __HAL_RCC_DCMI_RELEASE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR and (not (RCC_AHB2RSTR_DCMIRST));
end;

procedure __HAL_RCC_RNG_RELEASE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR and (not (RCC_AHB2RSTR_RNGRST));
end;

procedure __HAL_RCC_USB_OTG_FS_RELEASE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR and (not (RCC_AHB2RSTR_OTGFSRST));
end;

{$if defined(STM32F756xx)}
procedure __HAL_RCC_CRYP_FORCE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR or (RCC_AHB2RSTR_CRYPRST);
end;

procedure __HAL_RCC_HASH_FORCE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR or (RCC_AHB2RSTR_HASHRST);
end;

procedure __HAL_RCC_CRYP_RELEASE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR and (not (RCC_AHB2RSTR_CRYPRST));
end;

procedure __HAL_RCC_HASH_RELEASE_RESET;
begin
  RCC.AHB2RSTR := RCC.AHB2RSTR and (not (RCC_AHB2RSTR_HASHRST));
end;

{$endif}

(** @brief  Force or release AHB3 peripheral reset
  *)
procedure __HAL_RCC_AHB3_FORCE_RESET;
begin
  RCC.AHB3RSTR := $FFFFFFFF;
end;

procedure __HAL_RCC_FMC_FORCE_RESET;
begin
  RCC.AHB3RSTR := RCC.AHB3RSTR or (RCC_AHB3RSTR_FMCRST);
end;

procedure __HAL_RCC_QSPI_FORCE_RESET;
begin
  RCC.AHB3RSTR := RCC.AHB3RSTR or (RCC_AHB3RSTR_QSPIRST);
end;

procedure __HAL_RCC_AHB3_RELEASE_RESET;
begin
  RCC.AHB3RSTR := $00;
end;

procedure __HAL_RCC_FMC_RELEASE_RESET;
begin
  RCC.AHB3RSTR := RCC.AHB3RSTR and (not (RCC_AHB3RSTR_FMCRST));
end;

procedure __HAL_RCC_QSPI_RELEASE_RESET;
begin
  RCC.AHB3RSTR := RCC.AHB3RSTR and (not (RCC_AHB3RSTR_QSPIRST));
end;

(** @brief  Force or release APB1 peripheral reset.
  *)
procedure __HAL_RCC_TIM2_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM2RST);
end;

procedure __HAL_RCC_TIM3_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM3RST);
end;

procedure __HAL_RCC_TIM4_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM4RST);
end;

procedure __HAL_RCC_TIM5_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM5RST);
end;

procedure __HAL_RCC_TIM6_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM6RST);
end;

procedure __HAL_RCC_TIM7_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM7RST);
end;

procedure __HAL_RCC_TIM12_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM12RST);
end;

procedure __HAL_RCC_TIM13_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM13RST);
end;

procedure __HAL_RCC_TIM14_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM14RST);
end;

procedure __HAL_RCC_LPTIM1_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_LPTIM1RST);
end;

procedure __HAL_RCC_SPI2_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_SPI2RST);
end;

procedure __HAL_RCC_SPI3_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_SPI3RST);
end;

procedure __HAL_RCC_SPDIFRX_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_SPDIFRXRST);
end;

procedure __HAL_RCC_USART2_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_USART2RST);
end;

procedure __HAL_RCC_USART3_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_USART3RST);
end;

procedure __HAL_RCC_UART4_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_UART4RST);
end;

procedure __HAL_RCC_UART5_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_UART5RST);
end;

procedure __HAL_RCC_I2C1_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_I2C1RST);
end;

procedure __HAL_RCC_I2C2_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_I2C2RST);
end;

procedure __HAL_RCC_I2C3_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_I2C3RST);
end;

procedure __HAL_RCC_I2C4_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_I2C4RST);
end;

procedure __HAL_RCC_CAN1_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_CAN1RST);
end;

procedure __HAL_RCC_CAN2_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_CAN2RST);
end;

procedure __HAL_RCC_CEC_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_CECRST);
end;

procedure __HAL_RCC_DAC_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_DACRST);
end;

procedure __HAL_RCC_UART7_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_UART7RST);
end;

procedure __HAL_RCC_UART8_FORCE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_UART8RST);
end;

procedure __HAL_RCC_TIM2_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM2RST));
end;

procedure __HAL_RCC_TIM3_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM3RST));
end;

procedure __HAL_RCC_TIM4_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM4RST));
end;

procedure __HAL_RCC_TIM5_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM5RST));
end;

procedure __HAL_RCC_TIM6_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM6RST));
end;

procedure __HAL_RCC_TIM7_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM7RST));
end;

procedure __HAL_RCC_TIM12_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM12RST));
end;

procedure __HAL_RCC_TIM13_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM13RST));
end;

procedure __HAL_RCC_TIM14_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_TIM14RST));
end;

procedure __HAL_RCC_LPTIM1_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_LPTIM1RST));
end;

procedure __HAL_RCC_SPI2_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_SPI2RST));
end;

procedure __HAL_RCC_SPI3_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_SPI3RST));
end;

procedure __HAL_RCC_SPDIFRX_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_SPDIFRXRST));
end;

procedure __HAL_RCC_USART2_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_USART2RST));
end;

procedure __HAL_RCC_USART3_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_USART3RST));
end;

procedure __HAL_RCC_UART4_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_UART4RST));
end;

procedure __HAL_RCC_UART5_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_UART5RST));
end;

procedure __HAL_RCC_I2C1_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_I2C1RST));
end;

procedure __HAL_RCC_I2C2_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_I2C2RST));
end;

procedure __HAL_RCC_I2C3_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_I2C3RST));
end;

procedure __HAL_RCC_I2C4_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_I2C4RST));
end;

procedure __HAL_RCC_CAN1_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_CAN1RST));
end;

procedure __HAL_RCC_CAN2_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_CAN2RST));
end;

procedure __HAL_RCC_CEC_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_CECRST));
end;

procedure __HAL_RCC_DAC_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_DACRST));
end;

procedure __HAL_RCC_UART7_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_UART7RST));
end;

procedure __HAL_RCC_UART8_RELEASE_RESET;
begin
  RCC.APB1RSTR := RCC.APB1RSTR and (not (RCC_APB1RSTR_UART8RST));
end;

(** @brief  Force or release APB2 peripheral reset.
  *)
procedure __HAL_RCC_TIM1_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM1RST);
end;

procedure __HAL_RCC_TIM8_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM8RST);
end;

procedure __HAL_RCC_USART1_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_USART1RST);
end;

procedure __HAL_RCC_USART6_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_USART6RST);
end;

procedure __HAL_RCC_ADC_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_ADCRST);
end;

procedure __HAL_RCC_SDMMC1_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SDMMC1RST);
end;

procedure __HAL_RCC_SPI1_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SPI1RST);
end;

procedure __HAL_RCC_SPI4_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SPI4RST);
end;

procedure __HAL_RCC_TIM9_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM9RST);
end;

procedure __HAL_RCC_TIM10_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM10RST);
end;

procedure __HAL_RCC_TIM11_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM11RST);
end;

procedure __HAL_RCC_SPI5_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SPI5RST);
end;

procedure __HAL_RCC_SPI6_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SPI6RST);
end;

procedure __HAL_RCC_SAI1_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SAI1RST);
end;

procedure __HAL_RCC_SAI2_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SAI2RST);
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_FORCE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_LTDCRST);
end;

{$endif}

procedure __HAL_RCC_TIM1_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_TIM1RST));
end;

procedure __HAL_RCC_TIM8_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_TIM8RST));
end;

procedure __HAL_RCC_USART1_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_USART1RST));
end;

procedure __HAL_RCC_USART6_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_USART6RST));
end;

procedure __HAL_RCC_ADC_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_ADCRST));
end;

procedure __HAL_RCC_SDMMC1_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_SDMMC1RST));
end;

procedure __HAL_RCC_SPI1_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_SPI1RST));
end;

procedure __HAL_RCC_SPI4_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_SPI4RST));
end;

procedure __HAL_RCC_TIM9_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_TIM9RST));
end;

procedure __HAL_RCC_TIM10_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_TIM10RST));
end;

procedure __HAL_RCC_TIM11_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_TIM11RST));
end;

procedure __HAL_RCC_SPI5_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_SPI5RST));
end;

procedure __HAL_RCC_SPI6_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_SPI6RST));
end;

procedure __HAL_RCC_SAI1_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_SAI1RST));
end;

procedure __HAL_RCC_SAI2_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_SAI2RST));
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_RELEASE_RESET;
begin
  RCC.APB2RSTR := RCC.APB2RSTR and (not (RCC_APB2RSTR_LTDCRST));
end;

{$endif}
(**
  * @end
  *)

(** @defgroup RCCEx_Peripheral_Clock_Sleep_Enable_Disable RCCEx Peripheral Clock Sleep Enable Disable
  * @brief  Enables or disables the AHB/APB peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @begin
  *)

(** @brief  Enable or disable the AHB1 peripheral clock during Low Power (Sleep) mode.
  *)
procedure __HAL_RCC_FLITF_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_FLITFLPEN);
end;

procedure __HAL_RCC_AXI_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_AXILPEN);
end;

procedure __HAL_RCC_SRAM1_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_SRAM1LPEN);
end;

procedure __HAL_RCC_SRAM2_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_SRAM2LPEN);
end;

procedure __HAL_RCC_BKPSRAM_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_BKPSRAMLPEN);
end;

procedure __HAL_RCC_DTCM_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_DTCMLPEN);
end;

procedure __HAL_RCC_DMA2_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_DMA2LPEN);
end;

procedure __HAL_RCC_DMA2D_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_DMA2DLPEN);
end;

procedure __HAL_RCC_ETHMAC_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_ETHMACLPEN);
end;

procedure __HAL_RCC_ETHMACTX_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_ETHMACTXLPEN);
end;

procedure __HAL_RCC_ETHMACRX_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_ETHMACRXLPEN);
end;

procedure __HAL_RCC_ETHMACPTP_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_ETHMACPTPLPEN);
end;

procedure __HAL_RCC_USB_OTG_HS_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_OTGHSLPEN);
end;

procedure __HAL_RCC_USB_OTG_HS_ULPI_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_OTGHSULPILPEN);
end;

procedure __HAL_RCC_GPIOA_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOALPEN);
end;

procedure __HAL_RCC_GPIOB_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOBLPEN);
end;

procedure __HAL_RCC_GPIOC_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOCLPEN);
end;

procedure __HAL_RCC_GPIOD_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIODLPEN);
end;

procedure __HAL_RCC_GPIOE_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOELPEN);
end;

procedure __HAL_RCC_GPIOF_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOFLPEN);
end;

procedure __HAL_RCC_GPIOG_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOGLPEN);
end;

procedure __HAL_RCC_GPIOH_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOHLPEN);
end;

procedure __HAL_RCC_GPIOI_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOILPEN);
end;

procedure __HAL_RCC_GPIOJ_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOJLPEN);
end;

procedure __HAL_RCC_GPIOK_CLK_SLEEP_ENABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR or (RCC_AHB1LPENR_GPIOKLPEN);
end;

procedure __HAL_RCC_FLITF_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_FLITFLPEN));
end;

procedure __HAL_RCC_AXI_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_AXILPEN));
end;

procedure __HAL_RCC_SRAM1_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_SRAM1LPEN));
end;

procedure __HAL_RCC_SRAM2_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_SRAM2LPEN));
end;

procedure __HAL_RCC_BKPSRAM_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_BKPSRAMLPEN));
end;

procedure __HAL_RCC_DTCM_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_DTCMLPEN));
end;

procedure __HAL_RCC_DMA2_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_DMA2LPEN));
end;

procedure __HAL_RCC_DMA2D_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_DMA2DLPEN));
end;

procedure __HAL_RCC_ETHMAC_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_ETHMACLPEN));
end;

procedure __HAL_RCC_ETHMACTX_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_ETHMACTXLPEN));
end;

procedure __HAL_RCC_ETHMACRX_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_ETHMACRXLPEN));
end;

procedure __HAL_RCC_ETHMACPTP_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_ETHMACPTPLPEN));
end;

procedure __HAL_RCC_USB_OTG_HS_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_OTGHSLPEN));
end;

procedure __HAL_RCC_USB_OTG_HS_ULPI_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_OTGHSULPILPEN));
end;

procedure __HAL_RCC_GPIOA_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOALPEN));
end;

procedure __HAL_RCC_GPIOB_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOBLPEN));
end;

procedure __HAL_RCC_GPIOC_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOCLPEN));
end;

procedure __HAL_RCC_GPIOD_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIODLPEN));
end;

procedure __HAL_RCC_GPIOE_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOELPEN));
end;

procedure __HAL_RCC_GPIOF_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOFLPEN));
end;

procedure __HAL_RCC_GPIOG_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOGLPEN));
end;

procedure __HAL_RCC_GPIOH_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOHLPEN));
end;

procedure __HAL_RCC_GPIOI_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOILPEN));
end;

procedure __HAL_RCC_GPIOJ_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOJLPEN));
end;

procedure __HAL_RCC_GPIOK_CLK_SLEEP_DISABLE;
begin
  RCC.AHB1LPENR := RCC.AHB1LPENR and (not (RCC_AHB1LPENR_GPIOKLPEN));
end;

(** @brief  Enable or disable the AHB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
procedure __HAL_RCC_DCMI_CLK_SLEEP_ENABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR or (RCC_AHB2LPENR_DCMILPEN);
end;

procedure __HAL_RCC_DCMI_CLK_SLEEP_DISABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR and (not (RCC_AHB2LPENR_DCMILPEN));
end;

procedure __HAL_RCC_RNG_CLK_SLEEP_ENABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR or (RCC_AHB2LPENR_RNGLPEN);
end;

procedure __HAL_RCC_RNG_CLK_SLEEP_DISABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR and (not (RCC_AHB2LPENR_RNGLPEN));
end;

procedure __HAL_RCC_USB_OTG_FS_CLK_SLEEP_ENABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR or (RCC_AHB2LPENR_OTGFSLPEN);
end;

procedure __HAL_RCC_USB_OTG_FS_CLK_SLEEP_DISABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR and (not (RCC_AHB2LPENR_OTGFSLPEN));
end;

{$if defined(STM32F756xx)}
procedure __HAL_RCC_CRYP_CLK_SLEEP_ENABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR or (RCC_AHB2LPENR_CRYPLPEN);
end;

procedure __HAL_RCC_HASH_CLK_SLEEP_ENABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR or (RCC_AHB2LPENR_HASHLPEN);
end;

procedure __HAL_RCC_CRYP_CLK_SLEEP_DISABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR and (not (RCC_AHB2LPENR_CRYPLPEN));
end;

procedure __HAL_RCC_HASH_CLK_SLEEP_DISABLE;
begin
  RCC.AHB2LPENR := RCC.AHB2LPENR and (not (RCC_AHB2LPENR_HASHLPEN));
end;

{$endif}

(** @brief  Enable or disable the AHB3 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
procedure __HAL_RCC_FMC_CLK_SLEEP_ENABLE;
begin
  RCC.AHB3LPENR := RCC.AHB3LPENR or (RCC_AHB3LPENR_FMCLPEN);
end;

procedure __HAL_RCC_FMC_CLK_SLEEP_DISABLE;
begin
  RCC.AHB3LPENR := RCC.AHB3LPENR and (not (RCC_AHB3LPENR_FMCLPEN));
end;

procedure __HAL_RCC_QSPI_CLK_SLEEP_ENABLE;
begin
  RCC.AHB3LPENR := RCC.AHB3LPENR or (RCC_AHB3LPENR_QSPILPEN);
end;

procedure __HAL_RCC_QSPI_CLK_SLEEP_DISABLE;
begin
  RCC.AHB3LPENR := RCC.AHB3LPENR and (not (RCC_AHB3LPENR_QSPILPEN));
end;

(** @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
procedure __HAL_RCC_TIM2_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM2LPEN);
end;

procedure __HAL_RCC_TIM3_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM3LPEN);
end;

procedure __HAL_RCC_TIM4_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM4LPEN);
end;

procedure __HAL_RCC_TIM5_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM5LPEN);
end;

procedure __HAL_RCC_TIM6_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM6LPEN);
end;

procedure __HAL_RCC_TIM7_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM7LPEN);
end;

procedure __HAL_RCC_TIM12_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM12LPEN);
end;

procedure __HAL_RCC_TIM13_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM13LPEN);
end;

procedure __HAL_RCC_TIM14_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_TIM14LPEN);
end;

procedure __HAL_RCC_LPTIM1_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_LPTIM1LPEN);
end;

procedure __HAL_RCC_SPI2_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_SPI2LPEN);
end;

procedure __HAL_RCC_SPI3_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_SPI3LPEN);
end;

procedure __HAL_RCC_SPDIFRX_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_SPDIFRXLPEN);
end;

procedure __HAL_RCC_USART2_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_USART2LPEN);
end;

procedure __HAL_RCC_USART3_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_USART3LPEN);
end;

procedure __HAL_RCC_UART4_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_UART4LPEN);
end;

procedure __HAL_RCC_UART5_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_UART5LPEN);
end;

procedure __HAL_RCC_I2C1_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_I2C1LPEN);
end;

procedure __HAL_RCC_I2C2_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_I2C2LPEN);
end;

procedure __HAL_RCC_I2C3_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_I2C3LPEN);
end;

procedure __HAL_RCC_I2C4_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_I2C4LPEN);
end;

procedure __HAL_RCC_CAN1_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_CAN1LPEN);
end;

procedure __HAL_RCC_CAN2_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_CAN2LPEN);
end;

procedure __HAL_RCC_CEC_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_CECLPEN);
end;

procedure __HAL_RCC_DAC_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_DACLPEN);
end;

procedure __HAL_RCC_UART7_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_UART7LPEN);
end;

procedure __HAL_RCC_UART8_CLK_SLEEP_ENABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR or (RCC_APB1LPENR_UART8LPEN);
end;

procedure __HAL_RCC_TIM2_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM2LPEN));
end;

procedure __HAL_RCC_TIM3_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM3LPEN));
end;

procedure __HAL_RCC_TIM4_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM4LPEN));
end;

procedure __HAL_RCC_TIM5_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM5LPEN));
end;

procedure __HAL_RCC_TIM6_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM6LPEN));
end;

procedure __HAL_RCC_TIM7_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM7LPEN));
end;

procedure __HAL_RCC_TIM12_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM12LPEN));
end;

procedure __HAL_RCC_TIM13_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM13LPEN));
end;

procedure __HAL_RCC_TIM14_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_TIM14LPEN));
end;

procedure __HAL_RCC_LPTIM1_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_LPTIM1LPEN));
end;

procedure __HAL_RCC_SPI2_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_SPI2LPEN));
end;

procedure __HAL_RCC_SPI3_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_SPI3LPEN));
end;

procedure __HAL_RCC_SPDIFRX_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_SPDIFRXLPEN));
end;

procedure __HAL_RCC_USART2_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_USART2LPEN));
end;

procedure __HAL_RCC_USART3_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_USART3LPEN));
end;

procedure __HAL_RCC_UART4_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_UART4LPEN));
end;

procedure __HAL_RCC_UART5_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_UART5LPEN));
end;

procedure __HAL_RCC_I2C1_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_I2C1LPEN));
end;

procedure __HAL_RCC_I2C2_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_I2C2LPEN));
end;

procedure __HAL_RCC_I2C3_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_I2C3LPEN));
end;

procedure __HAL_RCC_I2C4_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_I2C4LPEN));
end;

procedure __HAL_RCC_CAN1_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_CAN1LPEN));
end;

procedure __HAL_RCC_CAN2_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_CAN2LPEN));
end;

procedure __HAL_RCC_CEC_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_CECLPEN));
end;

procedure __HAL_RCC_DAC_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_DACLPEN));
end;

procedure __HAL_RCC_UART7_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_UART7LPEN));
end;

procedure __HAL_RCC_UART8_CLK_SLEEP_DISABLE;
begin
  RCC.APB1LPENR := RCC.APB1LPENR and (not (RCC_APB1LPENR_UART8LPEN));
end;

(** @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
procedure __HAL_RCC_TIM1_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_TIM1LPEN);
end;

procedure __HAL_RCC_TIM8_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_TIM8LPEN);
end;

procedure __HAL_RCC_USART1_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_USART1LPEN);
end;

procedure __HAL_RCC_USART6_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_USART6LPEN);
end;

procedure __HAL_RCC_ADC1_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_ADC1LPEN);
end;

procedure __HAL_RCC_ADC2_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_ADC2LPEN);
end;

procedure __HAL_RCC_ADC3_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_ADC3LPEN);
end;

procedure __HAL_RCC_SDMMC1_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_SDMMC1LPEN);
end;

procedure __HAL_RCC_SPI1_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_SPI1LPEN);
end;

procedure __HAL_RCC_SPI4_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_SPI4LPEN);
end;

procedure __HAL_RCC_TIM9_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_TIM9LPEN);
end;

procedure __HAL_RCC_TIM10_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_TIM10LPEN);
end;

procedure __HAL_RCC_TIM11_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_TIM11LPEN);
end;

procedure __HAL_RCC_SPI5_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_SPI5LPEN);
end;

procedure __HAL_RCC_SPI6_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_SPI6LPEN);
end;

procedure __HAL_RCC_SAI1_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_SAI1LPEN);
end;

procedure __HAL_RCC_SAI2_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_SAI2LPEN);
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_CLK_SLEEP_ENABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR or (RCC_APB2LPENR_LTDCLPEN);
end;

{$endif}

procedure __HAL_RCC_TIM1_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_TIM1LPEN));
end;

procedure __HAL_RCC_TIM8_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_TIM8LPEN));
end;

procedure __HAL_RCC_USART1_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_USART1LPEN));
end;

procedure __HAL_RCC_USART6_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_USART6LPEN));
end;

procedure __HAL_RCC_ADC1_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_ADC1LPEN));
end;

procedure __HAL_RCC_ADC2_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_ADC2LPEN));
end;

procedure __HAL_RCC_ADC3_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_ADC3LPEN));
end;

procedure __HAL_RCC_SDMMC1_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_SDMMC1LPEN));
end;

procedure __HAL_RCC_SPI1_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_SPI1LPEN));
end;

procedure __HAL_RCC_SPI4_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_SPI4LPEN));
end;

procedure __HAL_RCC_TIM9_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_TIM9LPEN));
end;

procedure __HAL_RCC_TIM10_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_TIM10LPEN));
end;

procedure __HAL_RCC_TIM11_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_TIM11LPEN));
end;

procedure __HAL_RCC_SPI5_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_SPI5LPEN));
end;

procedure __HAL_RCC_SPI6_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_SPI6LPEN));
end;

procedure __HAL_RCC_SAI1_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_SAI1LPEN));
end;

procedure __HAL_RCC_SAI2_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_SAI2LPEN));
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
procedure __HAL_RCC_LTDC_CLK_SLEEP_DISABLE;
begin
  RCC.APB2LPENR := RCC.APB2LPENR and (not (RCC_APB2LPENR_LTDCLPEN));
end;

{$endif}
(**
  * @end
  *)

(** @defgroup RCC_Clock_Sleep_Enable_Disable_Status AHB/APB Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the AHB/APB peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @begin
  *)

(** @brief  Get the enable or disable status of the AHB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
function __HAL_RCC_FLITF_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_FLITFLPEN)) <> 0));
end;

function __HAL_RCC_AXI_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_AXILPEN)) <> 0));
end;

function __HAL_RCC_SRAM1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_SRAM1LPEN)) <> 0));
end;

function __HAL_RCC_SRAM2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_SRAM2LPEN)) <> 0));
end;

function __HAL_RCC_BKPSRAM_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_BKPSRAMLPEN)) <> 0));
end;

function __HAL_RCC_DTCM_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_DTCMLPEN)) <> 0));
end;

function __HAL_RCC_DMA2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_DMA2LPEN)) <> 0));
end;

function __HAL_RCC_DMA2D_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_DMA2DLPEN)) <> 0));
end;

function __HAL_RCC_ETHMAC_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_ETHMACLPEN)) <> 0));
end;

function __HAL_RCC_ETHMACTX_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_ETHMACTXLPEN)) <> 0));
end;

function __HAL_RCC_ETHMACRX_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_ETHMACRXLPEN)) <> 0));
end;

function __HAL_RCC_ETHMACPTP_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_ETHMACPTPLPEN)) <> 0));
end;

function __HAL_RCC_USB_OTG_HS_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_OTGHSLPEN)) <> 0));
end;

function __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_OTGHSULPILPEN)) <> 0));
end;

function __HAL_RCC_GPIOA_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOALPEN)) <> 0));
end;

function __HAL_RCC_GPIOB_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOBLPEN)) <> 0));
end;

function __HAL_RCC_GPIOC_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOCLPEN)) <> 0));
end;

function __HAL_RCC_GPIOD_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIODLPEN)) <> 0));
end;

function __HAL_RCC_GPIOE_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOELPEN)) <> 0));
end;

function __HAL_RCC_GPIOF_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOFLPEN)) <> 0));
end;

function __HAL_RCC_GPIOG_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOGLPEN)) <> 0));
end;

function __HAL_RCC_GPIOH_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOHLPEN)) <> 0));
end;

function __HAL_RCC_GPIOI_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOILPEN)) <> 0));
end;

function __HAL_RCC_GPIOJ_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOJLPEN)) <> 0));
end;

function __HAL_RCC_GPIOK_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOKLPEN)) <> 0));
end;

function __HAL_RCC_FLITF_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_FLITFLPEN)) = 0));
end;

function __HAL_RCC_AXI_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_AXILPEN)) = 0));
end;

function __HAL_RCC_SRAM1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_SRAM1LPEN)) = 0));
end;

function __HAL_RCC_SRAM2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_SRAM2LPEN)) = 0));
end;

function __HAL_RCC_BKPSRAM_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_BKPSRAMLPEN)) = 0));
end;

function __HAL_RCC_DTCM_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_DTCMLPEN)) = 0));
end;

function __HAL_RCC_DMA2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_DMA2LPEN)) = 0));
end;

function __HAL_RCC_DMA2D_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_DMA2DLPEN)) = 0));
end;

function __HAL_RCC_ETHMAC_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_ETHMACLPEN)) = 0));
end;

function __HAL_RCC_ETHMACTX_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_ETHMACTXLPEN)) = 0));
end;

function __HAL_RCC_ETHMACRX_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_ETHMACRXLPEN)) = 0));
end;

function __HAL_RCC_ETHMACPTP_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_ETHMACPTPLPEN)) = 0));
end;

function __HAL_RCC_USB_OTG_HS_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_OTGHSLPEN)) = 0));
end;

function __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_OTGHSULPILPEN)) = 0));
end;

function __HAL_RCC_GPIOA_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOALPEN)) = 0));
end;

function __HAL_RCC_GPIOB_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOBLPEN)) = 0));
end;

function __HAL_RCC_GPIOC_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOCLPEN)) = 0));
end;

function __HAL_RCC_GPIOD_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIODLPEN)) = 0));
end;

function __HAL_RCC_GPIOE_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOELPEN)) = 0));
end;

function __HAL_RCC_GPIOF_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOFLPEN)) = 0));
end;

function __HAL_RCC_GPIOG_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOGLPEN)) = 0));
end;

function __HAL_RCC_GPIOH_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOHLPEN)) = 0));
end;

function __HAL_RCC_GPIOI_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOILPEN)) = 0));
end;

function __HAL_RCC_GPIOJ_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOJLPEN)) = 0));
end;

function __HAL_RCC_GPIOK_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB1LPENR and (RCC_AHB1LPENR_GPIOKLPEN)) = 0));
end;

(** @brief  Get the enable or disable status of the AHB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
function __HAL_RCC_DCMI_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_DCMILPEN)) <> 0));
end;

function __HAL_RCC_DCMI_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_DCMILPEN)) = 0));
end;

function __HAL_RCC_RNG_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_RNGLPEN)) <> 0));
end;

function __HAL_RCC_RNG_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_RNGLPEN)) = 0));
end;

function __HAL_RCC_USB_OTG_FS_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_OTGFSLPEN)) <> 0));
end;

function __HAL_RCC_USB_OTG_FS_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_OTGFSLPEN)) = 0));
end;

{$if defined(STM32F756xx)}
function __HAL_RCC_CRYP_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_CRYPLPEN)) <> 0));
end;

function __HAL_RCC_HASH_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_HASHLPEN)) <> 0));
end;

function __HAL_RCC_CRYP_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_CRYPLPEN)) = 0));
end;

function __HAL_RCC_HASH_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB2LPENR and (RCC_AHB2LPENR_HASHLPEN)) = 0));
end;

{$endif}

(** @brief  Get the enable or disable status of the AHB3 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
function __HAL_RCC_FMC_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB3LPENR and (RCC_AHB3LPENR_FMCLPEN)) <> 0));
end;

function __HAL_RCC_FMC_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB3LPENR and (RCC_AHB3LPENR_FMCLPEN)) = 0));
end;

function __HAL_RCC_QSPI_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.AHB3LPENR and (RCC_AHB3LPENR_QSPILPEN)) <> 0));
end;

function __HAL_RCC_QSPI_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.AHB3LPENR and (RCC_AHB3LPENR_QSPILPEN)) = 0));
end;

(** @brief  Get the enable or disable status of the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
function __HAL_RCC_TIM2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM2LPEN)) <> 0));
end;

function __HAL_RCC_TIM3_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM3LPEN)) <> 0));
end;

function __HAL_RCC_TIM4_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM4LPEN)) <> 0));
end;

function __HAL_RCC_TIM5_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM5LPEN)) <> 0));
end;

function __HAL_RCC_TIM6_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM6LPEN)) <> 0));
end;

function __HAL_RCC_TIM7_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM7LPEN)) <> 0));
end;

function __HAL_RCC_TIM12_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM12LPEN)) <> 0));
end;

function __HAL_RCC_TIM13_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM13LPEN)) <> 0));
end;

function __HAL_RCC_TIM14_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM14LPEN)) <> 0));
end;

function __HAL_RCC_LPTIM1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_LPTIM1LPEN)) <> 0));
end;

function __HAL_RCC_SPI2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_SPI2LPEN)) <> 0));
end;

function __HAL_RCC_SPI3_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_SPI3LPEN)) <> 0));
end;

function __HAL_RCC_SPDIFRX_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_SPDIFRXLPEN)) <> 0));
end;

function __HAL_RCC_USART2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_USART2LPEN)) <> 0));
end;

function __HAL_RCC_USART3_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_USART3LPEN)) <> 0));
end;

function __HAL_RCC_UART4_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_UART4LPEN)) <> 0));
end;

function __HAL_RCC_UART5_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_UART5LPEN)) <> 0));
end;

function __HAL_RCC_I2C1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_I2C1LPEN)) <> 0));
end;

function __HAL_RCC_I2C2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_I2C2LPEN)) <> 0));
end;

function __HAL_RCC_I2C3_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_I2C3LPEN)) <> 0));
end;

function __HAL_RCC_I2C4_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_I2C4LPEN)) <> 0));
end;

function __HAL_RCC_CAN1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_CAN1LPEN)) <> 0));
end;

function __HAL_RCC_CAN2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_CAN2LPEN)) <> 0));
end;

function __HAL_RCC_CEC_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_CECLPEN)) <> 0));
end;

function __HAL_RCC_DAC_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_DACLPEN)) <> 0));
end;

function __HAL_RCC_UART7_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_UART7LPEN)) <> 0));
end;

function __HAL_RCC_UART8_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_UART8LPEN)) <> 0));
end;

function __HAL_RCC_TIM2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM2LPEN)) = 0));
end;

function __HAL_RCC_TIM3_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM3LPEN)) = 0));
end;

function __HAL_RCC_TIM4_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM4LPEN)) = 0));
end;

function __HAL_RCC_TIM5_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM5LPEN)) = 0));
end;

function __HAL_RCC_TIM6_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM6LPEN)) = 0));
end;

function __HAL_RCC_TIM7_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM7LPEN)) = 0));
end;

function __HAL_RCC_TIM12_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM12LPEN)) = 0));
end;

function __HAL_RCC_TIM13_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM13LPEN)) = 0));
end;

function __HAL_RCC_TIM14_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_TIM14LPEN)) = 0));
end;

function __HAL_RCC_LPTIM1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_LPTIM1LPEN)) = 0));
end;

function __HAL_RCC_SPI2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_SPI2LPEN)) = 0));
end;

function __HAL_RCC_SPI3_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_SPI3LPEN)) = 0));
end;

function __HAL_RCC_SPDIFRX_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_SPDIFRXLPEN)) = 0));
end;

function __HAL_RCC_USART2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_USART2LPEN)) = 0));
end;

function __HAL_RCC_USART3_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_USART3LPEN)) = 0));
end;

function __HAL_RCC_UART4_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_UART4LPEN)) = 0));
end;

function __HAL_RCC_UART5_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_UART5LPEN)) = 0));
end;

function __HAL_RCC_I2C1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_I2C1LPEN)) = 0));
end;

function __HAL_RCC_I2C2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_I2C2LPEN)) = 0));
end;

function __HAL_RCC_I2C3_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_I2C3LPEN)) = 0));
end;

function __HAL_RCC_I2C4_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_I2C4LPEN)) = 0));
end;

function __HAL_RCC_CAN1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_CAN1LPEN)) = 0));
end;

function __HAL_RCC_CAN2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_CAN2LPEN)) = 0));
end;

function __HAL_RCC_CEC_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_CECLPEN)) = 0));
end;

function __HAL_RCC_DAC_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_DACLPEN)) = 0));
end;

function __HAL_RCC_UART7_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_UART7LPEN)) = 0));
end;

function __HAL_RCC_UART8_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB1LPENR and (RCC_APB1LPENR_UART8LPEN)) = 0));
end;

(** @brief  Get the enable or disable status of the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  *)
function __HAL_RCC_TIM1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM1LPEN)) <> 0));
end;

function __HAL_RCC_TIM8_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM8LPEN)) <> 0));
end;

function __HAL_RCC_USART1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_USART1LPEN)) <> 0));
end;

function __HAL_RCC_USART6_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_USART6LPEN)) <> 0));
end;

function __HAL_RCC_ADC1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_ADC1LPEN)) <> 0));
end;

function __HAL_RCC_ADC2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_ADC2LPEN)) <> 0));
end;

function __HAL_RCC_ADC3_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_ADC3LPEN)) <> 0));
end;

function __HAL_RCC_SDMMC1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SDMMC1LPEN)) <> 0));
end;

function __HAL_RCC_SPI1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SPI1LPEN)) <> 0));
end;

function __HAL_RCC_SPI4_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SPI4LPEN)) <> 0));
end;

function __HAL_RCC_TIM9_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM9LPEN)) <> 0));
end;

function __HAL_RCC_TIM10_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM10LPEN)) <> 0));
end;

function __HAL_RCC_TIM11_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM11LPEN)) <> 0));
end;

function __HAL_RCC_SPI5_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SPI5LPEN)) <> 0));
end;

function __HAL_RCC_SPI6_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SPI6LPEN)) <> 0));
end;

function __HAL_RCC_SAI1_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SAI1LPEN)) <> 0));
end;

function __HAL_RCC_SAI2_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SAI2LPEN)) <> 0));
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
function __HAL_RCC_LTDC_IS_CLK_SLEEP_ENABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_LTDCLPEN)) <> 0));
end;

{$endif}

function __HAL_RCC_TIM1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM1LPEN)) = 0));
end;

function __HAL_RCC_TIM8_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM8LPEN)) = 0));
end;

function __HAL_RCC_USART1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_USART1LPEN)) = 0));
end;

function __HAL_RCC_USART6_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_USART6LPEN)) = 0));
end;

function __HAL_RCC_ADC1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_ADC1LPEN)) = 0));
end;

function __HAL_RCC_ADC2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_ADC2LPEN)) = 0));
end;

function __HAL_RCC_ADC3_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_ADC3LPEN)) = 0));
end;

function __HAL_RCC_SDMMC1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SDMMC1LPEN)) = 0));
end;

function __HAL_RCC_SPI1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SPI1LPEN)) = 0));
end;

function __HAL_RCC_SPI4_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SPI4LPEN)) = 0));
end;

function __HAL_RCC_TIM9_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM9LPEN)) = 0));
end;

function __HAL_RCC_TIM10_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM10LPEN)) = 0));
end;

function __HAL_RCC_TIM11_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_TIM11LPEN)) = 0));
end;

function __HAL_RCC_SPI5_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SPI5LPEN)) = 0));
end;

function __HAL_RCC_SPI6_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SPI6LPEN)) = 0));
end;

function __HAL_RCC_SAI1_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SAI1LPEN)) = 0));
end;

function __HAL_RCC_SAI2_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_SAI2LPEN)) = 0));
end;

{$if defined(STM32F756xx)  or  defined(STM32F746xx)}
function __HAL_RCC_LTDC_IS_CLK_SLEEP_DISABLED: boolean;
begin
  exit(((RCC.APB2LPENR and (RCC_APB2LPENR_LTDCLPEN)) = 0));
end;

{$endif}
(**
  * @end
  *)

(*---------------------------------------------------------------------------------------------*)

(** @brief  Macro to configure the Timers clocks prescalers
  * @param  __PRESC__ : specifies the Timers clocks prescalers selection
  *         This parameter can be one of the following values:
  *            @arg RCC_TIMPRES_DESACTIVATED: The Timers kernels clocks prescaler is
  *                 equal to HPRE if PPREx is corresponding to division by 1 or 2,
  *                 else it is equal to [(HPRE * PPREx) / 2] if PPREx is corresponding to
  *                 division by 4 or more.
  *            @arg RCC_TIMPRES_ACTIVATED: The Timers kernels clocks prescaler is
  *                 equal to HPRE if PPREx is corresponding to division by 1, 2 or 4,
  *                 else it is equal to [(HPRE * PPREx) / 4] if PPREx is corresponding
  *                 to division by 8 or more.
  *)
procedure __HAL_RCC_TIMCLKPRESCALER(__PRESC__: longword);
begin
  RCC.DCKCFGR1 := RCC.DCKCFGR1 and (not (RCC_DCKCFGR1_TIMPRE));
  RCC.DCKCFGR1 := RCC.DCKCFGR1 or (__PRESC__);
end;

(** @brief Macros to Enable or Disable the PLLISAI.
  * @note  The PLLSAI is disabled by hardware when entering STOP and STANDBY modes.
  *)
procedure __HAL_RCC_PLLSAI_ENABLE;
begin
  RCC.CR := RCC.CR or (RCC_CR_PLLSAION);
end;

procedure __HAL_RCC_PLLSAI_DISABLE;
begin
  RCC.CR := RCC.CR and (not (RCC_CR_PLLSAION));
end;

(** @brief  Macro to configure the PLLSAI clock multiplication and division factors.
  * @note   This function must be used only when the PLLSAI is disabled.
  * @note   PLLSAI clock source is common with the main PLL (configured in
  *         RCC_PLLConfig function )
  * @param  __PLLSAIN__: specifies the multiplication factor for PLLSAI VCO output clock.
  *         This parameter must be a number between Min_Data := 49 and Max_Data := 432.
  * @note   You have to set the PLLSAIN parameter correctly to ensure that the VCO
  *         output frequency is between Min_Data := 49 and Max_Data := 432 MHz.
  * @param  __PLLSAIQ__: specifies the division factor for SAI clock
  *         This parameter must be a number between Min_Data := 2 and Max_Data := 15.
  * @param  __PLLSAIR__: specifies the division factor for LTDC clock
  *         This parameter must be a number between Min_Data := 2 and Max_Data := 7.
  * @param  __PLLSAIP__: specifies the division factor for USB, RNG, SDMMC clocks
  *         This parameter can be a value of @ref RCCEx_PLLSAIP_Clock_Divider .
  *)
procedure __HAL_RCC_PLLSAI_CONFIG(__PLLSAIN__, __PLLSAIP__, __PLLSAIQ__, __PLLSAIR__: longword);
begin
  RCC.PLLSAICFGR := ((__PLLSAIN__) shl 6) or ((__PLLSAIP__) shl 16) or ((__PLLSAIQ__) shl 24) or ((__PLLSAIR__) shl 28);
end;

(** @brief  Macro used by the SAI HAL driver to configure the PLLI2S clock multiplication and division factors.
  * @note   This macro must be used only when the PLLI2S is disabled.
  * @note   PLLI2S clock source is common with the main PLL (configured in
  *         HAL_RCC_ClockConfig() API)
  * @param  __PLLI2SN__: specifies the multiplication factor for PLLI2S VCO output clock.
  *         This parameter must be a number between Min_Data := 192 and Max_Data := 432.
  * @note   You have to set the PLLI2SN parameter correctly to ensure that the VCO
  *         output frequency is between Min_Data := 192 and Max_Data := 432 MHz.
  * @param  __PLLI2SQ__: specifies the division factor for SAI clock.
  *         This parameter must be a number between Min_Data := 2 and Max_Data := 15.
  * @param  __PLLI2SR__: specifies the division factor for I2S clock
  *         This parameter must be a number between Min_Data := 2 and Max_Data := 7.
  * @note   You have to set the PLLI2SR parameter correctly to not exceed 192 MHz
  *         on the I2S clock frequency.
  * @param  __PLLI2SP__: specifies the division factor for SPDDIF-RX clock.
  *         This parameter can be a number between 0 and 3 for respective values 2, 4, 6 and 8
  *)
procedure __HAL_RCC_PLLI2S_CONFIG(__PLLI2SN__, __PLLI2SP__, __PLLI2SQ__, __PLLI2SR__: longword);
begin
  RCC.PLLI2SCFGR := ((__PLLI2SN__) shl 6) or ((__PLLI2SP__) shl 16) or ((__PLLI2SQ__) shl 24) or ((__PLLI2SR__) shl 28);
end;

(** @brief  Macro to configure the SAI clock Divider coming from PLLI2S.
  * @note   This function must be called before enabling the PLLI2S.
  * @param  __PLLI2SDivQ__: specifies the PLLI2S division factor for SAI1 clock .
  *          This parameter must be a number between 1 and 32.
  *          SAI1 clock frequency := f(PLLI2SQ) / __PLLI2SDivQ__
  *)
procedure __HAL_RCC_PLLI2S_PLLSAICLKDIVQ_CONFIG(__PLLI2SDivQ__: longword);
begin
  RCC.DCKCFGR1 := (RCC.DCKCFGR1 and (not longword(RCC_DCKCFGR1_PLLI2SDIVQ))) or (__PLLI2SDivQ__) - 1;
end;

(** @brief  Macro to configure the SAI clock Divider coming from PLLSAI.
  * @note   This function must be called before enabling the PLLSAI.
  * @param  __PLLSAIDivQ__: specifies the PLLSAI division factor for SAI1 clock .
  *         This parameter must be a number between Min_Data := 1 and Max_Data := 32.
  *         SAI1 clock frequency := f(PLLSAIQ) / __PLLSAIDivQ__
  *)
procedure __HAL_RCC_PLLSAI_PLLSAICLKDIVQ_CONFIG(__PLLSAIDivQ__: longword);
begin
  RCC.DCKCFGR1 := (RCC.DCKCFGR1 and (not longword(RCC_DCKCFGR1_PLLSAIDIVQ))) or ((__PLLSAIDivQ__) - 1) shl 8;
end;

(** @brief  Macro to configure the LTDC clock Divider coming from PLLSAI.
  *
  * @note   This function must be called before enabling the PLLSAI.
  * @param  __PLLSAIDivR__: specifies the PLLSAI division factor for LTDC clock .
  *          This parameter must be a number between Min_Data := 2 and Max_Data := 16.
  *          LTDC clock frequency := f(PLLSAIR) / __PLLSAIDivR__
  *)
procedure __HAL_RCC_PLLSAI_PLLSAICLKDIVR_CONFIG(__PLLSAIDivR__: longword);
begin

  RCC.DCKCFGR1 := (RCC.DCKCFGR1 and (not longword(RCC_DCKCFGR1_PLLSAIDIVR))) or (__PLLSAIDivR__);
end;

(** @brief  Macro to configure SAI1 clock source selection.
  * @note   This function must be called before enabling PLLSAI, PLLI2S and
  *         the SAI clock.
  * @param  __SOURCE__: specifies the SAI1 clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_SAI1CLKSOURCE_PLLI2S: PLLI2S_Q clock divided by PLLI2SDIVQ used
  *                                           as SAI1 clock.
  *            @arg RCC_SAI1CLKSOURCE_PLLSAI: PLLISAI_Q clock divided by PLLSAIDIVQ used
  *                                           as SAI1 clock.
  *            @arg RCC_SAI1CLKSOURCE_PIN: External clock mapped on the I2S_CKIN pin
  *                                        used as SAI1 clock.
  *)
procedure __HAL_RCC_SAI1_CONFIG(__SOURCE__: longword);
begin

  RCC.DCKCFGR1 := (RCC.DCKCFGR1 and (not longword(RCC_DCKCFGR1_SAI1SEL))) or (__SOURCE__);
end;

(** @brief  Macro to get the SAI1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_SAI1CLKSOURCE_PLLI2S: PLLI2S_Q clock divided by PLLI2SDIVQ used
  *                                           as SAI1 clock.
  *            @arg RCC_SAI1CLKSOURCE_PLLSAI: PLLISAI_Q clock divided by PLLSAIDIVQ used
  *                                           as SAI1 clock.
  *            @arg RCC_SAI1CLKSOURCE_PIN: External clock mapped on the I2S_CKIN pin
  *                                        used as SAI1 clock.
  *)
function __HAL_RCC_GET_SAI1_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR1 and RCC_DCKCFGR1_SAI1SEL))));
end;


(** @brief  Macro to configure SAI2 clock source selection.
  * @note   This function must be called before enabling PLLSAI, PLLI2S and
  *         the SAI clock.
  * @param  __SOURCE__: specifies the SAI2 clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_SAI2CLKSOURCE_PLLI2S: PLLI2S_Q clock divided by PLLI2SDIVQ used
  *                                           as SAI2 clock.
  *            @arg RCC_SAI2CLKSOURCE_PLLSAI: PLLISAI_Q clock divided by PLLSAIDIVQ used
  *                                           as SAI2 clock.
  *            @arg RCC_SAI2CLKSOURCE_PIN: External clock mapped on the I2S_CKIN pin
  *                                        used as SAI2 clock.
  *)
procedure __HAL_RCC_SAI2_CONFIG(__SOURCE__: longword);
begin

  RCC.DCKCFGR1 := (RCC.DCKCFGR1 and (not longword(RCC_DCKCFGR1_SAI2SEL))) or (__SOURCE__);
end;


(** @brief  Macro to get the SAI2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_SAI2CLKSOURCE_PLLI2S: PLLI2S_Q clock divided by PLLI2SDIVQ used
  *                                           as SAI2 clock.
  *            @arg RCC_SAI2CLKSOURCE_PLLSAI: PLLISAI_Q clock divided by PLLSAIDIVQ used
  *                                           as SAI2 clock.
  *            @arg RCC_SAI2CLKSOURCE_PIN: External clock mapped on the I2S_CKIN pin
  *                                        used as SAI2 clock.
  *)
function __HAL_RCC_GET_SAI2_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR1 and RCC_DCKCFGR1_SAI2SEL))));
end;


(** @brief Enable PLLSAI_RDY interrupt.
  *)
procedure __HAL_RCC_PLLSAI_ENABLE_IT;
begin
  RCC.CIR := RCC.CIR or (RCC_CIR_PLLSAIRDYIE);
end;

(** @brief Disable PLLSAI_RDY interrupt.
  *)
procedure __HAL_RCC_PLLSAI_DISABLE_IT;
begin
  RCC.CIR := RCC.CIR and (not (RCC_CIR_PLLSAIRDYIE));
end;

(** @brief Clear the PLLSAI RDY interrupt pending bits.
  *)
procedure __HAL_RCC_PLLSAI_CLEAR_IT;
begin
  RCC.CIR := RCC.CIR or (RCC_CIR_PLLSAIRDYF);
end;

(** @brief Check the PLLSAI RDY interrupt has occurred or not.
  * @retval The new state (TRUE or FALSE).
  *)
function __HAL_RCC_PLLSAI_GET_IT: boolean;
begin
  exit(((RCC.CIR and (RCC_CIR_PLLSAIRDYIE)) = (RCC_CIR_PLLSAIRDYIE)));
end;

(** @brief  Check PLLSAI RDY flag is set or not.
  * @retval The new state (TRUE or FALSE).
  *)
function __HAL_RCC_PLLSAI_GET_FLAG: boolean;
begin
  exit(((RCC.CR and (RCC_CR_PLLSAIRDY)) = (RCC_CR_PLLSAIRDY)));
end;

(** @brief  Macro to Get I2S clock source selection.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2SCLKSOURCE_PLLI2S: PLLI2S VCO output clock divided by PLLI2SR used as I2S clock.
  *            @arg RCC_I2SCLKSOURCE_EXT: External clock mapped on the I2S_CKIN pin used as I2S clock source
  *)
function __HAL_RCC_GET_I2SCLKSOURCE: longword;
begin
  exit(((RCC.CFGR and RCC_CFGR_I2SSRC)));
end;

(** @brief  Macro to configure the I2C1 clock (I2C1CLK).
  *
  * @param  __I2C1_CLKSOURCE__: specifies the I2C1 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2C1CLKSOURCE_PCLK1: PCLK1 selected as I2C1 clock
  *            @arg RCC_I2C1CLKSOURCE_HSI: HSI selected as I2C1 clock
  *            @arg RCC_I2C1CLKSOURCE_SYSCLK: System Clock selected as I2C1 clock
  *)
procedure __HAL_RCC_I2C1_CONFIG(__I2C1_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_I2C1SEL))) or (__I2C1_CLKSOURCE__);
end;

(** @brief  Macro to get the I2C1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2C1CLKSOURCE_PCLK1: PCLK1 selected as I2C1 clock
  *            @arg RCC_I2C1CLKSOURCE_HSI: HSI selected as I2C1 clock
  *            @arg RCC_I2C1CLKSOURCE_SYSCLK: System Clock selected as I2C1 clock
  *)
function __HAL_RCC_GET_I2C1_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_I2C1SEL))));
end;

(** @brief  Macro to configure the I2C2 clock (I2C2CLK).
  *
  * @param  __I2C2_CLKSOURCE__: specifies the I2C2 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2C2CLKSOURCE_PCLK1: PCLK1 selected as I2C2 clock
  *            @arg RCC_I2C2CLKSOURCE_HSI: HSI selected as I2C2 clock
  *            @arg RCC_I2C2CLKSOURCE_SYSCLK: System Clock selected as I2C2 clock
  *)
procedure __HAL_RCC_I2C2_CONFIG(__I2C2_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_I2C2SEL))) or (__I2C2_CLKSOURCE__);
end;

(** @brief  Macro to get the I2C2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2C2CLKSOURCE_PCLK1: PCLK1 selected as I2C2 clock
  *            @arg RCC_I2C2CLKSOURCE_HSI: HSI selected as I2C2 clock
  *            @arg RCC_I2C2CLKSOURCE_SYSCLK: System Clock selected as I2C2 clock
  *)
function __HAL_RCC_GET_I2C2_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_I2C2SEL))));
end;

(** @brief  Macro to configure the I2C3 clock (I2C3CLK).
  *
  * @param  __I2C3_CLKSOURCE__: specifies the I2C3 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2C3CLKSOURCE_PCLK1: PCLK1 selected as I2C3 clock
  *            @arg RCC_I2C3CLKSOURCE_HSI: HSI selected as I2C3 clock
  *            @arg RCC_I2C3CLKSOURCE_SYSCLK: System Clock selected as I2C3 clock
  *)
procedure __HAL_RCC_I2C3_CONFIG(__I2C3_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_I2C3SEL))) or (__I2C3_CLKSOURCE__);
end;

(** @brief  macro to get the I2C3 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2C3CLKSOURCE_PCLK1: PCLK1 selected as I2C3 clock
  *            @arg RCC_I2C3CLKSOURCE_HSI: HSI selected as I2C3 clock
  *            @arg RCC_I2C3CLKSOURCE_SYSCLK: System Clock selected as I2C3 clock
  *)
function __HAL_RCC_GET_I2C3_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_I2C3SEL))));
end;

(** @brief  Macro to configure the I2C4 clock (I2C4CLK).
  *
  * @param  __I2C4_CLKSOURCE__: specifies the I2C4 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2C4CLKSOURCE_PCLK1: PCLK1 selected as I2C4 clock
  *            @arg RCC_I2C4CLKSOURCE_HSI: HSI selected as I2C4 clock
  *            @arg RCC_I2C4CLKSOURCE_SYSCLK: System Clock selected as I2C4 clock
  *)
procedure __HAL_RCC_I2C4_CONFIG(__I2C4_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_I2C4SEL))) or (__I2C4_CLKSOURCE__);
end;

(** @brief  macro to get the I2C4 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2C4CLKSOURCE_PCLK1: PCLK1 selected as I2C4 clock
  *            @arg RCC_I2C4CLKSOURCE_HSI: HSI selected as I2C4 clock
  *            @arg RCC_I2C4CLKSOURCE_SYSCLK: System Clock selected as I2C4 clock
  *)
function __HAL_RCC_GET_I2C4_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_I2C4SEL))));
end;

(** @brief  Macro to configure the USART1 clock (USART1CLK).
  *
  * @param  __USART1_CLKSOURCE__: specifies the USART1 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_USART1CLKSOURCE_PCLK2: PCLK2 selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_HSI: HSI selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_SYSCLK: System Clock selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_LSE: LSE selected as USART1 clock
  *)
procedure __HAL_RCC_USART1_CONFIG(__USART1_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_USART1SEL))) or (__USART1_CLKSOURCE__);
end;

(** @brief  macro to get the USART1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_USART1CLKSOURCE_PCLK2: PCLK2 selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_HSI: HSI selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_SYSCLK: System Clock selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_LSE: LSE selected as USART1 clock
  *)
function __HAL_RCC_GET_USART1_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_USART1SEL))));
end;

(** @brief  Macro to configure the USART2 clock (USART2CLK).
  *
  * @param  __USART2_CLKSOURCE__: specifies the USART2 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_USART2CLKSOURCE_PCLK1: PCLK1 selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_HSI: HSI selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_SYSCLK: System Clock selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_LSE: LSE selected as USART2 clock
  *)
procedure __HAL_RCC_USART2_CONFIG(__USART2_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_USART2SEL))) or (__USART2_CLKSOURCE__);
end;

(** @brief  macro to get the USART2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_USART2CLKSOURCE_PCLK1: PCLK1 selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_HSI: HSI selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_SYSCLK: System Clock selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_LSE: LSE selected as USART2 clock
  *)
function __HAL_RCC_GET_USART2_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_USART2SEL))));
end;

(** @brief  Macro to configure the USART3 clock (USART3CLK).
  *
  * @param  __USART3_CLKSOURCE__: specifies the USART3 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_USART3CLKSOURCE_PCLK1: PCLK1 selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_HSI: HSI selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_SYSCLK: System Clock selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_LSE: LSE selected as USART3 clock
  *)
procedure __HAL_RCC_USART3_CONFIG(__USART3_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_USART3SEL))) or (__USART3_CLKSOURCE__);
end;

(** @brief  macro to get the USART3 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_USART3CLKSOURCE_PCLK1: PCLK1 selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_HSI: HSI selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_SYSCLK: System Clock selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_LSE: LSE selected as USART3 clock
  *)
function __HAL_RCC_GET_USART3_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_USART3SEL))));
end;

 (** @brief  Macro to configure the UART4 clock (UART4CLK).
  *
  * @param  __UART4_CLKSOURCE__: specifies the UART4 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_UART4CLKSOURCE_PCLK1: PCLK1 selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_HSI: HSI selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_SYSCLK: System Clock selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_LSE: LSE selected as UART4 clock
  *)
procedure __HAL_RCC_UART4_CONFIG(__UART4_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_UART4SEL))) or (__UART4_CLKSOURCE__);
end;

(** @brief  macro to get the UART4 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_UART4CLKSOURCE_PCLK1: PCLK1 selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_HSI: HSI selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_SYSCLK: System Clock selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_LSE: LSE selected as UART4 clock
  *)
function __HAL_RCC_GET_UART4_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_UART4SEL))));
end;

 (** @brief  Macro to configure the UART5 clock (UART5CLK).
  *
  * @param  __UART5_CLKSOURCE__: specifies the UART5 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_UART5CLKSOURCE_PCLK1: PCLK1 selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_HSI: HSI selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_SYSCLK: System Clock selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_LSE: LSE selected as UART5 clock
  *)
procedure __HAL_RCC_UART5_CONFIG(__UART5_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_UART5SEL))) or (__UART5_CLKSOURCE__);
end;

(** @brief  macro to get the UART5 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_UART5CLKSOURCE_PCLK1: PCLK1 selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_HSI: HSI selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_SYSCLK: System Clock selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_LSE: LSE selected as UART5 clock
  *)
function __HAL_RCC_GET_UART5_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_UART5SEL))));
end;

 (** @brief  Macro to configure the USART6 clock (USART6CLK).
  *
  * @param  __USART6_CLKSOURCE__: specifies the USART6 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_USART6CLKSOURCE_PCLK1: PCLK1 selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_HSI: HSI selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_SYSCLK: System Clock selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_LSE: LSE selected as USART6 clock
  *)
procedure __HAL_RCC_USART6_CONFIG(__USART6_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_USART6SEL))) or (__USART6_CLKSOURCE__);
end;

(** @brief  macro to get the USART6 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_USART6CLKSOURCE_PCLK1: PCLK1 selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_HSI: HSI selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_SYSCLK: System Clock selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_LSE: LSE selected as USART6 clock
  *)
function __HAL_RCC_GET_USART6_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_USART6SEL))));
end;

 (** @brief  Macro to configure the UART7 clock (UART7CLK).
  *
  * @param  __UART7_CLKSOURCE__: specifies the UART7 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_UART7CLKSOURCE_PCLK1: PCLK1 selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_HSI: HSI selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_SYSCLK: System Clock selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_LSE: LSE selected as UART7 clock
  *)
procedure __HAL_RCC_UART7_CONFIG(__UART7_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_UART7SEL))) or (__UART7_CLKSOURCE__);
end;

(** @brief  macro to get the UART7 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_UART7CLKSOURCE_PCLK1: PCLK1 selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_HSI: HSI selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_SYSCLK: System Clock selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_LSE: LSE selected as UART7 clock
  *)
function __HAL_RCC_GET_UART7_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_UART7SEL))));
end;

(** @brief  Macro to configure the UART8 clock (UART8CLK).
  *
  * @param  __UART8_CLKSOURCE__: specifies the UART8 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_UART8CLKSOURCE_PCLK1: PCLK1 selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_HSI: HSI selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_SYSCLK: System Clock selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_LSE: LSE selected as UART8 clock
  *)
procedure __HAL_RCC_UART8_CONFIG(__UART8_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_UART8SEL))) or (__UART8_CLKSOURCE__);
end;

(** @brief  macro to get the UART8 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_UART8CLKSOURCE_PCLK1: PCLK1 selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_HSI: HSI selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_SYSCLK: System Clock selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_LSE: LSE selected as UART8 clock
  *)
function __HAL_RCC_GET_UART8_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_UART8SEL))));
end;

(** @brief  Macro to configure the LPTIM1 clock (LPTIM1CLK).
  *
  * @param  __LPTIM1_CLKSOURCE__: specifies the LPTIM1 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_LPTIM1CLKSOURCE_PCLK: PCLK selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_HSI: HSI selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_LSI: LSI selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_LSE: LSE selected as LPTIM1 clock
  *)
procedure __HAL_RCC_LPTIM1_CONFIG(__LPTIM1_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_LPTIM1SEL))) or (__LPTIM1_CLKSOURCE__);
end;

(** @brief  macro to get the LPTIM1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_LPTIM1CLKSOURCE_PCLK: PCLK selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_HSI: HSI selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_LSI: LSI selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_LSE: LSE selected as LPTIM1 clock
  *)
function __HAL_RCC_GET_LPTIM1_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_LPTIM1SEL))));
end;

(** @brief  Macro to configure the CEC clock (CECCLK).
  *
  * @param  __CEC_CLKSOURCE__: specifies the CEC clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_CECCLKSOURCE_LSE: LSE selected as CEC clock
  *            @arg RCC_CECCLKSOURCE_HSI: HSI selected as CEC clock
  *)
procedure __HAL_RCC_CEC_CONFIG(__CEC_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_CECSEL))) or (__CEC_CLKSOURCE__);
end;

(** @brief  macro to get the CEC clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_CECCLKSOURCE_LSE: LSE selected as CEC clock
  *            @arg RCC_CECCLKSOURCE_HSI: HSI selected as CEC clock
  *)
function __HAL_RCC_GET_CEC_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_CECSEL))));
end;

(** @brief  Macro to configure the CLK48 source (CLK48CLK).
  *
  * @param  __CLK48_SOURCE__: specifies the CLK48 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_CLK48SOURCE_PLL: PLL selected as CLK48 source
  *            @arg RCC_CLK48SOURCE_PLSAI1: PLLSAI1 selected as CLK48 source
  *)
procedure __HAL_RCC_CLK48_CONFIG(__CLK48_SOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_CK48MSEL))) or (__CLK48_SOURCE__);
end;

(** @brief  macro to get the CLK48 source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_CLK48SOURCE_PLL: PLL used as CLK48 source
  *            @arg RCC_CLK48SOURCE_PLSAI1: PLLSAI1 used as CLK48 source
  *)
function __HAL_RCC_GET_CLK48_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_CK48MSEL))));
end;

(** @brief  Macro to configure the SDMMC1 clock (SDMMC1CLK).
  *
  * @param  __SDMMC1_CLKSOURCE__: specifies the SDMMC1 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_SDMMC1CLKSOURCE_CLK48: CLK48 selected as SDMMC clock
  *            @arg RCC_SDMMC1CLKSOURCE_SYSCLK: SYSCLK selected as SDMMC clock
  *)
procedure __HAL_RCC_SDMMC1_CONFIG(__SDMMC1_CLKSOURCE__: longword);
begin

  RCC.DCKCFGR2 := (RCC.DCKCFGR2 and (not longword(RCC_DCKCFGR2_SDMMC1SEL))) or (__SDMMC1_CLKSOURCE__);
end;

(** @brief  macro to get the SDMMC1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_SDMMC1CLKSOURCE_CLK48: CLK48 selected as SDMMC1 clock
  *            @arg RCC_SDMMC1CLKSOURCE_SYSCLK: SYSCLK selected as SDMMC1 clock
  *)
function __HAL_RCC_GET_SDMMC1_SOURCE: longword;
begin
  exit((((RCC.DCKCFGR2 and RCC_DCKCFGR2_SDMMC1SEL))));
end;

function HAL_RCCEx_PeriphCLKConfig(const PeriphClkInit: RCC_PeriphCLKInitTypeDef): HAL_StatusTypeDef;
var
  tickstart, tmpreg0, tmpreg1, plli2sused, pllsaiused: longword;
begin
  tickstart := 0;
  tmpreg0 := 0;
  tmpreg1 := 0;
  plli2sused := 0;
  pllsaiused := 0;

  (*----------------------------------- I2S configuration ----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_I2S) = (RCC_PERIPHCLK_I2S)) then
  begin
    (* Configure I2S Clock source *)
    __HAL_RCC_I2S_CONFIG(PeriphClkInit.I2sClockSelection);

    (* Enable the PLLI2S when it's used as clock source for I2S *)
    if (PeriphClkInit.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S) then
    begin
      plli2sused := 1;
    end;
  end;

  (*------------------------------------ SAI1 configuration --------------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SAI1) = (RCC_PERIPHCLK_SAI1)) then
  begin
    (* Configure SAI1 Clock source *)
    __HAL_RCC_SAI1_CONFIG(PeriphClkInit.Sai1ClockSelection);
    (* Enable the PLLI2S when it's used as clock source for SAI *)
    if (PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S) then
    begin
      plli2sused := 1;
    end;
    (* Enable the PLLSAI when it's used as clock source for SAI *)
    if (PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI) then
    begin
      pllsaiused := 1;
    end;
  end;

  (*------------------------------------ SAI2 configuration --------------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SAI2) = (RCC_PERIPHCLK_SAI2)) then
  begin
    (* Configure SAI2 Clock source *)
    __HAL_RCC_SAI2_CONFIG(PeriphClkInit.Sai2ClockSelection);

    (* Enable the PLLI2S when it's used as clock source for SAI *)
    if (PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S) then
    begin
      plli2sused := 1;
    end;
    (* Enable the PLLSAI when it's used as clock source for SAI *)
    if (PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI) then
    begin
      pllsaiused := 1;
    end;
  end;

  (*-------------------------------------- SPDIF-RX Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SPDIFRX) = RCC_PERIPHCLK_SPDIFRX) then
  begin
    plli2sused := 1;
  end;

  (*------------------------------------ RTC configuration --------------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_RTC) = (RCC_PERIPHCLK_RTC)) then
  begin
    (* Reset the Backup domain only if the RTC Clock source selection is modified *)
    if ((RCC.BDCR and RCC_BDCR_RTCSEL) <> (PeriphClkInit.RTCClockSelection and RCC_BDCR_RTCSEL)) then
    begin
      (* Enable Power Clock*)
      __HAL_RCC_PWR_CLK_ENABLE();

      (* Enable write access to Backup domain *)
      PWR.CR1 := PWR.CR1 or PWR_CR1_DBP;

      (* Get Start Tick*)
      tickstart := HAL_GetTick();

      (* Wait for Backup domain Write protection disable *)
      while ((PWR.CR1 and PWR_CR1_DBP) = 0) do
      begin
        if ((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE) then
        begin
          exit(HAL_TIMEOUT);
        end;
      end;

      (* Store the content of BDCR register before the false of Backup Domain *)
      tmpreg0 := (RCC.BDCR and not (RCC_BDCR_RTCSEL));

      (* RTC Clock selection can be changed only if the Backup Domain is false *)
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();

      (* Restore the Content of BDCR register *)
      RCC.BDCR := tmpreg0;

      (* If LSE is selected as RTC clock source, wait for LSE reactivation *)
      if (HAL_IS_BIT_SET(tmpreg0, RCC_BDCR_LSERDY)) then
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
      end;
      __HAL_RCC_RTC_CONFIG(PeriphClkInit.RTCClockSelection);
    end;
  end;

  (*------------------------------------ TIM configuration --------------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_TIM) = (RCC_PERIPHCLK_TIM)) then
  begin
    (* Configure Timer Prescaler *)
    __HAL_RCC_TIMCLKPRESCALER(PeriphClkInit.TIMPresSelection);
  end;

  (*-------------------------------------- I2C1 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_I2C1) = RCC_PERIPHCLK_I2C1) then
  begin
    (* Configure the I2C1 clock source *)
    __HAL_RCC_I2C1_CONFIG(PeriphClkInit.I2c1ClockSelection);
  end;

  (*-------------------------------------- I2C2 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_I2C2) = RCC_PERIPHCLK_I2C2) then
  begin
    (* Configure the I2C2 clock source *)
    __HAL_RCC_I2C2_CONFIG(PeriphClkInit.I2c2ClockSelection);
  end;

  (*-------------------------------------- I2C3 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_I2C3) = RCC_PERIPHCLK_I2C3) then
  begin
    (* Configure the I2C3 clock source *)
    __HAL_RCC_I2C3_CONFIG(PeriphClkInit.I2c3ClockSelection);
  end;

  (*-------------------------------------- I2C4 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_I2C4) = RCC_PERIPHCLK_I2C4) then
  begin
    (* Configure the I2C4 clock source *)
    __HAL_RCC_I2C4_CONFIG(PeriphClkInit.I2c4ClockSelection);
  end;

  (*-------------------------------------- USART1 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_USART1) = RCC_PERIPHCLK_USART1) then
  begin
    (* Configure the USART1 clock source *)
    __HAL_RCC_USART1_CONFIG(PeriphClkInit.Usart1ClockSelection);
  end;

  (*-------------------------------------- USART2 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_USART2) = RCC_PERIPHCLK_USART2) then
  begin
    (* Configure the USART2 clock source *)
    __HAL_RCC_USART2_CONFIG(PeriphClkInit.Usart2ClockSelection);
  end;

  (*-------------------------------------- USART3 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_USART3) = RCC_PERIPHCLK_USART3) then
  begin
    (* Configure the USART3 clock source *)
    __HAL_RCC_USART3_CONFIG(PeriphClkInit.Usart3ClockSelection);
  end;

  (*-------------------------------------- UART4 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_UART4) = RCC_PERIPHCLK_UART4) then
  begin
    (* Configure the UART4 clock source *)
    __HAL_RCC_UART4_CONFIG(PeriphClkInit.Uart4ClockSelection);
  end;

  (*-------------------------------------- UART5 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_UART5) = RCC_PERIPHCLK_UART5) then
  begin
    (* Configure the UART5 clock source *)
    __HAL_RCC_UART5_CONFIG(PeriphClkInit.Uart5ClockSelection);
  end;

  (*-------------------------------------- USART6 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_USART6) = RCC_PERIPHCLK_USART6) then
  begin
    (* Configure the USART6 clock source *)
    __HAL_RCC_USART6_CONFIG(PeriphClkInit.Usart6ClockSelection);
  end;

  (*-------------------------------------- UART7 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_UART7) = RCC_PERIPHCLK_UART7) then
  begin
    (* Configure the UART7 clock source *)
    __HAL_RCC_UART7_CONFIG(PeriphClkInit.Uart7ClockSelection);
  end;

  (*-------------------------------------- UART8 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_UART8) = RCC_PERIPHCLK_UART8) then
  begin
    (* Configure the UART8 clock source *)
    __HAL_RCC_UART8_CONFIG(PeriphClkInit.Uart8ClockSelection);
  end;

  (*--------------------------------------- CEC Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_CEC) = RCC_PERIPHCLK_CEC) then
  begin
    (* Configure the CEC clock source *)
    __HAL_RCC_CEC_CONFIG(PeriphClkInit.CecClockSelection);
  end;

  (*-------------------------------------- CK48 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_CLK48) = RCC_PERIPHCLK_CLK48) then
  begin
    (* Configure the CLK48 source *)
    __HAL_RCC_CLK48_CONFIG(PeriphClkInit.Clk48ClockSelection);

    (* Enable the PLLSAI when it's used as clock source for CK48 *)
    if (PeriphClkInit.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP) then
    begin
      pllsaiused := 1;
    end;
  end;

  (*-------------------------------------- LTDC Configuration -----------------------------------*)
{$if defined(STM32F756xx) or defined(STM32F746xx)}
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_LTDC) = RCC_PERIPHCLK_LTDC) then
  begin
    pllsaiused := 1;
  end;
{$endif}
  (*-------------------------------------- LPTIM1 Configuration -----------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_LPTIM1) = RCC_PERIPHCLK_LPTIM1) then
  begin
    (* Configure the LTPIM1 clock source *)
    __HAL_RCC_LPTIM1_CONFIG(PeriphClkInit.Lptim1ClockSelection);
  end;

  (*------------------------------------- SDMMC Configuration ------------------------------------*)
  if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SDMMC1) = RCC_PERIPHCLK_SDMMC1) then
  begin
    (* Configure the SDMMC1 clock source *)
    __HAL_RCC_SDMMC1_CONFIG(PeriphClkInit.Sdmmc1ClockSelection);
  end;

  (*-------------------------------------- PLLI2S Configuration ---------------------------------*)
  (* PLLI2S is configured when a peripheral will use it as source clock : SAI1, SAI2, I2S or SPDIF-RX *)
  if ((plli2sused = 1) or (PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_PLLI2S)) then
  begin
    (* Disable the PLLI2S *)
    __HAL_RCC_PLLI2S_DISABLE();

    (* Get Start Tick*)
    tickstart := HAL_GetTick();

    (* Wait till PLLI2S is disabled *)
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY) <> False) do
    begin
      if ((HAL_GetTick() - tickstart) > PLLI2S_TIMEOUT_VALUE) then
      begin
        (* return in case of Timeout detected *)
        exit(HAL_TIMEOUT);
      end;
    end;

    (*----------------- In Case of PLLI2S is selected as source clock for I2S -------------------*)
    if (((((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_I2S) = RCC_PERIPHCLK_I2S) and (PeriphClkInit.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S))) then
    begin
      (* Read PLLI2SP and PLLI2SQ value from PLLI2SCFGR register (this value is not needed for I2S configuration) *)
      tmpreg0 := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SP) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SP));
      tmpreg1 := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SQ) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SQ));
      (* Configure the PLLI2S division factors *)
      (* PLLI2S_VCO := f(VCO clock) := f(PLLI2S clock input) x (PLLI2SN/PLLM) *)
      (* I2SCLK := f(PLLI2S clock output) := f(VCO clock) / PLLI2SR *)
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit.PLLI2S.PLLI2SN, tmpreg0, tmpreg1, PeriphClkInit.PLLI2S.PLLI2SR);
    end;

    (*----------------- In Case of PLLI2S is selected as source clock for SAI -------------------*)
    if (((((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SAI1) = RCC_PERIPHCLK_SAI1) and (PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S)) or
      ((((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SAI2) = RCC_PERIPHCLK_SAI2) and (PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S))) then
    begin
      (* Read PLLI2SP and PLLI2SR values from PLLI2SCFGR register (this value is not needed for SAI configuration) *)
      tmpreg0 := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SP) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SP));
      tmpreg1 := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SR) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SR));
      (* Configure the PLLI2S division factors *)
      (* PLLI2S_VCO Input  := PLL_SOURCE/PLLM *)
      (* PLLI2S_VCO Output := PLLI2S_VCO Input * PLLI2SN *)
      (* SAI_CLK(first level) := PLLI2S_VCO Output/PLLI2SQ *)
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit.PLLI2S.PLLI2SN, tmpreg0, PeriphClkInit.PLLI2S.PLLI2SQ, tmpreg1);

      (* SAI_CLK_x := SAI_CLK(first level)/PLLI2SDIVQ *)
      __HAL_RCC_PLLI2S_PLLSAICLKDIVQ_CONFIG(PeriphClkInit.PLLI2SDivQ);
    end;

    (*----------------- In Case of PLLI2S is selected as source clock for SPDIF-RX -------------------*)
    if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SPDIFRX) = RCC_PERIPHCLK_SPDIFRX) then
    begin
      (* Read PLLI2SR value from PLLI2SCFGR register (this value is not needed for SPDIF-RX configuration) *)
      tmpreg0 := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SQ) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SQ));
      tmpreg1 := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SR) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SR));
      (* Configure the PLLI2S division factors *)
      (* PLLI2S_VCO := f(VCO clock) := f(PLLI2S clock input) x (PLLI2SN/PLLM) *)
      (* SPDIFCLK := f(PLLI2S clock output) := f(VCO clock) / PLLI2SP *)
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit.PLLI2S.PLLI2SN, PeriphClkInit.PLLI2S.PLLI2SP, tmpreg0, tmpreg1);
    end;

    (*----------------- In Case of PLLI2S is just selected  -----------------*)
    if ((PeriphClkInit.PeriphClockSelection and RCC_PERIPHCLK_PLLI2S) = RCC_PERIPHCLK_PLLI2S) then
    begin
      (* Configure the PLLI2S division factors *)
      (* PLLI2S_VCO := f(VCO clock) := f(PLLI2S clock input) x (PLLI2SN/PLLI2SM) *)
      (* SPDIFRXCLK := f(PLLI2S clock output) := f(VCO clock) / PLLI2SP *)
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit.PLLI2S.PLLI2SN, PeriphClkInit.PLLI2S.PLLI2SP, PeriphClkInit.PLLI2S.PLLI2SQ, PeriphClkInit.PLLI2S.PLLI2SR);
    end;

    (* Enable the PLLI2S *)
    __HAL_RCC_PLLI2S_ENABLE();

    (* Get Start Tick*)
    tickstart := HAL_GetTick();

    (* Wait till PLLI2S is ready *)
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY) = False) do
    begin
      if ((HAL_GetTick() - tickstart) > PLLI2S_TIMEOUT_VALUE) then
      begin
        (* return in case of Timeout detected *)
        exit(HAL_TIMEOUT);
      end;
    end;
  end;

  (*-------------------------------------- PLLSAI Configuration ---------------------------------*)
  (* PLLSAI is configured when a peripheral will use it as source clock : SAI1, SAI2, LTDC or CK48 *)
  if (pllsaiused = 1) then
  begin
    (* Disable PLLSAI Clock *)
    __HAL_RCC_PLLSAI_DISABLE();

    (* Get Start Tick*)
    tickstart := HAL_GetTick();

    (* Wait till PLLSAI is disabled *)
    while (__HAL_RCC_PLLSAI_GET_FLAG() <> False) do
    begin
      if ((HAL_GetTick() - tickstart) > PLLSAI_TIMEOUT_VALUE) then
      begin
        (* return in case of Timeout detected *)
        exit(HAL_TIMEOUT);
      end;
    end;

    (*----------------- In Case of PLLSAI is selected as source clock for SAI -------------------*)
    if (((((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SAI1) = RCC_PERIPHCLK_SAI1) and (PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI)) or
      ((((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_SAI2) = RCC_PERIPHCLK_SAI2) and (PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI))) then
    begin
      (* Read PLLSAIP value from PLLSAICFGR register (this value is not needed for SAI configuration) *)
      tmpreg0 := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIP) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIP));
      tmpreg1 := ((RCC.PLLSAICFGR and RCC_PLLI2SCFGR_PLLI2SR) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIR));
      (* PLLSAI_VCO Input  := PLL_SOURCE/PLLM *)
      (* PLLSAI_VCO Output := PLLSAI_VCO Input * PLLSAIN *)
      (* SAI_CLK(first level) := PLLSAI_VCO Output/PLLSAIQ *)
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit.PLLSAI.PLLSAIN, tmpreg0, PeriphClkInit.PLLSAI.PLLSAIQ, tmpreg1);

      (* SAI_CLK_x := SAI_CLK(first level)/PLLSAIDIVQ *)
      __HAL_RCC_PLLSAI_PLLSAICLKDIVQ_CONFIG(PeriphClkInit.PLLSAIDivQ);
    end;

    (*----------------- In Case of PLLSAI is selected as source clock for CLK48 -------------------*)
    (* In Case of PLLI2S is selected as source clock for CK48 *)
    if ((((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_CLK48) = RCC_PERIPHCLK_CLK48) and (PeriphClkInit.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP)) then
    begin
      (* Read PLLSAIQ and PLLSAIR value from PLLSAICFGR register (this value is not needed for CK48 configuration) *)
      tmpreg0 := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIQ) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIQ));
      tmpreg1 := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIR) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIR));

      (* Configure the PLLSAI division factors *)
      (* PLLSAI_VCO := f(VCO clock) := f(PLLSAI clock input) x (PLLI2SN/PLLM) *)
      (* 48CLK := f(PLLSAI clock output) := f(VCO clock) / PLLSAIP *)
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit.PLLSAI.PLLSAIN, PeriphClkInit.PLLSAI.PLLSAIP, tmpreg0, tmpreg1);
    end;

{$if defined(STM32F756xx) or defined(STM32F746xx)}
    (*---------------------------- LTDC configuration -------------------------------*)
    if (((PeriphClkInit.PeriphClockSelection) and RCC_PERIPHCLK_LTDC) = (RCC_PERIPHCLK_LTDC)) then
    begin
      (* Read PLLSAIP and PLLSAIQ value from PLLSAICFGR register (these value are not needed for LTDC configuration) *)
      tmpreg0 := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIQ) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIQ));
      tmpreg1 := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIP) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIP));

      (* PLLSAI_VCO Input  := PLL_SOURCE/PLLM *)
      (* PLLSAI_VCO Output := PLLSAI_VCO Input * PLLSAIN *)
      (* LTDC_CLK(first level) := PLLSAI_VCO Output/PLLSAIR *)
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit.PLLSAI.PLLSAIN, tmpreg1, tmpreg0, PeriphClkInit.PLLSAI.PLLSAIR);

      (* LTDC_CLK := LTDC_CLK(first level)/PLLSAIDIVR *)
      __HAL_RCC_PLLSAI_PLLSAICLKDIVR_CONFIG(PeriphClkInit.PLLSAIDivR);
    end;
{$endif}

    (* Enable PLLSAI Clock *)
    __HAL_RCC_PLLSAI_ENABLE();

    (* Get Start Tick*)
    tickstart := HAL_GetTick();

    (* Wait till PLLSAI is ready *)
    while (__HAL_RCC_PLLSAI_GET_FLAG() = False) do
    begin
      if ((HAL_GetTick() - tickstart) > PLLSAI_TIMEOUT_VALUE) then
      begin
        (* return in case of Timeout detected *)
        exit(HAL_TIMEOUT);
      end;
    end;
  end;
  exit(HAL_OK);
end;

procedure HAL_RCCEx_GetPeriphCLKConfig(var PeriphClkInit: RCC_PeriphCLKInitTypeDef);
var
  tempreg: longword;
begin
  tempreg := 0;

  (* Set all possible values for the extended clock type parameter------------*)
  PeriphClkInit.PeriphClockSelection := RCC_PERIPHCLK_I2S or RCC_PERIPHCLK_LPTIM1 or RCC_PERIPHCLK_SAI1 or RCC_PERIPHCLK_SAI2 or
    RCC_PERIPHCLK_TIM or RCC_PERIPHCLK_RTC or RCC_PERIPHCLK_CEC or RCC_PERIPHCLK_I2C4 or
    RCC_PERIPHCLK_I2C1 or RCC_PERIPHCLK_I2C2 or RCC_PERIPHCLK_I2C3 or RCC_PERIPHCLK_USART1 or
    RCC_PERIPHCLK_USART2 or RCC_PERIPHCLK_USART3 or RCC_PERIPHCLK_UART4 or RCC_PERIPHCLK_UART5 or
    RCC_PERIPHCLK_USART6 or RCC_PERIPHCLK_UART7 or RCC_PERIPHCLK_UART8 or RCC_PERIPHCLK_SDMMC1 or RCC_PERIPHCLK_CLK48;

  (* Get the PLLI2S Clock configuration -----------------------------------------------*)
  PeriphClkInit.PLLI2S.PLLI2SN := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SN) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SN));
  PeriphClkInit.PLLI2S.PLLI2SP := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SP) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SP));
  PeriphClkInit.PLLI2S.PLLI2SQ := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SQ) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SQ));
  PeriphClkInit.PLLI2S.PLLI2SR := ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SR) shr BsfDWord(RCC_PLLI2SCFGR_PLLI2SR));

  (* Get the PLLSAI Clock configuration -----------------------------------------------*)
  PeriphClkInit.PLLSAI.PLLSAIN := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIN) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIN));
  PeriphClkInit.PLLSAI.PLLSAIP := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIP) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIP));
  PeriphClkInit.PLLSAI.PLLSAIQ := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIQ) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIQ));
  PeriphClkInit.PLLSAI.PLLSAIR := ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIR) shr BsfDWord(RCC_PLLSAICFGR_PLLSAIR));

  (* Get the PLLSAI/PLLI2S division factors -------------------------------------------*)
  PeriphClkInit.PLLI2SDivQ := ((RCC.DCKCFGR1 and RCC_DCKCFGR1_PLLI2SDIVQ) shr BsfDWord(RCC_DCKCFGR1_PLLI2SDIVQ));
  PeriphClkInit.PLLSAIDivQ := ((RCC.DCKCFGR1 and RCC_DCKCFGR1_PLLSAIDIVQ) shr BsfDWord(RCC_DCKCFGR1_PLLSAIDIVQ));
  PeriphClkInit.PLLSAIDivR := ((RCC.DCKCFGR1 and RCC_DCKCFGR1_PLLSAIDIVR) shr BsfDWord(RCC_DCKCFGR1_PLLSAIDIVR));

  (* Get the SAI1 clock configuration ----------------------------------------------*)
  PeriphClkInit.Sai1ClockSelection := __HAL_RCC_GET_SAI1_SOURCE();

  (* Get the SAI2 clock configuration ----------------------------------------------*)
  PeriphClkInit.Sai2ClockSelection := __HAL_RCC_GET_SAI2_SOURCE();

  (* Get the I2S clock configuration ------------------------------------------*)
  PeriphClkInit.I2sClockSelection := __HAL_RCC_GET_I2SCLKSOURCE();

  (* Get the I2C1 clock configuration ------------------------------------------*)
  PeriphClkInit.I2c1ClockSelection := __HAL_RCC_GET_I2C1_SOURCE();

  (* Get the I2C2 clock configuration ------------------------------------------*)
  PeriphClkInit.I2c2ClockSelection := __HAL_RCC_GET_I2C2_SOURCE();

  (* Get the I2C3 clock configuration ------------------------------------------*)
  PeriphClkInit.I2c3ClockSelection := __HAL_RCC_GET_I2C3_SOURCE();

  (* Get the I2C4 clock configuration ------------------------------------------*)
  PeriphClkInit.I2c4ClockSelection := __HAL_RCC_GET_I2C4_SOURCE();

  (* Get the USART1 clock configuration ------------------------------------------*)
  PeriphClkInit.Usart1ClockSelection := __HAL_RCC_GET_USART1_SOURCE();

  (* Get the USART2 clock configuration ------------------------------------------*)
  PeriphClkInit.Usart2ClockSelection := __HAL_RCC_GET_USART2_SOURCE();

  (* Get the USART3 clock configuration ------------------------------------------*)
  PeriphClkInit.Usart3ClockSelection := __HAL_RCC_GET_USART3_SOURCE();

  (* Get the UART4 clock configuration ------------------------------------------*)
  PeriphClkInit.Uart4ClockSelection := __HAL_RCC_GET_UART4_SOURCE();

  (* Get the UART5 clock configuration ------------------------------------------*)
  PeriphClkInit.Uart5ClockSelection := __HAL_RCC_GET_UART5_SOURCE();

  (* Get the USART6 clock configuration ------------------------------------------*)
  PeriphClkInit.Usart6ClockSelection := __HAL_RCC_GET_USART6_SOURCE();

  (* Get the UART7 clock configuration ------------------------------------------*)
  PeriphClkInit.Uart7ClockSelection := __HAL_RCC_GET_UART7_SOURCE();

  (* Get the UART8 clock configuration ------------------------------------------*)
  PeriphClkInit.Uart8ClockSelection := __HAL_RCC_GET_UART8_SOURCE();

  (* Get the LPTIM1 clock configuration ------------------------------------------*)
  PeriphClkInit.Lptim1ClockSelection := __HAL_RCC_GET_LPTIM1_SOURCE();

  (* Get the CEC clock configuration -----------------------------------------------*)
  PeriphClkInit.CecClockSelection := __HAL_RCC_GET_CEC_SOURCE();

  (* Get the CK48 clock configuration -----------------------------------------------*)
  PeriphClkInit.Clk48ClockSelection := __HAL_RCC_GET_CLK48_SOURCE();

  (* Get the SDMMC clock configuration -----------------------------------------------*)
  PeriphClkInit.Sdmmc1ClockSelection := __HAL_RCC_GET_SDMMC1_SOURCE();

  (* Get the RTC Clock configuration -----------------------------------------------*)
  tempreg := (RCC.CFGR and RCC_CFGR_RTCPRE);
  PeriphClkInit.RTCClockSelection := ((tempreg) or (RCC.BDCR and RCC_BDCR_RTCSEL));

  (* Get the TIM Prescaler configuration --------------------------------------------*)
  if ((RCC.DCKCFGR1 and RCC_DCKCFGR1_TIMPRE) = 0) then
  begin
    PeriphClkInit.TIMPresSelection := RCC_TIMPRES_DESACTIVATED;
  end
  else
  begin
    PeriphClkInit.TIMPresSelection := RCC_TIMPRES_ACTIVATED;
  end;
end;

function HAL_RCCEx_GetPeriphCLKFreq(PeriphClk: longword): longword;
var
  tmpreg, frequency, vcoinput, saiclocksource: longword;
begin
  tmpreg := 0;
  (* This variable used to store the SAI clock frequency (value in Hz) *)
  frequency := 0;
  (* This variable used to store the VCO Input (value in Hz) *)
  vcoinput := 0;
  (* This variable used to store the SAI clock source *)
  saiclocksource := 0;

  if ((PeriphClk = RCC_PERIPHCLK_SAI1) or (PeriphClk = RCC_PERIPHCLK_SAI2)) then
  begin
    saiclocksource := RCC.DCKCFGR1;
    saiclocksource := saiclocksource and ((RCC_DCKCFGR1_SAI1SEL or RCC_DCKCFGR1_SAI2SEL));
    case saiclocksource of
      0: (* PLLSAI is the clock source for SAI*)
      begin
        (* Configure the PLLSAI division factor *)
        (* PLLSAI_VCO Input  := PLL_SOURCE div PLLM *)
        if ((RCC.PLLCFGR and RCC_PLLCFGR_PLLSRC) = RCC_PLLSOURCE_HSI) then
        begin
          (* In Case the PLL Source is HSI (Internal Clock) *)
          vcoinput := (HSI_VALUE div (RCC.PLLCFGR and RCC_PLLCFGR_PLLM));
        end
        else
        begin
          (* In Case the PLL Source is HSE (External Clock) *)
          vcoinput := ((HSE_VALUE div (RCC.PLLCFGR and RCC_PLLCFGR_PLLM)));
        end;
        (* PLLSAI_VCO Output := PLLSAI_VCO Input * PLLSAIN *)
        (* SAI_CLK(first level) := PLLSAI_VCO Output div PLLSAIQ *)
        tmpreg := (RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIQ) shr 24;
        frequency := (vcoinput * ((RCC.PLLSAICFGR and RCC_PLLSAICFGR_PLLSAIN) shr 6)) div (tmpreg);

        (* SAI_CLK_x := SAI_CLK(first level) div PLLSAIDIVQ *)
        tmpreg := (((RCC.DCKCFGR1 and RCC_DCKCFGR1_PLLSAIDIVQ) shr 8) + 1);
        frequency := frequency div (tmpreg);
      end;
      RCC_DCKCFGR1_SAI1SEL_0, (* PLLI2S is the clock source for SAI*)
      RCC_DCKCFGR1_SAI2SEL_0: (* PLLI2S is the clock source for SAI*)
      begin
        (* Configure the PLLI2S division factor *)
        (* PLLI2S_VCO Input  := PLL_SOURCE div PLLM *)
        if ((RCC.PLLCFGR and RCC_PLLCFGR_PLLSRC) = RCC_PLLSOURCE_HSI) then
        begin
          (* In Case the PLL Source is HSI (Internal Clock) *)
          vcoinput := (HSI_VALUE div (RCC.PLLCFGR and RCC_PLLCFGR_PLLM));
        end
        else
        begin
          (* In Case the PLL Source is HSE (External Clock) *)
          vcoinput := ((HSE_VALUE div (RCC.PLLCFGR and RCC_PLLCFGR_PLLM)));
        end;

        (* PLLI2S_VCO Output := PLLI2S_VCO Input * PLLI2SN *)
        (* SAI_CLK(first level) := PLLI2S_VCO Output div PLLI2SQ *)
        tmpreg := (RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SQ) shr 24;
        frequency := (vcoinput * ((RCC.PLLI2SCFGR and RCC_PLLI2SCFGR_PLLI2SN) shr 6)) div (tmpreg);

        (* SAI_CLK_x := SAI_CLK(first level) div PLLI2SDIVQ *)
        tmpreg := ((RCC.DCKCFGR1 and RCC_DCKCFGR1_PLLI2SDIVQ) + 1);
        frequency := frequency div (tmpreg);
      end;
      RCC_DCKCFGR1_SAI1SEL_1, (* External clock is the clock source for SAI*)
      RCC_DCKCFGR1_SAI2SEL_1: (* External clock is the clock source for SAI*)
        frequency := EXTERNAL_CLOCK_VALUE;
    end;
  end;
  exit(frequency);
end;

end.
