(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_gpio_ex.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of GPIO HAL Extension module.
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

unit stm32f7xx_hal_gpio_ex;

interface

uses
  stm32f7xx_hal_gpio;

(**
  * @brief   AF 0 selection
   *)

const
  GPIO_AF0_RTC_50Hz = ($00);  (* RTC_50Hz Alternate Function mapping                        *)
  GPIO_AF0_MCO = ($00);  (* MCO (MCO1 and MCO2) Alternate Function mapping             *)
  GPIO_AF0_SWJ = ($00);  (* SWJ (SWD and JTAG) Alternate Function mapping              *)
  GPIO_AF0_TRACE = ($00);  (* TRACE Alternate Function mapping                           *)
  (**
  * @brief   AF 1 selection
   *)

  GPIO_AF1_TIM1 = ($01);  (* TIM1 Alternate Function mapping  *)
  GPIO_AF1_TIM2 = ($01);  (* TIM2 Alternate Function mapping  *)
  (**
  * @brief   AF 2 selection
   *)

  GPIO_AF2_TIM3 = ($02);  (* TIM3 Alternate Function mapping  *)
  GPIO_AF2_TIM4 = ($02);  (* TIM4 Alternate Function mapping  *)
  GPIO_AF2_TIM5 = ($02);  (* TIM5 Alternate Function mapping  *)
  (**
  * @brief   AF 3 selection
   *)

  GPIO_AF3_TIM8 = ($03);  (* TIM8 Alternate Function mapping   *)
  GPIO_AF3_TIM9 = ($03);  (* TIM9 Alternate Function mapping   *)
  GPIO_AF3_TIM10 = ($03);  (* TIM10 Alternate Function mapping  *)
  GPIO_AF3_TIM11 = ($03);  (* TIM11 Alternate Function mapping  *)
  GPIO_AF3_LPTIM1 = ($03);  (* LPTIM1 Alternate Function mapping  *)
  GPIO_AF3_CEC = ($03);  (* CEC Alternate Function mapping  *)
  (**
  * @brief   AF 4 selection
   *)

  GPIO_AF4_I2C1 = ($04);  (* I2C1 Alternate Function mapping  *)
  GPIO_AF4_I2C2 = ($04);  (* I2C2 Alternate Function mapping  *)
  GPIO_AF4_I2C3 = ($04);  (* I2C3 Alternate Function mapping  *)
  GPIO_AF4_I2C4 = ($04);  (* I2C4 Alternate Function mapping  *)
  GPIO_AF4_CEC = ($04);  (* CEC Alternate Function mapping  *)
  (**
  * @brief   AF 5 selection
   *)

  GPIO_AF5_SPI1 = ($05);  (* SPI1 Alternate Function mapping         *)
  GPIO_AF5_SPI2 = ($05);  (* SPI2/I2S2 Alternate Function mapping    *)
  GPIO_AF5_SPI3 = ($05);  (* SPI3/I2S3 Alternate Function mapping    *)
  GPIO_AF5_SPI4 = ($05);  (* SPI4 Alternate Function mapping         *)
  GPIO_AF5_SPI5 = ($05);  (* SPI5 Alternate Function mapping         *)
  GPIO_AF5_SPI6 = ($05);  (* SPI6 Alternate Function mapping         *)
  (**
  * @brief   AF 6 selection
   *)

  GPIO_AF6_SPI3 = ($06);  (* SPI3/I2S3 Alternate Function mapping   *)
  GPIO_AF6_SAI1 = ($06);  (* SAI1 Alternate Function mapping        *)
  (**
  * @brief   AF 7 selection
   *)

  GPIO_AF7_USART1 = ($07);  (* USART1 Alternate Function mapping      *)
  GPIO_AF7_USART2 = ($07);  (* USART2 Alternate Function mapping      *)
  GPIO_AF7_USART3 = ($07);  (* USART3 Alternate Function mapping      *)
  GPIO_AF7_UART5 = ($07);  (* UART5 Alternate Function mapping       *)
  GPIO_AF7_SPDIFRX = ($07);  (* SPDIF-RX Alternate Function mapping    *)
  GPIO_AF7_SPI2 = ($07);  (* SPI2 Alternate Function mapping        *)
  GPIO_AF7_SPI3 = ($07);  (* SPI3 Alternate Function mapping        *)
  (**
  * @brief   AF 8 selection
   *)

  GPIO_AF8_UART4 = ($08);  (* UART4 Alternate Function mapping   *)
  GPIO_AF8_UART5 = ($08);  (* UART5 Alternate Function mapping   *)
  GPIO_AF8_USART6 = ($08);  (* USART6 Alternate Function mapping  *)
  GPIO_AF8_UART7 = ($08);  (* UART7 Alternate Function mapping   *)
  GPIO_AF8_UART8 = ($08);  (* UART8 Alternate Function mapping   *)
  GPIO_AF8_SPDIFRX = ($08);  (* SPIDIF-RX Alternate Function mapping   *)
  GPIO_AF8_SAI2 = ($08);  (* SAI2 Alternate Function mapping   *)
  (**
  * @brief   AF 9 selection
   *)

  GPIO_AF9_CAN1 = ($09);  (* CAN1 Alternate Function mapping     *)
  GPIO_AF9_CAN2 = ($09);  (* CAN2 Alternate Function mapping     *)
  GPIO_AF9_TIM12 = ($09);  (* TIM12 Alternate Function mapping    *)
  GPIO_AF9_TIM13 = ($09);  (* TIM13 Alternate Function mapping    *)
  GPIO_AF9_TIM14 = ($09);  (* TIM14 Alternate Function mapping    *)
  GPIO_AF9_QUADSPI = ($09);  (* QUADSPI Alternate Function mapping  *)
  GPIO_AF9_LTDC = ($09);  (* LCD-TFT Alternate Function mapping  *)
  (**
  * @brief   AF 10 selection
   *)

  GPIO_AF10_OTG_FS = ($A);  (* OTG_FS Alternate Function mapping  *)
  GPIO_AF10_OTG_HS = ($A);  (* OTG_HS Alternate Function mapping  *)
  GPIO_AF10_QUADSPI = ($A);  (* QUADSPI Alternate Function mapping  *)
  GPIO_AF10_SAI2 = ($A);  (* SAI2 Alternate Function mapping  *)
  (**
  * @brief   AF 11 selection
   *)

  GPIO_AF11_ETH = ($0B);  (* ETHERNET Alternate Function mapping  *)
  (**
  * @brief   AF 12 selection
   *)

  GPIO_AF12_FMC = ($C);  (* FMC Alternate Function mapping                       *)
  GPIO_AF12_OTG_HS_FS = ($C);  (* OTG HS configured in FS, Alternate Function mapping  *)
  GPIO_AF12_SDMMC1 = ($C);  (* SDMMC1 Alternate Function mapping                      *)
  (**
  * @brief   AF 13 selection
   *)

  GPIO_AF13_DCMI = ($0D);  (* DCMI Alternate Function mapping  *)
  (**
  * @brief   AF 14 selection
   *)

  GPIO_AF14_LTDC = ($0E);  (* LCD-TFT Alternate Function mapping  *)
  (**
  * @brief   AF 15 selection
   *)

  GPIO_AF15_EVENTOUT = ($0F);  (* EVENTOUT Alternate Function mapping  *)
  (**
  * @brief   GPIO pin available on the platform
   *)

  (* Defines the available pins per GPIOs  *)

  GPIOA_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOB_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOC_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOD_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOE_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOF_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOG_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOI_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOJ_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOH_PIN_AVAILABLE = GPIO_PIN_All;
  GPIOK_PIN_AVAILABLE = (GPIO_PIN_0 or GPIO_PIN_1 or GPIO_PIN_3 or GPIO_PIN_4 or GPIO_PIN_5 or GPIO_PIN_6 or GPIO_PIN_7);

function GPIO_GET_INDEX(var __GPIOx__: TGPIOA_Registers): longword;

implementation

function GPIO_GET_INDEX(var __GPIOx__: TGPIOA_Registers): longword;
begin
  if @__GPIOx__ = @GPIOA then
    exit(0)
  else if @__GPIOx__ = @GPIOB then
    exit(1)
  else if @__GPIOx__ = @GPIOC then
    exit(2)
  else if @__GPIOx__ = @GPIOD then
    exit(3)
  else if @__GPIOx__ = @GPIOE then
    exit(4)
  else if @__GPIOx__ = @GPIOF then
    exit(5)
  else if @__GPIOx__ = @GPIOG then
    exit(6)
  else if @__GPIOx__ = @GPIOH then
    exit(7)
  else if @__GPIOx__ = @GPIOI then
    exit(8)
  else if @__GPIOx__ = @GPIOJ then
    exit(9)
  else
    exit(10);
end;

end.

