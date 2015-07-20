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

procedure __HAL_RCC_I2C1_CLK_ENABLE();
procedure __HAL_RCC_I2C2_CLK_ENABLE();
procedure __HAL_RCC_I2C3_CLK_ENABLE();
procedure __HAL_RCC_I2C4_CLK_ENABLE();

procedure __HAL_RCC_I2C1_CLK_DISABLE();
procedure __HAL_RCC_I2C2_CLK_DISABLE();
procedure __HAL_RCC_I2C3_CLK_DISABLE();
procedure __HAL_RCC_I2C4_CLK_DISABLE();

procedure __HAL_RCC_TIM2_FORCE_RESET()     ;
procedure __HAL_RCC_TIM3_FORCE_RESET()     ;
procedure __HAL_RCC_TIM4_FORCE_RESET()     ;
procedure __HAL_RCC_TIM5_FORCE_RESET()     ;
procedure __HAL_RCC_TIM6_FORCE_RESET()     ;
procedure __HAL_RCC_TIM7_FORCE_RESET()     ;
procedure __HAL_RCC_TIM12_FORCE_RESET()    ;
procedure __HAL_RCC_TIM13_FORCE_RESET()    ;
procedure __HAL_RCC_TIM14_FORCE_RESET()    ;
procedure __HAL_RCC_LPTIM1_FORCE_RESET()   ;
procedure __HAL_RCC_SPI2_FORCE_RESET()     ;
procedure __HAL_RCC_SPI3_FORCE_RESET()     ;
procedure __HAL_RCC_SPDIFRX_FORCE_RESET()  ;
procedure __HAL_RCC_USART2_FORCE_RESET()   ;
procedure __HAL_RCC_USART3_FORCE_RESET()   ;
procedure __HAL_RCC_UART4_FORCE_RESET()    ;
procedure __HAL_RCC_UART5_FORCE_RESET()    ;
procedure __HAL_RCC_I2C1_FORCE_RESET()     ;
procedure __HAL_RCC_I2C2_FORCE_RESET()     ;
procedure __HAL_RCC_I2C3_FORCE_RESET()     ;
procedure __HAL_RCC_I2C4_FORCE_RESET()     ;
procedure __HAL_RCC_CAN1_FORCE_RESET()     ;
procedure __HAL_RCC_CAN2_FORCE_RESET()     ;
procedure __HAL_RCC_CEC_FORCE_RESET()      ;
procedure __HAL_RCC_DAC_FORCE_RESET()      ;
procedure __HAL_RCC_UART7_FORCE_RESET()    ;
procedure __HAL_RCC_UART8_FORCE_RESET()    ;
procedure __HAL_RCC_TIM2_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM3_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM4_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM5_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM6_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM7_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM12_RELEASE_RESET()  ;
procedure __HAL_RCC_TIM13_RELEASE_RESET()  ;
procedure __HAL_RCC_TIM14_RELEASE_RESET()  ;
procedure __HAL_RCC_LPTIM1_RELEASE_RESET() ;
procedure __HAL_RCC_SPI2_RELEASE_RESET()   ;
procedure __HAL_RCC_SPI3_RELEASE_RESET()   ;
procedure __HAL_RCC_SPDIFRX_RELEASE_RESET();
procedure __HAL_RCC_USART2_RELEASE_RESET() ;
procedure __HAL_RCC_USART3_RELEASE_RESET() ;
procedure __HAL_RCC_UART4_RELEASE_RESET()  ;
procedure __HAL_RCC_UART5_RELEASE_RESET()  ;
procedure __HAL_RCC_I2C1_RELEASE_RESET()   ;
procedure __HAL_RCC_I2C2_RELEASE_RESET()   ;
procedure __HAL_RCC_I2C3_RELEASE_RESET()   ;
procedure __HAL_RCC_I2C4_RELEASE_RESET()   ;
procedure __HAL_RCC_CAN1_RELEASE_RESET()   ;
procedure __HAL_RCC_CAN2_RELEASE_RESET()   ;
procedure __HAL_RCC_CEC_RELEASE_RESET()    ;
procedure __HAL_RCC_DAC_RELEASE_RESET()    ;
procedure __HAL_RCC_UART7_RELEASE_RESET()  ;
procedure __HAL_RCC_UART8_RELEASE_RESET()  ;
procedure __HAL_RCC_TIM1_FORCE_RESET()     ;
procedure __HAL_RCC_TIM8_FORCE_RESET()     ;
procedure __HAL_RCC_USART1_FORCE_RESET()   ;
procedure __HAL_RCC_USART6_FORCE_RESET()   ;
procedure __HAL_RCC_ADC_FORCE_RESET()      ;
procedure __HAL_RCC_SDMMC1_FORCE_RESET()   ;
procedure __HAL_RCC_SPI1_FORCE_RESET()     ;
procedure __HAL_RCC_SPI4_FORCE_RESET()     ;
procedure __HAL_RCC_TIM9_FORCE_RESET()     ;
procedure __HAL_RCC_TIM10_FORCE_RESET()    ;
procedure __HAL_RCC_TIM11_FORCE_RESET()    ;
procedure __HAL_RCC_SPI5_FORCE_RESET()     ;
procedure __HAL_RCC_SPI6_FORCE_RESET()     ;
procedure __HAL_RCC_SAI1_FORCE_RESET()     ;
procedure __HAL_RCC_SAI2_FORCE_RESET()     ;
//procedure __HAL_RCC_LTDC_FORCE_RESET()     ;
procedure __HAL_RCC_TIM1_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM8_RELEASE_RESET()   ;
procedure __HAL_RCC_USART1_RELEASE_RESET() ;
procedure __HAL_RCC_USART6_RELEASE_RESET() ;
procedure __HAL_RCC_ADC_RELEASE_RESET()    ;
procedure __HAL_RCC_SDMMC1_RELEASE_RESET() ;
procedure __HAL_RCC_SPI1_RELEASE_RESET()   ;
procedure __HAL_RCC_SPI4_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM9_RELEASE_RESET()   ;
procedure __HAL_RCC_TIM10_RELEASE_RESET()  ;
procedure __HAL_RCC_TIM11_RELEASE_RESET()  ;
procedure __HAL_RCC_SPI5_RELEASE_RESET()   ;
procedure __HAL_RCC_SPI6_RELEASE_RESET()   ;
procedure __HAL_RCC_SAI1_RELEASE_RESET()   ;
procedure __HAL_RCC_SAI2_RELEASE_RESET()   ;
//procedure __HAL_RCC_LTDC_RELEASE_RESET()   ;

procedure __HAL_RCC_SYSCFG_CLK_ENABLE();
procedure __HAL_RCC_SYSCFG_CLK_DISABLE();

procedure __HAL_RCC_ETHMAC_CLK_ENABLE();
procedure __HAL_RCC_ETHMACTX_CLK_ENABLE();
procedure __HAL_RCC_ETHMACRX_CLK_ENABLE();
procedure __HAL_RCC_ETHMACPTP_CLK_ENABLE();

procedure __HAL_RCC_ETH_CLK_ENABLE();

implementation

uses
  stm32f7xx_defs;

procedure __HAL_RCC_GPIOA_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOAEN; end;
procedure __HAL_RCC_GPIOB_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOBEN; end;
procedure __HAL_RCC_GPIOC_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOCEN; end;
procedure __HAL_RCC_GPIOD_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIODEN; end;
procedure __HAL_RCC_GPIOE_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOEEN; end;
procedure __HAL_RCC_GPIOF_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOFEN; end;
procedure __HAL_RCC_GPIOG_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOGEN; end;
procedure __HAL_RCC_GPIOH_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOHEN; end;
procedure __HAL_RCC_GPIOI_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOIEN; end;
procedure __HAL_RCC_GPIOJ_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOJEN; end;
procedure __HAL_RCC_GPIOK_CLK_ENABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_GPIOKEN; end;

procedure __HAL_RCC_GPIOA_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOAEN); end;
procedure __HAL_RCC_GPIOB_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOBEN); end;
procedure __HAL_RCC_GPIOC_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOCEN); end;
procedure __HAL_RCC_GPIOD_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIODEN); end;
procedure __HAL_RCC_GPIOE_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOEEN); end;
procedure __HAL_RCC_GPIOF_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOFEN); end;
procedure __HAL_RCC_GPIOG_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOGEN); end;
procedure __HAL_RCC_GPIOH_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOHEN); end;
procedure __HAL_RCC_GPIOI_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOIEN); end;
procedure __HAL_RCC_GPIOJ_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOJEN); end;
procedure __HAL_RCC_GPIOK_CLK_DISABLE; inline; begin RCC.AHB1ENR := RCC.AHB1ENR and (not RCC_AHB1ENR_GPIOKEN); end;

procedure __HAL_RCC_I2C1_CLK_ENABLE(); var tmp: longword; begin RCC.APB1ENR := RCC.APB1ENR or RCC_APB1ENR_I2C1EN; tmp:=RCC.APB1ENR; end;
procedure __HAL_RCC_I2C2_CLK_ENABLE(); var tmp: longword; begin RCC.APB1ENR := RCC.APB1ENR or RCC_APB1ENR_I2C2EN; tmp:=RCC.APB1ENR; end;
procedure __HAL_RCC_I2C3_CLK_ENABLE(); var tmp: longword; begin RCC.APB1ENR := RCC.APB1ENR or RCC_APB1ENR_I2C3EN; tmp:=RCC.APB1ENR; end;
procedure __HAL_RCC_I2C4_CLK_ENABLE(); var tmp: longword; begin RCC.APB1ENR := RCC.APB1ENR or RCC_APB1ENR_I2C4EN; tmp:=RCC.APB1ENR; end;

procedure __HAL_RCC_I2C1_CLK_DISABLE(); var tmp: longword; begin RCC.APB1ENR := RCC.APB1ENR and (not RCC_APB1ENR_I2C1EN); tmp:=RCC.APB1ENR; end;
procedure __HAL_RCC_I2C2_CLK_DISABLE(); var tmp: longword; begin RCC.APB1ENR := RCC.APB1ENR and (not RCC_APB1ENR_I2C2EN); tmp:=RCC.APB1ENR; end;
procedure __HAL_RCC_I2C3_CLK_DISABLE(); var tmp: longword; begin RCC.APB1ENR := RCC.APB1ENR and (not RCC_APB1ENR_I2C3EN); tmp:=RCC.APB1ENR; end;
procedure __HAL_RCC_I2C4_CLK_DISABLE(); var tmp: longword; begin RCC.APB1ENR := RCC.APB1ENR and (not RCC_APB1ENR_I2C4EN); tmp:=RCC.APB1ENR; end;

procedure __HAL_RCC_SYSCFG_CLK_ENABLE(); var tmp: longword; begin RCC.APB2ENR := RCC.APB2ENR or RCC_APB2ENR_SYSCFGEN; tmp:=RCC.APB2ENR; end;
procedure __HAL_RCC_SYSCFG_CLK_DISABLE(); begin RCC.APB1ENR := RCC.APB1ENR and (not RCC_APB2ENR_SYSCFGEN); end;

procedure __HAL_RCC_ETHMAC_CLK_ENABLE(); var tmp: longword;    begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_ETHMACEN; tmp:=RCC.AHB1ENR; end;
procedure __HAL_RCC_ETHMACTX_CLK_ENABLE(); var tmp: longword;  begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_ETHMACTXEN; tmp:=RCC.AHB1ENR; end;
procedure __HAL_RCC_ETHMACRX_CLK_ENABLE(); var tmp: longword;  begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_ETHMACRXEN; tmp:=RCC.AHB1ENR; end;
procedure __HAL_RCC_ETHMACPTP_CLK_ENABLE(); var tmp: longword; begin RCC.AHB1ENR := RCC.AHB1ENR or RCC_AHB1ENR_ETHMACPTPEN; tmp:=RCC.AHB1ENR; end;

procedure __HAL_RCC_ETH_CLK_ENABLE(); begin __HAL_RCC_ETHMAC_CLK_ENABLE();      __HAL_RCC_ETHMACTX_CLK_ENABLE();    __HAL_RCC_ETHMACRX_CLK_ENABLE();    end;

procedure __HAL_RCC_TIM2_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM2RST)         ; end;
procedure __HAL_RCC_TIM3_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM3RST)         ; end;
procedure __HAL_RCC_TIM4_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM4RST)         ; end;
procedure __HAL_RCC_TIM5_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM5RST)         ; end;
procedure __HAL_RCC_TIM6_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM6RST)         ; end;
procedure __HAL_RCC_TIM7_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM7RST)         ; end;
procedure __HAL_RCC_TIM12_FORCE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM12RST)        ; end;
procedure __HAL_RCC_TIM13_FORCE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM13RST)        ; end;
procedure __HAL_RCC_TIM14_FORCE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_TIM14RST)        ; end;
procedure __HAL_RCC_LPTIM1_FORCE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_LPTIM1RST)       ; end;
procedure __HAL_RCC_SPI2_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_SPI2RST)         ; end;
procedure __HAL_RCC_SPI3_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_SPI3RST)         ; end;
procedure __HAL_RCC_SPDIFRX_FORCE_RESET()  ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_SPDIFRXRST)      ; end;
procedure __HAL_RCC_USART2_FORCE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_USART2RST)       ; end;
procedure __HAL_RCC_USART3_FORCE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_USART3RST)       ; end;
procedure __HAL_RCC_UART4_FORCE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_UART4RST)        ; end;
procedure __HAL_RCC_UART5_FORCE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_UART5RST)        ; end;
procedure __HAL_RCC_I2C1_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_I2C1RST)         ; end;
procedure __HAL_RCC_I2C2_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_I2C2RST)         ; end;
procedure __HAL_RCC_I2C3_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_I2C3RST)         ; end;
procedure __HAL_RCC_I2C4_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_I2C4RST)         ; end;
procedure __HAL_RCC_CAN1_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_CAN1RST)         ; end;
procedure __HAL_RCC_CAN2_FORCE_RESET()     ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_CAN2RST)         ; end;
procedure __HAL_RCC_CEC_FORCE_RESET()      ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_CECRST)          ; end;
procedure __HAL_RCC_DAC_FORCE_RESET()      ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_DACRST)          ; end;
procedure __HAL_RCC_UART7_FORCE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_UART7RST)        ; end;
procedure __HAL_RCC_UART8_FORCE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR or (RCC_APB1RSTR_UART8RST)        ; end;
procedure __HAL_RCC_TIM2_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM2RST)        ; end;
procedure __HAL_RCC_TIM3_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM3RST)        ; end;
procedure __HAL_RCC_TIM4_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM4RST)        ; end;
procedure __HAL_RCC_TIM5_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM5RST)        ; end;
procedure __HAL_RCC_TIM6_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM6RST)        ; end;
procedure __HAL_RCC_TIM7_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM7RST)        ; end;
procedure __HAL_RCC_TIM12_RELEASE_RESET()  ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM12RST)       ; end;
procedure __HAL_RCC_TIM13_RELEASE_RESET()  ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM13RST)       ; end;
procedure __HAL_RCC_TIM14_RELEASE_RESET()  ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_TIM14RST)       ; end;
procedure __HAL_RCC_LPTIM1_RELEASE_RESET() ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_LPTIM1RST)      ; end;
procedure __HAL_RCC_SPI2_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_SPI2RST)        ; end;
procedure __HAL_RCC_SPI3_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_SPI3RST)        ; end;
procedure __HAL_RCC_SPDIFRX_RELEASE_RESET(); begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_SPDIFRXRST)     ; end;
procedure __HAL_RCC_USART2_RELEASE_RESET() ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_USART2RST)      ; end;
procedure __HAL_RCC_USART3_RELEASE_RESET() ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_USART3RST)      ; end;
procedure __HAL_RCC_UART4_RELEASE_RESET()  ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_UART4RST)       ; end;
procedure __HAL_RCC_UART5_RELEASE_RESET()  ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_UART5RST)       ; end;
procedure __HAL_RCC_I2C1_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_I2C1RST)        ; end;
procedure __HAL_RCC_I2C2_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_I2C2RST)        ; end;
procedure __HAL_RCC_I2C3_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_I2C3RST)        ; end;
procedure __HAL_RCC_I2C4_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_I2C4RST)        ; end;
procedure __HAL_RCC_CAN1_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_CAN1RST)        ; end;
procedure __HAL_RCC_CAN2_RELEASE_RESET()   ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_CAN2RST)        ; end;
procedure __HAL_RCC_CEC_RELEASE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_CECRST)         ; end;
procedure __HAL_RCC_DAC_RELEASE_RESET()    ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_DACRST)         ; end;
procedure __HAL_RCC_UART7_RELEASE_RESET()  ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_UART7RST)       ; end;
procedure __HAL_RCC_UART8_RELEASE_RESET()  ; begin RCC.APB1RSTR := RCC.APB1RSTR and (not RCC_APB1RSTR_UART8RST)       ; end;
procedure __HAL_RCC_TIM1_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM1RST)         ; end;
procedure __HAL_RCC_TIM8_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM8RST)         ; end;
procedure __HAL_RCC_USART1_FORCE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_USART1RST)       ; end;
procedure __HAL_RCC_USART6_FORCE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_USART6RST)       ; end;
procedure __HAL_RCC_ADC_FORCE_RESET()      ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_ADCRST)          ; end;
procedure __HAL_RCC_SDMMC1_FORCE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SDMMC1RST)       ; end;
procedure __HAL_RCC_SPI1_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SPI1RST)         ; end;
procedure __HAL_RCC_SPI4_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SPI4RST)         ; end;
procedure __HAL_RCC_TIM9_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM9RST)         ; end;
procedure __HAL_RCC_TIM10_FORCE_RESET()    ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM10RST)        ; end;
procedure __HAL_RCC_TIM11_FORCE_RESET()    ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_TIM11RST)        ; end;
procedure __HAL_RCC_SPI5_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SPI5RST)         ; end;
procedure __HAL_RCC_SPI6_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SPI6RST)         ; end;
procedure __HAL_RCC_SAI1_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SAI1RST)         ; end;
procedure __HAL_RCC_SAI2_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_SAI2RST)         ; end;
//procedure __HAL_RCC_LTDC_FORCE_RESET()     ; begin RCC.APB2RSTR := RCC.APB2RSTR or (RCC_APB2RSTR_LTDCRST)         ; end;
procedure __HAL_RCC_TIM1_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_TIM1RST)        ; end;
procedure __HAL_RCC_TIM8_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_TIM8RST)        ; end;
procedure __HAL_RCC_USART1_RELEASE_RESET() ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_USART1RST)      ; end;
procedure __HAL_RCC_USART6_RELEASE_RESET() ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_USART6RST)      ; end;
procedure __HAL_RCC_ADC_RELEASE_RESET()    ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_ADCRST)         ; end;
procedure __HAL_RCC_SDMMC1_RELEASE_RESET() ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_SDMMC1RST)      ; end;
procedure __HAL_RCC_SPI1_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_SPI1RST)        ; end;
procedure __HAL_RCC_SPI4_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_SPI4RST)        ; end;
procedure __HAL_RCC_TIM9_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_TIM9RST)        ; end;
procedure __HAL_RCC_TIM10_RELEASE_RESET()  ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_TIM10RST)       ; end;
procedure __HAL_RCC_TIM11_RELEASE_RESET()  ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_TIM11RST)       ; end;
procedure __HAL_RCC_SPI5_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_SPI5RST)        ; end;
procedure __HAL_RCC_SPI6_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_SPI6RST)        ; end;
procedure __HAL_RCC_SAI1_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_SAI1RST)        ; end;
procedure __HAL_RCC_SAI2_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_SAI2RST)        ; end;
//procedure __HAL_RCC_LTDC_RELEASE_RESET()   ; begin RCC.APB2RSTR := RCC.APB2RSTR and (not RCC_APB2RSTR_LTDCRST)        ; end;

end.

