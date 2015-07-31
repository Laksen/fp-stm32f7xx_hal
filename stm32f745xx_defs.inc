{$packrecords C}

(**
  ******************************************************************************
  * @file    stm32f745xx.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   CMSIS STM32F745xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripherals registers hardware
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

(** @addtogroup CMSIS_Device
  * @{
   *)

(**
 * @brief STM32F7xx Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
  *)

const
  (******  Cortex-M7 Processor Exceptions Numbers *************************************************************** *)
  NonMaskableInt_IRQn = - 14;  (*!< 2 Non Maskable Interrupt                                           *)
  MemoryManagement_IRQn = - 12;  (*!< 4 Cortex-M7 Memory Management Interrupt                            *)
  BusFault_IRQn = - 11;  (*!< 5 Cortex-M7 Bus Fault Interrupt                                    *)
  UsageFault_IRQn = - 10;  (*!< 6 Cortex-M7 Usage Fault Interrupt                                  *)
  SVCall_IRQn = - 5;  (*!< 11 Cortex-M7 SV Call Interrupt                                     *)
  DebugMonitor_IRQn = - 4;  (*!< 12 Cortex-M7 Debug Monitor Interrupt                               *)
  PendSV_IRQn = - 2;  (*!< 14 Cortex-M7 Pend SV Interrupt                                     *)
  SysTick_IRQn = - 1;  (*!< 15 Cortex-M7 System Tick Interrupt                                 *)
  (******  STM32 specific Interrupt Numbers ********************************************************************* *)
  WWDG_IRQn = 0;  (*!< Window WatchDog Interrupt                                          *)
  PVD_IRQn = 1;  (*!< PVD through EXTI Line detection Interrupt                          *)
  TAMP_STAMP_IRQn = 2;  (*!< Tamper and TimeStamp interrupts through the EXTI line              *)
  RTC_WKUP_IRQn = 3;  (*!< RTC Wakeup interrupt through the EXTI line                         *)
  FLASH_IRQn = 4;  (*!< FLASH global Interrupt                                             *)
  RCC_IRQn = 5;  (*!< RCC global Interrupt                                               *)
  EXTI0_IRQn = 6;  (*!< EXTI Line0 Interrupt                                               *)
  EXTI1_IRQn = 7;  (*!< EXTI Line1 Interrupt                                               *)
  EXTI2_IRQn = 8;  (*!< EXTI Line2 Interrupt                                               *)
  EXTI3_IRQn = 9;  (*!< EXTI Line3 Interrupt                                               *)
  EXTI4_IRQn = 10;  (*!< EXTI Line4 Interrupt                                               *)
  DMA1_Stream0_IRQn = 11;  (*!< DMA1 Stream 0 global Interrupt                                     *)
  DMA1_Stream1_IRQn = 12;  (*!< DMA1 Stream 1 global Interrupt                                     *)
  DMA1_Stream2_IRQn = 13;  (*!< DMA1 Stream 2 global Interrupt                                     *)
  DMA1_Stream3_IRQn = 14;  (*!< DMA1 Stream 3 global Interrupt                                     *)
  DMA1_Stream4_IRQn = 15;  (*!< DMA1 Stream 4 global Interrupt                                     *)
  DMA1_Stream5_IRQn = 16;  (*!< DMA1 Stream 5 global Interrupt                                     *)
  DMA1_Stream6_IRQn = 17;  (*!< DMA1 Stream 6 global Interrupt                                     *)
  ADC_IRQn = 18;  (*!< ADC1, ADC2 and ADC3 global Interrupts                              *)
  CAN1_TX_IRQn = 19;  (*!< CAN1 TX Interrupt                                                  *)
  CAN1_RX0_IRQn = 20;  (*!< CAN1 RX0 Interrupt                                                 *)
  CAN1_RX1_IRQn = 21;  (*!< CAN1 RX1 Interrupt                                                 *)
  CAN1_SCE_IRQn = 22;  (*!< CAN1 SCE Interrupt                                                 *)
  EXTI9_5_IRQn = 23;  (*!< External Line[9:5] Interrupts                                      *)
  TIM1_BRK_TIM9_IRQn = 24;  (*!< TIM1 Break interrupt and TIM9 global interrupt                     *)
  TIM1_UP_TIM10_IRQn = 25;  (*!< TIM1 Update Interrupt and TIM10 global interrupt                   *)
  TIM1_TRG_COM_TIM11_IRQn = 26;  (*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt  *)
  TIM1_CC_IRQn = 27;  (*!< TIM1 Capture Compare Interrupt                                     *)
  TIM2_IRQn = 28;  (*!< TIM2 global Interrupt                                              *)
  TIM3_IRQn = 29;  (*!< TIM3 global Interrupt                                              *)
  TIM4_IRQn = 30;  (*!< TIM4 global Interrupt                                              *)
  I2C1_EV_IRQn = 31;  (*!< I2C1 Event Interrupt                                               *)
  I2C1_ER_IRQn = 32;  (*!< I2C1 Error Interrupt                                               *)
  I2C2_EV_IRQn = 33;  (*!< I2C2 Event Interrupt                                               *)
  I2C2_ER_IRQn = 34;  (*!< I2C2 Error Interrupt                                               *)
  SPI1_IRQn = 35;  (*!< SPI1 global Interrupt                                              *)
  SPI2_IRQn = 36;  (*!< SPI2 global Interrupt                                              *)
  USART1_IRQn = 37;  (*!< USART1 global Interrupt                                            *)
  USART2_IRQn = 38;  (*!< USART2 global Interrupt                                            *)
  USART3_IRQn = 39;  (*!< USART3 global Interrupt                                            *)
  EXTI15_10_IRQn = 40;  (*!< External Line[15:10] Interrupts                                    *)
  RTC_Alarm_IRQn = 41;  (*!< RTC Alarm (A and B) through EXTI Line Interrupt                    *)
  OTG_FS_WKUP_IRQn = 42;  (*!< USB OTG FS Wakeup through EXTI line interrupt                      *)
  TIM8_BRK_TIM12_IRQn = 43;  (*!< TIM8 Break Interrupt and TIM12 global interrupt                    *)
  TIM8_UP_TIM13_IRQn = 44;  (*!< TIM8 Update Interrupt and TIM13 global interrupt                   *)
  TIM8_TRG_COM_TIM14_IRQn = 45;  (*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt  *)
  TIM8_CC_IRQn = 46;  (*!< TIM8 Capture Compare Interrupt                                     *)
  DMA1_Stream7_IRQn = 47;  (*!< DMA1 Stream7 Interrupt                                             *)
  FMC_IRQn = 48;  (*!< FMC global Interrupt                                               *)
  SDMMC1_IRQn = 49;  (*!< SDMMC1 global Interrupt                                              *)
  TIM5_IRQn = 50;  (*!< TIM5 global Interrupt                                              *)
  SPI3_IRQn = 51;  (*!< SPI3 global Interrupt                                              *)
  UART4_IRQn = 52;  (*!< UART4 global Interrupt                                             *)
  UART5_IRQn = 53;  (*!< UART5 global Interrupt                                             *)
  TIM6_DAC_IRQn = 54;  (*!< TIM6 global and DAC1&2 underrun error  interrupts                  *)
  TIM7_IRQn = 55;  (*!< TIM7 global interrupt                                              *)
  DMA2_Stream0_IRQn = 56;  (*!< DMA2 Stream 0 global Interrupt                                     *)
  DMA2_Stream1_IRQn = 57;  (*!< DMA2 Stream 1 global Interrupt                                     *)
  DMA2_Stream2_IRQn = 58;  (*!< DMA2 Stream 2 global Interrupt                                     *)
  DMA2_Stream3_IRQn = 59;  (*!< DMA2 Stream 3 global Interrupt                                     *)
  DMA2_Stream4_IRQn = 60;  (*!< DMA2 Stream 4 global Interrupt                                     *)
  ETH_IRQn = 61;  (*!< Ethernet global Interrupt                                          *)
  ETH_WKUP_IRQn = 62;  (*!< Ethernet Wakeup through EXTI line Interrupt                        *)
  CAN2_TX_IRQn = 63;  (*!< CAN2 TX Interrupt                                                  *)
  CAN2_RX0_IRQn = 64;  (*!< CAN2 RX0 Interrupt                                                 *)
  CAN2_RX1_IRQn = 65;  (*!< CAN2 RX1 Interrupt                                                 *)
  CAN2_SCE_IRQn = 66;  (*!< CAN2 SCE Interrupt                                                 *)
  OTG_FS_IRQn = 67;  (*!< USB OTG FS global Interrupt                                        *)
  DMA2_Stream5_IRQn = 68;  (*!< DMA2 Stream 5 global interrupt                                     *)
  DMA2_Stream6_IRQn = 69;  (*!< DMA2 Stream 6 global interrupt                                     *)
  DMA2_Stream7_IRQn = 70;  (*!< DMA2 Stream 7 global interrupt                                     *)
  USART6_IRQn = 71;  (*!< USART6 global interrupt                                            *)
  I2C3_EV_IRQn = 72;  (*!< I2C3 event interrupt                                               *)
  I2C3_ER_IRQn = 73;  (*!< I2C3 error interrupt                                               *)
  OTG_HS_EP1_OUT_IRQn = 74;  (*!< USB OTG HS End Point 1 Out global interrupt                        *)
  OTG_HS_EP1_IN_IRQn = 75;  (*!< USB OTG HS End Point 1 In global interrupt                         *)
  OTG_HS_WKUP_IRQn = 76;  (*!< USB OTG HS Wakeup through EXTI interrupt                           *)
  OTG_HS_IRQn = 77;  (*!< USB OTG HS global interrupt                                        *)
  DCMI_IRQn = 78;  (*!< DCMI global interrupt                                              *)
  RNG_IRQn = 80;  (*!< RNG global interrupt                                               *)
  FPU_IRQn = 81;  (*!< FPU global interrupt                                               *)
  UART7_IRQn = 82;  (*!< UART7 global interrupt                                             *)
  UART8_IRQn = 83;  (*!< UART8 global interrupt                                             *)
  SPI4_IRQn = 84;  (*!< SPI4 global Interrupt                                              *)
  SPI5_IRQn = 85;  (*!< SPI5 global Interrupt                                              *)
  SPI6_IRQn = 86;  (*!< SPI6 global Interrupt                                              *)
  SAI1_IRQn = 87;  (*!< SAI1 global Interrupt                                              *)
  DMA2D_IRQn = 90;  (*!< DMA2D global Interrupt                                             *)
  SAI2_IRQn = 91;  (*!< SAI2 global Interrupt                                              *)
  QUADSPI_IRQn = 92;  (*!< Quad SPI global interrupt                                          *)
  LPTIM1_IRQn = 93;  (*!< LP TIM1 interrupt                                                  *)
  CEC_IRQn = 94;  (*!< HDMI-CEC global Interrupt                                          *)
  I2C4_EV_IRQn = 95;  (*!< I2C4 Event Interrupt                                               *)
  I2C4_ER_IRQn = 96;  (*!< I2C4 Error Interrupt                                               *)
  SPDIF_RX_IRQn = 97;  (*!< SPDIF-RX global Interrupt                                          *)
type
  IRQn = Integer;
  IRQn_Type = IRQn;

  (***************************************************************************** *)

  (*                         Peripheral Registers_Bits_Definition                *)

  (***************************************************************************** *)

  (***************************************************************************** *)

  (*                                                                             *)

  (*                        Analog to Digital Converter                          *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for ADC_SR register  ******************* *)

const
  ADC_SR_AWD = $00000001;  (*!<Analog watchdog flag                                  *)
  ADC_SR_EOC = $00000002;  (*!<End of conversion                                     *)
  ADC_SR_JEOC = $00000004;  (*!<Injected channel end of conversion                    *)
  ADC_SR_JSTRT = $00000008;  (*!<Injected channel Start flag                           *)
  ADC_SR_STRT = $00000010;  (*!<Regular channel Start flag                            *)
  ADC_SR_OVR = $00000020;  (*!<Overrun flag                                          *)
  (*******************  Bit definition for ADC_CR1 register  ******************* *)

  ADC_CR1_AWDCH = $0000001F;  (*!<AWDCH[4:0] bits (Analog watchdog channel select bits)  *)
  ADC_CR1_AWDCH_0 = $00000001;  (*!<Bit 0  *)
  ADC_CR1_AWDCH_1 = $00000002;  (*!<Bit 1  *)
  ADC_CR1_AWDCH_2 = $00000004;  (*!<Bit 2  *)
  ADC_CR1_AWDCH_3 = $00000008;  (*!<Bit 3  *)
  ADC_CR1_AWDCH_4 = $00000010;  (*!<Bit 4  *)
  ADC_CR1_EOCIE = $00000020;  (*!<Interrupt enable for EOC                              *)
  ADC_CR1_AWDIE = $00000040;  (*!<AAnalog Watchdog interrupt enable                     *)
  ADC_CR1_JEOCIE = $00000080;  (*!<Interrupt enable for injected channels                *)
  ADC_CR1_SCAN = $00000100;  (*!<Scan mode  *)
  ADC_CR1_AWDSGL = $00000200;  (*!<Enable the watchdog on a single channel in scan mode  *)
  ADC_CR1_JAUTO = $00000400;  (*!<Automatic injected group conversion                   *)
  ADC_CR1_DISCEN = $00000800;  (*!<Discontinuous mode on regular channels                *)
  ADC_CR1_JDISCEN = $00001000;  (*!<Discontinuous mode on injected channels               *)
  ADC_CR1_DISCNUM = $0000E000;  (*!<DISCNUM[2:0] bits (Discontinuous mode channel count)  *)
  ADC_CR1_DISCNUM_0 = $00002000;  (*!<Bit 0  *)
  ADC_CR1_DISCNUM_1 = $00004000;  (*!<Bit 1  *)
  ADC_CR1_DISCNUM_2 = $00008000;  (*!<Bit 2  *)
  ADC_CR1_JAWDEN = $00400000;  (*!<Analog watchdog enable on injected channels           *)
  ADC_CR1_AWDEN = $00800000;  (*!<Analog watchdog enable on regular channels            *)
  ADC_CR1_RES = $03000000;  (*!<RES[2:0] bits (Resolution)                            *)
  ADC_CR1_RES_0 = $01000000;  (*!<Bit 0  *)
  ADC_CR1_RES_1 = $02000000;  (*!<Bit 1  *)
  ADC_CR1_OVRIE = $04000000;  (*!<overrun interrupt enable  *)
  (*******************  Bit definition for ADC_CR2 register  ******************* *)

  ADC_CR2_ADON = $00000001;  (*!<A/D Converter ON / OFF                                        *)
  ADC_CR2_CONT = $00000002;  (*!<Continuous Conversion                                         *)
  ADC_CR2_DMA = $00000100;  (*!<Direct Memory access mode                                     *)
  ADC_CR2_DDS = $00000200;  (*!<DMA disable selection (Single ADC)                            *)
  ADC_CR2_EOCS = $00000400;  (*!<End of conversion selection                                   *)
  ADC_CR2_ALIGN = $00000800;  (*!<Data Alignment                                                *)
  ADC_CR2_JEXTSEL = $000F0000;  (*!<JEXTSEL[3:0] bits (External event select for injected group)  *)
  ADC_CR2_JEXTSEL_0 = $00010000;  (*!<Bit 0  *)
  ADC_CR2_JEXTSEL_1 = $00020000;  (*!<Bit 1  *)
  ADC_CR2_JEXTSEL_2 = $00040000;  (*!<Bit 2  *)
  ADC_CR2_JEXTSEL_3 = $00080000;  (*!<Bit 3  *)
  ADC_CR2_JEXTEN = $00300000;  (*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp)  *)
  ADC_CR2_JEXTEN_0 = $00100000;  (*!<Bit 0  *)
  ADC_CR2_JEXTEN_1 = $00200000;  (*!<Bit 1  *)
  ADC_CR2_JSWSTART = $00400000;  (*!<Start Conversion of injected channels  *)
  ADC_CR2_EXTSEL = $0F000000;  (*!<EXTSEL[3:0] bits (External Event Select for regular group)  *)
  ADC_CR2_EXTSEL_0 = $01000000;  (*!<Bit 0  *)
  ADC_CR2_EXTSEL_1 = $02000000;  (*!<Bit 1  *)
  ADC_CR2_EXTSEL_2 = $04000000;  (*!<Bit 2  *)
  ADC_CR2_EXTSEL_3 = $08000000;  (*!<Bit 3  *)
  ADC_CR2_EXTEN = $30000000;  (*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp)  *)
  ADC_CR2_EXTEN_0 = $10000000;  (*!<Bit 0  *)
  ADC_CR2_EXTEN_1 = $20000000;  (*!<Bit 1  *)
  ADC_CR2_SWSTART = $40000000;  (*!<Start Conversion of regular channels  *)
  (******************  Bit definition for ADC_SMPR1 register  ****************** *)

  ADC_SMPR1_SMP10 = $00000007;  (*!<SMP10[2:0] bits (Channel 10 Sample time selection)  *)
  ADC_SMPR1_SMP10_0 = $00000001;  (*!<Bit 0  *)
  ADC_SMPR1_SMP10_1 = $00000002;  (*!<Bit 1  *)
  ADC_SMPR1_SMP10_2 = $00000004;  (*!<Bit 2  *)
  ADC_SMPR1_SMP11 = $00000038;  (*!<SMP11[2:0] bits (Channel 11 Sample time selection)  *)
  ADC_SMPR1_SMP11_0 = $00000008;  (*!<Bit 0  *)
  ADC_SMPR1_SMP11_1 = $00000010;  (*!<Bit 1  *)
  ADC_SMPR1_SMP11_2 = $00000020;  (*!<Bit 2  *)
  ADC_SMPR1_SMP12 = $000001C0;  (*!<SMP12[2:0] bits (Channel 12 Sample time selection)  *)
  ADC_SMPR1_SMP12_0 = $00000040;  (*!<Bit 0  *)
  ADC_SMPR1_SMP12_1 = $00000080;  (*!<Bit 1  *)
  ADC_SMPR1_SMP12_2 = $00000100;  (*!<Bit 2  *)
  ADC_SMPR1_SMP13 = $00000E00;  (*!<SMP13[2:0] bits (Channel 13 Sample time selection)  *)
  ADC_SMPR1_SMP13_0 = $00000200;  (*!<Bit 0  *)
  ADC_SMPR1_SMP13_1 = $00000400;  (*!<Bit 1  *)
  ADC_SMPR1_SMP13_2 = $00000800;  (*!<Bit 2  *)
  ADC_SMPR1_SMP14 = $00007000;  (*!<SMP14[2:0] bits (Channel 14 Sample time selection)  *)
  ADC_SMPR1_SMP14_0 = $00001000;  (*!<Bit 0  *)
  ADC_SMPR1_SMP14_1 = $00002000;  (*!<Bit 1  *)
  ADC_SMPR1_SMP14_2 = $00004000;  (*!<Bit 2  *)
  ADC_SMPR1_SMP15 = $00038000;  (*!<SMP15[2:0] bits (Channel 15 Sample time selection)  *)
  ADC_SMPR1_SMP15_0 = $00008000;  (*!<Bit 0  *)
  ADC_SMPR1_SMP15_1 = $00010000;  (*!<Bit 1  *)
  ADC_SMPR1_SMP15_2 = $00020000;  (*!<Bit 2  *)
  ADC_SMPR1_SMP16 = $001C0000;  (*!<SMP16[2:0] bits (Channel 16 Sample time selection)  *)
  ADC_SMPR1_SMP16_0 = $00040000;  (*!<Bit 0  *)
  ADC_SMPR1_SMP16_1 = $00080000;  (*!<Bit 1  *)
  ADC_SMPR1_SMP16_2 = $00100000;  (*!<Bit 2  *)
  ADC_SMPR1_SMP17 = $00E00000;  (*!<SMP17[2:0] bits (Channel 17 Sample time selection)  *)
  ADC_SMPR1_SMP17_0 = $00200000;  (*!<Bit 0  *)
  ADC_SMPR1_SMP17_1 = $00400000;  (*!<Bit 1  *)
  ADC_SMPR1_SMP17_2 = $00800000;  (*!<Bit 2  *)
  ADC_SMPR1_SMP18 = $07000000;  (*!<SMP18[2:0] bits (Channel 18 Sample time selection)  *)
  ADC_SMPR1_SMP18_0 = $01000000;  (*!<Bit 0  *)
  ADC_SMPR1_SMP18_1 = $02000000;  (*!<Bit 1  *)
  ADC_SMPR1_SMP18_2 = $04000000;  (*!<Bit 2  *)
  (******************  Bit definition for ADC_SMPR2 register  ****************** *)

  ADC_SMPR2_SMP0 = $00000007;  (*!<SMP0[2:0] bits (Channel 0 Sample time selection)  *)
  ADC_SMPR2_SMP0_0 = $00000001;  (*!<Bit 0  *)
  ADC_SMPR2_SMP0_1 = $00000002;  (*!<Bit 1  *)
  ADC_SMPR2_SMP0_2 = $00000004;  (*!<Bit 2  *)
  ADC_SMPR2_SMP1 = $00000038;  (*!<SMP1[2:0] bits (Channel 1 Sample time selection)  *)
  ADC_SMPR2_SMP1_0 = $00000008;  (*!<Bit 0  *)
  ADC_SMPR2_SMP1_1 = $00000010;  (*!<Bit 1  *)
  ADC_SMPR2_SMP1_2 = $00000020;  (*!<Bit 2  *)
  ADC_SMPR2_SMP2 = $000001C0;  (*!<SMP2[2:0] bits (Channel 2 Sample time selection)  *)
  ADC_SMPR2_SMP2_0 = $00000040;  (*!<Bit 0  *)
  ADC_SMPR2_SMP2_1 = $00000080;  (*!<Bit 1  *)
  ADC_SMPR2_SMP2_2 = $00000100;  (*!<Bit 2  *)
  ADC_SMPR2_SMP3 = $00000E00;  (*!<SMP3[2:0] bits (Channel 3 Sample time selection)  *)
  ADC_SMPR2_SMP3_0 = $00000200;  (*!<Bit 0  *)
  ADC_SMPR2_SMP3_1 = $00000400;  (*!<Bit 1  *)
  ADC_SMPR2_SMP3_2 = $00000800;  (*!<Bit 2  *)
  ADC_SMPR2_SMP4 = $00007000;  (*!<SMP4[2:0] bits (Channel 4 Sample time selection)  *)
  ADC_SMPR2_SMP4_0 = $00001000;  (*!<Bit 0  *)
  ADC_SMPR2_SMP4_1 = $00002000;  (*!<Bit 1  *)
  ADC_SMPR2_SMP4_2 = $00004000;  (*!<Bit 2  *)
  ADC_SMPR2_SMP5 = $00038000;  (*!<SMP5[2:0] bits (Channel 5 Sample time selection)  *)
  ADC_SMPR2_SMP5_0 = $00008000;  (*!<Bit 0  *)
  ADC_SMPR2_SMP5_1 = $00010000;  (*!<Bit 1  *)
  ADC_SMPR2_SMP5_2 = $00020000;  (*!<Bit 2  *)
  ADC_SMPR2_SMP6 = $001C0000;  (*!<SMP6[2:0] bits (Channel 6 Sample time selection)  *)
  ADC_SMPR2_SMP6_0 = $00040000;  (*!<Bit 0  *)
  ADC_SMPR2_SMP6_1 = $00080000;  (*!<Bit 1  *)
  ADC_SMPR2_SMP6_2 = $00100000;  (*!<Bit 2  *)
  ADC_SMPR2_SMP7 = $00E00000;  (*!<SMP7[2:0] bits (Channel 7 Sample time selection)  *)
  ADC_SMPR2_SMP7_0 = $00200000;  (*!<Bit 0  *)
  ADC_SMPR2_SMP7_1 = $00400000;  (*!<Bit 1  *)
  ADC_SMPR2_SMP7_2 = $00800000;  (*!<Bit 2  *)
  ADC_SMPR2_SMP8 = $07000000;  (*!<SMP8[2:0] bits (Channel 8 Sample time selection)  *)
  ADC_SMPR2_SMP8_0 = $01000000;  (*!<Bit 0  *)
  ADC_SMPR2_SMP8_1 = $02000000;  (*!<Bit 1  *)
  ADC_SMPR2_SMP8_2 = $04000000;  (*!<Bit 2  *)
  ADC_SMPR2_SMP9 = $38000000;  (*!<SMP9[2:0] bits (Channel 9 Sample time selection)  *)
  ADC_SMPR2_SMP9_0 = $08000000;  (*!<Bit 0  *)
  ADC_SMPR2_SMP9_1 = $10000000;  (*!<Bit 1  *)
  ADC_SMPR2_SMP9_2 = $20000000;  (*!<Bit 2  *)
  (******************  Bit definition for ADC_JOFR1 register  ****************** *)

  ADC_JOFR1_JOFFSET1 = $0FFF;  (*!<Data offset for injected channel 1  *)
  (******************  Bit definition for ADC_JOFR2 register  ****************** *)

  ADC_JOFR2_JOFFSET2 = $0FFF;  (*!<Data offset for injected channel 2  *)
  (******************  Bit definition for ADC_JOFR3 register  ****************** *)

  ADC_JOFR3_JOFFSET3 = $0FFF;  (*!<Data offset for injected channel 3  *)
  (******************  Bit definition for ADC_JOFR4 register  ****************** *)

  ADC_JOFR4_JOFFSET4 = $0FFF;  (*!<Data offset for injected channel 4  *)
  (*******************  Bit definition for ADC_HTR register  ******************* *)

  ADC_HTR_HT = $0FFF;  (*!<Analog watchdog high threshold  *)
  (*******************  Bit definition for ADC_LTR register  ******************* *)

  ADC_LTR_LT = $0FFF;  (*!<Analog watchdog low threshold  *)
  (*******************  Bit definition for ADC_SQR1 register  ****************** *)

  ADC_SQR1_SQ13 = $0000001F;  (*!<SQ13[4:0] bits (13th conversion in regular sequence)  *)
  ADC_SQR1_SQ13_0 = $00000001;  (*!<Bit 0  *)
  ADC_SQR1_SQ13_1 = $00000002;  (*!<Bit 1  *)
  ADC_SQR1_SQ13_2 = $00000004;  (*!<Bit 2  *)
  ADC_SQR1_SQ13_3 = $00000008;  (*!<Bit 3  *)
  ADC_SQR1_SQ13_4 = $00000010;  (*!<Bit 4  *)
  ADC_SQR1_SQ14 = $000003E0;  (*!<SQ14[4:0] bits (14th conversion in regular sequence)  *)
  ADC_SQR1_SQ14_0 = $00000020;  (*!<Bit 0  *)
  ADC_SQR1_SQ14_1 = $00000040;  (*!<Bit 1  *)
  ADC_SQR1_SQ14_2 = $00000080;  (*!<Bit 2  *)
  ADC_SQR1_SQ14_3 = $00000100;  (*!<Bit 3  *)
  ADC_SQR1_SQ14_4 = $00000200;  (*!<Bit 4  *)
  ADC_SQR1_SQ15 = $00007C00;  (*!<SQ15[4:0] bits (15th conversion in regular sequence)  *)
  ADC_SQR1_SQ15_0 = $00000400;  (*!<Bit 0  *)
  ADC_SQR1_SQ15_1 = $00000800;  (*!<Bit 1  *)
  ADC_SQR1_SQ15_2 = $00001000;  (*!<Bit 2  *)
  ADC_SQR1_SQ15_3 = $00002000;  (*!<Bit 3  *)
  ADC_SQR1_SQ15_4 = $00004000;  (*!<Bit 4  *)
  ADC_SQR1_SQ16 = $000F8000;  (*!<SQ16[4:0] bits (16th conversion in regular sequence)  *)
  ADC_SQR1_SQ16_0 = $00008000;  (*!<Bit 0  *)
  ADC_SQR1_SQ16_1 = $00010000;  (*!<Bit 1  *)
  ADC_SQR1_SQ16_2 = $00020000;  (*!<Bit 2  *)
  ADC_SQR1_SQ16_3 = $00040000;  (*!<Bit 3  *)
  ADC_SQR1_SQ16_4 = $00080000;  (*!<Bit 4  *)
  ADC_SQR1_L = $00F00000;  (*!<L[3:0] bits (Regular channel sequence length)  *)
  ADC_SQR1_L_0 = $00100000;  (*!<Bit 0  *)
  ADC_SQR1_L_1 = $00200000;  (*!<Bit 1  *)
  ADC_SQR1_L_2 = $00400000;  (*!<Bit 2  *)
  ADC_SQR1_L_3 = $00800000;  (*!<Bit 3  *)
  (*******************  Bit definition for ADC_SQR2 register  ****************** *)

  ADC_SQR2_SQ7 = $0000001F;  (*!<SQ7[4:0] bits (7th conversion in regular sequence)  *)
  ADC_SQR2_SQ7_0 = $00000001;  (*!<Bit 0  *)
  ADC_SQR2_SQ7_1 = $00000002;  (*!<Bit 1  *)
  ADC_SQR2_SQ7_2 = $00000004;  (*!<Bit 2  *)
  ADC_SQR2_SQ7_3 = $00000008;  (*!<Bit 3  *)
  ADC_SQR2_SQ7_4 = $00000010;  (*!<Bit 4  *)
  ADC_SQR2_SQ8 = $000003E0;  (*!<SQ8[4:0] bits (8th conversion in regular sequence)  *)
  ADC_SQR2_SQ8_0 = $00000020;  (*!<Bit 0  *)
  ADC_SQR2_SQ8_1 = $00000040;  (*!<Bit 1  *)
  ADC_SQR2_SQ8_2 = $00000080;  (*!<Bit 2  *)
  ADC_SQR2_SQ8_3 = $00000100;  (*!<Bit 3  *)
  ADC_SQR2_SQ8_4 = $00000200;  (*!<Bit 4  *)
  ADC_SQR2_SQ9 = $00007C00;  (*!<SQ9[4:0] bits (9th conversion in regular sequence)  *)
  ADC_SQR2_SQ9_0 = $00000400;  (*!<Bit 0  *)
  ADC_SQR2_SQ9_1 = $00000800;  (*!<Bit 1  *)
  ADC_SQR2_SQ9_2 = $00001000;  (*!<Bit 2  *)
  ADC_SQR2_SQ9_3 = $00002000;  (*!<Bit 3  *)
  ADC_SQR2_SQ9_4 = $00004000;  (*!<Bit 4  *)
  ADC_SQR2_SQ10 = $000F8000;  (*!<SQ10[4:0] bits (10th conversion in regular sequence)  *)
  ADC_SQR2_SQ10_0 = $00008000;  (*!<Bit 0  *)
  ADC_SQR2_SQ10_1 = $00010000;  (*!<Bit 1  *)
  ADC_SQR2_SQ10_2 = $00020000;  (*!<Bit 2  *)
  ADC_SQR2_SQ10_3 = $00040000;  (*!<Bit 3  *)
  ADC_SQR2_SQ10_4 = $00080000;  (*!<Bit 4  *)
  ADC_SQR2_SQ11 = $01F00000;  (*!<SQ11[4:0] bits (11th conversion in regular sequence)  *)
  ADC_SQR2_SQ11_0 = $00100000;  (*!<Bit 0  *)
  ADC_SQR2_SQ11_1 = $00200000;  (*!<Bit 1  *)
  ADC_SQR2_SQ11_2 = $00400000;  (*!<Bit 2  *)
  ADC_SQR2_SQ11_3 = $00800000;  (*!<Bit 3  *)
  ADC_SQR2_SQ11_4 = $01000000;  (*!<Bit 4  *)
  ADC_SQR2_SQ12 = $3E000000;  (*!<SQ12[4:0] bits (12th conversion in regular sequence)  *)
  ADC_SQR2_SQ12_0 = $02000000;  (*!<Bit 0  *)
  ADC_SQR2_SQ12_1 = $04000000;  (*!<Bit 1  *)
  ADC_SQR2_SQ12_2 = $08000000;  (*!<Bit 2  *)
  ADC_SQR2_SQ12_3 = $10000000;  (*!<Bit 3  *)
  ADC_SQR2_SQ12_4 = $20000000;  (*!<Bit 4  *)
  (*******************  Bit definition for ADC_SQR3 register  ****************** *)

  ADC_SQR3_SQ1 = $0000001F;  (*!<SQ1[4:0] bits (1st conversion in regular sequence)  *)
  ADC_SQR3_SQ1_0 = $00000001;  (*!<Bit 0  *)
  ADC_SQR3_SQ1_1 = $00000002;  (*!<Bit 1  *)
  ADC_SQR3_SQ1_2 = $00000004;  (*!<Bit 2  *)
  ADC_SQR3_SQ1_3 = $00000008;  (*!<Bit 3  *)
  ADC_SQR3_SQ1_4 = $00000010;  (*!<Bit 4  *)
  ADC_SQR3_SQ2 = $000003E0;  (*!<SQ2[4:0] bits (2nd conversion in regular sequence)  *)
  ADC_SQR3_SQ2_0 = $00000020;  (*!<Bit 0  *)
  ADC_SQR3_SQ2_1 = $00000040;  (*!<Bit 1  *)
  ADC_SQR3_SQ2_2 = $00000080;  (*!<Bit 2  *)
  ADC_SQR3_SQ2_3 = $00000100;  (*!<Bit 3  *)
  ADC_SQR3_SQ2_4 = $00000200;  (*!<Bit 4  *)
  ADC_SQR3_SQ3 = $00007C00;  (*!<SQ3[4:0] bits (3rd conversion in regular sequence)  *)
  ADC_SQR3_SQ3_0 = $00000400;  (*!<Bit 0  *)
  ADC_SQR3_SQ3_1 = $00000800;  (*!<Bit 1  *)
  ADC_SQR3_SQ3_2 = $00001000;  (*!<Bit 2  *)
  ADC_SQR3_SQ3_3 = $00002000;  (*!<Bit 3  *)
  ADC_SQR3_SQ3_4 = $00004000;  (*!<Bit 4  *)
  ADC_SQR3_SQ4 = $000F8000;  (*!<SQ4[4:0] bits (4th conversion in regular sequence)  *)
  ADC_SQR3_SQ4_0 = $00008000;  (*!<Bit 0  *)
  ADC_SQR3_SQ4_1 = $00010000;  (*!<Bit 1  *)
  ADC_SQR3_SQ4_2 = $00020000;  (*!<Bit 2  *)
  ADC_SQR3_SQ4_3 = $00040000;  (*!<Bit 3  *)
  ADC_SQR3_SQ4_4 = $00080000;  (*!<Bit 4  *)
  ADC_SQR3_SQ5 = $01F00000;  (*!<SQ5[4:0] bits (5th conversion in regular sequence)  *)
  ADC_SQR3_SQ5_0 = $00100000;  (*!<Bit 0  *)
  ADC_SQR3_SQ5_1 = $00200000;  (*!<Bit 1  *)
  ADC_SQR3_SQ5_2 = $00400000;  (*!<Bit 2  *)
  ADC_SQR3_SQ5_3 = $00800000;  (*!<Bit 3  *)
  ADC_SQR3_SQ5_4 = $01000000;  (*!<Bit 4  *)
  ADC_SQR3_SQ6 = $3E000000;  (*!<SQ6[4:0] bits (6th conversion in regular sequence)  *)
  ADC_SQR3_SQ6_0 = $02000000;  (*!<Bit 0  *)
  ADC_SQR3_SQ6_1 = $04000000;  (*!<Bit 1  *)
  ADC_SQR3_SQ6_2 = $08000000;  (*!<Bit 2  *)
  ADC_SQR3_SQ6_3 = $10000000;  (*!<Bit 3  *)
  ADC_SQR3_SQ6_4 = $20000000;  (*!<Bit 4  *)
  (*******************  Bit definition for ADC_JSQR register  ****************** *)

  ADC_JSQR_JSQ1 = $0000001F;  (*!<JSQ1[4:0] bits (1st conversion in injected sequence)  *)
  ADC_JSQR_JSQ1_0 = $00000001;  (*!<Bit 0  *)
  ADC_JSQR_JSQ1_1 = $00000002;  (*!<Bit 1  *)
  ADC_JSQR_JSQ1_2 = $00000004;  (*!<Bit 2  *)
  ADC_JSQR_JSQ1_3 = $00000008;  (*!<Bit 3  *)
  ADC_JSQR_JSQ1_4 = $00000010;  (*!<Bit 4  *)
  ADC_JSQR_JSQ2 = $000003E0;  (*!<JSQ2[4:0] bits (2nd conversion in injected sequence)  *)
  ADC_JSQR_JSQ2_0 = $00000020;  (*!<Bit 0  *)
  ADC_JSQR_JSQ2_1 = $00000040;  (*!<Bit 1  *)
  ADC_JSQR_JSQ2_2 = $00000080;  (*!<Bit 2  *)
  ADC_JSQR_JSQ2_3 = $00000100;  (*!<Bit 3  *)
  ADC_JSQR_JSQ2_4 = $00000200;  (*!<Bit 4  *)
  ADC_JSQR_JSQ3 = $00007C00;  (*!<JSQ3[4:0] bits (3rd conversion in injected sequence)  *)
  ADC_JSQR_JSQ3_0 = $00000400;  (*!<Bit 0  *)
  ADC_JSQR_JSQ3_1 = $00000800;  (*!<Bit 1  *)
  ADC_JSQR_JSQ3_2 = $00001000;  (*!<Bit 2  *)
  ADC_JSQR_JSQ3_3 = $00002000;  (*!<Bit 3  *)
  ADC_JSQR_JSQ3_4 = $00004000;  (*!<Bit 4  *)
  ADC_JSQR_JSQ4 = $000F8000;  (*!<JSQ4[4:0] bits (4th conversion in injected sequence)  *)
  ADC_JSQR_JSQ4_0 = $00008000;  (*!<Bit 0  *)
  ADC_JSQR_JSQ4_1 = $00010000;  (*!<Bit 1  *)
  ADC_JSQR_JSQ4_2 = $00020000;  (*!<Bit 2  *)
  ADC_JSQR_JSQ4_3 = $00040000;  (*!<Bit 3  *)
  ADC_JSQR_JSQ4_4 = $00080000;  (*!<Bit 4  *)
  ADC_JSQR_JL = $00300000;  (*!<JL[1:0] bits (Injected Sequence length)  *)
  ADC_JSQR_JL_0 = $00100000;  (*!<Bit 0  *)
  ADC_JSQR_JL_1 = $00200000;  (*!<Bit 1  *)
  (*******************  Bit definition for ADC_JDR1 register  ****************** *)

  ADC_JDR1_JDATA = $FFFF;  (*!<Injected data  *)
  (*******************  Bit definition for ADC_JDR2 register  ****************** *)

  ADC_JDR2_JDATA = $FFFF;  (*!<Injected data  *)
  (*******************  Bit definition for ADC_JDR3 register  ****************** *)

  ADC_JDR3_JDATA = $FFFF;  (*!<Injected data  *)
  (*******************  Bit definition for ADC_JDR4 register  ****************** *)

  ADC_JDR4_JDATA = $FFFF;  (*!<Injected data  *)
  (********************  Bit definition for ADC_DR register  ******************* *)

  ADC_DR_DATA = $0000FFFF;  (*!<Regular data  *)
  ADC_DR_ADC2DATA = $FFFF0000;  (*!<ADC2 data  *)
  (*******************  Bit definition for ADC_CSR register  ******************* *)

  ADC_CSR_AWD1 = $00000001;  (*!<ADC1 Analog watchdog flag                *)
  ADC_CSR_EOC1 = $00000002;  (*!<ADC1 End of conversion                   *)
  ADC_CSR_JEOC1 = $00000004;  (*!<ADC1 Injected channel end of conversion  *)
  ADC_CSR_JSTRT1 = $00000008;  (*!<ADC1 Injected channel Start flag         *)
  ADC_CSR_STRT1 = $00000010;  (*!<ADC1 Regular channel Start flag          *)
  ADC_CSR_DOVR1 = $00000020;  (*!<ADC1 DMA overrun  flag                   *)
  ADC_CSR_AWD2 = $00000100;  (*!<ADC2 Analog watchdog flag                *)
  ADC_CSR_EOC2 = $00000200;  (*!<ADC2 End of conversion                   *)
  ADC_CSR_JEOC2 = $00000400;  (*!<ADC2 Injected channel end of conversion  *)
  ADC_CSR_JSTRT2 = $00000800;  (*!<ADC2 Injected channel Start flag         *)
  ADC_CSR_STRT2 = $00001000;  (*!<ADC2 Regular channel Start flag          *)
  ADC_CSR_DOVR2 = $00002000;  (*!<ADC2 DMA overrun  flag                   *)
  ADC_CSR_AWD3 = $00010000;  (*!<ADC3 Analog watchdog flag                *)
  ADC_CSR_EOC3 = $00020000;  (*!<ADC3 End of conversion                   *)
  ADC_CSR_JEOC3 = $00040000;  (*!<ADC3 Injected channel end of conversion  *)
  ADC_CSR_JSTRT3 = $00080000;  (*!<ADC3 Injected channel Start flag         *)
  ADC_CSR_STRT3 = $00100000;  (*!<ADC3 Regular channel Start flag          *)
  ADC_CSR_DOVR3 = $00200000;  (*!<ADC3 DMA overrun  flag                   *)
  (*******************  Bit definition for ADC_CCR register  ******************* *)

  ADC_CCR_MULTI = $0000001F;  (*!<MULTI[4:0] bits (Multi-ADC mode selection)  *)
  ADC_CCR_MULTI_0 = $00000001;  (*!<Bit 0  *)
  ADC_CCR_MULTI_1 = $00000002;  (*!<Bit 1  *)
  ADC_CCR_MULTI_2 = $00000004;  (*!<Bit 2  *)
  ADC_CCR_MULTI_3 = $00000008;  (*!<Bit 3  *)
  ADC_CCR_MULTI_4 = $00000010;  (*!<Bit 4  *)
  ADC_CCR_DELAY = $00000F00;  (*!<DELAY[3:0] bits (Delay between 2 sampling phases)  *)
  ADC_CCR_DELAY_0 = $00000100;  (*!<Bit 0  *)
  ADC_CCR_DELAY_1 = $00000200;  (*!<Bit 1  *)
  ADC_CCR_DELAY_2 = $00000400;  (*!<Bit 2  *)
  ADC_CCR_DELAY_3 = $00000800;  (*!<Bit 3  *)
  ADC_CCR_DDS = $00002000;  (*!<DMA disable selection (Multi-ADC mode)  *)
  ADC_CCR_DMA = $0000C000;  (*!<DMA[1:0] bits (Direct Memory Access mode for multimode)  *)
  ADC_CCR_DMA_0 = $00004000;  (*!<Bit 0  *)
  ADC_CCR_DMA_1 = $00008000;  (*!<Bit 1  *)
  ADC_CCR_ADCPRE = $00030000;  (*!<ADCPRE[1:0] bits (ADC prescaler)  *)
  ADC_CCR_ADCPRE_0 = $00010000;  (*!<Bit 0  *)
  ADC_CCR_ADCPRE_1 = $00020000;  (*!<Bit 1  *)
  ADC_CCR_VBATE = $00400000;  (*!<VBAT Enable  *)
  ADC_CCR_TSVREFE = $00800000;  (*!<Temperature Sensor and VREFINT Enable  *)
  (*******************  Bit definition for ADC_CDR register  ******************* *)

  ADC_CDR_DATA1 = $0000FFFF;  (*!<1st data of a pair of regular conversions  *)
  ADC_CDR_DATA2 = $FFFF0000;  (*!<2nd data of a pair of regular conversions  *)

(***************************************************************************** *)

(*                                                                             *)

(*                         Controller Area Network                             *)

(*                                                                             *)

(***************************************************************************** *)

(*!<CAN control and status registers  *)

(*******************  Bit definition for CAN_MCR register  ******************* *)

const
  CAN_MCR_INRQ = $00000001;  (*!<Initialization Request             *)
  CAN_MCR_SLEEP = $00000002;  (*!<Sleep Mode Request                 *)
  CAN_MCR_TXFP = $00000004;  (*!<Transmit FIFO Priority             *)
  CAN_MCR_RFLM = $00000008;  (*!<Receive FIFO Locked Mode           *)
  CAN_MCR_NART = $00000010;  (*!<No Automatic Retransmission        *)
  CAN_MCR_AWUM = $00000020;  (*!<Automatic Wakeup Mode              *)
  CAN_MCR_ABOM = $00000040;  (*!<Automatic Bus-Off Management       *)
  CAN_MCR_TTCM = $00000080;  (*!<Time Triggered Communication Mode  *)
  CAN_MCR_RESET = $00008000;  (*!<bxCAN software master reset        *)
  (*******************  Bit definition for CAN_MSR register  ******************* *)

  CAN_MSR_INAK = $00000001;  (*!<Initialization Acknowledge   *)
  CAN_MSR_SLAK = $00000002;  (*!<Sleep Acknowledge            *)
  CAN_MSR_ERRI = $00000004;  (*!<Error Interrupt              *)
  CAN_MSR_WKUI = $00000008;  (*!<Wakeup Interrupt             *)
  CAN_MSR_SLAKI = $00000010;  (*!<Sleep Acknowledge Interrupt  *)
  CAN_MSR_TXM = $00000100;  (*!<Transmit Mode                *)
  CAN_MSR_RXM = $00000200;  (*!<Receive Mode                 *)
  CAN_MSR_SAMP = $00000400;  (*!<Last Sample Point            *)
  CAN_MSR_RX = $00000800;  (*!<CAN Rx Signal                *)
  (*******************  Bit definition for CAN_TSR register  ******************* *)

  CAN_TSR_RQCP0 = $00000001;  (*!<Request Completed Mailbox0       *)
  CAN_TSR_TXOK0 = $00000002;  (*!<Transmission OK of Mailbox0      *)
  CAN_TSR_ALST0 = $00000004;  (*!<Arbitration Lost for Mailbox0    *)
  CAN_TSR_TERR0 = $00000008;  (*!<Transmission Error of Mailbox0   *)
  CAN_TSR_ABRQ0 = $00000080;  (*!<Abort Request for Mailbox0       *)
  CAN_TSR_RQCP1 = $00000100;  (*!<Request Completed Mailbox1       *)
  CAN_TSR_TXOK1 = $00000200;  (*!<Transmission OK of Mailbox1      *)
  CAN_TSR_ALST1 = $00000400;  (*!<Arbitration Lost for Mailbox1    *)
  CAN_TSR_TERR1 = $00000800;  (*!<Transmission Error of Mailbox1   *)
  CAN_TSR_ABRQ1 = $00008000;  (*!<Abort Request for Mailbox 1      *)
  CAN_TSR_RQCP2 = $00010000;  (*!<Request Completed Mailbox2       *)
  CAN_TSR_TXOK2 = $00020000;  (*!<Transmission OK of Mailbox 2     *)
  CAN_TSR_ALST2 = $00040000;  (*!<Arbitration Lost for mailbox 2   *)
  CAN_TSR_TERR2 = $00080000;  (*!<Transmission Error of Mailbox 2  *)
  CAN_TSR_ABRQ2 = $00800000;  (*!<Abort Request for Mailbox 2      *)
  CAN_TSR_CODE = $03000000;  (*!<Mailbox Code                     *)
  CAN_TSR_TME = $1C000000;  (*!<TME[2:0] bits  *)
  CAN_TSR_TME0 = $04000000;  (*!<Transmit Mailbox 0 Empty  *)
  CAN_TSR_TME1 = $08000000;  (*!<Transmit Mailbox 1 Empty  *)
  CAN_TSR_TME2 = $10000000;  (*!<Transmit Mailbox 2 Empty  *)
  CAN_TSR_LOW = $E0000000;  (*!<LOW[2:0] bits  *)
  CAN_TSR_LOW0 = $20000000;  (*!<Lowest Priority Flag for Mailbox 0  *)
  CAN_TSR_LOW1 = $40000000;  (*!<Lowest Priority Flag for Mailbox 1  *)
  CAN_TSR_LOW2 = $80000000;  (*!<Lowest Priority Flag for Mailbox 2  *)
  (*******************  Bit definition for CAN_RF0R register  ****************** *)

  CAN_RF0R_FMP0 = $00000003;  (*!<FIFO 0 Message Pending         *)
  CAN_RF0R_FULL0 = $00000008;  (*!<FIFO 0 Full                    *)
  CAN_RF0R_FOVR0 = $00000010;  (*!<FIFO 0 Overrun                 *)
  CAN_RF0R_RFOM0 = $00000020;  (*!<Release FIFO 0 Output Mailbox  *)
  (*******************  Bit definition for CAN_RF1R register  ****************** *)

  CAN_RF1R_FMP1 = $00000003;  (*!<FIFO 1 Message Pending         *)
  CAN_RF1R_FULL1 = $00000008;  (*!<FIFO 1 Full                    *)
  CAN_RF1R_FOVR1 = $00000010;  (*!<FIFO 1 Overrun                 *)
  CAN_RF1R_RFOM1 = $00000020;  (*!<Release FIFO 1 Output Mailbox  *)
  (********************  Bit definition for CAN_IER register  ****************** *)

  CAN_IER_TMEIE = $00000001;  (*!<Transmit Mailbox Empty Interrupt Enable  *)
  CAN_IER_FMPIE0 = $00000002;  (*!<FIFO Message Pending Interrupt Enable    *)
  CAN_IER_FFIE0 = $00000004;  (*!<FIFO Full Interrupt Enable               *)
  CAN_IER_FOVIE0 = $00000008;  (*!<FIFO Overrun Interrupt Enable            *)
  CAN_IER_FMPIE1 = $00000010;  (*!<FIFO Message Pending Interrupt Enable    *)
  CAN_IER_FFIE1 = $00000020;  (*!<FIFO Full Interrupt Enable               *)
  CAN_IER_FOVIE1 = $00000040;  (*!<FIFO Overrun Interrupt Enable            *)
  CAN_IER_EWGIE = $00000100;  (*!<Error Warning Interrupt Enable           *)
  CAN_IER_EPVIE = $00000200;  (*!<Error Passive Interrupt Enable           *)
  CAN_IER_BOFIE = $00000400;  (*!<Bus-Off Interrupt Enable                 *)
  CAN_IER_LECIE = $00000800;  (*!<Last Error Code Interrupt Enable         *)
  CAN_IER_ERRIE = $00008000;  (*!<Error Interrupt Enable                   *)
  CAN_IER_WKUIE = $00010000;  (*!<Wakeup Interrupt Enable                  *)
  CAN_IER_SLKIE = $00020000;  (*!<Sleep Interrupt Enable                   *)
  (********************  Bit definition for CAN_ESR register  ****************** *)

  CAN_ESR_EWGF = $00000001;  (*!<Error Warning Flag  *)
  CAN_ESR_EPVF = $00000002;  (*!<Error Passive Flag  *)
  CAN_ESR_BOFF = $00000004;  (*!<Bus-Off Flag  *)
  CAN_ESR_LEC = $00000070;  (*!<LEC[2:0] bits (Last Error Code)  *)
  CAN_ESR_LEC_0 = $00000010;  (*!<Bit 0  *)
  CAN_ESR_LEC_1 = $00000020;  (*!<Bit 1  *)
  CAN_ESR_LEC_2 = $00000040;  (*!<Bit 2  *)
  CAN_ESR_TEC = $00FF0000;  (*!<Least significant byte of the 9-bit Transmit Error Counter  *)
  CAN_ESR_REC = $FF000000;  (*!<Receive Error Counter  *)
  (*******************  Bit definition for CAN_BTR register  ******************* *)

  CAN_BTR_BRP = $000003FF;  (*!<Baud Rate Prescaler            *)
  CAN_BTR_TS1 = $000F0000;  (*!<Time Segment 1                 *)
  CAN_BTR_TS1_0 = $00010000;  (*!<Bit 0  *)
  CAN_BTR_TS1_1 = $00020000;  (*!<Bit 1  *)
  CAN_BTR_TS1_2 = $00040000;  (*!<Bit 2  *)
  CAN_BTR_TS1_3 = $00080000;  (*!<Bit 3  *)
  CAN_BTR_TS2 = $00700000;  (*!<Time Segment 2                 *)
  CAN_BTR_TS2_0 = $00100000;  (*!<Bit 0  *)
  CAN_BTR_TS2_1 = $00200000;  (*!<Bit 1  *)
  CAN_BTR_TS2_2 = $00400000;  (*!<Bit 2  *)
  CAN_BTR_SJW = $03000000;  (*!<Resynchronization Jump Width   *)
  CAN_BTR_SJW_0 = $01000000;  (*!<Bit 0  *)
  CAN_BTR_SJW_1 = $02000000;  (*!<Bit 1  *)
  CAN_BTR_LBKM = $40000000;  (*!<Loop Back Mode (Debug)         *)
  CAN_BTR_SILM = $80000000;  (*!<Silent Mode                    *)
  (*!<Mailbox registers  *)

  (******************  Bit definition for CAN_TI0R register  ******************* *)

  CAN_TI0R_TXRQ = $00000001;  (*!<Transmit Mailbox Request                    *)
  CAN_TI0R_RTR = $00000002;  (*!<Remote Transmission Request                 *)
  CAN_TI0R_IDE = $00000004;  (*!<Identifier Extension                        *)
  CAN_TI0R_EXID = $001FFFF8;  (*!<Extended Identifier                         *)
  CAN_TI0R_STID = $FFE00000;  (*!<Standard Identifier or Extended Identifier  *)
  (******************  Bit definition for CAN_TDT0R register  ****************** *)

  CAN_TDT0R_DLC = $0000000F;  (*!<Data Length Code      *)
  CAN_TDT0R_TGT = $00000100;  (*!<Transmit Global Time  *)
  CAN_TDT0R_TIME = $FFFF0000;  (*!<Message Time Stamp    *)
  (******************  Bit definition for CAN_TDL0R register  ****************** *)

  CAN_TDL0R_DATA0 = $000000FF;  (*!<Data byte 0  *)
  CAN_TDL0R_DATA1 = $0000FF00;  (*!<Data byte 1  *)
  CAN_TDL0R_DATA2 = $00FF0000;  (*!<Data byte 2  *)
  CAN_TDL0R_DATA3 = $FF000000;  (*!<Data byte 3  *)
  (******************  Bit definition for CAN_TDH0R register  ****************** *)

  CAN_TDH0R_DATA4 = $000000FF;  (*!<Data byte 4  *)
  CAN_TDH0R_DATA5 = $0000FF00;  (*!<Data byte 5  *)
  CAN_TDH0R_DATA6 = $00FF0000;  (*!<Data byte 6  *)
  CAN_TDH0R_DATA7 = $FF000000;  (*!<Data byte 7  *)
  (*******************  Bit definition for CAN_TI1R register  ****************** *)

  CAN_TI1R_TXRQ = $00000001;  (*!<Transmit Mailbox Request                    *)
  CAN_TI1R_RTR = $00000002;  (*!<Remote Transmission Request                 *)
  CAN_TI1R_IDE = $00000004;  (*!<Identifier Extension                        *)
  CAN_TI1R_EXID = $001FFFF8;  (*!<Extended Identifier                         *)
  CAN_TI1R_STID = $FFE00000;  (*!<Standard Identifier or Extended Identifier  *)
  (*******************  Bit definition for CAN_TDT1R register  ***************** *)

  CAN_TDT1R_DLC = $0000000F;  (*!<Data Length Code      *)
  CAN_TDT1R_TGT = $00000100;  (*!<Transmit Global Time  *)
  CAN_TDT1R_TIME = $FFFF0000;  (*!<Message Time Stamp    *)
  (*******************  Bit definition for CAN_TDL1R register  ***************** *)

  CAN_TDL1R_DATA0 = $000000FF;  (*!<Data byte 0  *)
  CAN_TDL1R_DATA1 = $0000FF00;  (*!<Data byte 1  *)
  CAN_TDL1R_DATA2 = $00FF0000;  (*!<Data byte 2  *)
  CAN_TDL1R_DATA3 = $FF000000;  (*!<Data byte 3  *)
  (*******************  Bit definition for CAN_TDH1R register  ***************** *)

  CAN_TDH1R_DATA4 = $000000FF;  (*!<Data byte 4  *)
  CAN_TDH1R_DATA5 = $0000FF00;  (*!<Data byte 5  *)
  CAN_TDH1R_DATA6 = $00FF0000;  (*!<Data byte 6  *)
  CAN_TDH1R_DATA7 = $FF000000;  (*!<Data byte 7  *)
  (*******************  Bit definition for CAN_TI2R register  ****************** *)

  CAN_TI2R_TXRQ = $00000001;  (*!<Transmit Mailbox Request                    *)
  CAN_TI2R_RTR = $00000002;  (*!<Remote Transmission Request                 *)
  CAN_TI2R_IDE = $00000004;  (*!<Identifier Extension                        *)
  CAN_TI2R_EXID = $001FFFF8;  (*!<Extended identifier                         *)
  CAN_TI2R_STID = $FFE00000;  (*!<Standard Identifier or Extended Identifier  *)
  (*******************  Bit definition for CAN_TDT2R register  ***************** *)

  CAN_TDT2R_DLC = $0000000F;  (*!<Data Length Code       *)
  CAN_TDT2R_TGT = $00000100;  (*!<Transmit Global Time   *)
  CAN_TDT2R_TIME = $FFFF0000;  (*!<Message Time Stamp     *)
  (*******************  Bit definition for CAN_TDL2R register  ***************** *)

  CAN_TDL2R_DATA0 = $000000FF;  (*!<Data byte 0  *)
  CAN_TDL2R_DATA1 = $0000FF00;  (*!<Data byte 1  *)
  CAN_TDL2R_DATA2 = $00FF0000;  (*!<Data byte 2  *)
  CAN_TDL2R_DATA3 = $FF000000;  (*!<Data byte 3  *)
  (*******************  Bit definition for CAN_TDH2R register  ***************** *)

  CAN_TDH2R_DATA4 = $000000FF;  (*!<Data byte 4  *)
  CAN_TDH2R_DATA5 = $0000FF00;  (*!<Data byte 5  *)
  CAN_TDH2R_DATA6 = $00FF0000;  (*!<Data byte 6  *)
  CAN_TDH2R_DATA7 = $FF000000;  (*!<Data byte 7  *)
  (*******************  Bit definition for CAN_RI0R register  ****************** *)

  CAN_RI0R_RTR = $00000002;  (*!<Remote Transmission Request                 *)
  CAN_RI0R_IDE = $00000004;  (*!<Identifier Extension                        *)
  CAN_RI0R_EXID = $001FFFF8;  (*!<Extended Identifier                         *)
  CAN_RI0R_STID = $FFE00000;  (*!<Standard Identifier or Extended Identifier  *)
  (*******************  Bit definition for CAN_RDT0R register  ***************** *)

  CAN_RDT0R_DLC = $0000000F;  (*!<Data Length Code  *)
  CAN_RDT0R_FMI = $0000FF00;  (*!<Filter Match Index  *)
  CAN_RDT0R_TIME = $FFFF0000;  (*!<Message Time Stamp  *)
  (*******************  Bit definition for CAN_RDL0R register  ***************** *)

  CAN_RDL0R_DATA0 = $000000FF;  (*!<Data byte 0  *)
  CAN_RDL0R_DATA1 = $0000FF00;  (*!<Data byte 1  *)
  CAN_RDL0R_DATA2 = $00FF0000;  (*!<Data byte 2  *)
  CAN_RDL0R_DATA3 = $FF000000;  (*!<Data byte 3  *)
  (*******************  Bit definition for CAN_RDH0R register  ***************** *)

  CAN_RDH0R_DATA4 = $000000FF;  (*!<Data byte 4  *)
  CAN_RDH0R_DATA5 = $0000FF00;  (*!<Data byte 5  *)
  CAN_RDH0R_DATA6 = $00FF0000;  (*!<Data byte 6  *)
  CAN_RDH0R_DATA7 = $FF000000;  (*!<Data byte 7  *)
  (*******************  Bit definition for CAN_RI1R register  ****************** *)

  CAN_RI1R_RTR = $00000002;  (*!<Remote Transmission Request                 *)
  CAN_RI1R_IDE = $00000004;  (*!<Identifier Extension                        *)
  CAN_RI1R_EXID = $001FFFF8;  (*!<Extended identifier                         *)
  CAN_RI1R_STID = $FFE00000;  (*!<Standard Identifier or Extended Identifier  *)
  (*******************  Bit definition for CAN_RDT1R register  ***************** *)

  CAN_RDT1R_DLC = $0000000F;  (*!<Data Length Code    *)
  CAN_RDT1R_FMI = $0000FF00;  (*!<Filter Match Index  *)
  CAN_RDT1R_TIME = $FFFF0000;  (*!<Message Time Stamp  *)
  (*******************  Bit definition for CAN_RDL1R register  ***************** *)

  CAN_RDL1R_DATA0 = $000000FF;  (*!<Data byte 0  *)
  CAN_RDL1R_DATA1 = $0000FF00;  (*!<Data byte 1  *)
  CAN_RDL1R_DATA2 = $00FF0000;  (*!<Data byte 2  *)
  CAN_RDL1R_DATA3 = $FF000000;  (*!<Data byte 3  *)
  (*******************  Bit definition for CAN_RDH1R register  ***************** *)

  CAN_RDH1R_DATA4 = $000000FF;  (*!<Data byte 4  *)
  CAN_RDH1R_DATA5 = $0000FF00;  (*!<Data byte 5  *)
  CAN_RDH1R_DATA6 = $00FF0000;  (*!<Data byte 6  *)
  CAN_RDH1R_DATA7 = $FF000000;  (*!<Data byte 7  *)
  (*!<CAN filter registers  *)

  (*******************  Bit definition for CAN_FMR register  ******************* *)

  CAN_FMR_FINIT = $01;  (*!<Filter Init Mode  *)
  CAN_FMR_CAN2SB = $00003F00;  (*!<CAN2 start bank  *)
  (*******************  Bit definition for CAN_FM1R register  ****************** *)

  CAN_FM1R_FBM = $3FFF;  (*!<Filter Mode  *)
  CAN_FM1R_FBM0 = $0001;  (*!<Filter Init Mode bit 0   *)
  CAN_FM1R_FBM1 = $0002;  (*!<Filter Init Mode bit 1   *)
  CAN_FM1R_FBM2 = $0004;  (*!<Filter Init Mode bit 2   *)
  CAN_FM1R_FBM3 = $0008;  (*!<Filter Init Mode bit 3   *)
  CAN_FM1R_FBM4 = $0010;  (*!<Filter Init Mode bit 4   *)
  CAN_FM1R_FBM5 = $0020;  (*!<Filter Init Mode bit 5   *)
  CAN_FM1R_FBM6 = $0040;  (*!<Filter Init Mode bit 6   *)
  CAN_FM1R_FBM7 = $0080;  (*!<Filter Init Mode bit 7   *)
  CAN_FM1R_FBM8 = $0100;  (*!<Filter Init Mode bit 8   *)
  CAN_FM1R_FBM9 = $0200;  (*!<Filter Init Mode bit 9   *)
  CAN_FM1R_FBM10 = $0400;  (*!<Filter Init Mode bit 10  *)
  CAN_FM1R_FBM11 = $0800;  (*!<Filter Init Mode bit 11  *)
  CAN_FM1R_FBM12 = $1000;  (*!<Filter Init Mode bit 12  *)
  CAN_FM1R_FBM13 = $2000;  (*!<Filter Init Mode bit 13  *)
  (*******************  Bit definition for CAN_FS1R register  ****************** *)

  CAN_FS1R_FSC = $00003FFF;  (*!<Filter Scale Configuration         *)
  CAN_FS1R_FSC0 = $00000001;  (*!<Filter Scale Configuration bit 0   *)
  CAN_FS1R_FSC1 = $00000002;  (*!<Filter Scale Configuration bit 1   *)
  CAN_FS1R_FSC2 = $00000004;  (*!<Filter Scale Configuration bit 2   *)
  CAN_FS1R_FSC3 = $00000008;  (*!<Filter Scale Configuration bit 3   *)
  CAN_FS1R_FSC4 = $00000010;  (*!<Filter Scale Configuration bit 4   *)
  CAN_FS1R_FSC5 = $00000020;  (*!<Filter Scale Configuration bit 5   *)
  CAN_FS1R_FSC6 = $00000040;  (*!<Filter Scale Configuration bit 6   *)
  CAN_FS1R_FSC7 = $00000080;  (*!<Filter Scale Configuration bit 7   *)
  CAN_FS1R_FSC8 = $00000100;  (*!<Filter Scale Configuration bit 8   *)
  CAN_FS1R_FSC9 = $00000200;  (*!<Filter Scale Configuration bit 9   *)
  CAN_FS1R_FSC10 = $00000400;  (*!<Filter Scale Configuration bit 10  *)
  CAN_FS1R_FSC11 = $00000800;  (*!<Filter Scale Configuration bit 11  *)
  CAN_FS1R_FSC12 = $00001000;  (*!<Filter Scale Configuration bit 12  *)
  CAN_FS1R_FSC13 = $00002000;  (*!<Filter Scale Configuration bit 13  *)
  (******************  Bit definition for CAN_FFA1R register  ****************** *)

  CAN_FFA1R_FFA = $00003FFF;  (*!<Filter FIFO Assignment  *)
  CAN_FFA1R_FFA0 = $00000001;  (*!<Filter FIFO Assignment for Filter 0  *)
  CAN_FFA1R_FFA1 = $00000002;  (*!<Filter FIFO Assignment for Filter 1  *)
  CAN_FFA1R_FFA2 = $00000004;  (*!<Filter FIFO Assignment for Filter 2  *)
  CAN_FFA1R_FFA3 = $00000008;  (*!<Filter FIFO Assignment for Filter 3  *)
  CAN_FFA1R_FFA4 = $00000010;  (*!<Filter FIFO Assignment for Filter 4  *)
  CAN_FFA1R_FFA5 = $00000020;  (*!<Filter FIFO Assignment for Filter 5  *)
  CAN_FFA1R_FFA6 = $00000040;  (*!<Filter FIFO Assignment for Filter 6  *)
  CAN_FFA1R_FFA7 = $00000080;  (*!<Filter FIFO Assignment for Filter 7  *)
  CAN_FFA1R_FFA8 = $00000100;  (*!<Filter FIFO Assignment for Filter 8  *)
  CAN_FFA1R_FFA9 = $00000200;  (*!<Filter FIFO Assignment for Filter 9  *)
  CAN_FFA1R_FFA10 = $00000400;  (*!<Filter FIFO Assignment for Filter 10  *)
  CAN_FFA1R_FFA11 = $00000800;  (*!<Filter FIFO Assignment for Filter 11  *)
  CAN_FFA1R_FFA12 = $00001000;  (*!<Filter FIFO Assignment for Filter 12  *)
  CAN_FFA1R_FFA13 = $00002000;  (*!<Filter FIFO Assignment for Filter 13  *)
  (*******************  Bit definition for CAN_FA1R register  ****************** *)

  CAN_FA1R_FACT = $00003FFF;  (*!<Filter Active     *)
  CAN_FA1R_FACT0 = $00000001;  (*!<Filter 0 Active   *)
  CAN_FA1R_FACT1 = $00000002;  (*!<Filter 1 Active   *)
  CAN_FA1R_FACT2 = $00000004;  (*!<Filter 2 Active   *)
  CAN_FA1R_FACT3 = $00000008;  (*!<Filter 3 Active   *)
  CAN_FA1R_FACT4 = $00000010;  (*!<Filter 4 Active   *)
  CAN_FA1R_FACT5 = $00000020;  (*!<Filter 5 Active   *)
  CAN_FA1R_FACT6 = $00000040;  (*!<Filter 6 Active   *)
  CAN_FA1R_FACT7 = $00000080;  (*!<Filter 7 Active   *)
  CAN_FA1R_FACT8 = $00000100;  (*!<Filter 8 Active   *)
  CAN_FA1R_FACT9 = $00000200;  (*!<Filter 9 Active   *)
  CAN_FA1R_FACT10 = $00000400;  (*!<Filter 10 Active  *)
  CAN_FA1R_FACT11 = $00000800;  (*!<Filter 11 Active  *)
  CAN_FA1R_FACT12 = $00001000;  (*!<Filter 12 Active  *)
  CAN_FA1R_FACT13 = $00002000;  (*!<Filter 13 Active  *)
  (*******************  Bit definition for CAN_F0R1 register  ****************** *)

  CAN_F0R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F0R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F0R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F0R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F0R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F0R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F0R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F0R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F0R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F0R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F0R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F0R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F0R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F0R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F0R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F0R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F0R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F0R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F0R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F0R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F0R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F0R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F0R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F0R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F0R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F0R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F0R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F0R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F0R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F0R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F0R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F0R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F1R1 register  ****************** *)

  CAN_F1R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F1R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F1R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F1R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F1R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F1R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F1R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F1R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F1R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F1R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F1R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F1R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F1R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F1R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F1R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F1R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F1R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F1R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F1R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F1R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F1R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F1R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F1R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F1R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F1R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F1R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F1R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F1R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F1R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F1R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F1R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F1R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F2R1 register  ****************** *)

  CAN_F2R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F2R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F2R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F2R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F2R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F2R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F2R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F2R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F2R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F2R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F2R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F2R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F2R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F2R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F2R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F2R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F2R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F2R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F2R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F2R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F2R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F2R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F2R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F2R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F2R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F2R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F2R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F2R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F2R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F2R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F2R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F2R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F3R1 register  ****************** *)

  CAN_F3R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F3R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F3R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F3R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F3R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F3R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F3R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F3R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F3R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F3R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F3R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F3R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F3R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F3R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F3R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F3R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F3R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F3R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F3R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F3R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F3R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F3R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F3R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F3R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F3R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F3R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F3R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F3R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F3R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F3R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F3R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F3R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F4R1 register  ****************** *)

  CAN_F4R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F4R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F4R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F4R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F4R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F4R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F4R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F4R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F4R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F4R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F4R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F4R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F4R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F4R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F4R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F4R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F4R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F4R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F4R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F4R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F4R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F4R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F4R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F4R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F4R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F4R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F4R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F4R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F4R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F4R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F4R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F4R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F5R1 register  ****************** *)

  CAN_F5R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F5R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F5R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F5R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F5R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F5R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F5R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F5R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F5R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F5R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F5R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F5R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F5R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F5R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F5R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F5R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F5R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F5R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F5R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F5R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F5R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F5R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F5R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F5R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F5R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F5R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F5R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F5R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F5R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F5R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F5R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F5R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F6R1 register  ****************** *)

  CAN_F6R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F6R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F6R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F6R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F6R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F6R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F6R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F6R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F6R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F6R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F6R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F6R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F6R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F6R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F6R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F6R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F6R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F6R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F6R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F6R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F6R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F6R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F6R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F6R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F6R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F6R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F6R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F6R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F6R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F6R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F6R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F6R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F7R1 register  ****************** *)

  CAN_F7R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F7R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F7R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F7R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F7R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F7R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F7R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F7R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F7R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F7R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F7R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F7R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F7R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F7R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F7R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F7R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F7R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F7R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F7R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F7R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F7R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F7R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F7R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F7R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F7R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F7R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F7R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F7R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F7R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F7R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F7R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F7R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F8R1 register  ****************** *)

  CAN_F8R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F8R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F8R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F8R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F8R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F8R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F8R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F8R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F8R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F8R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F8R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F8R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F8R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F8R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F8R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F8R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F8R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F8R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F8R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F8R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F8R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F8R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F8R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F8R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F8R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F8R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F8R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F8R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F8R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F8R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F8R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F8R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F9R1 register  ****************** *)

  CAN_F9R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F9R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F9R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F9R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F9R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F9R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F9R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F9R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F9R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F9R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F9R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F9R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F9R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F9R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F9R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F9R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F9R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F9R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F9R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F9R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F9R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F9R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F9R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F9R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F9R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F9R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F9R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F9R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F9R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F9R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F9R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F9R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F10R1 register  ***************** *)

  CAN_F10R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F10R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F10R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F10R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F10R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F10R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F10R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F10R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F10R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F10R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F10R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F10R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F10R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F10R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F10R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F10R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F10R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F10R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F10R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F10R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F10R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F10R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F10R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F10R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F10R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F10R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F10R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F10R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F10R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F10R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F10R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F10R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F11R1 register  ***************** *)

  CAN_F11R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F11R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F11R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F11R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F11R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F11R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F11R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F11R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F11R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F11R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F11R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F11R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F11R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F11R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F11R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F11R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F11R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F11R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F11R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F11R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F11R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F11R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F11R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F11R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F11R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F11R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F11R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F11R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F11R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F11R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F11R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F11R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F12R1 register  ***************** *)

  CAN_F12R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F12R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F12R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F12R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F12R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F12R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F12R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F12R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F12R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F12R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F12R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F12R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F12R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F12R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F12R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F12R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F12R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F12R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F12R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F12R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F12R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F12R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F12R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F12R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F12R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F12R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F12R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F12R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F12R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F12R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F12R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F12R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F13R1 register  ***************** *)

  CAN_F13R1_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F13R1_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F13R1_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F13R1_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F13R1_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F13R1_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F13R1_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F13R1_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F13R1_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F13R1_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F13R1_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F13R1_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F13R1_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F13R1_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F13R1_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F13R1_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F13R1_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F13R1_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F13R1_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F13R1_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F13R1_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F13R1_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F13R1_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F13R1_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F13R1_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F13R1_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F13R1_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F13R1_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F13R1_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F13R1_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F13R1_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F13R1_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F0R2 register  ****************** *)

  CAN_F0R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F0R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F0R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F0R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F0R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F0R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F0R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F0R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F0R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F0R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F0R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F0R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F0R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F0R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F0R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F0R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F0R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F0R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F0R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F0R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F0R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F0R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F0R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F0R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F0R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F0R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F0R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F0R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F0R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F0R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F0R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F0R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F1R2 register  ****************** *)

  CAN_F1R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F1R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F1R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F1R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F1R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F1R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F1R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F1R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F1R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F1R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F1R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F1R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F1R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F1R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F1R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F1R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F1R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F1R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F1R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F1R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F1R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F1R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F1R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F1R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F1R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F1R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F1R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F1R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F1R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F1R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F1R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F1R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F2R2 register  ****************** *)

  CAN_F2R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F2R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F2R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F2R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F2R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F2R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F2R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F2R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F2R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F2R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F2R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F2R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F2R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F2R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F2R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F2R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F2R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F2R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F2R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F2R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F2R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F2R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F2R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F2R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F2R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F2R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F2R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F2R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F2R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F2R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F2R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F2R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F3R2 register  ****************** *)

  CAN_F3R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F3R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F3R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F3R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F3R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F3R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F3R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F3R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F3R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F3R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F3R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F3R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F3R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F3R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F3R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F3R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F3R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F3R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F3R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F3R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F3R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F3R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F3R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F3R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F3R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F3R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F3R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F3R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F3R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F3R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F3R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F3R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F4R2 register  ****************** *)

  CAN_F4R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F4R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F4R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F4R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F4R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F4R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F4R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F4R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F4R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F4R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F4R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F4R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F4R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F4R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F4R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F4R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F4R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F4R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F4R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F4R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F4R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F4R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F4R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F4R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F4R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F4R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F4R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F4R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F4R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F4R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F4R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F4R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F5R2 register  ****************** *)

  CAN_F5R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F5R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F5R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F5R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F5R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F5R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F5R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F5R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F5R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F5R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F5R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F5R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F5R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F5R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F5R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F5R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F5R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F5R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F5R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F5R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F5R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F5R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F5R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F5R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F5R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F5R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F5R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F5R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F5R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F5R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F5R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F5R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F6R2 register  ****************** *)

  CAN_F6R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F6R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F6R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F6R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F6R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F6R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F6R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F6R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F6R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F6R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F6R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F6R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F6R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F6R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F6R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F6R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F6R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F6R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F6R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F6R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F6R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F6R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F6R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F6R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F6R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F6R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F6R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F6R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F6R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F6R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F6R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F6R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F7R2 register  ****************** *)

  CAN_F7R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F7R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F7R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F7R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F7R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F7R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F7R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F7R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F7R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F7R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F7R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F7R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F7R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F7R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F7R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F7R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F7R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F7R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F7R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F7R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F7R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F7R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F7R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F7R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F7R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F7R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F7R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F7R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F7R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F7R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F7R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F7R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F8R2 register  ****************** *)

  CAN_F8R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F8R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F8R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F8R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F8R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F8R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F8R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F8R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F8R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F8R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F8R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F8R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F8R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F8R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F8R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F8R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F8R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F8R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F8R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F8R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F8R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F8R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F8R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F8R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F8R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F8R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F8R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F8R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F8R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F8R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F8R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F8R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F9R2 register  ****************** *)

  CAN_F9R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F9R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F9R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F9R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F9R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F9R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F9R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F9R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F9R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F9R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F9R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F9R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F9R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F9R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F9R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F9R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F9R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F9R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F9R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F9R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F9R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F9R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F9R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F9R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F9R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F9R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F9R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F9R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F9R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F9R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F9R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F9R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F10R2 register  ***************** *)

  CAN_F10R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F10R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F10R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F10R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F10R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F10R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F10R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F10R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F10R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F10R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F10R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F10R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F10R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F10R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F10R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F10R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F10R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F10R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F10R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F10R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F10R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F10R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F10R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F10R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F10R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F10R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F10R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F10R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F10R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F10R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F10R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F10R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F11R2 register  ***************** *)

  CAN_F11R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F11R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F11R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F11R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F11R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F11R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F11R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F11R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F11R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F11R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F11R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F11R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F11R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F11R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F11R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F11R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F11R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F11R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F11R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F11R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F11R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F11R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F11R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F11R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F11R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F11R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F11R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F11R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F11R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F11R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F11R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F11R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F12R2 register  ***************** *)

  CAN_F12R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F12R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F12R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F12R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F12R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F12R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F12R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F12R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F12R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F12R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F12R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F12R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F12R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F12R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F12R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F12R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F12R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F12R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F12R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F12R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F12R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F12R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F12R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F12R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F12R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F12R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F12R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F12R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F12R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F12R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F12R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F12R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (*******************  Bit definition for CAN_F13R2 register  ***************** *)

  CAN_F13R2_FB0 = $00000001;  (*!<Filter bit 0  *)
  CAN_F13R2_FB1 = $00000002;  (*!<Filter bit 1  *)
  CAN_F13R2_FB2 = $00000004;  (*!<Filter bit 2  *)
  CAN_F13R2_FB3 = $00000008;  (*!<Filter bit 3  *)
  CAN_F13R2_FB4 = $00000010;  (*!<Filter bit 4  *)
  CAN_F13R2_FB5 = $00000020;  (*!<Filter bit 5  *)
  CAN_F13R2_FB6 = $00000040;  (*!<Filter bit 6  *)
  CAN_F13R2_FB7 = $00000080;  (*!<Filter bit 7  *)
  CAN_F13R2_FB8 = $00000100;  (*!<Filter bit 8  *)
  CAN_F13R2_FB9 = $00000200;  (*!<Filter bit 9  *)
  CAN_F13R2_FB10 = $00000400;  (*!<Filter bit 10  *)
  CAN_F13R2_FB11 = $00000800;  (*!<Filter bit 11  *)
  CAN_F13R2_FB12 = $00001000;  (*!<Filter bit 12  *)
  CAN_F13R2_FB13 = $00002000;  (*!<Filter bit 13  *)
  CAN_F13R2_FB14 = $00004000;  (*!<Filter bit 14  *)
  CAN_F13R2_FB15 = $00008000;  (*!<Filter bit 15  *)
  CAN_F13R2_FB16 = $00010000;  (*!<Filter bit 16  *)
  CAN_F13R2_FB17 = $00020000;  (*!<Filter bit 17  *)
  CAN_F13R2_FB18 = $00040000;  (*!<Filter bit 18  *)
  CAN_F13R2_FB19 = $00080000;  (*!<Filter bit 19  *)
  CAN_F13R2_FB20 = $00100000;  (*!<Filter bit 20  *)
  CAN_F13R2_FB21 = $00200000;  (*!<Filter bit 21  *)
  CAN_F13R2_FB22 = $00400000;  (*!<Filter bit 22  *)
  CAN_F13R2_FB23 = $00800000;  (*!<Filter bit 23  *)
  CAN_F13R2_FB24 = $01000000;  (*!<Filter bit 24  *)
  CAN_F13R2_FB25 = $02000000;  (*!<Filter bit 25  *)
  CAN_F13R2_FB26 = $04000000;  (*!<Filter bit 26  *)
  CAN_F13R2_FB27 = $08000000;  (*!<Filter bit 27  *)
  CAN_F13R2_FB28 = $10000000;  (*!<Filter bit 28  *)
  CAN_F13R2_FB29 = $20000000;  (*!<Filter bit 29  *)
  CAN_F13R2_FB30 = $40000000;  (*!<Filter bit 30  *)
  CAN_F13R2_FB31 = $80000000;  (*!<Filter bit 31  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                                 HDMI-CEC (CEC)                              *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*******************  Bit definition for CEC_CR register  ******************** *)

  CEC_CR_CECEN = $00000001;  (*!< CEC Enable                          *)
  CEC_CR_TXSOM = $00000002;  (*!< CEC Tx Start Of Message             *)
  CEC_CR_TXEOM = $00000004;  (*!< CEC Tx End Of Message               *)
  (*******************  Bit definition for CEC_CFGR register  ****************** *)

  CEC_CFGR_SFT = $00000007;  (*!< CEC Signal Free Time                *)
  CEC_CFGR_RXTOL = $00000008;  (*!< CEC Tolerance                       *)
  CEC_CFGR_BRESTP = $00000010;  (*!< CEC Rx Stop                         *)
  CEC_CFGR_BREGEN = $00000020;  (*!< CEC Bit Rising Error generation     *)
  CEC_CFGR_LBPEGEN = $00000040;  (*!< CEC Long Period Error generation    *)
  CEC_CFGR_BRDNOGEN = $00000080;  (*!< CEC Broadcast no Error generation   *)
  CEC_CFGR_SFTOPT = $00000100;  (*!< CEC Signal Free Time optional       *)
  CEC_CFGR_OAR = $7FFF0000;  (*!< CEC Own Address                     *)
  CEC_CFGR_LSTN = $80000000;  (*!< CEC Listen mode                     *)
  (*******************  Bit definition for CEC_TXDR register  ****************** *)

  CEC_TXDR_TXD = $000000FF;  (*!< CEC Tx Data                         *)
  (*******************  Bit definition for CEC_RXDR register  ****************** *)

  CEC_TXDR_RXD = $000000FF;  (*!< CEC Rx Data                         *)
  (*******************  Bit definition for CEC_ISR register  ******************* *)

  CEC_ISR_RXBR = $00000001;  (*!< CEC Rx-Byte Received                    *)
  CEC_ISR_RXEND = $00000002;  (*!< CEC End Of Reception                    *)
  CEC_ISR_RXOVR = $00000004;  (*!< CEC Rx-Overrun                          *)
  CEC_ISR_BRE = $00000008;  (*!< CEC Rx Bit Rising Error                 *)
  CEC_ISR_SBPE = $00000010;  (*!< CEC Rx Short Bit period Error           *)
  CEC_ISR_LBPE = $00000020;  (*!< CEC Rx Long Bit period Error            *)
  CEC_ISR_RXACKE = $00000040;  (*!< CEC Rx Missing Acknowledge              *)
  CEC_ISR_ARBLST = $00000080;  (*!< CEC Arbitration Lost                    *)
  CEC_ISR_TXBR = $00000100;  (*!< CEC Tx Byte Request                     *)
  CEC_ISR_TXEND = $00000200;  (*!< CEC End of Transmission                 *)
  CEC_ISR_TXUDR = $00000400;  (*!< CEC Tx-Buffer Underrun                  *)
  CEC_ISR_TXERR = $00000800;  (*!< CEC Tx-Error                            *)
  CEC_ISR_TXACKE = $00001000;  (*!< CEC Tx Missing Acknowledge              *)
  (*******************  Bit definition for CEC_IER register  ******************* *)

  CEC_IER_RXBRIE = $00000001;  (*!< CEC Rx-Byte Received IT Enable          *)
  CEC_IER_RXENDIE = $00000002;  (*!< CEC End Of Reception IT Enable          *)
  CEC_IER_RXOVRIE = $00000004;  (*!< CEC Rx-Overrun IT Enable                *)
  CEC_IER_BREIE = $00000008;  (*!< CEC Rx Bit Rising Error IT Enable       *)
  CEC_IER_SBPEIE = $00000010;  (*!< CEC Rx Short Bit period Error IT Enable *)
  CEC_IER_LBPEIE = $00000020;  (*!< CEC Rx Long Bit period Error IT Enable  *)
  CEC_IER_RXACKEIE = $00000040;  (*!< CEC Rx Missing Acknowledge IT Enable    *)
  CEC_IER_ARBLSTIE = $00000080;  (*!< CEC Arbitration Lost IT Enable          *)
  CEC_IER_TXBRIE = $00000100;  (*!< CEC Tx Byte Request  IT Enable          *)
  CEC_IER_TXENDIE = $00000200;  (*!< CEC End of Transmission IT Enable       *)
  CEC_IER_TXUDRIE = $00000400;  (*!< CEC Tx-Buffer Underrun IT Enable        *)
  CEC_IER_TXERRIE = $00000800;  (*!< CEC Tx-Error IT Enable                  *)
  CEC_IER_TXACKEIE = $00001000;  (*!< CEC Tx Missing Acknowledge IT Enable    *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                          CRC calculation unit                               *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*******************  Bit definition for CRC_DR register  ******************** *)

  CRC_DR_DR = $FFFFFFFF;  (*!< Data register bits  *)
  (*******************  Bit definition for CRC_IDR register  ******************* *)

  CRC_IDR_IDR = $000000FF;  (*!< General-purpose 8-bit data register bits  *)
  (********************  Bit definition for CRC_CR register  ******************* *)

  CRC_CR_RESET = $00000001;  (*!< RESET the CRC computation unit bit  *)
  CRC_CR_POLYSIZE = $00000018;  (*!< Polynomial size bits                *)
  CRC_CR_POLYSIZE_0 = $00000008;  (*!< Polynomial size bit 0               *)
  CRC_CR_POLYSIZE_1 = $00000010;  (*!< Polynomial size bit 1               *)
  CRC_CR_REV_IN = $00000060;  (*!< REV_IN Reverse Input Data bits      *)
  CRC_CR_REV_IN_0 = $00000020;  (*!< Bit 0  *)
  CRC_CR_REV_IN_1 = $00000040;  (*!< Bit 1  *)
  CRC_CR_REV_OUT = $00000080;  (*!< REV_OUT Reverse Output Data bits    *)
  (*******************  Bit definition for CRC_INIT register  ****************** *)

  CRC_INIT_INIT = $FFFFFFFF;  (*!< Initial CRC value bits          *)
  (*******************  Bit definition for CRC_POL register  ******************* *)

  CRC_POL_POL = $FFFFFFFF;  (*!< Coefficients of the polynomial  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                      Digital to Analog Converter                            *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for DAC_CR register  ******************* *)

  DAC_CR_EN1 = $00000001;  (*!<DAC channel1 enable                          *)
  DAC_CR_BOFF1 = $00000002;  (*!<DAC channel1 output buffer disable           *)
  DAC_CR_TEN1 = $00000004;  (*!<DAC channel1 Trigger enable                  *)
  DAC_CR_TSEL1 = $00000038;  (*!<TSEL1[2:0] (DAC channel1 Trigger selection)  *)
  DAC_CR_TSEL1_0 = $00000008;  (*!<Bit 0  *)
  DAC_CR_TSEL1_1 = $00000010;  (*!<Bit 1  *)
  DAC_CR_TSEL1_2 = $00000020;  (*!<Bit 2  *)
  DAC_CR_WAVE1 = $000000C0;  (*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable)  *)
  DAC_CR_WAVE1_0 = $00000040;  (*!<Bit 0  *)
  DAC_CR_WAVE1_1 = $00000080;  (*!<Bit 1  *)
  DAC_CR_MAMP1 = $00000F00;  (*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector)  *)
  DAC_CR_MAMP1_0 = $00000100;  (*!<Bit 0  *)
  DAC_CR_MAMP1_1 = $00000200;  (*!<Bit 1  *)
  DAC_CR_MAMP1_2 = $00000400;  (*!<Bit 2  *)
  DAC_CR_MAMP1_3 = $00000800;  (*!<Bit 3  *)
  DAC_CR_DMAEN1 = $00001000;  (*!<DAC channel1 DMA enable                      *)
  DAC_CR_EN2 = $00010000;  (*!<DAC channel2 enable                          *)
  DAC_CR_BOFF2 = $00020000;  (*!<DAC channel2 output buffer disable           *)
  DAC_CR_TEN2 = $00040000;  (*!<DAC channel2 Trigger enable                  *)
  DAC_CR_TSEL2 = $00380000;  (*!<TSEL2[2:0] (DAC channel2 Trigger selection)  *)
  DAC_CR_TSEL2_0 = $00080000;  (*!<Bit 0  *)
  DAC_CR_TSEL2_1 = $00100000;  (*!<Bit 1  *)
  DAC_CR_TSEL2_2 = $00200000;  (*!<Bit 2  *)
  DAC_CR_WAVE2 = $00C00000;  (*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable)  *)
  DAC_CR_WAVE2_0 = $00400000;  (*!<Bit 0  *)
  DAC_CR_WAVE2_1 = $00800000;  (*!<Bit 1  *)
  DAC_CR_MAMP2 = $0F000000;  (*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector)  *)
  DAC_CR_MAMP2_0 = $01000000;  (*!<Bit 0  *)
  DAC_CR_MAMP2_1 = $02000000;  (*!<Bit 1  *)
  DAC_CR_MAMP2_2 = $04000000;  (*!<Bit 2  *)
  DAC_CR_MAMP2_3 = $08000000;  (*!<Bit 3  *)
  DAC_CR_DMAEN2 = $10000000;  (*!<DAC channel2 DMA enabled  *)
  (*****************  Bit definition for DAC_SWTRIGR register  ***************** *)

  DAC_SWTRIGR_SWTRIG1 = $01;  (*!<DAC channel1 software trigger  *)
  DAC_SWTRIGR_SWTRIG2 = $02;  (*!<DAC channel2 software trigger  *)
  (*****************  Bit definition for DAC_DHR12R1 register  ***************** *)

  DAC_DHR12R1_DACC1DHR = $0FFF;  (*!<DAC channel1 12-bit Right aligned data  *)
  (*****************  Bit definition for DAC_DHR12L1 register  ***************** *)

  DAC_DHR12L1_DACC1DHR = $FFF0;  (*!<DAC channel1 12-bit Left aligned data  *)
  (******************  Bit definition for DAC_DHR8R1 register  ***************** *)

  DAC_DHR8R1_DACC1DHR = $FF;  (*!<DAC channel1 8-bit Right aligned data  *)
  (*****************  Bit definition for DAC_DHR12R2 register  ***************** *)

  DAC_DHR12R2_DACC2DHR = $0FFF;  (*!<DAC channel2 12-bit Right aligned data  *)
  (*****************  Bit definition for DAC_DHR12L2 register  ***************** *)

  DAC_DHR12L2_DACC2DHR = $FFF0;  (*!<DAC channel2 12-bit Left aligned data  *)
  (******************  Bit definition for DAC_DHR8R2 register  ***************** *)

  DAC_DHR8R2_DACC2DHR = $FF;  (*!<DAC channel2 8-bit Right aligned data  *)
  (*****************  Bit definition for DAC_DHR12RD register  ***************** *)

  DAC_DHR12RD_DACC1DHR = $00000FFF;  (*!<DAC channel1 12-bit Right aligned data  *)
  DAC_DHR12RD_DACC2DHR = $0FFF0000;  (*!<DAC channel2 12-bit Right aligned data  *)
  (*****************  Bit definition for DAC_DHR12LD register  ***************** *)

  DAC_DHR12LD_DACC1DHR = $0000FFF0;  (*!<DAC channel1 12-bit Left aligned data  *)
  DAC_DHR12LD_DACC2DHR = $FFF00000;  (*!<DAC channel2 12-bit Left aligned data  *)
  (******************  Bit definition for DAC_DHR8RD register  ***************** *)

  DAC_DHR8RD_DACC1DHR = $00FF;  (*!<DAC channel1 8-bit Right aligned data  *)
  DAC_DHR8RD_DACC2DHR = $FF00;  (*!<DAC channel2 8-bit Right aligned data  *)
  (*******************  Bit definition for DAC_DOR1 register  ****************** *)

  DAC_DOR1_DACC1DOR = $0FFF;  (*!<DAC channel1 data output  *)
  (*******************  Bit definition for DAC_DOR2 register  ****************** *)

  DAC_DOR2_DACC2DOR = $0FFF;  (*!<DAC channel2 data output  *)
  (********************  Bit definition for DAC_SR register  ******************* *)

  DAC_SR_DMAUDR1 = $00002000;  (*!<DAC channel1 DMA underrun flag  *)
  DAC_SR_DMAUDR2 = $20000000;  (*!<DAC channel2 DMA underrun flag  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                                 Debug MCU                                   *)

  (*                                                                             *)

  (***************************************************************************** *)

  (***************************************************************************** *)

  (*                                                                             *)

  (*                                    DCMI                                     *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bits definition for DCMI_CR register  ***************** *)

  DCMI_CR_CAPTURE = $00000001;
  DCMI_CR_CM = $00000002;
  DCMI_CR_CROP = $00000004;
  DCMI_CR_JPEG = $00000008;
  DCMI_CR_ESS = $00000010;
  DCMI_CR_PCKPOL = $00000020;
  DCMI_CR_HSPOL = $00000040;
  DCMI_CR_VSPOL = $00000080;
  DCMI_CR_FCRC_0 = $00000100;
  DCMI_CR_FCRC_1 = $00000200;
  DCMI_CR_EDM_0 = $00000400;
  DCMI_CR_EDM_1 = $00000800;
  DCMI_CR_CRE = $00001000;
  DCMI_CR_ENABLE = $00004000;
  DCMI_CR_BSM = $00030000;
  DCMI_CR_BSM_0 = $00010000;
  DCMI_CR_BSM_1 = $00020000;
  DCMI_CR_OEBS = $00040000;
  DCMI_CR_LSM = $00080000;
  DCMI_CR_OELS = $00100000;
  (********************  Bits definition for DCMI_SR register  ***************** *)

  DCMI_SR_HSYNC = $00000001;
  DCMI_SR_VSYNC = $00000002;
  DCMI_SR_FNE = $00000004;
  (********************  Bits definition for DCMI_RISR register  *************** *)

  DCMI_RISR_FRAME_RIS = $00000001;
  DCMI_RISR_OVF_RIS = $00000002;
  DCMI_RISR_ERR_RIS = $00000004;
  DCMI_RISR_VSYNC_RIS = $00000008;
  DCMI_RISR_LINE_RIS = $00000010;
  (********************  Bits definition for DCMI_IER register  **************** *)

  DCMI_IER_FRAME_IE = $00000001;
  DCMI_IER_OVF_IE = $00000002;
  DCMI_IER_ERR_IE = $00000004;
  DCMI_IER_VSYNC_IE = $00000008;
  DCMI_IER_LINE_IE = $00000010;
  (********************  Bits definition for DCMI_MISR register  *************** *)

  DCMI_MISR_FRAME_MIS = $00000001;
  DCMI_MISR_OVF_MIS = $00000002;
  DCMI_MISR_ERR_MIS = $00000004;
  DCMI_MISR_VSYNC_MIS = $00000008;
  DCMI_MISR_LINE_MIS = $00000010;
  (********************  Bits definition for DCMI_ICR register  **************** *)

  DCMI_ICR_FRAME_ISC = $00000001;
  DCMI_ICR_OVF_ISC = $00000002;
  DCMI_ICR_ERR_ISC = $00000004;
  DCMI_ICR_VSYNC_ISC = $00000008;
  DCMI_ICR_LINE_ISC = $00000010;

(***************************************************************************** *)

(*                                                                             *)

(*                             DMA Controller                                  *)

(*                                                                             *)

(***************************************************************************** *)

(********************  Bits definition for DMA_SxCR register  **************** *)

const
  DMA_SxCR_CHSEL = $0E000000;
  DMA_SxCR_CHSEL_0 = $02000000;
  DMA_SxCR_CHSEL_1 = $04000000;
  DMA_SxCR_CHSEL_2 = $08000000;
  DMA_SxCR_MBURST = $01800000;
  DMA_SxCR_MBURST_0 = $00800000;
  DMA_SxCR_MBURST_1 = $01000000;
  DMA_SxCR_PBURST = $00600000;
  DMA_SxCR_PBURST_0 = $00200000;
  DMA_SxCR_PBURST_1 = $00400000;
  DMA_SxCR_ACK = $00100000;
  DMA_SxCR_CT = $00080000;
  DMA_SxCR_DBM = $00040000;
  DMA_SxCR_PL = $00030000;
  DMA_SxCR_PL_0 = $00010000;
  DMA_SxCR_PL_1 = $00020000;
  DMA_SxCR_PINCOS = $00008000;
  DMA_SxCR_MSIZE = $00006000;
  DMA_SxCR_MSIZE_0 = $00002000;
  DMA_SxCR_MSIZE_1 = $00004000;
  DMA_SxCR_PSIZE = $00001800;
  DMA_SxCR_PSIZE_0 = $00000800;
  DMA_SxCR_PSIZE_1 = $00001000;
  DMA_SxCR_MINC = $00000400;
  DMA_SxCR_PINC = $00000200;
  DMA_SxCR_CIRC = $00000100;
  DMA_SxCR_DIR = $000000C0;
  DMA_SxCR_DIR_0 = $00000040;
  DMA_SxCR_DIR_1 = $00000080;
  DMA_SxCR_PFCTRL = $00000020;
  DMA_SxCR_TCIE = $00000010;
  DMA_SxCR_HTIE = $00000008;
  DMA_SxCR_TEIE = $00000004;
  DMA_SxCR_DMEIE = $00000002;
  DMA_SxCR_EN = $00000001;
  (********************  Bits definition for DMA_SxCNDTR register  ************* *)

  DMA_SxNDT = $0000FFFF;
  DMA_SxNDT_0 = $00000001;
  DMA_SxNDT_1 = $00000002;
  DMA_SxNDT_2 = $00000004;
  DMA_SxNDT_3 = $00000008;
  DMA_SxNDT_4 = $00000010;
  DMA_SxNDT_5 = $00000020;
  DMA_SxNDT_6 = $00000040;
  DMA_SxNDT_7 = $00000080;
  DMA_SxNDT_8 = $00000100;
  DMA_SxNDT_9 = $00000200;
  DMA_SxNDT_10 = $00000400;
  DMA_SxNDT_11 = $00000800;
  DMA_SxNDT_12 = $00001000;
  DMA_SxNDT_13 = $00002000;
  DMA_SxNDT_14 = $00004000;
  DMA_SxNDT_15 = $00008000;
  (********************  Bits definition for DMA_SxFCR register  *************** *)

  DMA_SxFCR_FEIE = $00000080;
  DMA_SxFCR_FS = $00000038;
  DMA_SxFCR_FS_0 = $00000008;
  DMA_SxFCR_FS_1 = $00000010;
  DMA_SxFCR_FS_2 = $00000020;
  DMA_SxFCR_DMDIS = $00000004;
  DMA_SxFCR_FTH = $00000003;
  DMA_SxFCR_FTH_0 = $00000001;
  DMA_SxFCR_FTH_1 = $00000002;
  (********************  Bits definition for DMA_LISR register  **************** *)

  DMA_LISR_TCIF3 = $08000000;
  DMA_LISR_HTIF3 = $04000000;
  DMA_LISR_TEIF3 = $02000000;
  DMA_LISR_DMEIF3 = $01000000;
  DMA_LISR_FEIF3 = $00400000;
  DMA_LISR_TCIF2 = $00200000;
  DMA_LISR_HTIF2 = $00100000;
  DMA_LISR_TEIF2 = $00080000;
  DMA_LISR_DMEIF2 = $00040000;
  DMA_LISR_FEIF2 = $00010000;
  DMA_LISR_TCIF1 = $00000800;
  DMA_LISR_HTIF1 = $00000400;
  DMA_LISR_TEIF1 = $00000200;
  DMA_LISR_DMEIF1 = $00000100;
  DMA_LISR_FEIF1 = $00000040;
  DMA_LISR_TCIF0 = $00000020;
  DMA_LISR_HTIF0 = $00000010;
  DMA_LISR_TEIF0 = $00000008;
  DMA_LISR_DMEIF0 = $00000004;
  DMA_LISR_FEIF0 = $00000001;
  (********************  Bits definition for DMA_HISR register  **************** *)

  DMA_HISR_TCIF7 = $08000000;
  DMA_HISR_HTIF7 = $04000000;
  DMA_HISR_TEIF7 = $02000000;
  DMA_HISR_DMEIF7 = $01000000;
  DMA_HISR_FEIF7 = $00400000;
  DMA_HISR_TCIF6 = $00200000;
  DMA_HISR_HTIF6 = $00100000;
  DMA_HISR_TEIF6 = $00080000;
  DMA_HISR_DMEIF6 = $00040000;
  DMA_HISR_FEIF6 = $00010000;
  DMA_HISR_TCIF5 = $00000800;
  DMA_HISR_HTIF5 = $00000400;
  DMA_HISR_TEIF5 = $00000200;
  DMA_HISR_DMEIF5 = $00000100;
  DMA_HISR_FEIF5 = $00000040;
  DMA_HISR_TCIF4 = $00000020;
  DMA_HISR_HTIF4 = $00000010;
  DMA_HISR_TEIF4 = $00000008;
  DMA_HISR_DMEIF4 = $00000004;
  DMA_HISR_FEIF4 = $00000001;
  (********************  Bits definition for DMA_LIFCR register  *************** *)

  DMA_LIFCR_CTCIF3 = $08000000;
  DMA_LIFCR_CHTIF3 = $04000000;
  DMA_LIFCR_CTEIF3 = $02000000;
  DMA_LIFCR_CDMEIF3 = $01000000;
  DMA_LIFCR_CFEIF3 = $00400000;
  DMA_LIFCR_CTCIF2 = $00200000;
  DMA_LIFCR_CHTIF2 = $00100000;
  DMA_LIFCR_CTEIF2 = $00080000;
  DMA_LIFCR_CDMEIF2 = $00040000;
  DMA_LIFCR_CFEIF2 = $00010000;
  DMA_LIFCR_CTCIF1 = $00000800;
  DMA_LIFCR_CHTIF1 = $00000400;
  DMA_LIFCR_CTEIF1 = $00000200;
  DMA_LIFCR_CDMEIF1 = $00000100;
  DMA_LIFCR_CFEIF1 = $00000040;
  DMA_LIFCR_CTCIF0 = $00000020;
  DMA_LIFCR_CHTIF0 = $00000010;
  DMA_LIFCR_CTEIF0 = $00000008;
  DMA_LIFCR_CDMEIF0 = $00000004;
  DMA_LIFCR_CFEIF0 = $00000001;
  (********************  Bits definition for DMA_HIFCR  register  *************** *)

  DMA_HIFCR_CTCIF7 = $08000000;
  DMA_HIFCR_CHTIF7 = $04000000;
  DMA_HIFCR_CTEIF7 = $02000000;
  DMA_HIFCR_CDMEIF7 = $01000000;
  DMA_HIFCR_CFEIF7 = $00400000;
  DMA_HIFCR_CTCIF6 = $00200000;
  DMA_HIFCR_CHTIF6 = $00100000;
  DMA_HIFCR_CTEIF6 = $00080000;
  DMA_HIFCR_CDMEIF6 = $00040000;
  DMA_HIFCR_CFEIF6 = $00010000;
  DMA_HIFCR_CTCIF5 = $00000800;
  DMA_HIFCR_CHTIF5 = $00000400;
  DMA_HIFCR_CTEIF5 = $00000200;
  DMA_HIFCR_CDMEIF5 = $00000100;
  DMA_HIFCR_CFEIF5 = $00000040;
  DMA_HIFCR_CTCIF4 = $00000020;
  DMA_HIFCR_CHTIF4 = $00000010;
  DMA_HIFCR_CTEIF4 = $00000008;
  DMA_HIFCR_CDMEIF4 = $00000004;
  DMA_HIFCR_CFEIF4 = $00000001;
  (***************************************************************************** *)

  (*                                                                             *)

  (*                         AHB Master DMA2D Controller (DMA2D)                 *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for DMA2D_CR register  ***************** *)

  DMA2D_CR_START = $00000001;  (*!< Start transfer                           *)
  DMA2D_CR_SUSP = $00000002;  (*!< Suspend transfer                         *)
  DMA2D_CR_ABORT = $00000004;  (*!< Abort transfer                           *)
  DMA2D_CR_TEIE = $00000100;  (*!< Transfer Error Interrupt Enable          *)
  DMA2D_CR_TCIE = $00000200;  (*!< Transfer Complete Interrupt Enable       *)
  DMA2D_CR_TWIE = $00000400;  (*!< Transfer Watermark Interrupt Enable      *)
  DMA2D_CR_CAEIE = $00000800;  (*!< CLUT Access Error Interrupt Enable       *)
  DMA2D_CR_CTCIE = $00001000;  (*!< CLUT Transfer Complete Interrupt Enable  *)
  DMA2D_CR_CEIE = $00002000;  (*!< Configuration Error Interrupt Enable     *)
  DMA2D_CR_MODE = $00030000;  (*!< DMA2D Mode                               *)
  (********************  Bit definition for DMA2D_ISR register  **************** *)

  DMA2D_ISR_TEIF = $00000001;  (*!< Transfer Error Interrupt Flag          *)
  DMA2D_ISR_TCIF = $00000002;  (*!< Transfer Complete Interrupt Flag       *)
  DMA2D_ISR_TWIF = $00000004;  (*!< Transfer Watermark Interrupt Flag      *)
  DMA2D_ISR_CAEIF = $00000008;  (*!< CLUT Access Error Interrupt Flag       *)
  DMA2D_ISR_CTCIF = $00000010;  (*!< CLUT Transfer Complete Interrupt Flag  *)
  DMA2D_ISR_CEIF = $00000020;  (*!< Configuration Error Interrupt Flag     *)
  (********************  Bit definition for DMA2D_IFSR register  *************** *)

  DMA2D_IFSR_CTEIF = $00000001;  (*!< Clears Transfer Error Interrupt Flag          *)
  DMA2D_IFSR_CTCIF = $00000002;  (*!< Clears Transfer Complete Interrupt Flag       *)
  DMA2D_IFSR_CTWIF = $00000004;  (*!< Clears Transfer Watermark Interrupt Flag      *)
  DMA2D_IFSR_CCAEIF = $00000008;  (*!< Clears CLUT Access Error Interrupt Flag       *)
  DMA2D_IFSR_CCTCIF = $00000010;  (*!< Clears CLUT Transfer Complete Interrupt Flag  *)
  DMA2D_IFSR_CCEIF = $00000020;  (*!< Clears Configuration Error Interrupt Flag     *)
  (********************  Bit definition for DMA2D_FGMAR register  ************** *)

  DMA2D_FGMAR_MA = $FFFFFFFF;  (*!< Memory Address  *)
  (********************  Bit definition for DMA2D_FGOR register  *************** *)

  DMA2D_FGOR_LO = $00003FFF;  (*!< Line Offset  *)
  (********************  Bit definition for DMA2D_BGMAR register  ************** *)

  DMA2D_BGMAR_MA = $FFFFFFFF;  (*!< Memory Address  *)
  (********************  Bit definition for DMA2D_BGOR register  *************** *)

  DMA2D_BGOR_LO = $00003FFF;  (*!< Line Offset  *)
  (********************  Bit definition for DMA2D_FGPFCCR register  ************ *)

  DMA2D_FGPFCCR_CM = $0000000F;  (*!< Color mode       *)
  DMA2D_FGPFCCR_CCM = $00000010;  (*!< CLUT Color mode  *)
  DMA2D_FGPFCCR_START = $00000020;  (*!< Start            *)
  DMA2D_FGPFCCR_CS = $0000FF00;  (*!< CLUT size        *)
  DMA2D_FGPFCCR_AM = $00030000;  (*!< Alpha mode       *)
  DMA2D_FGPFCCR_ALPHA = $FF000000;  (*!< Alpha value      *)
  (********************  Bit definition for DMA2D_FGCOLR register  ************* *)

  DMA2D_FGCOLR_BLUE = $000000FF;  (*!< Blue Value   *)
  DMA2D_FGCOLR_GREEN = $0000FF00;  (*!< Green Value  *)
  DMA2D_FGCOLR_RED = $00FF0000;  (*!< Red Value    *)
  (********************  Bit definition for DMA2D_BGPFCCR register  ************ *)

  DMA2D_BGPFCCR_CM = $0000000F;  (*!< Color mode       *)
  DMA2D_BGPFCCR_CCM = $00000010;  (*!< CLUT Color mode  *)
  DMA2D_BGPFCCR_START = $00000020;  (*!< Start            *)
  DMA2D_BGPFCCR_CS = $0000FF00;  (*!< CLUT size        *)
  DMA2D_BGPFCCR_AM = $00030000;  (*!< Alpha Mode       *)
  DMA2D_BGPFCCR_ALPHA = $FF000000;  (*!< Alpha value      *)
  (********************  Bit definition for DMA2D_BGCOLR register  ************* *)

  DMA2D_BGCOLR_BLUE = $000000FF;  (*!< Blue Value   *)
  DMA2D_BGCOLR_GREEN = $0000FF00;  (*!< Green Value  *)
  DMA2D_BGCOLR_RED = $00FF0000;  (*!< Red Value    *)
  (********************  Bit definition for DMA2D_FGCMAR register  ************* *)

  DMA2D_FGCMAR_MA = $FFFFFFFF;  (*!< Memory Address  *)
  (********************  Bit definition for DMA2D_BGCMAR register  ************* *)

  DMA2D_BGCMAR_MA = $FFFFFFFF;  (*!< Memory Address  *)
  (********************  Bit definition for DMA2D_OPFCCR register  ************* *)

  DMA2D_OPFCCR_CM = $00000007;  (*!< Color mode  *)
  (********************  Bit definition for DMA2D_OCOLR register  ************** *)

  (*!<Mode_ARGB8888/RGB888  *)

  DMA2D_OCOLR_BLUE_1 = $000000FF;  (*!< BLUE Value           *)
  DMA2D_OCOLR_GREEN_1 = $0000FF00;  (*!< GREEN Value          *)
  DMA2D_OCOLR_RED_1 = $00FF0000;  (*!< Red Value            *)
  DMA2D_OCOLR_ALPHA_1 = $FF000000;  (*!< Alpha Channel Value  *)
  (*!<Mode_RGB565  *)

  DMA2D_OCOLR_BLUE_2 = $0000001F;  (*!< BLUE Value    *)
  DMA2D_OCOLR_GREEN_2 = $000007E0;  (*!< GREEN Value   *)
  DMA2D_OCOLR_RED_2 = $0000F800;  (*!< Red Value     *)
  (*!<Mode_ARGB1555  *)

  DMA2D_OCOLR_BLUE_3 = $0000001F;  (*!< BLUE Value           *)
  DMA2D_OCOLR_GREEN_3 = $000003E0;  (*!< GREEN Value          *)
  DMA2D_OCOLR_RED_3 = $00007C00;  (*!< Red Value            *)
  DMA2D_OCOLR_ALPHA_3 = $00008000;  (*!< Alpha Channel Value  *)
  (*!<Mode_ARGB4444  *)

  DMA2D_OCOLR_BLUE_4 = $0000000F;  (*!< BLUE Value           *)
  DMA2D_OCOLR_GREEN_4 = $000000F0;  (*!< GREEN Value          *)
  DMA2D_OCOLR_RED_4 = $00000F00;  (*!< Red Value            *)
  DMA2D_OCOLR_ALPHA_4 = $0000F000;  (*!< Alpha Channel Value  *)
  (********************  Bit definition for DMA2D_OMAR register  *************** *)

  DMA2D_OMAR_MA = $FFFFFFFF;  (*!< Memory Address  *)
  (********************  Bit definition for DMA2D_OOR register  **************** *)

  DMA2D_OOR_LO = $00003FFF;  (*!< Line Offset  *)
  (********************  Bit definition for DMA2D_NLR register  **************** *)

  DMA2D_NLR_NL = $0000FFFF;  (*!< Number of Lines  *)
  DMA2D_NLR_PL = $3FFF0000;  (*!< Pixel per Lines  *)
  (********************  Bit definition for DMA2D_LWR register  **************** *)

  DMA2D_LWR_LW = $0000FFFF;  (*!< Line Watermark  *)
  (********************  Bit definition for DMA2D_AMTCR register  ************** *)

  DMA2D_AMTCR_EN = $00000001;  (*!< Enable     *)
  DMA2D_AMTCR_DT = $0000FF00;  (*!< Dead Time  *)
  (********************  Bit definition for DMA2D_FGCLUT register  ************* *)

  (********************  Bit definition for DMA2D_BGCLUT register  ************* *)


  (***************************************************************************** *)

  (*                                                                             *)

  (*                    External Interrupt/Event Controller                      *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*******************  Bit definition for EXTI_IMR register  ****************** *)

  EXTI_IMR_MR0 = $00000001;  (*!< Interrupt Mask on line 0  *)
  EXTI_IMR_MR1 = $00000002;  (*!< Interrupt Mask on line 1  *)
  EXTI_IMR_MR2 = $00000004;  (*!< Interrupt Mask on line 2  *)
  EXTI_IMR_MR3 = $00000008;  (*!< Interrupt Mask on line 3  *)
  EXTI_IMR_MR4 = $00000010;  (*!< Interrupt Mask on line 4  *)
  EXTI_IMR_MR5 = $00000020;  (*!< Interrupt Mask on line 5  *)
  EXTI_IMR_MR6 = $00000040;  (*!< Interrupt Mask on line 6  *)
  EXTI_IMR_MR7 = $00000080;  (*!< Interrupt Mask on line 7  *)
  EXTI_IMR_MR8 = $00000100;  (*!< Interrupt Mask on line 8  *)
  EXTI_IMR_MR9 = $00000200;  (*!< Interrupt Mask on line 9  *)
  EXTI_IMR_MR10 = $00000400;  (*!< Interrupt Mask on line 10  *)
  EXTI_IMR_MR11 = $00000800;  (*!< Interrupt Mask on line 11  *)
  EXTI_IMR_MR12 = $00001000;  (*!< Interrupt Mask on line 12  *)
  EXTI_IMR_MR13 = $00002000;  (*!< Interrupt Mask on line 13  *)
  EXTI_IMR_MR14 = $00004000;  (*!< Interrupt Mask on line 14  *)
  EXTI_IMR_MR15 = $00008000;  (*!< Interrupt Mask on line 15  *)
  EXTI_IMR_MR16 = $00010000;  (*!< Interrupt Mask on line 16  *)
  EXTI_IMR_MR17 = $00020000;  (*!< Interrupt Mask on line 17  *)
  EXTI_IMR_MR18 = $00040000;  (*!< Interrupt Mask on line 18  *)
  EXTI_IMR_MR19 = $00080000;  (*!< Interrupt Mask on line 19  *)
  EXTI_IMR_MR20 = $00100000;  (*!< Interrupt Mask on line 20  *)
  EXTI_IMR_MR21 = $00200000;  (*!< Interrupt Mask on line 21  *)
  EXTI_IMR_MR22 = $00400000;  (*!< Interrupt Mask on line 22  *)
  EXTI_IMR_MR23 = $00800000;  (*!< Interrupt Mask on line 23  *)
  (*******************  Bit definition for EXTI_EMR register  ****************** *)

  EXTI_EMR_MR0 = $00000001;  (*!< Event Mask on line 0  *)
  EXTI_EMR_MR1 = $00000002;  (*!< Event Mask on line 1  *)
  EXTI_EMR_MR2 = $00000004;  (*!< Event Mask on line 2  *)
  EXTI_EMR_MR3 = $00000008;  (*!< Event Mask on line 3  *)
  EXTI_EMR_MR4 = $00000010;  (*!< Event Mask on line 4  *)
  EXTI_EMR_MR5 = $00000020;  (*!< Event Mask on line 5  *)
  EXTI_EMR_MR6 = $00000040;  (*!< Event Mask on line 6  *)
  EXTI_EMR_MR7 = $00000080;  (*!< Event Mask on line 7  *)
  EXTI_EMR_MR8 = $00000100;  (*!< Event Mask on line 8  *)
  EXTI_EMR_MR9 = $00000200;  (*!< Event Mask on line 9  *)
  EXTI_EMR_MR10 = $00000400;  (*!< Event Mask on line 10  *)
  EXTI_EMR_MR11 = $00000800;  (*!< Event Mask on line 11  *)
  EXTI_EMR_MR12 = $00001000;  (*!< Event Mask on line 12  *)
  EXTI_EMR_MR13 = $00002000;  (*!< Event Mask on line 13  *)
  EXTI_EMR_MR14 = $00004000;  (*!< Event Mask on line 14  *)
  EXTI_EMR_MR15 = $00008000;  (*!< Event Mask on line 15  *)
  EXTI_EMR_MR16 = $00010000;  (*!< Event Mask on line 16  *)
  EXTI_EMR_MR17 = $00020000;  (*!< Event Mask on line 17  *)
  EXTI_EMR_MR18 = $00040000;  (*!< Event Mask on line 18  *)
  EXTI_EMR_MR19 = $00080000;  (*!< Event Mask on line 19  *)
  EXTI_EMR_MR20 = $00100000;  (*!< Event Mask on line 20  *)
  EXTI_EMR_MR21 = $00200000;  (*!< Event Mask on line 21  *)
  EXTI_EMR_MR22 = $00400000;  (*!< Event Mask on line 22  *)
  EXTI_EMR_MR23 = $00800000;  (*!< Event Mask on line 23  *)
  (******************  Bit definition for EXTI_RTSR register  ****************** *)

  EXTI_RTSR_TR0 = $00000001;  (*!< Rising trigger event configuration bit of line 0  *)
  EXTI_RTSR_TR1 = $00000002;  (*!< Rising trigger event configuration bit of line 1  *)
  EXTI_RTSR_TR2 = $00000004;  (*!< Rising trigger event configuration bit of line 2  *)
  EXTI_RTSR_TR3 = $00000008;  (*!< Rising trigger event configuration bit of line 3  *)
  EXTI_RTSR_TR4 = $00000010;  (*!< Rising trigger event configuration bit of line 4  *)
  EXTI_RTSR_TR5 = $00000020;  (*!< Rising trigger event configuration bit of line 5  *)
  EXTI_RTSR_TR6 = $00000040;  (*!< Rising trigger event configuration bit of line 6  *)
  EXTI_RTSR_TR7 = $00000080;  (*!< Rising trigger event configuration bit of line 7  *)
  EXTI_RTSR_TR8 = $00000100;  (*!< Rising trigger event configuration bit of line 8  *)
  EXTI_RTSR_TR9 = $00000200;  (*!< Rising trigger event configuration bit of line 9  *)
  EXTI_RTSR_TR10 = $00000400;  (*!< Rising trigger event configuration bit of line 10  *)
  EXTI_RTSR_TR11 = $00000800;  (*!< Rising trigger event configuration bit of line 11  *)
  EXTI_RTSR_TR12 = $00001000;  (*!< Rising trigger event configuration bit of line 12  *)
  EXTI_RTSR_TR13 = $00002000;  (*!< Rising trigger event configuration bit of line 13  *)
  EXTI_RTSR_TR14 = $00004000;  (*!< Rising trigger event configuration bit of line 14  *)
  EXTI_RTSR_TR15 = $00008000;  (*!< Rising trigger event configuration bit of line 15  *)
  EXTI_RTSR_TR16 = $00010000;  (*!< Rising trigger event configuration bit of line 16  *)
  EXTI_RTSR_TR17 = $00020000;  (*!< Rising trigger event configuration bit of line 17  *)
  EXTI_RTSR_TR18 = $00040000;  (*!< Rising trigger event configuration bit of line 18  *)
  EXTI_RTSR_TR19 = $00080000;  (*!< Rising trigger event configuration bit of line 19  *)
  EXTI_RTSR_TR20 = $00100000;  (*!< Rising trigger event configuration bit of line 20  *)
  EXTI_RTSR_TR21 = $00200000;  (*!< Rising trigger event configuration bit of line 21  *)
  EXTI_RTSR_TR22 = $00400000;  (*!< Rising trigger event configuration bit of line 22  *)
  EXTI_RTSR_TR23 = $00800000;  (*!< Rising trigger event configuration bit of line 23  *)
  (******************  Bit definition for EXTI_FTSR register  ****************** *)

  EXTI_FTSR_TR0 = $00000001;  (*!< Falling trigger event configuration bit of line 0  *)
  EXTI_FTSR_TR1 = $00000002;  (*!< Falling trigger event configuration bit of line 1  *)
  EXTI_FTSR_TR2 = $00000004;  (*!< Falling trigger event configuration bit of line 2  *)
  EXTI_FTSR_TR3 = $00000008;  (*!< Falling trigger event configuration bit of line 3  *)
  EXTI_FTSR_TR4 = $00000010;  (*!< Falling trigger event configuration bit of line 4  *)
  EXTI_FTSR_TR5 = $00000020;  (*!< Falling trigger event configuration bit of line 5  *)
  EXTI_FTSR_TR6 = $00000040;  (*!< Falling trigger event configuration bit of line 6  *)
  EXTI_FTSR_TR7 = $00000080;  (*!< Falling trigger event configuration bit of line 7  *)
  EXTI_FTSR_TR8 = $00000100;  (*!< Falling trigger event configuration bit of line 8  *)
  EXTI_FTSR_TR9 = $00000200;  (*!< Falling trigger event configuration bit of line 9  *)
  EXTI_FTSR_TR10 = $00000400;  (*!< Falling trigger event configuration bit of line 10  *)
  EXTI_FTSR_TR11 = $00000800;  (*!< Falling trigger event configuration bit of line 11  *)
  EXTI_FTSR_TR12 = $00001000;  (*!< Falling trigger event configuration bit of line 12  *)
  EXTI_FTSR_TR13 = $00002000;  (*!< Falling trigger event configuration bit of line 13  *)
  EXTI_FTSR_TR14 = $00004000;  (*!< Falling trigger event configuration bit of line 14  *)
  EXTI_FTSR_TR15 = $00008000;  (*!< Falling trigger event configuration bit of line 15  *)
  EXTI_FTSR_TR16 = $00010000;  (*!< Falling trigger event configuration bit of line 16  *)
  EXTI_FTSR_TR17 = $00020000;  (*!< Falling trigger event configuration bit of line 17  *)
  EXTI_FTSR_TR18 = $00040000;  (*!< Falling trigger event configuration bit of line 18  *)
  EXTI_FTSR_TR19 = $00080000;  (*!< Falling trigger event configuration bit of line 19  *)
  EXTI_FTSR_TR20 = $00100000;  (*!< Falling trigger event configuration bit of line 20  *)
  EXTI_FTSR_TR21 = $00200000;  (*!< Falling trigger event configuration bit of line 21  *)
  EXTI_FTSR_TR22 = $00400000;  (*!< Falling trigger event configuration bit of line 22  *)
  EXTI_FTSR_TR23 = $00800000;  (*!< Falling trigger event configuration bit of line 23  *)
  (******************  Bit definition for EXTI_SWIER register  ***************** *)

  EXTI_SWIER_SWIER0 = $00000001;  (*!< Software Interrupt on line 0  *)
  EXTI_SWIER_SWIER1 = $00000002;  (*!< Software Interrupt on line 1  *)
  EXTI_SWIER_SWIER2 = $00000004;  (*!< Software Interrupt on line 2  *)
  EXTI_SWIER_SWIER3 = $00000008;  (*!< Software Interrupt on line 3  *)
  EXTI_SWIER_SWIER4 = $00000010;  (*!< Software Interrupt on line 4  *)
  EXTI_SWIER_SWIER5 = $00000020;  (*!< Software Interrupt on line 5  *)
  EXTI_SWIER_SWIER6 = $00000040;  (*!< Software Interrupt on line 6  *)
  EXTI_SWIER_SWIER7 = $00000080;  (*!< Software Interrupt on line 7  *)
  EXTI_SWIER_SWIER8 = $00000100;  (*!< Software Interrupt on line 8  *)
  EXTI_SWIER_SWIER9 = $00000200;  (*!< Software Interrupt on line 9  *)
  EXTI_SWIER_SWIER10 = $00000400;  (*!< Software Interrupt on line 10  *)
  EXTI_SWIER_SWIER11 = $00000800;  (*!< Software Interrupt on line 11  *)
  EXTI_SWIER_SWIER12 = $00001000;  (*!< Software Interrupt on line 12  *)
  EXTI_SWIER_SWIER13 = $00002000;  (*!< Software Interrupt on line 13  *)
  EXTI_SWIER_SWIER14 = $00004000;  (*!< Software Interrupt on line 14  *)
  EXTI_SWIER_SWIER15 = $00008000;  (*!< Software Interrupt on line 15  *)
  EXTI_SWIER_SWIER16 = $00010000;  (*!< Software Interrupt on line 16  *)
  EXTI_SWIER_SWIER17 = $00020000;  (*!< Software Interrupt on line 17  *)
  EXTI_SWIER_SWIER18 = $00040000;  (*!< Software Interrupt on line 18  *)
  EXTI_SWIER_SWIER19 = $00080000;  (*!< Software Interrupt on line 19  *)
  EXTI_SWIER_SWIER20 = $00100000;  (*!< Software Interrupt on line 20  *)
  EXTI_SWIER_SWIER21 = $00200000;  (*!< Software Interrupt on line 21  *)
  EXTI_SWIER_SWIER22 = $00400000;  (*!< Software Interrupt on line 22  *)
  EXTI_SWIER_SWIER23 = $00800000;  (*!< Software Interrupt on line 23  *)
  (*******************  Bit definition for EXTI_PR register  ******************* *)

  EXTI_PR_PR0 = $00000001;  (*!< Pending bit for line 0  *)
  EXTI_PR_PR1 = $00000002;  (*!< Pending bit for line 1  *)
  EXTI_PR_PR2 = $00000004;  (*!< Pending bit for line 2  *)
  EXTI_PR_PR3 = $00000008;  (*!< Pending bit for line 3  *)
  EXTI_PR_PR4 = $00000010;  (*!< Pending bit for line 4  *)
  EXTI_PR_PR5 = $00000020;  (*!< Pending bit for line 5  *)
  EXTI_PR_PR6 = $00000040;  (*!< Pending bit for line 6  *)
  EXTI_PR_PR7 = $00000080;  (*!< Pending bit for line 7  *)
  EXTI_PR_PR8 = $00000100;  (*!< Pending bit for line 8  *)
  EXTI_PR_PR9 = $00000200;  (*!< Pending bit for line 9  *)
  EXTI_PR_PR10 = $00000400;  (*!< Pending bit for line 10  *)
  EXTI_PR_PR11 = $00000800;  (*!< Pending bit for line 11  *)
  EXTI_PR_PR12 = $00001000;  (*!< Pending bit for line 12  *)
  EXTI_PR_PR13 = $00002000;  (*!< Pending bit for line 13  *)
  EXTI_PR_PR14 = $00004000;  (*!< Pending bit for line 14  *)
  EXTI_PR_PR15 = $00008000;  (*!< Pending bit for line 15  *)
  EXTI_PR_PR16 = $00010000;  (*!< Pending bit for line 16  *)
  EXTI_PR_PR17 = $00020000;  (*!< Pending bit for line 17  *)
  EXTI_PR_PR18 = $00040000;  (*!< Pending bit for line 18  *)
  EXTI_PR_PR19 = $00080000;  (*!< Pending bit for line 19  *)
  EXTI_PR_PR20 = $00100000;  (*!< Pending bit for line 20  *)
  EXTI_PR_PR21 = $00200000;  (*!< Pending bit for line 21  *)
  EXTI_PR_PR22 = $00400000;  (*!< Pending bit for line 22  *)
  EXTI_PR_PR23 = $00800000;  (*!< Pending bit for line 23  *)

(***************************************************************************** *)

(*                                                                             *)

(*                                    FLASH                                    *)

(*                                                                             *)

(***************************************************************************** *)

(*******************  Bits definition for FLASH_ACR register  **************** *)

const
  FLASH_ACR_LATENCY = $0000000F;
  FLASH_ACR_LATENCY_0WS = $00000000;
  FLASH_ACR_LATENCY_1WS = $00000001;
  FLASH_ACR_LATENCY_2WS = $00000002;
  FLASH_ACR_LATENCY_3WS = $00000003;
  FLASH_ACR_LATENCY_4WS = $00000004;
  FLASH_ACR_LATENCY_5WS = $00000005;
  FLASH_ACR_LATENCY_6WS = $00000006;
  FLASH_ACR_LATENCY_7WS = $00000007;
  FLASH_ACR_LATENCY_8WS = $00000008;
  FLASH_ACR_LATENCY_9WS = $00000009;
  FLASH_ACR_LATENCY_10WS = $0000000A;
  FLASH_ACR_LATENCY_11WS = $0000000B;
  FLASH_ACR_LATENCY_12WS = $0000000C;
  FLASH_ACR_LATENCY_13WS = $0000000D;
  FLASH_ACR_LATENCY_14WS = $0000000E;
  FLASH_ACR_LATENCY_15WS = $0000000F;
  FLASH_ACR_PRFTEN = $00000100;
  FLASH_ACR_ARTEN = $00000200;
  FLASH_ACR_ARTRST = $00000800;
  (*******************  Bits definition for FLASH_SR register  ***************** *)

  FLASH_SR_EOP = $00000001;
  FLASH_SR_OPERR = $00000002;
  FLASH_SR_WRPERR = $00000010;
  FLASH_SR_PGAERR = $00000020;
  FLASH_SR_PGPERR = $00000040;
  FLASH_SR_ERSERR = $00000080;
  FLASH_SR_BSY = $00010000;
  (*******************  Bits definition for FLASH_CR register  ***************** *)

  FLASH_CR_PG = $00000001;
  FLASH_CR_SER = $00000002;
  FLASH_CR_MER = $00000004;
  FLASH_CR_SNB = $00000078;
  FLASH_CR_SNB_0 = $00000008;
  FLASH_CR_SNB_1 = $00000010;
  FLASH_CR_SNB_2 = $00000020;
  FLASH_CR_SNB_3 = $00000040;
  FLASH_CR_PSIZE = $00000300;
  FLASH_CR_PSIZE_0 = $00000100;
  FLASH_CR_PSIZE_1 = $00000200;
  FLASH_CR_STRT = $00010000;
  FLASH_CR_EOPIE = $01000000;
  FLASH_CR_ERRIE = $02000000;
  FLASH_CR_LOCK = $80000000;
  (*******************  Bits definition for FLASH_OPTCR register  ************** *)

  FLASH_OPTCR_OPTLOCK = $00000001;
  FLASH_OPTCR_OPTSTRT = $00000002;
  FLASH_OPTCR_BOR_LEV = $0000000C;
  FLASH_OPTCR_BOR_LEV_0 = $00000004;
  FLASH_OPTCR_BOR_LEV_1 = $00000008;
  FLASH_OPTCR_WWDG_SW = $00000010;
  FLASH_OPTCR_IWDG_SW = $00000020;
  FLASH_OPTCR_nRST_STOP = $00000040;
  FLASH_OPTCR_nRST_STDBY = $00000080;
  FLASH_OPTCR_RDP = $0000FF00;
  FLASH_OPTCR_RDP_0 = $00000100;
  FLASH_OPTCR_RDP_1 = $00000200;
  FLASH_OPTCR_RDP_2 = $00000400;
  FLASH_OPTCR_RDP_3 = $00000800;
  FLASH_OPTCR_RDP_4 = $00001000;
  FLASH_OPTCR_RDP_5 = $00002000;
  FLASH_OPTCR_RDP_6 = $00004000;
  FLASH_OPTCR_RDP_7 = $00008000;
  FLASH_OPTCR_nWRP = $00FF0000;
  FLASH_OPTCR_nWRP_0 = $00010000;
  FLASH_OPTCR_nWRP_1 = $00020000;
  FLASH_OPTCR_nWRP_2 = $00040000;
  FLASH_OPTCR_nWRP_3 = $00080000;
  FLASH_OPTCR_nWRP_4 = $00100000;
  FLASH_OPTCR_nWRP_5 = $00200000;
  FLASH_OPTCR_nWRP_6 = $00400000;
  FLASH_OPTCR_nWRP_7 = $00800000;
  FLASH_OPTCR_IWDG_STDBY = $40000000;
  FLASH_OPTCR_IWDG_STOP = $80000000;
  (*******************  Bits definition for FLASH_OPTCR1 register  ************** *)

  FLASH_OPTCR1_BOOT_ADD0 = $0000FFFF;
  FLASH_OPTCR1_BOOT_ADD1 = $FFFF0000;
  (***************************************************************************** *)

  (*                                                                             *)

  (*                          Flexible Memory Controller                         *)

  (*                                                                             *)

  (***************************************************************************** *)

  (******************  Bit definition for FMC_BCR1 register  ****************** *)

  FMC_BCR1_MBKEN = $00000001;  (*!<Memory bank enable bit                  *)
  FMC_BCR1_MUXEN = $00000002;  (*!<Address/data multiplexing enable bit    *)
  FMC_BCR1_MTYP = $0000000C;  (*!<MTYP[1:0] bits (Memory type)            *)
  FMC_BCR1_MTYP_0 = $00000004;  (*!<Bit 0  *)
  FMC_BCR1_MTYP_1 = $00000008;  (*!<Bit 1  *)
  FMC_BCR1_MWID = $00000030;  (*!<MWID[1:0] bits (Memory data bus width)  *)
  FMC_BCR1_MWID_0 = $00000010;  (*!<Bit 0  *)
  FMC_BCR1_MWID_1 = $00000020;  (*!<Bit 1  *)
  FMC_BCR1_FACCEN = $00000040;  (*!<Flash access enable         *)
  FMC_BCR1_BURSTEN = $00000100;  (*!<Burst enable bit            *)
  FMC_BCR1_WAITPOL = $00000200;  (*!<Wait signal polarity bit    *)
  FMC_BCR1_WRAPMOD = $00000400;  (*!<Wrapped burst mode support  *)
  FMC_BCR1_WAITCFG = $00000800;  (*!<Wait timing configuration   *)
  FMC_BCR1_WREN = $00001000;  (*!<Write enable bit            *)
  FMC_BCR1_WAITEN = $00002000;  (*!<Wait enable bit             *)
  FMC_BCR1_EXTMOD = $00004000;  (*!<Extended mode enable        *)
  FMC_BCR1_ASYNCWAIT = $00008000;  (*!<Asynchronous wait           *)
  FMC_BCR1_CPSIZE = $00070000;  (*!<CRAM page size              *)
  FMC_BCR1_CPSIZE_0 = $00010000;  (*!<Bit 0  *)
  FMC_BCR1_CPSIZE_1 = $00020000;  (*!<Bit 1  *)
  FMC_BCR1_CPSIZE_2 = $00040000;  (*!<Bit 2  *)
  FMC_BCR1_CBURSTRW = $00080000;  (*!<Write burst enable          *)
  FMC_BCR1_CCLKEN = $00100000;  (*!<Continous clock enable      *)
  FMC_BCR1_WFDIS = $00200000;  (*!<Write FIFO Disable          *)
  (******************  Bit definition for FMC_BCR2 register  ****************** *)

  FMC_BCR2_MBKEN = $00000001;  (*!<Memory bank enable bit                  *)
  FMC_BCR2_MUXEN = $00000002;  (*!<Address/data multiplexing enable bit    *)
  FMC_BCR2_MTYP = $0000000C;  (*!<MTYP[1:0] bits (Memory type)            *)
  FMC_BCR2_MTYP_0 = $00000004;  (*!<Bit 0  *)
  FMC_BCR2_MTYP_1 = $00000008;  (*!<Bit 1  *)
  FMC_BCR2_MWID = $00000030;  (*!<MWID[1:0] bits (Memory data bus width)  *)
  FMC_BCR2_MWID_0 = $00000010;  (*!<Bit 0  *)
  FMC_BCR2_MWID_1 = $00000020;  (*!<Bit 1  *)
  FMC_BCR2_FACCEN = $00000040;  (*!<Flash access enable         *)
  FMC_BCR2_BURSTEN = $00000100;  (*!<Burst enable bit            *)
  FMC_BCR2_WAITPOL = $00000200;  (*!<Wait signal polarity bit    *)
  FMC_BCR2_WRAPMOD = $00000400;  (*!<Wrapped burst mode support  *)
  FMC_BCR2_WAITCFG = $00000800;  (*!<Wait timing configuration   *)
  FMC_BCR2_WREN = $00001000;  (*!<Write enable bit            *)
  FMC_BCR2_WAITEN = $00002000;  (*!<Wait enable bit             *)
  FMC_BCR2_EXTMOD = $00004000;  (*!<Extended mode enable        *)
  FMC_BCR2_ASYNCWAIT = $00008000;  (*!<Asynchronous wait           *)
  FMC_BCR2_CPSIZE = $00070000;  (*!<CRAM page size              *)
  FMC_BCR2_CPSIZE_0 = $00010000;  (*!<Bit 0  *)
  FMC_BCR2_CPSIZE_1 = $00020000;  (*!<Bit 1  *)
  FMC_BCR2_CPSIZE_2 = $00040000;  (*!<Bit 2  *)
  FMC_BCR2_CBURSTRW = $00080000;  (*!<Write burst enable          *)
  (******************  Bit definition for FMC_BCR3 register  ****************** *)

  FMC_BCR3_MBKEN = $00000001;  (*!<Memory bank enable bit                  *)
  FMC_BCR3_MUXEN = $00000002;  (*!<Address/data multiplexing enable bit    *)
  FMC_BCR3_MTYP = $0000000C;  (*!<MTYP[1:0] bits (Memory type)            *)
  FMC_BCR3_MTYP_0 = $00000004;  (*!<Bit 0  *)
  FMC_BCR3_MTYP_1 = $00000008;  (*!<Bit 1  *)
  FMC_BCR3_MWID = $00000030;  (*!<MWID[1:0] bits (Memory data bus width)  *)
  FMC_BCR3_MWID_0 = $00000010;  (*!<Bit 0  *)
  FMC_BCR3_MWID_1 = $00000020;  (*!<Bit 1  *)
  FMC_BCR3_FACCEN = $00000040;  (*!<Flash access enable         *)
  FMC_BCR3_BURSTEN = $00000100;  (*!<Burst enable bit            *)
  FMC_BCR3_WAITPOL = $00000200;  (*!<Wait signal polarity bit    *)
  FMC_BCR3_WRAPMOD = $00000400;  (*!<Wrapped burst mode support  *)
  FMC_BCR3_WAITCFG = $00000800;  (*!<Wait timing configuration   *)
  FMC_BCR3_WREN = $00001000;  (*!<Write enable bit            *)
  FMC_BCR3_WAITEN = $00002000;  (*!<Wait enable bit             *)
  FMC_BCR3_EXTMOD = $00004000;  (*!<Extended mode enable        *)
  FMC_BCR3_ASYNCWAIT = $00008000;  (*!<Asynchronous wait           *)
  FMC_BCR3_CPSIZE = $00070000;  (*!<CRAM page size              *)
  FMC_BCR3_CPSIZE_0 = $00010000;  (*!<Bit 0  *)
  FMC_BCR3_CPSIZE_1 = $00020000;  (*!<Bit 1  *)
  FMC_BCR3_CPSIZE_2 = $00040000;  (*!<Bit 2  *)
  FMC_BCR3_CBURSTRW = $00080000;  (*!<Write burst enable          *)
  (******************  Bit definition for FMC_BCR4 register  ****************** *)

  FMC_BCR4_MBKEN = $00000001;  (*!<Memory bank enable bit                  *)
  FMC_BCR4_MUXEN = $00000002;  (*!<Address/data multiplexing enable bit    *)
  FMC_BCR4_MTYP = $0000000C;  (*!<MTYP[1:0] bits (Memory type)            *)
  FMC_BCR4_MTYP_0 = $00000004;  (*!<Bit 0  *)
  FMC_BCR4_MTYP_1 = $00000008;  (*!<Bit 1  *)
  FMC_BCR4_MWID = $00000030;  (*!<MWID[1:0] bits (Memory data bus width)  *)
  FMC_BCR4_MWID_0 = $00000010;  (*!<Bit 0  *)
  FMC_BCR4_MWID_1 = $00000020;  (*!<Bit 1  *)
  FMC_BCR4_FACCEN = $00000040;  (*!<Flash access enable         *)
  FMC_BCR4_BURSTEN = $00000100;  (*!<Burst enable bit            *)
  FMC_BCR4_WAITPOL = $00000200;  (*!<Wait signal polarity bit    *)
  FMC_BCR4_WRAPMOD = $00000400;  (*!<Wrapped burst mode support  *)
  FMC_BCR4_WAITCFG = $00000800;  (*!<Wait timing configuration   *)
  FMC_BCR4_WREN = $00001000;  (*!<Write enable bit            *)
  FMC_BCR4_WAITEN = $00002000;  (*!<Wait enable bit             *)
  FMC_BCR4_EXTMOD = $00004000;  (*!<Extended mode enable        *)
  FMC_BCR4_ASYNCWAIT = $00008000;  (*!<Asynchronous wait           *)
  FMC_BCR4_CPSIZE = $00070000;  (*!<CRAM page size              *)
  FMC_BCR4_CPSIZE_0 = $00010000;  (*!<Bit 0  *)
  FMC_BCR4_CPSIZE_1 = $00020000;  (*!<Bit 1  *)
  FMC_BCR4_CPSIZE_2 = $00040000;  (*!<Bit 2  *)
  FMC_BCR4_CBURSTRW = $00080000;  (*!<Write burst enable          *)
  (******************  Bit definition for FMC_BTR1 register  ***************** *)

  FMC_BTR1_ADDSET = $0000000F;  (*!<ADDSET[3:0] bits (Address setup phase duration)  *)
  FMC_BTR1_ADDSET_0 = $00000001;  (*!<Bit 0  *)
  FMC_BTR1_ADDSET_1 = $00000002;  (*!<Bit 1  *)
  FMC_BTR1_ADDSET_2 = $00000004;  (*!<Bit 2  *)
  FMC_BTR1_ADDSET_3 = $00000008;  (*!<Bit 3  *)
  FMC_BTR1_ADDHLD = $000000F0;  (*!<ADDHLD[3:0] bits (Address-hold phase duration)   *)
  FMC_BTR1_ADDHLD_0 = $00000010;  (*!<Bit 0  *)
  FMC_BTR1_ADDHLD_1 = $00000020;  (*!<Bit 1  *)
  FMC_BTR1_ADDHLD_2 = $00000040;  (*!<Bit 2  *)
  FMC_BTR1_ADDHLD_3 = $00000080;  (*!<Bit 3  *)
  FMC_BTR1_DATAST = $0000FF00;  (*!<DATAST [3:0] bits (Data-phase duration)  *)
  FMC_BTR1_DATAST_0 = $00000100;  (*!<Bit 0  *)
  FMC_BTR1_DATAST_1 = $00000200;  (*!<Bit 1  *)
  FMC_BTR1_DATAST_2 = $00000400;  (*!<Bit 2  *)
  FMC_BTR1_DATAST_3 = $00000800;  (*!<Bit 3  *)
  FMC_BTR1_DATAST_4 = $00001000;  (*!<Bit 4  *)
  FMC_BTR1_DATAST_5 = $00002000;  (*!<Bit 5  *)
  FMC_BTR1_DATAST_6 = $00004000;  (*!<Bit 6  *)
  FMC_BTR1_DATAST_7 = $00008000;  (*!<Bit 7  *)
  FMC_BTR1_BUSTURN = $000F0000;  (*!<BUSTURN[3:0] bits (Bus turnaround phase duration)  *)
  FMC_BTR1_BUSTURN_0 = $00010000;  (*!<Bit 0  *)
  FMC_BTR1_BUSTURN_1 = $00020000;  (*!<Bit 1  *)
  FMC_BTR1_BUSTURN_2 = $00040000;  (*!<Bit 2  *)
  FMC_BTR1_BUSTURN_3 = $00080000;  (*!<Bit 3  *)
  FMC_BTR1_CLKDIV = $00F00000;  (*!<CLKDIV[3:0] bits (Clock divide ratio)  *)
  FMC_BTR1_CLKDIV_0 = $00100000;  (*!<Bit 0  *)
  FMC_BTR1_CLKDIV_1 = $00200000;  (*!<Bit 1  *)
  FMC_BTR1_CLKDIV_2 = $00400000;  (*!<Bit 2  *)
  FMC_BTR1_CLKDIV_3 = $00800000;  (*!<Bit 3  *)
  FMC_BTR1_DATLAT = $0F000000;  (*!<DATLA[3:0] bits (Data latency)  *)
  FMC_BTR1_DATLAT_0 = $01000000;  (*!<Bit 0  *)
  FMC_BTR1_DATLAT_1 = $02000000;  (*!<Bit 1  *)
  FMC_BTR1_DATLAT_2 = $04000000;  (*!<Bit 2  *)
  FMC_BTR1_DATLAT_3 = $08000000;  (*!<Bit 3  *)
  FMC_BTR1_ACCMOD = $30000000;  (*!<ACCMOD[1:0] bits (Access mode)  *)
  FMC_BTR1_ACCMOD_0 = $10000000;  (*!<Bit 0  *)
  FMC_BTR1_ACCMOD_1 = $20000000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_BTR2 register  ****************** *)

  FMC_BTR2_ADDSET = $0000000F;  (*!<ADDSET[3:0] bits (Address setup phase duration)  *)
  FMC_BTR2_ADDSET_0 = $00000001;  (*!<Bit 0  *)
  FMC_BTR2_ADDSET_1 = $00000002;  (*!<Bit 1  *)
  FMC_BTR2_ADDSET_2 = $00000004;  (*!<Bit 2  *)
  FMC_BTR2_ADDSET_3 = $00000008;  (*!<Bit 3  *)
  FMC_BTR2_ADDHLD = $000000F0;  (*!<ADDHLD[3:0] bits (Address-hold phase duration)  *)
  FMC_BTR2_ADDHLD_0 = $00000010;  (*!<Bit 0  *)
  FMC_BTR2_ADDHLD_1 = $00000020;  (*!<Bit 1  *)
  FMC_BTR2_ADDHLD_2 = $00000040;  (*!<Bit 2  *)
  FMC_BTR2_ADDHLD_3 = $00000080;  (*!<Bit 3  *)
  FMC_BTR2_DATAST = $0000FF00;  (*!<DATAST [3:0] bits (Data-phase duration)  *)
  FMC_BTR2_DATAST_0 = $00000100;  (*!<Bit 0  *)
  FMC_BTR2_DATAST_1 = $00000200;  (*!<Bit 1  *)
  FMC_BTR2_DATAST_2 = $00000400;  (*!<Bit 2  *)
  FMC_BTR2_DATAST_3 = $00000800;  (*!<Bit 3  *)
  FMC_BTR2_DATAST_4 = $00001000;  (*!<Bit 4  *)
  FMC_BTR2_DATAST_5 = $00002000;  (*!<Bit 5  *)
  FMC_BTR2_DATAST_6 = $00004000;  (*!<Bit 6  *)
  FMC_BTR2_DATAST_7 = $00008000;  (*!<Bit 7  *)
  FMC_BTR2_BUSTURN = $000F0000;  (*!<BUSTURN[3:0] bits (Bus turnaround phase duration)  *)
  FMC_BTR2_BUSTURN_0 = $00010000;  (*!<Bit 0  *)
  FMC_BTR2_BUSTURN_1 = $00020000;  (*!<Bit 1  *)
  FMC_BTR2_BUSTURN_2 = $00040000;  (*!<Bit 2  *)
  FMC_BTR2_BUSTURN_3 = $00080000;  (*!<Bit 3  *)
  FMC_BTR2_CLKDIV = $00F00000;  (*!<CLKDIV[3:0] bits (Clock divide ratio)  *)
  FMC_BTR2_CLKDIV_0 = $00100000;  (*!<Bit 0  *)
  FMC_BTR2_CLKDIV_1 = $00200000;  (*!<Bit 1  *)
  FMC_BTR2_CLKDIV_2 = $00400000;  (*!<Bit 2  *)
  FMC_BTR2_CLKDIV_3 = $00800000;  (*!<Bit 3  *)
  FMC_BTR2_DATLAT = $0F000000;  (*!<DATLA[3:0] bits (Data latency)  *)
  FMC_BTR2_DATLAT_0 = $01000000;  (*!<Bit 0  *)
  FMC_BTR2_DATLAT_1 = $02000000;  (*!<Bit 1  *)
  FMC_BTR2_DATLAT_2 = $04000000;  (*!<Bit 2  *)
  FMC_BTR2_DATLAT_3 = $08000000;  (*!<Bit 3  *)
  FMC_BTR2_ACCMOD = $30000000;  (*!<ACCMOD[1:0] bits (Access mode)  *)
  FMC_BTR2_ACCMOD_0 = $10000000;  (*!<Bit 0  *)
  FMC_BTR2_ACCMOD_1 = $20000000;  (*!<Bit 1  *)
  (*******************  Bit definition for FMC_BTR3 register  ****************** *)

  FMC_BTR3_ADDSET = $0000000F;  (*!<ADDSET[3:0] bits (Address setup phase duration)  *)
  FMC_BTR3_ADDSET_0 = $00000001;  (*!<Bit 0  *)
  FMC_BTR3_ADDSET_1 = $00000002;  (*!<Bit 1  *)
  FMC_BTR3_ADDSET_2 = $00000004;  (*!<Bit 2  *)
  FMC_BTR3_ADDSET_3 = $00000008;  (*!<Bit 3  *)
  FMC_BTR3_ADDHLD = $000000F0;  (*!<ADDHLD[3:0] bits (Address-hold phase duration)  *)
  FMC_BTR3_ADDHLD_0 = $00000010;  (*!<Bit 0  *)
  FMC_BTR3_ADDHLD_1 = $00000020;  (*!<Bit 1  *)
  FMC_BTR3_ADDHLD_2 = $00000040;  (*!<Bit 2  *)
  FMC_BTR3_ADDHLD_3 = $00000080;  (*!<Bit 3  *)
  FMC_BTR3_DATAST = $0000FF00;  (*!<DATAST [3:0] bits (Data-phase duration)  *)
  FMC_BTR3_DATAST_0 = $00000100;  (*!<Bit 0  *)
  FMC_BTR3_DATAST_1 = $00000200;  (*!<Bit 1  *)
  FMC_BTR3_DATAST_2 = $00000400;  (*!<Bit 2  *)
  FMC_BTR3_DATAST_3 = $00000800;  (*!<Bit 3  *)
  FMC_BTR3_DATAST_4 = $00001000;  (*!<Bit 4  *)
  FMC_BTR3_DATAST_5 = $00002000;  (*!<Bit 5  *)
  FMC_BTR3_DATAST_6 = $00004000;  (*!<Bit 6  *)
  FMC_BTR3_DATAST_7 = $00008000;  (*!<Bit 7  *)
  FMC_BTR3_BUSTURN = $000F0000;  (*!<BUSTURN[3:0] bits (Bus turnaround phase duration)  *)
  FMC_BTR3_BUSTURN_0 = $00010000;  (*!<Bit 0  *)
  FMC_BTR3_BUSTURN_1 = $00020000;  (*!<Bit 1  *)
  FMC_BTR3_BUSTURN_2 = $00040000;  (*!<Bit 2  *)
  FMC_BTR3_BUSTURN_3 = $00080000;  (*!<Bit 3  *)
  FMC_BTR3_CLKDIV = $00F00000;  (*!<CLKDIV[3:0] bits (Clock divide ratio)  *)
  FMC_BTR3_CLKDIV_0 = $00100000;  (*!<Bit 0  *)
  FMC_BTR3_CLKDIV_1 = $00200000;  (*!<Bit 1  *)
  FMC_BTR3_CLKDIV_2 = $00400000;  (*!<Bit 2  *)
  FMC_BTR3_CLKDIV_3 = $00800000;  (*!<Bit 3  *)
  FMC_BTR3_DATLAT = $0F000000;  (*!<DATLA[3:0] bits (Data latency)  *)
  FMC_BTR3_DATLAT_0 = $01000000;  (*!<Bit 0  *)
  FMC_BTR3_DATLAT_1 = $02000000;  (*!<Bit 1  *)
  FMC_BTR3_DATLAT_2 = $04000000;  (*!<Bit 2  *)
  FMC_BTR3_DATLAT_3 = $08000000;  (*!<Bit 3  *)
  FMC_BTR3_ACCMOD = $30000000;  (*!<ACCMOD[1:0] bits (Access mode)  *)
  FMC_BTR3_ACCMOD_0 = $10000000;  (*!<Bit 0  *)
  FMC_BTR3_ACCMOD_1 = $20000000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_BTR4 register  ****************** *)

  FMC_BTR4_ADDSET = $0000000F;  (*!<ADDSET[3:0] bits (Address setup phase duration)  *)
  FMC_BTR4_ADDSET_0 = $00000001;  (*!<Bit 0  *)
  FMC_BTR4_ADDSET_1 = $00000002;  (*!<Bit 1  *)
  FMC_BTR4_ADDSET_2 = $00000004;  (*!<Bit 2  *)
  FMC_BTR4_ADDSET_3 = $00000008;  (*!<Bit 3  *)
  FMC_BTR4_ADDHLD = $000000F0;  (*!<ADDHLD[3:0] bits (Address-hold phase duration)  *)
  FMC_BTR4_ADDHLD_0 = $00000010;  (*!<Bit 0  *)
  FMC_BTR4_ADDHLD_1 = $00000020;  (*!<Bit 1  *)
  FMC_BTR4_ADDHLD_2 = $00000040;  (*!<Bit 2  *)
  FMC_BTR4_ADDHLD_3 = $00000080;  (*!<Bit 3  *)
  FMC_BTR4_DATAST = $0000FF00;  (*!<DATAST [3:0] bits (Data-phase duration)  *)
  FMC_BTR4_DATAST_0 = $00000100;  (*!<Bit 0  *)
  FMC_BTR4_DATAST_1 = $00000200;  (*!<Bit 1  *)
  FMC_BTR4_DATAST_2 = $00000400;  (*!<Bit 2  *)
  FMC_BTR4_DATAST_3 = $00000800;  (*!<Bit 3  *)
  FMC_BTR4_DATAST_4 = $00001000;  (*!<Bit 4  *)
  FMC_BTR4_DATAST_5 = $00002000;  (*!<Bit 5  *)
  FMC_BTR4_DATAST_6 = $00004000;  (*!<Bit 6  *)
  FMC_BTR4_DATAST_7 = $00008000;  (*!<Bit 7  *)
  FMC_BTR4_BUSTURN = $000F0000;  (*!<BUSTURN[3:0] bits (Bus turnaround phase duration)  *)
  FMC_BTR4_BUSTURN_0 = $00010000;  (*!<Bit 0  *)
  FMC_BTR4_BUSTURN_1 = $00020000;  (*!<Bit 1  *)
  FMC_BTR4_BUSTURN_2 = $00040000;  (*!<Bit 2  *)
  FMC_BTR4_BUSTURN_3 = $00080000;  (*!<Bit 3  *)
  FMC_BTR4_CLKDIV = $00F00000;  (*!<CLKDIV[3:0] bits (Clock divide ratio)  *)
  FMC_BTR4_CLKDIV_0 = $00100000;  (*!<Bit 0  *)
  FMC_BTR4_CLKDIV_1 = $00200000;  (*!<Bit 1  *)
  FMC_BTR4_CLKDIV_2 = $00400000;  (*!<Bit 2  *)
  FMC_BTR4_CLKDIV_3 = $00800000;  (*!<Bit 3  *)
  FMC_BTR4_DATLAT = $0F000000;  (*!<DATLA[3:0] bits (Data latency)  *)
  FMC_BTR4_DATLAT_0 = $01000000;  (*!<Bit 0  *)
  FMC_BTR4_DATLAT_1 = $02000000;  (*!<Bit 1  *)
  FMC_BTR4_DATLAT_2 = $04000000;  (*!<Bit 2  *)
  FMC_BTR4_DATLAT_3 = $08000000;  (*!<Bit 3  *)
  FMC_BTR4_ACCMOD = $30000000;  (*!<ACCMOD[1:0] bits (Access mode)  *)
  FMC_BTR4_ACCMOD_0 = $10000000;  (*!<Bit 0  *)
  FMC_BTR4_ACCMOD_1 = $20000000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_BWTR1 register  ***************** *)

  FMC_BWTR1_ADDSET = $0000000F;  (*!<ADDSET[3:0] bits (Address setup phase duration)  *)
  FMC_BWTR1_ADDSET_0 = $00000001;  (*!<Bit 0  *)
  FMC_BWTR1_ADDSET_1 = $00000002;  (*!<Bit 1  *)
  FMC_BWTR1_ADDSET_2 = $00000004;  (*!<Bit 2  *)
  FMC_BWTR1_ADDSET_3 = $00000008;  (*!<Bit 3  *)
  FMC_BWTR1_ADDHLD = $000000F0;  (*!<ADDHLD[3:0] bits (Address-hold phase duration)  *)
  FMC_BWTR1_ADDHLD_0 = $00000010;  (*!<Bit 0  *)
  FMC_BWTR1_ADDHLD_1 = $00000020;  (*!<Bit 1  *)
  FMC_BWTR1_ADDHLD_2 = $00000040;  (*!<Bit 2  *)
  FMC_BWTR1_ADDHLD_3 = $00000080;  (*!<Bit 3  *)
  FMC_BWTR1_DATAST = $0000FF00;  (*!<DATAST [3:0] bits (Data-phase duration)  *)
  FMC_BWTR1_DATAST_0 = $00000100;  (*!<Bit 0  *)
  FMC_BWTR1_DATAST_1 = $00000200;  (*!<Bit 1  *)
  FMC_BWTR1_DATAST_2 = $00000400;  (*!<Bit 2  *)
  FMC_BWTR1_DATAST_3 = $00000800;  (*!<Bit 3  *)
  FMC_BWTR1_DATAST_4 = $00001000;  (*!<Bit 4  *)
  FMC_BWTR1_DATAST_5 = $00002000;  (*!<Bit 5  *)
  FMC_BWTR1_DATAST_6 = $00004000;  (*!<Bit 6  *)
  FMC_BWTR1_DATAST_7 = $00008000;  (*!<Bit 7  *)
  FMC_BWTR1_BUSTURN = $000F0000;  (*!<BUSTURN[3:0] bits (Bus turnaround phase duration)  *)
  FMC_BWTR1_BUSTURN_0 = $00010000;  (*!<Bit 0  *)
  FMC_BWTR1_BUSTURN_1 = $00020000;  (*!<Bit 1  *)
  FMC_BWTR1_BUSTURN_2 = $00040000;  (*!<Bit 2  *)
  FMC_BWTR1_BUSTURN_3 = $00080000;  (*!<Bit 3  *)
  FMC_BWTR1_ACCMOD = $30000000;  (*!<ACCMOD[1:0] bits (Access mode)  *)
  FMC_BWTR1_ACCMOD_0 = $10000000;  (*!<Bit 0  *)
  FMC_BWTR1_ACCMOD_1 = $20000000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_BWTR2 register  ***************** *)

  FMC_BWTR2_ADDSET = $0000000F;  (*!<ADDSET[3:0] bits (Address setup phase duration)  *)
  FMC_BWTR2_ADDSET_0 = $00000001;  (*!<Bit 0  *)
  FMC_BWTR2_ADDSET_1 = $00000002;  (*!<Bit 1  *)
  FMC_BWTR2_ADDSET_2 = $00000004;  (*!<Bit 2  *)
  FMC_BWTR2_ADDSET_3 = $00000008;  (*!<Bit 3  *)
  FMC_BWTR2_ADDHLD = $000000F0;  (*!<ADDHLD[3:0] bits (Address-hold phase duration)  *)
  FMC_BWTR2_ADDHLD_0 = $00000010;  (*!<Bit 0  *)
  FMC_BWTR2_ADDHLD_1 = $00000020;  (*!<Bit 1  *)
  FMC_BWTR2_ADDHLD_2 = $00000040;  (*!<Bit 2  *)
  FMC_BWTR2_ADDHLD_3 = $00000080;  (*!<Bit 3  *)
  FMC_BWTR2_DATAST = $0000FF00;  (*!<DATAST [3:0] bits (Data-phase duration)  *)
  FMC_BWTR2_DATAST_0 = $00000100;  (*!<Bit 0  *)
  FMC_BWTR2_DATAST_1 = $00000200;  (*!<Bit 1  *)
  FMC_BWTR2_DATAST_2 = $00000400;  (*!<Bit 2  *)
  FMC_BWTR2_DATAST_3 = $00000800;  (*!<Bit 3  *)
  FMC_BWTR2_DATAST_4 = $00001000;  (*!<Bit 4  *)
  FMC_BWTR2_DATAST_5 = $00002000;  (*!<Bit 5  *)
  FMC_BWTR2_DATAST_6 = $00004000;  (*!<Bit 6  *)
  FMC_BWTR2_DATAST_7 = $00008000;  (*!<Bit 7  *)
  FMC_BWTR2_BUSTURN = $000F0000;  (*!<BUSTURN[3:0] bits (Bus turnaround phase duration)  *)
  FMC_BWTR2_BUSTURN_0 = $00010000;  (*!<Bit 0  *)
  FMC_BWTR2_BUSTURN_1 = $00020000;  (*!<Bit 1  *)
  FMC_BWTR2_BUSTURN_2 = $00040000;  (*!<Bit 2  *)
  FMC_BWTR2_BUSTURN_3 = $00080000;  (*!<Bit 3  *)
  FMC_BWTR2_ACCMOD = $30000000;  (*!<ACCMOD[1:0] bits (Access mode)  *)
  FMC_BWTR2_ACCMOD_0 = $10000000;  (*!<Bit 0  *)
  FMC_BWTR2_ACCMOD_1 = $20000000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_BWTR3 register  ***************** *)

  FMC_BWTR3_ADDSET = $0000000F;  (*!<ADDSET[3:0] bits (Address setup phase duration)  *)
  FMC_BWTR3_ADDSET_0 = $00000001;  (*!<Bit 0  *)
  FMC_BWTR3_ADDSET_1 = $00000002;  (*!<Bit 1  *)
  FMC_BWTR3_ADDSET_2 = $00000004;  (*!<Bit 2  *)
  FMC_BWTR3_ADDSET_3 = $00000008;  (*!<Bit 3  *)
  FMC_BWTR3_ADDHLD = $000000F0;  (*!<ADDHLD[3:0] bits (Address-hold phase duration)  *)
  FMC_BWTR3_ADDHLD_0 = $00000010;  (*!<Bit 0  *)
  FMC_BWTR3_ADDHLD_1 = $00000020;  (*!<Bit 1  *)
  FMC_BWTR3_ADDHLD_2 = $00000040;  (*!<Bit 2  *)
  FMC_BWTR3_ADDHLD_3 = $00000080;  (*!<Bit 3  *)
  FMC_BWTR3_DATAST = $0000FF00;  (*!<DATAST [3:0] bits (Data-phase duration)  *)
  FMC_BWTR3_DATAST_0 = $00000100;  (*!<Bit 0  *)
  FMC_BWTR3_DATAST_1 = $00000200;  (*!<Bit 1  *)
  FMC_BWTR3_DATAST_2 = $00000400;  (*!<Bit 2  *)
  FMC_BWTR3_DATAST_3 = $00000800;  (*!<Bit 3  *)
  FMC_BWTR3_DATAST_4 = $00001000;  (*!<Bit 4  *)
  FMC_BWTR3_DATAST_5 = $00002000;  (*!<Bit 5  *)
  FMC_BWTR3_DATAST_6 = $00004000;  (*!<Bit 6  *)
  FMC_BWTR3_DATAST_7 = $00008000;  (*!<Bit 7  *)
  FMC_BWTR3_BUSTURN = $000F0000;  (*!<BUSTURN[3:0] bits (Bus turnaround phase duration)  *)
  FMC_BWTR3_BUSTURN_0 = $00010000;  (*!<Bit 0  *)
  FMC_BWTR3_BUSTURN_1 = $00020000;  (*!<Bit 1  *)
  FMC_BWTR3_BUSTURN_2 = $00040000;  (*!<Bit 2  *)
  FMC_BWTR3_BUSTURN_3 = $00080000;  (*!<Bit 3  *)
  FMC_BWTR3_ACCMOD = $30000000;  (*!<ACCMOD[1:0] bits (Access mode)  *)
  FMC_BWTR3_ACCMOD_0 = $10000000;  (*!<Bit 0  *)
  FMC_BWTR3_ACCMOD_1 = $20000000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_BWTR4 register  ***************** *)

  FMC_BWTR4_ADDSET = $0000000F;  (*!<ADDSET[3:0] bits (Address setup phase duration)  *)
  FMC_BWTR4_ADDSET_0 = $00000001;  (*!<Bit 0  *)
  FMC_BWTR4_ADDSET_1 = $00000002;  (*!<Bit 1  *)
  FMC_BWTR4_ADDSET_2 = $00000004;  (*!<Bit 2  *)
  FMC_BWTR4_ADDSET_3 = $00000008;  (*!<Bit 3  *)
  FMC_BWTR4_ADDHLD = $000000F0;  (*!<ADDHLD[3:0] bits (Address-hold phase duration)  *)
  FMC_BWTR4_ADDHLD_0 = $00000010;  (*!<Bit 0  *)
  FMC_BWTR4_ADDHLD_1 = $00000020;  (*!<Bit 1  *)
  FMC_BWTR4_ADDHLD_2 = $00000040;  (*!<Bit 2  *)
  FMC_BWTR4_ADDHLD_3 = $00000080;  (*!<Bit 3  *)
  FMC_BWTR4_DATAST = $0000FF00;  (*!<DATAST [3:0] bits (Data-phase duration)  *)
  FMC_BWTR4_DATAST_0 = $00000100;  (*!<Bit 0  *)
  FMC_BWTR4_DATAST_1 = $00000200;  (*!<Bit 1  *)
  FMC_BWTR4_DATAST_2 = $00000400;  (*!<Bit 2  *)
  FMC_BWTR4_DATAST_3 = $00000800;  (*!<Bit 3  *)
  FMC_BWTR4_DATAST_4 = $00001000;  (*!<Bit 4  *)
  FMC_BWTR4_DATAST_5 = $00002000;  (*!<Bit 5  *)
  FMC_BWTR4_DATAST_6 = $00004000;  (*!<Bit 6  *)
  FMC_BWTR4_DATAST_7 = $00008000;  (*!<Bit 7  *)
  FMC_BWTR4_BUSTURN = $000F0000;  (*!<BUSTURN[3:0] bits (Bus turnaround phase duration)  *)
  FMC_BWTR4_BUSTURN_0 = $00010000;  (*!<Bit 0  *)
  FMC_BWTR4_BUSTURN_1 = $00020000;  (*!<Bit 1  *)
  FMC_BWTR4_BUSTURN_2 = $00040000;  (*!<Bit 2  *)
  FMC_BWTR4_BUSTURN_3 = $00080000;  (*!<Bit 3  *)
  FMC_BWTR4_ACCMOD = $30000000;  (*!<ACCMOD[1:0] bits (Access mode)  *)
  FMC_BWTR4_ACCMOD_0 = $10000000;  (*!<Bit 0  *)
  FMC_BWTR4_ACCMOD_1 = $20000000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_PCR register  ****************** *)

  FMC_PCR_PWAITEN = $00000002;  (*!<Wait feature enable bit                    *)
  FMC_PCR_PBKEN = $00000004;  (*!<PC Card/NAND Flash memory bank enable bit  *)
  FMC_PCR_PTYP = $00000008;  (*!<Memory type                                *)
  FMC_PCR_PWID = $00000030;  (*!<PWID[1:0] bits (NAND Flash databus width)  *)
  FMC_PCR_PWID_0 = $00000010;  (*!<Bit 0  *)
  FMC_PCR_PWID_1 = $00000020;  (*!<Bit 1  *)
  FMC_PCR_ECCEN = $00000040;  (*!<ECC computation logic enable bit           *)
  FMC_PCR_TCLR = $00001E00;  (*!<TCLR[3:0] bits (CLE to RE delay)           *)
  FMC_PCR_TCLR_0 = $00000200;  (*!<Bit 0  *)
  FMC_PCR_TCLR_1 = $00000400;  (*!<Bit 1  *)
  FMC_PCR_TCLR_2 = $00000800;  (*!<Bit 2  *)
  FMC_PCR_TCLR_3 = $00001000;  (*!<Bit 3  *)
  FMC_PCR_TAR = $0001E000;  (*!<TAR[3:0] bits (ALE to RE delay)            *)
  FMC_PCR_TAR_0 = $00002000;  (*!<Bit 0  *)
  FMC_PCR_TAR_1 = $00004000;  (*!<Bit 1  *)
  FMC_PCR_TAR_2 = $00008000;  (*!<Bit 2  *)
  FMC_PCR_TAR_3 = $00010000;  (*!<Bit 3  *)
  FMC_PCR_ECCPS = $000E0000;  (*!<ECCPS[2:0] bits (ECC page size)            *)
  FMC_PCR_ECCPS_0 = $00020000;  (*!<Bit 0  *)
  FMC_PCR_ECCPS_1 = $00040000;  (*!<Bit 1  *)
  FMC_PCR_ECCPS_2 = $00080000;  (*!<Bit 2  *)
  (*******************  Bit definition for FMC_SR register  ****************** *)

  FMC_SR_IRS = $01;  (*!<Interrupt Rising Edge status                 *)
  FMC_SR_ILS = $02;  (*!<Interrupt Level status                       *)
  FMC_SR_IFS = $04;  (*!<Interrupt Falling Edge status                *)
  FMC_SR_IREN = $08;  (*!<Interrupt Rising Edge detection Enable bit   *)
  FMC_SR_ILEN = $10;  (*!<Interrupt Level detection Enable bit         *)
  FMC_SR_IFEN = $20;  (*!<Interrupt Falling Edge detection Enable bit  *)
  FMC_SR_FEMPT = $40;  (*!<FIFO empty                                   *)
  (******************  Bit definition for FMC_PMEM register  ***************** *)

  FMC_PMEM_MEMSET3 = $000000FF;  (*!<MEMSET3[7:0] bits (Common memory 3 setup time)  *)
  FMC_PMEM_MEMSET3_0 = $00000001;  (*!<Bit 0  *)
  FMC_PMEM_MEMSET3_1 = $00000002;  (*!<Bit 1  *)
  FMC_PMEM_MEMSET3_2 = $00000004;  (*!<Bit 2  *)
  FMC_PMEM_MEMSET3_3 = $00000008;  (*!<Bit 3  *)
  FMC_PMEM_MEMSET3_4 = $00000010;  (*!<Bit 4  *)
  FMC_PMEM_MEMSET3_5 = $00000020;  (*!<Bit 5  *)
  FMC_PMEM_MEMSET3_6 = $00000040;  (*!<Bit 6  *)
  FMC_PMEM_MEMSET3_7 = $00000080;  (*!<Bit 7  *)
  FMC_PMEM_MEMWAIT3 = $0000FF00;  (*!<MEMWAIT3[7:0] bits (Common memory 3 wait time)  *)
  FMC_PMEM_MEMWAIT3_0 = $00000100;  (*!<Bit 0  *)
  FMC_PMEM_MEMWAIT3_1 = $00000200;  (*!<Bit 1  *)
  FMC_PMEM_MEMWAIT3_2 = $00000400;  (*!<Bit 2  *)
  FMC_PMEM_MEMWAIT3_3 = $00000800;  (*!<Bit 3  *)
  FMC_PMEM_MEMWAIT3_4 = $00001000;  (*!<Bit 4  *)
  FMC_PMEM_MEMWAIT3_5 = $00002000;  (*!<Bit 5  *)
  FMC_PMEM_MEMWAIT3_6 = $00004000;  (*!<Bit 6  *)
  FMC_PMEM_MEMWAIT3_7 = $00008000;  (*!<Bit 7  *)
  FMC_PMEM_MEMHOLD3 = $00FF0000;  (*!<MEMHOLD3[7:0] bits (Common memory 3 hold time)  *)
  FMC_PMEM_MEMHOLD3_0 = $00010000;  (*!<Bit 0  *)
  FMC_PMEM_MEMHOLD3_1 = $00020000;  (*!<Bit 1  *)
  FMC_PMEM_MEMHOLD3_2 = $00040000;  (*!<Bit 2  *)
  FMC_PMEM_MEMHOLD3_3 = $00080000;  (*!<Bit 3  *)
  FMC_PMEM_MEMHOLD3_4 = $00100000;  (*!<Bit 4  *)
  FMC_PMEM_MEMHOLD3_5 = $00200000;  (*!<Bit 5  *)
  FMC_PMEM_MEMHOLD3_6 = $00400000;  (*!<Bit 6  *)
  FMC_PMEM_MEMHOLD3_7 = $00800000;  (*!<Bit 7  *)
  FMC_PMEM_MEMHIZ3 = $FF000000;  (*!<MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time)  *)
  FMC_PMEM_MEMHIZ3_0 = $01000000;  (*!<Bit 0  *)
  FMC_PMEM_MEMHIZ3_1 = $02000000;  (*!<Bit 1  *)
  FMC_PMEM_MEMHIZ3_2 = $04000000;  (*!<Bit 2  *)
  FMC_PMEM_MEMHIZ3_3 = $08000000;  (*!<Bit 3  *)
  FMC_PMEM_MEMHIZ3_4 = $10000000;  (*!<Bit 4  *)
  FMC_PMEM_MEMHIZ3_5 = $20000000;  (*!<Bit 5  *)
  FMC_PMEM_MEMHIZ3_6 = $40000000;  (*!<Bit 6  *)
  FMC_PMEM_MEMHIZ3_7 = $80000000;  (*!<Bit 7  *)
  (******************  Bit definition for FMC_PATT register  ***************** *)

  FMC_PATT_ATTSET3 = $000000FF;  (*!<ATTSET3[7:0] bits (Attribute memory 3 setup time)  *)
  FMC_PATT_ATTSET3_0 = $00000001;  (*!<Bit 0  *)
  FMC_PATT_ATTSET3_1 = $00000002;  (*!<Bit 1  *)
  FMC_PATT_ATTSET3_2 = $00000004;  (*!<Bit 2  *)
  FMC_PATT_ATTSET3_3 = $00000008;  (*!<Bit 3  *)
  FMC_PATT_ATTSET3_4 = $00000010;  (*!<Bit 4  *)
  FMC_PATT_ATTSET3_5 = $00000020;  (*!<Bit 5  *)
  FMC_PATT_ATTSET3_6 = $00000040;  (*!<Bit 6  *)
  FMC_PATT_ATTSET3_7 = $00000080;  (*!<Bit 7  *)
  FMC_PATT_ATTWAIT3 = $0000FF00;  (*!<ATTWAIT3[7:0] bits (Attribute memory 3 wait time)  *)
  FMC_PATT_ATTWAIT3_0 = $00000100;  (*!<Bit 0  *)
  FMC_PATT_ATTWAIT3_1 = $00000200;  (*!<Bit 1  *)
  FMC_PATT_ATTWAIT3_2 = $00000400;  (*!<Bit 2  *)
  FMC_PATT_ATTWAIT3_3 = $00000800;  (*!<Bit 3  *)
  FMC_PATT_ATTWAIT3_4 = $00001000;  (*!<Bit 4  *)
  FMC_PATT_ATTWAIT3_5 = $00002000;  (*!<Bit 5  *)
  FMC_PATT_ATTWAIT3_6 = $00004000;  (*!<Bit 6  *)
  FMC_PATT_ATTWAIT3_7 = $00008000;  (*!<Bit 7  *)
  FMC_PATT_ATTHOLD3 = $00FF0000;  (*!<ATTHOLD3[7:0] bits (Attribute memory 3 hold time)  *)
  FMC_PATT_ATTHOLD3_0 = $00010000;  (*!<Bit 0  *)
  FMC_PATT_ATTHOLD3_1 = $00020000;  (*!<Bit 1  *)
  FMC_PATT_ATTHOLD3_2 = $00040000;  (*!<Bit 2  *)
  FMC_PATT_ATTHOLD3_3 = $00080000;  (*!<Bit 3  *)
  FMC_PATT_ATTHOLD3_4 = $00100000;  (*!<Bit 4  *)
  FMC_PATT_ATTHOLD3_5 = $00200000;  (*!<Bit 5  *)
  FMC_PATT_ATTHOLD3_6 = $00400000;  (*!<Bit 6  *)
  FMC_PATT_ATTHOLD3_7 = $00800000;  (*!<Bit 7  *)
  FMC_PATT_ATTHIZ3 = $FF000000;  (*!<ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time)  *)
  FMC_PATT_ATTHIZ3_0 = $01000000;  (*!<Bit 0  *)
  FMC_PATT_ATTHIZ3_1 = $02000000;  (*!<Bit 1  *)
  FMC_PATT_ATTHIZ3_2 = $04000000;  (*!<Bit 2  *)
  FMC_PATT_ATTHIZ3_3 = $08000000;  (*!<Bit 3  *)
  FMC_PATT_ATTHIZ3_4 = $10000000;  (*!<Bit 4  *)
  FMC_PATT_ATTHIZ3_5 = $20000000;  (*!<Bit 5  *)
  FMC_PATT_ATTHIZ3_6 = $40000000;  (*!<Bit 6  *)
  FMC_PATT_ATTHIZ3_7 = $80000000;  (*!<Bit 7  *)
  (******************  Bit definition for FMC_ECCR register  ***************** *)

  FMC_ECCR_ECC3 = $FFFFFFFF;  (*!<ECC result  *)
  (******************  Bit definition for FMC_SDCR1 register  ***************** *)

  FMC_SDCR1_NC = $00000003;  (*!<NC[1:0] bits (Number of column bits)  *)
  FMC_SDCR1_NC_0 = $00000001;  (*!<Bit 0  *)
  FMC_SDCR1_NC_1 = $00000002;  (*!<Bit 1  *)
  FMC_SDCR1_NR = $0000000C;  (*!<NR[1:0] bits (Number of row bits)  *)
  FMC_SDCR1_NR_0 = $00000004;  (*!<Bit 0  *)
  FMC_SDCR1_NR_1 = $00000008;  (*!<Bit 1  *)
  FMC_SDCR1_MWID = $00000030;  (*!<NR[1:0] bits (Number of row bits)  *)
  FMC_SDCR1_MWID_0 = $00000010;  (*!<Bit 0  *)
  FMC_SDCR1_MWID_1 = $00000020;  (*!<Bit 1  *)
  FMC_SDCR1_NB = $00000040;  (*!<Number of internal bank  *)
  FMC_SDCR1_CAS = $00000180;  (*!<CAS[1:0] bits (CAS latency)  *)
  FMC_SDCR1_CAS_0 = $00000080;  (*!<Bit 0  *)
  FMC_SDCR1_CAS_1 = $00000100;  (*!<Bit 1  *)
  FMC_SDCR1_WP = $00000200;  (*!<Write protection  *)
  FMC_SDCR1_SDCLK = $00000C00;  (*!<SDRAM clock configuration  *)
  FMC_SDCR1_SDCLK_0 = $00000400;  (*!<Bit 0  *)
  FMC_SDCR1_SDCLK_1 = $00000800;  (*!<Bit 1  *)
  FMC_SDCR1_RBURST = $00001000;  (*!<Read burst  *)
  FMC_SDCR1_RPIPE = $00006000;  (*!<Write protection  *)
  FMC_SDCR1_RPIPE_0 = $00002000;  (*!<Bit 0  *)
  FMC_SDCR1_RPIPE_1 = $00004000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_SDCR2 register  ***************** *)

  FMC_SDCR2_NC = $00000003;  (*!<NC[1:0] bits (Number of column bits)  *)
  FMC_SDCR2_NC_0 = $00000001;  (*!<Bit 0  *)
  FMC_SDCR2_NC_1 = $00000002;  (*!<Bit 1  *)
  FMC_SDCR2_NR = $0000000C;  (*!<NR[1:0] bits (Number of row bits)  *)
  FMC_SDCR2_NR_0 = $00000004;  (*!<Bit 0  *)
  FMC_SDCR2_NR_1 = $00000008;  (*!<Bit 1  *)
  FMC_SDCR2_MWID = $00000030;  (*!<NR[1:0] bits (Number of row bits)  *)
  FMC_SDCR2_MWID_0 = $00000010;  (*!<Bit 0  *)
  FMC_SDCR2_MWID_1 = $00000020;  (*!<Bit 1  *)
  FMC_SDCR2_NB = $00000040;  (*!<Number of internal bank  *)
  FMC_SDCR2_CAS = $00000180;  (*!<CAS[1:0] bits (CAS latency)  *)
  FMC_SDCR2_CAS_0 = $00000080;  (*!<Bit 0  *)
  FMC_SDCR2_CAS_1 = $00000100;  (*!<Bit 1  *)
  FMC_SDCR2_WP = $00000200;  (*!<Write protection  *)
  FMC_SDCR2_SDCLK = $00000C00;  (*!<SDCLK[1:0] (SDRAM clock configuration)  *)
  FMC_SDCR2_SDCLK_0 = $00000400;  (*!<Bit 0  *)
  FMC_SDCR2_SDCLK_1 = $00000800;  (*!<Bit 1  *)
  FMC_SDCR2_RBURST = $00001000;  (*!<Read burst  *)
  FMC_SDCR2_RPIPE = $00006000;  (*!<RPIPE[1:0](Read pipe)  *)
  FMC_SDCR2_RPIPE_0 = $00002000;  (*!<Bit 0  *)
  FMC_SDCR2_RPIPE_1 = $00004000;  (*!<Bit 1  *)
  (******************  Bit definition for FMC_SDTR1 register  ***************** *)

  FMC_SDTR1_TMRD = $0000000F;  (*!<TMRD[3:0] bits (Load mode register to active)  *)
  FMC_SDTR1_TMRD_0 = $00000001;  (*!<Bit 0  *)
  FMC_SDTR1_TMRD_1 = $00000002;  (*!<Bit 1  *)
  FMC_SDTR1_TMRD_2 = $00000004;  (*!<Bit 2  *)
  FMC_SDTR1_TMRD_3 = $00000008;  (*!<Bit 3  *)
  FMC_SDTR1_TXSR = $000000F0;  (*!<TXSR[3:0] bits (Exit self refresh)  *)
  FMC_SDTR1_TXSR_0 = $00000010;  (*!<Bit 0  *)
  FMC_SDTR1_TXSR_1 = $00000020;  (*!<Bit 1  *)
  FMC_SDTR1_TXSR_2 = $00000040;  (*!<Bit 2  *)
  FMC_SDTR1_TXSR_3 = $00000080;  (*!<Bit 3  *)
  FMC_SDTR1_TRAS = $00000F00;  (*!<TRAS[3:0] bits (Self refresh time)  *)
  FMC_SDTR1_TRAS_0 = $00000100;  (*!<Bit 0  *)
  FMC_SDTR1_TRAS_1 = $00000200;  (*!<Bit 1  *)
  FMC_SDTR1_TRAS_2 = $00000400;  (*!<Bit 2  *)
  FMC_SDTR1_TRAS_3 = $00000800;  (*!<Bit 3  *)
  FMC_SDTR1_TRC = $0000F000;  (*!<TRC[2:0] bits (Row cycle delay)  *)
  FMC_SDTR1_TRC_0 = $00001000;  (*!<Bit 0  *)
  FMC_SDTR1_TRC_1 = $00002000;  (*!<Bit 1  *)
  FMC_SDTR1_TRC_2 = $00004000;  (*!<Bit 2  *)
  FMC_SDTR1_TWR = $000F0000;  (*!<TRC[2:0] bits (Write recovery delay)  *)
  FMC_SDTR1_TWR_0 = $00010000;  (*!<Bit 0  *)
  FMC_SDTR1_TWR_1 = $00020000;  (*!<Bit 1  *)
  FMC_SDTR1_TWR_2 = $00040000;  (*!<Bit 2  *)
  FMC_SDTR1_TRP = $00F00000;  (*!<TRP[2:0] bits (Row precharge delay)  *)
  FMC_SDTR1_TRP_0 = $00100000;  (*!<Bit 0  *)
  FMC_SDTR1_TRP_1 = $00200000;  (*!<Bit 1  *)
  FMC_SDTR1_TRP_2 = $00400000;  (*!<Bit 2  *)
  FMC_SDTR1_TRCD = $0F000000;  (*!<TRP[2:0] bits (Row to column delay)  *)
  FMC_SDTR1_TRCD_0 = $01000000;  (*!<Bit 0  *)
  FMC_SDTR1_TRCD_1 = $02000000;  (*!<Bit 1  *)
  FMC_SDTR1_TRCD_2 = $04000000;  (*!<Bit 2  *)
  (******************  Bit definition for FMC_SDTR2 register  ***************** *)

  FMC_SDTR2_TMRD = $0000000F;  (*!<TMRD[3:0] bits (Load mode register to active)  *)
  FMC_SDTR2_TMRD_0 = $00000001;  (*!<Bit 0  *)
  FMC_SDTR2_TMRD_1 = $00000002;  (*!<Bit 1  *)
  FMC_SDTR2_TMRD_2 = $00000004;  (*!<Bit 2  *)
  FMC_SDTR2_TMRD_3 = $00000008;  (*!<Bit 3  *)
  FMC_SDTR2_TXSR = $000000F0;  (*!<TXSR[3:0] bits (Exit self refresh)  *)
  FMC_SDTR2_TXSR_0 = $00000010;  (*!<Bit 0  *)
  FMC_SDTR2_TXSR_1 = $00000020;  (*!<Bit 1  *)
  FMC_SDTR2_TXSR_2 = $00000040;  (*!<Bit 2  *)
  FMC_SDTR2_TXSR_3 = $00000080;  (*!<Bit 3  *)
  FMC_SDTR2_TRAS = $00000F00;  (*!<TRAS[3:0] bits (Self refresh time)  *)
  FMC_SDTR2_TRAS_0 = $00000100;  (*!<Bit 0  *)
  FMC_SDTR2_TRAS_1 = $00000200;  (*!<Bit 1  *)
  FMC_SDTR2_TRAS_2 = $00000400;  (*!<Bit 2  *)
  FMC_SDTR2_TRAS_3 = $00000800;  (*!<Bit 3  *)
  FMC_SDTR2_TRC = $0000F000;  (*!<TRC[2:0] bits (Row cycle delay)  *)
  FMC_SDTR2_TRC_0 = $00001000;  (*!<Bit 0  *)
  FMC_SDTR2_TRC_1 = $00002000;  (*!<Bit 1  *)
  FMC_SDTR2_TRC_2 = $00004000;  (*!<Bit 2  *)
  FMC_SDTR2_TWR = $000F0000;  (*!<TRC[2:0] bits (Write recovery delay)  *)
  FMC_SDTR2_TWR_0 = $00010000;  (*!<Bit 0  *)
  FMC_SDTR2_TWR_1 = $00020000;  (*!<Bit 1  *)
  FMC_SDTR2_TWR_2 = $00040000;  (*!<Bit 2  *)
  FMC_SDTR2_TRP = $00F00000;  (*!<TRP[2:0] bits (Row precharge delay)  *)
  FMC_SDTR2_TRP_0 = $00100000;  (*!<Bit 0  *)
  FMC_SDTR2_TRP_1 = $00200000;  (*!<Bit 1  *)
  FMC_SDTR2_TRP_2 = $00400000;  (*!<Bit 2  *)
  FMC_SDTR2_TRCD = $0F000000;  (*!<TRP[2:0] bits (Row to column delay)  *)
  FMC_SDTR2_TRCD_0 = $01000000;  (*!<Bit 0  *)
  FMC_SDTR2_TRCD_1 = $02000000;  (*!<Bit 1  *)
  FMC_SDTR2_TRCD_2 = $04000000;  (*!<Bit 2  *)
  (******************  Bit definition for FMC_SDCMR register  ***************** *)

  FMC_SDCMR_MODE = $00000007;  (*!<MODE[2:0] bits (Command mode)  *)
  FMC_SDCMR_MODE_0 = $00000001;  (*!<Bit 0  *)
  FMC_SDCMR_MODE_1 = $00000002;  (*!<Bit 1  *)
  FMC_SDCMR_MODE_2 = $00000003;  (*!<Bit 2  *)
  FMC_SDCMR_CTB2 = $00000008;  (*!<Command target 2  *)
  FMC_SDCMR_CTB1 = $00000010;  (*!<Command target 1  *)
  FMC_SDCMR_NRFS = $000001E0;  (*!<NRFS[3:0] bits (Number of auto-refresh)  *)
  FMC_SDCMR_NRFS_0 = $00000020;  (*!<Bit 0  *)
  FMC_SDCMR_NRFS_1 = $00000040;  (*!<Bit 1  *)
  FMC_SDCMR_NRFS_2 = $00000080;  (*!<Bit 2  *)
  FMC_SDCMR_NRFS_3 = $00000100;  (*!<Bit 3  *)
  FMC_SDCMR_MRD = $003FFE00;  (*!<MRD[12:0] bits (Mode register definition)  *)
  (******************  Bit definition for FMC_SDRTR register  ***************** *)

  FMC_SDRTR_CRE = $00000001;  (*!<Clear refresh error flag  *)
  FMC_SDRTR_COUNT = $00003FFE;  (*!<COUNT[12:0] bits (Refresh timer count)  *)
  FMC_SDRTR_REIE = $00004000;  (*!<RES interupt enable  *)
  (******************  Bit definition for FMC_SDSR register  ***************** *)

  FMC_SDSR_RE = $00000001;  (*!<Refresh error flag  *)
  FMC_SDSR_MODES1 = $00000006;  (*!<MODES1[1:0]bits (Status mode for bank 1)  *)
  FMC_SDSR_MODES1_0 = $00000002;  (*!<Bit 0  *)
  FMC_SDSR_MODES1_1 = $00000004;  (*!<Bit 1  *)
  FMC_SDSR_MODES2 = $00000018;  (*!<MODES2[1:0]bits (Status mode for bank 2)  *)
  FMC_SDSR_MODES2_0 = $00000008;  (*!<Bit 0  *)
  FMC_SDSR_MODES2_1 = $00000010;  (*!<Bit 1  *)
  FMC_SDSR_BUSY = $00000020;  (*!<Busy status  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                            General Purpose I/O                              *)

  (*                                                                             *)

  (***************************************************************************** *)

  (******************  Bits definition for GPIO_MODER register  **************** *)

  GPIO_MODER_MODER0 = $00000003;
  GPIO_MODER_MODER0_0 = $00000001;
  GPIO_MODER_MODER0_1 = $00000002;
  GPIO_MODER_MODER1 = $0000000C;
  GPIO_MODER_MODER1_0 = $00000004;
  GPIO_MODER_MODER1_1 = $00000008;
  GPIO_MODER_MODER2 = $00000030;
  GPIO_MODER_MODER2_0 = $00000010;
  GPIO_MODER_MODER2_1 = $00000020;
  GPIO_MODER_MODER3 = $000000C0;
  GPIO_MODER_MODER3_0 = $00000040;
  GPIO_MODER_MODER3_1 = $00000080;
  GPIO_MODER_MODER4 = $00000300;
  GPIO_MODER_MODER4_0 = $00000100;
  GPIO_MODER_MODER4_1 = $00000200;
  GPIO_MODER_MODER5 = $00000C00;
  GPIO_MODER_MODER5_0 = $00000400;
  GPIO_MODER_MODER5_1 = $00000800;
  GPIO_MODER_MODER6 = $00003000;
  GPIO_MODER_MODER6_0 = $00001000;
  GPIO_MODER_MODER6_1 = $00002000;
  GPIO_MODER_MODER7 = $0000C000;
  GPIO_MODER_MODER7_0 = $00004000;
  GPIO_MODER_MODER7_1 = $00008000;
  GPIO_MODER_MODER8 = $00030000;
  GPIO_MODER_MODER8_0 = $00010000;
  GPIO_MODER_MODER8_1 = $00020000;
  GPIO_MODER_MODER9 = $000C0000;
  GPIO_MODER_MODER9_0 = $00040000;
  GPIO_MODER_MODER9_1 = $00080000;
  GPIO_MODER_MODER10 = $00300000;
  GPIO_MODER_MODER10_0 = $00100000;
  GPIO_MODER_MODER10_1 = $00200000;
  GPIO_MODER_MODER11 = $00C00000;
  GPIO_MODER_MODER11_0 = $00400000;
  GPIO_MODER_MODER11_1 = $00800000;
  GPIO_MODER_MODER12 = $03000000;
  GPIO_MODER_MODER12_0 = $01000000;
  GPIO_MODER_MODER12_1 = $02000000;
  GPIO_MODER_MODER13 = $0C000000;
  GPIO_MODER_MODER13_0 = $04000000;
  GPIO_MODER_MODER13_1 = $08000000;
  GPIO_MODER_MODER14 = $30000000;
  GPIO_MODER_MODER14_0 = $10000000;
  GPIO_MODER_MODER14_1 = $20000000;
  GPIO_MODER_MODER15 = $C0000000;
  GPIO_MODER_MODER15_0 = $40000000;
  GPIO_MODER_MODER15_1 = $80000000;
  (******************  Bits definition for GPIO_OTYPER register  *************** *)

  GPIO_OTYPER_OT_0 = $00000001;
  GPIO_OTYPER_OT_1 = $00000002;
  GPIO_OTYPER_OT_2 = $00000004;
  GPIO_OTYPER_OT_3 = $00000008;
  GPIO_OTYPER_OT_4 = $00000010;
  GPIO_OTYPER_OT_5 = $00000020;
  GPIO_OTYPER_OT_6 = $00000040;
  GPIO_OTYPER_OT_7 = $00000080;
  GPIO_OTYPER_OT_8 = $00000100;
  GPIO_OTYPER_OT_9 = $00000200;
  GPIO_OTYPER_OT_10 = $00000400;
  GPIO_OTYPER_OT_11 = $00000800;
  GPIO_OTYPER_OT_12 = $00001000;
  GPIO_OTYPER_OT_13 = $00002000;
  GPIO_OTYPER_OT_14 = $00004000;
  GPIO_OTYPER_OT_15 = $00008000;
  (******************  Bits definition for GPIO_OSPEEDR register  ************** *)

  GPIO_OSPEEDER_OSPEEDR0 = $00000003;
  GPIO_OSPEEDER_OSPEEDR0_0 = $00000001;
  GPIO_OSPEEDER_OSPEEDR0_1 = $00000002;
  GPIO_OSPEEDER_OSPEEDR1 = $0000000C;
  GPIO_OSPEEDER_OSPEEDR1_0 = $00000004;
  GPIO_OSPEEDER_OSPEEDR1_1 = $00000008;
  GPIO_OSPEEDER_OSPEEDR2 = $00000030;
  GPIO_OSPEEDER_OSPEEDR2_0 = $00000010;
  GPIO_OSPEEDER_OSPEEDR2_1 = $00000020;
  GPIO_OSPEEDER_OSPEEDR3 = $000000C0;
  GPIO_OSPEEDER_OSPEEDR3_0 = $00000040;
  GPIO_OSPEEDER_OSPEEDR3_1 = $00000080;
  GPIO_OSPEEDER_OSPEEDR4 = $00000300;
  GPIO_OSPEEDER_OSPEEDR4_0 = $00000100;
  GPIO_OSPEEDER_OSPEEDR4_1 = $00000200;
  GPIO_OSPEEDER_OSPEEDR5 = $00000C00;
  GPIO_OSPEEDER_OSPEEDR5_0 = $00000400;
  GPIO_OSPEEDER_OSPEEDR5_1 = $00000800;
  GPIO_OSPEEDER_OSPEEDR6 = $00003000;
  GPIO_OSPEEDER_OSPEEDR6_0 = $00001000;
  GPIO_OSPEEDER_OSPEEDR6_1 = $00002000;
  GPIO_OSPEEDER_OSPEEDR7 = $0000C000;
  GPIO_OSPEEDER_OSPEEDR7_0 = $00004000;
  GPIO_OSPEEDER_OSPEEDR7_1 = $00008000;
  GPIO_OSPEEDER_OSPEEDR8 = $00030000;
  GPIO_OSPEEDER_OSPEEDR8_0 = $00010000;
  GPIO_OSPEEDER_OSPEEDR8_1 = $00020000;
  GPIO_OSPEEDER_OSPEEDR9 = $000C0000;
  GPIO_OSPEEDER_OSPEEDR9_0 = $00040000;
  GPIO_OSPEEDER_OSPEEDR9_1 = $00080000;
  GPIO_OSPEEDER_OSPEEDR10 = $00300000;
  GPIO_OSPEEDER_OSPEEDR10_0 = $00100000;
  GPIO_OSPEEDER_OSPEEDR10_1 = $00200000;
  GPIO_OSPEEDER_OSPEEDR11 = $00C00000;
  GPIO_OSPEEDER_OSPEEDR11_0 = $00400000;
  GPIO_OSPEEDER_OSPEEDR11_1 = $00800000;
  GPIO_OSPEEDER_OSPEEDR12 = $03000000;
  GPIO_OSPEEDER_OSPEEDR12_0 = $01000000;
  GPIO_OSPEEDER_OSPEEDR12_1 = $02000000;
  GPIO_OSPEEDER_OSPEEDR13 = $0C000000;
  GPIO_OSPEEDER_OSPEEDR13_0 = $04000000;
  GPIO_OSPEEDER_OSPEEDR13_1 = $08000000;
  GPIO_OSPEEDER_OSPEEDR14 = $30000000;
  GPIO_OSPEEDER_OSPEEDR14_0 = $10000000;
  GPIO_OSPEEDER_OSPEEDR14_1 = $20000000;
  GPIO_OSPEEDER_OSPEEDR15 = $C0000000;
  GPIO_OSPEEDER_OSPEEDR15_0 = $40000000;
  GPIO_OSPEEDER_OSPEEDR15_1 = $80000000;
  (******************  Bits definition for GPIO_PUPDR register  **************** *)

  GPIO_PUPDR_PUPDR0 = $00000003;
  GPIO_PUPDR_PUPDR0_0 = $00000001;
  GPIO_PUPDR_PUPDR0_1 = $00000002;
  GPIO_PUPDR_PUPDR1 = $0000000C;
  GPIO_PUPDR_PUPDR1_0 = $00000004;
  GPIO_PUPDR_PUPDR1_1 = $00000008;
  GPIO_PUPDR_PUPDR2 = $00000030;
  GPIO_PUPDR_PUPDR2_0 = $00000010;
  GPIO_PUPDR_PUPDR2_1 = $00000020;
  GPIO_PUPDR_PUPDR3 = $000000C0;
  GPIO_PUPDR_PUPDR3_0 = $00000040;
  GPIO_PUPDR_PUPDR3_1 = $00000080;
  GPIO_PUPDR_PUPDR4 = $00000300;
  GPIO_PUPDR_PUPDR4_0 = $00000100;
  GPIO_PUPDR_PUPDR4_1 = $00000200;
  GPIO_PUPDR_PUPDR5 = $00000C00;
  GPIO_PUPDR_PUPDR5_0 = $00000400;
  GPIO_PUPDR_PUPDR5_1 = $00000800;
  GPIO_PUPDR_PUPDR6 = $00003000;
  GPIO_PUPDR_PUPDR6_0 = $00001000;
  GPIO_PUPDR_PUPDR6_1 = $00002000;
  GPIO_PUPDR_PUPDR7 = $0000C000;
  GPIO_PUPDR_PUPDR7_0 = $00004000;
  GPIO_PUPDR_PUPDR7_1 = $00008000;
  GPIO_PUPDR_PUPDR8 = $00030000;
  GPIO_PUPDR_PUPDR8_0 = $00010000;
  GPIO_PUPDR_PUPDR8_1 = $00020000;
  GPIO_PUPDR_PUPDR9 = $000C0000;
  GPIO_PUPDR_PUPDR9_0 = $00040000;
  GPIO_PUPDR_PUPDR9_1 = $00080000;
  GPIO_PUPDR_PUPDR10 = $00300000;
  GPIO_PUPDR_PUPDR10_0 = $00100000;
  GPIO_PUPDR_PUPDR10_1 = $00200000;
  GPIO_PUPDR_PUPDR11 = $00C00000;
  GPIO_PUPDR_PUPDR11_0 = $00400000;
  GPIO_PUPDR_PUPDR11_1 = $00800000;
  GPIO_PUPDR_PUPDR12 = $03000000;
  GPIO_PUPDR_PUPDR12_0 = $01000000;
  GPIO_PUPDR_PUPDR12_1 = $02000000;
  GPIO_PUPDR_PUPDR13 = $0C000000;
  GPIO_PUPDR_PUPDR13_0 = $04000000;
  GPIO_PUPDR_PUPDR13_1 = $08000000;
  GPIO_PUPDR_PUPDR14 = $30000000;
  GPIO_PUPDR_PUPDR14_0 = $10000000;
  GPIO_PUPDR_PUPDR14_1 = $20000000;
  GPIO_PUPDR_PUPDR15 = $C0000000;
  GPIO_PUPDR_PUPDR15_0 = $40000000;
  GPIO_PUPDR_PUPDR15_1 = $80000000;
  (******************  Bits definition for GPIO_IDR register  ****************** *)

  GPIO_IDR_IDR_0 = $00000001;
  GPIO_IDR_IDR_1 = $00000002;
  GPIO_IDR_IDR_2 = $00000004;
  GPIO_IDR_IDR_3 = $00000008;
  GPIO_IDR_IDR_4 = $00000010;
  GPIO_IDR_IDR_5 = $00000020;
  GPIO_IDR_IDR_6 = $00000040;
  GPIO_IDR_IDR_7 = $00000080;
  GPIO_IDR_IDR_8 = $00000100;
  GPIO_IDR_IDR_9 = $00000200;
  GPIO_IDR_IDR_10 = $00000400;
  GPIO_IDR_IDR_11 = $00000800;
  GPIO_IDR_IDR_12 = $00001000;
  GPIO_IDR_IDR_13 = $00002000;
  GPIO_IDR_IDR_14 = $00004000;
  GPIO_IDR_IDR_15 = $00008000;
  (******************  Bits definition for GPIO_ODR register  ****************** *)

  GPIO_ODR_ODR_0 = $00000001;
  GPIO_ODR_ODR_1 = $00000002;
  GPIO_ODR_ODR_2 = $00000004;
  GPIO_ODR_ODR_3 = $00000008;
  GPIO_ODR_ODR_4 = $00000010;
  GPIO_ODR_ODR_5 = $00000020;
  GPIO_ODR_ODR_6 = $00000040;
  GPIO_ODR_ODR_7 = $00000080;
  GPIO_ODR_ODR_8 = $00000100;
  GPIO_ODR_ODR_9 = $00000200;
  GPIO_ODR_ODR_10 = $00000400;
  GPIO_ODR_ODR_11 = $00000800;
  GPIO_ODR_ODR_12 = $00001000;
  GPIO_ODR_ODR_13 = $00002000;
  GPIO_ODR_ODR_14 = $00004000;
  GPIO_ODR_ODR_15 = $00008000;
  (******************  Bits definition for GPIO_BSRR register  ***************** *)

  GPIO_BSRR_BS_0 = $00000001;
  GPIO_BSRR_BS_1 = $00000002;
  GPIO_BSRR_BS_2 = $00000004;
  GPIO_BSRR_BS_3 = $00000008;
  GPIO_BSRR_BS_4 = $00000010;
  GPIO_BSRR_BS_5 = $00000020;
  GPIO_BSRR_BS_6 = $00000040;
  GPIO_BSRR_BS_7 = $00000080;
  GPIO_BSRR_BS_8 = $00000100;
  GPIO_BSRR_BS_9 = $00000200;
  GPIO_BSRR_BS_10 = $00000400;
  GPIO_BSRR_BS_11 = $00000800;
  GPIO_BSRR_BS_12 = $00001000;
  GPIO_BSRR_BS_13 = $00002000;
  GPIO_BSRR_BS_14 = $00004000;
  GPIO_BSRR_BS_15 = $00008000;
  GPIO_BSRR_BR_0 = $00010000;
  GPIO_BSRR_BR_1 = $00020000;
  GPIO_BSRR_BR_2 = $00040000;
  GPIO_BSRR_BR_3 = $00080000;
  GPIO_BSRR_BR_4 = $00100000;
  GPIO_BSRR_BR_5 = $00200000;
  GPIO_BSRR_BR_6 = $00400000;
  GPIO_BSRR_BR_7 = $00800000;
  GPIO_BSRR_BR_8 = $01000000;
  GPIO_BSRR_BR_9 = $02000000;
  GPIO_BSRR_BR_10 = $04000000;
  GPIO_BSRR_BR_11 = $08000000;
  GPIO_BSRR_BR_12 = $10000000;
  GPIO_BSRR_BR_13 = $20000000;
  GPIO_BSRR_BR_14 = $40000000;
  GPIO_BSRR_BR_15 = $80000000;
  (****************** Bit definition for GPIO_LCKR register ******************** *)

  GPIO_LCKR_LCK0 = $00000001;
  GPIO_LCKR_LCK1 = $00000002;
  GPIO_LCKR_LCK2 = $00000004;
  GPIO_LCKR_LCK3 = $00000008;
  GPIO_LCKR_LCK4 = $00000010;
  GPIO_LCKR_LCK5 = $00000020;
  GPIO_LCKR_LCK6 = $00000040;
  GPIO_LCKR_LCK7 = $00000080;
  GPIO_LCKR_LCK8 = $00000100;
  GPIO_LCKR_LCK9 = $00000200;
  GPIO_LCKR_LCK10 = $00000400;
  GPIO_LCKR_LCK11 = $00000800;
  GPIO_LCKR_LCK12 = $00001000;
  GPIO_LCKR_LCK13 = $00002000;
  GPIO_LCKR_LCK14 = $00004000;
  GPIO_LCKR_LCK15 = $00008000;
  GPIO_LCKR_LCKK = $00010000;
  (***************************************************************************** *)

  (*                                                                             *)

  (*                      Inter-integrated Circuit Interface (I2C)               *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*******************  Bit definition for I2C_CR1 register  ****************** *)

  I2C_CR1_PE = $00000001;  (*!< Peripheral enable                    *)
  I2C_CR1_TXIE = $00000002;  (*!< TX interrupt enable                  *)
  I2C_CR1_RXIE = $00000004;  (*!< RX interrupt enable                  *)
  I2C_CR1_ADDRIE = $00000008;  (*!< Address match interrupt enable       *)
  I2C_CR1_NACKIE = $00000010;  (*!< NACK received interrupt enable       *)
  I2C_CR1_STOPIE = $00000020;  (*!< STOP detection interrupt enable      *)
  I2C_CR1_TCIE = $00000040;  (*!< Transfer complete interrupt enable   *)
  I2C_CR1_ERRIE = $00000080;  (*!< Errors interrupt enable              *)
  I2C_CR1_DFN = $00000F00;  (*!< Digital noise filter                 *)
  I2C_CR1_ANFOFF = $00001000;  (*!< Analog noise filter OFF              *)
  I2C_CR1_SWRST = $00002000;  (*!< Software reset                       *)
  I2C_CR1_TXDMAEN = $00004000;  (*!< DMA transmission requests enable     *)
  I2C_CR1_RXDMAEN = $00008000;  (*!< DMA reception requests enable        *)
  I2C_CR1_SBC = $00010000;  (*!< Slave byte control                   *)
  I2C_CR1_NOSTRETCH = $00020000;  (*!< Clock stretching disable             *)
  I2C_CR1_WUPEN = $00040000;  (*!< Wakeup from STOP enable              *)
  I2C_CR1_GCEN = $00080000;  (*!< General call enable                  *)
  I2C_CR1_SMBHEN = $00100000;  (*!< SMBus host address enable            *)
  I2C_CR1_SMBDEN = $00200000;  (*!< SMBus device default address enable  *)
  I2C_CR1_ALERTEN = $00400000;  (*!< SMBus alert enable                   *)
  I2C_CR1_PECEN = $00800000;  (*!< PEC enable                           *)
  (******************  Bit definition for I2C_CR2 register  ******************* *)

  I2C_CR2_SADD = $000003FF;  (*!< Slave address (master mode)                              *)
  I2C_CR2_RD_WRN = $00000400;  (*!< Transfer direction (master mode)                         *)
  I2C_CR2_ADD10 = $00000800;  (*!< 10-bit addressing mode (master mode)                     *)
  I2C_CR2_HEAD10R = $00001000;  (*!< 10-bit address header only read direction (master mode)  *)
  I2C_CR2_START = $00002000;  (*!< START generation                                         *)
  I2C_CR2_STOP = $00004000;  (*!< STOP generation (master mode)                            *)
  I2C_CR2_NACK = $00008000;  (*!< NACK generation (slave mode)                             *)
  I2C_CR2_NBYTES = $00FF0000;  (*!< Number of bytes                                          *)
  I2C_CR2_RELOAD = $01000000;  (*!< NBYTES reload mode                                       *)
  I2C_CR2_AUTOEND = $02000000;  (*!< Automatic end mode (master mode)                         *)
  I2C_CR2_PECBYTE = $04000000;  (*!< Packet error checking byte                               *)
  (*******************  Bit definition for I2C_OAR1 register  ***************** *)

  I2C_OAR1_OA1 = $000003FF;  (*!< Interface own address 1    *)
  I2C_OAR1_OA1MODE = $00000400;  (*!< Own address 1 10-bit mode  *)
  I2C_OAR1_OA1EN = $00008000;  (*!< Own address 1 enable       *)
  (*******************  Bit definition for I2C_OAR2 register  ***************** *)

  I2C_OAR2_OA2 = $000000FE;  (*!< Interface own address 2  *)
  I2C_OAR2_OA2MSK = $00000700;  (*!< Own address 2 masks      *)
  I2C_OAR2_OA2NOMASK = $00000000;  (*!< No mask  *)
  I2C_OAR2_OA2MASK01 = $00000100;  (*!< OA2[1] is masked, Only OA2[7:2] are compared  *)
  I2C_OAR2_OA2MASK02 = $00000200;  (*!< OA2[2:1] is masked, Only OA2[7:3] are compared  *)
  I2C_OAR2_OA2MASK03 = $00000300;  (*!< OA2[3:1] is masked, Only OA2[7:4] are compared  *)
  I2C_OAR2_OA2MASK04 = $00000400;  (*!< OA2[4:1] is masked, Only OA2[7:5] are compared  *)
  I2C_OAR2_OA2MASK05 = $00000500;  (*!< OA2[5:1] is masked, Only OA2[7:6] are compared  *)
  I2C_OAR2_OA2MASK06 = $00000600;  (*!< OA2[6:1] is masked, Only OA2[7] are compared  *)
  I2C_OAR2_OA2MASK07 = $00000700;  (*!< OA2[7:1] is masked, No comparison is done  *)
  I2C_OAR2_OA2EN = $00008000;  (*!< Own address 2 enable     *)
  (*******************  Bit definition for I2C_TIMINGR register ****************** *)

  I2C_TIMINGR_SCLL = $000000FF;  (*!< SCL low period (master mode)   *)
  I2C_TIMINGR_SCLH = $0000FF00;  (*!< SCL high period (master mode)  *)
  I2C_TIMINGR_SDADEL = $000F0000;  (*!< Data hold time                 *)
  I2C_TIMINGR_SCLDEL = $00F00000;  (*!< Data setup time                *)
  I2C_TIMINGR_PRESC = $F0000000;  (*!< Timings prescaler              *)
  (******************* Bit definition for I2C_TIMEOUTR register ****************** *)

  I2C_TIMEOUTR_TIMEOUTA = $00000FFF;  (*!< Bus timeout A                  *)
  I2C_TIMEOUTR_TIDLE = $00001000;  (*!< Idle clock timeout detection   *)
  I2C_TIMEOUTR_TIMOUTEN = $00008000;  (*!< Clock timeout enable           *)
  I2C_TIMEOUTR_TIMEOUTB = $0FFF0000;  (*!< Bus timeout B                  *)
  I2C_TIMEOUTR_TEXTEN = $80000000;  (*!< Extended clock timeout enable  *)
  (******************  Bit definition for I2C_ISR register  ******************** *)

  I2C_ISR_TXE = $00000001;  (*!< Transmit data register empty     *)
  I2C_ISR_TXIS = $00000002;  (*!< Transmit interrupt status        *)
  I2C_ISR_RXNE = $00000004;  (*!< Receive data register not empty  *)
  I2C_ISR_ADDR = $00000008;  (*!< Address matched (slave mode)     *)
  I2C_ISR_NACKF = $00000010;  (*!< NACK received flag               *)
  I2C_ISR_STOPF = $00000020;  (*!< STOP detection flag              *)
  I2C_ISR_TC = $00000040;  (*!< Transfer complete (master mode)  *)
  I2C_ISR_TCR = $00000080;  (*!< Transfer complete reload         *)
  I2C_ISR_BERR = $00000100;  (*!< Bus error                        *)
  I2C_ISR_ARLO = $00000200;  (*!< Arbitration lost                 *)
  I2C_ISR_OVR = $00000400;  (*!< Overrun/Underrun                 *)
  I2C_ISR_PECERR = $00000800;  (*!< PEC error in reception           *)
  I2C_ISR_TIMEOUT = $00001000;  (*!< Timeout or Tlow detection flag   *)
  I2C_ISR_ALERT = $00002000;  (*!< SMBus alert                      *)
  I2C_ISR_BUSY = $00008000;  (*!< Bus busy                         *)
  I2C_ISR_DIR = $00010000;  (*!< Transfer direction (slave mode)  *)
  I2C_ISR_ADDCODE = $00FE0000;  (*!< Address match code (slave mode)  *)
  (******************  Bit definition for I2C_ICR register  ******************** *)

  I2C_ICR_ADDRCF = $00000008;  (*!< Address matched clear flag       *)
  I2C_ICR_NACKCF = $00000010;  (*!< NACK clear flag                  *)
  I2C_ICR_STOPCF = $00000020;  (*!< STOP detection clear flag        *)
  I2C_ICR_BERRCF = $00000100;  (*!< Bus error clear flag             *)
  I2C_ICR_ARLOCF = $00000200;  (*!< Arbitration lost clear flag      *)
  I2C_ICR_OVRCF = $00000400;  (*!< Overrun/Underrun clear flag      *)
  I2C_ICR_PECCF = $00000800;  (*!< PAC error clear flag             *)
  I2C_ICR_TIMOUTCF = $00001000;  (*!< Timeout clear flag               *)
  I2C_ICR_ALERTCF = $00002000;  (*!< Alert clear flag                 *)
  (******************  Bit definition for I2C_PECR register  ******************** *)

  I2C_PECR_PEC = $000000FF;  (*!< PEC register         *)
  (******************  Bit definition for I2C_RXDR register  ******************** *)

  I2C_RXDR_RXDATA = $000000FF;  (*!< 8-bit receive data   *)
  (******************  Bit definition for I2C_TXDR register  ******************** *)

  I2C_TXDR_TXDATA = $000000FF;  (*!< 8-bit transmit data  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                           Independent WATCHDOG                              *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*******************  Bit definition for IWDG_KR register  ******************* *)

  IWDG_KR_KEY = $FFFF;  (*!<Key value (write only, read 0000h)   *)
  (*******************  Bit definition for IWDG_PR register  ******************* *)

  IWDG_PR_PR = $07;  (*!<PR[2:0] (Prescaler divider)          *)
  IWDG_PR_PR_0 = $01;  (*!<Bit 0  *)
  IWDG_PR_PR_1 = $02;  (*!<Bit 1  *)
  IWDG_PR_PR_2 = $04;  (*!<Bit 2  *)
  (*******************  Bit definition for IWDG_RLR register  ****************** *)

  IWDG_RLR_RL = $0FFF;  (*!<Watchdog counter reload value         *)
  (*******************  Bit definition for IWDG_SR register  ******************* *)

  IWDG_SR_PVU = $01;  (*!< Watchdog prescaler value update  *)
  IWDG_SR_RVU = $02;  (*!< Watchdog counter reload value update  *)
  IWDG_SR_WVU = $04;  (*!< Watchdog counter window value update  *)
  (*******************  Bit definition for IWDG_KR register  ******************* *)

  IWDG_WINR_WIN = $0FFF;  (*!< Watchdog counter window value  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                             Power Control                                   *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for PWR_CR1 register  ******************* *)

  PWR_CR1_LPDS = $00000001;  (*!< Low-Power Deepsleep                  *)
  PWR_CR1_PDDS = $00000002;  (*!< Power Down Deepsleep                 *)
  PWR_CR1_CSBF = $00000008;  (*!< Clear Standby Flag                   *)
  PWR_CR1_PVDE = $00000010;  (*!< Power Voltage Detector Enable        *)
  PWR_CR1_PLS = $000000E0;  (*!< PLS[2:0] bits (PVD Level Selection)  *)
  PWR_CR1_PLS_0 = $00000020;  (*!< Bit 0  *)
  PWR_CR1_PLS_1 = $00000040;  (*!< Bit 1  *)
  PWR_CR1_PLS_2 = $00000080;  (*!< Bit 2  *)
  (*!< PVD level configuration  *)

  PWR_CR1_PLS_LEV0 = $00000000;  (*!< PVD level 0  *)
  PWR_CR1_PLS_LEV1 = $00000020;  (*!< PVD level 1  *)
  PWR_CR1_PLS_LEV2 = $00000040;  (*!< PVD level 2  *)
  PWR_CR1_PLS_LEV3 = $00000060;  (*!< PVD level 3  *)
  PWR_CR1_PLS_LEV4 = $00000080;  (*!< PVD level 4  *)
  PWR_CR1_PLS_LEV5 = $000000A0;  (*!< PVD level 5  *)
  PWR_CR1_PLS_LEV6 = $000000C0;  (*!< PVD level 6  *)
  PWR_CR1_PLS_LEV7 = $000000E0;  (*!< PVD level 7  *)
  PWR_CR1_DBP = $00000100;  (*!< Disable Backup Domain write protection                      *)
  PWR_CR1_FPDS = $00000200;  (*!< Flash power down in Stop mode                               *)
  PWR_CR1_LPUDS = $00000400;  (*!< Low-power regulator in deepsleep under-drive mode           *)
  PWR_CR1_MRUDS = $00000800;  (*!< Main regulator in deepsleep under-drive mode                *)
  PWR_CR1_ADCDC1 = $00002000;  (*!< Refer to AN4073 on how to use this bit  *)
  PWR_CR1_VOS = $0000C000;  (*!< VOS[1:0] bits (Regulator voltage scaling output selection)  *)
  PWR_CR1_VOS_0 = $00004000;  (*!< Bit 0  *)
  PWR_CR1_VOS_1 = $00008000;  (*!< Bit 1  *)
  PWR_CR1_ODEN = $00010000;  (*!< Over Drive enable                    *)
  PWR_CR1_ODSWEN = $00020000;  (*!< Over Drive switch enabled            *)
  PWR_CR1_UDEN = $000C0000;  (*!< Under Drive enable in stop mode      *)
  PWR_CR1_UDEN_0 = $00040000;  (*!< Bit 0                                *)
  PWR_CR1_UDEN_1 = $00080000;  (*!< Bit 1                                *)
  (*******************  Bit definition for PWR_CSR1 register  ******************* *)

  PWR_CSR1_WUIF = $00000001;  (*!< Wake up internal Flag                             *)
  PWR_CSR1_SBF = $00000002;  (*!< Standby Flag                                      *)
  PWR_CSR1_PVDO = $00000004;  (*!< PVD Output                                        *)
  PWR_CSR1_BRR = $00000008;  (*!< Backup regulator ready                            *)
  PWR_CSR1_BRE = $00000200;  (*!< Backup regulator enable                           *)
  PWR_CSR1_VOSRDY = $00004000;  (*!< Regulator voltage scaling output selection ready  *)
  PWR_CSR1_ODRDY = $00010000;  (*!< Over Drive generator ready                        *)
  PWR_CSR1_ODSWRDY = $00020000;  (*!< Over Drive Switch ready                           *)
  PWR_CSR1_UDSWRDY = $000C0000;  (*!< Under Drive ready                                 *)
  (********************  Bit definition for PWR_CR2 register  ******************* *)

  PWR_CR2_CWUPF1 = $00000001;  (*!< Clear Wakeup Pin Flag for PA0       *)
  PWR_CR2_CWUPF2 = $00000002;  (*!< Clear Wakeup Pin Flag for PA2       *)
  PWR_CR2_CWUPF3 = $00000004;  (*!< Clear Wakeup Pin Flag for PC1       *)
  PWR_CR2_CWUPF4 = $00000008;  (*!< Clear Wakeup Pin Flag for PC13      *)
  PWR_CR2_CWUPF5 = $00000010;  (*!< Clear Wakeup Pin Flag for PI8       *)
  PWR_CR2_CWUPF6 = $00000020;  (*!< Clear Wakeup Pin Flag for PI11      *)
  PWR_CR2_WUPP1 = $00000100;  (*!< Wakeup Pin Polarity bit for PA0     *)
  PWR_CR2_WUPP2 = $00000200;  (*!< Wakeup Pin Polarity bit for PA2     *)
  PWR_CR2_WUPP3 = $00000400;  (*!< Wakeup Pin Polarity bit for PC1     *)
  PWR_CR2_WUPP4 = $00000800;  (*!< Wakeup Pin Polarity bit for PC13    *)
  PWR_CR2_WUPP5 = $00001000;  (*!< Wakeup Pin Polarity bit for PI8     *)
  PWR_CR2_WUPP6 = $00002000;  (*!< Wakeup Pin Polarity bit for PI11    *)
  (*******************  Bit definition for PWR_CSR2 register  ******************* *)

  PWR_CSR2_WUPF1 = $00000001;  (*!< Wakeup Pin Flag for PA0             *)
  PWR_CSR2_WUPF2 = $00000002;  (*!< Wakeup Pin Flag for PA2             *)
  PWR_CSR2_WUPF3 = $00000004;  (*!< Wakeup Pin Flag for PC1             *)
  PWR_CSR2_WUPF4 = $00000008;  (*!< Wakeup Pin Flag for PC13            *)
  PWR_CSR2_WUPF5 = $00000010;  (*!< Wakeup Pin Flag for PI8             *)
  PWR_CSR2_WUPF6 = $00000020;  (*!< Wakeup Pin Flag for PI11            *)
  PWR_CSR2_EWUP1 = $00000100;  (*!< Enable Wakeup Pin PA0               *)
  PWR_CSR2_EWUP2 = $00000200;  (*!< Enable Wakeup Pin PA2               *)
  PWR_CSR2_EWUP3 = $00000400;  (*!< Enable Wakeup Pin PC1               *)
  PWR_CSR2_EWUP4 = $00000800;  (*!< Enable Wakeup Pin PC13              *)
  PWR_CSR2_EWUP5 = $00001000;  (*!< Enable Wakeup Pin PI8               *)
  PWR_CSR2_EWUP6 = $00002000;  (*!< Enable Wakeup Pin PI11              *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                                    QUADSPI                                  *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*****************  Bit definition for QUADSPI_CR register  ****************** *)

  QUADSPI_CR_EN = $00000001;  (*!< Enable                             *)
  QUADSPI_CR_ABORT = $00000002;  (*!< Abort request                      *)
  QUADSPI_CR_DMAEN = $00000004;  (*!< DMA Enable                         *)
  QUADSPI_CR_TCEN = $00000008;  (*!< Timeout Counter Enable             *)
  QUADSPI_CR_SSHIFT = $00000010;  (*!< Sample Shift                       *)
  QUADSPI_CR_DFM = $00000040;  (*!< Dual Flash Mode                    *)
  QUADSPI_CR_FSEL = $00000080;  (*!< Flash Select                       *)
  QUADSPI_CR_FTHRES = $00000F00;  (*!< FTHRES[3:0] FIFO Level             *)
  QUADSPI_CR_FTHRES_0 = $00000100;  (*!< Bit 0  *)
  QUADSPI_CR_FTHRES_1 = $00000200;  (*!< Bit 1  *)
  QUADSPI_CR_FTHRES_2 = $00000400;  (*!< Bit 2  *)
  QUADSPI_CR_FTHRES_3 = $00000800;  (*!< Bit 3  *)
  QUADSPI_CR_TEIE = $00010000;  (*!< Transfer Error Interrupt Enable     *)
  QUADSPI_CR_TCIE = $00020000;  (*!< Transfer Complete Interrupt Enable  *)
  QUADSPI_CR_FTIE = $00040000;  (*!< FIFO Threshold Interrupt Enable     *)
  QUADSPI_CR_SMIE = $00080000;  (*!< Status Match Interrupt Enable       *)
  QUADSPI_CR_TOIE = $00100000;  (*!< TimeOut Interrupt Enable            *)
  QUADSPI_CR_APMS = $00400000;  (*!< Bit 1                               *)
  QUADSPI_CR_PMM = $00800000;  (*!< Polling Match Mode                  *)
  QUADSPI_CR_PRESCALER = $FF000000;  (*!< PRESCALER[7:0] Clock prescaler      *)
  QUADSPI_CR_PRESCALER_0 = $01000000;  (*!< Bit 0  *)
  QUADSPI_CR_PRESCALER_1 = $02000000;  (*!< Bit 1  *)
  QUADSPI_CR_PRESCALER_2 = $04000000;  (*!< Bit 2  *)
  QUADSPI_CR_PRESCALER_3 = $08000000;  (*!< Bit 3  *)
  QUADSPI_CR_PRESCALER_4 = $10000000;  (*!< Bit 4  *)
  QUADSPI_CR_PRESCALER_5 = $20000000;  (*!< Bit 5  *)
  QUADSPI_CR_PRESCALER_6 = $40000000;  (*!< Bit 6  *)
  QUADSPI_CR_PRESCALER_7 = $80000000;  (*!< Bit 7  *)
  (*****************  Bit definition for QUADSPI_DCR register  ***************** *)

  QUADSPI_DCR_CKMODE = $00000001;  (*!< Mode 0 / Mode 3                  *)
  QUADSPI_DCR_CSHT = $00000700;  (*!< CSHT[2:0]: ChipSelect High Time  *)
  QUADSPI_DCR_CSHT_0 = $00000100;  (*!< Bit 0  *)
  QUADSPI_DCR_CSHT_1 = $00000200;  (*!< Bit 1  *)
  QUADSPI_DCR_CSHT_2 = $00000400;  (*!< Bit 2  *)
  QUADSPI_DCR_FSIZE = $001F0000;  (*!< FSIZE[4:0]: Flash Size           *)
  QUADSPI_DCR_FSIZE_0 = $00010000;  (*!< Bit 0  *)
  QUADSPI_DCR_FSIZE_1 = $00020000;  (*!< Bit 1  *)
  QUADSPI_DCR_FSIZE_2 = $00040000;  (*!< Bit 2  *)
  QUADSPI_DCR_FSIZE_3 = $00080000;  (*!< Bit 3  *)
  QUADSPI_DCR_FSIZE_4 = $00100000;  (*!< Bit 4  *)
  (******************  Bit definition for QUADSPI_SR register  ****************** *)

  QUADSPI_SR_TEF = $00000001;  (*!< Transfer Error Flag     *)
  QUADSPI_SR_TCF = $00000002;  (*!< Transfer Complete Flag  *)
  QUADSPI_SR_FTF = $00000004;  (*!< FIFO Threshlod Flag     *)
  QUADSPI_SR_SMF = $00000008;  (*!< Status Match Flag       *)
  QUADSPI_SR_TOF = $00000010;  (*!< Timeout Flag            *)
  QUADSPI_SR_BUSY = $00000020;  (*!< Busy                    *)
  QUADSPI_SR_FLEVEL = $00001F00;  (*!< FIFO Threshlod Flag     *)
  QUADSPI_SR_FLEVEL_0 = $00000100;  (*!< Bit 0  *)
  QUADSPI_SR_FLEVEL_1 = $00000200;  (*!< Bit 1  *)
  QUADSPI_SR_FLEVEL_2 = $00000400;  (*!< Bit 2  *)
  QUADSPI_SR_FLEVEL_3 = $00000800;  (*!< Bit 3  *)
  QUADSPI_SR_FLEVEL_4 = $00001000;  (*!< Bit 4  *)
  (******************  Bit definition for QUADSPI_FCR register  ***************** *)

  QUADSPI_FCR_CTEF = $00000001;  (*!< Clear Transfer Error Flag     *)
  QUADSPI_FCR_CTCF = $00000002;  (*!< Clear Transfer Complete Flag  *)
  QUADSPI_FCR_CSMF = $00000008;  (*!< Clear Status Match Flag       *)
  QUADSPI_FCR_CTOF = $00000010;  (*!< Clear Timeout Flag            *)
  (******************  Bit definition for QUADSPI_DLR register  ***************** *)

  QUADSPI_DLR_DL = $FFFFFFFF;  (*!< DL[31:0]: Data Length  *)
  (******************  Bit definition for QUADSPI_CCR register  ***************** *)

  QUADSPI_CCR_INSTRUCTION = $000000FF;  (*!< INSTRUCTION[7:0]: Instruction     *)
  QUADSPI_CCR_INSTRUCTION_0 = $00000001;  (*!< Bit 0  *)
  QUADSPI_CCR_INSTRUCTION_1 = $00000002;  (*!< Bit 1  *)
  QUADSPI_CCR_INSTRUCTION_2 = $00000004;  (*!< Bit 2  *)
  QUADSPI_CCR_INSTRUCTION_3 = $00000008;  (*!< Bit 3  *)
  QUADSPI_CCR_INSTRUCTION_4 = $00000010;  (*!< Bit 4  *)
  QUADSPI_CCR_INSTRUCTION_5 = $00000020;  (*!< Bit 5  *)
  QUADSPI_CCR_INSTRUCTION_6 = $00000040;  (*!< Bit 6  *)
  QUADSPI_CCR_INSTRUCTION_7 = $00000080;  (*!< Bit 7  *)
  QUADSPI_CCR_IMODE = $00000300;  (*!< IMODE[1:0]: Instruction Mode       *)
  QUADSPI_CCR_IMODE_0 = $00000100;  (*!< Bit 0  *)
  QUADSPI_CCR_IMODE_1 = $00000200;  (*!< Bit 1  *)
  QUADSPI_CCR_ADMODE = $00000C00;  (*!< ADMODE[1:0]: Address Mode          *)
  QUADSPI_CCR_ADMODE_0 = $00000400;  (*!< Bit 0  *)
  QUADSPI_CCR_ADMODE_1 = $00000800;  (*!< Bit 1  *)
  QUADSPI_CCR_ADSIZE = $00003000;  (*!< ADSIZE[1:0]: Address Size          *)
  QUADSPI_CCR_ADSIZE_0 = $00001000;  (*!< Bit 0  *)
  QUADSPI_CCR_ADSIZE_1 = $00002000;  (*!< Bit 1  *)
  QUADSPI_CCR_ABMODE = $0000C000;  (*!< ABMODE[1:0]: Alternate Bytes Mode  *)
  QUADSPI_CCR_ABMODE_0 = $00004000;  (*!< Bit 0  *)
  QUADSPI_CCR_ABMODE_1 = $00008000;  (*!< Bit 1  *)
  QUADSPI_CCR_ABSIZE = $00030000;  (*!< ABSIZE[1:0]: Instruction Mode      *)
  QUADSPI_CCR_ABSIZE_0 = $00010000;  (*!< Bit 0  *)
  QUADSPI_CCR_ABSIZE_1 = $00020000;  (*!< Bit 1  *)
  QUADSPI_CCR_DCYC = $007C0000;  (*!< DCYC[4:0]: Dummy Cycles            *)
  QUADSPI_CCR_DCYC_0 = $00040000;  (*!< Bit 0  *)
  QUADSPI_CCR_DCYC_1 = $00080000;  (*!< Bit 1  *)
  QUADSPI_CCR_DCYC_2 = $00100000;  (*!< Bit 2  *)
  QUADSPI_CCR_DCYC_3 = $00200000;  (*!< Bit 3  *)
  QUADSPI_CCR_DCYC_4 = $00400000;  (*!< Bit 4  *)
  QUADSPI_CCR_DMODE = $03000000;  (*!< DMODE[1:0]: Data Mode               *)
  QUADSPI_CCR_DMODE_0 = $01000000;  (*!< Bit 0  *)
  QUADSPI_CCR_DMODE_1 = $02000000;  (*!< Bit 1  *)
  QUADSPI_CCR_FMODE = $0C000000;  (*!< FMODE[1:0]: Functional Mode         *)
  QUADSPI_CCR_FMODE_0 = $04000000;  (*!< Bit 0  *)
  QUADSPI_CCR_FMODE_1 = $08000000;  (*!< Bit 1  *)
  QUADSPI_CCR_SIOO = $10000000;  (*!< SIOO: Send Instruction Only Once Mode  *)
  QUADSPI_CCR_DHHC = $40000000;  (*!< DHHC: Delay Half Hclk Cycle            *)
  QUADSPI_CCR_DDRM = $80000000;  (*!< DDRM: Double Data Rate Mode            *)
  (******************  Bit definition for QUADSPI_AR register  ****************** *)

  QUADSPI_AR_ADDRESS = $FFFFFFFF;  (*!< ADDRESS[31:0]: Address  *)
  (******************  Bit definition for QUADSPI_ABR register  ***************** *)

  QUADSPI_ABR_ALTERNATE = $FFFFFFFF;  (*!< ALTERNATE[31:0]: Alternate Bytes  *)
  (******************  Bit definition for QUADSPI_DR register  ****************** *)

  QUADSPI_DR_DATA = $FFFFFFFF;  (*!< DATA[31:0]: Data  *)
  (******************  Bit definition for QUADSPI_PSMKR register  *************** *)

  QUADSPI_PSMKR_MASK = $FFFFFFFF;  (*!< MASK[31:0]: Status Mask  *)
  (******************  Bit definition for QUADSPI_PSMAR register  *************** *)

  QUADSPI_PSMAR_MATCH = $FFFFFFFF;  (*!< MATCH[31:0]: Status Match  *)
  (******************  Bit definition for QUADSPI_PIR register  **************** *)

  QUADSPI_PIR_INTERVAL = $0000FFFF;  (*!< INTERVAL[15:0]: Polling Interval  *)
  (******************  Bit definition for QUADSPI_LPTR register  **************** *)

  QUADSPI_LPTR_TIMEOUT = $0000FFFF;  (*!< TIMEOUT[15:0]: Timeout period  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                         Reset and Clock Control             *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for RCC_CR register  ******************* *)

  RCC_CR_HSION = $00000001;
  RCC_CR_HSIRDY = $00000002;
  RCC_CR_HSITRIM = $000000F8;
  RCC_CR_HSITRIM_0 = $00000008;  (*!<Bit 0  *)
  RCC_CR_HSITRIM_1 = $00000010;  (*!<Bit 1  *)
  RCC_CR_HSITRIM_2 = $00000020;  (*!<Bit 2  *)
  RCC_CR_HSITRIM_3 = $00000040;  (*!<Bit 3  *)
  RCC_CR_HSITRIM_4 = $00000080;  (*!<Bit 4  *)
  RCC_CR_HSICAL = $0000FF00;
  RCC_CR_HSICAL_0 = $00000100;  (*!<Bit 0  *)
  RCC_CR_HSICAL_1 = $00000200;  (*!<Bit 1  *)
  RCC_CR_HSICAL_2 = $00000400;  (*!<Bit 2  *)
  RCC_CR_HSICAL_3 = $00000800;  (*!<Bit 3  *)
  RCC_CR_HSICAL_4 = $00001000;  (*!<Bit 4  *)
  RCC_CR_HSICAL_5 = $00002000;  (*!<Bit 5  *)
  RCC_CR_HSICAL_6 = $00004000;  (*!<Bit 6  *)
  RCC_CR_HSICAL_7 = $00008000;  (*!<Bit 7  *)
  RCC_CR_HSEON = $00010000;
  RCC_CR_HSERDY = $00020000;
  RCC_CR_HSEBYP = $00040000;
  RCC_CR_CSSON = $00080000;
  RCC_CR_PLLON = $01000000;
  RCC_CR_PLLRDY = $02000000;
  RCC_CR_PLLI2SON = $04000000;
  RCC_CR_PLLI2SRDY = $08000000;
  RCC_CR_PLLSAION = $10000000;
  RCC_CR_PLLSAIRDY = $20000000;
  (********************  Bit definition for RCC_PLLCFGR register  ************** *)

  RCC_PLLCFGR_PLLM = $0000003F;
  RCC_PLLCFGR_PLLM_0 = $00000001;
  RCC_PLLCFGR_PLLM_1 = $00000002;
  RCC_PLLCFGR_PLLM_2 = $00000004;
  RCC_PLLCFGR_PLLM_3 = $00000008;
  RCC_PLLCFGR_PLLM_4 = $00000010;
  RCC_PLLCFGR_PLLM_5 = $00000020;
  RCC_PLLCFGR_PLLN = $00007FC0;
  RCC_PLLCFGR_PLLN_0 = $00000040;
  RCC_PLLCFGR_PLLN_1 = $00000080;
  RCC_PLLCFGR_PLLN_2 = $00000100;
  RCC_PLLCFGR_PLLN_3 = $00000200;
  RCC_PLLCFGR_PLLN_4 = $00000400;
  RCC_PLLCFGR_PLLN_5 = $00000800;
  RCC_PLLCFGR_PLLN_6 = $00001000;
  RCC_PLLCFGR_PLLN_7 = $00002000;
  RCC_PLLCFGR_PLLN_8 = $00004000;
  RCC_PLLCFGR_PLLP = $00030000;
  RCC_PLLCFGR_PLLP_0 = $00010000;
  RCC_PLLCFGR_PLLP_1 = $00020000;
  RCC_PLLCFGR_PLLSRC = $00400000;
  RCC_PLLCFGR_PLLSRC_HSE = $00400000;
  RCC_PLLCFGR_PLLSRC_HSI = $00000000;
  RCC_PLLCFGR_PLLQ = $0F000000;
  RCC_PLLCFGR_PLLQ_0 = $01000000;
  RCC_PLLCFGR_PLLQ_1 = $02000000;
  RCC_PLLCFGR_PLLQ_2 = $04000000;
  RCC_PLLCFGR_PLLQ_3 = $08000000;
  (********************  Bit definition for RCC_CFGR register  ***************** *)

  (*!< SW configuration  *)

  RCC_CFGR_SW = $00000003;  (*!< SW[1:0] bits (System clock Switch)  *)
  RCC_CFGR_SW_0 = $00000001;  (*!< Bit 0  *)
  RCC_CFGR_SW_1 = $00000002;  (*!< Bit 1  *)
  RCC_CFGR_SW_HSI = $00000000;  (*!< HSI selected as system clock  *)
  RCC_CFGR_SW_HSE = $00000001;  (*!< HSE selected as system clock  *)
  RCC_CFGR_SW_PLL = $00000002;  (*!< PLL selected as system clock  *)
  (*!< SWS configuration  *)

  RCC_CFGR_SWS = $0000000C;  (*!< SWS[1:0] bits (System Clock Switch Status)  *)
  RCC_CFGR_SWS_0 = $00000004;  (*!< Bit 0  *)
  RCC_CFGR_SWS_1 = $00000008;  (*!< Bit 1  *)
  RCC_CFGR_SWS_HSI = $00000000;  (*!< HSI oscillator used as system clock  *)
  RCC_CFGR_SWS_HSE = $00000004;  (*!< HSE oscillator used as system clock  *)
  RCC_CFGR_SWS_PLL = $00000008;  (*!< PLL used as system clock  *)
  (*!< HPRE configuration  *)

  RCC_CFGR_HPRE = $000000F0;  (*!< HPRE[3:0] bits (AHB prescaler)  *)
  RCC_CFGR_HPRE_0 = $00000010;  (*!< Bit 0  *)
  RCC_CFGR_HPRE_1 = $00000020;  (*!< Bit 1  *)
  RCC_CFGR_HPRE_2 = $00000040;  (*!< Bit 2  *)
  RCC_CFGR_HPRE_3 = $00000080;  (*!< Bit 3  *)
  RCC_CFGR_HPRE_DIV1 = $00000000;  (*!< SYSCLK not divided  *)
  RCC_CFGR_HPRE_DIV2 = $00000080;  (*!< SYSCLK divided by 2  *)
  RCC_CFGR_HPRE_DIV4 = $00000090;  (*!< SYSCLK divided by 4  *)
  RCC_CFGR_HPRE_DIV8 = $000000A0;  (*!< SYSCLK divided by 8  *)
  RCC_CFGR_HPRE_DIV16 = $000000B0;  (*!< SYSCLK divided by 16  *)
  RCC_CFGR_HPRE_DIV64 = $000000C0;  (*!< SYSCLK divided by 64  *)
  RCC_CFGR_HPRE_DIV128 = $000000D0;  (*!< SYSCLK divided by 128  *)
  RCC_CFGR_HPRE_DIV256 = $000000E0;  (*!< SYSCLK divided by 256  *)
  RCC_CFGR_HPRE_DIV512 = $000000F0;  (*!< SYSCLK divided by 512  *)
  (*!< PPRE1 configuration  *)

  RCC_CFGR_PPRE1 = $00001C00;  (*!< PRE1[2:0] bits (APB1 prescaler)  *)
  RCC_CFGR_PPRE1_0 = $00000400;  (*!< Bit 0  *)
  RCC_CFGR_PPRE1_1 = $00000800;  (*!< Bit 1  *)
  RCC_CFGR_PPRE1_2 = $00001000;  (*!< Bit 2  *)
  RCC_CFGR_PPRE1_DIV1 = $00000000;  (*!< HCLK not divided  *)
  RCC_CFGR_PPRE1_DIV2 = $00001000;  (*!< HCLK divided by 2  *)
  RCC_CFGR_PPRE1_DIV4 = $00001400;  (*!< HCLK divided by 4  *)
  RCC_CFGR_PPRE1_DIV8 = $00001800;  (*!< HCLK divided by 8  *)
  RCC_CFGR_PPRE1_DIV16 = $00001C00;  (*!< HCLK divided by 16  *)
  (*!< PPRE2 configuration  *)

  RCC_CFGR_PPRE2 = $0000E000;  (*!< PRE2[2:0] bits (APB2 prescaler)  *)
  RCC_CFGR_PPRE2_0 = $00002000;  (*!< Bit 0  *)
  RCC_CFGR_PPRE2_1 = $00004000;  (*!< Bit 1  *)
  RCC_CFGR_PPRE2_2 = $00008000;  (*!< Bit 2  *)
  RCC_CFGR_PPRE2_DIV1 = $00000000;  (*!< HCLK not divided  *)
  RCC_CFGR_PPRE2_DIV2 = $00008000;  (*!< HCLK divided by 2  *)
  RCC_CFGR_PPRE2_DIV4 = $0000A000;  (*!< HCLK divided by 4  *)
  RCC_CFGR_PPRE2_DIV8 = $0000C000;  (*!< HCLK divided by 8  *)
  RCC_CFGR_PPRE2_DIV16 = $0000E000;  (*!< HCLK divided by 16  *)
  (*!< RTCPRE configuration  *)

  RCC_CFGR_RTCPRE = $001F0000;
  RCC_CFGR_RTCPRE_0 = $00010000;
  RCC_CFGR_RTCPRE_1 = $00020000;
  RCC_CFGR_RTCPRE_2 = $00040000;
  RCC_CFGR_RTCPRE_3 = $00080000;
  RCC_CFGR_RTCPRE_4 = $00100000;
  (*!< MCO1 configuration  *)

  RCC_CFGR_MCO1 = $00600000;
  RCC_CFGR_MCO1_0 = $00200000;
  RCC_CFGR_MCO1_1 = $00400000;
  RCC_CFGR_I2SSRC = $00800000;
  RCC_CFGR_MCO1PRE = $07000000;
  RCC_CFGR_MCO1PRE_0 = $01000000;
  RCC_CFGR_MCO1PRE_1 = $02000000;
  RCC_CFGR_MCO1PRE_2 = $04000000;
  RCC_CFGR_MCO2PRE = $38000000;
  RCC_CFGR_MCO2PRE_0 = $08000000;
  RCC_CFGR_MCO2PRE_1 = $10000000;
  RCC_CFGR_MCO2PRE_2 = $20000000;
  RCC_CFGR_MCO2 = $C0000000;
  RCC_CFGR_MCO2_0 = $40000000;
  RCC_CFGR_MCO2_1 = $80000000;
  (********************  Bit definition for RCC_CIR register  ****************** *)

  RCC_CIR_LSIRDYF = $00000001;
  RCC_CIR_LSERDYF = $00000002;
  RCC_CIR_HSIRDYF = $00000004;
  RCC_CIR_HSERDYF = $00000008;
  RCC_CIR_PLLRDYF = $00000010;
  RCC_CIR_PLLI2SRDYF = $00000020;
  RCC_CIR_PLLSAIRDYF = $00000040;
  RCC_CIR_CSSF = $00000080;
  RCC_CIR_LSIRDYIE = $00000100;
  RCC_CIR_LSERDYIE = $00000200;
  RCC_CIR_HSIRDYIE = $00000400;
  RCC_CIR_HSERDYIE = $00000800;
  RCC_CIR_PLLRDYIE = $00001000;
  RCC_CIR_PLLI2SRDYIE = $00002000;
  RCC_CIR_PLLSAIRDYIE = $00004000;
  RCC_CIR_LSIRDYC = $00010000;
  RCC_CIR_LSERDYC = $00020000;
  RCC_CIR_HSIRDYC = $00040000;
  RCC_CIR_HSERDYC = $00080000;
  RCC_CIR_PLLRDYC = $00100000;
  RCC_CIR_PLLI2SRDYC = $00200000;
  RCC_CIR_PLLSAIRDYC = $00400000;
  RCC_CIR_CSSC = $00800000;
  (********************  Bit definition for RCC_AHB1RSTR register  ************* *)

  RCC_AHB1RSTR_GPIOARST = $00000001;
  RCC_AHB1RSTR_GPIOBRST = $00000002;
  RCC_AHB1RSTR_GPIOCRST = $00000004;
  RCC_AHB1RSTR_GPIODRST = $00000008;
  RCC_AHB1RSTR_GPIOERST = $00000010;
  RCC_AHB1RSTR_GPIOFRST = $00000020;
  RCC_AHB1RSTR_GPIOGRST = $00000040;
  RCC_AHB1RSTR_GPIOHRST = $00000080;
  RCC_AHB1RSTR_GPIOIRST = $00000100;
  RCC_AHB1RSTR_GPIOJRST = $00000200;
  RCC_AHB1RSTR_GPIOKRST = $00000400;
  RCC_AHB1RSTR_CRCRST = $00001000;
  RCC_AHB1RSTR_DMA1RST = $00200000;
  RCC_AHB1RSTR_DMA2RST = $00400000;
  RCC_AHB1RSTR_DMA2DRST = $00800000;
  RCC_AHB1RSTR_ETHMACRST = $02000000;
  RCC_AHB1RSTR_OTGHRST = $20000000;
  (********************  Bit definition for RCC_AHB2RSTR register  ************* *)

  RCC_AHB2RSTR_DCMIRST = $00000001;
  RCC_AHB2RSTR_RNGRST = $00000040;
  RCC_AHB2RSTR_OTGFSRST = $00000080;
  (********************  Bit definition for RCC_AHB3RSTR register  ************* *)

  RCC_AHB3RSTR_FMCRST = $00000001;
  RCC_AHB3RSTR_QSPIRST = $00000002;
  (********************  Bit definition for RCC_APB1RSTR register  ************* *)

  RCC_APB1RSTR_TIM2RST = $00000001;
  RCC_APB1RSTR_TIM3RST = $00000002;
  RCC_APB1RSTR_TIM4RST = $00000004;
  RCC_APB1RSTR_TIM5RST = $00000008;
  RCC_APB1RSTR_TIM6RST = $00000010;
  RCC_APB1RSTR_TIM7RST = $00000020;
  RCC_APB1RSTR_TIM12RST = $00000040;
  RCC_APB1RSTR_TIM13RST = $00000080;
  RCC_APB1RSTR_TIM14RST = $00000100;
  RCC_APB1RSTR_LPTIM1RST = $00000200;
  RCC_APB1RSTR_WWDGRST = $00000800;
  RCC_APB1RSTR_SPI2RST = $00004000;
  RCC_APB1RSTR_SPI3RST = $00008000;
  RCC_APB1RSTR_SPDIFRXRST = $00010000;
  RCC_APB1RSTR_USART2RST = $00020000;
  RCC_APB1RSTR_USART3RST = $00040000;
  RCC_APB1RSTR_UART4RST = $00080000;
  RCC_APB1RSTR_UART5RST = $00100000;
  RCC_APB1RSTR_I2C1RST = $00200000;
  RCC_APB1RSTR_I2C2RST = $00400000;
  RCC_APB1RSTR_I2C3RST = $00800000;
  RCC_APB1RSTR_I2C4RST = $01000000;
  RCC_APB1RSTR_CAN1RST = $02000000;
  RCC_APB1RSTR_CAN2RST = $04000000;
  RCC_APB1RSTR_CECRST = $08000000;
  RCC_APB1RSTR_PWRRST = $10000000;
  RCC_APB1RSTR_DACRST = $20000000;
  RCC_APB1RSTR_UART7RST = $40000000;
  RCC_APB1RSTR_UART8RST = $80000000;
  (********************  Bit definition for RCC_APB2RSTR register  ************* *)

  RCC_APB2RSTR_TIM1RST = $00000001;
  RCC_APB2RSTR_TIM8RST = $00000002;
  RCC_APB2RSTR_USART1RST = $00000010;
  RCC_APB2RSTR_USART6RST = $00000020;
  RCC_APB2RSTR_ADCRST = $00000100;
  RCC_APB2RSTR_SDMMC1RST = $00000800;
  RCC_APB2RSTR_SPI1RST = $00001000;
  RCC_APB2RSTR_SPI4RST = $00002000;
  RCC_APB2RSTR_SYSCFGRST = $00004000;
  RCC_APB2RSTR_TIM9RST = $00010000;
  RCC_APB2RSTR_TIM10RST = $00020000;
  RCC_APB2RSTR_TIM11RST = $00040000;
  RCC_APB2RSTR_SPI5RST = $00100000;
  RCC_APB2RSTR_SPI6RST = $00200000;
  RCC_APB2RSTR_SAI1RST = $00400000;
  RCC_APB2RSTR_SAI2RST = $00800000;
  (********************  Bit definition for RCC_AHB1ENR register  ************** *)

  RCC_AHB1ENR_GPIOAEN = $00000001;
  RCC_AHB1ENR_GPIOBEN = $00000002;
  RCC_AHB1ENR_GPIOCEN = $00000004;
  RCC_AHB1ENR_GPIODEN = $00000008;
  RCC_AHB1ENR_GPIOEEN = $00000010;
  RCC_AHB1ENR_GPIOFEN = $00000020;
  RCC_AHB1ENR_GPIOGEN = $00000040;
  RCC_AHB1ENR_GPIOHEN = $00000080;
  RCC_AHB1ENR_GPIOIEN = $00000100;
  RCC_AHB1ENR_GPIOJEN = $00000200;
  RCC_AHB1ENR_GPIOKEN = $00000400;
  RCC_AHB1ENR_CRCEN = $00001000;
  RCC_AHB1ENR_BKPSRAMEN = $00040000;
  RCC_AHB1ENR_DTCMRAMEN = $00100000;
  RCC_AHB1ENR_DMA1EN = $00200000;
  RCC_AHB1ENR_DMA2EN = $00400000;
  RCC_AHB1ENR_DMA2DEN = $00800000;
  RCC_AHB1ENR_ETHMACEN = $02000000;
  RCC_AHB1ENR_ETHMACTXEN = $04000000;
  RCC_AHB1ENR_ETHMACRXEN = $08000000;
  RCC_AHB1ENR_ETHMACPTPEN = $10000000;
  RCC_AHB1ENR_OTGHSEN = $20000000;
  RCC_AHB1ENR_OTGHSULPIEN = $40000000;
  (********************  Bit definition for RCC_AHB2ENR register  ************** *)

  RCC_AHB2ENR_DCMIEN = $00000001;
  RCC_AHB2ENR_RNGEN = $00000040;
  RCC_AHB2ENR_OTGFSEN = $00000080;
  (********************  Bit definition for RCC_AHB3ENR register  ************** *)

  RCC_AHB3ENR_FMCEN = $00000001;
  RCC_AHB3ENR_QSPIEN = $00000002;
  (********************  Bit definition for RCC_APB1ENR register  ************** *)

  RCC_APB1ENR_TIM2EN = $00000001;
  RCC_APB1ENR_TIM3EN = $00000002;
  RCC_APB1ENR_TIM4EN = $00000004;
  RCC_APB1ENR_TIM5EN = $00000008;
  RCC_APB1ENR_TIM6EN = $00000010;
  RCC_APB1ENR_TIM7EN = $00000020;
  RCC_APB1ENR_TIM12EN = $00000040;
  RCC_APB1ENR_TIM13EN = $00000080;
  RCC_APB1ENR_TIM14EN = $00000100;
  RCC_APB1ENR_LPTIM1EN = $00000200;
  RCC_APB1ENR_WWDGEN = $00000800;
  RCC_APB1ENR_SPI2EN = $00004000;
  RCC_APB1ENR_SPI3EN = $00008000;
  RCC_APB1ENR_SPDIFRXEN = $00010000;
  RCC_APB1ENR_USART2EN = $00020000;
  RCC_APB1ENR_USART3EN = $00040000;
  RCC_APB1ENR_UART4EN = $00080000;
  RCC_APB1ENR_UART5EN = $00100000;
  RCC_APB1ENR_I2C1EN = $00200000;
  RCC_APB1ENR_I2C2EN = $00400000;
  RCC_APB1ENR_I2C3EN = $00800000;
  RCC_APB1ENR_I2C4EN = $01000000;
  RCC_APB1ENR_CAN1EN = $02000000;
  RCC_APB1ENR_CAN2EN = $04000000;
  RCC_APB1ENR_CECEN = $08000000;
  RCC_APB1ENR_PWREN = $10000000;
  RCC_APB1ENR_DACEN = $20000000;
  RCC_APB1ENR_UART7EN = $40000000;
  RCC_APB1ENR_UART8EN = $80000000;
  (********************  Bit definition for RCC_APB2ENR register  ************** *)

  RCC_APB2ENR_TIM1EN = $00000001;
  RCC_APB2ENR_TIM8EN = $00000002;
  RCC_APB2ENR_USART1EN = $00000010;
  RCC_APB2ENR_USART6EN = $00000020;
  RCC_APB2ENR_ADC1EN = $00000100;
  RCC_APB2ENR_ADC2EN = $00000200;
  RCC_APB2ENR_ADC3EN = $00000400;
  RCC_APB2ENR_SDMMC1EN = $00000800;
  RCC_APB2ENR_SPI1EN = $00001000;
  RCC_APB2ENR_SPI4EN = $00002000;
  RCC_APB2ENR_SYSCFGEN = $00004000;
  RCC_APB2ENR_TIM9EN = $00010000;
  RCC_APB2ENR_TIM10EN = $00020000;
  RCC_APB2ENR_TIM11EN = $00040000;
  RCC_APB2ENR_SPI5EN = $00100000;
  RCC_APB2ENR_SPI6EN = $00200000;
  RCC_APB2ENR_SAI1EN = $00400000;
  RCC_APB2ENR_SAI2EN = $00800000;
  (********************  Bit definition for RCC_AHB1LPENR register  ************ *)

  RCC_AHB1LPENR_GPIOALPEN = $00000001;
  RCC_AHB1LPENR_GPIOBLPEN = $00000002;
  RCC_AHB1LPENR_GPIOCLPEN = $00000004;
  RCC_AHB1LPENR_GPIODLPEN = $00000008;
  RCC_AHB1LPENR_GPIOELPEN = $00000010;
  RCC_AHB1LPENR_GPIOFLPEN = $00000020;
  RCC_AHB1LPENR_GPIOGLPEN = $00000040;
  RCC_AHB1LPENR_GPIOHLPEN = $00000080;
  RCC_AHB1LPENR_GPIOILPEN = $00000100;
  RCC_AHB1LPENR_GPIOJLPEN = $00000200;
  RCC_AHB1LPENR_GPIOKLPEN = $00000400;
  RCC_AHB1LPENR_CRCLPEN = $00001000;
  RCC_AHB1LPENR_AXILPEN = $00002000;
  RCC_AHB1LPENR_FLITFLPEN = $00008000;
  RCC_AHB1LPENR_SRAM1LPEN = $00010000;
  RCC_AHB1LPENR_SRAM2LPEN = $00020000;
  RCC_AHB1LPENR_BKPSRAMLPEN = $00040000;
  RCC_AHB1LPENR_DTCMLPEN = $00100000;
  RCC_AHB1LPENR_DMA1LPEN = $00200000;
  RCC_AHB1LPENR_DMA2LPEN = $00400000;
  RCC_AHB1LPENR_DMA2DLPEN = $00800000;
  RCC_AHB1LPENR_ETHMACLPEN = $02000000;
  RCC_AHB1LPENR_ETHMACTXLPEN = $04000000;
  RCC_AHB1LPENR_ETHMACRXLPEN = $08000000;
  RCC_AHB1LPENR_ETHMACPTPLPEN = $10000000;
  RCC_AHB1LPENR_OTGHSLPEN = $20000000;
  RCC_AHB1LPENR_OTGHSULPILPEN = $40000000;
  (********************  Bit definition for RCC_AHB2LPENR register  ************ *)

  RCC_AHB2LPENR_DCMILPEN = $00000001;
  RCC_AHB2LPENR_RNGLPEN = $00000040;
  RCC_AHB2LPENR_OTGFSLPEN = $00000080;
  (********************  Bit definition for RCC_AHB3LPENR register  ************ *)

  RCC_AHB3LPENR_FMCLPEN = $00000001;
  RCC_AHB3LPENR_QSPILPEN = $00000002;
  (********************  Bit definition for RCC_APB1LPENR register  ************ *)

  RCC_APB1LPENR_TIM2LPEN = $00000001;
  RCC_APB1LPENR_TIM3LPEN = $00000002;
  RCC_APB1LPENR_TIM4LPEN = $00000004;
  RCC_APB1LPENR_TIM5LPEN = $00000008;
  RCC_APB1LPENR_TIM6LPEN = $00000010;
  RCC_APB1LPENR_TIM7LPEN = $00000020;
  RCC_APB1LPENR_TIM12LPEN = $00000040;
  RCC_APB1LPENR_TIM13LPEN = $00000080;
  RCC_APB1LPENR_TIM14LPEN = $00000100;
  RCC_APB1LPENR_LPTIM1LPEN = $00000200;
  RCC_APB1LPENR_WWDGLPEN = $00000800;
  RCC_APB1LPENR_SPI2LPEN = $00004000;
  RCC_APB1LPENR_SPI3LPEN = $00008000;
  RCC_APB1LPENR_SPDIFRXLPEN = $00010000;
  RCC_APB1LPENR_USART2LPEN = $00020000;
  RCC_APB1LPENR_USART3LPEN = $00040000;
  RCC_APB1LPENR_UART4LPEN = $00080000;
  RCC_APB1LPENR_UART5LPEN = $00100000;
  RCC_APB1LPENR_I2C1LPEN = $00200000;
  RCC_APB1LPENR_I2C2LPEN = $00400000;
  RCC_APB1LPENR_I2C3LPEN = $00800000;
  RCC_APB1LPENR_I2C4LPEN = $01000000;
  RCC_APB1LPENR_CAN1LPEN = $02000000;
  RCC_APB1LPENR_CAN2LPEN = $04000000;
  RCC_APB1LPENR_CECLPEN = $08000000;
  RCC_APB1LPENR_PWRLPEN = $10000000;
  RCC_APB1LPENR_DACLPEN = $20000000;
  RCC_APB1LPENR_UART7LPEN = $40000000;
  RCC_APB1LPENR_UART8LPEN = $80000000;
  (********************  Bit definition for RCC_APB2LPENR register  ************ *)

  RCC_APB2LPENR_TIM1LPEN = $00000001;
  RCC_APB2LPENR_TIM8LPEN = $00000002;
  RCC_APB2LPENR_USART1LPEN = $00000010;
  RCC_APB2LPENR_USART6LPEN = $00000020;
  RCC_APB2LPENR_ADC1LPEN = $00000100;
  RCC_APB2LPENR_ADC2LPEN = $00000200;
  RCC_APB2LPENR_ADC3LPEN = $00000400;
  RCC_APB2LPENR_SDMMC1LPEN = $00000800;
  RCC_APB2LPENR_SPI1LPEN = $00001000;
  RCC_APB2LPENR_SPI4LPEN = $00002000;
  RCC_APB2LPENR_SYSCFGLPEN = $00004000;
  RCC_APB2LPENR_TIM9LPEN = $00010000;
  RCC_APB2LPENR_TIM10LPEN = $00020000;
  RCC_APB2LPENR_TIM11LPEN = $00040000;
  RCC_APB2LPENR_SPI5LPEN = $00100000;
  RCC_APB2LPENR_SPI6LPEN = $00200000;
  RCC_APB2LPENR_SAI1LPEN = $00400000;
  RCC_APB2LPENR_SAI2LPEN = $00800000;
  (********************  Bit definition for RCC_BDCR register  ***************** *)

  RCC_BDCR_LSEON = $00000001;
  RCC_BDCR_LSERDY = $00000002;
  RCC_BDCR_LSEBYP = $00000004;
  RCC_BDCR_LSEDRV = $00000018;
  RCC_BDCR_LSEDRV_0 = $00000008;
  RCC_BDCR_LSEDRV_1 = $00000010;
  RCC_BDCR_RTCSEL = $00000300;
  RCC_BDCR_RTCSEL_0 = $00000100;
  RCC_BDCR_RTCSEL_1 = $00000200;
  RCC_BDCR_RTCEN = $00008000;
  RCC_BDCR_BDRST = $00010000;
  (********************  Bit definition for RCC_CSR register  ****************** *)

  RCC_CSR_LSION = $00000001;
  RCC_CSR_LSIRDY = $00000002;
  RCC_CSR_RMVF = $01000000;
  RCC_CSR_BORRSTF = $02000000;
  RCC_CSR_PINRSTF = $04000000;
  RCC_CSR_PORRSTF = $08000000;
  RCC_CSR_SFTRSTF = $10000000;
  RCC_CSR_IWDGRSTF = $20000000;
  RCC_CSR_WWDGRSTF = $40000000;
  RCC_CSR_LPWRRSTF = $80000000;
  (********************  Bit definition for RCC_SSCGR register  **************** *)

  RCC_SSCGR_MODPER = $00001FFF;
  RCC_SSCGR_INCSTEP = $0FFFE000;
  RCC_SSCGR_SPREADSEL = $40000000;
  RCC_SSCGR_SSCGEN = $80000000;
  (********************  Bit definition for RCC_PLLI2SCFGR register  *********** *)

  RCC_PLLI2SCFGR_PLLI2SN = $00007FC0;
  RCC_PLLI2SCFGR_PLLI2SN_0 = $00000040;
  RCC_PLLI2SCFGR_PLLI2SN_1 = $00000080;
  RCC_PLLI2SCFGR_PLLI2SN_2 = $00000100;
  RCC_PLLI2SCFGR_PLLI2SN_3 = $00000200;
  RCC_PLLI2SCFGR_PLLI2SN_4 = $00000400;
  RCC_PLLI2SCFGR_PLLI2SN_5 = $00000800;
  RCC_PLLI2SCFGR_PLLI2SN_6 = $00001000;
  RCC_PLLI2SCFGR_PLLI2SN_7 = $00002000;
  RCC_PLLI2SCFGR_PLLI2SN_8 = $00004000;
  RCC_PLLI2SCFGR_PLLI2SP = $00030000;
  RCC_PLLI2SCFGR_PLLI2SP_0 = $00010000;
  RCC_PLLI2SCFGR_PLLI2SP_1 = $00020000;
  RCC_PLLI2SCFGR_PLLI2SQ = $0F000000;
  RCC_PLLI2SCFGR_PLLI2SQ_0 = $01000000;
  RCC_PLLI2SCFGR_PLLI2SQ_1 = $02000000;
  RCC_PLLI2SCFGR_PLLI2SQ_2 = $04000000;
  RCC_PLLI2SCFGR_PLLI2SQ_3 = $08000000;
  RCC_PLLI2SCFGR_PLLI2SR = $70000000;
  RCC_PLLI2SCFGR_PLLI2SR_0 = $10000000;
  RCC_PLLI2SCFGR_PLLI2SR_1 = $20000000;
  RCC_PLLI2SCFGR_PLLI2SR_2 = $40000000;
  (********************  Bit definition for RCC_PLLSAICFGR register  *********** *)

  RCC_PLLSAICFGR_PLLSAIN = $00007FC0;
  RCC_PLLSAICFGR_PLLSAIN_0 = $00000040;
  RCC_PLLSAICFGR_PLLSAIN_1 = $00000080;
  RCC_PLLSAICFGR_PLLSAIN_2 = $00000100;
  RCC_PLLSAICFGR_PLLSAIN_3 = $00000200;
  RCC_PLLSAICFGR_PLLSAIN_4 = $00000400;
  RCC_PLLSAICFGR_PLLSAIN_5 = $00000800;
  RCC_PLLSAICFGR_PLLSAIN_6 = $00001000;
  RCC_PLLSAICFGR_PLLSAIN_7 = $00002000;
  RCC_PLLSAICFGR_PLLSAIN_8 = $00004000;
  RCC_PLLSAICFGR_PLLSAIP = $00030000;
  RCC_PLLSAICFGR_PLLSAIP_0 = $00010000;
  RCC_PLLSAICFGR_PLLSAIP_1 = $00020000;
  RCC_PLLSAICFGR_PLLSAIQ = $0F000000;
  RCC_PLLSAICFGR_PLLSAIQ_0 = $01000000;
  RCC_PLLSAICFGR_PLLSAIQ_1 = $02000000;
  RCC_PLLSAICFGR_PLLSAIQ_2 = $04000000;
  RCC_PLLSAICFGR_PLLSAIQ_3 = $08000000;
  RCC_PLLSAICFGR_PLLSAIR = $70000000;
  RCC_PLLSAICFGR_PLLSAIR_0 = $10000000;
  RCC_PLLSAICFGR_PLLSAIR_1 = $20000000;
  RCC_PLLSAICFGR_PLLSAIR_2 = $40000000;
  (********************  Bit definition for RCC_DCKCFGR1 register  ************** *)

  RCC_DCKCFGR1_PLLI2SDIVQ = $0000001F;
  RCC_DCKCFGR1_PLLI2SDIVQ_0 = $00000001;
  RCC_DCKCFGR1_PLLI2SDIVQ_1 = $00000002;
  RCC_DCKCFGR1_PLLI2SDIVQ_2 = $00000004;
  RCC_DCKCFGR1_PLLI2SDIVQ_3 = $00000008;
  RCC_DCKCFGR1_PLLI2SDIVQ_4 = $00000010;
  RCC_DCKCFGR1_PLLSAIDIVQ = $00001F00;
  RCC_DCKCFGR1_PLLSAIDIVQ_0 = $00000100;
  RCC_DCKCFGR1_PLLSAIDIVQ_1 = $00000200;
  RCC_DCKCFGR1_PLLSAIDIVQ_2 = $00000400;
  RCC_DCKCFGR1_PLLSAIDIVQ_3 = $00000800;
  RCC_DCKCFGR1_PLLSAIDIVQ_4 = $00001000;
  RCC_DCKCFGR1_PLLSAIDIVR = $00030000;
  RCC_DCKCFGR1_PLLSAIDIVR_0 = $00010000;
  RCC_DCKCFGR1_PLLSAIDIVR_1 = $00020000;
  RCC_DCKCFGR1_SAI1SEL = $00300000;
  RCC_DCKCFGR1_SAI1SEL_0 = $00100000;
  RCC_DCKCFGR1_SAI1SEL_1 = $00200000;
  RCC_DCKCFGR1_SAI2SEL = $00C00000;
  RCC_DCKCFGR1_SAI2SEL_0 = $00400000;
  RCC_DCKCFGR1_SAI2SEL_1 = $00800000;
  RCC_DCKCFGR1_TIMPRE = $01000000;
  (********************  Bit definition for RCC_DCKCFGR2 register  ************** *)

  RCC_DCKCFGR2_USART1SEL = $00000003;
  RCC_DCKCFGR2_USART1SEL_0 = $00000001;
  RCC_DCKCFGR2_USART1SEL_1 = $00000002;
  RCC_DCKCFGR2_USART2SEL = $0000000C;
  RCC_DCKCFGR2_USART2SEL_0 = $00000004;
  RCC_DCKCFGR2_USART2SEL_1 = $00000008;
  RCC_DCKCFGR2_USART3SEL = $00000030;
  RCC_DCKCFGR2_USART3SEL_0 = $00000010;
  RCC_DCKCFGR2_USART3SEL_1 = $00000020;
  RCC_DCKCFGR2_UART4SEL = $000000C0;
  RCC_DCKCFGR2_UART4SEL_0 = $00000040;
  RCC_DCKCFGR2_UART4SEL_1 = $00000080;
  RCC_DCKCFGR2_UART5SEL = $00000300;
  RCC_DCKCFGR2_UART5SEL_0 = $00000100;
  RCC_DCKCFGR2_UART5SEL_1 = $00000200;
  RCC_DCKCFGR2_USART6SEL = $00000C00;
  RCC_DCKCFGR2_USART6SEL_0 = $00000400;
  RCC_DCKCFGR2_USART6SEL_1 = $00000800;
  RCC_DCKCFGR2_UART7SEL = $00003000;
  RCC_DCKCFGR2_UART7SEL_0 = $00001000;
  RCC_DCKCFGR2_UART7SEL_1 = $00002000;
  RCC_DCKCFGR2_UART8SEL = $0000C000;
  RCC_DCKCFGR2_UART8SEL_0 = $00004000;
  RCC_DCKCFGR2_UART8SEL_1 = $00008000;
  RCC_DCKCFGR2_I2C1SEL = $00030000;
  RCC_DCKCFGR2_I2C1SEL_0 = $00010000;
  RCC_DCKCFGR2_I2C1SEL_1 = $00020000;
  RCC_DCKCFGR2_I2C2SEL = $000C0000;
  RCC_DCKCFGR2_I2C2SEL_0 = $00040000;
  RCC_DCKCFGR2_I2C2SEL_1 = $00080000;
  RCC_DCKCFGR2_I2C3SEL = $00300000;
  RCC_DCKCFGR2_I2C3SEL_0 = $00100000;
  RCC_DCKCFGR2_I2C3SEL_1 = $00200000;
  RCC_DCKCFGR2_I2C4SEL = $00C00000;
  RCC_DCKCFGR2_I2C4SEL_0 = $00400000;
  RCC_DCKCFGR2_I2C4SEL_1 = $00800000;
  RCC_DCKCFGR2_LPTIM1SEL = $03000000;
  RCC_DCKCFGR2_LPTIM1SEL_0 = $01000000;
  RCC_DCKCFGR2_LPTIM1SEL_1 = $02000000;
  RCC_DCKCFGR2_CECSEL = $04000000;
  RCC_DCKCFGR2_CK48MSEL = $08000000;
  RCC_DCKCFGR2_SDMMC1SEL = $10000000;

(***************************************************************************** *)

(*                                                                             *)

(*                                    RNG                                      *)

(*                                                                             *)

(***************************************************************************** *)

(********************  Bits definition for RNG_CR register  ****************** *)

const
  RNG_CR_RNGEN = $00000004;
  RNG_CR_IE = $00000008;
  (********************  Bits definition for RNG_SR register  ****************** *)

  RNG_SR_DRDY = $00000001;
  RNG_SR_CECS = $00000002;
  RNG_SR_SECS = $00000004;
  RNG_SR_CEIS = $00000020;
  RNG_SR_SEIS = $00000040;
  (***************************************************************************** *)

  (*                                                                             *)

  (*                           Real-Time Clock (RTC)                             *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bits definition for RTC_TR register  ****************** *)

  RTC_TR_PM = $00400000;
  RTC_TR_HT = $00300000;
  RTC_TR_HT_0 = $00100000;
  RTC_TR_HT_1 = $00200000;
  RTC_TR_HU = $000F0000;
  RTC_TR_HU_0 = $00010000;
  RTC_TR_HU_1 = $00020000;
  RTC_TR_HU_2 = $00040000;
  RTC_TR_HU_3 = $00080000;
  RTC_TR_MNT = $00007000;
  RTC_TR_MNT_0 = $00001000;
  RTC_TR_MNT_1 = $00002000;
  RTC_TR_MNT_2 = $00004000;
  RTC_TR_MNU = $00000F00;
  RTC_TR_MNU_0 = $00000100;
  RTC_TR_MNU_1 = $00000200;
  RTC_TR_MNU_2 = $00000400;
  RTC_TR_MNU_3 = $00000800;
  RTC_TR_ST = $00000070;
  RTC_TR_ST_0 = $00000010;
  RTC_TR_ST_1 = $00000020;
  RTC_TR_ST_2 = $00000040;
  RTC_TR_SU = $0000000F;
  RTC_TR_SU_0 = $00000001;
  RTC_TR_SU_1 = $00000002;
  RTC_TR_SU_2 = $00000004;
  RTC_TR_SU_3 = $00000008;
  (********************  Bits definition for RTC_DR register  ****************** *)

  RTC_DR_YT = $00F00000;
  RTC_DR_YT_0 = $00100000;
  RTC_DR_YT_1 = $00200000;
  RTC_DR_YT_2 = $00400000;
  RTC_DR_YT_3 = $00800000;
  RTC_DR_YU = $000F0000;
  RTC_DR_YU_0 = $00010000;
  RTC_DR_YU_1 = $00020000;
  RTC_DR_YU_2 = $00040000;
  RTC_DR_YU_3 = $00080000;
  RTC_DR_WDU = $0000E000;
  RTC_DR_WDU_0 = $00002000;
  RTC_DR_WDU_1 = $00004000;
  RTC_DR_WDU_2 = $00008000;
  RTC_DR_MT = $00001000;
  RTC_DR_MU = $00000F00;
  RTC_DR_MU_0 = $00000100;
  RTC_DR_MU_1 = $00000200;
  RTC_DR_MU_2 = $00000400;
  RTC_DR_MU_3 = $00000800;
  RTC_DR_DT = $00000030;
  RTC_DR_DT_0 = $00000010;
  RTC_DR_DT_1 = $00000020;
  RTC_DR_DU = $0000000F;
  RTC_DR_DU_0 = $00000001;
  RTC_DR_DU_1 = $00000002;
  RTC_DR_DU_2 = $00000004;
  RTC_DR_DU_3 = $00000008;
  (********************  Bits definition for RTC_CR register  ****************** *)

  RTC_CR_ITSE = $01000000;
  RTC_CR_COE = $00800000;
  RTC_CR_OSEL = $00600000;
  RTC_CR_OSEL_0 = $00200000;
  RTC_CR_OSEL_1 = $00400000;
  RTC_CR_POL = $00100000;
  RTC_CR_COSEL = $00080000;
  RTC_CR_BCK = $00040000;
  RTC_CR_SUB1H = $00020000;
  RTC_CR_ADD1H = $00010000;
  RTC_CR_TSIE = $00008000;
  RTC_CR_WUTIE = $00004000;
  RTC_CR_ALRBIE = $00002000;
  RTC_CR_ALRAIE = $00001000;
  RTC_CR_TSE = $00000800;
  RTC_CR_WUTE = $00000400;
  RTC_CR_ALRBE = $00000200;
  RTC_CR_ALRAE = $00000100;
  RTC_CR_FMT = $00000040;
  RTC_CR_BYPSHAD = $00000020;
  RTC_CR_REFCKON = $00000010;
  RTC_CR_TSEDGE = $00000008;
  RTC_CR_WUCKSEL = $00000007;
  RTC_CR_WUCKSEL_0 = $00000001;
  RTC_CR_WUCKSEL_1 = $00000002;
  RTC_CR_WUCKSEL_2 = $00000004;
  (********************  Bits definition for RTC_ISR register  ***************** *)

  RTC_ISR_ITSF = $00020000;
  RTC_ISR_RECALPF = $00010000;
  RTC_ISR_TAMP3F = $00008000;
  RTC_ISR_TAMP2F = $00004000;
  RTC_ISR_TAMP1F = $00002000;
  RTC_ISR_TSOVF = $00001000;
  RTC_ISR_TSF = $00000800;
  RTC_ISR_WUTF = $00000400;
  RTC_ISR_ALRBF = $00000200;
  RTC_ISR_ALRAF = $00000100;
  RTC_ISR_INIT = $00000080;
  RTC_ISR_INITF = $00000040;
  RTC_ISR_RSF = $00000020;
  RTC_ISR_INITS = $00000010;
  RTC_ISR_SHPF = $00000008;
  RTC_ISR_WUTWF = $00000004;
  RTC_ISR_ALRBWF = $00000002;
  RTC_ISR_ALRAWF = $00000001;
  (********************  Bits definition for RTC_PRER register  **************** *)

  RTC_PRER_PREDIV_A = $007F0000;
  RTC_PRER_PREDIV_S = $00007FFF;
  (********************  Bits definition for RTC_WUTR register  **************** *)

  RTC_WUTR_WUT = $0000FFFF;
  (********************  Bits definition for RTC_ALRMAR register  ************** *)

  RTC_ALRMAR_MSK4 = $80000000;
  RTC_ALRMAR_WDSEL = $40000000;
  RTC_ALRMAR_DT = $30000000;
  RTC_ALRMAR_DT_0 = $10000000;
  RTC_ALRMAR_DT_1 = $20000000;
  RTC_ALRMAR_DU = $0F000000;
  RTC_ALRMAR_DU_0 = $01000000;
  RTC_ALRMAR_DU_1 = $02000000;
  RTC_ALRMAR_DU_2 = $04000000;
  RTC_ALRMAR_DU_3 = $08000000;
  RTC_ALRMAR_MSK3 = $00800000;
  RTC_ALRMAR_PM = $00400000;
  RTC_ALRMAR_HT = $00300000;
  RTC_ALRMAR_HT_0 = $00100000;
  RTC_ALRMAR_HT_1 = $00200000;
  RTC_ALRMAR_HU = $000F0000;
  RTC_ALRMAR_HU_0 = $00010000;
  RTC_ALRMAR_HU_1 = $00020000;
  RTC_ALRMAR_HU_2 = $00040000;
  RTC_ALRMAR_HU_3 = $00080000;
  RTC_ALRMAR_MSK2 = $00008000;
  RTC_ALRMAR_MNT = $00007000;
  RTC_ALRMAR_MNT_0 = $00001000;
  RTC_ALRMAR_MNT_1 = $00002000;
  RTC_ALRMAR_MNT_2 = $00004000;
  RTC_ALRMAR_MNU = $00000F00;
  RTC_ALRMAR_MNU_0 = $00000100;
  RTC_ALRMAR_MNU_1 = $00000200;
  RTC_ALRMAR_MNU_2 = $00000400;
  RTC_ALRMAR_MNU_3 = $00000800;
  RTC_ALRMAR_MSK1 = $00000080;
  RTC_ALRMAR_ST = $00000070;
  RTC_ALRMAR_ST_0 = $00000010;
  RTC_ALRMAR_ST_1 = $00000020;
  RTC_ALRMAR_ST_2 = $00000040;
  RTC_ALRMAR_SU = $0000000F;
  RTC_ALRMAR_SU_0 = $00000001;
  RTC_ALRMAR_SU_1 = $00000002;
  RTC_ALRMAR_SU_2 = $00000004;
  RTC_ALRMAR_SU_3 = $00000008;
  (********************  Bits definition for RTC_ALRMBR register  ************** *)

  RTC_ALRMBR_MSK4 = $80000000;
  RTC_ALRMBR_WDSEL = $40000000;
  RTC_ALRMBR_DT = $30000000;
  RTC_ALRMBR_DT_0 = $10000000;
  RTC_ALRMBR_DT_1 = $20000000;
  RTC_ALRMBR_DU = $0F000000;
  RTC_ALRMBR_DU_0 = $01000000;
  RTC_ALRMBR_DU_1 = $02000000;
  RTC_ALRMBR_DU_2 = $04000000;
  RTC_ALRMBR_DU_3 = $08000000;
  RTC_ALRMBR_MSK3 = $00800000;
  RTC_ALRMBR_PM = $00400000;
  RTC_ALRMBR_HT = $00300000;
  RTC_ALRMBR_HT_0 = $00100000;
  RTC_ALRMBR_HT_1 = $00200000;
  RTC_ALRMBR_HU = $000F0000;
  RTC_ALRMBR_HU_0 = $00010000;
  RTC_ALRMBR_HU_1 = $00020000;
  RTC_ALRMBR_HU_2 = $00040000;
  RTC_ALRMBR_HU_3 = $00080000;
  RTC_ALRMBR_MSK2 = $00008000;
  RTC_ALRMBR_MNT = $00007000;
  RTC_ALRMBR_MNT_0 = $00001000;
  RTC_ALRMBR_MNT_1 = $00002000;
  RTC_ALRMBR_MNT_2 = $00004000;
  RTC_ALRMBR_MNU = $00000F00;
  RTC_ALRMBR_MNU_0 = $00000100;
  RTC_ALRMBR_MNU_1 = $00000200;
  RTC_ALRMBR_MNU_2 = $00000400;
  RTC_ALRMBR_MNU_3 = $00000800;
  RTC_ALRMBR_MSK1 = $00000080;
  RTC_ALRMBR_ST = $00000070;
  RTC_ALRMBR_ST_0 = $00000010;
  RTC_ALRMBR_ST_1 = $00000020;
  RTC_ALRMBR_ST_2 = $00000040;
  RTC_ALRMBR_SU = $0000000F;
  RTC_ALRMBR_SU_0 = $00000001;
  RTC_ALRMBR_SU_1 = $00000002;
  RTC_ALRMBR_SU_2 = $00000004;
  RTC_ALRMBR_SU_3 = $00000008;
  (********************  Bits definition for RTC_WPR register  ***************** *)

  RTC_WPR_KEY = $000000FF;
  (********************  Bits definition for RTC_SSR register  ***************** *)

  RTC_SSR_SS = $0000FFFF;
  (********************  Bits definition for RTC_SHIFTR register  ************** *)

  RTC_SHIFTR_SUBFS = $00007FFF;
  RTC_SHIFTR_ADD1S = $80000000;
  (********************  Bits definition for RTC_TSTR register  **************** *)

  RTC_TSTR_PM = $00400000;
  RTC_TSTR_HT = $00300000;
  RTC_TSTR_HT_0 = $00100000;
  RTC_TSTR_HT_1 = $00200000;
  RTC_TSTR_HU = $000F0000;
  RTC_TSTR_HU_0 = $00010000;
  RTC_TSTR_HU_1 = $00020000;
  RTC_TSTR_HU_2 = $00040000;
  RTC_TSTR_HU_3 = $00080000;
  RTC_TSTR_MNT = $00007000;
  RTC_TSTR_MNT_0 = $00001000;
  RTC_TSTR_MNT_1 = $00002000;
  RTC_TSTR_MNT_2 = $00004000;
  RTC_TSTR_MNU = $00000F00;
  RTC_TSTR_MNU_0 = $00000100;
  RTC_TSTR_MNU_1 = $00000200;
  RTC_TSTR_MNU_2 = $00000400;
  RTC_TSTR_MNU_3 = $00000800;
  RTC_TSTR_ST = $00000070;
  RTC_TSTR_ST_0 = $00000010;
  RTC_TSTR_ST_1 = $00000020;
  RTC_TSTR_ST_2 = $00000040;
  RTC_TSTR_SU = $0000000F;
  RTC_TSTR_SU_0 = $00000001;
  RTC_TSTR_SU_1 = $00000002;
  RTC_TSTR_SU_2 = $00000004;
  RTC_TSTR_SU_3 = $00000008;
  (********************  Bits definition for RTC_TSDR register  **************** *)

  RTC_TSDR_WDU = $0000E000;
  RTC_TSDR_WDU_0 = $00002000;
  RTC_TSDR_WDU_1 = $00004000;
  RTC_TSDR_WDU_2 = $00008000;
  RTC_TSDR_MT = $00001000;
  RTC_TSDR_MU = $00000F00;
  RTC_TSDR_MU_0 = $00000100;
  RTC_TSDR_MU_1 = $00000200;
  RTC_TSDR_MU_2 = $00000400;
  RTC_TSDR_MU_3 = $00000800;
  RTC_TSDR_DT = $00000030;
  RTC_TSDR_DT_0 = $00000010;
  RTC_TSDR_DT_1 = $00000020;
  RTC_TSDR_DU = $0000000F;
  RTC_TSDR_DU_0 = $00000001;
  RTC_TSDR_DU_1 = $00000002;
  RTC_TSDR_DU_2 = $00000004;
  RTC_TSDR_DU_3 = $00000008;
  (********************  Bits definition for RTC_TSSSR register  *************** *)

  RTC_TSSSR_SS = $0000FFFF;
  (********************  Bits definition for RTC_CAL register  **************** *)

  RTC_CALR_CALP = $00008000;
  RTC_CALR_CALW8 = $00004000;
  RTC_CALR_CALW16 = $00002000;
  RTC_CALR_CALM = $000001FF;
  RTC_CALR_CALM_0 = $00000001;
  RTC_CALR_CALM_1 = $00000002;
  RTC_CALR_CALM_2 = $00000004;
  RTC_CALR_CALM_3 = $00000008;
  RTC_CALR_CALM_4 = $00000010;
  RTC_CALR_CALM_5 = $00000020;
  RTC_CALR_CALM_6 = $00000040;
  RTC_CALR_CALM_7 = $00000080;
  RTC_CALR_CALM_8 = $00000100;
  (********************  Bits definition for RTC_TAMPCR register  *************** *)

  RTC_TAMPCR_TAMP3MF = $01000000;
  RTC_TAMPCR_TAMP3NOERASE = $00800000;
  RTC_TAMPCR_TAMP3IE = $00400000;
  RTC_TAMPCR_TAMP2MF = $00200000;
  RTC_TAMPCR_TAMP2NOERASE = $00100000;
  RTC_TAMPCR_TAMP2IE = $00080000;
  RTC_TAMPCR_TAMP1MF = $00040000;
  RTC_TAMPCR_TAMP1NOERASE = $00020000;
  RTC_TAMPCR_TAMP1IE = $00010000;
  RTC_TAMPCR_TAMPPUDIS = $00008000;
  RTC_TAMPCR_TAMPPRCH = $00006000;
  RTC_TAMPCR_TAMPPRCH_0 = $00002000;
  RTC_TAMPCR_TAMPPRCH_1 = $00004000;
  RTC_TAMPCR_TAMPFLT = $00001800;
  RTC_TAMPCR_TAMPFLT_0 = $00000800;
  RTC_TAMPCR_TAMPFLT_1 = $00001000;
  RTC_TAMPCR_TAMPFREQ = $00000700;
  RTC_TAMPCR_TAMPFREQ_0 = $00000100;
  RTC_TAMPCR_TAMPFREQ_1 = $00000200;
  RTC_TAMPCR_TAMPFREQ_2 = $00000400;
  RTC_TAMPCR_TAMPTS = $00000080;
  RTC_TAMPCR_TAMP3_TRG = $00000040;
  RTC_TAMPCR_TAMP3E = $00000020;
  RTC_TAMPCR_TAMP2_TRG = $00000010;
  RTC_TAMPCR_TAMP2E = $00000008;
  RTC_TAMPCR_TAMPIE = $00000004;
  RTC_TAMPCR_TAMP1_TRG = $00000002;
  RTC_TAMPCR_TAMP1E = $00000001;
  (********************  Bits definition for RTC_ALRMASSR register  ************ *)

  RTC_ALRMASSR_MASKSS = $0F000000;
  RTC_ALRMASSR_MASKSS_0 = $01000000;
  RTC_ALRMASSR_MASKSS_1 = $02000000;
  RTC_ALRMASSR_MASKSS_2 = $04000000;
  RTC_ALRMASSR_MASKSS_3 = $08000000;
  RTC_ALRMASSR_SS = $00007FFF;
  (********************  Bits definition for RTC_ALRMBSSR register  ************ *)

  RTC_ALRMBSSR_MASKSS = $0F000000;
  RTC_ALRMBSSR_MASKSS_0 = $01000000;
  RTC_ALRMBSSR_MASKSS_1 = $02000000;
  RTC_ALRMBSSR_MASKSS_2 = $04000000;
  RTC_ALRMBSSR_MASKSS_3 = $08000000;
  RTC_ALRMBSSR_SS = $00007FFF;
  (********************  Bits definition for RTC_OR register  *************** *)

  RTC_OR_TSINSEL = $00000006;
  RTC_OR_TSINSEL_0 = $00000002;
  RTC_OR_TSINSEL_1 = $00000004;
  RTC_OR_ALARMTYPE = $00000008;
  (********************  Bits definition for RTC_BKP0R register  *************** *)

  RTC_BKP0R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP1R register  *************** *)

  RTC_BKP1R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP2R register  *************** *)

  RTC_BKP2R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP3R register  *************** *)

  RTC_BKP3R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP4R register  *************** *)

  RTC_BKP4R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP5R register  *************** *)

  RTC_BKP5R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP6R register  *************** *)

  RTC_BKP6R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP7R register  *************** *)

  RTC_BKP7R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP8R register  *************** *)

  RTC_BKP8R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP9R register  *************** *)

  RTC_BKP9R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP10R register  ************** *)

  RTC_BKP10R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP11R register  ************** *)

  RTC_BKP11R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP12R register  ************** *)

  RTC_BKP12R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP13R register  ************** *)

  RTC_BKP13R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP14R register  ************** *)

  RTC_BKP14R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP15R register  ************** *)

  RTC_BKP15R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP16R register  ************** *)

  RTC_BKP16R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP17R register  ************** *)

  RTC_BKP17R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP18R register  ************** *)

  RTC_BKP18R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP19R register  ************** *)

  RTC_BKP19R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP20R register  ************** *)

  RTC_BKP20R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP21R register  ************** *)

  RTC_BKP21R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP22R register  ************** *)

  RTC_BKP22R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP23R register  ************** *)

  RTC_BKP23R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP24R register  ************** *)

  RTC_BKP24R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP25R register  ************** *)

  RTC_BKP25R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP26R register  ************** *)

  RTC_BKP26R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP27R register  ************** *)

  RTC_BKP27R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP28R register  ************** *)

  RTC_BKP28R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP29R register  ************** *)

  RTC_BKP29R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP30R register  ************** *)

  RTC_BKP30R = $FFFFFFFF;
  (********************  Bits definition for RTC_BKP31R register  ************** *)

  RTC_BKP31R = $FFFFFFFF;
  (******************** Number of backup registers ***************************** *)

  RTC_BKP_NUMBER = $00000020;
  (***************************************************************************** *)

  (*                                                                             *)

  (*                          Serial Audio Interface                             *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for SAI_GCR register  ****************** *)

  SAI_GCR_SYNCIN = $00000003;  (*!<SYNCIN[1:0] bits (Synchronization Inputs)    *)
  SAI_GCR_SYNCIN_0 = $00000001;  (*!<Bit 0  *)
  SAI_GCR_SYNCIN_1 = $00000002;  (*!<Bit 1  *)
  SAI_GCR_SYNCOUT = $00000030;  (*!<SYNCOUT[1:0] bits (Synchronization Outputs)  *)
  SAI_GCR_SYNCOUT_0 = $00000010;  (*!<Bit 0  *)
  SAI_GCR_SYNCOUT_1 = $00000020;  (*!<Bit 1  *)
  (*******************  Bit definition for SAI_xCR1 register  ****************** *)

  SAI_xCR1_MODE = $00000003;  (*!<MODE[1:0] bits (Audio Block Mode)            *)
  SAI_xCR1_MODE_0 = $00000001;  (*!<Bit 0  *)
  SAI_xCR1_MODE_1 = $00000002;  (*!<Bit 1  *)
  SAI_xCR1_PRTCFG = $0000000C;  (*!<PRTCFG[1:0] bits (Protocol Configuration)    *)
  SAI_xCR1_PRTCFG_0 = $00000004;  (*!<Bit 0  *)
  SAI_xCR1_PRTCFG_1 = $00000008;  (*!<Bit 1  *)
  SAI_xCR1_DS = $000000E0;  (*!<DS[1:0] bits (Data Size)  *)
  SAI_xCR1_DS_0 = $00000020;  (*!<Bit 0  *)
  SAI_xCR1_DS_1 = $00000040;  (*!<Bit 1  *)
  SAI_xCR1_DS_2 = $00000080;  (*!<Bit 2  *)
  SAI_xCR1_LSBFIRST = $00000100;  (*!<LSB First Configuration   *)
  SAI_xCR1_CKSTR = $00000200;  (*!<ClocK STRobing edge       *)
  SAI_xCR1_SYNCEN = $00000C00;  (*!<SYNCEN[1:0](SYNChronization ENable)  *)
  SAI_xCR1_SYNCEN_0 = $00000400;  (*!<Bit 0  *)
  SAI_xCR1_SYNCEN_1 = $00000800;  (*!<Bit 1  *)
  SAI_xCR1_MONO = $00001000;  (*!<Mono mode                   *)
  SAI_xCR1_OUTDRIV = $00002000;  (*!<Output Drive                *)
  SAI_xCR1_SAIEN = $00010000;  (*!<Audio Block enable          *)
  SAI_xCR1_DMAEN = $00020000;  (*!<DMA enable                  *)
  SAI_xCR1_NODIV = $00080000;  (*!<No Divider Configuration    *)
  SAI_xCR1_MCKDIV = $00F00000;  (*!<MCKDIV[3:0] (Master ClocK Divider)   *)
  SAI_xCR1_MCKDIV_0 = $00100000;  (*!<Bit 0   *)
  SAI_xCR1_MCKDIV_1 = $00200000;  (*!<Bit 1   *)
  SAI_xCR1_MCKDIV_2 = $00400000;  (*!<Bit 2   *)
  SAI_xCR1_MCKDIV_3 = $00800000;  (*!<Bit 3   *)
  (*******************  Bit definition for SAI_xCR2 register  ****************** *)

  SAI_xCR2_FTH = $00000007;  (*!<FTH[2:0](Fifo THreshold)   *)
  SAI_xCR2_FTH_0 = $00000001;  (*!<Bit 0  *)
  SAI_xCR2_FTH_1 = $00000002;  (*!<Bit 1  *)
  SAI_xCR2_FTH_2 = $00000004;  (*!<Bit 2  *)
  SAI_xCR2_FFLUSH = $00000008;  (*!<Fifo FLUSH                        *)
  SAI_xCR2_TRIS = $00000010;  (*!<TRIState Management on data line  *)
  SAI_xCR2_MUTE = $00000020;  (*!<Mute mode                         *)
  SAI_xCR2_MUTEVAL = $00000040;  (*!<Muate value                       *)
  SAI_xCR2_MUTECNT = $00001F80;  (*!<MUTECNT[5:0] (MUTE counter)  *)
  SAI_xCR2_MUTECNT_0 = $00000080;  (*!<Bit 0  *)
  SAI_xCR2_MUTECNT_1 = $00000100;  (*!<Bit 1  *)
  SAI_xCR2_MUTECNT_2 = $00000200;  (*!<Bit 2  *)
  SAI_xCR2_MUTECNT_3 = $00000400;  (*!<Bit 3  *)
  SAI_xCR2_MUTECNT_4 = $00000800;  (*!<Bit 4  *)
  SAI_xCR2_MUTECNT_5 = $00001000;  (*!<Bit 5  *)
  SAI_xCR2_CPL = $00080000;  (*!< Complement Bit              *)
  SAI_xCR2_COMP = $0000C000;  (*!<COMP[1:0] (Companding mode)  *)
  SAI_xCR2_COMP_0 = $00004000;  (*!<Bit 0  *)
  SAI_xCR2_COMP_1 = $00008000;  (*!<Bit 1  *)
  (******************  Bit definition for SAI_xFRCR register  ****************** *)

  SAI_xFRCR_FRL = $000000FF;  (*!<FRL[1:0](Frame length)   *)
  SAI_xFRCR_FRL_0 = $00000001;  (*!<Bit 0  *)
  SAI_xFRCR_FRL_1 = $00000002;  (*!<Bit 1  *)
  SAI_xFRCR_FRL_2 = $00000004;  (*!<Bit 2  *)
  SAI_xFRCR_FRL_3 = $00000008;  (*!<Bit 3  *)
  SAI_xFRCR_FRL_4 = $00000010;  (*!<Bit 4  *)
  SAI_xFRCR_FRL_5 = $00000020;  (*!<Bit 5  *)
  SAI_xFRCR_FRL_6 = $00000040;  (*!<Bit 6  *)
  SAI_xFRCR_FRL_7 = $00000080;  (*!<Bit 7  *)
  SAI_xFRCR_FSALL = $00007F00;  (*!<FRL[1:0] (Frame synchronization active level length)   *)
  SAI_xFRCR_FSALL_0 = $00000100;  (*!<Bit 0  *)
  SAI_xFRCR_FSALL_1 = $00000200;  (*!<Bit 1  *)
  SAI_xFRCR_FSALL_2 = $00000400;  (*!<Bit 2  *)
  SAI_xFRCR_FSALL_3 = $00000800;  (*!<Bit 3  *)
  SAI_xFRCR_FSALL_4 = $00001000;  (*!<Bit 4  *)
  SAI_xFRCR_FSALL_5 = $00002000;  (*!<Bit 5  *)
  SAI_xFRCR_FSALL_6 = $00004000;  (*!<Bit 6  *)
  SAI_xFRCR_FSDEF = $00010000;  (*!< Frame Synchronization Definition  *)
  SAI_xFRCR_FSPO = $00020000;  (*!<Frame Synchronization POLarity     *)
  SAI_xFRCR_FSOFF = $00040000;  (*!<Frame Synchronization OFFset       *)
  (******************  Bit definition for SAI_xSLOTR register  ****************** *)

  SAI_xSLOTR_FBOFF = $0000001F;  (*!<FRL[4:0](First Bit Offset)   *)
  SAI_xSLOTR_FBOFF_0 = $00000001;  (*!<Bit 0  *)
  SAI_xSLOTR_FBOFF_1 = $00000002;  (*!<Bit 1  *)
  SAI_xSLOTR_FBOFF_2 = $00000004;  (*!<Bit 2  *)
  SAI_xSLOTR_FBOFF_3 = $00000008;  (*!<Bit 3  *)
  SAI_xSLOTR_FBOFF_4 = $00000010;  (*!<Bit 4  *)
  SAI_xSLOTR_SLOTSZ = $000000C0;  (*!<SLOTSZ[1:0] (Slot size)   *)
  SAI_xSLOTR_SLOTSZ_0 = $00000040;  (*!<Bit 0  *)
  SAI_xSLOTR_SLOTSZ_1 = $00000080;  (*!<Bit 1  *)
  SAI_xSLOTR_NBSLOT = $00000F00;  (*!<NBSLOT[3:0] (Number of Slot in audio Frame)   *)
  SAI_xSLOTR_NBSLOT_0 = $00000100;  (*!<Bit 0  *)
  SAI_xSLOTR_NBSLOT_1 = $00000200;  (*!<Bit 1  *)
  SAI_xSLOTR_NBSLOT_2 = $00000400;  (*!<Bit 2  *)
  SAI_xSLOTR_NBSLOT_3 = $00000800;  (*!<Bit 3  *)
  SAI_xSLOTR_SLOTEN = $FFFF0000;  (*!<SLOTEN[15:0] (Slot Enable)   *)
  (*******************  Bit definition for SAI_xIMR register  ****************** *)

  SAI_xIMR_OVRUDRIE = $00000001;  (*!<Overrun underrun interrupt enable                               *)
  SAI_xIMR_MUTEDETIE = $00000002;  (*!<Mute detection interrupt enable                                 *)
  SAI_xIMR_WCKCFGIE = $00000004;  (*!<Wrong Clock Configuration interrupt enable                      *)
  SAI_xIMR_FREQIE = $00000008;  (*!<FIFO request interrupt enable                                   *)
  SAI_xIMR_CNRDYIE = $00000010;  (*!<Codec not ready interrupt enable                                *)
  SAI_xIMR_AFSDETIE = $00000020;  (*!<Anticipated frame synchronization detection interrupt enable    *)
  SAI_xIMR_LFSDETIE = $00000040;  (*!<Late frame synchronization detection interrupt enable           *)
  (********************  Bit definition for SAI_xSR register  ****************** *)

  SAI_xSR_OVRUDR = $00000001;  (*!<Overrun underrun                                *)
  SAI_xSR_MUTEDET = $00000002;  (*!<Mute detection                                  *)
  SAI_xSR_WCKCFG = $00000004;  (*!<Wrong Clock Configuration                       *)
  SAI_xSR_FREQ = $00000008;  (*!<FIFO request                                    *)
  SAI_xSR_CNRDY = $00000010;  (*!<Codec not ready                                 *)
  SAI_xSR_AFSDET = $00000020;  (*!<Anticipated frame synchronization detection     *)
  SAI_xSR_LFSDET = $00000040;  (*!<Late frame synchronization detection            *)
  SAI_xSR_FLVL = $00070000;  (*!<FLVL[2:0] (FIFO Level Threshold)                *)
  SAI_xSR_FLVL_0 = $00010000;  (*!<Bit 0  *)
  SAI_xSR_FLVL_1 = $00020000;  (*!<Bit 1  *)
  SAI_xSR_FLVL_2 = $00040000;  (*!<Bit 2  *)
  (******************  Bit definition for SAI_xCLRFR register  ***************** *)

  SAI_xCLRFR_COVRUDR = $00000001;  (*!<Clear Overrun underrun                                *)
  SAI_xCLRFR_CMUTEDET = $00000002;  (*!<Clear Mute detection                                  *)
  SAI_xCLRFR_CWCKCFG = $00000004;  (*!<Clear Wrong Clock Configuration                       *)
  SAI_xCLRFR_CFREQ = $00000008;  (*!<Clear FIFO request                                    *)
  SAI_xCLRFR_CCNRDY = $00000010;  (*!<Clear Codec not ready                                 *)
  SAI_xCLRFR_CAFSDET = $00000020;  (*!<Clear Anticipated frame synchronization detection     *)
  SAI_xCLRFR_CLFSDET = $00000040;  (*!<Clear Late frame synchronization detection            *)
  (******************  Bit definition for SAI_xDR register  ******************** *)

  SAI_xDR_DATA = $FFFFFFFF;
  (***************************************************************************** *)

  (*                                                                             *)

  (*                              SPDIF-RX Interface                             *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for SPDIF_CR register  ****************** *)

  SPDIFRX_CR_SPDIFEN = $00000003;  (*!<Peripheral Block Enable                       *)
  SPDIFRX_CR_RXDMAEN = $00000004;  (*!<Receiver DMA Enable for data flow             *)
  SPDIFRX_CR_RXSTEO = $00000008;  (*!<Stereo Mode                                   *)
  SPDIFRX_CR_DRFMT = $00000030;  (*!<RX Data format                                *)
  SPDIFRX_CR_PMSK = $00000040;  (*!<Mask Parity error bit                         *)
  SPDIFRX_CR_VMSK = $00000080;  (*!<Mask of Validity bit                          *)
  SPDIFRX_CR_CUMSK = $00000100;  (*!<Mask of channel status and user bits          *)
  SPDIFRX_CR_PTMSK = $00000200;  (*!<Mask of Preamble Type bits                    *)
  SPDIFRX_CR_CBDMAEN = $00000400;  (*!<Control Buffer DMA ENable for control flow    *)
  SPDIFRX_CR_CHSEL = $00000800;  (*!<Channel Selection                             *)
  SPDIFRX_CR_NBTR = $00003000;  (*!<Maximum allowed re-tries during synchronization phase  *)
  SPDIFRX_CR_WFA = $00004000;  (*!<Wait For Activity      *)
  SPDIFRX_CR_INSEL = $00070000;  (*!<SPDIF input selection  *)
  (*******************  Bit definition for SPDIFRX_IMR register  ****************** *)

  SPDIFRX_IMR_RXNEIE = $00000001;  (*!<RXNE interrupt enable                               *)
  SPDIFRX_IMR_CSRNEIE = $00000002;  (*!<Control Buffer Ready Interrupt Enable               *)
  SPDIFRX_IMR_PERRIE = $00000004;  (*!<Parity error interrupt enable                       *)
  SPDIFRX_IMR_OVRIE = $00000008;  (*!<Overrun error Interrupt Enable                      *)
  SPDIFRX_IMR_SBLKIE = $00000010;  (*!<Synchronization Block Detected Interrupt Enable     *)
  SPDIFRX_IMR_SYNCDIE = $00000020;  (*!<Synchronization Done                                *)
  SPDIFRX_IMR_IFEIE = $00000040;  (*!<Serial Interface Error Interrupt Enable             *)
  (*******************  Bit definition for SPDIFRX_SR register  ****************** *)

  SPDIFRX_SR_RXNE = $00000001;  (*!<Read data register not empty                           *)
  SPDIFRX_SR_CSRNE = $00000002;  (*!<The Control Buffer register is not empty               *)
  SPDIFRX_SR_PERR = $00000004;  (*!<Parity error                                           *)
  SPDIFRX_SR_OVR = $00000008;  (*!<Overrun error                                          *)
  SPDIFRX_SR_SBD = $00000010;  (*!<Synchronization Block Detected                         *)
  SPDIFRX_SR_SYNCD = $00000020;  (*!<Synchronization Done                                   *)
  SPDIFRX_SR_FERR = $00000040;  (*!<Framing error                                          *)
  SPDIFRX_SR_SERR = $00000080;  (*!<Synchronization error                                  *)
  SPDIFRX_SR_TERR = $00000100;  (*!<Time-out error                                         *)
  SPDIFRX_SR_WIDTH5 = $7FFF0000;  (*!<Duration of 5 symbols counted with spdif_clk           *)
  (*******************  Bit definition for SPDIFRX_IFCR register  ****************** *)

  SPDIFRX_IFCR_PERRCF = $00000004;  (*!<Clears the Parity error flag                          *)
  SPDIFRX_IFCR_OVRCF = $00000008;  (*!<Clears the Overrun error flag                         *)
  SPDIFRX_IFCR_SBDCF = $00000010;  (*!<Clears the Synchronization Block Detected flag        *)
  SPDIFRX_IFCR_SYNCDCF = $00000020;  (*!<Clears the Synchronization Done flag                  *)
  (*******************  Bit definition for SPDIFRX_DR register  (DRFMT = 0b00 case) ****************** *)

  SPDIFRX_DR0_DR = $00FFFFFF;  (*!<Data value             *)
  SPDIFRX_DR0_PE = $01000000;  (*!<Parity Error bit       *)
  SPDIFRX_DR0_V = $02000000;  (*!<Validity bit           *)
  SPDIFRX_DR0_U = $04000000;  (*!<User bit               *)
  SPDIFRX_DR0_C = $08000000;  (*!<Channel Status bit     *)
  SPDIFRX_DR0_PT = $30000000;  (*!<Preamble Type          *)
  (*******************  Bit definition for SPDIFRX_DR register  (DRFMT = 0b01 case) ****************** *)

  SPDIFRX_DR1_DR = $FFFFFF00;  (*!<Data value             *)
  SPDIFRX_DR1_PT = $00000030;  (*!<Preamble Type          *)
  SPDIFRX_DR1_C = $00000008;  (*!<Channel Status bit     *)
  SPDIFRX_DR1_U = $00000004;  (*!<User bit               *)
  SPDIFRX_DR1_V = $00000002;  (*!<Validity bit           *)
  SPDIFRX_DR1_PE = $00000001;  (*!<Parity Error bit       *)
  (*******************  Bit definition for SPDIFRX_DR register  (DRFMT = 0b10 case) ****************** *)

  SPDIFRX_DR1_DRNL1 = $FFFF0000;  (*!<Data value Channel B       *)
  SPDIFRX_DR1_DRNL2 = $0000FFFF;  (*!<Data value Channel A       *)
  (*******************  Bit definition for SPDIFRX_CSR register   ****************** *)

  SPDIFRX_CSR_USR = $0000FFFF;  (*!<User data information            *)
  SPDIFRX_CSR_CS = $00FF0000;  (*!<Channel A status information     *)
  SPDIFRX_CSR_SOB = $01000000;  (*!<Start Of Block                   *)
  (*******************  Bit definition for SPDIFRX_DIR register    ****************** *)

  SPDIFRX_DIR_THI = $000013FF;  (*!<Threshold LOW       *)
  SPDIFRX_DIR_TLO = $1FFF0000;  (*!<Threshold HIGH      *)

(***************************************************************************** *)

(*                                                                             *)

(*                          SD host Interface                                  *)

(*                                                                             *)

(***************************************************************************** *)

(******************  Bit definition for SDMMC_POWER register  ***************** *)

const
  SDMMC_POWER_PWRCTRL = $03;  (*!<PWRCTRL[1:0] bits (Power supply control bits)  *)
  SDMMC_POWER_PWRCTRL_0 = $01;  (*!<Bit 0  *)
  SDMMC_POWER_PWRCTRL_1 = $02;  (*!<Bit 1  *)
  (******************  Bit definition for SDMMC_CLKCR register  ***************** *)

  SDMMC_CLKCR_CLKDIV = $00FF;  (*!<Clock divide factor              *)
  SDMMC_CLKCR_CLKEN = $0100;  (*!<Clock enable bit                 *)
  SDMMC_CLKCR_PWRSAV = $0200;  (*!<Power saving configuration bit   *)
  SDMMC_CLKCR_BYPASS = $0400;  (*!<Clock divider bypass enable bit  *)
  SDMMC_CLKCR_WIDBUS = $1800;  (*!<WIDBUS[1:0] bits (Wide bus mode enable bit)  *)
  SDMMC_CLKCR_WIDBUS_0 = $0800;  (*!<Bit 0  *)
  SDMMC_CLKCR_WIDBUS_1 = $1000;  (*!<Bit 1  *)
  SDMMC_CLKCR_NEGEDGE = $2000;  (*!<SDMMC_CK dephasing selection bit  *)
  SDMMC_CLKCR_HWFC_EN = $4000;  (*!<HW Flow Control enable           *)
  (*******************  Bit definition for SDMMC_ARG register  ****************** *)

  SDMMC_ARG_CMDARG = $FFFFFFFF;  (*!<Command argument  *)
  (*******************  Bit definition for SDMMC_CMD register  ****************** *)

  SDMMC_CMD_CMDINDEX = $003F;  (*!<Command Index                                *)
  SDMMC_CMD_WAITRESP = $00C0;  (*!<WAITRESP[1:0] bits (Wait for response bits)  *)
  SDMMC_CMD_WAITRESP_0 = $0040;  (*!< Bit 0  *)
  SDMMC_CMD_WAITRESP_1 = $0080;  (*!< Bit 1  *)
  SDMMC_CMD_WAITINT = $0100;  (*!<CPSM Waits for Interrupt Request                                *)
  SDMMC_CMD_WAITPEND = $0200;  (*!<CPSM Waits for ends of data transfer (CmdPend internal signal)  *)
  SDMMC_CMD_CPSMEN = $0400;  (*!<Command path state machine (CPSM) Enable bit                    *)
  SDMMC_CMD_SDIOSUSPEND = $0800;  (*!<SD I/O suspend command                                          *)
  (*****************  Bit definition for SDMMC_RESPCMD register  **************** *)

  SDMMC_RESPCMD_RESPCMD = $3F;  (*!<Response command index  *)
  (******************  Bit definition for SDMMC_RESP0 register  ***************** *)

  SDMMC_RESP0_CARDSTATUS0 = $FFFFFFFF;  (*!<Card Status  *)
  (******************  Bit definition for SDMMC_RESP1 register  ***************** *)

  SDMMC_RESP1_CARDSTATUS1 = $FFFFFFFF;  (*!<Card Status  *)
  (******************  Bit definition for SDMMC_RESP2 register  ***************** *)

  SDMMC_RESP2_CARDSTATUS2 = $FFFFFFFF;  (*!<Card Status  *)
  (******************  Bit definition for SDMMC_RESP3 register  ***************** *)

  SDMMC_RESP3_CARDSTATUS3 = $FFFFFFFF;  (*!<Card Status  *)
  (******************  Bit definition for SDMMC_RESP4 register  ***************** *)

  SDMMC_RESP4_CARDSTATUS4 = $FFFFFFFF;  (*!<Card Status  *)
  (******************  Bit definition for SDMMC_DTIMER register  **************** *)

  SDMMC_DTIMER_DATATIME = $FFFFFFFF;  (*!<Data timeout period.  *)
  (******************  Bit definition for SDMMC_DLEN register  ****************** *)

  SDMMC_DLEN_DATALENGTH = $01FFFFFF;  (*!<Data length value     *)
  (******************  Bit definition for SDMMC_DCTRL register  ***************** *)

  SDMMC_DCTRL_DTEN = $0001;  (*!<Data transfer enabled bit          *)
  SDMMC_DCTRL_DTDIR = $0002;  (*!<Data transfer direction selection  *)
  SDMMC_DCTRL_DTMODE = $0004;  (*!<Data transfer mode selection       *)
  SDMMC_DCTRL_DMAEN = $0008;  (*!<DMA enabled bit                    *)
  SDMMC_DCTRL_DBLOCKSIZE = $00F0;  (*!<DBLOCKSIZE[3:0] bits (Data block size)  *)
  SDMMC_DCTRL_DBLOCKSIZE_0 = $0010;  (*!<Bit 0  *)
  SDMMC_DCTRL_DBLOCKSIZE_1 = $0020;  (*!<Bit 1  *)
  SDMMC_DCTRL_DBLOCKSIZE_2 = $0040;  (*!<Bit 2  *)
  SDMMC_DCTRL_DBLOCKSIZE_3 = $0080;  (*!<Bit 3  *)
  SDMMC_DCTRL_RWSTART = $0100;  (*!<Read wait start          *)
  SDMMC_DCTRL_RWSTOP = $0200;  (*!<Read wait stop           *)
  SDMMC_DCTRL_RWMOD = $0400;  (*!<Read wait mode           *)
  SDMMC_DCTRL_SDIOEN = $0800;  (*!<SD I/O enable functions  *)
  (******************  Bit definition for SDMMC_DCOUNT register  **************** *)

  SDMMC_DCOUNT_DATACOUNT = $01FFFFFF;  (*!<Data count value  *)
  (******************  Bit definition for SDMMC_STA register  ******************* *)

  SDMMC_STA_CCRCFAIL = $00000001;  (*!<Command response received (CRC check failed)   *)
  SDMMC_STA_DCRCFAIL = $00000002;  (*!<Data block sent/received (CRC check failed)    *)
  SDMMC_STA_CTIMEOUT = $00000004;  (*!<Command response timeout                       *)
  SDMMC_STA_DTIMEOUT = $00000008;  (*!<Data timeout                                   *)
  SDMMC_STA_TXUNDERR = $00000010;  (*!<Transmit FIFO underrun error                   *)
  SDMMC_STA_RXOVERR = $00000020;  (*!<Received FIFO overrun error                    *)
  SDMMC_STA_CMDREND = $00000040;  (*!<Command response received (CRC check passed)   *)
  SDMMC_STA_CMDSENT = $00000080;  (*!<Command sent (no response required)            *)
  SDMMC_STA_DATAEND = $00000100;  (*!<Data end (data counter, SDIDCOUNT, is zero)    *)
  SDMMC_STA_DBCKEND = $00000400;  (*!<Data block sent/received (CRC check passed)    *)
  SDMMC_STA_CMDACT = $00000800;  (*!<Command transfer in progress                   *)
  SDMMC_STA_TXACT = $00001000;  (*!<Data transmit in progress                      *)
  SDMMC_STA_RXACT = $00002000;  (*!<Data receive in progress                       *)
  SDMMC_STA_TXFIFOHE = $00004000;  (*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO  *)
  SDMMC_STA_RXFIFOHF = $00008000;  (*!<Receive FIFO Half Full: there are at least 8 words in the FIFO  *)
  SDMMC_STA_TXFIFOF = $00010000;  (*!<Transmit FIFO full                             *)
  SDMMC_STA_RXFIFOF = $00020000;  (*!<Receive FIFO full                              *)
  SDMMC_STA_TXFIFOE = $00040000;  (*!<Transmit FIFO empty                            *)
  SDMMC_STA_RXFIFOE = $00080000;  (*!<Receive FIFO empty                             *)
  SDMMC_STA_TXDAVL = $00100000;  (*!<Data available in transmit FIFO                *)
  SDMMC_STA_RXDAVL = $00200000;  (*!<Data available in receive FIFO                 *)
  SDMMC_STA_SDIOIT = $00400000;  (*!<SDMMC interrupt received                        *)
  (*******************  Bit definition for SDMMC_ICR register  ****************** *)

  SDMMC_ICR_CCRCFAILC = $00000001;  (*!<CCRCFAIL flag clear bit  *)
  SDMMC_ICR_DCRCFAILC = $00000002;  (*!<DCRCFAIL flag clear bit  *)
  SDMMC_ICR_CTIMEOUTC = $00000004;  (*!<CTIMEOUT flag clear bit  *)
  SDMMC_ICR_DTIMEOUTC = $00000008;  (*!<DTIMEOUT flag clear bit  *)
  SDMMC_ICR_TXUNDERRC = $00000010;  (*!<TXUNDERR flag clear bit  *)
  SDMMC_ICR_RXOVERRC = $00000020;  (*!<RXOVERR flag clear bit   *)
  SDMMC_ICR_CMDRENDC = $00000040;  (*!<CMDREND flag clear bit   *)
  SDMMC_ICR_CMDSENTC = $00000080;  (*!<CMDSENT flag clear bit   *)
  SDMMC_ICR_DATAENDC = $00000100;  (*!<DATAEND flag clear bit   *)
  SDMMC_ICR_DBCKENDC = $00000400;  (*!<DBCKEND flag clear bit   *)
  SDMMC_ICR_SDIOITC = $00400000;  (*!<SDMMCIT flag clear bit    *)
  (******************  Bit definition for SDMMC_MASK register  ****************** *)

  SDMMC_MASK_CCRCFAILIE = $00000001;  (*!<Command CRC Fail Interrupt Enable           *)
  SDMMC_MASK_DCRCFAILIE = $00000002;  (*!<Data CRC Fail Interrupt Enable              *)
  SDMMC_MASK_CTIMEOUTIE = $00000004;  (*!<Command TimeOut Interrupt Enable            *)
  SDMMC_MASK_DTIMEOUTIE = $00000008;  (*!<Data TimeOut Interrupt Enable               *)
  SDMMC_MASK_TXUNDERRIE = $00000010;  (*!<Tx FIFO UnderRun Error Interrupt Enable     *)
  SDMMC_MASK_RXOVERRIE = $00000020;  (*!<Rx FIFO OverRun Error Interrupt Enable      *)
  SDMMC_MASK_CMDRENDIE = $00000040;  (*!<Command Response Received Interrupt Enable  *)
  SDMMC_MASK_CMDSENTIE = $00000080;  (*!<Command Sent Interrupt Enable               *)
  SDMMC_MASK_DATAENDIE = $00000100;  (*!<Data End Interrupt Enable                   *)
  SDMMC_MASK_DBCKENDIE = $00000400;  (*!<Data Block End Interrupt Enable             *)
  SDMMC_MASK_CMDACTIE = $00000800;  (*!<CCommand Acting Interrupt Enable            *)
  SDMMC_MASK_TXACTIE = $00001000;  (*!<Data Transmit Acting Interrupt Enable       *)
  SDMMC_MASK_RXACTIE = $00002000;  (*!<Data receive acting interrupt enabled       *)
  SDMMC_MASK_TXFIFOHEIE = $00004000;  (*!<Tx FIFO Half Empty interrupt Enable         *)
  SDMMC_MASK_RXFIFOHFIE = $00008000;  (*!<Rx FIFO Half Full interrupt Enable          *)
  SDMMC_MASK_TXFIFOFIE = $00010000;  (*!<Tx FIFO Full interrupt Enable               *)
  SDMMC_MASK_RXFIFOFIE = $00020000;  (*!<Rx FIFO Full interrupt Enable               *)
  SDMMC_MASK_TXFIFOEIE = $00040000;  (*!<Tx FIFO Empty interrupt Enable              *)
  SDMMC_MASK_RXFIFOEIE = $00080000;  (*!<Rx FIFO Empty interrupt Enable              *)
  SDMMC_MASK_TXDAVLIE = $00100000;  (*!<Data available in Tx FIFO interrupt Enable  *)
  SDMMC_MASK_RXDAVLIE = $00200000;  (*!<Data available in Rx FIFO interrupt Enable  *)
  SDMMC_MASK_SDIOITIE = $00400000;  (*!<SDMMC Mode Interrupt Received interrupt Enable  *)
  (*****************  Bit definition for SDMMC_FIFOCNT register  **************** *)

  SDMMC_FIFOCNT_FIFOCOUNT = $00FFFFFF;  (*!<Remaining number of words to be written to or read from the FIFO  *)
  (******************  Bit definition for SDMMC_FIFO register  ****************** *)

  SDMMC_FIFO_FIFODATA = $FFFFFFFF;  (*!<Receive and transmit FIFO data  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                        Serial Peripheral Interface (SPI)                    *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*******************  Bit definition for SPI_CR1 register  ******************* *)

  SPI_CR1_CPHA = $00000001;  (*!< Clock Phase                         *)
  SPI_CR1_CPOL = $00000002;  (*!< Clock Polarity                      *)
  SPI_CR1_MSTR = $00000004;  (*!< Master Selection                    *)
  SPI_CR1_BR = $00000038;  (*!< BR[2:0] bits (Baud Rate Control)    *)
  SPI_CR1_BR_0 = $00000008;  (*!< Bit 0  *)
  SPI_CR1_BR_1 = $00000010;  (*!< Bit 1  *)
  SPI_CR1_BR_2 = $00000020;  (*!< Bit 2  *)
  SPI_CR1_SPE = $00000040;  (*!< SPI Enable                           *)
  SPI_CR1_LSBFIRST = $00000080;  (*!< Frame Format                         *)
  SPI_CR1_SSI = $00000100;  (*!< Internal slave select                *)
  SPI_CR1_SSM = $00000200;  (*!< Software slave management            *)
  SPI_CR1_RXONLY = $00000400;  (*!< Receive only                         *)
  SPI_CR1_CRCL = $00000800;  (*!< CRC Length                           *)
  SPI_CR1_CRCNEXT = $00001000;  (*!< Transmit CRC next                    *)
  SPI_CR1_CRCEN = $00002000;  (*!< Hardware CRC calculation enable      *)
  SPI_CR1_BIDIOE = $00004000;  (*!< Output enable in bidirectional mode  *)
  SPI_CR1_BIDIMODE = $00008000;  (*!< Bidirectional data mode enable       *)
  (*******************  Bit definition for SPI_CR2 register  ******************* *)

  SPI_CR2_RXDMAEN = $00000001;  (*!< Rx Buffer DMA Enable                  *)
  SPI_CR2_TXDMAEN = $00000002;  (*!< Tx Buffer DMA Enable                  *)
  SPI_CR2_SSOE = $00000004;  (*!< SS Output Enable                      *)
  SPI_CR2_NSSP = $00000008;  (*!< NSS pulse management Enable           *)
  SPI_CR2_FRF = $00000010;  (*!< Frame Format Enable                   *)
  SPI_CR2_ERRIE = $00000020;  (*!< Error Interrupt Enable                *)
  SPI_CR2_RXNEIE = $00000040;  (*!< RX buffer Not Empty Interrupt Enable  *)
  SPI_CR2_TXEIE = $00000080;  (*!< Tx buffer Empty Interrupt Enable      *)
  SPI_CR2_DS = $00000F00;  (*!< DS[3:0] Data Size                     *)
  SPI_CR2_DS_0 = $00000100;  (*!< Bit 0  *)
  SPI_CR2_DS_1 = $00000200;  (*!< Bit 1  *)
  SPI_CR2_DS_2 = $00000400;  (*!< Bit 2  *)
  SPI_CR2_DS_3 = $00000800;  (*!< Bit 3  *)
  SPI_CR2_FRXTH = $00001000;  (*!< FIFO reception Threshold            *)
  SPI_CR2_LDMARX = $00002000;  (*!< Last DMA transfer for reception     *)
  SPI_CR2_LDMATX = $00004000;  (*!< Last DMA transfer for transmission  *)
  (********************  Bit definition for SPI_SR register  ******************* *)

  SPI_SR_RXNE = $00000001;  (*!< Receive buffer Not Empty   *)
  SPI_SR_TXE = $00000002;  (*!< Transmit buffer Empty      *)
  SPI_SR_CHSIDE = $00000004;  (*!< Channel side               *)
  SPI_SR_UDR = $00000008;  (*!< Underrun flag              *)
  SPI_SR_CRCERR = $00000010;  (*!< CRC Error flag             *)
  SPI_SR_MODF = $00000020;  (*!< Mode fault                 *)
  SPI_SR_OVR = $00000040;  (*!< Overrun flag               *)
  SPI_SR_BSY = $00000080;  (*!< Busy flag                  *)
  SPI_SR_FRE = $00000100;  (*!< TI frame format error      *)
  SPI_SR_FRLVL = $00000600;  (*!< FIFO Reception Level       *)
  SPI_SR_FRLVL_0 = $00000200;  (*!< Bit 0  *)
  SPI_SR_FRLVL_1 = $00000400;  (*!< Bit 1  *)
  SPI_SR_FTLVL = $00001800;  (*!< FIFO Transmission Level    *)
  SPI_SR_FTLVL_0 = $00000800;  (*!< Bit 0  *)
  SPI_SR_FTLVL_1 = $00001000;  (*!< Bit 1  *)
  (********************  Bit definition for SPI_DR register  ******************* *)

  SPI_DR_DR = $FFFF;  (*!< Data Register  *)
  (*******************  Bit definition for SPI_CRCPR register  ***************** *)

  SPI_CRCPR_CRCPOLY = $FFFF;  (*!< CRC polynomial register  *)
  (******************  Bit definition for SPI_RXCRCR register  ***************** *)

  SPI_RXCRCR_RXCRC = $FFFF;  (*!< Rx CRC Register  *)
  (******************  Bit definition for SPI_TXCRCR register  ***************** *)

  SPI_TXCRCR_TXCRC = $FFFF;  (*!< Tx CRC Register  *)
  (******************  Bit definition for SPI_I2SCFGR register  **************** *)

  SPI_I2SCFGR_CHLEN = $00000001;  (*!<Channel length (number of bits per audio channel)  *)
  SPI_I2SCFGR_DATLEN = $00000006;  (*!<DATLEN[1:0] bits (Data length to be transferred)   *)
  SPI_I2SCFGR_DATLEN_0 = $00000002;  (*!<Bit 0  *)
  SPI_I2SCFGR_DATLEN_1 = $00000004;  (*!<Bit 1  *)
  SPI_I2SCFGR_CKPOL = $00000008;  (*!<steady state clock polarity                        *)
  SPI_I2SCFGR_I2SSTD = $00000030;  (*!<I2SSTD[1:0] bits (I2S standard selection)          *)
  SPI_I2SCFGR_I2SSTD_0 = $00000010;  (*!<Bit 0  *)
  SPI_I2SCFGR_I2SSTD_1 = $00000020;  (*!<Bit 1  *)
  SPI_I2SCFGR_PCMSYNC = $00000080;  (*!<PCM frame synchronization                          *)
  SPI_I2SCFGR_I2SCFG = $00000300;  (*!<I2SCFG[1:0] bits (I2S configuration mode)          *)
  SPI_I2SCFGR_I2SCFG_0 = $00000100;  (*!<Bit 0  *)
  SPI_I2SCFGR_I2SCFG_1 = $00000200;  (*!<Bit 1  *)
  SPI_I2SCFGR_I2SE = $00000400;  (*!<I2S Enable                                         *)
  SPI_I2SCFGR_I2SMOD = $00000800;  (*!<I2S mode selection                                 *)
  SPI_I2SCFGR_ASTRTEN = $00001000;  (*!<Asynchronous start enable                         *)
  (******************  Bit definition for SPI_I2SPR register  ****************** *)

  SPI_I2SPR_I2SDIV = $00FF;  (*!<I2S Linear prescaler          *)
  SPI_I2SPR_ODD = $0100;  (*!<Odd factor for the prescaler  *)
  SPI_I2SPR_MCKOE = $0200;  (*!<Master Clock Output Enable    *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                                 SYSCFG                                      *)

  (*                                                                             *)

  (***************************************************************************** *)

  (******************  Bit definition for SYSCFG_MEMRMP register  ************** *)

  SYSCFG_MEMRMP_MEM_BOOT = $00000001;  (*!< Boot information after Reset  *)
  SYSCFG_MEMRMP_SWP_FMC = $00000C00;  (*!< FMC Memory Mapping swapping  *)
  SYSCFG_MEMRMP_SWP_FMC_0 = $00000400;
  SYSCFG_MEMRMP_SWP_FMC_1 = $00000800;
  (******************  Bit definition for SYSCFG_PMC register  ***************** *)

  SYSCFG_PMC_ADCxDC2 = $00070000;  (*!< Refer to AN4073 on how to use this bit   *)
  SYSCFG_PMC_ADC1DC2 = $00010000;  (*!< Refer to AN4073 on how to use this bit   *)
  SYSCFG_PMC_ADC2DC2 = $00020000;  (*!< Refer to AN4073 on how to use this bit   *)
  SYSCFG_PMC_ADC3DC2 = $00040000;  (*!< Refer to AN4073 on how to use this bit   *)
  SYSCFG_PMC_MII_RMII_SEL = $00800000;  (*!<Ethernet PHY interface selection  *)
  (*****************  Bit definition for SYSCFG_EXTICR1 register  ************** *)

  SYSCFG_EXTICR1_EXTI0 = $000F;  (*!<EXTI 0 configuration  *)
  SYSCFG_EXTICR1_EXTI1 = $00F0;  (*!<EXTI 1 configuration  *)
  SYSCFG_EXTICR1_EXTI2 = $0F00;  (*!<EXTI 2 configuration  *)
  SYSCFG_EXTICR1_EXTI3 = $F000;  (*!<EXTI 3 configuration  *)
  (**
  * @brief   EXTI0 configuration
   *)

  SYSCFG_EXTICR1_EXTI0_PA = $0000;  (*!<PA[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PB = $0001;  (*!<PB[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PC = $0002;  (*!<PC[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PD = $0003;  (*!<PD[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PE = $0004;  (*!<PE[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PF = $0005;  (*!<PF[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PG = $0006;  (*!<PG[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PH = $0007;  (*!<PH[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PI = $0008;  (*!<PI[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PJ = $0009;  (*!<PJ[0] pin  *)
  SYSCFG_EXTICR1_EXTI0_PK = $000A;  (*!<PK[0] pin  *)
  (**
  * @brief   EXTI1 configuration
   *)

  SYSCFG_EXTICR1_EXTI1_PA = $0000;  (*!<PA[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PB = $0010;  (*!<PB[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PC = $0020;  (*!<PC[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PD = $0030;  (*!<PD[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PE = $0040;  (*!<PE[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PF = $0050;  (*!<PF[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PG = $0060;  (*!<PG[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PH = $0070;  (*!<PH[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PI = $0080;  (*!<PI[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PJ = $0090;  (*!<PJ[1] pin  *)
  SYSCFG_EXTICR1_EXTI1_PK = $00A0;  (*!<PK[1] pin  *)
  (**
  * @brief   EXTI2 configuration
   *)

  SYSCFG_EXTICR1_EXTI2_PA = $0000;  (*!<PA[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PB = $0100;  (*!<PB[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PC = $0200;  (*!<PC[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PD = $0300;  (*!<PD[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PE = $0400;  (*!<PE[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PF = $0500;  (*!<PF[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PG = $0600;  (*!<PG[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PH = $0700;  (*!<PH[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PI = $0800;  (*!<PI[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PJ = $0900;  (*!<PJ[2] pin  *)
  SYSCFG_EXTICR1_EXTI2_PK = $0A00;  (*!<PK[2] pin  *)
  (**
  * @brief   EXTI3 configuration
   *)

  SYSCFG_EXTICR1_EXTI3_PA = $0000;  (*!<PA[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PB = $1000;  (*!<PB[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PC = $2000;  (*!<PC[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PD = $3000;  (*!<PD[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PE = $4000;  (*!<PE[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PF = $5000;  (*!<PF[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PG = $6000;  (*!<PG[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PH = $7000;  (*!<PH[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PI = $8000;  (*!<PI[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PJ = $9000;  (*!<PJ[3] pin  *)
  SYSCFG_EXTICR1_EXTI3_PK = $A000;  (*!<PK[3] pin  *)
  (*****************  Bit definition for SYSCFG_EXTICR2 register  ************** *)

  SYSCFG_EXTICR2_EXTI4 = $000F;  (*!<EXTI 4 configuration  *)
  SYSCFG_EXTICR2_EXTI5 = $00F0;  (*!<EXTI 5 configuration  *)
  SYSCFG_EXTICR2_EXTI6 = $0F00;  (*!<EXTI 6 configuration  *)
  SYSCFG_EXTICR2_EXTI7 = $F000;  (*!<EXTI 7 configuration  *)
  (**
  * @brief   EXTI4 configuration
   *)

  SYSCFG_EXTICR2_EXTI4_PA = $0000;  (*!<PA[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PB = $0001;  (*!<PB[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PC = $0002;  (*!<PC[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PD = $0003;  (*!<PD[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PE = $0004;  (*!<PE[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PF = $0005;  (*!<PF[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PG = $0006;  (*!<PG[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PH = $0007;  (*!<PH[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PI = $0008;  (*!<PI[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PJ = $0009;  (*!<PJ[4] pin  *)
  SYSCFG_EXTICR2_EXTI4_PK = $000A;  (*!<PK[4] pin  *)
  (**
  * @brief   EXTI5 configuration
   *)

  SYSCFG_EXTICR2_EXTI5_PA = $0000;  (*!<PA[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PB = $0010;  (*!<PB[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PC = $0020;  (*!<PC[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PD = $0030;  (*!<PD[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PE = $0040;  (*!<PE[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PF = $0050;  (*!<PF[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PG = $0060;  (*!<PG[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PH = $0070;  (*!<PH[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PI = $0080;  (*!<PI[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PJ = $0090;  (*!<PJ[5] pin  *)
  SYSCFG_EXTICR2_EXTI5_PK = $00A0;  (*!<PK[5] pin  *)
  (**
  * @brief   EXTI6 configuration
   *)

  SYSCFG_EXTICR2_EXTI6_PA = $0000;  (*!<PA[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PB = $0100;  (*!<PB[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PC = $0200;  (*!<PC[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PD = $0300;  (*!<PD[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PE = $0400;  (*!<PE[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PF = $0500;  (*!<PF[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PG = $0600;  (*!<PG[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PH = $0700;  (*!<PH[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PI = $0800;  (*!<PI[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PJ = $0900;  (*!<PJ[6] pin  *)
  SYSCFG_EXTICR2_EXTI6_PK = $0A00;  (*!<PK[6] pin  *)
  (**
  * @brief   EXTI7 configuration
   *)

  SYSCFG_EXTICR2_EXTI7_PA = $0000;  (*!<PA[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PB = $1000;  (*!<PB[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PC = $2000;  (*!<PC[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PD = $3000;  (*!<PD[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PE = $4000;  (*!<PE[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PF = $5000;  (*!<PF[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PG = $6000;  (*!<PG[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PH = $7000;  (*!<PH[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PI = $8000;  (*!<PI[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PJ = $9000;  (*!<PJ[7] pin  *)
  SYSCFG_EXTICR2_EXTI7_PK = $A000;  (*!<PK[7] pin  *)
  (*****************  Bit definition for SYSCFG_EXTICR3 register  ************** *)

  SYSCFG_EXTICR3_EXTI8 = $000F;  (*!<EXTI 8 configuration  *)
  SYSCFG_EXTICR3_EXTI9 = $00F0;  (*!<EXTI 9 configuration  *)
  SYSCFG_EXTICR3_EXTI10 = $0F00;  (*!<EXTI 10 configuration  *)
  SYSCFG_EXTICR3_EXTI11 = $F000;  (*!<EXTI 11 configuration  *)
  (**
  * @brief   EXTI8 configuration
   *)

  SYSCFG_EXTICR3_EXTI8_PA = $0000;  (*!<PA[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PB = $0001;  (*!<PB[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PC = $0002;  (*!<PC[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PD = $0003;  (*!<PD[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PE = $0004;  (*!<PE[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PF = $0005;  (*!<PF[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PG = $0006;  (*!<PG[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PH = $0007;  (*!<PH[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PI = $0008;  (*!<PI[8] pin  *)
  SYSCFG_EXTICR3_EXTI8_PJ = $0009;  (*!<PJ[8] pin  *)
  (**
  * @brief   EXTI9 configuration
   *)

  SYSCFG_EXTICR3_EXTI9_PA = $0000;  (*!<PA[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PB = $0010;  (*!<PB[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PC = $0020;  (*!<PC[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PD = $0030;  (*!<PD[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PE = $0040;  (*!<PE[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PF = $0050;  (*!<PF[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PG = $0060;  (*!<PG[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PH = $0070;  (*!<PH[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PI = $0080;  (*!<PI[9] pin  *)
  SYSCFG_EXTICR3_EXTI9_PJ = $0090;  (*!<PJ[9] pin  *)
  (**
  * @brief   EXTI10 configuration
   *)

  SYSCFG_EXTICR3_EXTI10_PA = $0000;  (*!<PA[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PB = $0100;  (*!<PB[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PC = $0200;  (*!<PC[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PD = $0300;  (*!<PD[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PE = $0400;  (*!<PE[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PF = $0500;  (*!<PF[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PG = $0600;  (*!<PG[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PH = $0700;  (*!<PH[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PI = $0800;  (*!<PI[10] pin  *)
  SYSCFG_EXTICR3_EXTI10_PJ = $0900;  (*!<PJ[10] pin  *)
  (**
  * @brief   EXTI11 configuration
   *)

  SYSCFG_EXTICR3_EXTI11_PA = $0000;  (*!<PA[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PB = $1000;  (*!<PB[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PC = $2000;  (*!<PC[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PD = $3000;  (*!<PD[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PE = $4000;  (*!<PE[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PF = $5000;  (*!<PF[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PG = $6000;  (*!<PG[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PH = $7000;  (*!<PH[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PI = $8000;  (*!<PI[11] pin  *)
  SYSCFG_EXTICR3_EXTI11_PJ = $9000;  (*!<PJ[11] pin  *)
  (*****************  Bit definition for SYSCFG_EXTICR4 register  ************** *)

  SYSCFG_EXTICR4_EXTI12 = $000F;  (*!<EXTI 12 configuration  *)
  SYSCFG_EXTICR4_EXTI13 = $00F0;  (*!<EXTI 13 configuration  *)
  SYSCFG_EXTICR4_EXTI14 = $0F00;  (*!<EXTI 14 configuration  *)
  SYSCFG_EXTICR4_EXTI15 = $F000;  (*!<EXTI 15 configuration  *)
  (**
  * @brief   EXTI12 configuration
   *)

  SYSCFG_EXTICR4_EXTI12_PA = $0000;  (*!<PA[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PB = $0001;  (*!<PB[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PC = $0002;  (*!<PC[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PD = $0003;  (*!<PD[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PE = $0004;  (*!<PE[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PF = $0005;  (*!<PF[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PG = $0006;  (*!<PG[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PH = $0007;  (*!<PH[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PI = $0008;  (*!<PI[12] pin  *)
  SYSCFG_EXTICR4_EXTI12_PJ = $0009;  (*!<PJ[12] pin  *)
  (**
  * @brief   EXTI13 configuration
   *)

  SYSCFG_EXTICR4_EXTI13_PA = $0000;  (*!<PA[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PB = $0010;  (*!<PB[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PC = $0020;  (*!<PC[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PD = $0030;  (*!<PD[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PE = $0040;  (*!<PE[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PF = $0050;  (*!<PF[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PG = $0060;  (*!<PG[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PH = $0070;  (*!<PH[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PI = $0008;  (*!<PI[13] pin  *)
  SYSCFG_EXTICR4_EXTI13_PJ = $0009;  (*!<PJ[13] pin  *)
  (**
  * @brief   EXTI14 configuration
   *)

  SYSCFG_EXTICR4_EXTI14_PA = $0000;  (*!<PA[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PB = $0100;  (*!<PB[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PC = $0200;  (*!<PC[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PD = $0300;  (*!<PD[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PE = $0400;  (*!<PE[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PF = $0500;  (*!<PF[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PG = $0600;  (*!<PG[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PH = $0700;  (*!<PH[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PI = $0800;  (*!<PI[14] pin  *)
  SYSCFG_EXTICR4_EXTI14_PJ = $0900;  (*!<PJ[14] pin  *)
  (**
  * @brief   EXTI15 configuration
   *)

  SYSCFG_EXTICR4_EXTI15_PA = $0000;  (*!<PA[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PB = $1000;  (*!<PB[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PC = $2000;  (*!<PC[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PD = $3000;  (*!<PD[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PE = $4000;  (*!<PE[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PF = $5000;  (*!<PF[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PG = $6000;  (*!<PG[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PH = $7000;  (*!<PH[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PI = $8000;  (*!<PI[15] pin  *)
  SYSCFG_EXTICR4_EXTI15_PJ = $9000;  (*!<PJ[15] pin  *)
  (******************  Bit definition for SYSCFG_CMPCR register  *************** *)

  SYSCFG_CMPCR_CMP_PD = $00000001;  (*!<Compensation cell power-down  *)
  SYSCFG_CMPCR_READY = $00000100;  (*!<Compensation cell ready flag *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                                    TIM                                      *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*******************  Bit definition for TIM_CR1 register  ******************* *)

  TIM_CR1_CEN = $0001;  (*!<Counter enable         *)
  TIM_CR1_UDIS = $0002;  (*!<Update disable         *)
  TIM_CR1_URS = $0004;  (*!<Update request source  *)
  TIM_CR1_OPM = $0008;  (*!<One pulse mode         *)
  TIM_CR1_DIR = $0010;  (*!<Direction              *)
  TIM_CR1_CMS = $0060;  (*!<CMS[1:0] bits (Center-aligned mode selection)  *)
  TIM_CR1_CMS_0 = $0020;  (*!<Bit 0  *)
  TIM_CR1_CMS_1 = $0040;  (*!<Bit 1  *)
  TIM_CR1_ARPE = $0080;  (*!<Auto-reload preload enable      *)
  TIM_CR1_CKD = $0300;  (*!<CKD[1:0] bits (clock division)  *)
  TIM_CR1_CKD_0 = $0100;  (*!<Bit 0  *)
  TIM_CR1_CKD_1 = $0200;  (*!<Bit 1  *)
  TIM_CR1_UIFREMAP = $0800;  (*!<UIF status bit  *)
  (*******************  Bit definition for TIM_CR2 register  ******************* *)

  TIM_CR2_CCPC = $00000001;  (*!<Capture/Compare Preloaded Control         *)
  TIM_CR2_CCUS = $00000004;  (*!<Capture/Compare Control Update Selection  *)
  TIM_CR2_CCDS = $00000008;  (*!<Capture/Compare DMA Selection             *)
  TIM_CR2_OIS5 = $00010000;  (*!<Output Idle state 4 (OC4 output)  *)
  TIM_CR2_OIS6 = $00040000;  (*!<Output Idle state 4 (OC4 output)  *)
  TIM_CR2_MMS = $0070;  (*!<MMS[2:0] bits (Master Mode Selection)  *)
  TIM_CR2_MMS_0 = $0010;  (*!<Bit 0  *)
  TIM_CR2_MMS_1 = $0020;  (*!<Bit 1  *)
  TIM_CR2_MMS_2 = $0040;  (*!<Bit 2  *)
  TIM_CR2_MMS2 = $00F00000;  (*!<MMS[2:0] bits (Master Mode Selection)  *)
  TIM_CR2_MMS2_0 = $00100000;  (*!<Bit 0  *)
  TIM_CR2_MMS2_1 = $00200000;  (*!<Bit 1  *)
  TIM_CR2_MMS2_2 = $00400000;  (*!<Bit 2  *)
  TIM_CR2_MMS2_3 = $00800000;  (*!<Bit 2  *)
  TIM_CR2_TI1S = $0080;  (*!<TI1 Selection  *)
  TIM_CR2_OIS1 = $0100;  (*!<Output Idle state 1 (OC1 output)   *)
  TIM_CR2_OIS1N = $0200;  (*!<Output Idle state 1 (OC1N output)  *)
  TIM_CR2_OIS2 = $0400;  (*!<Output Idle state 2 (OC2 output)   *)
  TIM_CR2_OIS2N = $0800;  (*!<Output Idle state 2 (OC2N output)  *)
  TIM_CR2_OIS3 = $1000;  (*!<Output Idle state 3 (OC3 output)   *)
  TIM_CR2_OIS3N = $2000;  (*!<Output Idle state 3 (OC3N output)  *)
  TIM_CR2_OIS4 = $4000;  (*!<Output Idle state 4 (OC4 output)   *)
  (*******************  Bit definition for TIM_SMCR register  ****************** *)

  TIM_SMCR_SMS = $00010007;  (*!<SMS[2:0] bits (Slave mode selection)     *)
  TIM_SMCR_SMS_0 = $00000001;  (*!<Bit 0  *)
  TIM_SMCR_SMS_1 = $00000002;  (*!<Bit 1  *)
  TIM_SMCR_SMS_2 = $00000004;  (*!<Bit 2  *)
  TIM_SMCR_SMS_3 = $00010000;  (*!<Bit 3  *)
  TIM_SMCR_OCCS = $00000008;  (*!< OCREF clear selection  *)
  TIM_SMCR_TS = $0070;  (*!<TS[2:0] bits (Trigger selection)         *)
  TIM_SMCR_TS_0 = $0010;  (*!<Bit 0  *)
  TIM_SMCR_TS_1 = $0020;  (*!<Bit 1  *)
  TIM_SMCR_TS_2 = $0040;  (*!<Bit 2  *)
  TIM_SMCR_MSM = $0080;  (*!<Master/slave mode                        *)
  TIM_SMCR_ETF = $0F00;  (*!<ETF[3:0] bits (External trigger filter)  *)
  TIM_SMCR_ETF_0 = $0100;  (*!<Bit 0  *)
  TIM_SMCR_ETF_1 = $0200;  (*!<Bit 1  *)
  TIM_SMCR_ETF_2 = $0400;  (*!<Bit 2  *)
  TIM_SMCR_ETF_3 = $0800;  (*!<Bit 3  *)
  TIM_SMCR_ETPS = $3000;  (*!<ETPS[1:0] bits (External trigger prescaler)  *)
  TIM_SMCR_ETPS_0 = $1000;  (*!<Bit 0  *)
  TIM_SMCR_ETPS_1 = $2000;  (*!<Bit 1  *)
  TIM_SMCR_ECE = $4000;  (*!<External clock enable      *)
  TIM_SMCR_ETP = $8000;  (*!<External trigger polarity  *)
  (*******************  Bit definition for TIM_DIER register  ****************** *)

  TIM_DIER_UIE = $0001;  (*!<Update interrupt enable  *)
  TIM_DIER_CC1IE = $0002;  (*!<Capture/Compare 1 interrupt enable    *)
  TIM_DIER_CC2IE = $0004;  (*!<Capture/Compare 2 interrupt enable    *)
  TIM_DIER_CC3IE = $0008;  (*!<Capture/Compare 3 interrupt enable    *)
  TIM_DIER_CC4IE = $0010;  (*!<Capture/Compare 4 interrupt enable    *)
  TIM_DIER_COMIE = $0020;  (*!<COM interrupt enable                  *)
  TIM_DIER_TIE = $0040;  (*!<Trigger interrupt enable              *)
  TIM_DIER_BIE = $0080;  (*!<Break interrupt enable                *)
  TIM_DIER_UDE = $0100;  (*!<Update DMA request enable             *)
  TIM_DIER_CC1DE = $0200;  (*!<Capture/Compare 1 DMA request enable  *)
  TIM_DIER_CC2DE = $0400;  (*!<Capture/Compare 2 DMA request enable  *)
  TIM_DIER_CC3DE = $0800;  (*!<Capture/Compare 3 DMA request enable  *)
  TIM_DIER_CC4DE = $1000;  (*!<Capture/Compare 4 DMA request enable  *)
  TIM_DIER_COMDE = $2000;  (*!<COM DMA request enable                *)
  TIM_DIER_TDE = $4000;  (*!<Trigger DMA request enable            *)
  (********************  Bit definition for TIM_SR register  ******************* *)

  TIM_SR_UIF = $0001;  (*!<Update interrupt Flag               *)
  TIM_SR_CC1IF = $0002;  (*!<Capture/Compare 1 interrupt Flag    *)
  TIM_SR_CC2IF = $0004;  (*!<Capture/Compare 2 interrupt Flag    *)
  TIM_SR_CC3IF = $0008;  (*!<Capture/Compare 3 interrupt Flag    *)
  TIM_SR_CC4IF = $0010;  (*!<Capture/Compare 4 interrupt Flag    *)
  TIM_SR_COMIF = $0020;  (*!<COM interrupt Flag                  *)
  TIM_SR_TIF = $0040;  (*!<Trigger interrupt Flag              *)
  TIM_SR_BIF = $0080;  (*!<Break interrupt Flag                *)
  TIM_SR_B2IF = $0100;  (*!<Break2 interrupt Flag                *)
  TIM_SR_CC1OF = $0200;  (*!<Capture/Compare 1 Overcapture Flag  *)
  TIM_SR_CC2OF = $0400;  (*!<Capture/Compare 2 Overcapture Flag  *)
  TIM_SR_CC3OF = $0800;  (*!<Capture/Compare 3 Overcapture Flag  *)
  TIM_SR_CC4OF = $1000;  (*!<Capture/Compare 4 Overcapture Flag  *)
  (*******************  Bit definition for TIM_EGR register  ******************* *)

  TIM_EGR_UG = $00000001;  (*!<Update Generation                          *)
  TIM_EGR_CC1G = $00000002;  (*!<Capture/Compare 1 Generation               *)
  TIM_EGR_CC2G = $00000004;  (*!<Capture/Compare 2 Generation               *)
  TIM_EGR_CC3G = $00000008;  (*!<Capture/Compare 3 Generation               *)
  TIM_EGR_CC4G = $00000010;  (*!<Capture/Compare 4 Generation               *)
  TIM_EGR_COMG = $00000020;  (*!<Capture/Compare Control Update Generation  *)
  TIM_EGR_TG = $00000040;  (*!<Trigger Generation                         *)
  TIM_EGR_BG = $00000080;  (*!<Break Generation                           *)
  TIM_EGR_B2G = $00000100;  (*!<Break2 Generation                           *)
  (******************  Bit definition for TIM_CCMR1 register  ****************** *)

  TIM_CCMR1_CC1S = $00000003;  (*!<CC1S[1:0] bits (Capture/Compare 1 Selection)  *)
  TIM_CCMR1_CC1S_0 = $00000001;  (*!<Bit 0  *)
  TIM_CCMR1_CC1S_1 = $00000002;  (*!<Bit 1  *)
  TIM_CCMR1_OC1FE = $00000004;  (*!<Output Compare 1 Fast enable                  *)
  TIM_CCMR1_OC1PE = $00000008;  (*!<Output Compare 1 Preload enable               *)
  TIM_CCMR1_OC1M = $00010070;  (*!<OC1M[2:0] bits (Output Compare 1 Mode)        *)
  TIM_CCMR1_OC1M_0 = $00000010;  (*!<Bit 0  *)
  TIM_CCMR1_OC1M_1 = $00000020;  (*!<Bit 1  *)
  TIM_CCMR1_OC1M_2 = $00000040;  (*!<Bit 2  *)
  TIM_CCMR1_OC1M_3 = $00010000;  (*!<Bit 3  *)
  TIM_CCMR1_OC1CE = $00000080;  (*!<Output Compare 1Clear Enable                  *)
  TIM_CCMR1_CC2S = $00000300;  (*!<CC2S[1:0] bits (Capture/Compare 2 Selection)  *)
  TIM_CCMR1_CC2S_0 = $00000100;  (*!<Bit 0  *)
  TIM_CCMR1_CC2S_1 = $00000200;  (*!<Bit 1  *)
  TIM_CCMR1_OC2FE = $00000400;  (*!<Output Compare 2 Fast enable                  *)
  TIM_CCMR1_OC2PE = $00000800;  (*!<Output Compare 2 Preload enable               *)
  TIM_CCMR1_OC2M = $01007000;  (*!<OC2M[2:0] bits (Output Compare 2 Mode)        *)
  TIM_CCMR1_OC2M_0 = $00001000;  (*!<Bit 0  *)
  TIM_CCMR1_OC2M_1 = $00002000;  (*!<Bit 1  *)
  TIM_CCMR1_OC2M_2 = $00004000;  (*!<Bit 2  *)
  TIM_CCMR1_OC2M_3 = $01000000;  (*!<Bit 3  *)
  TIM_CCMR1_OC2CE = $00008000;  (*!<Output Compare 2 Clear Enable  *)
  (*---------------------------------------------------------------------------- *)

  TIM_CCMR1_IC1PSC = $000C;  (*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler)  *)
  TIM_CCMR1_IC1PSC_0 = $0004;  (*!<Bit 0  *)
  TIM_CCMR1_IC1PSC_1 = $0008;  (*!<Bit 1  *)
  TIM_CCMR1_IC1F = $00F0;  (*!<IC1F[3:0] bits (Input Capture 1 Filter)       *)
  TIM_CCMR1_IC1F_0 = $0010;  (*!<Bit 0  *)
  TIM_CCMR1_IC1F_1 = $0020;  (*!<Bit 1  *)
  TIM_CCMR1_IC1F_2 = $0040;  (*!<Bit 2  *)
  TIM_CCMR1_IC1F_3 = $0080;  (*!<Bit 3  *)
  TIM_CCMR1_IC2PSC = $0C00;  (*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler)   *)
  TIM_CCMR1_IC2PSC_0 = $0400;  (*!<Bit 0  *)
  TIM_CCMR1_IC2PSC_1 = $0800;  (*!<Bit 1  *)
  TIM_CCMR1_IC2F = $F000;  (*!<IC2F[3:0] bits (Input Capture 2 Filter)        *)
  TIM_CCMR1_IC2F_0 = $1000;  (*!<Bit 0  *)
  TIM_CCMR1_IC2F_1 = $2000;  (*!<Bit 1  *)
  TIM_CCMR1_IC2F_2 = $4000;  (*!<Bit 2  *)
  TIM_CCMR1_IC2F_3 = $8000;  (*!<Bit 3  *)
  (******************  Bit definition for TIM_CCMR2 register  ****************** *)

  TIM_CCMR2_CC3S = $00000003;  (*!<CC3S[1:0] bits (Capture/Compare 3 Selection)   *)
  TIM_CCMR2_CC3S_0 = $00000001;  (*!<Bit 0  *)
  TIM_CCMR2_CC3S_1 = $00000002;  (*!<Bit 1  *)
  TIM_CCMR2_OC3FE = $00000004;  (*!<Output Compare 3 Fast enable            *)
  TIM_CCMR2_OC3PE = $00000008;  (*!<Output Compare 3 Preload enable         *)
  TIM_CCMR2_OC3M = $00010070;  (*!<OC3M[2:0] bits (Output Compare 3 Mode)  *)
  TIM_CCMR2_OC3M_0 = $00000010;  (*!<Bit 0  *)
  TIM_CCMR2_OC3M_1 = $00000020;  (*!<Bit 1  *)
  TIM_CCMR2_OC3M_2 = $00000040;  (*!<Bit 2  *)
  TIM_CCMR2_OC3M_3 = $00010000;  (*!<Bit 3  *)
  TIM_CCMR2_OC3CE = $00000080;  (*!<Output Compare 3 Clear Enable  *)
  TIM_CCMR2_CC4S = $00000300;  (*!<CC4S[1:0] bits (Capture/Compare 4 Selection)  *)
  TIM_CCMR2_CC4S_0 = $00000100;  (*!<Bit 0  *)
  TIM_CCMR2_CC4S_1 = $00000200;  (*!<Bit 1  *)
  TIM_CCMR2_OC4FE = $00000400;  (*!<Output Compare 4 Fast enable     *)
  TIM_CCMR2_OC4PE = $00000800;  (*!<Output Compare 4 Preload enable  *)
  TIM_CCMR2_OC4M = $01007000;  (*!<OC4M[2:0] bits (Output Compare 4 Mode)  *)
  TIM_CCMR2_OC4M_0 = $00001000;  (*!<Bit 0  *)
  TIM_CCMR2_OC4M_1 = $00002000;  (*!<Bit 1  *)
  TIM_CCMR2_OC4M_2 = $00004000;  (*!<Bit 2  *)
  TIM_CCMR2_OC4M_3 = $01000000;  (*!<Bit 3  *)
  TIM_CCMR2_OC4CE = $8000;  (*!<Output Compare 4 Clear Enable  *)
  (*---------------------------------------------------------------------------- *)

  TIM_CCMR2_IC3PSC = $000C;  (*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler)  *)
  TIM_CCMR2_IC3PSC_0 = $0004;  (*!<Bit 0  *)
  TIM_CCMR2_IC3PSC_1 = $0008;  (*!<Bit 1  *)
  TIM_CCMR2_IC3F = $00F0;  (*!<IC3F[3:0] bits (Input Capture 3 Filter)  *)
  TIM_CCMR2_IC3F_0 = $0010;  (*!<Bit 0  *)
  TIM_CCMR2_IC3F_1 = $0020;  (*!<Bit 1  *)
  TIM_CCMR2_IC3F_2 = $0040;  (*!<Bit 2  *)
  TIM_CCMR2_IC3F_3 = $0080;  (*!<Bit 3  *)
  TIM_CCMR2_IC4PSC = $0C00;  (*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler)  *)
  TIM_CCMR2_IC4PSC_0 = $0400;  (*!<Bit 0  *)
  TIM_CCMR2_IC4PSC_1 = $0800;  (*!<Bit 1  *)
  TIM_CCMR2_IC4F = $F000;  (*!<IC4F[3:0] bits (Input Capture 4 Filter)  *)
  TIM_CCMR2_IC4F_0 = $1000;  (*!<Bit 0  *)
  TIM_CCMR2_IC4F_1 = $2000;  (*!<Bit 1  *)
  TIM_CCMR2_IC4F_2 = $4000;  (*!<Bit 2  *)
  TIM_CCMR2_IC4F_3 = $8000;  (*!<Bit 3  *)
  (*******************  Bit definition for TIM_CCER register  ****************** *)

  TIM_CCER_CC1E = $00000001;  (*!<Capture/Compare 1 output enable  *)
  TIM_CCER_CC1P = $00000002;  (*!<Capture/Compare 1 output Polarity  *)
  TIM_CCER_CC1NE = $00000004;  (*!<Capture/Compare 1 Complementary output enable  *)
  TIM_CCER_CC1NP = $00000008;  (*!<Capture/Compare 1 Complementary output Polarity  *)
  TIM_CCER_CC2E = $00000010;  (*!<Capture/Compare 2 output enable  *)
  TIM_CCER_CC2P = $00000020;  (*!<Capture/Compare 2 output Polarity  *)
  TIM_CCER_CC2NE = $00000040;  (*!<Capture/Compare 2 Complementary output enable  *)
  TIM_CCER_CC2NP = $00000080;  (*!<Capture/Compare 2 Complementary output Polarity  *)
  TIM_CCER_CC3E = $00000100;  (*!<Capture/Compare 3 output enable  *)
  TIM_CCER_CC3P = $00000200;  (*!<Capture/Compare 3 output Polarity  *)
  TIM_CCER_CC3NE = $00000400;  (*!<Capture/Compare 3 Complementary output enable  *)
  TIM_CCER_CC3NP = $00000800;  (*!<Capture/Compare 3 Complementary output Polarity  *)
  TIM_CCER_CC4E = $00001000;  (*!<Capture/Compare 4 output enable  *)
  TIM_CCER_CC4P = $00002000;  (*!<Capture/Compare 4 output Polarity  *)
  TIM_CCER_CC4NP = $00008000;  (*!<Capture/Compare 4 Complementary output Polarity  *)
  TIM_CCER_CC5E = $00010000;  (*!<Capture/Compare 5 output enable  *)
  TIM_CCER_CC5P = $00020000;  (*!<Capture/Compare 5 output Polarity  *)
  TIM_CCER_CC6E = $00100000;  (*!<Capture/Compare 6 output enable  *)
  TIM_CCER_CC6P = $00200000;  (*!<Capture/Compare 6 output Polarity  *)
  (*******************  Bit definition for TIM_CNT register  ******************* *)

  TIM_CNT_CNT = $FFFF;  (*!<Counter Value             *)
  (*******************  Bit definition for TIM_PSC register  ******************* *)

  TIM_PSC_PSC = $FFFF;  (*!<Prescaler Value           *)
  (*******************  Bit definition for TIM_ARR register  ******************* *)

  TIM_ARR_ARR = $FFFF;  (*!<actual auto-reload Value  *)
  (*******************  Bit definition for TIM_RCR register  ******************* *)

  TIM_RCR_REP = $FF;  (*!<Repetition Counter Value  *)
  (*******************  Bit definition for TIM_CCR1 register  ****************** *)

  TIM_CCR1_CCR1 = $FFFF;  (*!<Capture/Compare 1 Value   *)
  (*******************  Bit definition for TIM_CCR2 register  ****************** *)

  TIM_CCR2_CCR2 = $FFFF;  (*!<Capture/Compare 2 Value   *)
  (*******************  Bit definition for TIM_CCR3 register  ****************** *)

  TIM_CCR3_CCR3 = $FFFF;  (*!<Capture/Compare 3 Value   *)
  (*******************  Bit definition for TIM_CCR4 register  ****************** *)

  TIM_CCR4_CCR4 = $FFFF;  (*!<Capture/Compare 4 Value   *)
  (*******************  Bit definition for TIM_BDTR register  ****************** *)

  TIM_BDTR_DTG = $000000FF;  (*!<DTG[0:7] bits (Dead-Time Generator set-up)  *)
  TIM_BDTR_DTG_0 = $00000001;  (*!<Bit 0  *)
  TIM_BDTR_DTG_1 = $00000002;  (*!<Bit 1  *)
  TIM_BDTR_DTG_2 = $00000004;  (*!<Bit 2  *)
  TIM_BDTR_DTG_3 = $00000008;  (*!<Bit 3  *)
  TIM_BDTR_DTG_4 = $00000010;  (*!<Bit 4  *)
  TIM_BDTR_DTG_5 = $00000020;  (*!<Bit 5  *)
  TIM_BDTR_DTG_6 = $00000040;  (*!<Bit 6  *)
  TIM_BDTR_DTG_7 = $00000080;  (*!<Bit 7  *)
  TIM_BDTR_LOCK = $00000300;  (*!<LOCK[1:0] bits (Lock Configuration)  *)
  TIM_BDTR_LOCK_0 = $00000100;  (*!<Bit 0  *)
  TIM_BDTR_LOCK_1 = $00000200;  (*!<Bit 1  *)
  TIM_BDTR_OSSI = $00000400;  (*!<Off-State Selection for Idle mode  *)
  TIM_BDTR_OSSR = $00000800;  (*!<Off-State Selection for Run mode   *)
  TIM_BDTR_BKE = $00001000;  (*!<Break enable                       *)
  TIM_BDTR_BKP = $00002000;  (*!<Break Polarity                     *)
  TIM_BDTR_AOE = $00004000;  (*!<Automatic Output enable            *)
  TIM_BDTR_MOE = $00008000;  (*!<Main Output enable                 *)
  TIM_BDTR_BKF = $000F0000;  (*!<Break Filter for Break1  *)
  TIM_BDTR_BK2F = $00F00000;  (*!<Break Filter for Break2  *)
  TIM_BDTR_BK2E = $01000000;  (*!<Break enable for Break2  *)
  TIM_BDTR_BK2P = $02000000;  (*!<Break Polarity for Break2  *)
  (*******************  Bit definition for TIM_DCR register  ******************* *)

  TIM_DCR_DBA = $001F;  (*!<DBA[4:0] bits (DMA Base Address)  *)
  TIM_DCR_DBA_0 = $0001;  (*!<Bit 0  *)
  TIM_DCR_DBA_1 = $0002;  (*!<Bit 1  *)
  TIM_DCR_DBA_2 = $0004;  (*!<Bit 2  *)
  TIM_DCR_DBA_3 = $0008;  (*!<Bit 3  *)
  TIM_DCR_DBA_4 = $0010;  (*!<Bit 4  *)
  TIM_DCR_DBL = $1F00;  (*!<DBL[4:0] bits (DMA Burst Length)  *)
  TIM_DCR_DBL_0 = $0100;  (*!<Bit 0  *)
  TIM_DCR_DBL_1 = $0200;  (*!<Bit 1  *)
  TIM_DCR_DBL_2 = $0400;  (*!<Bit 2  *)
  TIM_DCR_DBL_3 = $0800;  (*!<Bit 3  *)
  TIM_DCR_DBL_4 = $1000;  (*!<Bit 4  *)
  (*******************  Bit definition for TIM_DMAR register  ****************** *)

  TIM_DMAR_DMAB = $FFFF;  (*!<DMA register for burst accesses                     *)
  (*******************  Bit definition for TIM_OR register  ******************** *)

  TIM_OR_TI4_RMP = $00C0;  (*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)              *)
  TIM_OR_TI4_RMP_0 = $0040;  (*!<Bit 0  *)
  TIM_OR_TI4_RMP_1 = $0080;  (*!<Bit 1  *)
  TIM_OR_ITR1_RMP = $0C00;  (*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap)  *)
  TIM_OR_ITR1_RMP_0 = $0400;  (*!<Bit 0  *)
  TIM_OR_ITR1_RMP_1 = $0800;  (*!<Bit 1  *)
  (******************  Bit definition for TIM_CCMR3 register  ****************** *)

  TIM_CCMR3_OC5FE = $00000004;  (*!<Output Compare 5 Fast enable  *)
  TIM_CCMR3_OC5PE = $00000008;  (*!<Output Compare 5 Preload enable  *)
  TIM_CCMR3_OC5M = $00010070;  (*!<OC5M[2:0] bits (Output Compare 5 Mode)  *)
  TIM_CCMR3_OC5M_0 = $00000010;  (*!<Bit 0  *)
  TIM_CCMR3_OC5M_1 = $00000020;  (*!<Bit 1  *)
  TIM_CCMR3_OC5M_2 = $00000040;  (*!<Bit 2  *)
  TIM_CCMR3_OC5M_3 = $00010000;  (*!<Bit 3  *)
  TIM_CCMR3_OC5CE = $00000080;  (*!<Output Compare 5 Clear Enable  *)
  TIM_CCMR3_OC6FE = $00000400;  (*!<Output Compare 4 Fast enable  *)
  TIM_CCMR3_OC6PE = $00000800;  (*!<Output Compare 4 Preload enable  *)
  TIM_CCMR3_OC6M = $01007000;  (*!<OC4M[2:0] bits (Output Compare 4 Mode)  *)
  TIM_CCMR3_OC6M_0 = $00001000;  (*!<Bit 0  *)
  TIM_CCMR3_OC6M_1 = $00002000;  (*!<Bit 1  *)
  TIM_CCMR3_OC6M_2 = $00004000;  (*!<Bit 2  *)
  TIM_CCMR3_OC6M_3 = $01000000;  (*!<Bit 3  *)
  TIM_CCMR3_OC6CE = $00008000;  (*!<Output Compare 4 Clear Enable  *)
  (*******************  Bit definition for TIM_CCR5 register  ****************** *)

  TIM_CCR5_CCR5 = $FFFFFFFF;  (*!<Capture/Compare 5 Value  *)
  TIM_CCR5_GC5C1 = $20000000;  (*!<Group Channel 5 and Channel 1  *)
  TIM_CCR5_GC5C2 = $40000000;  (*!<Group Channel 5 and Channel 2  *)
  TIM_CCR5_GC5C3 = $80000000;  (*!<Group Channel 5 and Channel 3  *)
  (*******************  Bit definition for TIM_CCR6 register  ****************** *)

  TIM_CCR6_CCR6 = $FFFF;  (*!<Capture/Compare 6 Value  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                         Low Power Timer (LPTIM)                             *)

  (*                                                                             *)

  (***************************************************************************** *)

  (******************  Bit definition for LPTIM_ISR register  ****************** *)

  LPTIM_ISR_CMPM = $00000001;  (*!< Compare match                        *)
  LPTIM_ISR_ARRM = $00000002;  (*!< Autoreload match                     *)
  LPTIM_ISR_EXTTRIG = $00000004;  (*!< External trigger edge event          *)
  LPTIM_ISR_CMPOK = $00000008;  (*!< Compare register update OK           *)
  LPTIM_ISR_ARROK = $00000010;  (*!< Autoreload register update OK        *)
  LPTIM_ISR_UP = $00000020;  (*!< Counter direction change down to up  *)
  LPTIM_ISR_DOWN = $00000040;  (*!< Counter direction change up to down  *)
  (******************  Bit definition for LPTIM_ICR register  ****************** *)

  LPTIM_ICR_CMPMCF = $00000001;  (*!< Compare match Clear Flag                        *)
  LPTIM_ICR_ARRMCF = $00000002;  (*!< Autoreload match Clear Flag                     *)
  LPTIM_ICR_EXTTRIGCF = $00000004;  (*!< External trigger edge event Clear Flag          *)
  LPTIM_ICR_CMPOKCF = $00000008;  (*!< Compare register update OK Clear Flag           *)
  LPTIM_ICR_ARROKCF = $00000010;  (*!< Autoreload register update OK Clear Flag        *)
  LPTIM_ICR_UPCF = $00000020;  (*!< Counter direction change down to up Clear Flag  *)
  LPTIM_ICR_DOWNCF = $00000040;  (*!< Counter direction change up to down Clear Flag  *)
  (******************  Bit definition for LPTIM_IER register ******************* *)

  LPTIM_IER_CMPMIE = $00000001;  (*!< Compare match Interrupt Enable                        *)
  LPTIM_IER_ARRMIE = $00000002;  (*!< Autoreload match Interrupt Enable                     *)
  LPTIM_IER_EXTTRIGIE = $00000004;  (*!< External trigger edge event Interrupt Enable          *)
  LPTIM_IER_CMPOKIE = $00000008;  (*!< Compare register update OK Interrupt Enable           *)
  LPTIM_IER_ARROKIE = $00000010;  (*!< Autoreload register update OK Interrupt Enable        *)
  LPTIM_IER_UPIE = $00000020;  (*!< Counter direction change down to up Interrupt Enable  *)
  LPTIM_IER_DOWNIE = $00000040;  (*!< Counter direction change up to down Interrupt Enable  *)
  (******************  Bit definition for LPTIM_CFGR register ****************** *)

  LPTIM_CFGR_CKSEL = $00000001;  (*!< Clock selector  *)
  LPTIM_CFGR_CKPOL = $00000006;  (*!< CKPOL[1:0] bits (Clock polarity)  *)
  LPTIM_CFGR_CKPOL_0 = $00000002;  (*!< Bit 0  *)
  LPTIM_CFGR_CKPOL_1 = $00000004;  (*!< Bit 1  *)
  LPTIM_CFGR_CKFLT = $00000018;  (*!< CKFLT[1:0] bits (Configurable digital filter for external clock)  *)
  LPTIM_CFGR_CKFLT_0 = $00000008;  (*!< Bit 0  *)
  LPTIM_CFGR_CKFLT_1 = $00000010;  (*!< Bit 1  *)
  LPTIM_CFGR_TRGFLT = $000000C0;  (*!< TRGFLT[1:0] bits (Configurable digital filter for trigger)  *)
  LPTIM_CFGR_TRGFLT_0 = $00000040;  (*!< Bit 0  *)
  LPTIM_CFGR_TRGFLT_1 = $00000080;  (*!< Bit 1  *)
  LPTIM_CFGR_PRESC = $00000E00;  (*!< PRESC[2:0] bits (Clock prescaler)  *)
  LPTIM_CFGR_PRESC_0 = $00000200;  (*!< Bit 0  *)
  LPTIM_CFGR_PRESC_1 = $00000400;  (*!< Bit 1  *)
  LPTIM_CFGR_PRESC_2 = $00000800;  (*!< Bit 2  *)
  LPTIM_CFGR_TRIGSEL = $0000E000;  (*!< TRIGSEL[2:0]] bits (Trigger selector)  *)
  LPTIM_CFGR_TRIGSEL_0 = $00002000;  (*!< Bit 0  *)
  LPTIM_CFGR_TRIGSEL_1 = $00004000;  (*!< Bit 1  *)
  LPTIM_CFGR_TRIGSEL_2 = $00008000;  (*!< Bit 2  *)
  LPTIM_CFGR_TRIGEN = $00060000;  (*!< TRIGEN[1:0] bits (Trigger enable and polarity)  *)
  LPTIM_CFGR_TRIGEN_0 = $00020000;  (*!< Bit 0  *)
  LPTIM_CFGR_TRIGEN_1 = $00040000;  (*!< Bit 1  *)
  LPTIM_CFGR_TIMOUT = $00080000;  (*!< Timout enable            *)
  LPTIM_CFGR_WAVE = $00100000;  (*!< Waveform shape           *)
  LPTIM_CFGR_WAVPOL = $00200000;  (*!< Waveform shape polarity  *)
  LPTIM_CFGR_PRELOAD = $00400000;  (*!< Reg update mode          *)
  LPTIM_CFGR_COUNTMODE = $00800000;  (*!< Counter mode enable      *)
  LPTIM_CFGR_ENC = $01000000;  (*!< Encoder mode enable      *)
  (******************  Bit definition for LPTIM_CR register  ******************* *)

  LPTIM_CR_ENABLE = $00000001;  (*!< LPTIMer enable                  *)
  LPTIM_CR_SNGSTRT = $00080002;  (*!< Timer start in single mode      *)
  LPTIM_CR_CNTSTRT = $00000004;  (*!< Timer start in continuous mode  *)
  (******************  Bit definition for LPTIM_CMP register  ****************** *)

  LPTIM_CMP_CMP = $0000FFFF;  (*!< Compare register      *)
  (******************  Bit definition for LPTIM_ARR register  ****************** *)

  LPTIM_ARR_ARR = $0000FFFF;  (*!< Auto reload register  *)
  (******************  Bit definition for LPTIM_CNT register  ****************** *)

  LPTIM_CNT_CNT = $0000FFFF;  (*!< Counter register      *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*      Universal Synchronous Asynchronous Receiver Transmitter (USART)        *)

  (*                                                                             *)

  (***************************************************************************** *)

  (******************  Bit definition for USART_CR1 register  ****************** *)

  USART_CR1_UE = $00000001;  (*!< USART Enable                                     *)
  USART_CR1_RE = $00000004;  (*!< Receiver Enable                                  *)
  USART_CR1_TE = $00000008;  (*!< Transmitter Enable                               *)
  USART_CR1_IDLEIE = $00000010;  (*!< IDLE Interrupt Enable                            *)
  USART_CR1_RXNEIE = $00000020;  (*!< RXNE Interrupt Enable                            *)
  USART_CR1_TCIE = $00000040;  (*!< Transmission Complete Interrupt Enable           *)
  USART_CR1_TXEIE = $00000080;  (*!< TXE Interrupt Enable                             *)
  USART_CR1_PEIE = $00000100;  (*!< PE Interrupt Enable                              *)
  USART_CR1_PS = $00000200;  (*!< Parity Selection                                 *)
  USART_CR1_PCE = $00000400;  (*!< Parity Control Enable                            *)
  USART_CR1_WAKE = $00000800;  (*!< Receiver Wakeup method                           *)
  USART_CR1_M = $10001000;  (*!< Word length                                      *)
  USART_CR1_M_0 = $00001000;  (*!< Word length - Bit 0                              *)
  USART_CR1_MME = $00002000;  (*!< Mute Mode Enable                                 *)
  USART_CR1_CMIE = $00004000;  (*!< Character match interrupt enable                 *)
  USART_CR1_OVER8 = $00008000;  (*!< Oversampling by 8-bit or 16-bit mode             *)
  USART_CR1_DEDT = $001F0000;  (*!< DEDT[4:0] bits (Driver Enable Deassertion Time)  *)
  USART_CR1_DEDT_0 = $00010000;  (*!< Bit 0  *)
  USART_CR1_DEDT_1 = $00020000;  (*!< Bit 1  *)
  USART_CR1_DEDT_2 = $00040000;  (*!< Bit 2  *)
  USART_CR1_DEDT_3 = $00080000;  (*!< Bit 3  *)
  USART_CR1_DEDT_4 = $00100000;  (*!< Bit 4  *)
  USART_CR1_DEAT = $03E00000;  (*!< DEAT[4:0] bits (Driver Enable Assertion Time)    *)
  USART_CR1_DEAT_0 = $00200000;  (*!< Bit 0  *)
  USART_CR1_DEAT_1 = $00400000;  (*!< Bit 1  *)
  USART_CR1_DEAT_2 = $00800000;  (*!< Bit 2  *)
  USART_CR1_DEAT_3 = $01000000;  (*!< Bit 3  *)
  USART_CR1_DEAT_4 = $02000000;  (*!< Bit 4  *)
  USART_CR1_RTOIE = $04000000;  (*!< Receive Time Out interrupt enable  *)
  USART_CR1_EOBIE = $08000000;  (*!< End of Block interrupt enable      *)
  USART_CR1_M_1 = $10000000;  (*!< Word length - Bit 1                *)
  (******************  Bit definition for USART_CR2 register  ****************** *)

  USART_CR2_ADDM7 = $00000010;  (*!< 7-bit or 4-bit Address Detection        *)
  USART_CR2_LBDL = $00000020;  (*!< LIN Break Detection Length              *)
  USART_CR2_LBDIE = $00000040;  (*!< LIN Break Detection Interrupt Enable    *)
  USART_CR2_LBCL = $00000100;  (*!< Last Bit Clock pulse                    *)
  USART_CR2_CPHA = $00000200;  (*!< Clock Phase                             *)
  USART_CR2_CPOL = $00000400;  (*!< Clock Polarity                          *)
  USART_CR2_CLKEN = $00000800;  (*!< Clock Enable                            *)
  USART_CR2_STOP = $00003000;  (*!< STOP[1:0] bits (STOP bits)              *)
  USART_CR2_STOP_0 = $00001000;  (*!< Bit 0  *)
  USART_CR2_STOP_1 = $00002000;  (*!< Bit 1  *)
  USART_CR2_LINEN = $00004000;  (*!< LIN mode enable                         *)
  USART_CR2_SWAP = $00008000;  (*!< SWAP TX/RX pins                         *)
  USART_CR2_RXINV = $00010000;  (*!< RX pin active level inversion           *)
  USART_CR2_TXINV = $00020000;  (*!< TX pin active level inversion           *)
  USART_CR2_DATAINV = $00040000;  (*!< Binary data inversion                   *)
  USART_CR2_MSBFIRST = $00080000;  (*!< Most Significant Bit First              *)
  USART_CR2_ABREN = $00100000;  (*!< Auto Baud-Rate Enable                   *)
  USART_CR2_ABRMODE = $00600000;  (*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode)  *)
  USART_CR2_ABRMODE_0 = $00200000;  (*!< Bit 0  *)
  USART_CR2_ABRMODE_1 = $00400000;  (*!< Bit 1  *)
  USART_CR2_RTOEN = $00800000;  (*!< Receiver Time-Out enable   *)
  USART_CR2_ADD = $FF000000;  (*!< Address of the USART node  *)
  (******************  Bit definition for USART_CR3 register  ****************** *)

  USART_CR3_EIE = $00000001;  (*!< Error Interrupt Enable                          *)
  USART_CR3_IREN = $00000002;  (*!< IrDA mode Enable                                *)
  USART_CR3_IRLP = $00000004;  (*!< IrDA Low-Power                                  *)
  USART_CR3_HDSEL = $00000008;  (*!< Half-Duplex Selection                           *)
  USART_CR3_NACK = $00000010;  (*!< SmartCard NACK enable                           *)
  USART_CR3_SCEN = $00000020;  (*!< SmartCard mode enable                           *)
  USART_CR3_DMAR = $00000040;  (*!< DMA Enable Receiver                             *)
  USART_CR3_DMAT = $00000080;  (*!< DMA Enable Transmitter                          *)
  USART_CR3_RTSE = $00000100;  (*!< RTS Enable                                      *)
  USART_CR3_CTSE = $00000200;  (*!< CTS Enable                                      *)
  USART_CR3_CTSIE = $00000400;  (*!< CTS Interrupt Enable                            *)
  USART_CR3_ONEBIT = $00000800;  (*!< One sample bit method enable                    *)
  USART_CR3_OVRDIS = $00001000;  (*!< Overrun Disable                                 *)
  USART_CR3_DDRE = $00002000;  (*!< DMA Disable on Reception Error                  *)
  USART_CR3_DEM = $00004000;  (*!< Driver Enable Mode                              *)
  USART_CR3_DEP = $00008000;  (*!< Driver Enable Polarity Selection                *)
  USART_CR3_SCARCNT = $000E0000;  (*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count)  *)
  USART_CR3_SCARCNT_0 = $00020000;  (*!< Bit 0  *)
  USART_CR3_SCARCNT_1 = $00040000;  (*!< Bit 1  *)
  USART_CR3_SCARCNT_2 = $00080000;  (*!< Bit 2  *)
  (******************  Bit definition for USART_BRR register  ****************** *)

  USART_BRR_DIV_FRACTION = $000F;  (*!< Fraction of USARTDIV  *)
  USART_BRR_DIV_MANTISSA = $FFF0;  (*!< Mantissa of USARTDIV  *)
  (******************  Bit definition for USART_GTPR register  ***************** *)

  USART_GTPR_PSC = $00FF;  (*!< PSC[7:0] bits (Prescaler value)  *)
  USART_GTPR_GT = $FF00;  (*!< GT[7:0] bits (Guard time value)  *)
  (*******************  Bit definition for USART_RTOR register  **************** *)

  USART_RTOR_RTO = $00FFFFFF;  (*!< Receiver Time Out Value  *)
  USART_RTOR_BLEN = $FF000000;  (*!< Block Length  *)
  (*******************  Bit definition for USART_RQR register  ***************** *)

  USART_RQR_ABRRQ = $0001;  (*!< Auto-Baud Rate Request       *)
  USART_RQR_SBKRQ = $0002;  (*!< Send Break Request           *)
  USART_RQR_MMRQ = $0004;  (*!< Mute Mode Request            *)
  USART_RQR_RXFRQ = $0008;  (*!< Receive Data flush Request   *)
  USART_RQR_TXFRQ = $0010;  (*!< Transmit data flush Request  *)
  (*******************  Bit definition for USART_ISR register  ***************** *)

  USART_ISR_PE = $00000001;  (*!< Parity Error                         *)
  USART_ISR_FE = $00000002;  (*!< Framing Error                        *)
  USART_ISR_NE = $00000004;  (*!< Noise detected Flag                  *)
  USART_ISR_ORE = $00000008;  (*!< OverRun Error                        *)
  USART_ISR_IDLE = $00000010;  (*!< IDLE line detected                   *)
  USART_ISR_RXNE = $00000020;  (*!< Read Data Register Not Empty         *)
  USART_ISR_TC = $00000040;  (*!< Transmission Complete                *)
  USART_ISR_TXE = $00000080;  (*!< Transmit Data Register Empty         *)
  USART_ISR_LBD = $00000100;  (*!< LIN Break Detection Flag             *)
  USART_ISR_CTSIF = $00000200;  (*!< CTS interrupt flag                   *)
  USART_ISR_CTS = $00000400;  (*!< CTS flag                             *)
  USART_ISR_RTOF = $00000800;  (*!< Receiver Time Out                    *)
  USART_ISR_EOBF = $00001000;  (*!< End Of Block Flag                    *)
  USART_ISR_ABRE = $00004000;  (*!< Auto-Baud Rate Error                 *)
  USART_ISR_ABRF = $00008000;  (*!< Auto-Baud Rate Flag                  *)
  USART_ISR_BUSY = $00010000;  (*!< Busy Flag                            *)
  USART_ISR_CMF = $00020000;  (*!< Character Match Flag                 *)
  USART_ISR_SBKF = $00040000;  (*!< Send Break Flag                      *)
  USART_ISR_RWU = $00080000;  (*!< Receive Wake Up from mute mode Flag  *)
  USART_ISR_WUF = $00100000;  (*!< Wake Up from stop mode Flag          *)
  USART_ISR_TEACK = $00200000;  (*!< Transmit Enable Acknowledge Flag     *)
  USART_ISR_REACK = $00400000;  (*!< Receive Enable Acknowledge Flag      *)
  (*******************  Bit definition for USART_ICR register  ***************** *)

  USART_ICR_PECF = $00000001;  (*!< Parity Error Clear Flag              *)
  USART_ICR_FECF = $00000002;  (*!< Framing Error Clear Flag             *)
  USART_ICR_NCF = $00000004;  (*!< Noise detected Clear Flag            *)
  USART_ICR_ORECF = $00000008;  (*!< OverRun Error Clear Flag             *)
  USART_ICR_IDLECF = $00000010;  (*!< IDLE line detected Clear Flag        *)
  USART_ICR_TCCF = $00000040;  (*!< Transmission Complete Clear Flag     *)
  USART_ICR_LBDCF = $00000100;  (*!< LIN Break Detection Clear Flag       *)
  USART_ICR_CTSCF = $00000200;  (*!< CTS Interrupt Clear Flag             *)
  USART_ICR_RTOCF = $00000800;  (*!< Receiver Time Out Clear Flag         *)
  USART_ICR_EOBCF = $00001000;  (*!< End Of Block Clear Flag              *)
  USART_ICR_CMCF = $00020000;  (*!< Character Match Clear Flag           *)
  USART_ICR_WUCF = $00100000;  (*!< Wake Up from stop mode Clear Flag    *)
  (*******************  Bit definition for USART_RDR register  ***************** *)

  USART_RDR_RDR = $01FF;  (*!< RDR[8:0] bits (Receive Data value)  *)
  (*******************  Bit definition for USART_TDR register  ***************** *)

  USART_TDR_TDR = $01FF;  (*!< TDR[8:0] bits (Transmit Data value)  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                            Window WATCHDOG                                  *)

  (*                                                                             *)

  (***************************************************************************** *)

  (*******************  Bit definition for WWDG_CR register  ******************* *)

  WWDG_CR_T = $7F;  (*!<T[6:0] bits (7-Bit counter (MSB to LSB))  *)
  WWDG_CR_T0 = $01;  (*!<Bit 0  *)
  WWDG_CR_T1 = $02;  (*!<Bit 1  *)
  WWDG_CR_T2 = $04;  (*!<Bit 2  *)
  WWDG_CR_T3 = $08;  (*!<Bit 3  *)
  WWDG_CR_T4 = $10;  (*!<Bit 4  *)
  WWDG_CR_T5 = $20;  (*!<Bit 5  *)
  WWDG_CR_T6 = $40;  (*!<Bit 6  *)
  WWDG_CR_WDGA = $80;  (*!<Activation bit  *)
  (*******************  Bit definition for WWDG_CFR register  ****************** *)

  WWDG_CFR_W = $007F;  (*!<W[6:0] bits (7-bit window value)  *)
  WWDG_CFR_W0 = $0001;  (*!<Bit 0  *)
  WWDG_CFR_W1 = $0002;  (*!<Bit 1  *)
  WWDG_CFR_W2 = $0004;  (*!<Bit 2  *)
  WWDG_CFR_W3 = $0008;  (*!<Bit 3  *)
  WWDG_CFR_W4 = $0010;  (*!<Bit 4  *)
  WWDG_CFR_W5 = $0020;  (*!<Bit 5  *)
  WWDG_CFR_W6 = $0040;  (*!<Bit 6  *)
  WWDG_CFR_WDGTB = $0180;  (*!<WDGTB[1:0] bits (Timer Base)  *)
  WWDG_CFR_WDGTB0 = $0080;  (*!<Bit 0  *)
  WWDG_CFR_WDGTB1 = $0100;  (*!<Bit 1  *)
  WWDG_CFR_EWI = $0200;  (*!<Early Wakeup Interrupt  *)
  (*******************  Bit definition for WWDG_SR register  ******************* *)

  WWDG_SR_EWIF = $01;  (*!<Early Wakeup Interrupt Flag  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                                DBG                                          *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for DBGMCU_IDCODE register  ************ *)

  DBGMCU_IDCODE_DEV_ID = $00000FFF;
  DBGMCU_IDCODE_REV_ID = $FFFF0000;
  (********************  Bit definition for DBGMCU_CR register  **************** *)

  DBGMCU_CR_DBG_SLEEP = $00000001;
  DBGMCU_CR_DBG_STOP = $00000002;
  DBGMCU_CR_DBG_STANDBY = $00000004;
  DBGMCU_CR_TRACE_IOEN = $00000020;
  DBGMCU_CR_TRACE_MODE = $000000C0;
  DBGMCU_CR_TRACE_MODE_0 = $00000040;  (*!<Bit 0  *)
  DBGMCU_CR_TRACE_MODE_1 = $00000080;  (*!<Bit 1  *)
  (********************  Bit definition for DBGMCU_APB1_FZ register  *********** *)

  DBGMCU_APB1_FZ_DBG_TIM2_STOP = $00000001;
  DBGMCU_APB1_FZ_DBG_TIM3_STOP = $00000002;
  DBGMCU_APB1_FZ_DBG_TIM4_STOP = $00000004;
  DBGMCU_APB1_FZ_DBG_TIM5_STOP = $00000008;
  DBGMCU_APB1_FZ_DBG_TIM6_STOP = $00000010;
  DBGMCU_APB1_FZ_DBG_TIM7_STOP = $00000020;
  DBGMCU_APB1_FZ_DBG_TIM12_STOP = $00000040;
  DBGMCU_APB1_FZ_DBG_TIM13_STOP = $00000080;
  DBGMCU_APB1_FZ_DBG_TIM14_STOP = $00000100;
  DBGMCU_APB1_FZ_DBG_RTC_STOP = $00000400;
  DBGMCU_APB1_FZ_DBG_WWDG_STOP = $00000800;
  DBGMCU_APB1_FZ_DBG_IWDG_STOP = $00001000;
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT = $00200000;
  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT = $00400000;
  DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT = $00800000;
  DBGMCU_APB1_FZ_DBG_CAN1_STOP = $02000000;
  DBGMCU_APB1_FZ_DBG_CAN2_STOP = $04000000;
  (********************  Bit definition for DBGMCU_APB2_FZ register  *********** *)

  DBGMCU_APB1_FZ_DBG_TIM1_STOP = $00000001;
  DBGMCU_APB1_FZ_DBG_TIM8_STOP = $00000002;
  DBGMCU_APB1_FZ_DBG_TIM9_STOP = $00010000;
  DBGMCU_APB1_FZ_DBG_TIM10_STOP = $00020000;
  DBGMCU_APB1_FZ_DBG_TIM11_STOP = $00040000;

(***************************************************************************** *)

(*                                                                             *)

(*                Ethernet MAC Registers bits definitions                      *)

(*                                                                             *)

(***************************************************************************** *)

(* Bit definition for Ethernet MAC Control Register register  *)

const
  ETH_MACCR_WD = $00800000;  (* Watchdog disable  *)
  ETH_MACCR_JD = $00400000;  (* Jabber disable  *)
  ETH_MACCR_IFG = $000E0000;  (* Inter-frame gap  *)
  ETH_MACCR_IFG_96Bit = $00000000;  (* Minimum IFG between frames during transmission is 96Bit  *)
  ETH_MACCR_IFG_88Bit = $00020000;  (* Minimum IFG between frames during transmission is 88Bit  *)
  ETH_MACCR_IFG_80Bit = $00040000;  (* Minimum IFG between frames during transmission is 80Bit  *)
  ETH_MACCR_IFG_72Bit = $00060000;  (* Minimum IFG between frames during transmission is 72Bit  *)
  ETH_MACCR_IFG_64Bit = $00080000;  (* Minimum IFG between frames during transmission is 64Bit  *)
  ETH_MACCR_IFG_56Bit = $000A0000;  (* Minimum IFG between frames during transmission is 56Bit  *)
  ETH_MACCR_IFG_48Bit = $000C0000;  (* Minimum IFG between frames during transmission is 48Bit  *)
  ETH_MACCR_IFG_40Bit = $000E0000;  (* Minimum IFG between frames during transmission is 40Bit  *)
  ETH_MACCR_CSD = $00010000;  (* Carrier sense disable (during transmission)  *)
  ETH_MACCR_FES = $00004000;  (* Fast ethernet speed  *)
  ETH_MACCR_ROD = $00002000;  (* Receive own disable  *)
  ETH_MACCR_LM = $00001000;  (* loopback mode  *)
  ETH_MACCR_DM = $00000800;  (* Duplex mode  *)
  ETH_MACCR_IPCO = $00000400;  (* IP Checksum offload  *)
  ETH_MACCR_RD = $00000200;  (* Retry disable  *)
  ETH_MACCR_APCS = $00000080;  (* Automatic Pad/CRC stripping  *)
  ETH_MACCR_BL = $00000060;  (* Back-off limit: random integer number (r) of slot time delays before rescheduling
                                                       a transmission attempt during retries after a collision: 0 =< r <2^k  *)
  ETH_MACCR_BL_10 = $00000000;  (* k = min (n, 10)  *)
  ETH_MACCR_BL_8 = $00000020;  (* k = min (n, 8)  *)
  ETH_MACCR_BL_4 = $00000040;  (* k = min (n, 4)  *)
  ETH_MACCR_BL_1 = $00000060;  (* k = min (n, 1)  *)
  ETH_MACCR_DC = $00000010;  (* Defferal check  *)
  ETH_MACCR_TE = $00000008;  (* Transmitter enable  *)
  ETH_MACCR_RE = $00000004;  (* Receiver enable  *)
  (* Bit definition for Ethernet MAC Frame Filter Register  *)

  ETH_MACFFR_RA = $80000000;  (* Receive all  *)
  ETH_MACFFR_HPF = $00000400;  (* Hash or perfect filter  *)
  ETH_MACFFR_SAF = $00000200;  (* Source address filter enable  *)
  ETH_MACFFR_SAIF = $00000100;  (* SA inverse filtering  *)
  ETH_MACFFR_PCF = $000000C0;  (* Pass control frames: 3 cases  *)
  ETH_MACFFR_PCF_BlockAll = $00000040;  (* MAC filters all control frames from reaching the application  *)
  ETH_MACFFR_PCF_ForwardAll = $00000080;  (* MAC forwards all control frames to application even if they fail the Address Filter  *)
  ETH_MACFFR_PCF_ForwardPassedAddrFilter = $000000C0;  (* MAC forwards control frames that pass the Address Filter.  *)
  ETH_MACFFR_BFD = $00000020;  (* Broadcast frame disable  *)
  ETH_MACFFR_PAM = $00000010;  (* Pass all mutlicast  *)
  ETH_MACFFR_DAIF = $00000008;  (* DA Inverse filtering  *)
  ETH_MACFFR_HM = $00000004;  (* Hash multicast  *)
  ETH_MACFFR_HU = $00000002;  (* Hash unicast  *)
  ETH_MACFFR_PM = $00000001;  (* Promiscuous mode  *)
  (* Bit definition for Ethernet MAC Hash Table High Register  *)

  ETH_MACHTHR_HTH = $FFFFFFFF;  (* Hash table high  *)
  (* Bit definition for Ethernet MAC Hash Table Low Register  *)

  ETH_MACHTLR_HTL = $FFFFFFFF;  (* Hash table low  *)
  (* Bit definition for Ethernet MAC MII Address Register  *)

  ETH_MACMIIAR_PA = $0000F800;  (* Physical layer address  *)
  ETH_MACMIIAR_MR = $000007C0;  (* MII register in the selected PHY  *)
  ETH_MACMIIAR_CR = $0000001C;  (* CR clock range: 6 cases  *)
  ETH_MACMIIAR_CR_Div42 = $00000000;  (* HCLK:60-100 MHz; MDC clock= HCLK/42  *)
  ETH_MACMIIAR_CR_Div62 = $00000004;  (* HCLK:100-150 MHz; MDC clock= HCLK/62  *)
  ETH_MACMIIAR_CR_Div16 = $00000008;  (* HCLK:20-35 MHz; MDC clock= HCLK/16  *)
  ETH_MACMIIAR_CR_Div26 = $0000000C;  (* HCLK:35-60 MHz; MDC clock= HCLK/26  *)
  ETH_MACMIIAR_CR_Div102 = $00000010;  (* HCLK:150-168 MHz; MDC clock= HCLK/102  *)
  ETH_MACMIIAR_MW = $00000002;  (* MII write  *)
  ETH_MACMIIAR_MB = $00000001;  (* MII busy  *)
  (* Bit definition for Ethernet MAC MII Data Register  *)

  ETH_MACMIIDR_MD = $0000FFFF;  (* MII data: read/write data from/to PHY  *)
  (* Bit definition for Ethernet MAC Flow Control Register  *)

  ETH_MACFCR_PT = $FFFF0000;  (* Pause time  *)
  ETH_MACFCR_ZQPD = $00000080;  (* Zero-quanta pause disable  *)
  ETH_MACFCR_PLT = $00000030;  (* Pause low threshold: 4 cases  *)
  ETH_MACFCR_PLT_Minus4 = $00000000;  (* Pause time minus 4 slot times  *)
  ETH_MACFCR_PLT_Minus28 = $00000010;  (* Pause time minus 28 slot times  *)
  ETH_MACFCR_PLT_Minus144 = $00000020;  (* Pause time minus 144 slot times  *)
  ETH_MACFCR_PLT_Minus256 = $00000030;  (* Pause time minus 256 slot times  *)
  ETH_MACFCR_UPFD = $00000008;  (* Unicast pause frame detect  *)
  ETH_MACFCR_RFCE = $00000004;  (* Receive flow control enable  *)
  ETH_MACFCR_TFCE = $00000002;  (* Transmit flow control enable  *)
  ETH_MACFCR_FCBBPA = $00000001;  (* Flow control busy/backpressure activate  *)
  (* Bit definition for Ethernet MAC VLAN Tag Register  *)

  ETH_MACVLANTR_VLANTC = $00010000;  (* 12-bit VLAN tag comparison  *)
  ETH_MACVLANTR_VLANTI = $0000FFFF;  (* VLAN tag identifier (for receive frames)  *)
  (* Bit definition for Ethernet MAC Remote Wake-UpFrame Filter Register  *)

  ETH_MACRWUFFR_D = $FFFFFFFF;  (* Wake-up frame filter register data  *)
  (* Eight sequential Writes to this address (offset 0x28) will write all Wake-UpFrame Filter Registers.
   Eight sequential Reads from this address (offset 0x28) will read all Wake-UpFrame Filter Registers.  *)

  (* Wake-UpFrame Filter Reg0 : Filter 0 Byte Mask
   Wake-UpFrame Filter Reg1 : Filter 1 Byte Mask
   Wake-UpFrame Filter Reg2 : Filter 2 Byte Mask
   Wake-UpFrame Filter Reg3 : Filter 3 Byte Mask
   Wake-UpFrame Filter Reg4 : RSVD - Filter3 Command - RSVD - Filter2 Command -
                              RSVD - Filter1 Command - RSVD - Filter0 Command
   Wake-UpFrame Filter Re5 : Filter3 Offset - Filter2 Offset - Filter1 Offset - Filter0 Offset
   Wake-UpFrame Filter Re6 : Filter1 CRC16 - Filter0 CRC16
   Wake-UpFrame Filter Re7 : Filter3 CRC16 - Filter2 CRC16  *)

  (* Bit definition for Ethernet MAC PMT Control and Status Register  *)

  ETH_MACPMTCSR_WFFRPR = $80000000;  (* Wake-Up Frame Filter Register Pointer Reset  *)
  ETH_MACPMTCSR_GU = $00000200;  (* Global Unicast  *)
  ETH_MACPMTCSR_WFR = $00000040;  (* Wake-Up Frame Received  *)
  ETH_MACPMTCSR_MPR = $00000020;  (* Magic Packet Received  *)
  ETH_MACPMTCSR_WFE = $00000004;  (* Wake-Up Frame Enable  *)
  ETH_MACPMTCSR_MPE = $00000002;  (* Magic Packet Enable  *)
  ETH_MACPMTCSR_PD = $00000001;  (* Power Down  *)
  (* Bit definition for Ethernet MAC Status Register  *)

  ETH_MACSR_TSTS = $00000200;  (* Time stamp trigger status  *)
  ETH_MACSR_MMCTS = $00000040;  (* MMC transmit status  *)
  ETH_MACSR_MMMCRS = $00000020;  (* MMC receive status  *)
  ETH_MACSR_MMCS = $00000010;  (* MMC status  *)
  ETH_MACSR_PMTS = $00000008;  (* PMT status  *)
  (* Bit definition for Ethernet MAC Interrupt Mask Register  *)

  ETH_MACIMR_TSTIM = $00000200;  (* Time stamp trigger interrupt mask  *)
  ETH_MACIMR_PMTIM = $00000008;  (* PMT interrupt mask  *)
  (* Bit definition for Ethernet MAC Address0 High Register  *)

  ETH_MACA0HR_MACA0H = $0000FFFF;  (* MAC address0 high  *)
  (* Bit definition for Ethernet MAC Address0 Low Register  *)

  ETH_MACA0LR_MACA0L = $FFFFFFFF;  (* MAC address0 low  *)
  (* Bit definition for Ethernet MAC Address1 High Register  *)

  ETH_MACA1HR_AE = $80000000;  (* Address enable  *)
  ETH_MACA1HR_SA = $40000000;  (* Source address  *)
  ETH_MACA1HR_MBC = $3F000000;  (* Mask byte control: bits to mask for comparison of the MAC Address bytes  *)
  ETH_MACA1HR_MBC_HBits15_8 = $20000000;  (* Mask MAC Address high reg bits [15:8]  *)
  ETH_MACA1HR_MBC_HBits7_0 = $10000000;  (* Mask MAC Address high reg bits [7:0]  *)
  ETH_MACA1HR_MBC_LBits31_24 = $08000000;  (* Mask MAC Address low reg bits [31:24]  *)
  ETH_MACA1HR_MBC_LBits23_16 = $04000000;  (* Mask MAC Address low reg bits [23:16]  *)
  ETH_MACA1HR_MBC_LBits15_8 = $02000000;  (* Mask MAC Address low reg bits [15:8]  *)
  ETH_MACA1HR_MBC_LBits7_0 = $01000000;  (* Mask MAC Address low reg bits [7:0]  *)
  ETH_MACA1HR_MACA1H = $0000FFFF;  (* MAC address1 high  *)
  (* Bit definition for Ethernet MAC Address1 Low Register  *)

  ETH_MACA1LR_MACA1L = $FFFFFFFF;  (* MAC address1 low  *)
  (* Bit definition for Ethernet MAC Address2 High Register  *)

  ETH_MACA2HR_AE = $80000000;  (* Address enable  *)
  ETH_MACA2HR_SA = $40000000;  (* Source address  *)
  ETH_MACA2HR_MBC = $3F000000;  (* Mask byte control  *)
  ETH_MACA2HR_MBC_HBits15_8 = $20000000;  (* Mask MAC Address high reg bits [15:8]  *)
  ETH_MACA2HR_MBC_HBits7_0 = $10000000;  (* Mask MAC Address high reg bits [7:0]  *)
  ETH_MACA2HR_MBC_LBits31_24 = $08000000;  (* Mask MAC Address low reg bits [31:24]  *)
  ETH_MACA2HR_MBC_LBits23_16 = $04000000;  (* Mask MAC Address low reg bits [23:16]  *)
  ETH_MACA2HR_MBC_LBits15_8 = $02000000;  (* Mask MAC Address low reg bits [15:8]  *)
  ETH_MACA2HR_MBC_LBits7_0 = $01000000;  (* Mask MAC Address low reg bits [70]  *)
  ETH_MACA2HR_MACA2H = $0000FFFF;  (* MAC address1 high  *)
  (* Bit definition for Ethernet MAC Address2 Low Register  *)

  ETH_MACA2LR_MACA2L = $FFFFFFFF;  (* MAC address2 low  *)
  (* Bit definition for Ethernet MAC Address3 High Register  *)

  ETH_MACA3HR_AE = $80000000;  (* Address enable  *)
  ETH_MACA3HR_SA = $40000000;  (* Source address  *)
  ETH_MACA3HR_MBC = $3F000000;  (* Mask byte control  *)
  ETH_MACA3HR_MBC_HBits15_8 = $20000000;  (* Mask MAC Address high reg bits [15:8]  *)
  ETH_MACA3HR_MBC_HBits7_0 = $10000000;  (* Mask MAC Address high reg bits [7:0]  *)
  ETH_MACA3HR_MBC_LBits31_24 = $08000000;  (* Mask MAC Address low reg bits [31:24]  *)
  ETH_MACA3HR_MBC_LBits23_16 = $04000000;  (* Mask MAC Address low reg bits [23:16]  *)
  ETH_MACA3HR_MBC_LBits15_8 = $02000000;  (* Mask MAC Address low reg bits [15:8]  *)
  ETH_MACA3HR_MBC_LBits7_0 = $01000000;  (* Mask MAC Address low reg bits [70]  *)
  ETH_MACA3HR_MACA3H = $0000FFFF;  (* MAC address3 high  *)
  (* Bit definition for Ethernet MAC Address3 Low Register  *)

  ETH_MACA3LR_MACA3L = $FFFFFFFF;  (* MAC address3 low  *)
  (***************************************************************************** *)

  (*                Ethernet MMC Registers bits definition                       *)

  (***************************************************************************** *)

  (* Bit definition for Ethernet MMC Contol Register  *)

  ETH_MMCCR_MCFHP = $00000020;  (* MMC counter Full-Half preset  *)
  ETH_MMCCR_MCP = $00000010;  (* MMC counter preset  *)
  ETH_MMCCR_MCF = $00000008;  (* MMC Counter Freeze  *)
  ETH_MMCCR_ROR = $00000004;  (* Reset on Read  *)
  ETH_MMCCR_CSR = $00000002;  (* Counter Stop Rollover  *)
  ETH_MMCCR_CR = $00000001;  (* Counters Reset  *)
  (* Bit definition for Ethernet MMC Receive Interrupt Register  *)

  ETH_MMCRIR_RGUFS = $00020000;  (* Set when Rx good unicast frames counter reaches half the maximum value  *)
  ETH_MMCRIR_RFAES = $00000040;  (* Set when Rx alignment error counter reaches half the maximum value  *)
  ETH_MMCRIR_RFCES = $00000020;  (* Set when Rx crc error counter reaches half the maximum value  *)
  (* Bit definition for Ethernet MMC Transmit Interrupt Register  *)

  ETH_MMCTIR_TGFS = $00200000;  (* Set when Tx good frame count counter reaches half the maximum value  *)
  ETH_MMCTIR_TGFMSCS = $00008000;  (* Set when Tx good multi col counter reaches half the maximum value  *)
  ETH_MMCTIR_TGFSCS = $00004000;  (* Set when Tx good single col counter reaches half the maximum value  *)
  (* Bit definition for Ethernet MMC Receive Interrupt Mask Register  *)

  ETH_MMCRIMR_RGUFM = $00020000;  (* Mask the interrupt when Rx good unicast frames counter reaches half the maximum value  *)
  ETH_MMCRIMR_RFAEM = $00000040;  (* Mask the interrupt when when Rx alignment error counter reaches half the maximum value  *)
  ETH_MMCRIMR_RFCEM = $00000020;  (* Mask the interrupt when Rx crc error counter reaches half the maximum value  *)
  (* Bit definition for Ethernet MMC Transmit Interrupt Mask Register  *)

  ETH_MMCTIMR_TGFM = $00200000;  (* Mask the interrupt when Tx good frame count counter reaches half the maximum value  *)
  ETH_MMCTIMR_TGFMSCM = $00008000;  (* Mask the interrupt when Tx good multi col counter reaches half the maximum value  *)
  ETH_MMCTIMR_TGFSCM = $00004000;  (* Mask the interrupt when Tx good single col counter reaches half the maximum value  *)
  (* Bit definition for Ethernet MMC Transmitted Good Frames after Single Collision Counter Register  *)

  ETH_MMCTGFSCCR_TGFSCC = $FFFFFFFF;  (* Number of successfully transmitted frames after a single collision in Half-duplex mode.  *)
  (* Bit definition for Ethernet MMC Transmitted Good Frames after More than a Single Collision Counter Register  *)

  ETH_MMCTGFMSCCR_TGFMSCC = $FFFFFFFF;  (* Number of successfully transmitted frames after more than a single collision in Half-duplex mode.  *)
  (* Bit definition for Ethernet MMC Transmitted Good Frames Counter Register  *)

  ETH_MMCTGFCR_TGFC = $FFFFFFFF;  (* Number of good frames transmitted.  *)
  (* Bit definition for Ethernet MMC Received Frames with CRC Error Counter Register  *)

  ETH_MMCRFCECR_RFCEC = $FFFFFFFF;  (* Number of frames received with CRC error.  *)
  (* Bit definition for Ethernet MMC Received Frames with Alignement Error Counter Register  *)

  ETH_MMCRFAECR_RFAEC = $FFFFFFFF;  (* Number of frames received with alignment (dribble) error  *)
  (* Bit definition for Ethernet MMC Received Good Unicast Frames Counter Register  *)

  ETH_MMCRGUFCR_RGUFC = $FFFFFFFF;  (* Number of good unicast frames received.  *)
  (***************************************************************************** *)

  (*               Ethernet PTP Registers bits definition                        *)

  (***************************************************************************** *)

  (* Bit definition for Ethernet PTP Time Stamp Contol Register  *)

  ETH_PTPTSCR_TSCNT = $00030000;  (* Time stamp clock node type  *)
  ETH_PTPTSSR_TSSMRME = $00008000;  (* Time stamp snapshot for message relevant to master enable  *)
  ETH_PTPTSSR_TSSEME = $00004000;  (* Time stamp snapshot for event message enable  *)
  ETH_PTPTSSR_TSSIPV4FE = $00002000;  (* Time stamp snapshot for IPv4 frames enable  *)
  ETH_PTPTSSR_TSSIPV6FE = $00001000;  (* Time stamp snapshot for IPv6 frames enable  *)
  ETH_PTPTSSR_TSSPTPOEFE = $00000800;  (* Time stamp snapshot for PTP over ethernet frames enable  *)
  ETH_PTPTSSR_TSPTPPSV2E = $00000400;  (* Time stamp PTP packet snooping for version2 format enable  *)
  ETH_PTPTSSR_TSSSR = $00000200;  (* Time stamp Sub-seconds rollover  *)
  ETH_PTPTSSR_TSSARFE = $00000100;  (* Time stamp snapshot for all received frames enable  *)
  ETH_PTPTSCR_TSARU = $00000020;  (* Addend register update  *)
  ETH_PTPTSCR_TSITE = $00000010;  (* Time stamp interrupt trigger enable  *)
  ETH_PTPTSCR_TSSTU = $00000008;  (* Time stamp update  *)
  ETH_PTPTSCR_TSSTI = $00000004;  (* Time stamp initialize  *)
  ETH_PTPTSCR_TSFCU = $00000002;  (* Time stamp fine or coarse update  *)
  ETH_PTPTSCR_TSE = $00000001;  (* Time stamp enable  *)
  (* Bit definition for Ethernet PTP Sub-Second Increment Register  *)

  ETH_PTPSSIR_STSSI = $000000FF;  (* System time Sub-second increment value  *)
  (* Bit definition for Ethernet PTP Time Stamp High Register  *)

  ETH_PTPTSHR_STS = $FFFFFFFF;  (* System Time second  *)
  (* Bit definition for Ethernet PTP Time Stamp Low Register  *)

  ETH_PTPTSLR_STPNS = $80000000;  (* System Time Positive or negative time  *)
  ETH_PTPTSLR_STSS = $7FFFFFFF;  (* System Time sub-seconds  *)
  (* Bit definition for Ethernet PTP Time Stamp High Update Register  *)

  ETH_PTPTSHUR_TSUS = $FFFFFFFF;  (* Time stamp update seconds  *)
  (* Bit definition for Ethernet PTP Time Stamp Low Update Register  *)

  ETH_PTPTSLUR_TSUPNS = $80000000;  (* Time stamp update Positive or negative time  *)
  ETH_PTPTSLUR_TSUSS = $7FFFFFFF;  (* Time stamp update sub-seconds  *)
  (* Bit definition for Ethernet PTP Time Stamp Addend Register  *)

  ETH_PTPTSAR_TSA = $FFFFFFFF;  (* Time stamp addend  *)
  (* Bit definition for Ethernet PTP Target Time High Register  *)

  ETH_PTPTTHR_TTSH = $FFFFFFFF;  (* Target time stamp high  *)
  (* Bit definition for Ethernet PTP Target Time Low Register  *)

  ETH_PTPTTLR_TTSL = $FFFFFFFF;  (* Target time stamp low  *)
  (* Bit definition for Ethernet PTP Time Stamp Status Register  *)

  ETH_PTPTSSR_TSTTR = $00000020;  (* Time stamp target time reached  *)
  ETH_PTPTSSR_TSSO = $00000010;  (* Time stamp seconds overflow  *)
  (***************************************************************************** *)

  (*                 Ethernet DMA Registers bits definition                      *)

  (***************************************************************************** *)

  (* Bit definition for Ethernet DMA Bus Mode Register  *)

  ETH_DMABMR_AAB = $02000000;  (* Address-Aligned beats  *)
  ETH_DMABMR_FPM = $01000000;  (* 4xPBL mode  *)
  ETH_DMABMR_USP = $00800000;  (* Use separate PBL  *)
  ETH_DMABMR_RDP = $007E0000;  (* RxDMA PBL  *)
  ETH_DMABMR_RDP_1Beat = $00020000;  (* maximum number of beats to be transferred in one RxDMA transaction is 1  *)
  ETH_DMABMR_RDP_2Beat = $00040000;  (* maximum number of beats to be transferred in one RxDMA transaction is 2  *)
  ETH_DMABMR_RDP_4Beat = $00080000;  (* maximum number of beats to be transferred in one RxDMA transaction is 4  *)
  ETH_DMABMR_RDP_8Beat = $00100000;  (* maximum number of beats to be transferred in one RxDMA transaction is 8  *)
  ETH_DMABMR_RDP_16Beat = $00200000;  (* maximum number of beats to be transferred in one RxDMA transaction is 16  *)
  ETH_DMABMR_RDP_32Beat = $00400000;  (* maximum number of beats to be transferred in one RxDMA transaction is 32  *)
  ETH_DMABMR_RDP_4xPBL_4Beat = $01020000;  (* maximum number of beats to be transferred in one RxDMA transaction is 4  *)
  ETH_DMABMR_RDP_4xPBL_8Beat = $01040000;  (* maximum number of beats to be transferred in one RxDMA transaction is 8  *)
  ETH_DMABMR_RDP_4xPBL_16Beat = $01080000;  (* maximum number of beats to be transferred in one RxDMA transaction is 16  *)
  ETH_DMABMR_RDP_4xPBL_32Beat = $01100000;  (* maximum number of beats to be transferred in one RxDMA transaction is 32  *)
  ETH_DMABMR_RDP_4xPBL_64Beat = $01200000;  (* maximum number of beats to be transferred in one RxDMA transaction is 64  *)
  ETH_DMABMR_RDP_4xPBL_128Beat = $01400000;  (* maximum number of beats to be transferred in one RxDMA transaction is 128  *)
  ETH_DMABMR_FB = $00010000;  (* Fixed Burst  *)
  ETH_DMABMR_RTPR = $0000C000;  (* Rx Tx priority ratio  *)
  ETH_DMABMR_RTPR_1_1 = $00000000;  (* Rx Tx priority ratio  *)
  ETH_DMABMR_RTPR_2_1 = $00004000;  (* Rx Tx priority ratio  *)
  ETH_DMABMR_RTPR_3_1 = $00008000;  (* Rx Tx priority ratio  *)
  ETH_DMABMR_RTPR_4_1 = $0000C000;  (* Rx Tx priority ratio  *)
  ETH_DMABMR_PBL = $00003F00;  (* Programmable burst length  *)
  ETH_DMABMR_PBL_1Beat = $00000100;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 1  *)
  ETH_DMABMR_PBL_2Beat = $00000200;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 2  *)
  ETH_DMABMR_PBL_4Beat = $00000400;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4  *)
  ETH_DMABMR_PBL_8Beat = $00000800;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8  *)
  ETH_DMABMR_PBL_16Beat = $00001000;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16  *)
  ETH_DMABMR_PBL_32Beat = $00002000;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32  *)
  ETH_DMABMR_PBL_4xPBL_4Beat = $01000100;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4  *)
  ETH_DMABMR_PBL_4xPBL_8Beat = $01000200;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8  *)
  ETH_DMABMR_PBL_4xPBL_16Beat = $01000400;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16  *)
  ETH_DMABMR_PBL_4xPBL_32Beat = $01000800;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32  *)
  ETH_DMABMR_PBL_4xPBL_64Beat = $01001000;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 64  *)
  ETH_DMABMR_PBL_4xPBL_128Beat = $01002000;  (* maximum number of beats to be transferred in one TxDMA (or both) transaction is 128  *)
  ETH_DMABMR_EDE = $00000080;  (* Enhanced Descriptor Enable  *)
  ETH_DMABMR_DSL = $0000007C;  (* Descriptor Skip Length  *)
  ETH_DMABMR_DA = $00000002;  (* DMA arbitration scheme  *)
  ETH_DMABMR_SR = $00000001;  (* Software reset  *)
  (* Bit definition for Ethernet DMA Transmit Poll Demand Register  *)

  ETH_DMATPDR_TPD = $FFFFFFFF;  (* Transmit poll demand  *)
  (* Bit definition for Ethernet DMA Receive Poll Demand Register  *)

  ETH_DMARPDR_RPD = $FFFFFFFF;  (* Receive poll demand   *)
  (* Bit definition for Ethernet DMA Receive Descriptor List Address Register  *)

  ETH_DMARDLAR_SRL = $FFFFFFFF;  (* Start of receive list  *)
  (* Bit definition for Ethernet DMA Transmit Descriptor List Address Register  *)

  ETH_DMATDLAR_STL = $FFFFFFFF;  (* Start of transmit list  *)
  (* Bit definition for Ethernet DMA Status Register  *)

  ETH_DMASR_TSTS = $20000000;  (* Time-stamp trigger status  *)
  ETH_DMASR_PMTS = $10000000;  (* PMT status  *)
  ETH_DMASR_MMCS = $08000000;  (* MMC status  *)
  ETH_DMASR_EBS = $03800000;  (* Error bits status  *)
  (* combination with EBS[2:0] for GetFlagStatus function  *)

  ETH_DMASR_EBS_DescAccess = $02000000;  (* Error bits 0-data buffer, 1-desc. access  *)
  ETH_DMASR_EBS_ReadTransf = $01000000;  (* Error bits 0-write trnsf, 1-read transfr  *)
  ETH_DMASR_EBS_DataTransfTx = $00800000;  (* Error bits 0-Rx DMA, 1-Tx DMA  *)
  ETH_DMASR_TPS = $00700000;  (* Transmit process state  *)
  ETH_DMASR_TPS_Stopped = $00000000;  (* Stopped - Reset or Stop Tx Command issued   *)
  ETH_DMASR_TPS_Fetching = $00100000;  (* Running - fetching the Tx descriptor  *)
  ETH_DMASR_TPS_Waiting = $00200000;  (* Running - waiting for status  *)
  ETH_DMASR_TPS_Reading = $00300000;  (* Running - reading the data from host memory  *)
  ETH_DMASR_TPS_Suspended = $00600000;  (* Suspended - Tx Descriptor unavailabe  *)
  ETH_DMASR_TPS_Closing = $00700000;  (* Running - closing Rx descriptor  *)
  ETH_DMASR_RPS = $000E0000;  (* Receive process state  *)
  ETH_DMASR_RPS_Stopped = $00000000;  (* Stopped - Reset or Stop Rx Command issued  *)
  ETH_DMASR_RPS_Fetching = $00020000;  (* Running - fetching the Rx descriptor  *)
  ETH_DMASR_RPS_Waiting = $00060000;  (* Running - waiting for packet  *)
  ETH_DMASR_RPS_Suspended = $00080000;  (* Suspended - Rx Descriptor unavailable  *)
  ETH_DMASR_RPS_Closing = $000A0000;  (* Running - closing descriptor  *)
  ETH_DMASR_RPS_Queuing = $000E0000;  (* Running - queuing the recieve frame into host memory  *)
  ETH_DMASR_NIS = $00010000;  (* Normal interrupt summary  *)
  ETH_DMASR_AIS = $00008000;  (* Abnormal interrupt summary  *)
  ETH_DMASR_ERS = $00004000;  (* Early receive status  *)
  ETH_DMASR_FBES = $00002000;  (* Fatal bus error status  *)
  ETH_DMASR_ETS = $00000400;  (* Early transmit status  *)
  ETH_DMASR_RWTS = $00000200;  (* Receive watchdog timeout status  *)
  ETH_DMASR_RPSS = $00000100;  (* Receive process stopped status  *)
  ETH_DMASR_RBUS = $00000080;  (* Receive buffer unavailable status  *)
  ETH_DMASR_RS = $00000040;  (* Receive status  *)
  ETH_DMASR_TUS = $00000020;  (* Transmit underflow status  *)
  ETH_DMASR_ROS = $00000010;  (* Receive overflow status  *)
  ETH_DMASR_TJTS = $00000008;  (* Transmit jabber timeout status  *)
  ETH_DMASR_TBUS = $00000004;  (* Transmit buffer unavailable status  *)
  ETH_DMASR_TPSS = $00000002;  (* Transmit process stopped status  *)
  ETH_DMASR_TS = $00000001;  (* Transmit status  *)
  (* Bit definition for Ethernet DMA Operation Mode Register  *)

  ETH_DMAOMR_DTCEFD = $04000000;  (* Disable Dropping of TCP/IP checksum error frames  *)
  ETH_DMAOMR_RSF = $02000000;  (* Receive store and forward  *)
  ETH_DMAOMR_DFRF = $01000000;  (* Disable flushing of received frames  *)
  ETH_DMAOMR_TSF = $00200000;  (* Transmit store and forward  *)
  ETH_DMAOMR_FTF = $00100000;  (* Flush transmit FIFO  *)
  ETH_DMAOMR_TTC = $0001C000;  (* Transmit threshold control  *)
  ETH_DMAOMR_TTC_64Bytes = $00000000;  (* threshold level of the MTL Transmit FIFO is 64 Bytes  *)
  ETH_DMAOMR_TTC_128Bytes = $00004000;  (* threshold level of the MTL Transmit FIFO is 128 Bytes  *)
  ETH_DMAOMR_TTC_192Bytes = $00008000;  (* threshold level of the MTL Transmit FIFO is 192 Bytes  *)
  ETH_DMAOMR_TTC_256Bytes = $0000C000;  (* threshold level of the MTL Transmit FIFO is 256 Bytes  *)
  ETH_DMAOMR_TTC_40Bytes = $00010000;  (* threshold level of the MTL Transmit FIFO is 40 Bytes  *)
  ETH_DMAOMR_TTC_32Bytes = $00014000;  (* threshold level of the MTL Transmit FIFO is 32 Bytes  *)
  ETH_DMAOMR_TTC_24Bytes = $00018000;  (* threshold level of the MTL Transmit FIFO is 24 Bytes  *)
  ETH_DMAOMR_TTC_16Bytes = $0001C000;  (* threshold level of the MTL Transmit FIFO is 16 Bytes  *)
  ETH_DMAOMR_ST = $00002000;  (* Start/stop transmission command  *)
  ETH_DMAOMR_FEF = $00000080;  (* Forward error frames  *)
  ETH_DMAOMR_FUGF = $00000040;  (* Forward undersized good frames  *)
  ETH_DMAOMR_RTC = $00000018;  (* receive threshold control  *)
  ETH_DMAOMR_RTC_64Bytes = $00000000;  (* threshold level of the MTL Receive FIFO is 64 Bytes  *)
  ETH_DMAOMR_RTC_32Bytes = $00000008;  (* threshold level of the MTL Receive FIFO is 32 Bytes  *)
  ETH_DMAOMR_RTC_96Bytes = $00000010;  (* threshold level of the MTL Receive FIFO is 96 Bytes  *)
  ETH_DMAOMR_RTC_128Bytes = $00000018;  (* threshold level of the MTL Receive FIFO is 128 Bytes  *)
  ETH_DMAOMR_OSF = $00000004;  (* operate on second frame  *)
  ETH_DMAOMR_SR = $00000002;  (* Start/stop receive  *)
  (* Bit definition for Ethernet DMA Interrupt Enable Register  *)

  ETH_DMAIER_NISE = $00010000;  (* Normal interrupt summary enable  *)
  ETH_DMAIER_AISE = $00008000;  (* Abnormal interrupt summary enable  *)
  ETH_DMAIER_ERIE = $00004000;  (* Early receive interrupt enable  *)
  ETH_DMAIER_FBEIE = $00002000;  (* Fatal bus error interrupt enable  *)
  ETH_DMAIER_ETIE = $00000400;  (* Early transmit interrupt enable  *)
  ETH_DMAIER_RWTIE = $00000200;  (* Receive watchdog timeout interrupt enable  *)
  ETH_DMAIER_RPSIE = $00000100;  (* Receive process stopped interrupt enable  *)
  ETH_DMAIER_RBUIE = $00000080;  (* Receive buffer unavailable interrupt enable  *)
  ETH_DMAIER_RIE = $00000040;  (* Receive interrupt enable  *)
  ETH_DMAIER_TUIE = $00000020;  (* Transmit Underflow interrupt enable  *)
  ETH_DMAIER_ROIE = $00000010;  (* Receive Overflow interrupt enable  *)
  ETH_DMAIER_TJTIE = $00000008;  (* Transmit jabber timeout interrupt enable  *)
  ETH_DMAIER_TBUIE = $00000004;  (* Transmit buffer unavailable interrupt enable  *)
  ETH_DMAIER_TPSIE = $00000002;  (* Transmit process stopped interrupt enable  *)
  ETH_DMAIER_TIE = $00000001;  (* Transmit interrupt enable  *)
  (* Bit definition for Ethernet DMA Missed Frame and Buffer Overflow Counter Register  *)

  ETH_DMAMFBOCR_OFOC = $10000000;  (* Overflow bit for FIFO overflow counter  *)
  ETH_DMAMFBOCR_MFA = $0FFE0000;  (* Number of frames missed by the application  *)
  ETH_DMAMFBOCR_OMFC = $00010000;  (* Overflow bit for missed frame counter  *)
  ETH_DMAMFBOCR_MFC = $0000FFFF;  (* Number of frames missed by the controller  *)
  (* Bit definition for Ethernet DMA Current Host Transmit Descriptor Register  *)

  ETH_DMACHTDR_HTDAP = $FFFFFFFF;  (* Host transmit descriptor address pointer  *)
  (* Bit definition for Ethernet DMA Current Host Receive Descriptor Register  *)

  ETH_DMACHRDR_HRDAP = $FFFFFFFF;  (* Host receive descriptor address pointer  *)
  (* Bit definition for Ethernet DMA Current Host Transmit Buffer Address Register  *)

  ETH_DMACHTBAR_HTBAP = $FFFFFFFF;  (* Host transmit buffer address pointer  *)
  (* Bit definition for Ethernet DMA Current Host Receive Buffer Address Register  *)

  ETH_DMACHRBAR_HRBAP = $FFFFFFFF;  (* Host receive buffer address pointer  *)
  (***************************************************************************** *)

  (*                                                                             *)

  (*                                       USB_OTG                               *)

  (*                                                                             *)

  (***************************************************************************** *)

  (********************  Bit definition for USB_OTG_GOTGCTL register  ******************* *)

  USB_OTG_GOTGCTL_SRQSCS = $00000001;  (*!< Session request success  *)
  USB_OTG_GOTGCTL_SRQ = $00000002;  (*!< Session request  *)
  USB_OTG_GOTGCTL_VBVALOEN = $00000004;  (*!< VBUS valid override enable  *)
  USB_OTG_GOTGCTL_VBVALOVAL = $00000008;  (*!< VBUS valid override value  *)
  USB_OTG_GOTGCTL_AVALOEN = $00000010;  (*!< A-peripheral session valid override enable  *)
  USB_OTG_GOTGCTL_AVALOVAL = $00000020;  (*!< A-peripheral session valid override value  *)
  USB_OTG_GOTGCTL_BVALOEN = $00000040;  (*!< B-peripheral session valid override enable  *)
  USB_OTG_GOTGCTL_BVALOVAL = $00000080;  (*!< B-peripheral session valid override value   *)
  USB_OTG_GOTGCTL_HNGSCS = $00000100;  (*!< Host set HNP enable  *)
  USB_OTG_GOTGCTL_HNPRQ = $00000200;  (*!< HNP request  *)
  USB_OTG_GOTGCTL_HSHNPEN = $00000400;  (*!< Host set HNP enable  *)
  USB_OTG_GOTGCTL_DHNPEN = $00000800;  (*!< Device HNP enabled  *)
  USB_OTG_GOTGCTL_EHEN = $00001000;  (*!< Embedded host enable  *)
  USB_OTG_GOTGCTL_CIDSTS = $00010000;  (*!< Connector ID status  *)
  USB_OTG_GOTGCTL_DBCT = $00020000;  (*!< Long/short debounce time  *)
  USB_OTG_GOTGCTL_ASVLD = $00040000;  (*!< A-session valid   *)
  USB_OTG_GOTGCTL_BSESVLD = $00080000;  (*!< B-session valid  *)
  USB_OTG_GOTGCTL_OTGVER = $00100000;  (*!< OTG version   *)
  (********************  Bit definition for USB_OTG_HCFG register  ******************* *)

  USB_OTG_HCFG_FSLSPCS = $00000003;  (*!< FS/LS PHY clock select   *)
  USB_OTG_HCFG_FSLSPCS_0 = $00000001;  (*!<Bit 0  *)
  USB_OTG_HCFG_FSLSPCS_1 = $00000002;  (*!<Bit 1  *)
  USB_OTG_HCFG_FSLSS = $00000004;  (*!< FS- and LS-only support  *)
  (********************  Bit definition for USB_OTG_DCFG register  ******************* *)

  USB_OTG_DCFG_DSPD = $00000003;  (*!< Device speed  *)
  USB_OTG_DCFG_DSPD_0 = $00000001;  (*!<Bit 0  *)
  USB_OTG_DCFG_DSPD_1 = $00000002;  (*!<Bit 1  *)
  USB_OTG_DCFG_NZLSOHSK = $00000004;  (*!< Nonzero-length status OUT handshake  *)
  USB_OTG_DCFG_DAD = $000007F0;  (*!< Device address  *)
  USB_OTG_DCFG_DAD_0 = $00000010;  (*!<Bit 0  *)
  USB_OTG_DCFG_DAD_1 = $00000020;  (*!<Bit 1  *)
  USB_OTG_DCFG_DAD_2 = $00000040;  (*!<Bit 2  *)
  USB_OTG_DCFG_DAD_3 = $00000080;  (*!<Bit 3  *)
  USB_OTG_DCFG_DAD_4 = $00000100;  (*!<Bit 4  *)
  USB_OTG_DCFG_DAD_5 = $00000200;  (*!<Bit 5  *)
  USB_OTG_DCFG_DAD_6 = $00000400;  (*!<Bit 6  *)
  USB_OTG_DCFG_PFIVL = $00001800;  (*!< Periodic (micro)frame interval  *)
  USB_OTG_DCFG_PFIVL_0 = $00000800;  (*!<Bit 0  *)
  USB_OTG_DCFG_PFIVL_1 = $00001000;  (*!<Bit 1  *)
  USB_OTG_DCFG_PERSCHIVL = $03000000;  (*!< Periodic scheduling interval  *)
  USB_OTG_DCFG_PERSCHIVL_0 = $01000000;  (*!<Bit 0  *)
  USB_OTG_DCFG_PERSCHIVL_1 = $02000000;  (*!<Bit 1  *)
  (********************  Bit definition for USB_OTG_PCGCR register  ******************* *)

  USB_OTG_PCGCR_STPPCLK = $00000001;  (*!< Stop PHY clock  *)
  USB_OTG_PCGCR_GATEHCLK = $00000002;  (*!< Gate HCLK  *)
  USB_OTG_PCGCR_PHYSUSP = $00000010;  (*!< PHY suspended  *)
  (********************  Bit definition for USB_OTG_GOTGINT register  ******************* *)

  USB_OTG_GOTGINT_SEDET = $00000004;  (*!< Session end detected                    *)
  USB_OTG_GOTGINT_SRSSCHG = $00000100;  (*!< Session request success status change   *)
  USB_OTG_GOTGINT_HNSSCHG = $00000200;  (*!< Host negotiation success status change  *)
  USB_OTG_GOTGINT_HNGDET = $00020000;  (*!< Host negotiation detected               *)
  USB_OTG_GOTGINT_ADTOCHG = $00040000;  (*!< A-device timeout change                 *)
  USB_OTG_GOTGINT_DBCDNE = $00080000;  (*!< Debounce done                           *)
  USB_OTG_GOTGINT_IDCHNG = $00100000;  (*!< Change in ID pin input value            *)
  (********************  Bit definition for USB_OTG_DCTL register  ******************* *)

  USB_OTG_DCTL_RWUSIG = $00000001;  (*!< Remote wakeup signaling  *)
  USB_OTG_DCTL_SDIS = $00000002;  (*!< Soft disconnect          *)
  USB_OTG_DCTL_GINSTS = $00000004;  (*!< Global IN NAK status     *)
  USB_OTG_DCTL_GONSTS = $00000008;  (*!< Global OUT NAK status    *)
  USB_OTG_DCTL_TCTL = $00000070;  (*!< Test control  *)
  USB_OTG_DCTL_TCTL_0 = $00000010;  (*!<Bit 0  *)
  USB_OTG_DCTL_TCTL_1 = $00000020;  (*!<Bit 1  *)
  USB_OTG_DCTL_TCTL_2 = $00000040;  (*!<Bit 2  *)
  USB_OTG_DCTL_SGINAK = $00000080;  (*!< Set global IN NAK          *)
  USB_OTG_DCTL_CGINAK = $00000100;  (*!< Clear global IN NAK        *)
  USB_OTG_DCTL_SGONAK = $00000200;  (*!< Set global OUT NAK         *)
  USB_OTG_DCTL_CGONAK = $00000400;  (*!< Clear global OUT NAK       *)
  USB_OTG_DCTL_POPRGDNE = $00000800;  (*!< Power-on programming done  *)
  (********************  Bit definition for USB_OTG_HFIR register  ******************* *)

  USB_OTG_HFIR_FRIVL = $0000FFFF;  (*!< Frame interval  *)
  (********************  Bit definition for USB_OTG_HFNUM register  ******************* *)

  USB_OTG_HFNUM_FRNUM = $0000FFFF;  (*!< Frame number          *)
  USB_OTG_HFNUM_FTREM = $FFFF0000;  (*!< Frame time remaining  *)
  (********************  Bit definition for USB_OTG_DSTS register  ******************* *)

  USB_OTG_DSTS_SUSPSTS = $00000001;  (*!< Suspend status    *)
  USB_OTG_DSTS_ENUMSPD = $00000006;  (*!< Enumerated speed  *)
  USB_OTG_DSTS_ENUMSPD_0 = $00000002;  (*!<Bit 0  *)
  USB_OTG_DSTS_ENUMSPD_1 = $00000004;  (*!<Bit 1  *)
  USB_OTG_DSTS_EERR = $00000008;  (*!< Erratic error      *)
  USB_OTG_DSTS_FNSOF = $003FFF00;  (*!< Frame number of the received SOF  *)
  (********************  Bit definition for USB_OTG_GAHBCFG register  ******************* *)

  USB_OTG_GAHBCFG_GINT = $00000001;  (*!< Global interrupt mask  *)
  USB_OTG_GAHBCFG_HBSTLEN = $0000001E;  (*!< Burst length/type  *)
  USB_OTG_GAHBCFG_HBSTLEN_0 = $00000002;  (*!<Bit 0  *)
  USB_OTG_GAHBCFG_HBSTLEN_1 = $00000004;  (*!<Bit 1  *)
  USB_OTG_GAHBCFG_HBSTLEN_2 = $00000008;  (*!<Bit 2  *)
  USB_OTG_GAHBCFG_HBSTLEN_3 = $00000010;  (*!<Bit 3  *)
  USB_OTG_GAHBCFG_DMAEN = $00000020;  (*!< DMA enable  *)
  USB_OTG_GAHBCFG_TXFELVL = $00000080;  (*!< TxFIFO empty level  *)
  USB_OTG_GAHBCFG_PTXFELVL = $00000100;  (*!< Periodic TxFIFO empty level  *)
  (********************  Bit definition for USB_OTG_GUSBCFG register  ******************* *)

  USB_OTG_GUSBCFG_TOCAL = $00000007;  (*!< FS timeout calibration  *)
  USB_OTG_GUSBCFG_TOCAL_0 = $00000001;  (*!<Bit 0  *)
  USB_OTG_GUSBCFG_TOCAL_1 = $00000002;  (*!<Bit 1  *)
  USB_OTG_GUSBCFG_TOCAL_2 = $00000004;  (*!<Bit 2  *)
  USB_OTG_GUSBCFG_PHYSEL = $00000040;  (*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select  *)
  USB_OTG_GUSBCFG_SRPCAP = $00000100;  (*!< SRP-capable  *)
  USB_OTG_GUSBCFG_HNPCAP = $00000200;  (*!< HNP-capable  *)
  USB_OTG_GUSBCFG_TRDT = $00003C00;  (*!< USB turnaround time  *)
  USB_OTG_GUSBCFG_TRDT_0 = $00000400;  (*!<Bit 0  *)
  USB_OTG_GUSBCFG_TRDT_1 = $00000800;  (*!<Bit 1  *)
  USB_OTG_GUSBCFG_TRDT_2 = $00001000;  (*!<Bit 2  *)
  USB_OTG_GUSBCFG_TRDT_3 = $00002000;  (*!<Bit 3  *)
  USB_OTG_GUSBCFG_PHYLPCS = $00008000;  (*!< PHY Low-power clock select  *)
  USB_OTG_GUSBCFG_ULPIFSLS = $00020000;  (*!< ULPI FS/LS select                *)
  USB_OTG_GUSBCFG_ULPIAR = $00040000;  (*!< ULPI Auto-resume                 *)
  USB_OTG_GUSBCFG_ULPICSM = $00080000;  (*!< ULPI Clock SuspendM              *)
  USB_OTG_GUSBCFG_ULPIEVBUSD = $00100000;  (*!< ULPI External VBUS Drive         *)
  USB_OTG_GUSBCFG_ULPIEVBUSI = $00200000;  (*!< ULPI external VBUS indicator     *)
  USB_OTG_GUSBCFG_TSDPS = $00400000;  (*!< TermSel DLine pulsing selection  *)
  USB_OTG_GUSBCFG_PCCI = $00800000;  (*!< Indicator complement             *)
  USB_OTG_GUSBCFG_PTCI = $01000000;  (*!< Indicator pass through           *)
  USB_OTG_GUSBCFG_ULPIIPD = $02000000;  (*!< ULPI interface protect disable   *)
  USB_OTG_GUSBCFG_FHMOD = $20000000;  (*!< Forced host mode                 *)
  USB_OTG_GUSBCFG_FDMOD = $40000000;  (*!< Forced peripheral mode           *)
  USB_OTG_GUSBCFG_CTXPKT = $80000000;  (*!< Corrupt Tx packet                *)
  (********************  Bit definition for USB_OTG_GRSTCTL register  ******************* *)

  USB_OTG_GRSTCTL_CSRST = $00000001;  (*!< Core soft reset           *)
  USB_OTG_GRSTCTL_HSRST = $00000002;  (*!< HCLK soft reset           *)
  USB_OTG_GRSTCTL_FCRST = $00000004;  (*!< Host frame counter reset  *)
  USB_OTG_GRSTCTL_RXFFLSH = $00000010;  (*!< RxFIFO flush              *)
  USB_OTG_GRSTCTL_TXFFLSH = $00000020;  (*!< TxFIFO flush              *)
  USB_OTG_GRSTCTL_TXFNUM = $000007C0;  (*!< TxFIFO number  *)
  USB_OTG_GRSTCTL_TXFNUM_0 = $00000040;  (*!<Bit 0  *)
  USB_OTG_GRSTCTL_TXFNUM_1 = $00000080;  (*!<Bit 1  *)
  USB_OTG_GRSTCTL_TXFNUM_2 = $00000100;  (*!<Bit 2  *)
  USB_OTG_GRSTCTL_TXFNUM_3 = $00000200;  (*!<Bit 3  *)
  USB_OTG_GRSTCTL_TXFNUM_4 = $00000400;  (*!<Bit 4  *)
  USB_OTG_GRSTCTL_DMAREQ = $40000000;  (*!< DMA request signal  *)
  USB_OTG_GRSTCTL_AHBIDL = $80000000;  (*!< AHB master idle  *)
  (********************  Bit definition for USB_OTG_DIEPMSK register  ******************* *)

  USB_OTG_DIEPMSK_XFRCM = $00000001;  (*!< Transfer completed interrupt mask                  *)
  USB_OTG_DIEPMSK_EPDM = $00000002;  (*!< Endpoint disabled interrupt mask                   *)
  USB_OTG_DIEPMSK_TOM = $00000008;  (*!< Timeout condition mask (nonisochronous endpoints)  *)
  USB_OTG_DIEPMSK_ITTXFEMSK = $00000010;  (*!< IN token received when TxFIFO empty mask           *)
  USB_OTG_DIEPMSK_INEPNMM = $00000020;  (*!< IN token received with EP mismatch mask            *)
  USB_OTG_DIEPMSK_INEPNEM = $00000040;  (*!< IN endpoint NAK effective mask                     *)
  USB_OTG_DIEPMSK_TXFURM = $00000100;  (*!< FIFO underrun mask                                 *)
  USB_OTG_DIEPMSK_BIM = $00000200;  (*!< BNA interrupt mask                                 *)
  (********************  Bit definition for USB_OTG_HPTXSTS register  ******************* *)

  USB_OTG_HPTXSTS_PTXFSAVL = $0000FFFF;  (*!< Periodic transmit data FIFO space available      *)
  USB_OTG_HPTXSTS_PTXQSAV = $00FF0000;  (*!< Periodic transmit request queue space available  *)
  USB_OTG_HPTXSTS_PTXQSAV_0 = $00010000;  (*!<Bit 0  *)
  USB_OTG_HPTXSTS_PTXQSAV_1 = $00020000;  (*!<Bit 1  *)
  USB_OTG_HPTXSTS_PTXQSAV_2 = $00040000;  (*!<Bit 2  *)
  USB_OTG_HPTXSTS_PTXQSAV_3 = $00080000;  (*!<Bit 3  *)
  USB_OTG_HPTXSTS_PTXQSAV_4 = $00100000;  (*!<Bit 4  *)
  USB_OTG_HPTXSTS_PTXQSAV_5 = $00200000;  (*!<Bit 5  *)
  USB_OTG_HPTXSTS_PTXQSAV_6 = $00400000;  (*!<Bit 6  *)
  USB_OTG_HPTXSTS_PTXQSAV_7 = $00800000;  (*!<Bit 7  *)
  USB_OTG_HPTXSTS_PTXQTOP = $FF000000;  (*!< Top of the periodic transmit request queue  *)
  USB_OTG_HPTXSTS_PTXQTOP_0 = $01000000;  (*!<Bit 0  *)
  USB_OTG_HPTXSTS_PTXQTOP_1 = $02000000;  (*!<Bit 1  *)
  USB_OTG_HPTXSTS_PTXQTOP_2 = $04000000;  (*!<Bit 2  *)
  USB_OTG_HPTXSTS_PTXQTOP_3 = $08000000;  (*!<Bit 3  *)
  USB_OTG_HPTXSTS_PTXQTOP_4 = $10000000;  (*!<Bit 4  *)
  USB_OTG_HPTXSTS_PTXQTOP_5 = $20000000;  (*!<Bit 5  *)
  USB_OTG_HPTXSTS_PTXQTOP_6 = $40000000;  (*!<Bit 6  *)
  USB_OTG_HPTXSTS_PTXQTOP_7 = $80000000;  (*!<Bit 7  *)
  (********************  Bit definition for USB_OTG_HAINT register  ******************* *)

  USB_OTG_HAINT_HAINT = $0000FFFF;  (*!< Channel interrupts  *)
  (********************  Bit definition for USB_OTG_DOEPMSK register  ******************* *)

  USB_OTG_DOEPMSK_XFRCM = $00000001;  (*!< Transfer completed interrupt mask  *)
  USB_OTG_DOEPMSK_EPDM = $00000002;  (*!< Endpoint disabled interrupt mask                *)
  USB_OTG_DOEPMSK_STUPM = $00000008;  (*!< SETUP phase done mask                           *)
  USB_OTG_DOEPMSK_OTEPDM = $00000010;  (*!< OUT token received when endpoint disabled mask  *)
  USB_OTG_DOEPMSK_B2BSTUP = $00000040;  (*!< Back-to-back SETUP packets received mask        *)
  USB_OTG_DOEPMSK_OPEM = $00000100;  (*!< OUT packet error mask                           *)
  USB_OTG_DOEPMSK_BOIM = $00000200;  (*!< BNA interrupt mask                              *)
  (********************  Bit definition for USB_OTG_GINTSTS register  ******************* *)

  USB_OTG_GINTSTS_CMOD = $00000001;  (*!< Current mode of operation                       *)
  USB_OTG_GINTSTS_MMIS = $00000002;  (*!< Mode mismatch interrupt                         *)
  USB_OTG_GINTSTS_OTGINT = $00000004;  (*!< OTG interrupt                                   *)
  USB_OTG_GINTSTS_SOF = $00000008;  (*!< Start of frame                                  *)
  USB_OTG_GINTSTS_RXFLVL = $00000010;  (*!< RxFIFO nonempty                                 *)
  USB_OTG_GINTSTS_NPTXFE = $00000020;  (*!< Nonperiodic TxFIFO empty                        *)
  USB_OTG_GINTSTS_GINAKEFF = $00000040;  (*!< Global IN nonperiodic NAK effective             *)
  USB_OTG_GINTSTS_BOUTNAKEFF = $00000080;  (*!< Global OUT NAK effective                        *)
  USB_OTG_GINTSTS_ESUSP = $00000400;  (*!< Early suspend                                   *)
  USB_OTG_GINTSTS_USBSUSP = $00000800;  (*!< USB suspend                                     *)
  USB_OTG_GINTSTS_USBRST = $00001000;  (*!< USB reset                                       *)
  USB_OTG_GINTSTS_ENUMDNE = $00002000;  (*!< Enumeration done                                *)
  USB_OTG_GINTSTS_ISOODRP = $00004000;  (*!< Isochronous OUT packet dropped interrupt        *)
  USB_OTG_GINTSTS_EOPF = $00008000;  (*!< End of periodic frame interrupt                 *)
  USB_OTG_GINTSTS_IEPINT = $00040000;  (*!< IN endpoint interrupt                           *)
  USB_OTG_GINTSTS_OEPINT = $00080000;  (*!< OUT endpoint interrupt                          *)
  USB_OTG_GINTSTS_IISOIXFR = $00100000;  (*!< Incomplete isochronous IN transfer              *)
  USB_OTG_GINTSTS_PXFR_INCOMPISOOUT = $00200000;  (*!< Incomplete periodic transfer                    *)
  USB_OTG_GINTSTS_DATAFSUSP = $00400000;  (*!< Data fetch suspended                            *)
  USB_OTG_GINTSTS_RSTDET = $00800000;  (*!< Reset detected interrupt                        *)
  USB_OTG_GINTSTS_HPRTINT = $01000000;  (*!< Host port interrupt                             *)
  USB_OTG_GINTSTS_HCINT = $02000000;  (*!< Host channels interrupt                         *)
  USB_OTG_GINTSTS_PTXFE = $04000000;  (*!< Periodic TxFIFO empty                           *)
  USB_OTG_GINTSTS_LPMINT = $08000000;  (*!< LPM interrupt                                   *)
  USB_OTG_GINTSTS_CIDSCHG = $10000000;  (*!< Connector ID status change                      *)
  USB_OTG_GINTSTS_DISCINT = $20000000;  (*!< Disconnect detected interrupt                   *)
  USB_OTG_GINTSTS_SRQINT = $40000000;  (*!< Session request/new session detected interrupt  *)
  USB_OTG_GINTSTS_WKUINT = $80000000;  (*!< Resume/remote wakeup detected interrupt         *)
  (********************  Bit definition for USB_OTG_GINTMSK register  ******************* *)

  USB_OTG_GINTMSK_MMISM = $00000002;  (*!< Mode mismatch interrupt mask                         *)
  USB_OTG_GINTMSK_OTGINT = $00000004;  (*!< OTG interrupt mask                                   *)
  USB_OTG_GINTMSK_SOFM = $00000008;  (*!< Start of frame mask                                  *)
  USB_OTG_GINTMSK_RXFLVLM = $00000010;  (*!< Receive FIFO nonempty mask                           *)
  USB_OTG_GINTMSK_NPTXFEM = $00000020;  (*!< Nonperiodic TxFIFO empty mask                        *)
  USB_OTG_GINTMSK_GINAKEFFM = $00000040;  (*!< Global nonperiodic IN NAK effective mask             *)
  USB_OTG_GINTMSK_GONAKEFFM = $00000080;  (*!< Global OUT NAK effective mask                        *)
  USB_OTG_GINTMSK_ESUSPM = $00000400;  (*!< Early suspend mask                                   *)
  USB_OTG_GINTMSK_USBSUSPM = $00000800;  (*!< USB suspend mask                                     *)
  USB_OTG_GINTMSK_USBRST = $00001000;  (*!< USB reset mask                                       *)
  USB_OTG_GINTMSK_ENUMDNEM = $00002000;  (*!< Enumeration done mask                                *)
  USB_OTG_GINTMSK_ISOODRPM = $00004000;  (*!< Isochronous OUT packet dropped interrupt mask        *)
  USB_OTG_GINTMSK_EOPFM = $00008000;  (*!< End of periodic frame interrupt mask                 *)
  USB_OTG_GINTMSK_EPMISM = $00020000;  (*!< Endpoint mismatch interrupt mask                     *)
  USB_OTG_GINTMSK_IEPINT = $00040000;  (*!< IN endpoints interrupt mask                          *)
  USB_OTG_GINTMSK_OEPINT = $00080000;  (*!< OUT endpoints interrupt mask                         *)
  USB_OTG_GINTMSK_IISOIXFRM = $00100000;  (*!< Incomplete isochronous IN transfer mask              *)
  USB_OTG_GINTMSK_PXFRM_IISOOXFRM = $00200000;  (*!< Incomplete periodic transfer mask                    *)
  USB_OTG_GINTMSK_FSUSPM = $00400000;  (*!< Data fetch suspended mask                            *)
  USB_OTG_GINTMSK_RSTDEM = $00800000;  (*!< Reset detected interrupt mask                       *)
  USB_OTG_GINTMSK_PRTIM = $01000000;  (*!< Host port interrupt mask                             *)
  USB_OTG_GINTMSK_HCIM = $02000000;  (*!< Host channels interrupt mask                         *)
  USB_OTG_GINTMSK_PTXFEM = $04000000;  (*!< Periodic TxFIFO empty mask                           *)
  USB_OTG_GINTMSK_LPMINTM = $08000000;  (*!< LPM interrupt Mask                                   *)
  USB_OTG_GINTMSK_CIDSCHGM = $10000000;  (*!< Connector ID status change mask                      *)
  USB_OTG_GINTMSK_DISCINT = $20000000;  (*!< Disconnect detected interrupt mask                   *)
  USB_OTG_GINTMSK_SRQIM = $40000000;  (*!< Session request/new session detected interrupt mask  *)
  USB_OTG_GINTMSK_WUIM = $80000000;  (*!< Resume/remote wakeup detected interrupt mask         *)
  (********************  Bit definition for USB_OTG_DAINT register  ******************* *)

  USB_OTG_DAINT_IEPINT = $0000FFFF;  (*!< IN endpoint interrupt bits   *)
  USB_OTG_DAINT_OEPINT = $FFFF0000;  (*!< OUT endpoint interrupt bits  *)
  (********************  Bit definition for USB_OTG_HAINTMSK register  ******************* *)

  USB_OTG_HAINTMSK_HAINTM = $0000FFFF;  (*!< Channel interrupt mask  *)
  (********************  Bit definition for USB_OTG_GRXSTSP register  ******************* *)

  USB_OTG_GRXSTSP_EPNUM = $0000000F;  (*!< IN EP interrupt mask bits   *)
  USB_OTG_GRXSTSP_BCNT = $00007FF0;  (*!< OUT EP interrupt mask bits  *)
  USB_OTG_GRXSTSP_DPID = $00018000;  (*!< OUT EP interrupt mask bits  *)
  USB_OTG_GRXSTSP_PKTSTS = $001E0000;  (*!< OUT EP interrupt mask bits  *)
  (********************  Bit definition for USB_OTG_DAINTMSK register  ******************* *)

  USB_OTG_DAINTMSK_IEPM = $0000FFFF;  (*!< IN EP interrupt mask bits  *)
  USB_OTG_DAINTMSK_OEPM = $FFFF0000;  (*!< OUT EP interrupt mask bits  *)
  (********************  Bit definition for OTG register  ******************* *)

  USB_OTG_CHNUM = $0000000F;  (*!< Channel number  *)
  USB_OTG_CHNUM_0 = $00000001;  (*!<Bit 0  *)
  USB_OTG_CHNUM_1 = $00000002;  (*!<Bit 1  *)
  USB_OTG_CHNUM_2 = $00000004;  (*!<Bit 2  *)
  USB_OTG_CHNUM_3 = $00000008;  (*!<Bit 3  *)
  USB_OTG_BCNT = $00007FF0;  (*!< Byte count  *)
  USB_OTG_DPID = $00018000;  (*!< Data PID  *)
  USB_OTG_DPID_0 = $00008000;  (*!<Bit 0  *)
  USB_OTG_DPID_1 = $00010000;  (*!<Bit 1  *)
  USB_OTG_PKTSTS = $001E0000;  (*!< Packet status  *)
  USB_OTG_PKTSTS_0 = $00020000;  (*!<Bit 0  *)
  USB_OTG_PKTSTS_1 = $00040000;  (*!<Bit 1  *)
  USB_OTG_PKTSTS_2 = $00080000;  (*!<Bit 2  *)
  USB_OTG_PKTSTS_3 = $00100000;  (*!<Bit 3  *)
  USB_OTG_EPNUM = $0000000F;  (*!< Endpoint number  *)
  USB_OTG_EPNUM_0 = $00000001;  (*!<Bit 0  *)
  USB_OTG_EPNUM_1 = $00000002;  (*!<Bit 1  *)
  USB_OTG_EPNUM_2 = $00000004;  (*!<Bit 2  *)
  USB_OTG_EPNUM_3 = $00000008;  (*!<Bit 3  *)
  USB_OTG_FRMNUM = $01E00000;  (*!< Frame number  *)
  USB_OTG_FRMNUM_0 = $00200000;  (*!<Bit 0  *)
  USB_OTG_FRMNUM_1 = $00400000;  (*!<Bit 1  *)
  USB_OTG_FRMNUM_2 = $00800000;  (*!<Bit 2  *)
  USB_OTG_FRMNUM_3 = $01000000;  (*!<Bit 3  *)
  (********************  Bit definition for USB_OTG_GRXFSIZ register  ******************* *)

  USB_OTG_GRXFSIZ_RXFD = $0000FFFF;  (*!< RxFIFO depth  *)
  (********************  Bit definition for USB_OTG_DVBUSDIS register  ******************* *)

  USB_OTG_DVBUSDIS_VBUSDT = $0000FFFF;  (*!< Device VBUS discharge time  *)
  (********************  Bit definition for OTG register  ******************* *)

  USB_OTG_NPTXFSA = $0000FFFF;  (*!< Nonperiodic transmit RAM start address  *)
  USB_OTG_NPTXFD = $FFFF0000;  (*!< Nonperiodic TxFIFO depth                *)
  USB_OTG_TX0FSA = $0000FFFF;  (*!< Endpoint 0 transmit RAM start address   *)
  USB_OTG_TX0FD = $FFFF0000;  (*!< Endpoint 0 TxFIFO depth                 *)
  (********************  Bit definition for USB_OTG_DVBUSPULSE register  ******************* *)

  USB_OTG_DVBUSPULSE_DVBUSP = $00000FFF;  (*!< Device VBUS pulsing time  *)
  (********************  Bit definition for USB_OTG_GNPTXSTS register  ******************* *)

  USB_OTG_GNPTXSTS_NPTXFSAV = $0000FFFF;  (*!< Nonperiodic TxFIFO space available  *)
  USB_OTG_GNPTXSTS_NPTQXSAV = $00FF0000;  (*!< Nonperiodic transmit request queue space available  *)
  USB_OTG_GNPTXSTS_NPTQXSAV_0 = $00010000;  (*!<Bit 0  *)
  USB_OTG_GNPTXSTS_NPTQXSAV_1 = $00020000;  (*!<Bit 1  *)
  USB_OTG_GNPTXSTS_NPTQXSAV_2 = $00040000;  (*!<Bit 2  *)
  USB_OTG_GNPTXSTS_NPTQXSAV_3 = $00080000;  (*!<Bit 3  *)
  USB_OTG_GNPTXSTS_NPTQXSAV_4 = $00100000;  (*!<Bit 4  *)
  USB_OTG_GNPTXSTS_NPTQXSAV_5 = $00200000;  (*!<Bit 5  *)
  USB_OTG_GNPTXSTS_NPTQXSAV_6 = $00400000;  (*!<Bit 6  *)
  USB_OTG_GNPTXSTS_NPTQXSAV_7 = $00800000;  (*!<Bit 7  *)
  USB_OTG_GNPTXSTS_NPTXQTOP = $7F000000;  (*!< Top of the nonperiodic transmit request queue  *)
  USB_OTG_GNPTXSTS_NPTXQTOP_0 = $01000000;  (*!<Bit 0  *)
  USB_OTG_GNPTXSTS_NPTXQTOP_1 = $02000000;  (*!<Bit 1  *)
  USB_OTG_GNPTXSTS_NPTXQTOP_2 = $04000000;  (*!<Bit 2  *)
  USB_OTG_GNPTXSTS_NPTXQTOP_3 = $08000000;  (*!<Bit 3  *)
  USB_OTG_GNPTXSTS_NPTXQTOP_4 = $10000000;  (*!<Bit 4  *)
  USB_OTG_GNPTXSTS_NPTXQTOP_5 = $20000000;  (*!<Bit 5  *)
  USB_OTG_GNPTXSTS_NPTXQTOP_6 = $40000000;  (*!<Bit 6  *)
  (********************  Bit definition for USB_OTG_DTHRCTL register  ******************* *)

  USB_OTG_DTHRCTL_NONISOTHREN = $00000001;  (*!< Nonisochronous IN endpoints threshold enable  *)
  USB_OTG_DTHRCTL_ISOTHREN = $00000002;  (*!< ISO IN endpoint threshold enable  *)
  USB_OTG_DTHRCTL_TXTHRLEN = $000007FC;  (*!< Transmit threshold length  *)
  USB_OTG_DTHRCTL_TXTHRLEN_0 = $00000004;  (*!<Bit 0  *)
  USB_OTG_DTHRCTL_TXTHRLEN_1 = $00000008;  (*!<Bit 1  *)
  USB_OTG_DTHRCTL_TXTHRLEN_2 = $00000010;  (*!<Bit 2  *)
  USB_OTG_DTHRCTL_TXTHRLEN_3 = $00000020;  (*!<Bit 3  *)
  USB_OTG_DTHRCTL_TXTHRLEN_4 = $00000040;  (*!<Bit 4  *)
  USB_OTG_DTHRCTL_TXTHRLEN_5 = $00000080;  (*!<Bit 5  *)
  USB_OTG_DTHRCTL_TXTHRLEN_6 = $00000100;  (*!<Bit 6  *)
  USB_OTG_DTHRCTL_TXTHRLEN_7 = $00000200;  (*!<Bit 7  *)
  USB_OTG_DTHRCTL_TXTHRLEN_8 = $00000400;  (*!<Bit 8  *)
  USB_OTG_DTHRCTL_RXTHREN = $00010000;  (*!< Receive threshold enable  *)
  USB_OTG_DTHRCTL_RXTHRLEN = $03FE0000;  (*!< Receive threshold length  *)
  USB_OTG_DTHRCTL_RXTHRLEN_0 = $00020000;  (*!<Bit 0  *)
  USB_OTG_DTHRCTL_RXTHRLEN_1 = $00040000;  (*!<Bit 1  *)
  USB_OTG_DTHRCTL_RXTHRLEN_2 = $00080000;  (*!<Bit 2  *)
  USB_OTG_DTHRCTL_RXTHRLEN_3 = $00100000;  (*!<Bit 3  *)
  USB_OTG_DTHRCTL_RXTHRLEN_4 = $00200000;  (*!<Bit 4  *)
  USB_OTG_DTHRCTL_RXTHRLEN_5 = $00400000;  (*!<Bit 5  *)
  USB_OTG_DTHRCTL_RXTHRLEN_6 = $00800000;  (*!<Bit 6  *)
  USB_OTG_DTHRCTL_RXTHRLEN_7 = $01000000;  (*!<Bit 7  *)
  USB_OTG_DTHRCTL_RXTHRLEN_8 = $02000000;  (*!<Bit 8  *)
  USB_OTG_DTHRCTL_ARPEN = $08000000;  (*!< Arbiter parking enable  *)
  (********************  Bit definition for USB_OTG_DIEPEMPMSK register  ******************* *)

  USB_OTG_DIEPEMPMSK_INEPTXFEM = $0000FFFF;  (*!< IN EP Tx FIFO empty interrupt mask bits  *)
  (********************  Bit definition for USB_OTG_DEACHINT register  ******************* *)

  USB_OTG_DEACHINT_IEP1INT = $00000002;  (*!< IN endpoint 1interrupt bit    *)
  USB_OTG_DEACHINT_OEP1INT = $00020000;  (*!< OUT endpoint 1 interrupt bit  *)
  (********************  Bit definition for USB_OTG_GCCFG register  ******************* *)

  USB_OTG_GCCFG_PWRDWN = $00010000;  (*!< Power down  *)
  USB_OTG_GCCFG_VBDEN = $00200000;  (*!< USB VBUS Detection Enable  *)
  (********************  Bit definition for USB_OTG_GPWRDN) register  ******************* *)

  USB_OTG_GPWRDN_ADPMEN = $00000001;  (*!< ADP module enable  *)
  USB_OTG_GPWRDN_ADPIF = $00800000;  (*!< ADP Interrupt flag  *)
  (********************  Bit definition for USB_OTG_DEACHINTMSK register  ******************* *)

  USB_OTG_DEACHINTMSK_IEP1INTM = $00000002;  (*!< IN Endpoint 1 interrupt mask bit   *)
  USB_OTG_DEACHINTMSK_OEP1INTM = $00020000;  (*!< OUT Endpoint 1 interrupt mask bit  *)
  (********************  Bit definition for USB_OTG_CID register  ******************* *)

  USB_OTG_CID_PRODUCT_ID = $FFFFFFFF;  (*!< Product ID field  *)
  (********************  Bit definition for USB_OTG_GLPMCFG register  ******************* *)

  USB_OTG_GLPMCFG_LPMEN = $00000001;  (*!< LPM support enable                                      *)
  USB_OTG_GLPMCFG_LPMACK = $00000002;  (*!< LPM Token acknowledge enable                            *)
  USB_OTG_GLPMCFG_BESL = $0000003C;  (*!< BESL value received with last ACKed LPM Token           *)
  USB_OTG_GLPMCFG_REMWAKE = $00000040;  (*!< bRemoteWake value received with last ACKed LPM Token    *)
  USB_OTG_GLPMCFG_L1SSEN = $00000080;  (*!< L1 shallow sleep enable                                 *)
  USB_OTG_GLPMCFG_BESLTHRS = $00000F00;  (*!< BESL threshold                                          *)
  USB_OTG_GLPMCFG_L1DSEN = $00001000;  (*!< L1 deep sleep enable                                    *)
  USB_OTG_GLPMCFG_LPMRSP = $00006000;  (*!< LPM response                                            *)
  USB_OTG_GLPMCFG_SLPSTS = $00008000;  (*!< Port sleep status                                       *)
  USB_OTG_GLPMCFG_L1RSMOK = $00010000;  (*!< Sleep State Resume OK                                   *)
  USB_OTG_GLPMCFG_LPMCHIDX = $001E0000;  (*!< LPM Channel Index                                       *)
  USB_OTG_GLPMCFG_LPMRCNT = $00E00000;  (*!< LPM retry count                                         *)
  USB_OTG_GLPMCFG_SNDLPM = $01000000;  (*!< Send LPM transaction                                    *)
  USB_OTG_GLPMCFG_LPMRCNTSTS = $0E000000;  (*!< LPM retry count status                                  *)
  USB_OTG_GLPMCFG_ENBESL = $10000000;  (*!< Enable best effort service latency                      *)
  (********************  Bit definition for USB_OTG_DIEPEACHMSK1 register  ******************* *)

  USB_OTG_DIEPEACHMSK1_XFRCM = $00000001;  (*!< Transfer completed interrupt mask                  *)
  USB_OTG_DIEPEACHMSK1_EPDM = $00000002;  (*!< Endpoint disabled interrupt mask                   *)
  USB_OTG_DIEPEACHMSK1_TOM = $00000008;  (*!< Timeout condition mask (nonisochronous endpoints)  *)
  USB_OTG_DIEPEACHMSK1_ITTXFEMSK = $00000010;  (*!< IN token received when TxFIFO empty mask           *)
  USB_OTG_DIEPEACHMSK1_INEPNMM = $00000020;  (*!< IN token received with EP mismatch mask            *)
  USB_OTG_DIEPEACHMSK1_INEPNEM = $00000040;  (*!< IN endpoint NAK effective mask                     *)
  USB_OTG_DIEPEACHMSK1_TXFURM = $00000100;  (*!< FIFO underrun mask                                 *)
  USB_OTG_DIEPEACHMSK1_BIM = $00000200;  (*!< BNA interrupt mask                                 *)
  USB_OTG_DIEPEACHMSK1_NAKM = $00002000;  (*!< NAK interrupt mask                                 *)
  (********************  Bit definition for USB_OTG_HPRT register  ******************* *)

  USB_OTG_HPRT_PCSTS = $00000001;  (*!< Port connect status         *)
  USB_OTG_HPRT_PCDET = $00000002;  (*!< Port connect detected       *)
  USB_OTG_HPRT_PENA = $00000004;  (*!< Port enable                 *)
  USB_OTG_HPRT_PENCHNG = $00000008;  (*!< Port enable/disable change  *)
  USB_OTG_HPRT_POCA = $00000010;  (*!< Port overcurrent active     *)
  USB_OTG_HPRT_POCCHNG = $00000020;  (*!< Port overcurrent change     *)
  USB_OTG_HPRT_PRES = $00000040;  (*!< Port resume                 *)
  USB_OTG_HPRT_PSUSP = $00000080;  (*!< Port suspend                *)
  USB_OTG_HPRT_PRST = $00000100;  (*!< Port reset                  *)
  USB_OTG_HPRT_PLSTS = $00000C00;  (*!< Port line status            *)
  USB_OTG_HPRT_PLSTS_0 = $00000400;  (*!<Bit 0  *)
  USB_OTG_HPRT_PLSTS_1 = $00000800;  (*!<Bit 1  *)
  USB_OTG_HPRT_PPWR = $00001000;  (*!< Port power                  *)
  USB_OTG_HPRT_PTCTL = $0001E000;  (*!< Port test control           *)
  USB_OTG_HPRT_PTCTL_0 = $00002000;  (*!<Bit 0  *)
  USB_OTG_HPRT_PTCTL_1 = $00004000;  (*!<Bit 1  *)
  USB_OTG_HPRT_PTCTL_2 = $00008000;  (*!<Bit 2  *)
  USB_OTG_HPRT_PTCTL_3 = $00010000;  (*!<Bit 3  *)
  USB_OTG_HPRT_PSPD = $00060000;  (*!< Port speed                  *)
  USB_OTG_HPRT_PSPD_0 = $00020000;  (*!<Bit 0  *)
  USB_OTG_HPRT_PSPD_1 = $00040000;  (*!<Bit 1  *)
  (********************  Bit definition for USB_OTG_DOEPEACHMSK1 register  ******************* *)

  USB_OTG_DOEPEACHMSK1_XFRCM = $00000001;  (*!< Transfer completed interrupt mask          *)
  USB_OTG_DOEPEACHMSK1_EPDM = $00000002;  (*!< Endpoint disabled interrupt mask           *)
  USB_OTG_DOEPEACHMSK1_TOM = $00000008;  (*!< Timeout condition mask                     *)
  USB_OTG_DOEPEACHMSK1_ITTXFEMSK = $00000010;  (*!< IN token received when TxFIFO empty mask   *)
  USB_OTG_DOEPEACHMSK1_INEPNMM = $00000020;  (*!< IN token received with EP mismatch mask    *)
  USB_OTG_DOEPEACHMSK1_INEPNEM = $00000040;  (*!< IN endpoint NAK effective mask             *)
  USB_OTG_DOEPEACHMSK1_TXFURM = $00000100;  (*!< OUT packet error mask                      *)
  USB_OTG_DOEPEACHMSK1_BIM = $00000200;  (*!< BNA interrupt mask                         *)
  USB_OTG_DOEPEACHMSK1_BERRM = $00001000;  (*!< Bubble error interrupt mask                *)
  USB_OTG_DOEPEACHMSK1_NAKM = $00002000;  (*!< NAK interrupt mask                         *)
  USB_OTG_DOEPEACHMSK1_NYETM = $00004000;  (*!< NYET interrupt mask                        *)
  (********************  Bit definition for USB_OTG_HPTXFSIZ register  ******************* *)

  USB_OTG_HPTXFSIZ_PTXSA = $0000FFFF;  (*!< Host periodic TxFIFO start address             *)
  USB_OTG_HPTXFSIZ_PTXFD = $FFFF0000;  (*!< Host periodic TxFIFO depth                     *)
  (********************  Bit definition for USB_OTG_DIEPCTL register  ******************* *)

  USB_OTG_DIEPCTL_MPSIZ = $000007FF;  (*!< Maximum packet size               *)
  USB_OTG_DIEPCTL_USBAEP = $00008000;  (*!< USB active endpoint               *)
  USB_OTG_DIEPCTL_EONUM_DPID = $00010000;  (*!< Even/odd frame                    *)
  USB_OTG_DIEPCTL_NAKSTS = $00020000;  (*!< NAK status                        *)
  USB_OTG_DIEPCTL_EPTYP = $000C0000;  (*!< Endpoint type                     *)
  USB_OTG_DIEPCTL_EPTYP_0 = $00040000;  (*!<Bit 0  *)
  USB_OTG_DIEPCTL_EPTYP_1 = $00080000;  (*!<Bit 1  *)
  USB_OTG_DIEPCTL_STALL = $00200000;  (*!< STALL handshake                   *)
  USB_OTG_DIEPCTL_TXFNUM = $03C00000;  (*!< TxFIFO number                     *)
  USB_OTG_DIEPCTL_TXFNUM_0 = $00400000;  (*!<Bit 0  *)
  USB_OTG_DIEPCTL_TXFNUM_1 = $00800000;  (*!<Bit 1  *)
  USB_OTG_DIEPCTL_TXFNUM_2 = $01000000;  (*!<Bit 2  *)
  USB_OTG_DIEPCTL_TXFNUM_3 = $02000000;  (*!<Bit 3  *)
  USB_OTG_DIEPCTL_CNAK = $04000000;  (*!< Clear NAK                         *)
  USB_OTG_DIEPCTL_SNAK = $08000000;  (*!< Set NAK  *)
  USB_OTG_DIEPCTL_SD0PID_SEVNFRM = $10000000;  (*!< Set DATA0 PID                     *)
  USB_OTG_DIEPCTL_SODDFRM = $20000000;  (*!< Set odd frame                     *)
  USB_OTG_DIEPCTL_EPDIS = $40000000;  (*!< Endpoint disable                  *)
  USB_OTG_DIEPCTL_EPENA = $80000000;  (*!< Endpoint enable                   *)
  (********************  Bit definition for USB_OTG_HCCHAR register  ******************* *)

  USB_OTG_HCCHAR_MPSIZ = $000007FF;  (*!< Maximum packet size  *)
  USB_OTG_HCCHAR_EPNUM = $00007800;  (*!< Endpoint number  *)
  USB_OTG_HCCHAR_EPNUM_0 = $00000800;  (*!<Bit 0  *)
  USB_OTG_HCCHAR_EPNUM_1 = $00001000;  (*!<Bit 1  *)
  USB_OTG_HCCHAR_EPNUM_2 = $00002000;  (*!<Bit 2  *)
  USB_OTG_HCCHAR_EPNUM_3 = $00004000;  (*!<Bit 3  *)
  USB_OTG_HCCHAR_EPDIR = $00008000;  (*!< Endpoint direction  *)
  USB_OTG_HCCHAR_LSDEV = $00020000;  (*!< Low-speed device  *)
  USB_OTG_HCCHAR_EPTYP = $000C0000;  (*!< Endpoint type  *)
  USB_OTG_HCCHAR_EPTYP_0 = $00040000;  (*!<Bit 0  *)
  USB_OTG_HCCHAR_EPTYP_1 = $00080000;  (*!<Bit 1  *)
  USB_OTG_HCCHAR_MC = $00300000;  (*!< Multi Count (MC) / Error Count (EC)  *)
  USB_OTG_HCCHAR_MC_0 = $00100000;  (*!<Bit 0  *)
  USB_OTG_HCCHAR_MC_1 = $00200000;  (*!<Bit 1  *)
  USB_OTG_HCCHAR_DAD = $1FC00000;  (*!< Device address  *)
  USB_OTG_HCCHAR_DAD_0 = $00400000;  (*!<Bit 0  *)
  USB_OTG_HCCHAR_DAD_1 = $00800000;  (*!<Bit 1  *)
  USB_OTG_HCCHAR_DAD_2 = $01000000;  (*!<Bit 2  *)
  USB_OTG_HCCHAR_DAD_3 = $02000000;  (*!<Bit 3  *)
  USB_OTG_HCCHAR_DAD_4 = $04000000;  (*!<Bit 4  *)
  USB_OTG_HCCHAR_DAD_5 = $08000000;  (*!<Bit 5  *)
  USB_OTG_HCCHAR_DAD_6 = $10000000;  (*!<Bit 6  *)
  USB_OTG_HCCHAR_ODDFRM = $20000000;  (*!< Odd frame  *)
  USB_OTG_HCCHAR_CHDIS = $40000000;  (*!< Channel disable  *)
  USB_OTG_HCCHAR_CHENA = $80000000;  (*!< Channel enable  *)
  (********************  Bit definition for USB_OTG_HCSPLT register  ******************* *)

  USB_OTG_HCSPLT_PRTADDR = $0000007F;  (*!< Port address  *)
  USB_OTG_HCSPLT_PRTADDR_0 = $00000001;  (*!<Bit 0  *)
  USB_OTG_HCSPLT_PRTADDR_1 = $00000002;  (*!<Bit 1  *)
  USB_OTG_HCSPLT_PRTADDR_2 = $00000004;  (*!<Bit 2  *)
  USB_OTG_HCSPLT_PRTADDR_3 = $00000008;  (*!<Bit 3  *)
  USB_OTG_HCSPLT_PRTADDR_4 = $00000010;  (*!<Bit 4  *)
  USB_OTG_HCSPLT_PRTADDR_5 = $00000020;  (*!<Bit 5  *)
  USB_OTG_HCSPLT_PRTADDR_6 = $00000040;  (*!<Bit 6  *)
  USB_OTG_HCSPLT_HUBADDR = $00003F80;  (*!< Hub address  *)
  USB_OTG_HCSPLT_HUBADDR_0 = $00000080;  (*!<Bit 0  *)
  USB_OTG_HCSPLT_HUBADDR_1 = $00000100;  (*!<Bit 1  *)
  USB_OTG_HCSPLT_HUBADDR_2 = $00000200;  (*!<Bit 2  *)
  USB_OTG_HCSPLT_HUBADDR_3 = $00000400;  (*!<Bit 3  *)
  USB_OTG_HCSPLT_HUBADDR_4 = $00000800;  (*!<Bit 4  *)
  USB_OTG_HCSPLT_HUBADDR_5 = $00001000;  (*!<Bit 5  *)
  USB_OTG_HCSPLT_HUBADDR_6 = $00002000;  (*!<Bit 6  *)
  USB_OTG_HCSPLT_XACTPOS = $0000C000;  (*!< XACTPOS  *)
  USB_OTG_HCSPLT_XACTPOS_0 = $00004000;  (*!<Bit 0  *)
  USB_OTG_HCSPLT_XACTPOS_1 = $00008000;  (*!<Bit 1  *)
  USB_OTG_HCSPLT_COMPLSPLT = $00010000;  (*!< Do complete split  *)
  USB_OTG_HCSPLT_SPLITEN = $80000000;  (*!< Split enable  *)
  (********************  Bit definition for USB_OTG_HCINT register  ******************* *)

  USB_OTG_HCINT_XFRC = $00000001;  (*!< Transfer completed  *)
  USB_OTG_HCINT_CHH = $00000002;  (*!< Channel halted  *)
  USB_OTG_HCINT_AHBERR = $00000004;  (*!< AHB error  *)
  USB_OTG_HCINT_STALL = $00000008;  (*!< STALL response received interrupt  *)
  USB_OTG_HCINT_NAK = $00000010;  (*!< NAK response received interrupt  *)
  USB_OTG_HCINT_ACK = $00000020;  (*!< ACK response received/transmitted interrupt  *)
  USB_OTG_HCINT_NYET = $00000040;  (*!< Response received interrupt  *)
  USB_OTG_HCINT_TXERR = $00000080;  (*!< Transaction error  *)
  USB_OTG_HCINT_BBERR = $00000100;  (*!< Babble error  *)
  USB_OTG_HCINT_FRMOR = $00000200;  (*!< Frame overrun  *)
  USB_OTG_HCINT_DTERR = $00000400;  (*!< Data toggle error  *)
  (********************  Bit definition for USB_OTG_DIEPINT register  ******************* *)

  USB_OTG_DIEPINT_XFRC = $00000001;  (*!< Transfer completed interrupt  *)
  USB_OTG_DIEPINT_EPDISD = $00000002;  (*!< Endpoint disabled interrupt  *)
  USB_OTG_DIEPINT_TOC = $00000008;  (*!< Timeout condition  *)
  USB_OTG_DIEPINT_ITTXFE = $00000010;  (*!< IN token received when TxFIFO is empty  *)
  USB_OTG_DIEPINT_INEPNE = $00000040;  (*!< IN endpoint NAK effective  *)
  USB_OTG_DIEPINT_TXFE = $00000080;  (*!< Transmit FIFO empty  *)
  USB_OTG_DIEPINT_TXFIFOUDRN = $00000100;  (*!< Transmit Fifo Underrun  *)
  USB_OTG_DIEPINT_BNA = $00000200;  (*!< Buffer not available interrupt  *)
  USB_OTG_DIEPINT_PKTDRPSTS = $00000800;  (*!< Packet dropped status  *)
  USB_OTG_DIEPINT_BERR = $00001000;  (*!< Babble error interrupt  *)
  USB_OTG_DIEPINT_NAK = $00002000;  (*!< NAK interrupt  *)
  (********************  Bit definition for USB_OTG_HCINTMSK register  ******************* *)

  USB_OTG_HCINTMSK_XFRCM = $00000001;  (*!< Transfer completed mask  *)
  USB_OTG_HCINTMSK_CHHM = $00000002;  (*!< Channel halted mask  *)
  USB_OTG_HCINTMSK_AHBERR = $00000004;  (*!< AHB error  *)
  USB_OTG_HCINTMSK_STALLM = $00000008;  (*!< STALL response received interrupt mask  *)
  USB_OTG_HCINTMSK_NAKM = $00000010;  (*!< NAK response received interrupt mask  *)
  USB_OTG_HCINTMSK_ACKM = $00000020;  (*!< ACK response received/transmitted interrupt mask  *)
  USB_OTG_HCINTMSK_NYET = $00000040;  (*!< response received interrupt mask  *)
  USB_OTG_HCINTMSK_TXERRM = $00000080;  (*!< Transaction error mask  *)
  USB_OTG_HCINTMSK_BBERRM = $00000100;  (*!< Babble error mask  *)
  USB_OTG_HCINTMSK_FRMORM = $00000200;  (*!< Frame overrun mask  *)
  USB_OTG_HCINTMSK_DTERRM = $00000400;  (*!< Data toggle error mask  *)
  (********************  Bit definition for USB_OTG_DIEPTSIZ register  ******************* *)

  USB_OTG_DIEPTSIZ_XFRSIZ = $0007FFFF;  (*!< Transfer size  *)
  USB_OTG_DIEPTSIZ_PKTCNT = $1FF80000;  (*!< Packet count  *)
  USB_OTG_DIEPTSIZ_MULCNT = $60000000;  (*!< Packet count  *)
  (********************  Bit definition for USB_OTG_HCTSIZ register  ******************* *)

  USB_OTG_HCTSIZ_XFRSIZ = $0007FFFF;  (*!< Transfer size  *)
  USB_OTG_HCTSIZ_PKTCNT = $1FF80000;  (*!< Packet count  *)
  USB_OTG_HCTSIZ_DOPING = $80000000;  (*!< Do PING  *)
  USB_OTG_HCTSIZ_DPID = $60000000;  (*!< Data PID  *)
  USB_OTG_HCTSIZ_DPID_0 = $20000000;  (*!<Bit 0  *)
  USB_OTG_HCTSIZ_DPID_1 = $40000000;  (*!<Bit 1  *)
  (********************  Bit definition for USB_OTG_DIEPDMA register  ******************* *)

  USB_OTG_DIEPDMA_DMAADDR = $FFFFFFFF;  (*!< DMA address  *)
  (********************  Bit definition for USB_OTG_HCDMA register  ******************* *)

  USB_OTG_HCDMA_DMAADDR = $FFFFFFFF;  (*!< DMA address  *)
  (********************  Bit definition for USB_OTG_DTXFSTS register  ******************* *)

  USB_OTG_DTXFSTS_INEPTFSAV = $0000FFFF;  (*!< IN endpoint TxFIFO space available  *)
  (********************  Bit definition for USB_OTG_DIEPTXF register  ******************* *)

  USB_OTG_DIEPTXF_INEPTXSA = $0000FFFF;  (*!< IN endpoint FIFOx transmit RAM start address  *)
  USB_OTG_DIEPTXF_INEPTXFD = $FFFF0000;  (*!< IN endpoint TxFIFO depth  *)
  (********************  Bit definition for USB_OTG_DOEPCTL register  ******************* *)

  USB_OTG_DOEPCTL_MPSIZ = $000007FF;  (*!< Maximum packet size  *)(*!<Bit 1  *)
  USB_OTG_DOEPCTL_USBAEP = $00008000;  (*!< USB active endpoint  *)
  USB_OTG_DOEPCTL_NAKSTS = $00020000;  (*!< NAK status  *)
  USB_OTG_DOEPCTL_SD0PID_SEVNFRM = $10000000;  (*!< Set DATA0 PID  *)
  USB_OTG_DOEPCTL_SODDFRM = $20000000;  (*!< Set odd frame  *)
  USB_OTG_DOEPCTL_EPTYP = $000C0000;  (*!< Endpoint type  *)
  USB_OTG_DOEPCTL_EPTYP_0 = $00040000;  (*!<Bit 0  *)
  USB_OTG_DOEPCTL_EPTYP_1 = $00080000;  (*!<Bit 1  *)
  USB_OTG_DOEPCTL_SNPM = $00100000;  (*!< Snoop mode  *)
  USB_OTG_DOEPCTL_STALL = $00200000;  (*!< STALL handshake  *)
  USB_OTG_DOEPCTL_CNAK = $04000000;  (*!< Clear NAK  *)
  USB_OTG_DOEPCTL_SNAK = $08000000;  (*!< Set NAK  *)
  USB_OTG_DOEPCTL_EPDIS = $40000000;  (*!< Endpoint disable  *)
  USB_OTG_DOEPCTL_EPENA = $80000000;  (*!< Endpoint enable  *)
  (********************  Bit definition for USB_OTG_DOEPINT register  ******************* *)

  USB_OTG_DOEPINT_XFRC = $00000001;  (*!< Transfer completed interrupt  *)
  USB_OTG_DOEPINT_EPDISD = $00000002;  (*!< Endpoint disabled interrupt  *)
  USB_OTG_DOEPINT_STUP = $00000008;  (*!< SETUP phase done  *)
  USB_OTG_DOEPINT_OTEPDIS = $00000010;  (*!< OUT token received when endpoint disabled  *)
  USB_OTG_DOEPINT_B2BSTUP = $00000040;  (*!< Back-to-back SETUP packets received  *)
  USB_OTG_DOEPINT_NYET = $00004000;  (*!< NYET interrupt  *)
  (********************  Bit definition for USB_OTG_DOEPTSIZ register  ******************* *)

  USB_OTG_DOEPTSIZ_XFRSIZ = $0007FFFF;  (*!< Transfer size  *)
  USB_OTG_DOEPTSIZ_PKTCNT = $1FF80000;  (*!< Packet count  *)
  USB_OTG_DOEPTSIZ_STUPCNT = $60000000;  (*!< SETUP packet count  *)
  USB_OTG_DOEPTSIZ_STUPCNT_0 = $20000000;  (*!<Bit 0  *)
  USB_OTG_DOEPTSIZ_STUPCNT_1 = $40000000;  (*!<Bit 1  *)
  (********************  Bit definition for PCGCCTL register  ******************* *)

  USB_OTG_PCGCCTL_STOPCLK = $00000001;  (*!< SETUP packet count  *)
  USB_OTG_PCGCCTL_GATECLK = $00000002;  (*!<Bit 0  *)
  USB_OTG_PCGCCTL_PHYSUSP = $00000010;  (*!<Bit 1  *)

(************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE*** *)
