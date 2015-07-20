(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_cortex.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of CORTEX HAL module.
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

unit stm32f7xx_hal_cortex;

interface

const
  (******  Cortex-M7 Processor Exceptions Numbers *************************************************************** *)
  NonMaskableInt_IRQn = -14;
  (*!< 2 Non Maskable Interrupt                                           *)
  MemoryManagement_IRQn = -12;
  (*!< 4 Cortex-M7 Memory Management Interrupt                            *)
  BusFault_IRQn = -11;
  (*!< 5 Cortex-M7 Bus Fault Interrupt                                    *)
  UsageFault_IRQn = -10;
  (*!< 6 Cortex-M7 Usage Fault Interrupt                                  *)
  SVCall_IRQn = -5;
  (*!< 11 Cortex-M7 SV Call Interrupt                                     *)
  DebugMonitor_IRQn = -4;
  (*!< 12 Cortex-M7 Debug Monitor Interrupt                               *)
  PendSV_IRQn = -2;
  (*!< 14 Cortex-M7 Pend SV Interrupt                                     *)
  SysTick_IRQn = -1;
  (*!< 15 Cortex-M7 System Tick Interrupt                                 *)
  (******  STM32 specific Interrupt Numbers ********************************************************************* *)
  WWDG_IRQn = 0;  (*!< Window WatchDog Interrupt                                          *)
  PVD_IRQn = 1;  (*!< PVD through EXTI Line detection Interrupt                          *)
  TAMP_STAMP_IRQn = 2;
  (*!< Tamper and TimeStamp interrupts through the EXTI line              *)
  RTC_WKUP_IRQn = 3;
  (*!< RTC Wakeup interrupt through the EXTI line                         *)
  FLASH_IRQn = 4;  (*!< FLASH global Interrupt                                             *)
  RCC_IRQn = 5;  (*!< RCC global Interrupt                                               *)
  EXTI0_IRQn = 6;  (*!< EXTI Line0 Interrupt                                               *)
  EXTI1_IRQn = 7;  (*!< EXTI Line1 Interrupt                                               *)
  EXTI2_IRQn = 8;  (*!< EXTI Line2 Interrupt                                               *)
  EXTI3_IRQn = 9;  (*!< EXTI Line3 Interrupt                                               *)
  EXTI4_IRQn = 10;  (*!< EXTI Line4 Interrupt                                               *)
  DMA1_Stream0_IRQn = 11;
  (*!< DMA1 Stream 0 global Interrupt                                     *)
  DMA1_Stream1_IRQn = 12;
  (*!< DMA1 Stream 1 global Interrupt                                     *)
  DMA1_Stream2_IRQn = 13;
  (*!< DMA1 Stream 2 global Interrupt                                     *)
  DMA1_Stream3_IRQn = 14;
  (*!< DMA1 Stream 3 global Interrupt                                     *)
  DMA1_Stream4_IRQn = 15;
  (*!< DMA1 Stream 4 global Interrupt                                     *)
  DMA1_Stream5_IRQn = 16;
  (*!< DMA1 Stream 5 global Interrupt                                     *)
  DMA1_Stream6_IRQn = 17;
  (*!< DMA1 Stream 6 global Interrupt                                     *)
  ADC_IRQn = 18;  (*!< ADC1, ADC2 and ADC3 global Interrupts                              *)
  CAN1_TX_IRQn = 19;
  (*!< CAN1 TX Interrupt                                                  *)
  CAN1_RX0_IRQn = 20;
  (*!< CAN1 RX0 Interrupt                                                 *)
  CAN1_RX1_IRQn = 21;
  (*!< CAN1 RX1 Interrupt                                                 *)
  CAN1_SCE_IRQn = 22;
  (*!< CAN1 SCE Interrupt                                                 *)
  EXTI9_5_IRQn = 23;
  (*!< External Line[9:5] Interrupts                                      *)
  TIM1_BRK_TIM9_IRQn = 24;
  (*!< TIM1 Break interrupt and TIM9 global interrupt                     *)
  TIM1_UP_TIM10_IRQn = 25;
  (*!< TIM1 Update Interrupt and TIM10 global interrupt                   *)
  TIM1_TRG_COM_TIM11_IRQn = 26;
  (*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt  *)
  TIM1_CC_IRQn = 27;
  (*!< TIM1 Capture Compare Interrupt                                     *)
  TIM2_IRQn = 28;  (*!< TIM2 global Interrupt                                              *)
  TIM3_IRQn = 29;  (*!< TIM3 global Interrupt                                              *)
  TIM4_IRQn = 30;  (*!< TIM4 global Interrupt                                              *)
  I2C1_EV_IRQn = 31;
  (*!< I2C1 Event Interrupt                                               *)
  I2C1_ER_IRQn = 32;
  (*!< I2C1 Error Interrupt                                               *)
  I2C2_EV_IRQn = 33;
  (*!< I2C2 Event Interrupt                                               *)
  I2C2_ER_IRQn = 34;
  (*!< I2C2 Error Interrupt                                               *)
  SPI1_IRQn = 35;  (*!< SPI1 global Interrupt                                              *)
  SPI2_IRQn = 36;  (*!< SPI2 global Interrupt                                              *)
  USART1_IRQn = 37;
  (*!< USART1 global Interrupt                                            *)
  USART2_IRQn = 38;
  (*!< USART2 global Interrupt                                            *)
  USART3_IRQn = 39;
  (*!< USART3 global Interrupt                                            *)
  EXTI15_10_IRQn = 40;
  (*!< External Line[15:10] Interrupts                                    *)
  RTC_Alarm_IRQn = 41;
  (*!< RTC Alarm (A and B) through EXTI Line Interrupt                    *)
  OTG_FS_WKUP_IRQn = 42;
  (*!< USB OTG FS Wakeup through EXTI line interrupt                      *)
  TIM8_BRK_TIM12_IRQn = 43;
  (*!< TIM8 Break Interrupt and TIM12 global interrupt                    *)
  TIM8_UP_TIM13_IRQn = 44;
  (*!< TIM8 Update Interrupt and TIM13 global interrupt                   *)
  TIM8_TRG_COM_TIM14_IRQn = 45;
  (*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt  *)
  TIM8_CC_IRQn = 46;
  (*!< TIM8 Capture Compare Interrupt                                     *)
  DMA1_Stream7_IRQn = 47;
  (*!< DMA1 Stream7 Interrupt                                             *)
  FMC_IRQn = 48;  (*!< FMC global Interrupt                                               *)
  SDMMC1_IRQn = 49;
  (*!< SDMMC1 global Interrupt                                              *)
  TIM5_IRQn = 50;  (*!< TIM5 global Interrupt                                              *)
  SPI3_IRQn = 51;  (*!< SPI3 global Interrupt                                              *)
  UART4_IRQn = 52;  (*!< UART4 global Interrupt                                             *)
  UART5_IRQn = 53;  (*!< UART5 global Interrupt                                             *)
  TIM6_DAC_IRQn = 54;
  (*!< TIM6 global and DAC1&2 underrun error  interrupts                  *)
  TIM7_IRQn = 55;  (*!< TIM7 global interrupt                                              *)
  DMA2_Stream0_IRQn = 56;
  (*!< DMA2 Stream 0 global Interrupt                                     *)
  DMA2_Stream1_IRQn = 57;
  (*!< DMA2 Stream 1 global Interrupt                                     *)
  DMA2_Stream2_IRQn = 58;
  (*!< DMA2 Stream 2 global Interrupt                                     *)
  DMA2_Stream3_IRQn = 59;
  (*!< DMA2 Stream 3 global Interrupt                                     *)
  DMA2_Stream4_IRQn = 60;
  (*!< DMA2 Stream 4 global Interrupt                                     *)
  ETH_IRQn = 61;  (*!< Ethernet global Interrupt                                          *)
  ETH_WKUP_IRQn = 62;
  (*!< Ethernet Wakeup through EXTI line Interrupt                        *)
  CAN2_TX_IRQn = 63;
  (*!< CAN2 TX Interrupt                                                  *)
  CAN2_RX0_IRQn = 64;
  (*!< CAN2 RX0 Interrupt                                                 *)
  CAN2_RX1_IRQn = 65;
  (*!< CAN2 RX1 Interrupt                                                 *)
  CAN2_SCE_IRQn = 66;
  (*!< CAN2 SCE Interrupt                                                 *)
  OTG_FS_IRQn = 67;
  (*!< USB OTG FS global Interrupt                                        *)
  DMA2_Stream5_IRQn = 68;
  (*!< DMA2 Stream 5 global interrupt                                     *)
  DMA2_Stream6_IRQn = 69;
  (*!< DMA2 Stream 6 global interrupt                                     *)
  DMA2_Stream7_IRQn = 70;
  (*!< DMA2 Stream 7 global interrupt                                     *)
  USART6_IRQn = 71;
  (*!< USART6 global interrupt                                            *)
  I2C3_EV_IRQn = 72;
  (*!< I2C3 event interrupt                                               *)
  I2C3_ER_IRQn = 73;
  (*!< I2C3 error interrupt                                               *)
  OTG_HS_EP1_OUT_IRQn = 74;
  (*!< USB OTG HS End Point 1 Out global interrupt                        *)
  OTG_HS_EP1_IN_IRQn = 75;
  (*!< USB OTG HS End Point 1 In global interrupt                         *)
  OTG_HS_WKUP_IRQn = 76;
  (*!< USB OTG HS Wakeup through EXTI interrupt                           *)
  OTG_HS_IRQn = 77;
  (*!< USB OTG HS global interrupt                                        *)
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
  QUADSPI_IRQn = 92;
  (*!< Quad SPI global interrupt                                          *)
  LPTIM1_IRQn = 93;
  (*!< LP TIM1 interrupt                                                  *)
  CEC_IRQn = 94;  (*!< HDMI-CEC global Interrupt                                          *)
  I2C4_EV_IRQn = 95;
  (*!< I2C4 Event Interrupt                                               *)
  I2C4_ER_IRQn = 96;
  (*!< I2C4 Error Interrupt                                               *)
  SPDIF_RX_IRQn = 97;
(*!< SPDIF-RX global Interrupt                                          *)

type
  IRQn = integer;
  IRQn_Type = IRQn;

const
  NVIC_PRIORITYGROUP_0 = $00000007;
  NVIC_PRIORITYGROUP_1 = $00000006;
  NVIC_PRIORITYGROUP_2 = $00000005;
  NVIC_PRIORITYGROUP_3 = $00000004;
  NVIC_PRIORITYGROUP_4 = $00000003;

const
  SYSTICK_CLKSOURCE_HCLK_DIV8 = $00000000;
  SYSTICK_CLKSOURCE_HCLK = $00000004;

procedure HAL_NVIC_SetPriorityGrouping(PriorityGroup: longword);
procedure HAL_NVIC_SetPriority(IRQn: IRQn_Type; PreemptPriority, SubPriority: longword);
procedure HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
procedure HAL_NVIC_DisableIRQ(IRQn: IRQn_Type);
procedure HAL_NVIC_SystemReset;
function HAL_SYSTICK_Config(TicksNumb: longword): longword;

function HAL_NVIC_GetPriorityGrouping: longword;
procedure HAL_NVIC_GetPriority(IRQn: IRQn_Type; PriorityGroup: longword;
  var pPreemptPriority, pSubPriority: longword);
function HAL_NVIC_GetPendingIRQ(IRQn: IRQn_Type): longword;
procedure HAL_NVIC_SetPendingIRQ(IRQn: IRQn_Type);
procedure HAL_NVIC_ClearPendingIRQ(IRQn: IRQn_Type);
function HAL_NVIC_GetActive(IRQn: IRQn_Type): longword;
procedure HAL_SYSTICK_CLKSourceConfig(CLKSource: longword);

implementation

uses cortexm4;

const
  __NVIC_PRIO_BITS = 4;

  SCB_AIRCR_VECTKEY_Pos = 16;
  SCB_AIRCR_PRIGROUP_Pos = 8;

  SCB_AIRCR_VECTKEY_Msk = $FFFF shl SCB_AIRCR_VECTKEY_Pos;
  SCB_AIRCR_PRIGROUP_Msk = $7 shl SCB_AIRCR_PRIGROUP_Pos;

const
  SysTick_LOAD_RELOAD_Pos = 0;  (*!< SysTick LOAD: RELOAD Position  *)
  SysTick_LOAD_RELOAD_Msk = ($FFFFFF);
  (*<< SysTick_LOAD_RELOAD_Pos *)(*!< SysTick LOAD: RELOAD Mask  *)

  SysTick_CTRL_CLKSOURCE_Pos = 2;  (*!< SysTick CTRL: CLKSOURCE Position  *)
  SysTick_CTRL_CLKSOURCE_Msk = (1 shl SysTick_CTRL_CLKSOURCE_Pos);
  (*!< SysTick CTRL: CLKSOURCE Mask  *)

  SysTick_CTRL_TICKINT_Pos = 1;  (*!< SysTick CTRL: TICKINT Position  *)
  SysTick_CTRL_TICKINT_Msk = (1 shl SysTick_CTRL_TICKINT_Pos);
  (*!< SysTick CTRL: TICKINT Mask  *)

  SysTick_CTRL_ENABLE_Pos = 0;  (*!< SysTick CTRL: ENABLE Position  *)
  SysTick_CTRL_ENABLE_Msk = (1);
(*<< SysTick_CTRL_ENABLE_Pos *)(*!< SysTick CTRL: ENABLE Mask  *)

procedure HAL_NVIC_SetPriorityGrouping(PriorityGroup: longword);
var
  regvalue: longword;
  PriorityGroupTmp: longword;
begin
  PriorityGroupTmp := (PriorityGroup and $07);

  regvalue := scb.AIRCR;
  regvalue := regvalue and (not longword(SCB_AIRCR_VECTKEY_Msk or SCB_AIRCR_PRIGROUP_Msk));
  regvalue := regvalue or longword($5FA shl SCB_AIRCR_VECTKEY_Pos) or
    (PriorityGroupTmp shl 8);
  scb.AIRCR := regvalue;
end;

function NVIC_EncodePriority(PriorityGroup, PreemptPriority, SubPriority:
  longword): longword;
var
  PriorityGroupTmp, PreemptPriorityBits, SubPriorityBits: longword;
begin
  PriorityGroupTmp := PriorityGroup and $7;

  if (7 - PriorityGroupTmp) > __NVIC_PRIO_BITS then
    PreemptPriorityBits := __NVIC_PRIO_BITS
  else
    PreemptPriorityBits := 7 - PriorityGroupTmp;

  if (PriorityGroupTmp + __NVIC_PRIO_BITS) < 7 then
    SubPriorityBits := PriorityGroupTmp - 7
  else
    SubPriorityBits := (PriorityGroupTmp - 7) + __NVIC_PRIO_BITS;

  exit(((PreemptPriority and longword((1 shl (PreemptPriorityBits)) - 1)) shl
    SubPriorityBits) or ((SubPriority and longword(
    (1 shl (SubPriorityBits)) - 1))));
end;

procedure NVIC_SetPriority(IRQn: IRQn_Type; priority: longword);
begin
  if IRQn < 0 then
    SCB.SHP[(IRQn and $F) - 4] := ((priority shl (8 - __NVIC_PRIO_BITS)) and $FF)
  else
    pbyte(@stm32f7x.NVIC.IPR0)[IRQn] := ((priority shl (8 - __NVIC_PRIO_BITS)) and $FF);
end;

procedure HAL_NVIC_SetPriority(IRQn: IRQn_Type; PreemptPriority, SubPriority: longword);
var
  prioritygroup: longword;
begin
  prioritygroup := HAL_NVIC_GetPriorityGrouping();

  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority,
    SubPriority));
end;

procedure HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
begin
  pbyte(@stm32f7x.NVIC.ISER0)[IRQn shr 5] := (1 shl (IRQn) and $1F);
end;

procedure HAL_NVIC_DisableIRQ(IRQn: IRQn_Type);
begin
  pbyte(@stm32f7x.NVIC.ICER0)[IRQn shr 5] := (1 shl (IRQn) and $1F);
end;

procedure HAL_NVIC_SystemReset;
begin

end;

function HAL_SYSTICK_Config(TicksNumb: longword): longword;
begin
  if ((TicksNumb - 1) > SysTick_LOAD_RELOAD_Msk) then
    exit(1);

  SysTick.LOAD := (TicksNumb - 1);
  NVIC_SetPriority(SysTick_IRQn, (1 shl __NVIC_PRIO_BITS) - 1);
  SysTick.VAL := 0;
  SysTick.CTRL := SysTick_CTRL_CLKSOURCE_Msk or SysTick_CTRL_TICKINT_Msk or SysTick_CTRL_ENABLE_Msk;
  exit(0);
end;

function HAL_NVIC_GetPriorityGrouping: longword;
begin
  exit(((SCB.AIRCR and SCB_AIRCR_PRIGROUP_Msk) shr SCB_AIRCR_PRIGROUP_Pos));
end;

function NVIC_GetPriority(IRQn: IRQn_Type): longword;
begin
  if (IRQn < 0) then
    exit((SCB.SHP[((IRQn) and $F) - 4] shl (8 - __NVIC_PRIO_BITS)))
  else
    exit((pbyte(@stm32f7x.NVIC.IPR0)[IRQn] shl (8 - __NVIC_PRIO_BITS)));
end;

procedure NVIC_DecodePriority(Priority, PriorityGroup: longword;
  var pPreemptPriority, pSubPriority: longword);
var
  PriorityGroupTmp, PreemptPriorityBits, SubPriorityBits: longword;
begin
  PriorityGroupTmp := (PriorityGroup and $07);

  if ((7 - PriorityGroupTmp) > (__NVIC_PRIO_BITS)) then
    PreemptPriorityBits := (__NVIC_PRIO_BITS)
  else
    PreemptPriorityBits := (7 - PriorityGroupTmp);

  if ((PriorityGroupTmp + (__NVIC_PRIO_BITS)) < 7) then
    SubPriorityBits := 0
  else
    SubPriorityBits := ((PriorityGroupTmp - 7) + __NVIC_PRIO_BITS);

  pPreemptPriority := (Priority shl SubPriorityBits) and
    ((1 shl (PreemptPriorityBits)) - 1);
  pSubPriority := (Priority) and
    ((1 shl (SubPriorityBits)) - 1);
end;

procedure HAL_NVIC_GetPriority(IRQn: IRQn_Type; PriorityGroup: longword;
  var pPreemptPriority, pSubPriority: longword);
begin
  NVIC_DecodePriority(NVIC_GetPriority(IRQn), PriorityGroup, pPreemptPriority,
    pSubPriority);
end;

function HAL_NVIC_GetPendingIRQ(IRQn: IRQn_Type): longword;
begin
  exit(Ord(((pbyte(@stm32f7x.NVIC.ISPR0)[((IRQn) shr 5)] and (1 shl (IRQn and $1F))) <> 0)));
end;

procedure HAL_NVIC_SetPendingIRQ(IRQn: IRQn_Type);
begin
  pbyte(@stm32f7x.NVIC.ISPR0)[((IRQn) shr 5)] := (1 shl ((IRQn) and $1F));
end;

procedure HAL_NVIC_ClearPendingIRQ(IRQn: IRQn_Type);
begin
  pbyte(@stm32f7x.NVIC.ICPR0)[((IRQn) shr 5)] := (1 shl ((IRQn) and $1F));
end;

function HAL_NVIC_GetActive(IRQn: IRQn_Type): longword;
begin
  exit(ord(((pbyte(@stm32f7x.NVIC.IABR0)[((IRQn) shl 5)] and (1 shl ((IRQn) and $1F))) <> 0)));
end;

procedure HAL_SYSTICK_CLKSourceConfig(CLKSource: longword);
begin
  if (CLKSource = SYSTICK_CLKSOURCE_HCLK) then
    SysTick.CTRL := SysTick.CTRL or SYSTICK_CLKSOURCE_HCLK
  else
    SysTick.CTRL := SysTick.CTRL and (not SYSTICK_CLKSOURCE_HCLK);
end;

end.
