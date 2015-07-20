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

uses
  stm32f7xx_defs;

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
procedure HAL_NVIC_GetPriority(IRQn: IRQn_Type; PriorityGroup: longword; var pPreemptPriority, pSubPriority: longword);
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
