(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   HAL module driver.
  *          This is the common part of the HAL initialization
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The common HAL driver contains a set of generic and common APIs that can be
    used by the PPP peripheral drivers and the user to start using the HAL.
    [..]
    The HAL contains two APIs' categories:
         (+) Common HAL APIs
         (+) Services HAL APIs

  @endverbatim
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

unit stm32f7xx_hal;

{$define ART_ACCLERATOR_ENABLE}

interface

type
  HAL_StatusTypeDef = (HAL_OK,
                       HAL_ERROR,HAL_TIMEOUT,HAL_BUSY);

  HAL_LockTypeDef = longint;

const
  HAL_MAX_DELAY = $FFFFFFFF;

  HAL_UNLOCKED = 0;
  HAL_LOCKED = 1;

procedure HAL_MspInit; external name 'HAL_MspInit';

procedure __HAL_LOCK(var Lock: HAL_LockTypeDef);
procedure __HAL_UNLOCK(var Lock: HAL_LockTypeDef);

function HAL_Init: HAL_StatusTypeDef;
function HAL_InitTick(TickPriority: longword): HAL_StatusTypeDef;

function HAL_GetTick: longword;
procedure HAL_Delay(Delay: longword);

function HAL_IS_BIT_SET(REG, BIT: longword): boolean;
function HAL_IS_BIT_CLR(REG, BIT: longword): boolean;

var
  SystemCoreClock: longword;

implementation

uses
  stm32f7xx_defs,
  stm32f7xx_hal_conf, stm32f7xx_hal_flash, stm32f7xx_hal_rcc, stm32f7xx_hal_cortex;

function HAL_IS_BIT_SET(REG, BIT: longword): boolean; begin exit(((REG) and (BIT)) <> 0); end;
function HAL_IS_BIT_CLR(REG, BIT: longword): boolean; begin exit(((REG) and (BIT)) = 0); end;

var
  uwTick: longword;

procedure HAL_MspInit_stub; assembler; nostackframe; public name 'HAL_MspInit';
  asm
    .weak HAL_MspInit
  end;

procedure __HAL_LOCK(var Lock: HAL_LockTypeDef);
  begin
    while InterlockedCompareExchange(Lock, HAL_LOCKED, HAL_UNLOCKED)<>0 do;
  end;

procedure __HAL_UNLOCK(var Lock: HAL_LockTypeDef);
  begin
    Lock:=HAL_UNLOCKED;
  end;

function HAL_Init: HAL_StatusTypeDef;
  begin
    (* Configure Flash prefetch and Instruction cache through ART accelerator *)
    if ART_ACCLERATOR_ENABLE then
      __HAL_FLASH_ART_ENABLE();

    (* Set Interrupt Group Priority *)
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    (* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) *)
    HAL_InitTick(TICK_INT_PRIORITY);

    (* Init the low level hardware *)
    HAL_MspInit();

    (* Return function status *)
    exit(HAL_OK);
  end;

function HAL_InitTick(TickPriority: longword): HAL_StatusTypeDef;
  begin
    (*Configure the SysTick to have interrupt in 1ms time basis*)
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() div 1000);

    (*Configure the SysTick IRQ priority *)
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0);

    (* Return function status *)
    exit(HAL_OK);
  end;

function HAL_GetTick: longword;
  begin
    exit(uwTick);
  end;

procedure HAL_Delay(Delay: longword);
  var
    tickstart: LongWord;
  begin
    tickstart:=HAL_GetTick;
    while (HAL_GetTick-tickstart) < Delay do;
  end;

procedure Systick_Interrupt; [public, alias: 'SysTick_Interrupt'];
  begin
    inc(uwTick);
  end;

end.

