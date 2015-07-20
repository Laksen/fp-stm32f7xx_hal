(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_pwr_ex.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of PWR HAL Extension module.
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

unit stm32f7xx_hal_pwr_ex;

interface

uses
  stm32f7xx_hal;

procedure __HAL_PWR_OVERDRIVE_ENABLE;
procedure __HAL_PWR_OVERDRIVE_DISABLE;

procedure __HAL_PWR_OVERDRIVESWITCHING_ENABLE;
procedure __HAL_PWR_OVERDRIVESWITCHING_DISABLE;

function HAL_PWREx_EnableOverDrive: HAL_StatusTypeDef;

implementation

uses
  stm32f7xx_defs,
  stm32f7xx_hal_rcc;

const
  PWR_OVERDRIVE_TIMEOUT_VALUE = 1000;

const
  PWR_FLAG_ODRDY = PWR_CSR1_ODRDY;
  PWR_FLAG_ODSWRDY = PWR_CSR1_ODSWRDY;
  PWR_FLAG_UDRDY = PWR_CSR1_UDSWRDY;

procedure __HAL_PWR_OVERDRIVE_ENABLE;  begin PWR.CR1 := PWR.CR1 or PWR_CR1_ODEN; end;
procedure __HAL_PWR_OVERDRIVE_DISABLE; begin PWR.CR1 := PWR.CR1 and (not PWR_CR1_ODEN); end;

procedure __HAL_PWR_OVERDRIVESWITCHING_ENABLE;  begin PWR.CR1 := PWR.CR1 or PWR_CR1_ODSWEN; end;
procedure __HAL_PWR_OVERDRIVESWITCHING_DISABLE; begin PWR.CR1 := PWR.CR1 and (not PWR_CR1_ODSWEN); end;

function __HAL_PWR_GET_FLAG(__FLAG__: longword): boolean; begin exit((PWR.CSR1 and (__FLAG__)) = (__FLAG__)); end;

function HAL_PWREx_EnableOverDrive: HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  tickstart := 0;

  __HAL_RCC_PWR_CLK_ENABLE();

  (* Enable the Over-drive to extend the clock frequency to 216 MHz *)
  __HAL_PWR_OVERDRIVE_ENABLE();

  (* Get tick *)
  tickstart := HAL_GetTick();

  while not __HAL_PWR_GET_FLAG(PWR_FLAG_ODRDY) do
    if (HAL_GetTick() - tickstart) > PWR_OVERDRIVE_TIMEOUT_VALUE then
      exit(HAL_TIMEOUT);

  (* Enable the Over-drive switch *)
  __HAL_PWR_OVERDRIVESWITCHING_ENABLE();

  (* Get tick *)
  tickstart := HAL_GetTick();

  while not __HAL_PWR_GET_FLAG(PWR_FLAG_ODSWRDY) do
    if (HAL_GetTick() - tickstart) > PWR_OVERDRIVE_TIMEOUT_VALUE then
      exit(HAL_TIMEOUT);

  exit(HAL_OK);
end;

end.

