(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_pcd_ex.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of PCD HAL module.
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
unit stm32f7xx_hal_pcd_ex;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal,
  stm32f7xx_hal_pcd;

const
  PCD_LPM_L0_ACTIVE = $00;  (* on  *)
  PCD_LPM_L1_ACTIVE = $01;  (* LPM L1 sleep  *)

type
  PCD_LPM_MsgTypeDef = integer;

function HAL_PCDEx_SetTxFiFo(var hpcd: PCD_HandleTypeDef; fifo: byte; size: word): HAL_StatusTypeDef;
function HAL_PCDEx_SetRxFiFo(var hpcd: PCD_HandleTypeDef; size: word): HAL_StatusTypeDef;
function HAL_PCDEx_ActivateLPM(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
function HAL_PCDEx_DeActivateLPM(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_PCDEx_LPM_Callback(var hpcd: PCD_HandleTypeDef; msg: PCD_LPM_MsgTypeDef); external name 'HAL_PCDEx_LPM_Callback';

implementation

function HAL_PCDEx_SetTxFiFo(var hpcd: PCD_HandleTypeDef; fifo: byte; size: word): HAL_StatusTypeDef;
var
  Tx_Offset, i: longword;
begin
  Tx_Offset := hpcd.Instance^.GRXFSIZ;

  if (fifo = 0) then
    hpcd.Instance^.DIEPTXF0_HNPTXFSIZ := (size shl 16) or Tx_Offset
  else
  begin
    Tx_Offset := Tx_Offset + ((hpcd.Instance^.DIEPTXF0_HNPTXFSIZ) shr 16);

    for i := 0 to fifo - 2 do
      Tx_Offset := Tx_Offset + (hpcd.Instance^.DIEPTXF[i] shr 16);

    (* Multiply Tx_Size by 2 to get higher performance *)
    hpcd.Instance^.DIEPTXF[fifo - 1] := (size shl 16) or Tx_Offset;
  end;

  exit(HAL_OK);
end;

function HAL_PCDEx_SetRxFiFo(var hpcd: PCD_HandleTypeDef; size: word): HAL_StatusTypeDef;
begin
  hpcd.Instance^.GRXFSIZ := size;

  exit(HAL_OK);
end;

function HAL_PCDEx_ActivateLPM(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
var
  USBx: PPCD_TypeDef;
begin
  USBx := hpcd.Instance;

  hpcd.lpm_active := True;
  hpcd.LPM_State := LPM_L0;
  USBx^.GINTMSK := USBx^.GINTMSK or USB_OTG_GINTMSK_LPMINTM;
  USBx^.GLPMCFG := USBx^.GLPMCFG or (USB_OTG_GLPMCFG_LPMEN or USB_OTG_GLPMCFG_LPMACK or USB_OTG_GLPMCFG_ENBESL);

  exit(HAL_OK);
end;

function HAL_PCDEx_DeActivateLPM(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
var
  USBx: PPCD_TypeDef;
begin
  USBx := hpcd.Instance;

  hpcd.lpm_active := False;
  USBx^.GINTMSK := USBx^.GINTMSK and (not USB_OTG_GINTMSK_LPMINTM);
  USBx^.GLPMCFG := USBx^.GLPMCFG and (not (USB_OTG_GLPMCFG_LPMEN or USB_OTG_GLPMCFG_LPMACK or USB_OTG_GLPMCFG_ENBESL));

  exit(HAL_OK);
end;

procedure HAL_PCDEx_LPM_Callback_stub(var hpcd: PCD_HandleTypeDef; msg: PCD_LPM_MsgTypeDef); assembler; public name 'HAL_PCDEx_LPM_Callback';
asm
  .weak HAL_PCDEx_LPM_Callback
end;

end.

