(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_sdram.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of SDRAM HAL module.
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
unit stm32f7xx_hal_sdram;

interface

uses
  stm32f7xx_hal,
  stm32f7xx_defs,
  stm32f7xx_hal_dma,
  stm32f7xx_ll_fmc;

(**
  * @brief  HAL SDRAM State structure definition
   *)

const
  HAL_SDRAM_STATE_RESET = $00;  (*!< SDRAM not yet initialized or disabled  *)
  HAL_SDRAM_STATE_READY = $01;  (*!< SDRAM initialized and ready for use    *)
  HAL_SDRAM_STATE_BUSY = $02;  (*!< SDRAM internal process is ongoing      *)
  HAL_SDRAM_STATE_ERROR = $03;  (*!< SDRAM error state                      *)
  HAL_SDRAM_STATE_WRITE_PROTECTED = $04;  (*!< SDRAM device write protected           *)
  HAL_SDRAM_STATE_PRECHARGED = $05;  (*!< SDRAM device precharged                *)

type
  HAL_SDRAM_StateTypeDef = integer;

  (**
  * @brief  SDRAM handle Structure definition
   *)

type
  PDMA_HandleTypeDef = ^DMA_HandleTypeDef;
  PFMC_SDRAM_TypeDef = ^FMC_SDRAM_TypeDef;

  SDRAM_HandleTypeDef = record
    Instance: PFMC_SDRAM_TypeDef;  (*!< Register base address                  *)
    Init: FMC_SDRAM_InitTypeDef;  (*!< SDRAM device configuration parameters  *)
    State: HAL_SDRAM_StateTypeDef;  (*!< SDRAM access state                     *)
    Lock: HAL_LockTypeDef;  (*!< SDRAM locking object                   *)
    hdma: PDMA_HandleTypeDef;  (*!< Pointer DMA handler                    *)
  end;

(* Initialization/de-initialization functions ******************************** *)
function HAL_SDRAM_Init(var hsdram: SDRAM_HandleTypeDef; const Timing: FMC_SDRAM_TimingTypeDef): HAL_StatusTypeDef;
function HAL_SDRAM_DeInit(var hsdram: SDRAM_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_SDRAM_MspInit(var hsdram: SDRAM_HandleTypeDef); external name 'HAL_SDRAM_MspInit';
procedure HAL_SDRAM_MspDeInit(var hsdram: SDRAM_HandleTypeDef); external name 'HAL_SDRAM_MspDeInit';
procedure HAL_SDRAM_IRQHandler(var hsdram: SDRAM_HandleTypeDef);
procedure HAL_SDRAM_RefreshErrorCallback(var hsdram: SDRAM_HandleTypeDef); external name 'HAL_SDRAM_RefreshErrorCallback';
procedure HAL_SDRAM_DMA_XferCpltCallback(var hdma: DMA_HandleTypeDef); external name 'HAL_SDRAM_DMA_XferCpltCallback';
procedure HAL_SDRAM_DMA_XferErrorCallback(var hdma: DMA_HandleTypeDef); external name 'HAL_SDRAM_DMA_XferErrorCallback';

(* I/O operation functions *************************************************** *)
function HAL_SDRAM_Read_8b(var hsdram: SDRAM_HandleTypeDef; pAddress: Plongword; pDstBuffer: Pbyte; BufferSize: longword): HAL_StatusTypeDef;
function HAL_SDRAM_Write_8b(var hsdram: SDRAM_HandleTypeDef; pAddress: Plongword; pSrcBuffer: Pbyte; BufferSize: longword): HAL_StatusTypeDef;
function HAL_SDRAM_Read_16b(var hsdram: SDRAM_HandleTypeDef; pAddress: Plongword; pDstBuffer: Pword; BufferSize: longword): HAL_StatusTypeDef;
function HAL_SDRAM_Write_16b(var hsdram: SDRAM_HandleTypeDef; pAddress: Plongword; pSrcBuffer: Pword; BufferSize: longword): HAL_StatusTypeDef;
function HAL_SDRAM_Read_32b(var hsdram: SDRAM_HandleTypeDef; pAddress, pDstBuffer: Plongword; BufferSize: longword): HAL_StatusTypeDef;
function HAL_SDRAM_Write_32b(var hsdram: SDRAM_HandleTypeDef; pAddress, pSrcBuffer: Plongword; BufferSize: longword): HAL_StatusTypeDef;
function HAL_SDRAM_Read_DMA(var hsdram: SDRAM_HandleTypeDef; pAddress, pDstBuffer: Plongword; BufferSize: longword): HAL_StatusTypeDef;
function HAL_SDRAM_Write_DMA(var hsdram: SDRAM_HandleTypeDef; pAddress, pSrcBuffer: Plongword; BufferSize: longword): HAL_StatusTypeDef;

(* SDRAM Control functions  **************************************************** *)
function HAL_SDRAM_WriteProtection_Enable(var hsdram: SDRAM_HandleTypeDef): HAL_StatusTypeDef;
function HAL_SDRAM_WriteProtection_Disable(var hsdram: SDRAM_HandleTypeDef): HAL_StatusTypeDef;
function HAL_SDRAM_SendCommand(var hsdram: SDRAM_HandleTypeDef; const Command: FMC_SDRAM_CommandTypeDef; Timeout: longword): HAL_StatusTypeDef;
function HAL_SDRAM_ProgramRefreshRate(var hsdram: SDRAM_HandleTypeDef; RefreshRate: longword): HAL_StatusTypeDef;
function HAL_SDRAM_SetAutoRefreshNumber(var hsdram: SDRAM_HandleTypeDef; AutoRefreshNumber: longword): HAL_StatusTypeDef;
function HAL_SDRAM_GetModeStatus(var hsdram: SDRAM_HandleTypeDef): longword;

(* SDRAM State functions ******************************************************* *)
function HAL_SDRAM_GetState(var hsdram: SDRAM_HandleTypeDef): HAL_SDRAM_StateTypeDef;

procedure __HAL_SDRAM_RESET_HANDLE_STATE(__HANDLE__: SDRAM_HandleTypeDef);

implementation

function HAL_SDRAM_Init(var hsdram: SDRAM_HandleTypeDef; const Timing: FMC_SDRAM_TimingTypeDef): HAL_StatusTypeDef;
begin
  if hsdram.State = HAL_SDRAM_STATE_RESET then
  begin
    (* Allocate lock resource and initialize it *)
    hsdram.Lock := HAL_UNLOCKED;
    (* Initialize the low level hardware (MSP) *)
    HAL_SDRAM_MspInit(hsdram);
  end;

  (* Initialize the SDRAM controller state *)
  hsdram.State := HAL_SDRAM_STATE_BUSY;

  (* Initialize SDRAM control Interface *)
  FMC_SDRAM_Init(hsdram.Instance^, hsdram.Init);

  (* Initialize SDRAM timing Interface *)
  FMC_SDRAM_Timing_Init(hsdram.Instance^, Timing, hsdram.Init.SDBank);

  (* Update the SDRAM controller state *)
  hsdram.State := HAL_SDRAM_STATE_READY;

  exit(HAL_OK);
end;

function HAL_SDRAM_DeInit(var hsdram: SDRAM_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Initialize the low level hardware (MSP) *)
  HAL_SDRAM_MspDeInit(hsdram);

  (* Configure the SDRAM registers with their reset values *)
  FMC_SDRAM_DeInit(hsdram.Instance^, hsdram.Init.SDBank);

  (* Reset the SDRAM controller state *)
  hsdram.State := HAL_SDRAM_STATE_RESET;

  (* Release Lock *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

procedure HAL_SDRAM_MspInit_stub(var hsdram: SDRAM_HandleTypeDef); assembler; nostackframe; public name 'HAL_SDRAM_MspInit';
asm
  .weak HAL_SDRAM_MspInit
end;

procedure HAL_SDRAM_MspDeInit_stub(var hsdram: SDRAM_HandleTypeDef); assembler; nostackframe; public name 'HAL_SDRAM_MspDeInit';
asm
  .weak HAL_SDRAM_MspDeInit
end;

procedure HAL_SDRAM_IRQHandler(var hsdram: SDRAM_HandleTypeDef);
begin
  (* Check SDRAM interrupt Rising edge flag *)
  if __FMC_SDRAM_GET_FLAG(hsdram.Instance^, FMC_SDRAM_FLAG_REFRESH_IT) then
  begin
    (* SDRAM refresh error interrupt callback *)
    HAL_SDRAM_RefreshErrorCallback(hsdram);

    (* Clear SDRAM refresh error interrupt pending bit *)
    __FMC_SDRAM_CLEAR_FLAG(hsdram.Instance^, FMC_SDRAM_FLAG_REFRESH_ERROR);
  end;
end;

procedure HAL_SDRAM_RefreshErrorCallback_stub(var hsdram: SDRAM_HandleTypeDef); assembler; nostackframe; public name 'HAL_SDRAM_RefreshErrorCallback';
asm
  .weak HAL_SDRAM_RefreshErrorCallback
end;

procedure HAL_SDRAM_DMA_XferCpltCallback_stub(var hdma: DMA_HandleTypeDef); assembler; nostackframe; public name 'HAL_SDRAM_DMA_XferCpltCallback';
asm
  .weak HAL_SDRAM_DMA_XferCpltCallback
end;

procedure HAL_SDRAM_DMA_XferErrorCallback_stub(var hdma: DMA_HandleTypeDef); assembler; nostackframe; public name 'HAL_SDRAM_DMA_XferErrorCallback';
asm
  .weak HAL_SDRAM_DMA_XferErrorCallback
end;

function HAL_SDRAM_Read_8b(var hsdram: SDRAM_HandleTypeDef; pAddress: Plongword; pDstBuffer: Pbyte; BufferSize: longword): HAL_StatusTypeDef;
var
  pSdramAddress: pbyte;
begin
  pSdramAddress := pbyte(pAddress);

  (* Process Locked *)
  __HAL_Lock(hsdram.lock);

  (* Check the SDRAM controller state *)
  if hsdram.State = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY)
  else if hsdram.State = HAL_SDRAM_STATE_PRECHARGED then
    exit(HAL_ERROR);

  (* Read data from source *)
  while buffersize<>0 do
  begin
    pDstBuffer^ := pSdramAddress^;
    inc(pDstBuffer);
    inc(pSdramAddress);
    dec(BufferSize);
  end;

  (* Process Unlocked *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

function HAL_SDRAM_Write_8b(var hsdram: SDRAM_HandleTypeDef; pAddress: Plongword; pSrcBuffer: Pbyte; BufferSize: longword): HAL_StatusTypeDef;
var
  pSdramAddress: pbyte;
  tmp: HAL_SDRAM_StateTypeDef;
begin
  pSdramAddress := pbyte(pAddress);

  (* Process Locked *)
  __HAL_Lock(hsdram.lock);

  (* Check the SDRAM controller state *)
  tmp := hsdram.State;

  if tmp = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY)
  else if (tmp = HAL_SDRAM_STATE_PRECHARGED) or (tmp = HAL_SDRAM_STATE_WRITE_PROTECTED) then
    exit(HAL_ERROR);

  (* Write data to memory *)
  while buffersize<>0 do
  begin
    pSdramAddress^ := pSrcBuffer^;
    inc(pSrcBuffer);
    inc(pSdramAddress);
    dec(BufferSize);
  end;

  (* Process Unlocked *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

function HAL_SDRAM_Read_16b(var hsdram: SDRAM_HandleTypeDef; pAddress: Plongword; pDstBuffer: Pword; BufferSize: longword): HAL_StatusTypeDef;
var
  pSdramAddress: PWord;
begin
  pSdramAddress := pword(pAddress);

  (* Process Locked *)
  __HAL_Lock(hsdram.lock);

  (* Check the SDRAM controller state *)
  if hsdram.State = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY)
  else if hsdram.State = HAL_SDRAM_STATE_PRECHARGED then
    exit(HAL_ERROR);

  (* Read data from source *)
  while buffersize<>0 do
  begin
    pDstBuffer^ := pSdramAddress^;
    inc(pDstBuffer);
    inc(pSdramAddress);
    dec(BufferSize);
  end;

  (* Process Unlocked *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

function HAL_SDRAM_Write_16b(var hsdram: SDRAM_HandleTypeDef; pAddress: Plongword; pSrcBuffer: Pword; BufferSize: longword): HAL_StatusTypeDef;
var
  pSdramAddress: PWord;
  tmp: HAL_SDRAM_StateTypeDef;
begin
  pSdramAddress := pword(pAddress);

  (* Process Locked *)
  __HAL_Lock(hsdram.lock);

  (* Check the SDRAM controller state *)
  tmp := hsdram.State;

  if tmp = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY)
  else if (tmp = HAL_SDRAM_STATE_PRECHARGED) or (tmp = HAL_SDRAM_STATE_WRITE_PROTECTED) then
    exit(HAL_ERROR);

  (* Write data to memory *)
  while buffersize<>0 do
  begin
    pSdramAddress^ := pSrcBuffer^;
    inc(pSrcBuffer);
    inc(pSdramAddress);
    dec(BufferSize);
  end;

  (* Process Unlocked *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

function HAL_SDRAM_Read_32b(var hsdram: SDRAM_HandleTypeDef; pAddress, pDstBuffer: Plongword; BufferSize: longword): HAL_StatusTypeDef;
var
  pSdramAddress: PLongWord;
begin
  pSdramAddress := pAddress;

  (* Process Locked *)
  __HAL_Lock(hsdram.lock);

  (* Check the SDRAM controller state *)
  if hsdram.State = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY)
  else if hsdram.State = HAL_SDRAM_STATE_PRECHARGED then
    exit(HAL_ERROR);

  (* Read data from source *)
  while buffersize<>0 do
  begin
    pDstBuffer^ := pSdramAddress^;
    inc(pDstBuffer);
    inc(pSdramAddress);
    dec(BufferSize);
  end;

  (* Process Unlocked *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

function HAL_SDRAM_Write_32b(var hsdram: SDRAM_HandleTypeDef; pAddress, pSrcBuffer: Plongword; BufferSize: longword): HAL_StatusTypeDef;
var
  pSdramAddress: PLongWord;
  tmp: HAL_SDRAM_StateTypeDef;
begin
  pSdramAddress := pAddress;

  (* Process Locked *)
  __HAL_Lock(hsdram.lock);

  (* Check the SDRAM controller state *)
  tmp := hsdram.State;

  if tmp = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY)
  else if (tmp = HAL_SDRAM_STATE_PRECHARGED) or (tmp = HAL_SDRAM_STATE_WRITE_PROTECTED) then
    exit(HAL_ERROR);

  (* Write data to memory *)
  while buffersize<>0 do
  begin
    pSdramAddress^ := pSrcBuffer^;
    inc(pSrcBuffer);
    inc(pSdramAddress);
    dec(BufferSize);
  end;

  (* Process Unlocked *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

function HAL_SDRAM_Read_DMA(var hsdram: SDRAM_HandleTypeDef; pAddress, pDstBuffer: Plongword; BufferSize: longword): HAL_StatusTypeDef;
var
  tmp: HAL_SDRAM_StateTypeDef;
begin
  (* Process Locked *)
  __HAL_Lock(hsdram.lock);

  (* Check the SDRAM controller state *)
  tmp := hsdram.State;

  if tmp = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY)
  else if tmp = HAL_SDRAM_STATE_PRECHARGED then
    exit(HAL_ERROR);

  (* Configure DMA user callbacks *)
  hsdram.hdma^.XferCpltCallback  := @HAL_SDRAM_DMA_XferCpltCallback;
  hsdram.hdma^.XferErrorCallback := @HAL_SDRAM_DMA_XferErrorCallback;

  (* Enable the DMA Stream *)
  HAL_DMA_Start_IT(hsdram.hdma^, pAddress, pDstBuffer, BufferSize);

  (* Process Unlocked *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

function HAL_SDRAM_Write_DMA(var hsdram: SDRAM_HandleTypeDef; pAddress, pSrcBuffer: Plongword; BufferSize: longword): HAL_StatusTypeDef;
var
  tmp: HAL_SDRAM_StateTypeDef;
begin
  (* Process Locked *)
  __HAL_Lock(hsdram.lock);

  (* Check the SDRAM controller state *)
  tmp := hsdram.State;

  if tmp = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY)
  else if (tmp = HAL_SDRAM_STATE_PRECHARGED) or (tmp = HAL_SDRAM_STATE_WRITE_PROTECTED) then
    exit(HAL_ERROR);

  (* Configure DMA user callbacks *)
  hsdram.hdma^.XferCpltCallback  := @HAL_SDRAM_DMA_XferCpltCallback;
  hsdram.hdma^.XferErrorCallback := @HAL_SDRAM_DMA_XferErrorCallback;

  (* Enable the DMA Stream *)
  HAL_DMA_Start_IT(hsdram.hdma^, pSrcBuffer, pAddress, BufferSize);

  (* Process Unlocked *)
  __HAL_Unlock(hsdram.lock);

  exit(HAL_OK);
end;

function HAL_SDRAM_WriteProtection_Enable(var hsdram: SDRAM_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Check the SDRAM controller state *)
  if hsdram.State = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_BUSY;

  (* Enable write protection *)
  FMC_SDRAM_WriteProtection_Enable(hsdram.Instance^, hsdram.Init.SDBank);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_WRITE_PROTECTED;

  exit(HAL_OK);
end;

function HAL_SDRAM_WriteProtection_Disable(var hsdram: SDRAM_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Check the SDRAM controller state *)
  if hsdram.State = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_BUSY;

  (* Disable write protection *)
  FMC_SDRAM_WriteProtection_Disable(hsdram.Instance^, hsdram.Init.SDBank);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_READY;

  exit(HAL_OK);
end;

function HAL_SDRAM_SendCommand(var hsdram: SDRAM_HandleTypeDef; const Command: FMC_SDRAM_CommandTypeDef; Timeout: longword): HAL_StatusTypeDef;
begin
  (* Check the SDRAM controller state *)
  if hsdram.State = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_BUSY;

  (* Send SDRAM command *)
  FMC_SDRAM_SendCommand(hsdram.Instance^, Command, Timeout);

  (* Update the SDRAM controller state state *)
  if Command.CommandMode = FMC_SDRAM_CMD_PALL then
    hsdram.State := HAL_SDRAM_STATE_PRECHARGED
  else
    hsdram.State := HAL_SDRAM_STATE_READY;

  exit(HAL_OK);
end;

function HAL_SDRAM_ProgramRefreshRate(var hsdram: SDRAM_HandleTypeDef; RefreshRate: longword): HAL_StatusTypeDef;
begin
  (* Check the SDRAM controller state *)
  if hsdram.State = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_BUSY;

  (* Program the refresh rate *)
  FMC_SDRAM_ProgramRefreshRate(hsdram.Instance^ ,RefreshRate);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_READY;

  exit(HAL_OK);
end;

function HAL_SDRAM_SetAutoRefreshNumber(var hsdram: SDRAM_HandleTypeDef; AutoRefreshNumber: longword): HAL_StatusTypeDef;
begin
  (* Check the SDRAM controller state *)
  if hsdram.State = HAL_SDRAM_STATE_BUSY then
    exit(HAL_BUSY);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_BUSY;

  (* Set the Auto-Refresh number *)
  FMC_SDRAM_SetAutoRefreshNumber(hsdram.Instance^ ,AutoRefreshNumber);

  (* Update the SDRAM state *)
  hsdram.State := HAL_SDRAM_STATE_READY;

  exit(HAL_OK);
end;

function HAL_SDRAM_GetModeStatus(var hsdram: SDRAM_HandleTypeDef): longword;
begin
  (* Return the SDRAM memory current mode *)
  exit(FMC_SDRAM_GetModeStatus(hsdram.Instance^, hsdram.Init.SDBank));
end;

function HAL_SDRAM_GetState(var hsdram: SDRAM_HandleTypeDef): HAL_SDRAM_StateTypeDef;
begin
  exit(hsdram.State);
end;

procedure __HAL_SDRAM_RESET_HANDLE_STATE(__HANDLE__: SDRAM_HandleTypeDef);
begin
  __HANDLE__.State := HAL_SDRAM_STATE_RESET;
end;

end.
