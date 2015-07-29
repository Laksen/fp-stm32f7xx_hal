(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_dma_ex.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of DMA HAL extension module.
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
unit stm32f7xx_hal_dma_ex;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal,
  stm32f7xx_hal_dma;

type
  HAL_DMA_MemoryTypeDef = (
    MEMORY0,  (*!< Memory 0      *)
    MEMORY1   (*!< Memory 1      *)
  );

function HAL_DMAEx_MultiBufferStart(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress, SecondMemAddress, DataLength: longword): HAL_StatusTypeDef;
function HAL_DMAEx_MultiBufferStart_IT(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress, SecondMemAddress, DataLength: longword): HAL_StatusTypeDef;
function HAL_DMAEx_ChangeMemory(var hdma: DMA_HandleTypeDef; Address: longword; memory: HAL_DMA_MemoryTypeDef): HAL_StatusTypeDef;

implementation

procedure DMA_MultiBufferSetConfig(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress, DataLength: longword);
begin
  (* Configure DMA Stream data length *)
  hdma.Instance^.NDTR := DataLength;

  (* Peripheral to Memory *)
  if ((hdma.Init.Direction) = DMA_MEMORY_TO_PERIPH) then
  begin
    (* Configure DMA Stream destination address *)
    hdma.instance^.PAR := DstAddress;

    (* Configure DMA Stream source address *)
    hdma.instance^.M0AR := SrcAddress;
  end
  (* Memory to Peripheral *)
  else
  begin
    (* Configure DMA Stream source address *)
    hdma.instance^.PAR := SrcAddress;

    (* Configure DMA Stream destination address *)
    hdma.instance^.M0AR := DstAddress;
  end;
end;

function HAL_DMAEx_MultiBufferStart(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress, SecondMemAddress, DataLength: longword): HAL_StatusTypeDef;
begin
  (* Process Locked *)
  __HAL_Lock(hdma.lock);

  (* Current memory buffer used is Memory 0 *)
  if ((hdma.instance^.CR and DMA_SxCR_CT) = 0) then
    hdma.State := HAL_DMA_STATE_BUSY_MEM0
  (* Current memory buffer used is Memory 1 *)
  else if ((hdma.instance^.CR and DMA_SxCR_CT) <> 0) then
    hdma.State := HAL_DMA_STATE_BUSY_MEM1;

  (* Disable the peripheral *)
  __HAL_DMA_DISABLE(hdma);

  (* Enable the double buffer mode *)
  hdma.instance^.CR := hdma.instance^.CR or DMA_SxCR_DBM;

  (* Configure DMA Stream destination address *)
  hdma.instance^.M1AR := SecondMemAddress;

  (* Configure the source, destination address and the data length *)
  DMA_MultiBufferSetConfig(hdma, SrcAddress, DstAddress, DataLength);

  (* Enable the peripheral *)
  __HAL_DMA_ENABLE(hdma);

  exit(HAL_OK);
end;

function HAL_DMAEx_MultiBufferStart_IT(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress, SecondMemAddress, DataLength: longword): HAL_StatusTypeDef;
begin
  (* Process Locked *)
  __HAL_Lock(hdma.lock);

  (* Current memory buffer used is Memory 0 *)
  if ((hdma.instance^.CR and DMA_SxCR_CT) = 0) then
    hdma.State := HAL_DMA_STATE_BUSY_MEM0
  (* Current memory buffer used is Memory 1 *)
  else if ((hdma.instance^.CR and DMA_SxCR_CT) <> 0) then
    hdma.State := HAL_DMA_STATE_BUSY_MEM1;

  (* Disable the peripheral *)
  __HAL_DMA_DISABLE(hdma);

  (* Enable the Double buffer mode *)
  hdma.instance^.CR := hdma.instance^.CR or DMA_SxCR_DBM;

  (* Configure DMA Stream destination address *)
  hdma.instance^.M1AR := SecondMemAddress;

  (* Configure the source, destination address and the data length *)
  DMA_MultiBufferSetConfig(hdma, SrcAddress, DstAddress, DataLength);

  (* Enable the transfer complete interrupt *)
  __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);

  (* Enable the Half transfer interrupt *)
  __HAL_DMA_ENABLE_IT(hdma, DMA_IT_HT);

  (* Enable the transfer Error interrupt *)
  __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TE);

  (* Enable the fifo Error interrupt *)
  __HAL_DMA_ENABLE_IT(hdma, DMA_IT_FE);

  (* Enable the direct mode Error interrupt *)
  __HAL_DMA_ENABLE_IT(hdma, DMA_IT_DME);

  (* Enable the peripheral *)
  __HAL_DMA_ENABLE(hdma);

  exit(HAL_OK);
end;

function HAL_DMAEx_ChangeMemory(var hdma: DMA_HandleTypeDef; Address: longword; memory: HAL_DMA_MemoryTypeDef): HAL_StatusTypeDef;
begin
  if (memory = MEMORY0) then
    (* change the memory0 address *)
    hdma.instance^.M0AR := Address
  else
    (* change the memory1 address *)
    hdma.instance^.M1AR := Address;

  exit(HAL_OK);
end;

end.
