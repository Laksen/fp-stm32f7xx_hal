(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_flash.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of FLASH HAL module.
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
unit stm32f7xx_hal_flash;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal;

(**
  * @brief  FLASH Procedure structure definition
   *)

const
  FLASH_PROC_NONE = 0;
  FLASH_PROC_SECTERASE = 1;
  FLASH_PROC_MASSERASE = 2;
  FLASH_PROC_PROGRAM = 3;

type
  FLASH_ProcedureTypeDef = integer;


  (**
  * @brief  FLASH handle Structure definition
   *)

type
  FLASH_ProcessTypeDef = record
    ProcedureOnGoing: FLASH_ProcedureTypeDef;  (* Internal variable to indicate which procedure is ongoing or not in IT context  *)
    NbSectorsToErase: longword;  (* Internal variable to save the remaining sectors to erase in IT context         *)
    VoltageForErase: byte;  (* Internal variable to provide voltage range selected by user in IT context     *)
    Sector: longword;  (* Internal variable to define the current sector which is erasing                *)
    Address: pointer;  (* Internal variable to save address selected for program                         *)
    Lock: HAL_LockTypeDef;  (* FLASH locking object                                                           *)
    ErrorCode: longword;  (* FLASH error code                     *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup FLASH_Exported_Constants FLASH Exported Constants
  * @{
   *)

  (** @defgroup FLASH_Error_Code FLASH Error Code
  * @brief    FLASH Error Code
  * @{
   *)

const
  HAL_FLASH_ERROR_NONE = ($00000000);  (*!< No error                       *)
  HAL_FLASH_ERROR_ERS = ($00000002);  (*!< Programming Sequence error     *)
  HAL_FLASH_ERROR_PGP = ($00000004);  (*!< Programming Parallelism error  *)
  HAL_FLASH_ERROR_PGA = ($00000008);  (*!< Programming Alignment error    *)
  HAL_FLASH_ERROR_WRP = ($00000010);  (*!< Write protection error         *)
  HAL_FLASH_ERROR_OPERATION = ($00000020);  (*!< Operation Error                *)
  (**
  * @}
   *)

  (** @defgroup FLASH_Type_Program FLASH Type Program
  * @{
   *)

  FLASH_TYPEPROGRAM_BYTE = ($00);  (*!< Program byte (8-bit) at a specified address            *)
  FLASH_TYPEPROGRAM_HALFWORD = ($01);  (*!< Program a half-word (16-bit) at a specified address    *)
  FLASH_TYPEPROGRAM_WORD = ($02);  (*!< Program a word (32-bit) at a specified address         *)
  FLASH_TYPEPROGRAM_DOUBLEWORD = ($03);  (*!< Program a double word (64-bit) at a specified address  *)
  (**
  * @}
   *)

  (** @defgroup FLASH_Flag_definition FLASH Flag definition
  * @brief Flag definition
  * @{
   *)

  FLASH_FLAG_EOP = FLASH_SR_EOP;  (*!< FLASH End of Operation flag                *)
  FLASH_FLAG_OPERR = FLASH_SR_OPERR;  (*!< FLASH operation Error flag                 *)
  FLASH_FLAG_WRPERR = FLASH_SR_WRPERR;  (*!< FLASH Write protected error flag           *)
  FLASH_FLAG_PGAERR = FLASH_SR_PGAERR;  (*!< FLASH Programming Alignment error flag     *)
  FLASH_FLAG_PGPERR = FLASH_SR_PGPERR;  (*!< FLASH Programming Parallelism error flag   *)
  FLASH_FLAG_ERSERR = FLASH_SR_ERSERR;  (*!< FLASH Erasing Sequence error flag          *)
  FLASH_FLAG_BSY = FLASH_SR_BSY;  (*!< FLASH Busy flag                            *)
  (**
  * @}
   *)

  (** @defgroup FLASH_Interrupt_definition FLASH Interrupt definition
  * @brief FLASH Interrupt definition
  * @{
   *)

  FLASH_IT_EOP = FLASH_CR_EOPIE;  (*!< End of FLASH Operation Interrupt source  *)
  FLASH_IT_ERR = ($02000000);  (*!< Error Interrupt source                   *)
  (**
  * @}
   *)

  (** @defgroup FLASH_Program_Parallelism FLASH Program Parallelism
  * @{
   *)

  FLASH_PSIZE_BYTE = ($00000000);
  FLASH_PSIZE_HALF_WORD = (FLASH_CR_PSIZE_0);
  FLASH_PSIZE_WORD = (FLASH_CR_PSIZE_1);
  FLASH_PSIZE_DOUBLE_WORD = (FLASH_CR_PSIZE);
  CR_PSIZE_MASK = ($FFFFFCFF);
  (**
  * @}
   *)

  (** @defgroup FLASH_Keys FLASH Keys
  * @{
   *)

  FLASH_KEY1 = ($45670123);
  FLASH_KEY2 = ($CDEF89AB);
  FLASH_OPT_KEY1 = ($08192A3B);
  FLASH_OPT_KEY2 = ($4C5D6E7F);

procedure __HAL_FLASH_ART_ENABLE;
procedure __HAL_FLASH_ART_DISABLE;
procedure __HAL_FLASH_ART_RESET();

procedure __HAL_FLASH_PREFETCH_BUFFER_ENABLE;
procedure __HAL_FLASH_PREFETCH_BUFFER_DISABLE;

procedure __HAL_FLASH_SET_LATENCY(__LATENCY__: longword);
function __HAL_FLASH_GET_LATENCY(): longword;

procedure __HAL_FLASH_DISABLE_IT(__INTERRUPT__: longword);
procedure __HAL_FLASH_ENABLE_IT(__INTERRUPT__: longword);

function __HAL_FLASH_GET_FLAG(__FLAG__: longword): boolean;
procedure __HAL_FLASH_CLEAR_FLAG(__FLAG__: longword);

(* Program operation functions  ********************************************** *)
function HAL_FLASH_Program(TypeProgram: longword; Address: pointer; Data: qword): HAL_StatusTypeDef;
function HAL_FLASH_Program_IT(TypeProgram: longword; Address: pointer; Data: qword): HAL_StatusTypeDef;
(* FLASH IRQ handler method  *)
procedure HAL_FLASH_IRQHandler;
(* Callbacks in non blocking modes  *)
procedure HAL_FLASH_EndOfOperationCallback(ReturnValue: longword); external name 'HAL_FLASH_EndOfOperationCallback';
procedure HAL_FLASH_OperationErrorCallback(ReturnValue: pointer); external name 'HAL_FLASH_OperationErrorCallback';

(* Peripheral Control functions  ********************************************* *)
function HAL_FLASH_Unlock: HAL_StatusTypeDef;
function HAL_FLASH_Lock: HAL_StatusTypeDef;
function HAL_FLASH_OB_Unlock: HAL_StatusTypeDef;
function HAL_FLASH_OB_Lock: HAL_StatusTypeDef;
(* Option bytes control  *)
function HAL_FLASH_OB_Launch: HAL_StatusTypeDef;

(* Peripheral State functions  *********************************************** *)
function HAL_FLASH_GetError: longword;

function FLASH_WaitForLastOperation(Timeout: longword): HAL_StatusTypeDef;

var
  pFlash: FLASH_ProcessTypeDef;

implementation

uses
  stm32f7xx_hal_flash_ex;

const
  SECTOR_MASK = ($FFFFFF07);
  FLASH_TIMEOUT_VALUE = (50000);  (* 50 s  *)

(**
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__: FLASH Latency
  *         The value of this parameter depend on device used within the same series
  * @retval none
  *)
procedure __HAL_FLASH_SET_LATENCY(__LATENCY__: longword);
begin
  FLASH.ACR := ((FLASH.ACR and (not FLASH_ACR_LATENCY)) or __LATENCY__);
end;

(**
  * @brief  Get the FLASH Latency.
  * @retval FLASH Latency
  *          The value of this parameter depend on device used within the same series
  *)
function __HAL_FLASH_GET_LATENCY(): longword;
begin
  exit(FLASH.ACR and FLASH_ACR_LATENCY);
end;

(**
  * @brief  Enable the FLASH prefetch buffer.
  * @retval none
  *)
procedure __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
begin
  FLASH.ACR := FLASH.ACR or FLASH_ACR_PRFTEN;
end;

(**
  * @brief  Disable the FLASH prefetch buffer.
  * @retval none
  *)
procedure __HAL_FLASH_PREFETCH_BUFFER_DISABLE();
begin
  FLASH.ACR := FLASH.ACR and (not FLASH_ACR_PRFTEN);
end;

(**
  * @brief  Enable the FLASH Adaptive Real-Time memory accelerator.
  * @note   The ART accelerator is available only for flash access on ITCM interface.
  * @retval none
  *)
procedure __HAL_FLASH_ART_ENABLE();
begin
  FLASH.ACR := FLASH.ACR or FLASH_ACR_ARTEN;
end;

(**
  * @brief  Disable the FLASH Adaptive Real-Time memory accelerator.
  * @retval none
  *)
procedure __HAL_FLASH_ART_DISABLE();
begin
  FLASH.ACR := FLASH.ACR and (not FLASH_ACR_ARTEN);
end;

(**
  * @brief  Resets the FLASH Adaptive Real-Time memory accelerator.
  * @note   This function must be used only when the Adaptive Real-Time memory accelerator
  *         is disabled.
  * @retval None
  *)
procedure __HAL_FLASH_ART_RESET();
begin
  FLASH.ACR := FLASH.ACR or FLASH_ACR_ARTRST;
end;

(**
  * @brief  Enable the specified FLASH interrupt.
  * @param  __INTERRUPT__ : FLASH interrupt
  *         This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: End of FLASH Operation Interrupt
  *     @arg FLASH_IT_ERR: Error Interrupt
  * @retval none
  *)
procedure __HAL_FLASH_ENABLE_IT(__INTERRUPT__: longword);
begin
  FLASH.CR := FLASH.CR or (__INTERRUPT__);
end;

(**
  * @brief  Disable the specified FLASH interrupt.
  * @param  __INTERRUPT__ : FLASH interrupt
  *         This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: End of FLASH Operation Interrupt
  *     @arg FLASH_IT_ERR: Error Interrupt
  * @retval none
  *)
procedure __HAL_FLASH_DISABLE_IT(__INTERRUPT__: longword);
begin
  FLASH.CR := FLASH.CR and (not (__INTERRUPT__));
end;

(**
  * @brief  Get the specified FLASH flag status.
  * @param  __FLAG__: specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *            @arg FLASH_FLAG_EOP   : FLASH End of Operation flag
  *            @arg FLASH_FLAG_OPERR : FLASH operation Error flag
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_ERSERR : FLASH Erasing Sequence error flag
  *            @arg FLASH_FLAG_BSY   : FLASH Busy flag
  * @retval The new state of __FLAG__ (SET or RESET).
  *)
function __HAL_FLASH_GET_FLAG(__FLAG__: longword): boolean;
begin
  exit((FLASH.SR and (__FLAG__)) <> 0);
end;

(**
  * @brief  Clear the specified FLASH flag.
  * @param  __FLAG__: specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *            @arg FLASH_FLAG_EOP   : FLASH End of Operation flag
  *            @arg FLASH_FLAG_OPERR : FLASH operation Error flag
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_ERSERR : FLASH Erasing Sequence error flag
  * @retval none
  *)
procedure __HAL_FLASH_CLEAR_FLAG(__FLAG__: longword);
begin
  FLASH.SR := (__FLAG__);
end;

  (**
    * @brief  Program a double word (64-bit) at a specified address.
    * @note   This function must be used when the device voltage range is from
    *         2.7V to 3.6V and an External Vpp is present.
    *
    * @note   If an erase and a program operations are requested simultaneously,
    *         the erase operation is performed before the program one.
    *
    * @param  Address: specifies the address to be programmed.
    * @param  Data: specifies the data to be programmed.
    * @retval None
    *)
procedure FLASH_Program_DoubleWord(Address: pointer; Data: qword);
begin
  (* If the previous operation is completed, proceed to program the new data *)
  FLASH.CR := FLASH.CR and (CR_PSIZE_MASK);
  FLASH.CR := FLASH.CR or FLASH_PSIZE_DOUBLE_WORD;
  FLASH.CR := FLASH.CR or FLASH_CR_PG;

  pqword(Address)^ := Data;

  (* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*)
  __DSB();
end;


(**
  * @brief  Program word (32-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  *)
procedure FLASH_Program_Word(Address: pointer; Data: longword);
begin
  (* If the previous operation is completed, proceed to program the new data *)
  FLASH.CR := FLASH.CR and (CR_PSIZE_MASK);
  FLASH.CR := FLASH.CR or FLASH_PSIZE_WORD;
  FLASH.CR := FLASH.CR or FLASH_CR_PG;

  plongword(Address)^ := Data;

  (* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*)
  __DSB();
end;

(**
  * @brief  Program a half-word (16-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  *)
procedure FLASH_Program_HalfWord(Address: pointer; Data: word);
begin
  (* If the previous operation is completed, proceed to program the new data *)
  FLASH.CR := FLASH.CR and (CR_PSIZE_MASK);
  FLASH.CR := FLASH.CR or FLASH_PSIZE_HALF_WORD;
  FLASH.CR := FLASH.CR or FLASH_CR_PG;

  pword(Address)^ := Data;

  (* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*)
  __DSB();
end;

(**
  * @brief  Program byte (8-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  *)
procedure FLASH_Program_Byte(Address: pointer; Data: byte);
begin
  (* If the previous operation is completed, proceed to program the new data *)
  FLASH.CR := FLASH.CR and (CR_PSIZE_MASK);
  FLASH.CR := FLASH.CR or FLASH_PSIZE_BYTE;
  FLASH.CR := FLASH.CR or FLASH_CR_PG;

  pbyte(Address)^ := Data;

  (* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*)
  __DSB();
end;

(**
  * @brief  Set the specific FLASH error flag.
  * @retval None
  *)
procedure FLASH_SetErrorCode;
begin
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR)) then
    pFlash.ErrorCode := pFlash.ErrorCode or HAL_FLASH_ERROR_WRP;

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR)) then
    pFlash.ErrorCode := pFlash.ErrorCode or HAL_FLASH_ERROR_PGA;

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGPERR)) then
    pFlash.ErrorCode := pFlash.ErrorCode or HAL_FLASH_ERROR_PGP;

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ERSERR)) then
    pFlash.ErrorCode := pFlash.ErrorCode or HAL_FLASH_ERROR_ERS;

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR)) then
    pFlash.ErrorCode := pFlash.ErrorCode or HAL_FLASH_ERROR_OPERATION;
end;

function FLASH_WaitForLastOperation(Timeout: longword): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  (* Clear Error Code *)
  pFlash.ErrorCode := HAL_FLASH_ERROR_NONE;

  (* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set *)
  (* Get tick *)
  tickstart := HAL_GetTick();

  while __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) do
  begin
    if (Timeout <> HAL_MAX_DELAY) then
    begin
      if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
      begin
        exit(HAL_TIMEOUT);
      end;
    end;
  end;

  if __HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR or FLASH_FLAG_WRPERR or FLASH_FLAG_PGAERR or FLASH_FLAG_PGPERR or FLASH_FLAG_ERSERR)) then
  begin
    (*Save the error code*)
    FLASH_SetErrorCode();
    exit(HAL_ERROR);
  end;

  (* If there is an error flag set *)
  exit(HAL_OK);
end;

function HAL_FLASH_Program(TypeProgram: longword; Address: pointer; Data: qword): HAL_StatusTypeDef;
var
  status: HAL_StatusTypeDef;
begin
  status := HAL_ERROR;

  (* Process Locked *)
  __HAL_Lock(pFlash.lock);

  (* Wait for last operation to be completed *)
  status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

  if (status = HAL_OK) then
  begin
    case TypeProgram of
      FLASH_TYPEPROGRAM_BYTE:
        (*Program byte (8-bit) at a specified address.*)
        FLASH_Program_Byte(Address, Data);

      FLASH_TYPEPROGRAM_HALFWORD:
        (*Program halfword (16-bit) at a specified address.*)
        FLASH_Program_HalfWord(Address, Data);

      FLASH_TYPEPROGRAM_WORD:
        (*Program word (32-bit) at a specified address.*)
        FLASH_Program_Word(Address, Data);

      FLASH_TYPEPROGRAM_DOUBLEWORD:
        (*Program double word (64-bit) at a specified address.*)
        FLASH_Program_DoubleWord(Address, Data);
    end;
    (* Wait for last operation to be completed *)
    status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

    (* If the program operation is completed, disable the PG Bit *)
    FLASH.CR := FLASH.CR and ((not FLASH_CR_PG));
  end;

  (* Process Unlocked *)
  __HAL_Unlock(pFlash.lock);

  exit(status);
end;

function HAL_FLASH_Program_IT(TypeProgram: longword; Address: pointer; Data: qword): HAL_StatusTypeDef;
var
  status: HAL_StatusTypeDef;
begin
  status := HAL_OK;

  (* Process Locked *)
  __HAL_Lock(pFlash.lock);

  (* Enable End of FLASH Operation interrupt *)
  __HAL_FLASH_ENABLE_IT(FLASH_IT_EOP);

  (* Enable Error source interrupt *)
  __HAL_FLASH_ENABLE_IT(FLASH_IT_ERR);

  (* Clear pending flags (if any) *)
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP or FLASH_FLAG_OPERR or FLASH_FLAG_WRPERR or FLASH_FLAG_PGAERR or FLASH_FLAG_PGPERR or FLASH_FLAG_ERSERR);

  pFlash.ProcedureOnGoing := FLASH_PROC_PROGRAM;
  pFlash.Address := Address;

  case TypeProgram of
    FLASH_TYPEPROGRAM_BYTE:
      (*Program byte (8-bit) at a specified address.*)
      FLASH_Program_Byte(Address, Data);

    FLASH_TYPEPROGRAM_HALFWORD:
      (*Program halfword (16-bit) at a specified address.*)
      FLASH_Program_HalfWord(Address, Data);

    FLASH_TYPEPROGRAM_WORD:
      (*Program word (32-bit) at a specified address.*)
      FLASH_Program_Word(Address, Data);

    FLASH_TYPEPROGRAM_DOUBLEWORD:
      (*Program double word (64-bit) at a specified address.*)
      FLASH_Program_DoubleWord(Address, Data);
  end;

  exit(status);
end;

procedure HAL_FLASH_IRQHandler;
var
  temp: longword;
begin
  temp := 0;

  (* If the program operation is completed, disable the PG Bit *)
  FLASH.CR := FLASH.CR and ((not FLASH_CR_PG));

  (* If the erase operation is completed, disable the SER Bit *)
  FLASH.CR := FLASH.CR and ((not FLASH_CR_SER));
  FLASH.CR := FLASH.CR and (SECTOR_MASK);

  (* if the erase operation is completed, disable the MER Bit *)
  FLASH.CR := FLASH.CR and ((not FLASH_MER_BIT));

  (* Check FLASH End of Operation flag  *)
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP)) then
  begin
    case pFlash.ProcedureOnGoing of
      FLASH_PROC_SECTERASE:
      begin
        (* Nb of sector to erased can be decreased *)
        Dec(pFlash.NbSectorsToErase);

        (* Check if there are still sectors to erase *)
        if (pFlash.NbSectorsToErase <> 0) then
        begin
          temp := pFlash.Sector;
          (* Indicate user which sector has been erased *)
          HAL_FLASH_EndOfOperationCallback(temp);

          (* Clear pending flags (if any) *)
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);

          (* Increment sector number *)
          Inc(pFlash.Sector);
          temp := pFlash.Sector;
          FLASH_Erase_Sector(temp, pFlash.VoltageForErase);
        end
        else
        begin
          (* No more sectors to Erase, user callback can be called.*)
          (* Reset Sector and stop Erase sectors procedure *)
          temp := $FFFFFFFF;
          pFlash.Sector := temp;
          (* FLASH EOP interrupt user callback *)
          HAL_FLASH_EndOfOperationCallback(temp);
          (* Sector Erase procedure is completed *)
          pFlash.ProcedureOnGoing := FLASH_PROC_NONE;
          (* Clear FLASH End of Operation pending bit *)
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
        end;
      end;

      FLASH_PROC_MASSERASE:
      begin
        (* MassErase ended. Return the selected bank : in this product we don't have Banks *)
        (* FLASH EOP interrupt user callback *)
        HAL_FLASH_EndOfOperationCallback(0);
        (* MAss Erase procedure is completed *)
        pFlash.ProcedureOnGoing := FLASH_PROC_NONE;
        (* Clear FLASH End of Operation pending bit *)
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
      end;

      FLASH_PROC_PROGRAM:
      begin
        (*Program ended. Return the selected address*)
        (* FLASH EOP interrupt user callback *)
        HAL_FLASH_EndOfOperationCallback(longword(pFlash.Address));
        (* Programming procedure is completed *)
        pFlash.ProcedureOnGoing := FLASH_PROC_NONE;
        (* Clear FLASH End of Operation pending bit *)
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
      end;
    end;
  end;

  (* Check FLASH operation error flags *)
  if (__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR or FLASH_FLAG_WRPERR or FLASH_FLAG_PGAERR or FLASH_FLAG_PGPERR or FLASH_FLAG_ERSERR))) then
  begin
    case pFlash.ProcedureOnGoing of
      FLASH_PROC_SECTERASE:
      begin
        (* return the faulty sector *)
        temp := pFlash.Sector;
        pFlash.Sector := $FFFFFFFF;
      end;
      FLASH_PROC_MASSERASE:
      begin
        (* No return in case of Mass Erase *)
        temp := 0;
      end;
      FLASH_PROC_PROGRAM:
      begin
        (*return the faulty address*)
        temp := longword(pFlash.Address);
      end
    end;
    (*Save the Error code*)
    FLASH_SetErrorCode();

    (* FLASH error interrupt user callback *)
    HAL_FLASH_OperationErrorCallback(pointer(temp));
    (* Clear FLASH error pending bits *)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR or FLASH_FLAG_WRPERR or FLASH_FLAG_PGAERR or FLASH_FLAG_PGPERR or FLASH_FLAG_ERSERR);

    (*Stop the procedure ongoing *)
    pFlash.ProcedureOnGoing := FLASH_PROC_NONE;
  end;

  if (pFlash.ProcedureOnGoing = FLASH_PROC_NONE) then
  begin
    (* Disable End of FLASH Operation interrupt *)
    __HAL_FLASH_DISABLE_IT(FLASH_IT_EOP);

    (* Disable Error source interrupt *)
    __HAL_FLASH_DISABLE_IT(FLASH_IT_ERR);

    (* Process Unlocked *)
    __HAL_Unlock(pFlash.lock);
  end;
end;

procedure HAL_FLASH_EndOfOperationCallback_stub(ReturnValue: longword); assembler; nostackframe; public name 'HAL_FLASH_EndOfOperationCallback';
  asm
    .weak HAL_FLASH_EndOfOperationCallback
  end;

procedure HAL_FLASH_OperationErrorCallback_stub(ReturnValue: pointer);  assembler; nostackframe; public name 'HAL_FLASH_OperationErrorCallback';
  asm
    .weak HAL_FLASH_OperationErrorCallback
  end;

function HAL_FLASH_Unlock: HAL_StatusTypeDef;
begin
  if ((FLASH.CR and FLASH_CR_LOCK) <> 0) then
  begin
    (* Authorize the FLASH Registers access *)
    FLASH.KEYR := FLASH_KEY1;
    FLASH.KEYR := FLASH_KEY2;
  end
  else
    exit(HAL_ERROR);

  exit(HAL_OK);
end;

function HAL_FLASH_Lock: HAL_StatusTypeDef;
begin
  (* Set the LOCK Bit to lock the FLASH Registers access *)
  FLASH.CR := FLASH.CR or FLASH_CR_LOCK;

  exit(HAL_OK);
end;

function HAL_FLASH_OB_Unlock: HAL_StatusTypeDef;
begin
  if ((FLASH.OPTCR and FLASH_OPTCR_OPTLOCK) <> 0) then
  begin
    (* Authorizes the Option Byte register programming *)
    FLASH.OPTKEYR := FLASH_OPT_KEY1;
    FLASH.OPTKEYR := FLASH_OPT_KEY2;
  end
  else
    exit(HAL_ERROR);

  exit(HAL_OK);
end;

function HAL_FLASH_OB_Lock: HAL_StatusTypeDef;
begin
  (* Set the OPTLOCK Bit to lock the FLASH Option Byte Registers access *)
  FLASH.OPTCR := FLASH.OPTCR or FLASH_OPTCR_OPTLOCK;

  exit(HAL_OK);
end;

function HAL_FLASH_OB_Launch: HAL_StatusTypeDef;
begin
  (* Set the OPTSTRT bit in OPTCR register *)
  FLASH.OPTCR := FLASH.OPTCR or FLASH_OPTCR_OPTSTRT;

  (* Wait for last operation to be completed *)
  exit(FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE));
end;

function HAL_FLASH_GetError: longword;
begin
  exit(pFlash.ErrorCode);
end;

end.

