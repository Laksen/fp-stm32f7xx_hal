(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_i2c.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of I2C HAL module.
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

unit stm32f7xx_hal_i2c;

interface

uses
  stm32f7xx_hal_dma, stm32f7xx_hal, stm32f7xx_defs;

(** @defgroup I2C_Configuration_Structure_definition I2C Configuration Structure definition
  * @brief  I2C Configuration Structure definition
  * @begin
   *)

{$packrecords c}

type
  I2C_InitTypeDef = record
    Timing: longword;  (*!< Specifies the I2C_TIMINGR_register value.
                                  This parameter calculated by referring to I2C initialization
                                         section in Reference manual  *)
    OwnAddress1: longword;  (*!< Specifies the first device own address.
                                  This parameter can be a 7-bit or 10-bit address.  *)
    AddressingMode: longword;  (*!< Specifies if 7-bit or 10-bit addressing mode is selected.
                                  This parameter can be a value of @ref I2C_addressing_mode  *)
    DualAddressMode: longword;  (*!< Specifies if dual addressing mode is selected.
                                  This parameter can be a value of @ref I2C_dual_addressing_mode  *)
    OwnAddress2: longword;  (*!< Specifies the second device own address if dual addressing mode is selected
                                  This parameter can be a 7-bit address.  *)
    OwnAddress2Masks: longword;  (*!< Specifies the acknoledge mask address second device own address if dual addressing mode is selected
                                  This parameter can be a value of @ref I2C_own_address2_masks  *)
    GeneralCallMode: longword;  (*!< Specifies if general call mode is selected.
                                  This parameter can be a value of @ref I2C_general_call_addressing_mode  *)
    NoStretchMode: longword;  (*!< Specifies if nostretch mode is selected.
                                  This parameter can be a value of @ref I2C_nostretch_mode  *)
  end;

  HAL_I2C_StateTypeDef = (
    HAL_I2C_STATE_RESET = $00,
      (*!< I2C not yet initialized or disabled          *)
    HAL_I2C_STATE_READY = $01,
      (*!< I2C initialized and ready for use            *)
    HAL_I2C_STATE_BUSY = $02,
      (*!< I2C internal process is ongoing              *)
    HAL_I2C_STATE_MASTER_BUSY_TX = $12,
      (*!< Master Data Transmission process is ongoing  *)
    HAL_I2C_STATE_MASTER_BUSY_RX = $22,
      (*!< Master Data Reception process is ongoing     *)
    HAL_I2C_STATE_SLAVE_BUSY_TX = $32,
      (*!< Slave Data Transmission process is ongoing   *)
    HAL_I2C_STATE_SLAVE_BUSY_RX = $42,
      (*!< Slave Data Reception process is ongoing      *)
    HAL_I2C_STATE_MEM_BUSY_TX = $52,
      (*!< Memory Data Transmission process is ongoing  *)
    HAL_I2C_STATE_MEM_BUSY_RX = $62,
      (*!< Memory Data Reception process is ongoing     *)
    HAL_I2C_STATE_TIMEOUT = $03,
      (*!< Timeout state                                *)
    HAL_I2C_STATE_ERROR =
    $04  (*!< Reception process is ongoing                 *)
  );

const
  HAL_I2C_ERROR_NONE    = ($00000000);  (*!< No error               *)
  HAL_I2C_ERROR_BERR    = ($00000001);  (*!< BERR error             *)
  HAL_I2C_ERROR_ARLO    = ($00000002);  (*!< ARLO error             *)
  HAL_I2C_ERROR_AF      = ($00000004);  (*!< ACKF error             *)
  HAL_I2C_ERROR_OVR     = ($00000008);  (*!< OVR error              *)
  HAL_I2C_ERROR_DMA     = ($00000010);  (*!< DMA transfer error     *)
  HAL_I2C_ERROR_TIMEOUT = ($00000020);  (*!< Timeout error          *)
  HAL_I2C_ERROR_SIZE    = ($00000040);  (*!< Size Management error  *)

type
  PI2C_HandleTypeDef = ^I2C_HandleTypeDef;

  I2C_HandleTypeDef = record
    Instance: ^TI2C1_Registers;  (*!< I2C registers base address      *)
    Init: I2C_InitTypeDef;  (*!< I2C communication parameters    *)
    pBuffPtr: pbyte;  (*!< Pointer to I2C transfer buffer  *)
    XferSize: word;  (*!< I2C transfer size               *)
    XferCount: word;  (*!< I2C transfer counter            *)
    hdmatx: ^DMA_HandleTypeDef;  (*!< I2C Tx DMA handle parameters    *)
    hdmarx: ^DMA_HandleTypeDef;  (*!< I2C Rx DMA handle parameters    *)
    Lock: HAL_LockTypeDef;  (*!< I2C locking object              *)
    State: HAL_I2C_StateTypeDef;  (*!< I2C communication state         *)
    ErrorCode: longword;  (*!< I2C Error code                    *)
  end;

const
  I2C_ADDRESSINGMODE_7BIT  = ($00000001);
  I2C_ADDRESSINGMODE_10BIT = ($00000002);

  I2C_DUALADDRESS_DISABLE = ($00000000);
  I2C_DUALADDRESS_ENABLE  = I2C_OAR2_OA2EN;

  I2C_OA2_NOMASK = ($00);
  I2C_OA2_MASK01 = ($01);
  I2C_OA2_MASK02 = ($02);
  I2C_OA2_MASK03 = ($03);
  I2C_OA2_MASK04 = ($04);
  I2C_OA2_MASK05 = ($05);
  I2C_OA2_MASK06 = ($06);
  I2C_OA2_MASK07 = ($07);

  I2C_GENERALCALL_DISABLE = ($00000000);
  I2C_GENERALCALL_ENABLE  = I2C_CR1_GCEN;

  I2C_NOSTRETCH_DISABLE = ($00000000);
  I2C_NOSTRETCH_ENABLE  = I2C_CR1_NOSTRETCH;

  I2C_MEMADD_SIZE_8BIT  = ($00000001);
  I2C_MEMADD_SIZE_16BIT = ($00000002);

  I2C_RELOAD_MODE  = I2C_CR2_RELOAD;
  I2C_AUTOEND_MODE = I2C_CR2_AUTOEND;
  I2C_SOFTEND_MODE = ($00000000);

  I2C_NO_STARTSTOP        = ($00000000);
  I2C_GENERATE_STOP       = I2C_CR2_STOP;
  I2C_GENERATE_START_READ = (I2C_CR2_START or I2C_CR2_RD_WRN);
  I2C_GENERATE_START_WRITE = I2C_CR2_START;

  I2C_IT_ERRI  = I2C_CR1_ERRIE;
  I2C_IT_TCI   = I2C_CR1_TCIE;
  I2C_IT_STOPI = I2C_CR1_STOPIE;
  I2C_IT_NACKI = I2C_CR1_NACKIE;
  I2C_IT_ADDRI = I2C_CR1_ADDRIE;
  I2C_IT_RXI   = I2C_CR1_RXIE;
  I2C_IT_TXI   = I2C_CR1_TXIE;

  I2C_FLAG_TXE     = I2C_ISR_TXE;
  I2C_FLAG_TXIS    = I2C_ISR_TXIS;
  I2C_FLAG_RXNE    = I2C_ISR_RXNE;
  I2C_FLAG_ADDR    = I2C_ISR_ADDR;
  I2C_FLAG_AF      = I2C_ISR_NACKF;
  I2C_FLAG_STOPF   = I2C_ISR_STOPF;
  I2C_FLAG_TC      = I2C_ISR_TC;
  I2C_FLAG_TCR     = I2C_ISR_TCR;
  I2C_FLAG_BERR    = I2C_ISR_BERR;
  I2C_FLAG_ARLO    = I2C_ISR_ARLO;
  I2C_FLAG_OVR     = I2C_ISR_OVR;
  I2C_FLAG_PECERR  = I2C_ISR_PECERR;
  I2C_FLAG_TIMEOUT = I2C_ISR_TIMEOUT;
  I2C_FLAG_ALERT   = I2C_ISR_ALERT;
  I2C_FLAG_BUSY    = I2C_ISR_BUSY;
  I2C_FLAG_DIR     = I2C_ISR_DIR;

procedure __HAL_I2C_RESET_HANDLE_STATE(var __HANDLE__: I2C_HandleTypeDef);
procedure __HAL_I2C_ENABLE_IT(var __HANDLE__: I2C_HandleTypeDef; __INTERRUPT__: longword);
procedure __HAL_I2C_DISABLE_IT(var __HANDLE__: I2C_HandleTypeDef; __INTERRUPT__: longword);
function __HAL_I2C_GET_IT_SOURCE(var __HANDLE__: I2C_HandleTypeDef; __INTERRUPT__: longword): boolean;

function __HAL_I2C_GET_FLAG(var __HANDLE__: I2C_HandleTypeDef; __FLAG__: longword): boolean;
procedure __HAL_I2C_CLEAR_FLAG(var __HANDLE__: I2C_HandleTypeDef; __FLAG__: longword);
procedure __HAL_I2C_ENABLE(var __HANDLE__: I2C_HandleTypeDef);
procedure __HAL_I2C_DISABLE(var __HANDLE__: I2C_HandleTypeDef);

(* Initialization and de-initialization functions***************************** *)
function HAL_I2C_Init(var hi2c: I2C_HandleTypeDef): HAL_StatusTypeDef;
function HAL_I2C_DeInit(var hi2c: I2C_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_I2C_MspInit(var hi2c: I2C_HandleTypeDef); external name 'HAL_I2C_MspInit';
procedure HAL_I2C_MspDeInit(var hi2c: I2C_HandleTypeDef); external name 'HAL_I2C_MspDeInit';

(* IO operation functions  *************************************************** *)
(******* Blocking mode: Polling  *)
function HAL_I2C_Master_Transmit(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
function HAL_I2C_Master_Receive(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
function HAL_I2C_Slave_Transmit(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
function HAL_I2C_Slave_Receive(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
function HAL_I2C_Mem_Write(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
function HAL_I2C_Mem_Read(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
function HAL_I2C_IsDeviceReady(var hi2c: I2C_HandleTypeDef; DevAddress: word; Trials, Timeout: longword): HAL_StatusTypeDef;
(******* Non-Blocking mode: Interrupt  *)
function HAL_I2C_Master_Transmit_IT(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Master_Receive_IT(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Slave_Transmit_IT(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Slave_Receive_IT(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Mem_Write_IT(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Mem_Read_IT(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
(******* Non-Blocking mode: DMA  *)
function HAL_I2C_Master_Transmit_DMA(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Master_Receive_DMA(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Slave_Transmit_DMA(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Slave_Receive_DMA(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Mem_Write_DMA(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
function HAL_I2C_Mem_Read_DMA(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;

(******* I2C IRQHandler and Callbacks used in non blocking modes (Interrupt and DMA)  *)
procedure HAL_I2C_EV_IRQHandler(var hi2c: I2C_HandleTypeDef);
procedure HAL_I2C_ER_IRQHandler(var hi2c: I2C_HandleTypeDef);
procedure HAL_I2C_MasterTxCpltCallback(var hi2c: I2C_HandleTypeDef);
procedure HAL_I2C_MasterRxCpltCallback(var hi2c: I2C_HandleTypeDef);
procedure HAL_I2C_SlaveTxCpltCallback(var hi2c: I2C_HandleTypeDef);
procedure HAL_I2C_SlaveRxCpltCallback(var hi2c: I2C_HandleTypeDef);
procedure HAL_I2C_MemTxCpltCallback(var hi2c: I2C_HandleTypeDef);
procedure HAL_I2C_MemRxCpltCallback(var hi2c: I2C_HandleTypeDef);
procedure HAL_I2C_ErrorCallback(var hi2c: I2C_HandleTypeDef);

(* Peripheral State and Errors functions  ************************************ *)
function HAL_I2C_GetState(var hi2c: I2C_HandleTypeDef): HAL_I2C_StateTypeDef;
function HAL_I2C_GetError(var hi2c: I2C_HandleTypeDef): longword;

implementation

function I2C_IsAcknowledgeFailed(var hi2c: I2C_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef; forward;
function I2C_WaitOnSTOPFlagUntilTimeout(var hi2c: I2C_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef; forward;
function I2C_WaitOnFlagUntilTimeout(var hi2c: I2C_HandleTypeDef; Flag: longword; Status: boolean; Timeout: longword): HAL_StatusTypeDef; forward;
function I2C_WaitOnTXISFlagUntilTimeout(var hi2c: I2C_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef; forward;
procedure I2C_TransferConfig(var hi2c: I2C_HandleTypeDef; DevAddress: word; Size: byte; Mode, Request: longword); forward;

const
  TIMING_CLEAR_MASK = ($F0FFFFFF);  (*<! I2C TIMING clear register Mask  *)
  I2C_TIMEOUT_ADDR  = (10000);  (* 10 s   *)
  I2C_TIMEOUT_BUSY  = (25);  (* 25 ms  *)
  I2C_TIMEOUT_DIR   = (25);  (* 25 ms  *)
  I2C_TIMEOUT_RXNE  = (25);  (* 25 ms  *)
  I2C_TIMEOUT_STOPF = (25);  (* 25 ms  *)
  I2C_TIMEOUT_TC    = (25);  (* 25 ms  *)
  I2C_TIMEOUT_TCR   = (25);  (* 25 ms  *)
  I2C_TIMEOUT_TXIS  = (25);  (* 25 ms  *)
  I2C_TIMEOUT_FLAG  = (25);  (* 25 ms  *)

  I2C_FLAG_MASK = ($0001FFFF);

procedure __HAL_I2C_RESET_HANDLE_STATE(var __HANDLE__: I2C_HandleTypeDef);
  begin
    __HANDLE__.State := HAL_I2C_STATE_RESET;
  end;

procedure __HAL_I2C_ENABLE_IT(var __HANDLE__: I2C_HandleTypeDef; __INTERRUPT__: longword);
  begin
    __HANDLE__.Instance^.CR1 := __HANDLE__.Instance^.CR1 or (__INTERRUPT__);
  end;

procedure __HAL_I2C_DISABLE_IT(var __HANDLE__: I2C_HandleTypeDef; __INTERRUPT__: longword);
  begin
    __HANDLE__.Instance^.CR1 := __HANDLE__.Instance^.CR1 and (not (__INTERRUPT__));
  end;

function __HAL_I2C_GET_IT_SOURCE(var __HANDLE__: I2C_HandleTypeDef; __INTERRUPT__: longword): boolean;
  begin
    exit(((__HANDLE__.Instance^.CR1 and (__INTERRUPT__)) = (__INTERRUPT__)));
  end;

function __HAL_I2C_GET_FLAG(var __HANDLE__: I2C_HandleTypeDef; __FLAG__: longword): boolean;
  begin
    exit((((__HANDLE__.Instance^.ISR) and ((__FLAG__) and I2C_FLAG_MASK)) = ((__FLAG__) and I2C_FLAG_MASK)));
  end;

procedure __HAL_I2C_CLEAR_FLAG(var __HANDLE__: I2C_HandleTypeDef; __FLAG__: longword);
  begin
    __HANDLE__.Instance^.ICR := (__FLAG__) and I2C_FLAG_MASK;
  end;

procedure __HAL_I2C_ENABLE(var __HANDLE__: I2C_HandleTypeDef);
  begin
    __HANDLE__.Instance^.CR1 := __HANDLE__.Instance^.CR1 or I2C_CR1_PE;
  end;

procedure __HAL_I2C_DISABLE(var __HANDLE__: I2C_HandleTypeDef);
  begin
    __HANDLE__.Instance^.CR1 := __HANDLE__.Instance^.CR1 and (not I2C_CR1_PE);
  end;

procedure I2C_RESET_CR2(var __HANDLE__: I2C_HandleTypeDef);
  begin
    __HANDLE__.Instance^.CR2 := __HANDLE__.Instance^.CR2 and (not ((I2C_CR2_SADD or I2C_CR2_HEAD10R or I2C_CR2_NBYTES or I2C_CR2_RELOAD or I2C_CR2_RD_WRN)));
  end;

function I2C_MEM_ADD_MSB(__ADDRESS__: longword): longword;
  begin
    exit((((((__ADDRESS__) and ($FF00))) shr 8)));
  end;

function I2C_MEM_ADD_LSB(__ADDRESS__: longword): longword;
  begin
    exit((((__ADDRESS__) and ($00FF))));
  end;

function I2C_GENERATE_START(__ADDMODE__, __ADDRESS__: longword): longword;
  begin
    if ((__ADDMODE__) = I2C_ADDRESSINGMODE_7BIT) then
      exit((((__ADDRESS__) and (I2C_CR2_SADD)) or (I2C_CR2_START) or (I2C_CR2_AUTOEND)) and (not I2C_CR2_RD_WRN))
    else
      exit((((__ADDRESS__) and (I2C_CR2_SADD)) or (I2C_CR2_ADD10) or (I2C_CR2_START)) and (not I2C_CR2_RD_WRN));
  end;

(**
  * @brief  Handle Interrupt Flags Master Transmit Mode
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  *)
function I2C_MasterTransmit_ISR(var hi2c: I2C_HandleTypeDef): HAL_StatusTypeDef;
  var
    DevAddress: word;
  begin
    (* Process Locked *)
    __HAL_LOCK(hi2c.lock);

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS)) then
      begin
      (* Write data to TXDR *)
      hi2c.Instance^.TXDR := hi2c.pBuffPtr^;
      Inc(hi2c.pBuffPtr);
      Dec(hi2c.XferSize);
      Dec(hi2c.XferCount);
      end
    else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TCR)) then
        begin
        if ((hi2c.XferSize = 0) and (hi2c.XferCount <> 0)) then
          begin
          DevAddress := (hi2c.Instance^.CR2 and I2C_CR2_SADD);

          if (hi2c.XferCount > 255) then
            begin
            I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            hi2c.XferSize := 255;
            end
          else
            begin
            I2C_TransferConfig(hi2c, DevAddress, hi2c.XferCount, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
            hi2c.XferSize := hi2c.XferCount;
            end;
          end
        else
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.lock);

          (* Wrong size Status regarding TCR flag event *)
          hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_SIZE;
          HAL_I2C_ErrorCallback(hi2c);
          end;
        end
      else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TC)) then
          begin
          if (hi2c.XferCount = 0) then
            begin
            (* Generate Stop *)
            hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_STOP;
            end
          else
            begin
            (* Process Unlocked *)
            __HAL_UNLOCK(hi2c.lock);

            (* Wrong size Status regarding TCR flag event *)
            hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_SIZE;
            HAL_I2C_ErrorCallback(hi2c);
            end;
          end
        else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) then
            begin
            (* Disable ERR, TC, STOP, NACK, TXI interrupt *)
            __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_TXI);

            (* Clear STOP Flag *)
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

            (* Clear Configuration Register 2 *)
            I2C_RESET_CR2(hi2c);

            hi2c.State := HAL_I2C_STATE_READY;

            (* Process Unlocked *)
            __HAL_UNLOCK(hi2c.lock);

            if (hi2c.State = HAL_I2C_STATE_MEM_BUSY_TX) then
              begin
              HAL_I2C_MemTxCpltCallback(hi2c);
              end
            else
              begin
              HAL_I2C_MasterTxCpltCallback(hi2c);
              end;
            end
          else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) then
              begin
              (* Clear NACK Flag *)
              __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

              (* Process Unlocked *)
              __HAL_UNLOCK(hi2c.lock);

              hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_AF;
              HAL_I2C_ErrorCallback(hi2c);
              end;

    (* Process Unlocked *)
    __HAL_UNLOCK(hi2c.lock);

    exit(HAL_OK);
  end;

(**
  * @brief  Handle Interrupt Flags Master Receive Mode
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  *)
function I2C_MasterReceive_ISR(var hi2c: I2C_HandleTypeDef): HAL_StatusTypeDef;
  var
    DevAddress: word;
  begin
    (* Process Locked *)
    __HAL_LOCK(hi2c.lock);

    if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) then
      begin
      (* Read data from RXDR *)
      hi2c.pBuffPtr^ := hi2c.Instance^.RXDR;
      Inc(hi2c.pBuffPtr);
      Dec(hi2c.XferSize);
      Dec(hi2c.XferCount);
      end
    else if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TCR) then
        begin
        if ((hi2c.XferSize = 0) and (hi2c.XferCount <> 0)) then
          begin
          DevAddress := (hi2c.Instance^.CR2 and I2C_CR2_SADD);

          if (hi2c.XferCount > 255) then
            begin
            I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            hi2c.XferSize := 255;
            end
          else
            begin
            I2C_TransferConfig(hi2c, DevAddress, hi2c.XferCount, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
            hi2c.XferSize := hi2c.XferCount;
            end;
          end
        else
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.lock);

          (* Wrong size Status regarding TCR flag event *)
          hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_SIZE;
          HAL_I2C_ErrorCallback(hi2c);
          end;
        end
      else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TC)) then
          begin
          if (hi2c.XferCount = 0) then
            begin
            (* Generate Stop *)
            hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_STOP;
            end
          else
            begin
            (* Process Unlocked *)
            __HAL_UNLOCK(hi2c.lock);

            (* Wrong size Status regarding TCR flag event *)
            hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_SIZE;
            HAL_I2C_ErrorCallback(hi2c);
            end;
          end
        else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) then
            begin
            (* Disable ERR, TC, STOP, NACK, TXI interrupt *)
            __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_RXI);

            (* Clear STOP Flag *)
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

            (* Clear Configuration Register 2 *)
            I2C_RESET_CR2(hi2c);

            hi2c.State := HAL_I2C_STATE_READY;

            (* Process Unlocked *)
            __HAL_UNLOCK(hi2c.lock);

            if (hi2c.State = HAL_I2C_STATE_MEM_BUSY_RX) then
              begin
              HAL_I2C_MemRxCpltCallback(hi2c);
              end
            else
              begin
              HAL_I2C_MasterRxCpltCallback(hi2c);
              end;
            end
          else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) then
              begin
              (* Clear NACK Flag *)
              __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

              (* Process Unlocked *)
              __HAL_UNLOCK(hi2c.lock);

              hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_AF;
              HAL_I2C_ErrorCallback(hi2c);
              end;

    (* Process Unlocked *)
    __HAL_UNLOCK(hi2c.lock);

    exit(HAL_OK);

  end;

(**
  * @brief  Handle Interrupt Flags Slave Transmit Mode
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  *)
function I2C_SlaveTransmit_ISR(var hi2c: I2C_HandleTypeDef): HAL_StatusTypeDef;
  begin
    (* Process locked *)
    __HAL_LOCK(hi2c.lock);

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) then
      begin
      (* Check that I2C transfer finished *)
      (* if yes, normal usecase, a NACK is sent by the MASTER when Transfer is finished *)
      (* Mean XferCount = 0*)
      (* So clear Flag NACKF only *)
      if (hi2c.XferCount = 0) then
        begin
        (* Clear NACK Flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.lock);
        end
      else
        begin
        (* if no, error usecase, a Non-Acknowledge of last Data is generated by the MASTER*)
        (* Clear NACK Flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        (* Set ErrorCode corresponding to a Non-Acknowledge *)
        hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_AF;

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.lock);

        (* Call the Error callback to prevent upper layer *)
        HAL_I2C_ErrorCallback(hi2c);
        end;
      end
    else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR)) then
        begin
        (* Clear ADDR flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        end
      (* Check first if STOPF is set          *)
      (* to prevent a Write Data in TX buffer *)
      (* which is stuck in TXDR until next    *)
      (* communication with Master            *)
      else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) then
          begin
          (* Disable ERRI, TCI, STOPI, NACKI, ADDRI, RXI, TXI interrupt *)
          __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI or I2C_IT_TXI);

          (* Disable Address Acknowledge *)
          hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;

          (* Clear STOP Flag *)
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

          hi2c.State := HAL_I2C_STATE_READY;

          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.lock);

          HAL_I2C_SlaveTxCpltCallback(hi2c);
          end
        else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS)) then
            begin
            (* Write data to TXDR only if XferCount not reach "0" *)
            (* A TXIS flag can be set, during STOP treatment      *)
            if (hi2c.XferCount > 0) then
              begin
              (* Write data to TXDR *)
              hi2c.Instance^.TXDR := hi2c.pBuffPtr^;
              Inc(hi2c.pBuffPtr);
              Dec(hi2c.XferCount);
              end;
            end;
    (* Process Unlocked *)
    __HAL_UNLOCK(hi2c.lock);

    exit(HAL_OK);
  end;

(**
  * @brief  Handle Interrupt Flags Slave Receive Mode
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  *)
function I2C_SlaveReceive_ISR(var hi2c: I2C_HandleTypeDef): HAL_StatusTypeDef;
  begin
    (* Process Locked *)
    __HAL_LOCK(hi2c.lock);

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) then
      begin
      (* Clear NACK Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.lock);

      hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_AF;
      HAL_I2C_ErrorCallback(hi2c);
      end
    else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR)) then
        begin
        (* Clear ADDR flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        end
      else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE)) then
          begin
          (* Read data from RXDR *)
          hi2c.pBuffPtr^ := hi2c.Instance^.RXDR;
          Inc(hi2c.pBuffPtr);
          Dec(hi2c.XferSize);
          Dec(hi2c.XferCount);
          end
        else if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) then
            begin
            (* Disable ERRI, TCI, STOPI, NACKI, ADDRI, RXI, TXI interrupt *)
            __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI or I2C_IT_RXI);

            (* Disable Address Acknowledge *)
            hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;

            (* Clear STOP Flag *)
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

            hi2c.State := HAL_I2C_STATE_READY;

            (* Process Unlocked *)
            __HAL_UNLOCK(hi2c.lock);

            HAL_I2C_SlaveRxCpltCallback(hi2c);
            end;

    (* Process Unlocked *)
    __HAL_UNLOCK(hi2c.lock);

    exit(HAL_OK);
  end;

(**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  MemAddSize: Size of internal memory address
  * @param  Timeout: Timeout duration
  * @retval HAL status
  *)
function I2C_RequestMemoryWrite(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; Timeout: longword): HAL_StatusTypeDef;
  begin
    I2C_TransferConfig(hi2c, DevAddress, MemAddSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);

    (* Wait until TXIS flag is set *)
    if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
      begin
      if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
        begin
        exit(HAL_ERROR);
        end
      else
        begin
        exit(HAL_TIMEOUT);
        end;
      end;

    (* If Memory address size is 8Bit *)
    if (MemAddSize = I2C_MEMADD_SIZE_8BIT) then
      begin
      (* Send Memory Address *)
      hi2c.Instance^.TXDR := I2C_MEM_ADD_LSB(MemAddress);
      end
    (* If Memory address size is 16Bit *)
    else
      begin
      (* Send MSB of Memory Address *)
      hi2c.Instance^.TXDR := I2C_MEM_ADD_MSB(MemAddress);

      (* Wait until TXIS flag is set *)
      if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          exit(HAL_ERROR);
          end
        else
          begin
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Send LSB of Memory Address *)
      hi2c.Instance^.TXDR := I2C_MEM_ADD_LSB(MemAddress);
      end;

    (* Wait until TCR flag is set *)
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, False, Timeout) <> HAL_OK) then
      begin
      exit(HAL_TIMEOUT);
      end;

    exit(HAL_OK);
  end;

(**
  * @brief  Master sends target device address followed by internal memory address for read request.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  MemAddSize: Size of internal memory address
  * @param  Timeout: Timeout duration
  * @retval HAL status
  *)
function I2C_RequestMemoryRead(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; Timeout: longword): HAL_StatusTypeDef;
  begin
    I2C_TransferConfig(hi2c, DevAddress, MemAddSize, I2C_SOFTEND_MODE, I2C_GENERATE_START_WRITE);

    (* Wait until TXIS flag is set *)
    if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
      begin
      if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
        begin
        exit(HAL_ERROR);
        end
      else
        begin
        exit(HAL_TIMEOUT);
        end;
      end;

    (* If Memory address size is 8Bit *)
    if (MemAddSize = I2C_MEMADD_SIZE_8BIT) then
      begin
      (* Send Memory Address *)
      hi2c.Instance^.TXDR := I2C_MEM_ADD_LSB(MemAddress);
      end
    (* If Memory address size is 16Bit *)
    else
      begin
      (* Send MSB of Memory Address *)
      hi2c.Instance^.TXDR := I2C_MEM_ADD_MSB(MemAddress);

      (* Wait until TXIS flag is set *)
      if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          exit(HAL_ERROR);
          end
        else
          begin
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Send LSB of Memory Address *)
      hi2c.Instance^.TXDR := I2C_MEM_ADD_LSB(MemAddress);
      end;

    (* Wait until TC flag is set *)
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TC, False, Timeout) <> HAL_OK) then
      begin
      exit(HAL_TIMEOUT);
      end;

    exit(HAL_OK);
  end;

(**
  * @brief  DMA I2C master transmit process complete callback.
  * @param  hdma: DMA handle
  * @retval None
  *)
procedure I2C_DMAMasterTransmitCplt(var hdma: DMA_HandleTypeDef);
  var
    DevAddress: word;
    hi2c: PI2C_HandleTypeDef;
  begin
    hi2c := PI2C_HandleTypeDef(hdma.Parent);

    (* Check if last DMA request was done with RELOAD *)
    (* Set NBYTES to write and reload if size > 255 *)
    if ((hi2c^.XferSize = 255) and (hi2c^.XferSize < hi2c^.XferCount)) then
      begin
      (* Wait until TCR flag is set *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c^, I2C_FLAG_TCR, False, I2C_TIMEOUT_TCR) <> HAL_OK) then
        begin
        hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
        end;

      (* Disable DMA Request *)
      hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_TXDMAEN);

      (* Check if Errors has been detected during transfer *)
      if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
        begin
        (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
        (* Wait until STOPF flag is reset *)
        if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
          begin
          if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
            begin
            hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
            end
          else
            begin
            hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
            end;
          end;

        (* Clear STOP Flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

        (* Clear Configuration Register 2 *)
        I2C_RESET_CR2(hi2c^);

        hi2c^.XferCount := 0;

        hi2c^.State := HAL_I2C_STATE_READY;
        HAL_I2C_ErrorCallback(hi2c^);
        end
      else
        begin
        hi2c^.pBuffPtr := hi2c^.pBuffPtr + hi2c^.XferSize;
        hi2c^.XferCount := hi2c^.XferCount - hi2c^.XferSize;
        if (hi2c^.XferCount > 255) then
          begin
          hi2c^.XferSize := 255;
          end
        else
          begin
          hi2c^.XferSize := hi2c^.XferCount;
          end;

        DevAddress := (hi2c^.Instance^.CR2 and I2C_CR2_SADD);

        (* Enable the DMA channel *)
        HAL_DMA_Start_IT(hi2c^.hdmatx^, hi2c^.pBuffPtr, @hi2c^.Instance^.TXDR, hi2c^.XferSize);

        (* Send Slave Address *)
        (* Set NBYTES to write and reload if size > 255 *)
        if ((hi2c^.XferSize = 255) and (hi2c^.XferSize < hi2c^.XferCount)) then
          begin
          I2C_TransferConfig(hi2c^, DevAddress, hi2c^.XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
          end
        else
          begin
          I2C_TransferConfig(hi2c^, DevAddress, hi2c^.XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
          end;

        (* Wait until TXIS flag is set *)
        if (I2C_WaitOnTXISFlagUntilTimeout(hi2c^, I2C_TIMEOUT_TXIS) <> HAL_OK) then
          begin
          (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
          (* Wait until STOPF flag is reset *)
          if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
            begin
            if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
              begin
              hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
              end
            else
              begin
              hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
              end;
            end;

          (* Clear STOP Flag *)
          __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

          (* Clear Configuration Register 2 *)
          I2C_RESET_CR2(hi2c^);

          hi2c^.XferCount := 0;

          hi2c^.State := HAL_I2C_STATE_READY;
          HAL_I2C_ErrorCallback(hi2c^);
          end
        else
          begin
          (* Enable DMA Request *)
          hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 or I2C_CR1_TXDMAEN;
          end;
        end;
      end
    else
      begin
      (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
      (* Wait until STOPF flag is reset *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
          end
        else
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
          end;
        end;

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c^);

      (* Disable DMA Request *)
      hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_TXDMAEN);

      hi2c^.XferCount := 0;

      hi2c^.State := HAL_I2C_STATE_READY;

      (* Check if Errors has been detected during transfer *)
      if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
        begin
        HAL_I2C_ErrorCallback(hi2c^);
        end
      else
        begin
        HAL_I2C_MasterTxCpltCallback(hi2c^);
        end;
      end;
  end;

(**
  * @brief  DMA I2C slave transmit process complete callback.
  * @param  hdma: DMA handle
  * @retval None
  *)
procedure I2C_DMASlaveTransmitCplt(var hdma: DMA_HandleTypeDef);
  var
    hi2c: PI2C_HandleTypeDef;
  begin
    hi2c := PI2C_HandleTypeDef(hdma.Parent);

    (* Wait until STOP flag is set *)
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
      begin
      if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
        begin
        (* Normal Use case, a AF is generated by master *)
        (* to inform slave the end of transfer *)
        hi2c^.ErrorCode := HAL_I2C_ERROR_NONE;
        end
      else
        begin
        hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
        end;
      end;

    (* Clear STOP flag *)
    __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

    (* Wait until BUSY flag is reset *)
    if (I2C_WaitOnFlagUntilTimeout(hi2c^, I2C_FLAG_BUSY, True, I2C_TIMEOUT_BUSY) <> HAL_OK) then
      begin
      hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
      end;

    (* Disable DMA Request *)
    hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_TXDMAEN);

    hi2c^.XferCount := 0;

    hi2c^.State := HAL_I2C_STATE_READY;

    (* Check if Errors has been detected during transfer *)
    if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
      begin
      HAL_I2C_ErrorCallback(hi2c^);
      end
    else
      begin
      HAL_I2C_SlaveTxCpltCallback(hi2c^);
      end;
  end;

(**
  * @brief DMA I2C master receive process complete callback
  * @param  hdma: DMA handle
  * @retval None
  *)
procedure I2C_DMAMasterReceiveCplt(var hdma: DMA_HandleTypeDef);
  var
    hi2c: PI2C_HandleTypeDef;
    DevAddress: word;
  begin
    hi2c := PI2C_HandleTypeDef(hdma.Parent);

    (* Check if last DMA request was done with RELOAD *)
    (* Set NBYTES to write and reload if size > 255 *)
    if ((hi2c^.XferSize = 255) and (hi2c^.XferSize < hi2c^.XferCount)) then
      begin
      (* Wait until TCR flag is set *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c^, I2C_FLAG_TCR, False, I2C_TIMEOUT_TCR) <> HAL_OK) then
        begin
        hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
        end;

      (* Disable DMA Request *)
      hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_RXDMAEN);

      (* Check if Errors has been detected during transfer *)
      if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
        begin
        (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
        (* Wait until STOPF flag is reset *)
        if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
          begin
          if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
            begin
            hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
            end
          else
            begin
            hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
            end;
          end;

        (* Clear STOP Flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

        (* Clear Configuration Register 2 *)
        I2C_RESET_CR2(hi2c^);

        hi2c^.XferCount := 0;

        hi2c^.State := HAL_I2C_STATE_READY;
        HAL_I2C_ErrorCallback(hi2c^);
        end
      else
        begin
        hi2c^.pBuffPtr := hi2c^.pBuffPtr + hi2c^.XferSize;
        hi2c^.XferCount := hi2c^.XferCount - hi2c^.XferSize;
        if (hi2c^.XferCount > 255) then
          begin
          hi2c^.XferSize := 255;
          end
        else
          begin
          hi2c^.XferSize := hi2c^.XferCount;
          end;

        DevAddress := (hi2c^.Instance^.CR2 and I2C_CR2_SADD);

        (* Enable the DMA channel *)
        HAL_DMA_Start_IT(hi2c^.hdmarx^, @hi2c^.Instance^.RXDR, hi2c^.pBuffPtr, hi2c^.XferSize);

        (* Send Slave Address *)
        (* Set NBYTES to write and reload if size > 255 *)
        if ((hi2c^.XferSize = 255) and (hi2c^.XferSize < hi2c^.XferCount)) then
          begin
          I2C_TransferConfig(hi2c^, DevAddress, hi2c^.XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
          end
        else
          begin
          I2C_TransferConfig(hi2c^, DevAddress, hi2c^.XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
          end;

        (* Wait until RXNE flag is set *)
        if (I2C_WaitOnFlagUntilTimeout(hi2c^, I2C_FLAG_RXNE, False, I2C_TIMEOUT_RXNE) <> HAL_OK) then
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
          end;

        (* Check if Errors has been detected during transfer *)
        if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
          begin
          (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
          (* Wait until STOPF flag is reset *)
          if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
            begin
            if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
              begin
              hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
              end
            else
              begin
              hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
              end;
            end;

          (* Clear STOP Flag *)
          __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

          (* Clear Configuration Register 2 *)
          I2C_RESET_CR2(hi2c^);

          hi2c^.XferCount := 0;

          hi2c^.State := HAL_I2C_STATE_READY;

          HAL_I2C_ErrorCallback(hi2c^);
          end
        else
          begin
          (* Enable DMA Request *)
          hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 or I2C_CR1_RXDMAEN;
          end;
        end;
      end
    else
      begin
      (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
      (* Wait until STOPF flag is reset *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
          end
        else
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
          end;
        end;

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c^);

      (* Disable DMA Request *)
      hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_RXDMAEN);

      hi2c^.XferCount := 0;

      hi2c^.State := HAL_I2C_STATE_READY;

      (* Check if Errors has been detected during transfer *)
      if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
        begin
        HAL_I2C_ErrorCallback(hi2c^);
        end
      else
        begin
        HAL_I2C_MasterRxCpltCallback(hi2c^);
        end;
      end;
  end;

(**
  * @brief  DMA I2C slave receive process complete callback.
  * @param  hdma: DMA handle
  * @retval None
  *)
procedure I2C_DMASlaveReceiveCplt(var hdma: DMA_HandleTypeDef);
  var
    hi2c: PI2C_HandleTypeDef;
  begin
    hi2c := PI2C_HandleTypeDef(hdma.Parent);

    (* Wait until STOPF flag is reset *)
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
      begin
      if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
        begin
        hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
        end
      else
        begin
        hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
        end;
      end;

    (* Clear STOPF flag *)
    __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

    (* Wait until BUSY flag is reset *)
    if (I2C_WaitOnFlagUntilTimeout(hi2c^, I2C_FLAG_BUSY, True, I2C_TIMEOUT_BUSY) <> HAL_OK) then
      begin
      hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
      end;

    (* Disable DMA Request *)
    hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_RXDMAEN);

    (* Disable Address Acknowledge *)
    hi2c^.Instance^.CR2 := hi2c^.Instance^.CR2 or I2C_CR2_NACK;

    hi2c^.XferCount := 0;

    hi2c^.State := HAL_I2C_STATE_READY;

    (* Check if Errors has been detected during transfer *)
    if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
      begin
      HAL_I2C_ErrorCallback(hi2c^);
      end
    else
      begin
      HAL_I2C_SlaveRxCpltCallback(hi2c^);
      end;
  end;

(**
  * @brief DMA I2C Memory Write process complete callback
  * @param hdma : DMA handle
  * @retval None
  *)
procedure I2C_DMAMemTransmitCplt(var hdma: DMA_HandleTypeDef);
  var
    hi2c: PI2C_HandleTypeDef;
    DevAddress: word;
  begin
    hi2c := PI2C_HandleTypeDef(hdma.Parent);

    (* Check if last DMA request was done with RELOAD *)
    (* Set NBYTES to write and reload if size > 255 *)
    if ((hi2c^.XferSize = 255) and (hi2c^.XferSize < hi2c^.XferCount)) then
      begin
      (* Wait until TCR flag is set *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c^, I2C_FLAG_TCR, False, I2C_TIMEOUT_TCR) <> HAL_OK) then
        begin
        hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
        end;

      (* Disable DMA Request *)
      hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_TXDMAEN);

      (* Check if Errors has been detected during transfer *)
      if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
        begin
        (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
        (* Wait until STOPF flag is reset *)
        if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
          begin
          if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
            begin
            hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
            end
          else
            begin
            hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
            end;
          end;

        (* Clear STOP Flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

        (* Clear Configuration Register 2 *)
        I2C_RESET_CR2(hi2c^);

        hi2c^.XferCount := 0;

        hi2c^.State := HAL_I2C_STATE_READY;
        HAL_I2C_ErrorCallback(hi2c^);
        end
      else
        begin
        hi2c^.pBuffPtr := hi2c^.pBuffPtr + hi2c^.XferSize;
        hi2c^.XferCount := hi2c^.XferCount - hi2c^.XferSize;
        if (hi2c^.XferCount > 255) then
          begin
          hi2c^.XferSize := 255;
          end
        else
          begin
          hi2c^.XferSize := hi2c^.XferCount;
          end;

        DevAddress := (hi2c^.Instance^.CR2 and I2C_CR2_SADD);

        (* Enable the DMA channel *)
        HAL_DMA_Start_IT(hi2c^.hdmatx^, hi2c^.pBuffPtr, @hi2c^.Instance^.TXDR, hi2c^.XferSize);

        (* Send Slave Address *)
        (* Set NBYTES to write and reload if size > 255 *)
        if ((hi2c^.XferSize = 255) and (hi2c^.XferSize < hi2c^.XferCount)) then
          begin
          I2C_TransferConfig(hi2c^, DevAddress, hi2c^.XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
          end
        else
          begin
          I2C_TransferConfig(hi2c^, DevAddress, hi2c^.XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
          end;

        (* Wait until TXIS flag is set *)
        if (I2C_WaitOnTXISFlagUntilTimeout(hi2c^, I2C_TIMEOUT_TXIS) <> HAL_OK) then
          begin
          (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
          (* Wait until STOPF flag is reset *)
          if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
            begin
            if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
              begin
              hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
              end
            else
              begin
              hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
              end;
            end;

          (* Clear STOP Flag *)
          __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

          (* Clear Configuration Register 2 *)
          I2C_RESET_CR2(hi2c^);

          hi2c^.XferCount := 0;

          hi2c^.State := HAL_I2C_STATE_READY;
          HAL_I2C_ErrorCallback(hi2c^);
          end
        else
          begin
          (* Enable DMA Request *)
          hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 or I2C_CR1_TXDMAEN;
          end;
        end;
      end
    else
      begin
      (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
      (* Wait until STOPF flag is reset *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
          end
        else
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
          end;
        end;

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c^);

      (* Disable DMA Request *)
      hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_TXDMAEN);

      hi2c^.XferCount := 0;

      hi2c^.State := HAL_I2C_STATE_READY;

      (* Check if Errors has been detected during transfer *)
      if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
        begin
        HAL_I2C_ErrorCallback(hi2c^);
        end
      else
        begin
        HAL_I2C_MemTxCpltCallback(hi2c^);
        end;
      end;
  end;

(**
  * @brief  DMA I2C Memory Read process complete callback
  * @param  hdma: DMA handle
  * @retval None
  *)
procedure I2C_DMAMemReceiveCplt(var hdma: DMA_HandleTypeDef);
  var
    hi2c: PI2C_HandleTypeDef;
    devaddress: word;
  begin
    hi2c := PI2C_HandleTypeDef(hdma.Parent);

    (* Check if last DMA request was done with RELOAD *)
    (* Set NBYTES to write and reload if size > 255 *)
    if ((hi2c^.XferSize = 255) and (hi2c^.XferSize < hi2c^.XferCount)) then
      begin
      (* Wait until TCR flag is set *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c^, I2C_FLAG_TCR, False, I2C_TIMEOUT_TCR) <> HAL_OK) then
        begin
        hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
        end;

      (* Disable DMA Request *)
      hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_RXDMAEN);

      (* Check if Errors has been detected during transfer *)
      if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
        begin
        (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
        (* Wait until STOPF flag is reset *)
        if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
          begin
          if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
            begin
            hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
            end
          else
            begin
            hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
            end;
          end;

        (* Clear STOP Flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

        (* Clear Configuration Register 2 *)
        I2C_RESET_CR2(hi2c^);

        hi2c^.XferCount := 0;

        hi2c^.State := HAL_I2C_STATE_READY;
        HAL_I2C_ErrorCallback(hi2c^);
        end
      else
        begin
        hi2c^.pBuffPtr := hi2c^.pBuffPtr + hi2c^.XferSize;
        hi2c^.XferCount := hi2c^.XferCount - hi2c^.XferSize;
        if (hi2c^.XferCount > 255) then
          begin
          hi2c^.XferSize := 255;
          end
        else
          begin
          hi2c^.XferSize := hi2c^.XferCount;
          end;

        DevAddress := (hi2c^.Instance^.CR2 and I2C_CR2_SADD);

        (* Enable the DMA channel *)
        HAL_DMA_Start_IT(hi2c^.hdmarx^, @hi2c^.Instance^.RXDR, hi2c^.pBuffPtr, hi2c^.XferSize);

        (* Send Slave Address *)
        (* Set NBYTES to write and reload if size > 255 *)
        if ((hi2c^.XferSize = 255) and (hi2c^.XferSize < hi2c^.XferCount)) then
          begin
          I2C_TransferConfig(hi2c^, DevAddress, hi2c^.XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
          end
        else
          begin
          I2C_TransferConfig(hi2c^, DevAddress, hi2c^.XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
          end;

        (* Wait until RXNE flag is set *)
        if (I2C_WaitOnFlagUntilTimeout(hi2c^, I2C_FLAG_RXNE, False, I2C_TIMEOUT_RXNE) <> HAL_OK) then
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
          end;

        (* Check if Errors has been detected during transfer *)
        if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
          begin
          (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
          (* Wait until STOPF flag is reset *)
          if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
            begin
            if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
              begin
              hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
              end
            else
              begin
              hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
              end;
            end;

          (* Clear STOP Flag *)
          __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

          (* Clear Configuration Register 2 *)
          I2C_RESET_CR2(hi2c^);

          hi2c^.XferCount := 0;

          hi2c^.State := HAL_I2C_STATE_READY;
          HAL_I2C_ErrorCallback(hi2c^);
          end
        else
          begin
          (* Enable DMA Request *)
          hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 or I2C_CR1_RXDMAEN;
          end;
        end;
      end
    else
      begin
      (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
      (* Wait until STOPF flag is reset *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c^, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        if (hi2c^.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_AF;
          end
        else
          begin
          hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
          end;
        end;

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c^, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c^);

      (* Disable DMA Request *)
      hi2c^.Instance^.CR1 := hi2c^.Instance^.CR1 and (not I2C_CR1_RXDMAEN);

      hi2c^.XferCount := 0;

      hi2c^.State := HAL_I2C_STATE_READY;

      (* Check if Errors has been detected during transfer *)
      if (hi2c^.ErrorCode <> HAL_I2C_ERROR_NONE) then
        begin
        HAL_I2C_ErrorCallback(hi2c^);
        end
      else
        begin
        HAL_I2C_MemRxCpltCallback(hi2c^);
        end;
      end;
  end;

(**
  * @brief  DMA I2C communication error callback.
  * @param hdma : DMA handle
  * @retval None
  *)
procedure I2C_DMAError(var hdma: DMA_HandleTypeDef);
  var
    hi2c: PI2C_HandleTypeDef;
  begin
    hi2c := PI2C_HandleTypeDef(hdma.Parent);

    (* Disable Acknowledge *)
    hi2c^.Instance^.CR2 := hi2c^.Instance^.CR2 or I2C_CR2_NACK;

    hi2c^.XferCount := 0;

    hi2c^.State := HAL_I2C_STATE_READY;

    hi2c^.ErrorCode := hi2c^.ErrorCode or HAL_I2C_ERROR_DMA;

    HAL_I2C_ErrorCallback(hi2c^);
  end;

(**
  * @brief  This function handles I2C Communication Timeout.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Flag: specifies the I2C flag to check.
  * @param  Status: The new Flag status (SET or RESET).
  * @param  Timeout: Timeout duration
  * @retval HAL status
  *)
function I2C_WaitOnFlagUntilTimeout(var hi2c: I2C_HandleTypeDef; Flag: longword; Status: boolean; Timeout: longword): HAL_StatusTypeDef;
  var
    tickstart: longword;
  begin
    tickstart := HAL_GetTick();

    (* Wait until flag is set *)
    if (not Status) then
      begin
      while (not __HAL_I2C_GET_FLAG(hi2c, Flag)) do
        begin
        (* Check for the Timeout *)
        if (Timeout <> HAL_MAX_DELAY) then
          begin
          if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
            begin
            hi2c.State := HAL_I2C_STATE_READY;
            (* Process Unlocked *)
            __HAL_UNLOCK(hi2c.lock);
            exit(HAL_TIMEOUT);
            end;
          end;
        end;
      end
    else
      begin
      while (__HAL_I2C_GET_FLAG(hi2c, Flag)) do
        begin
        (* Check for the Timeout *)
        if (Timeout <> HAL_MAX_DELAY) then
          begin
          if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
            begin
            hi2c.State := HAL_I2C_STATE_READY;
            (* Process Unlocked *)
            __HAL_UNLOCK(hi2c.lock);
            exit(HAL_TIMEOUT);
            end;
          end;
        end;
      end;
    exit(HAL_OK);
  end;

(**
  * @brief  This function handles I2C Communication Timeout for specific usage of TXIS flag.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout: Timeout duration
  * @retval HAL status
  *)
function I2C_WaitOnTXISFlagUntilTimeout(var hi2c: I2C_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
  var
    tickstart: longword;
  begin
    tickstart := HAL_GetTick();

    while (not __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS)) do
      begin
      (* Check if a NACK is detected *)
      if (I2C_IsAcknowledgeFailed(hi2c, Timeout) <> HAL_OK) then
        begin
        exit(HAL_ERROR);
        end;

      (* Check for the Timeout *)
      if (Timeout <> HAL_MAX_DELAY) then
        begin
        if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
          begin
          hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
          hi2c.State := HAL_I2C_STATE_READY;

          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.lock);

          exit(HAL_TIMEOUT);
          end;
        end;
      end;
    exit(HAL_OK);
  end;

(**
  * @brief  This function handles I2C Communication Timeout for specific usage of STOP flag.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout: Timeout duration
  * @retval HAL status
  *)
function I2C_WaitOnSTOPFlagUntilTimeout(var hi2c: I2C_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
  var
    tickstart: longword;
  begin
    tickstart := HAL_GetTick();

    while (not __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) do
      begin
      (* Check if a NACK is detected *)
      if (I2C_IsAcknowledgeFailed(hi2c, Timeout) <> HAL_OK) then
        begin
        exit(HAL_ERROR);
        end;

      (* Check for the Timeout *)
      if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
        begin
        hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
        hi2c.State := HAL_I2C_STATE_READY;

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.lock);

        exit(HAL_TIMEOUT);
        end;
      end;
    exit(HAL_OK);
  end;

(**
  * @brief  This function handles I2C Communication Timeout for specific usage of RXNE flag.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout: Timeout duration
  * @retval HAL status
  *)
function I2C_WaitOnRXNEFlagUntilTimeout(var hi2c: I2C_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
  var
    tickstart: longword;
  begin
    tickstart := HAL_GetTick();

    while (not __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE)) do
      begin
      (* Check if a STOPF is detected *)
      if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) then
        begin
        (* Clear STOP Flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        (* Clear Configuration Register 2 *)
        I2C_RESET_CR2(hi2c);

        hi2c.ErrorCode := HAL_I2C_ERROR_NONE;
        hi2c.State := HAL_I2C_STATE_READY;

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.lock);

        exit(HAL_ERROR);
        end;

      (* Check for the Timeout *)
      if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
        begin
        hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_TIMEOUT;
        hi2c.State := HAL_I2C_STATE_READY;

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.lock);

        exit(HAL_TIMEOUT);
        end;
      end;
    exit(HAL_OK);
  end;

(**
  * @brief  This function handles Acknowledge failed detection during an I2C Communication.
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout: Timeout duration
  * @retval HAL status
  *)
function I2C_IsAcknowledgeFailed(var hi2c: I2C_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
  var
    tickstart: longword;
  begin
    tickstart := HAL_GetTick();

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) then
      begin
      (* Generate stop if necessary only in case of I2C peripheral in MASTER mode *)
      if ((hi2c.State = HAL_I2C_STATE_MASTER_BUSY_TX) or (hi2c.State = HAL_I2C_STATE_MEM_BUSY_TX) or (hi2c.State = HAL_I2C_STATE_MEM_BUSY_RX)) then
        begin
        (* No need to generate the STOP condition if AUTOEND mode is enabled *)
        (* Generate the STOP condition only in case of SOFTEND mode is enabled *)
        if ((hi2c.Instance^.CR2 and I2C_AUTOEND_MODE) <> I2C_AUTOEND_MODE) then
          begin
          (* Generate Stop *)
          hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_STOP;
          end;
        end;

      (* Wait until STOP Flag is reset *)
      (* AutoEnd should be initiate after AF *)
      while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) = False) do
        begin
        (* Check for the Timeout *)
        if (Timeout <> HAL_MAX_DELAY) then
          begin
          if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
            begin
            hi2c.State := HAL_I2C_STATE_READY;
            (* Process Unlocked *)
            __HAL_UNLOCK(hi2c.lock);
            exit(HAL_TIMEOUT);
            end;
          end;
        end;

      (* Clear NACKF Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c);

      hi2c.ErrorCode := HAL_I2C_ERROR_AF;
      hi2c.State := HAL_I2C_STATE_READY;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.lock);

      exit(HAL_ERROR);
      end;
    exit(HAL_OK);
  end;

(**
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  hi2c: I2C handle.
  * @param  DevAddress: specifies the slave address to be programmed.
  * @param  Size: specifies the number of bytes to be programmed.
  *   This parameter must be a value between 0 and 255.
  * @param  Mode: new state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg I2C_RELOAD_MODE: Enable Reload mode .
  *     @arg I2C_AUTOEND_MODE: Enable Automatic end mode.
  *     @arg I2C_SOFTEND_MODE: Enable Software end mode.
  * @param  Request: new state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg I2C_NO_STARTSTOP: Don't Generate stop and start condition.
  *     @arg I2C_GENERATE_STOP: Generate stop condition (Size should be set to 0).
  *     @arg I2C_GENERATE_START_READ: Generate Restart for read request.
  *     @arg I2C_GENERATE_START_WRITE: Generate Restart for write request.
  * @retval None
  *)
procedure I2C_TransferConfig(var hi2c: I2C_HandleTypeDef; DevAddress: word; Size: byte; Mode, Request: longword);
  var
    tmpreg: longword;
  begin
    (* Get the CR2 register value *)
    tmpreg := hi2c.Instance^.CR2;

    (* clear tmpreg specific bits *)
    tmpreg := tmpreg and (not ((I2C_CR2_SADD or I2C_CR2_NBYTES or I2C_CR2_RELOAD or I2C_CR2_AUTOEND or I2C_CR2_RD_WRN or I2C_CR2_START or I2C_CR2_STOP)));

    (* update tmpreg *)
    tmpreg := tmpreg or (longword(DevAddress and I2C_CR2_SADD) or longword((Size shl 16) and I2C_CR2_NBYTES) or Mode or Request);

    (* update CR2 register *)
    hi2c.Instance^.CR2 := tmpreg;
  end;

function HAL_I2C_Init(var hi2c: I2C_HandleTypeDef): HAL_StatusTypeDef;
  begin
    if (hi2c.State = HAL_I2C_STATE_RESET) then
      begin
      (* Allocate lock resource and initialize it *)
      hi2c.Lock := HAL_UNLOCKED;
      (* Init the low level hardware : GPIO, CLOCK, CORTEX...etc *)
      HAL_I2C_MspInit(hi2c);
      end;

    hi2c.State := HAL_I2C_STATE_BUSY;

    (* Disable the selected I2C peripheral *)
    __HAL_I2C_DISABLE(hi2c);

    (*---------------------------- I2Cx TIMINGR Configuration ------------------*)
    (* Configure I2Cx: Frequency range *)
    hi2c.Instance^.TIMINGR := hi2c.Init.Timing and TIMING_CLEAR_MASK;

    (*---------------------------- I2Cx OAR1 Configuration ---------------------*)
    (* Configure I2Cx: Own Address1 and ack own address1 mode *)
    hi2c.Instance^.OAR1 := hi2c.Instance^.OAR1 and (not I2C_OAR1_OA1EN);
    if (hi2c.Init.OwnAddress1 <> 0) then
      begin
        if (hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT) then
          hi2c.Instance^.OAR1 := (I2C_OAR1_OA1EN or hi2c.Init.OwnAddress1)
        else (* I2C_ADDRESSINGMODE_10BIT *)
          hi2c.Instance^.OAR1 := (I2C_OAR1_OA1EN or I2C_OAR1_OA1MODE or hi2c.Init.OwnAddress1);
      end;

    (*---------------------------- I2Cx CR2 Configuration ----------------------*)
    (* Configure I2Cx: Addressing Master mode *)
    if (hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT) then
      hi2c.Instance^.CR2 := (I2C_CR2_ADD10);
    (* Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process *)
    hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or (I2C_CR2_AUTOEND or I2C_CR2_NACK);

    (*---------------------------- I2Cx OAR2 Configuration ---------------------*)
    (* Configure I2Cx: Dual mode and Own Address2 *)
    hi2c.Instance^.OAR2 := (hi2c.Init.DualAddressMode or hi2c.Init.OwnAddress2 or (hi2c.Init.OwnAddress2Masks shl 8));

    (*---------------------------- I2Cx CR1 Configuration ----------------------*)
    (* Configure I2Cx: Generalcall and NoStretch mode *)
    hi2c.Instance^.CR1 := (hi2c.Init.GeneralCallMode or hi2c.Init.NoStretchMode);

    (* Enable the selected I2C peripheral *)
    __HAL_I2C_ENABLE(hi2c);

    hi2c.ErrorCode := HAL_I2C_ERROR_NONE;
    hi2c.State := HAL_I2C_STATE_READY;

    exit(HAL_OK);
  end;

function HAL_I2C_DeInit(var hi2c: I2C_HandleTypeDef): HAL_StatusTypeDef;
  begin
    hi2c.State := HAL_I2C_STATE_BUSY;

    (* Disable the I2C Peripheral Clock *)
    __HAL_I2C_DISABLE(hi2c);

    (* DeInit the low level hardware: GPIO, CLOCK, NVIC *)
    HAL_I2C_MspDeInit(hi2c);

    hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

    hi2c.State := HAL_I2C_STATE_RESET;

    (* Release Lock *)
    __HAL_UNLOCK(hi2c.Lock);

    exit(HAL_OK);
  end;

procedure HAL_I2C_MspInit_stub(var hi2c: I2C_HandleTypeDef); assembler; nostackframe; public name 'HAL_I2C_MspInit';
  asm
    .weak HAL_I2C_MspInit
  end;

procedure HAL_I2C_MspDeInit_stub(var hi2c: I2C_HandleTypeDef); assembler; nostackframe; public name 'HAL_I2C_MspDeInit';
  asm
    .weak HAL_I2C_MspDeInit
  end;

function HAL_I2C_Master_Transmit(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
  var
    sizetmp: longword;
  begin
    sizetmp := 0;

    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        begin
        exit(HAL_ERROR);
        end;

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        begin
        exit(HAL_BUSY);
        end;

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MASTER_BUSY_TX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      (* Send Slave Address *)
      (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
      (* Size > 255, need to set RELOAD bit *)
      if (Size > 255) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);
        sizetmp := 255;
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, Size, I2C_AUTOEND_MODE, I2C_GENERATE_START_WRITE);
        sizetmp := Size;
        end;

      repeat
        (* Wait until TXIS flag is set *)
        if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
          begin
          if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
            begin
            exit(HAL_ERROR);
            end
          else
            begin
            exit(HAL_TIMEOUT);
            end;
          end;
        (* Write data to TXDR *)
        hi2c.Instance^.TXDR := pData^;
        Inc(pdata);
        Dec(sizetmp);
        Dec(Size);

        if ((sizetmp = 0) and (Size <> 0)) then
          begin
          (* Wait until TXE flag is set *)
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, False, Timeout) <> HAL_OK) then
            begin
            exit(HAL_TIMEOUT);
            end;

          if (Size > 255) then
            begin
            I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            sizetmp := 255;
            end
          else
            begin
            I2C_TransferConfig(hi2c, DevAddress, Size, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
            sizetmp := Size;
            end;
          end;
      until (Size <= 0);

      (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
      (* Wait until STOPF flag is set *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          exit(HAL_ERROR);
          end
        else
          begin
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c);

      hi2c.State := HAL_I2C_STATE_READY;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Master_Receive(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
  var
    sizetmp: longword;
  begin
    sizetmp := 0;

    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        begin
        exit(HAL_ERROR);
        end;

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        begin
        exit(HAL_BUSY);
        end;

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MASTER_BUSY_RX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      (* Send Slave Address *)
      (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
      (* Size > 255, need to set RELOAD bit *)
      if (Size > 255) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
        sizetmp := 255;
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, Size, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
        sizetmp := Size;
        end;

      repeat
        (* Wait until RXNE flag is set *)
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, False, Timeout) <> HAL_OK) then
          begin
          exit(HAL_TIMEOUT);
          end;

        (* Write data to RXDR *)
        pData^ := hi2c.Instance^.RXDR;
        Inc(pdata);
        Dec(sizetmp);
        Dec(Size);

        if ((sizetmp = 0) and (Size <> 0)) then
          begin
          (* Wait until TCR flag is set *)
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, False, Timeout) <> HAL_OK) then
            begin
            exit(HAL_TIMEOUT);
            end;

          if (Size > 255) then
            begin
            I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            sizetmp := 255;
            end
          else
            begin
            I2C_TransferConfig(hi2c, DevAddress, Size, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
            sizetmp := Size;
            end;
          end;
      until not (Size > 0);

      (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
      (* Wait until STOPF flag is set *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          exit(HAL_ERROR);
          end
        else
          begin
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c);

      hi2c.State := HAL_I2C_STATE_READY;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Slave_Transmit(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        begin
        exit(HAL_ERROR);
        end;

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_SLAVE_BUSY_RX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      (* Enable Address Acknowledge *)
      hi2c.Instance^.CR2 := hi2c.Instance^.CR2 and (not I2C_CR2_NACK);

      (* Wait until ADDR flag is set *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, False, Timeout) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
        exit(HAL_TIMEOUT);
        end;

      (* Clear ADDR flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

      (* If 10bit addressing mode is selected *)
      if (hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT) then
        begin
        (* Wait until ADDR flag is set *)
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, False, Timeout) <> HAL_OK) then
          begin
          (* Disable Address Acknowledge *)
          hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
          exit(HAL_TIMEOUT);
          end;

        (* Clear ADDR flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        end;

      (* Wait until DIR flag is set Transmitter mode *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, False, Timeout) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
        exit(HAL_TIMEOUT);
        end;

      repeat
        (* Wait until TXIS flag is set *)
        if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
          begin
          (* Disable Address Acknowledge *)
          hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;

          if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
            begin
            exit(HAL_ERROR);
            end
          else
            begin
            exit(HAL_TIMEOUT);
            end;
          end;

        (* Read data from TXDR *)
        hi2c.Instance^.TXDR := pData^;
        Inc(pdata);
        Dec(Size);
      until not (Size > 0);

      (* Wait until STOP flag is set *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;

        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          (* Normal use case for Transmitter mode *)
          (* A NACK is generated to confirm the end of transfer *)
          hi2c.ErrorCode := HAL_I2C_ERROR_NONE;
          end
        else
          begin
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Clear STOP flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      (* Wait until BUSY flag is reset *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, True, Timeout) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
        exit(HAL_TIMEOUT);
        end;

      (* Disable Address Acknowledge *)
      hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;

      hi2c.State := HAL_I2C_STATE_READY;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Slave_Receive(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        begin
        exit(HAL_ERROR);
        end;

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_SLAVE_BUSY_RX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      (* Enable Address Acknowledge *)
      hi2c.Instance^.CR2 := hi2c.Instance^.CR2 and (not I2C_CR2_NACK);

      (* Wait until ADDR flag is set *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, False, Timeout) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
        exit(HAL_TIMEOUT);
        end;

      (* Clear ADDR flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

      (* Wait until DIR flag is reset Receiver mode *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, True, Timeout) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
        exit(HAL_TIMEOUT);
        end;

      while (Size > 0) do
        begin
        (* Wait until RXNE flag is set *)
        if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
          begin
          (* Disable Address Acknowledge *)
          hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
          if (hi2c.ErrorCode = HAL_I2C_ERROR_TIMEOUT) then
            begin
            exit(HAL_TIMEOUT);
            end
          else
            begin
            exit(HAL_ERROR);
            end;
          end;

        (* Read data from RXDR *)
        pData^ := hi2c.Instance^.RXDR;
        Inc(pdata);
        Dec(Size);
        end;

      (* Wait until STOP flag is set *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;

        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          exit(HAL_ERROR);
          end
        else
          begin
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Clear STOP flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      (* Wait until BUSY flag is reset *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, True, Timeout) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
        exit(HAL_TIMEOUT);
        end;


      (* Disable Address Acknowledge *)
      hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;

      hi2c.State := HAL_I2C_STATE_READY;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Mem_Write(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
  var
    Sizetmp: longword;
  begin
    Sizetmp := 0;

    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        begin
        exit(HAL_ERROR);
        end;

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        begin
        exit(HAL_BUSY);
        end;

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MEM_BUSY_TX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      (* Send Slave Address and Memory Address *)
      if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, Timeout) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_ERROR);
          end
        else
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Set NBYTES to write and reload if size > 255 *)
      (* Size > 255, need to set RELOAD bit *)
      if (Size > 255) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        Sizetmp := 255;
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, Size, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        Sizetmp := Size;
        end;

      repeat
        (* Wait until TXIS flag is set *)
        if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout) <> HAL_OK) then
          begin
          if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
            begin
            exit(HAL_ERROR);
            end
          else
            begin
            exit(HAL_TIMEOUT);
            end;
          end;

        (* Write data to DR *)
        hi2c.Instance^.TXDR := pData^;
        Inc(pdata);
        Dec(Sizetmp);
        Dec(Size);

        if ((Sizetmp = 0) and (Size <> 0)) then
          begin
          (* Wait until TCR flag is set *)
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, False, Timeout) <> HAL_OK) then
            exit(HAL_TIMEOUT);

          if (Size > 255) then
            begin
            I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            Sizetmp := 255;
            end
          else
            begin
            I2C_TransferConfig(hi2c, DevAddress, Size, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
            Sizetmp := Size;
            end;
          end;

      until not (Size > 0);

      (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
      (* Wait until STOPF flag is reset *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          exit(HAL_ERROR);
          end
        else
          begin
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c);

      hi2c.State := HAL_I2C_STATE_READY;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Mem_Read(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word; Timeout: longword): HAL_StatusTypeDef;
  var
    Sizetmp: longword;
  begin
    Sizetmp := 0;

    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        exit(HAL_ERROR);

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        exit(HAL_BUSY);

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MEM_BUSY_RX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      (* Send Slave Address and Memory Address *)
      if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, Timeout) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_ERROR);
          end
        else
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Send Slave Address *)
      (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
      (* Size > 255, need to set RELOAD bit *)
      if (Size > 255) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
        Sizetmp := 255;
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, Size, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
        Sizetmp := Size;
        end;

      repeat
        (* Wait until RXNE flag is set *)
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, False, Timeout) <> HAL_OK) then
          begin
          exit(HAL_TIMEOUT);
          end;

        (* Read data from RXDR *)
        pData^ := hi2c.Instance^.RXDR;
        Inc(pdata);

        (* Decrement the Size counter *)
        Dec(Sizetmp);
        Dec(Size);

        if ((Sizetmp = 0) and (Size <> 0)) then
          begin
          (* Wait until TCR flag is set *)
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, False, Timeout) <> HAL_OK) then
            begin
            exit(HAL_TIMEOUT);
            end;

          if (Size > 255) then
            begin
            I2C_TransferConfig(hi2c, DevAddress, 255, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            Sizetmp := 255;
            end
          else
            begin
            I2C_TransferConfig(hi2c, DevAddress, Size, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
            Sizetmp := Size;
            end;
          end;

      until not (Size > 0);

      (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
      (* Wait until STOPF flag is reset *)
      if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, I2C_TIMEOUT_STOPF) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          exit(HAL_ERROR);
          end
        else
          begin
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Clear STOP Flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      (* Clear Configuration Register 2 *)
      I2C_RESET_CR2(hi2c);

      hi2c.State := HAL_I2C_STATE_READY;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_IsDeviceReady(var hi2c: I2C_HandleTypeDef; DevAddress: word; Trials, Timeout: longword): HAL_StatusTypeDef;
  var
    tickstart, I2C_Trials: longword;
  begin
    tickstart := 0;

    I2C_Trials := 0;

    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        begin
        exit(HAL_BUSY);
        end;

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_BUSY;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      repeat
        (* Generate Start *)
        hi2c.Instance^.CR2 := I2C_GENERATE_START(hi2c.Init.AddressingMode, DevAddress);

        (* No need to Check TC flag, with AUTOEND mode the stop is automatically generated *)
        (* Wait until STOPF flag is set or a NACK flag is set*)
        tickstart := HAL_GetTick();
        while ((not __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) and (not __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) and (hi2c.State <> HAL_I2C_STATE_TIMEOUT)) do
          begin
          if (Timeout <> HAL_MAX_DELAY) then
            begin
            if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
              begin
              (* Device is ready *)
              hi2c.State := HAL_I2C_STATE_READY;
              (* Process Unlocked *)
              __HAL_UNLOCK(hi2c.Lock);
              exit(HAL_TIMEOUT);
              end;
            end;
          end;

        (* Check if the NACKF flag has not been set *)
        if (not __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) then
          begin
          (* Wait until STOPF flag is reset *)
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, False, Timeout) <> HAL_OK) then
            begin
            exit(HAL_TIMEOUT);
            end;

          (* Clear STOP Flag *)
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

          (* Device is ready *)
          hi2c.State := HAL_I2C_STATE_READY;

          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);

          exit(HAL_OK);
          end
        else
          begin
          (* Wait until STOPF flag is reset *)
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, False, Timeout) <> HAL_OK) then
            begin
            exit(HAL_TIMEOUT);
            end;

          (* Clear NACK Flag *)
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

          (* Clear STOP Flag, auto generated with autoend*)
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
          end;

        (* Check if the maximum allowed number of trials has been reached *)
        if (I2C_Trials = Trials) then
          begin
          Inc(I2C_Trials);

          (* Generate Stop *)
          hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_STOP;

          (* Wait until STOPF flag is reset *)
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, False, Timeout) <> HAL_OK) then
            begin
            exit(HAL_TIMEOUT);
            end;

          (* Clear STOP Flag *)
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
          end
      until not (I2C_Trials < Trials);

      hi2c.State := HAL_I2C_STATE_READY;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_TIMEOUT);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Master_Transmit_IT(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        begin
        exit(HAL_ERROR);
        end;

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        begin
        exit(HAL_BUSY);
        end;

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MASTER_BUSY_TX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      hi2c.pBuffPtr := pData;
      hi2c.XferCount := Size;
      if (Size > 255) then
        begin
        hi2c.XferSize := 255;
        end
      else
        begin
        hi2c.XferSize := Size;
        end;

      (* Send Slave Address *)
      (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
      if ((hi2c.XferSize = 255) and (hi2c.XferSize < hi2c.XferCount)) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_WRITE);
        end;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

    (* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock *)


      (* Enable ERR, TC, STOP, NACK, TXI interrupt *)
      (* possible to enable all of these *)
      (* I2C_IT_ERRI or I2C_IT_TCIor I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI or I2C_IT_TXI *)
      __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_TXI);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Master_Receive_IT(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        exit(HAL_ERROR);

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        exit(HAL_BUSY);

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MASTER_BUSY_RX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      hi2c.pBuffPtr := pData;
      hi2c.XferCount := Size;
      if (Size > 255) then
        begin
        hi2c.XferSize := 255;
        end
      else
        begin
        hi2c.XferSize := Size;
        end;

      (* Send Slave Address *)
      (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
      if ((hi2c.XferSize = 255) and (hi2c.XferSize < hi2c.XferCount)) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
        end;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

    (* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock *)

      (* Enable ERR, TC, STOP, NACK, RXI interrupt *)
      (* possible to enable all of these *)
      (* I2C_IT_ERRI or I2C_IT_TCIor I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI or I2C_IT_TXI *)
      __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_RXI);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Slave_Transmit_IT(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        exit(HAL_ERROR);

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_SLAVE_BUSY_TX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      (* Enable Address Acknowledge *)
      hi2c.Instance^.CR2 := hi2c.Instance^.CR2 and (not I2C_CR2_NACK);

      hi2c.pBuffPtr := pData;
      hi2c.XferSize := Size;
      hi2c.XferCount := Size;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

    (* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock *)

      (* Enable ERR, TC, STOP, NACK, TXI interrupt *)
      (* possible to enable all of these *)
      (* I2C_IT_ERRI or I2C_IT_TCIor I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI or I2C_IT_TXI *)
      __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_TXI);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Slave_Receive_IT(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        begin
        exit(HAL_ERROR);
        end;

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_SLAVE_BUSY_RX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      (* Enable Address Acknowledge *)
      hi2c.Instance^.CR2 := hi2c.Instance^.CR2 and (not I2C_CR2_NACK);

      hi2c.pBuffPtr := pData;
      hi2c.XferSize := Size;
      hi2c.XferCount := Size;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

    (* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock *)

      (* Enable ERR, TC, STOP, NACK, RXI interrupt *)
      (* possible to enable all of these *)
      (* I2C_IT_ERRI or I2C_IT_TCIor I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI or I2C_IT_TXI *)
      __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Mem_Write_IT(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        exit(HAL_ERROR);

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        exit(HAL_BUSY);

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MEM_BUSY_TX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      hi2c.pBuffPtr := pData;
      hi2c.XferCount := Size;
      if (Size > 255) then
        begin
        hi2c.XferSize := 255;
        end
      else
        begin
        hi2c.XferSize := Size;
        end;

      (* Send Slave Address and Memory Address *)
      if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_ERROR);
          end
        else
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Set NBYTES to write and reload if size > 255 *)
      (* Size > 255, need to set RELOAD bit *)
      if ((hi2c.XferSize = 255) and (hi2c.XferSize < hi2c.XferCount)) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        end;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

    (* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock *)

      (* Enable ERR, TC, STOP, NACK, TXI interrupt *)
      (* possible to enable all of these *)
      (* I2C_IT_ERRI or I2C_IT_TCIor I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI or I2C_IT_TXI *)
      __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_TXI);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Mem_Read_IT(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        exit(HAL_ERROR);

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        exit(HAL_BUSY);

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MEM_BUSY_RX;

      hi2c.pBuffPtr := pData;
      hi2c.XferCount := Size;
      if (Size > 255) then
        begin
        hi2c.XferSize := 255;
        end
      else
        begin
        hi2c.XferSize := Size;
        end;

      (* Send Slave Address and Memory Address *)
      if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_ERROR);
          end
        else
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
      (* Size > 255, need to set RELOAD bit *)
      if ((hi2c.XferSize = 255) and (hi2c.XferSize < hi2c.XferCount)) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
        end;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

    (* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock *)

      (* Enable ERR, TC, STOP, NACK, RXI interrupt *)
      (* possible to enable all of these *)
      (* I2C_IT_ERRI or I2C_IT_TCIor I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_ADDRI or I2C_IT_RXI or I2C_IT_TXI *)
      __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI or I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_RXI);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Master_Transmit_DMA(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
        if (pData = nil) or (Size = 0) then
          exit(HAL_ERROR);

        if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
          exit(HAL_BUSY);

        (* Process Locked *)
        __HAL_LOCK(hi2c.Lock);

        hi2c.State := HAL_I2C_STATE_MASTER_BUSY_TX;
        hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

        hi2c.pBuffPtr := pData;
        hi2c.XferCount := Size;
        if (Size > 255) then
          begin
          hi2c.XferSize := 255;
          end
        else
          begin
          hi2c.XferSize := Size;
          end;

        (* Set the I2C DMA transfer complete callback *)
        hi2c.hdmatx^.XferCpltCallback := @I2C_DMAMasterTransmitCplt;

        (* Set the DMA error callback *)
        hi2c.hdmatx^.XferErrorCallback := @I2C_DMAError;

        (* Enable the DMA channel *)
        HAL_DMA_Start_IT(hi2c.hdmatx^, pData, @hi2c.Instance^.TXDR, hi2c.XferSize);

        (* Send Slave Address *)
        (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
        if ((hi2c.XferSize = 255) and (hi2c.XferSize < hi2c.XferCount)) then
          begin
          I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);
          end
        else
          begin
          I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_WRITE);
          end;

        (* Wait until TXIS flag is set *)
        if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, I2C_TIMEOUT_TXIS) <> HAL_OK) then
          begin
          (* Disable Address Acknowledge *)
          hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;

          if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
            begin
            exit(HAL_ERROR);
            end
          else
            begin
            exit(HAL_TIMEOUT);
            end;
          end;

        (* Enable DMA Request *)
        hi2c.Instance^.CR1 := hi2c.Instance^.CR1 or I2C_CR1_TXDMAEN;

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.Lock);

        exit(HAL_OK);
      end
    else
      exit(HAL_BUSY);
  end;

function HAL_I2C_Master_Receive_DMA(var hi2c: I2C_HandleTypeDef; DevAddress: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
        if (pData = nil) or (Size = 0) then
          exit(HAL_ERROR);

        if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
          exit(HAL_BUSY);

        (* Process Locked *)
        __HAL_LOCK(hi2c.Lock);

        hi2c.State := HAL_I2C_STATE_MASTER_BUSY_RX;
        hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

        hi2c.pBuffPtr := pData;
        hi2c.XferCount := Size;
        if (Size > 255) then
          begin
          hi2c.XferSize := 255;
          end
        else
          hi2c.XferSize := Size;

        (* Set the I2C DMA transfer complete callback *)
        hi2c.hdmarx^.XferCpltCallback := @I2C_DMAMasterReceiveCplt;

        (* Set the DMA error callback *)
        hi2c.hdmarx^.XferErrorCallback := @I2C_DMAError;

        (* Enable the DMA channel *)
        HAL_DMA_Start_IT(hi2c.hdmarx^, @hi2c.Instance^.RXDR, pData, hi2c.XferSize);

        (* Send Slave Address *)
        (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
        if ((hi2c.XferSize = 255) and (hi2c.XferSize < hi2c.XferCount)) then
          I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ)
        else
          I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);

        (* Wait until RXNE flag is set *)
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, False, I2C_TIMEOUT_RXNE) <> HAL_OK) then
          exit(HAL_TIMEOUT);

        (* Enable DMA Request *)
        hi2c.Instance^.CR1 := hi2c.Instance^.CR1 or I2C_CR1_RXDMAEN;

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.Lock);

        exit(HAL_OK);
      end
    else
      exit(HAL_BUSY);
  end;

function HAL_I2C_Slave_Transmit_DMA(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
        if (pData = nil) or (Size = 0) then
          exit(HAL_ERROR);

        (* Process Locked *)
        __HAL_LOCK(hi2c.Lock);

        hi2c.State := HAL_I2C_STATE_SLAVE_BUSY_TX;
        hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

        hi2c.pBuffPtr := pData;
        hi2c.XferCount := Size;
        hi2c.XferSize := Size;

        (* Set the I2C DMA transfer complete callback *)
        hi2c.hdmatx^.XferCpltCallback := @I2C_DMASlaveTransmitCplt;

        (* Set the DMA error callback *)
        hi2c.hdmatx^.XferErrorCallback := @I2C_DMAError;

        (* Enable the DMA channel *)
        HAL_DMA_Start_IT(hi2c.hdmatx^, pData, @hi2c.Instance^.TXDR, hi2c.XferSize);

        (* Enable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 and (not I2C_CR2_NACK);

        (* Wait until ADDR flag is set *)
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, False, I2C_TIMEOUT_ADDR) <> HAL_OK) then
          begin
            (* Disable Address Acknowledge *)
            hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
            exit(HAL_TIMEOUT);
          end;

        (* Clear ADDR flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

        (* If 10bits addressing mode is selected *)
        if (hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT) then
          begin
            (* Wait until ADDR flag is set *)
            if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, False, I2C_TIMEOUT_ADDR) <> HAL_OK) then
              begin
                (* Disable Address Acknowledge *)
                hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
                exit(HAL_TIMEOUT);
              end;

            (* Clear ADDR flag *)
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
          end;

        (* Wait until DIR flag is set Transmitter mode *)
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, False, I2C_TIMEOUT_BUSY) <> HAL_OK) then
          begin
            (* Disable Address Acknowledge *)
            hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
            exit(HAL_TIMEOUT);
          end;

        (* Enable DMA Request *)
        hi2c.Instance^.CR1 := hi2c.Instance^.CR1 or I2C_CR1_TXDMAEN;

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.Lock);

        exit(HAL_OK);
      end
    else
      exit(HAL_BUSY);
  end;

function HAL_I2C_Slave_Receive_DMA(var hi2c: I2C_HandleTypeDef; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        exit(HAL_ERROR);

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_SLAVE_BUSY_RX;
      hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

      hi2c.pBuffPtr := pData;
      hi2c.XferSize := Size;
      hi2c.XferCount := Size;

      (* Set the I2C DMA transfer complete callback *)
      hi2c.hdmarx^.XferCpltCallback := @I2C_DMASlaveReceiveCplt;

      (* Set the DMA error callback *)
      hi2c.hdmarx^.XferErrorCallback := @I2C_DMAError;

      (* Enable the DMA channel *)
      HAL_DMA_Start_IT(hi2c.hdmarx^, @hi2c.Instance^.RXDR, pData, Size);

      (* Enable Address Acknowledge *)
      hi2c.Instance^.CR2 := hi2c.Instance^.CR2 and (not I2C_CR2_NACK);

      (* Wait until ADDR flag is set *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, False, I2C_TIMEOUT_ADDR) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
        exit(HAL_TIMEOUT);
        end;

      (* Clear ADDR flag *)
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

      (* Wait until DIR flag is set Receiver mode *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, True, I2C_TIMEOUT_DIR) <> HAL_OK) then
        begin
        (* Disable Address Acknowledge *)
        hi2c.Instance^.CR2 := hi2c.Instance^.CR2 or I2C_CR2_NACK;
        exit(HAL_TIMEOUT);
        end;

      (* Enable DMA Request *)
      hi2c.Instance^.CR1 := hi2c.Instance^.CR1 or I2C_CR1_RXDMAEN;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

function HAL_I2C_Mem_Write_DMA(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
        if (pData = nil) or (Size = 0) then
          exit(HAL_ERROR);

        if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
          exit(HAL_BUSY);

        (* Process Locked *)
        __HAL_LOCK(hi2c.Lock);

        hi2c.State := HAL_I2C_STATE_MEM_BUSY_TX;
        hi2c.ErrorCode := HAL_I2C_ERROR_NONE;

        hi2c.pBuffPtr := pData;
        hi2c.XferCount := Size;
        if Size > 255 then
          hi2c.XferSize := 255
        else
          hi2c.XferSize := Size;

        (* Set the I2C DMA transfer complete callback *)
        hi2c.hdmatx^.XferCpltCallback := @I2C_DMAMemTransmitCplt;

        (* Set the DMA error callback *)
        hi2c.hdmatx^.XferErrorCallback := @I2C_DMAError;

        (* Enable the DMA channel *)
        HAL_DMA_Start_IT(hi2c.hdmatx^, pData, @hi2c.Instance^.TXDR, hi2c.XferSize);

        (* Send Slave Address and Memory Address *)
        if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG) <> HAL_OK) then
          begin
            if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
              begin
                (* Process Unlocked *)
                __HAL_UNLOCK(hi2c.Lock);
                exit(HAL_ERROR);
              end
            else
              begin
                (* Process Unlocked *)
                __HAL_UNLOCK(hi2c.Lock);
                exit(HAL_TIMEOUT);
              end;
          end;

        (* Send Slave Address *)
        (* Set NBYTES to write and reload if size > 255 *)
        if (hi2c.XferSize = 255) and (hi2c.XferSize < hi2c.XferCount) then
          I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP)
        else
          I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);

        (* Wait until TXIS flag is set *)
        if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, I2C_TIMEOUT_TXIS) <> HAL_OK) then
          begin
            if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
              exit(HAL_ERROR)
            else
              exit(HAL_TIMEOUT);
          end;

        (* Enable DMA Request *)
        hi2c.Instance^.CR1 := hi2c.Instance^.CR1 or I2C_CR1_TXDMAEN;

        (* Process Unlocked *)
        __HAL_UNLOCK(hi2c.Lock);

        exit(HAL_OK);
      end
    else
      exit(HAL_BUSY);
  end;

function HAL_I2C_Mem_Read_DMA(var hi2c: I2C_HandleTypeDef; DevAddress, MemAddress, MemAddSize: word; pData: Pbyte; Size: word): HAL_StatusTypeDef;
  begin
    if hi2c.State = HAL_I2C_STATE_READY then
      begin
      if (pData = nil) or (Size = 0) then
        exit(HAL_ERROR);

      if __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) then
        exit(HAL_BUSY);

      (* Process Locked *)
      __HAL_LOCK(hi2c.Lock);

      hi2c.State := HAL_I2C_STATE_MEM_BUSY_RX;

      hi2c.pBuffPtr := pData;
      hi2c.XferCount := Size;
      if (Size > 255) then
        hi2c.XferSize := 255
      else
        hi2c.XferSize := Size;

      (* Set the I2C DMA transfer complete callback *)
      hi2c.hdmarx^.XferCpltCallback := @I2C_DMAMemReceiveCplt;

      (* Set the DMA error callback *)
      hi2c.hdmarx^.XferErrorCallback := @I2C_DMAError;

      (* Enable the DMA channel *)
      HAL_DMA_Start_IT(hi2c.hdmarx^, @hi2c.Instance^.RXDR, pData, hi2c.XferSize);

      (* Send Slave Address and Memory Address *)
      if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG) <> HAL_OK) then
        begin
        if (hi2c.ErrorCode = HAL_I2C_ERROR_AF) then
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_ERROR);
          end
        else
          begin
          (* Process Unlocked *)
          __HAL_UNLOCK(hi2c.Lock);
          exit(HAL_TIMEOUT);
          end;
        end;

      (* Set NBYTES to write and reload if size > 255 and generate RESTART *)
      if ((hi2c.XferSize = 255) and (hi2c.XferSize < hi2c.XferCount)) then
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
        end
      else
        begin
        I2C_TransferConfig(hi2c, DevAddress, hi2c.XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
        end;

      (* Wait until RXNE flag is set *)
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, False, I2C_TIMEOUT_RXNE) <> HAL_OK) then
        begin
        exit(HAL_TIMEOUT);
        end;

      (* Enable DMA Request *)
      hi2c.Instance^.CR1 := hi2c.Instance^.CR1 or I2C_CR1_RXDMAEN;

      (* Process Unlocked *)
      __HAL_UNLOCK(hi2c.Lock);

      exit(HAL_OK);
      end
    else
      begin
      exit(HAL_BUSY);
      end;
  end;

procedure HAL_I2C_EV_IRQHandler(var hi2c: I2C_HandleTypeDef);
  begin
    (* I2C in mode Transmitter ---------------------------------------------------*)
    if (((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TCR)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TC)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) or
      (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR))) and (__HAL_I2C_GET_IT_SOURCE(hi2c, (I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_TXI or I2C_IT_ADDRI)))) then
      begin
        (* Slave mode selected *)
        if (hi2c.State = HAL_I2C_STATE_SLAVE_BUSY_TX) then
          I2C_SlaveTransmit_ISR(hi2c);
      end;

    if (((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TCR)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TC)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) or
      (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF))) and (__HAL_I2C_GET_IT_SOURCE(hi2c, (I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_TXI)))) then
      begin
        (* Master mode selected *)
        if ((hi2c.State = HAL_I2C_STATE_MASTER_BUSY_TX) or (hi2c.State = HAL_I2C_STATE_MEM_BUSY_TX)) then
          I2C_MasterTransmit_ISR(hi2c);
      end;

    (* I2C in mode Receiver ----------------------------------------------------*)
    if (((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TCR)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TC)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) or
      (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR))) and (__HAL_I2C_GET_IT_SOURCE(hi2c, (I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_RXI or I2C_IT_ADDRI)))) then
      begin
        (* Slave mode selected *)
        if (hi2c.State = HAL_I2C_STATE_SLAVE_BUSY_RX) then
          I2C_SlaveReceive_ISR(hi2c);
      end;

    if (((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TCR)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TC)) or (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF)) or
      (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF))) and (__HAL_I2C_GET_IT_SOURCE(hi2c, (I2C_IT_TCI or I2C_IT_STOPI or I2C_IT_NACKI or I2C_IT_RXI)))) then
      begin
        (* Master mode selected *)
        if ((hi2c.State = HAL_I2C_STATE_MASTER_BUSY_RX) or (hi2c.State = HAL_I2C_STATE_MEM_BUSY_RX)) then
          I2C_MasterReceive_ISR(hi2c);
      end;
  end;

procedure HAL_I2C_ER_IRQHandler(var hi2c: I2C_HandleTypeDef);
  begin
    (* I2C Bus error interrupt occurred ------------------------------------*)
    if ((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BERR)) and (__HAL_I2C_GET_IT_SOURCE(hi2c, I2C_IT_ERRI))) then
      begin
        hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_BERR;

        (* Clear BERR flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);
      end;

    (* I2C Over-Run/Under-Run interrupt occurred ----------------------------------------*)
    if ((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_OVR)) and (__HAL_I2C_GET_IT_SOURCE(hi2c, I2C_IT_ERRI))) then
      begin
        hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_OVR;

        (* Clear OVR flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_OVR);
      end;

    (* I2C Arbitration Loss error interrupt occurred -------------------------------------*)
    if ((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ARLO)) and (__HAL_I2C_GET_IT_SOURCE(hi2c, I2C_IT_ERRI))) then
      begin
        hi2c.ErrorCode := hi2c.ErrorCode or HAL_I2C_ERROR_ARLO;

        (* Clear ARLO flag *)
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ARLO);
      end;

    (* Call the Error Callback in case of Error detected *)
    if (hi2c.ErrorCode <> HAL_I2C_ERROR_NONE) then
      begin
        hi2c.State := HAL_I2C_STATE_READY;

        HAL_I2C_ErrorCallback(hi2c);
      end;
  end;

procedure HAL_I2C_MasterTxCpltCallback(var hi2c: I2C_HandleTypeDef);
  begin
  end;

procedure HAL_I2C_MasterRxCpltCallback(var hi2c: I2C_HandleTypeDef);
  begin
  end;

procedure HAL_I2C_SlaveTxCpltCallback(var hi2c: I2C_HandleTypeDef);
  begin
  end;

procedure HAL_I2C_SlaveRxCpltCallback(var hi2c: I2C_HandleTypeDef);
  begin
  end;

procedure HAL_I2C_MemTxCpltCallback(var hi2c: I2C_HandleTypeDef);
  begin
  end;

procedure HAL_I2C_MemRxCpltCallback(var hi2c: I2C_HandleTypeDef);
  begin
  end;

procedure HAL_I2C_ErrorCallback(var hi2c: I2C_HandleTypeDef);
  begin
  end;

function HAL_I2C_GetState(var hi2c: I2C_HandleTypeDef): HAL_I2C_StateTypeDef;
  begin
    exit(hi2c.State);
  end;

function HAL_I2C_GetError(var hi2c: I2C_HandleTypeDef): longword;
  begin
    exit(hi2c.ErrorCode);
  end;

end.
