(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_ll_sdmmc.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of SDMMC HAL module.
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
unit stm32f7xx_ll_sdmmc;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal;

(**
  * @brief  SDMMC Configuration Structure definition
   *)

type
  SDMMC_InitTypeDef = record
    ClockEdge: longword;  (*!< Specifies the clock transition on which the bit capture is made.
                                      This parameter can be a value of @ref SDMMC_LL_Clock_Edge                  *)
    ClockBypass: longword;  (*!< Specifies whether the SDMMC Clock divider bypass is
                                      enabled or disabled.
                                      This parameter can be a value of @ref SDMMC_LL_Clock_Bypass                *)
    ClockPowerSave: longword;  (*!< Specifies whether SDMMC Clock output is enabled or
                                      disabled when the bus is idle.
                                      This parameter can be a value of @ref SDMMC_LL_Clock_Power_Save            *)
    BusWide: longword;  (*!< Specifies the SDMMC bus width.
                                      This parameter can be a value of @ref SDMMC_LL_Bus_Wide                    *)
    HardwareFlowControl: longword;  (*!< Specifies whether the SDMMC hardware flow control is enabled or disabled.
                                      This parameter can be a value of @ref SDMMC_LL_Hardware_Flow_Control       *)
    ClockDiv: longword;  (*!< Specifies the clock frequency of the SDMMC controller.
                                      This parameter can be a value between Min_Data = 0 and Max_Data = 255  *)
  end;


  (**
  * @brief  SDMMC Command Control structure
   *)

  SDMMC_CmdInitTypeDef = record
    Argument: longword;  (*!< Specifies the SDMMC command argument which is sent
                                     to a card as part of a command message. If a command
                                     contains an argument, it must be loaded into this register
                                     before writing the command to the command register.               *)
    CmdIndex: longword;  (*!< Specifies the SDMMC command index. It must be Min_Data = 0 and
                                     Max_Data = 64                                                     *)
    Response: longword;  (*!< Specifies the SDMMC response type.
                                     This parameter can be a value of @ref SDMMC_LL_Response_Type          *)
    WaitForInterrupt: longword;  (*!< Specifies whether SDMMC wait for interrupt request is
                                     enabled or disabled.
                                     This parameter can be a value of @ref SDMMC_LL_Wait_Interrupt_State   *)
    CPSM: longword;  (*!< Specifies whether SDMMC Command path state machine (CPSM)
                                     is enabled or disabled.
                                     This parameter can be a value of @ref SDMMC_LL_CPSM_State             *)
  end;


  (**
  * @brief  SDMMC Data Control structure
   *)

  SDMMC_DataInitTypeDef = record
    DataTimeOut: longword;  (*!< Specifies the data timeout period in card bus clock periods.   *)
    DataLength: longword;  (*!< Specifies the number of data bytes to be transferred.          *)
    DataBlockSize: longword;  (*!< Specifies the data block size for block transfer.
                                     This parameter can be a value of @ref SDMMC_LL_Data_Block_Size     *)
    TransferDir: longword;  (*!< Specifies the data transfer direction, whether the transfer
                                     is a read or write.
                                     This parameter can be a value of @ref SDMMC_LL_Transfer_Direction  *)
    TransferMode: longword;  (*!< Specifies whether data transfer is in stream or block mode.
                                     This parameter can be a value of @ref SDMMC_LL_Transfer_Type       *)
    DPSM: longword;  (*!< Specifies whether SDMMC Data path state machine (DPSM)
                                     is enabled or disabled.
                                     This parameter can be a value of @ref SDMMC_LL_DPSM_State          *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup SDMMC_LL_Exported_Constants SDMMC_LL Exported Constants
  * @{
   *)

  (** @defgroup SDMMC_LL_Clock_Edge Clock Edge
  * @{
   *)

const
  SDMMC_CLOCK_EDGE_RISING = ($00000000);
  SDMMC_CLOCK_EDGE_FALLING = SDMMC_CLKCR_NEGEDGE;

  SDMMC_CLOCK_BYPASS_DISABLE = ($00000000);
  SDMMC_CLOCK_BYPASS_ENABLE = SDMMC_CLKCR_BYPASS;

  SDMMC_CLOCK_POWER_SAVE_DISABLE = ($00000000);
  SDMMC_CLOCK_POWER_SAVE_ENABLE = SDMMC_CLKCR_PWRSAV;

  SDMMC_BUS_WIDE_1B = ($00000000);
  SDMMC_BUS_WIDE_4B = SDMMC_CLKCR_WIDBUS_0;
  SDMMC_BUS_WIDE_8B = SDMMC_CLKCR_WIDBUS_1;

  SDMMC_HARDWARE_FLOW_CONTROL_DISABLE = ($00000000);
  SDMMC_HARDWARE_FLOW_CONTROL_ENABLE = SDMMC_CLKCR_HWFC_EN;

  SDMMC_RESPONSE_NO = ($00000000);
  SDMMC_RESPONSE_SHORT = SDMMC_CMD_WAITRESP_0;
  SDMMC_RESPONSE_LONG = SDMMC_CMD_WAITRESP;

  SDMMC_WAIT_NO = ($00000000);
  SDMMC_WAIT_IT = SDMMC_CMD_WAITINT;
  SDMMC_WAIT_PEND = SDMMC_CMD_WAITPEND;

  SDMMC_CPSM_DISABLE = ($00000000);
  SDMMC_CPSM_ENABLE = SDMMC_CMD_CPSMEN;

  SDMMC_RESP1 = ($00000000);
  SDMMC_RESP2 = ($00000004);
  SDMMC_RESP3 = ($00000008);
  SDMMC_RESP4 = ($0000000C);

  SDMMC_DATABLOCK_SIZE_1B = ($00000000);
  SDMMC_DATABLOCK_SIZE_2B = SDMMC_DCTRL_DBLOCKSIZE_0;
  SDMMC_DATABLOCK_SIZE_4B = SDMMC_DCTRL_DBLOCKSIZE_1;
  SDMMC_DATABLOCK_SIZE_8B = (SDMMC_DCTRL_DBLOCKSIZE_0 or SDMMC_DCTRL_DBLOCKSIZE_1);
  SDMMC_DATABLOCK_SIZE_16B = SDMMC_DCTRL_DBLOCKSIZE_2;
  SDMMC_DATABLOCK_SIZE_32B = (SDMMC_DCTRL_DBLOCKSIZE_0 or SDMMC_DCTRL_DBLOCKSIZE_2);
  SDMMC_DATABLOCK_SIZE_64B = (SDMMC_DCTRL_DBLOCKSIZE_1 or SDMMC_DCTRL_DBLOCKSIZE_2);
  SDMMC_DATABLOCK_SIZE_128B = (SDMMC_DCTRL_DBLOCKSIZE_0 or SDMMC_DCTRL_DBLOCKSIZE_1 or SDMMC_DCTRL_DBLOCKSIZE_2);
  SDMMC_DATABLOCK_SIZE_256B = SDMMC_DCTRL_DBLOCKSIZE_3;
  SDMMC_DATABLOCK_SIZE_512B = (SDMMC_DCTRL_DBLOCKSIZE_0 or SDMMC_DCTRL_DBLOCKSIZE_3);
  SDMMC_DATABLOCK_SIZE_1024B = (SDMMC_DCTRL_DBLOCKSIZE_1 or SDMMC_DCTRL_DBLOCKSIZE_3);
  SDMMC_DATABLOCK_SIZE_2048B = (SDMMC_DCTRL_DBLOCKSIZE_0 or SDMMC_DCTRL_DBLOCKSIZE_1 or SDMMC_DCTRL_DBLOCKSIZE_3);
  SDMMC_DATABLOCK_SIZE_4096B = (SDMMC_DCTRL_DBLOCKSIZE_2 or SDMMC_DCTRL_DBLOCKSIZE_3);
  SDMMC_DATABLOCK_SIZE_8192B = (SDMMC_DCTRL_DBLOCKSIZE_0 or SDMMC_DCTRL_DBLOCKSIZE_2 or SDMMC_DCTRL_DBLOCKSIZE_3);
  SDMMC_DATABLOCK_SIZE_16384B = (SDMMC_DCTRL_DBLOCKSIZE_1 or SDMMC_DCTRL_DBLOCKSIZE_2 or SDMMC_DCTRL_DBLOCKSIZE_3);

  SDMMC_TRANSFER_DIR_TO_CARD = ($00000000);
  SDMMC_TRANSFER_DIR_TO_SDMMC = SDMMC_DCTRL_DTDIR;

  SDMMC_TRANSFER_MODE_BLOCK = ($00000000);
  SDMMC_TRANSFER_MODE_STREAM = SDMMC_DCTRL_DTMODE;

  SDMMC_DPSM_DISABLE = ($00000000);
  SDMMC_DPSM_ENABLE = SDMMC_DCTRL_DTEN;

  SDMMC_READ_WAIT_MODE_DATA2 = ($00000000);
  SDMMC_READ_WAIT_MODE_CLK = (SDMMC_DCTRL_RWMOD);

  SDMMC_IT_CCRCFAIL = SDMMC_STA_CCRCFAIL;
  SDMMC_IT_DCRCFAIL = SDMMC_STA_DCRCFAIL;
  SDMMC_IT_CTIMEOUT = SDMMC_STA_CTIMEOUT;
  SDMMC_IT_DTIMEOUT = SDMMC_STA_DTIMEOUT;
  SDMMC_IT_TXUNDERR = SDMMC_STA_TXUNDERR;
  SDMMC_IT_RXOVERR = SDMMC_STA_RXOVERR;
  SDMMC_IT_CMDREND = SDMMC_STA_CMDREND;
  SDMMC_IT_CMDSENT = SDMMC_STA_CMDSENT;
  SDMMC_IT_DATAEND = SDMMC_STA_DATAEND;
  SDMMC_IT_DBCKEND = SDMMC_STA_DBCKEND;
  SDMMC_IT_CMDACT = SDMMC_STA_CMDACT;
  SDMMC_IT_TXACT = SDMMC_STA_TXACT;
  SDMMC_IT_RXACT = SDMMC_STA_RXACT;
  SDMMC_IT_TXFIFOHE = SDMMC_STA_TXFIFOHE;
  SDMMC_IT_RXFIFOHF = SDMMC_STA_RXFIFOHF;
  SDMMC_IT_TXFIFOF = SDMMC_STA_TXFIFOF;
  SDMMC_IT_RXFIFOF = SDMMC_STA_RXFIFOF;
  SDMMC_IT_TXFIFOE = SDMMC_STA_TXFIFOE;
  SDMMC_IT_RXFIFOE = SDMMC_STA_RXFIFOE;
  SDMMC_IT_TXDAVL = SDMMC_STA_TXDAVL;
  SDMMC_IT_RXDAVL = SDMMC_STA_RXDAVL;
  SDMMC_IT_SDIOIT = SDMMC_STA_SDIOIT;

  SDMMC_FLAG_CCRCFAIL = SDMMC_STA_CCRCFAIL;
  SDMMC_FLAG_DCRCFAIL = SDMMC_STA_DCRCFAIL;
  SDMMC_FLAG_CTIMEOUT = SDMMC_STA_CTIMEOUT;
  SDMMC_FLAG_DTIMEOUT = SDMMC_STA_DTIMEOUT;
  SDMMC_FLAG_TXUNDERR = SDMMC_STA_TXUNDERR;
  SDMMC_FLAG_RXOVERR = SDMMC_STA_RXOVERR;
  SDMMC_FLAG_CMDREND = SDMMC_STA_CMDREND;
  SDMMC_FLAG_CMDSENT = SDMMC_STA_CMDSENT;
  SDMMC_FLAG_DATAEND = SDMMC_STA_DATAEND;
  SDMMC_FLAG_DBCKEND = SDMMC_STA_DBCKEND;
  SDMMC_FLAG_CMDACT = SDMMC_STA_CMDACT;
  SDMMC_FLAG_TXACT = SDMMC_STA_TXACT;
  SDMMC_FLAG_RXACT = SDMMC_STA_RXACT;
  SDMMC_FLAG_TXFIFOHE = SDMMC_STA_TXFIFOHE;
  SDMMC_FLAG_RXFIFOHF = SDMMC_STA_RXFIFOHF;
  SDMMC_FLAG_TXFIFOF = SDMMC_STA_TXFIFOF;
  SDMMC_FLAG_RXFIFOF = SDMMC_STA_RXFIFOF;
  SDMMC_FLAG_TXFIFOE = SDMMC_STA_TXFIFOE;
  SDMMC_FLAG_RXFIFOE = SDMMC_STA_RXFIFOE;
  SDMMC_FLAG_TXDAVL = SDMMC_STA_TXDAVL;
  SDMMC_FLAG_RXDAVL = SDMMC_STA_RXDAVL;
  SDMMC_FLAG_SDIOIT = SDMMC_STA_SDIOIT;

  CLKCR_CLEAR_MASK = ((SDMMC_CLKCR_CLKDIV or SDMMC_CLKCR_PWRSAV or SDMMC_CLKCR_BYPASS or SDMMC_CLKCR_WIDBUS or SDMMC_CLKCR_NEGEDGE or SDMMC_CLKCR_HWFC_EN));

  DCTRL_CLEAR_MASK = ((SDMMC_DCTRL_DTEN or SDMMC_DCTRL_DTDIR or SDMMC_DCTRL_DTMODE or SDMMC_DCTRL_DBLOCKSIZE));

  CMD_CLEAR_MASK = ((SDMMC_CMD_CMDINDEX or SDMMC_CMD_WAITRESP or SDMMC_CMD_WAITINT or SDMMC_CMD_WAITPEND or SDMMC_CMD_CPSMEN or SDMMC_CMD_SDIOSUSPEND));

  (* SDMMC Initialization Frequency (400KHz max)  *)
  SDMMC_INIT_CLK_DIV = ($76);

  (* SDMMC Data Transfer Frequency (25MHz max)  *)
  SDMMC_TRANSFER_CLK_DIV = ($0);

(* Initialization/de-initialization functions  ********************************* *)
function SDMMC_Init(var SDMMCx: SDMMC_TypeDef; Init: SDMMC_InitTypeDef): HAL_StatusTypeDef;

(* I/O operation functions  **************************************************** *)
(* Blocking mode: Polling  *)
function SDMMC_ReadFIFO(var SDMMCx: SDMMC_TypeDef): longword;
function SDMMC_WriteFIFO(var SDMMCx: SDMMC_TypeDef; const pWriteData: longword): HAL_StatusTypeDef;

(* Peripheral Control functions  *********************************************** *)
function SDMMC_PowerState_ON(var SDMMCx: SDMMC_TypeDef): HAL_StatusTypeDef;
function SDMMC_PowerState_OFF(var SDMMCx: SDMMC_TypeDef): HAL_StatusTypeDef;
function SDMMC_GetPowerState(var SDMMCx: SDMMC_TypeDef): longword;

(* Command path state machine (CPSM) management functions  *)
function SDMMC_SendCommand(var SDMMCx: SDMMC_TypeDef; const Command: SDMMC_CmdInitTypeDef): HAL_StatusTypeDef;
function SDMMC_GetCommandResponse(var SDMMCx: SDMMC_TypeDef): byte;
function SDMMC_GetResponse(var SDMMCx: SDMMC_TypeDef; Response: longword): longword;

(* Data path state machine (DPSM) management functions  *)
function SDMMC_DataConfig(var SDMMCx: SDMMC_TypeDef; const Data: SDMMC_DataInitTypeDef): HAL_StatusTypeDef;
function SDMMC_GetDataCounter(var SDMMCx: SDMMC_TypeDef): longword;
function SDMMC_GetFIFOCount(var SDMMCx: SDMMC_TypeDef): longword;

(* SDMMC Cards mode management functions  *)
function SDMMC_SetSDMMCReadWaitMode(var SDMMCx: SDMMC_TypeDef; SDMMC_ReadWaitMode: longword): HAL_StatusTypeDef;

procedure __SDMMC_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_DMA_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_DMA_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_ENABLE_IT(var __INSTANCE__: SDMMC_TypeDef; __INTERRUPT__: longword);
procedure __SDMMC_DISABLE_IT(var __INSTANCE__: SDMMC_TypeDef; __INTERRUPT__: longword);
function __SDMMC_GET_FLAG(var __INSTANCE__: SDMMC_TypeDef; __FLAG__: longword): boolean;
procedure __SDMMC_CLEAR_FLAG(var __INSTANCE__: SDMMC_TypeDef; __FLAG__: longword);
function __SDMMC_GET_IT(var __INSTANCE__: SDMMC_TypeDef; __INTERRUPT__: longword): boolean;
procedure __SDMMC_CLEAR_IT(var __INSTANCE__: SDMMC_TypeDef; __INTERRUPT__: longword);
procedure __SDMMC_START_READWAIT_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_START_READWAIT_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_STOP_READWAIT_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_STOP_READWAIT_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_OPERATION_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_OPERATION_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_SUSPEND_CMD_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
procedure __SDMMC_SUSPEND_CMD_DISABLE(var __INSTANCE__: SDMMC_TypeDef);

implementation

function SDMMC_Init(var SDMMCx: SDMMC_TypeDef; Init: SDMMC_InitTypeDef): HAL_StatusTypeDef;
var
  tmpreg: longword;
begin
  (* Set SDMMC configuration parameters *)
  tmpreg := (Init.ClockEdge or Init.ClockBypass or Init.ClockPowerSave or Init.BusWide or Init.HardwareFlowControl or Init.ClockDiv);

  (* Write to SDMMC CLKCR *)
  SDMMCx.CLKCR := (SDMMCx.CLKCR and (not longword(CLKCR_CLEAR_MASK))) or tmpreg;

  exit(HAL_OK);
end;

function SDMMC_ReadFIFO(var SDMMCx: SDMMC_TypeDef): longword;
begin
  (* Read data from Rx FIFO *)
  exit(SDMMCx.FIFO);
end;

function SDMMC_WriteFIFO(var SDMMCx: SDMMC_TypeDef; const pWriteData: longword): HAL_StatusTypeDef;
begin
  (* Write data to FIFO *)
  SDMMCx.FIFO := pWriteData;

  exit(HAL_OK);
end;

function SDMMC_PowerState_ON(var SDMMCx: SDMMC_TypeDef): HAL_StatusTypeDef;
begin
  (* Set power state to ON *)
  SDMMCx.POWER := SDMMC_POWER_PWRCTRL;

  exit(HAL_OK);
end;

function SDMMC_PowerState_OFF(var SDMMCx: SDMMC_TypeDef): HAL_StatusTypeDef;
begin
  (* Set power state to OFF *)
  SDMMCx.POWER := $00000000;

  exit(HAL_OK);
end;

function SDMMC_GetPowerState(var SDMMCx: SDMMC_TypeDef): longword;
begin
  exit(SDMMCx.POWER and SDMMC_POWER_PWRCTRL);
end;

function SDMMC_SendCommand(var SDMMCx: SDMMC_TypeDef; const Command: SDMMC_CmdInitTypeDef): HAL_StatusTypeDef;
var
  tmpreg: longword;
begin
  (* Set the SDMMC Argument value *)
  SDMMCx.ARG := Command.Argument;

  (* Set SDMMC command parameters *)
  tmpreg := (Command.CmdIndex or Command.Response or Command.WaitForInterrupt or Command.CPSM);

  (* Write to SDMMC CMD register *)
  SDMMCx.CMD := (SDMMCx.CMD and (not longword(CMD_CLEAR_MASK))) or tmpreg;

  exit(HAL_OK);
end;

function SDMMC_GetCommandResponse(var SDMMCx: SDMMC_TypeDef): byte;
begin
  exit(byte(SDMMCx.RESPCMD));
end;

function SDMMC_GetResponse(var SDMMCx: SDMMC_TypeDef; Response: longword): longword;
var
  tmp: pbyte;
begin
  (* Get the response *)
  tmp := @pbyte(@SDMMCx.RESP1)[Response];

  exit(plongword(tmp)^);
end;

function SDMMC_DataConfig(var SDMMCx: SDMMC_TypeDef; const Data: SDMMC_DataInitTypeDef): HAL_StatusTypeDef;
var
  tmpreg: longword;
begin
  (* Set the SDMMC Data TimeOut value *)
  SDMMCx.DTIMER := Data.DataTimeOut;

  (* Set the SDMMC DataLength value *)
  SDMMCx.DLEN := Data.DataLength;

  (* Set the SDMMC data configuration parameters *)
  tmpreg := (Data.DataBlockSize or Data.TransferDir or Data.TransferMode or Data.DPSM);

  (* Write to SDMMC DCTRL *)
  SDMMCx.DCTRL := (SDMMCx.DCTRL and (not longword(DCTRL_CLEAR_MASK))) or tmpreg;

  exit(HAL_OK);
end;

function SDMMC_GetDataCounter(var SDMMCx: SDMMC_TypeDef): longword;
begin
  exit(SDMMCx.DCOUNT);
end;

function SDMMC_GetFIFOCount(var SDMMCx: SDMMC_TypeDef): longword;
begin
  exit(SDMMCx.FIFO);
end;

function SDMMC_SetSDMMCReadWaitMode(var SDMMCx: SDMMC_TypeDef; SDMMC_ReadWaitMode: longword): HAL_StatusTypeDef;
begin
  (* Set SDMMC read wait mode *)
  SDMMCx.DCTRL := SDMMCx.DCTRL or SDMMC_ReadWaitMode;

  exit(HAL_OK);
end;

(**
  * @brief  Enable the SDMMC device.
  * @param  __INSTANCE__: SDMMC Instance
  * @retval None
  *)
procedure __SDMMC_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.CLKCR := __INSTANCE__.CLKCR or SDMMC_CLKCR_CLKEN;
end;

(**
  * @brief  Disable the SDMMC device.
  * @param  __INSTANCE__: SDMMC Instance
  * @retval None
  *)
procedure __SDMMC_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.CLKCR := __INSTANCE__.CLKCR and (not SDMMC_CLKCR_CLKEN);
end;

(**
  * @brief  Enable the SDMMC DMA transfer.
  * @param  __INSTANCE__: SDMMC Instance
  * @retval None
  *)
procedure __SDMMC_DMA_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.DCTRL := __INSTANCE__.DCTRL or SDMMC_DCTRL_DMAEN;
end;

(**
  * @brief  Disable the SDMMC DMA transfer.
  * @param  __INSTANCE__: SDMMC Instance
  * @retval None
  *)
procedure __SDMMC_DMA_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.DCTRL := __INSTANCE__.DCTRL and (not SDMMC_DCTRL_DMAEN);
end;

(**
  * @brief  Enable the SDMMC device interrupt.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @param  __INTERRUPT__ : specifies the SDMMC interrupt sources to be enabled.
  *         This parameter can be one or a combination of the following values:
  *            @arg SDMMC_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDMMC_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDMMC_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDMMC_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDMMC_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDMMC_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDMMC_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDMMC_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDMMC_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *            @arg SDMMC_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDMMC_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDMMC_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDMMC_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDMMC_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDMMC_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDMMC_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDMMC_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDMMC_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDMMC_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDMMC_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDMMC_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDMMC_IT_SDIOIT:   SD I/O interrupt received interrupt
  * @retval None
  *)
procedure __SDMMC_ENABLE_IT(var __INSTANCE__: SDMMC_TypeDef; __INTERRUPT__: longword);
begin
  __INSTANCE__.MASK := __INSTANCE__.MASK or (__INTERRUPT__);
end;

(**
  * @brief  Disable the SDMMC device interrupt.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @param  __INTERRUPT__ : specifies the SDMMC interrupt sources to be disabled.
  *          This parameter can be one or a combination of the following values:
  *            @arg SDMMC_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDMMC_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDMMC_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDMMC_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDMMC_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDMMC_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDMMC_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDMMC_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDMMC_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *            @arg SDMMC_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDMMC_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDMMC_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDMMC_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDMMC_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDMMC_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDMMC_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDMMC_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDMMC_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDMMC_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDMMC_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDMMC_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDMMC_IT_SDIOIT:   SD I/O interrupt received interrupt
  * @retval None
  *)
procedure __SDMMC_DISABLE_IT(var __INSTANCE__: SDMMC_TypeDef; __INTERRUPT__: longword);
begin
  __INSTANCE__.MASK := __INSTANCE__.MASK and (not (__INTERRUPT__));
end;

(**
  * @brief  Checks whether the specified SDMMC flag is set or not.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @param  __FLAG__: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg SDMMC_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *            @arg SDMMC_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *            @arg SDMMC_FLAG_CTIMEOUT: Command response timeout
  *            @arg SDMMC_FLAG_DTIMEOUT: Data timeout
  *            @arg SDMMC_FLAG_TXUNDERR: Transmit FIFO underrun error
  *            @arg SDMMC_FLAG_RXOVERR:  Received FIFO overrun error
  *            @arg SDMMC_FLAG_CMDREND:  Command response received (CRC check passed)
  *            @arg SDMMC_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDMMC_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is zero)
  *            @arg SDMMC_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *            @arg SDMMC_FLAG_CMDACT:   Command transfer in progress
  *            @arg SDMMC_FLAG_TXACT:    Data transmit in progress
  *            @arg SDMMC_FLAG_RXACT:    Data receive in progress
  *            @arg SDMMC_FLAG_TXFIFOHE: Transmit FIFO Half Empty
  *            @arg SDMMC_FLAG_RXFIFOHF: Receive FIFO Half Full
  *            @arg SDMMC_FLAG_TXFIFOF:  Transmit FIFO full
  *            @arg SDMMC_FLAG_RXFIFOF:  Receive FIFO full
  *            @arg SDMMC_FLAG_TXFIFOE:  Transmit FIFO empty
  *            @arg SDMMC_FLAG_RXFIFOE:  Receive FIFO empty
  *            @arg SDMMC_FLAG_TXDAVL:   Data available in transmit FIFO
  *            @arg SDMMC_FLAG_RXDAVL:   Data available in receive FIFO
  *            @arg SDMMC_FLAG_SDMMCIT:   SD I/O interrupt received
  * @retval The new state of SDMMC_FLAG (SET or RESET).
  *)
function __SDMMC_GET_FLAG(var __INSTANCE__: SDMMC_TypeDef; __FLAG__: longword): boolean;
begin
  exit((__INSTANCE__.STA and (__FLAG__)) <> 0);
end;


(**
  * @brief  Clears the SDMMC pending flags.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be one or a combination of the following values:
  *            @arg SDMMC_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *            @arg SDMMC_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *            @arg SDMMC_FLAG_CTIMEOUT: Command response timeout
  *            @arg SDMMC_FLAG_DTIMEOUT: Data timeout
  *            @arg SDMMC_FLAG_TXUNDERR: Transmit FIFO underrun error
  *            @arg SDMMC_FLAG_RXOVERR:  Received FIFO overrun error
  *            @arg SDMMC_FLAG_CMDREND:  Command response received (CRC check passed)
  *            @arg SDMMC_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDMMC_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is zero)
  *            @arg SDMMC_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *            @arg SDMMC_FLAG_SDMMCIT:   SD I/O interrupt received
  * @retval None
  *)
procedure __SDMMC_CLEAR_FLAG(var __INSTANCE__: SDMMC_TypeDef; __FLAG__: longword);
begin
  __INSTANCE__.ICR := (__FLAG__);
end;

(**
  * @brief  Checks whether the specified SDMMC interrupt has occurred or not.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @param  __INTERRUPT__: specifies the SDMMC interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SDMMC_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDMMC_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDMMC_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDMMC_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDMMC_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDMMC_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDMMC_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDMMC_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDMMC_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *            @arg SDMMC_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDMMC_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDMMC_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDMMC_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDMMC_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDMMC_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDMMC_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDMMC_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDMMC_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDMMC_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDMMC_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDMMC_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDMMC_IT_SDIOIT:   SD I/O interrupt received interrupt
  * @retval The new state of SDMMC_IT (SET or RESET).
  *)
function __SDMMC_GET_IT(var __INSTANCE__: SDMMC_TypeDef; __INTERRUPT__: longword): boolean;
begin
  exit((__INSTANCE__.STA and (__INTERRUPT__)) = (__INTERRUPT__));
end;

(**
  * @brief  Clears the SDMMC's interrupt pending bits.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @param  __INTERRUPT__: specifies the interrupt pending bit to clear.
  *          This parameter can be one or a combination of the following values:
  *            @arg SDMMC_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDMMC_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDMMC_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDMMC_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDMMC_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDMMC_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDMMC_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDMMC_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDMMC_IT_DATAEND:  Data end (data counter, SDMMC_DCOUNT, is zero) interrupt
  *            @arg SDMMC_IT_SDIOIT:   SD I/O interrupt received interrupt
  * @retval None
  *)
procedure __SDMMC_CLEAR_IT(var __INSTANCE__: SDMMC_TypeDef; __INTERRUPT__: longword);
begin
  __INSTANCE__.ICR := (__INTERRUPT__);
end;

(**
  * @brief  Enable Start the SD I/O Read Wait operation.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @retval None
  *)
procedure __SDMMC_START_READWAIT_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.DCTRL := __INSTANCE__.DCTRL or SDMMC_DCTRL_RWSTART;
end;

(**
  * @brief  Disable Start the SD I/O Read Wait operations.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @retval None
  *)
procedure __SDMMC_START_READWAIT_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.DCTRL := __INSTANCE__.DCTRL and (not SDMMC_DCTRL_RWSTART);
end;

(**
  * @brief  Enable Start the SD I/O Read Wait operation.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @retval None
  *)
procedure __SDMMC_STOP_READWAIT_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.DCTRL := __INSTANCE__.DCTRL or SDMMC_DCTRL_RWSTOP;
end;

(**
  * @brief  Disable Stop the SD I/O Read Wait operations.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @retval None
  *)
procedure __SDMMC_STOP_READWAIT_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.DCTRL := __INSTANCE__.DCTRL and (not SDMMC_DCTRL_RWSTOP);
end;

(**
  * @brief  Enable the SD I/O Mode Operation.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @retval None
  *)
procedure __SDMMC_OPERATION_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.DCTRL := __INSTANCE__.DCTRL or SDMMC_DCTRL_SDIOEN;
end;

(**
  * @brief  Disable the SD I/O Mode Operation.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @retval None
  *)
procedure __SDMMC_OPERATION_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.DCTRL := __INSTANCE__.DCTRL and (not SDMMC_DCTRL_SDIOEN);
end;

(**
  * @brief  Enable the SD I/O Suspend command sending.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @retval None
  *)
procedure __SDMMC_SUSPEND_CMD_ENABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.CMD := __INSTANCE__.CMD or SDMMC_CMD_SDIOSUSPEND;
end;

(**
  * @brief  Disable the SD I/O Suspend command sending.
  * @param  __INSTANCE__ : Pointer to SDMMC register base
  * @retval None
  *)
procedure __SDMMC_SUSPEND_CMD_DISABLE(var __INSTANCE__: SDMMC_TypeDef);
begin
  __INSTANCE__.CMD := __INSTANCE__.CMD and (not SDMMC_CMD_SDIOSUSPEND);
end;

end.
