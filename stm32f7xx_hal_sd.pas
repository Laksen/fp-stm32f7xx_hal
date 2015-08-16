(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_sd.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of SD HAL module.
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
unit stm32f7xx_hal_sd;

interface

{$mode objfpc}

uses
  stm32f7xx_hal,
  stm32f7xx_defs,
  stm32f7xx_hal_dma,
  stm32f7xx_ll_sdmmc;

{$packrecords c}

type
  SD_InitTypeDef = SDMMC_InitTypeDef;
  SD_TypeDef = SDMMC_TypeDef;
  PSD_TypeDef = ^SD_TypeDef;

type
  HAL_SD_ErrorTypedef = (
    SD_OK =     ( 0 ),
    SD_CMD_CRC_FAIL = ( 1 ),
      (*!< Command response received (but CRC check failed)               *)
    SD_DATA_CRC_FAIL = ( 2 ),
      (*!< Data block sent/received (CRC check failed)                    *)
    SD_CMD_RSP_TIMEOUT = ( 3 ),
      (*!< Command response timeout                                       *)
    SD_DATA_TIMEOUT = ( 4 ),
      (*!< Data timeout                                                   *)
    SD_TX_UNDERRUN = ( 5 ),
      (*!< Transmit FIFO underrun                                         *)
    SD_RX_OVERRUN = ( 6 ),
      (*!< Receive FIFO overrun                                           *)
    SD_START_BIT_ERR = ( 7 ),
      (*!< Start bit not detected on all data signals in wide bus mode    *)
    SD_CMD_OUT_OF_RANGE = ( 8 ),
      (*!< Command's argument was out of range.                           *)
    SD_ADDR_MISALIGNED = ( 9 ),
      (*!< Misaligned address                                             *)
    SD_BLOCK_LEN_ERR = ( 10 ),
      (*!< Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length  *)

    SD_ERASE_SEQ_ERR = ( 11 ),
      (*!< An error in the sequence of erase command occurs.             *)
    SD_BAD_ERASE_PARAM = ( 12 ),
      (*!< An invalid selection for erase groups                         *)
    SD_WRITE_PROT_VIOLATION = ( 13 ),
      (*!< Attempt to program a write protect block                      *)
    SD_LOCK_UNLOCK_FAILED = ( 14 ),
      (*!< Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card  *)

    SD_COM_CRC_FAILED = ( 15 ),
      (*!< CRC check of the previous command failed                      *)
    SD_ILLEGAL_CMD = ( 16 ),
      (*!< Command is not legal for the card state                       *)
    SD_CARD_ECC_FAILED = ( 17 ),
      (*!< Card internal ECC was applied but failed to correct the data  *)
    SD_CC_ERROR = ( 18 ),
      (*!< Internal card controller error                                *)
    SD_GENERAL_UNKNOWN_ERROR = ( 19 ),
      (*!< General or unknown error                                      *)
    SD_STREAM_READ_UNDERRUN = ( 20 ),
      (*!< The card could not sustain data transfer in stream read operation.  *)
    SD_STREAM_WRITE_OVERRUN = ( 21 ),
      (*!< The card could not sustain data programming in stream mode    *)
    SD_CID_CSD_OVERWRITE = ( 22 ),
      (*!< CID/CSD overwrite error                                       *)
    SD_WP_ERASE_SKIP = ( 23 ),
      (*!< Only partial address space was erased                         *)
    SD_CARD_ECC_DISABLED = ( 24 ),
      (*!< Command has been executed without using internal ECC          *)
    SD_ERASE_RESET = ( 25 ),
      (*!< Erase sequence was cleared before executing because an out of erase sequence command was received  *)

    SD_AKE_SEQ_ERROR = ( 26 ),
      (*!< Error in sequence of authentication.                          *)
    SD_INVALID_VOLTRANGE = ( 27 ),
    SD_ADDR_OUT_OF_RANGE = ( 28 ),
    SD_SWITCH_ERROR = ( 29 ),
    SD_SDMMC_DISABLED = ( 30 ),
    SD_SDMMC_FUNCTION_BUSY = ( 31 ),
    SD_SDMMC_FUNCTION_FAILED = ( 32 ),
    SD_SDMMC_UNKNOWN_FUNCTION = ( 33 ),
    (**
    * @brief  Standard error defines
     *)
    SD_INTERNAL_ERROR = ( 34 ),
    SD_NOT_CONFIGURED = ( 35 ),
    SD_REQUEST_PENDING = ( 36 ),
    SD_REQUEST_NOT_APPLICABLE = ( 37 ),
    SD_INVALID_PARAMETER = ( 38 ),
    SD_UNSUPPORTED_FEATURE = ( 39 ),
    SD_UNSUPPORTED_HW = ( 40 ),
    SD_ERROR = ( 41 )
  );

  HAL_SD_OperationTypedef = (
    SD_READ_SINGLE_BLOCK = 0,   (*!< Read single block operation       *)
    SD_READ_MULTIPLE_BLOCK = 1, (*!< Read multiple blocks operation    *)
    SD_WRITE_SINGLE_BLOCK = 2,  (*!< Write single block operation      *)
    SD_WRITE_MULTIPLE_BLOCK = 3 (*!< Write multiple blocks operation   *)
  );

  PSD_HandleTypeDef = ^SD_HandleTypeDef;

  SD_HandleTypeDef = record
    Instance: PSD_TypeDef;  (*!< SDMMC register base address                      *)
    Init: SD_InitTypeDef;  (*!< SD required parameters                          *)
    Lock: HAL_LockTypeDef;  (*!< SD locking object                               *)
    CardType: longword;  (*!< SD card type                                    *)
    RCA: longword;  (*!< SD relative card address                        *)
    CSD: array [0..3] of longword;  (*!< SD card specific data table                     *)
    CID: array [0..3] of longword;  (*!< SD card identification number table             *)
    SdTransferCplt: longword;  (*!< SD transfer complete flag in non blocking mode  *)
    SdTransferErr: HAL_SD_ErrorTypedef;  (*!< SD transfer error flag in non blocking mode     *)
    DmaTransferCplt: longword;  (*!< SD DMA transfer complete flag                   *)
    SdOperation: HAL_SD_OperationTypedef;  (*!< SD transfer operation (read/write)              *)
    hdmarx: PDMA_HandleTypeDef;  (*!< SD Rx DMA handle parameters                     *)
    hdmatx: PDMA_HandleTypeDef;  (*!< SD Tx DMA handle parameters                     *)
  end;

  (**
  * @}
   *)

  (** @defgroup SD_Exported_Types_Group2 Card Specific Data: CSD Register
  * @{
   *)

  HAL_SD_CSDTypedef = record
    CSDStruct: byte;  (*!< CSD structure                          *)
    SysSpecVersion: byte;  (*!< System specification version           *)
    Reserved1: byte;  (*!< Reserved                               *)
    TAAC: byte;  (*!< Data read access time 1                *)
    NSAC: byte;  (*!< Data read access time 2 in CLK cycles  *)
    MaxBusClkFrec: byte;  (*!< Max. bus clock frequency               *)
    CardComdClasses: word;  (*!< Card command classes                   *)
    RdBlockLen: byte;  (*!< Max. read data block length            *)
    PartBlockRead: byte;  (*!< Partial blocks for read allowed        *)
    WrBlockMisalign: byte;  (*!< Write block misalignment               *)
    RdBlockMisalign: byte;  (*!< Read block misalignment                *)
    DSRImpl: byte;  (*!< DSR implemented                        *)
    Reserved2: byte;  (*!< Reserved                               *)
    DeviceSize: longword;  (*!< Device Size                            *)
    MaxRdCurrentVDDMin: byte;  (*!< Max. read current @ VDD min            *)
    MaxRdCurrentVDDMax: byte;  (*!< Max. read current @ VDD max            *)
    MaxWrCurrentVDDMin: byte;  (*!< Max. write current @ VDD min           *)
    MaxWrCurrentVDDMax: byte;  (*!< Max. write current @ VDD max           *)
    DeviceSizeMul: byte;  (*!< Device size multiplier                 *)
    EraseGrSize: byte;  (*!< Erase group size                       *)
    EraseGrMul: byte;  (*!< Erase group size multiplier            *)
    WrProtectGrSize: byte;  (*!< Write protect group size               *)
    WrProtectGrEnable: byte;  (*!< Write protect group enable             *)
    ManDeflECC: byte;  (*!< Manufacturer default ECC               *)
    WrSpeedFact: byte;  (*!< Write speed factor                     *)
    MaxWrBlockLen: byte;  (*!< Max. write data block length           *)
    WriteBlockPaPartial: byte;  (*!< Partial blocks for write allowed       *)
    Reserved3: byte;  (*!< Reserved                               *)
    ContentProtectAppli: byte;  (*!< Content protection application         *)
    FileFormatGrouop: byte;  (*!< File format group                      *)
    CopyFlag: byte;  (*!< Copy flag (OTP)                        *)
    PermWrProtect: byte;  (*!< Permanent write protection             *)
    TempWrProtect: byte;  (*!< Temporary write protection             *)
    FileFormat: byte;  (*!< File format                            *)
    ECC: byte;  (*!< ECC code                               *)
    CSD_CRC: byte;  (*!< CSD CRC                                *)
    Reserved4: byte;  (*!< Always 1                               *)
  end;

  (**
  * @}
   *)

  (** @defgroup SD_Exported_Types_Group3 Card Identification Data: CID Register
  * @{
   *)

  HAL_SD_CIDTypedef = record
    ManufacturerID: byte;  (*!< Manufacturer ID        *)
    OEM_AppliID: word;  (*!< OEM/Application ID     *)
    ProdName1: longword;  (*!< Product Name part1     *)
    ProdName2: byte;  (*!< Product Name part2     *)
    ProdRev: byte;  (*!< Product Revision       *)
    ProdSN: longword;  (*!< Product Serial Number  *)
    Reserved1: byte;  (*!< Reserved1              *)
    ManufactDate: word;  (*!< Manufacturing Date     *)
    CID_CRC: byte;  (*!< CID CRC                *)
    Reserved2: byte;  (*!< Always 1               *)
  end;

  (**
  * @}
   *)

  (** @defgroup SD_Exported_Types_Group4 SD Card Status returned by ACMD13
  * @{
   *)

  HAL_SD_CardStatusTypedef = record
    DAT_BUS_WIDTH: byte;  (*!< Shows the currently defined data bus width                  *)
    SECURED_MODE: byte;  (*!< Card is in secured mode of operation                        *)
    SD_CARD_TYPE: word;  (*!< Carries information about card type                         *)
    SIZE_OF_PROTECTED_AREA: longword;  (*!< Carries information about the capacity of protected area    *)
    SPEED_CLASS: byte;  (*!< Carries information about the speed class of the card       *)
    PERFORMANCE_MOVE: byte;  (*!< Carries information about the card's performance move       *)
    AU_SIZE: byte;  (*!< Carries information about the card's allocation unit size   *)
    ERASE_SIZE: word;  (*!< Determines the number of AUs to be erased in one operation  *)
    ERASE_TIMEOUT: byte;  (*!< Determines the timeout for any number of AU erase           *)
    ERASE_OFFSET: byte;  (*!< Carries information about the erase offset                  *)
  end;

  (**
  * @}
   *)

  (** @defgroup SD_Exported_Types_Group5 SD Card information structure
  * @{
   *)

  HAL_SD_CardInfoTypedef = record
    SD_csd: HAL_SD_CSDTypedef;  (*!< SD card specific data register          *)
    SD_cid: HAL_SD_CIDTypedef;  (*!< SD card identification number register  *)
    CardCapacity: qword;  (*!< Card capacity                           *)
    CardBlockSize: longword;  (*!< Card block size                         *)
    RCA: word;  (*!< SD relative card address                *)
    CardType: byte;  (*!< SD card type                            *)
  end;

  HAL_SD_TransferStateTypedef = (
    SD_TRANSFER_OK = 0,    (*!< Transfer success       *)
    SD_TRANSFER_BUSY = 1,  (*!< Transfer is occurring  *)
    SD_TRANSFER_ERROR = 2  (*!< Transfer failed        *)
  );

  HAL_SD_CardStateTypedef = (
    SD_CARD_READY = (  $00000001 ),         (*!< Card state is ready                      *)
    SD_CARD_IDENTIFICATION = (  $00000002 ),(*!< Card is in identification state          *)
    SD_CARD_STANDBY = (  $00000003 ),       (*!< Card is in standby state                 *)
    SD_CARD_TRANSFER = (  $00000004 ),      (*!< Card is in transfer state                *)
    SD_CARD_SENDING = (  $00000005 ),       (*!< Card is sending an operation             *)
    SD_CARD_RECEIVING = (  $00000006 ),     (*!< Card is receiving operation information  *)
    SD_CARD_PROGRAMMING = (  $00000007 ),   (*!< Card is in programming state             *)
    SD_CARD_DISCONNECTED = (  $00000008 ),  (*!< Card is disconnected                     *)
    SD_CARD_ERROR = (  $000000FF )          (*!< Card is in error state                   *)
  );

  (**
  * @brief SD Commands Index
   *)

const
  SD_CMD_GO_IDLE_STATE = (0);  (*!< Resets the SD memory card.                                                                *)
  SD_CMD_SEND_OP_COND = (1);  (*!< Sends host capacity support information and activates the card's initialization process.  *)
  SD_CMD_ALL_SEND_CID = (2);  (*!< Asks any card connected to the host to send the CID numbers on the CMD line.              *)
  SD_CMD_SET_REL_ADDR = (3);  (*!< Asks the card to publish a new relative address (RCA).                                    *)
  SD_CMD_SET_DSR = (4);  (*!< Programs the DSR of all cards.                                                            *)
  SD_CMD_SDMMC_SEN_OP_COND = (5);  (*!< Sends host capacity support information (HCS) and asks the accessed card to send its
                                                                       operating condition register (OCR) content in the response on the CMD line.               *)
  SD_CMD_HS_SWITCH = (6);  (*!< Checks switchable function (mode 0) and switch card function (mode 1).                    *)
  SD_CMD_SEL_DESEL_CARD = (7);  (*!< Selects the card by its own relative address and gets deselected by any other address     *)
  SD_CMD_HS_SEND_EXT_CSD = (8);  (*!< Sends SD Memory Card interface condition, which includes host supply voltage information
                                                                       and asks the card whether card supports voltage.                                          *)
  SD_CMD_SEND_CSD = (9);  (*!< Addressed card sends its card specific data (CSD) on the CMD line.                        *)
  SD_CMD_SEND_CID = (10);  (*!< Addressed card sends its card identification (CID) on the CMD line.                       *)
  SD_CMD_READ_DAT_UNTIL_STOP = (11);  (*!< SD card doesn't support it.                                                               *)
  SD_CMD_STOP_TRANSMISSION = (12);  (*!< Forces the card to stop transmission.                                                     *)
  SD_CMD_SEND_STATUS = (13);  (*!< Addressed card sends its status register.                                                 *)
  SD_CMD_HS_BUSTEST_READ = (14);
  SD_CMD_GO_INACTIVE_STATE = (15);  (*!< Sends an addressed card into the inactive state.                                          *)
  SD_CMD_SET_BLOCKLEN = (16);  (*!< Sets the block length (in bytes for SDSC) for all following block commands
                                                                       (read, write, lock). Default block length is fixed to 512 Bytes. Not effective
                                                                       for SDHS and SDXC.                                                                        *)
  SD_CMD_READ_SINGLE_BLOCK = (17);  (*!< Reads single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of
                                                                       fixed 512 bytes in case of SDHC and SDXC.                                                 *)
  SD_CMD_READ_MULT_BLOCK = (18);  (*!< Continuously transfers data blocks from card to host until interrupted by
                                                                       STOP_TRANSMISSION command.                                                                *)
  SD_CMD_HS_BUSTEST_WRITE = (19);  (*!< 64 bytes tuning pattern is sent for SDR50 and SDR104.                                     *)
  SD_CMD_WRITE_DAT_UNTIL_STOP = (20);  (*!< Speed class control command.                                                              *)
  SD_CMD_SET_BLOCK_COUNT = (23);  (*!< Specify block count for CMD18 and CMD25.                                                  *)
  SD_CMD_WRITE_SINGLE_BLOCK = (24);  (*!< Writes single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of
                                                                       fixed 512 bytes in case of SDHC and SDXC.                                                 *)
  SD_CMD_WRITE_MULT_BLOCK = (25);  (*!< Continuously writes blocks of data until a STOP_TRANSMISSION follows.                     *)
  SD_CMD_PROG_CID = (26);  (*!< Reserved for manufacturers.                                                               *)
  SD_CMD_PROG_CSD = (27);  (*!< Programming of the programmable bits of the CSD.                                          *)
  SD_CMD_SET_WRITE_PROT = (28);  (*!< Sets the write protection bit of the addressed group.                                     *)
  SD_CMD_CLR_WRITE_PROT = (29);  (*!< Clears the write protection bit of the addressed group.                                   *)
  SD_CMD_SEND_WRITE_PROT = (30);  (*!< Asks the card to send the status of the write protection bits.                            *)
  SD_CMD_SD_ERASE_GRP_START = (32);  (*!< Sets the address of the first write block to be erased. (For SD card only).               *)
  SD_CMD_SD_ERASE_GRP_END = (33);  (*!< Sets the address of the last write block of the continuous range to be erased.            *)
  SD_CMD_ERASE_GRP_START = (35);  (*!< Sets the address of the first write block to be erased. Reserved for each command
                                                                       system set by switch function command (CMD6).                                             *)
  SD_CMD_ERASE_GRP_END = (36);  (*!< Sets the address of the last write block of the continuous range to be erased.
                                                                       Reserved for each command system set by switch function command (CMD6).                   *)
  SD_CMD_ERASE = (38);  (*!< Reserved for SD security applications.                                                    *)
  SD_CMD_FAST_IO = (39);  (*!< SD card doesn't support it (Reserved).                                                    *)
  SD_CMD_GO_IRQ_STATE = (40);  (*!< SD card doesn't support it (Reserved).                                                    *)
  SD_CMD_LOCK_UNLOCK = (42);  (*!< Sets/resets the password or lock/unlock the card. The size of the data block is set by
                                                                       the SET_BLOCK_LEN command.                                                                *)
  SD_CMD_APP_CMD = (55);  (*!< Indicates to the card that the next command is an application specific command rather
                                                                       than a standard command.                                                                  *)
  SD_CMD_GEN_CMD = (56);  (*!< Used either to transfer a data block to the card or to get a data block from the card
                                                                       for general purpose/application specific commands.                                        *)
  SD_CMD_NO_CMD = (64);
  (**
  * @brief Following commands are SD Card Specific commands.
  *        SDMMC_APP_CMD should be sent before sending these commands.
   *)

  SD_CMD_APP_SD_SET_BUSWIDTH = (6);  (*!< (ACMD6) Defines the data bus width to be used for data transfer. The allowed data bus
                                                                       widths are given in SCR register.                                                           *)
  SD_CMD_SD_APP_STATUS = (13);  (*!< (ACMD13) Sends the SD status.                                                               *)
  SD_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS = (22);  (*!< (ACMD22) Sends the number of the written (without errors) write blocks. Responds with
                                                                       32bit+CRC data block.                                                                       *)
  SD_CMD_SD_APP_OP_COND = (41);  (*!< (ACMD41) Sends host capacity support information (HCS) and asks the accessed card to
                                                                       send its operating condition register (OCR) content in the response on the CMD line.        *)
  SD_CMD_SD_APP_SET_CLR_CARD_DETECT = (42);  (*!< (ACMD42) Connects/Disconnects the 50 KOhm pull-up resistor on CD/DAT3 (pin 1) of the card.  *)
  SD_CMD_SD_APP_SEND_SCR = (51);  (*!< Reads the SD Configuration Register (SCR).                                                  *)
  SD_CMD_SDMMC_RW_DIRECT = (52);  (*!< For SD I/O card only, reserved for security specification.                                  *)
  SD_CMD_SDMMC_RW_EXTENDED = (53);  (*!< For SD I/O card only, reserved for security specification.                                  *)
  (**
  * @brief Following commands are SD Card Specific security commands.
  *        SD_CMD_APP_CMD should be sent before sending these commands.
   *)

  SD_CMD_SD_APP_GET_MKB = (43);  (*!< For SD card only  *)
  SD_CMD_SD_APP_GET_MID = (44);  (*!< For SD card only  *)
  SD_CMD_SD_APP_SET_CER_RN1 = (45);  (*!< For SD card only  *)
  SD_CMD_SD_APP_GET_CER_RN2 = (46);  (*!< For SD card only  *)
  SD_CMD_SD_APP_SET_CER_RES2 = (47);  (*!< For SD card only  *)
  SD_CMD_SD_APP_GET_CER_RES1 = (48);  (*!< For SD card only  *)
  SD_CMD_SD_APP_SECURE_READ_MULTIPLE_BLOCK = (18);  (*!< For SD card only  *)
  SD_CMD_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK = (25);  (*!< For SD card only  *)
  SD_CMD_SD_APP_SECURE_ERASE = (38);  (*!< For SD card only  *)
  SD_CMD_SD_APP_CHANGE_SECURE_AREA = (49);  (*!< For SD card only  *)
  SD_CMD_SD_APP_SECURE_WRITE_MKB = (48);  (*!< For SD card only  *)
  (**
  * @brief Supported SD Memory Cards
   *)

  STD_CAPACITY_SD_CARD_V1_1 = ($00000000);
  STD_CAPACITY_SD_CARD_V2_0 = ($00000001);
  HIGH_CAPACITY_SD_CARD = ($00000002);
  MULTIMEDIA_CARD = ($00000003);
  SECURE_DIGITAL_IO_CARD = ($00000004);
  HIGH_SPEED_MULTIMEDIA_CARD = ($00000005);
  SECURE_DIGITAL_IO_COMBO_CARD = ($00000006);
  HIGH_CAPACITY_MMC_CARD = ($00000007);

procedure __HAL_SD_SDMMC_ENABLE(var __HANDLE__: SD_HandleTypeDef);
procedure __HAL_SD_SDMMC_DISABLE(var __HANDLE__: SD_HandleTypeDef);
procedure __HAL_SD_SDMMC_DMA_ENABLE(var __HANDLE__: SD_HandleTypeDef);
procedure __HAL_SD_SDMMC_DMA_DISABLE(var __HANDLE__: SD_HandleTypeDef);
procedure __HAL_SD_SDMMC_ENABLE_IT(var __HANDLE__: SD_HandleTypeDef; __INTERRUPT__: longword);
procedure __HAL_SD_SDMMC_DISABLE_IT(var __HANDLE__: SD_HandleTypeDef; __INTERRUPT__: longword);
function __HAL_SD_SDMMC_GET_FLAG(var __HANDLE__: SD_HandleTypeDef; __FLAG__: longword): boolean;
procedure __HAL_SD_SDMMC_CLEAR_FLAG(var __HANDLE__: SD_HandleTypeDef; __FLAG__: longword);
function __HAL_SD_SDMMC_GET_IT(var __HANDLE__: SD_HandleTypeDef; __INTERRUPT__: longword): boolean;
procedure __HAL_SD_SDMMC_CLEAR_IT(var __HANDLE__: SD_HandleTypeDef; __INTERRUPT__: longword);

function HAL_SD_Init(var hsd: SD_HandleTypeDef; var SDCardInfo: HAL_SD_CardInfoTypedef): HAL_SD_ErrorTypedef;
function HAL_SD_DeInit(var hsd: SD_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_SD_MspInit(var hsd: SD_HandleTypeDef); external name 'HAL_SD_MspInit';
procedure HAL_SD_MspDeInit(var hsd: SD_HandleTypeDef); external name 'HAL_SD_MspDeInit';

(* Blocking mode: Polling  *)
function HAL_SD_ReadBlocks(var hsd: SD_HandleTypeDef; var pReadBuffer; ReadAddr: qword; BlockSize, NumberOfBlocks: longword): HAL_SD_ErrorTypedef;
function HAL_SD_WriteBlocks(var hsd: SD_HandleTypeDef; const pWriteBuffer; WriteAddr: qword; BlockSize, NumberOfBlocks: longword): HAL_SD_ErrorTypedef;
function HAL_SD_Erase(var hsd: SD_HandleTypeDef; startaddr, endaddr: qword): HAL_SD_ErrorTypedef;

(* Non-Blocking mode: Interrupt  *)
procedure HAL_SD_IRQHandler(var hsd: SD_HandleTypeDef);

(* Callback in non blocking modes (DMA)  *)
procedure HAL_SD_DMA_RxCpltCallback(var hdma: DMA_HandleTypeDef);   external name 'HAL_SD_DMA_RxCpltCallback';
procedure HAL_SD_DMA_RxErrorCallback(var hdma: DMA_HandleTypeDef);  external name 'HAL_SD_DMA_RxErrorCallback';
procedure HAL_SD_DMA_TxCpltCallback(var hdma: DMA_HandleTypeDef);   external name 'HAL_SD_DMA_TxCpltCallback';
procedure HAL_SD_DMA_TxErrorCallback(var hdma: DMA_HandleTypeDef);  external name 'HAL_SD_DMA_TxErrorCallback';
procedure HAL_SD_XferCpltCallback(var hsd: SD_HandleTypeDef);       external name 'HAL_SD_XferCpltCallback';
procedure HAL_SD_XferErrorCallback(var hsd: SD_HandleTypeDef);      external name 'HAL_SD_XferErrorCallback';

(* Non-Blocking mode: DMA  *)
function HAL_SD_ReadBlocks_DMA(var hsd: SD_HandleTypeDef; var pReadBuffer; ReadAddr: qword; BlockSize, NumberOfBlocks: longword): HAL_SD_ErrorTypedef;
function HAL_SD_WriteBlocks_DMA(var hsd: SD_HandleTypeDef; const pWriteBuffer; WriteAddr: qword; BlockSize, NumberOfBlocks: longword): HAL_SD_ErrorTypedef;
function HAL_SD_CheckWriteOperation(var hsd: SD_HandleTypeDef; Timeout: longword): HAL_SD_ErrorTypedef;
function HAL_SD_CheckReadOperation(var hsd: SD_HandleTypeDef; Timeout: longword): HAL_SD_ErrorTypedef;

function HAL_SD_Get_CardInfo(var hsd: SD_HandleTypeDef; var pCardInfo: HAL_SD_CardInfoTypedef): HAL_SD_ErrorTypedef;
function HAL_SD_WideBusOperation_Config(var hsd: SD_HandleTypeDef; WideMode: longword): HAL_SD_ErrorTypedef;
function HAL_SD_StopTransfer(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
function HAL_SD_HighSpeed(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;

(* Peripheral State functions  *********************************************** *)
function HAL_SD_SendSDStatus(var hsd: SD_HandleTypeDef; var pSDstatus): HAL_SD_ErrorTypedef;
function HAL_SD_GetCardStatus(var hsd: SD_HandleTypeDef; var pCardStatus: HAL_SD_CardStatusTypedef): HAL_SD_ErrorTypedef;
function HAL_SD_GetStatus(var hsd: SD_HandleTypeDef): HAL_SD_TransferStateTypedef;


implementation

(**
  * @brief  SDMMC Data block size
   *)

const
  DATA_BLOCK_SIZE = ((9 shl 4));
  (**
  * @brief  SDMMC Static flags, Timeout, FIFO Address
   *)

  SDMMC_STATIC_FLAGS = ((SDMMC_FLAG_CCRCFAIL or SDMMC_FLAG_DCRCFAIL or SDMMC_FLAG_CTIMEOUT or SDMMC_FLAG_DTIMEOUT or SDMMC_FLAG_TXUNDERR or SDMMC_FLAG_RXOVERR or SDMMC_FLAG_CMDREND or
    SDMMC_FLAG_CMDSENT or SDMMC_FLAG_DATAEND or SDMMC_FLAG_DBCKEND));
  SDMMC_CMD0TIMEOUT = ($00010000);
  (**
  * @brief  Mask for errors Card Status R1 (OCR Register)
   *)

  SD_OCR_ADDR_OUT_OF_RANGE = ($80000000);
  SD_OCR_ADDR_MISALIGNED = ($40000000);
  SD_OCR_BLOCK_LEN_ERR = ($20000000);
  SD_OCR_ERASE_SEQ_ERR = ($10000000);
  SD_OCR_BAD_ERASE_PARAM = ($08000000);
  SD_OCR_WRITE_PROT_VIOLATION = ($04000000);
  SD_OCR_LOCK_UNLOCK_FAILED = ($01000000);
  SD_OCR_COM_CRC_FAILED = ($00800000);
  SD_OCR_ILLEGAL_CMD = ($00400000);
  SD_OCR_CARD_ECC_FAILED = ($00200000);
  SD_OCR_CC_ERROR = ($00100000);
  SD_OCR_GENERAL_UNKNOWN_ERROR = ($00080000);
  SD_OCR_STREAM_READ_UNDERRUN = ($00040000);
  SD_OCR_STREAM_WRITE_OVERRUN = ($00020000);
  SD_OCR_CID_CSD_OVERWRITE = ($00010000);
  SD_OCR_WP_ERASE_SKIP = ($00008000);
  SD_OCR_CARD_ECC_DISABLED = ($00004000);
  SD_OCR_ERASE_RESET = ($00002000);
  SD_OCR_AKE_SEQ_ERROR = ($00000008);
  SD_OCR_ERRORBITS = ($FDFFE008);
  (**
  * @brief  Masks for R6 Response
   *)

  SD_R6_GENERAL_UNKNOWN_ERROR = ($00002000);
  SD_R6_ILLEGAL_CMD = ($00004000);
  SD_R6_COM_CRC_FAILED = ($00008000);
  SD_VOLTAGE_WINDOW_SD = ($80100000);
  SD_HIGH_CAPACITY = ($40000000);
  SD_STD_CAPACITY = ($00000000);
  SD_CHECK_PATTERN = ($000001AA);
  SD_MAX_VOLT_TRIAL = ($0000FFFF);
  SD_ALLZERO = ($00000000);
  SD_WIDE_BUS_SUPPORT = ($00040000);
  SD_SINGLE_BUS_SUPPORT = ($00010000);
  SD_CARD_LOCKED = ($02000000);
  SD_DATATIMEOUT = ($FFFFFFFF);
  SD_0TO7BITS = ($000000FF);
  SD_8TO15BITS = ($0000FF00);
  SD_16TO23BITS = ($00FF0000);
  SD_24TO31BITS = ($FF000000);
  SD_MAX_DATA_LENGTH = ($01FFFFFF);
  SD_HALFFIFO = ($00000008);
  SD_HALFFIFOBYTES = ($00000020);
  (**
  * @brief  Command Class Supported
   *)

  SD_CCCC_LOCK_UNLOCK = ($00000080);
  SD_CCCC_WRITE_PROT = ($00000040);
  SD_CCCC_ERASE = ($00000020);
  (**
  * @brief  Following commands are SD Card Specific commands.
  *         SDMMC_APP_CMD should be sent before sending these commands.
   *)

  SD_SDMMC_SEND_IF_COND = (SD_CMD_HS_SEND_EXT_CSD);

(**
  * @brief  Enable the SD device.
  * @retval None
  *)
procedure __HAL_SD_SDMMC_ENABLE(var __HANDLE__: SD_HandleTypeDef);
begin
  __SDMMC_ENABLE((__HANDLE__).Instance^);
end;

(**
  * @brief  Disable the SD device.
  * @retval None
  *)
procedure __HAL_SD_SDMMC_DISABLE(var __HANDLE__: SD_HandleTypeDef);
begin
  __SDMMC_DISABLE((__HANDLE__).Instance^);
end;

(**
  * @brief  Enable the SDMMC DMA transfer.
  * @retval None
  *)
procedure __HAL_SD_SDMMC_DMA_ENABLE(var __HANDLE__: SD_HandleTypeDef);
begin
  __SDMMC_DMA_ENABLE((__HANDLE__).Instance^);
end;

(**
  * @brief  Disable the SDMMC DMA transfer.
  * @retval None
  *)
procedure __HAL_SD_SDMMC_DMA_DISABLE(var __HANDLE__: SD_HandleTypeDef);
begin
  __SDMMC_DMA_DISABLE((__HANDLE__).Instance^);
end;

(**
  * @brief  Enable the SD device interrupt.
  * @param  __HANDLE__: SD Handle
  * @param  __INTERRUPT__: specifies the SDMMC interrupt sources to be enabled.
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
procedure __HAL_SD_SDMMC_ENABLE_IT(var __HANDLE__: SD_HandleTypeDef; __INTERRUPT__: longword);
begin
  __SDMMC_ENABLE_IT(__HANDLE__.Instance^, (__INTERRUPT__));
end;

(**
  * @brief  Disable the SD device interrupt.
  * @param  __HANDLE__: SD Handle
  * @param  __INTERRUPT__: specifies the SDMMC interrupt sources to be disabled.
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
procedure __HAL_SD_SDMMC_DISABLE_IT(var __HANDLE__: SD_HandleTypeDef; __INTERRUPT__: longword);
begin
  __SDMMC_DISABLE_IT((__HANDLE__).Instance^, (__INTERRUPT__));
end;

(**
  * @brief  Check whether the specified SD flag is set or not.
  * @param  __HANDLE__: SD Handle
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
  *            @arg SDMMC_FLAG_SDIOIT:   SD I/O interrupt received
  * @retval The new state of SD FLAG (SET or RESET).
  *)
function __HAL_SD_SDMMC_GET_FLAG(var __HANDLE__: SD_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit(__SDMMC_GET_FLAG((__HANDLE__).Instance^, (__FLAG__)));
end;

(**
  * @brief  Clear the SD's pending flags.
  * @param  __HANDLE__: SD Handle
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
  *            @arg SDMMC_FLAG_SDIOIT:   SD I/O interrupt received
  * @retval None
  *)
procedure __HAL_SD_SDMMC_CLEAR_FLAG(var __HANDLE__: SD_HandleTypeDef; __FLAG__: longword);
begin
  __SDMMC_CLEAR_FLAG((__HANDLE__).Instance^, (__FLAG__));
end;

(**
  * @brief  Check whether the specified SD interrupt has occurred or not.
  * @param  __HANDLE__: SD Handle
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
  * @retval The new state of SD IT (SET or RESET).
  *)
function __HAL_SD_SDMMC_GET_IT(var __HANDLE__: SD_HandleTypeDef; __INTERRUPT__: longword): boolean;
begin
  exit(__SDMMC_GET_IT((__HANDLE__).Instance^, (__INTERRUPT__)));
end;

(**
  * @brief  Clear the SD's interrupt pending bits.
  * @param  __HANDLE__: SD Handle
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
procedure __HAL_SD_SDMMC_CLEAR_IT(var __HANDLE__: SD_HandleTypeDef; __INTERRUPT__: longword);
begin
  __SDMMC_CLEAR_IT((__HANDLE__).Instance^, (__INTERRUPT__));
end;

function SD_CmdError(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  timeout: longword;
  tmp: boolean;
  errorstate: HAL_SD_ErrorTypedef;
begin
  errorstate:=SD_OK;

  timeout := SDMMC_CMD0TIMEOUT;

  tmp := __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CMDSENT);

  while ((timeout > 0) and (not tmp)) do
  begin
    tmp := __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CMDSENT);
    Dec(timeout);
  end;

  if (timeout = 0) then
  begin
    errorstate := SD_CMD_RSP_TIMEOUT;
    exit(errorstate);
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  exit(errorstate);
end;

function SD_CmdResp1Error(var hsd: SD_HandleTypeDef; SD_CMD: byte): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  response_r1: longword;
begin
  errorstate:=SD_OK;

  while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL or SDMMC_FLAG_CMDREND or SDMMC_FLAG_CTIMEOUT)) do
  begin
  end;

  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CTIMEOUT)) then
  begin
    errorstate := SD_CMD_RSP_TIMEOUT;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CTIMEOUT);

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL)) then
  begin
    errorstate := SD_CMD_CRC_FAIL;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CCRCFAIL);

    exit(errorstate);
  end;

  (* Check response received is of desired command *)
  if (SDMMC_GetCommandResponse(hsd.Instance^) <> SD_CMD) then
  begin
    errorstate := SD_ILLEGAL_CMD;

    exit(errorstate);
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  (* We have received response, retrieve it for analysis  *)
  response_r1 := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1);

  if ((response_r1 and SD_OCR_ERRORBITS) = SD_ALLZERO) then
    exit(errorstate);

  if ((response_r1 and SD_OCR_ADDR_OUT_OF_RANGE) = SD_OCR_ADDR_OUT_OF_RANGE) then
    exit((SD_ADDR_OUT_OF_RANGE));

  if ((response_r1 and SD_OCR_ADDR_MISALIGNED) = SD_OCR_ADDR_MISALIGNED) then
    exit((SD_ADDR_MISALIGNED));

  if ((response_r1 and SD_OCR_BLOCK_LEN_ERR) = SD_OCR_BLOCK_LEN_ERR) then
    exit((SD_BLOCK_LEN_ERR));

  if ((response_r1 and SD_OCR_ERASE_SEQ_ERR) = SD_OCR_ERASE_SEQ_ERR) then
    exit((SD_ERASE_SEQ_ERR));

  if ((response_r1 and SD_OCR_BAD_ERASE_PARAM) = SD_OCR_BAD_ERASE_PARAM) then
    exit((SD_BAD_ERASE_PARAM));

  if ((response_r1 and SD_OCR_WRITE_PROT_VIOLATION) = SD_OCR_WRITE_PROT_VIOLATION) then
    exit((SD_WRITE_PROT_VIOLATION));

  if ((response_r1 and SD_OCR_LOCK_UNLOCK_FAILED) = SD_OCR_LOCK_UNLOCK_FAILED) then
    exit((SD_LOCK_UNLOCK_FAILED));

  if ((response_r1 and SD_OCR_COM_CRC_FAILED) = SD_OCR_COM_CRC_FAILED) then
    exit((SD_COM_CRC_FAILED));

  if ((response_r1 and SD_OCR_ILLEGAL_CMD) = SD_OCR_ILLEGAL_CMD) then
    exit((SD_ILLEGAL_CMD));

  if ((response_r1 and SD_OCR_CARD_ECC_FAILED) = SD_OCR_CARD_ECC_FAILED) then
    exit((SD_CARD_ECC_FAILED));

  if ((response_r1 and SD_OCR_CC_ERROR) = SD_OCR_CC_ERROR) then
    exit((SD_CC_ERROR));

  if ((response_r1 and SD_OCR_GENERAL_UNKNOWN_ERROR) = SD_OCR_GENERAL_UNKNOWN_ERROR) then
    exit((SD_GENERAL_UNKNOWN_ERROR));

  if ((response_r1 and SD_OCR_STREAM_READ_UNDERRUN) = SD_OCR_STREAM_READ_UNDERRUN) then
    exit((SD_STREAM_READ_UNDERRUN));

  if ((response_r1 and SD_OCR_STREAM_WRITE_OVERRUN) = SD_OCR_STREAM_WRITE_OVERRUN) then
    exit((SD_STREAM_WRITE_OVERRUN));

  if ((response_r1 and SD_OCR_CID_CSD_OVERWRITE) = SD_OCR_CID_CSD_OVERWRITE) then
    exit((SD_CID_CSD_OVERWRITE));

  if ((response_r1 and SD_OCR_WP_ERASE_SKIP) = SD_OCR_WP_ERASE_SKIP) then
    exit((SD_WP_ERASE_SKIP));

  if ((response_r1 and SD_OCR_CARD_ECC_DISABLED) = SD_OCR_CARD_ECC_DISABLED) then
    exit((SD_CARD_ECC_DISABLED));

  if ((response_r1 and SD_OCR_ERASE_RESET) = SD_OCR_ERASE_RESET) then
    exit((SD_ERASE_RESET));

  if ((response_r1 and SD_OCR_AKE_SEQ_ERROR) = SD_OCR_AKE_SEQ_ERROR) then
    exit((SD_AKE_SEQ_ERROR));

  exit(errorstate);
end;

function SD_CmdResp7Error(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  tmp: boolean;
  errorstate: HAL_SD_ErrorTypedef;
  timeout: integer;
begin
  errorstate:=SD_OK;

  timeout := SDMMC_CMD0TIMEOUT;

  tmp := __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL or SDMMC_FLAG_CMDREND or SDMMC_FLAG_CTIMEOUT);

  while ((not tmp) and (timeout > 0)) do
  begin
    tmp := __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL or SDMMC_FLAG_CMDREND or SDMMC_FLAG_CTIMEOUT);
    Dec(timeout);
  end;

  tmp := __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CTIMEOUT);

  if ((timeout = 0) or tmp) then
  begin
    (* Card is not V2.0 compliant or card does not support the set voltage range *)
    errorstate := SD_CMD_RSP_TIMEOUT;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CTIMEOUT);

    exit(errorstate);
  end;

  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CMDREND)) then
  begin
    (* Card is SD V2.0 compliant *)
    errorstate := SD_OK;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CMDREND);

    exit(errorstate);
  end;

  exit(errorstate);

end;

function SD_CmdResp3Error(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
begin
  errorstate:=SD_OK;

  while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL or SDMMC_FLAG_CMDREND or SDMMC_FLAG_CTIMEOUT)) do ;

  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CTIMEOUT)) then
  begin
    errorstate := SD_CMD_RSP_TIMEOUT;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CTIMEOUT);

    exit(errorstate);
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  exit(errorstate);
end;

function SD_CmdResp2Error(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
begin
  errorstate:=SD_OK;

  while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL or SDMMC_FLAG_CMDREND or SDMMC_FLAG_CTIMEOUT)) do ;

  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CTIMEOUT)) then
  begin
    errorstate := SD_CMD_RSP_TIMEOUT;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CTIMEOUT);

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL)) then
  begin
    errorstate := SD_CMD_CRC_FAIL;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CCRCFAIL);

    exit(errorstate);
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  exit(errorstate);
end;

function SD_CmdResp6Error(var hsd: SD_HandleTypeDef; SD_CMD: byte; var pRCA: word): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  response_r1: longword;
begin
  errorstate:=SD_OK;

  while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL or SDMMC_FLAG_CMDREND or SDMMC_FLAG_CTIMEOUT)) do ;

  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CTIMEOUT)) then
  begin
    errorstate := SD_CMD_RSP_TIMEOUT;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CTIMEOUT);

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL)) then
  begin
    errorstate := SD_CMD_CRC_FAIL;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CCRCFAIL);

    exit(errorstate);
  end;

  (* Check response received is of desired command *)
  if (SDMMC_GetCommandResponse(hsd.Instance^) <> SD_CMD) then
  begin
    errorstate := SD_ILLEGAL_CMD;

    exit(errorstate);
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  (* We have received response, retrieve it.  *)
  response_r1 := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1);

  if ((response_r1 and (SD_R6_GENERAL_UNKNOWN_ERROR or SD_R6_ILLEGAL_CMD or SD_R6_COM_CRC_FAILED)) = SD_ALLZERO) then
  begin
    pRCA := (response_r1 shr 16);

    exit(errorstate);
  end;

  if ((response_r1 and SD_R6_GENERAL_UNKNOWN_ERROR) = SD_R6_GENERAL_UNKNOWN_ERROR) then
    exit((SD_GENERAL_UNKNOWN_ERROR));

  if ((response_r1 and SD_R6_ILLEGAL_CMD) = SD_R6_ILLEGAL_CMD) then
    exit((SD_ILLEGAL_CMD));

  if ((response_r1 and SD_R6_COM_CRC_FAILED) = SD_R6_COM_CRC_FAILED) then
    exit((SD_COM_CRC_FAILED));

  exit(errorstate);

end;

function SD_Initialize_Cards(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
  sd_rca: word;
begin
  errorstate:=SD_OK;

  if (SDMMC_GetPowerState(hsd.Instance^) = 0) (* Power off *) then
  begin
    errorstate := SD_REQUEST_NOT_APPLICABLE;

    exit(errorstate);
  end;

  if (hsd.CardType <> SECURE_DIGITAL_IO_CARD) then
  begin
    (* Send CMD2 ALL_SEND_CID *)
    sdmmc_cmdinitstructure.Argument := 0;
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_ALL_SEND_CID;
    sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_LONG;
    sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
    sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp2Error(hsd);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    (* Get Card identification number data *)
    hsd.CID[0] := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1);
    hsd.CID[1] := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP2);
    hsd.CID[2] := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP3);
    hsd.CID[3] := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP4);
  end;

  if ((hsd.CardType = STD_CAPACITY_SD_CARD_V1_1) or (hsd.CardType = STD_CAPACITY_SD_CARD_V2_0) or (hsd.CardType = SECURE_DIGITAL_IO_COMBO_CARD) or (hsd.CardType = HIGH_CAPACITY_SD_CARD)) then
  begin
    (* Send CMD3 SET_REL_ADDR with argument 0 *)
    (* SD Card publishes its RCA. *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SET_REL_ADDR;
    sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp6Error(hsd, SD_CMD_SET_REL_ADDR, sd_rca);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;
  end;

  if (hsd.CardType <> SECURE_DIGITAL_IO_CARD) then
  begin
    (* Get the SD card RCA *)
    hsd.RCA := sd_rca;

    (* Send CMD9 SEND_CSD with argument as card's RCA *)
    sdmmc_cmdinitstructure.Argument := (hsd.RCA shl 16);
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SEND_CSD;
    sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_LONG;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp2Error(hsd);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    (* Get Card Specific Data *)
    hsd.CSD[0] := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1);
    hsd.CSD[1] := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP2);
    hsd.CSD[2] := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP3);
    hsd.CSD[3] := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP4);
  end;

  (* All cards are initialized *)
  exit(errorstate);
end;

function SD_Select_Deselect(var hsd: SD_HandleTypeDef; addr: qword): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
begin
  (* Send CMD7 SDMMC_SEL_DESEL_CARD *)
  sdmmc_cmdinitstructure.Argument := addr;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SEL_DESEL_CARD;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SEL_DESEL_CARD);

  exit(errorstate);
end;

function SD_PowerON(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
  Count, sdtype: longword;
  validvoltage: boolean;
  response: longword;
begin
  sdtype := SD_STD_CAPACITY;
  validvoltage := False;
  Count := 0;

  (* Power ON Sequence -------------------------------------------------------*)
  (* Disable SDMMC Clock *)
  __HAL_SD_SDMMC_DISABLE(hsd);

  (* Set Power State to ON *)
  SDMMC_PowerState_ON(hsd.Instance^);

  (* 1ms: required power up waiting time before starting the SD initialization
     sequence *)
  HAL_Delay(1);

  (* Enable SDMMC Clock *)
  __HAL_SD_SDMMC_ENABLE(hsd);

  (* CMD0: GO_IDLE_STATE -----------------------------------------------------*)
  (* No CMD response required *)
  sdmmc_cmdinitstructure.Argument := 0;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_GO_IDLE_STATE;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_NO;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdError(hsd);

  if (errorstate <> SD_OK) then
  begin
    (* CMD Response Timeout (wait for CMDSENT flag) *)
    exit(errorstate);
  end;

  (* CMD8: SEND_IF_COND ------------------------------------------------------*)
  (* Send CMD8 to verify SD card interface operating condition *)
  (* Argument: - [31:12]: Reserved (shall be set to '0')
  - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
  - [7:0]: Check Pattern (recommended 0xAA) *)
  (* CMD Response: R7 *)
  sdmmc_cmdinitstructure.Argument := SD_CHECK_PATTERN;
  sdmmc_cmdinitstructure.CmdIndex := SD_SDMMC_SEND_IF_COND;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp7Error(hsd);

  if (errorstate = SD_OK) then
  begin
    (* SD Card 2.0 *)
    hsd.CardType := STD_CAPACITY_SD_CARD_V2_0;
    sdtype := SD_HIGH_CAPACITY;
  end;

  (* Send CMD55 *)
  sdmmc_cmdinitstructure.Argument := 0;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_APP_CMD;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_APP_CMD);

  (* If errorstate is Command Timeout, it is a MMC card *)
  (* If errorstate is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
     or SD card 1.x *)
  if (errorstate = SD_OK) then
  begin
    (* SD CARD *)
    (* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 *)
    while ((not validvoltage) and (Count < SD_MAX_VOLT_TRIAL)) do
    begin

      (* SEND CMD55 APP_CMD with RCA as 0 *)
      sdmmc_cmdinitstructure.Argument := 0;
      sdmmc_cmdinitstructure.CmdIndex := SD_CMD_APP_CMD;
      sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
      sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
      sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
      SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

      (* Check for error conditions *)
      errorstate := SD_CmdResp1Error(hsd, SD_CMD_APP_CMD);

      if (errorstate <> SD_OK) then
      begin
        exit(errorstate);
      end;

      (* Send CMD41 *)
      sdmmc_cmdinitstructure.Argument := SD_VOLTAGE_WINDOW_SD or sdtype;
      sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SD_APP_OP_COND;
      sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
      sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
      sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
      SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

      (* Check for error conditions *)
      errorstate := SD_CmdResp3Error(hsd);

      if (errorstate <> SD_OK) then
      begin
        exit(errorstate);
      end;

      (* Get command response *)
      response := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1);

      (* Get operating voltage*)
      validvoltage := ((response shr 31) = 1);

      Inc(Count);
    end;

    if (Count >= SD_MAX_VOLT_TRIAL) then
    begin
      errorstate := SD_INVALID_VOLTRANGE;

      exit(errorstate);
    end;

    if ((response and SD_HIGH_CAPACITY) = SD_HIGH_CAPACITY) (* (response  and := SD_HIGH_CAPACITY) *) then
    begin
      hsd.CardType := HIGH_CAPACITY_SD_CARD;
    end;

  end (* else MMC Card *);

  exit(errorstate);

end;

function SD_PowerOFF(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
begin
  (* Set Power State to OFF *)
  SDMMC_PowerState_OFF(hsd.Instance^);

  exit(SD_OK);
end;

function SD_SendStatus(var hsd: SD_HandleTypeDef; out pCardStatus: longword): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
begin
  (* Send Status command *)
  sdmmc_cmdinitstructure.Argument := (hsd.RCA shl 16);
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SEND_STATUS;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SEND_STATUS);

  if (errorstate <> SD_OK) then
    begin
      pCardStatus:=0;
      exit(errorstate);
    end;

  (* Get SD card status *)
  pCardStatus := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1);

  exit(errorstate);
end;

function SD_GetState(var hsd: SD_HandleTypeDef): HAL_SD_CardStateTypedef;
var
  resp1: longword;
begin
  if (SD_SendStatus(hsd, resp1) <> SD_OK) then
    exit(SD_CARD_ERROR)
  else
    exit(HAL_SD_CardStateTypedef((resp1 shr 9) and $0F));
end;

function SD_IsCardProgramming(var hsd: SD_HandleTypeDef; var pStatus: HAL_SD_CardStateTypedef): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  responseR1: longword;
begin
  errorstate:=SD_OK;

  sdmmc_cmdinitstructure.Argument := (hsd.RCA shl 16);
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SEND_STATUS;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL or SDMMC_FLAG_CMDREND or SDMMC_FLAG_CTIMEOUT)) do
  begin
  end;

  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CTIMEOUT)) then
  begin
    errorstate := SD_CMD_RSP_TIMEOUT;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CTIMEOUT);

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_CCRCFAIL)) then
  begin
    errorstate := SD_CMD_CRC_FAIL;

    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_CCRCFAIL);

    exit(errorstate);
  end
  else
  begin
    (* No error flag set *)
  end;

  (* Check response received is of desired command *)
  if (SDMMC_GetCommandResponse(hsd.Instance^) <> SD_CMD_SEND_STATUS) then
  begin
    errorstate := SD_ILLEGAL_CMD;

    exit(errorstate);
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);


  (* We have received response, retrieve it for analysis *)
  responseR1 := SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1);

  (* Find out card status *)
  pStatus := HAL_SD_CardStateTypedef((responseR1 shr 9) and $0000000F);

  if ((responseR1 and SD_OCR_ERRORBITS) = SD_ALLZERO) then
  begin
    exit(errorstate);
  end;

  if ((responseR1 and SD_OCR_ADDR_OUT_OF_RANGE) = SD_OCR_ADDR_OUT_OF_RANGE) then
    exit((SD_ADDR_OUT_OF_RANGE));

  if ((responseR1 and SD_OCR_ADDR_MISALIGNED) = SD_OCR_ADDR_MISALIGNED) then
    exit((SD_ADDR_MISALIGNED));

  if ((responseR1 and SD_OCR_BLOCK_LEN_ERR) = SD_OCR_BLOCK_LEN_ERR) then
    exit((SD_BLOCK_LEN_ERR));

  if ((responseR1 and SD_OCR_ERASE_SEQ_ERR) = SD_OCR_ERASE_SEQ_ERR) then
    exit((SD_ERASE_SEQ_ERR));

  if ((responseR1 and SD_OCR_BAD_ERASE_PARAM) = SD_OCR_BAD_ERASE_PARAM) then
    exit((SD_BAD_ERASE_PARAM));

  if ((responseR1 and SD_OCR_WRITE_PROT_VIOLATION) = SD_OCR_WRITE_PROT_VIOLATION) then
    exit((SD_WRITE_PROT_VIOLATION));

  if ((responseR1 and SD_OCR_LOCK_UNLOCK_FAILED) = SD_OCR_LOCK_UNLOCK_FAILED) then
    exit((SD_LOCK_UNLOCK_FAILED));

  if ((responseR1 and SD_OCR_COM_CRC_FAILED) = SD_OCR_COM_CRC_FAILED) then
    exit((SD_COM_CRC_FAILED));

  if ((responseR1 and SD_OCR_ILLEGAL_CMD) = SD_OCR_ILLEGAL_CMD) then
    exit((SD_ILLEGAL_CMD));

  if ((responseR1 and SD_OCR_CARD_ECC_FAILED) = SD_OCR_CARD_ECC_FAILED) then
    exit((SD_CARD_ECC_FAILED));

  if ((responseR1 and SD_OCR_CC_ERROR) = SD_OCR_CC_ERROR) then
    exit((SD_CC_ERROR));

  if ((responseR1 and SD_OCR_GENERAL_UNKNOWN_ERROR) = SD_OCR_GENERAL_UNKNOWN_ERROR) then
    exit((SD_GENERAL_UNKNOWN_ERROR));

  if ((responseR1 and SD_OCR_STREAM_READ_UNDERRUN) = SD_OCR_STREAM_READ_UNDERRUN) then
    exit((SD_STREAM_READ_UNDERRUN));

  if ((responseR1 and SD_OCR_STREAM_WRITE_OVERRUN) = SD_OCR_STREAM_WRITE_OVERRUN) then
    exit((SD_STREAM_WRITE_OVERRUN));

  if ((responseR1 and SD_OCR_CID_CSD_OVERWRITE) = SD_OCR_CID_CSD_OVERWRITE) then
    exit((SD_CID_CSD_OVERWRITE));

  if ((responseR1 and SD_OCR_WP_ERASE_SKIP) = SD_OCR_WP_ERASE_SKIP) then
    exit((SD_WP_ERASE_SKIP));

  if ((responseR1 and SD_OCR_CARD_ECC_DISABLED) = SD_OCR_CARD_ECC_DISABLED) then
    exit((SD_CARD_ECC_DISABLED));

  if ((responseR1 and SD_OCR_ERASE_RESET) = SD_OCR_ERASE_RESET) then
    exit((SD_ERASE_RESET));

  if ((responseR1 and SD_OCR_AKE_SEQ_ERROR) = SD_OCR_AKE_SEQ_ERROR) then
    exit((SD_AKE_SEQ_ERROR));

  exit(errorstate);
end;

function SD_FindSCR(var hsd: SD_HandleTypeDef; out pSCR): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  sdmmc_datainitstructure: SDMMC_DataInitTypeDef;
  tempscr: array[0..1] of longword;
  index: longword;
  errorstate: HAL_SD_ErrorTypedef;
begin
  index := 0;

  (* Set Block Size To 8 Bytes *)
  (* Send CMD55 APP_CMD with argument as card's RCA *)
  sdmmc_cmdinitstructure.Argument := 8;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SET_BLOCKLEN;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SET_BLOCKLEN);

  if (errorstate <> SD_OK) then
    exit(errorstate);

  (* Send CMD55 APP_CMD with argument as card's RCA *)
  sdmmc_cmdinitstructure.Argument := ((hsd.RCA) shl 16);
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_APP_CMD;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_APP_CMD);

  if (errorstate <> SD_OK) then
    exit(errorstate);

  sdmmc_datainitstructure.DataTimeOut := SD_DATATIMEOUT;
  sdmmc_datainitstructure.DataLength := 8;
  sdmmc_datainitstructure.DataBlockSize := SDMMC_DATABLOCK_SIZE_8B;
  sdmmc_datainitstructure.TransferDir := SDMMC_TRANSFER_DIR_TO_SDMMC;
  sdmmc_datainitstructure.TransferMode := SDMMC_TRANSFER_MODE_BLOCK;
  sdmmc_datainitstructure.DPSM := SDMMC_DPSM_ENABLE;
  SDMMC_DataConfig(hsd.Instance^, sdmmc_datainitstructure);

  (* Send ACMD51 SD_APP_SEND_SCR with argument as 0 *)
  sdmmc_cmdinitstructure.Argument := 0;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SD_APP_SEND_SCR;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SD_APP_SEND_SCR);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR or SDMMC_FLAG_DCRCFAIL or SDMMC_FLAG_DTIMEOUT or SDMMC_FLAG_DBCKEND)) do
  begin
    if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXDAVL)) then
    begin
      tempscr[index] := SDMMC_ReadFIFO(hsd.Instance^);
      Inc(index);
    end;
  end;

  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DTIMEOUT)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DTIMEOUT);

    errorstate := SD_DATA_TIMEOUT;

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DCRCFAIL);

    errorstate := SD_DATA_CRC_FAIL;

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_RXOVERR);

    errorstate := SD_RX_OVERRUN;

    exit(errorstate);
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  pbyte(@pSCR)[1] := ((tempscr[0] and SD_0TO7BITS) shl 24) or ((tempscr[0] and SD_8TO15BITS) shl 8) or ((tempscr[0] and SD_16TO23BITS) shr 8) or ((tempscr[0] and SD_24TO31BITS) shr 24);
  pbyte(@pSCR)^ := ((tempscr[1] and SD_0TO7BITS) shl 24) or ((tempscr[1] and SD_8TO15BITS) shl 8) or ((tempscr[1] and SD_16TO23BITS) shr 8) or ((tempscr[1] and SD_24TO31BITS) shr 24);

  exit(errorstate);
end;

function SD_WideBus_Enable(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  scr: array[0..1] of byte;
begin
  if ((SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1) and SD_CARD_LOCKED) = SD_CARD_LOCKED) then
  begin
    errorstate := SD_LOCK_UNLOCK_FAILED;

    exit(errorstate);
  end;

  (* Get SCR Register *)
  errorstate := SD_FindSCR(hsd, scr[0]);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* If requested card supports wide bus operation *)
  if ((scr[1] and SD_WIDE_BUS_SUPPORT) <> SD_ALLZERO) then
  begin
    (* Send CMD55 APP_CMD with argument as card's RCA.*)
    sdmmc_cmdinitstructure.Argument := (hsd.RCA shl 16);
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_APP_CMD;
    sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
    sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
    sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_APP_CMD);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    (* Send ACMD6 APP_CMD with argument as 2 for wide bus mode *)
    sdmmc_cmdinitstructure.Argument := 2;
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_APP_SD_SET_BUSWIDTH;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_APP_SD_SET_BUSWIDTH);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    exit(errorstate);
  end
  else
  begin
    errorstate := SD_REQUEST_NOT_APPLICABLE;

    exit(errorstate);
  end;
end;

function SD_WideBus_Disable(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  scr: array[0..1] of byte;
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
begin
  if ((SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1) and SD_CARD_LOCKED) = SD_CARD_LOCKED) then
  begin
    errorstate := SD_LOCK_UNLOCK_FAILED;

    exit(errorstate);
  end;

  (* Get SCR Register *)
  errorstate := SD_FindSCR(hsd, scr[0]);

  if (errorstate <> SD_OK) then
    exit(errorstate);

  (* If requested card supports 1 bit mode operation *)
  if ((scr[1] and SD_SINGLE_BUS_SUPPORT) <> SD_ALLZERO) then
  begin
    (* Send CMD55 APP_CMD with argument as card's RCA *)
    sdmmc_cmdinitstructure.Argument := (hsd.RCA shl 16);
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_APP_CMD;
    sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
    sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
    sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_APP_CMD);

    if (errorstate <> SD_OK) then
      exit(errorstate);

    (* Send ACMD6 APP_CMD with argument as 0 for single bus mode *)
    sdmmc_cmdinitstructure.Argument := 0;
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_APP_SD_SET_BUSWIDTH;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_APP_SD_SET_BUSWIDTH);

    if (errorstate <> SD_OK) then
      exit(errorstate);

    exit(errorstate);
  end
  else
  begin
    errorstate := SD_REQUEST_NOT_APPLICABLE;

    exit(errorstate);
  end;
end;

procedure SD_DMA_RxCplt(var hdma: DMA_HandleTypeDef);
var
  hsd: PSD_HandleTypeDef;
begin
  hsd := PSD_HandleTypeDef(hdma.Parent);

  (* DMA transfer is complete *)
  hsd^.DmaTransferCplt := 1;

  (* Wait until SD transfer is complete *)
  while (hsd^.SdTransferCplt = 0) do ;

  (* Disable the DMA channel *)
  HAL_DMA_Abort(hdma);

  (* Transfer complete user callback *)
  HAL_SD_DMA_RxCpltCallback(hsd^.hdmarx^);
end;

procedure SD_DMA_RxError(var hdma: DMA_HandleTypeDef);
var
  hsd: PSD_HandleTypeDef;
begin
  hsd := PSD_HandleTypeDef(hdma.Parent);

  (* Transfer complete user callback *)
  HAL_SD_DMA_RxErrorCallback(hsd^.hdmarx^);
end;

procedure SD_DMA_TxCplt(var hdma: DMA_HandleTypeDef);
var
  hsd: PSD_HandleTypeDef;
begin
  hsd := PSD_HandleTypeDef(hdma.Parent);

  (* DMA transfer is complete *)
  hsd^.DmaTransferCplt := 1;

  (* Wait until SD transfer is complete *)
  while (hsd^.SdTransferCplt = 0) do ;

  (* Disable the DMA channel *)
  HAL_DMA_Abort(hdma);

  (* Transfer complete user callback *)
  HAL_SD_DMA_TxCpltCallback(hsd^.hdmatx^);
end;

procedure SD_DMA_TxError(var hdma: DMA_HandleTypeDef);
var
  hsd: PSD_HandleTypeDef;
begin
  hsd := PSD_HandleTypeDef(hdma.Parent);

  (* Transfer complete user callback *)
  HAL_SD_DMA_TxErrorCallback(hsd^.hdmatx^);
end;


function HAL_SD_Init(var hsd: SD_HandleTypeDef; var SDCardInfo: HAL_SD_CardInfoTypedef): HAL_SD_ErrorTypedef;
var
  tmpinit: SDMMC_InitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
begin
  (* Allocate lock resource and initialize it *)
  hsd.Lock := HAL_UNLOCKED;

  (* Initialize the low level hardware (MSP) *)
  HAL_SD_MspInit(hsd);

  (* Default SDMMC peripheral configuration for SD card initialization *)
  tmpinit.ClockEdge := SDMMC_CLOCK_EDGE_RISING;
  tmpinit.ClockBypass := SDMMC_CLOCK_BYPASS_DISABLE;
  tmpinit.ClockPowerSave := SDMMC_CLOCK_POWER_SAVE_DISABLE;
  tmpinit.BusWide := SDMMC_BUS_WIDE_1B;
  tmpinit.HardwareFlowControl := SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  tmpinit.ClockDiv := SDMMC_INIT_CLK_DIV;

  (* Initialize SDMMC peripheral interface with default configuration *)
  SDMMC_Init(hsd.Instance^, tmpinit);

  (* Identify card operating voltage *)
  errorstate := SD_PowerON(hsd);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Initialize the present SDMMC card(s) and put them in idle state *)
  errorstate := SD_Initialize_Cards(hsd);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Read CSD/CID MSD registers *)
  errorstate := HAL_SD_Get_CardInfo(hsd, SDCardInfo);

  if (errorstate = SD_OK) then
  begin
    (* Select the Card *)
    errorstate := SD_Select_Deselect(hsd, ((SDCardInfo.RCA) shl 16));
  end;

  (* Configure SDMMC peripheral interface *)
  SDMMC_Init(hsd.Instance^, hsd.Init);

  exit(errorstate);
end;

function HAL_SD_DeInit(var hsd: SD_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Set SD power state to off *)
  SD_PowerOFF(hsd);

  (* De-Initialize the MSP layer *)
  HAL_SD_MspDeInit(hsd);

  exit(HAL_OK);
end;

procedure HAL_SD_MspInit_stub(var hsd: SD_HandleTypeDef); assembler; nostackframe; public name 'HAL_SD_MspInit';
asm
  .weak HAL_SD_MspInit
end;

procedure HAL_SD_MspDeInit_stub(var hsd: SD_HandleTypeDef); assembler; nostackframe; public name 'HAL_SD_MspDeInit';
asm
  .weak HAL_SD_MspDeInit
end;

function HAL_SD_ReadBlocks(var hsd: SD_HandleTypeDef; var pReadBuffer; ReadAddr: qword; BlockSize, NumberOfBlocks: longword): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
  sdmmc_datainitstructure: SDMMC_DataInitTypeDef;
  tempbuff: plongword;
  Count: longword;
begin
  tempbuff := @pReadBuffer;

  (* Initialize data control register *)
  hsd.Instance^.DCTRL := 0;

  if (hsd.CardType = HIGH_CAPACITY_SD_CARD) then
  begin
    BlockSize := 512;
    ReadAddr := ReadAddr div 512;
  end;

  (* Set Block Size for Card *)
  sdmmc_cmdinitstructure.Argument := BlockSize;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SET_BLOCKLEN;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SET_BLOCKLEN);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Configure the SD DPSM (Data Path State Machine) *)
  sdmmc_datainitstructure.DataTimeOut := SD_DATATIMEOUT;
  sdmmc_datainitstructure.DataLength := NumberOfBlocks * BlockSize;
  sdmmc_datainitstructure.DataBlockSize := DATA_BLOCK_SIZE;
  sdmmc_datainitstructure.TransferDir := SDMMC_TRANSFER_DIR_TO_SDMMC;
  sdmmc_datainitstructure.TransferMode := SDMMC_TRANSFER_MODE_BLOCK;
  sdmmc_datainitstructure.DPSM := SDMMC_DPSM_ENABLE;
  SDMMC_DataConfig(hsd.Instance^, sdmmc_datainitstructure);

  if (NumberOfBlocks > 1) then
  begin
    (* Send CMD18 READ_MULT_BLOCK with argument data address *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_READ_MULT_BLOCK;
  end
  else
  begin
    (* Send CMD17 READ_SINGLE_BLOCK *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_READ_SINGLE_BLOCK;
  end;

  sdmmc_cmdinitstructure.Argument := ReadAddr;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Read block(s) in polling mode *)
  if (NumberOfBlocks > 1) then
  begin
    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_READ_MULT_BLOCK);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    (* Poll on SDMMC flags *)
    while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR or SDMMC_FLAG_DCRCFAIL or SDMMC_FLAG_DTIMEOUT or SDMMC_FLAG_DATAEND)) do
    begin
      if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXFIFOHF)) then
      begin
        (* Read data from SDMMC Rx FIFO *)
        for Count := 0 to 8 - 1 do
          tempbuff[Count] := SDMMC_ReadFIFO(hsd.Instance^);

        Inc(tempbuff, 8);
      end;
    end;
  end
  else
  begin
    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_READ_SINGLE_BLOCK);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    (* In case of single block transfer, no need of stop transfer at all *)
    while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR or SDMMC_FLAG_DCRCFAIL or SDMMC_FLAG_DTIMEOUT or SDMMC_FLAG_DBCKEND)) do
    begin
      if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXFIFOHF)) then
      begin
        (* Read data from SDMMC Rx FIFO *)
        for Count := 0 to 8 - 1 do
          tempbuff[Count] := SDMMC_ReadFIFO(hsd.Instance^);

        Inc(tempbuff, 8);
      end;
    end;
  end;

  (* Send stop transmission command in case of multiblock read *)
  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DATAEND) and (NumberOfBlocks > 1)) then
  begin
    if ((hsd.CardType = STD_CAPACITY_SD_CARD_V1_1) or (hsd.CardType = STD_CAPACITY_SD_CARD_V2_0) or (hsd.CardType = HIGH_CAPACITY_SD_CARD)) then
    begin
      (* Send stop transmission command *)
      errorstate := HAL_SD_StopTransfer(hsd);
    end;
  end;

  (* Get error state *)
  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DTIMEOUT)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DTIMEOUT);

    errorstate := SD_DATA_TIMEOUT;

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DCRCFAIL);

    errorstate := SD_DATA_CRC_FAIL;

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_RXOVERR);

    errorstate := SD_RX_OVERRUN;

    exit(errorstate);
  end
  else
  begin
    (* No error flag set *)
  end;

  Count := SD_DATATIMEOUT;

  (* Empty FIFO if there is still any data *)
  while ((__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXDAVL)) and (Count > 0)) do
  begin
    tempbuff^ := SDMMC_ReadFIFO(hsd.Instance^);
    Inc(tempbuff);
    Dec(Count);
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  exit(errorstate);
end;

function HAL_SD_WriteBlocks(var hsd: SD_HandleTypeDef; const pWriteBuffer; WriteAddr: qword; BlockSize, NumberOfBlocks: longword): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
  sdmmc_datainitstructure: SDMMC_DataInitTypeDef;
  totalnumberofbytes, bytestransferred, restwords, Count: longword;
  tempbuff: PLongWord;
  cardstate: HAL_SD_CardStateTypedef;
begin
  tempbuff := @pwritebuffer;

  (* Initialize data control register *)
  hsd.Instance^.DCTRL := 0;

  if (hsd.CardType = HIGH_CAPACITY_SD_CARD) then
  begin
    BlockSize := 512;
    WriteAddr := WriteAddr div 512;
  end;

  (* Set Block Size for Card *)
  sdmmc_cmdinitstructure.Argument := BlockSize;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SET_BLOCKLEN;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SET_BLOCKLEN);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  if (NumberOfBlocks > 1) then
  begin
    (* Send CMD25 WRITE_MULT_BLOCK with argument data address *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_WRITE_MULT_BLOCK;
  end
  else
  begin
    (* Send CMD24 WRITE_SINGLE_BLOCK *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_WRITE_SINGLE_BLOCK;
  end;

  sdmmc_cmdinitstructure.Argument := WriteAddr;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  if (NumberOfBlocks > 1) then
  begin
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_WRITE_MULT_BLOCK);
  end
  else
  begin
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_WRITE_SINGLE_BLOCK);
  end;

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Set total number of bytes to write *)
  totalnumberofbytes := NumberOfBlocks * BlockSize;
  bytestransferred := 0;

  (* Configure the SD DPSM (Data Path State Machine) *)
  sdmmc_datainitstructure.DataTimeOut := SD_DATATIMEOUT;
  sdmmc_datainitstructure.DataLength := NumberOfBlocks * BlockSize;
  sdmmc_datainitstructure.DataBlockSize := SDMMC_DATABLOCK_SIZE_512B;
  sdmmc_datainitstructure.TransferDir := SDMMC_TRANSFER_DIR_TO_CARD;
  sdmmc_datainitstructure.TransferMode := SDMMC_TRANSFER_MODE_BLOCK;
  sdmmc_datainitstructure.DPSM := SDMMC_DPSM_ENABLE;
  SDMMC_DataConfig(hsd.Instance^, sdmmc_datainitstructure);

  (* Write block(s) in polling mode *)
  if (NumberOfBlocks > 1) then
  begin
    while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_TXUNDERR or SDMMC_FLAG_DCRCFAIL or SDMMC_FLAG_DTIMEOUT or SDMMC_FLAG_DATAEND)) do
    begin
      if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_TXFIFOHE)) then
      begin
        if ((totalnumberofbytes - bytestransferred) < 32) then
        begin
          if (((totalnumberofbytes - bytestransferred) and 3) = 0) then
            restwords := ((totalnumberofbytes - bytestransferred) shr 2)
          else
            restwords := (((totalnumberofbytes - bytestransferred) shr 2) + 1);

          (* Write data to SDMMC Tx FIFO *)
          for Count := 0 to restwords - 1 do
          begin
            SDMMC_WriteFIFO(hsd.Instance^, tempbuff^);
            Inc(tempbuff);
            bytestransferred := bytestransferred + (4);
          end;
        end
        else
        begin
          (* Write data to SDMMC Tx FIFO *)
          for Count := 0 to 8 - 1 do
            SDMMC_WriteFIFO(hsd.Instance^, tempbuff[Count]);

          tempbuff := tempbuff + (8);
          bytestransferred := bytestransferred + (32);
        end;
      end;
    end;
  end
  else
  begin
    (* In case of single data block transfer no need of stop command at all *)
    while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_TXUNDERR or SDMMC_FLAG_DCRCFAIL or SDMMC_FLAG_DTIMEOUT or SDMMC_FLAG_DBCKEND)) do
    begin
      if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_TXFIFOHE)) then
      begin
        if ((totalnumberofbytes - bytestransferred) < 32) then
        begin
          if (((totalnumberofbytes - bytestransferred) and 3) = 0) then
            restwords := ((totalnumberofbytes - bytestransferred) shr 2)
          else
            restwords := (((totalnumberofbytes - bytestransferred) shr 2) + 1);

          (* Write data to SDMMC Tx FIFO *)
          for Count := 0 to restwords - 1 do
          begin
            SDMMC_WriteFIFO(hsd.Instance^, tempbuff^);
            Inc(tempbuff);
            bytestransferred := bytestransferred + (4);
          end;
        end
        else
        begin
          (* Write data to SDMMC Tx FIFO *)
          for Count := 0 to 8 - 1 do
            SDMMC_WriteFIFO(hsd.Instance^, tempbuff[Count]);

          tempbuff := tempbuff + (8);
          bytestransferred := bytestransferred + (32);
        end;
      end;
    end;
  end;

  (* Send stop transmission command in case of multiblock write *)
  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DATAEND) and (NumberOfBlocks > 1)) then
  begin
    if ((hsd.CardType = STD_CAPACITY_SD_CARD_V1_1) or (hsd.CardType = STD_CAPACITY_SD_CARD_V2_0) or (hsd.CardType = HIGH_CAPACITY_SD_CARD)) then
    begin
      (* Send stop transmission command *)
      errorstate := HAL_SD_StopTransfer(hsd);
    end;
  end;

  (* Get error state *)
  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DTIMEOUT)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DTIMEOUT);

    errorstate := SD_DATA_TIMEOUT;

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DCRCFAIL);

    errorstate := SD_DATA_CRC_FAIL;

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_TXUNDERR)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_TXUNDERR);

    errorstate := SD_TX_UNDERRUN;

    exit(errorstate);
  end
  else
  begin
    (* No error flag set *)
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  (* Wait till the card is in programming state *)
  cardstate:=SD_CARD_READY;
  errorstate := SD_IsCardProgramming(hsd, cardstate);
  while ((errorstate = SD_OK) and ((cardstate = SD_CARD_PROGRAMMING) or (cardstate = SD_CARD_RECEIVING))) do
    errorstate := SD_IsCardProgramming(hsd, cardstate);

  exit(errorstate);
end;

function HAL_SD_Erase(var hsd: SD_HandleTypeDef; startaddr, endaddr: qword): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  maxdelay, delay: longword;
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  cardstate: HAL_SD_CardStateTypedef;
begin
  (* Check if the card command class supports erase command *)
  if (((hsd.CSD[1] shr 20) and SD_CCCC_ERASE) = 0) then
  begin
    errorstate := SD_REQUEST_NOT_APPLICABLE;

    exit(errorstate);
  end;

  (* Get max delay value *)
  maxdelay := 120000 div (((hsd.Instance^.CLKCR) and $FF) + 2);

  if ((SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1) and SD_CARD_LOCKED) = SD_CARD_LOCKED) then
  begin
    errorstate := SD_LOCK_UNLOCK_FAILED;

    exit(errorstate);
  end;

  (* Get start and end block for high capacity cards *)
  if (hsd.CardType = HIGH_CAPACITY_SD_CARD) then
  begin
    startaddr := startaddr div 512;
    endaddr := endaddr div 512;
  end;

  (* According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) *)
  if ((hsd.CardType = STD_CAPACITY_SD_CARD_V1_1) or (hsd.CardType = STD_CAPACITY_SD_CARD_V2_0) or (hsd.CardType = HIGH_CAPACITY_SD_CARD)) then
  begin
    (* Send CMD32 SD_ERASE_GRP_START with argument as addr  *)
    sdmmc_cmdinitstructure.Argument := startaddr;
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SD_ERASE_GRP_START;
    sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
    sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
    sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_SD_ERASE_GRP_START);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    (* Send CMD33 SD_ERASE_GRP_END with argument as addr  *)
    sdmmc_cmdinitstructure.Argument := endaddr;
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SD_ERASE_GRP_END;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_SD_ERASE_GRP_END);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;
  end;

  (* Send CMD38 ERASE *)
  sdmmc_cmdinitstructure.Argument := 0;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_ERASE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_ERASE);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  delay:=0;
  while delay < maxdelay do
    Inc(delay);

  cardstate:=SD_CARD_READY;
  (* Wait until the card is in programming state *)
  errorstate := SD_IsCardProgramming(hsd, cardstate);

  delay := SD_DATATIMEOUT;

  while ((delay > 0) and (errorstate = SD_OK) and ((cardstate = SD_CARD_PROGRAMMING) or (cardstate = SD_CARD_RECEIVING))) do
  begin
    errorstate := SD_IsCardProgramming(hsd, cardstate);
    Dec(delay);
  end;

  exit(errorstate);
end;

procedure HAL_SD_IRQHandler(var hsd: SD_HandleTypeDef);
begin
  (* Check for SDMMC interrupt flags *)
  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_IT_DATAEND)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_IT_DATAEND);

    (* SD transfer is complete *)
    hsd.SdTransferCplt := 1;

    (* No transfer error *)
    hsd.SdTransferErr := SD_OK;

    HAL_SD_XferCpltCallback(hsd);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_IT_DCRCFAIL)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DCRCFAIL);

    hsd.SdTransferErr := SD_DATA_CRC_FAIL;

    HAL_SD_XferErrorCallback(hsd);

  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_IT_DTIMEOUT)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DTIMEOUT);

    hsd.SdTransferErr := SD_DATA_TIMEOUT;

    HAL_SD_XferErrorCallback(hsd);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_IT_RXOVERR)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_RXOVERR);

    hsd.SdTransferErr := SD_RX_OVERRUN;

    HAL_SD_XferErrorCallback(hsd);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_IT_TXUNDERR)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_TXUNDERR);

    hsd.SdTransferErr := SD_TX_UNDERRUN;

    HAL_SD_XferErrorCallback(hsd);
  end
  else
  begin
    (* No error flag set *)
  end;

  (* Disable all SDMMC peripheral interrupt sources *)
  __HAL_SD_SDMMC_DISABLE_IT(hsd, SDMMC_IT_DCRCFAIL or SDMMC_IT_DTIMEOUT or SDMMC_IT_DATAEND or SDMMC_IT_TXFIFOHE or SDMMC_IT_RXFIFOHF or SDMMC_IT_TXUNDERR or SDMMC_IT_RXOVERR);

end;

procedure HAL_SD_DMA_RxCpltCallback_stub(var hdma: DMA_HandleTypeDef); assembler; nostackframe; public name 'HAL_SD_DMA_RxCpltCallback';
asm
  .weak HAL_SD_DMA_RxCpltCallback
end;

procedure HAL_SD_DMA_RxErrorCallback_stub(var hdma: DMA_HandleTypeDef);assembler; nostackframe; public name 'HAL_SD_DMA_RxErrorCallback';
asm
  .weak HAL_SD_DMA_RxErrorCallback
end;

procedure HAL_SD_DMA_TxCpltCallback_stub(var hdma: DMA_HandleTypeDef); assembler; nostackframe; public name 'HAL_SD_DMA_TxCpltCallback';
asm
  .weak HAL_SD_DMA_TxCpltCallback
end;

procedure HAL_SD_DMA_TxErrorCallback_stub(var hdma: DMA_HandleTypeDef); assembler; nostackframe; public name 'HAL_SD_DMA_TxErrorCallback';
asm
  .weak HAL_SD_DMA_TxErrorCallback
end;

procedure HAL_SD_XferCpltCallback_stub(var hsd: SD_HandleTypeDef); assembler; nostackframe; public name 'HAL_SD_XferCpltCallback';
asm
  .weak HAL_SD_XferCpltCallback
end;

procedure HAL_SD_XferErrorCallback_stub(var hsd: SD_HandleTypeDef); assembler; nostackframe; public name 'HAL_SD_XferErrorCallback';
asm
  .weak HAL_SD_XferErrorCallback
end;

function HAL_SD_ReadBlocks_DMA(var hsd: SD_HandleTypeDef; var pReadBuffer; ReadAddr: qword; BlockSize, NumberOfBlocks: longword): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  sdmmc_datainitstructure: SDMMC_DataInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
begin
  (* Initialize data control register *)
  hsd.Instance^.DCTRL := 0;

  (* Initialize handle flags *)
  hsd.SdTransferCplt := 0;
  hsd.DmaTransferCplt := 0;
  hsd.SdTransferErr := SD_OK;

  (* Initialize SD Read operation *)
  if (NumberOfBlocks > 1) then
  begin
    hsd.SdOperation := SD_READ_MULTIPLE_BLOCK;
  end
  else
  begin
    hsd.SdOperation := SD_READ_SINGLE_BLOCK;
  end;

  (* Enable transfer interrupts *)
  __HAL_SD_SDMMC_ENABLE_IT(hsd, (SDMMC_IT_DCRCFAIL or SDMMC_IT_DTIMEOUT or SDMMC_IT_DATAEND or SDMMC_IT_RXOVERR));

  (* Enable SDMMC DMA transfer *)
  __HAL_SD_SDMMC_DMA_ENABLE(hsd);

  (* Configure DMA user callbacks *)
  hsd.hdmarx^.XferCpltCallback := @SD_DMA_RxCplt;
  hsd.hdmarx^.XferErrorCallback := @SD_DMA_RxError;

  (* Enable the DMA Channel *)
  HAL_DMA_Start_IT(hsd.hdmarx^, @hsd.Instance^.FIFO, @pReadBuffer, (BlockSize * NumberOfBlocks) div 4);

  if (hsd.CardType = HIGH_CAPACITY_SD_CARD) then
  begin
    BlockSize := 512;
    ReadAddr := ReadAddr div 512;
  end;

  (* Set Block Size for Card *)
  sdmmc_cmdinitstructure.Argument := BlockSize;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SET_BLOCKLEN;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SET_BLOCKLEN);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Configure the SD DPSM (Data Path State Machine) *)
  sdmmc_datainitstructure.DataTimeOut := SD_DATATIMEOUT;
  sdmmc_datainitstructure.DataLength := BlockSize * NumberOfBlocks;
  sdmmc_datainitstructure.DataBlockSize := SDMMC_DATABLOCK_SIZE_512B;
  sdmmc_datainitstructure.TransferDir := SDMMC_TRANSFER_DIR_TO_SDMMC;
  sdmmc_datainitstructure.TransferMode := SDMMC_TRANSFER_MODE_BLOCK;
  sdmmc_datainitstructure.DPSM := SDMMC_DPSM_ENABLE;
  SDMMC_DataConfig(hsd.Instance^, sdmmc_datainitstructure);

  (* Check number of blocks command *)
  if (NumberOfBlocks > 1) then
  begin
    (* Send CMD18 READ_MULT_BLOCK with argument data address *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_READ_MULT_BLOCK;
  end
  else
  begin
    (* Send CMD17 READ_SINGLE_BLOCK *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_READ_SINGLE_BLOCK;
  end;

  sdmmc_cmdinitstructure.Argument := ReadAddr;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  if (NumberOfBlocks > 1) then
  begin
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_READ_MULT_BLOCK);
  end
  else
  begin
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_READ_SINGLE_BLOCK);
  end;

  (* Update the SD transfer error in SD handle *)
  hsd.SdTransferErr := errorstate;

  exit(errorstate);

end;

function HAL_SD_WriteBlocks_DMA(var hsd: SD_HandleTypeDef; const pWriteBuffer; WriteAddr: qword; BlockSize, NumberOfBlocks: longword): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  sdmmc_datainitstructure: SDMMC_DataInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
begin
  (* Initialize data control register *)
  hsd.Instance^.DCTRL := 0;

  (* Initialize handle flags *)
  hsd.SdTransferCplt := 0;
  hsd.DmaTransferCplt := 0;
  hsd.SdTransferErr := SD_OK;

  (* Initialize SD Write operation *)
  if (NumberOfBlocks > 1) then
  begin
    hsd.SdOperation := SD_WRITE_MULTIPLE_BLOCK;
  end
  else
  begin
    hsd.SdOperation := SD_WRITE_SINGLE_BLOCK;
  end;

  (* Enable transfer interrupts *)
  __HAL_SD_SDMMC_ENABLE_IT(hsd, (SDMMC_IT_DCRCFAIL or SDMMC_IT_DTIMEOUT or SDMMC_IT_DATAEND or SDMMC_IT_TXUNDERR));

  (* Configure DMA user callbacks *)
  hsd.hdmatx^.XferCpltCallback := @SD_DMA_TxCplt;
  hsd.hdmatx^.XferErrorCallback := @SD_DMA_TxError;

  (* Enable the DMA Channel *)
  HAL_DMA_Start_IT(hsd.hdmatx^, @pWriteBuffer, @hsd.Instance^.FIFO, (BlockSize * NumberOfBlocks) shr 2);

  (* Enable SDMMC DMA transfer *)
  __HAL_SD_SDMMC_DMA_ENABLE(hsd);

  if (hsd.CardType = HIGH_CAPACITY_SD_CARD) then
  begin
    BlockSize := 512;
    WriteAddr := WriteAddr div 512;
  end;

  (* Set Block Size for Card *)
  sdmmc_cmdinitstructure.Argument := BlockSize;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SET_BLOCKLEN;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SET_BLOCKLEN);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Check number of blocks command *)
  if (NumberOfBlocks <= 1) then
  begin
    (* Send CMD24 WRITE_SINGLE_BLOCK *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_WRITE_SINGLE_BLOCK;
  end
  else
  begin
    (* Send CMD25 WRITE_MULT_BLOCK with argument data address *)
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_WRITE_MULT_BLOCK;
  end;

  sdmmc_cmdinitstructure.Argument := WriteAddr;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  if (NumberOfBlocks > 1) then
  begin
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_WRITE_MULT_BLOCK);
  end
  else
  begin
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_WRITE_SINGLE_BLOCK);
  end;

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Configure the SD DPSM (Data Path State Machine) *)
  sdmmc_datainitstructure.DataTimeOut := SD_DATATIMEOUT;
  sdmmc_datainitstructure.DataLength := BlockSize * NumberOfBlocks;
  sdmmc_datainitstructure.DataBlockSize := SDMMC_DATABLOCK_SIZE_512B;
  sdmmc_datainitstructure.TransferDir := SDMMC_TRANSFER_DIR_TO_CARD;
  sdmmc_datainitstructure.TransferMode := SDMMC_TRANSFER_MODE_BLOCK;
  sdmmc_datainitstructure.DPSM := SDMMC_DPSM_ENABLE;
  SDMMC_DataConfig(hsd.Instance^, sdmmc_datainitstructure);

  hsd.SdTransferErr := errorstate;

  exit(errorstate);

end;

function HAL_SD_CheckWriteOperation(var hsd: SD_HandleTypeDef; Timeout: longword): HAL_SD_ErrorTypedef;
var
  tmp1: longword;
  tmp2: longword;
  tmp3: HAL_SD_ErrorTypedef;
  errorstate: HAL_SD_ErrorTypedef;
begin
  errorstate:=SD_OK;

  (* Wait for DMA/SD transfer end or SD error variables to be in SD handle *)
  tmp1 := hsd.DmaTransferCplt;
  tmp2 := hsd.SdTransferCplt;
  tmp3 := HAL_SD_ErrorTypedef(hsd.SdTransferErr);

  while (((tmp1 and tmp2) = 0) and (tmp3 = SD_OK) and (timeout > 0)) do
  begin
    tmp1 := hsd.DmaTransferCplt;
    tmp2 := hsd.SdTransferCplt;
    tmp3 := HAL_SD_ErrorTypedef(hsd.SdTransferErr);
    Dec(timeout);
  end;

  timeout := Timeout;

  (* Wait until the Tx transfer is no longer active *)
  while ((__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_TXACT)) and (timeout > 0)) do
  begin
    Dec(timeout);
  end;

  (* Send stop command in multiblock write *)
  if (hsd.SdOperation = SD_WRITE_MULTIPLE_BLOCK) then
  begin
    errorstate := HAL_SD_StopTransfer(hsd);
  end;

  if ((timeout = 0) and (errorstate = SD_OK)) then
  begin
    errorstate := SD_DATA_TIMEOUT;
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  (* Return error state *)
  if (hsd.SdTransferErr <> SD_OK) then
  begin
    exit(HAL_SD_ErrorTypedef(hsd.SdTransferErr));
  end;

  (* Wait until write is complete *)
  while (HAL_SD_GetStatus(hsd) <> SD_TRANSFER_OK) do ;

  exit(errorstate);

end;

function HAL_SD_CheckReadOperation(var hsd: SD_HandleTypeDef; Timeout: longword): HAL_SD_ErrorTypedef;
var
  tmp1: longword;
  tmp2: longword;
  tmp3: HAL_SD_ErrorTypedef;
  errorstate: HAL_SD_ErrorTypedef;
begin
  errorstate:=SD_OK;

  (* Wait for DMA/SD transfer end or SD error variables to be in SD handle *)
  tmp1 := hsd.DmaTransferCplt;
  tmp2 := hsd.SdTransferCplt;
  tmp3 := HAL_SD_ErrorTypedef(hsd.SdTransferErr);

  while (((tmp1 and tmp2) = 0) and (tmp3 = SD_OK) and (timeout > 0)) do
  begin
    tmp1 := hsd.DmaTransferCplt;
    tmp2 := hsd.SdTransferCplt;
    tmp3 := HAL_SD_ErrorTypedef(hsd.SdTransferErr);
    Dec(timeout);
  end;

  timeout := Timeout;

  (* Wait until the Rx transfer is no longer active *)
  while ((__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXACT)) and (timeout > 0)) do
  begin
    Dec(timeout);
  end;

  (* Send stop command in multiblock read *)
  if (hsd.SdOperation = SD_READ_MULTIPLE_BLOCK) then
  begin
    errorstate := HAL_SD_StopTransfer(hsd);
  end;

  if ((timeout = 0) and (errorstate = SD_OK)) then
  begin
    errorstate := SD_DATA_TIMEOUT;
  end;

  (* Clear all the static flags *)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  (* Return error state *)
  if (hsd.SdTransferErr <> SD_OK) then
  begin
    exit(HAL_SD_ErrorTypedef(hsd.SdTransferErr));
  end;

  exit(errorstate);
end;

function HAL_SD_Get_CardInfo(var hsd: SD_HandleTypeDef; var pCardInfo: HAL_SD_CardInfoTypedef): HAL_SD_ErrorTypedef;
var
  tmp: longword;
  errorstate: HAL_SD_ErrorTypedef;
begin
  errorstate:=SD_OK;

  pCardInfo.CardType := (hsd.CardType);
  pCardInfo.RCA := (hsd.RCA);

  (* Byte 0 *)
  tmp := (hsd.CSD[0] and $FF000000) shr 24;
  pCardInfo.SD_csd.CSDStruct := ((tmp and $C0) shr 6);
  pCardInfo.SD_csd.SysSpecVersion := ((tmp and $3C) shr 2);
  pCardInfo.SD_csd.Reserved1 := tmp and $03;

  (* Byte 1 *)
  tmp := (hsd.CSD[0] and $00FF0000) shr 16;
  pCardInfo.SD_csd.TAAC := tmp;

  (* Byte 2 *)
  tmp := (hsd.CSD[0] and $0000FF00) shr 8;
  pCardInfo.SD_csd.NSAC := tmp;

  (* Byte 3 *)
  tmp := hsd.CSD[0] and $000000FF;
  pCardInfo.SD_csd.MaxBusClkFrec := tmp;

  (* Byte 4 *)
  tmp := (hsd.CSD[1] and $FF000000) shr 24;
  pCardInfo.SD_csd.CardComdClasses := (tmp shl 4);

  (* Byte 5 *)
  tmp := (hsd.CSD[1] and $00FF0000) shr 16;
  pCardInfo.SD_csd.CardComdClasses := pCardInfo.SD_csd.CardComdClasses or ((tmp and $F0) shr 4);
  pCardInfo.SD_csd.RdBlockLen := (tmp and $0F);

  (* Byte 6 *)
  tmp := (hsd.CSD[1] and $0000FF00) shr 8;
  pCardInfo.SD_csd.PartBlockRead := ((tmp and $80) shr 7);
  pCardInfo.SD_csd.WrBlockMisalign := ((tmp and $40) shr 6);
  pCardInfo.SD_csd.RdBlockMisalign := ((tmp and $20) shr 5);
  pCardInfo.SD_csd.DSRImpl := ((tmp and $10) shr 4);
  pCardInfo.SD_csd.Reserved2 := 0; (*not < Reserved *)

  if ((hsd.CardType = STD_CAPACITY_SD_CARD_V1_1) or (hsd.CardType = STD_CAPACITY_SD_CARD_V2_0)) then
  begin
    pCardInfo.SD_csd.DeviceSize := (tmp and $03) shl 10;

    (* Byte 7 *)
    tmp := (hsd.CSD[1] and $000000FF);
    pCardInfo.SD_csd.DeviceSize := pCardInfo.SD_csd.DeviceSize or (tmp) shl 2;

    (* Byte 8 *)
    tmp := ((hsd.CSD[2] and $FF000000) shr 24);
    pCardInfo.SD_csd.DeviceSize := pCardInfo.SD_csd.DeviceSize or (tmp and $C0) shr 6;

    pCardInfo.SD_csd.MaxRdCurrentVDDMin := (tmp and $38) shr 3;
    pCardInfo.SD_csd.MaxRdCurrentVDDMax := (tmp and $07);

    (* Byte 9 *)
    tmp := ((hsd.CSD[2] and $00FF0000) shr 16);
    pCardInfo.SD_csd.MaxWrCurrentVDDMin := (tmp and $E0) shr 5;
    pCardInfo.SD_csd.MaxWrCurrentVDDMax := (tmp and $1C) shr 2;
    pCardInfo.SD_csd.DeviceSizeMul := (tmp and $03) shl 1;
    (* Byte 10 *)
    tmp := ((hsd.CSD[2] and $0000FF00) shr 8);
    pCardInfo.SD_csd.DeviceSizeMul := pCardInfo.SD_csd.DeviceSizeMul or (tmp and $80) shr 7;

    pCardInfo.CardCapacity := (pCardInfo.SD_csd.DeviceSize + 1);
    pCardInfo.CardCapacity := pCardInfo.CardCapacity * (1 shl (pCardInfo.SD_csd.DeviceSizeMul + 2));
    pCardInfo.CardBlockSize := 1 shl (pCardInfo.SD_csd.RdBlockLen);
    pCardInfo.CardCapacity := pCardInfo.CardCapacity * pCardInfo.CardBlockSize;
  end
  else if (hsd.CardType = HIGH_CAPACITY_SD_CARD) then
  begin
    (* Byte 7 *)
    tmp := (hsd.CSD[1] and $000000FF);
    pCardInfo.SD_csd.DeviceSize := (tmp and $3F) shl 16;

    (* Byte 8 *)
    tmp := ((hsd.CSD[2] and $FF000000) shr 24);

    pCardInfo.SD_csd.DeviceSize := pCardInfo.SD_csd.DeviceSize or (tmp shl 8);

    (* Byte 9 *)
    tmp := ((hsd.CSD[2] and $00FF0000) shr 16);

    pCardInfo.SD_csd.DeviceSize := pCardInfo.SD_csd.DeviceSize or (tmp);

    (* Byte 10 *)
    tmp := ((hsd.CSD[2] and $0000FF00) shr 8);

    pCardInfo.CardCapacity := ((pCardInfo.SD_csd.DeviceSize + 1)) * 512 * 1024;
    pCardInfo.CardBlockSize := 512;
  end
  else
  begin
    (* Not supported card type *)
    errorstate := SD_ERROR;
  end;

  pCardInfo.SD_csd.EraseGrSize := (tmp and $40) shr 6;
  pCardInfo.SD_csd.EraseGrMul := (tmp and $3F) shl 1;

  (* Byte 11 *)
  tmp := (hsd.CSD[2] and $000000FF);
  pCardInfo.SD_csd.EraseGrMul := pCardInfo.SD_csd.EraseGrMul or (tmp and $80) shr 7;
  pCardInfo.SD_csd.WrProtectGrSize := (tmp and $7F);

  (* Byte 12 *)
  tmp := ((hsd.CSD[3] and $FF000000) shr 24);
  pCardInfo.SD_csd.WrProtectGrEnable := (tmp and $80) shr 7;
  pCardInfo.SD_csd.ManDeflECC := (tmp and $60) shr 5;
  pCardInfo.SD_csd.WrSpeedFact := (tmp and $1C) shr 2;
  pCardInfo.SD_csd.MaxWrBlockLen := (tmp and $03) shl 2;

  (* Byte 13 *)
  tmp := ((hsd.CSD[3] and $00FF0000) shr 16);
  pCardInfo.SD_csd.MaxWrBlockLen := pCardInfo.SD_csd.MaxWrBlockLen or (tmp and $C0) shr 6;
  pCardInfo.SD_csd.WriteBlockPaPartial := (tmp and $20) shr 5;
  pCardInfo.SD_csd.Reserved3 := 0;
  pCardInfo.SD_csd.ContentProtectAppli := (tmp and $01);

  (* Byte 14 *)
  tmp := ((hsd.CSD[3] and $0000FF00) shr 8);
  pCardInfo.SD_csd.FileFormatGrouop := (tmp and $80) shr 7;
  pCardInfo.SD_csd.CopyFlag := (tmp and $40) shr 6;
  pCardInfo.SD_csd.PermWrProtect := (tmp and $20) shr 5;
  pCardInfo.SD_csd.TempWrProtect := (tmp and $10) shr 4;
  pCardInfo.SD_csd.FileFormat := (tmp and $0C) shr 2;
  pCardInfo.SD_csd.ECC := (tmp and $03);

  (* Byte 15 *)
  tmp := (hsd.CSD[3] and $000000FF);
  pCardInfo.SD_csd.CSD_CRC := (tmp and $FE) shr 1;
  pCardInfo.SD_csd.Reserved4 := 1;

  (* Byte 0 *)
  tmp := ((hsd.CID[0] and $FF000000) shr 24);
  pCardInfo.SD_cid.ManufacturerID := tmp;

  (* Byte 1 *)
  tmp := ((hsd.CID[0] and $00FF0000) shr 16);
  pCardInfo.SD_cid.OEM_AppliID := tmp shl 8;

  (* Byte 2 *)
  tmp := ((hsd.CID[0] and $000000FF00) shr 8);
  pCardInfo.SD_cid.OEM_AppliID := pCardInfo.SD_cid.OEM_AppliID or tmp;

  (* Byte 3 *)
  tmp := (hsd.CID[0] and $000000FF);
  pCardInfo.SD_cid.ProdName1 := tmp shl 24;

  (* Byte 4 *)
  tmp := ((hsd.CID[1] and $FF000000) shr 24);
  pCardInfo.SD_cid.ProdName1 := pCardInfo.SD_cid.ProdName1 or tmp shl 16;

  (* Byte 5 *)
  tmp := ((hsd.CID[1] and $00FF0000) shr 16);
  pCardInfo.SD_cid.ProdName1 := pCardInfo.SD_cid.ProdName1 or tmp shl 8;

  (* Byte 6 *)
  tmp := ((hsd.CID[1] and $0000FF00) shr 8);
  pCardInfo.SD_cid.ProdName1 := pCardInfo.SD_cid.ProdName1 or tmp;

  (* Byte 7 *)
  tmp := (hsd.CID[1] and $000000FF);
  pCardInfo.SD_cid.ProdName2 := tmp;

  (* Byte 8 *)
  tmp := ((hsd.CID[2] and $FF000000) shr 24);
  pCardInfo.SD_cid.ProdRev := tmp;

  (* Byte 9 *)
  tmp := ((hsd.CID[2] and $00FF0000) shr 16);
  pCardInfo.SD_cid.ProdSN := tmp shl 24;

  (* Byte 10 *)
  tmp := ((hsd.CID[2] and $0000FF00) shr 8);
  pCardInfo.SD_cid.ProdSN := pCardInfo.SD_cid.ProdSN or tmp shl 16;

  (* Byte 11 *)
  tmp := (hsd.CID[2] and $000000FF);
  pCardInfo.SD_cid.ProdSN := pCardInfo.SD_cid.ProdSN or tmp shl 8;

  (* Byte 12 *)
  tmp := ((hsd.CID[3] and $FF000000) shr 24);
  pCardInfo.SD_cid.ProdSN := pCardInfo.SD_cid.ProdSN or tmp;

  (* Byte 13 *)
  tmp := ((hsd.CID[3] and $00FF0000) shr 16);
  pCardInfo.SD_cid.Reserved1 := pCardInfo.SD_cid.Reserved1 or (tmp and $F0) shr 4;
  pCardInfo.SD_cid.ManufactDate := (tmp and $0F) shl 8;

  (* Byte 14 *)
  tmp := ((hsd.CID[3] and $0000FF00) shr 8);
  pCardInfo.SD_cid.ManufactDate := pCardInfo.SD_cid.ManufactDate or tmp;

  (* Byte 15 *)
  tmp := (hsd.CID[3] and $000000FF);
  pCardInfo.SD_cid.CID_CRC := (tmp and $FE) shr 1;
  pCardInfo.SD_cid.Reserved2 := 1;

  exit(errorstate);
end;

function HAL_SD_WideBusOperation_Config(var hsd: SD_HandleTypeDef; WideMode: longword): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  tmpinit: SDMMC_InitTypeDef;
begin
  errorstate:=SD_OK;

  (* MMC Card does not support this feature *)
  if (hsd.CardType = MULTIMEDIA_CARD) then
  begin
    errorstate := SD_UNSUPPORTED_FEATURE;

    exit(errorstate);
  end
  else if ((hsd.CardType = STD_CAPACITY_SD_CARD_V1_1) or (hsd.CardType = STD_CAPACITY_SD_CARD_V2_0) or (hsd.CardType = HIGH_CAPACITY_SD_CARD)) then
  begin
    if (WideMode = SDMMC_BUS_WIDE_8B) then
    begin
      errorstate := SD_UNSUPPORTED_FEATURE;
    end
    else if (WideMode = SDMMC_BUS_WIDE_4B) then
    begin
      errorstate := SD_WideBus_Enable(hsd);
    end
    else if (WideMode = SDMMC_BUS_WIDE_1B) then
    begin
      errorstate := SD_WideBus_Disable(hsd);
    end
    else
    begin
      (* WideMode is not a valid argument*)
      errorstate := SD_INVALID_PARAMETER;
    end;

    if (errorstate = SD_OK) then
    begin
      (* Configure the SDMMC peripheral *)
      tmpinit.ClockEdge := hsd.Init.ClockEdge;
      tmpinit.ClockBypass := hsd.Init.ClockBypass;
      tmpinit.ClockPowerSave := hsd.Init.ClockPowerSave;
      tmpinit.BusWide := WideMode;
      tmpinit.HardwareFlowControl := hsd.Init.HardwareFlowControl;
      tmpinit.ClockDiv := hsd.Init.ClockDiv;

      SDMMC_Init(hsd.Instance^, tmpinit);
    end;
  end;

  exit(errorstate);
end;

function HAL_SD_StopTransfer(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  errorstate: HAL_SD_ErrorTypedef;
begin
  (* Send CMD12 STOP_TRANSMISSION  *)
  sdmmc_cmdinitstructure.Argument := 0;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_STOP_TRANSMISSION;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_STOP_TRANSMISSION);

  exit(errorstate);
end;

function HAL_SD_HighSpeed(var hsd: SD_HandleTypeDef): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  sdmmc_datainitstructure: SDMMC_DataInitTypeDef;
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  SD_hs: array[0..63] of byte;
  SD_scr: array[0..1] of byte;
  Count, SD_SPEC: longword;
  tempbuff: plongword;
begin
  pword(Unaligned(@SD_scr[0]))^ := 0;
  SD_SPEC := 0;
  Count := 0;
  tempbuff := @SD_hs[0];

  (* Initialize the Data control register *)
  hsd.Instance^.DCTRL := 0;

  (* Get SCR Register *)
  errorstate := SD_FindSCR(hsd, SD_scr[0]);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Test the Version supported by the card*)
  SD_SPEC := (SD_scr[1] and $01000000) or (SD_scr[1] and $02000000);

  if (SD_SPEC <> SD_ALLZERO) then
  begin
    (* Set Block Size for Card *)
    sdmmc_cmdinitstructure.Argument := 64;
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SET_BLOCKLEN;
    sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
    sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
    sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_SET_BLOCKLEN);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    (* Configure the SD DPSM (Data Path State Machine) *)
    sdmmc_datainitstructure.DataTimeOut := SD_DATATIMEOUT;
    sdmmc_datainitstructure.DataLength := 64;
    sdmmc_datainitstructure.DataBlockSize := SDMMC_DATABLOCK_SIZE_64B;
    sdmmc_datainitstructure.TransferDir := SDMMC_TRANSFER_DIR_TO_SDMMC;
    sdmmc_datainitstructure.TransferMode := SDMMC_TRANSFER_MODE_BLOCK;
    sdmmc_datainitstructure.DPSM := SDMMC_DPSM_ENABLE;
    SDMMC_DataConfig(hsd.Instance^, sdmmc_datainitstructure);

    (* Send CMD6 switch mode *)
    sdmmc_cmdinitstructure.Argument := $80FFFF01;
    sdmmc_cmdinitstructure.CmdIndex := SD_CMD_HS_SWITCH;
    SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

    (* Check for error conditions *)
    errorstate := SD_CmdResp1Error(hsd, SD_CMD_HS_SWITCH);

    if (errorstate <> SD_OK) then
    begin
      exit(errorstate);
    end;

    while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR or SDMMC_FLAG_DCRCFAIL or SDMMC_FLAG_DTIMEOUT or SDMMC_FLAG_DBCKEND)) do
    begin
      if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXFIFOHF)) then
      begin
        for Count := 0 to 7 do
        begin
          tempbuff[Count] := SDMMC_ReadFIFO(hsd.Instance^);
        end;

        Inc(tempbuff, 8);
      end;
    end;

    if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DTIMEOUT)) then
    begin
      __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DTIMEOUT);

      errorstate := SD_DATA_TIMEOUT;

      exit(errorstate);
    end
    else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL)) then
    begin
      __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DCRCFAIL);

      errorstate := SD_DATA_CRC_FAIL;

      exit(errorstate);
    end
    else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR)) then
    begin
      __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_RXOVERR);

      errorstate := SD_RX_OVERRUN;

      exit(errorstate);
    end
    else
    begin
      (* No error flag set *)
    end;

    Count := SD_DATATIMEOUT;

    while ((__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXDAVL)) and (Count > 0)) do
    begin
      tempbuff^ := SDMMC_ReadFIFO(hsd.Instance^);
      Inc(tempbuff);
      Dec(Count);
    end;

    (* Clear all the static flags *)
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

    (* Test if the switch mode HS is ok *)
    if ((SD_hs[13] and 2) <> 2) then
    begin
      errorstate := SD_UNSUPPORTED_FEATURE;
    end;
  end;

  exit(errorstate);
end;

function HAL_SD_SendSDStatus(var hsd: SD_HandleTypeDef; var pSDstatus): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  sdmmc_cmdinitstructure: SDMMC_CmdInitTypeDef;
  sdmmc_datainitstructure: SDMMC_DataInitTypeDef;
  pSDstatusPTR: plongword;
  Count: longword;
begin
  (* Check SD response *)
  if ((SDMMC_GetResponse(hsd.Instance^, SDMMC_RESP1) and SD_CARD_LOCKED) = SD_CARD_LOCKED) then
  begin
    errorstate := SD_LOCK_UNLOCK_FAILED;

    exit(errorstate);
  end;

  (* Set block size for card if it is not equal to current block size for card *)
  sdmmc_cmdinitstructure.Argument := 64;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SET_BLOCKLEN;
  sdmmc_cmdinitstructure.Response := SDMMC_RESPONSE_SHORT;
  sdmmc_cmdinitstructure.WaitForInterrupt := SDMMC_WAIT_NO;
  sdmmc_cmdinitstructure.CPSM := SDMMC_CPSM_ENABLE;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SET_BLOCKLEN);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Send CMD55 *)
  sdmmc_cmdinitstructure.Argument := (hsd.RCA shl 16);
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_APP_CMD;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_APP_CMD);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Configure the SD DPSM (Data Path State Machine) *)
  sdmmc_datainitstructure.DataTimeOut := SD_DATATIMEOUT;
  sdmmc_datainitstructure.DataLength := 64;
  sdmmc_datainitstructure.DataBlockSize := SDMMC_DATABLOCK_SIZE_64B;
  sdmmc_datainitstructure.TransferDir := SDMMC_TRANSFER_DIR_TO_SDMMC;
  sdmmc_datainitstructure.TransferMode := SDMMC_TRANSFER_MODE_BLOCK;
  sdmmc_datainitstructure.DPSM := SDMMC_DPSM_ENABLE;
  SDMMC_DataConfig(hsd.Instance^, sdmmc_datainitstructure);

  (* Send ACMD13 (SD_APP_STAUS)  with argument as card's RCA *)
  sdmmc_cmdinitstructure.Argument := 0;
  sdmmc_cmdinitstructure.CmdIndex := SD_CMD_SD_APP_STATUS;
  SDMMC_SendCommand(hsd.Instance^, sdmmc_cmdinitstructure);

  (* Check for error conditions *)
  errorstate := SD_CmdResp1Error(hsd, SD_CMD_SD_APP_STATUS);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  pSDstatusPTR := @pSDstatus;
  (* Get status data *)
  while (not __HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR or SDMMC_FLAG_DCRCFAIL or SDMMC_FLAG_DTIMEOUT or SDMMC_FLAG_DBCKEND)) do
  begin
    if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXFIFOHF)) then
    begin
      for Count := 0 to 7 do
        plongword(pSDstatusPTR)[Count] := SDMMC_ReadFIFO(hsd.Instance^);

      pSDstatusPTR := pSDstatusPTR + (8);
    end;
  end;

  if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DTIMEOUT)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DTIMEOUT);

    errorstate := SD_DATA_TIMEOUT;

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_DCRCFAIL);

    errorstate := SD_DATA_CRC_FAIL;

    exit(errorstate);
  end
  else if (__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR)) then
  begin
    __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_FLAG_RXOVERR);

    errorstate := SD_RX_OVERRUN;

    exit(errorstate);
  end
  else
  begin
    (* No error flag set *)
  end;

  Count := SD_DATATIMEOUT;
  while ((__HAL_SD_SDMMC_GET_FLAG(hsd, SDMMC_FLAG_RXDAVL)) and (Count > 0)) do
  begin
    pSDstatusPTR^ := SDMMC_ReadFIFO(hsd.Instance^);
    Inc(pSDstatusPTR);
    Dec(Count);
  end;

  (* Clear all the static status flags*)
  __HAL_SD_SDMMC_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

  exit(errorstate);
end;

function HAL_SD_GetCardStatus(var hsd: SD_HandleTypeDef; var pCardStatus: HAL_SD_CardStatusTypedef): HAL_SD_ErrorTypedef;
var
  errorstate: HAL_SD_ErrorTypedef;
  tmp: longword;
  sd_status: array[0..15] of byte;
begin
  errorstate := SD_OK;
  tmp := 0;

  errorstate := HAL_SD_SendSDStatus(hsd, sd_status);

  if (errorstate <> SD_OK) then
  begin
    exit(errorstate);
  end;

  (* Byte 0 *)
  tmp := (sd_status[0] and $C0) shr 6;
  pCardStatus.DAT_BUS_WIDTH := tmp;

  (* Byte 0 *)
  tmp := (sd_status[0] and $20) shr 5;
  pCardStatus.SECURED_MODE := tmp;

  (* Byte 2 *)
  tmp := (sd_status[2] and $FF);
  pCardStatus.SD_CARD_TYPE := (tmp shl 8);

  (* Byte 3 *)
  tmp := (sd_status[3] and $FF);
  pCardStatus.SD_CARD_TYPE := pCardStatus.SD_CARD_TYPE or tmp;

  (* Byte 4 *)
  tmp := (sd_status[4] and $FF);
  pCardStatus.SIZE_OF_PROTECTED_AREA := (tmp shl 24);

  (* Byte 5 *)
  tmp := (sd_status[5] and $FF);
  pCardStatus.SIZE_OF_PROTECTED_AREA := pCardStatus.SIZE_OF_PROTECTED_AREA or (tmp shl 16);

  (* Byte 6 *)
  tmp := (sd_status[6] and $FF);
  pCardStatus.SIZE_OF_PROTECTED_AREA := pCardStatus.SIZE_OF_PROTECTED_AREA or (tmp shl 8);

  (* Byte 7 *)
  tmp := (sd_status[7] and $FF);
  pCardStatus.SIZE_OF_PROTECTED_AREA := pCardStatus.SIZE_OF_PROTECTED_AREA or tmp;

  (* Byte 8 *)
  tmp := (sd_status[8] and $FF);
  pCardStatus.SPEED_CLASS := tmp;

  (* Byte 9 *)
  tmp := (sd_status[9] and $FF);
  pCardStatus.PERFORMANCE_MOVE := tmp;

  (* Byte 10 *)
  tmp := (sd_status[10] and $F0) shr 4;
  pCardStatus.AU_SIZE := tmp;

  (* Byte 11 *)
  tmp := (sd_status[11] and $FF);
  pCardStatus.ERASE_SIZE := (tmp shl 8);

  (* Byte 12 *)
  tmp := (sd_status[12] and $FF);
  pCardStatus.ERASE_SIZE := pCardStatus.ERASE_SIZE or tmp;

  (* Byte 13 *)
  tmp := (sd_status[13] and $FC) shr 2;
  pCardStatus.ERASE_TIMEOUT := tmp;

  (* Byte 13 *)
  tmp := (sd_status[13] and $3);
  pCardStatus.ERASE_OFFSET := tmp;

  exit(errorstate);

end;

function HAL_SD_GetStatus(var hsd: SD_HandleTypeDef): HAL_SD_TransferStateTypedef;
var
  cardstate: HAL_SD_CardStateTypedef;
begin
  (* Get SD card state *)
  cardstate := SD_GetState(hsd);

  (* Find SD status according to card state*)
  if (cardstate = SD_CARD_TRANSFER) then
    exit(SD_TRANSFER_OK)
  else if (cardstate = SD_CARD_ERROR) then
    exit(SD_TRANSFER_ERROR)
  else
    exit(SD_TRANSFER_BUSY);
end;

end.
