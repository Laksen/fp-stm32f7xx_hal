(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_dma.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of DMA HAL module.
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

unit stm32f7xx_hal_dma;

interface

{$packrecords C}

uses
  stm32f7xx_hal, stm32f7xx_defs;

(**
  * @brief  DMA Configuration Structure definition
   *)

type
  TDMA_StreamPtr = record
    CR: longword;                   // 0x10 stream x configuration register
    NDTR_bits: TDMA1_S0NDTR_bits;
    NDTR: longword;                   // 0x14 stream x number of data register
    PAR_bits: TDMA1_S0PAR_bits;
    PAR: longword;                   // 0x18 stream x peripheral address register
    M0AR_bits: TDMA1_S0M0AR_bits;
    M0AR: longword;                   // 0x1C stream x memory 0 address register
    M1AR_bits: TDMA1_S0M1AR_bits;
    M1AR: longword;                   // 0x20 stream x memory 1 address register
    FCR_bits: TDMA1_S0FCR_bits;
    FCR: longword;                   // 0x24 stream x FIFO control register
  end;

  DMA_InitTypeDef = record
    Channel: longword;  (*!< Specifies the channel used for the specified stream.
                                      This parameter can be a value of @ref DMA_Channel_selection                     *)
    Direction: longword;  (*!< Specifies if the data will be transferred from memory to peripheral,
                                      from memory to memory or from peripheral to memory.
                                      This parameter can be a value of @ref DMA_Data_transfer_direction               *)
    PeriphInc: longword;  (*!< Specifies whether the Peripheral address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Peripheral_incremented_mode           *)
    MemInc: longword;  (*!< Specifies whether the memory address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Memory_incremented_mode               *)
    PeriphDataAlignment: longword;  (*!< Specifies the Peripheral data width.
                                      This parameter can be a value of @ref DMA_Peripheral_data_size                  *)
    MemDataAlignment: longword;  (*!< Specifies the Memory data width.
                                      This parameter can be a value of @ref DMA_Memory_data_size                      *)
    Mode: longword;  (*!< Specifies the operation mode of the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_mode
                                      @note The circular buffer mode cannot be used if the memory-to-memory
                                            data transfer is configured on the selected Stream                         *)
    Priority: longword;  (*!< Specifies the software priority for the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_Priority_level                        *)
    FIFOMode: longword;  (*!< Specifies if the FIFO mode or Direct mode will be used for the specified stream.
                                      This parameter can be a value of @ref DMA_FIFO_direct_mode
                                      @note The Direct mode (FIFO mode disabled) cannot be used if the
                                            memory-to-memory data transfer is configured on the selected stream        *)
    FIFOThreshold: longword;  (*!< Specifies the FIFO threshold level.
                                      This parameter can be a value of @ref DMA_FIFO_threshold_level                   *)
    MemBurst: longword;  (*!< Specifies the Burst transfer configuration for the memory transfers.
                                      It specifies the amount of data to be transferred in a single non interruptible
                                      transaction.
                                      This parameter can be a value of @ref DMA_Memory_burst
                                      @note The burst mode is possible only if the address Increment mode is enabled.  *)
    PeriphBurst: longword;  (*!< Specifies the Burst transfer configuration for the peripheral transfers.
                                      It specifies the amount of data to be transferred in a single non interruptible
                                      transaction.
                                      This parameter can be a value of @ref DMA_Peripheral_burst
                                      @note The burst mode is possible only if the address Increment mode is enabled.  *)
  end;

  (**
  * @brief  HAL DMA State structures definition
   *)

const
  HAL_DMA_STATE_RESET      = $00;  (*!< DMA not yet initialized or disabled  *)
  HAL_DMA_STATE_READY      = $01;  (*!< DMA initialized and ready for use    *)
  HAL_DMA_STATE_READY_MEM0 = $11;  (*!< DMA Mem0 process success             *)
  HAL_DMA_STATE_READY_MEM1 = $21;  (*!< DMA Mem1 process success             *)
  HAL_DMA_STATE_READY_HALF_MEM0 = $31;  (*!< DMA Mem0 Half process success        *)
  HAL_DMA_STATE_READY_HALF_MEM1 = $41;  (*!< DMA Mem1 Half process success        *)
  HAL_DMA_STATE_BUSY       = $02;  (*!< DMA process is ongoing               *)
  HAL_DMA_STATE_BUSY_MEM0  = $12;  (*!< DMA Mem0 process is ongoing          *)
  HAL_DMA_STATE_BUSY_MEM1  = $22;  (*!< DMA Mem1 process is ongoing          *)
  HAL_DMA_STATE_TIMEOUT    = $03;  (*!< DMA timeout state                    *)
  HAL_DMA_STATE_ERROR      = $04;  (*!< DMA error state                      *)

type
  HAL_DMA_StateTypeDef = integer;

  (**
  * @brief  HAL DMA Error Code structure definition
   *)

const
  HAL_DMA_FULL_TRANSFER = $00;  (*!< Full transfer      *)
  HAL_DMA_HALF_TRANSFER = $01;  (*!< Half Transfer      *)

type
  HAL_DMA_LevelCompleteTypeDef = integer;

  (**
  * @brief  DMA handle Structure definition
   *)

type
  P__DMA_HandleTypeDef = ^__DMA_HandleTypeDef;

  __DMA_HandleTypeDef = record
    Instance: ^TDMA_StreamPtr;  (*!< Register base address                   *)
    Init: DMA_InitTypeDef;  (*!< DMA communication parameters            *)
    Lock: HAL_LockTypeDef;  (*!< DMA locking object                      *)
    State: HAL_DMA_StateTypeDef;  (*!< DMA transfer state                      *)
    Parent: Pointer;  (*!< Parent object state                     *)
    XferCpltCallback: procedure(var hdma: __DMA_HandleTypeDef);  (*!< DMA transfer complete callback          *)
    XferHalfCpltCallback: procedure(var hdma: __DMA_HandleTypeDef);  (*!< DMA Half transfer complete callback     *)
    XferM1CpltCallback: procedure(var hdma: __DMA_HandleTypeDef);  (*!< DMA transfer complete Memory1 callback  *)
    XferErrorCallback: procedure(var hdma: __DMA_HandleTypeDef);   (*!< DMA transfer error callback             *)
    ErrorCode: longword;  (*!< DMA Error code                           *)
  end;
  DMA_HandleTypeDef = __DMA_HandleTypeDef;

  (**
  * @end;
   *)


(* Exported constants -------------------------------------------------------- *)

  (** @defgroup DMA_Exported_Constants DMA Exported Constants
  * @brief    DMA Exported constants
  * @begin
   *)

  (** @defgroup DMA_Error_Code DMA Error Code
  * @brief    DMA Error Code
  * @begin
   *)

const
  HAL_DMA_ERROR_NONE    = ($00000000);  (*!< No error              *)
  HAL_DMA_ERROR_TE      = ($00000001);  (*!< Transfer error        *)
  HAL_DMA_ERROR_FE      = ($00000002);  (*!< FIFO error            *)
  HAL_DMA_ERROR_DME     = ($00000004);  (*!< Direct Mode error     *)
  HAL_DMA_ERROR_TIMEOUT = ($00000020);  (*!< Timeout error         *)
  (**
  * @end;
   *)

  (** @defgroup DMA_Channel_selection DMA Channel selection
  * @brief    DMA channel selection
  * @begin
   *)

  DMA_CHANNEL_0 = ($00000000);  (*!< DMA Channel 0  *)
  DMA_CHANNEL_1 = ($02000000);  (*!< DMA Channel 1  *)
  DMA_CHANNEL_2 = ($04000000);  (*!< DMA Channel 2  *)
  DMA_CHANNEL_3 = ($06000000);  (*!< DMA Channel 3  *)
  DMA_CHANNEL_4 = ($08000000);  (*!< DMA Channel 4  *)
  DMA_CHANNEL_5 = ($0A000000);  (*!< DMA Channel 5  *)
  DMA_CHANNEL_6 = ($0C000000);  (*!< DMA Channel 6  *)
  DMA_CHANNEL_7 = ($0E000000);  (*!< DMA Channel 7  *)
  (**
  * @end;
   *)

  (** @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @brief    DMA data transfer direction
  * @begin
   *)

  DMA_PERIPH_TO_MEMORY = ($00000000);  (*!< Peripheral to memory direction  *)
  DMA_MEMORY_TO_PERIPH = (DMA_SxCR_DIR_0);  (*!< Memory to peripheral direction  *)
  DMA_MEMORY_TO_MEMORY = (DMA_SxCR_DIR_1);  (*!< Memory to memory direction      *)
  (**
  * @end;
   *)

  (** @defgroup DMA_Peripheral_incremented_mode DMA Peripheral incremented mode
  * @brief    DMA peripheral incremented mode
  * @begin
   *)

  DMA_PINC_ENABLE  = (DMA_SxCR_PINC);  (*!< Peripheral increment mode enable   *)
  DMA_PINC_DISABLE = ($00000000);  (*!< Peripheral increment mode disable  *)
  (**
  * @end;
   *)

  (** @defgroup DMA_Memory_incremented_mode DMA Memory incremented mode
  * @brief    DMA memory incremented mode
  * @begin
   *)

  DMA_MINC_ENABLE  = (DMA_SxCR_MINC);  (*!< Memory increment mode enable   *)
  DMA_MINC_DISABLE = ($00000000);  (*!< Memory increment mode disable  *)
  (**
  * @end;
   *)


  (** @defgroup DMA_Peripheral_data_size DMA Peripheral data size
  * @brief    DMA peripheral data size
  * @begin
   *)

  DMA_PDATAALIGN_BYTE     = ($00000000);  (*!< Peripheral data alignment: Byte      *)
  DMA_PDATAALIGN_HALFWORD = (DMA_SxCR_PSIZE_0);  (*!< Peripheral data alignment: HalfWord  *)
  DMA_PDATAALIGN_WORD     = (DMA_SxCR_PSIZE_1);  (*!< Peripheral data alignment: Word      *)
  (**
  * @end;
   *)


  (** @defgroup DMA_Memory_data_size DMA Memory data size
  * @brief    DMA memory data size
  * @begin
   *)

  DMA_MDATAALIGN_BYTE     = ($00000000);  (*!< Memory data alignment: Byte      *)
  DMA_MDATAALIGN_HALFWORD = (DMA_SxCR_MSIZE_0);  (*!< Memory data alignment: HalfWord  *)
  DMA_MDATAALIGN_WORD     = (DMA_SxCR_MSIZE_1);  (*!< Memory data alignment: Word      *)
  (**
  * @end;
   *)

  (** @defgroup DMA_mode DMA mode
  * @brief    DMA mode
  * @begin
   *)

  DMA_NORMAL   = ($00000000);  (*!< Normal mode                   *)
  DMA_CIRCULAR = (DMA_SxCR_CIRC);  (*!< Circular mode                 *)
  DMA_PFCTRL   = (DMA_SxCR_PFCTRL);  (*!< Peripheral flow control mode  *)
  (**
  * @end;
   *)


  (** @defgroup DMA_Priority_level DMA Priority level
  * @brief    DMA priority levels
  * @begin
   *)

  DMA_PRIORITY_LOW       = ($00000000);  (*!< Priority level: Low        *)
  DMA_PRIORITY_MEDIUM    = (DMA_SxCR_PL_0);  (*!< Priority level: Medium     *)
  DMA_PRIORITY_HIGH      = (DMA_SxCR_PL_1);  (*!< Priority level: High       *)
  DMA_PRIORITY_VERY_HIGH = (DMA_SxCR_PL);  (*!< Priority level: Very High  *)
  (**
  * @end;
   *)


  (** @defgroup DMA_FIFO_direct_mode DMA FIFO direct mode
  * @brief    DMA FIFO direct mode
  * @begin
   *)

  DMA_FIFOMODE_DISABLE = ($00000000);  (*!< FIFO mode disable  *)
  DMA_FIFOMODE_ENABLE  = (DMA_SxFCR_DMDIS);  (*!< FIFO mode enable   *)
  (**
  * @end;
   *)

  (** @defgroup DMA_FIFO_threshold_level DMA FIFO threshold level
  * @brief    DMA FIFO level
  * @begin
   *)

  DMA_FIFO_THRESHOLD_1QUARTERFULL = ($00000000);  (*!< FIFO threshold 1 quart full configuration   *)
  DMA_FIFO_THRESHOLD_HALFFULL = (DMA_SxFCR_FTH_0);  (*!< FIFO threshold half full configuration      *)
  DMA_FIFO_THRESHOLD_3QUARTERSFULL = (DMA_SxFCR_FTH_1);  (*!< FIFO threshold 3 quarts full configuration  *)
  DMA_FIFO_THRESHOLD_FULL = (DMA_SxFCR_FTH);  (*!< FIFO threshold full configuration           *)
  (**
  * @end;
   *)

  (** @defgroup DMA_Memory_burst DMA Memory burst
  * @brief    DMA memory burst
  * @begin
   *)

  DMA_MBURST_SINGLE = ($00000000);
  DMA_MBURST_INC4   = (DMA_SxCR_MBURST_0);
  DMA_MBURST_INC8   = (DMA_SxCR_MBURST_1);
  DMA_MBURST_INC16  = (DMA_SxCR_MBURST);
  (**
  * @end;
   *)


  (** @defgroup DMA_Peripheral_burst DMA Peripheral burst
  * @brief    DMA peripheral burst
  * @begin
   *)

  DMA_PBURST_SINGLE = ($00000000);
  DMA_PBURST_INC4   = (DMA_SxCR_PBURST_0);
  DMA_PBURST_INC8   = (DMA_SxCR_PBURST_1);
  DMA_PBURST_INC16  = (DMA_SxCR_PBURST);
  (**
  * @end;
   *)

  (** @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @brief    DMA interrupts definition
  * @begin
   *)

  DMA_IT_TC  = (DMA_SxCR_TCIE);
  DMA_IT_HT  = (DMA_SxCR_HTIE);
  DMA_IT_TE  = (DMA_SxCR_TEIE);
  DMA_IT_DME = (DMA_SxCR_DMEIE);
  DMA_IT_FE  = ($00000080);
  (**
  * @end;
   *)

  (** @defgroup DMA_flag_definitions DMA flag definitions
  * @brief    DMA flag definitions
  * @begin
   *)

  DMA_FLAG_FEIF0_4  = ($00800001);
  DMA_FLAG_DMEIF0_4 = ($00800004);
  DMA_FLAG_TEIF0_4  = ($00000008);
  DMA_FLAG_HTIF0_4  = ($00000010);
  DMA_FLAG_TCIF0_4  = ($00000020);
  DMA_FLAG_FEIF1_5  = ($00000040);
  DMA_FLAG_DMEIF1_5 = ($00000100);
  DMA_FLAG_TEIF1_5  = ($00000200);
  DMA_FLAG_HTIF1_5  = ($00000400);
  DMA_FLAG_TCIF1_5  = ($00000800);
  DMA_FLAG_FEIF2_6  = ($00010000);
  DMA_FLAG_DMEIF2_6 = ($00040000);
  DMA_FLAG_TEIF2_6  = ($00080000);
  DMA_FLAG_HTIF2_6  = ($00100000);
  DMA_FLAG_TCIF2_6  = ($00200000);
  DMA_FLAG_FEIF3_7  = ($00400000);
  DMA_FLAG_DMEIF3_7 = ($01000000);
  DMA_FLAG_TEIF3_7  = ($02000000);
  DMA_FLAG_HTIF3_7  = ($04000000);
  DMA_FLAG_TCIF3_7  = ($08000000);

function HAL_DMA_Init(var hdma: DMA_HandleTypeDef): HAL_StatusTypeDef;
function HAL_DMA_DeInit(var hdma: DMA_HandleTypeDef): HAL_StatusTypeDef;

(**
    * @end;
     *)
(** @defgroup DMA_Exported_Functions_Group2 I/O operation functions
    * @brief   I/O operation functions
    * @begin
     *)
function HAL_DMA_Start(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress: pointer; DataLength: longword): HAL_StatusTypeDef;
function HAL_DMA_Start_IT(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress: pointer; DataLength: longword): HAL_StatusTypeDef;
function HAL_DMA_Abort(var hdma: DMA_HandleTypeDef): HAL_StatusTypeDef;
function HAL_DMA_PollForTransfer(var hdma: DMA_HandleTypeDef; CompleteLevel, Timeout: longword): HAL_StatusTypeDef;
procedure HAL_DMA_IRQHandler(var hdma: DMA_HandleTypeDef);

(**
    * @end;
     *)

(** @defgroup DMA_Exported_Functions_Group3 Peripheral State functions
    * @brief    Peripheral State functions
    * @begin
     *)
function HAL_DMA_GetState(var hdma: DMA_HandleTypeDef): HAL_DMA_StateTypeDef;
function HAL_DMA_GetError(var hdma: DMA_HandleTypeDef): longword;

implementation

const
  HAL_TIMEOUT_DMA_ABORT = (1000);  (* 1s  *)


function __HAL_DMA_GET_IT_SOURCE(var __HANDLE__: DMA_HandleTypeDef; __INTERRUPT__: longword): boolean;
  begin
    if __INTERRUPT__ <> DMA_IT_FE then
      exit((__HANDLE__.Instance^.CR and __INTERRUPT__) <> 0)
    else
      exit((__HANDLE__.Instance^.FCR and __INTERRUPT__) <> 0);
  end;

procedure __HAL_DMA_ENABLE_IT(var __HANDLE__: DMA_HandleTypeDef; __INTERRUPT__: longword);
  begin
    if __INTERRUPT__ <> DMA_IT_FE then
      __HANDLE__.Instance^.CR := __HANDLE__.Instance^.CR or __INTERRUPT__
    else
      __HANDLE__.Instance^.FCR := __HANDLE__.Instance^.FCR or __INTERRUPT__;
  end;

procedure __HAL_DMA_DISABLE_IT(var __HANDLE__: DMA_HandleTypeDef; __INTERRUPT__: longword);
  begin
    if __INTERRUPT__ <> DMA_IT_FE then
      __HANDLE__.Instance^.CR := __HANDLE__.Instance^.CR and (not __INTERRUPT__)
    else
      __HANDLE__.Instance^.FCR := __HANDLE__.Instance^.FCR and (not __INTERRUPT__);
  end;

procedure __HAL_DMA_CLEAR_FLAG(var __HANDLE__: DMA_HandleTypeDef; __FLAG__: longword);
  begin
    if ptruint(__HANDLE__.Instance) > ptruint(@DMA2.S3CR) then
      DMA2.HIFCR := __FLAG__
    else if ptruint(__HANDLE__.Instance) > ptruint(@DMA1.S7CR) then
        DMA2.LIFCR := __FLAG__
      else if ptruint(__HANDLE__.Instance) > ptruint(@DMA1.S3CR) then
          DMA1.HIFCR := __FLAG__
        else
          DMA1.LIFCR := __FLAG__;
  end;

function __HAL_DMA_GET_DME_FLAG_INDEX(var __HANDLE__: DMA_HandleTypeDef): longword;
  begin
    if __HANDLE__.Instance = @dma1.S0CR then
      exit(DMA_FLAG_DMEIF0_4)
    else if __HANDLE__.Instance = @dma2.S0CR then
        exit(DMA_FLAG_DMEIF0_4)
      else if __HANDLE__.Instance = @dma1.S4CR then
          exit(DMA_FLAG_DMEIF0_4)
        else if __HANDLE__.Instance = @dma2.S4CR then
            exit(DMA_FLAG_DMEIF0_4)

          else if __HANDLE__.Instance = @dma1.S1CR then
              exit(DMA_FLAG_DMEIF1_5)
            else if __HANDLE__.Instance = @dma2.S1CR then
                exit(DMA_FLAG_DMEIF1_5)
              else if __HANDLE__.Instance = @dma1.S5CR then
                  exit(DMA_FLAG_DMEIF1_5)
                else if __HANDLE__.Instance = @dma2.S5CR then
                    exit(DMA_FLAG_DMEIF1_5)

                  else if __HANDLE__.Instance = @dma1.S2CR then
                      exit(DMA_FLAG_DMEIF2_6)
                    else if __HANDLE__.Instance = @dma2.S2CR then
                        exit(DMA_FLAG_DMEIF2_6)
                      else if __HANDLE__.Instance = @dma1.S6CR then
                          exit(DMA_FLAG_DMEIF2_6)
                        else if __HANDLE__.Instance = @dma2.S6CR then
                            exit(DMA_FLAG_DMEIF2_6)

                          else
                            exit(DMA_FLAG_DMEIF3_7);
  end;

function __HAL_DMA_GET_TC_FLAG_INDEX(var __HANDLE__: DMA_HandleTypeDef): longword;
  begin
    if __HANDLE__.Instance = @dma1.S0CR then
      exit(DMA_FLAG_TCIF0_4)
    else if __HANDLE__.Instance = @dma2.S0CR then
        exit(DMA_FLAG_TCIF0_4)
      else if __HANDLE__.Instance = @dma1.S4CR then
          exit(DMA_FLAG_TCIF0_4)
        else if __HANDLE__.Instance = @dma2.S4CR then
            exit(DMA_FLAG_TCIF0_4)

          else if __HANDLE__.Instance = @dma1.S1CR then
              exit(DMA_FLAG_TCIF1_5)
            else if __HANDLE__.Instance = @dma2.S1CR then
                exit(DMA_FLAG_TCIF1_5)
              else if __HANDLE__.Instance = @dma1.S5CR then
                  exit(DMA_FLAG_TCIF1_5)
                else if __HANDLE__.Instance = @dma2.S5CR then
                    exit(DMA_FLAG_TCIF1_5)

                  else if __HANDLE__.Instance = @dma1.S2CR then
                      exit(DMA_FLAG_TCIF2_6)
                    else if __HANDLE__.Instance = @dma2.S2CR then
                        exit(DMA_FLAG_TCIF2_6)
                      else if __HANDLE__.Instance = @dma1.S6CR then
                          exit(DMA_FLAG_TCIF2_6)
                        else if __HANDLE__.Instance = @dma2.S6CR then
                            exit(DMA_FLAG_TCIF2_6)

                          else
                            exit(DMA_FLAG_TCIF3_7);
  end;

function __HAL_DMA_GET_TE_FLAG_INDEX(var __HANDLE__: DMA_HandleTypeDef): longword;
  begin
    if __HANDLE__.Instance = @dma1.S0CR then
      exit(DMA_FLAG_TEIF0_4)
    else if __HANDLE__.Instance = @dma2.S0CR then
        exit(DMA_FLAG_TEIF0_4)
      else if __HANDLE__.Instance = @dma1.S4CR then
          exit(DMA_FLAG_TEIF0_4)
        else if __HANDLE__.Instance = @dma2.S4CR then
            exit(DMA_FLAG_TEIF0_4)
          else if __HANDLE__.Instance = @dma1.S1CR then
              exit(DMA_FLAG_TEIF1_5)
            else if __HANDLE__.Instance = @dma2.S1CR then
                exit(DMA_FLAG_TEIF1_5)
              else if __HANDLE__.Instance = @dma1.S5CR then
                  exit(DMA_FLAG_TEIF1_5)
                else if __HANDLE__.Instance = @dma2.S5CR then
                    exit(DMA_FLAG_TEIF1_5)
                  else if __HANDLE__.Instance = @dma1.S2CR then
                      exit(DMA_FLAG_TEIF2_6)
                    else if __HANDLE__.Instance = @dma2.S2CR then
                        exit(DMA_FLAG_TEIF2_6)
                      else if __HANDLE__.Instance = @dma1.S6CR then
                          exit(DMA_FLAG_TEIF2_6)
                        else if __HANDLE__.Instance = @dma2.S6CR then
                            exit(DMA_FLAG_TEIF2_6)
                          else
                            exit(DMA_FLAG_TEIF3_7);
  end;

function __HAL_DMA_GET_FE_FLAG_INDEX(var __HANDLE__: DMA_HandleTypeDef): longword;
  begin
    if __HANDLE__.Instance = @dma1.S0CR then
      exit(DMA_FLAG_FEIF0_4)
    else if __HANDLE__.Instance = @dma2.S0CR then
        exit(DMA_FLAG_FEIF0_4)
      else if __HANDLE__.Instance = @dma1.S4CR then
          exit(DMA_FLAG_FEIF0_4)
        else if __HANDLE__.Instance = @dma2.S4CR then
            exit(DMA_FLAG_FEIF0_4)
          else if __HANDLE__.Instance = @dma1.S1CR then
              exit(DMA_FLAG_FEIF1_5)
            else if __HANDLE__.Instance = @dma2.S1CR then
                exit(DMA_FLAG_FEIF1_5)
              else if __HANDLE__.Instance = @dma1.S5CR then
                  exit(DMA_FLAG_FEIF1_5)
                else if __HANDLE__.Instance = @dma2.S5CR then
                    exit(DMA_FLAG_FEIF1_5)
                  else if __HANDLE__.Instance = @dma1.S2CR then
                      exit(DMA_FLAG_FEIF2_6)
                    else if __HANDLE__.Instance = @dma2.S2CR then
                        exit(DMA_FLAG_FEIF2_6)
                      else if __HANDLE__.Instance = @dma1.S6CR then
                          exit(DMA_FLAG_FEIF2_6)
                        else if __HANDLE__.Instance = @dma2.S6CR then
                            exit(DMA_FLAG_FEIF2_6)
                          else
                            exit(DMA_FLAG_FEIF3_7);
  end;

function __HAL_DMA_GET_HT_FLAG_INDEX(var __HANDLE__: DMA_HandleTypeDef): longword;
  begin
    if __HANDLE__.Instance = @dma1.S0CR then
      exit(DMA_FLAG_HTIF0_4)
    else if __HANDLE__.Instance = @dma2.S0CR then
        exit(DMA_FLAG_HTIF0_4)
      else if __HANDLE__.Instance = @dma1.S4CR then
          exit(DMA_FLAG_HTIF0_4)
        else if __HANDLE__.Instance = @dma2.S4CR then
            exit(DMA_FLAG_HTIF0_4)
          else if __HANDLE__.Instance = @dma1.S1CR then
              exit(DMA_FLAG_HTIF1_5)
            else if __HANDLE__.Instance = @dma2.S1CR then
                exit(DMA_FLAG_HTIF1_5)
              else if __HANDLE__.Instance = @dma1.S5CR then
                  exit(DMA_FLAG_HTIF1_5)
                else if __HANDLE__.Instance = @dma2.S5CR then
                    exit(DMA_FLAG_HTIF1_5)
                  else if __HANDLE__.Instance = @dma1.S2CR then
                      exit(DMA_FLAG_HTIF2_6)
                    else if __HANDLE__.Instance = @dma2.S2CR then
                        exit(DMA_FLAG_HTIF2_6)
                      else if __HANDLE__.Instance = @dma1.S6CR then
                          exit(DMA_FLAG_HTIF2_6)
                        else if __HANDLE__.Instance = @dma2.S6CR then
                            exit(DMA_FLAG_HTIF2_6)
                          else
                            exit(DMA_FLAG_HTIF3_7);
  end;

function __HAL_DMA_GET_FLAG(var __HANDLE__: DMA_HandleTypeDef; __FLAG__: longword): boolean;
  begin
    if ptruint(__HANDLE__.Instance) > ptruint(@DMA2.S3CR) then
      exit((DMA2.HISR and __FLAG__) <> 0)
    else if ptruint(__HANDLE__.Instance) > ptruint(@DMA1.S7CR) then
        exit((DMA2.LISR and __FLAG__) <> 0)
      else if ptruint(__HANDLE__.Instance) > ptruint(@DMA1.S3CR) then
          exit((DMA1.HISR and __FLAG__) <> 0)
        else
          exit((DMA1.LISR and __FLAG__) <> 0);
  end;

procedure __HAL_DMA_ENABLE(var __HANDLE__: DMA_HandleTypeDef);
  begin
    __HANDLE__.Instance^.CR := __HANDLE__.Instance^.CR or DMA_SxCR_EN;
  end;

procedure __HAL_DMA_DISABLE(var __HANDLE__: DMA_HandleTypeDef);
  begin
    __HANDLE__.Instance^.CR := __HANDLE__.Instance^.CR and (not DMA_SxCR_EN);
  end;

procedure DMA_SetConfig(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress: pointer; DataLength: longword);
  begin
    (* Clear DBM bit *)
    hdma.Instance^.CR := hdma.Instance^.CR and ((not DMA_SxCR_DBM));

    (* Configure DMA Stream data length *)
    hdma.Instance^.NDTR := DataLength;

    (* Peripheral to Memory *)
    if ((hdma.Init.Direction) = DMA_MEMORY_TO_PERIPH) then
      begin
      (* Configure DMA Stream destination address *)
      hdma.Instance^.PAR := ptruint(DstAddress);

      (* Configure DMA Stream source address *)
      hdma.Instance^.M0AR := ptruint(SrcAddress);
      end
    (* Memory to Peripheral *)
    else
      begin
      (* Configure DMA Stream source address *)
      hdma.Instance^.PAR := ptruint(SrcAddress);

      (* Configure DMA Stream destination address *)
      hdma.Instance^.M0AR := ptruint(DstAddress);
      end;
  end;

function HAL_DMA_Init(var hdma: DMA_HandleTypeDef): HAL_StatusTypeDef;
  var
    tmp: longword;
  begin
    tmp := 0;

    (* Change DMA peripheral state *)
    hdma.State := HAL_DMA_STATE_BUSY;

    (* Get the CR register value *)
    tmp := hdma.Instance^.CR;

    (* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, CT and DBM bits *)
    tmp := tmp and ((not (DMA_SxCR_CHSEL or DMA_SxCR_MBURST or DMA_SxCR_PBURST or DMA_SxCR_PL or DMA_SxCR_MSIZE or DMA_SxCR_PSIZE or DMA_SxCR_MINC or DMA_SxCR_PINC or DMA_SxCR_CIRC or
      DMA_SxCR_DIR or DMA_SxCR_CT or DMA_SxCR_DBM)));

    (* Prepare the DMA Stream configuration *)
    tmp := tmp or (hdma.Init.Channel or hdma.Init.Direction or hdma.Init.PeriphInc or hdma.Init.MemInc or hdma.Init.PeriphDataAlignment or
      hdma.Init.MemDataAlignment or hdma.Init.Mode or hdma.Init.Priority);

    (* the Memory burst and peripheral burst are not used when the FIFO is disabled *)
    if (hdma.Init.FIFOMode = DMA_FIFOMODE_ENABLE) then
      begin
      (* Get memory burst and peripheral burst *)
      tmp := tmp or (hdma.Init.MemBurst or hdma.Init.PeriphBurst);
      end;

    (* Write to DMA Stream CR register *)
    hdma.Instance^.CR := tmp;

    (* Get the FCR register value *)
    tmp := hdma.Instance^.FCR;

    (* Clear Direct mode and FIFO threshold bits *)
    tmp := tmp and (not (DMA_SxFCR_DMDIS or DMA_SxFCR_FTH));

    (* Prepare the DMA Stream FIFO configuration *)
    tmp := tmp or (hdma.Init.FIFOMode);

    (* the FIFO threshold is not used when the FIFO mode is disabled *)
    if (hdma.Init.FIFOMode = DMA_FIFOMODE_ENABLE) then
      begin
      (* Get the FIFO threshold *)
      tmp := tmp or (hdma.Init.FIFOThreshold);
      end;

    (* Write to DMA Stream FCR *)
    hdma.Instance^.FCR := tmp;

    (* Initialize the error code *)
    hdma.ErrorCode := HAL_DMA_ERROR_NONE;

    (* Initialize the DMA state *)
    hdma.State := HAL_DMA_STATE_READY;

    exit(HAL_OK);
  end;

function HAL_DMA_DeInit(var hdma: DMA_HandleTypeDef): HAL_StatusTypeDef;
  begin
    (* Check the DMA peripheral state *)
    if (hdma.State = HAL_DMA_STATE_BUSY) then
      begin
      exit(HAL_ERROR);
      end;

    (* Disable the selected DMA Streamx *)
    __HAL_DMA_DISABLE(hdma);

    (* Reset DMA Streamx control register *)
    hdma.Instance^.CR := 0;

    (* Reset DMA Streamx number of data to transfer register *)
    hdma.Instance^.NDTR := 0;

    (* Reset DMA Streamx peripheral address register *)
    hdma.Instance^.PAR := 0;

    (* Reset DMA Streamx memory 0 address register *)
    hdma.Instance^.M0AR := 0;

    (* Reset DMA Streamx memory 1 address register *)
    hdma.Instance^.M1AR := 0;

    (* Reset DMA Streamx FIFO control register *)
    hdma.Instance^.FCR := $00000021;

    (* Clear all flags *)
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));

    (* Initialize the error code *)
    hdma.ErrorCode := HAL_DMA_ERROR_NONE;

    (* Initialize the DMA state *)
    hdma.State := HAL_DMA_STATE_RESET;

    (* Release Lock *)
    __HAL_UNLOCK(hdma.Lock);

    exit(HAL_OK);
  end;

function HAL_DMA_Start(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress: pointer; DataLength: longword): HAL_StatusTypeDef;
  begin
    (* Process locked *)
    __HAL_LOCK(hdma.lock);

    (* Change DMA peripheral state *)
    hdma.State := HAL_DMA_STATE_BUSY;

    (* Disable the peripheral *)
    __HAL_DMA_DISABLE(hdma);

    (* Configure the source, destination address and the data length *)
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

    (* Enable the Peripheral *)
    __HAL_DMA_ENABLE(hdma);

    exit(HAL_OK);
  end;

function HAL_DMA_Start_IT(var hdma: DMA_HandleTypeDef; SrcAddress, DstAddress: pointer; DataLength: longword): HAL_StatusTypeDef;
  begin
    (* Process locked *)
    __HAL_LOCK(hdma.lock);

    (* Change DMA peripheral state *)
    hdma.State := HAL_DMA_STATE_BUSY;

    (* Disable the peripheral *)
    __HAL_DMA_DISABLE(hdma);

    (* Configure the source, destination address and the data length *)
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

    (* Enable the transfer complete interrupt *)
    __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);

    (* Enable the Half transfer complete interrupt *)
    __HAL_DMA_ENABLE_IT(hdma, DMA_IT_HT);

    (* Enable the transfer Error interrupt *)
    __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TE);

    (* Enable the FIFO Error interrupt *)
    __HAL_DMA_ENABLE_IT(hdma, DMA_IT_FE);

    (* Enable the direct mode Error interrupt *)
    __HAL_DMA_ENABLE_IT(hdma, DMA_IT_DME);

    (* Enable the Peripheral *)
    __HAL_DMA_ENABLE(hdma);

    exit(HAL_OK);
  end;

function HAL_DMA_Abort(var hdma: DMA_HandleTypeDef): HAL_StatusTypeDef;
  var
    tickstart: longword;
  begin
    tickstart := 0;

    (* Disable the stream *)
    __HAL_DMA_DISABLE(hdma);

    (* Get tick *)
    tickstart := HAL_GetTick();

    (* Check if the DMA Stream is effectively disabled *)
    while ((hdma.Instance^.CR and DMA_SxCR_EN) <> 0) do
      begin
      (* Check for the Timeout *)
      if ((HAL_GetTick() - tickstart) > HAL_TIMEOUT_DMA_ABORT) then
        begin
        (* Update error code *)
        hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_TIMEOUT);

        (* Process Unlocked *)
        __HAL_UNLOCK(hdma.Lock);

        (* Change the DMA state *)
        hdma.State := HAL_DMA_STATE_TIMEOUT;

        exit(HAL_TIMEOUT);
        end;
      end;
    (* Process Unlocked *)
    __HAL_UNLOCK(hdma.Lock);

    (* Change the DMA state*)
    hdma.State := HAL_DMA_STATE_READY;

    exit(HAL_OK);
  end;

function HAL_DMA_PollForTransfer(var hdma: DMA_HandleTypeDef; CompleteLevel, Timeout: longword): HAL_StatusTypeDef;
  var
    tmp, tmp1, tmp2: boolean;
    temp, tickstart: longword;
  begin
    tickstart := 0;

    (* Get the level transfer complete flag *)
    if (CompleteLevel = HAL_DMA_FULL_TRANSFER) then
      begin
      (* Transfer Complete flag *)
      temp := __HAL_DMA_GET_TC_FLAG_INDEX(hdma);
      end
    else
      begin
      (* Half Transfer Complete flag *)
      temp := __HAL_DMA_GET_HT_FLAG_INDEX(hdma);
      end;

    (* Get tick *)
    tickstart := HAL_GetTick();

    while not __HAL_DMA_GET_FLAG(hdma, temp) do
      begin
      tmp := __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
      tmp1 := __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));
      tmp2 := __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));
      if tmp or tmp1 or tmp2 then
        begin
        if tmp then
          begin
          (* Update error code *)
          hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_TE);

          (* Clear the transfer error flag *)
          __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
          end;
        if tmp1 then
          begin
          (* Update error code *)
          hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_FE);

          (* Clear the FIFO error flag *)
          __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));
          end;
        if tmp2 then
          begin
          (* Update error code *)
          hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_DME);

          (* Clear the Direct Mode error flag *)
          __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));
          end;
        (* Change the DMA state *)
        hdma.State := HAL_DMA_STATE_ERROR;

        (* Process Unlocked *)
        __HAL_UNLOCK(hdma.lock);

        exit(HAL_ERROR);
        end;
      (* Check for the Timeout *)
      if (Timeout <> HAL_MAX_DELAY) then
        begin
        if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
          begin
          (* Update error code *)
          hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_TIMEOUT);

          (* Change the DMA state *)
          hdma.State := HAL_DMA_STATE_TIMEOUT;

          (* Process Unlocked *)
          __HAL_UNLOCK(hdma.lock);

          exit(HAL_TIMEOUT);
          end;
        end;
      end;

    if (CompleteLevel = HAL_DMA_FULL_TRANSFER) then
      begin
      (* Multi_Buffering mode enabled *)
      if (((hdma.Instance^.CR) and (DMA_SxCR_DBM)) <> 0) then
        begin
        (* Clear the half transfer complete flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
        (* Clear the transfer complete flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

        (* Current memory buffer used is Memory 0 *)
        if ((hdma.Instance^.CR and DMA_SxCR_CT) = 0) then
          begin
          (* Change DMA peripheral state *)
          hdma.State := HAL_DMA_STATE_READY_MEM0;
          end
        (* Current memory buffer used is Memory 1 *)
        else if ((hdma.Instance^.CR and DMA_SxCR_CT) <> 0) then
            begin
            (* Change DMA peripheral state *)
            hdma.State := HAL_DMA_STATE_READY_MEM1;
            end;
        end
      else
        begin
        (* Clear the half transfer complete flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
        (* Clear the transfer complete flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

      (* The selected Streamx EN bit is cleared (DMA is disabled and all transfers
         are complete) *)
        hdma.State := HAL_DMA_STATE_READY_MEM0;
        end;
      (* Process Unlocked *)
      __HAL_UNLOCK(hdma.lock);
      end
    else
      begin
      (* Multi_Buffering mode enabled *)
      if (((hdma.Instance^.CR) and (DMA_SxCR_DBM)) <> 0) then
        begin
        (* Clear the half transfer complete flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));

        (* Current memory buffer used is Memory 0 *)
        if ((hdma.Instance^.CR and DMA_SxCR_CT) = 0) then
          begin
          (* Change DMA peripheral state *)
          hdma.State := HAL_DMA_STATE_READY_HALF_MEM0;
          end
        (* Current memory buffer used is Memory 1 *)
        else if ((hdma.Instance^.CR and DMA_SxCR_CT) <> 0) then
            begin
            (* Change DMA peripheral state *)
            hdma.State := HAL_DMA_STATE_READY_HALF_MEM1;
            end;
        end
      else
        begin
        (* Clear the half transfer complete flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));

        (* Change DMA peripheral state *)
        hdma.State := HAL_DMA_STATE_READY_HALF_MEM0;
        end;
      end;
    exit(HAL_OK);
  end;

procedure HAL_DMA_IRQHandler(var hdma: DMA_HandleTypeDef);
  begin
    (* Transfer Error Interrupt management ***************************************)
    if __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma)) then
      begin
      if __HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TE) then
        begin
        (* Disable the transfer error interrupt *)
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TE);

        (* Clear the transfer error flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));

        (* Update error code *)
        hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_TE);

        (* Change the DMA state *)
        hdma.State := HAL_DMA_STATE_ERROR;

        (* Process Unlocked *)
        __HAL_UNLOCK(hdma.lock);

        if (hdma.XferErrorCallback <> nil) then
          begin
          (* Transfer error callback *)
          hdma.XferErrorCallback(hdma);
          end;
        end;
      end;
    (* FIFO Error Interrupt management ******************************************)
    if __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma)) then
      begin
      if __HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_FE) then
        begin
        (* Disable the FIFO Error interrupt *)
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_FE);

        (* Clear the FIFO error flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));

        (* Update error code *)
        hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_FE);

        (* Change the DMA state *)
        hdma.State := HAL_DMA_STATE_ERROR;

        (* Process Unlocked *)
        __HAL_UNLOCK(hdma.lock);

        if (hdma.XferErrorCallback <> nil) then
          begin
          (* Transfer error callback *)
          hdma.XferErrorCallback(hdma);
          end;
        end;
      end;
    (* Direct Mode Error Interrupt management ***********************************)
    if __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma)) then
      begin
      if __HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_DME) then
        begin
        (* Disable the direct mode Error interrupt *)
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_DME);

        (* Clear the direct mode error flag *)
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));

        (* Update error code *)
        hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_DME);

        (* Change the DMA state *)
        hdma.State := HAL_DMA_STATE_ERROR;

        (* Process Unlocked *)
        __HAL_UNLOCK(hdma.lock);

        if (hdma.XferErrorCallback <> nil) then
          begin
          (* Transfer error callback *)
          hdma.XferErrorCallback(hdma);
          end;
        end;
      end;
    (* Half Transfer Complete Interrupt management ******************************)
    if __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma)) then
      begin
      if __HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_HT) then
        begin
        (* Multi_Buffering mode enabled *)
        if (((hdma.Instance^.CR) and (DMA_SxCR_DBM)) <> 0) then
          begin
          (* Clear the half transfer complete flag *)
          __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));

          (* Current memory buffer used is Memory 0 *)
          if ((hdma.Instance^.CR and DMA_SxCR_CT) = 0) then
            begin
            (* Change DMA peripheral state *)
            hdma.State := HAL_DMA_STATE_READY_HALF_MEM0;
            end
          (* Current memory buffer used is Memory 1 *)
          else if ((hdma.Instance^.CR and DMA_SxCR_CT) <> 0) then
              begin
              (* Change DMA peripheral state *)
              hdma.State := HAL_DMA_STATE_READY_HALF_MEM1;
              end;
          end
        else
          begin
          (* Disable the half transfer interrupt if the DMA mode is not CIRCULAR *)
          if ((hdma.Instance^.CR and DMA_SxCR_CIRC) = 0) then
            begin
            (* Disable the half transfer interrupt *)
            __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
            end;
          (* Clear the half transfer complete flag *)
          __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));

          (* Change DMA peripheral state *)
          hdma.State := HAL_DMA_STATE_READY_HALF_MEM0;
          end;

        if (hdma.XferHalfCpltCallback <> nil) then
          begin
          (* Half transfer callback *)
          hdma.XferHalfCpltCallback(hdma);
          end;
        end;
      end;
    (* Transfer Complete Interrupt management ***********************************)
    if __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma)) then
      begin
      if __HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) then
        begin
        if ((hdma.Instance^.CR) and (DMA_SxCR_DBM)) <> 0 then
          begin
          (* Clear the transfer complete flag *)
          __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

          (* Current memory buffer used is Memory 1 *)
          if ((hdma.Instance^.CR and DMA_SxCR_CT) = 0) then
            begin
            if (hdma.XferM1CpltCallback <> nil) then
              begin
              (* Transfer complete Callback for memory1 *)
              hdma.XferM1CpltCallback(hdma);
              end;
            end
          (* Current memory buffer used is Memory 0 *)
          else if ((hdma.Instance^.CR and DMA_SxCR_CT) <> 0) then
              begin
              if (hdma.XferCpltCallback <> nil) then
                begin
                (* Transfer complete Callback for memory0 *)
                hdma.XferCpltCallback(hdma);
                end;
              end;
          end
        (* Disable the transfer complete interrupt if the DMA mode is not CIRCULAR *)
        else
          begin
          if ((hdma.Instance^.CR and DMA_SxCR_CIRC) = 0) then
            begin
            (* Disable the transfer complete interrupt *)
            __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
            end;
          (* Clear the transfer complete flag *)
          __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

          (* Update error code *)
          hdma.ErrorCode := hdma.ErrorCode or (HAL_DMA_ERROR_NONE);

          (* Change the DMA state *)
          hdma.State := HAL_DMA_STATE_READY_MEM0;

          (* Process Unlocked *)
          __HAL_UNLOCK(hdma.lock);

          if (hdma.XferCpltCallback <> nil) then
            begin
            (* Transfer complete callback *)
            hdma.XferCpltCallback(hdma);
            end;
          end;
        end;
      end;
  end;

function HAL_DMA_GetState(var hdma: DMA_HandleTypeDef): HAL_DMA_StateTypeDef;
  begin
    exit(hdma.State);
  end;

function HAL_DMA_GetError(var hdma: DMA_HandleTypeDef): longword;
  begin
    exit(hdma.ErrorCode);
  end;

end.
