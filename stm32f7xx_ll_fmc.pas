(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_ll_fmc.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of FMC HAL module.
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
unit stm32f7xx_ll_fmc;

interface

uses
  stm32f7xx_hal,
  stm32f7xx_defs;

(** @defgroup FMC_Exported_typedef FMC Low Layer Exported Types
  * @{
   *)

type
  FMC_NORSRAM_TypeDef = FMC_Bank1_TypeDef;
  FMC_NORSRAM_EXTENDED_TypeDef = FMC_Bank1E_TypeDef;
  FMC_NAND_TypeDef = FMC_Bank3_TypeDef;
  FMC_SDRAM_TypeDef = FMC_Bank5_6_TypeDef;
{const
  FMC_NORSRAM_DEVICE = FMC_Bank1;
  FMC_NORSRAM_EXTENDED_DEVICE = FMC_Bank1E;
  FMC_NAND_DEVICE = FMC_Bank3;
  FMC_SDRAM_DEVICE = FMC_Bank5_6;}
  (**
  * @brief  FMC NORSRAM Configuration Structure definition
   *)

type
  FMC_NORSRAM_InitTypeDef = record
    NSBank: longword;  (*!< Specifies the NORSRAM memory device that will be used.
                                              This parameter can be a value of @ref FMC_NORSRAM_Bank                      *)
    DataAddressMux: longword;  (*!< Specifies whether the address and data values are
                                              multiplexed on the data bus or not.
                                              This parameter can be a value of @ref FMC_Data_Address_Bus_Multiplexing     *)
    MemoryType: longword;  (*!< Specifies the type of external memory attached to
                                              the corresponding memory device.
                                              This parameter can be a value of @ref FMC_Memory_Type                       *)
    MemoryDataWidth: longword;  (*!< Specifies the external memory device width.
                                              This parameter can be a value of @ref FMC_NORSRAM_Data_Width                *)
    BurstAccessMode: longword;  (*!< Enables or disables the burst access mode for Flash memory,
                                              valid only with synchronous burst Flash memories.
                                              This parameter can be a value of @ref FMC_Burst_Access_Mode                 *)
    WaitSignalPolarity: longword;  (*!< Specifies the wait signal polarity, valid only when accessing
                                              the Flash memory in burst mode.
                                              This parameter can be a value of @ref FMC_Wait_Signal_Polarity              *)
    WaitSignalActive: longword;  (*!< Specifies if the wait signal is asserted by the memory one
                                              clock cycle before the wait state or during the wait state,
                                              valid only when accessing memories in burst mode.
                                              This parameter can be a value of @ref FMC_Wait_Timing                       *)
    WriteOperation: longword;  (*!< Enables or disables the write operation in the selected device by the FMC.
                                              This parameter can be a value of @ref FMC_Write_Operation                   *)
    WaitSignal: longword;  (*!< Enables or disables the wait state insertion via wait
                                              signal, valid for Flash memory access in burst mode.
                                              This parameter can be a value of @ref FMC_Wait_Signal                       *)
    ExtendedMode: longword;  (*!< Enables or disables the extended mode.
                                              This parameter can be a value of @ref FMC_Extended_Mode                     *)
    AsynchronousWait: longword;  (*!< Enables or disables wait signal during asynchronous transfers,
                                              valid only with asynchronous Flash memories.
                                              This parameter can be a value of @ref FMC_AsynchronousWait                  *)
    WriteBurst: longword;  (*!< Enables or disables the write burst operation.
                                              This parameter can be a value of @ref FMC_Write_Burst                       *)
    ContinuousClock: longword;  (*!< Enables or disables the FMC clock output to external memory devices.
                                              This parameter is only enabled through the FMC_BCR1 register, and don't care
                                              through FMC_BCR2..4 registers.
                                              This parameter can be a value of @ref FMC_Continous_Clock                   *)
    WriteFifo: longword;  (*!< Enables or disables the write FIFO used by the FMC controller.
                                              This parameter is only enabled through the FMC_BCR1 register, and don't care
                                              through FMC_BCR2..4 registers.
                                              This parameter can be a value of @ref FMC_Write_FIFO                       *)
    PageSize: longword;  (*!< Specifies the memory page size.
                                              This parameter can be a value of @ref FMC_Page_Size                         *)
  end;

  (**
  * @brief  FMC NORSRAM Timing parameters structure definition
   *)

  FMC_NORSRAM_TimingTypeDef = record
    AddressSetupTime: longword;  (*!< Defines the number of HCLK cycles to configure
                                              the duration of the address setup time.
                                              This parameter can be a value between Min_Data = 0 and Max_Data = 15.
                                              @note This parameter is not used with synchronous NOR Flash memories.       *)
    AddressHoldTime: longword;  (*!< Defines the number of HCLK cycles to configure
                                              the duration of the address hold time.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 15.
                                              @note This parameter is not used with synchronous NOR Flash memories.       *)
    DataSetupTime: longword;  (*!< Defines the number of HCLK cycles to configure
                                              the duration of the data setup time.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 255.
                                              @note This parameter is used for SRAMs, ROMs and asynchronous multiplexed
                                              NOR Flash memories.                                                         *)
    BusTurnAroundDuration: longword;  (*!< Defines the number of HCLK cycles to configure
                                              the duration of the bus turnaround.
                                              This parameter can be a value between Min_Data = 0 and Max_Data = 15.
                                              @note This parameter is only used for multiplexed NOR Flash memories.       *)
    CLKDivision: longword;  (*!< Defines the period of CLK clock output signal, expressed in number of
                                              HCLK cycles. This parameter can be a value between Min_Data = 2 and Max_Data = 16.
                                              @note This parameter is not used for asynchronous NOR Flash, SRAM or ROM
                                              accesses.                                                                   *)
    DataLatency: longword;  (*!< Defines the number of memory clock cycles to issue
                                              to the memory before getting the first data.
                                              The parameter value depends on the memory type as shown below:
                                              - It must be set to 0 in case of a CRAM
                                              - It is don't care in asynchronous NOR, SRAM or ROM accesses
                                              - It may assume a value between Min_Data = 2 and Max_Data = 17 in NOR Flash memories
                                                with synchronous burst mode enable                                        *)
    AccessMode: longword;  (*!< Specifies the asynchronous access mode.
                                              This parameter can be a value of @ref FMC_Access_Mode                       *)
  end;

  (**
  * @brief  FMC NAND Configuration Structure definition
   *)

  FMC_NAND_InitTypeDef = record
    NandBank: longword;  (*!< Specifies the NAND memory device that will be used.
                                        This parameter can be a value of @ref FMC_NAND_Bank                     *)
    Waitfeature: longword;  (*!< Enables or disables the Wait feature for the NAND Memory device.
                                        This parameter can be any value of @ref FMC_Wait_feature                *)
    MemoryDataWidth: longword;  (*!< Specifies the external memory device width.
                                        This parameter can be any value of @ref FMC_NAND_Data_Width             *)
    EccComputation: longword;  (*!< Enables or disables the ECC computation.
                                        This parameter can be any value of @ref FMC_ECC                         *)
    ECCPageSize: longword;  (*!< Defines the page size for the extended ECC.
                                        This parameter can be any value of @ref FMC_ECC_Page_Size               *)
    TCLRSetupTime: longword;  (*!< Defines the number of HCLK cycles to configure the
                                        delay between CLE low and RE low.
                                        This parameter can be a value between Min_Data = 0 and Max_Data = 255   *)
    TARSetupTime: longword;  (*!< Defines the number of HCLK cycles to configure the
                                        delay between ALE low and RE low.
                                        This parameter can be a number between Min_Data = 0 and Max_Data = 255  *)
  end;

  (**
  * @brief  FMC NAND Timing parameters structure definition
   *)

  FMC_NAND_PCC_TimingTypeDef = record
    SetupTime: longword;  (*!< Defines the number of HCLK cycles to setup address before
                                      the command assertion for NAND-Flash read or write access
                                      to common/Attribute or I/O memory space (depending on
                                      the memory space timing to be configured).
                                      This parameter can be a value between Min_Data = 0 and Max_Data = 254     *)
    WaitSetupTime: longword;  (*!< Defines the minimum number of HCLK cycles to assert the
                                      command for NAND-Flash read or write access to
                                      common/Attribute or I/O memory space (depending on the
                                      memory space timing to be configured).
                                      This parameter can be a number between Min_Data = 0 and Max_Data = 254    *)
    HoldSetupTime: longword;  (*!< Defines the number of HCLK clock cycles to hold address
                                      (and data for write access) after the command de-assertion
                                      for NAND-Flash read or write access to common/Attribute
                                      or I/O memory space (depending on the memory space timing
                                      to be configured).
                                      This parameter can be a number between Min_Data = 0 and Max_Data = 254    *)
    HiZSetupTime: longword;  (*!< Defines the number of HCLK clock cycles during which the
                                      data bus is kept in HiZ after the start of a NAND-Flash
                                      write access to common/Attribute or I/O memory space (depending
                                      on the memory space timing to be configured).
                                      This parameter can be a number between Min_Data = 0 and Max_Data = 254    *)
  end;

  (**
  * @brief  FMC SDRAM Configuration Structure definition
   *)

  FMC_SDRAM_InitTypeDef = record
    SDBank: longword;  (*!< Specifies the SDRAM memory device that will be used.
                                             This parameter can be a value of @ref FMC_SDRAM_Bank                 *)
    ColumnBitsNumber: longword;  (*!< Defines the number of bits of column address.
                                             This parameter can be a value of @ref FMC_SDRAM_Column_Bits_number.  *)
    RowBitsNumber: longword;  (*!< Defines the number of bits of column address.
                                             This parameter can be a value of @ref FMC_SDRAM_Row_Bits_number.     *)
    MemoryDataWidth: longword;  (*!< Defines the memory device width.
                                             This parameter can be a value of @ref FMC_SDRAM_Memory_Bus_Width.    *)
    InternalBankNumber: longword;  (*!< Defines the number of the device's internal banks.
                                             This parameter can be of @ref FMC_SDRAM_Internal_Banks_Number.       *)
    CASLatency: longword;  (*!< Defines the SDRAM CAS latency in number of memory clock cycles.
                                             This parameter can be a value of @ref FMC_SDRAM_CAS_Latency.         *)
    WriteProtection: longword;  (*!< Enables the SDRAM device to be accessed in write mode.
                                             This parameter can be a value of @ref FMC_SDRAM_Write_Protection.    *)
    SDClockPeriod: longword;  (*!< Define the SDRAM Clock Period for both SDRAM devices and they allow
                                             to disable the clock before changing frequency.
                                             This parameter can be a value of @ref FMC_SDRAM_Clock_Period.        *)
    ReadBurst: longword;  (*!< This bit enable the SDRAM controller to anticipate the next read
                                             commands during the CAS latency and stores data in the Read FIFO.
                                             This parameter can be a value of @ref FMC_SDRAM_Read_Burst.          *)
    ReadPipeDelay: longword;  (*!< Define the delay in system clock cycles on read data path.
                                             This parameter can be a value of @ref FMC_SDRAM_Read_Pipe_Delay.     *)
  end;

  (**
  * @brief FMC SDRAM Timing parameters structure definition
   *)

  FMC_SDRAM_TimingTypeDef = record
    LoadToActiveDelay: longword;  (*!< Defines the delay between a Load Mode Register command and
                                              an active or Refresh command in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16   *)
    ExitSelfRefreshDelay: longword;  (*!< Defines the delay from releasing the self refresh command to
                                              issuing the Activate command in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16   *)
    SelfRefreshTime: longword;  (*!< Defines the minimum Self Refresh period in number of memory clock
                                              cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16   *)
    RowCycleDelay: longword;  (*!< Defines the delay between the Refresh command and the Activate command
                                              and the delay between two consecutive Refresh commands in number of
                                              memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16   *)
    WriteRecoveryTime: longword;  (*!< Defines the Write recovery Time in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16   *)
    RPDelay: longword;  (*!< Defines the delay between a Precharge Command and an other command
                                              in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16   *)
    RCDDelay: longword;  (*!< Defines the delay between the Activate Command and a Read/Write
                                              command in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16   *)
  end;

  (**
  * @brief SDRAM command parameters structure definition
   *)

  FMC_SDRAM_CommandTypeDef = record
    CommandMode: longword;  (*!< Defines the command issued to the SDRAM device.
                                              This parameter can be a value of @ref FMC_SDRAM_Command_Mode.           *)
    CommandTarget: longword;  (*!< Defines which device (1 or 2) the command will be issued to.
                                              This parameter can be a value of @ref FMC_SDRAM_Command_Target.         *)
    AutoRefreshNumber: longword;  (*!< Defines the number of consecutive auto refresh command issued
                                              in auto refresh mode.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16    *)
    ModeRegisterDefinition: longword;  (*!< Defines the SDRAM Mode register content                                 *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @addtogroup FMC_LL_Exported_Constants FMC Low Layer Exported Constants
  * @{
   *)

  (** @defgroup FMC_LL_NOR_SRAM_Controller FMC NOR/SRAM Controller
  * @{
   *)

  (** @defgroup FMC_NORSRAM_Bank FMC NOR/SRAM Bank
  * @{
   *)

const
  FMC_NORSRAM_BANK1 = ($00000000);
  FMC_NORSRAM_BANK2 = ($00000002);
  FMC_NORSRAM_BANK3 = ($00000004);
  FMC_NORSRAM_BANK4 = ($00000006);
  (**
  * @}
   *)

  (** @defgroup FMC_Data_Address_Bus_Multiplexing FMC Data Address Bus Multiplexing
  * @{
   *)

  FMC_DATA_ADDRESS_MUX_DISABLE = ($00000000);
  FMC_DATA_ADDRESS_MUX_ENABLE = ($00000002);
  (**
  * @}
   *)

  (** @defgroup FMC_Memory_Type FMC Memory Type
  * @{
   *)

  FMC_MEMORY_TYPE_SRAM = ($00000000);
  FMC_MEMORY_TYPE_PSRAM = ($00000004);
  FMC_MEMORY_TYPE_NOR = ($00000008);
  (**
  * @}
   *)

  (** @defgroup FMC_NORSRAM_Data_Width FMC NORSRAM Data Width
  * @{
   *)

  FMC_NORSRAM_MEM_BUS_WIDTH_8 = ($00000000);
  FMC_NORSRAM_MEM_BUS_WIDTH_16 = ($00000010);
  FMC_NORSRAM_MEM_BUS_WIDTH_32 = ($00000020);
  (**
  * @}
   *)

  (** @defgroup FMC_NORSRAM_Flash_Access FMC NOR/SRAM Flash Access
  * @{
   *)

  FMC_NORSRAM_FLASH_ACCESS_ENABLE = ($00000040);
  FMC_NORSRAM_FLASH_ACCESS_DISABLE = ($00000000);
  (**
  * @}
   *)

  (** @defgroup FMC_Burst_Access_Mode FMC Burst Access Mode
  * @{
   *)

  FMC_BURST_ACCESS_MODE_DISABLE = ($00000000);
  FMC_BURST_ACCESS_MODE_ENABLE = ($00000100);
  (**
  * @}
   *)

  (** @defgroup FMC_Wait_Signal_Polarity FMC Wait Signal Polarity
  * @{
   *)

  FMC_WAIT_SIGNAL_POLARITY_LOW = ($00000000);
  FMC_WAIT_SIGNAL_POLARITY_HIGH = ($00000200);
  (**
  * @}
   *)

  (** @defgroup FMC_Wait_Timing FMC Wait Timing
  * @{
   *)

  FMC_WAIT_TIMING_BEFORE_WS = ($00000000);
  FMC_WAIT_TIMING_DURING_WS = ($00000800);
  (**
  * @}
   *)

  (** @defgroup FMC_Write_Operation FMC Write Operation
  * @{
   *)

  FMC_WRITE_OPERATION_DISABLE = ($00000000);
  FMC_WRITE_OPERATION_ENABLE = ($00001000);
  (**
  * @}
   *)

  (** @defgroup FMC_Wait_Signal FMC Wait Signal
  * @{
   *)

  FMC_WAIT_SIGNAL_DISABLE = ($00000000);
  FMC_WAIT_SIGNAL_ENABLE = ($00002000);
  (**
  * @}
   *)

  (** @defgroup FMC_Extended_Mode FMC Extended Mode
  * @{
   *)

  FMC_EXTENDED_MODE_DISABLE = ($00000000);
  FMC_EXTENDED_MODE_ENABLE = ($00004000);
  (**
  * @}
   *)

  (** @defgroup FMC_AsynchronousWait FMC Asynchronous Wait
  * @{
   *)

  FMC_ASYNCHRONOUS_WAIT_DISABLE = ($00000000);
  FMC_ASYNCHRONOUS_WAIT_ENABLE = ($00008000);
  (**
  * @}
   *)

  (** @defgroup FMC_Page_Size FMC Page Size
  * @{
   *)

  FMC_PAGE_SIZE_NONE = ($00000000);
  FMC_PAGE_SIZE_128 = (FMC_BCR1_CPSIZE_0);
  FMC_PAGE_SIZE_256 = (FMC_BCR1_CPSIZE_1);
  FMC_PAGE_SIZE_1024 = (FMC_BCR1_CPSIZE_2);
  (**
  * @}
   *)

  (** @defgroup FMC_Write_Burst FMC Write Burst
  * @{
   *)

  FMC_WRITE_BURST_DISABLE = ($00000000);
  FMC_WRITE_BURST_ENABLE = ($00080000);
  (**
  * @}
   *)

  (** @defgroup FMC_Continous_Clock FMC Continuous Clock
  * @{
   *)

  FMC_CONTINUOUS_CLOCK_SYNC_ONLY = ($00000000);
  FMC_CONTINUOUS_CLOCK_SYNC_ASYNC = ($00100000);
  (**
  * @}
   *)

  (** @defgroup FMC_Write_FIFO FMC Write FIFO
  * @{
   *)

  FMC_WRITE_FIFO_DISABLE = ($00000000);
  FMC_WRITE_FIFO_ENABLE = (FMC_BCR1_WFDIS);
  (**
  * @}
   *)

  (** @defgroup FMC_Access_Mode FMC Access Mode
  * @{
   *)

  FMC_ACCESS_MODE_A = ($00000000);
  FMC_ACCESS_MODE_B = ($10000000);
  FMC_ACCESS_MODE_C = ($20000000);
  FMC_ACCESS_MODE_D = ($30000000);
  (**
  * @}
   *)

  (**
  * @}
   *)

  (** @defgroup FMC_LL_NAND_Controller FMC NAND Controller
  * @{
   *)

  (** @defgroup FMC_NAND_Bank FMC NAND Bank
  * @{
   *)

  FMC_NAND_BANK3 = ($00000100);
  (**
  * @}
   *)

  (** @defgroup FMC_Wait_feature FMC Wait feature
  * @{
   *)

  FMC_NAND_WAIT_FEATURE_DISABLE = ($00000000);
  FMC_NAND_WAIT_FEATURE_ENABLE = ($00000002);
  (**
  * @}
   *)

  (** @defgroup FMC_PCR_Memory_Type FMC PCR Memory Type
  * @{
   *)

  FMC_PCR_MEMORY_TYPE_NAND = ($00000008);
  (**
  * @}
   *)

  (** @defgroup FMC_NAND_Data_Width FMC NAND Data Width
  * @{
   *)

  FMC_NAND_MEM_BUS_WIDTH_8 = ($00000000);
  FMC_NAND_MEM_BUS_WIDTH_16 = ($00000010);
  (**
  * @}
   *)

  (** @defgroup FMC_ECC FMC ECC
  * @{
   *)

  FMC_NAND_ECC_DISABLE_ = ($00000000);
  FMC_NAND_ECC_ENABLE_ = ($00000040);
  (**
  * @}
   *)

  (** @defgroup FMC_ECC_Page_Size FMC ECC Page Size
  * @{
   *)

  FMC_NAND_ECC_PAGE_SIZE_256BYTE = ($00000000);
  FMC_NAND_ECC_PAGE_SIZE_512BYTE = ($00020000);
  FMC_NAND_ECC_PAGE_SIZE_1024BYTE = ($00040000);
  FMC_NAND_ECC_PAGE_SIZE_2048BYTE = ($00060000);
  FMC_NAND_ECC_PAGE_SIZE_4096BYTE = ($00080000);
  FMC_NAND_ECC_PAGE_SIZE_8192BYTE = ($000A0000);
  (**
  * @}
   *)

  (**
  * @}
   *)

  (** @defgroup FMC_LL_SDRAM_Controller FMC SDRAM Controller
  * @{
   *)

  (** @defgroup FMC_SDRAM_Bank FMC SDRAM Bank
  * @{
   *)

  FMC_SDRAM_BANK1 = ($00000000);
  FMC_SDRAM_BANK2 = ($00000001);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Column_Bits_number FMC SDRAM Column Bits number
  * @{
   *)

  FMC_SDRAM_COLUMN_BITS_NUM_8 = ($00000000);
  FMC_SDRAM_COLUMN_BITS_NUM_9 = ($00000001);
  FMC_SDRAM_COLUMN_BITS_NUM_10 = ($00000002);
  FMC_SDRAM_COLUMN_BITS_NUM_11 = ($00000003);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Row_Bits_number FMC SDRAM Row Bits number
  * @{
   *)

  FMC_SDRAM_ROW_BITS_NUM_11 = ($00000000);
  FMC_SDRAM_ROW_BITS_NUM_12 = ($00000004);
  FMC_SDRAM_ROW_BITS_NUM_13 = ($00000008);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Memory_Bus_Width FMC SDRAM Memory Bus Width
  * @{
   *)

  FMC_SDRAM_MEM_BUS_WIDTH_8 = ($00000000);
  FMC_SDRAM_MEM_BUS_WIDTH_16 = ($00000010);
  FMC_SDRAM_MEM_BUS_WIDTH_32 = ($00000020);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Internal_Banks_Number FMC SDRAM Internal Banks Number
  * @{
   *)

  FMC_SDRAM_INTERN_BANKS_NUM_2 = ($00000000);
  FMC_SDRAM_INTERN_BANKS_NUM_4 = ($00000040);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_CAS_Latency FMC SDRAM CAS Latency
  * @{
   *)

  FMC_SDRAM_CAS_LATENCY_1 = ($00000080);
  FMC_SDRAM_CAS_LATENCY_2 = ($00000100);
  FMC_SDRAM_CAS_LATENCY_3 = ($00000180);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Write_Protection FMC SDRAM Write Protection
  * @{
   *)

  FMC_SDRAM_WRITE_PROTECTION_DISABLE = ($00000000);
  FMC_SDRAM_WRITE_PROTECTION_ENABLE = ($00000200);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Clock_Period FMC SDRAM Clock Period
  * @{
   *)

  FMC_SDRAM_CLOCK_DISABLE = ($00000000);
  FMC_SDRAM_CLOCK_PERIOD_2 = ($00000800);
  FMC_SDRAM_CLOCK_PERIOD_3 = ($00000C00);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Read_Burst FMC SDRAM Read Burst
  * @{
   *)

  FMC_SDRAM_RBURST_DISABLE = ($00000000);
  FMC_SDRAM_RBURST_ENABLE = ($00001000);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Read_Pipe_Delay FMC SDRAM Read Pipe Delay
  * @{
   *)

  FMC_SDRAM_RPIPE_DELAY_0 = ($00000000);
  FMC_SDRAM_RPIPE_DELAY_1 = ($00002000);
  FMC_SDRAM_RPIPE_DELAY_2 = ($00004000);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Command_Mode FMC SDRAM Command Mode
  * @{
   *)

  FMC_SDRAM_CMD_NORMAL_MODE = ($00000000);
  FMC_SDRAM_CMD_CLK_ENABLE = ($00000001);
  FMC_SDRAM_CMD_PALL = ($00000002);
  FMC_SDRAM_CMD_AUTOREFRESH_MODE = ($00000003);
  FMC_SDRAM_CMD_LOAD_MODE = ($00000004);
  FMC_SDRAM_CMD_SELFREFRESH_MODE = ($00000005);
  FMC_SDRAM_CMD_POWERDOWN_MODE = ($00000006);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Command_Target FMC SDRAM Command Target
  * @{
   *)

  FMC_SDRAM_CMD_TARGET_BANK2 = FMC_SDCMR_CTB2;
  FMC_SDRAM_CMD_TARGET_BANK1 = FMC_SDCMR_CTB1;
  FMC_SDRAM_CMD_TARGET_BANK1_2 = ($00000018);
  (**
  * @}
   *)

  (** @defgroup FMC_SDRAM_Mode_Status FMC SDRAM Mode Status
  * @{
   *)

  FMC_SDRAM_NORMAL_MODE = ($00000000);
  FMC_SDRAM_SELF_REFRESH_MODE = FMC_SDSR_MODES1_0;
  FMC_SDRAM_POWER_DOWN_MODE = FMC_SDSR_MODES1_1;
  (**
  * @}
   *)

  (**
  * @}
   *)

  (** @defgroup FMC_LL_Interrupt_definition FMC Low Layer Interrupt definition
  * @{
   *)

  FMC_IT_RISING_EDGE = ($00000008);
  FMC_IT_LEVEL = ($00000010);
  FMC_IT_FALLING_EDGE = ($00000020);
  FMC_IT_REFRESH_ERROR = ($00004000);
  (**
  * @}
   *)

  (** @defgroup FMC_LL_Flag_definition FMC Low Layer Flag definition
  * @{
   *)

  FMC_FLAG_RISING_EDGE = ($00000001);
  FMC_FLAG_LEVEL = ($00000002);
  FMC_FLAG_FALLING_EDGE = ($00000004);
  FMC_FLAG_FEMPT = ($00000040);
  FMC_SDRAM_FLAG_REFRESH_IT = FMC_SDSR_RE;
  FMC_SDRAM_FLAG_BUSY = FMC_SDSR_BUSY;
  FMC_SDRAM_FLAG_REFRESH_ERROR = FMC_SDRTR_CRE;

procedure __FMC_NORSRAM_ENABLE(var __INSTANCE__: FMC_NORSRAM_TypeDef; __BANK__: longword);
procedure __FMC_NORSRAM_DISABLE(var __INSTANCE__: FMC_NORSRAM_TypeDef; __BANK__: longword);
procedure __FMC_NAND_ENABLE(var __INSTANCE__: FMC_NAND_TypeDef);
procedure __FMC_NAND_DISABLE(var __INSTANCE__: FMC_NAND_TypeDef);
procedure __FMC_NAND_ENABLE_IT(var __INSTANCE__: FMC_NAND_TypeDef; __INTERRUPT__: longword);
procedure __FMC_NAND_DISABLE_IT(var __INSTANCE__: FMC_NAND_TypeDef; __INTERRUPT__: longword);
function __FMC_NAND_GET_FLAG(var __INSTANCE__: FMC_NAND_TypeDef; __BANK__, __FLAG__: longword): boolean;
procedure __FMC_NAND_CLEAR_FLAG(var __INSTANCE__: FMC_NAND_TypeDef; __FLAG__: longword);
procedure __FMC_SDRAM_ENABLE_IT(var __INSTANCE__: FMC_SDRAM_TypeDef; __INTERRUPT__: longword);
procedure __FMC_SDRAM_DISABLE_IT(var __INSTANCE__: FMC_SDRAM_TypeDef; __INTERRUPT__: longword);
function __FMC_SDRAM_GET_FLAG(var __INSTANCE__: FMC_SDRAM_TypeDef; __FLAG__: longword): boolean;
procedure __FMC_SDRAM_CLEAR_FLAG(var __INSTANCE__: FMC_SDRAM_TypeDef; __FLAG__: longword);

function FMC_NORSRAM_Init(var Device: FMC_NORSRAM_TypeDef; var Init: FMC_NORSRAM_InitTypeDef): HAL_StatusTypeDef;
function FMC_NORSRAM_Timing_Init(var Device: FMC_NORSRAM_TypeDef; const Timing: FMC_NORSRAM_TimingTypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_NORSRAM_Extended_Timing_Init(var Device: FMC_NORSRAM_EXTENDED_TypeDef; const Timing: FMC_NORSRAM_TimingTypeDef; Bank, ExtendedMode: longword): HAL_StatusTypeDef;
function FMC_NORSRAM_DeInit(var Device: FMC_NORSRAM_TypeDef; var ExDevice: FMC_NORSRAM_EXTENDED_TypeDef; Bank: longword): HAL_StatusTypeDef;

function FMC_NORSRAM_WriteOperation_Enable(var Device: FMC_NORSRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_NORSRAM_WriteOperation_Disable(var Device: FMC_NORSRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;

function FMC_NAND_Init(var Device: FMC_NAND_TypeDef; const Init: FMC_NAND_InitTypeDef): HAL_StatusTypeDef;
function FMC_NAND_CommonSpace_Timing_Init(var Device: FMC_NAND_TypeDef; const Timing: FMC_NAND_PCC_TimingTypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_NAND_AttributeSpace_Timing_Init(var Device: FMC_NAND_TypeDef; const Timing: FMC_NAND_PCC_TimingTypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_NAND_DeInit(var Device: FMC_NAND_TypeDef; Bank: longword): HAL_StatusTypeDef;

function FMC_NAND_ECC_Enable(var Device: FMC_NAND_TypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_NAND_ECC_Disable(var Device: FMC_NAND_TypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_NAND_GetECC(var Device: FMC_NAND_TypeDef; var ECCval: longword; Bank, Timeout: longword): HAL_StatusTypeDef;

function FMC_SDRAM_Init(var Device: FMC_SDRAM_TypeDef; const Init: FMC_SDRAM_InitTypeDef): HAL_StatusTypeDef;
function FMC_SDRAM_Timing_Init(var Device: FMC_SDRAM_TypeDef; const Timing: FMC_SDRAM_TimingTypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_SDRAM_DeInit(var Device: FMC_SDRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
(**
  * @}
   *)

(** @defgroup FMC_LL_SDRAM_Private_Functions_Group2 SDRAM Control functions
  *  @{
   *)
function FMC_SDRAM_WriteProtection_Enable(var Device: FMC_SDRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_SDRAM_WriteProtection_Disable(var Device: FMC_SDRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
function FMC_SDRAM_SendCommand(var Device: FMC_SDRAM_TypeDef; const Command: FMC_SDRAM_CommandTypeDef; Timeout: longword): HAL_StatusTypeDef;
function FMC_SDRAM_ProgramRefreshRate(var Device: FMC_SDRAM_TypeDef; RefreshRate: longword): HAL_StatusTypeDef;
function FMC_SDRAM_SetAutoRefreshNumber(var Device: FMC_SDRAM_TypeDef; AutoRefreshNumber: longword): HAL_StatusTypeDef;
function FMC_SDRAM_GetModeStatus(var Device: FMC_SDRAM_TypeDef; Bank: longword): longword;

implementation

(**
  * @brief  Enable the NORSRAM device access.
  * @param  __INSTANCE__: FMC_NORSRAM Instance
  * @param  __BANK__: FMC_NORSRAM Bank
  * @retval None
  *)
procedure __FMC_NORSRAM_ENABLE(var __INSTANCE__: FMC_NORSRAM_TypeDef; __BANK__: longword);
begin
  __INSTANCE__.BTCR[__BANK__] := __INSTANCE__.BTCR[__BANK__] or FMC_BCR1_MBKEN;
end;

(**
  * @brief  Disable the NORSRAM device access.
  * @param  __INSTANCE__: FMC_NORSRAM Instance
  * @param  __BANK__: FMC_NORSRAM Bank
  * @retval None
  *)
procedure __FMC_NORSRAM_DISABLE(var __INSTANCE__: FMC_NORSRAM_TypeDef; __BANK__: longword);
begin
  __INSTANCE__.BTCR[__BANK__] := __INSTANCE__.BTCR[__BANK__] and not longword(FMC_BCR1_MBKEN);
end;

(** @defgroup FMC_LL_NAND_Macros FMC NAND Macros
 *  @brief macros to handle NAND device enable/disable
 *  @begin
 *)

(**
  * @brief  Enable the NAND device access.
  * @param  __INSTANCE__: FMC_NAND Instance
  * @retval None
  *)
procedure __FMC_NAND_ENABLE(var __INSTANCE__: FMC_NAND_TypeDef);
begin
  __INSTANCE__.PCR := __INSTANCE__.PCR or FMC_PCR_PBKEN;
end;

(**
  * @brief  Disable the NAND device access.
  * @param  __INSTANCE__: FMC_NAND Instance
  * @retval None
  *)
procedure __FMC_NAND_DISABLE(var __INSTANCE__: FMC_NAND_TypeDef);
begin
  __INSTANCE__.PCR := __INSTANCE__.PCR and (not longword(FMC_PCR_PBKEN));
end;

(**
  * @end
  *)

(** @defgroup FMC_Interrupt FMC Interrupt
 *  @brief macros to handle FMC interrupts
 * @begin
 *)

(**
  * @brief  Enable the NAND device interrupt.
  * @param  __INSTANCE__:  FMC_NAND instance
  * @param  __INTERRUPT__: FMC_NAND interrupt
  *         This parameter can be any combination of the following values:
  *            @arg FMC_IT_RISING_EDGE: Interrupt rising edge.
  *            @arg FMC_IT_LEVEL: Interrupt level.
  *            @arg FMC_IT_FALLING_EDGE: Interrupt falling edge.
  * @retval None
  *)
procedure __FMC_NAND_ENABLE_IT(var __INSTANCE__: FMC_NAND_TypeDef; __INTERRUPT__: longword);
begin
  __INSTANCE__.SR := __INSTANCE__.SR or (__INTERRUPT__);
end;

(**
  * @brief  Disable the NAND device interrupt.
  * @param  __INSTANCE__:  FMC_NAND Instance
  * @param  __INTERRUPT__: FMC_NAND interrupt
  *         This parameter can be any combination of the following values:
  *            @arg FMC_IT_RISING_EDGE: Interrupt rising edge.
  *            @arg FMC_IT_LEVEL: Interrupt level.
  *            @arg FMC_IT_FALLING_EDGE: Interrupt falling edge.
  * @retval None
  *)
procedure __FMC_NAND_DISABLE_IT(var __INSTANCE__: FMC_NAND_TypeDef; __INTERRUPT__: longword);
begin
  __INSTANCE__.SR := __INSTANCE__.SR and (not (__INTERRUPT__));
end;

(**
  * @brief  Get flag status of the NAND device.
  * @param  __INSTANCE__: FMC_NAND Instance
  * @param  __BANK__:     FMC_NAND Bank
  * @param  __FLAG__: FMC_NAND flag
  *         This parameter can be any combination of the following values:
  *            @arg FMC_FLAG_RISING_EDGE: Interrupt rising edge flag.
  *            @arg FMC_FLAG_LEVEL: Interrupt level edge flag.
  *            @arg FMC_FLAG_FALLING_EDGE: Interrupt falling edge flag.
  *            @arg FMC_FLAG_FEMPT: FIFO empty flag.
  * @retval The state of FLAG (SET or RESET).
  *)
function __FMC_NAND_GET_FLAG(var __INSTANCE__: FMC_NAND_TypeDef; __BANK__, __FLAG__: longword): boolean;
begin
  exit((__INSTANCE__.SR and (__FLAG__)) = (__FLAG__));
end;

(**
  * @brief  Clear flag status of the NAND device.
  * @param  __INSTANCE__: FMC_NAND Instance
  * @param  __FLAG__: FMC_NAND flag
  *         This parameter can be any combination of the following values:
  *            @arg FMC_FLAG_RISING_EDGE: Interrupt rising edge flag.
  *            @arg FMC_FLAG_LEVEL: Interrupt level edge flag.
  *            @arg FMC_FLAG_FALLING_EDGE: Interrupt falling edge flag.
  *            @arg FMC_FLAG_FEMPT: FIFO empty flag.
  * @retval None
  *)
procedure __FMC_NAND_CLEAR_FLAG(var __INSTANCE__: FMC_NAND_TypeDef; __FLAG__: longword);
begin
  __INSTANCE__.SR := __INSTANCE__.SR and (not (__FLAG__));
end;

(**
  * @brief  Enable the SDRAM device interrupt.
  * @param  __INSTANCE__: FMC_SDRAM instance
  * @param  __INTERRUPT__: FMC_SDRAM interrupt
  *         This parameter can be any combination of the following values:
  *            @arg FMC_IT_REFRESH_ERROR: Interrupt refresh error
  * @retval None
  *)
procedure __FMC_SDRAM_ENABLE_IT(var __INSTANCE__: FMC_SDRAM_TypeDef; __INTERRUPT__: longword);
begin
  __INSTANCE__.SDRTR := __INSTANCE__.SDRTR or (__INTERRUPT__);
end;

(**
  * @brief  Disable the SDRAM device interrupt.
  * @param  __INSTANCE__: FMC_SDRAM instance
  * @param  __INTERRUPT__: FMC_SDRAM interrupt
  *         This parameter can be any combination of the following values:
  *            @arg FMC_IT_REFRESH_ERROR: Interrupt refresh error
  * @retval None
  *)
procedure __FMC_SDRAM_DISABLE_IT(var __INSTANCE__: FMC_SDRAM_TypeDef; __INTERRUPT__: longword);
begin
  __INSTANCE__.SDRTR := __INSTANCE__.SDRTR and (not __INTERRUPT__);
end;

(**
  * @brief  Get flag status of the SDRAM device.
  * @param  __INSTANCE__: FMC_SDRAM instance
  * @param  __FLAG__: FMC_SDRAM flag
  *         This parameter can be any combination of the following values:
  *            @arg FMC_SDRAM_FLAG_REFRESH_IT: Interrupt refresh error.
  *            @arg FMC_SDRAM_FLAG_BUSY: SDRAM busy flag.
  *            @arg FMC_SDRAM_FLAG_REFRESH_ERROR: Refresh error flag.
  * @retval The state of FLAG (SET or RESET).
  *)
function __FMC_SDRAM_GET_FLAG(var __INSTANCE__: FMC_SDRAM_TypeDef; __FLAG__: longword): boolean;
begin
  exit((__INSTANCE__.SDSR and (__FLAG__)) = (__FLAG__));
end;

(**
  * @brief  Clear flag status of the SDRAM device.
  * @param  __INSTANCE__: FMC_SDRAM instance
  * @param  __FLAG__: FMC_SDRAM flag
  *         This parameter can be any combination of the following values:
  *           @arg FMC_SDRAM_FLAG_REFRESH_ERROR
  * @retval None
  *)
procedure __FMC_SDRAM_CLEAR_FLAG(var __INSTANCE__: FMC_SDRAM_TypeDef; __FLAG__: longword);
begin
  __INSTANCE__.SDRTR := __INSTANCE__.SDRTR or (__FLAG__);
end;

function FMC_NORSRAM_Init(var Device: FMC_NORSRAM_TypeDef; var Init: FMC_NORSRAM_InitTypeDef): HAL_StatusTypeDef;
var
  tmpr: longword;
begin
  (* Get the BTCR register value *)
  tmpr := Device.BTCR[Init.NSBank];

  (* Clear MBKEN, MUXEN, MTYP, MWID, FACCEN, BURSTEN, WAITPOL, WAITCFG, WREN,
           WAITEN, EXTMOD, ASYNCWAIT, CBURSTRW and CCLKEN bits *)
  tmpr := tmpr and ((not (FMC_BCR1_MBKEN or FMC_BCR1_MUXEN or FMC_BCR1_MTYP or FMC_BCR1_MWID or FMC_BCR1_FACCEN or FMC_BCR1_BURSTEN or FMC_BCR1_WAITPOL or FMC_BCR1_CPSIZE or FMC_BCR1_WAITCFG or
    FMC_BCR1_WREN or FMC_BCR1_WAITEN or FMC_BCR1_EXTMOD or FMC_BCR1_ASYNCWAIT or FMC_BCR1_CBURSTRW or FMC_BCR1_CCLKEN or FMC_BCR1_WFDIS)));

  (* Set NORSRAM device control parameters *)
  tmpr := tmpr or (Init.DataAddressMux or Init.MemoryType or Init.MemoryDataWidth or Init.BurstAccessMode or Init.WaitSignalPolarity or Init.WaitSignalActive or Init.WriteOperation or
    Init.WaitSignal or Init.ExtendedMode or Init.AsynchronousWait or Init.WriteBurst or Init.ContinuousClock or Init.PageSize or Init.WriteFifo);

  if (Init.MemoryType = FMC_MEMORY_TYPE_NOR) then
    tmpr := tmpr or FMC_NORSRAM_FLASH_ACCESS_ENABLE;

  Device.BTCR[Init.NSBank] := tmpr;

  (* Configure synchronous mode when Continuous clock is enabled for bank2..4 *)
  if ((Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ASYNC) and (Init.NSBank <> FMC_NORSRAM_BANK1)) then
  begin
    Init.BurstAccessMode := FMC_BURST_ACCESS_MODE_ENABLE;
    Device.BTCR[FMC_NORSRAM_BANK1] := Device.BTCR[FMC_NORSRAM_BANK1] or (Init.BurstAccessMode or Init.ContinuousClock);
  end;

  if (Init.NSBank <> FMC_NORSRAM_BANK1) then
    Device.BTCR[FMC_NORSRAM_BANK1] := Device.BTCR[FMC_NORSRAM_BANK1] or (Init.WriteFifo);

  exit(HAL_OK);
end;

function FMC_NORSRAM_Timing_Init(var Device: FMC_NORSRAM_TypeDef; const Timing: FMC_NORSRAM_TimingTypeDef; Bank: longword): HAL_StatusTypeDef;
var
  tmpr: longword;
begin
  (* Get the BTCR register value *)
  tmpr := Device.BTCR[Bank + 1];

  (* Clear ADDSET, ADDHLD, DATAST, BUSTURN, CLKDIV, DATLAT and ACCMOD bits *)
  tmpr := tmpr and ((not (FMC_BTR1_ADDSET or FMC_BTR1_ADDHLD or FMC_BTR1_DATAST or FMC_BTR1_BUSTURN or FMC_BTR1_CLKDIV or FMC_BTR1_DATLAT or FMC_BTR1_ACCMOD)));

  (* Set FMC_NORSRAM device timing parameters *)
  tmpr := tmpr or (Timing.AddressSetupTime or ((Timing.AddressHoldTime) shl 4) or ((Timing.DataSetupTime) shl 8) or ((Timing.BusTurnAroundDuration) shl 16) or (((Timing.CLKDivision) - 1) shl 20) or
    (((Timing.DataLatency) - 2) shl 24) or (Timing.AccessMode));

  Device.BTCR[Bank + 1] := tmpr;

  (* Configure Clock division value (in NORSRAM bank 1) when continuous clock is enabled *)
  if HAL_IS_BIT_SET(Device.BTCR[FMC_NORSRAM_BANK1], FMC_BCR1_CCLKEN) then
  begin
    tmpr := (Device.BTCR[FMC_NORSRAM_BANK1 + 1] and not (($0F) shl 20));
    tmpr := tmpr or (((Timing.CLKDivision) - 1) shl 20);
    Device.BTCR[FMC_NORSRAM_BANK1 + 1] := tmpr;
  end;

  exit(HAL_OK);
end;

function FMC_NORSRAM_Extended_Timing_Init(var Device: FMC_NORSRAM_EXTENDED_TypeDef; const Timing: FMC_NORSRAM_TimingTypeDef; Bank, ExtendedMode: longword): HAL_StatusTypeDef;
var
  tmpr: longword;
begin
  (* Set NORSRAM device timing register for write configuration, if extended mode is used *)
  if ExtendedMode = FMC_EXTENDED_MODE_ENABLE then
  begin
    (* Get the BWTR register value *)
    tmpr := Device.BWTR[Bank];

    (* Clear ADDSET, ADDHLD, DATAST, BUSTURN, CLKDIV, DATLAT and ACCMOD bits *)
    tmpr := tmpr and ((not (FMC_BWTR1_ADDSET or FMC_BWTR1_ADDHLD or FMC_BWTR1_DATAST or FMC_BWTR1_BUSTURN or FMC_BWTR1_ACCMOD)));

    tmpr := tmpr or (Timing.AddressSetupTime or ((Timing.AddressHoldTime) shl 4) or ((Timing.DataSetupTime) shl 8) or ((Timing.BusTurnAroundDuration) shl 16) or (Timing.AccessMode));

    Device.BWTR[Bank] := tmpr;
  end
  else
    Device.BWTR[Bank] := $0FFFFFFF;

  exit(HAL_OK);
end;

function FMC_NORSRAM_DeInit(var Device: FMC_NORSRAM_TypeDef; var ExDevice: FMC_NORSRAM_EXTENDED_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* Disable the FMC_NORSRAM device *)
  __FMC_NORSRAM_DISABLE(Device, Bank);

  (* De-initialize the FMC_NORSRAM device *)
  (* FMC_NORSRAM_BANK1 *)
  if (Bank = FMC_NORSRAM_BANK1) then
    Device.BTCR[Bank] := $000030DB
  (* FMC_NORSRAM_BANK2, FMC_NORSRAM_BANK3 or FMC_NORSRAM_BANK4 *)
  else
    Device.BTCR[Bank] := $000030D2;

  Device.BTCR[Bank + 1] := $0FFFFFFF;
  ExDevice.BWTR[Bank] := $0FFFFFFF;

  exit(HAL_OK);
end;

function FMC_NORSRAM_WriteOperation_Enable(var Device: FMC_NORSRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* Enable write operation *)
  Device.BTCR[Bank] := Device.BTCR[Bank] or FMC_WRITE_OPERATION_ENABLE;

  exit(HAL_OK);
end;

function FMC_NORSRAM_WriteOperation_Disable(var Device: FMC_NORSRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* Disable write operation *)
  Device.BTCR[Bank] := Device.BTCR[Bank] and (not FMC_WRITE_OPERATION_ENABLE);

  exit(HAL_OK);
end;

function FMC_NAND_Init(var Device: FMC_NAND_TypeDef; const Init: FMC_NAND_InitTypeDef): HAL_StatusTypeDef;
var
  tmpr: longword;
begin
  (* Get the NAND bank 3 register value *)
  tmpr := Device.PCR;

  (* Clear PWAITEN, PBKEN, PTYP, PWID, ECCEN, TCLR, TAR and ECCPS bits *)
  tmpr := tmpr and ((not (FMC_PCR_PWAITEN or FMC_PCR_PBKEN or FMC_PCR_PTYP or FMC_PCR_PWID or FMC_PCR_ECCEN or FMC_PCR_TCLR or FMC_PCR_TAR or FMC_PCR_ECCPS)));
  (* Set NAND device control parameters *)
  tmpr := tmpr or (Init.Waitfeature or FMC_PCR_MEMORY_TYPE_NAND or Init.MemoryDataWidth or Init.EccComputation or Init.ECCPageSize or ((Init.TCLRSetupTime) shl 9) or ((Init.TARSetupTime) shl 13));

  (* NAND bank 3 registers configuration *)
  Device.PCR := tmpr;

  exit(HAL_OK);
end;

function FMC_NAND_CommonSpace_Timing_Init(var Device: FMC_NAND_TypeDef; const Timing: FMC_NAND_PCC_TimingTypeDef; Bank: longword): HAL_StatusTypeDef;
var
  tmpr: longword;
begin
  (* Get the NAND bank 3 register value *)
  tmpr := Device.PMEM;

  (* Clear MEMSETx, MEMWAITx, MEMHOLDx and MEMHIZx bits *)
  tmpr := tmpr and ((not (FMC_PMEM_MEMSET3 or FMC_PMEM_MEMWAIT3 or FMC_PMEM_MEMHOLD3 or FMC_PMEM_MEMHIZ3)));
  (* Set FMC_NAND device timing parameters *)
  tmpr := tmpr or (Timing.SetupTime or ((Timing.WaitSetupTime) shl 8) or ((Timing.HoldSetupTime) shl 16) or ((Timing.HiZSetupTime) shl 24));

  (* NAND bank 3 registers configuration *)
  Device.PMEM := tmpr;

  exit(HAL_OK);
end;

function FMC_NAND_AttributeSpace_Timing_Init(var Device: FMC_NAND_TypeDef; const Timing: FMC_NAND_PCC_TimingTypeDef; Bank: longword): HAL_StatusTypeDef;
var
  tmpr: longword;
begin
  (* Get the NAND bank 3 register value *)
  tmpr := Device.PATT;

  (* Clear ATTSETx, ATTWAITx, ATTHOLDx and ATTHIZx bits *)
  tmpr := tmpr and ((not (FMC_PATT_ATTSET3 or FMC_PATT_ATTWAIT3 or FMC_PATT_ATTHOLD3 or FMC_PATT_ATTHIZ3)));
  (* Set FMC_NAND device timing parameters *)
  tmpr := tmpr or (Timing.SetupTime or ((Timing.WaitSetupTime) shl 8) or ((Timing.HoldSetupTime) shl 16) or ((Timing.HiZSetupTime) shl 24));

  (* NAND bank 3 registers configuration *)
  Device.PATT := tmpr;

  exit(HAL_OK);
end;

function FMC_NAND_DeInit(var Device: FMC_NAND_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* Disable the NAND Bank *)
  __FMC_NAND_DISABLE(Device);

  (* Set the FMC_NAND_BANK3 registers to their reset values *)
  Device.PCR := $00000018;
  Device.SR := $00000040;
  Device.PMEM := $FCFCFCFC;
  Device.PATT := $FCFCFCFC;

  exit(HAL_OK);
end;

function FMC_NAND_ECC_Enable(var Device: FMC_NAND_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* Enable ECC feature *)
  Device.PCR := Device.PCR or FMC_PCR_ECCEN;

  exit(HAL_OK);
end;

function FMC_NAND_ECC_Disable(var Device: FMC_NAND_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* Disable ECC feature *)
  Device.PCR := Device.PCR and (not FMC_PCR_ECCEN);

  exit(HAL_OK);
end;

function FMC_NAND_GetECC(var Device: FMC_NAND_TypeDef; var ECCval: longword; Bank, Timeout: longword): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Wait until FIFO is empty *)
  while not __FMC_NAND_GET_FLAG(Device, Bank, FMC_FLAG_FEMPT) do
  begin
    (* Check for the Timeout *)
    if (Timeout <> HAL_MAX_DELAY) then
      if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
        exit(HAL_TIMEOUT);
  end;

  (* Get the ECCR register value *)
  ECCval := Device.ECCR;

  exit(HAL_OK);
end;

function FMC_SDRAM_Init(var Device: FMC_SDRAM_TypeDef; const Init: FMC_SDRAM_InitTypeDef): HAL_StatusTypeDef;
var
  tmpr1, tmpr2: longword;
begin
  (* Set SDRAM bank configuration parameters *)
  if (Init.SDBank <> FMC_SDRAM_BANK2) then
  begin
    tmpr1 := Device.SDCR[FMC_SDRAM_BANK1];

    (* Clear NC, NR, MWID, NB, CAS, WP, SDCLK, RBURST, and RPIPE bits *)
    tmpr1 := tmpr1 and ((not (FMC_SDCR1_NC or FMC_SDCR1_NR or FMC_SDCR1_MWID or FMC_SDCR1_NB or FMC_SDCR1_CAS or FMC_SDCR1_WP or FMC_SDCR1_SDCLK or FMC_SDCR1_RBURST or FMC_SDCR1_RPIPE)));

    tmpr1 := tmpr1 or (Init.ColumnBitsNumber or Init.RowBitsNumber or Init.MemoryDataWidth or Init.InternalBankNumber or Init.CASLatency or Init.WriteProtection or Init.SDClockPeriod or Init.ReadBurst or Init.ReadPipeDelay);
    Device.SDCR[FMC_SDRAM_BANK1] := tmpr1;
  end
  else (* FMC_Bank2_SDRAM *)
  begin
    tmpr1 := Device.SDCR[FMC_SDRAM_BANK1];

    (* Clear NC, NR, MWID, NB, CAS, WP, SDCLK, RBURST, and RPIPE bits *)
    tmpr1 := tmpr1 and ((not (FMC_SDCR1_NC or FMC_SDCR1_NR or FMC_SDCR1_MWID or FMC_SDCR1_NB or FMC_SDCR1_CAS or FMC_SDCR1_WP or FMC_SDCR1_SDCLK or FMC_SDCR1_RBURST or FMC_SDCR1_RPIPE)));

    tmpr1 := tmpr1 or (Init.SDClockPeriod or Init.ReadBurst or Init.ReadPipeDelay);

    tmpr2 := Device.SDCR[FMC_SDRAM_BANK2];

    (* Clear NC, NR, MWID, NB, CAS, WP, SDCLK, RBURST, and RPIPE bits *)
    tmpr2 := tmpr2 and ((not (FMC_SDCR1_NC or FMC_SDCR1_NR or FMC_SDCR1_MWID or FMC_SDCR1_NB or FMC_SDCR1_CAS or FMC_SDCR1_WP or FMC_SDCR1_SDCLK or FMC_SDCR1_RBURST or FMC_SDCR1_RPIPE)));

    tmpr2 := tmpr2 or (Init.ColumnBitsNumber or Init.RowBitsNumber or Init.MemoryDataWidth or Init.InternalBankNumber or Init.CASLatency or Init.WriteProtection);

    Device.SDCR[FMC_SDRAM_BANK1] := tmpr1;
    Device.SDCR[FMC_SDRAM_BANK2] := tmpr2;
  end;

  exit(HAL_OK);
end;

function FMC_SDRAM_Timing_Init(var Device: FMC_SDRAM_TypeDef; const Timing: FMC_SDRAM_TimingTypeDef; Bank: longword): HAL_StatusTypeDef;
var
  tmpr1, tmpr2: longword;
begin
  (* Set SDRAM device timing parameters *)
  if (Bank <> FMC_SDRAM_BANK2) then
  begin
    tmpr1 := Device.SDTR[FMC_SDRAM_BANK1];

    (* Clear TMRD, TXSR, TRAS, TRC, TWR, TRP and TRCD bits *)
    tmpr1 := tmpr1 and ((not (FMC_SDTR1_TMRD or FMC_SDTR1_TXSR or FMC_SDTR1_TRAS or FMC_SDTR1_TRC or FMC_SDTR1_TWR or FMC_SDTR1_TRP or FMC_SDTR1_TRCD)));

    tmpr1 := tmpr1 or (((Timing.LoadToActiveDelay) - 1) or (((Timing.ExitSelfRefreshDelay) - 1) shl 4) or (((Timing.SelfRefreshTime) - 1) shl 8) or
      (((Timing.RowCycleDelay) - 1) shl 12) or (((Timing.WriteRecoveryTime) - 1) shl 16) or (((Timing.RPDelay) - 1) shl 20) or
      (((Timing.RCDDelay) - 1) shl 24));
    Device.SDTR[FMC_SDRAM_BANK1] := tmpr1;
  end
  else (* FMC_Bank2_SDRAM *)
  begin
    tmpr1 := Device.SDTR[FMC_SDRAM_BANK2];

    (* Clear TMRD, TXSR, TRAS, TRC, TWR, TRP and TRCD bits *)
    tmpr1 := tmpr1 and ((not (FMC_SDTR1_TMRD or FMC_SDTR1_TXSR or FMC_SDTR1_TRAS or FMC_SDTR1_TRC or FMC_SDTR1_TWR or FMC_SDTR1_TRP or FMC_SDTR1_TRCD)));

    tmpr1 := tmpr1 or (((Timing.LoadToActiveDelay) - 1) or (((Timing.ExitSelfRefreshDelay) - 1) shl 4) or (((Timing.SelfRefreshTime) - 1) shl 8) or
      (((Timing.WriteRecoveryTime) - 1) shl 16) or (((Timing.RCDDelay) - 1) shl 24));

    tmpr2 := Device.SDTR[FMC_SDRAM_BANK1];

    (* Clear TMRD, TXSR, TRAS, TRC, TWR, TRP and TRCD bits *)
    tmpr2 := tmpr2 and ((not (FMC_SDTR1_TMRD or FMC_SDTR1_TXSR or FMC_SDTR1_TRAS or FMC_SDTR1_TRC or FMC_SDTR1_TWR or FMC_SDTR1_TRP or FMC_SDTR1_TRCD)));
    tmpr2 := tmpr2 or ((((Timing.RowCycleDelay) - 1) shl 12) or (((Timing.RPDelay) - 1) shl 20));

    Device.SDTR[FMC_SDRAM_BANK2] := tmpr1;
    Device.SDTR[FMC_SDRAM_BANK1] := tmpr2;
  end;

  exit(HAL_OK);
end;

function FMC_SDRAM_DeInit(var Device: FMC_SDRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* De-initialize the SDRAM device *)
  Device.SDCR[Bank] := $000002D0;
  Device.SDTR[Bank] := $0FFFFFFF;
  Device.SDCMR := $00000000;
  Device.SDRTR := $00000000;
  Device.SDSR := $00000000;

  exit(HAL_OK);
end;

function FMC_SDRAM_WriteProtection_Enable(var Device: FMC_SDRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* Enable write protection *)
  Device.SDCR[Bank] := Device.SDCR[Bank] or FMC_SDRAM_WRITE_PROTECTION_ENABLE;

  exit(HAL_OK);
end;

function FMC_SDRAM_WriteProtection_Disable(var Device: FMC_SDRAM_TypeDef; Bank: longword): HAL_StatusTypeDef;
begin
  (* Disable write protection *)
  Device.SDCR[Bank] := Device.SDCR[Bank] and (not FMC_SDRAM_WRITE_PROTECTION_ENABLE);

  exit(HAL_OK);
end;

function FMC_SDRAM_SendCommand(var Device: FMC_SDRAM_TypeDef; const Command: FMC_SDRAM_CommandTypeDef; Timeout: longword): HAL_StatusTypeDef;
var
  tmpr: longword;
  tickstart: longword;
begin
  (* Set command register *)
  tmpr := ((Command.CommandMode) or (Command.CommandTarget) or (((Command.AutoRefreshNumber) - 1) shl 5) or
    ((Command.ModeRegisterDefinition) shl 9));

  Device.SDCMR := tmpr;

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* wait until command is send *)
  while HAL_IS_BIT_SET(Device.SDSR, FMC_SDSR_BUSY) do
  begin
    (* Check for the Timeout *)
    if (Timeout <> HAL_MAX_DELAY) then
      if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
        exit(HAL_TIMEOUT);

    exit(HAL_ERROR);
  end;

  exit(HAL_OK);
end;

function FMC_SDRAM_ProgramRefreshRate(var Device: FMC_SDRAM_TypeDef; RefreshRate: longword): HAL_StatusTypeDef;
begin
  (* Set the refresh rate in command register *)
  Device.SDRTR := Device.SDRTR or (RefreshRate shl 1);

  exit(HAL_OK);
end;

function FMC_SDRAM_SetAutoRefreshNumber(var Device: FMC_SDRAM_TypeDef; AutoRefreshNumber: longword): HAL_StatusTypeDef;
begin
  (* Set the Auto-refresh number in command register *)
  Device.SDCMR := Device.SDCMR or (AutoRefreshNumber shl 5);

  exit(HAL_OK);
end;

function FMC_SDRAM_GetModeStatus(var Device: FMC_SDRAM_TypeDef; Bank: longword): longword;
var
  tmpreg: longword;
begin
  (* Get the corresponding bank mode *)
  if (Bank = FMC_SDRAM_BANK1) then
    tmpreg := (Device.SDSR and FMC_SDSR_MODES1)
  else
    tmpreg := ((Device.SDSR and FMC_SDSR_MODES2) shr 2);

  (* Return the mode status *)
  exit(tmpreg);
end;

end.
