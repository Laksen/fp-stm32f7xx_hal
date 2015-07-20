(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_flash_ex.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of FLASH HAL Extension module.
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

unit stm32f7xx_hal_flash_ex;

interface

uses
  stm32f7xx_hal,
  stm32f7xx_defs;

(**
  * @brief  FLASH Erase structure definition
   *)

type
  FLASH_EraseInitTypeDef = record
    TypeErase: longword;  (*!< Mass erase or sector Erase.
                             This parameter can be a value of @ref FLASHEx_Type_Erase  *)
    Sector: longword;  (*!< Initial FLASH sector to erase when Mass erase is disabled
                             This parameter must be a value of @ref FLASHEx_Sectors  *)
    NbSectors: longword;  (*!< Number of sectors to be erased.
                             This parameter must be a value between 1 and (max number of sectors - value of Initial sector) *)
    VoltageRange: longword;  (*!< The device voltage range which defines the erase parallelism
                             This parameter must be a value of @ref FLASHEx_Voltage_Range  *)
  end;

  (**
  * @brief  FLASH Option Bytes Program structure definition
   *)

  FLASH_OBProgramInitTypeDef = record
    OptionType: longword;  (*!< Option byte to be configured.
                              This parameter can be a value of @ref FLASHEx_Option_Type  *)
    WRPState: longword;  (*!< Write protection activation or deactivation.
                              This parameter can be a value of @ref FLASHEx_WRP_State  *)
    WRPSector: longword;  (*!< Specifies the sector(s) to be write protected.
                              The value of this parameter depend on device used within the same series  *)
    RDPLevel: longword;  (*!< Set the read protection level.
                              This parameter can be a value of @ref FLASHEx_Option_Bytes_Read_Protection  *)
    BORLevel: longword;  (*!< Set the BOR Level.
                              This parameter can be a value of @ref FLASHEx_BOR_Reset_Level  *)
    USERConfig: longword;  (*!< Program the FLASH User Option Byte: WWDG_SW / IWDG_SW / RST_STOP / RST_STDBY /
                              IWDG_FREEZE_STOP / IWDG_FREEZE_SANDBY.  *)
    BootAddr0: longword;  (*!< Boot base address when Boot pin = 0.
                              This parameter can be a value of @ref FLASHEx_Boot_Address  *)
    BootAddr1: longword;  (*!< Boot base address when Boot pin = 1.
                              This parameter can be a value of @ref FLASHEx_Boot_Address  *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup FLASHEx_Exported_Constants FLASH Exported Constants
  * @{
   *)

  (** @defgroup FLASHEx_Type_Erase FLASH Type Erase
  * @{
   *)

const
  FLASH_TYPEERASE_SECTORS = ($00);  (*!< Sectors erase only           *)
  FLASH_TYPEERASE_MASSERASE = ($01);  (*!< Flash Mass erase activation  *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Voltage_Range FLASH Voltage Range
  * @{
   *)

  FLASH_VOLTAGE_RANGE_1 = ($00);  (*!< Device operating range: 1.8V to 2.1V                 *)
  FLASH_VOLTAGE_RANGE_2 = ($01);  (*!< Device operating range: 2.1V to 2.7V                 *)
  FLASH_VOLTAGE_RANGE_3 = ($02);  (*!< Device operating range: 2.7V to 3.6V                 *)
  FLASH_VOLTAGE_RANGE_4 = ($03);  (*!< Device operating range: 2.7V to 3.6V + External Vpp  *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_WRP_State FLASH WRP State
  * @{
   *)

  OB_WRPSTATE_DISABLE = ($00);  (*!< Disable the write protection of the desired bank 1 sectors  *)
  OB_WRPSTATE_ENABLE = ($01);  (*!< Enable the write protection of the desired bank 1 sectors   *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Option_Type FLASH Option Type
  * @{
   *)

  OPTIONBYTE_WRP = ($01);  (*!< WRP option byte configuration   *)
  OPTIONBYTE_RDP = ($02);  (*!< RDP option byte configuration   *)
  OPTIONBYTE_USER = ($04);  (*!< USER option byte configuration  *)
  OPTIONBYTE_BOR = ($08);  (*!< BOR option byte configuration   *)
  OPTIONBYTE_BOOTADDR_0 = ($10);  (*!< Boot 0 Address configuration    *)
  OPTIONBYTE_BOOTADDR_1 = ($20);  (*!< Boot 1 Address configuration    *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Option_Bytes_Read_Protection FLASH Option Bytes Read Protection
  * @{
   *)

  OB_RDP_LEVEL_0 = ($AA);
  OB_RDP_LEVEL_1 = ($55);
  OB_RDP_LEVEL_2 = ($CC);  (*!< Warning: When enabling read protection level 2
                                                  it s no more possible to go back to level 1 or 0  *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Option_Bytes_WWatchdog FLASH Option Bytes WWatchdog
  * @{
   *)

  OB_WWDG_SW = ($10);  (*!< Software WWDG selected  *)
  OB_WWDG_HW = ($00);  (*!< Hardware WWDG selected  *)
  (**
  * @}
   *)


  (** @defgroup FLASHEx_Option_Bytes_IWatchdog FLASH Option Bytes IWatchdog
  * @{
   *)

  OB_IWDG_SW = ($20);  (*!< Software IWDG selected  *)
  OB_IWDG_HW = ($00);  (*!< Hardware IWDG selected  *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Option_Bytes_nRST_STOP FLASH Option Bytes nRST_STOP
  * @{
   *)

  OB_STOP_NO_RST = ($40);  (*!< No reset generated when entering in STOP  *)
  OB_STOP_RST = ($00);  (*!< Reset generated when entering in STOP     *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Option_Bytes_nRST_STDBY FLASH Option Bytes nRST_STDBY
  * @{
   *)

  OB_STDBY_NO_RST = ($80);  (*!< No reset generated when entering in STANDBY  *)
  OB_STDBY_RST = ($00);  (*!< Reset generated when entering in STANDBY     *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Option_Bytes_IWDG_FREEZE_STOP FLASH IWDG Counter Freeze in STOP
  * @{
   *)

  OB_IWDG_STOP_FREEZE = ($00000000);  (*!< Freeze IWDG counter in STOP mode  *)
  OB_IWDG_STOP_ACTIVE = ($40000000);  (*!< IWDG counter active in STOP mode  *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Option_Bytes_IWDG_FREEZE_SANDBY FLASH IWDG Counter Freeze in STANDBY
  * @{
   *)

  OB_IWDG_STDBY_FREEZE = ($00000000);  (*!< Freeze IWDG counter in STANDBY mode  *)
  OB_IWDG_STDBY_ACTIVE = ($40000000);  (*!< IWDG counter active in STANDBY mode  *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_BOR_Reset_Level FLASH BOR Reset Level
  * @{
   *)

  OB_BOR_LEVEL3 = ($00);  (*!< Supply voltage ranges from 2.70 to 3.60 V  *)
  OB_BOR_LEVEL2 = ($04);  (*!< Supply voltage ranges from 2.40 to 2.70 V  *)
  OB_BOR_LEVEL1 = ($08);  (*!< Supply voltage ranges from 2.10 to 2.40 V  *)
  OB_BOR_OFF = ($0C);  (*!< Supply voltage ranges from 1.62 to 2.10 V  *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Boot_Address FLASH Boot Address
  * @{
   *)

  OB_BOOTADDR_ITCM_RAM = ($0000);  (*!< Boot from ITCM RAM (0x00000000)                  *)
  OB_BOOTADDR_SYSTEM = ($0040);  (*!< Boot from System memory bootloader (0x00100000)  *)
  OB_BOOTADDR_ITCM_FLASH = ($0080);  (*!< Boot from Flash on ITCM interface (0x00200000)   *)
  OB_BOOTADDR_AXIM_FLASH = ($2000);  (*!< Boot from Flash on AXIM interface (0x08000000)   *)
  OB_BOOTADDR_DTCM_RAM = ($8000);  (*!< Boot from DTCM RAM (0x20000000)                  *)
  OB_BOOTADDR_SRAM1 = ($8004);  (*!< Boot from SRAM1 (0x20010000)                     *)
  OB_BOOTADDR_SRAM2 = ($8013);  (*!< Boot from SRAM2 (0x2004C000)                     *)
  (**
  * @}
   *)

  (** @defgroup FLASH_Latency FLASH Latency
  * @{
   *)

  FLASH_LATENCY_0 = FLASH_ACR_LATENCY_0WS;  (*!< FLASH Zero Latency cycle       *)
  FLASH_LATENCY_1 = FLASH_ACR_LATENCY_1WS;  (*!< FLASH One Latency cycle        *)
  FLASH_LATENCY_2 = FLASH_ACR_LATENCY_2WS;  (*!< FLASH Two Latency cycles       *)
  FLASH_LATENCY_3 = FLASH_ACR_LATENCY_3WS;  (*!< FLASH Three Latency cycles     *)
  FLASH_LATENCY_4 = FLASH_ACR_LATENCY_4WS;  (*!< FLASH Four Latency cycles      *)
  FLASH_LATENCY_5 = FLASH_ACR_LATENCY_5WS;  (*!< FLASH Five Latency cycles      *)
  FLASH_LATENCY_6 = FLASH_ACR_LATENCY_6WS;  (*!< FLASH Six Latency cycles       *)
  FLASH_LATENCY_7 = FLASH_ACR_LATENCY_7WS;  (*!< FLASH Seven Latency cycles     *)
  FLASH_LATENCY_8 = FLASH_ACR_LATENCY_8WS;  (*!< FLASH Eight Latency cycles     *)
  FLASH_LATENCY_9 = FLASH_ACR_LATENCY_9WS;  (*!< FLASH Nine Latency cycles      *)
  FLASH_LATENCY_10 = FLASH_ACR_LATENCY_10WS;  (*!< FLASH Ten Latency cycles       *)
  FLASH_LATENCY_11 = FLASH_ACR_LATENCY_11WS;  (*!< FLASH Eleven Latency cycles    *)
  FLASH_LATENCY_12 = FLASH_ACR_LATENCY_12WS;  (*!< FLASH Twelve Latency cycles    *)
  FLASH_LATENCY_13 = FLASH_ACR_LATENCY_13WS;  (*!< FLASH Thirteen Latency cycles  *)
  FLASH_LATENCY_14 = FLASH_ACR_LATENCY_14WS;  (*!< FLASH Fourteen Latency cycles  *)
  FLASH_LATENCY_15 = FLASH_ACR_LATENCY_15WS;  (*!< FLASH Fifteen Latency cycles   *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_MassErase_bit FLASH Mass Erase bit
  * @{
   *)

  FLASH_MER_BIT = (FLASH_CR_MER);  (*!< MER bit to clear  *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Sectors FLASH Sectors
  * @{
   *)

  FLASH_SECTOR_0 = (0);  (*!< Sector Number 0    *)
  FLASH_SECTOR_1 = (1);  (*!< Sector Number 1    *)
  FLASH_SECTOR_2 = (2);  (*!< Sector Number 2    *)
  FLASH_SECTOR_3 = (3);  (*!< Sector Number 3    *)
  FLASH_SECTOR_4 = (4);  (*!< Sector Number 4    *)
  FLASH_SECTOR_5 = (5);  (*!< Sector Number 5    *)
  FLASH_SECTOR_6 = (6);  (*!< Sector Number 6    *)
  FLASH_SECTOR_7 = (7);  (*!< Sector Number 7    *)
  (**
  * @}
   *)

  (** @defgroup FLASHEx_Option_Bytes_Write_Protection FLASH Option Bytes Write Protection
  * @{
   *)

  OB_WRP_SECTOR_0 = ($00010000);  (*!< Write protection of Sector0      *)
  OB_WRP_SECTOR_1 = ($00020000);  (*!< Write protection of Sector1      *)
  OB_WRP_SECTOR_2 = ($00040000);  (*!< Write protection of Sector2      *)
  OB_WRP_SECTOR_3 = ($00080000);  (*!< Write protection of Sector3      *)
  OB_WRP_SECTOR_4 = ($00100000);  (*!< Write protection of Sector4      *)
  OB_WRP_SECTOR_5 = ($00200000);  (*!< Write protection of Sector5      *)
  OB_WRP_SECTOR_6 = ($00400000);  (*!< Write protection of Sector6      *)
  OB_WRP_SECTOR_7 = ($00800000);  (*!< Write protection of Sector7      *)
  OB_WRP_SECTOR_All = ($00FF0000);  (*!< Write protection of all Sectors  *)

(* Extension Program operation functions  ************************************ *)
function HAL_FLASHEx_Erase(var pEraseInit: FLASH_EraseInitTypeDef; var SectorError: longword): HAL_StatusTypeDef;
function HAL_FLASHEx_Erase_IT(var pEraseInit: FLASH_EraseInitTypeDef): HAL_StatusTypeDef;
function HAL_FLASHEx_OBProgram(var pOBInit: FLASH_OBProgramInitTypeDef): HAL_StatusTypeDef;
procedure HAL_FLASHEx_OBGetConfig(var pOBInit: FLASH_OBProgramInitTypeDef);

procedure FLASH_Erase_Sector(Sector: longword; VoltageRange: byte);

implementation

uses
  stm32f7xx_hal_flash;

const
  SECTOR_MASK = ($FFFFFF07);
  FLASH_TIMEOUT_VALUE = (50000);  (* 50 s  *)

procedure FLASH_Erase_Sector(Sector: longword; VoltageRange: byte);
begin
end;

(**
  * @brief  Full erase of FLASH memory sectors
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.
  *          This parameter can be one of the following values:
  *            @arg VOLTAGE_RANGE_1: when the device voltage range is 1.8V to 2.1V,
  *                                  the operation will be done by byte (8-bit)
  *            @arg VOLTAGE_RANGE_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg VOLTAGE_RANGE_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg VOLTAGE_RANGE_4: when the device voltage range is 2.7V to 3.6V + External Vpp,
  *                                  the operation will be done by double word (64-bit)
  *
  * @retval HAL Status
  *)
procedure FLASH_MassErase(VoltageRange: byte);
var
  tmp_psize: longword;
begin
  tmp_psize := 0;

  (* if the previous operation is completed, proceed to erase all sectors *)
  FLASH.CR := FLASH.CR and (CR_PSIZE_MASK);
  FLASH.CR := FLASH.CR or tmp_psize;
  FLASH.CR := FLASH.CR or FLASH_CR_MER;
  FLASH.CR := FLASH.CR or FLASH_CR_STRT;
  (* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*)
  __DSB();
end;

(**
  * @brief  Enable the write protection of the desired bank1 or bank 2 sectors
  *
  * @note   When the memory read protection level is selected (RDP level := 1),
  *         it is not possible to program or erase the flash sector i if CortexM4
  *         debug features are connected or boot code is executed in RAM, even if nWRPi := 1
  * @note   Active value of nWRPi bits is inverted when PCROP mode is active (SPRMOD :=1).
  *
  * @param  WRPSector: specifies the sector(s) to be write protected.
  *          This parameter can be one of the following values:
  *            @arg WRPSector: A value between OB_WRP_SECTOR_0 and OB_WRP_SECTOR_7
  *            @arg OB_WRP_SECTOR_All
  *
  * @retval HAL FLASH State
  *)
function FLASH_OB_EnableWRP(WRPSector: longword): HAL_StatusTypeDef;
var
  status: HAL_StatusTypeDef;
begin
  status := HAL_OK;

  (* Wait for last operation to be completed *)
  status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

  if (status = HAL_OK) then
    (*Write protection enabled on sectors *)
    FLASH.OPTCR := FLASH.OPTCR and ((not WRPSector));

  exit(status);
end;

(**
  * @brief  Disable the write protection of the desired bank1 or bank 2 sectors
  *
  * @note   When the memory read protection level is selected (RDP level := 1),
  *         it is not possible to program or erase the flash sector i if CortexM4
  *         debug features are connected or boot code is executed in RAM, even if nWRPi := 1
  *
  * @param  WRPSector: specifies the sector(s) to be write protected.
  *          This parameter can be one of the following values:
  *            @arg WRPSector: A value between OB_WRP_SECTOR_0 and OB_WRP_SECTOR_7
  *            @arg OB_WRP_Sector_All
  *
  *
  * @retval HAL Status
  *)
function FLASH_OB_DisableWRP(WRPSector: longword): HAL_StatusTypeDef;
var
  status: HAL_StatusTypeDef;
begin
  status := HAL_OK;

  (* Wait for last operation to be completed *)
  status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

  if (status = HAL_OK) then
    (* Write protection disabled on sectors *)
    FLASH.OPTCR := FLASH.OPTCR or (WRPSector);

  exit(status);
end;

(**
  * @brief  Set the read protection level.
  * @param  Level: specifies the read protection level.
  *          This parameter can be one of the following values:
  *            @arg OB_RDP_LEVEL_0: No protection
  *            @arg OB_RDP_LEVEL_1: Read protection of the memory
  *            @arg OB_RDP_LEVEL_2: Full chip protection
  *
  * @note WARNING: When enabling OB_RDP level 2 it's no more possible to go back to level 1 or 0
  *
  * @retval HAL Status
  *)
function FLASH_OB_RDP_LevelConfig(Level: longword): HAL_StatusTypeDef;
var
  status: HAL_StatusTypeDef;
begin
  status := HAL_OK;

  (* Wait for last operation to be completed *)
  status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

  if (status = HAL_OK) then
    FLASH.OPTCR := (FLASH.OPTCR and (not FLASH_OPTCR_RDP)) or Level;

  exit(status);
end;

(**
  * @brief  Program the FLASH User Option Byte: IWDG_SW / RST_STOP / RST_STDBY.
  * @param  Wwdg: Selects the IWDG mode
  *          This parameter can be one of the following values:
  *            @arg OB_WWDG_SW: Software WWDG selected
  *            @arg OB_WWDG_HW: Hardware WWDG selected
  * @param  Iwdg: Selects the WWDG mode
  *          This parameter can be one of the following values:
  *            @arg OB_IWDG_SW: Software IWDG selected
  *            @arg OB_IWDG_HW: Hardware IWDG selected
  * @param  Stop: Reset event when entering STOP mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_STOP_NO_RST: No reset generated when entering in STOP
  *            @arg OB_STOP_RST: Reset generated when entering in STOP
  * @param  Stdby: Reset event when entering Standby mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_STDBY_NO_RST: No reset generated when entering in STANDBY
  *            @arg OB_STDBY_RST: Reset generated when entering in STANDBY
  * @param  Iwdgstop: Independent watchdog counter freeze in Stop mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_IWDG_STOP_FREEZE: Freeze IWDG counter in STOP
  *            @arg OB_IWDG_STOP_ACTIVE: IWDG counter active in STOP
  * @param  Iwdgstdby: Independent watchdog counter freeze in standby mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_IWDG_STDBY_FREEZE: Freeze IWDG counter in STANDBY
  *            @arg OB_IWDG_STDBY_ACTIVE: IWDG counter active in STANDBY
  * @retval HAL Status
  *)
function FLASH_OB_UserConfig(Wwdg, Iwdg, Stop, Stdby, Iwdgstop, Iwdgstdby: longword): HAL_StatusTypeDef;
var
  useroptionmask, useroptionvalue: longword;
  status: HAL_StatusTypeDef;
begin
  useroptionmask := $00;
  useroptionvalue := $00;

  (* Wait for last operation to be completed *)
  status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

  if (status = HAL_OK) then
  begin
    useroptionmask := (FLASH_OPTCR_WWDG_SW or FLASH_OPTCR_IWDG_SW or FLASH_OPTCR_nRST_STOP or FLASH_OPTCR_nRST_STDBY or FLASH_OPTCR_IWDG_STOP or FLASH_OPTCR_IWDG_STDBY);
    useroptionvalue := (Iwdg or Wwdg or Stop or Stdby or Iwdgstop or Iwdgstdby);

    (* Update User Option Byte *)
    FLASH.OPTCR := (FLASH.OPTCR and (not useroptionmask)) or useroptionvalue;
  end;

  exit(status);
end;

(**
  * @brief  Set the BOR Level.
  * @param  Level: specifies the Option Bytes BOR Reset Level.
  *          This parameter can be one of the following values:
  *            @arg OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  *            @arg OB_BOR_LEVEL2: Supply voltage ranges from 2.4 to 2.7 V
  *            @arg OB_BOR_LEVEL1: Supply voltage ranges from 2.1 to 2.4 V
  *            @arg OB_BOR_OFF: Supply voltage ranges from 1.62 to 2.1 V
  * @retval HAL Status
  *)
function FLASH_OB_BOR_LevelConfig(Level: byte): HAL_StatusTypeDef;
begin
  (* Set the BOR Level *)
  FLASH.OPTCR := (FLASH.OPTCR and (not FLASH_OPTCR_BOR_LEV)) or Level;
  exit(HAL_OK);
end;

(**
  * @brief  Configure Boot base address.
  *
  * @param   BootOption : specifies Boot base address depending from Boot pin := 0 or pin := 1
  *          This parameter can be one of the following values:
  *            @arg OPTIONBYTE_BOOTADDR_0 : Boot address based when Boot pin := 0
  *            @arg OPTIONBYTE_BOOTADDR_1 : Boot address based when Boot pin := 1
  * @param   Address: specifies Boot base address
  *          This parameter can be one of the following values:
  *            @arg OB_BOOTADDR_ITCM_RAM : Boot from ITCM RAM ($00000000)
  *            @arg OB_BOOTADDR_SYSTEM : Boot from System memory bootloader ($00100000)
  *            @arg OB_BOOTADDR_ITCM_FLASH : Boot from Flash on ITCM interface ($00200000)
  *            @arg OB_BOOTADDR_AXIM_FLASH : Boot from Flash on AXIM interface ($08000000)
  *            @arg OB_BOOTADDR_DTCM_RAM : Boot from DTCM RAM ($20000000)
  *            @arg OB_BOOTADDR_SRAM1 : Boot from SRAM1 ($20010000)
  *            @arg OB_BOOTADDR_SRAM2 : Boot from SRAM2 ($2004C000)
  *
  * @retval HAL Status
  *)
function FLASH_OB_BootAddressConfig(BootOption, Address: longword): HAL_StatusTypeDef;
var
  status: HAL_StatusTypeDef;
begin
  (* Wait for last operation to be completed *)
  status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

  if (status = HAL_OK) then
  begin
    if (BootOption = OPTIONBYTE_BOOTADDR_0) then
      FLASH.OPTCR1 := (FLASH.OPTCR1 and (not FLASH_OPTCR1_BOOT_ADD0)) or Address
    else
      FLASH.OPTCR1 := (FLASH.OPTCR1 and (not FLASH_OPTCR1_BOOT_ADD1)) or (Address shl 16);
  end;

  exit(status);
end;

(**
  * @brief  Return the FLASH User Option Byte value.
  * @retval longword FLASH User Option Bytes values: IWDG_SW(Bit0), RST_STOP(Bit1)
  *         and RST_STDBY(Bit2).
  *)
function FLASH_OB_GetUser: longword;
begin
  (* Return the User Option Byte *)
  exit((FLASH.OPTCR and $C00000F0));
end;

(**
  * @brief  Return the FLASH Write Protection Option Bytes value.
  * @retval longword FLASH Write Protection Option Bytes value
  *)
function FLASH_OB_GetWRP: longword;
begin
  (* Return the FLASH write protection Register value *)
  exit((FLASH.OPTCR and $00FF0000));
end;

(**
  * @brief  Returns the FLASH Read Protection level.
  * @retval FlagStatus FLASH ReadOut Protection Status:
  *         This parameter can be one of the following values:
  *            @arg OB_RDP_LEVEL_0: No protection
  *            @arg OB_RDP_LEVEL_1: Read protection of the memory
  *            @arg OB_RDP_LEVEL_2: Full chip protection
  *)
function FLASH_OB_GetRDP: byte;
var
  readstatus: byte;
begin
  readstatus := OB_RDP_LEVEL_0;

  if (((FLASH.OPTCR and FLASH_OPTCR_RDP) shr 8) = OB_RDP_LEVEL_0) then
    readstatus := OB_RDP_LEVEL_0
  else if (((FLASH.OPTCR and FLASH_OPTCR_RDP) shr 8) = OB_RDP_LEVEL_2) then
    readstatus := OB_RDP_LEVEL_2
  else
    readstatus := OB_RDP_LEVEL_1;

  exit(readstatus);
end;

(**
  * @brief  Returns the FLASH BOR level.
  * @retval longword The FLASH BOR level:
  *           - OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  *           - OB_BOR_LEVEL2: Supply voltage ranges from 2.4 to 2.7 V
  *           - OB_BOR_LEVEL1: Supply voltage ranges from 2.1 to 2.4 V
  *           - OB_BOR_OFF   : Supply voltage ranges from 1.62 to 2.1 V
  *)
function FLASH_OB_GetBOR: longword;
begin
  (* Return the FLASH BOR level *)
  exit((FLASH.OPTCR and $0C));
end;

(**
  * @brief  Configure Boot base address.
  *
  * @param   BootOption : specifies Boot base address depending from Boot pin := 0 or pin := 1
  *          This parameter can be one of the following values:
  *            @arg OPTIONBYTE_BOOTADDR_0 : Boot address based when Boot pin := 0
  *            @arg OPTIONBYTE_BOOTADDR_1 : Boot address based when Boot pin := 1
  *
  * @retval longword Boot Base Address:
  *            - OB_BOOTADDR_ITCM_RAM : Boot from ITCM RAM ($00000000)
  *            - OB_BOOTADDR_SYSTEM : Boot from System memory bootloader ($00100000)
  *            - OB_BOOTADDR_ITCM_FLASH : Boot from Flash on ITCM interface ($00200000)
  *            - OB_BOOTADDR_AXIM_FLASH : Boot from Flash on AXIM interface ($08000000)
  *            - OB_BOOTADDR_DTCM_RAM : Boot from DTCM RAM ($20000000)
  *            - OB_BOOTADDR_SRAM1 : Boot from SRAM1 ($20010000)
  *            - OB_BOOTADDR_SRAM2 : Boot from SRAM2 ($2004C000)
  *)
function FLASH_OB_GetBootAddress(BootOption: longword): longword;
var
  Address: longword;
begin
  (* Return the Boot base Address *)
  if (BootOption = OPTIONBYTE_BOOTADDR_0) then
    Address := FLASH.OPTCR1 and FLASH_OPTCR1_BOOT_ADD0
  else
    Address := ((FLASH.OPTCR1 and FLASH_OPTCR1_BOOT_ADD1) shr 16);

  exit(Address);
end;

function HAL_FLASHEx_Erase(var pEraseInit: FLASH_EraseInitTypeDef; var SectorError: longword): HAL_StatusTypeDef;
var
  status: HAL_StatusTypeDef;
  index: longword;
begin
  status := HAL_ERROR;
  index := 0;

  (* Process Locked *)
  __HAL_Lock(pFlash.lock);

  (* Wait for last operation to be completed *)
  status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

  if (status = HAL_OK) then
  begin
    (*Initialization of SectorError variable*)
    SectorError := $FFFFFFFF;

    if (pEraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE) then
    begin
      (*Mass erase to be done*)
      FLASH_MassErase(pEraseInit.VoltageRange);

      (* Wait for last operation to be completed *)
      status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

      (* if the erase operation is completed, disable the MER Bit *)
      FLASH.CR := FLASH.CR and ((not FLASH_MER_BIT));
    end
    else
    begin
      (* Erase by sector by sector to be done*)
      for index := pEraseInit.Sector to (pEraseInit.NbSectors + pEraseInit.Sector) - 1 do
      begin
        FLASH_Erase_Sector(index, pEraseInit.VoltageRange);

        (* Wait for last operation to be completed *)
        status := FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

        (* If the erase operation is completed, disable the SER Bit *)
        FLASH.CR := FLASH.CR and ((not FLASH_CR_SER));
        FLASH.CR := FLASH.CR and (SECTOR_MASK);

        if (status <> HAL_OK) then
        begin
          (* In case of error, stop erase procedure and return the faulty sector*)
          SectorError := index;
          break;
        end;
      end;
    end;
  end;

  (* Process Unlocked *)
  __HAL_Unlock(pFlash.lock);

  exit(status);
end;

function HAL_FLASHEx_Erase_IT(var pEraseInit: FLASH_EraseInitTypeDef): HAL_StatusTypeDef;
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

  if (pEraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE) then
  begin
    (*Mass erase to be done*)
    pFlash.ProcedureOnGoing := FLASH_PROC_MASSERASE;
    FLASH_MassErase(pEraseInit.VoltageRange);
  end
  else
  begin
    (* Erase by sector to be done*)

    pFlash.ProcedureOnGoing := FLASH_PROC_SECTERASE;
    pFlash.NbSectorsToErase := pEraseInit.NbSectors;
    pFlash.Sector := pEraseInit.Sector;
    pFlash.VoltageForErase := pEraseInit.VoltageRange;

    (*Erase 1st sector and wait for IT*)
    FLASH_Erase_Sector(pEraseInit.Sector, pEraseInit.VoltageRange);
  end;

  exit(status);
end;

function HAL_FLASHEx_OBProgram(var pOBInit: FLASH_OBProgramInitTypeDef): HAL_StatusTypeDef;
var
  status: HAL_StatusTypeDef;
begin
  status := HAL_ERROR;

  (* Process Locked *)
  __HAL_Lock(pFlash.lock);

  (* Write protection configuration *)
  if ((pOBInit.OptionType and OPTIONBYTE_WRP) = OPTIONBYTE_WRP) then
  begin
    if (pOBInit.WRPState = OB_WRPSTATE_ENABLE) then
      (*Enable of Write protection on the selected Sector*)
      status := FLASH_OB_EnableWRP(pOBInit.WRPSector)
    else
      (*Disable of Write protection on the selected Sector*)
      status := FLASH_OB_DisableWRP(pOBInit.WRPSector);
  end;

  (* Read protection configuration *)
  if ((pOBInit.OptionType and OPTIONBYTE_RDP) = OPTIONBYTE_RDP) then
    status := FLASH_OB_RDP_LevelConfig(pOBInit.RDPLevel);

  (* USER  configuration *)
  if ((pOBInit.OptionType and OPTIONBYTE_USER) = OPTIONBYTE_USER) then
    status := FLASH_OB_UserConfig(pOBInit.USERConfig and OB_WWDG_SW, pOBInit.USERConfig and OB_IWDG_SW, pOBInit.USERConfig and
      OB_STOP_NO_RST, pOBInit.USERConfig and OB_STDBY_NO_RST, pOBInit.USERConfig and OB_IWDG_STOP_ACTIVE, pOBInit.USERConfig and OB_IWDG_STDBY_ACTIVE);

  (* BOR Level  configuration *)
  if ((pOBInit.OptionType and OPTIONBYTE_BOR) = OPTIONBYTE_BOR) then
    status := FLASH_OB_BOR_LevelConfig(pOBInit.BORLevel);

  (* Boot 0 Address configuration *)
  if ((pOBInit.OptionType and OPTIONBYTE_BOOTADDR_0) = OPTIONBYTE_BOOTADDR_0) then
    status := FLASH_OB_BootAddressConfig(OPTIONBYTE_BOOTADDR_0, pOBInit.BootAddr0);

  (* Boot 1 Address configuration *)
  if ((pOBInit.OptionType and OPTIONBYTE_BOOTADDR_1) = OPTIONBYTE_BOOTADDR_1) then
    status := FLASH_OB_BootAddressConfig(OPTIONBYTE_BOOTADDR_1, pOBInit.BootAddr1);

  (* Process Unlocked *)
  __HAL_Unlock(pFlash.lock);

  exit(status);
end;

procedure HAL_FLASHEx_OBGetConfig(var pOBInit: FLASH_OBProgramInitTypeDef);
begin
  pOBInit.OptionType := OPTIONBYTE_WRP or OPTIONBYTE_RDP or OPTIONBYTE_USER or OPTIONBYTE_BOR or OPTIONBYTE_BOOTADDR_0 or OPTIONBYTE_BOOTADDR_1;

  (*Get WRP*)
  pOBInit.WRPSector := FLASH_OB_GetWRP();

  (*Get RDP Level*)
  pOBInit.RDPLevel := FLASH_OB_GetRDP();

  (*Get USER*)
  pOBInit.USERConfig := FLASH_OB_GetUser();

  (*Get BOR Level*)
  pOBInit.BORLevel := FLASH_OB_GetBOR();

  (*Get Boot Address when Boot pin := 0 *)
  pOBInit.BootAddr0 := FLASH_OB_GetBootAddress(OPTIONBYTE_BOOTADDR_0);

  (*Get Boot Address when Boot pin := 1 *)
  pOBInit.BootAddr1 := FLASH_OB_GetBootAddress(OPTIONBYTE_BOOTADDR_1);
end;

end.
