(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_pwr_ex.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of PWR HAL Extension module.
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

unit stm32f7xx_hal_pwr_ex;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal;

const
  PWR_WAKEUP_PIN1 = PWR_CSR2_EWUP1;
  PWR_WAKEUP_PIN2 = PWR_CSR2_EWUP2;
  PWR_WAKEUP_PIN3 = PWR_CSR2_EWUP3;
  PWR_WAKEUP_PIN4 = PWR_CSR2_EWUP4;
  PWR_WAKEUP_PIN5 = PWR_CSR2_EWUP5;
  PWR_WAKEUP_PIN6 = PWR_CSR2_EWUP6;
  PWR_WAKEUP_PIN1_HIGH = PWR_CSR2_EWUP1;
  PWR_WAKEUP_PIN2_HIGH = PWR_CSR2_EWUP2;
  PWR_WAKEUP_PIN3_HIGH = PWR_CSR2_EWUP3;
  PWR_WAKEUP_PIN4_HIGH = PWR_CSR2_EWUP4;
  PWR_WAKEUP_PIN5_HIGH = PWR_CSR2_EWUP5;
  PWR_WAKEUP_PIN6_HIGH = PWR_CSR2_EWUP6;
  PWR_WAKEUP_PIN1_LOW = ((PWR_CR2_WUPP1 shl 6) or PWR_CSR2_EWUP1);
  PWR_WAKEUP_PIN2_LOW = ((PWR_CR2_WUPP2 shl 6) or PWR_CSR2_EWUP2);
  PWR_WAKEUP_PIN3_LOW = ((PWR_CR2_WUPP3 shl 6) or PWR_CSR2_EWUP3);
  PWR_WAKEUP_PIN4_LOW = ((PWR_CR2_WUPP4 shl 6) or PWR_CSR2_EWUP4);
  PWR_WAKEUP_PIN5_LOW = ((PWR_CR2_WUPP5 shl 6) or PWR_CSR2_EWUP5);
  PWR_WAKEUP_PIN6_LOW = ((PWR_CR2_WUPP6 shl 6) or PWR_CSR2_EWUP6);
  (**
  * @}
   *)

  (** @defgroup PWREx_Regulator_state_in_UnderDrive_mode PWREx Regulator state in UnderDrive mode
  * @{
   *)

  PWR_MAINREGULATOR_UNDERDRIVE_ON = PWR_CR1_MRUDS;
  PWR_LOWPOWERREGULATOR_UNDERDRIVE_ON = ((PWR_CR1_LPDS or PWR_CR1_LPUDS));
  (**
  * @}
   *)

  (** @defgroup PWREx_Over_Under_Drive_Flag PWREx Over Under Drive Flag
  * @{
   *)

  PWR_FLAG_ODRDY = PWR_CSR1_ODRDY;
  PWR_FLAG_ODSWRDY = PWR_CSR1_ODSWRDY;
  PWR_FLAG_UDRDY = PWR_CSR1_UDSWRDY;
  (**
  * @}
   *)

  (** @defgroup PWREx_Wakeup_Pins_Flag PWREx Wake Up Pin Flags
  * @{
   *)

  PWR_WAKEUP_PIN_FLAG1 = PWR_CSR2_WUPF1;
  PWR_WAKEUP_PIN_FLAG2 = PWR_CSR2_WUPF2;
  PWR_WAKEUP_PIN_FLAG3 = PWR_CSR2_WUPF3;
  PWR_WAKEUP_PIN_FLAG4 = PWR_CSR2_WUPF4;
  PWR_WAKEUP_PIN_FLAG5 = PWR_CSR2_WUPF5;
  PWR_WAKEUP_PIN_FLAG6 = PWR_CSR2_WUPF6;


procedure __HAL_PWR_OVERDRIVE_ENABLE;
procedure __HAL_PWR_OVERDRIVE_DISABLE;

procedure __HAL_PWR_OVERDRIVESWITCHING_ENABLE;
procedure __HAL_PWR_OVERDRIVESWITCHING_DISABLE;

procedure __HAL_PWR_UNDERDRIVE_ENABLE;
procedure __HAL_PWR_UNDERDRIVE_DISABLE;

function __HAL_PWR_GET_ODRUDR_FLAG(__FLAG__: longword): boolean;
procedure __HAL_PWR_CLEAR_ODRUDR_FLAG;

function __HAL_PWR_GET_WAKEUP_FLAG(__WUFLAG__: longword): longword;
procedure __HAL_PWR_CLEAR_WAKEUP_FLAG(__WUFLAG__: longword);

function HAL_PWREx_GetVoltageRange: longword;
function HAL_PWREx_ControlVoltageScaling(VoltageScaling: longword): HAL_StatusTypeDef;

procedure HAL_PWREx_EnableFlashPowerDown;
procedure HAL_PWREx_DisableFlashPowerDown;
function HAL_PWREx_EnableBkUpReg: HAL_StatusTypeDef;
function HAL_PWREx_DisableBkUpReg: HAL_StatusTypeDef;

procedure HAL_PWREx_EnableMainRegulatorLowVoltage;
procedure HAL_PWREx_DisableMainRegulatorLowVoltage;
procedure HAL_PWREx_EnableLowRegulatorLowVoltage;
procedure HAL_PWREx_DisableLowRegulatorLowVoltage;

function HAL_PWREx_EnableOverDrive: HAL_StatusTypeDef;
function HAL_PWREx_DisableOverDrive: HAL_StatusTypeDef;
function HAL_PWREx_EnterUnderDriveSTOPMode(Regulator: longword; STOPEntry: byte): HAL_StatusTypeDef;

implementation

uses
  cortexm4,
  stm32f7xx_hal_rcc,
  stm32f7xx_hal_pwr;

const
  SCB_SCR_SLEEPDEEP_Pos = 2;  (*!< SCB SCR: SLEEPDEEP Position  *)
  SCB_SCR_SLEEPDEEP_Msk = ( 1 shl SCB_SCR_SLEEPDEEP_Pos );  (*!< SCB SCR: SLEEPDEEP Mask  *)

const
  PWR_OVERDRIVE_TIMEOUT_VALUE = 1000;
  PWR_UDERDRIVE_TIMEOUT_VALUE = 1000;
  PWR_BKPREG_TIMEOUT_VALUE = 1000;
  PWR_VOSRDY_TIMEOUT_VALUE = 1000;

procedure __HAL_PWR_OVERDRIVE_ENABLE;
begin
  PWR.CR1 := PWR.CR1 or PWR_CR1_ODEN;
end;

procedure __HAL_PWR_OVERDRIVE_DISABLE;
begin
  PWR.CR1 := PWR.CR1 and (not PWR_CR1_ODEN);
end;

procedure __HAL_PWR_OVERDRIVESWITCHING_ENABLE;
begin
  PWR.CR1 := PWR.CR1 or PWR_CR1_ODSWEN;
end;

procedure __HAL_PWR_OVERDRIVESWITCHING_DISABLE;
begin
  PWR.CR1 := PWR.CR1 and (not PWR_CR1_ODSWEN);
end;

function __HAL_PWR_GET_FLAG(__FLAG__: longword): boolean;
begin
  exit((PWR.CSR1 and (__FLAG__)) = (__FLAG__));
end;

procedure __HAL_PWR_UNDERDRIVE_ENABLE;
begin
  PWR.CR1 := PWR.CR1 or PWR_CR1_UDEN;
end;

procedure __HAL_PWR_UNDERDRIVE_DISABLE;
begin
  PWR.CR1 := PWR.CR1 and ((not PWR_CR1_UDEN));
end;

function __HAL_PWR_GET_ODRUDR_FLAG(__FLAG__: longword): boolean;
begin
  exit((PWR.CSR1 and (__FLAG__)) = (__FLAG__));
end;

(** @brief Clear the Under-Drive Ready flag.
  *)
procedure __HAL_PWR_CLEAR_ODRUDR_FLAG;
begin
  PWR.CSR1 := PWR.CSR1 or PWR_FLAG_UDRDY;
end;

function __HAL_PWR_GET_WAKEUP_FLAG(__WUFLAG__: longword): longword;
begin
  exit(PWR.CSR2 and (__WUFLAG__));
end;

procedure __HAL_PWR_CLEAR_WAKEUP_FLAG(__WUFLAG__: longword);
begin
  PWR.CR2 := PWR.CR2 or (__WUFLAG__);
end;

function HAL_PWREx_GetVoltageRange: longword;
begin
  exit(PWR.CR1 and PWR_CR1_VOS);
end;

function HAL_PWREx_ControlVoltageScaling(VoltageScaling: longword): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  (* Enable Power ctrl clock *)
  __HAL_RCC_PWR_CLK_ENABLE();

  (* Check if the PLL is used as system clock or not *)
  if (__HAL_RCC_GET_SYSCLK_SOURCE() <> RCC_CFGR_SWS_PLL) then
  begin
    (* Disable the main PLL *)
    __HAL_RCC_PLL_DISABLE();

    (* Get Start Tick *)
    tickstart := HAL_GetTick();
    (* Wait till PLL is disabled *)
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)) do
      if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) then
        exit(HAL_TIMEOUT);

    (* Set Range *)
    __HAL_PWR_VOLTAGESCALING_CONFIG(VoltageScaling);

    (* Enable the main PLL *)
    __HAL_RCC_PLL_ENABLE();

    (* Get Start Tick *)
    tickstart := HAL_GetTick();
    (* Wait till PLL is ready *)
    while (not __HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)) do
      if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) then
        exit(HAL_TIMEOUT);

    (* Get Start Tick *)
    tickstart := HAL_GetTick();
    while ((not __HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))) do
      if ((HAL_GetTick() - tickstart) > PWR_VOSRDY_TIMEOUT_VALUE) then
        exit(HAL_TIMEOUT);
  end
  else
    exit(HAL_ERROR);

  exit(HAL_OK);
end;

procedure HAL_PWREx_EnableFlashPowerDown;
begin
  (* Enable the Flash Power Down *)
  PWR.CR1 := PWR.CR1 or PWR_CR1_FPDS;
end;

procedure HAL_PWREx_DisableFlashPowerDown;
begin
  (* Disable the Flash Power Down *)
  PWR.CR1 := PWR.CR1 and (not (PWR_CR1_FPDS));
end;

function HAL_PWREx_EnableBkUpReg: HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  (* Enable Backup regulator *)
  PWR.CSR1 := PWR.CSR1 or PWR_CSR1_BRE;

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Wait till Backup regulator ready flag is set *)
  while (not __HAL_PWR_GET_FLAG(PWR_FLAG_BRR)) do
    if ((HAL_GetTick() - tickstart) > PWR_BKPREG_TIMEOUT_VALUE) then
      exit(HAL_TIMEOUT);

  exit(HAL_OK);
end;

function HAL_PWREx_DisableBkUpReg: HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  (* Disable Backup regulator *)
  PWR.CSR1 := PWR.CSR1 and (not (PWR_CSR1_BRE));

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Wait till Backup regulator ready flag is set *)
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_BRR)) do
    if ((HAL_GetTick() - tickstart) > PWR_BKPREG_TIMEOUT_VALUE) then
      exit(HAL_TIMEOUT);

  exit(HAL_OK);
end;

procedure HAL_PWREx_EnableMainRegulatorLowVoltage;
begin
  (* Enable Main regulator low voltage *)
  PWR.CR1 := PWR.CR1 or PWR_CR1_MRUDS;
end;

procedure HAL_PWREx_DisableMainRegulatorLowVoltage;
begin
  (* Disable Main regulator low voltage *)
  PWR.CR1 := PWR.CR1 and (not (PWR_CR1_MRUDS));
end;

procedure HAL_PWREx_EnableLowRegulatorLowVoltage;
begin
  (* Enable low power regulator *)
  PWR.CR1 := PWR.CR1 or PWR_CR1_LPUDS;
end;

procedure HAL_PWREx_DisableLowRegulatorLowVoltage;
begin
  (* Disable low power regulator *)
  PWR.CR1 := PWR.CR1 and (not (PWR_CR1_LPUDS));
end;

function HAL_PWREx_EnableOverDrive: HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  __HAL_RCC_PWR_CLK_ENABLE();

  (* Enable the Over-drive to extend the clock frequency to 216 MHz *)
  __HAL_PWR_OVERDRIVE_ENABLE();

  (* Get tick *)
  tickstart := HAL_GetTick();

  while not __HAL_PWR_GET_FLAG(PWR_FLAG_ODRDY) do
    if (HAL_GetTick() - tickstart) > PWR_OVERDRIVE_TIMEOUT_VALUE then
      exit(HAL_TIMEOUT);

  (* Enable the Over-drive switch *)
  __HAL_PWR_OVERDRIVESWITCHING_ENABLE();

  (* Get tick *)
  tickstart := HAL_GetTick();

  while not __HAL_PWR_GET_FLAG(PWR_FLAG_ODSWRDY) do
    if (HAL_GetTick() - tickstart) > PWR_OVERDRIVE_TIMEOUT_VALUE then
      exit(HAL_TIMEOUT);

  exit(HAL_OK);
end;

function HAL_PWREx_DisableOverDrive: HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  __HAL_RCC_PWR_CLK_ENABLE();

  (* Disable the Over-drive switch *)
  __HAL_PWR_OVERDRIVESWITCHING_DISABLE();

  (* Get tick *)
  tickstart := HAL_GetTick();

  while (__HAL_PWR_GET_FLAG(PWR_FLAG_ODSWRDY)) do
    if ((HAL_GetTick() - tickstart) > PWR_OVERDRIVE_TIMEOUT_VALUE) then
      exit(HAL_TIMEOUT);

  (* Disable the Over-drive *)
  __HAL_PWR_OVERDRIVE_DISABLE();

  (* Get tick *)
  tickstart := HAL_GetTick();

  while (__HAL_PWR_GET_FLAG(PWR_FLAG_ODRDY)) do
    if ((HAL_GetTick() - tickstart) > PWR_OVERDRIVE_TIMEOUT_VALUE) then
      exit(HAL_TIMEOUT);

  exit(HAL_OK);
end;

function HAL_PWREx_EnterUnderDriveSTOPMode(Regulator: longword; STOPEntry: byte): HAL_StatusTypeDef;
var
  tickstart, tempreg: longword;
begin
  (* Enable Power ctrl clock *)
  __HAL_RCC_PWR_CLK_ENABLE();
  (* Enable the Under-drive Mode ---------------------------------------------*)
  (* Clear Under-drive flag *)
  __HAL_PWR_CLEAR_ODRUDR_FLAG();

  (* Enable the Under-drive *)
  __HAL_PWR_UNDERDRIVE_ENABLE();

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Wait for UnderDrive mode is ready *)
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_UDRDY)) do
    if ((HAL_GetTick() - tickstart) > PWR_UDERDRIVE_TIMEOUT_VALUE) then
      exit(HAL_TIMEOUT);

  (* Select the regulator state in STOP mode ---------------------------------*)
  tempreg := PWR.CR1;
  (* Clear PDDS, LPDS, MRLUDS and LPLUDS bits *)
  tempreg := tempreg and (not (PWR_CR1_PDDS or PWR_CR1_LPDS or PWR_CR1_LPUDS or PWR_CR1_MRUDS));

  (* Set LPDS, MRLUDS and LPLUDS bits according to PWR_Regulator value *)
  tempreg := tempreg or Regulator;

  (* Store the new value *)
  PWR.CR1 := tempreg;

  (* Set SLEEPDEEP bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR or SCB_SCR_SLEEPDEEP_Msk;

  (* Select STOP mode entry --------------------------------------------------*)
  if (STOPEntry = PWR_SLEEPENTRY_WFI) then
    (* Request Wait For Interrupt *)
    __WFI()
  else
    (* Request Wait For Event *)
    __WFE();
  (* Reset SLEEPDEEP bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR and (not (SCB_SCR_SLEEPDEEP_Msk));

  exit(HAL_OK);
end;

end.
