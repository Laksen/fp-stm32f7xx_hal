unit stm32f7xx_hal_pwr;

interface

uses
  stm32f7xx_defs;

type
  PWR_PVDTypeDef = packed record
    PVDLevel: longword;  (*!< PVDLevel: Specifies the PVD detection level.
                            This parameter can be a value of @ref PWR_PVD_detection_level  *)
    Mode: longword;  (*!< Mode: Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref PWR_PVD_Mode  *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup PWR_Exported_Constants PWR Exported Constants
  * @{
   *)

  (** @defgroup PWR_PVD_detection_level PWR PVD detection level
  * @{
   *)

const
  PWR_PVDLEVEL_0 = PWR_CR1_PLS_LEV0;
  PWR_PVDLEVEL_1 = PWR_CR1_PLS_LEV1;
  PWR_PVDLEVEL_2 = PWR_CR1_PLS_LEV2;
  PWR_PVDLEVEL_3 = PWR_CR1_PLS_LEV3;
  PWR_PVDLEVEL_4 = PWR_CR1_PLS_LEV4;
  PWR_PVDLEVEL_5 = PWR_CR1_PLS_LEV5;
  PWR_PVDLEVEL_6 = PWR_CR1_PLS_LEV6;
  PWR_PVDLEVEL_7 = PWR_CR1_PLS_LEV7;  (* External input analog voltage
                                                          (Compare internally to VREFINT)  *)
  (**
  * @}
   *)

  (** @defgroup PWR_PVD_Mode PWR PVD Mode
  * @{
   *)

  PWR_PVD_MODE_NORMAL = ($00000000);  (*!< basic mode is used  *)
  PWR_PVD_MODE_IT_RISING = ($00010001);  (*!< External Interrupt Mode with Rising edge trigger detection  *)
  PWR_PVD_MODE_IT_FALLING = ($00010002);  (*!< External Interrupt Mode with Falling edge trigger detection  *)
  PWR_PVD_MODE_IT_RISING_FALLING = ($00010003);  (*!< External Interrupt Mode with Rising/Falling edge trigger detection  *)
  PWR_PVD_MODE_EVENT_RISING = ($00020001);  (*!< Event Mode with Rising edge trigger detection  *)
  PWR_PVD_MODE_EVENT_FALLING = ($00020002);  (*!< Event Mode with Falling edge trigger detection  *)
  PWR_PVD_MODE_EVENT_RISING_FALLING = ($00020003);  (*!< Event Mode with Rising/Falling edge trigger detection  *)
  (**
  * @}
   *)

  (** @defgroup PWR_Regulator_state_in_STOP_mode PWR Regulator state in SLEEP/STOP mode
  * @{
   *)

  PWR_MAINREGULATOR_ON = ($00000000);
  PWR_LOWPOWERREGULATOR_ON = PWR_CR1_LPDS;
  (**
  * @}
   *)

  (** @defgroup PWR_SLEEP_mode_entry PWR SLEEP mode entry
  * @{
   *)

  PWR_SLEEPENTRY_WFI = ($01);
  PWR_SLEEPENTRY_WFE = ($02);
  (**
  * @}
   *)

  (** @defgroup PWR_STOP_mode_entry PWR STOP mode entry
  * @{
   *)

  PWR_STOPENTRY_WFI = ($01);
  PWR_STOPENTRY_WFE = ($02);
  (**
  * @}
   *)

  (** @defgroup PWR_Regulator_Voltage_Scale PWR Regulator Voltage Scale
  * @{
   *)

  PWR_REGULATOR_VOLTAGE_SCALE1 = PWR_CR1_VOS;
  PWR_REGULATOR_VOLTAGE_SCALE2 = PWR_CR1_VOS_1;
  PWR_REGULATOR_VOLTAGE_SCALE3 = PWR_CR1_VOS_0;
  (**
  * @}
   *)

  (** @defgroup PWR_Flag PWR Flag
  * @{
   *)

  PWR_FLAG_WU = PWR_CSR1_WUIF;
  PWR_FLAG_SB = PWR_CSR1_SBF;
  PWR_FLAG_PVDO = PWR_CSR1_PVDO;
  PWR_FLAG_BRR = PWR_CSR1_BRR;
  PWR_FLAG_VOSRDY = PWR_CSR1_VOSRDY;

  (**
  * @}
   *)


procedure __HAL_PWR_VOLTAGESCALING_CONFIG(__REGULATOR__: longword);
function __HAL_PWR_GET_FLAG(__FLAG__: longword): boolean;
procedure __HAL_PWR_CLEAR_FLAG(__FLAG__: longword);
procedure __HAL_PWR_PVD_EXTI_ENABLE_IT();
procedure __HAL_PWR_PVD_EXTI_DISABLE_IT();
procedure __HAL_PWR_PVD_EXTI_ENABLE_EVENT();
procedure __HAL_PWR_PVD_EXTI_DISABLE_EVENT();
procedure __HAL_PWR_PVD_EXTI_ENABLE_RISING_EDGE();
procedure __HAL_PWR_PVD_EXTI_DISABLE_RISING_EDGE();
procedure __HAL_PWR_PVD_EXTI_ENABLE_FALLING_EDGE();
procedure __HAL_PWR_PVD_EXTI_DISABLE_FALLING_EDGE();
procedure __HAL_PWR_PVD_EXTI_ENABLE_RISING_FALLING_EDGE();
procedure __HAL_PWR_PVD_EXTI_DISABLE_RISING_FALLING_EDGE();
function __HAL_PWR_PVD_EXTI_GET_FLAG(): boolean;
procedure __HAL_PWR_PVD_EXTI_CLEAR_FLAG();
procedure __HAL_PWR_PVD_EXTI_GENERATE_SWIT();

(* Initialization and de-initialization functions **************************** *)
procedure HAL_PWR_DeInit;
procedure HAL_PWR_EnableBkUpAccess;
procedure HAL_PWR_DisableBkUpAccess;
(**
  * @}
   *)

(** @addtogroup PWR_Exported_Functions_Group2 Peripheral Control functions
  * @{
   *)
(* Peripheral Control functions  ********************************************* *)
(* PVD configuration  *)
procedure HAL_PWR_ConfigPVD(var sConfigPVD: PWR_PVDTypeDef);
procedure HAL_PWR_EnablePVD;
procedure HAL_PWR_DisablePVD;

(* WakeUp pins configuration  *)
procedure HAL_PWR_EnableWakeUpPin(WakeUpPinPolarity: longword);
procedure HAL_PWR_DisableWakeUpPin(WakeUpPinx: longword);

(* Low Power modes entry  *)
procedure HAL_PWR_EnterSTOPMode(Regulator: longword; STOPEntry: byte);
procedure HAL_PWR_EnterSLEEPMode(Regulator: longword; SLEEPEntry: byte);
procedure HAL_PWR_EnterSTANDBYMode;

(* Power PVD IRQ Handler  *)
procedure HAL_PWR_PVD_IRQHandler;
procedure HAL_PWR_PVDCallback;

(* Cortex System Control functions  ****************************************** *)
procedure HAL_PWR_EnableSleepOnExit;
procedure HAL_PWR_DisableSleepOnExit;
procedure HAL_PWR_EnableSEVOnPend;
procedure HAL_PWR_DisableSEVOnPend;

const
  PVD_MODE_IT = ($00010000);
  PVD_MODE_EVT = ($00020000);
  PVD_RISING_EDGE = ($00000001);
  PVD_FALLING_EDGE = ($00000002);
  (**
  * @}
   *)

  (** @defgroup PWR_ENABLE_WUP_Mask PWR Enable WUP Mask
  * @{
   *)

  PWR_EWUP_MASK = ($00003F00);

const
  PWR_EXTI_LINE_PVD = (EXTI_IMR_MR16);  (*!< External interrupt line 16 Connected to the PVD EXTI Line  *)

implementation

uses
  cortexm4,
  stm32f7xx_hal_rcc,
  stm32f7xx_hal_rcc_ex;

(* SCB System Control Register Definitions  *)
const
  SCB_SCR_SEVONPEND_Pos = 4;  (*!< SCB SCR: SEVONPEND Position  *)
  SCB_SCR_SEVONPEND_Msk = ( 1 shl SCB_SCR_SEVONPEND_Pos );  (*!< SCB SCR: SEVONPEND Mask  *)

  SCB_SCR_SLEEPDEEP_Pos = 2;  (*!< SCB SCR: SLEEPDEEP Position  *)
  SCB_SCR_SLEEPDEEP_Msk = ( 1 shl SCB_SCR_SLEEPDEEP_Pos );  (*!< SCB SCR: SLEEPDEEP Mask  *)

  SCB_SCR_SLEEPONEXIT_Pos = 1;  (*!< SCB SCR: SLEEPONEXIT Position  *)
  SCB_SCR_SLEEPONEXIT_Msk = ( 1 shl SCB_SCR_SLEEPONEXIT_Pos );  (*!< SCB SCR: SLEEPONEXIT Mask  *)

procedure __HAL_PWR_VOLTAGESCALING_CONFIG(__REGULATOR__: longword);
var
  tmp: longword;
begin
  PWR.CR1 := (PWR.CR1 and (not PWR_CR1_VOS)) or __REGULATOR__;
  tmp := PWR.CR1;
end;

function __HAL_PWR_GET_FLAG(__FLAG__: longword): boolean;
begin
  exit((PWR.CSR1 and (__FLAG__)) = (__FLAG__));
end;

(** @brief  Clear the PWR's pending flags.
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be one of the following values:
  *            @arg PWR_FLAG_SB: StandBy flag
  *)
procedure __HAL_PWR_CLEAR_FLAG(__FLAG__: longword);
begin
  PWR.CR1 := PWR.CR1 or ((__FLAG__) shl 2);
end;

(**
  * @brief Enable the PVD Exti Line 16.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_ENABLE_IT();
begin
  EXTI.IMR := EXTI.IMR or (PWR_EXTI_LINE_PVD);
end;

(**
  * @brief Disable the PVD EXTI Line 16.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_DISABLE_IT();
begin
  EXTI.IMR := EXTI.IMR and (not (PWR_EXTI_LINE_PVD));
end;

(**
  * @brief Enable event on PVD Exti Line 16.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_ENABLE_EVENT();
begin
  EXTI.EMR := EXTI.EMR or (PWR_EXTI_LINE_PVD);
end;

(**
  * @brief Disable event on PVD Exti Line 16.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_DISABLE_EVENT();
begin
  EXTI.EMR := EXTI.EMR and (not (PWR_EXTI_LINE_PVD));
end;

(**
  * @brief Enable the PVD Extended Interrupt Rising Trigger.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_ENABLE_RISING_EDGE();
begin
  EXTI.RTSR := EXTI.RTSR or PWR_EXTI_LINE_PVD;
end;

(**
  * @brief Disable the PVD Extended Interrupt Rising Trigger.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_DISABLE_RISING_EDGE();
begin
  EXTI.RTSR := EXTI.RTSR and (not PWR_EXTI_LINE_PVD);
end;

(**
  * @brief Enable the PVD Extended Interrupt Falling Trigger.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_ENABLE_FALLING_EDGE();
begin
  EXTI.FTSR := EXTI.FTSR or PWR_EXTI_LINE_PVD;
end;


(**
  * @brief Disable the PVD Extended Interrupt Falling Trigger.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_DISABLE_FALLING_EDGE();
begin
  EXTI.FTSR := EXTI.FTSR and (not PWR_EXTI_LINE_PVD);
end;


(**
  * @brief  PVD EXTI line configuration: set rising and falling edge trigger.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_ENABLE_RISING_FALLING_EDGE();
begin
  __HAL_PWR_PVD_EXTI_ENABLE_RISING_EDGE();
  __HAL_PWR_PVD_EXTI_ENABLE_FALLING_EDGE();
  ;
end;

(**
  * @brief Disable the PVD Extended Interrupt Rising and Falling Trigger.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_DISABLE_RISING_FALLING_EDGE();
begin
  __HAL_PWR_PVD_EXTI_DISABLE_RISING_EDGE();
  __HAL_PWR_PVD_EXTI_DISABLE_FALLING_EDGE();
  ;
end;

(**
  * @brief checks whether the specified PVD Exti interrupt flag is set or not.
  * @retval EXTI PVD Line Status.
  *)
function __HAL_PWR_PVD_EXTI_GET_FLAG(): boolean;
begin
  exit((EXTI.PR and (PWR_EXTI_LINE_PVD)) <> 0);
end;

(**
  * @brief Clear the PVD Exti flag.
  * @retval None.
  *)
procedure __HAL_PWR_PVD_EXTI_CLEAR_FLAG();
begin
  EXTI.PR := (PWR_EXTI_LINE_PVD);
end;

(**
  * @brief  Generates a Software interrupt on PVD EXTI line.
  * @retval None
  *)
procedure __HAL_PWR_PVD_EXTI_GENERATE_SWIT();
begin
  EXTI.SWIER := EXTI.SWIER or (PWR_EXTI_LINE_PVD);
end;

procedure HAL_PWR_DeInit;
begin
  __HAL_RCC_PWR_FORCE_RESET();
  __HAL_RCC_PWR_RELEASE_RESET();
end;

procedure HAL_PWR_EnableBkUpAccess;
begin
  (* Enable access to RTC and backup registers  *)
  PWR.CR1:=PWR.CR1 or PWR_CR1_DBP;
end;

procedure HAL_PWR_DisableBkUpAccess;
begin
  (* Disable access to RTC and backup registers  *)
  PWR.CR1:=PWR.CR1 and (not PWR_CR1_DBP);
end;

procedure HAL_PWR_ConfigPVD(var sConfigPVD: PWR_PVDTypeDef);
begin
  (* Set PLS[7:5] bits according to PVDLevel value *)
  PWR.CR1:=(PWR.CR1 and (not PWR_CR1_PLS)) or sConfigPVD.PVDLevel;

  (* Clear any previous config. Keep it clear if no event or IT mode is selected *)
  __HAL_PWR_PVD_EXTI_DISABLE_EVENT();
  __HAL_PWR_PVD_EXTI_DISABLE_IT();
  __HAL_PWR_PVD_EXTI_DISABLE_RISING_EDGE();
  __HAL_PWR_PVD_EXTI_DISABLE_FALLING_EDGE();

  (* Configure interrupt mode *)
  if((sConfigPVD.Mode and PVD_MODE_IT) = PVD_MODE_IT) then
    __HAL_PWR_PVD_EXTI_ENABLE_IT();

  (* Configure event mode *)
  if((sConfigPVD.Mode and PVD_MODE_EVT) = PVD_MODE_EVT)then
    __HAL_PWR_PVD_EXTI_ENABLE_EVENT();

  (* Configure the edge *)
  if((sConfigPVD.Mode and PVD_RISING_EDGE) = PVD_RISING_EDGE)then
    __HAL_PWR_PVD_EXTI_ENABLE_RISING_EDGE();

  if((sConfigPVD.Mode and PVD_FALLING_EDGE) = PVD_FALLING_EDGE)  then
    __HAL_PWR_PVD_EXTI_ENABLE_FALLING_EDGE();
end;

procedure HAL_PWR_EnablePVD;
begin
  (* Enable the power voltage detector *)
	PWR.CR1:=PWR.CR1 or PWR_CR1_PVDE;
end;

procedure HAL_PWR_DisablePVD;
begin
  (* Disable the power voltage detector *)
	PWR.CR1:=(PWR.CR1 and (not PWR_CR1_PVDE));
end;

procedure HAL_PWR_EnableWakeUpPin(WakeUpPinPolarity: longword);
begin
  (* Enable wake-up pin *)
  PWR.CSR2 := PWR.CSR2 or longword((PWR_EWUP_MASK and WakeUpPinPolarity));

  (* Specifies the Wake-Up pin polarity for the event detection
    (rising or falling edge) *)
  PWR.CR2 := (PWR.CR2 and (not longword((PWR_EWUP_MASK and WakeUpPinPolarity)))) or (WakeUpPinPolarity shr $06);
end;

procedure HAL_PWR_DisableWakeUpPin(WakeUpPinx: longword);
begin
  PWR.CSR2 := PWR.CSR2 and (not longword(WakeUpPinx));
end;

procedure HAL_PWR_EnterSTOPMode(Regulator: longword; STOPEntry: byte);
var
  tmpreg: longword;
begin
  (* Select the regulator state in Stop mode ---------------------------------*)
  tmpreg := PWR.CR1;
  (* Clear PDDS and LPDS bits *)
  tmpreg := tmpreg and (not (PWR_CR1_PDDS or PWR_CR1_LPDS));

  (* Set LPDS, MRLVDS and LPLVDS bits according to Regulator value *)
  tmpreg := tmpreg or Regulator;

  (* Store the new value *)
  PWR.CR1 := tmpreg;

  (* Set SLEEPDEEP bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR or SCB_SCR_SLEEPDEEP_Msk;

  (* Select Stop mode entry --------------------------------------------------*)
  if(STOPEntry = PWR_STOPENTRY_WFI)then
  begin
    (* Request Wait For Interrupt *)
    __WFI();
  end
  else
  begin
    (* Request Wait For Event *)
    __SEV();
    __WFE();
    __WFE();
  end;
  (* Reset SLEEPDEEP bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR and (not (SCB_SCR_SLEEPDEEP_Msk));
end;

procedure HAL_PWR_EnterSLEEPMode(Regulator: longword; SLEEPEntry: byte);
begin
  (* Clear SLEEPDEEP bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR and (not longword((SCB_SCR_SLEEPDEEP_Msk)));

  (* Select SLEEP mode entry -------------------------------------------------*)
  if(SLEEPEntry = PWR_SLEEPENTRY_WFI)then
  begin
    (* Request Wait For Interrupt *)
    __WFI();
  end
  else
  begin
    (* Request Wait For Event *)
    __SEV();
    __WFE();
    __WFE();
  end;
end;

procedure HAL_PWR_EnterSTANDBYMode;
begin
  (* Select Standby mode *)
  PWR.CR1 := PWR.CR1 or PWR_CR1_PDDS;

  (* Set SLEEPDEEP bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR or SCB_SCR_SLEEPDEEP_Msk;

  (* Request Wait For Interrupt *)
  __WFI();
end;

procedure HAL_PWR_PVD_IRQHandler;
begin
  (* Check PWR Exti flag *)
  if(__HAL_PWR_PVD_EXTI_GET_FLAG() ) then
  begin
    (* PWR PVD interrupt user callback *)
    HAL_PWR_PVDCallback();

    (* Clear PWR Exti pending bit *)
    __HAL_PWR_PVD_EXTI_CLEAR_FLAG();
  end
end;

procedure HAL_PWR_PVDCallback;
begin

end;

procedure HAL_PWR_EnableSleepOnExit;
begin
  (* Set SLEEPONEXIT bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR or longword((SCB_SCR_SLEEPONEXIT_Msk));
end;

procedure HAL_PWR_DisableSleepOnExit;
begin
  (* Clear SLEEPONEXIT bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR and (not longword((SCB_SCR_SLEEPONEXIT_Msk)));
end;

procedure HAL_PWR_EnableSEVOnPend;
begin
  (* Set SEVONPEND bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR or longword((SCB_SCR_SEVONPEND_Msk));
end;

procedure HAL_PWR_DisableSEVOnPend;
begin
  (* Clear SEVONPEND bit of Cortex System Control Register *)
  SCB.SCR := SCB.SCR and (not longword((SCB_SCR_SEVONPEND_Msk)));
end;

end.
