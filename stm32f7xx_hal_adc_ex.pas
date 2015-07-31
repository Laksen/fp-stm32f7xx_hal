(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_adc.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of ADC HAL module.
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

unit stm32f7xx_hal_adc_ex;

interface

uses
  stm32f7xx_hal,
  stm32f7xx_hal_adc,
  stm32f7xx_defs;

(**
  * @brief   ADC Configuration injected Channel structure definition
   *)

type
  ADC_InjectionConfTypeDef = record
    InjectedChannel: longword;  (*!< Configure the ADC injected channel.
                                                This parameter can be a value of @ref ADC_channels  *)
    InjectedRank: longword;  (*!< The rank in the injected group sequencer
                                                This parameter must be a number between Min_Data = 1 and Max_Data = 4.  *)
    InjectedSamplingTime: longword;  (*!< The sample time value to be set for the selected channel.
                                                This parameter can be a value of @ref ADC_sampling_times  *)
    InjectedOffset: longword;  (*!< Defines the offset to be subtracted from the raw converted data when convert injected channels.
                                                This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF.  *)
    InjectedNbrOfConversion: longword;  (*!< Specifies the number of ADC conversions that will be done using the sequencer for
                                                injected channel group.
                                                This parameter must be a number between Min_Data = 1 and Max_Data = 4.  *)
    AutoInjectedConv: longword;  (*!< Enables or disables the selected ADC automatic injected group
                                                conversion after regular one  *)
    InjectedDiscontinuousConvMode: longword;  (*!< Specifies whether the conversion is performed in Discontinuous mode or not for injected channels.
                                                This parameter can be set to ENABLE or DISABLE.  *)
    ExternalTrigInjecConvEdge: longword;  (*!< Select the external trigger edge and enable the trigger of an injected channels.
                                                This parameter can be a value of @ref ADCEx_External_trigger_edge_Injected  *)
    ExternalTrigInjecConv: longword;  (*!< Select the external event used to trigger the start of conversion of a injected channels.
                                                This parameter can be a value of @ref ADCEx_External_trigger_Source_Injected  *)
  end;

  (**
  * @brief   ADC Configuration multi-mode structure definition
   *)

  ADC_MultiModeTypeDef = record
    Mode: longword;  (*!< Configures the ADC to operate in independent or multi mode.
                                   This parameter can be a value of @ref ADCEx_Common_mode  *)
    DMAAccessMode: longword;  (*!< Configures the Direct memory access mode for multi ADC mode.
                                   This parameter can be a value of @ref ADCEx_Direct_memory_access_mode_for_multi_mode  *)
    TwoSamplingDelay: longword;  (*!< Configures the Delay between 2 sampling phases.
                                   This parameter can be a value of @ref ADC_delay_between_2_sampling_phases  *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup ADCEx_Exported_Constants ADC Exported Constants
  * @{
   *)

  (** @defgroup ADCEx_Common_mode ADC Common Mode
  * @{
   *)

const
  ADC_MODE_INDEPENDENT = ($00000000);
  ADC_DUALMODE_REGSIMULT_INJECSIMULT = (ADC_CCR_MULTI_0);
  ADC_DUALMODE_REGSIMULT_ALTERTRIG = (ADC_CCR_MULTI_1);
  ADC_DUALMODE_INJECSIMULT = ((ADC_CCR_MULTI_2 or ADC_CCR_MULTI_0));
  ADC_DUALMODE_REGSIMULT = ((ADC_CCR_MULTI_2 or ADC_CCR_MULTI_1));
  ADC_DUALMODE_INTERL = ((ADC_CCR_MULTI_2 or ADC_CCR_MULTI_1 or ADC_CCR_MULTI_0));
  ADC_DUALMODE_ALTERTRIG = ((ADC_CCR_MULTI_3 or ADC_CCR_MULTI_0));
  ADC_TRIPLEMODE_REGSIMULT_INJECSIMULT = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_0));
  ADC_TRIPLEMODE_REGSIMULT_AlterTrig = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_1));
  ADC_TRIPLEMODE_INJECSIMULT = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_2 or ADC_CCR_MULTI_0));
  ADC_TRIPLEMODE_REGSIMULT = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_2 or ADC_CCR_MULTI_1));
  ADC_TRIPLEMODE_INTERL = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_2 or ADC_CCR_MULTI_1 or ADC_CCR_MULTI_0));
  ADC_TRIPLEMODE_ALTERTRIG = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_3 or ADC_CCR_MULTI_0));
  (**
  * @}
   *)

  (** @defgroup ADCEx_Direct_memory_access_mode_for_multi_mode ADC Direct Memory Access Mode For Multi Mode
  * @{
   *)

  ADC_DMAACCESSMODE_DISABLED = ($00000000);  (*!< DMA mode disabled  *)
  ADC_DMAACCESSMODE_1 = (ADC_CCR_DMA_0);  (*!< DMA mode 1 enabled (2 / 3 half-words one by one - 1 then 2 then 3) *)
  ADC_DMAACCESSMODE_2 = (ADC_CCR_DMA_1);  (*!< DMA mode 2 enabled (2 / 3 half-words by pairs - 2&1 then 1&3 then 3&2) *)
  ADC_DMAACCESSMODE_3 = (ADC_CCR_DMA);  (*!< DMA mode 3 enabled (2 / 3 bytes by pairs - 2&1 then 1&3 then 3&2)  *)
  (**
  * @}
   *)

  (** @defgroup ADCEx_External_trigger_edge_Injected ADC External Trigger Edge Injected
  * @{
   *)

  ADC_EXTERNALTRIGINJECCONVEDGE_NONE = ($00000000);
  ADC_EXTERNALTRIGINJECCONVEDGE_RISING = (ADC_CR2_JEXTEN_0);
  ADC_EXTERNALTRIGINJECCONVEDGE_FALLING = (ADC_CR2_JEXTEN_1);
  ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING = (ADC_CR2_JEXTEN);
  (**
  * @}
   *)

  (** @defgroup ADCEx_External_trigger_Source_Injected ADC External Trigger Source Injected
  * @{
   *)

  ADC_EXTERNALTRIGINJECCONV_T1_TRGO = ($00000000);
  ADC_EXTERNALTRIGINJECCONV_T1_CC4 = (ADC_CR2_JEXTSEL_0);
  ADC_EXTERNALTRIGINJECCONV_T2_TRGO = (ADC_CR2_JEXTSEL_1);
  ADC_EXTERNALTRIGINJECCONV_T2_CC1 = ((ADC_CR2_JEXTSEL_1 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T3_CC4 = (ADC_CR2_JEXTSEL_2);
  ADC_EXTERNALTRIGINJECCONV_T4_TRGO = ((ADC_CR2_JEXTSEL_2 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T8_CC4 = ((ADC_CR2_JEXTSEL_2 or ADC_CR2_JEXTSEL_1 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T1_TRGO2 = (ADC_CR2_JEXTSEL_3);
  ADC_EXTERNALTRIGINJECCONV_T8_TRGO = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T8_TRGO2 = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_1));
  ADC_EXTERNALTRIGINJECCONV_T3_CC3 = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_1 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T5_TRGO = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_2));
  ADC_EXTERNALTRIGINJECCONV_T3_CC1 = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_2 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T6_TRGO = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_2 or ADC_CR2_JEXTSEL_1));
  (**
  * @}
   *)

  (** @defgroup ADCEx_injected_channel_selection ADC Injected Channel Selection
  * @{
   *)

  ADC_INJECTED_RANK_1 = ($00000001);
  ADC_INJECTED_RANK_2 = ($00000002);
  ADC_INJECTED_RANK_3 = ($00000003);
  ADC_INJECTED_RANK_4 = ($00000004);
  (**
  * @}
   *)

  (** @defgroup ADCEx_channels  ADC Specific Channels
  * @{
   *)

  ADC_CHANNEL_TEMPSENSOR = (ADC_CHANNEL_16);

(* I/O operation functions ***************************************************** *)
function HAL_ADCEx_InjectedStart(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADCEx_InjectedStop(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADCEx_InjectedPollForConversion(var hadc: ADC_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
function HAL_ADCEx_InjectedStart_IT(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADCEx_InjectedStop_IT(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADCEx_InjectedGetValue(var hadc: ADC_HandleTypeDef; InjectedRank: longword): longword;
function HAL_ADCEx_MultiModeStart_DMA(var hadc: ADC_HandleTypeDef; pData: Plongword; Length: longword): HAL_StatusTypeDef;
function HAL_ADCEx_MultiModeStop_DMA(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ADCEx_MultiModeGetValue(var hadc: ADC_HandleTypeDef): longword;
procedure HAL_ADCEx_InjectedConvCpltCallback(var hadc: ADC_HandleTypeDef); external name 'HAL_ADCEx_InjectedConvCpltCallback';

(* Peripheral Control functions ************************************************ *)
function HAL_ADCEx_InjectedConfigChannel(var hadc: ADC_HandleTypeDef; var sConfigInjected: ADC_InjectionConfTypeDef): HAL_StatusTypeDef;
function HAL_ADCEx_MultiModeConfigChannel(var hadc: ADC_HandleTypeDef; var multimode: ADC_MultiModeTypeDef): HAL_StatusTypeDef;

function ADC_JSQR(_CHANNELNB_, _RANKNB_, _JSQR_JL_: longword): longword;

implementation

uses
  stm32f7xx_hal_dma;

procedure ADC_MultiModeDMAConvCplt(var hdma: DMA_HandleTypeDef);
var
  hadc: PADC_HandleTypeDef;
begin
  hadc := PADC_HandleTypeDef(hdma.Parent);
  (* Check if an injected conversion is ready *)
  if (hadc^.State = HAL_ADC_STATE_EOC_INJ) then
  begin
    (* Change ADC state *)
    hadc^.State := HAL_ADC_STATE_EOC_INJ_REG;
  end
  else
  begin
    (* Change ADC state *)
    hadc^.State := HAL_ADC_STATE_EOC_REG;
  end;

  HAL_ADC_ConvCpltCallback(hadc^);
end;

procedure ADC_MultiModeDMAError(var hdma: DMA_HandleTypeDef);
var
  hadc: PADC_HandleTypeDef;
begin
  hadc := PADC_HandleTypeDef(hdma.Parent);
  (* Conversion complete callback *)
  HAL_ADC_ConvHalfCpltCallback(hadc^);
end;

procedure ADC_MultiModeDMAHalfConvCplt(var hdma: DMA_HandleTypeDef);
var
  hadc: PADC_HandleTypeDef;
begin
  hadc := PADC_HandleTypeDef(hdma.Parent);

  hadc^.State := HAL_ADC_STATE_ERROR;
  (* Set ADC error code to DMA error *)
  hadc^.ErrorCode := hadc^.ErrorCode or HAL_ADC_ERROR_DMA;
  HAL_ADC_ErrorCallback(hadc^);
end;

function HAL_ADCEx_InjectedStart(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
var
  counter: longword;
  tmp1, tmp2: boolean;
begin
  counter := 0;

  (* Process locked *)
  __HAL_Lock(hadc.lock);

  (* Check if a regular conversion is ongoing *)
  if (hadc.State = HAL_ADC_STATE_BUSY_REG) then
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_BUSY_INJ_REG;
  end
  else
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_BUSY_INJ;
  end;

  (* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization *)
  if ((hadc.Instance^.CR2 and ADC_CR2_ADON) <> ADC_CR2_ADON) then
  begin
    (* Enable the Peripheral *)
    __HAL_ADC_ENABLE(hadc);

    (* Delay for temperature sensor stabilization time *)
    (* Compute number of CPU cycles to wait for *)
    counter := (ADC_STAB_DELAY_US * (SystemCoreClock div 1000000));
    while (counter <> 0) do
    begin
      Dec(counter);
    end;
  end;

  (* Check if Multimode enabled *)
  if (HAL_IS_BIT_CLR(ADC.CCR, ADC_CCR_MULTI)) then
  begin
    tmp1 := HAL_IS_BIT_CLR(hadc.Instance^.CR2, ADC_CR2_JEXTEN);
    tmp2 := HAL_IS_BIT_CLR(hadc.Instance^.CR1, ADC_CR1_JAUTO);
    if (tmp1 and tmp2) then
    begin
      (* Enable the selected ADC software conversion for injected group *)
      hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_JSWSTART;
    end;
  end
  else
  begin
    tmp1 := HAL_IS_BIT_CLR(hadc.Instance^.CR2, ADC_CR2_JEXTEN);
    tmp2 := HAL_IS_BIT_CLR(hadc.Instance^.CR1, ADC_CR1_JAUTO);
    if ((hadc.Instance = @ADC1) and tmp1 and tmp2) then
    begin
      (* Enable the selected ADC software conversion for injected group *)
      hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_JSWSTART;
    end;
  end;

  (* Process unlocked *)
  __HAL_Unlock(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);

end;

function HAL_ADCEx_InjectedStop(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Disable the Peripheral *)
  __HAL_ADC_DISABLE(hadc);

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_READY;

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADCEx_InjectedPollForConversion(var hadc: ADC_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Check End of conversion flag *)
  while (not (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC))) do
  begin
    (* Check for the Timeout *)
    if (Timeout <> HAL_MAX_DELAY) then
    begin
      if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
      begin
        hadc.State := HAL_ADC_STATE_TIMEOUT;
        (* Process unlocked *)
        __HAL_Unlock(hadc.lock);
        exit(HAL_TIMEOUT);
      end;
    end;
  end;

  (* Check if a regular conversion is ready *)
  if (hadc.State = HAL_ADC_STATE_EOC_REG) then
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_EOC_INJ_REG;
  end
  else
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_EOC_INJ;
  end;

  (* Return ADC state *)
  exit(HAL_OK);

end;

function HAL_ADCEx_InjectedStart_IT(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
var
  counter: longword;
  tmp1, tmp2: boolean;
begin
  counter := 0;

  (* Process locked *)
  __HAL_Lock(hadc.lock);

  (* Check if a regular conversion is ongoing *)
  if (hadc.State = HAL_ADC_STATE_BUSY_REG) then
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_BUSY_INJ_REG;
  end
  else
  begin
    (* Change ADC state *)
    hadc.State := HAL_ADC_STATE_BUSY_INJ;
  end;

  (* Set ADC error code to none *)
  hadc.ErrorCode := HAL_ADC_ERROR_NONE;

  (* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization *)
  if ((hadc.Instance^.CR2 and ADC_CR2_ADON) <> ADC_CR2_ADON) then
  begin
    (* Enable the Peripheral *)
    __HAL_ADC_ENABLE(hadc);

    (* Delay for temperature sensor stabilization time *)
    (* Compute number of CPU cycles to wait for *)
    counter := (ADC_STAB_DELAY_US * (SystemCoreClock div 1000000));
    while (counter <> 0) do
    begin
      Dec(counter);
    end;
  end;

  (* Enable the ADC end of conversion interrupt for injected group *)
  __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);

  (* Enable the ADC overrun interrupt *)
  __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

  (* Check if Multimode enabled *)
  if (HAL_IS_BIT_CLR(ADC.CCR, ADC_CCR_MULTI)) then
  begin
    tmp1 := HAL_IS_BIT_CLR(hadc.Instance^.CR2, ADC_CR2_JEXTEN);
    tmp2 := HAL_IS_BIT_CLR(hadc.Instance^.CR1, ADC_CR1_JAUTO);
    if (tmp1 and tmp2) then
    begin
      (* Enable the selected ADC software conversion for injected group *)
      hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_JSWSTART;
    end;
  end
  else
  begin
    tmp1 := HAL_IS_BIT_CLR(hadc.Instance^.CR2, ADC_CR2_JEXTEN);
    tmp2 := HAL_IS_BIT_CLR(hadc.Instance^.CR1, ADC_CR1_JAUTO);
    if ((hadc.Instance = @ADC1) and tmp1 and tmp2) then
    begin
      (* Enable the selected ADC software conversion for injected group *)
      hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_JSWSTART;
    end;
  end;

  (* Process unlocked *)
  __HAL_Unlock(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);

end;

function HAL_ADCEx_InjectedStop_IT(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Disable the ADC end of conversion interrupt for regular group *)
  __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);

  (* Disable the ADC end of conversion interrupt for injected group *)
  __HAL_ADC_DISABLE_IT(hadc, ADC_CR1_JEOCIE);

  (* Enable the Peripheral *)
  __HAL_ADC_DISABLE(hadc);

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_READY;

  (* Return function status *)
  exit(HAL_OK);

end;

function HAL_ADCEx_InjectedGetValue(var hadc: ADC_HandleTypeDef; InjectedRank: longword): longword;
var
  tmp: longword;
begin
  (* Clear the ADCx's flag for injected end of conversion *)
  __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);

  (* Return the selected ADC converted value *)
  case InjectedRank of
    ADC_INJECTED_RANK_4:
      tmp := hadc.Instance^.JDR4;
    ADC_INJECTED_RANK_3:
      tmp := hadc.Instance^.JDR3;
    ADC_INJECTED_RANK_2:
      tmp := hadc.Instance^.JDR2;
    ADC_INJECTED_RANK_1:
      tmp := hadc.Instance^.JDR1;
  end;
  exit(tmp);
end;

function HAL_ADCEx_MultiModeStart_DMA(var hadc: ADC_HandleTypeDef; pData: Plongword; Length: longword): HAL_StatusTypeDef;
var
  counter: longword;
begin
  (* Process locked *)
  __HAL_Lock(hadc.lock);

  (* Enable ADC overrun interrupt *)
  __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

  if (hadc.Init.DMAContinuousRequests <> 0) then
  begin
    (* Enable the selected ADC DMA request after last transfer *)
    ADC.CCR := ADC.CCR or ADC_CCR_DDS;
  end
  else
  begin
    (* Disable the selected ADC EOC rising on each regular channel conversion *)
    ADC.CCR := ADC.CCR and (not ADC_CCR_DDS);
  end;

  (* Set the DMA transfer complete callback *)
  hadc.DMA_Handle^.XferCpltCallback := @ADC_MultiModeDMAConvCplt;

  (* Set the DMA half transfer complete callback *)
  hadc.DMA_Handle^.XferHalfCpltCallback := @ADC_MultiModeDMAHalfConvCplt;

  (* Set the DMA error callback *)
  hadc.DMA_Handle^.XferErrorCallback := @ADC_MultiModeDMAError;

  (* Enable the DMA Stream *)
  HAL_DMA_Start_IT(hadc.DMA_Handle^, @ADC.CDR, pData, Length);

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_BUSY_REG;

  (* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization *)
  if ((hadc.Instance^.CR2 and ADC_CR2_ADON) <> ADC_CR2_ADON) then
  begin
    (* Enable the Peripheral *)
    __HAL_ADC_ENABLE(hadc);

    (* Delay for temperature sensor stabilization time *)
    (* Compute number of CPU cycles to wait for *)
    counter := (ADC_STAB_DELAY_US * (SystemCoreClock div 1000000));
    while (counter <> 0) do
    begin
      Dec(counter);
    end;
  end;

  (* if no external trigger present enable software conversion of regular channels *)
  if ((hadc.Instance^.CR2 and ADC_CR2_EXTEN) = 0) then
  begin
    (* Enable the selected ADC software conversion for regular group *)
    hadc.Instance^.CR2 := hadc.Instance^.CR2 or ADC_CR2_SWSTART;
  end;

  (* Process unlocked *)
  __HAL_Unlock(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADCEx_MultiModeStop_DMA(var hadc: ADC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hadc.lock);

  (* Enable the Peripheral *)
  __HAL_ADC_DISABLE(hadc);

  (* Disable ADC overrun interrupt *)
  __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

  (* Disable the selected ADC DMA request after last transfer *)
  ADC.CCR := ADC.CCR and (not ADC_CCR_DDS);

  (* Disable the ADC DMA Stream *)
  HAL_DMA_Abort(hadc.DMA_Handle^);

  (* Change ADC state *)
  hadc.State := HAL_ADC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADCEx_MultiModeGetValue(var hadc: ADC_HandleTypeDef): longword;
begin
  (* Return the multi mode conversion value *)
  exit(ADC.CDR);
end;

function HAL_ADCEx_InjectedConfigChannel(var hadc: ADC_HandleTypeDef; var sConfigInjected: ADC_InjectionConfTypeDef): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hadc.lock);

  (* if ADC_Channel_10 ... ADC_Channel_18 is selected *)
  if (sConfigInjected.InjectedChannel > ADC_CHANNEL_9) then
  begin
    (* Clear the old sample time *)
    hadc.Instance^.SMPR1 := hadc.Instance^.SMPR1 and (not ADC_SMPR1(ADC_SMPR1_SMP10, sConfigInjected.InjectedChannel));

    (* Set the new sample time *)
    hadc.Instance^.SMPR1 := hadc.Instance^.SMPR1 or ADC_SMPR1(sConfigInjected.InjectedSamplingTime, sConfigInjected.InjectedChannel);
  end
  else (* ADC_Channel include in ADC_Channel_[0..9] *)
  begin
    (* Clear the old sample time *)
    hadc.Instance^.SMPR2 := hadc.Instance^.SMPR2 and (not ADC_SMPR2(ADC_SMPR2_SMP0, sConfigInjected.InjectedChannel));

    (* Set the new sample time *)
    hadc.Instance^.SMPR2 := hadc.Instance^.SMPR2 or ADC_SMPR2(sConfigInjected.InjectedSamplingTime, sConfigInjected.InjectedChannel);
  end;

  (*---------------------------- ADCx JSQR Configuration -----------------*)
  hadc.Instance^.JSQR := hadc.Instance^.JSQR and (not (ADC_JSQR_JL));
  hadc.Instance^.JSQR := hadc.Instance^.JSQR or ADC_SQR1(sConfigInjected.InjectedNbrOfConversion);

  (* Rank configuration *)

  (* Clear the old SQx bits for the selected rank *)
  hadc.Instance^.JSQR := hadc.Instance^.JSQR and (not ADC_JSQR(ADC_JSQR_JSQ1, sConfigInjected.InjectedRank, sConfigInjected.InjectedNbrOfConversion));

  (* Set the SQx bits for the selected rank *)
  hadc.Instance^.JSQR := hadc.Instance^.JSQR or ADC_JSQR(sConfigInjected.InjectedChannel, sConfigInjected.InjectedRank, sConfigInjected.InjectedNbrOfConversion);

  (* Select external trigger to start conversion *)
  hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_JEXTSEL));
  hadc.Instance^.CR2 := hadc.Instance^.CR2 or sConfigInjected.ExternalTrigInjecConv;

  (* Select external trigger polarity *)
  hadc.Instance^.CR2 := hadc.Instance^.CR2 and (not (ADC_CR2_JEXTEN));
  hadc.Instance^.CR2 := hadc.Instance^.CR2 or sConfigInjected.ExternalTrigInjecConvEdge;

  if (sConfigInjected.AutoInjectedConv <> 0) then
  begin
    (* Enable the selected ADC automatic injected group conversion *)
    hadc.Instance^.CR1 := hadc.Instance^.CR1 or ADC_CR1_JAUTO;
  end
  else
  begin
    (* Disable the selected ADC automatic injected group conversion *)
    hadc.Instance^.CR1 := hadc.Instance^.CR1 and (not (ADC_CR1_JAUTO));
  end;

  if (sConfigInjected.InjectedDiscontinuousConvMode <> 0) then
  begin
    (* Enable the selected ADC injected discontinuous mode *)
    hadc.Instance^.CR1 := hadc.Instance^.CR1 or ADC_CR1_JDISCEN;
  end
  else
  begin
    (* Disable the selected ADC injected discontinuous mode *)
    hadc.Instance^.CR1 := hadc.Instance^.CR1 and (not (ADC_CR1_JDISCEN));
  end;

  case sConfigInjected.InjectedRank of
    1:
    begin
      (* Set injected channel 1 offset *)
      hadc.Instance^.JOFR1 := hadc.Instance^.JOFR1 and (not (ADC_JOFR1_JOFFSET1));
      hadc.Instance^.JOFR1 := hadc.Instance^.JOFR1 or sConfigInjected.InjectedOffset;
    end;
    2:
    begin
      (* Set injected channel 2 offset *)
      hadc.Instance^.JOFR2 := hadc.Instance^.JOFR2 and (not (ADC_JOFR2_JOFFSET2));
      hadc.Instance^.JOFR2 := hadc.Instance^.JOFR2 or sConfigInjected.InjectedOffset;
    end;
    3:
    begin
      (* Set injected channel 3 offset *)
      hadc.Instance^.JOFR3 := hadc.Instance^.JOFR3 and (not (ADC_JOFR3_JOFFSET3));
      hadc.Instance^.JOFR3 := hadc.Instance^.JOFR3 or sConfigInjected.InjectedOffset;
    end;
    else
    begin
      (* Set injected channel 4 offset *)
      hadc.Instance^.JOFR4 := hadc.Instance^.JOFR4 and (not (ADC_JOFR4_JOFFSET4));
      hadc.Instance^.JOFR4 := hadc.Instance^.JOFR4 or sConfigInjected.InjectedOffset;
    end;
  end;

  (* if ADC1 Channel_18 is selected enable VBAT Channel *)
  if ((hadc.Instance = @ADC1) and (sConfigInjected.InjectedChannel = ADC_CHANNEL_VBAT)) then
  begin
    (* Enable the VBAT channel*)
    ADC.CCR := ADC.CCR or ADC_CCR_VBATE;
  end;

  (* if ADC1 Channel_16 or Channel_17 is selected enable TSVREFE Channel(Temperature sensor and VREFINT) *)
  if ((hadc.Instance = @ADC1) and ((sConfigInjected.InjectedChannel = ADC_CHANNEL_TEMPSENSOR) or (sConfigInjected.InjectedChannel = ADC_CHANNEL_VREFINT))) then
  begin
    (* Enable the TSVREFE channel*)
    ADC.CCR := ADC.CCR or ADC_CCR_TSVREFE;
  end;

  (* Process unlocked *)
  __HAL_Unlock(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ADCEx_MultiModeConfigChannel(var hadc: ADC_HandleTypeDef; var multimode: ADC_MultiModeTypeDef): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hadc.lock);

  (* Set ADC mode *)
  ADC.CCR := ADC.CCR and (not (ADC_CCR_MULTI));
  ADC.CCR := ADC.CCR or multimode.Mode;

  (* Set the ADC DMA access mode *)
  ADC.CCR := ADC.CCR and (not (ADC_CCR_DMA));
  ADC.CCR := ADC.CCR or multimode.DMAAccessMode;

  (* Set delay between two sampling phases *)
  ADC.CCR := ADC.CCR and (not (ADC_CCR_DELAY));
  ADC.CCR := ADC.CCR or multimode.TwoSamplingDelay;

  (* Process unlocked *)
  __HAL_Unlock(hadc.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function ADC_JSQR(_CHANNELNB_, _RANKNB_, _JSQR_JL_: longword): longword;
begin
  exit((((_CHANNELNB_))) shl (5 * (((_RANKNB_) + 3) - (_JSQR_JL_))));
end;

procedure HAL_ADCEx_InjectedConvCpltCallback_stub(var hadc: ADC_HandleTypeDef); assembler; nostackframe; public name 'HAL_ADCEx_InjectedConvCpltCallback';
  asm
    .weak HAL_ADCEx_InjectedConvCpltCallback
  end;

end.

