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
  stm32f7xx_hal_adc, stm32f7xx_defs;

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
  ADC_MODE_INDEPENDENT       = ($00000000);
  ADC_DUALMODE_REGSIMULT_INJECSIMULT = (ADC_CCR_MULTI_0);
  ADC_DUALMODE_REGSIMULT_ALTERTRIG = (ADC_CCR_MULTI_1);
  ADC_DUALMODE_INJECSIMULT   = ((ADC_CCR_MULTI_2 or ADC_CCR_MULTI_0));
  ADC_DUALMODE_REGSIMULT     = ((ADC_CCR_MULTI_2 or ADC_CCR_MULTI_1));
  ADC_DUALMODE_INTERL        = ((ADC_CCR_MULTI_2 or ADC_CCR_MULTI_1 or ADC_CCR_MULTI_0));
  ADC_DUALMODE_ALTERTRIG     = ((ADC_CCR_MULTI_3 or ADC_CCR_MULTI_0));
  ADC_TRIPLEMODE_REGSIMULT_INJECSIMULT = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_0));
  ADC_TRIPLEMODE_REGSIMULT_AlterTrig = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_1));
  ADC_TRIPLEMODE_INJECSIMULT = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_2 or ADC_CCR_MULTI_0));
  ADC_TRIPLEMODE_REGSIMULT   = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_2 or ADC_CCR_MULTI_1));
  ADC_TRIPLEMODE_INTERL      = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_2 or ADC_CCR_MULTI_1 or ADC_CCR_MULTI_0));
  ADC_TRIPLEMODE_ALTERTRIG   = ((ADC_CCR_MULTI_4 or ADC_CCR_MULTI_3 or ADC_CCR_MULTI_0));
  (**
  * @}
   *)

  (** @defgroup ADCEx_Direct_memory_access_mode_for_multi_mode ADC Direct Memory Access Mode For Multi Mode
  * @{
   *)

  ADC_DMAACCESSMODE_DISABLED = ($00000000);  (*!< DMA mode disabled  *)
  ADC_DMAACCESSMODE_1        = (ADC_CCR_DMA_0);  (*!< DMA mode 1 enabled (2 / 3 half-words one by one - 1 then 2 then 3) *)
  ADC_DMAACCESSMODE_2        = (ADC_CCR_DMA_1);  (*!< DMA mode 2 enabled (2 / 3 half-words by pairs - 2&1 then 1&3 then 3&2) *)
  ADC_DMAACCESSMODE_3        = (ADC_CCR_DMA);  (*!< DMA mode 3 enabled (2 / 3 bytes by pairs - 2&1 then 1&3 then 3&2)  *)
  (**
  * @}
   *)

  (** @defgroup ADCEx_External_trigger_edge_Injected ADC External Trigger Edge Injected
  * @{
   *)

  ADC_EXTERNALTRIGINJECCONVEDGE_NONE    = ($00000000);
  ADC_EXTERNALTRIGINJECCONVEDGE_RISING  = (ADC_CR2_JEXTEN_0);
  ADC_EXTERNALTRIGINJECCONVEDGE_FALLING = (ADC_CR2_JEXTEN_1);
  ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING = (ADC_CR2_JEXTEN);
  (**
  * @}
   *)

  (** @defgroup ADCEx_External_trigger_Source_Injected ADC External Trigger Source Injected
  * @{
   *)

  ADC_EXTERNALTRIGINJECCONV_T1_TRGO  = ($00000000);
  ADC_EXTERNALTRIGINJECCONV_T1_CC4   = (ADC_CR2_JEXTSEL_0);
  ADC_EXTERNALTRIGINJECCONV_T2_TRGO  = (ADC_CR2_JEXTSEL_1);
  ADC_EXTERNALTRIGINJECCONV_T2_CC1   = ((ADC_CR2_JEXTSEL_1 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T3_CC4   = (ADC_CR2_JEXTSEL_2);
  ADC_EXTERNALTRIGINJECCONV_T4_TRGO  = ((ADC_CR2_JEXTSEL_2 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T8_CC4   = ((ADC_CR2_JEXTSEL_2 or ADC_CR2_JEXTSEL_1 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T1_TRGO2 = (ADC_CR2_JEXTSEL_3);
  ADC_EXTERNALTRIGINJECCONV_T8_TRGO  = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T8_TRGO2 = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_1));
  ADC_EXTERNALTRIGINJECCONV_T3_CC3   = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_1 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T5_TRGO  = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_2));
  ADC_EXTERNALTRIGINJECCONV_T3_CC1   = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_2 or ADC_CR2_JEXTSEL_0));
  ADC_EXTERNALTRIGINJECCONV_T6_TRGO  = ((ADC_CR2_JEXTSEL_3 or ADC_CR2_JEXTSEL_2 or ADC_CR2_JEXTSEL_1));
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

procedure HAL_ADCEx_InjectedConvCpltCallback(var hadc: ADC_HandleTypeDef); external name 'HAL_ADCEx_InjectedConvCpltCallback';

implementation

procedure HAL_ADCEx_InjectedConvCpltCallback_stub(var hadc: ADC_HandleTypeDef); assembler; nostackframe; public name 'HAL_ADCEx_InjectedConvCpltCallback';
  asm
    .weak HAL_ADCEx_InjectedConvCpltCallback
  end;

end.


