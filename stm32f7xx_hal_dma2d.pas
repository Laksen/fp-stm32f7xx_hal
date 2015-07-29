(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_dma2d.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of DMA2D HAL module.
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
unit stm32f7xx_hal_dma2d;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal;

const
  MAX_DMA2D_LAYER = 2;

(**
    * @brief DMA2D color Structure definition
     *)

type
  DMA2D_ColorTypeDef = record
    Blue: longword;  (*!< Configures the blue value.
                                      This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.  *)
    Green: longword;  (*!< Configures the green value.
                                      This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.  *)
    Red: longword;  (*!< Configures the red value.
                                      This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.  *)
  end;

  (**
    * @brief DMA2D CLUT Structure definition
     *)

  DMA2D_CLUTCfgTypeDef = record
    pCLUT: Plongword;  (*!< Configures the DMA2D CLUT memory address. *)
    CLUTColorMode: longword;  (*!< configures the DMA2D CLUT color mode.
                                           This parameter can be one value of @ref DMA2D_CLUT_CM  *)
    Size: longword;  (*!< configures the DMA2D CLUT size.
                                           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. *)
  end;

  (**
    * @brief DMA2D Init structure definition
     *)

  DMA2D_InitTypeDef = record
    Mode: longword;  (*!< configures the DMA2D transfer mode.
                                                  This parameter can be one value of @ref DMA2D_Mode  *)
    ColorMode: longword;  (*!< configures the color format of the output image.
                                                  This parameter can be one value of @ref DMA2D_Color_Mode  *)
    OutputOffset: longword;  (*!< Specifies the Offset value.
                                                  This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF.  *)
  end;

  (**
    * @brief DMA2D Layer structure definition
     *)

  PDMA2D_LayerCfgTypeDef = ^DMA2D_LayerCfgTypeDef;

  DMA2D_LayerCfgTypeDef = record
    InputOffset: longword;  (*!< configures the DMA2D foreground offset.
                                                 This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF.  *)
    InputColorMode: longword;  (*!< configures the DMA2D foreground color mode .
                                                 This parameter can be one value of @ref DMA2D_Input_Color_Mode  *)
    AlphaMode: longword;  (*!< configures the DMA2D foreground alpha mode.
                                                 This parameter can be one value of @ref DMA2D_ALPHA_MODE  *)
    InputAlpha: longword;  (*!< Specifies the DMA2D foreground alpha value and color value in case of A8 or A4 color mode.
                                                 This parameter must be a number between Min_Data = 0x00000000 and Max_Data = 0xFFFFFFFF
                                                 in case of A8 or A4 color mode (ARGB).
                                                 Otherwise, This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. *)
  end;

  (**
    * @brief  HAL DMA2D State structures definition
     *)
type
  HAL_DMA2D_StateTypeDef = (
    HAL_DMA2D_STATE_RESET,  (*!< DMA2D not yet initialized or disabled        *)
    HAL_DMA2D_STATE_READY,  (*!< Peripheral Initialized and ready for use     *)
    HAL_DMA2D_STATE_BUSY,   (*!< an internal process is ongoing               *)
    HAL_DMA2D_STATE_TIMEOUT,(*!< Timeout state                                *)
    HAL_DMA2D_STATE_ERROR,  (*!< DMA2D state error                            *)
    HAL_DMA2D_STATE_SUSPEND (*!< DMA2D process is suspended                   *)
  );

  (**
    * @brief  DMA2D handle Structure definition
     *)

type
  PDMA2D_TypeDef = ^DMA2D_TypeDef;

  DMA2D_HandleTypeDef = record
    Instance: PDMA2D_TypeDef;  (*!< DMA2D Register base address        *)
    Init: DMA2D_InitTypeDef;  (*!< DMA2D communication parameters     *)
    XferCpltCallback: procedure(var hdma2d: DMA2D_HandleTypeDef);   (*!< DMA2D transfer complete callback   *)
    XferErrorCallback: procedure(var hdma2d: DMA2D_HandleTypeDef);  (*!< DMA2D transfer error callback      *)
    LayerCfg: array [0..MAX_DMA2D_LAYER - 1] of DMA2D_LayerCfgTypeDef;  (*!< DMA2D Layers parameters            *)
    Lock: HAL_LockTypeDef;  (*!< DMA2D Lock                         *)
    State: HAL_DMA2D_StateTypeDef;  (*!< DMA2D transfer state               *)
    ErrorCode: longword;  (*!< DMA2D Error code                   *)
  end;

  (**
    * @}
     *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup DMA2D_Exported_Constants DMA2D Exported Constants
    * @{
     *)

  (** @defgroup DMA2D_Error_Code DMA2D Error Code
    * @{
     *)

const
  HAL_DMA2D_ERROR_NONE = ($00000000);  (*!< No error              *)
  HAL_DMA2D_ERROR_TE = ($00000001);  (*!< Transfer error        *)
  HAL_DMA2D_ERROR_CE = ($00000002);  (*!< Configuration error   *)
  HAL_DMA2D_ERROR_TIMEOUT = ($00000020);  (*!< Timeout error         *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_Mode DMA2D Mode
    * @{
     *)

  DMA2D_M2M = ($00000000);  (*!< DMA2D memory to memory transfer mode  *)
  DMA2D_M2M_PFC = ($00010000);  (*!< DMA2D memory to memory with pixel format conversion transfer mode  *)
  DMA2D_M2M_BLEND = ($00020000);  (*!< DMA2D memory to memory with blending transfer mode  *)
  DMA2D_R2M = ($00030000);  (*!< DMA2D register to memory transfer mode  *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_Color_Mode DMA2D Color Mode
    * @{
     *)

  DMA2D_ARGB8888 = ($00000000);  (*!< ARGB8888 DMA2D color mode  *)
  DMA2D_RGB888 = ($00000001);  (*!< RGB888 DMA2D color mode    *)
  DMA2D_RGB565 = ($00000002);  (*!< RGB565 DMA2D color mode    *)
  DMA2D_ARGB1555 = ($00000003);  (*!< ARGB1555 DMA2D color mode  *)
  DMA2D_ARGB4444 = ($00000004);  (*!< ARGB4444 DMA2D color mode  *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_COLOR_VALUE DMA2D COLOR VALUE
    * @{
     *)

  COLOR_VALUE = ($000000FF);  (*!< color value mask  *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_SIZE DMA2D SIZE
    * @{
     *)

  DMA2D_PIXEL = (DMA2D_NLR_PL shr 16);  (*!< DMA2D pixel per line  *)
  DMA2D_LINE = DMA2D_NLR_NL;  (*!< DMA2D number of line  *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_Offset DMA2D Offset
    * @{
     *)

  DMA2D_OFFSET = DMA2D_FGOR_LO;  (*!< Line Offset  *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_Input_Color_Mode DMA2D Input Color Mode
    * @{
     *)

  CM_ARGB8888 = ($00000000);  (*!< ARGB8888 color mode  *)
  CM_RGB888 = ($00000001);  (*!< RGB888 color mode  *)
  CM_RGB565 = ($00000002);  (*!< RGB565 color mode  *)
  CM_ARGB1555 = ($00000003);  (*!< ARGB1555 color mode  *)
  CM_ARGB4444 = ($00000004);  (*!< ARGB4444 color mode  *)
  CM_L8 = ($00000005);  (*!< L8 color mode  *)
  CM_AL44 = ($00000006);  (*!< AL44 color mode  *)
  CM_AL88 = ($00000007);  (*!< AL88 color mode  *)
  CM_L4 = ($00000008);  (*!< L4 color mode  *)
  CM_A8 = ($00000009);  (*!< A8 color mode  *)
  CM_A4 = ($0000000A);  (*!< A4 color mode  *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_ALPHA_MODE DMA2D ALPHA MODE
    * @{
     *)

  DMA2D_NO_MODIF_ALPHA = ($00000000);  (*!< No modification of the alpha channel value  *)
  DMA2D_REPLACE_ALPHA = ($00000001);  (*!< Replace original alpha channel value by programmed alpha value  *)
  DMA2D_COMBINE_ALPHA = ($00000002);  (*!< Replace original alpha channel value by programmed alpha value
                                                                  with original alpha channel value                               *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_CLUT_CM DMA2D CLUT CM
    * @{
     *)

  DMA2D_CCM_ARGB8888 = ($00000000);  (*!< ARGB8888 DMA2D C-LUT color mode  *)
  DMA2D_CCM_RGB888 = ($00000001);  (*!< RGB888 DMA2D C-LUT color mode    *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_Size_Clut DMA2D Size Clut
    * @{
     *)

  DMA2D_CLUT_SIZE = (DMA2D_FGPFCCR_CS shr 8);  (*!< DMA2D C-LUT size  *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_DeadTime DMA2D DeadTime
    * @{
     *)

  LINE_WATERMARK = DMA2D_LWR_LW;
  (**
    * @}
     *)

  (** @defgroup DMA2D_Interrupts DMA2D Interrupts
    * @{
     *)

  DMA2D_IT_CE = DMA2D_CR_CEIE;  (*!< Configuration Error Interrupt  *)
  DMA2D_IT_CTC = DMA2D_CR_CTCIE;  (*!< C-LUT Transfer Complete Interrupt  *)
  DMA2D_IT_CAE = DMA2D_CR_CAEIE;  (*!< C-LUT Access Error Interrupt  *)
  DMA2D_IT_TW = DMA2D_CR_TWIE;  (*!< Transfer Watermark Interrupt  *)
  DMA2D_IT_TC = DMA2D_CR_TCIE;  (*!< Transfer Complete Interrupt  *)
  DMA2D_IT_TE = DMA2D_CR_TEIE;  (*!< Transfer Error Interrupt  *)
  (**
    * @}
     *)

  (** @defgroup DMA2D_Flag DMA2D Flag
    * @{
     *)

  DMA2D_FLAG_CE = DMA2D_ISR_CEIF;  (*!< Configuration Error Interrupt Flag  *)
  DMA2D_FLAG_CTC = DMA2D_ISR_CTCIF;  (*!< C-LUT Transfer Complete Interrupt Flag  *)
  DMA2D_FLAG_CAE = DMA2D_ISR_CAEIF;  (*!< C-LUT Access Error Interrupt Flag  *)
  DMA2D_FLAG_TW = DMA2D_ISR_TWIF;  (*!< Transfer Watermark Interrupt Flag  *)
  DMA2D_FLAG_TC = DMA2D_ISR_TCIF;  (*!< Transfer Complete Interrupt Flag  *)
  DMA2D_FLAG_TE = DMA2D_ISR_TEIF;  (*!< Transfer Error Interrupt Flag  *)

procedure __HAL_DMA2D_RESET_HANDLE_STATE(var __HANDLE__: DMA2D_HandleTypeDef);
procedure __HAL_DMA2D_ENABLE(var __HANDLE__: DMA2D_HandleTypeDef);
procedure __HAL_DMA2D_DISABLE(var __HANDLE__: DMA2D_HandleTypeDef);
function __HAL_DMA2D_GET_FLAG(var __HANDLE__: DMA2D_HandleTypeDef; __FLAG__: longword): boolean;
procedure __HAL_DMA2D_CLEAR_FLAG(var __HANDLE__: DMA2D_HandleTypeDef; __FLAG__: longword);
procedure __HAL_DMA2D_ENABLE_IT(var __HANDLE__: DMA2D_HandleTypeDef; __INTERRUPT__: longword);
procedure __HAL_DMA2D_DISABLE_IT(var __HANDLE__: DMA2D_HandleTypeDef; __INTERRUPT__: longword);
function __HAL_DMA2D_GET_IT_SOURCE(var __HANDLE__: DMA2D_HandleTypeDef; __INTERRUPT__: longword): boolean;

(* Initialization and de-initialization functions ****************************** *)
function HAL_DMA2D_Init(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
function HAL_DMA2D_DeInit(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_DMA2D_MspInit(var hdma2d: DMA2D_HandleTypeDef); external name 'HAL_DMA2D_MspInit';
procedure HAL_DMA2D_MspDeInit(var hdma2d: DMA2D_HandleTypeDef); external name 'HAL_DMA2D_MspDeInit';

(* IO operation functions ****************************************************** *)
function HAL_DMA2D_Start(var hdma2d: DMA2D_HandleTypeDef; pdata, DstAddress, Width, Height: longword): HAL_StatusTypeDef;
function HAL_DMA2D_BlendingStart(var hdma2d: DMA2D_HandleTypeDef; SrcAddress1, SrcAddress2, DstAddress, Width, Height: longword): HAL_StatusTypeDef;
function HAL_DMA2D_Start_IT(var hdma2d: DMA2D_HandleTypeDef; pdata, DstAddress, Width, Height: longword): HAL_StatusTypeDef;
function HAL_DMA2D_BlendingStart_IT(var hdma2d: DMA2D_HandleTypeDef; SrcAddress1, SrcAddress2, DstAddress, Width, Height: longword): HAL_StatusTypeDef;
function HAL_DMA2D_Suspend(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
function HAL_DMA2D_Resume(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
function HAL_DMA2D_Abort(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
function HAL_DMA2D_PollForTransfer(var hdma2d: DMA2D_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
procedure HAL_DMA2D_IRQHandler(var hdma2d: DMA2D_HandleTypeDef);

(* Peripheral Control functions ************************************************ *)
function HAL_DMA2D_ConfigLayer(var hdma2d: DMA2D_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_DMA2D_ConfigCLUT(var hdma2d: DMA2D_HandleTypeDef; CLUTCfg: DMA2D_CLUTCfgTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_DMA2D_EnableCLUT(var hdma2d: DMA2D_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_DMA2D_DisableCLUT(var hdma2d: DMA2D_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_DMA2D_ProgramLineEvent(var hdma2d: DMA2D_HandleTypeDef; Line: longword): HAL_StatusTypeDef;

(* Peripheral State functions ************************************************** *)
function HAL_DMA2D_GetState(var hdma2d: DMA2D_HandleTypeDef): HAL_DMA2D_StateTypeDef;
function HAL_DMA2D_GetError(var hdma2d: DMA2D_HandleTypeDef): longword;

implementation

const
  HAL_TIMEOUT_DMA2D_ABORT = (1000);  (* 1s   *)
  HAL_TIMEOUT_DMA2D_SUSPEND = (1000);  (* 1s   *)

procedure DMA2D_SetConfig(var hdma2d: DMA2D_HandleTypeDef; pdata, DstAddress, Width, Height: longword);
var
  tmp,
  tmp1,tmp2,tmp3,tmp4: longword;
begin
  tmp := Width shl 16;

  (* Configure DMA2D data size *)
  hdma2d.Instance^.NLR := (Height or tmp);

  (* Configure DMA2D destination address *)
  hdma2d.Instance^.OMAR := DstAddress;

  (* Register to memory DMA2D mode selected *)
  if (hdma2d.Init.Mode = DMA2D_R2M) then
  begin
    tmp1 := pdata and DMA2D_OCOLR_ALPHA_1;
    tmp2 := pdata and DMA2D_OCOLR_RED_1;
    tmp3 := pdata and DMA2D_OCOLR_GREEN_1;
    tmp4 := pdata and DMA2D_OCOLR_BLUE_1;

    (* Prepare the value to be wrote to the OCOLR register according to the color mode *)
    if (hdma2d.Init.ColorMode = DMA2D_ARGB8888) then
    begin
      tmp := (tmp3 or tmp2 or tmp1 or tmp4);
    end
    else if (hdma2d.Init.ColorMode = DMA2D_RGB888) then
    begin
      tmp := (tmp3 or tmp2 or tmp4);
    end
    else if (hdma2d.Init.ColorMode = DMA2D_RGB565) then
    begin
      tmp2 := (tmp2 shr 19);
      tmp3 := (tmp3 shr 10);
      tmp4 := (tmp4 shr 3);
      tmp := ((tmp3 shl 5) or (tmp2 shl 11) or tmp4);
    end
    else if (hdma2d.Init.ColorMode = DMA2D_ARGB1555) then
    begin
      tmp1 := (tmp1 shr 31);
      tmp2 := (tmp2 shr 19);
      tmp3 := (tmp3 shr 11);
      tmp4 := (tmp4 shr 3);
      tmp := ((tmp3 shl 5) or (tmp2 shl 10) or (tmp1 shl 15) or tmp4);
    end
    else (* DMA2D_CMode := DMA2D_ARGB4444 *)
    begin
      tmp1 := (tmp1 shr 28);
      tmp2 := (tmp2 shr 20);
      tmp3 := (tmp3 shr 12);
      tmp4 := (tmp4 shr 4);
      tmp := ((tmp3 shl 4) or (tmp2 shl 8) or (tmp1 shl 12) or tmp4);
    end;
    (* Write to DMA2D OCOLR register *)
    hdma2d.Instance^.OCOLR := tmp;
  end
  else (* M2M, M2M_PFC or M2M_Blending DMA2D Mode *)
  begin
    (* Configure DMA2D source address *)
    hdma2d.Instance^.FGMAR := pdata;
  end;

end;

(** @brief Reset DMA2D handle state
  * @param  __HANDLE__: specifies the DMA2D handle.
  * @retval None
  *)
procedure __HAL_DMA2D_RESET_HANDLE_STATE(var __HANDLE__: DMA2D_HandleTypeDef);
begin
  __HANDLE__.State := HAL_DMA2D_STATE_RESET;
end;

(**
  * @brief  Enable the DMA2D.
  * @param  __HANDLE__: DMA2D handle
  * @retval None.
  *)
procedure __HAL_DMA2D_ENABLE(var __HANDLE__: DMA2D_HandleTypeDef);
begin
  __HANDLE__.Instance^.CR := __HANDLE__.Instance^.CR or DMA2D_CR_START;
end;

(**
  * @brief  Disable the DMA2D.
  * @param  __HANDLE__: DMA2D handle
  * @retval None.
  *)
procedure __HAL_DMA2D_DISABLE(var __HANDLE__: DMA2D_HandleTypeDef);
begin
  __HANDLE__.Instance^.CR := __HANDLE__.Instance^.CR and (not DMA2D_CR_START);
end;

(* Interrupt  and  Flag management *)
(**
  * @brief  Get the DMA2D pending flags.
  * @param  __HANDLE__: DMA2D handle
  * @param  __FLAG__: Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DMA2D_FLAG_CE:  Configuration error flag
  *            @arg DMA2D_FLAG_CTC: C-LUT transfer complete flag
  *            @arg DMA2D_FLAG_CAE: C-LUT access error flag
  *            @arg DMA2D_FLAG_TW:  Transfer Watermark flag
  *            @arg DMA2D_FLAG_TC:  Transfer complete flag
  *            @arg DMA2D_FLAG_TE:  Transfer error flag
  * @retval The state of FLAG.
  *)
function __HAL_DMA2D_GET_FLAG(var __HANDLE__: DMA2D_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit((__HANDLE__.Instance^.ISR and (__FLAG__)) <> 0);
end;

(**
  * @brief  Clears the DMA2D pending flags.
  * @param  __HANDLE__: DMA2D handle
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DMA2D_FLAG_CE:  Configuration error flag
  *            @arg DMA2D_FLAG_CTC: C-LUT transfer complete flag
  *            @arg DMA2D_FLAG_CAE: C-LUT access error flag
  *            @arg DMA2D_FLAG_TW:  Transfer Watermark flag
  *            @arg DMA2D_FLAG_TC:  Transfer complete flag
  *            @arg DMA2D_FLAG_TE:  Transfer error flag
  * @retval None
  *)
procedure __HAL_DMA2D_CLEAR_FLAG(var __HANDLE__: DMA2D_HandleTypeDef; __FLAG__: longword);
begin
  __HANDLE__.Instance^.IFCR := (__FLAG__);
end;

(**
  * @brief  Enables the specified DMA2D interrupts.
  * @param  __HANDLE__: DMA2D handle
  * @param __INTERRUPT__: specifies the DMA2D interrupt sources to be enabled.
  *          This parameter can be any combination of the following values:
  *            @arg DMA2D_IT_CE:  Configuration error interrupt mask
  *            @arg DMA2D_IT_CTC: C-LUT transfer complete interrupt mask
  *            @arg DMA2D_IT_CAE: C-LUT access error interrupt mask
  *            @arg DMA2D_IT_TW:  Transfer Watermark interrupt mask
  *            @arg DMA2D_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA2D_IT_TE:  Transfer error interrupt mask
  * @retval None
  *)
procedure __HAL_DMA2D_ENABLE_IT(var __HANDLE__: DMA2D_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.CR := __HANDLE__.Instance^.CR or (__INTERRUPT__);
end;

(**
  * @brief  Disables the specified DMA2D interrupts.
  * @param  __HANDLE__: DMA2D handle
  * @param __INTERRUPT__: specifies the DMA2D interrupt sources to be disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DMA2D_IT_CE:  Configuration error interrupt mask
  *            @arg DMA2D_IT_CTC: C-LUT transfer complete interrupt mask
  *            @arg DMA2D_IT_CAE: C-LUT access error interrupt mask
  *            @arg DMA2D_IT_TW:  Transfer Watermark interrupt mask
  *            @arg DMA2D_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA2D_IT_TE:  Transfer error interrupt mask
  * @retval None
  *)
procedure __HAL_DMA2D_DISABLE_IT(var __HANDLE__: DMA2D_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.CR := __HANDLE__.Instance^.CR and (not (__INTERRUPT__));
end;

(**
  * @brief  Checks whether the specified DMA2D interrupt has occurred or not.
  * @param  __HANDLE__: DMA2D handle
  * @param  __INTERRUPT__: specifies the DMA2D interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg DMA2D_IT_CE:  Configuration error interrupt mask
  *            @arg DMA2D_IT_CTC: C-LUT transfer complete interrupt mask
  *            @arg DMA2D_IT_CAE: C-LUT access error interrupt mask
  *            @arg DMA2D_IT_TW:  Transfer Watermark interrupt mask
  *            @arg DMA2D_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA2D_IT_TE:  Transfer error interrupt mask
  * @retval The state of INTERRUPT.
  *)
function __HAL_DMA2D_GET_IT_SOURCE(var __HANDLE__: DMA2D_HandleTypeDef; __INTERRUPT__: longword): boolean;
begin
  exit((__HANDLE__.Instance^.CR and (__INTERRUPT__)) <> 0);
end;

function HAL_DMA2D_Init(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
var
  tmp: longword;
begin
  if (hdma2d.State = HAL_DMA2D_STATE_RESET) then
  begin
    (* Allocate lock resource and initialize it *)
    hdma2d.Lock := HAL_UNLOCKED;
    (* Init the low level hardware *)
    HAL_DMA2D_MspInit(hdma2d);
  end;

  (* Change DMA2D peripheral state *)
  hdma2d.State := HAL_DMA2D_STATE_BUSY;

  (* DMA2D CR register configuration -------------------------------------------*)
  (* Get the CR register value *)
  tmp := hdma2d.Instance^.CR;

  (* Clear Mode bits *)
  tmp := tmp and (not DMA2D_CR_MODE);

  (* Prepare the value to be wrote to the CR register *)
  tmp := tmp or hdma2d.Init.Mode;

  (* Write to DMA2D CR register *)
  hdma2d.Instance^.CR := tmp;

  (* DMA2D OPFCCR register configuration ---------------------------------------*)
  (* Get the OPFCCR register value *)
  tmp := hdma2d.Instance^.OPFCCR;

  (* Clear Color Mode bits *)
  tmp := tmp and (not DMA2D_OPFCCR_CM);

  (* Prepare the value to be wrote to the OPFCCR register *)
  tmp := tmp or hdma2d.Init.ColorMode;

  (* Write to DMA2D OPFCCR register *)
  hdma2d.Instance^.OPFCCR := tmp;

  (* DMA2D OOR register configuration ------------------------------------------*)
  (* Get the OOR register value *)
  tmp := hdma2d.Instance^.OOR;

  (* Clear Offset bits *)
  tmp := tmp and (not DMA2D_OOR_LO);

  (* Prepare the value to be wrote to the OOR register *)
  tmp := tmp or hdma2d.Init.OutputOffset;

  (* Write to DMA2D OOR register *)
  hdma2d.Instance^.OOR := tmp;

  (* Update error code *)
  hdma2d.ErrorCode := HAL_DMA2D_ERROR_NONE;

  (* Initialize the DMA2D state*)
  hdma2d.State := HAL_DMA2D_STATE_READY;

  exit(HAL_OK);
end;

function HAL_DMA2D_DeInit(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* DeInit the low level hardware *)
  HAL_DMA2D_MspDeInit(hdma2d);

  (* Update error code *)
  hdma2d.ErrorCode := HAL_DMA2D_ERROR_NONE;

  (* Initialize the DMA2D state*)
  hdma2d.State := HAL_DMA2D_STATE_RESET;

  (* Release Lock *)
  __HAL_Unlock(hdma2d.lock);

  exit(HAL_OK);
end;

procedure HAL_DMA2D_MspInit_stub(var hdma2d: DMA2D_HandleTypeDef); assembler; public name 'HAL_DMA2D_MspInit';
asm
  .weak HAL_DMA2D_MspInit
end;

procedure HAL_DMA2D_MspDeInit_stub(var hdma2d: DMA2D_HandleTypeDef); assembler; public name 'HAL_DMA2D_MspDeInit';
asm
  .weak HAL_DMA2D_MspDeInit
end;

function HAL_DMA2D_Start(var hdma2d: DMA2D_HandleTypeDef; pdata, DstAddress, Width, Height: longword): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hdma2d.lock);

  (* Change DMA2D peripheral state *)
  hdma2d.State := HAL_DMA2D_STATE_BUSY;

  (* Disable the Peripheral *)
  __HAL_DMA2D_DISABLE(hdma2d);

  (* Configure the source, destination address and the data size *)
  DMA2D_SetConfig(hdma2d, pdata, DstAddress, Width, Height);

  (* Enable the Peripheral *)
  __HAL_DMA2D_ENABLE(hdma2d);

  exit(HAL_OK);
end;

function HAL_DMA2D_BlendingStart(var hdma2d: DMA2D_HandleTypeDef; SrcAddress1, SrcAddress2, DstAddress, Width, Height: longword): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hdma2d.lock);

  (* Change DMA2D peripheral state *)
  hdma2d.State := HAL_DMA2D_STATE_BUSY;

  (* Disable the Peripheral *)
  __HAL_DMA2D_DISABLE(hdma2d);

  (* Configure DMA2D Stream source2 address *)
  hdma2d.Instance^.BGMAR := SrcAddress2;

  (* Configure the source, destination address and the data size *)
  DMA2D_SetConfig(hdma2d, SrcAddress1, DstAddress, Width, Height);

  (* Enable the Peripheral *)
  __HAL_DMA2D_ENABLE(hdma2d);

  exit(HAL_OK);
end;

function HAL_DMA2D_Start_IT(var hdma2d: DMA2D_HandleTypeDef; pdata, DstAddress, Width, Height: longword): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hdma2d.lock);

  (* Change DMA2D peripheral state *)
  hdma2d.State := HAL_DMA2D_STATE_BUSY;

  (* Disable the Peripheral *)
  __HAL_DMA2D_DISABLE(hdma2d);

  (* Configure the source, destination address and the data size *)
  DMA2D_SetConfig(hdma2d, pdata, DstAddress, Width, Height);

  (* Enable the transfer complete interrupt *)
  __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_TC);

  (* Enable the transfer Error interrupt *)
  __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_TE);

  (* Enable the Peripheral *)
  __HAL_DMA2D_ENABLE(hdma2d);

  (* Enable the configuration error interrupt *)
  __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_CE);

  exit(HAL_OK);
end;

function HAL_DMA2D_BlendingStart_IT(var hdma2d: DMA2D_HandleTypeDef; SrcAddress1, SrcAddress2, DstAddress, Width, Height: longword): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hdma2d.lock);

  (* Change DMA2D peripheral state *)
  hdma2d.State := HAL_DMA2D_STATE_BUSY;

  (* Disable the Peripheral *)
  __HAL_DMA2D_DISABLE(hdma2d);

  (* Configure DMA2D Stream source2 address *)
  hdma2d.Instance^.BGMAR := SrcAddress2;

  (* Configure the source, destination address and the data size *)
  DMA2D_SetConfig(hdma2d, SrcAddress1, DstAddress, Width, Height);

  (* Enable the configuration error interrupt *)
  __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_CE);

  (* Enable the transfer complete interrupt *)
  __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_TC);

  (* Enable the transfer Error interrupt *)
  __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_TE);

  (* Enable the Peripheral *)
  __HAL_DMA2D_ENABLE(hdma2d);

  exit(HAL_OK);
end;

function HAL_DMA2D_Suspend(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  (* Suspend the DMA2D transfer *)
  hdma2d.Instance^.CR := hdma2d.Instance^.CR or DMA2D_CR_SUSP;

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Check if the DMA2D is effectively suspended *)
  while ((hdma2d.Instance^.CR and DMA2D_CR_SUSP) <> DMA2D_CR_SUSP) do
  begin
    if ((HAL_GetTick() - tickstart) > HAL_TIMEOUT_DMA2D_SUSPEND) then
    begin
      (* Update error code *)
      hdma2d.ErrorCode := hdma2d.ErrorCode or HAL_DMA2D_ERROR_TIMEOUT;

      (* Change the DMA2D state *)
      hdma2d.State := HAL_DMA2D_STATE_TIMEOUT;

      exit(HAL_TIMEOUT);
    end;
  end;
  (* Change the DMA2D state*)
  hdma2d.State := HAL_DMA2D_STATE_SUSPEND;

  exit(HAL_OK);
end;

function HAL_DMA2D_Resume(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Resume the DMA2D transfer *)
  hdma2d.Instance^.CR := hdma2d.Instance^.CR and (not DMA2D_CR_SUSP);

  (* Change the DMA2D state*)
  hdma2d.State := HAL_DMA2D_STATE_BUSY;

  exit(HAL_OK);
end;

function HAL_DMA2D_Abort(var hdma2d: DMA2D_HandleTypeDef): HAL_StatusTypeDef;
var
  tickstart: longword;
begin
  (* Disable the DMA2D *)
  __HAL_DMA2D_DISABLE(hdma2d);

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Check if the DMA2D is effectively disabled *)
  while ((hdma2d.Instance^.CR and DMA2D_CR_START) <> 0) do
  begin
    if ((HAL_GetTick() - tickstart) > HAL_TIMEOUT_DMA2D_ABORT) then
    begin
      (* Update error code *)
      hdma2d.ErrorCode := hdma2d.ErrorCode or HAL_DMA2D_ERROR_TIMEOUT;

      (* Change the DMA2D state *)
      hdma2d.State := HAL_DMA2D_STATE_TIMEOUT;

      (* Process Unlocked *)
      __HAL_Unlock(hdma2d.lock);

      exit(HAL_TIMEOUT);
    end;
  end;
  (* Process Unlocked *)
  __HAL_Unlock(hdma2d.lock);

  (* Change the DMA2D state*)
  hdma2d.State := HAL_DMA2D_STATE_READY;

  exit(HAL_OK);
end;

function HAL_DMA2D_PollForTransfer(var hdma2d: DMA2D_HandleTypeDef; Timeout: longword): HAL_StatusTypeDef;
var
  tickstart: longword;
  tmp, tmp1: boolean;
begin
  (* Polling for DMA2D transfer *)
  if ((hdma2d.Instance^.CR and DMA2D_CR_START) <> 0) then
  begin
    (* Get tick *)
    tickstart := HAL_GetTick();

    while not __HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_TC) do
    begin
      tmp := __HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_CE);
      tmp1 := __HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_TE);

      if ((tmp) or (tmp1)) then
      begin
        (* Clear the transfer and configuration error flags *)
        __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CE);
        __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TE);

        (* Change DMA2D state *)
        hdma2d.State := HAL_DMA2D_STATE_ERROR;

        (* Process unlocked *)
        __HAL_Unlock(hdma2d.lock);

        exit(HAL_ERROR);
      end;
      (* Check for the Timeout *)
      if (Timeout <> HAL_MAX_DELAY) then
      begin
        if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
        begin
          (* Process unlocked *)
          __HAL_Unlock(hdma2d.lock);

          (* Update error code *)
          hdma2d.ErrorCode := hdma2d.ErrorCode or HAL_DMA2D_ERROR_TIMEOUT;

          (* Change the DMA2D state *)
          hdma2d.State := HAL_DMA2D_STATE_TIMEOUT;

          exit(HAL_TIMEOUT);
        end;
      end;
    end;
  end;
  (* Polling for CLUT loading *)
  if ((hdma2d.Instance^.FGPFCCR and DMA2D_FGPFCCR_START) <> 0) then
  begin
    (* Get tick *)
    tickstart := HAL_GetTick();

    while not (__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_CTC)) do
    begin
      if (__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_CAE)) then
      begin
        (* Clear the transfer and configuration error flags *)
        __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CAE);

        (* Change DMA2D state *)
        hdma2d.State := HAL_DMA2D_STATE_ERROR;

        exit(HAL_ERROR);
      end;
      (* Check for the Timeout *)
      if (Timeout <> HAL_MAX_DELAY) then
      begin
        if ((Timeout = 0) or ((HAL_GetTick() - tickstart) > Timeout)) then
        begin
          (* Update error code *)
          hdma2d.ErrorCode := hdma2d.ErrorCode or HAL_DMA2D_ERROR_TIMEOUT;

          (* Change the DMA2D state *)
          hdma2d.State := HAL_DMA2D_STATE_TIMEOUT;

          exit(HAL_TIMEOUT);
        end;
      end;
    end;
  end;
  (* Clear the transfer complete flag *)
  __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TC);

  (* Clear the CLUT loading flag *)
  __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CTC);

  (* Change DMA2D state *)
  hdma2d.State := HAL_DMA2D_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hdma2d.lock);

  exit(HAL_OK);
end;

procedure HAL_DMA2D_IRQHandler(var hdma2d: DMA2D_HandleTypeDef);
begin
  (* Transfer Error Interrupt management ***************************************)
  if (__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_TE)) then
  begin
    if (__HAL_DMA2D_GET_IT_SOURCE(hdma2d, DMA2D_IT_TE)) then
    begin
      (* Disable the transfer Error interrupt *)
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_TE);

      (* Update error code *)
      hdma2d.ErrorCode := hdma2d.ErrorCode or HAL_DMA2D_ERROR_TE;

      (* Clear the transfer error flag *)
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TE);

      (* Change DMA2D state *)
      hdma2d.State := HAL_DMA2D_STATE_ERROR;

      (* Process Unlocked *)
      __HAL_Unlock(hdma2d.lock);

      if (hdma2d.XferErrorCallback <> nil) then
      begin
        (* Transfer error Callback *)
        hdma2d.XferErrorCallback(hdma2d);
      end;
    end;
  end;
  (* Configuration Error Interrupt management **********************************)
  if (__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_CE)) then
  begin
    if (__HAL_DMA2D_GET_IT_SOURCE(hdma2d, DMA2D_IT_CE)) then
    begin
      (* Disable the Configuration Error interrupt *)
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_CE);

      (* Clear the Configuration error flag *)
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CE);

      (* Update error code *)
      hdma2d.ErrorCode := hdma2d.ErrorCode or HAL_DMA2D_ERROR_CE;

      (* Change DMA2D state *)
      hdma2d.State := HAL_DMA2D_STATE_ERROR;

      (* Process Unlocked *)
      __HAL_Unlock(hdma2d.lock);

      if (hdma2d.XferErrorCallback <> nil) then
      begin
        (* Transfer error Callback *)
        hdma2d.XferErrorCallback(hdma2d);
      end;
    end;
  end;
  (* Transfer Complete Interrupt management ************************************)
  if (__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_TC)) then
  begin
    if (__HAL_DMA2D_GET_IT_SOURCE(hdma2d, DMA2D_IT_TC)) then
    begin
      (* Disable the transfer complete interrupt *)
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_TC);

      (* Clear the transfer complete flag *)
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TC);

      (* Update error code *)
      hdma2d.ErrorCode := hdma2d.ErrorCode or HAL_DMA2D_ERROR_NONE;

      (* Change DMA2D state *)
      hdma2d.State := HAL_DMA2D_STATE_READY;

      (* Process Unlocked *)
      __HAL_Unlock(hdma2d.lock);

      if (hdma2d.XferCpltCallback <> nil) then
      begin
        (* Transfer complete Callback *)
        hdma2d.XferCpltCallback(hdma2d);
      end;
    end;
  end;
end;

function HAL_DMA2D_ConfigLayer(var hdma2d: DMA2D_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
var
  pLayerCfg: PDMA2D_LayerCfgTypeDef;
  tmp: longword;
begin
  pLayerCfg := @hdma2d.LayerCfg[LayerIdx];

  tmp := 0;

  (* Process locked *)
  __HAL_Lock(hdma2d.lock);

  (* Change DMA2D peripheral state *)
  hdma2d.State := HAL_DMA2D_STATE_BUSY;

  (* Configure the background DMA2D layer *)
  if (LayerIdx = 0) then
  begin
    (* DMA2D BGPFCR register configuration -----------------------------------*)
    (* Get the BGPFCCR register value *)
    tmp := hdma2d.Instance^.BGPFCCR;

    (* Clear Input color mode, alpha value and alpha mode bits *)
    tmp := tmp and (not (DMA2D_BGPFCCR_CM or DMA2D_BGPFCCR_AM or DMA2D_BGPFCCR_ALPHA));

    if ((pLayerCfg^.InputColorMode = CM_A4) or (pLayerCfg^.InputColorMode = CM_A8)) then
    begin
      (* Prepare the value to be wrote to the BGPFCCR register *)
      tmp := tmp or (pLayerCfg^.InputColorMode or (pLayerCfg^.AlphaMode shl 16) or ((pLayerCfg^.InputAlpha) and $FF000000));
    end
    else
    begin
      (* Prepare the value to be wrote to the BGPFCCR register *)
      tmp := tmp or (pLayerCfg^.InputColorMode or (pLayerCfg^.AlphaMode shl 16) or (pLayerCfg^.InputAlpha shl 24));
    end;

    (* Write to DMA2D BGPFCCR register *)
    hdma2d.Instance^.BGPFCCR := tmp;

    (* DMA2D BGOR register configuration -------------------------------------*)
    (* Get the BGOR register value *)
    tmp := hdma2d.Instance^.BGOR;

    (* Clear colors bits *)
    tmp := tmp and (not DMA2D_BGOR_LO);

    (* Prepare the value to be wrote to the BGOR register *)
    tmp := tmp or pLayerCfg^.InputOffset;

    (* Write to DMA2D BGOR register *)
    hdma2d.Instance^.BGOR := tmp;

    if ((pLayerCfg^.InputColorMode = CM_A4) or (pLayerCfg^.InputColorMode = CM_A8)) then
    begin
      (* Prepare the value to be wrote to the BGCOLR register *)
      tmp := ((pLayerCfg^.InputAlpha) and $00FFFFFF);

      (* Write to DMA2D BGCOLR register *)
      hdma2d.Instance^.BGCOLR := tmp;
    end;
  end
  (* Configure the foreground DMA2D layer *)
  else
  begin
    (* DMA2D FGPFCR register configuration -----------------------------------*)
    (* Get the FGPFCCR register value *)
    tmp := hdma2d.Instance^.FGPFCCR;

    (* Clear Input color mode, alpha value and alpha mode bits *)
    tmp := tmp and (not (DMA2D_FGPFCCR_CM or DMA2D_FGPFCCR_AM or DMA2D_FGPFCCR_ALPHA));

    if ((pLayerCfg^.InputColorMode = CM_A4) or (pLayerCfg^.InputColorMode = CM_A8)) then
    begin
      (* Prepare the value to be wrote to the FGPFCCR register *)
      tmp := tmp or (pLayerCfg^.InputColorMode or (pLayerCfg^.AlphaMode shl 16) or ((pLayerCfg^.InputAlpha) and $FF000000));
    end
    else
    begin
      (* Prepare the value to be wrote to the FGPFCCR register *)
      tmp := tmp or (pLayerCfg^.InputColorMode or (pLayerCfg^.AlphaMode shl 16) or (pLayerCfg^.InputAlpha shl 24));
    end;

    (* Write to DMA2D FGPFCCR register *)
    hdma2d.Instance^.FGPFCCR := tmp;

    (* DMA2D FGOR register configuration -------------------------------------*)
    (* Get the FGOR register value *)
    tmp := hdma2d.Instance^.FGOR;

    (* Clear colors bits *)
    tmp := tmp and (not DMA2D_FGOR_LO);

    (* Prepare the value to be wrote to the FGOR register *)
    tmp := tmp or pLayerCfg^.InputOffset;

    (* Write to DMA2D FGOR register *)
    hdma2d.Instance^.FGOR := tmp;

    if ((pLayerCfg^.InputColorMode = CM_A4) or (pLayerCfg^.InputColorMode = CM_A8)) then
    begin
      (* Prepare the value to be wrote to the FGCOLR register *)
      tmp := ((pLayerCfg^.InputAlpha) and $00FFFFFF);

      (* Write to DMA2D FGCOLR register *)
      hdma2d.Instance^.FGCOLR := tmp;
    end;
  end;
  (* Initialize the DMA2D state*)
  hdma2d.State := HAL_DMA2D_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hdma2d.lock);

  exit(HAL_OK);
end;

function HAL_DMA2D_ConfigCLUT(var hdma2d: DMA2D_HandleTypeDef; CLUTCfg: DMA2D_CLUTCfgTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
var
  tmp, tmp1: longword;
begin
  (* Configure the CLUT of the background DMA2D layer *)
  if (LayerIdx = 0) then
  begin
    (* Get the BGCMAR register value *)
    tmp := hdma2d.Instance^.BGCMAR;

    (* Clear CLUT address bits *)
    tmp := tmp and (not DMA2D_BGCMAR_MA);

    (* Prepare the value to be wrote to the BGCMAR register *)
    tmp := tmp or longword(CLUTCfg.pCLUT);

    (* Write to DMA2D BGCMAR register *)
    hdma2d.Instance^.BGCMAR := tmp;

    (* Get the BGPFCCR register value *)
    tmp := hdma2d.Instance^.BGPFCCR;

    (* Clear CLUT size and CLUT address bits *)
    tmp := tmp and (not (DMA2D_BGPFCCR_CS or DMA2D_BGPFCCR_CCM));

    (* Get the CLUT size *)
    tmp1 := CLUTCfg.Size shl 16;

    (* Prepare the value to be wrote to the BGPFCCR register *)
    tmp := tmp or (CLUTCfg.CLUTColorMode or tmp1);

    (* Write to DMA2D BGPFCCR register *)
    hdma2d.Instance^.BGPFCCR := tmp;
  end
  (* Configure the CLUT of the foreground DMA2D layer *)
  else
  begin
    (* Get the FGCMAR register value *)
    tmp := hdma2d.Instance^.FGCMAR;

    (* Clear CLUT address bits *)
    tmp := tmp and (not DMA2D_FGCMAR_MA);

    (* Prepare the value to be wrote to the FGCMAR register *)
    tmp := tmp or longword(CLUTCfg.pCLUT);

    (* Write to DMA2D FGCMAR register *)
    hdma2d.Instance^.FGCMAR := tmp;

    (* Get the FGPFCCR register value *)
    tmp := hdma2d.Instance^.FGPFCCR;

    (* Clear CLUT size and CLUT address bits *)
    tmp := tmp and (not (DMA2D_FGPFCCR_CS or DMA2D_FGPFCCR_CCM));

    (* Get the CLUT size *)
    tmp1 := CLUTCfg.Size shl 8;

    (* Prepare the value to be wrote to the FGPFCCR register *)
    tmp := tmp or (CLUTCfg.CLUTColorMode or tmp1);

    (* Write to DMA2D FGPFCCR register *)
    hdma2d.Instance^.FGPFCCR := tmp;
  end;

  exit(HAL_OK);
end;

function HAL_DMA2D_EnableCLUT(var hdma2d: DMA2D_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
begin
  if (LayerIdx = 0) then
  begin
    (* Enable the CLUT loading for the background *)
    hdma2d.Instance^.BGPFCCR := hdma2d.Instance^.BGPFCCR or DMA2D_BGPFCCR_START;
  end
  else
  begin
    (* Enable the CLUT loading for the foreground *)
    hdma2d.Instance^.FGPFCCR := hdma2d.Instance^.FGPFCCR or DMA2D_FGPFCCR_START;
  end;

  exit(HAL_OK);
end;

function HAL_DMA2D_DisableCLUT(var hdma2d: DMA2D_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
begin
  if (LayerIdx = 0) then
  begin
    (* Disable the CLUT loading for the background *)
    hdma2d.Instance^.BGPFCCR := hdma2d.Instance^.BGPFCCR and (not DMA2D_BGPFCCR_START);
  end
  else
  begin
    (* Disable the CLUT loading for the foreground *)
    hdma2d.Instance^.FGPFCCR := hdma2d.Instance^.FGPFCCR and (not DMA2D_FGPFCCR_START);
  end;

  exit(HAL_OK);
end;

function HAL_DMA2D_ProgramLineEvent(var hdma2d: DMA2D_HandleTypeDef; Line: longword): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hdma2d.lock);

  (* Change DMA2D peripheral state *)
  hdma2d.State := HAL_DMA2D_STATE_BUSY;

  (* Sets the Line watermark configuration *)
  DMA2D.LWR := Line;

  (* Initialize the DMA2D state*)
  hdma2d.State := HAL_DMA2D_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hdma2d.lock);

  exit(HAL_OK);
end;

function HAL_DMA2D_GetState(var hdma2d: DMA2D_HandleTypeDef): HAL_DMA2D_StateTypeDef;
begin
  exit(hdma2d.State);
end;

function HAL_DMA2D_GetError(var hdma2d: DMA2D_HandleTypeDef): longword;
begin
  exit(hdma2d.ErrorCode);
end;

end.
