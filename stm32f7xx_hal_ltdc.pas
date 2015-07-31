(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_ltdc.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of LTDC HAL module.
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

unit stm32f7xx_hal_ltdc;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal;

const
  MAX_LAYER = 2;
  (**
  * @brief  LTDC color structure definition
   *)

type
  LTDC_ColorTypeDef = record
    Blue: byte;  (*!< Configures the blue value.
                                        This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.  *)
    Green: byte;  (*!< Configures the green value.
                                        This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.  *)
    Red: byte;  (*!< Configures the red value.
                                        This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.  *)
    Reserved: byte;  (*!< Reserved 0xFF  *)
  end;

  (**
  * @brief  LTDC Init structure definition
   *)

  LTDC_InitTypeDef = record
    HSPolarity: longword;  (*!< configures the horizontal synchronization polarity.
                                                      This parameter can be one value of @ref LTDC_HS_POLARITY  *)
    VSPolarity: longword;  (*!< configures the vertical synchronization polarity.
                                                      This parameter can be one value of @ref LTDC_VS_POLARITY  *)
    DEPolarity: longword;  (*!< configures the data enable polarity.
                                                      This parameter can be one of value of @ref LTDC_DE_POLARITY  *)
    PCPolarity: longword;  (*!< configures the pixel clock polarity.
                                                      This parameter can be one of value of @ref LTDC_PC_POLARITY  *)
    HorizontalSync: longword;  (*!< configures the number of Horizontal synchronization width.
                                                      This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF.  *)
    VerticalSync: longword;  (*!< configures the number of Vertical synchronization height.
                                                      This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF.  *)
    AccumulatedHBP: longword;  (*!< configures the accumulated horizontal back porch width.
                                                      This parameter must be a number between Min_Data = LTDC_HorizontalSync and Max_Data = 0xFFF.  *)
    AccumulatedVBP: longword;  (*!< configures the accumulated vertical back porch height.
                                                      This parameter must be a number between Min_Data = LTDC_VerticalSync and Max_Data = 0x7FF.  *)
    AccumulatedActiveW: longword;  (*!< configures the accumulated active width.
                                                      This parameter must be a number between Min_Data = LTDC_AccumulatedHBP and Max_Data = 0xFFF.  *)
    AccumulatedActiveH: longword;  (*!< configures the accumulated active height.
                                                      This parameter must be a number between Min_Data = LTDC_AccumulatedVBP and Max_Data = 0x7FF.  *)
    TotalWidth: longword;  (*!< configures the total width.
                                                      This parameter must be a number between Min_Data = LTDC_AccumulatedActiveW and Max_Data = 0xFFF.  *)
    TotalHeigh: longword;  (*!< configures the total height.
                                                      This parameter must be a number between Min_Data = LTDC_AccumulatedActiveH and Max_Data = 0x7FF.  *)
    Backcolor: LTDC_ColorTypeDef;  (*!< Configures the background color.  *)
  end;

  (**
  * @brief  LTDC Layer structure definition
   *)

  PLTDC_LayerCfgTypeDef = ^LTDC_LayerCfgTypeDef;
  LTDC_LayerCfgTypeDef = record
    WindowX0: longword;  (*!< Configures the Window Horizontal Start Position.
                                            This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF.  *)
    WindowX1: longword;  (*!< Configures the Window Horizontal Stop Position.
                                            This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF.  *)
    WindowY0: longword;  (*!< Configures the Window vertical Start Position.
                                            This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF.  *)
    WindowY1: longword;  (*!< Configures the Window vertical Stop Position.
                                            This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x7FF.  *)
    PixelFormat: longword;  (*!< Specifies the pixel format.
                                            This parameter can be one of value of @ref LTDC_Pixelformat  *)
    Alpha: longword;  (*!< Specifies the constant alpha used for blending.
                                            This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.  *)
    Alpha0: longword;  (*!< Configures the default alpha value.
                                            This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.  *)
    BlendingFactor1: longword;  (*!< Select the blending factor 1.
                                            This parameter can be one of value of @ref LTDC_BlendingFactor1  *)
    BlendingFactor2: longword;  (*!< Select the blending factor 2.
                                            This parameter can be one of value of @ref LTDC_BlendingFactor2  *)
    FBStartAdress: longword;  (*!< Configures the color frame buffer address  *)
    ImageWidth: longword;  (*!< Configures the color frame buffer line length.
                                            This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x1FFF.  *)
    ImageHeight: longword;  (*!< Specifies the number of line in frame buffer.
                                            This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF.  *)
    Backcolor: LTDC_ColorTypeDef;  (*!< Configures the layer background color.  *)
  end;

  (**
  * @brief  HAL LTDC State structures definition
   *)

const
  HAL_LTDC_STATE_RESET = $00;  (*!< LTDC not yet initialized or disabled  *)
  HAL_LTDC_STATE_READY = $01;  (*!< LTDC initialized and ready for use    *)
  HAL_LTDC_STATE_BUSY = $02;  (*!< LTDC internal process is ongoing      *)
  HAL_LTDC_STATE_TIMEOUT = $03;  (*!< LTDC Timeout state                    *)
  HAL_LTDC_STATE_ERROR = $04;  (*!< LTDC state error                      *)

type
  HAL_LTDC_StateTypeDef = integer;

  (**
  * @brief  LTDC handle Structure definition
   *)

type
  PLTDC_TypeDef = ^LTDC_TypeDef;
  PLTDC_Layer_TypeDef = ^LTDC_Layer_TypeDef;

  LTDC_HandleTypeDef = record
    Instance: PLTDC_TypeDef;  (*!< LTDC Register base address                 *)
    Init: LTDC_InitTypeDef;  (*!< LTDC parameters                            *)
    LayerCfg: array [0..MAX_LAYER - 1] of LTDC_LayerCfgTypeDef;  (*!< LTDC Layers parameters                     *)
    Lock: HAL_LockTypeDef;  (*!< LTDC Lock                                  *)
    State: HAL_LTDC_StateTypeDef;  (*!< LTDC state                                 *)
    ErrorCode: longword;  (*!< LTDC Error code                            *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup LTDC_Exported_Constants LTDC Exported Constants
  * @{
   *)

  (** @defgroup LTDC_Error_Code LTDC Error Code
  * @{
   *)

const
  HAL_LTDC_ERROR_NONE = ($00000000);  (*!< LTDC No error              *)
  HAL_LTDC_ERROR_TE = ($00000001);  (*!< LTDC Transfer error        *)
  HAL_LTDC_ERROR_FU = ($00000002);  (*!< LTDC FIFO Underrun         *)
  HAL_LTDC_ERROR_TIMEOUT = ($00000020);  (*!< LTDC Timeout error         *)
  (**
  * @}
   *)

  (** @defgroup LTDC_HS_POLARITY LTDC HS POLARITY
  * @{
   *)

  LTDC_HSPOLARITY_AL = ($00000000);  (*!< Horizontal Synchronization is active low.  *)
  LTDC_HSPOLARITY_AH = LTDC_GCR_HSPOL;  (*!< Horizontal Synchronization is active high.  *)
  (**
  * @}
   *)

  (** @defgroup LTDC_VS_POLARITY LTDC VS POLARITY
  * @{
   *)

  LTDC_VSPOLARITY_AL = ($00000000);  (*!< Vertical Synchronization is active low.  *)
  LTDC_VSPOLARITY_AH = LTDC_GCR_VSPOL;  (*!< Vertical Synchronization is active high.  *)
  (**
  * @}
   *)

  (** @defgroup LTDC_DE_POLARITY LTDC DE POLARITY
  * @{
   *)

  LTDC_DEPOLARITY_AL = ($00000000);  (*!< Data Enable, is active low.  *)
  LTDC_DEPOLARITY_AH = LTDC_GCR_DEPOL;  (*!< Data Enable, is active high.  *)
  (**
  * @}
   *)

  (** @defgroup LTDC_PC_POLARITY LTDC PC POLARITY
  * @{
   *)

  LTDC_PCPOLARITY_IPC = ($00000000);  (*!< input pixel clock.  *)
  LTDC_PCPOLARITY_IIPC = LTDC_GCR_PCPOL;  (*!< inverted input pixel clock.  *)
  (**
  * @}
   *)

  (** @defgroup LTDC_SYNC LTDC SYNC
  * @{
   *)

  LTDC_HORIZONTALSYNC = (LTDC_SSCR_HSW shr 16);  (*!< Horizontal synchronization width.  *)
  LTDC_VERTICALSYNC = LTDC_SSCR_VSH;  (*!< Vertical synchronization height.  *)
  (**
  * @}
   *)

  (** @defgroup LTDC_BACK_COLOR LTDC BACK COLOR
  * @{
   *)

  LTDC_COLOR = ($000000FF);  (*!< Color mask  *)
  (**
  * @}
   *)

  (** @defgroup LTDC_BlendingFactor1 LTDC Blending Factor1
  * @{
   *)

  LTDC_BLENDING_FACTOR1_CA = ($00000400);  (*!< Blending factor : Cte Alpha  *)
  LTDC_BLENDING_FACTOR1_PAxCA = ($00000600);  (*!< Blending factor : Cte Alpha x Pixel Alpha *)
  (**
  * @}
   *)

  (** @defgroup LTDC_BlendingFactor2 LTDC Blending Factor2
  * @{
   *)

  LTDC_BLENDING_FACTOR2_CA = ($00000005);  (*!< Blending factor : Cte Alpha  *)
  LTDC_BLENDING_FACTOR2_PAxCA = ($00000007);  (*!< Blending factor : Cte Alpha x Pixel Alpha *)
  (**
  * @}
   *)

  (** @defgroup LTDC_Pixelformat LTDC Pixel format
  * @{
   *)

  LTDC_PIXEL_FORMAT_ARGB8888 = ($00000000);  (*!< ARGB8888 LTDC pixel format  *)
  LTDC_PIXEL_FORMAT_RGB888 = ($00000001);  (*!< RGB888 LTDC pixel format    *)
  LTDC_PIXEL_FORMAT_RGB565 = ($00000002);  (*!< RGB565 LTDC pixel format    *)
  LTDC_PIXEL_FORMAT_ARGB1555 = ($00000003);  (*!< ARGB1555 LTDC pixel format  *)
  LTDC_PIXEL_FORMAT_ARGB4444 = ($00000004);  (*!< ARGB4444 LTDC pixel format  *)
  LTDC_PIXEL_FORMAT_L8 = ($00000005);  (*!< L8 LTDC pixel format        *)
  LTDC_PIXEL_FORMAT_AL44 = ($00000006);  (*!< AL44 LTDC pixel format      *)
  LTDC_PIXEL_FORMAT_AL88 = ($00000007);  (*!< AL88 LTDC pixel format      *)
  (**
  * @}
   *)

  (** @defgroup LTDC_Alpha LTDC Alpha
  * @{
   *)

  LTDC_ALPHA = LTDC_LxCACR_CONSTA;  (*!< LTDC Cte Alpha mask  *)
  (**
  * @}
   *)

  (** @defgroup LTDC_LAYER_Config LTDC LAYER Config
  * @{
   *)

  LTDC_STOPPOSITION = (LTDC_LxWHPCR_WHSPPOS shr 16);  (*!< LTDC Layer stop position   *)
  LTDC_STARTPOSITION = LTDC_LxWHPCR_WHSTPOS;  (*!< LTDC Layer start position  *)
  LTDC_COLOR_FRAME_BUFFER = LTDC_LxCFBLR_CFBLL;  (*!< LTDC Layer Line length     *)
  LTDC_LINE_NUMBER = LTDC_LxCFBLNR_CFBLNBR;  (*!< LTDC Layer Line number     *)
  (**
  * @}
   *)

  (** @defgroup LTDC_Interrupts LTDC Interrupts
  * @{
   *)

  LTDC_IT_LI = LTDC_IER_LIE;
  LTDC_IT_FU = LTDC_IER_FUIE;
  LTDC_IT_TE = LTDC_IER_TERRIE;
  LTDC_IT_RR = LTDC_IER_RRIE;
  (**
  * @}
   *)

  (** @defgroup LTDC_Flag LTDC Flag
  * @{
   *)

  LTDC_FLAG_LI = LTDC_ISR_LIF;
  LTDC_FLAG_FU = LTDC_ISR_FUIF;
  LTDC_FLAG_TE = LTDC_ISR_TERRIF;
  LTDC_FLAG_RR = LTDC_ISR_RRIF;
  (**
  * @}
   *)
function HAL_LTDC_Init(var hltdc: LTDC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_LTDC_DeInit(var hltdc: LTDC_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_LTDC_MspInit(var hltdc: LTDC_HandleTypeDef); external name 'HAL_LTDC_MspInit';
procedure HAL_LTDC_MspDeInit(var hltdc: LTDC_HandleTypeDef); external name 'HAL_LTDC_MspDeInit';
procedure HAL_LTDC_ErrorCallback(var hltdc: LTDC_HandleTypeDef); external name 'HAL_LTDC_ErrorCallback';
procedure HAL_LTDC_LineEvenCallback(var hltdc: LTDC_HandleTypeDef); external name 'HAL_LTDC_LineEvenCallback';
(**
 * @}
  *)

(** @addtogroup LTDC_Exported_Functions_Group2
 * @{
  *)

(* IO operation functions **************************************************** *)
procedure HAL_LTDC_IRQHandler(var hltdc: LTDC_HandleTypeDef);
(**
 * @}
  *)

(** @addtogroup LTDC_Exported_Functions_Group3
 * @{
  *)
(* Peripheral Control functions ********************************************** *)
function HAL_LTDC_ConfigLayer(var hltdc: LTDC_HandleTypeDef; var pLayerCfg: LTDC_LayerCfgTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_SetWindowSize(var hltdc: LTDC_HandleTypeDef; XSize, YSize, LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_SetWindowPosition(var hltdc: LTDC_HandleTypeDef; X0, Y0, LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_SetPixelFormat(var hltdc: LTDC_HandleTypeDef; Pixelformat, LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_SetAlpha(var hltdc: LTDC_HandleTypeDef; Alpha, LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_SetAddress(var hltdc: LTDC_HandleTypeDef; Address, LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_ConfigColorKeying(var hltdc: LTDC_HandleTypeDef; RGBValue, LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_ConfigCLUT(var hltdc: LTDC_HandleTypeDef; pCLUT: Plongword; CLUTSize, LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_EnableColorKeying(var hltdc: LTDC_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_DisableColorKeying(var hltdc: LTDC_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_EnableCLUT(var hltdc: LTDC_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_DisableCLUT(var hltdc: LTDC_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
function HAL_LTDC_ProgramLineEvent(var hltdc: LTDC_HandleTypeDef; Line: longword): HAL_StatusTypeDef;
function HAL_LTDC_EnableDither(var hltdc: LTDC_HandleTypeDef): HAL_StatusTypeDef;
function HAL_LTDC_DisableDither(var hltdc: LTDC_HandleTypeDef): HAL_StatusTypeDef;

(**
 * @}
  *)

(** @addtogroup LTDC_Exported_Functions_Group4
 * @{
  *)

(* Peripheral State functions ************************************************ *)
function HAL_LTDC_GetState(var hltdc: LTDC_HandleTypeDef): HAL_LTDC_StateTypeDef;
function HAL_LTDC_GetError(var hltdc: LTDC_HandleTypeDef): longword;

implementation

function LTDC_LAYER(var __handle__: LTDC_HandleTypeDef; __LAYER__: longword): PLTDC_Layer_TypeDef;
begin
  exit(PLTDC_Layer_TypeDef(@pbyte(__HANDLE__.Instance)[$84 + ($80 * (__LAYER__))]));
end;

(** @brief Reset LTDC handle state
  * @param  __HANDLE__: specifies the LTDC handle.
  * @retval None
  *)
procedure __HAL_LTDC_RESET_HANDLE_STATE(var __handle__: LTDC_HandleTypeDef);
begin
  __HANDLE__.State := HAL_LTDC_STATE_RESET;
end;

(**
  * @brief  Enable the LTDC.
  * @param  __HANDLE__: LTDC handle
  * @retval None.
  *)
procedure __HAL_LTDC_ENABLE(var __handle__: LTDC_HandleTypeDef);
begin
  __HANDLE__.Instance^.GCR := __HANDLE__.Instance^.GCR or LTDC_GCR_LTDCEN;
end;

(**
  * @brief  Disable the LTDC.
  * @param  __HANDLE__: LTDC handle
  * @retval None.
  *)
procedure __HAL_LTDC_DISABLE(var __handle__: LTDC_HandleTypeDef);
begin
  __HANDLE__.Instance^.GCR := __HANDLE__.Instance^.GCR and (not (LTDC_GCR_LTDCEN));
end;

(**
  * @brief  Enable the LTDC Layer.
  * @param  __HANDLE__: LTDC handle
  * @param  __LAYER__: Specify the layer to be enabled
  *                     This parameter can be 0 or 1
  * @retval None.
  *)
procedure __HAL_LTDC_LAYER_ENABLE(var __handle__: LTDC_HandleTypeDef; __LAYER__: longword);
var
  tmp: PLTDC_Layer_TypeDef;
begin
  tmp := LTDC_LAYER(__HANDLE__, __LAYER__);
  tmp^.CR := tmp^.CR or LTDC_LxCR_LEN;
end;

(**
  * @brief  Disable the LTDC Layer.
  * @param  __HANDLE__: LTDC handle
  * @param  __LAYER__: Specify the layer to be disabled
  *                     This parameter can be 0 or 1
  * @retval None.
  *)
procedure __HAL_LTDC_LAYER_DISABLE(var __handle__: LTDC_HandleTypeDef; __LAYER__: longword);
var
  tmp: PLTDC_Layer_TypeDef;
begin
  tmp := LTDC_LAYER(__HANDLE__, __LAYER__);
  tmp^.CR := tmp^.CR and (not LTDC_LxCR_LEN);
end;

(**
  * @brief  Reload  Layer Configuration.
  * @param  __HANDLE__: LTDC handle
  * @retval None.
  *)
procedure __HAL_LTDC_RELOAD_CONFIG(var __handle__: LTDC_HandleTypeDef);
begin
  __HANDLE__.Instance^.SRCR := __HANDLE__.Instance^.SRCR or LTDC_SRCR_IMR;
end;

(* Interrupt and Flag management *)
(**
  * @brief  Get the LTDC pending flags.
  * @param  __HANDLE__: LTDC handle
  * @param  __FLAG__: Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg LTDC_FLAG_LI: Line Interrupt flag
  *            @arg LTDC_FLAG_FU: FIFO Underrun Interrupt flag
  *            @arg LTDC_FLAG_TE: Transfer Error interrupt flag
  *            @arg LTDC_FLAG_RR: Register Reload Interrupt Flag
  * @retval The state of FLAG (SET or RESET).
  *)
function __HAL_LTDC_GET_FLAG(var __handle__: LTDC_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit((__HANDLE__.Instance^.ISR and (__FLAG__)) <> 0);
end;

(**
  * @brief  Clears the LTDC pending flags.
  * @param  __HANDLE__: LTDC handle
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg LTDC_FLAG_LI: Line Interrupt flag
  *            @arg LTDC_FLAG_FU: FIFO Underrun Interrupt flag
  *            @arg LTDC_FLAG_TE: Transfer Error interrupt flag
  *            @arg LTDC_FLAG_RR: Register Reload Interrupt Flag
  * @retval None
  *)
procedure __HAL_LTDC_CLEAR_FLAG(var __handle__: LTDC_HandleTypeDef; __FLAG__: longword);
begin
  __HANDLE__.Instance^.ICR := (__FLAG__);
end;

(**
  * @brief  Enables the specified LTDC interrupts.
  * @param  __HANDLE__: LTDC handle
  * @param __INTERRUPT__: specifies the LTDC interrupt sources to be enabled.
  *          This parameter can be any combination of the following values:
  *            @arg LTDC_IT_LI: Line Interrupt flag
  *            @arg LTDC_IT_FU: FIFO Underrun Interrupt flag
  *            @arg LTDC_IT_TE: Transfer Error interrupt flag
  *            @arg LTDC_IT_RR: Register Reload Interrupt Flag
  * @retval None
  *)
procedure __HAL_LTDC_ENABLE_IT(var __handle__: LTDC_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.IER := __HANDLE__.Instance^.IER or (__INTERRUPT__);
end;

(**
  * @brief  Disables the specified LTDC interrupts.
  * @param  __HANDLE__: LTDC handle
  * @param __INTERRUPT__: specifies the LTDC interrupt sources to be disabled.
  *          This parameter can be any combination of the following values:
  *            @arg LTDC_IT_LI: Line Interrupt flag
  *            @arg LTDC_IT_FU: FIFO Underrun Interrupt flag
  *            @arg LTDC_IT_TE: Transfer Error interrupt flag
  *            @arg LTDC_IT_RR: Register Reload Interrupt Flag
  * @retval None
  *)
procedure __HAL_LTDC_DISABLE_IT(var __handle__: LTDC_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.IER := __HANDLE__.Instance^.IER and (not (__INTERRUPT__));
end;

(**
  * @brief  Checks whether the specified LTDC interrupt has occurred or not.
  * @param  __HANDLE__: LTDC handle
  * @param  __INTERRUPT__: specifies the LTDC interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg LTDC_IT_LI: Line Interrupt flag
  *            @arg LTDC_IT_FU: FIFO Underrun Interrupt flag
  *            @arg LTDC_IT_TE: Transfer Error interrupt flag
  *            @arg LTDC_IT_RR: Register Reload Interrupt Flag
  * @retval The state of INTERRUPT (SET or RESET).
  *)
function __HAL_LTDC_GET_IT_SOURCE(var __handle__: LTDC_HandleTypeDef; __INTERRUPT__: longword): boolean;
begin
  exit((__HANDLE__.Instance^.ISR and (__INTERRUPT__)) <> 0);
end;

procedure LTDC_SetConfig(var hltdc: LTDC_HandleTypeDef; var pLayerCfg: LTDC_LayerCfgTypeDef; LayerIdx: longword);
var
  tmp,tmp1,tmp2: longword;
  tmpl: PLTDC_Layer_TypeDef;
begin
  (* Configures the horizontal start and stop position *)
  tmp := ((pLayerCfg.WindowX1 + ((hltdc.Instance^.BPCR and LTDC_BPCR_AHBP) shr 16)) shl 16);
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.WHPCR := tmpl^.WHPCR and (not (LTDC_LxWHPCR_WHSTPOS or LTDC_LxWHPCR_WHSPPOS));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.WHPCR := ((pLayerCfg.WindowX0 + ((hltdc.Instance^.BPCR and LTDC_BPCR_AHBP) shr 16) + 1) or tmp);

  (* Configures the vertical start and stop position *)
  tmp := ((pLayerCfg.WindowY1 + (hltdc.Instance^.BPCR and LTDC_BPCR_AVBP)) shl 16);
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.WVPCR := tmpl^.WVPCR and (not (LTDC_LxWVPCR_WVSTPOS or LTDC_LxWVPCR_WVSPPOS));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.WVPCR  := ((pLayerCfg.WindowY0 + (hltdc.Instance^.BPCR and LTDC_BPCR_AVBP) + 1) or tmp);

  (* Specifies the pixel format *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.PFCR := tmpl^.PFCR and (not (LTDC_LxPFCR_PF));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.PFCR := (pLayerCfg.PixelFormat);

  (* Configures the default color values *)
  tmp := ((pLayerCfg.Backcolor.Green) shl 8);
  tmp1 := ((pLayerCfg.Backcolor.Red) shl 16);
  tmp2 := (pLayerCfg.Alpha0 shl 24);
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.DCCR := tmpl^.DCCR and (not (LTDC_LxDCCR_DCBLUE or LTDC_LxDCCR_DCGREEN or LTDC_LxDCCR_DCRED or LTDC_LxDCCR_DCALPHA));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.DCCR := (pLayerCfg.Backcolor.Blue or tmp or tmp1 or tmp2);

  (* Specifies the constant alpha value *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CACR := tmpl^.CACR and (not (LTDC_LxCACR_CONSTA));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CACR := (pLayerCfg.Alpha);

  (* Specifies the blending factors *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.BFCR := tmpl^.BFCR and (not (LTDC_LxBFCR_BF2 or LTDC_LxBFCR_BF1));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.BFCR := (pLayerCfg.BlendingFactor1 or pLayerCfg.BlendingFactor2);

  (* Configures the color frame buffer start address *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CFBAR := tmpl^.CFBAR and (not (LTDC_LxCFBAR_CFBADD));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CFBAR := (pLayerCfg.FBStartAdress);

  if(pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888) then
    tmp := 4
  else if (pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888) then
    tmp := 3
  else if((pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB4444) or     (pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565)   or       (pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB1555) or         (pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_AL88))then
    tmp := 2
  else
    tmp := 1;

  (* Configures the color frame buffer pitch in byte *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CFBLR := tmpl^.CFBLR and (not (LTDC_LxCFBLR_CFBLL or LTDC_LxCFBLR_CFBP));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CFBLR  := (((pLayerCfg.ImageWidth * tmp) shl 16) or (((pLayerCfg.WindowX1 - pLayerCfg.WindowX0) * tmp)  + 3));

  (* Configures the frame buffer line number *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CFBLNR := tmpl^.CFBLNR and (not (LTDC_LxCFBLNR_CFBLNBR));
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CFBLNR  := (pLayerCfg.ImageHeight);

  (* Enable LTDC_Layer by setting LEN bit *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CR := tmpl^.CR or LTDC_LxCR_LEN;
end;

function HAL_LTDC_Init(var hltdc: LTDC_HandleTypeDef): HAL_StatusTypeDef;
var
  tmp, tmp1: longword;
begin
  if(hltdc.State = HAL_LTDC_STATE_RESET) then
  begin
    (* Allocate lock resource and initialize it *)
    hltdc.Lock := HAL_UNLOCKED;
    (* Init the low level hardware *)
    HAL_LTDC_MspInit(hltdc);
  end;

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Configures the HS, VS, DE and PC polarity *)
  hltdc.Instance^.GCR := hltdc.Instance^.GCR and (not (LTDC_GCR_HSPOL or LTDC_GCR_VSPOL or LTDC_GCR_DEPOL or LTDC_GCR_PCPOL));
  hltdc.Instance^.GCR := hltdc.Instance^.GCR or (hltdc.Init.HSPolarity or hltdc.Init.VSPolarity or   hltdc.Init.DEPolarity or hltdc.Init.PCPolarity);

  (* Sets Synchronization size *)
  hltdc.Instance^.SSCR := hltdc.Instance^.SSCR and (not (LTDC_SSCR_VSH or LTDC_SSCR_HSW));
  tmp := (hltdc.Init.HorizontalSync shl 16);
  hltdc.Instance^.SSCR := hltdc.Instance^.SSCR or (tmp or hltdc.Init.VerticalSync);

  (* Sets Accumulated Back porch *)
  hltdc.Instance^.BPCR := hltdc.Instance^.BPCR and (not (LTDC_BPCR_AVBP or LTDC_BPCR_AHBP));
  tmp := (hltdc.Init.AccumulatedHBP shl 16);
  hltdc.Instance^.BPCR := hltdc.Instance^.BPCR or (tmp or hltdc.Init.AccumulatedVBP);

  (* Sets Accumulated Active Width *)
  hltdc.Instance^.AWCR := hltdc.Instance^.AWCR and (not (LTDC_AWCR_AAH or LTDC_AWCR_AAW));
  tmp := (hltdc.Init.AccumulatedActiveW shl 16);
  hltdc.Instance^.AWCR := hltdc.Instance^.AWCR or (tmp or hltdc.Init.AccumulatedActiveH);

  (* Sets Total Width *)
  hltdc.Instance^.TWCR := hltdc.Instance^.TWCR and (not (LTDC_TWCR_TOTALH or LTDC_TWCR_TOTALW));
  tmp := (hltdc.Init.TotalWidth shl 16);
  hltdc.Instance^.TWCR := hltdc.Instance^.TWCR or (tmp or hltdc.Init.TotalHeigh);

  (* Sets the background color value *)
  tmp := ((hltdc.Init.Backcolor.Green) shl 8);
  tmp1 := ((hltdc.Init.Backcolor.Red) shl 16);
  hltdc.Instance^.BCCR := hltdc.Instance^.BCCR and (not (LTDC_BCCR_BCBLUE or LTDC_BCCR_BCGREEN or LTDC_BCCR_BCRED));
  hltdc.Instance^.BCCR := hltdc.Instance^.BCCR or (tmp1 or tmp or hltdc.Init.Backcolor.Blue);

  (* Enable the transfer Error interrupt *)
  __HAL_LTDC_ENABLE_IT(hltdc, LTDC_IT_TE);

  (* Enable the FIFO underrun interrupt *)
  __HAL_LTDC_ENABLE_IT(hltdc, LTDC_IT_FU);

  (* Enable LTDC by setting LTDCEN bit *)
  __HAL_LTDC_ENABLE(hltdc);

  (* Initialize the error code *)
  hltdc.ErrorCode := HAL_LTDC_ERROR_NONE;

  (* Initialize the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  exit(HAL_OK);
end;

function HAL_LTDC_DeInit(var hltdc: LTDC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* DeInit the low level hardware *)
  HAL_LTDC_MspDeInit(hltdc);

  (* Initialize the error code *)
  hltdc.ErrorCode := HAL_LTDC_ERROR_NONE;

  (* Initialize the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_RESET;

  (* Release Lock *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

{procedure HAL_LTDC_MspInit(var hltdc: LTDC_HandleTypeDef); public name 'HAL_LTDC_MspInit';
asm
  .weak HAL_LTDC_MspInit
end;

procedure HAL_LTDC_MspDeInit(var hltdc: LTDC_HandleTypeDef);
begin

end;

procedure HAL_LTDC_ErrorCallback(var hltdc: LTDC_HandleTypeDef);
begin

end;

procedure HAL_LTDC_LineEvenCallback(var hltdc: LTDC_HandleTypeDef);
begin

end;}

procedure HAL_LTDC_IRQHandler(var hltdc: LTDC_HandleTypeDef);
begin
  (* Transfer Error Interrupt management ***************************************)
  if(__HAL_LTDC_GET_FLAG(hltdc, LTDC_FLAG_TE))then
  begin
    if(__HAL_LTDC_GET_IT_SOURCE(hltdc, LTDC_IT_TE))then
    begin
      (* Disable the transfer Error interrupt *)
      __HAL_LTDC_DISABLE_IT(hltdc, LTDC_IT_TE);

      (* Clear the transfer error flag *)
      __HAL_LTDC_CLEAR_FLAG(hltdc, LTDC_FLAG_TE);

      (* Update error code *)
      hltdc.ErrorCode := hltdc.ErrorCode or HAL_LTDC_ERROR_TE;

      (* Change LTDC state *)
      hltdc.State := HAL_LTDC_STATE_ERROR;

      (* Process unlocked *)
      __HAL_Unlock(hltdc.lock);

      (* Transfer error Callback *)
      HAL_LTDC_ErrorCallback(hltdc);
    end
  end ;
  (* FIFO underrun Interrupt management ***************************************)
  if(__HAL_LTDC_GET_FLAG(hltdc, LTDC_FLAG_FU) )then
  begin
    if(__HAL_LTDC_GET_IT_SOURCE(hltdc, LTDC_IT_FU))then
    begin
      (* Disable the FIFO underrun interrupt *)
      __HAL_LTDC_DISABLE_IT(hltdc, LTDC_IT_FU);

      (* Clear the FIFO underrun flag *)
      __HAL_LTDC_CLEAR_FLAG(hltdc, LTDC_FLAG_FU);

      (* Update error code *)
      hltdc.ErrorCode := hltdc.ErrorCode or HAL_LTDC_ERROR_FU;

      (* Change LTDC state *)
      hltdc.State := HAL_LTDC_STATE_ERROR;

      (* Process unlocked *)
      __HAL_Unlock(hltdc.lock);

      (* Transfer error Callback *)
      HAL_LTDC_ErrorCallback(hltdc);
    end
  end;
  (* Line Interrupt management ************************************************)
  if(__HAL_LTDC_GET_FLAG(hltdc, LTDC_FLAG_LI) )then
  begin
    if(__HAL_LTDC_GET_IT_SOURCE(hltdc, LTDC_IT_LI) )then
    begin
      (* Disable the Line interrupt *)
      __HAL_LTDC_DISABLE_IT(hltdc, LTDC_IT_LI);

      (* Clear the Line interrupt flag *)
      __HAL_LTDC_CLEAR_FLAG(hltdc, LTDC_FLAG_LI);

      (* Change LTDC state *)
      hltdc.State := HAL_LTDC_STATE_READY;

      (* Process unlocked *)
      __HAL_Unlock(hltdc.lock);

      (* Line interrupt Callback *)
      HAL_LTDC_LineEvenCallback(hltdc);
    end
  end
end;

function HAL_LTDC_ConfigLayer(var hltdc: LTDC_HandleTypeDef; var pLayerCfg: LTDC_LayerCfgTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Copy new layer configuration into handle structure *)
  hltdc.LayerCfg[LayerIdx] := pLayerCfg;

  (* Configure the LTDC Layer *)
  LTDC_SetConfig(hltdc, pLayerCfg, LayerIdx);

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Initialize the LTDC state*)
  hltdc.State  := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_SetWindowSize(var hltdc: LTDC_HandleTypeDef; XSize, YSize, LayerIdx: longword): HAL_StatusTypeDef;
var
  pLayerCfg: PLTDC_LayerCfgTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Get layer configuration from handle structure *)
  pLayerCfg := @hltdc.LayerCfg[LayerIdx];

  (* update horizontal start/stop *)
  pLayerCfg^.WindowX0 := 0;
  pLayerCfg^.WindowX1 := XSize + pLayerCfg^.WindowX0;

  (* update vertical start/stop *)
  pLayerCfg^.WindowY0 := 0;
  pLayerCfg^.WindowY1 := YSize + pLayerCfg^.WindowY0;

  (* Reconfigures the color frame buffer pitch in byte *)
  pLayerCfg^.ImageWidth := XSize;

  (* Reconfigures the frame buffer line number *)
  pLayerCfg^.ImageHeight := YSize;

  (* Set LTDC parameters *)
  LTDC_SetConfig(hltdc, pLayerCfg^, LayerIdx);

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_SetWindowPosition(var hltdc: LTDC_HandleTypeDef; X0, Y0, LayerIdx: longword): HAL_StatusTypeDef;
var
  pLayerCfg: PLTDC_LayerCfgTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Get layer configuration from handle structure *)
  pLayerCfg := @hltdc.LayerCfg[LayerIdx];

  (* update horizontal start/stop *)
  pLayerCfg^.WindowX0 := X0;
  pLayerCfg^.WindowX1 := X0 + pLayerCfg^.ImageWidth;

  (* update vertical start/stop *)
  pLayerCfg^.WindowY0 := Y0;
  pLayerCfg^.WindowY1 := Y0 + pLayerCfg^.ImageHeight;

  (* Set LTDC parameters *)
  LTDC_SetConfig(hltdc, pLayerCfg^, LayerIdx);

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_SetPixelFormat(var hltdc: LTDC_HandleTypeDef; Pixelformat, LayerIdx: longword): HAL_StatusTypeDef;
var
  pLayerCfg: PLTDC_LayerCfgTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Get layer configuration from handle structure *)
  pLayerCfg := @hltdc.LayerCfg[LayerIdx];

  (* Reconfigure the pixel format *)
  pLayerCfg^.PixelFormat := Pixelformat;

  (* Set LTDC parameters *)
  LTDC_SetConfig(hltdc, pLayerCfg^, LayerIdx);

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_SetAlpha(var hltdc: LTDC_HandleTypeDef; Alpha, LayerIdx: longword): HAL_StatusTypeDef;
var
  pLayerCfg: PLTDC_LayerCfgTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Get layer configuration from handle structure *)
  pLayerCfg := @hltdc.LayerCfg[LayerIdx];

  (* Reconfigure the Alpha value *)
  pLayerCfg^.Alpha := Alpha;

  (* Set LTDC parameters *)
  LTDC_SetConfig(hltdc, pLayerCfg^, LayerIdx);

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_SetAddress(var hltdc: LTDC_HandleTypeDef; Address, LayerIdx: longword): HAL_StatusTypeDef;
var
  pLayerCfg: PLTDC_LayerCfgTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Get layer configuration from handle structure *)
  pLayerCfg := @hltdc.LayerCfg[LayerIdx];

  (* Reconfigure the Address *)
  pLayerCfg^.FBStartAdress := Address;

  (* Set LTDC parameters *)
  LTDC_SetConfig(hltdc, pLayerCfg^, LayerIdx);

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_ConfigColorKeying(var hltdc: LTDC_HandleTypeDef; RGBValue, LayerIdx: longword): HAL_StatusTypeDef;
var
  tmpl: PLTDC_Layer_TypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Configures the default color values *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CKCR := tmpl^.CKCR and (not (LTDC_LxCKCR_CKBLUE or LTDC_LxCKCR_CKGREEN or LTDC_LxCKCR_CKRED));
  LTDC_LAYER(hltdc, LayerIdx)^.CKCR  := RGBValue;

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_ConfigCLUT(var hltdc: LTDC_HandleTypeDef; pCLUT: Plongword; CLUTSize, LayerIdx: longword): HAL_StatusTypeDef;
var
  pcounter: Plongword;
  counter,
  tmp: longword;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  for counter := 0 to CLUTSize-1 do
  begin
    if(hltdc.LayerCfg[LayerIdx].PixelFormat = LTDC_PIXEL_FORMAT_AL44)then
      tmp  := (((counter + 16*counter) shl 24) or (pCLUT^ and $FF) or (pCLUT^ and $FF00) or (pCLUT^ and $FF0000))
    else
      tmp  := ((counter shl 24) or ((pCLUT^) and $FF) or ((pCLUT^) and $FF00) or ((pCLUT^) and $FF0000));

    pcounter := @pbyte(pCLUT)[sizeof(longword)];
    pCLUT := pcounter;

    (* Specifies the C-LUT address and RGB value *)
    LTDC_LAYER(hltdc, LayerIdx)^.CLUTWR  := tmp;
  end;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_EnableColorKeying(var hltdc: LTDC_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
var
  tmpl: PLTDC_Layer_TypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Enable LTDC color keying by setting COLKEN bit *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CR := tmpl^.CR or LTDC_LxCR_COLKEN;

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_DisableColorKeying(var hltdc: LTDC_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
var
  tmpl: PLTDC_Layer_TypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Disable LTDC color keying by setting COLKEN bit *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CR := tmpl^.CR and (not LTDC_LxCR_COLKEN);

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_EnableCLUT(var hltdc: LTDC_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
var
  tmpl: PLTDC_Layer_TypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Disable LTDC color lookup table by setting CLUTEN bit *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CR := tmpl^.CR or LTDC_LxCR_CLUTEN;

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_DisableCLUT(var hltdc: LTDC_HandleTypeDef; LayerIdx: longword): HAL_StatusTypeDef;
var
  tmpl: PLTDC_Layer_TypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Disable LTDC color lookup table by setting CLUTEN bit *)
  tmpl:=LTDC_LAYER(hltdc, LayerIdx); tmpl^.CR := tmpl^.CR and (not LTDC_LxCR_CLUTEN);

  (* Sets the Reload type *)
  hltdc.Instance^.SRCR := LTDC_SRCR_IMR;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_ProgramLineEvent(var hltdc: LTDC_HandleTypeDef; Line: longword): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Enable the Line interrupt *)
  __HAL_LTDC_ENABLE_IT(hltdc, LTDC_IT_LI);

  (* Sets the Line Interrupt position *)
  LTDC.LIPCR := Line;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_EnableDither(var hltdc: LTDC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Enable Dither by setting DTEN bit *)
  LTDC.GCR := LTDC.GCR or LTDC_GCR_DTEN;

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_DisableDither(var hltdc: LTDC_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Process locked *)
  __HAL_Lock(hltdc.lock);

  (* Change LTDC peripheral state *)
  hltdc.State := HAL_LTDC_STATE_BUSY;

  (* Disable Dither by setting DTEN bit *)
  LTDC.GCR := LTDC.GCR and (not LTDC_GCR_DTEN);

  (* Change the LTDC state*)
  hltdc.State := HAL_LTDC_STATE_READY;

  (* Process unlocked *)
  __HAL_Unlock(hltdc.lock);

  exit(HAL_OK);
end;

function HAL_LTDC_GetState(var hltdc: LTDC_HandleTypeDef): HAL_LTDC_StateTypeDef;
begin
  exit(hltdc.State);
end;

function HAL_LTDC_GetError(var hltdc: LTDC_HandleTypeDef): longword;
begin
  exit(hltdc.ErrorCode);
end;

end.
