(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_pcd.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of PCD HAL module.
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
unit stm32f7xx_hal_pcd;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal,
  stm32f7xx_ll_usb;

(**
  * @brief  PCD State structure definition
   *)

const
  HAL_PCD_STATE_RESET = $00;
  HAL_PCD_STATE_READY = $01;
  HAL_PCD_STATE_ERROR = $02;
  HAL_PCD_STATE_BUSY = $03;
  HAL_PCD_STATE_TIMEOUT = $04;

type
  PCD_StateTypeDef = integer;

(* Device LPM suspend state  *)

const
  LPM_L0 = $00;  (* on  *)
  LPM_L1 = $01;  (* LPM L1 sleep  *)
  LPM_L2 = $02;  (* suspend  *)
  LPM_L3 = $03;  (* off  *)

type
  PCD_LPM_StateTypeDef = integer;

  PCD_TypeDef = USB_OTG_GlobalTypeDef;

  PCD_InitTypeDef = USB_OTG_CfgTypeDef;

  PCD_EPTypeDef = USB_OTG_EPTypeDef;
  PPCD_EPTypeDef = ^PCD_EPTypeDef;

  (**
  * @brief  PCD Handle Structure definition
   *)
  PPCD_TypeDef = ^PCD_TypeDef;

  PCD_HandleTypeDef = packed record
    Instance: PPCD_TypeDef;  (*!< Register base address               *)
    Init: PCD_InitTypeDef;  (*!< PCD required parameters             *)
    IN_ep: array [0..14] of PCD_EPTypeDef;  (*!< IN endpoint parameters              *)
    OUT_ep: array [0..14] of PCD_EPTypeDef;  (*!< OUT endpoint parameters             *)
    Lock: HAL_LockTypeDef;  (*!< PCD peripheral status               *)
    State: PCD_StateTypeDef;  (*!< PCD communication state             *)
    Setup: array [0..11] of longword;  (*!< Setup packet buffer                 *)
    LPM_State: PCD_LPM_StateTypeDef;  (*!< LPM State                           *)
    BESL: longword;
    lpm_active: boolean;  (*!< Enable or disable the Link Power Management .
                                        This parameter can be set to ENABLE or DISABLE  *)
    pData: pointer;  (*!< Pointer to upper stack Handler  *)
  end;

  (**
  * @}
   *)

(* Include PCD HAL Extension module  *)


(* Exported constants -------------------------------------------------------- *)

  (** @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
   *)

  (** @defgroup PCD_Speed PCD Speed
  * @{
   *)

const
  PCD_SPEED_HIGH = 0;
  PCD_SPEED_HIGH_IN_FULL = 1;
  PCD_SPEED_FULL = 2;
  (**
  * @}
   *)

  (** @defgroup PCD_PHY_Module PCD PHY Module
  * @{
   *)

  PCD_PHY_ULPI = 1;
  PCD_PHY_EMBEDDED = 2;
  (**
  * @}
   *)

  (** @defgroup PCD_Turnaround_Timeout Turnaround Timeout Value
  * @{
   *)

  USBD_HS_TRDT_VALUE = 9;
  USBD_FS_TRDT_VALUE = 5;

const
  USB_OTG_FS_WAKEUP_EXTI_RISING_EDGE = ($08);
  USB_OTG_FS_WAKEUP_EXTI_FALLING_EDGE = ($0C);
  USB_OTG_FS_WAKEUP_EXTI_RISING_FALLING_EDGE = ($10);
  USB_OTG_HS_WAKEUP_EXTI_RISING_EDGE = ($08);
  USB_OTG_HS_WAKEUP_EXTI_FALLING_EDGE = ($0C);
  USB_OTG_HS_WAKEUP_EXTI_RISING_FALLING_EDGE = ($10);
  USB_OTG_HS_WAKEUP_EXTI_LINE = ($00100000) (* not < External interrupt line 20 Connected to the USB HS EXTI Line *);
  USB_OTG_FS_WAKEUP_EXTI_LINE = ($00040000) (* not < External interrupt line 18 Connected to the USB FS EXTI Line *);

(* Initialization/de-initialization functions  ******************************* *)
function HAL_PCD_Init(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
function HAL_PCD_DeInit(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_PCD_MspInit(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_MspInit';
procedure HAL_PCD_MspDeInit(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_MspDeInit';

(* I/O operation functions  ************************************************** *)
(* Non-Blocking mode: Interrupt  *)
function HAL_PCD_Start(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
function HAL_PCD_Stop(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_PCD_IRQHandler(var hpcd: PCD_HandleTypeDef);
procedure HAL_PCD_DataOutStageCallback(var hpcd: PCD_HandleTypeDef; epnum: byte); external name 'HAL_PCD_DataOutStageCallback';
procedure HAL_PCD_DataInStageCallback(var hpcd: PCD_HandleTypeDef; epnum: byte); external name 'HAL_PCD_DataInStageCallback';
procedure HAL_PCD_SetupStageCallback(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_SetupStageCallback';
procedure HAL_PCD_SOFCallback(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_SOFCallback';
procedure HAL_PCD_ResetCallback(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_ResetCallback';
procedure HAL_PCD_SuspendCallback(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_SuspendCallback';
procedure HAL_PCD_ResumeCallback(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_ResumeCallback';
procedure HAL_PCD_ISOOUTIncompleteCallback(var hpcd: PCD_HandleTypeDef; epnum: byte); external name 'HAL_PCD_ISOOUTIncompleteCallback';
procedure HAL_PCD_ISOINIncompleteCallback(var hpcd: PCD_HandleTypeDef; epnum: byte); external name 'HAL_PCD_ISOINIncompleteCallback';
procedure HAL_PCD_ConnectCallback(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_ConnectCallback';
procedure HAL_PCD_DisconnectCallback(var hpcd: PCD_HandleTypeDef); external name 'HAL_PCD_DisconnectCallback';

(* Peripheral Control functions  ********************************************* *)
function HAL_PCD_DevConnect(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
function HAL_PCD_DevDisconnect(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
function HAL_PCD_SetAddress(var hpcd: PCD_HandleTypeDef; address: byte): HAL_StatusTypeDef;
function HAL_PCD_EP_Open(var hpcd: PCD_HandleTypeDef; ep_addr: byte; ep_mps: word; ep_type: byte): HAL_StatusTypeDef;
function HAL_PCD_EP_Close(var hpcd: PCD_HandleTypeDef; ep_addr: byte): HAL_StatusTypeDef;
function HAL_PCD_EP_Receive(var hpcd: PCD_HandleTypeDef; ep_addr: byte; var pBuf; len: longword): HAL_StatusTypeDef;
function HAL_PCD_EP_Transmit(var hpcd: PCD_HandleTypeDef; ep_addr: byte; const pBuf; len: longword): HAL_StatusTypeDef;
function HAL_PCD_EP_GetRxCount(var hpcd: PCD_HandleTypeDef; ep_addr: byte): word;
function HAL_PCD_EP_SetStall(var hpcd: PCD_HandleTypeDef; ep_addr: byte): HAL_StatusTypeDef;
function HAL_PCD_EP_ClrStall(var hpcd: PCD_HandleTypeDef; ep_addr: byte): HAL_StatusTypeDef;
function HAL_PCD_EP_Flush(var hpcd: PCD_HandleTypeDef; ep_addr: byte): HAL_StatusTypeDef;
function HAL_PCD_ActivateRemoteWakeup(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
function HAL_PCD_DeActivateRemoteWakeup(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;

(* Peripheral State functions  *********************************************** *)
function HAL_PCD_GetState(var hpcd: PCD_HandleTypeDef): PCD_StateTypeDef;

procedure __HAL_PCD_ENABLE(var __HANDLE__: PCD_HandleTypeDef);
procedure __HAL_PCD_DISABLE(var __HANDLE__: PCD_HandleTypeDef);
function __HAL_PCD_GET_FLAG(var __HANDLE__: PCD_HandleTypeDef; __INTERRUPT__: longword): boolean;
procedure __HAL_PCD_CLEAR_FLAG(var __HANDLE__: PCD_HandleTypeDef; __INTERRUPT__: longword);
function __HAL_PCD_IS_INVALID_INTERRUPT(var __HANDLE__: PCD_HandleTypeDef): boolean;
procedure __HAL_PCD_UNGATE_PHYCLOCK(var __HANDLE__: PCD_HandleTypeDef);
procedure __HAL_PCD_GATE_PHYCLOCK(var __HANDLE__: PCD_HandleTypeDef);
function __HAL_PCD_IS_PHY_SUSPENDED(var __HANDLE__: PCD_HandleTypeDef): boolean;
procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_IT;
procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_DISABLE_IT;
function __HAL_USB_OTG_HS_WAKEUP_EXTI_GET_FLAG: boolean;
procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_CLEAR_FLAG;
procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_RISING_EDGE;
procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_FALLING_EDGE;
procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_RISING_FALLING_EDGE;
procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_GENERATE_SWIT;
procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_IT;
procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_DISABLE_IT;
function __HAL_USB_OTG_FS_WAKEUP_EXTI_GET_FLAG: boolean;
procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG;
procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_RISING_EDGE;
procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_FALLING_EDGE;
procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_RISING_FALLING_EDGE;
procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_GENERATE_SWIT;

implementation

uses
  stm32f7xx_hal_pcd_ex;

function PCD_WriteEmptyTxFifo(var hpcd: PCD_HandleTypeDef; epnum: longword): HAL_StatusTypeDef;
var
  USBx: PPCD_TypeDef;
  ep: PPCD_EPTypeDef;
  len: longword;
  len32b, fifoemptymsk: longword;
begin
  USBx := hpcd.Instance;
  len := 0;
  fifoemptymsk := 0;

  ep := @hpcd.IN_ep[epnum];
  len := ep^.xfer_len - ep^.xfer_count;

  if (len > ep^.maxpacket) then
    len := ep^.maxpacket;

  len32b := (len + 3) div 4;

  while ((USBx_INEP(usbx^,epnum)^.DTXFSTS and USB_OTG_DTXFSTS_INEPTFSAV) > len32b) and (ep^.xfer_count < ep^.xfer_len) and (ep^.xfer_len <> 0) do
  begin
    (* Write the FIFO *)
    len := ep^.xfer_len - ep^.xfer_count;

    if (len > ep^.maxpacket) then
      len := ep^.maxpacket;

    len32b := (len + 3) div 4;

    USB_WritePacket(USBx^, ep^.xfer_buff, epnum, len, hpcd.Init.dma_enable);

    ep^.xfer_buff := ep^.xfer_buff + (len);
    ep^.xfer_count := ep^.xfer_count + (len);
  end;

  if (len <= 0) then
  begin
    fifoemptymsk := $1 shl epnum;
    USBx_DEVICE(USBx^)^.DIEPEMPMSK := USBx_DEVICE(USBx^)^.DIEPEMPMSK and (not fifoemptymsk);
  end;

  exit(HAL_OK);
end;

function HAL_PCD_Init(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
var
  i: integer;
begin
  hpcd.State := HAL_PCD_STATE_BUSY;

  (* Init the low level hardware : GPIO, CLOCK, NVIC... *)
  HAL_PCD_MspInit(hpcd);

  (* Disable the Interrupts *)
  __HAL_PCD_DISABLE(hpcd);

  (*Init the Core (common init.) *)
  USB_CoreInit(hpcd.Instance^, hpcd.Init);

  (* Force Device Mode*)
  USB_SetCurrentMode(hpcd.Instance^, USB_OTG_DEVICE_MODE);

  (* Init endpoints structures *)
  for i := 0 to 15 - 1 do
  begin
    (* Init ep structure *)
    hpcd.IN_ep[i].is_in := True;
    hpcd.IN_ep[i].num := i;
    hpcd.IN_ep[i].tx_fifo_num := i;
    (* Control until ep is activated *)
    hpcd.IN_ep[i].type_ := EP_TYPE_CTRL;
    hpcd.IN_ep[i].maxpacket := 0;
    hpcd.IN_ep[i].xfer_buff := nil;
    hpcd.IN_ep[i].xfer_len := 0;
  end;

  for i := 0 to 15 - 1 do
  begin
    hpcd.OUT_ep[i].is_in := False;
    hpcd.OUT_ep[i].num := i;
    hpcd.IN_ep[i].tx_fifo_num := i;
    (* Control until ep is activated *)
    hpcd.OUT_ep[i].type_ := EP_TYPE_CTRL;
    hpcd.OUT_ep[i].maxpacket := 0;
    hpcd.OUT_ep[i].xfer_buff := nil;
    hpcd.OUT_ep[i].xfer_len := 0;

    hpcd.Instance^.DIEPTXF[i] := 0;
  end;

  (* Init Device *)
  USB_DevInit(hpcd.Instance^, hpcd.Init);

  hpcd.State := HAL_PCD_STATE_READY;

  (* Activate LPM *)
  if hpcd.Init.lpm_enable then
  begin
    HAL_PCDEx_ActivateLPM(hpcd);
  end;

  USB_DevDisconnect(hpcd.Instance^);
  exit(HAL_OK);
end;

function HAL_PCD_DeInit(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
begin
  hpcd.State := HAL_PCD_STATE_BUSY;

  (* Stop Device *)
  HAL_PCD_Stop(hpcd);

  (* DeInit the low level hardware *)
  HAL_PCD_MspDeInit(hpcd);

  hpcd.State := HAL_PCD_STATE_RESET;

  exit(HAL_OK);
end;

procedure HAL_PCD_MspInit_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_MspInit';
asm
  .weak HAL_PCD_MspInit
end;

procedure HAL_PCD_MspDeInit_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_MspDeInit';
asm
  .weak HAL_PCD_MspDeInit
end;

function HAL_PCD_Start(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
begin
  __HAL_Lock(hpcd.lock);
  USB_DevConnect(hpcd.Instance^);
  __HAL_PCD_ENABLE(hpcd);
  __HAL_Unlock(hpcd.lock);
  exit(HAL_OK);
end;

function HAL_PCD_Stop(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
begin
  __HAL_Lock(hpcd.lock);
  __HAL_PCD_DISABLE(hpcd);
  USB_StopDevice(hpcd.Instance^);
  USB_DevDisconnect(hpcd.Instance^);
  __HAL_Unlock(hpcd.lock);
  exit(HAL_OK);
end;

procedure HAL_PCD_IRQHandler(var hpcd: PCD_HandleTypeDef);
var
  USBx: PPCD_TypeDef;
  ep: PPCD_EPTypeDef;
  i, ep_intr, epint, epnum, fifoemptymsk, temp: longword;
begin
  USBx := hpcd.Instance;

  (* ensure that we are in device mode *)
  if (USB_GetMode(hpcd.Instance^) = USB_OTG_MODE_DEVICE) then
  begin
    (* avoid spurious interrupt *)
    if (__HAL_PCD_IS_INVALID_INTERRUPT(hpcd)) then
    begin
      exit;
    end;

    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_MMIS)) then
    begin
      (* incorrect mode, acknowledge the interrupt *)
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_MMIS);
    end;

    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_OEPINT)) then
    begin
      epnum := 0;

      (* Read in the device interrupt bits *)
      ep_intr := USB_ReadDevAllOutEpInterrupt(hpcd.Instance^);

      while (ep_intr) <> 0 do
      begin
        if (ep_intr and $1) <> 0 then
        begin
          epint := USB_ReadDevOutEPInterrupt(hpcd.Instance^, epnum);

          if ((epint and USB_OTG_DOEPINT_XFRC) = USB_OTG_DOEPINT_XFRC) then
          begin
            CLEAR_OUT_EP_INTR(usbx^, epnum, USB_OTG_DOEPINT_XFRC);

            if hpcd.Init.dma_enable then
            begin
              hpcd.OUT_ep[epnum].xfer_count := hpcd.OUT_ep[epnum].maxpacket - (USBx_OUTEP(usbx^, epnum)^.DOEPTSIZ and USB_OTG_DOEPTSIZ_XFRSIZ);
              hpcd.OUT_ep[epnum].xfer_buff := hpcd.OUT_ep[epnum].xfer_buff + hpcd.OUT_ep[epnum].maxpacket;
            end;

            HAL_PCD_DataOutStageCallback(hpcd, epnum);
            if (hpcd.Init.dma_enable) then
            begin
              if ((epnum = 0) and (hpcd.OUT_ep[epnum].xfer_len = 0)) then
              begin
                (* this is ZLP, so prepare EP0 for next setup *)
                USB_EP0_OutStart(hpcd.Instance^, True, @hpcd.Setup[0]);
              end;
            end;
          end;

          if ((epint and USB_OTG_DOEPINT_STUP) = USB_OTG_DOEPINT_STUP) then
          begin
            (* Inform the upper layer that a setup packet is available *)
            HAL_PCD_SetupStageCallback(hpcd);
            CLEAR_OUT_EP_INTR(usbx^, epnum, USB_OTG_DOEPINT_STUP);
          end;

          if ((epint and USB_OTG_DOEPINT_OTEPDIS) = USB_OTG_DOEPINT_OTEPDIS) then
          begin
            CLEAR_OUT_EP_INTR(usbx^, epnum, USB_OTG_DOEPINT_OTEPDIS);
          end;
        end;
        Inc(epnum);
        ep_intr := ep_intr shr 1;
      end;
    end;

    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_IEPINT)) then
    begin
      (* Read in the device interrupt bits *)
      ep_intr := USB_ReadDevAllInEpInterrupt(hpcd.Instance^);

      epnum := 0;

      while (ep_intr <> 0) do
      begin
        if (ep_intr and $1) <> 0 (* In ITR *) then
        begin
          epint := USB_ReadDevInEPInterrupt(hpcd.Instance^, epnum);

          if ((epint and USB_OTG_DIEPINT_XFRC) = USB_OTG_DIEPINT_XFRC) then
          begin
            fifoemptymsk := $1 shl epnum;
            USBx_DEVICE(usbx^)^.DIEPEMPMSK := USBx_DEVICE(usbx^)^.DIEPEMPMSK and (not fifoemptymsk);

            CLEAR_IN_EP_INTR(usbx^, epnum, USB_OTG_DIEPINT_XFRC);

            if (hpcd.Init.dma_enable) then
            begin
              Inc(hpcd.IN_ep[epnum].xfer_buff, hpcd.IN_ep[epnum].maxpacket);
            end;

            HAL_PCD_DataInStageCallback(hpcd, epnum);

            if (hpcd.Init.dma_enable) then
            begin
              (* this is ZLP, so prepare EP0 for next setup *)
              if ((epnum = 0) and (hpcd.IN_ep[epnum].xfer_len = 0)) then
              begin
                (* prepare to rx more setup packets *)
                USB_EP0_OutStart(hpcd.Instance^, True, @hpcd.Setup[0]);
              end;
            end;
          end;
          if ((epint and USB_OTG_DIEPINT_TOC) = USB_OTG_DIEPINT_TOC) then
          begin
            CLEAR_IN_EP_INTR(usbx^, epnum, USB_OTG_DIEPINT_TOC);
          end;
          if ((epint and USB_OTG_DIEPINT_ITTXFE) = USB_OTG_DIEPINT_ITTXFE) then
          begin
            CLEAR_IN_EP_INTR(usbx^, epnum, USB_OTG_DIEPINT_ITTXFE);
          end;
          if ((epint and USB_OTG_DIEPINT_INEPNE) = USB_OTG_DIEPINT_INEPNE) then
          begin
            CLEAR_IN_EP_INTR(usbx^, epnum, USB_OTG_DIEPINT_INEPNE);
          end;
          if ((epint and USB_OTG_DIEPINT_EPDISD) = USB_OTG_DIEPINT_EPDISD) then
          begin
            CLEAR_IN_EP_INTR(usbx^, epnum, USB_OTG_DIEPINT_EPDISD);
          end;
          if ((epint and USB_OTG_DIEPINT_TXFE) = USB_OTG_DIEPINT_TXFE) then
          begin
            PCD_WriteEmptyTxFifo(hpcd, epnum);
          end;
        end;
        Inc(epnum);
        ep_intr := ep_intr shr 1;
      end;
    end;

    (* Handle Resume Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT)) then
    begin
      (* Clear the Remote Wake-up Signaling *)
      USBx_DEVICE(USBx^)^.DCTL := USBx_DEVICE(USBx^)^.DCTL and (not USB_OTG_DCTL_RWUSIG);

      if (hpcd.LPM_State = LPM_L1) then
      begin
        hpcd.LPM_State := LPM_L0;
        HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
      end
      else
      begin
        HAL_PCD_ResumeCallback(hpcd);
      end;
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT);
    end;

    (* Handle Suspend Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_USBSUSP)) then
    begin

      if ((USBx_DEVICE(USBx^)^.DSTS and USB_OTG_DSTS_SUSPSTS) = USB_OTG_DSTS_SUSPSTS) then
      begin

        HAL_PCD_SuspendCallback(hpcd);
      end;
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_USBSUSP);
    end;

    (* Handle LPM Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_LPMINT)) then
    begin
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_LPMINT);
      if (hpcd.LPM_State = LPM_L0) then
      begin
        hpcd.LPM_State := LPM_L1;
        hpcd.BESL := (hpcd.Instance^.GLPMCFG and USB_OTG_GLPMCFG_BESL) shr 2;
        HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L1_ACTIVE);
      end
      else
      begin
        HAL_PCD_SuspendCallback(hpcd);
      end;
    end;

    (* Handle Reset Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_USBRST)) then
    begin
      USBx_DEVICE(USBx^)^.DCTL := USBx_DEVICE(USBx^)^.DCTL and (not USB_OTG_DCTL_RWUSIG);
      USB_FlushTxFifo(hpcd.Instance^, 0);

      for i := 0 to hpcd.Init.dev_endpoints - 1 do
      begin
        USBx_INEP(usbx^, i)^.DIEPINT := $FF;
        USBx_OUTEP(usbx^, i)^.DOEPINT := $FF;
      end;
      USBx_DEVICE(USBx^)^.DAINT := $FFFFFFFF;
      USBx_DEVICE(USBx^)^.DAINTMSK := USBx_DEVICE(USBx^)^.DAINTMSK or $10001;

      if (hpcd.Init.use_dedicated_ep1) then
      begin
        USBx_DEVICE(USBx^)^.DOUTEP1MSK := USBx_DEVICE(USBx^)^.DOUTEP1MSK or (USB_OTG_DOEPMSK_STUPM or USB_OTG_DOEPMSK_XFRCM or USB_OTG_DOEPMSK_EPDM);
        USBx_DEVICE(USBx^)^.DINEP1MSK := USBx_DEVICE(USBx^)^.DINEP1MSK or (USB_OTG_DIEPMSK_TOM or USB_OTG_DIEPMSK_XFRCM or USB_OTG_DIEPMSK_EPDM);
      end
      else
      begin
        USBx_DEVICE(USBx^)^.DOEPMSK := USBx_DEVICE(USBx^)^.DOEPMSK or (USB_OTG_DOEPMSK_STUPM or USB_OTG_DOEPMSK_XFRCM or USB_OTG_DOEPMSK_EPDM);
        USBx_DEVICE(USBx^)^.DIEPMSK := USBx_DEVICE(USBx^)^.DIEPMSK or (USB_OTG_DIEPMSK_TOM or USB_OTG_DIEPMSK_XFRCM or USB_OTG_DIEPMSK_EPDM);
      end;

      (* Set Default Address to 0 *)
      USBx_DEVICE(USBx^)^.DCFG := USBx_DEVICE(USBx^)^.DCFG and (not USB_OTG_DCFG_DAD);

      (* setup EP0 to receive SETUP packets *)
      USB_EP0_OutStart(hpcd.Instance^, hpcd.Init.dma_enable, @hpcd.Setup[0]);

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_USBRST);
    end;

    (* Handle Enumeration done Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE)) then
    begin
      USB_ActivateSetup(hpcd.Instance^);
      hpcd.Instance^.GUSBCFG := hpcd.Instance^.GUSBCFG and (not USB_OTG_GUSBCFG_TRDT);

      if (USB_GetDevSpeed(hpcd.Instance^) = USB_OTG_SPEED_HIGH) then
      begin
        hpcd.Init.speed := USB_OTG_SPEED_HIGH;
        hpcd.Init.ep0_mps := USB_OTG_HS_MAX_PACKET_SIZE;
        hpcd.Instance^.GUSBCFG := hpcd.Instance^.GUSBCFG or ((USBD_HS_TRDT_VALUE shl 10) and USB_OTG_GUSBCFG_TRDT);
      end
      else
      begin
        hpcd.Init.speed := USB_OTG_SPEED_FULL;
        hpcd.Init.ep0_mps := USB_OTG_FS_MAX_PACKET_SIZE;
        hpcd.Instance^.GUSBCFG := hpcd.Instance^.GUSBCFG or ((USBD_FS_TRDT_VALUE shl 10) and USB_OTG_GUSBCFG_TRDT);
      end;

      HAL_PCD_ResetCallback(hpcd);

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE);
    end;

    (* Handle RxQLevel Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_RXFLVL)) then
    begin
      USB_MASK_INTERRUPT(hpcd.Instance^, USB_OTG_GINTSTS_RXFLVL);
      temp := USBx^.GRXSTSP;
      ep := @hpcd.OUT_ep[temp and USB_OTG_GRXSTSP_EPNUM];

      if (((temp and USB_OTG_GRXSTSP_PKTSTS) shr 17) = STS_DATA_UPDT) then
      begin
        if ((temp and USB_OTG_GRXSTSP_BCNT) <> 0) then
        begin
          USB_ReadPacket(USBx^, ep^.xfer_buff, (temp and USB_OTG_GRXSTSP_BCNT) shr 4);
          ep^.xfer_buff := ep^.xfer_buff + ((temp and USB_OTG_GRXSTSP_BCNT) shr 4);
          ep^.xfer_count := ep^.xfer_count + ((temp and USB_OTG_GRXSTSP_BCNT) shr 4);
        end;
      end
      else if (((temp and USB_OTG_GRXSTSP_PKTSTS) shr 17) = STS_SETUP_UPDT) then
      begin
        USB_ReadPacket(USBx^, hpcd.Setup[0], 8);
        ep^.xfer_count := ep^.xfer_count + ((temp and USB_OTG_GRXSTSP_BCNT) shr 4);
      end;
      USB_UNMASK_INTERRUPT(hpcd.Instance^, USB_OTG_GINTSTS_RXFLVL);
    end;

    (* Handle SOF Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_SOF)) then
    begin
      HAL_PCD_SOFCallback(hpcd);
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_SOF);
    end;

    (* Handle Incomplete ISO IN Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR)) then
    begin
      HAL_PCD_ISOINIncompleteCallback(hpcd, epnum);
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR);
    end;

    (* Handle Incomplete ISO OUT Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)) then
    begin
      HAL_PCD_ISOOUTIncompleteCallback(hpcd, epnum);
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
    end;

    (* Handle Connection event Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT)) then
    begin
      HAL_PCD_ConnectCallback(hpcd);
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT);
    end;

    (* Handle Disconnection event Interrupt *)
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_OTGINT)) then
    begin
      temp := hpcd.Instance^.GOTGINT;

      if ((temp and USB_OTG_GOTGINT_SEDET) = USB_OTG_GOTGINT_SEDET) then
      begin
        HAL_PCD_DisconnectCallback(hpcd);
      end;
      hpcd.Instance^.GOTGINT := hpcd.Instance^.GOTGINT or temp;
    end;
  end;
end;

procedure HAL_PCD_DataOutStageCallback_stub(var hpcd: PCD_HandleTypeDef; epnum: byte); assembler; public name 'HAL_PCD_DataOutStageCallback';
asm
  .weak HAL_PCD_DataOutStageCallback
end;

procedure HAL_PCD_DataInStageCallback_stub(var hpcd: PCD_HandleTypeDef; epnum: byte); assembler; public name 'HAL_PCD_DataInStageCallback';
asm
  .weak HAL_PCD_DataInStageCallback
end;

procedure HAL_PCD_SetupStageCallback_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_SetupStageCallback';
asm
  .weak HAL_PCD_SetupStageCallback
end;

procedure HAL_PCD_SOFCallback_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_SOFCallback';
asm
  .weak HAL_PCD_SOFCallback
end;

procedure HAL_PCD_ResetCallback_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_ResetCallback';
asm
  .weak HAL_PCD_ResetCallback
end;

procedure HAL_PCD_SuspendCallback_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_SuspendCallback';
asm
  .weak HAL_PCD_SuspendCallback
end;

procedure HAL_PCD_ResumeCallback_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_ResumeCallback';
asm
  .weak HAL_PCD_ResumeCallback
end;

procedure HAL_PCD_ISOOUTIncompleteCallback_stub(var hpcd: PCD_HandleTypeDef; epnum: byte); assembler; public name 'HAL_PCD_ISOOUTIncompleteCallback';
asm
  .weak HAL_PCD_ISOOUTIncompleteCallback
end;

procedure HAL_PCD_ISOINIncompleteCallback_stub(var hpcd: PCD_HandleTypeDef; epnum: byte); assembler; public name 'HAL_PCD_ISOINIncompleteCallback';
asm
  .weak HAL_PCD_ISOINIncompleteCallback
end;

procedure HAL_PCD_ConnectCallback_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_ConnectCallback';
asm
  .weak HAL_PCD_ConnectCallback
end;

procedure HAL_PCD_DisconnectCallback_stub(var hpcd: PCD_HandleTypeDef); assembler; public name 'HAL_PCD_DisconnectCallback';
asm
  .weak HAL_PCD_DisconnectCallback
end;

function HAL_PCD_DevConnect(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
begin
  __HAL_Lock(hpcd.lock);
  USB_DevConnect(hpcd.Instance^);
  __HAL_Unlock(hpcd.lock);
  exit(HAL_OK);
end;

function HAL_PCD_DevDisconnect(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
begin
  __HAL_Lock(hpcd.lock);
  USB_DevDisconnect(hpcd.Instance^);
  __HAL_Unlock(hpcd.lock);
  exit(HAL_OK);
end;

function HAL_PCD_SetAddress(var hpcd: PCD_HandleTypeDef; address: byte): HAL_StatusTypeDef;
begin
  __HAL_Lock(hpcd.lock);
  USB_SetDevAddress(hpcd.Instance^, address);
  __HAL_Unlock(hpcd.lock);
  exit(HAL_OK);
end;

function HAL_PCD_EP_Open(var hpcd: PCD_HandleTypeDef; ep_addr: byte; ep_mps: word; ep_type: byte): HAL_StatusTypeDef;
var
  ret: HAL_StatusTypeDef;
  ep: PPCD_EPTypeDef;
begin
  ret := HAL_OK;

  if ((ep_addr and $80) = $80) then
    ep := @hpcd.IN_ep[ep_addr and $7F]
  else
    ep := @hpcd.OUT_ep[ep_addr and $7F];

  ep^.num := ep_addr and $7F;

  ep^.is_in := ($80 and ep_addr) <> 0;
  ep^.maxpacket := ep_mps;
  ep^.type_ := ep_type;

  if (ep^.is_in) then
    (* Assign a Tx FIFO *)
    ep^.tx_fifo_num := ep^.num;

  (* Set initial data PID. *)
  if (ep_type = EP_TYPE_BULK) then
    ep^.data_pid_start := 0;

  __HAL_Lock(hpcd.lock);
  USB_ActivateEndpoint(hpcd.Instance^, ep^);
  __HAL_Unlock(hpcd.lock);
  exit(ret);
end;

function HAL_PCD_EP_Close(var hpcd: PCD_HandleTypeDef; ep_addr: byte): HAL_StatusTypeDef;
var
  ep: PPCD_EPTypeDef;
begin
  if ((ep_addr and $80) = $80) then
    ep := @hpcd.IN_ep[ep_addr and $7F]
  else
    ep := @hpcd.OUT_ep[ep_addr and $7F];

  ep^.num := ep_addr and $7F;

  ep^.is_in := ($80 and ep_addr) <> 0;

  __HAL_Lock(hpcd.lock);
  USB_DeactivateEndpoint(hpcd.Instance^, ep^);
  __HAL_Unlock(hpcd.lock);
  exit(HAL_OK);
end;

function HAL_PCD_EP_Receive(var hpcd: PCD_HandleTypeDef; ep_addr: byte; var pBuf; len: longword): HAL_StatusTypeDef;
var
  ep: PPCD_EPTypeDef;
begin
  ep := @hpcd.OUT_ep[ep_addr and $7F];

  (*setup and start the Xfer *)
  ep^.xfer_buff := @pBuf;
  ep^.xfer_len := len;
  ep^.xfer_count := 0;
  ep^.is_in := False;
  ep^.num := ep_addr and $7F;

  if hpcd.Init.dma_enable then
    ep^.dma_addr := @pBuf;

  __HAL_Lock(hpcd.lock);

  if ((ep_addr and $7F) = 0) then
    USB_EP0StartXfer(hpcd.Instance^, ep^, hpcd.Init.dma_enable)
  else
    USB_EPStartXfer(hpcd.Instance^, ep^, hpcd.Init.dma_enable);

  __HAL_Unlock(hpcd.lock);

  exit(HAL_OK);
end;

function HAL_PCD_EP_Transmit(var hpcd: PCD_HandleTypeDef; ep_addr: byte; const pBuf; len: longword): HAL_StatusTypeDef;
var
  ep: PPCD_EPTypeDef;
begin
  ep := @hpcd.IN_ep[ep_addr and $7F];

  (*setup and start the Xfer *)
  ep^.xfer_buff := @pBuf;
  ep^.xfer_len := len;
  ep^.xfer_count := 0;
  ep^.is_in := True;
  ep^.num := ep_addr and $7F;

  if hpcd.Init.dma_enable then
    ep^.dma_addr := @pBuf;

  __HAL_Lock(hpcd.lock);

  if ((ep_addr and $7F) = 0) then
    USB_EP0StartXfer(hpcd.Instance^, ep^, hpcd.Init.dma_enable)
  else
    USB_EPStartXfer(hpcd.Instance^, ep^, hpcd.Init.dma_enable);

  __HAL_Unlock(hpcd.lock);

  exit(HAL_OK);
end;

function HAL_PCD_EP_GetRxCount(var hpcd: PCD_HandleTypeDef; ep_addr: byte): word;
begin
  exit(hpcd.OUT_ep[ep_addr and $7F].xfer_count);
end;

function HAL_PCD_EP_SetStall(var hpcd: PCD_HandleTypeDef; ep_addr: byte): HAL_StatusTypeDef;
var
  ep: PPCD_EPTypeDef;
begin
  if (($80 and ep_addr) = $80) then
    ep := @hpcd.IN_ep[ep_addr and $7F]
  else
    ep := @hpcd.OUT_ep[ep_addr];

  ep^.is_stall := true;
  ep^.num := ep_addr and $7F;
  ep^.is_in := ((ep_addr and $80) = $80);


  __HAL_Lock(hpcd.lock);
  USB_EPSetStall(hpcd.Instance^, ep^);
  if ((ep_addr and $7F) = 0) then
    USB_EP0_OutStart(hpcd.Instance^, hpcd.Init.dma_enable, @hpcd.Setup[0]);

  __HAL_Unlock(hpcd.lock);

  exit(HAL_OK);
end;

function HAL_PCD_EP_ClrStall(var hpcd: PCD_HandleTypeDef; ep_addr: byte): HAL_StatusTypeDef;
var
  ep: PPCD_EPTypeDef;
begin
  if (($80 and ep_addr) = $80) then
    ep := @hpcd.IN_ep[ep_addr and $7F]
  else
    ep := @hpcd.OUT_ep[ep_addr];

  ep^.is_stall := false;
  ep^.num := ep_addr and $7F;
  ep^.is_in := ((ep_addr and $80) = $80);

  __HAL_Lock(hpcd.lock);
  USB_EPClearStall(hpcd.Instance^, ep^);
  __HAL_Unlock(hpcd.lock);

  exit(HAL_OK);
end;

function HAL_PCD_EP_Flush(var hpcd: PCD_HandleTypeDef; ep_addr: byte): HAL_StatusTypeDef;
begin
  __HAL_Lock(hpcd.lock);

  if ((ep_addr and $80) = $80) then
    USB_FlushTxFifo(hpcd.Instance^, ep_addr and $7F)
  else
    USB_FlushRxFifo(hpcd.Instance^);

  __HAL_Unlock(hpcd.lock);

  exit(HAL_OK);
end;

function HAL_PCD_ActivateRemoteWakeup(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
var
  USBx: PPCD_TypeDef;
begin
  USBx := hpcd.Instance;

  if ((USBx_DEVICE(USBx^)^.DSTS and USB_OTG_DSTS_SUSPSTS) = USB_OTG_DSTS_SUSPSTS) then
  begin
    (* Activate Remote wake-up signaling *)
    USBx_DEVICE(USBx^)^.DCTL := USBx_DEVICE(USBx^)^.DCTL or USB_OTG_DCTL_RWUSIG;
  end;

  exit(HAL_OK);
end;

function HAL_PCD_DeActivateRemoteWakeup(var hpcd: PCD_HandleTypeDef): HAL_StatusTypeDef;
var
  USBx: PPCD_TypeDef;
begin
  USBx := hpcd.Instance;

  if ((USBx_DEVICE(USBx^)^.DSTS and USB_OTG_DSTS_SUSPSTS) = USB_OTG_DSTS_SUSPSTS) then
  begin
    (* Activate Remote wake-up signaling *)
    USBx_DEVICE(USBx^)^.DCTL := USBx_DEVICE(USBx^)^.DCTL and (not longword(USB_OTG_DCTL_RWUSIG));
  end;

  exit(HAL_OK);
end;

function HAL_PCD_GetState(var hpcd: PCD_HandleTypeDef): PCD_StateTypeDef;
begin
  exit(hpcd.State);
end;

procedure __HAL_PCD_ENABLE(var __HANDLE__: PCD_HandleTypeDef);
begin
  USB_EnableGlobalInt((__HANDLE__).Instance^);
end;

procedure __HAL_PCD_DISABLE(var __HANDLE__: PCD_HandleTypeDef);
begin
  USB_DisableGlobalInt((__HANDLE__).Instance^);
end;

function __HAL_PCD_GET_FLAG(var __HANDLE__: PCD_HandleTypeDef; __INTERRUPT__: longword): boolean;
begin
  exit(((USB_ReadInterrupts((__HANDLE__).Instance^) and (__INTERRUPT__)) = (__INTERRUPT__)));
end;

procedure __HAL_PCD_CLEAR_FLAG(var __HANDLE__: PCD_HandleTypeDef; __INTERRUPT__: longword);
begin
  ((__HANDLE__).Instance^.GINTSTS) := (__INTERRUPT__);
end;

function __HAL_PCD_IS_INVALID_INTERRUPT(var __HANDLE__: PCD_HandleTypeDef): boolean;
begin
  exit((USB_ReadInterrupts((__HANDLE__).Instance^) = 0));
end;

procedure __HAL_PCD_UNGATE_PHYCLOCK(var __HANDLE__: PCD_HandleTypeDef);
begin
  plongword(@pbyte(__HANDLE__.Instance)[USB_OTG_PCGCCTL_BASE])^ := plongword(@pbyte(__HANDLE__.Instance)[USB_OTG_PCGCCTL_BASE])^ and (not longword(USB_OTG_PCGCCTL_STOPCLK));
end;

procedure __HAL_PCD_GATE_PHYCLOCK(var __HANDLE__: PCD_HandleTypeDef);
begin
  plongword(@pbyte(__HANDLE__.Instance)[USB_OTG_PCGCCTL_BASE])^ := plongword(@pbyte(__HANDLE__.Instance)[USB_OTG_PCGCCTL_BASE])^ or USB_OTG_PCGCCTL_STOPCLK;
end;

function __HAL_PCD_IS_PHY_SUSPENDED(var __HANDLE__: PCD_HandleTypeDef): boolean;
begin
  exit((plongword(@pbyte(__HANDLE__.Instance)[USB_OTG_PCGCCTL_BASE])^ and $10) <> 0);
end;

procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_IT;
begin
  EXTI.IMR := EXTI.IMR or (USB_OTG_HS_WAKEUP_EXTI_LINE);
end;

procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_DISABLE_IT;
begin
  EXTI.IMR := EXTI.IMR and (not (USB_OTG_HS_WAKEUP_EXTI_LINE));
end;

function __HAL_USB_OTG_HS_WAKEUP_EXTI_GET_FLAG: boolean;
begin
  exit((EXTI.PR and (USB_OTG_HS_WAKEUP_EXTI_LINE)) <> 0);
end;

procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_CLEAR_FLAG;
begin
  EXTI.PR := (USB_OTG_HS_WAKEUP_EXTI_LINE);
end;

procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_RISING_EDGE;
begin
  EXTI.FTSR := EXTI.FTSR and (not (USB_OTG_HS_WAKEUP_EXTI_LINE));
  EXTI.RTSR := EXTI.RTSR or USB_OTG_HS_WAKEUP_EXTI_LINE;
end;

procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_FALLING_EDGE;
begin
  EXTI.FTSR := EXTI.FTSR or (USB_OTG_HS_WAKEUP_EXTI_LINE);
  EXTI.RTSR := EXTI.RTSR and (not (USB_OTG_HS_WAKEUP_EXTI_LINE));
end;

procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_RISING_FALLING_EDGE;
begin
  EXTI.RTSR := EXTI.RTSR and (not (USB_OTG_HS_WAKEUP_EXTI_LINE));
  EXTI.FTSR := EXTI.FTSR and (not (USB_OTG_HS_WAKEUP_EXTI_LINE));
  EXTI.RTSR := EXTI.RTSR or USB_OTG_HS_WAKEUP_EXTI_LINE;
  EXTI.FTSR := EXTI.FTSR or USB_OTG_HS_WAKEUP_EXTI_LINE;
end;

procedure __HAL_USB_OTG_HS_WAKEUP_EXTI_GENERATE_SWIT;
begin
  EXTI.SWIER := EXTI.SWIER or USB_OTG_FS_WAKEUP_EXTI_LINE;
end;

procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_IT;
begin
  EXTI.IMR := EXTI.IMR or USB_OTG_FS_WAKEUP_EXTI_LINE;
end;

procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_DISABLE_IT;
begin
  EXTI.IMR := EXTI.IMR and (not (USB_OTG_FS_WAKEUP_EXTI_LINE));
end;

function __HAL_USB_OTG_FS_WAKEUP_EXTI_GET_FLAG: boolean;
begin
  exit((EXTI.PR and (USB_OTG_FS_WAKEUP_EXTI_LINE)) <> 0);
end;

procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG;
begin
  EXTI.PR := USB_OTG_FS_WAKEUP_EXTI_LINE;
end;

procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_RISING_EDGE;
begin
  EXTI.FTSR := EXTI.FTSR and (not (USB_OTG_FS_WAKEUP_EXTI_LINE));
  EXTI.RTSR := EXTI.RTSR or USB_OTG_FS_WAKEUP_EXTI_LINE;
end;


procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_FALLING_EDGE;
begin
  EXTI.FTSR := EXTI.FTSR or (USB_OTG_FS_WAKEUP_EXTI_LINE);
  EXTI.RTSR := EXTI.RTSR and (not (USB_OTG_FS_WAKEUP_EXTI_LINE));
end;

procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_RISING_FALLING_EDGE;
begin
  EXTI.RTSR := EXTI.RTSR and (not (USB_OTG_FS_WAKEUP_EXTI_LINE));
  EXTI.FTSR := EXTI.FTSR and (not (USB_OTG_FS_WAKEUP_EXTI_LINE));
  EXTI.RTSR := EXTI.RTSR or USB_OTG_FS_WAKEUP_EXTI_LINE;
  EXTI.FTSR := EXTI.FTSR or USB_OTG_FS_WAKEUP_EXTI_LINE;
end;

procedure __HAL_USB_OTG_FS_WAKEUP_EXTI_GENERATE_SWIT;
begin
  EXTI.SWIER := EXTI.SWIER or USB_OTG_FS_WAKEUP_EXTI_LINE;
end;

end.
