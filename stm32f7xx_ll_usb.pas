(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_ll_usb.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of USB Core HAL module.
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
unit stm32f7xx_ll_usb;

interface

uses
  stm32f7xx_defs,
  stm32f7xx_hal;

{$packrecords C}

(**
  * @brief  USB Mode definition
   *)

const
  USB_OTG_DEVICE_MODE = 0;
  USB_OTG_HOST_MODE = 1;
  USB_OTG_DRD_MODE = 2;

type
  USB_OTG_ModeTypeDef = integer;

  (**
  * @brief  URB States definition
   *)

const
  URB_IDLE = 0;
  URB_DONE = 1;
  URB_NOTREADY = 2;
  URB_NYET = 3;
  URB_ERROR = 4;
  URB_STALL = 5;

type
  USB_OTG_URBStateTypeDef = integer;

  (**
  * @brief  Host channel States  definition
   *)

const
  HC_IDLE = 0;
  HC_XFRC = 1;
  HC_HALTED = 2;
  HC_NAK = 3;
  HC_NYET = 4;
  HC_STALL = 5;
  HC_XACTERR = 6;
  HC_BBLERR = 7;
  HC_DATATGLERR = 8;

type
  USB_OTG_HCStateTypeDef = integer;

  (**
  * @brief  PCD Initialization Structure definition
   *)

  USB_OTG_CfgTypeDef = record
    dev_endpoints: longword;  (*!< Device Endpoints number.
                                      This parameter depends on the used USB core.
                                      This parameter must be a number between Min_Data = 1 and Max_Data = 15  *)
    Host_channels: longword;  (*!< Host Channels number.
                                      This parameter Depends on the used USB core.
                                      This parameter must be a number between Min_Data = 1 and Max_Data = 15  *)
    speed: longword;  (*!< USB Core speed.
                                      This parameter can be any value of @ref USB_Core_Speed_                 *)
    dma_enable: boolean;  (*!< Enable or disable of the USB embedded DMA.                              *)
    ep0_mps: longword;  (*!< Set the Endpoint 0 Max Packet size.
                                      This parameter can be any value of @ref USB_EP0_MPS_                    *)
    phy_itface: longword;  (*!< Select the used PHY interface.
                                      This parameter can be any value of @ref USB_Core_PHY_                   *)
    Sof_enable: boolean;  (*!< Enable or disable the output of the SOF signal.                         *)
    low_power_enable: boolean;  (*!< Enable or disable the low power mode.                                   *)
    lpm_enable: boolean;  (*!< Enable or disable Link Power Management.                                *)
    vbus_sensing_enable: boolean;  (*!< Enable or disable the VBUS Sensing feature.                             *)
    use_dedicated_ep1: boolean;  (*!< Enable or disable the use of the dedicated EP1 interrupt.               *)
    use_external_vbus: boolean;  (*!< Enable or disable the use of the external VBUS.                         *)
  end;

  USB_OTG_EPTypeDef = record
    num: byte;  (*!< Endpoint number
                                This parameter must be a number between Min_Data = 1 and Max_Data = 15     *)
    is_in: byte;  (*!< Endpoint direction
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1      *)
    is_stall: byte;  (*!< Endpoint stall condition
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1      *)
    type_: byte;  (*!< Endpoint type
                                 This parameter can be any value of @ref USB_EP_Type_                      *)
    data_pid_start: byte;  (*!< Initial data PID
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1      *)
    even_odd_frame: byte;  (*!< IFrame parity
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 1     *)
    tx_fifo_num: word;  (*!< Transmission FIFO number
                                 This parameter must be a number between Min_Data = 1 and Max_Data = 15    *)
    maxpacket: longword;  (*!< Endpoint Max packet size
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 64KB  *)
    xfer_buff: Pbyte;  (*!< Pointer to transfer buffer                                                *)
    dma_addr: longword;  (*!< 32 bits aligned transfer buffer address                                   *)
    xfer_len: longword;  (*!< Current transfer length                                                   *)
    xfer_count: longword;  (*!< Partial transfer length in case of multi packet transfer                  *)
  end;

  USB_OTG_HCTypeDef = record
    dev_addr: byte;  (*!< USB device address.
                                This parameter must be a number between Min_Data = 1 and Max_Data = 255     *)
    ch_num: byte;  (*!< Host channel number.
                                This parameter must be a number between Min_Data = 1 and Max_Data = 15      *)
    ep_num: byte;  (*!< Endpoint number.
                                This parameter must be a number between Min_Data = 1 and Max_Data = 15      *)
    ep_is_in: byte;  (*!< Endpoint direction
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1       *)
    speed: byte;  (*!< USB Host speed.
                                This parameter can be any value of @ref USB_Core_Speed_                     *)
    do_ping: byte;  (*!< Enable or disable the use of the PING protocol for HS mode.                 *)
    process_ping: byte;  (*!< Execute the PING protocol for HS mode.                                      *)
    ep_type: byte;  (*!< Endpoint Type.
                                This parameter can be any value of @ref USB_EP_Type_                        *)
    max_packet: word;  (*!< Endpoint Max packet size.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 64KB    *)
    data_pid: byte;  (*!< Initial data PID.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1       *)
    xfer_buff: Pbyte;  (*!< Pointer to transfer buffer.                                                 *)
    xfer_len: longword;  (*!< Current transfer length.                                                    *)
    xfer_count: longword;  (*!< Partial transfer length in case of multi packet transfer.                   *)
    toggle_in: byte;  (*!< IN transfer current toggle flag.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1       *)
    toggle_out: byte;  (*!< OUT transfer current toggle flag
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1       *)
    dma_addr: longword;  (*!< 32 bits aligned transfer buffer address.                                    *)
    ErrCnt: longword;  (*!< Host channel error count. *)
    urb_state: USB_OTG_URBStateTypeDef;  (*!< URB state.
                                           This parameter can be any value of @ref USB_OTG_URBStateTypeDef  *)
    state: USB_OTG_HCStateTypeDef;  (*!< Host Channel state.
                                           This parameter can be any value of @ref USB_OTG_HCStateTypeDef   *)
  end;

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
   *)

  (** @defgroup USB_Core_Mode_ USB Core Mode
  * @{
   *)

const
  USB_OTG_MODE_DEVICE = 0;
  USB_OTG_MODE_HOST = 1;
  USB_OTG_MODE_DRD = 2;
  (**
  * @}
   *)

  (** @defgroup USB_Core_Speed_   USB Core Speed
  * @{
   *)

  USB_OTG_SPEED_HIGH = 0;
  USB_OTG_SPEED_HIGH_IN_FULL = 1;
  USB_OTG_SPEED_LOW = 2;
  USB_OTG_SPEED_FULL = 3;
  (**
  * @}
   *)

  (** @defgroup USB_Core_PHY_   USB Core PHY
  * @{
   *)

  USB_OTG_ULPI_PHY = 1;
  USB_OTG_EMBEDDED_PHY = 2;
  (**
  * @}
   *)

  (** @defgroup USB_Core_MPS_   USB Core MPS
  * @{
   *)

  USB_OTG_HS_MAX_PACKET_SIZE = 512;
  USB_OTG_FS_MAX_PACKET_SIZE = 64;
  USB_OTG_MAX_EP0_SIZE = 64;
  (**
  * @}
   *)

  (** @defgroup USB_Core_Phy_Frequency_   USB Core Phy Frequency
  * @{
   *)

  DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ = (0 shl 1);
  DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ = (1 shl 1);
  DSTS_ENUMSPD_LS_PHY_6MHZ = (2 shl 1);
  DSTS_ENUMSPD_FS_PHY_48MHZ = (3 shl 1);
  (**
  * @}
   *)

  (** @defgroup USB_CORE_Frame_Interval_   USB CORE Frame Interval
  * @{
   *)

  DCFG_FRAME_INTERVAL_80 = 0;
  DCFG_FRAME_INTERVAL_85 = 1;
  DCFG_FRAME_INTERVAL_90 = 2;
  DCFG_FRAME_INTERVAL_95 = 3;
  (**
  * @}
   *)

  (** @defgroup USB_EP0_MPS_  USB EP0 MPS
  * @{
   *)

  DEP0CTL_MPS_64 = 0;
  DEP0CTL_MPS_32 = 1;
  DEP0CTL_MPS_16 = 2;
  DEP0CTL_MPS_8 = 3;
  (**
  * @}
   *)

  (** @defgroup USB_EP_Speed_  USB EP Speed
  * @{
   *)

  EP_SPEED_LOW = 0;
  EP_SPEED_FULL = 1;
  EP_SPEED_HIGH = 2;
  (**
  * @}
   *)

  (** @defgroup USB_EP_Type_  USB EP Type
  * @{
   *)

  EP_TYPE_CTRL = 0;
  EP_TYPE_ISOC = 1;
  EP_TYPE_BULK = 2;
  EP_TYPE_INTR = 3;
  EP_TYPE_MSK = 3;
  (**
  * @}
   *)

  (** @defgroup USB_STS_Defines_   USB STS Defines
  * @{
   *)

  STS_GOUT_NAK = 1;
  STS_DATA_UPDT = 2;
  STS_XFER_COMP = 3;
  STS_SETUP_COMP = 4;
  STS_SETUP_UPDT = 6;
  (**
  * @}
   *)

  (** @defgroup HCFG_SPEED_Defines_   HCFG SPEED Defines
  * @{
   *)

  HCFG_30_60_MHZ = 0;
  HCFG_48_MHZ = 1;
  HCFG_6_MHZ = 2;
  (**
  * @}
   *)

  (** @defgroup HPRT0_PRTSPD_SPEED_Defines_  HPRT0 PRTSPD SPEED Defines
  * @{
   *)

  HPRT0_PRTSPD_HIGH_SPEED = 0;
  HPRT0_PRTSPD_FULL_SPEED = 1;
  HPRT0_PRTSPD_LOW_SPEED = 2;
  (**
  * @}
   *)

  HCCHAR_CTRL = 0;
  HCCHAR_ISOC = 1;
  HCCHAR_BULK = 2;
  HCCHAR_INTR = 3;
  HC_PID_DATA0 = 0;
  HC_PID_DATA2 = 1;
  HC_PID_DATA1 = 2;
  HC_PID_SETUP = 3;
  GRXSTS_PKTSTS_IN = 2;
  GRXSTS_PKTSTS_IN_XFER_COMP = 3;
  GRXSTS_PKTSTS_DATA_TOGGLE_ERR = 5;
  GRXSTS_PKTSTS_CH_HALTED = 7;

(* Exported functions -------------------------------------------------------- *)
function USB_CoreInit(var USBx: USB_OTG_GlobalTypeDef; Init: USB_OTG_CfgTypeDef): HAL_StatusTypeDef;
function USB_DevInit(var USBx: USB_OTG_GlobalTypeDef; Init: USB_OTG_CfgTypeDef): HAL_StatusTypeDef;
function USB_EnableGlobalInt(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
function USB_DisableGlobalInt(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
function USB_SetCurrentMode(var USBx: USB_OTG_GlobalTypeDef; mode: USB_OTG_ModeTypeDef): HAL_StatusTypeDef;
function USB_SetDevSpeed(var USBx: USB_OTG_GlobalTypeDef; speed: byte): HAL_StatusTypeDef;
function USB_FlushRxFifo(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
function USB_FlushTxFifo(var USBx: USB_OTG_GlobalTypeDef; num: longword): HAL_StatusTypeDef;
function USB_ActivateEndpoint(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
function USB_DeactivateEndpoint(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
function USB_ActivateDedicatedEndpoint(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
function USB_DeactivateDedicatedEndpoint(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
function USB_EPStartXfer(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef; dma: boolean): HAL_StatusTypeDef;
function USB_EP0StartXfer(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef; dma: boolean): HAL_StatusTypeDef;
function USB_WritePacket(var USBx: USB_OTG_GlobalTypeDef; src: Pbyte; ch_ep_num: byte; len: word; dma: boolean): HAL_StatusTypeDef;
function USB_ReadPacket(var USBx: USB_OTG_GlobalTypeDef; dest: Pbyte; len: word): pointer;
function USB_EPSetStall(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
function USB_EPClearStall(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
function USB_SetDevAddress(var USBx: USB_OTG_GlobalTypeDef; address: byte): HAL_StatusTypeDef;
function USB_DevConnect(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
function USB_DevDisconnect(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
function USB_StopDevice(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
function USB_ActivateSetup(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
function USB_EP0_OutStart(var USBx: USB_OTG_GlobalTypeDef; dma: byte; psetup: Pbyte): HAL_StatusTypeDef;
function USB_GetDevSpeed(var USBx: USB_OTG_GlobalTypeDef): byte;
function USB_GetMode(var USBx: USB_OTG_GlobalTypeDef): longword;
function USB_ReadInterrupts(var USBx: USB_OTG_GlobalTypeDef): longword;
function USB_ReadDevAllOutEpInterrupt(var USBx: USB_OTG_GlobalTypeDef): longword;
function USB_ReadDevOutEPInterrupt(var USBx: USB_OTG_GlobalTypeDef; epnum: byte): longword;
function USB_ReadDevAllInEpInterrupt(var USBx: USB_OTG_GlobalTypeDef): longword;
function USB_ReadDevInEPInterrupt(var USBx: USB_OTG_GlobalTypeDef; epnum: byte): longword;
procedure USB_ClearInterrupts(var USBx: USB_OTG_GlobalTypeDef; interrupt: longword);
function USB_HostInit(var USBx: USB_OTG_GlobalTypeDef; cfg: USB_OTG_CfgTypeDef): HAL_StatusTypeDef;
function USB_InitFSLSPClkSel(var USBx: USB_OTG_GlobalTypeDef; freq: byte): HAL_StatusTypeDef;
function USB_ResetPort(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
function USB_DriveVbus(var USBx: USB_OTG_GlobalTypeDef; state: byte): HAL_StatusTypeDef;
function USB_GetHostSpeed(var USBx: USB_OTG_GlobalTypeDef): longword;
function USB_GetCurrentFrame(var USBx: USB_OTG_GlobalTypeDef): longword;
function USB_HC_Init(var USBx: USB_OTG_GlobalTypeDef; ch_num, epnum, dev_address, speed, ep_type: byte; mps: word): HAL_StatusTypeDef;
function USB_HC_StartXfer(var USBx: USB_OTG_GlobalTypeDef; var hc: USB_OTG_HCTypeDef; dma: boolean): HAL_StatusTypeDef;
function USB_HC_ReadInterrupt(var USBx: USB_OTG_GlobalTypeDef): longword;
function USB_HC_Halt(var USBx: USB_OTG_GlobalTypeDef; hc_num: byte): HAL_StatusTypeDef;
function USB_DoPing(var USBx: USB_OTG_GlobalTypeDef; ch_num: byte): HAL_StatusTypeDef;
function USB_StopHost(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;

implementation

function USBx_PCGCCTL(var USBx: USB_OTG_GlobalTypeDef): plongword; inline;
begin
  exit(plongword(@pbyte(@USBx)[USB_OTG_PCGCCTL_BASE]));
end;

function USBx_HPRT0(var USBx: USB_OTG_GlobalTypeDef): plongword; inline;
begin
  exit(plongword(@pbyte(@USBx)[USB_OTG_HOST_PORT_BASE]));
end;

function USBx_DEVICE(var USBx: USB_OTG_GlobalTypeDef): PUSB_OTG_DeviceTypeDef; inline;
begin
  exit(pUSB_OTG_DeviceTypeDef(@pbyte(@USBx)[USB_OTG_DEVICE_BASE]));
end;

function USBx_INEP(var USBx: USB_OTG_GlobalTypeDef; i: longword): PUSB_OTG_INEndpointTypeDef; inline;
begin
  exit(PUSB_OTG_INEndpointTypeDef(@pbyte(@USBx)[USB_OTG_IN_ENDPOINT_BASE + i * USB_OTG_EP_REG_SIZE]));
end;

function USBx_OUTEP(var USBx: USB_OTG_GlobalTypeDef; i: longword): PUSB_OTG_OUTEndpointTypeDef; inline;
begin
  exit(PUSB_OTG_OUTEndpointTypeDef(@pbyte(@USBx)[USB_OTG_OUT_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE]));
end;

function USBx_DFIFO(var USBx: USB_OTG_GlobalTypeDef; i: longword): plongword; inline;
begin
  exit(plongword(@pbyte(@USBx)[USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE]));
end;

function USBx_HOST(var USBx: USB_OTG_GlobalTypeDef): PUSB_OTG_HostTypeDef; inline;
begin
  exit(PUSB_OTG_HostTypeDef(@pbyte(@USBx)[USB_OTG_HOST_BASE]));
end;

function USBx_HC(var USBx: USB_OTG_GlobalTypeDef; i: longword): PUSB_OTG_HostChannelTypeDef; inline;
begin
  exit(PUSB_OTG_HostChannelTypeDef(@pbyte(@USBx)[USB_OTG_HOST_CHANNEL_BASE + (i) * USB_OTG_HOST_CHANNEL_SIZE]));
end;

procedure USB_MASK_INTERRUPT(var __INSTANCE__: USB_OTG_GlobalTypeDef; __INTERRUPT__: longword);
begin
  (__INSTANCE__).GINTMSK := __INSTANCE__.GINTMSK and (not (__INTERRUPT__));
end;

procedure USB_UNMASK_INTERRUPT(var __INSTANCE__: USB_OTG_GlobalTypeDef; __INTERRUPT__: longword);
begin
  (__INSTANCE__).GINTMSK := __INSTANCE__.GINTMSK or (__INTERRUPT__);
end;

procedure CLEAR_IN_EP_INTR(var __INSTANCE__: USB_OTG_GlobalTypeDef; __EPNUM__, __INTERRUPT__: longword);
begin
  USBx_INEP(__INSTANCE__, __EPNUM__)^.DIEPINT := (__INTERRUPT__);
end;

procedure CLEAR_OUT_EP_INTR(var __INSTANCE__: USB_OTG_GlobalTypeDef; __EPNUM__, __INTERRUPT__: longword);
begin
  USBx_OUTEP(__INSTANCE__, __EPNUM__)^.DOEPINT := (__INTERRUPT__);
end;

function USB_CoreReset(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
var
  Count: longword;
begin
  Count := 0;

  (* Wait for AHB master IDLE state. *)
  repeat
    Inc(Count);
    if Count > 200000 then
      exit(HAL_TIMEOUT);
  until ((USBx.GRSTCTL and USB_OTG_GRSTCTL_AHBIDL) <> 0);

  (* Core Soft Reset *)
  Count := 0;
  USBx.GRSTCTL := USBx.GRSTCTL or USB_OTG_GRSTCTL_CSRST;

  repeat
    Inc(Count);
    if Count > 200000 then
      exit(HAL_TIMEOUT);
  until ((USBx.GRSTCTL and USB_OTG_GRSTCTL_CSRST) <> USB_OTG_GRSTCTL_CSRST);

  exit(HAL_OK);
end;

function USB_CoreInit(var USBx: USB_OTG_GlobalTypeDef; Init: USB_OTG_CfgTypeDef): HAL_StatusTypeDef;
begin
  if (Init.phy_itface = USB_OTG_ULPI_PHY) then
  begin

    USBx.GCCFG := USBx.GCCFG and (not (USB_OTG_GCCFG_PWRDWN));

    (* Init The ULPI Interface *)
    USBx.GUSBCFG := USBx.GUSBCFG and (not (USB_OTG_GUSBCFG_TSDPS or USB_OTG_GUSBCFG_ULPIFSLS or USB_OTG_GUSBCFG_PHYSEL));

    (* Select vbus source *)
    USBx.GUSBCFG := USBx.GUSBCFG and (not (USB_OTG_GUSBCFG_ULPIEVBUSD or USB_OTG_GUSBCFG_ULPIEVBUSI));
    if (Init.use_external_vbus) then
      USBx.GUSBCFG := USBx.GUSBCFG or USB_OTG_GUSBCFG_ULPIEVBUSD;

    (* Reset after a PHY select  *)
    USB_CoreReset(USBx);
  end
  else (* FS interface (embedded Phy) *)
  begin

    (* Select FS Embedded PHY *)
    USBx.GUSBCFG := USBx.GUSBCFG or USB_OTG_GUSBCFG_PHYSEL;

    (* Reset after a PHY select and set Host mode *)
    USB_CoreReset(USBx);

    (* Deactivate the power down*)
    USBx.GCCFG := USB_OTG_GCCFG_PWRDWN;
  end;

  if (Init.dma_enable ) then
  begin
    USBx.GAHBCFG := USBx.GAHBCFG or (USB_OTG_GAHBCFG_HBSTLEN_1 or USB_OTG_GAHBCFG_HBSTLEN_2);
    USBx.GAHBCFG := USBx.GAHBCFG or USB_OTG_GAHBCFG_DMAEN;
  end;

  exit(HAL_OK);
end;

function USB_DevInit(var USBx: USB_OTG_GlobalTypeDef; Init: USB_OTG_CfgTypeDef): HAL_StatusTypeDef;
var
  i: Integer;
begin
  (*Activate VBUS Sensing B *)
  USBx.GCCFG := USBx.GCCFG or USB_OTG_GCCFG_VBDEN;

  if (not Init.vbus_sensing_enable) then
  begin
    (*Desactivate VBUS Sensing B *)
    USBx.GCCFG := USBx.GCCFG and (not USB_OTG_GCCFG_VBDEN);

    (* B-peripheral session valid override enable*)
    USBx.GOTGCTL := USBx.GOTGCTL or USB_OTG_GOTGCTL_BVALOEN;
    USBx.GOTGCTL := USBx.GOTGCTL or USB_OTG_GOTGCTL_BVALOVAL;
  end;

  (* Restart the Phy Clock *)
  USBx_PCGCCTL(USBx)^ := 0;

  (* Device mode configuration *)
  USBx_DEVICE(USBx)^.DCFG := USBx_DEVICE(USBx)^.DCFG or DCFG_FRAME_INTERVAL_80;

  if (Init.phy_itface = USB_OTG_ULPI_PHY) then
  begin
    if (Init.speed = USB_OTG_SPEED_HIGH) then
      (* Set High speed phy *)
      USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH)
    else
      (* set High speed phy in Full speed mode *)
      USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH_IN_FULL);
  end
  else
    (* Set Full speed phy *)
    USB_SetDevSpeed(USBx, USB_OTG_SPEED_FULL);

  (* Flush the FIFOs *)
  USB_FlushTxFifo(USBx, $10); (* all Tx FIFOs *)
  USB_FlushRxFifo(USBx);


  (* Clear all pending Device Interrupts *)
  USBx_DEVICE(USBx)^.DIEPMSK := 0;
  USBx_DEVICE(USBx)^.DOEPMSK := 0;
  USBx_DEVICE(USBx)^.DAINT := $FFFFFFFF;
  USBx_DEVICE(USBx)^.DAINTMSK := 0;

  for i := 0 to Init.dev_endpoints - 1 do
  begin
    if ((USBx_INEP(usbx,i)^.DIEPCTL and USB_OTG_DIEPCTL_EPENA) = USB_OTG_DIEPCTL_EPENA) then
      USBx_INEP(usbx,i)^.DIEPCTL := (USB_OTG_DIEPCTL_EPDIS or USB_OTG_DIEPCTL_SNAK)
    else
      USBx_INEP(usbx,i)^.DIEPCTL := 0;

    USBx_INEP(usbx,i)^.DIEPTSIZ := 0;
    USBx_INEP(usbx,i)^.DIEPINT := $FF;
  end;

  for i := 0 to Init.dev_endpoints - 1 do
  begin
    if ((USBx_OUTEP(usbx,i)^.DOEPCTL and USB_OTG_DOEPCTL_EPENA) = USB_OTG_DOEPCTL_EPENA) then
      USBx_OUTEP(usbx,i)^.DOEPCTL := (USB_OTG_DOEPCTL_EPDIS or USB_OTG_DOEPCTL_SNAK)
    else
      USBx_OUTEP(usbx,i)^.DOEPCTL := 0;

    USBx_OUTEP(usbx,i)^.DOEPTSIZ := 0;
    USBx_OUTEP(usbx,i)^.DOEPINT := $FF;
  end;

  USBx_DEVICE(USBx)^.DIEPMSK := USBx_DEVICE(USBx)^.DIEPMSK and (not (USB_OTG_DIEPMSK_TXFURM));

  if (Init.dma_enable) then
  begin
    (*Set threshold parameters *)
    USBx_DEVICE(USBx)^.DTHRCTL := (USB_OTG_DTHRCTL_TXTHRLEN_6 or USB_OTG_DTHRCTL_RXTHRLEN_6);
    USBx_DEVICE(USBx)^.DTHRCTL := USBx_DEVICE(USBx)^.DTHRCTL or (USB_OTG_DTHRCTL_RXTHREN or USB_OTG_DTHRCTL_ISOTHREN or USB_OTG_DTHRCTL_NONISOTHREN);

    i := USBx_DEVICE(USBx)^.DTHRCTL;
  end;

  (* Disable all interrupts. *)
  USBx.GINTMSK := 0;

  (* Clear any pending interrupts *)
  USBx.GINTSTS := $BFFFFFFF;

  (* Enable the common interrupts *)
  if not Init.dma_enable then
    USBx.GINTMSK := USBx.GINTMSK or USB_OTG_GINTMSK_RXFLVLM;

  (* Enable interrupts matching to the Device mode ONLY *)
  USBx.GINTMSK := USBx.GINTMSK or (USB_OTG_GINTMSK_USBSUSPM or USB_OTG_GINTMSK_USBRST or USB_OTG_GINTMSK_ENUMDNEM or USB_OTG_GINTMSK_IEPINT or USB_OTG_GINTMSK_OEPINT or USB_OTG_GINTMSK_IISOIXFRM or
    USB_OTG_GINTMSK_PXFRM_IISOOXFRM or USB_OTG_GINTMSK_WUIM);

  if (Init.Sof_enable) then
    USBx.GINTMSK := USBx.GINTMSK or USB_OTG_GINTMSK_SOFM;

  if (Init.vbus_sensing_enable) then
    USBx.GINTMSK := USBx.GINTMSK or (USB_OTG_GINTMSK_SRQIM or USB_OTG_GINTMSK_OTGINT);

  exit(HAL_OK);
end;

function USB_EnableGlobalInt(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
begin
  USBx.GAHBCFG := USBx.GAHBCFG or USB_OTG_GAHBCFG_GINT;
  exit(HAL_OK);
end;

function USB_DisableGlobalInt(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
begin
  USBx.GAHBCFG := USBx.GAHBCFG and (not USB_OTG_GAHBCFG_GINT);
  exit(HAL_OK);
end;

function USB_SetCurrentMode(var USBx: USB_OTG_GlobalTypeDef; mode: USB_OTG_ModeTypeDef): HAL_StatusTypeDef;
begin
  USBx.GUSBCFG := USBx.GUSBCFG and (not (USB_OTG_GUSBCFG_FHMOD or USB_OTG_GUSBCFG_FDMOD));

  if (mode = USB_OTG_HOST_MODE) then
    USBx.GUSBCFG := USBx.GUSBCFG or USB_OTG_GUSBCFG_FHMOD
  else if (mode = USB_OTG_DEVICE_MODE) then
    USBx.GUSBCFG := USBx.GUSBCFG or USB_OTG_GUSBCFG_FDMOD;

  HAL_Delay(50);

  exit(HAL_OK);
end;

function USB_SetDevSpeed(var USBx: USB_OTG_GlobalTypeDef; speed: byte): HAL_StatusTypeDef;
begin
  USBx_DEVICE(USBx)^.DCFG := USBx_DEVICE(USBx)^.DCFG or speed;
  exit(HAL_OK);
end;

function USB_FlushRxFifo(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
var
  Count: longword;
begin
  Count := 0;

  USBx.GRSTCTL := (USB_OTG_GRSTCTL_RXFFLSH);

  repeat
    Inc(Count);
    if Count > 200000 then
      exit(HAL_TIMEOUT);
  until not ((USBx.GRSTCTL and USB_OTG_GRSTCTL_RXFFLSH) = USB_OTG_GRSTCTL_RXFFLSH);

  exit(HAL_OK);
end;

function USB_FlushTxFifo(var USBx: USB_OTG_GlobalTypeDef; num: longword): HAL_StatusTypeDef;
var
  Count: longword;
begin
  Count := 0;

  USBx.GRSTCTL := (USB_OTG_GRSTCTL_TXFFLSH or (num shl 6));

  repeat
    Inc(Count);
    if Count > 200000 then
      exit(HAL_TIMEOUT);
  until not ((USBx.GRSTCTL and USB_OTG_GRSTCTL_TXFFLSH) = USB_OTG_GRSTCTL_TXFFLSH);

  exit(HAL_OK);
end;

function USB_ActivateEndpoint(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
begin
  if (ep.is_in = 1) then
  begin
    USBx_DEVICE(USBx)^.DAINTMSK := USBx_DEVICE(USBx)^.DAINTMSK or longword(USB_OTG_DAINTMSK_IEPM and ((1 shl (ep.num))));

    if (((USBx_INEP(usbx,ep.num)^.DIEPCTL) and USB_OTG_DIEPCTL_USBAEP) = 0) then
      USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL or longword((ep.maxpacket and USB_OTG_DIEPCTL_MPSIZ) or (ep.type_ shl 18) or ((ep.num) shl 22) or (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) or (USB_OTG_DIEPCTL_USBAEP));
  end
  else
  begin
    USBx_DEVICE(USBx)^.DAINTMSK := USBx_DEVICE(USBx)^.DAINTMSK or longword(USB_OTG_DAINTMSK_OEPM and ((1 shl (ep.num)) shl 16));

    if (((USBx_OUTEP(usbx,ep.num)^.DOEPCTL) and USB_OTG_DOEPCTL_USBAEP) = 0) then
      USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL or longword((ep.maxpacket and USB_OTG_DOEPCTL_MPSIZ) or (ep.type_ shl 18) or (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) or (USB_OTG_DOEPCTL_USBAEP));
  end;
  exit(HAL_OK);
end;

function USB_DeactivateEndpoint(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
begin
  (* Read DEPCTLn register *)
  if (ep.is_in = 1) then
  begin
    USBx_DEVICE(USBx)^.DEACHMSK := USBx_DEVICE(USBx)^.DEACHMSK and (not (USB_OTG_DAINTMSK_IEPM and ((1 shl (ep.num)))));
    USBx_DEVICE(USBx)^.DAINTMSK := USBx_DEVICE(USBx)^.DAINTMSK and (not (USB_OTG_DAINTMSK_IEPM and ((1 shl (ep.num)))));
    USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL and (not USB_OTG_DIEPCTL_USBAEP);
  end
  else
  begin

    USBx_DEVICE(USBx)^.DEACHMSK := USBx_DEVICE(USBx)^.DEACHMSK and (not (USB_OTG_DAINTMSK_OEPM and ((1 shl (ep.num)) shl 16)));
    USBx_DEVICE(USBx)^.DAINTMSK := USBx_DEVICE(USBx)^.DAINTMSK and (not (USB_OTG_DAINTMSK_OEPM and ((1 shl (ep.num)) shl 16)));
    USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL and (not USB_OTG_DOEPCTL_USBAEP);
  end;
  exit(HAL_OK);
end;

function USB_ActivateDedicatedEndpoint(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
  //var debug: longword;
begin
  //debug := 0;

  (* Read DEPCTLn register *)
  if (ep.is_in = 1) then
  begin
    if (((USBx_INEP(usbx,ep.num)^.DIEPCTL) and USB_OTG_DIEPCTL_USBAEP) = 0) then
    begin
      USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL or longword(((ep.maxpacket and USB_OTG_DIEPCTL_MPSIZ) or (ep.type_ shl 18) or ((ep.num) shl 22) or (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) or (USB_OTG_DIEPCTL_USBAEP)));
    end;
    //debug := debug or ((ep.maxpacket and USB_OTG_DIEPCTL_MPSIZ) or (ep.type_ shl 18) or ((ep.num) shl 22) or (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) or (USB_OTG_DIEPCTL_USBAEP));

    USBx_DEVICE(USBx)^.DEACHMSK := USBx_DEVICE(USBx)^.DEACHMSK or longword(USB_OTG_DAINTMSK_IEPM and ((1 shl (ep.num))));
  end
  else
  begin
    if (((USBx_OUTEP(usbx,ep.num)^.DOEPCTL) and USB_OTG_DOEPCTL_USBAEP) = 0) then
    begin
      USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL or longword(((ep.maxpacket and USB_OTG_DOEPCTL_MPSIZ) or (ep.type_ shl 18) or ((ep.num) shl 22) or (USB_OTG_DOEPCTL_USBAEP)));

      //debug := ((USBx) + USB_OTG_OUT_ENDPOINT_BASE + (0) * USB_OTG_EP_REG_SIZE);
      //debug := longword(@USBx_OUTEP(usbx,ep.num)^.DOEPCTL);
      //debug := debug or ((ep.maxpacket and USB_OTG_DOEPCTL_MPSIZ) or (ep.type_ shl 18) or ((ep.num) shl 22) or (USB_OTG_DOEPCTL_USBAEP));
    end;

    USBx_DEVICE(USBx)^.DEACHMSK := USBx_DEVICE(USBx)^.DEACHMSK or longword(USB_OTG_DAINTMSK_OEPM and ((1 shl (ep.num)) shl 16));
  end;

  exit(HAL_OK);
end;

function USB_DeactivateDedicatedEndpoint(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
begin
  (* Read DEPCTLn register *)
  if (ep.is_in = 1) then
  begin
    USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL and (not USB_OTG_DIEPCTL_USBAEP);
    USBx_DEVICE(USBx)^.DAINTMSK := USBx_DEVICE(USBx)^.DAINTMSK and (not (USB_OTG_DAINTMSK_IEPM and ((1 shl (ep.num)))));
  end
  else
  begin
    USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL and (not USB_OTG_DOEPCTL_USBAEP);
    USBx_DEVICE(USBx)^.DAINTMSK := USBx_DEVICE(USBx)^.DAINTMSK and (not (USB_OTG_DAINTMSK_OEPM and ((1 shl (ep.num)) shl 16)));
  end;
  exit(HAL_OK);
end;

function USB_EPStartXfer(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef; dma: boolean): HAL_StatusTypeDef;
var
  pktcnt: word;
begin
  pktcnt := 0;

  (* IN endpoint *)
  if (ep.is_in = 1) then
  begin
    (* Zero Length Packet? *)
    if (ep.xfer_len = 0) then
    begin
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_PKTCNT));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ or (USB_OTG_DIEPTSIZ_PKTCNT and (1 shl 19));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_XFRSIZ));
    end
    else
    begin
      (* Program the transfer size and packet count
      * as follows: xfersize := N * maxpacket +
      * short_packet pktcnt := N + (short_packet
      * exist ? 1 : 0)
      *)
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_XFRSIZ));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_PKTCNT));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ or ((USB_OTG_DIEPTSIZ_PKTCNT and (((ep.xfer_len + ep.maxpacket - 1) div ep.maxpacket) shl 19)));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ or ((USB_OTG_DIEPTSIZ_XFRSIZ and ep.xfer_len));

      if (ep.type_ = EP_TYPE_ISOC) then
      begin
        USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_MULCNT));
        USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ or ((USB_OTG_DIEPTSIZ_MULCNT and (1 shl 29)));
      end;
    end;

    if (dma) then
      USBx_INEP(usbx,ep.num)^.DIEPDMA := (ep.dma_addr)
    else
    begin
      if (ep.type_ <> EP_TYPE_ISOC) then
      begin
        (* Enable the Tx FIFO Empty Interrupt for this EP *)
        if (ep.xfer_len > 0) then
          USBx_DEVICE(USBx)^.DIEPEMPMSK := USBx_DEVICE(USBx)^.DIEPEMPMSK or longword(1 shl ep.num);
      end;
    end;

    if (ep.type_ = EP_TYPE_ISOC) then
    begin
      if ((USBx_DEVICE(USBx)^.DSTS and (1 shl 8)) = 0) then
        USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL or USB_OTG_DIEPCTL_SODDFRM
      else
        USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL or USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
    end;

    (* EP enable, IN data in FIFO *)
    USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL or (USB_OTG_DIEPCTL_CNAK or USB_OTG_DIEPCTL_EPENA);

    if (ep.type_ = EP_TYPE_ISOC) then
      USB_WritePacket(USBx, ep.xfer_buff, ep.num, ep.xfer_len, dma);
  end
  else (* OUT endpoint *)
  begin
    (* Program the transfer size and packet count as follows:
    * pktcnt := N
    * xfersize := N * maxpacket
    *)
    USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ and (not (USB_OTG_DOEPTSIZ_XFRSIZ));
    USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ and (not (USB_OTG_DOEPTSIZ_PKTCNT));

    if (ep.xfer_len = 0) then
    begin
      USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ or longword(USB_OTG_DOEPTSIZ_XFRSIZ and ep.maxpacket);
      USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ or longword(USB_OTG_DOEPTSIZ_PKTCNT and (1 shl 19));
    end
    else
    begin
      pktcnt := (ep.xfer_len + ep.maxpacket - 1) div ep.maxpacket;
      USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ or longword(USB_OTG_DOEPTSIZ_PKTCNT and (pktcnt shl 19));
      USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ or longword(USB_OTG_DOEPTSIZ_XFRSIZ and (ep.maxpacket * pktcnt));
    end;

    if (dma) then
      USBx_OUTEP(usbx,ep.num)^.DOEPDMA := longword(ep.xfer_buff);

    if (ep.type_ = EP_TYPE_ISOC) then
    begin
      if ((USBx_DEVICE(USBx)^.DSTS and (1 shl 8)) = 0) then
        USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL or USB_OTG_DOEPCTL_SODDFRM
      else
        USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL or USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
    end;
    (* EP enable *)
    USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL or (USB_OTG_DOEPCTL_CNAK or USB_OTG_DOEPCTL_EPENA);
  end;
  exit(HAL_OK);
end;

function USB_EP0StartXfer(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef; dma: boolean): HAL_StatusTypeDef;
begin
  (* IN endpoint *)
  if (ep.is_in = 1) then
  begin
    (* Zero Length Packet? *)
    if (ep.xfer_len = 0) then
    begin
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_PKTCNT));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ or (USB_OTG_DIEPTSIZ_PKTCNT and (1 shl 19));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_XFRSIZ));
    end
    else
    begin
      (* Program the transfer size and packet count
      * as follows: xfersize := N * maxpacket +
      * short_packet pktcnt := N + (short_packet
      * exist ? 1 : 0)
      *)
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_XFRSIZ));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ and (not (USB_OTG_DIEPTSIZ_PKTCNT));

      if (ep.xfer_len > ep.maxpacket) then
        ep.xfer_len := ep.maxpacket;

      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ or (USB_OTG_DIEPTSIZ_PKTCNT and (1 shl 19));
      USBx_INEP(usbx,ep.num)^.DIEPTSIZ := USBx_INEP(usbx,ep.num)^.DIEPTSIZ or (USB_OTG_DIEPTSIZ_XFRSIZ and ep.xfer_len);

    end;

    if dma then
      USBx_INEP(usbx,ep.num)^.DIEPDMA := (ep.dma_addr)
    else
    begin
      (* Enable the Tx FIFO Empty Interrupt for this EP *)
      if (ep.xfer_len > 0) then
        USBx_DEVICE(USBx)^.DIEPEMPMSK := USBx_DEVICE(USBx)^.DIEPEMPMSK or longword(1 shl (ep.num));
    end;

    (* EP enable, IN data in FIFO *)
    USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL or (USB_OTG_DIEPCTL_CNAK or USB_OTG_DIEPCTL_EPENA);
  end
  else (* OUT endpoint *)
  begin
    (* Program the transfer size and packet count as follows:
    * pktcnt := N
    * xfersize := N * maxpacket
    *)
    USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ and (not (USB_OTG_DOEPTSIZ_XFRSIZ));
    USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ and (not (USB_OTG_DOEPTSIZ_PKTCNT));

    if (ep.xfer_len > 0) then
      ep.xfer_len := ep.maxpacket;

    USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ or (USB_OTG_DOEPTSIZ_PKTCNT and (1 shl 19));
    USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ := USBx_OUTEP(usbx,ep.num)^.DOEPTSIZ or (USB_OTG_DOEPTSIZ_XFRSIZ and (ep.maxpacket));


    if dma then
      USBx_OUTEP(usbx,ep.num)^.DOEPDMA := longword(ep.xfer_buff);

    (* EP enable *)
    USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL or (USB_OTG_DOEPCTL_CNAK or USB_OTG_DOEPCTL_EPENA);
  end;
  exit(HAL_OK);
end;

function USB_WritePacket(var USBx: USB_OTG_GlobalTypeDef; src: Pbyte; ch_ep_num: byte; len: word; dma: boolean): HAL_StatusTypeDef;
var
  i, count32b: longword;
begin
  if (not dma) then
  begin
    count32b := (len + 3) div 4;
    for i := 0 to count32b - 1 do
    begin
      USBx_DFIFO(usbx,ch_ep_num)^ := plongword(unaligned(src))^;
      Inc(src, 4);
    end;
  end;

  exit(HAL_OK);
end;

function USB_ReadPacket(var USBx: USB_OTG_GlobalTypeDef; dest: Pbyte; len: word): pointer;
var
  i, count32b: longword;
begin
  count32b := (len + 3) div 4;

  for i := 0 to count32b - 1 do
  begin
    plongword(unaligned(dest))^ := USBx_DFIFO(usbx,0)^;
    Inc(dest, 4);
  end;

  exit(dest);
end;

function USB_EPSetStall(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
begin
  if (ep.is_in = 1) then
  begin
    if (((USBx_INEP(usbx,ep.num)^.DIEPCTL) and USB_OTG_DIEPCTL_EPENA) = 0) then
      USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL and (not (USB_OTG_DIEPCTL_EPDIS));

    USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL or USB_OTG_DIEPCTL_STALL;
  end
  else
  begin
    if (((USBx_OUTEP(usbx,ep.num)^.DOEPCTL) and USB_OTG_DOEPCTL_EPENA) = 0) then
      USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL and (not (USB_OTG_DOEPCTL_EPDIS));

    USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL or USB_OTG_DOEPCTL_STALL;
  end;
  exit(HAL_OK);
end;

function USB_EPClearStall(var USBx: USB_OTG_GlobalTypeDef; var ep: USB_OTG_EPTypeDef): HAL_StatusTypeDef;
begin
  if (ep.is_in = 1) then
  begin
    USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL and (not USB_OTG_DIEPCTL_STALL);

    if (ep.type_ = EP_TYPE_INTR) or (ep.type_ = EP_TYPE_BULK) then
      USBx_INEP(usbx,ep.num)^.DIEPCTL := USBx_INEP(usbx,ep.num)^.DIEPCTL or USB_OTG_DIEPCTL_SD0PID_SEVNFRM; (* DATA0 *)
  end
  else
  begin
    USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL and (not USB_OTG_DOEPCTL_STALL);

    if (ep.type_ = EP_TYPE_INTR )or (ep.type_ = EP_TYPE_BULK) then
      USBx_OUTEP(usbx,ep.num)^.DOEPCTL := USBx_OUTEP(usbx,ep.num)^.DOEPCTL or USB_OTG_DOEPCTL_SD0PID_SEVNFRM; (* DATA0 *)
  end;
  exit(HAL_OK);

end;

function USB_SetDevAddress(var USBx: USB_OTG_GlobalTypeDef; address: byte): HAL_StatusTypeDef;
begin
  USBx_DEVICE(USBx)^.DCFG := USBx_DEVICE(USBx)^.DCFG and (not (USB_OTG_DCFG_DAD));
  USBx_DEVICE(USBx)^.DCFG := USBx_DEVICE(USBx)^.DCFG or ((address shl 4) and USB_OTG_DCFG_DAD);

  exit(HAL_OK);
end;

function USB_DevConnect(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
begin
  USBx_DEVICE(USBx)^.DCTL := USBx_DEVICE(USBx)^.DCTL and (not USB_OTG_DCTL_SDIS);
  HAL_Delay(3);

  exit(HAL_OK);
end;

function USB_DevDisconnect(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
begin
  USBx_DEVICE(USBx)^.DCTL := USBx_DEVICE(USBx)^.DCTL or USB_OTG_DCTL_SDIS;
  HAL_Delay(3);

  exit(HAL_OK);
end;

function USB_StopDevice(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
var
  i: integer;
begin
  (* Clear Pending interrupt *)
  for i := 0 to 15 - 1 do
  begin
    USBx_INEP(usbx,i)^.DIEPINT := $FF;
    USBx_OUTEP(usbx,i)^.DOEPINT := $FF;
  end;
  USBx_DEVICE(USBx)^.DAINT := $FFFFFFFF;

  (* Clear interrupt masks *)
  USBx_DEVICE(USBx)^.DIEPMSK := 0;
  USBx_DEVICE(USBx)^.DOEPMSK := 0;
  USBx_DEVICE(USBx)^.DAINTMSK := 0;

  (* Flush the FIFO *)
  USB_FlushRxFifo(USBx);
  USB_FlushTxFifo(USBx, $10);

  exit(HAL_OK);
end;

function USB_ActivateSetup(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
begin
  (* Set the MPS of the IN EP based on the enumeration speed *)
  USBx_INEP(usbx,0)^.DIEPCTL := USBx_INEP(usbx,0)^.DIEPCTL and (not USB_OTG_DIEPCTL_MPSIZ);

  if ((USBx_DEVICE(USBx)^.DSTS and USB_OTG_DSTS_ENUMSPD) = DSTS_ENUMSPD_LS_PHY_6MHZ) then
  begin
    USBx_INEP(usbx,0)^.DIEPCTL := USBx_INEP(usbx,0)^.DIEPCTL or 3;
  end;
  USBx_DEVICE(USBx)^.DCTL := USBx_DEVICE(USBx)^.DCTL or USB_OTG_DCTL_CGINAK;

  exit(HAL_OK);
end;

function USB_EP0_OutStart(var USBx: USB_OTG_GlobalTypeDef; dma: byte; psetup: Pbyte): HAL_StatusTypeDef;
begin
  USBx_OUTEP(usbx,0)^.DOEPTSIZ := 0;
  USBx_OUTEP(usbx,0)^.DOEPTSIZ := USBx_OUTEP(usbx,0)^.DOEPTSIZ or (USB_OTG_DOEPTSIZ_PKTCNT and (1 shl 19));
  USBx_OUTEP(usbx,0)^.DOEPTSIZ := USBx_OUTEP(usbx,0)^.DOEPTSIZ or (3 * 8);
  USBx_OUTEP(usbx,0)^.DOEPTSIZ := USBx_OUTEP(usbx,0)^.DOEPTSIZ or USB_OTG_DOEPTSIZ_STUPCNT;

  if (dma = 1) then
  begin
    USBx_OUTEP(usbx,0)^.DOEPDMA := longword(psetup);
    (* EP enable *)
    USBx_OUTEP(usbx,0)^.DOEPCTL := $80008000;
  end;

  exit(HAL_OK);
end;

function USB_GetDevSpeed(var USBx: USB_OTG_GlobalTypeDef): byte;
var
  speed: longword;
begin
  if ((USBx_DEVICE(USBx)^.DSTS and USB_OTG_DSTS_ENUMSPD) = DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ) then
    speed := USB_OTG_SPEED_HIGH
  else if (((USBx_DEVICE(USBx)^.DSTS and USB_OTG_DSTS_ENUMSPD) = DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ) or ((USBx_DEVICE(USBx)^.DSTS and USB_OTG_DSTS_ENUMSPD) = DSTS_ENUMSPD_FS_PHY_48MHZ)) then
    speed := USB_OTG_SPEED_FULL
  else if ((USBx_DEVICE(USBx)^.DSTS and USB_OTG_DSTS_ENUMSPD) = DSTS_ENUMSPD_LS_PHY_6MHZ) then
    speed := USB_OTG_SPEED_LOW;

  exit(speed);
end;

function USB_GetMode(var USBx: USB_OTG_GlobalTypeDef): longword;
begin
  exit((USBx.GINTSTS) and $1);
end;

function USB_ReadInterrupts(var USBx: USB_OTG_GlobalTypeDef): longword;
var
  v: longword;
begin
  v := USBx.GINTSTS;
  v := v and (USBx.GINTMSK);
  exit(v);
end;

function USB_ReadDevAllOutEpInterrupt(var USBx: USB_OTG_GlobalTypeDef): longword;
var
  v: longword;
begin
  v := USBx_DEVICE(USBx)^.DAINT;
  v := v and (USBx_DEVICE(USBx)^.DAINTMSK);
  exit((v and $ffff0000) shr 16);
end;

function USB_ReadDevOutEPInterrupt(var USBx: USB_OTG_GlobalTypeDef; epnum: byte): longword;
var
  v: longword;
begin
  v := USBx_OUTEP(usbx,epnum)^.DOEPINT;
  v := v and (USBx_DEVICE(USBx)^.DOEPMSK);
  exit(v);
end;

function USB_ReadDevAllInEpInterrupt(var USBx: USB_OTG_GlobalTypeDef): longword;
var
  v: longword;
begin
  v := USBx_DEVICE(USBx)^.DAINT;
  v := v and (USBx_DEVICE(USBx)^.DAINTMSK);
  exit((v and $FFFF));
end;

function USB_ReadDevInEPInterrupt(var USBx: USB_OTG_GlobalTypeDef; epnum: byte): longword;
var
  msk, emp, v: longword;
begin
  msk := USBx_DEVICE(USBx)^.DIEPMSK;
  emp := USBx_DEVICE(USBx)^.DIEPEMPMSK;
  msk := msk or ((emp shr epnum) and $1) shl 7;
  v := USBx_INEP(usbx,epnum)^.DIEPINT and msk;

  exit(v);
end;

procedure USB_ClearInterrupts(var USBx: USB_OTG_GlobalTypeDef; interrupt: longword);
begin
  USBx.GINTSTS := USBx.GINTSTS or interrupt;
end;

function USB_HostInit(var USBx: USB_OTG_GlobalTypeDef; cfg: USB_OTG_CfgTypeDef): HAL_StatusTypeDef;
var
  i: integer;
begin
  (* Restart the Phy Clock *)
  USBx_PCGCCTL(usbx)^ := 0;

  (*Activate VBUS Sensing B *)
  USBx.GCCFG := USBx.GCCFG or USB_OTG_GCCFG_VBDEN;

  (* Disable the FS div LS support mode only *)
  if ((cfg.speed = USB_OTG_SPEED_FULL) and (@USBx <> @OTG_FS_GLOBAL)) then
    USBx_HOST(usbx)^.HCFG := USBx_HOST(usbx)^.HCFG or USB_OTG_HCFG_FSLSS
  else
    USBx_HOST(usbx)^.HCFG := USBx_HOST(usbx)^.HCFG and (not (USB_OTG_HCFG_FSLSS));

  (* Make sure the FIFOs are flushed. *)
  USB_FlushTxFifo(USBx, $10); (* all Tx FIFOs *)
  USB_FlushRxFifo(USBx);

  (* Clear all pending HC Interrupts *)
  for i := 0 to cfg.Host_channels - 1 do
  begin
    USBx_HC(usbx,i)^.HCINT := $FFFFFFFF;
    USBx_HC(usbx,i)^.HCINTMSK := 0;
  end;

  (* Enable VBUS driving *)
  USB_DriveVbus(USBx, 1);

  HAL_Delay(200);

  (* Disable all interrupts. *)
  USBx.GINTMSK := 0;

  (* Clear any pending interrupts *)
  USBx.GINTSTS := $FFFFFFFF;


  if (@USBx = @OTG_FS_GLOBAL) then
  begin
    (* set Rx FIFO size *)
    USBx.GRXFSIZ := $80;
    USBx.DIEPTXF0_HNPTXFSIZ := ((($60 shl 16) and USB_OTG_NPTXFD) or $80);
    USBx.HPTXFSIZ := ((($40 shl 16) and USB_OTG_HPTXFSIZ_PTXFD) or $E0);

  end

  else
  begin
    (* set Rx FIFO size *)
    USBx.GRXFSIZ := $200;
    USBx.DIEPTXF0_HNPTXFSIZ := ((($100 shl 16) and USB_OTG_NPTXFD) or $200);
    USBx.HPTXFSIZ := ((($E0 shl 16) and USB_OTG_HPTXFSIZ_PTXFD) or $300);
  end;

  (* Enable the common interrupts *)
  if not cfg.dma_enable then
    USBx.GINTMSK := USBx.GINTMSK or USB_OTG_GINTMSK_RXFLVLM;

  (* Enable interrupts matching to the Host mode ONLY *)
  USBx.GINTMSK := USBx.GINTMSK or (USB_OTG_GINTMSK_PRTIM or USB_OTG_GINTMSK_HCIM or USB_OTG_GINTMSK_SOFM or USB_OTG_GINTSTS_DISCINT or USB_OTG_GINTMSK_PXFRM_IISOOXFRM or USB_OTG_GINTMSK_WUIM);

  exit(HAL_OK);
end;

function USB_InitFSLSPClkSel(var USBx: USB_OTG_GlobalTypeDef; freq: byte): HAL_StatusTypeDef;
begin
  USBx_HOST(usbx)^.HCFG := USBx_HOST(usbx)^.HCFG and (not (USB_OTG_HCFG_FSLSPCS));
  USBx_HOST(usbx)^.HCFG := USBx_HOST(usbx)^.HCFG or longword(freq and USB_OTG_HCFG_FSLSPCS);

  if (freq = HCFG_48_MHZ) then
    USBx_HOST(usbx)^.HFIR := 48000
  else if (freq = HCFG_6_MHZ) then
    USBx_HOST(usbx)^.HFIR := 6000;

  exit(HAL_OK);
end;

function USB_ResetPort(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
var
  hprt0: longword;
begin
  hprt0 := USBx_HPRT0(USBx)^;

  hprt0 := hprt0 and (not (USB_OTG_HPRT_PENA or USB_OTG_HPRT_PCDET or USB_OTG_HPRT_PENCHNG or USB_OTG_HPRT_POCCHNG));

  USBx_HPRT0(USBx)^ := (USB_OTG_HPRT_PRST or hprt0);
  HAL_Delay(10);                                (* See Note #1 *)
  USBx_HPRT0(USBx)^ := ((not USB_OTG_HPRT_PRST) and hprt0);
  exit(HAL_OK);
end;

function USB_DriveVbus(var USBx: USB_OTG_GlobalTypeDef; state: byte): HAL_StatusTypeDef;
var
  hprt0: longword;
begin
  hprt0 := USBx_HPRT0(USBx)^;
  hprt0 := hprt0 and (not (USB_OTG_HPRT_PENA or USB_OTG_HPRT_PCDET or USB_OTG_HPRT_PENCHNG or USB_OTG_HPRT_POCCHNG));

  if (((hprt0 and USB_OTG_HPRT_PPWR) = 0) and (state = 1)) then
    USBx_HPRT0(USBx)^ := (USB_OTG_HPRT_PPWR or hprt0);

  if (((hprt0 and USB_OTG_HPRT_PPWR) = USB_OTG_HPRT_PPWR) and (state = 0)) then
    USBx_HPRT0(USBx)^ := ((not USB_OTG_HPRT_PPWR) and hprt0);

  exit(HAL_OK);
end;

function USB_GetHostSpeed(var USBx: USB_OTG_GlobalTypeDef): longword;
var
  hprt0: longword;
begin
  hprt0 := USBx_HPRT0(USBx)^;
  exit((hprt0 and USB_OTG_HPRT_PSPD) shr 17);
end;

function USB_GetCurrentFrame(var USBx: USB_OTG_GlobalTypeDef): longword;
begin
  exit(USBx_HOST(USBx)^.HFNUM and USB_OTG_HFNUM_FRNUM);
end;

function USB_HC_Init(var USBx: USB_OTG_GlobalTypeDef; ch_num, epnum, dev_address, speed, ep_type: byte; mps: word): HAL_StatusTypeDef;
begin
  (* Clear old interrupt conditions for this host channel. *)
  USBx_HC(usbx,ch_num)^.HCINT := $FFFFFFFF;

  (* Enable channel interrupts required for this transfer. *)
  case ep_type of
    EP_TYPE_CTRL,
    EP_TYPE_BULK:

    begin
      USBx_HC(usbx,ch_num)^.HCINTMSK := USB_OTG_HCINTMSK_XFRCM or USB_OTG_HCINTMSK_STALLM or USB_OTG_HCINTMSK_TXERRM or USB_OTG_HCINTMSK_DTERRM or USB_OTG_HCINTMSK_AHBERR or USB_OTG_HCINTMSK_NAKM;

      if (epnum and $80)<>0 then
        USBx_HC(usbx,ch_num)^.HCINTMSK := USBx_HC(usbx,ch_num)^.HCINTMSK or USB_OTG_HCINTMSK_BBERRM
      else
      begin
        if (@USBx <> @OTG_FS_GLOBAL) then
          USBx_HC(usbx,ch_num)^.HCINTMSK := USBx_HC(usbx,ch_num)^.HCINTMSK or (USB_OTG_HCINTMSK_NYET or USB_OTG_HCINTMSK_ACKM);
      end;
    end;
    EP_TYPE_INTR:
    begin
      USBx_HC(usbx,ch_num)^.HCINTMSK := USB_OTG_HCINTMSK_XFRCM or USB_OTG_HCINTMSK_STALLM or USB_OTG_HCINTMSK_TXERRM or USB_OTG_HCINTMSK_DTERRM or USB_OTG_HCINTMSK_NAKM or USB_OTG_HCINTMSK_AHBERR or USB_OTG_HCINTMSK_FRMORM;

      if (epnum and $80)<>0 then
        USBx_HC(usbx,ch_num)^.HCINTMSK := USBx_HC(usbx,ch_num)^.HCINTMSK or USB_OTG_HCINTMSK_BBERRM;
    end;
    EP_TYPE_ISOC:
    begin
      USBx_HC(usbx,ch_num)^.HCINTMSK := USB_OTG_HCINTMSK_XFRCM or USB_OTG_HCINTMSK_ACKM or USB_OTG_HCINTMSK_AHBERR or USB_OTG_HCINTMSK_FRMORM;

      if (epnum and $80)<>0 then
        USBx_HC(usbx,ch_num)^.HCINTMSK := USBx_HC(usbx,ch_num)^.HCINTMSK or (USB_OTG_HCINTMSK_TXERRM or USB_OTG_HCINTMSK_BBERRM);
    end;
  end;

  (* Enable the top level host channel interrupt. *)
  USBx_HOST(usbx)^.HAINTMSK := USBx_HOST(usbx)^.HAINTMSK or longword(1 shl ch_num);

  (* Make sure host channel interrupts are enabled. *)
  USBx.GINTMSK := USBx.GINTMSK or USB_OTG_GINTMSK_HCIM;

  (* Program the HCCHAR register *)
  USBx_HC(usbx,ch_num)^.HCCHAR := longword(
    longword((dev_address shl 22) and USB_OTG_HCCHAR_DAD) or
    longword(((epnum and $7F) shl 11) and USB_OTG_HCCHAR_EPNUM) or
    longword((ord((epnum and $80) = $80) shl 15) and USB_OTG_HCCHAR_EPDIR) or
    longword((ord(speed = HPRT0_PRTSPD_LOW_SPEED) shl 17) and USB_OTG_HCCHAR_LSDEV) or
    longword((ep_type shl 18) and USB_OTG_HCCHAR_EPTYP) or
    longword(mps and USB_OTG_HCCHAR_MPSIZ));

  if (ep_type = EP_TYPE_INTR) then
    USBx_HC(usbx,ch_num)^.HCCHAR := USBx_HC(usbx,ch_num)^.HCCHAR or USB_OTG_HCCHAR_ODDFRM;

  exit(HAL_OK);
end;

function USB_HC_StartXfer(var USBx: USB_OTG_GlobalTypeDef; var hc: USB_OTG_HCTypeDef; dma: boolean): HAL_StatusTypeDef;
var
  is_oddframe: byte;
  max_hc_pkt_count, num_packets, len_words: word;
  tmpreg: longword;
begin
  is_oddframe := 0;
  len_words := 0;
  num_packets := 0;
  max_hc_pkt_count := 256;
  tmpreg := 0;

  if ((@USBx <> @OTG_FS_GLOBAL) and (hc.speed = USB_OTG_SPEED_HIGH)) then
  begin
    if ((not dma) and (hc.do_ping = 1)) then
    begin
      USB_DoPing(USBx, hc.ch_num);
      exit(HAL_OK);
    end
    else if (dma) then
    begin
      USBx_HC(usbx,hc.ch_num)^.HCINTMSK := USBx_HC(usbx,hc.ch_num)^.HCINTMSK and (not (USB_OTG_HCINTMSK_NYET or USB_OTG_HCINTMSK_ACKM));
      hc.do_ping := 0;
    end;
  end;

  (* Compute the expected number of packets associated to the transfer *)
  if (hc.xfer_len > 0) then
  begin
    num_packets := (hc.xfer_len + hc.max_packet - 1) div hc.max_packet;

    if (num_packets > max_hc_pkt_count) then
    begin
      num_packets := max_hc_pkt_count;
      hc.xfer_len := num_packets * hc.max_packet;
    end;
  end
  else
    num_packets := 1;

  if (hc.ep_is_in <> 0) then
    hc.xfer_len := num_packets * hc.max_packet;

  (* Initialize the HCTSIZn register *)
  USBx_HC(usbx,hc.ch_num)^.HCTSIZ := (((hc.xfer_len) and USB_OTG_HCTSIZ_XFRSIZ)) or ((num_packets shl 19) and USB_OTG_HCTSIZ_PKTCNT) or (((hc.data_pid) shl 29) and USB_OTG_HCTSIZ_DPID);

  if (dma) then
    (* xfer_buff MUST be 32-bits aligned *)
    USBx_HC(usbx,hc.ch_num)^.HCDMA := longword(hc.xfer_buff);

  is_oddframe := (USBx_HOST(usbx)^.HFNUM and $01) xor $01;
  USBx_HC(usbx,hc.ch_num)^.HCCHAR := USBx_HC(usbx,hc.ch_num)^.HCCHAR and (not USB_OTG_HCCHAR_ODDFRM);
  USBx_HC(usbx,hc.ch_num)^.HCCHAR := USBx_HC(usbx,hc.ch_num)^.HCCHAR or (is_oddframe shl 29);

  (* Set host channel enable *)
  tmpreg := USBx_HC(usbx,hc.ch_num)^.HCCHAR;
  tmpreg := tmpreg and (not USB_OTG_HCCHAR_CHDIS);
  tmpreg := tmpreg or USB_OTG_HCCHAR_CHENA;
  USBx_HC(usbx,hc.ch_num)^.HCCHAR := tmpreg;

  if (not dma) (* Slave mode *) then
  begin
    if ((hc.ep_is_in = 0) and (hc.xfer_len > 0)) then
    begin
      case hc.ep_type of
        (* Non periodic transfer *)
        EP_TYPE_CTRL,
        EP_TYPE_BULK:
        begin
          len_words := (hc.xfer_len + 3) div 4;

          (* check if there is enough space in FIFO space *)
          if (len_words > (USBx.HNPTXSTS and $FFFF)) then
            (* need to process data in nptxfempty interrupt *)
            USBx.GINTMSK := USBx.GINTMSK or USB_OTG_GINTMSK_NPTXFEM;
        end;
        (* Periodic transfer *)
        EP_TYPE_INTR,
        EP_TYPE_ISOC:
        begin
          len_words := (hc.xfer_len + 3) div 4;
          (* check if there is enough space in FIFO space *)
          if (len_words > (USBx_HOST(usbx)^.HPTXSTS and $FFFF)) (* split the transfer *) then
            (* need to process data in ptxfempty interrupt *)
            USBx.GINTMSK := USBx.GINTMSK or USB_OTG_GINTMSK_PTXFEM;
        end;
      end;

      (* Write packet into the Tx FIFO. *)
      USB_WritePacket(USBx, hc.xfer_buff, hc.ch_num, hc.xfer_len, false);
    end;
  end;

  exit(HAL_OK);
end;

function USB_HC_ReadInterrupt(var USBx: USB_OTG_GlobalTypeDef): longword;
begin
  exit((USBx_HOST(usbx)^.HAINT) and $FFFF);
end;

function USB_HC_Halt(var USBx: USB_OTG_GlobalTypeDef; hc_num: byte): HAL_StatusTypeDef;
var
  Count: longword;
begin
  Count := 0;
  (* Check for space in the request queue to issue the halt. *)
  if (((USBx_HC(usbx,hc_num)^.HCCHAR) and (HCCHAR_CTRL shl 18)) or ((USBx_HC(usbx,hc_num)^.HCCHAR) and (HCCHAR_BULK shl 18)))<>0 then
  begin
    USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR or USB_OTG_HCCHAR_CHDIS;

    if ((USBx.HNPTXSTS and $FFFF) = 0) then
    begin
      USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR and (not USB_OTG_HCCHAR_CHENA);
      USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR or USB_OTG_HCCHAR_CHENA;
      USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR and (not USB_OTG_HCCHAR_EPDIR);

      repeat
        Inc(Count);
        if Count > 1000 then
          break;
      until not ((USBx_HC(usbx,hc_num)^.HCCHAR and USB_OTG_HCCHAR_CHENA) = USB_OTG_HCCHAR_CHENA);
    end
    else
      USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR or USB_OTG_HCCHAR_CHENA;
  end
  else
  begin
    USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR or USB_OTG_HCCHAR_CHDIS;

    if ((USBx_HOST(usbx)^.HPTXSTS and $FFFF) = 0) then
    begin
      USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR and (not USB_OTG_HCCHAR_CHENA);
      USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR or USB_OTG_HCCHAR_CHENA;
      USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR and (not USB_OTG_HCCHAR_EPDIR);

      repeat
        Inc(Count);
        if Count > 1000 then
          break;
      until not ((USBx_HC(usbx,hc_num)^.HCCHAR and USB_OTG_HCCHAR_CHENA) = USB_OTG_HCCHAR_CHENA);

      while ((USBx_HC(usbx,hc_num)^.HCCHAR and USB_OTG_HCCHAR_CHENA) = USB_OTG_HCCHAR_CHENA) do ;
    end
    else
      USBx_HC(usbx,hc_num)^.HCCHAR := USBx_HC(usbx,hc_num)^.HCCHAR or USB_OTG_HCCHAR_CHENA;
  end;

  exit(HAL_OK);
end;

function USB_DoPing(var USBx: USB_OTG_GlobalTypeDef; ch_num: byte): HAL_StatusTypeDef;
var
  tmpreg: longword;
  num_packets: integer;
begin
  num_packets := 1;

  USBx_HC(usbx,ch_num)^.HCTSIZ := longword((num_packets shl 19) and USB_OTG_HCTSIZ_PKTCNT) or USB_OTG_HCTSIZ_DOPING;

  (* Set host channel enable *)
  tmpreg := USBx_HC(usbx,ch_num)^.HCCHAR;
  tmpreg := tmpreg and (not USB_OTG_HCCHAR_CHDIS);
  tmpreg := tmpreg or USB_OTG_HCCHAR_CHENA;
  USBx_HC(usbx,ch_num)^.HCCHAR := tmpreg;

  exit(HAL_OK);
end;

function USB_StopHost(var USBx: USB_OTG_GlobalTypeDef): HAL_StatusTypeDef;
var
  i: integer;
  Count, Value: longword;
begin
  Count := 0;

  USB_DisableGlobalInt(USBx);

  (* Flush FIFO *)
  USB_FlushTxFifo(USBx, $10);
  USB_FlushRxFifo(USBx);

  (* Flush out any leftover queued requests. *)
  for i := 0 to 15 do
  begin
    Value := USBx_HC(usbx,i)^.HCCHAR;
    Value := Value or USB_OTG_HCCHAR_CHDIS;
    Value := Value and (not USB_OTG_HCCHAR_CHENA);
    Value := Value and (not USB_OTG_HCCHAR_EPDIR);
    USBx_HC(usbx,i)^.HCCHAR := Value;
  end;

  (* Halt all channels to put them into a known state. *)
  for i := 0 to 15 do
  begin
    Value := USBx_HC(usbx,i)^.HCCHAR;

    Value := Value or USB_OTG_HCCHAR_CHDIS;
    Value := Value or USB_OTG_HCCHAR_CHENA;
    Value := Value and (not USB_OTG_HCCHAR_EPDIR);

    USBx_HC(usbx,i)^.HCCHAR := Value;
    repeat
      Inc(Count);
      if Count > 1000 then
        break;
    until not ((USBx_HC(usbx,i)^.HCCHAR and USB_OTG_HCCHAR_CHENA) = USB_OTG_HCCHAR_CHENA);
  end;

  (* Clear any pending Host interrupts *)
  USBx_HOST(usbx)^.HAINT := $FFFFFFFF;
  USBx.GINTSTS := $FFFFFFFF;
  USB_EnableGlobalInt(USBx);
  exit(HAL_OK);
end;

end.
