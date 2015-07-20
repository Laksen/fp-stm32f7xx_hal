(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_eth.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of ETH HAL module.
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

unit stm32f7xx_hal_eth;

interface

{$packrecords 4}

uses
  stm32f7xx_defs,
  stm32f7xx_hal;

(** @addtogroup ETH_Private_Defines
  * @{
   *)

(* Delay to wait when writing to some Ethernet registers  *)

const
  ETH_REG_WRITE_DELAY = $00000001;
  (* ETHERNET Errors  *)

  ETH_SUCCESS = 0;
  ETH_ERROR = 1;
  (* ETHERNET DMA Tx descriptors Collision Count Shift  *)

  ETH_DMATXDESC_COLLISION_COUNTSHIFT = 3;
  (* ETHERNET DMA Tx descriptors Buffer2 Size Shift  *)

  ETH_DMATXDESC_BUFFER2_SIZESHIFT = 16;
  (* ETHERNET DMA Rx descriptors Frame Length Shift  *)

  ETH_DMARXDESC_FRAME_LENGTHSHIFT = 16;
  (* ETHERNET DMA Rx descriptors Buffer2 Size Shift  *)

  ETH_DMARXDESC_BUFFER2_SIZESHIFT = 16;
  (* ETHERNET DMA Rx descriptors Frame length Shift  *)

  ETH_DMARXDESC_FRAMELENGTHSHIFT = 16;
  (* ETHERNET MAC address offsets  *)

  ETH_MAC_ADDR_HBASE = (Ethernet_MAC_BASE + $40);  (* ETHERNET MAC address high offset  *)
  ETH_MAC_ADDR_LBASE = (Ethernet_MAC_BASE + $44);  (* ETHERNET MAC address low offset  *)
  (* ETHERNET MACMIIAR register Mask  *)

  ETH_MACMIIAR_CR_MASK = $FFFFFFE3;
  (* ETHERNET MACCR register Mask  *)

  ETH_MACCR_CLEAR_MASK = $FF20810F;
  (* ETHERNET MACFCR register Mask  *)

  ETH_MACFCR_CLEAR_MASK = $0000FF41;
  (* ETHERNET DMAOMR register Mask  *)

  ETH_DMAOMR_CLEAR_MASK = $F8DE3F23;
  (* ETHERNET Remote Wake-up frame register length  *)

  ETH_WAKEUP_REGISTER_LENGTH = 8;
  (* ETHERNET Missed frames counter Shift  *)

  ETH_DMA_RX_OVERFLOW_MISSEDFRAMES_COUNTERSHIFT = 17;
  (**
  * @}
   *)

(* Exported types ------------------------------------------------------------ *)

  (** @defgroup ETH_Exported_Types ETH Exported Types
  * @{
   *)

  (**
  * @brief  HAL State structures definition
   *)

const
  HAL_ETH_STATE_RESET = $00;  (*!< Peripheral not yet Initialized or disabled          *)
  HAL_ETH_STATE_READY = $01;  (*!< Peripheral Initialized and ready for use            *)
  HAL_ETH_STATE_BUSY = $02;  (*!< an internal process is ongoing                      *)
  HAL_ETH_STATE_BUSY_TX = $12;  (*!< Data Transmission process is ongoing                *)
  HAL_ETH_STATE_BUSY_RX = $22;  (*!< Data Reception process is ongoing                   *)
  HAL_ETH_STATE_BUSY_TX_RX = $32;  (*!< Data Transmission and Reception process is ongoing  *)
  HAL_ETH_STATE_BUSY_WR = $42;  (*!< Write process is ongoing                            *)
  HAL_ETH_STATE_BUSY_RD = $82;  (*!< Read process is ongoing                             *)
  HAL_ETH_STATE_TIMEOUT = $03;  (*!< Timeout state                                       *)
  HAL_ETH_STATE_ERROR = $04;  (*!< Reception process is ongoing                        *)

type
  HAL_ETH_StateTypeDef = integer;

  (**
  * @brief  ETH Init Structure definition
   *)

  ETH_InitTypeDef = record
    AutoNegotiation: longword;  (*!< Selects or not the AutoNegotiation mode for the external PHY
                                                           The AutoNegotiation allows an automatic setting of the Speed (10/100Mbps)
                                                           and the mode (half/full-duplex).
                                                           This parameter can be a value of @ref ETH_AutoNegotiation  *)
    Speed: longword;  (*!< Sets the Ethernet speed: 10/100 Mbps.
                                                           This parameter can be a value of @ref ETH_Speed  *)
    DuplexMode: longword;  (*!< Selects the MAC duplex mode: Half-Duplex or Full-Duplex mode
                                                           This parameter can be a value of @ref ETH_Duplex_Mode  *)
    PhyAddress: word;  (*!< Ethernet PHY address.
                                                           This parameter must be a number between Min_Data = 0 and Max_Data = 32  *)
    MACAddr: Pbyte;  (*!< MAC Address of used Hardware: must be pointer on an array of 6 bytes  *)
    RxMode: longword;  (*!< Selects the Ethernet Rx mode: Polling mode, Interrupt mode.
                                                           This parameter can be a value of @ref ETH_Rx_Mode  *)
    ChecksumMode: longword;  (*!< Selects if the checksum is check by hardware or by software.
                                                         This parameter can be a value of @ref ETH_Checksum_Mode  *)
    MediaInterface: longword;  (*!< Selects the media-independent interface or the reduced media-independent interface.
                                                         This parameter can be a value of @ref ETH_Media_Interface  *)
  end;


  (**
  * @brief  ETH MAC Configuration Structure definition
   *)

  PETH_MACInitTypeDef = ^ETH_MACInitTypeDef;

  ETH_MACInitTypeDef = record
    Watchdog: longword;  (*!< Selects or not the Watchdog timer
                                                           When enabled, the MAC allows no more then 2048 bytes to be received.
                                                           When disabled, the MAC can receive up to 16384 bytes.
                                                           This parameter can be a value of @ref ETH_Watchdog  *)
    Jabber: longword;  (*!< Selects or not Jabber timer
                                                           When enabled, the MAC allows no more then 2048 bytes to be sent.
                                                           When disabled, the MAC can send up to 16384 bytes.
                                                           This parameter can be a value of @ref ETH_Jabber  *)
    InterFrameGap: longword;  (*!< Selects the minimum IFG between frames during transmission.
                                                           This parameter can be a value of @ref ETH_Inter_Frame_Gap  *)
    CarrierSense: longword;  (*!< Selects or not the Carrier Sense.
                                                           This parameter can be a value of @ref ETH_Carrier_Sense  *)
    ReceiveOwn: longword;  (*!< Selects or not the ReceiveOwn,
                                                           ReceiveOwn allows the reception of frames when the TX_EN signal is asserted
                                                           in Half-Duplex mode.
                                                           This parameter can be a value of @ref ETH_Receive_Own  *)
    LoopbackMode: longword;  (*!< Selects or not the internal MAC MII Loopback mode.
                                                           This parameter can be a value of @ref ETH_Loop_Back_Mode  *)
    ChecksumOffload: longword;  (*!< Selects or not the IPv4 checksum checking for received frame payloads' TCP/UDP/ICMP headers.
                                                           This parameter can be a value of @ref ETH_Checksum_Offload  *)
    RetryTransmission: longword;  (*!< Selects or not the MAC attempt retries transmission, based on the settings of BL,
                                                           when a collision occurs (Half-Duplex mode).
                                                           This parameter can be a value of @ref ETH_Retry_Transmission  *)
    AutomaticPadCRCStrip: longword;  (*!< Selects or not the Automatic MAC Pad/CRC Stripping.
                                                           This parameter can be a value of @ref ETH_Automatic_Pad_CRC_Strip  *)
    BackOffLimit: longword;  (*!< Selects the BackOff limit value.
                                                           This parameter can be a value of @ref ETH_Back_Off_Limit  *)
    DeferralCheck: longword;  (*!< Selects or not the deferral check function (Half-Duplex mode).
                                                           This parameter can be a value of @ref ETH_Deferral_Check  *)
    ReceiveAll: longword;  (*!< Selects or not all frames reception by the MAC (No filtering).
                                                           This parameter can be a value of @ref ETH_Receive_All  *)
    SourceAddrFilter: longword;  (*!< Selects the Source Address Filter mode.
                                                           This parameter can be a value of @ref ETH_Source_Addr_Filter  *)
    PassControlFrames: longword;  (*!< Sets the forwarding mode of the control frames (including unicast and multicast PAUSE frames)
                                                           This parameter can be a value of @ref ETH_Pass_Control_Frames  *)
    BroadcastFramesReception: longword;  (*!< Selects or not the reception of Broadcast Frames.
                                                           This parameter can be a value of @ref ETH_Broadcast_Frames_Reception  *)
    DestinationAddrFilter: longword;  (*!< Sets the destination filter mode for both unicast and multicast frames.
                                                           This parameter can be a value of @ref ETH_Destination_Addr_Filter  *)
    PromiscuousMode: longword;  (*!< Selects or not the Promiscuous Mode
                                                           This parameter can be a value of @ref ETH_Promiscuous_Mode  *)
    MulticastFramesFilter: longword;  (*!< Selects the Multicast Frames filter mode: None/HashTableFilter/PerfectFilter/PerfectHashTableFilter.
                                                           This parameter can be a value of @ref ETH_Multicast_Frames_Filter  *)
    UnicastFramesFilter: longword;  (*!< Selects the Unicast Frames filter mode: HashTableFilter/PerfectFilter/PerfectHashTableFilter.
                                                           This parameter can be a value of @ref ETH_Unicast_Frames_Filter  *)
    HashTableHigh: longword;  (*!< This field holds the higher 32 bits of Hash table.
                                                           This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xFFFFFFFF  *)
    HashTableLow: longword;  (*!< This field holds the lower 32 bits of Hash table.
                                                           This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xFFFFFFFF   *)
    PauseTime: longword;  (*!< This field holds the value to be used in the Pause Time field in the transmit control frame.
                                                           This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xFFFF  *)
    ZeroQuantaPause: longword;  (*!< Selects or not the automatic generation of Zero-Quanta Pause Control frames.
                                                           This parameter can be a value of @ref ETH_Zero_Quanta_Pause  *)
    PauseLowThreshold: longword;  (*!< This field configures the threshold of the PAUSE to be checked for
                                                           automatic retransmission of PAUSE Frame.
                                                           This parameter can be a value of @ref ETH_Pause_Low_Threshold  *)
    UnicastPauseFrameDetect: longword;  (*!< Selects or not the MAC detection of the Pause frames (with MAC Address0
                                                           unicast address and unique multicast address).
                                                           This parameter can be a value of @ref ETH_Unicast_Pause_Frame_Detect  *)
    ReceiveFlowControl: longword;  (*!< Enables or disables the MAC to decode the received Pause frame and
                                                           disable its transmitter for a specified time (Pause Time)
                                                           This parameter can be a value of @ref ETH_Receive_Flow_Control  *)
    TransmitFlowControl: longword;  (*!< Enables or disables the MAC to transmit Pause frames (Full-Duplex mode)
                                                           or the MAC back-pressure operation (Half-Duplex mode)
                                                           This parameter can be a value of @ref ETH_Transmit_Flow_Control  *)
    VLANTagComparison: longword;  (*!< Selects the 12-bit VLAN identifier or the complete 16-bit VLAN tag for
                                                           comparison and filtering.
                                                           This parameter can be a value of @ref ETH_VLAN_Tag_Comparison  *)
    VLANTagIdentifier: longword;  (*!< Holds the VLAN tag identifier for receive frames  *)
  end;


  (**
  * @brief  ETH DMA Configuration Structure definition
   *)

  PETH_DMAInitTypeDef = ^ETH_DMAInitTypeDef;

  ETH_DMAInitTypeDef = record
    DropTCPIPChecksumErrorFrame: longword;  (*!< Selects or not the Dropping of TCP/IP Checksum Error Frames.
                                                             This parameter can be a value of @ref ETH_Drop_TCP_IP_Checksum_Error_Frame  *)
    ReceiveStoreForward: longword;  (*!< Enables or disables the Receive store and forward mode.
                                                             This parameter can be a value of @ref ETH_Receive_Store_Forward  *)
    FlushReceivedFrame: longword;  (*!< Enables or disables the flushing of received frames.
                                                             This parameter can be a value of @ref ETH_Flush_Received_Frame  *)
    TransmitStoreForward: longword;  (*!< Enables or disables Transmit store and forward mode.
                                                             This parameter can be a value of @ref ETH_Transmit_Store_Forward  *)
    TransmitThresholdControl: longword;  (*!< Selects or not the Transmit Threshold Control.
                                                             This parameter can be a value of @ref ETH_Transmit_Threshold_Control  *)
    ForwardErrorFrames: longword;  (*!< Selects or not the forward to the DMA of erroneous frames.
                                                             This parameter can be a value of @ref ETH_Forward_Error_Frames  *)
    ForwardUndersizedGoodFrames: longword;  (*!< Enables or disables the Rx FIFO to forward Undersized frames (frames with no Error
                                                             and length less than 64 bytes) including pad-bytes and CRC)
                                                             This parameter can be a value of @ref ETH_Forward_Undersized_Good_Frames  *)
    ReceiveThresholdControl: longword;  (*!< Selects the threshold level of the Receive FIFO.
                                                             This parameter can be a value of @ref ETH_Receive_Threshold_Control  *)
    SecondFrameOperate: longword;  (*!< Selects or not the Operate on second frame mode, which allows the DMA to process a second
                                                             frame of Transmit data even before obtaining the status for the first frame.
                                                             This parameter can be a value of @ref ETH_Second_Frame_Operate  *)
    AddressAlignedBeats: longword;  (*!< Enables or disables the Address Aligned Beats.
                                                             This parameter can be a value of @ref ETH_Address_Aligned_Beats  *)
    FixedBurst: longword;  (*!< Enables or disables the AHB Master interface fixed burst transfers.
                                                             This parameter can be a value of @ref ETH_Fixed_Burst  *)
    RxDMABurstLength: longword;  (*!< Indicates the maximum number of beats to be transferred in one Rx DMA transaction.
                                                             This parameter can be a value of @ref ETH_Rx_DMA_Burst_Length  *)
    TxDMABurstLength: longword;  (*!< Indicates the maximum number of beats to be transferred in one Tx DMA transaction.
                                                             This parameter can be a value of @ref ETH_Tx_DMA_Burst_Length  *)
    EnhancedDescriptorFormat: longword;  (*!< Enables the enhanced descriptor format.
                                                             This parameter can be a value of @ref ETH_DMA_Enhanced_descriptor_format  *)
    DescriptorSkipLength: longword;  (*!< Specifies the number of word to skip between two unchained descriptors (Ring mode)
                                                             This parameter must be a number between Min_Data = 0 and Max_Data = 32  *)
    DMAArbitration: longword;  (*!< Selects the DMA Tx/Rx arbitration.
                                                             This parameter can be a value of @ref ETH_DMA_Arbitration  *)
  end;


  (**
  * @brief  ETH DMA Descriptors data structure definition
   *)

  PETH_DMADescTypeDef = ^ETH_DMADescTypeDef;
  ETH_DMADescTypeDef = record
    Status: longword;  (*!< Status  *)
    ControlBufferSize: longword;  (*!< Control and Buffer1, Buffer2 lengths  *)
    Buffer1Addr: pbyte;  (*!< Buffer1 address pointer  *)
    Buffer2NextDescAddr: PETH_DMADescTypeDef;  (*!< Buffer2 or next descriptor address pointer  *)
    (*!< Enhanced ETHERNET DMA PTP Descriptors  *)
    ExtendedStatus: longword;  (*!< Extended status for PTP receive descriptor  *)
    Reserved1: longword;  (*!< Reserved  *)
    TimeStampLow: longword;  (*!< Time Stamp Low value for transmit and receive  *)
    TimeStampHigh: longword;  (*!< Time Stamp High value for transmit and receive  *)
  end;

  (**
  * @brief  Received Frame Informations structure definition
   *)
  ETH_DMARxFrameInfos = record
    FSRxDesc: PETH_DMADescTypeDef;  (*!< First Segment Rx Desc  *)
    LSRxDesc: PETH_DMADescTypeDef;  (*!< Last Segment Rx Desc  *)
    SegCount: longword;  (*!< Segment count  *)
    length: longword;  (*!< Frame length  *)
    buffer: PByte;  (*!< Frame buffer  *)
  end;


  (**
  * @brief  ETH Handle Structure definition
   *)

type
  ETH_HandleTypeDef = record
    Instance: ^ETH_TypeDef;  (*!< Register base address        *)
    Init: ETH_InitTypeDef;  (*!< Ethernet Init Configuration  *)
    LinkStatus: longword;  (*!< Ethernet link status         *)
    RxDesc: PETH_DMADescTypeDef;  (*!< Rx descriptor to Get         *)
    TxDesc: PETH_DMADescTypeDef;  (*!< Tx descriptor to Set         *)
    RxFrameInfos: ETH_DMARxFrameInfos;  (*!< last Rx frame infos          *)
    State: HAL_ETH_StateTypeDef;  (*!< ETH communication state      *)
    Lock: HAL_LockTypeDef;  (*!< ETH Lock                     *)
  end;

  (**
  * @}
   *)

(* Exported constants -------------------------------------------------------- *)

  (** @defgroup ETH_Exported_Constants ETH Exported Constants
  * @{
   *)

  (** @defgroup ETH_Buffers_setting ETH Buffers setting
  * @{
   *)

const
  ETH_MAX_PACKET_SIZE = 1524;  (*!< ETH_HEADER + ETH_EXTRA + ETH_VLAN_TAG + ETH_MAX_ETH_PAYLOAD + ETH_CRC  *)
  ETH_HEADER = 14;  (*!< 6 byte Dest addr, 6 byte Src addr, 2 byte length/type  *)
  ETH_CRC = 4;  (*!< Ethernet CRC  *)
  ETH_EXTRA = 2;  (*!< Extra bytes in some cases  *)
  ETH_VLAN_TAG = 4;  (*!< optional 802.1q VLAN Tag  *)
  ETH_MIN_ETH_PAYLOAD = 46;  (*!< Minimum Ethernet payload size  *)
  ETH_MAX_ETH_PAYLOAD = 1500;  (*!< Maximum Ethernet payload size  *)
  ETH_JUMBO_FRAME_PAYLOAD = 9000;  (*!< Jumbo frame payload size  *)
  (* Ethernet driver receive buffers are organized in a chained linked-list, when
    an ethernet packet is received, the Rx-DMA will transfer the packet from RxFIFO
    to the driver receive buffers memory.

    Depending on the size of the received ethernet packet and the size of
    each ethernet driver receive buffer, the received packet can take one or more
    ethernet driver receive buffer.

    In below are defined the size of one ethernet driver receive buffer ETH_RX_BUF_SIZE
    and the total count of the driver receive buffers ETH_RXBUFNB.

    The configured value for ETH_RX_BUF_SIZE and ETH_RXBUFNB are only provided as
    example, they can be reconfigured in the application layer to fit the application
    needs  *)

  (* Here we configure each Ethernet driver receive buffer to fit the Max size Ethernet
   packet  *)

  ETH_RX_BUF_SIZE = ETH_MAX_PACKET_SIZE;

  (* 5 Ethernet driver receive buffers are used (in a chained linked list) *)

  ETH_RXBUFNB = 5; (*  5 Rx buffers of size ETH_RX_BUF_SIZE  *)

  (* Ethernet driver transmit buffers are organized in a chained linked-list, when
    an ethernet packet is transmitted, Tx-DMA will transfer the packet from the
    driver transmit buffers memory to the TxFIFO.

    Depending on the size of the Ethernet packet to be transmitted and the size of
    each ethernet driver transmit buffer, the packet to be transmitted can take
    one or more ethernet driver transmit buffer.

    In below are defined the size of one ethernet driver transmit buffer ETH_TX_BUF_SIZE
    and the total count of the driver transmit buffers ETH_TXBUFNB.

    The configured value for ETH_TX_BUF_SIZE and ETH_TXBUFNB are only provided as
    example, they can be reconfigured in the application layer to fit the application
    needs  *)

  (* Here we configure each Ethernet driver transmit buffer to fit the Max size Ethernet
   packet  *)

  ETH_TX_BUF_SIZE = ETH_MAX_PACKET_SIZE;

  (* 5 ethernet driver transmit buffers are used (in a chained linked list) *)

  ETH_TXBUFNB = 5;  (* 5  Tx buffers of size ETH_TX_BUF_SIZE  *)

  (**
  * @}
   *)

  (** @defgroup ETH_DMA_TX_Descriptor ETH DMA TX Descriptor
  * @{
   *)

  (*
   DMA Tx Descriptor
  -----------------------------------------------------------------------------------------------
  TDES0 | OWN(31) | CTRL[30:26] | Reserved[25:24] | CTRL[23:20] | Reserved[19:17] | Status[16:0] |
  -----------------------------------------------------------------------------------------------
  TDES1 | Reserved[31:29] | Buffer2 ByteCount[28:16] | Reserved[15:13] | Buffer1 ByteCount[12:0] |
  -----------------------------------------------------------------------------------------------
  TDES2 |                         Buffer1 Address [31:0]                                         |
  -----------------------------------------------------------------------------------------------
  TDES3 |                   Buffer2 Address [31:0] / Next Descriptor Address [31:0]              |
  -----------------------------------------------------------------------------------------------
 *)

  (**
  * @brief  Bit definition of TDES0 register: DMA Tx descriptor status register
   *)

  ETH_DMATXDESC_OWN = $80000000;  (*!< OWN bit: descriptor is owned by DMA engine  *)
  ETH_DMATXDESC_IC = $40000000;  (*!< Interrupt on Completion  *)
  ETH_DMATXDESC_LS = $20000000;  (*!< Last Segment  *)
  ETH_DMATXDESC_FS = $10000000;  (*!< First Segment  *)
  ETH_DMATXDESC_DC = $08000000;  (*!< Disable CRC  *)
  ETH_DMATXDESC_DP = $04000000;  (*!< Disable Padding  *)
  ETH_DMATXDESC_TTSE = $02000000;  (*!< Transmit Time Stamp Enable  *)
  ETH_DMATXDESC_CIC = $00C00000;  (*!< Checksum Insertion Control: 4 cases  *)
  ETH_DMATXDESC_CIC_BYPASS = $00000000;  (*!< Do Nothing: Checksum Engine is bypassed  *)
  ETH_DMATXDESC_CIC_IPV4HEADER = $00400000;  (*!< IPV4 header Checksum Insertion  *)
  ETH_DMATXDESC_CIC_TCPUDPICMP_SEGMENT = $00800000;  (*!< TCP/UDP/ICMP Checksum Insertion calculated over segment only  *)
  ETH_DMATXDESC_CIC_TCPUDPICMP_FULL = $00C00000;  (*!< TCP/UDP/ICMP Checksum Insertion fully calculated  *)
  ETH_DMATXDESC_TER = $00200000;  (*!< Transmit End of Ring  *)
  ETH_DMATXDESC_TCH = $00100000;  (*!< Second Address Chained  *)
  ETH_DMATXDESC_TTSS = $00020000;  (*!< Tx Time Stamp Status  *)
  ETH_DMATXDESC_IHE = $00010000;  (*!< IP Header Error  *)
  ETH_DMATXDESC_ES = $00008000;  (*!< Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT  *)
  ETH_DMATXDESC_JT = $00004000;  (*!< Jabber Timeout  *)
  ETH_DMATXDESC_FF = $00002000;  (*!< Frame Flushed: DMA/MTL flushed the frame due to SW flush  *)
  ETH_DMATXDESC_PCE = $00001000;  (*!< Payload Checksum Error  *)
  ETH_DMATXDESC_LCA = $00000800;  (*!< Loss of Carrier: carrier lost during transmission  *)
  ETH_DMATXDESC_NC = $00000400;  (*!< No Carrier: no carrier signal from the transceiver  *)
  ETH_DMATXDESC_LCO = $00000200;  (*!< Late Collision: transmission aborted due to collision  *)
  ETH_DMATXDESC_EC = $00000100;  (*!< Excessive Collision: transmission aborted after 16 collisions  *)
  ETH_DMATXDESC_VF = $00000080;  (*!< VLAN Frame  *)
  ETH_DMATXDESC_CC = $00000078;  (*!< Collision Count  *)
  ETH_DMATXDESC_ED = $00000004;  (*!< Excessive Deferral  *)
  ETH_DMATXDESC_UF = $00000002;  (*!< Underflow Error: late data arrival from the memory  *)
  ETH_DMATXDESC_DB = $00000001;  (*!< Deferred Bit  *)
  (**
  * @brief  Bit definition of TDES1 register
   *)

  ETH_DMATXDESC_TBS2 = $1FFF0000;  (*!< Transmit Buffer2 Size  *)
  ETH_DMATXDESC_TBS1 = $00001FFF;  (*!< Transmit Buffer1 Size  *)
  (**
  * @brief  Bit definition of TDES2 register
   *)

  ETH_DMATXDESC_B1AP = $FFFFFFFF;  (*!< Buffer1 Address Pointer  *)
  (**
  * @brief  Bit definition of TDES3 register
   *)

  ETH_DMATXDESC_B2AP = $FFFFFFFF;  (*!< Buffer2 Address Pointer  *)
  (*---------------------------------------------------------------------------------------------
  TDES6 |                         Transmit Time Stamp Low [31:0]                                 |
  -----------------------------------------------------------------------------------------------
  TDES7 |                         Transmit Time Stamp High [31:0]                                |
  ---------------------------------------------------------------------------------------------- *)

  (* Bit definition of TDES6 register  *)

  ETH_DMAPTPTXDESC_TTSL = $FFFFFFFF;  (* Transmit Time Stamp Low  *)
  (* Bit definition of TDES7 register  *)

  ETH_DMAPTPTXDESC_TTSH = $FFFFFFFF;  (* Transmit Time Stamp High  *)
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_RX_Descriptor ETH DMA RX Descriptor
  * @{
   *)

  (*
  DMA Rx Descriptor
  --------------------------------------------------------------------------------------------------------------------
  RDES0 | OWN(31) |                                             Status [30:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES1 | CTRL(31) | Reserved[30:29] | Buffer2 ByteCount[28:16] | CTRL[15:14] | Reserved(13) | Buffer1 ByteCount[12:0] |
  ---------------------------------------------------------------------------------------------------------------------
  RDES2 |                                       Buffer1 Address [31:0]                                                 |
  ---------------------------------------------------------------------------------------------------------------------
  RDES3 |                          Buffer2 Address [31:0] / Next Descriptor Address [31:0]                             |
  ---------------------------------------------------------------------------------------------------------------------
 *)

  (**
  * @brief  Bit definition of RDES0 register: DMA Rx descriptor status register
   *)

  ETH_DMARXDESC_OWN = $80000000;  (*!< OWN bit: descriptor is owned by DMA engine   *)
  ETH_DMARXDESC_AFM = $40000000;  (*!< DA Filter Fail for the rx frame   *)
  ETH_DMARXDESC_FL = $3FFF0000;  (*!< Receive descriptor frame length   *)
  ETH_DMARXDESC_ES = $00008000;  (*!< Error summary: OR of the following bits: DE || OE || IPC || LC || RWT || RE || CE  *)
  ETH_DMARXDESC_DE = $00004000;  (*!< Descriptor error: no more descriptors for receive frame   *)
  ETH_DMARXDESC_SAF = $00002000;  (*!< SA Filter Fail for the received frame  *)
  ETH_DMARXDESC_LE = $00001000;  (*!< Frame size not matching with length field  *)
  ETH_DMARXDESC_OE = $00000800;  (*!< Overflow Error: Frame was damaged due to buffer overflow  *)
  ETH_DMARXDESC_VLAN = $00000400;  (*!< VLAN Tag: received frame is a VLAN frame  *)
  ETH_DMARXDESC_FS = $00000200;  (*!< First descriptor of the frame   *)
  ETH_DMARXDESC_LS = $00000100;  (*!< Last descriptor of the frame   *)
  ETH_DMARXDESC_IPV4HCE = $00000080;  (*!< IPC Checksum Error: Rx Ipv4 header checksum error    *)
  ETH_DMARXDESC_LC = $00000040;  (*!< Late collision occurred during reception    *)
  ETH_DMARXDESC_FT = $00000020;  (*!< Frame type - Ethernet, otherwise 802.3     *)
  ETH_DMARXDESC_RWT = $00000010;  (*!< Receive Watchdog Timeout: watchdog timer expired during reception     *)
  ETH_DMARXDESC_RE = $00000008;  (*!< Receive error: error reported by MII interface   *)
  ETH_DMARXDESC_DBE = $00000004;  (*!< Dribble bit error: frame contains non int multiple of 8 bits   *)
  ETH_DMARXDESC_CE = $00000002;  (*!< CRC error  *)
  ETH_DMARXDESC_MAMPCE = $00000001;  (*!< Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error  *)
  (**
  * @brief  Bit definition of RDES1 register
   *)

  ETH_DMARXDESC_DIC = $80000000;  (*!< Disable Interrupt on Completion  *)
  ETH_DMARXDESC_RBS2 = $1FFF0000;  (*!< Receive Buffer2 Size  *)
  ETH_DMARXDESC_RER = $00008000;  (*!< Receive End of Ring  *)
  ETH_DMARXDESC_RCH = $00004000;  (*!< Second Address Chained  *)
  ETH_DMARXDESC_RBS1 = $00001FFF;  (*!< Receive Buffer1 Size  *)
  (**
  * @brief  Bit definition of RDES2 register
   *)

  ETH_DMARXDESC_B1AP = $FFFFFFFF;  (*!< Buffer1 Address Pointer  *)
  (**
  * @brief  Bit definition of RDES3 register
   *)

  ETH_DMARXDESC_B2AP = $FFFFFFFF;  (*!< Buffer2 Address Pointer  *)
  (*---------------------------------------------------------------------------------------------------------------------
  RDES4 |                   Reserved[31:15]              |             Extended Status [14:0]                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES5 |                                            Reserved[31:0]                                                    |
  ---------------------------------------------------------------------------------------------------------------------
  RDES6 |                                       Receive Time Stamp Low [31:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES7 |                                       Receive Time Stamp High [31:0]                                         |
  -------------------------------------------------------------------------------------------------------------------- *)

  (* Bit definition of RDES4 register  *)

  ETH_DMAPTPRXDESC_PTPV = $00002000;  (* PTP Version  *)
  ETH_DMAPTPRXDESC_PTPFT = $00001000;  (* PTP Frame Type  *)
  ETH_DMAPTPRXDESC_PTPMT = $00000F00;  (* PTP Message Type  *)
  ETH_DMAPTPRXDESC_PTPMT_SYNC = $00000100;  (* SYNC message (all clock types)  *)
  ETH_DMAPTPRXDESC_PTPMT_FOLLOWUP = $00000200;  (* FollowUp message (all clock types)  *)
  ETH_DMAPTPRXDESC_PTPMT_DELAYREQ = $00000300;  (* DelayReq message (all clock types)  *)
  ETH_DMAPTPRXDESC_PTPMT_DELAYRESP = $00000400;  (* DelayResp message (all clock types)  *)
  ETH_DMAPTPRXDESC_PTPMT_PDELAYREQ_ANNOUNCE = $00000500;  (* PdelayReq message (peer-to-peer transparent clock) or Announce message (Ordinary or Boundary clock)  *)
  ETH_DMAPTPRXDESC_PTPMT_PDELAYRESP_MANAG = $00000600;  (* PdelayResp message (peer-to-peer transparent clock) or Management message (Ordinary or Boundary clock)   *)
  ETH_DMAPTPRXDESC_PTPMT_PDELAYRESPFOLLOWUP_SIGNAL = $00000700;  (* PdelayRespFollowUp message (peer-to-peer transparent clock) or Signaling message (Ordinary or Boundary clock)  *)
  ETH_DMAPTPRXDESC_IPV6PR = $00000080;  (* IPv6 Packet Received  *)
  ETH_DMAPTPRXDESC_IPV4PR = $00000040;  (* IPv4 Packet Received  *)
  ETH_DMAPTPRXDESC_IPCB = $00000020;  (* IP Checksum Bypassed  *)
  ETH_DMAPTPRXDESC_IPPE = $00000010;  (* IP Payload Error  *)
  ETH_DMAPTPRXDESC_IPHE = $00000008;  (* IP Header Error  *)
  ETH_DMAPTPRXDESC_IPPT = $00000007;  (* IP Payload Type  *)
  ETH_DMAPTPRXDESC_IPPT_UDP = $00000001;  (* UDP payload encapsulated in the IP datagram  *)
  ETH_DMAPTPRXDESC_IPPT_TCP = $00000002;  (* TCP payload encapsulated in the IP datagram  *)
  ETH_DMAPTPRXDESC_IPPT_ICMP = $00000003;  (* ICMP payload encapsulated in the IP datagram  *)
  (* Bit definition of RDES6 register  *)

  ETH_DMAPTPRXDESC_RTSL = $FFFFFFFF;  (* Receive Time Stamp Low  *)
  (* Bit definition of RDES7 register  *)

  ETH_DMAPTPRXDESC_RTSH = $FFFFFFFF;  (* Receive Time Stamp High  *)
  (**
  * @}
   *)

  (** @defgroup ETH_AutoNegotiation ETH AutoNegotiation
  * @{
   *)

  ETH_AUTONEGOTIATION_ENABLE = $00000001;
  ETH_AUTONEGOTIATION_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Speed ETH Speed
  * @{
   *)

  ETH_SPEED_10M = $00000000;
  ETH_SPEED_100M = $00004000;
  (**
  * @}
   *)

  (** @defgroup ETH_Duplex_Mode ETH Duplex Mode
  * @{
   *)

  ETH_MODE_FULLDUPLEX = $00000800;
  ETH_MODE_HALFDUPLEX = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Rx_Mode ETH Rx Mode
  * @{
   *)

  ETH_RXPOLLING_MODE = $00000000;
  ETH_RXINTERRUPT_MODE = $00000001;
  (**
  * @}
   *)

  (** @defgroup ETH_Checksum_Mode ETH Checksum Mode
  * @{
   *)

  ETH_CHECKSUM_BY_HARDWARE = $00000000;
  ETH_CHECKSUM_BY_SOFTWARE = $00000001;
  (**
  * @}
   *)

  (** @defgroup ETH_Media_Interface ETH Media Interface
  * @{
   *)

  ETH_MEDIA_INTERFACE_MII = $00000000;
  ETH_MEDIA_INTERFACE_RMII = SYSCFG_PMC_MII_RMII_SEL;
  (**
  * @}
   *)

  (** @defgroup ETH_Watchdog ETH Watchdog
  * @{
   *)

  ETH_WATCHDOG_ENABLE = $00000000;
  ETH_WATCHDOG_DISABLE = $00800000;
  (**
  * @}
   *)

  (** @defgroup ETH_Jabber ETH Jabber
  * @{
   *)

  ETH_JABBER_ENABLE = $00000000;
  ETH_JABBER_DISABLE = $00400000;
  (**
  * @}
   *)

  (** @defgroup ETH_Inter_Frame_Gap ETH Inter Frame Gap
  * @{
   *)

  ETH_INTERFRAMEGAP_96BIT = $00000000;  (*!< minimum IFG between frames during transmission is 96Bit  *)
  ETH_INTERFRAMEGAP_88BIT = $00020000;  (*!< minimum IFG between frames during transmission is 88Bit  *)
  ETH_INTERFRAMEGAP_80BIT = $00040000;  (*!< minimum IFG between frames during transmission is 80Bit  *)
  ETH_INTERFRAMEGAP_72BIT = $00060000;  (*!< minimum IFG between frames during transmission is 72Bit  *)
  ETH_INTERFRAMEGAP_64BIT = $00080000;  (*!< minimum IFG between frames during transmission is 64Bit  *)
  ETH_INTERFRAMEGAP_56BIT = $000A0000;  (*!< minimum IFG between frames during transmission is 56Bit  *)
  ETH_INTERFRAMEGAP_48BIT = $000C0000;  (*!< minimum IFG between frames during transmission is 48Bit  *)
  ETH_INTERFRAMEGAP_40BIT = $000E0000;  (*!< minimum IFG between frames during transmission is 40Bit  *)
  (**
  * @}
   *)

  (** @defgroup ETH_Carrier_Sense ETH Carrier Sense
  * @{
   *)

  ETH_CARRIERSENCE_ENABLE = $00000000;
  ETH_CARRIERSENCE_DISABLE = $00010000;
  (**
  * @}
   *)

  (** @defgroup ETH_Receive_Own ETH Receive Own
  * @{
   *)

  ETH_RECEIVEOWN_ENABLE = $00000000;
  ETH_RECEIVEOWN_DISABLE = $00002000;
  (**
  * @}
   *)

  (** @defgroup ETH_Loop_Back_Mode ETH Loop Back Mode
  * @{
   *)

  ETH_LOOPBACKMODE_ENABLE = $00001000;
  ETH_LOOPBACKMODE_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Checksum_Offload ETH Checksum Offload
  * @{
   *)

  ETH_CHECKSUMOFFLAOD_ENABLE = $00000400;
  ETH_CHECKSUMOFFLAOD_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Retry_Transmission ETH Retry Transmission
  * @{
   *)

  ETH_RETRYTRANSMISSION_ENABLE = $00000000;
  ETH_RETRYTRANSMISSION_DISABLE = $00000200;
  (**
  * @}
   *)

  (** @defgroup ETH_Automatic_Pad_CRC_Strip ETH Automatic Pad CRC Strip
  * @{
   *)

  ETH_AUTOMATICPADCRCSTRIP_ENABLE = $00000080;
  ETH_AUTOMATICPADCRCSTRIP_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Back_Off_Limit ETH Back Off Limit
  * @{
   *)

  ETH_BACKOFFLIMIT_10 = $00000000;
  ETH_BACKOFFLIMIT_8 = $00000020;
  ETH_BACKOFFLIMIT_4 = $00000040;
  ETH_BACKOFFLIMIT_1 = $00000060;
  (**
  * @}
   *)

  (** @defgroup ETH_Deferral_Check ETH Deferral Check
  * @{
   *)

  ETH_DEFFERRALCHECK_ENABLE = $00000010;
  ETH_DEFFERRALCHECK_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Receive_All ETH Receive All
  * @{
   *)

  ETH_RECEIVEALL_ENABLE = $80000000;
  ETH_RECEIVEAll_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Source_Addr_Filter ETH Source Addr Filter
  * @{
   *)

  ETH_SOURCEADDRFILTER_NORMAL_ENABLE = $00000200;
  ETH_SOURCEADDRFILTER_INVERSE_ENABLE = $00000300;
  ETH_SOURCEADDRFILTER_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Pass_Control_Frames ETH Pass Control Frames
  * @{
   *)

  ETH_PASSCONTROLFRAMES_BLOCKALL = $00000040;  (*!< MAC filters all control frames from reaching the application  *)
  ETH_PASSCONTROLFRAMES_FORWARDALL = $00000080;  (*!< MAC forwards all control frames to application even if they fail the Address Filter  *)
  ETH_PASSCONTROLFRAMES_FORWARDPASSEDADDRFILTER = $000000C0;  (*!< MAC forwards control frames that pass the Address Filter.  *)
  (**
  * @}
   *)

  (** @defgroup ETH_Broadcast_Frames_Reception ETH Broadcast Frames Reception
  * @{
   *)

  ETH_BROADCASTFRAMESRECEPTION_ENABLE = $00000000;
  ETH_BROADCASTFRAMESRECEPTION_DISABLE = $00000020;
  (**
  * @}
   *)

  (** @defgroup ETH_Destination_Addr_Filter ETH Destination Addr Filter
  * @{
   *)

  ETH_DESTINATIONADDRFILTER_NORMAL = $00000000;
  ETH_DESTINATIONADDRFILTER_INVERSE = $00000008;
  (**
  * @}
   *)

  (** @defgroup ETH_Promiscuous_Mode ETH Promiscuous Mode
  * @{
   *)

  ETH_PROMISCUOUS_MODE_ENABLE = $00000001;
  ETH_PROMISCUOUS_MODE_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Multicast_Frames_Filter ETH Multicast Frames Filter
  * @{
   *)

  ETH_MULTICASTFRAMESFILTER_PERFECTHASHTABLE = $00000404;
  ETH_MULTICASTFRAMESFILTER_HASHTABLE = $00000004;
  ETH_MULTICASTFRAMESFILTER_PERFECT = $00000000;
  ETH_MULTICASTFRAMESFILTER_NONE = $00000010;
  (**
  * @}
   *)

  (** @defgroup ETH_Unicast_Frames_Filter ETH Unicast Frames Filter
  * @{
   *)

  ETH_UNICASTFRAMESFILTER_PERFECTHASHTABLE = $00000402;
  ETH_UNICASTFRAMESFILTER_HASHTABLE = $00000002;
  ETH_UNICASTFRAMESFILTER_PERFECT = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Zero_Quanta_Pause ETH Zero Quanta Pause
  * @{
   *)

  ETH_ZEROQUANTAPAUSE_ENABLE = $00000000;
  ETH_ZEROQUANTAPAUSE_DISABLE = $00000080;
  (**
  * @}
   *)

  (** @defgroup ETH_Pause_Low_Threshold ETH Pause Low Threshold
  * @{
   *)

  ETH_PAUSELOWTHRESHOLD_MINUS4 = $00000000;  (*!< Pause time minus 4 slot times  *)
  ETH_PAUSELOWTHRESHOLD_MINUS28 = $00000010;  (*!< Pause time minus 28 slot times  *)
  ETH_PAUSELOWTHRESHOLD_MINUS144 = $00000020;  (*!< Pause time minus 144 slot times  *)
  ETH_PAUSELOWTHRESHOLD_MINUS256 = $00000030;  (*!< Pause time minus 256 slot times  *)
  (**
  * @}
   *)

  (** @defgroup ETH_Unicast_Pause_Frame_Detect ETH Unicast Pause Frame Detect
  * @{
   *)

  ETH_UNICASTPAUSEFRAMEDETECT_ENABLE = $00000008;
  ETH_UNICASTPAUSEFRAMEDETECT_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Receive_Flow_Control ETH Receive Flow Control
  * @{
   *)

  ETH_RECEIVEFLOWCONTROL_ENABLE = $00000004;
  ETH_RECEIVEFLOWCONTROL_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Transmit_Flow_Control ETH Transmit Flow Control
  * @{
   *)

  ETH_TRANSMITFLOWCONTROL_ENABLE = $00000002;
  ETH_TRANSMITFLOWCONTROL_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_VLAN_Tag_Comparison ETH VLAN Tag Comparison
  * @{
   *)

  ETH_VLANTAGCOMPARISON_12BIT = $00010000;
  ETH_VLANTAGCOMPARISON_16BIT = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_MAC_addresses ETH MAC addresses
  * @{
   *)

  ETH_MAC_ADDRESS0 = $00000000;
  ETH_MAC_ADDRESS1 = $00000008;
  ETH_MAC_ADDRESS2 = $00000010;
  ETH_MAC_ADDRESS3 = $00000018;
  (**
  * @}
   *)

  (** @defgroup ETH_MAC_addresses_filter_SA_DA ETH MAC addresses filter SA DA
  * @{
   *)

  ETH_MAC_ADDRESSFILTER_SA = $00000000;
  ETH_MAC_ADDRESSFILTER_DA = $00000008;
  (**
  * @}
   *)

  (** @defgroup ETH_MAC_addresses_filter_Mask_bytes ETH MAC addresses filter Mask bytes
  * @{
   *)

  ETH_MAC_ADDRESSMASK_BYTE6 = $20000000;  (*!< Mask MAC Address high reg bits [15:8]  *)
  ETH_MAC_ADDRESSMASK_BYTE5 = $10000000;  (*!< Mask MAC Address high reg bits [7:0]  *)
  ETH_MAC_ADDRESSMASK_BYTE4 = $08000000;  (*!< Mask MAC Address low reg bits [31:24]  *)
  ETH_MAC_ADDRESSMASK_BYTE3 = $04000000;  (*!< Mask MAC Address low reg bits [23:16]  *)
  ETH_MAC_ADDRESSMASK_BYTE2 = $02000000;  (*!< Mask MAC Address low reg bits [15:8]  *)
  ETH_MAC_ADDRESSMASK_BYTE1 = $01000000;  (*!< Mask MAC Address low reg bits [70]  *)
  (**
  * @}
   *)

  (** @defgroup ETH_MAC_Debug_flags ETH MAC Debug flags
  * @{
   *)

  ETH_MAC_TXFIFO_FULL = $02000000;  (* Tx FIFO full  *)
  ETH_MAC_TXFIFONOT_EMPTY = $01000000;  (* Tx FIFO not empty  *)
  ETH_MAC_TXFIFO_WRITE_ACTIVE = $00400000;  (* Tx FIFO write active  *)
  ETH_MAC_TXFIFO_IDLE = $00000000;  (* Tx FIFO read status: Idle  *)
  ETH_MAC_TXFIFO_READ = $00100000;  (* Tx FIFO read status: Read (transferring data to the MAC transmitter)  *)
  ETH_MAC_TXFIFO_WAITING = $00200000;  (* Tx FIFO read status: Waiting for TxStatus from MAC transmitter  *)
  ETH_MAC_TXFIFO_WRITING = $00300000;  (* Tx FIFO read status: Writing the received TxStatus or flushing the TxFIFO  *)
  ETH_MAC_TRANSMISSION_PAUSE = $00080000;  (* MAC transmitter in pause  *)
  ETH_MAC_TRANSMITFRAMECONTROLLER_IDLE = $00000000;  (* MAC transmit frame controller: Idle  *)
  ETH_MAC_TRANSMITFRAMECONTROLLER_WAITING = $00020000;  (* MAC transmit frame controller: Waiting for Status of previous frame or IFG/backoff period to be over  *)
  ETH_MAC_TRANSMITFRAMECONTROLLER_GENRATING_PCF = $00040000;  (* MAC transmit frame controller: Generating and transmitting a Pause control frame (in full duplex mode)  *)
  ETH_MAC_TRANSMITFRAMECONTROLLER_TRANSFERRING = $00060000;  (* MAC transmit frame controller: Transferring input frame for transmission  *)
  ETH_MAC_MII_TRANSMIT_ACTIVE = $00010000;  (* MAC MII transmit engine active  *)
  ETH_MAC_RXFIFO_EMPTY = $00000000;  (* Rx FIFO fill level: empty  *)
  ETH_MAC_RXFIFO_BELOW_THRESHOLD = $00000100;  (* Rx FIFO fill level: fill-level below flow-control de-activate threshold  *)
  ETH_MAC_RXFIFO_ABOVE_THRESHOLD = $00000200;  (* Rx FIFO fill level: fill-level above flow-control activate threshold  *)
  ETH_MAC_RXFIFO_FULL = $00000300;  (* Rx FIFO fill level: full  *)
  ETH_MAC_READCONTROLLER_IDLE = $00000000;  (* Rx FIFO read controller IDLE state  *)
  ETH_MAC_READCONTROLLER_READING_DATA = $00000020;  (* Rx FIFO read controller Reading frame data  *)
  ETH_MAC_READCONTROLLER_READING_STATUS = $00000040;  (* Rx FIFO read controller Reading frame status (or time-stamp)  *)
  ETH_MAC_READCONTROLLER_FLUSHING = $00000060;  (* Rx FIFO read controller Flushing the frame data and status  *)
  ETH_MAC_RXFIFO_WRITE_ACTIVE = $00000010;  (* Rx FIFO write controller active  *)
  ETH_MAC_SMALL_FIFO_NOTACTIVE = $00000000;  (* MAC small FIFO read / write controllers not active  *)
  ETH_MAC_SMALL_FIFO_READ_ACTIVE = $00000002;  (* MAC small FIFO read controller active  *)
  ETH_MAC_SMALL_FIFO_WRITE_ACTIVE = $00000004;  (* MAC small FIFO write controller active  *)
  ETH_MAC_SMALL_FIFO_RW_ACTIVE = $00000006;  (* MAC small FIFO read / write controllers active  *)
  ETH_MAC_MII_RECEIVE_PROTOCOL_ACTIVE = $00000001;  (* MAC MII receive protocol engine active  *)
  (**
  * @}
   *)

  (** @defgroup ETH_Drop_TCP_IP_Checksum_Error_Frame ETH Drop TCP IP Checksum Error Frame
  * @{
   *)

  ETH_DROPTCPIPCHECKSUMERRORFRAME_ENABLE = $00000000;
  ETH_DROPTCPIPCHECKSUMERRORFRAME_DISABLE = $04000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Receive_Store_Forward ETH Receive Store Forward
  * @{
   *)

  ETH_RECEIVESTOREFORWARD_ENABLE = $02000000;
  ETH_RECEIVESTOREFORWARD_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Flush_Received_Frame ETH Flush Received Frame
  * @{
   *)

  ETH_FLUSHRECEIVEDFRAME_ENABLE = $00000000;
  ETH_FLUSHRECEIVEDFRAME_DISABLE = $01000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Transmit_Store_Forward ETH Transmit Store Forward
  * @{
   *)

  ETH_TRANSMITSTOREFORWARD_ENABLE = $00200000;
  ETH_TRANSMITSTOREFORWARD_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Transmit_Threshold_Control ETH Transmit Threshold Control
  * @{
   *)

  ETH_TRANSMITTHRESHOLDCONTROL_64BYTES = $00000000;  (*!< threshold level of the MTL Transmit FIFO is 64 Bytes  *)
  ETH_TRANSMITTHRESHOLDCONTROL_128BYTES = $00004000;  (*!< threshold level of the MTL Transmit FIFO is 128 Bytes  *)
  ETH_TRANSMITTHRESHOLDCONTROL_192BYTES = $00008000;  (*!< threshold level of the MTL Transmit FIFO is 192 Bytes  *)
  ETH_TRANSMITTHRESHOLDCONTROL_256BYTES = $0000C000;  (*!< threshold level of the MTL Transmit FIFO is 256 Bytes  *)
  ETH_TRANSMITTHRESHOLDCONTROL_40BYTES = $00010000;  (*!< threshold level of the MTL Transmit FIFO is 40 Bytes  *)
  ETH_TRANSMITTHRESHOLDCONTROL_32BYTES = $00014000;  (*!< threshold level of the MTL Transmit FIFO is 32 Bytes  *)
  ETH_TRANSMITTHRESHOLDCONTROL_24BYTES = $00018000;  (*!< threshold level of the MTL Transmit FIFO is 24 Bytes  *)
  ETH_TRANSMITTHRESHOLDCONTROL_16BYTES = $0001C000;  (*!< threshold level of the MTL Transmit FIFO is 16 Bytes  *)
  (**
  * @}
   *)

  (** @defgroup ETH_Forward_Error_Frames ETH Forward Error Frames
  * @{
   *)

  ETH_FORWARDERRORFRAMES_ENABLE = $00000080;
  ETH_FORWARDERRORFRAMES_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Forward_Undersized_Good_Frames ETH Forward Undersized Good Frames
  * @{
   *)

  ETH_FORWARDUNDERSIZEDGOODFRAMES_ENABLE = $00000040;
  ETH_FORWARDUNDERSIZEDGOODFRAMES_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Receive_Threshold_Control ETH Receive Threshold Control
  * @{
   *)

  ETH_RECEIVEDTHRESHOLDCONTROL_64BYTES = $00000000;  (*!< threshold level of the MTL Receive FIFO is 64 Bytes  *)
  ETH_RECEIVEDTHRESHOLDCONTROL_32BYTES = $00000008;  (*!< threshold level of the MTL Receive FIFO is 32 Bytes  *)
  ETH_RECEIVEDTHRESHOLDCONTROL_96BYTES = $00000010;  (*!< threshold level of the MTL Receive FIFO is 96 Bytes  *)
  ETH_RECEIVEDTHRESHOLDCONTROL_128BYTES = $00000018;  (*!< threshold level of the MTL Receive FIFO is 128 Bytes  *)
  (**
  * @}
   *)

  (** @defgroup ETH_Second_Frame_Operate ETH Second Frame Operate
  * @{
   *)

  ETH_SECONDFRAMEOPERARTE_ENABLE = $00000004;
  ETH_SECONDFRAMEOPERARTE_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Address_Aligned_Beats ETH Address Aligned Beats
  * @{
   *)

  ETH_ADDRESSALIGNEDBEATS_ENABLE = $02000000;
  ETH_ADDRESSALIGNEDBEATS_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Fixed_Burst ETH Fixed Burst
  * @{
   *)

  ETH_FIXEDBURST_ENABLE = $00010000;
  ETH_FIXEDBURST_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_Rx_DMA_Burst_Length ETH Rx DMA Burst Length
  * @{
   *)

  ETH_RXDMABURSTLENGTH_1BEAT = $00020000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 1  *)
  ETH_RXDMABURSTLENGTH_2BEAT = $00040000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 2  *)
  ETH_RXDMABURSTLENGTH_4BEAT = $00080000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 4  *)
  ETH_RXDMABURSTLENGTH_8BEAT = $00100000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 8  *)
  ETH_RXDMABURSTLENGTH_16BEAT = $00200000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 16  *)
  ETH_RXDMABURSTLENGTH_32BEAT = $00400000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 32  *)
  ETH_RXDMABURSTLENGTH_4XPBL_4BEAT = $01020000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 4  *)
  ETH_RXDMABURSTLENGTH_4XPBL_8BEAT = $01040000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 8  *)
  ETH_RXDMABURSTLENGTH_4XPBL_16BEAT = $01080000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 16  *)
  ETH_RXDMABURSTLENGTH_4XPBL_32BEAT = $01100000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 32  *)
  ETH_RXDMABURSTLENGTH_4XPBL_64BEAT = $01200000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 64  *)
  ETH_RXDMABURSTLENGTH_4XPBL_128BEAT = $01400000;  (*!< maximum number of beats to be transferred in one RxDMA transaction is 128  *)
  (**
  * @}
   *)

  (** @defgroup ETH_Tx_DMA_Burst_Length ETH Tx DMA Burst Length
  * @{
   *)

  ETH_TXDMABURSTLENGTH_1BEAT = $00000100;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 1  *)
  ETH_TXDMABURSTLENGTH_2BEAT = $00000200;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 2  *)
  ETH_TXDMABURSTLENGTH_4BEAT = $00000400;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 4  *)
  ETH_TXDMABURSTLENGTH_8BEAT = $00000800;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 8  *)
  ETH_TXDMABURSTLENGTH_16BEAT = $00001000;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 16  *)
  ETH_TXDMABURSTLENGTH_32BEAT = $00002000;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 32  *)
  ETH_TXDMABURSTLENGTH_4XPBL_4BEAT = $01000100;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 4  *)
  ETH_TXDMABURSTLENGTH_4XPBL_8BEAT = $01000200;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 8  *)
  ETH_TXDMABURSTLENGTH_4XPBL_16BEAT = $01000400;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 16  *)
  ETH_TXDMABURSTLENGTH_4XPBL_32BEAT = $01000800;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 32  *)
  ETH_TXDMABURSTLENGTH_4XPBL_64BEAT = $01001000;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 64  *)
  ETH_TXDMABURSTLENGTH_4XPBL_128BEAT = $01002000;  (*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 128  *)
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_Enhanced_descriptor_format ETH DMA Enhanced descriptor format
  * @{
   *)

  ETH_DMAENHANCEDDESCRIPTOR_ENABLE = $00000080;
  ETH_DMAENHANCEDDESCRIPTOR_DISABLE = $00000000;
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_Arbitration ETH DMA Arbitration
  * @{
   *)

  ETH_DMAARBITRATION_ROUNDROBIN_RXTX_1_1 = $00000000;
  ETH_DMAARBITRATION_ROUNDROBIN_RXTX_2_1 = $00004000;
  ETH_DMAARBITRATION_ROUNDROBIN_RXTX_3_1 = $00008000;
  ETH_DMAARBITRATION_ROUNDROBIN_RXTX_4_1 = $0000C000;
  ETH_DMAARBITRATION_RXPRIORTX = $00000002;
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_Tx_descriptor_segment ETH DMA Tx descriptor segment
  * @{
   *)

  ETH_DMATXDESC_LASTSEGMENTS = $40000000;  (*!< Last Segment  *)
  ETH_DMATXDESC_FIRSTSEGMENT = $20000000;  (*!< First Segment  *)
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_Tx_descriptor_Checksum_Insertion_Control ETH DMA Tx descriptor Checksum Insertion Control
  * @{
   *)

  ETH_DMATXDESC_CHECKSUMBYPASS = $00000000;  (*!< Checksum engine bypass  *)
  ETH_DMATXDESC_CHECKSUMIPV4HEADER = $00400000;  (*!< IPv4 header checksum insertion   *)
  ETH_DMATXDESC_CHECKSUMTCPUDPICMPSEGMENT = $00800000;  (*!< TCP/UDP/ICMP checksum insertion. Pseudo header checksum is assumed to be present  *)
  ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL = $00C00000;  (*!< TCP/UDP/ICMP checksum fully in hardware including pseudo header  *)
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_Rx_descriptor_buffers ETH DMA Rx descriptor buffers
  * @{
   *)

  ETH_DMARXDESC_BUFFER1 = $00000000;  (*!< DMA Rx Desc Buffer1  *)
  ETH_DMARXDESC_BUFFER2 = $00000001;  (*!< DMA Rx Desc Buffer2  *)
  (**
  * @}
   *)

  (** @defgroup ETH_PMT_Flags ETH PMT Flags
  * @{
   *)

  ETH_PMT_FLAG_WUFFRPR = $80000000;  (*!< Wake-Up Frame Filter Register Pointer Reset  *)
  ETH_PMT_FLAG_WUFR = $00000040;  (*!< Wake-Up Frame Received  *)
  ETH_PMT_FLAG_MPR = $00000020;  (*!< Magic Packet Received  *)
  (**
  * @}
   *)

  (** @defgroup ETH_MMC_Tx_Interrupts ETH MMC Tx Interrupts
  * @{
   *)

  ETH_MMC_IT_TGF = $00200000;  (*!< When Tx good frame counter reaches half the maximum value  *)
  ETH_MMC_IT_TGFMSC = $00008000;  (*!< When Tx good multi col counter reaches half the maximum value  *)
  ETH_MMC_IT_TGFSC = $00004000;  (*!< When Tx good single col counter reaches half the maximum value  *)
  (**
  * @}
   *)

  (** @defgroup ETH_MMC_Rx_Interrupts ETH MMC Rx Interrupts
  * @{
   *)

  ETH_MMC_IT_RGUF = $10020000;  (*!< When Rx good unicast frames counter reaches half the maximum value  *)
  ETH_MMC_IT_RFAE = $10000040;  (*!< When Rx alignment error counter reaches half the maximum value  *)
  ETH_MMC_IT_RFCE = $10000020;  (*!< When Rx crc error counter reaches half the maximum value  *)
  (**
  * @}
   *)

  (** @defgroup ETH_MAC_Flags ETH MAC Flags
  * @{
   *)

  ETH_MAC_FLAG_TST = $00000200;  (*!< Time stamp trigger flag (on MAC)  *)
  ETH_MAC_FLAG_MMCT = $00000040;  (*!< MMC transmit flag   *)
  ETH_MAC_FLAG_MMCR = $00000020;  (*!< MMC receive flag  *)
  ETH_MAC_FLAG_MMC = $00000010;  (*!< MMC flag (on MAC)  *)
  ETH_MAC_FLAG_PMT = $00000008;  (*!< PMT flag (on MAC)  *)
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_Flags ETH DMA Flags
  * @{
   *)

  ETH_DMA_FLAG_TST = $20000000;  (*!< Time-stamp trigger interrupt (on DMA)  *)
  ETH_DMA_FLAG_PMT = $10000000;  (*!< PMT interrupt (on DMA)  *)
  ETH_DMA_FLAG_MMC = $08000000;  (*!< MMC interrupt (on DMA)  *)
  ETH_DMA_FLAG_DATATRANSFERERROR = $00800000;  (*!< Error bits 0-Rx DMA, 1-Tx DMA  *)
  ETH_DMA_FLAG_READWRITEERROR = $01000000;  (*!< Error bits 0-write transfer, 1-read transfer  *)
  ETH_DMA_FLAG_ACCESSERROR = $02000000;  (*!< Error bits 0-data buffer, 1-desc. access  *)
  ETH_DMA_FLAG_NIS = $00010000;  (*!< Normal interrupt summary flag  *)
  ETH_DMA_FLAG_AIS = $00008000;  (*!< Abnormal interrupt summary flag  *)
  ETH_DMA_FLAG_ER = $00004000;  (*!< Early receive flag  *)
  ETH_DMA_FLAG_FBE = $00002000;  (*!< Fatal bus error flag  *)
  ETH_DMA_FLAG_ET = $00000400;  (*!< Early transmit flag  *)
  ETH_DMA_FLAG_RWT = $00000200;  (*!< Receive watchdog timeout flag  *)
  ETH_DMA_FLAG_RPS = $00000100;  (*!< Receive process stopped flag  *)
  ETH_DMA_FLAG_RBU = $00000080;  (*!< Receive buffer unavailable flag  *)
  ETH_DMA_FLAG_R = $00000040;  (*!< Receive flag  *)
  ETH_DMA_FLAG_TU = $00000020;  (*!< Underflow flag  *)
  ETH_DMA_FLAG_RO = $00000010;  (*!< Overflow flag  *)
  ETH_DMA_FLAG_TJT = $00000008;  (*!< Transmit jabber timeout flag  *)
  ETH_DMA_FLAG_TBU = $00000004;  (*!< Transmit buffer unavailable flag  *)
  ETH_DMA_FLAG_TPS = $00000002;  (*!< Transmit process stopped flag  *)
  ETH_DMA_FLAG_T = $00000001;  (*!< Transmit flag  *)
  (**
  * @}
   *)

  (** @defgroup ETH_MAC_Interrupts ETH MAC Interrupts
  * @{
   *)

  ETH_MAC_IT_TST = $00000200;  (*!< Time stamp trigger interrupt (on MAC)  *)
  ETH_MAC_IT_MMCT = $00000040;  (*!< MMC transmit interrupt  *)
  ETH_MAC_IT_MMCR = $00000020;  (*!< MMC receive interrupt  *)
  ETH_MAC_IT_MMC = $00000010;  (*!< MMC interrupt (on MAC)  *)
  ETH_MAC_IT_PMT = $00000008;  (*!< PMT interrupt (on MAC)  *)
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_Interrupts ETH DMA Interrupts
  * @{
   *)

  ETH_DMA_IT_TST = $20000000;  (*!< Time-stamp trigger interrupt (on DMA)  *)
  ETH_DMA_IT_PMT = $10000000;  (*!< PMT interrupt (on DMA)  *)
  ETH_DMA_IT_MMC = $08000000;  (*!< MMC interrupt (on DMA)  *)
  ETH_DMA_IT_NIS = $00010000;  (*!< Normal interrupt summary  *)
  ETH_DMA_IT_AIS = $00008000;  (*!< Abnormal interrupt summary  *)
  ETH_DMA_IT_ER = $00004000;  (*!< Early receive interrupt  *)
  ETH_DMA_IT_FBE = $00002000;  (*!< Fatal bus error interrupt  *)
  ETH_DMA_IT_ET = $00000400;  (*!< Early transmit interrupt  *)
  ETH_DMA_IT_RWT = $00000200;  (*!< Receive watchdog timeout interrupt  *)
  ETH_DMA_IT_RPS = $00000100;  (*!< Receive process stopped interrupt  *)
  ETH_DMA_IT_RBU = $00000080;  (*!< Receive buffer unavailable interrupt  *)
  ETH_DMA_IT_R = $00000040;  (*!< Receive interrupt  *)
  ETH_DMA_IT_TU = $00000020;  (*!< Underflow interrupt  *)
  ETH_DMA_IT_RO = $00000010;  (*!< Overflow interrupt  *)
  ETH_DMA_IT_TJT = $00000008;  (*!< Transmit jabber timeout interrupt  *)
  ETH_DMA_IT_TBU = $00000004;  (*!< Transmit buffer unavailable interrupt  *)
  ETH_DMA_IT_TPS = $00000002;  (*!< Transmit process stopped interrupt  *)
  ETH_DMA_IT_T = $00000001;  (*!< Transmit interrupt  *)
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_transmit_process_state ETH DMA transmit process state
  * @{
   *)

  ETH_DMA_TRANSMITPROCESS_STOPPED = $00000000;  (*!< Stopped - Reset or Stop Tx Command issued  *)
  ETH_DMA_TRANSMITPROCESS_FETCHING = $00100000;  (*!< Running - fetching the Tx descriptor  *)
  ETH_DMA_TRANSMITPROCESS_WAITING = $00200000;  (*!< Running - waiting for status  *)
  ETH_DMA_TRANSMITPROCESS_READING = $00300000;  (*!< Running - reading the data from host memory  *)
  ETH_DMA_TRANSMITPROCESS_SUSPENDED = $00600000;  (*!< Suspended - Tx Descriptor unavailable  *)
  ETH_DMA_TRANSMITPROCESS_CLOSING = $00700000;  (*!< Running - closing Rx descriptor  *)
  (**
  * @}
   *)


  (** @defgroup ETH_DMA_receive_process_state ETH DMA receive process state
  * @{
   *)

  ETH_DMA_RECEIVEPROCESS_STOPPED = $00000000;  (*!< Stopped - Reset or Stop Rx Command issued  *)
  ETH_DMA_RECEIVEPROCESS_FETCHING = $00020000;  (*!< Running - fetching the Rx descriptor  *)
  ETH_DMA_RECEIVEPROCESS_WAITING = $00060000;  (*!< Running - waiting for packet  *)
  ETH_DMA_RECEIVEPROCESS_SUSPENDED = $00080000;  (*!< Suspended - Rx Descriptor unavailable  *)
  ETH_DMA_RECEIVEPROCESS_CLOSING = $000A0000;  (*!< Running - closing descriptor  *)
  ETH_DMA_RECEIVEPROCESS_QUEUING = $000E0000;  (*!< Running - queuing the receive frame into host memory  *)
  (**
  * @}
   *)

  (** @defgroup ETH_DMA_overflow ETH DMA overflow
  * @{
   *)

  ETH_DMA_OVERFLOW_RXFIFOCOUNTER = $10000000;  (*!< Overflow bit for FIFO overflow counter  *)
  ETH_DMA_OVERFLOW_MISSEDFRAMECOUNTER = $00010000;  (*!< Overflow bit for missed frame counter  *)
  (**
  * @}
   *)

  (** @defgroup ETH_EXTI_LINE_WAKEUP ETH EXTI LINE WAKEUP
  * @{
   *)

  ETH_EXTI_LINE_WAKEUP = $00080000;

(*!< External interrupt line 19 Connected to the ETH EXTI Line  *)
  (**
  * @}
   *)

(* Initialization and de-initialization functions  *************************** *)
(** @addtogroup ETH_Exported_Functions_Group1
  * @{
   *)
function HAL_ETH_Init(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ETH_DeInit(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_ETH_MspInit(var heth: ETH_HandleTypeDef); external name 'HAL_ETH_MspInit';
procedure HAL_ETH_MspDeInit(var heth: ETH_HandleTypeDef); external name 'HAL_ETH_MspDeInit';

function HAL_ETH_DMATxDescListInit(var heth: ETH_HandleTypeDef; DMATxDescTab: PETH_DMADescTypeDef; var TxBuff; TxBuffCount: longword): HAL_StatusTypeDef;
function HAL_ETH_DMARxDescListInit(var heth: ETH_HandleTypeDef; DMARxDescTab: PETH_DMADescTypeDef; var RxBuff; RxBuffCount: longword): HAL_StatusTypeDef;

(**
  * @}
   *)

(* IO operation functions  *************************************************** *)
(** @addtogroup ETH_Exported_Functions_Group2
  * @{
   *)
function HAL_ETH_TransmitFrame(var heth: ETH_HandleTypeDef; FrameLength: longword): HAL_StatusTypeDef;
function HAL_ETH_GetReceivedFrame(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;

(* Communication with PHY functions *)
function HAL_ETH_ReadPHYRegister(var heth: ETH_HandleTypeDef; PHYReg: word; var RegValue: longword): HAL_StatusTypeDef;
function HAL_ETH_WritePHYRegister(var heth: ETH_HandleTypeDef; PHYReg: word; RegValue: longword): HAL_StatusTypeDef;

(* Non-Blocking mode: Interrupt  *)
function HAL_ETH_GetReceivedFrame_IT(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
procedure HAL_ETH_IRQHandler(var heth: ETH_HandleTypeDef);

(* Callback in non blocking modes (Interrupt)  *)
procedure HAL_ETH_TxCpltCallback(var heth: ETH_HandleTypeDef); external name 'HAL_ETH_TxCpltCallback';
procedure HAL_ETH_RxCpltCallback(var heth: ETH_HandleTypeDef); external name 'HAL_ETH_RxCpltCallback';
procedure HAL_ETH_ErrorCallback(var heth: ETH_HandleTypeDef); external name 'HAL_ETH_ErrorCallback';

(**
  * @}
   *)
(* Peripheral Control functions  ********************************************* *)
(** @addtogroup ETH_Exported_Functions_Group3
  * @{
   *)
function HAL_ETH_Start(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ETH_Stop(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
function HAL_ETH_ConfigMAC(var heth: ETH_HandleTypeDef; macconf: PETH_MACInitTypeDef): HAL_StatusTypeDef;
function HAL_ETH_ConfigDMA(var heth: ETH_HandleTypeDef; dmaconf: PETH_DMAInitTypeDef): HAL_StatusTypeDef;

(**
  * @}
   *)
(* Peripheral State functions  *********************************************** *)
(** @addtogroup ETH_Exported_Functions_Group4
  * @{
   *)
function HAL_ETH_GetState(var heth: ETH_HandleTypeDef): HAL_ETH_StateTypeDef;

implementation

uses
  stm32f7xx_hal_conf,
  stm32f7xx_hal_rcc,
  stm32f7xx_hal_rcc_ex;

const
  LINKED_STATE_TIMEOUT_VALUE = 2000;  (* 2000 ms  *)
  AUTONEGO_COMPLETED_TIMEOUT_VALUE = 1000;  (* 1000 ms  *)

procedure ETH_MACAddressConfig(var heth: ETH_HandleTypeDef; MacAddr: longword; Addr: pbyte); forward;

(* Exported macro ------------------------------------------------------------*)
(** @defgroup ETH_Exported_Macros ETH Exported Macros
 *  @brief macros to handle interrupts and specific clock configurations
 * @begin
 *)

(** @brief Reset ETH handle state
  * @param  __HANDLE__: specifies the ETH handle.
  * @retval None
  *)
procedure __HAL_ETH_RESET_HANDLE_STATE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.State := HAL_ETH_STATE_RESET;
end;

(**
  * @brief  Checks whether the specified ETHERNET DMA Tx Desc flag is set or not.
  * @param  __HANDLE__: ETH Handle
  * @param  __FLAG__: specifies the flag of TDES0 to check.
  * @retval the ETH_DMATxDescFlag (SET or RESET).
  *)
function __HAL_ETH_DMATXDESC_GET_FLAG(var __HANDLE__: ETH_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit((__HANDLE__.TxDesc^.Status and (__FLAG__)) = (__FLAG__));
end;

(**
  * @brief  Checks whether the specified ETHERNET DMA Rx Desc flag is set or not.
  * @param  __HANDLE__: ETH Handle
  * @param  __FLAG__: specifies the flag of RDES0 to check.
  * @retval the ETH_DMATxDescFlag (SET or RESET).
  *)
function __HAL_ETH_DMARXDESC_GET_FLAG(var __HANDLE__: ETH_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit(__HANDLE__.RxDesc^.Status and (__FLAG__) = (__FLAG__));
end;

(**
  * @brief  Enables the specified DMA Rx Desc receive interrupt.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMARXDESC_ENABLE_IT(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.RxDesc^.ControlBufferSize := __Handle__.RxDesc^.ControlBufferSize and ((not ETH_DMARXDESC_DIC));
end;

(**
  * @brief  Disables the specified DMA Rx Desc receive interrupt.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMARXDESC_DISABLE_IT(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.RxDesc^.ControlBufferSize := __HANDLE__.RxDesc^.ControlBufferSize or ETH_DMARXDESC_DIC;
end;

(**
  * @brief  Set the specified DMA Rx Desc Own bit.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMARXDESC_SET_OWN_BIT(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.RxDesc^.Status := __HANDLE__.RxDesc^.Status or ETH_DMARXDESC_OWN;
end;

(**
  * @brief  Returns the specified ETHERNET DMA Tx Desc collision count.
  * @param  __HANDLE__: ETH Handle
  * @retval The Transmit descriptor collision counter value.
  *)
function __HAL_ETH_DMATXDESC_GET_COLLISION_COUNT(var __HANDLE__: ETH_HandleTypeDef): longword;
begin
  exit((__HANDLE__.TxDesc^.Status and ETH_DMATXDESC_CC) shr ETH_DMATXDESC_COLLISION_COUNTSHIFT);
end;

(**
  * @brief  Set the specified DMA Tx Desc Own bit.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMATXDESC_SET_OWN_BIT(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.TxDesc^.Status := __HANDLE__.TxDesc^.Status or ETH_DMATXDESC_OWN;
end;

(**
  * @brief  Enables the specified DMA Tx Desc Transmit interrupt.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMATXDESC_ENABLE_IT(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.TxDesc^.Status := __HANDLE__.TxDesc^.Status or ETH_DMATXDESC_IC;
end;

(**
  * @brief  Disables the specified DMA Tx Desc Transmit interrupt.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMATXDESC_DISABLE_IT(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.TxDesc^.Status := __HANDLE__.TxDesc^.Status and (not ETH_DMATXDESC_IC);
end;

(**
  * @brief  Selects the specified ETHERNET DMA Tx Desc Checksum Insertion.
  * @param  __HANDLE__: ETH Handle
  * @param  __CHECKSUM__: specifies is the DMA Tx desc checksum insertion.
  *   This parameter can be one of the following values:
  *     @arg ETH_DMATXDESC_CHECKSUMBYPASS : Checksum bypass
  *     @arg ETH_DMATXDESC_CHECKSUMIPV4HEADER : IPv4 header checksum
  *     @arg ETH_DMATXDESC_CHECKSUMTCPUDPICMPSEGMENT : TCP/UDP/ICMP checksum. Pseudo header checksum is assumed to be present
  *     @arg ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL : TCP/UDP/ICMP checksum fully in hardware including pseudo header
  * @retval None
  *)
procedure __HAL_ETH_DMATXDESC_CHECKSUM_INSERTION(var __HANDLE__: ETH_HandleTypeDef; __CHECKSUM__: longword);
begin
  __HANDLE__.TxDesc^.Status := __HANDLE__.TxDesc^.Status or (__CHECKSUM__);
end;

(**
  * @brief  Enables the DMA Tx Desc CRC.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMATXDESC_CRC_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.TxDesc^.Status := __HANDLE__.TxDesc^.Status and (not ETH_DMATXDESC_DC);
end;

(**
  * @brief  Disables the DMA Tx Desc CRC.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMATXDESC_CRC_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.TxDesc^.Status := __HANDLE__.TxDesc^.Status or ETH_DMATXDESC_DC;
end;

(**
  * @brief  Enables the DMA Tx Desc padding for frame shorter than 64 bytes.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMATXDESC_SHORT_FRAME_PADDING_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.TxDesc^.Status := __HANDLE__.TxDesc^.Status and (not ETH_DMATXDESC_DP);
end;

(**
  * @brief  Disables the DMA Tx Desc padding for frame shorter than 64 bytes.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_DMATXDESC_SHORT_FRAME_PADDING_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.TxDesc^.Status := __HANDLE__.TxDesc^.Status or ETH_DMATXDESC_DP;
end;

(**
 * @brief  Enables the specified ETHERNET MAC interrupts.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the ETHERNET MAC interrupt sources to be
  *   enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg ETH_MAC_IT_TST : Time stamp trigger interrupt
  *     @arg ETH_MAC_IT_PMT : PMT interrupt
  * @retval None
  *)
procedure __HAL_ETH_MAC_ENABLE_IT(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.MACIMR := __HANDLE__.Instance^.MACIMR or __INTERRUPT__;
end;

(**
  * @brief  Disables the specified ETHERNET MAC interrupts.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the ETHERNET MAC interrupt sources to be
  *   enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg ETH_MAC_IT_TST : Time stamp trigger interrupt
  *     @arg ETH_MAC_IT_PMT : PMT interrupt
  * @retval None
  *)
procedure __HAL_ETH_MAC_DISABLE_IT(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.MACIMR := __HANDLE__.Instance^.MACIMR and (not __INTERRUPT__);
end;

(**
  * @brief  Initiate a Pause Control Frame (Full-duplex only).
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_INITIATE_PAUSE_CONTROL_FRAME(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACFCR := __HANDLE__.Instance^.MACFCR or ETH_MACFCR_FCBBPA;
end;

(**
  * @brief  Checks whether the ETHERNET flow control busy bit is set or not.
  * @param  __HANDLE__: ETH Handle
  * @retval The new state of flow control busy status bit (SET or RESET).
  *)
function __HAL_ETH_GET_FLOW_CONTROL_BUSY_STATUS(var __HANDLE__: ETH_HandleTypeDef): boolean;
begin
  exit((__HANDLE__.Instance^.MACFCR and ETH_MACFCR_FCBBPA) = ETH_MACFCR_FCBBPA);
end;

(**
  * @brief  Enables the MAC Back Pressure operation activation (Half-duplex only).
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_BACK_PRESSURE_ACTIVATION_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACFCR := __HANDLE__.Instance^.MACFCR or ETH_MACFCR_FCBBPA;
end;

(**
  * @brief  Disables the MAC BackPressure operation activation (Half-duplex only).
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_BACK_PRESSURE_ACTIVATION_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACFCR := __HANDLE__.Instance^.MACFCR and (not ETH_MACFCR_FCBBPA);
end;

(**
  * @brief  Checks whether the specified ETHERNET MAC flag is set or not.
  * @param  __HANDLE__: ETH Handle
  * @param  __FLAG__: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg ETH_MAC_FLAG_TST  : Time stamp trigger flag
  *     @arg ETH_MAC_FLAG_MMCT : MMC transmit flag
  *     @arg ETH_MAC_FLAG_MMCR : MMC receive flag
  *     @arg ETH_MAC_FLAG_MMC  : MMC flag
  *     @arg ETH_MAC_FLAG_PMT  : PMT flag
  * @retval The state of ETHERNET MAC flag.
  *)
function __HAL_ETH_MAC_GET_FLAG(var __HANDLE__: ETH_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit((__HANDLE__.Instance^.MACSR and (__FLAG__)) = (__FLAG__));
end;

(**
  * @brief  Enables the specified ETHERNET DMA interrupts.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the ETHERNET DMA interrupt sources to be
  *   enabled @ref ETH_DMA_Interrupts
  * @retval None
  *)
procedure __HAL_ETH_DMA_ENABLE_IT(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.DMAIER := __HANDLE__.Instance^.DMAIER or __INTERRUPT__;
end;

(**
  * @brief  Disables the specified ETHERNET DMA interrupts.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the ETHERNET DMA interrupt sources to be
  *   disabled. @ref ETH_DMA_Interrupts
  * @retval None
  *)
procedure __HAL_ETH_DMA_DISABLE_IT(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.DMAIER := __HANDLE__.Instance^.DMAIER and (not __INTERRUPT__);
end;

(**
  * @brief  Clears the ETHERNET DMA IT pending bit.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the interrupt pending bit to clear. @ref ETH_DMA_Interrupts
  * @retval None
  *)
procedure __HAL_ETH_DMA_CLEAR_IT(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.DMASR := __INTERRUPT__;
end;

(**
  * @brief  Checks whether the specified ETHERNET DMA flag is set or not.
* @param  __HANDLE__: ETH Handle
  * @param  __FLAG__: specifies the flag to check. @ref ETH_DMA_Flags
  * @retval The new state of ETH_DMA_FLAG (SET or RESET).
  *)
function __HAL_ETH_DMA_GET_FLAG(var __HANDLE__: ETH_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit((__HANDLE__.Instance^.DMASR and (__FLAG__)) = (__FLAG__));
end;

(**
  * @brief  Checks whether the specified ETHERNET DMA flag is set or not.
  * @param  __HANDLE__: ETH Handle
  * @param  __FLAG__: specifies the flag to clear. @ref ETH_DMA_Flags
  * @retval The new state of ETH_DMA_FLAG (SET or RESET).
  *)
procedure __HAL_ETH_DMA_CLEAR_FLAG(var __HANDLE__: ETH_HandleTypeDef; __FLAG__: longword);
begin
  __HANDLE__.Instance^.DMASR := (__FLAG__);
end;

(**
  * @brief  Checks whether the specified ETHERNET DMA overflow flag is set or not.
  * @param  __HANDLE__: ETH Handle
  * @param  __OVERFLOW__: specifies the DMA overflow flag to check.
  *   This parameter can be one of the following values:
  *     @arg ETH_DMA_OVERFLOW_RXFIFOCOUNTER : Overflow for FIFO Overflows Counter
  *     @arg ETH_DMA_OVERFLOW_MISSEDFRAMECOUNTER : Overflow for Buffer Unavailable Missed Frame Counter
  * @retval The state of ETHERNET DMA overflow Flag (SET or RESET).
  *)
function __HAL_ETH_GET_DMA_OVERFLOW_STATUS(var __HANDLE__: ETH_HandleTypeDef; __OVERFLOW__: longword): boolean;
begin
  exit((__HANDLE__.Instance^.DMAMFBOCR and (__OVERFLOW__)) = (__OVERFLOW__));
end;

(**
  * @brief  Set the DMA Receive status watchdog timer register value
  * @param  __HANDLE__: ETH Handle
  * @param  __VALUE__: DMA Receive status watchdog timer register value
  * @retval None
  *)
procedure __HAL_ETH_SET_RECEIVE_WATCHDOG_TIMER(var __HANDLE__: ETH_HandleTypeDef; __VALUE__: longword);
begin
  __HANDLE__.Instance^.DMARSWTR := (__VALUE__);
end;

(**
  * @brief  Enables any unicast packet filtered by the MAC address
  *   recognition to be a wake-up frame.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_GLOBAL_UNICAST_WAKEUP_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACPMTCSR := __HANDLE__.Instance^.MACPMTCSR or ETH_MACPMTCSR_GU;
end;

(**
  * @brief  Disables any unicast packet filtered by the MAC address
  *   recognition to be a wake-up frame.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_GLOBAL_UNICAST_WAKEUP_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACPMTCSR := __HANDLE__.Instance^.MACPMTCSR and (not ETH_MACPMTCSR_GU);
end;

(**
  * @brief  Enables the MAC Wake-Up Frame Detection.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_FRAME_DETECTION_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACPMTCSR := __HANDLE__.Instance^.MACPMTCSR or ETH_MACPMTCSR_WFE;
end;

(**
  * @brief  Disables the MAC Wake-Up Frame Detection.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_FRAME_DETECTION_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACPMTCSR := __HANDLE__.Instance^.MACPMTCSR and (not ETH_MACPMTCSR_WFE);
end;

(**
  * @brief  Enables the MAC Magic Packet Detection.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_MAGIC_PACKET_DETECTION_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACPMTCSR := __HANDLE__.Instance^.MACPMTCSR or ETH_MACPMTCSR_MPE;
end;

(**
  * @brief  Disables the MAC Magic Packet Detection.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_MAGIC_PACKET_DETECTION_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACPMTCSR := __HANDLE__.Instance^.MACPMTCSR and (not ETH_MACPMTCSR_WFE);
end;

(**
  * @brief  Enables the MAC Power Down.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_POWER_DOWN_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACPMTCSR := __HANDLE__.Instance^.MACPMTCSR or ETH_MACPMTCSR_PD;
end;

(**
  * @brief  Disables the MAC Power Down.
  * @param  __HANDLE__: ETH Handle
  * @retval None
  *)
procedure __HAL_ETH_POWER_DOWN_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MACPMTCSR := __HANDLE__.Instance^.MACPMTCSR and (not ETH_MACPMTCSR_PD);
end;

(**
  * @brief  Checks whether the specified ETHERNET PMT flag is set or not.
  * @param  __HANDLE__: ETH Handle.
  * @param  __FLAG__: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg ETH_PMT_FLAG_WUFFRPR : Wake-Up Frame Filter Register Pointer Reset
  *     @arg ETH_PMT_FLAG_WUFR    : Wake-Up Frame Received
  *     @arg ETH_PMT_FLAG_MPR     : Magic Packet Received
  * @retval The new state of ETHERNET PMT Flag (SET or RESET).
  *)
function __HAL_ETH_GET_PMT_FLAG_STATUS(var __HANDLE__: ETH_HandleTypeDef; __FLAG__: longword): boolean;
begin
  exit((__HANDLE__.Instance^.MACPMTCSR and (__FLAG__)) = (__FLAG__));
end;

(**
  * @brief  Preset and Initialize the MMC counters to almost-full value: $FFFF_FFF0 (full - 16)
  * @param   __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_MMC_COUNTER_FULL_PRESET(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR or (ETH_MMCCR_MCFHP or ETH_MMCCR_MCP);
end;

(**
  * @brief  Preset and Initialize the MMC counters to almost-half value: $7FFF_FFF0 (half - 16)
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_MMC_COUNTER_HALF_PRESET(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR and (not ETH_MMCCR_MCFHP);
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR or ETH_MMCCR_MCP;
end;

(**
  * @brief  Enables the MMC Counter Freeze.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_MMC_COUNTER_FREEZE_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR or ETH_MMCCR_MCF;
end;

(**
  * @brief  Disables the MMC Counter Freeze.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_MMC_COUNTER_FREEZE_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR and (not ETH_MMCCR_MCF);
end;

(**
  * @brief  Enables the MMC Reset On Read.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_ETH_MMC_RESET_ONREAD_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR or ETH_MMCCR_ROR;
end;

(**
  * @brief  Disables the MMC Reset On Read.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_ETH_MMC_RESET_ONREAD_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR and (not ETH_MMCCR_ROR);
end;

(**
  * @brief  Enables the MMC Counter Stop Rollover.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_ETH_MMC_COUNTER_ROLLOVER_ENABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR and (not ETH_MMCCR_CSR);
end;

(**
  * @brief  Disables the MMC Counter Stop Rollover.
  * @param  __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_ETH_MMC_COUNTER_ROLLOVER_DISABLE(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR or ETH_MMCCR_CSR;
end;

(**
  * @brief  Resets the MMC Counters.
  * @param   __HANDLE__: ETH Handle.
  * @retval None
  *)
procedure __HAL_ETH_MMC_COUNTERS_RESET(var __HANDLE__: ETH_HandleTypeDef);
begin
  __HANDLE__.Instance^.MMCCR := __HANDLE__.Instance^.MMCCR or ETH_MMCCR_CR;
end;

(**
  * @brief  Enables the specified ETHERNET MMC Rx interrupts.
  * @param   __HANDLE__: ETH Handle.
  * @param  __INTERRUPT__: specifies the ETHERNET MMC interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg ETH_MMC_IT_RGUF  : When Rx good unicast frames counter reaches half the maximum value
  *     @arg ETH_MMC_IT_RFAE  : When Rx alignment error counter reaches half the maximum value
  *     @arg ETH_MMC_IT_RFCE  : When Rx crc error counter reaches half the maximum value
  * @retval None
  *)
procedure __HAL_ETH_MMC_RX_IT_ENABLE(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.MMCRIMR := __HANDLE__.Instance^.MMCRIMR and (not (__INTERRUPT__ and $EFFFFFFF));
end;

(**
  * @brief  Disables the specified ETHERNET MMC Rx interrupts.
  * @param   __HANDLE__: ETH Handle.
  * @param  __INTERRUPT__: specifies the ETHERNET MMC interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg ETH_MMC_IT_RGUF  : When Rx good unicast frames counter reaches half the maximum value
  *     @arg ETH_MMC_IT_RFAE  : When Rx alignment error counter reaches half the maximum value
  *     @arg ETH_MMC_IT_RFCE  : When Rx crc error counter reaches half the maximum value
  * @retval None
  *)
procedure __HAL_ETH_MMC_RX_IT_DISABLE(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.MMCRIMR := __HANDLE__.Instance^.MMCRIMR or (__INTERRUPT__ and $EFFFFFFF);
end;

(**
  * @brief  Enables the specified ETHERNET MMC Tx interrupts.
  * @param   __HANDLE__: ETH Handle.
  * @param  __INTERRUPT__: specifies the ETHERNET MMC interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg ETH_MMC_IT_TGF   : When Tx good frame counter reaches half the maximum value
  *     @arg ETH_MMC_IT_TGFMSC: When Tx good multi col counter reaches half the maximum value
  *     @arg ETH_MMC_IT_TGFSC : When Tx good single col counter reaches half the maximum value
  * @retval None
  *)
procedure __HAL_ETH_MMC_TX_IT_ENABLE(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.MMCRIMR := __HANDLE__.Instance^.MMCRIMR and (not __INTERRUPT__);
end;

(**
  * @brief  Disables the specified ETHERNET MMC Tx interrupts.
  * @param   __HANDLE__: ETH Handle.
  * @param  __INTERRUPT__: specifies the ETHERNET MMC interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg ETH_MMC_IT_TGF   : When Tx good frame counter reaches half the maximum value
  *     @arg ETH_MMC_IT_TGFMSC: When Tx good multi col counter reaches half the maximum value
  *     @arg ETH_MMC_IT_TGFSC : When Tx good single col counter reaches half the maximum value
  * @retval None
  *)
procedure __HAL_ETH_MMC_TX_IT_DISABLE(var __HANDLE__: ETH_HandleTypeDef; __INTERRUPT__: longword);
begin
  __HANDLE__.Instance^.MMCRIMR := __HANDLE__.Instance^.MMCRIMR or __INTERRUPT__;
end;

(**
  * @brief  Enables the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_ENABLE_IT();
begin
  EXTI.IMR := EXTI.IMR or (ETH_EXTI_LINE_WAKEUP);
end;

(**
  * @brief  Disables the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_DISABLE_IT();
begin
  EXTI.IMR := EXTI.IMR and (not (ETH_EXTI_LINE_WAKEUP));
end;

(**
  * @brief Enable event on ETH External event line.
  * @retval None.
  *)
procedure __HAL_ETH_WAKEUP_EXTI_ENABLE_EVENT();
begin
  EXTI.EMR := EXTI.EMR or (ETH_EXTI_LINE_WAKEUP);
end;

(**
  * @brief Disable event on ETH External event line
  * @retval None.
  *)
procedure __HAL_ETH_WAKEUP_EXTI_DISABLE_EVENT();
begin
  EXTI.EMR := EXTI.EMR and (not (ETH_EXTI_LINE_WAKEUP));
end;

(**
  * @brief  Get flag of the ETH External interrupt line.
  * @retval None
  *)
function __HAL_ETH_WAKEUP_EXTI_GET_FLAG(): longword;
begin
  exit(EXTI.PR and (ETH_EXTI_LINE_WAKEUP));
end;

(**
  * @brief  Clear flag of the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_CLEAR_FLAG();
begin
  EXTI.PR := (ETH_EXTI_LINE_WAKEUP);
end;

(**
  * @brief  Enables rising edge trigger to the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_ENABLE_RISING_EDGE_TRIGGER();
begin
  EXTI.RTSR := EXTI.RTSR or ETH_EXTI_LINE_WAKEUP;
end;

(**
  * @brief  Disables the rising edge trigger to the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_DISABLE_RISING_EDGE_TRIGGER();
begin
  EXTI.RTSR := EXTI.RTSR and (not (ETH_EXTI_LINE_WAKEUP));
end;

(**
  * @brief  Enables falling edge trigger to the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_ENABLE_FALLING_EDGE_TRIGGER();
begin
  EXTI.FTSR := EXTI.FTSR or (ETH_EXTI_LINE_WAKEUP);
end;

(**
  * @brief  Disables falling edge trigger to the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_DISABLE_FALLING_EDGE_TRIGGER();
begin
  EXTI.FTSR := EXTI.FTSR and (not (ETH_EXTI_LINE_WAKEUP));
end;

(**
  * @brief  Enables rising/falling edge trigger to the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_ENABLE_FALLINGRISING_TRIGGER();
begin
  EXTI.RTSR := EXTI.RTSR or ETH_EXTI_LINE_WAKEUP;
  ;
  EXTI.FTSR := EXTI.FTSR or ETH_EXTI_LINE_WAKEUP;
end;

(**
  * @brief  Disables rising/falling edge trigger to the ETH External interrupt line.
  * @retval None
  *)
procedure __HAL_ETH_WAKEUP_EXTI_DISABLE_FALLINGRISING_TRIGGER();
begin
  EXTI.RTSR := EXTI.RTSR and (not (ETH_EXTI_LINE_WAKEUP));
  EXTI.FTSR := EXTI.FTSR and (not (ETH_EXTI_LINE_WAKEUP));
end;

(**
  * @brief Generate a Software interrupt on selected EXTI line.
  * @retval None.
  *)
procedure __HAL_ETH_WAKEUP_EXTI_GENERATE_SWIT();
begin
  EXTI.SWIER := EXTI.SWIER or ETH_EXTI_LINE_WAKEUP;
end;

(**
  * @brief  Configures Ethernet MAC and DMA with default parameters.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  err: Ethernet Init error
  * @retval HAL status
  *)
procedure ETH_MACDMAConfig(var heth: ETH_HandleTypeDef; err: longword);
var
  macinit: ETH_MACInitTypeDef;
  dmainit: ETH_DMAInitTypeDef;
  tmpreg: longword;
begin
  if (err <> ETH_SUCCESS) then (* Auto-negotiation failed *)
  begin
    (* Set Ethernet duplex mode to Full-duplex *)
    (heth.Init).DuplexMode := ETH_MODE_FULLDUPLEX;

    (* Set Ethernet speed to 100M *)
    (heth.Init).Speed := ETH_SPEED_100M;
  end;

  (* Ethernet MAC default initialization **************************************)
  macinit.Watchdog := ETH_WATCHDOG_ENABLE;
  macinit.Jabber := ETH_JABBER_ENABLE;
  macinit.InterFrameGap := ETH_INTERFRAMEGAP_96BIT;
  macinit.CarrierSense := ETH_CARRIERSENCE_ENABLE;
  macinit.ReceiveOwn := ETH_RECEIVEOWN_ENABLE;
  macinit.LoopbackMode := ETH_LOOPBACKMODE_DISABLE;

  if (heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE) then
    macinit.ChecksumOffload := ETH_CHECKSUMOFFLAOD_ENABLE
  else
    macinit.ChecksumOffload := ETH_CHECKSUMOFFLAOD_DISABLE;

  macinit.RetryTransmission := ETH_RETRYTRANSMISSION_DISABLE;
  macinit.AutomaticPadCRCStrip := ETH_AUTOMATICPADCRCSTRIP_DISABLE;
  macinit.BackOffLimit := ETH_BACKOFFLIMIT_10;
  macinit.DeferralCheck := ETH_DEFFERRALCHECK_DISABLE;
  macinit.ReceiveAll := ETH_RECEIVEAll_DISABLE;
  macinit.SourceAddrFilter := ETH_SOURCEADDRFILTER_DISABLE;
  macinit.PassControlFrames := ETH_PASSCONTROLFRAMES_BLOCKALL;
  macinit.BroadcastFramesReception := ETH_BROADCASTFRAMESRECEPTION_ENABLE;
  macinit.DestinationAddrFilter := ETH_DESTINATIONADDRFILTER_NORMAL;
  macinit.PromiscuousMode := ETH_PROMISCUOUS_MODE_DISABLE;
  macinit.MulticastFramesFilter := ETH_MULTICASTFRAMESFILTER_PERFECT;
  macinit.UnicastFramesFilter := ETH_UNICASTFRAMESFILTER_PERFECT;
  macinit.HashTableHigh := $0;
  macinit.HashTableLow := $0;
  macinit.PauseTime := $0;
  macinit.ZeroQuantaPause := ETH_ZEROQUANTAPAUSE_DISABLE;
  macinit.PauseLowThreshold := ETH_PAUSELOWTHRESHOLD_MINUS4;
  macinit.UnicastPauseFrameDetect := ETH_UNICASTPAUSEFRAMEDETECT_DISABLE;
  macinit.ReceiveFlowControl := ETH_RECEIVEFLOWCONTROL_DISABLE;
  macinit.TransmitFlowControl := ETH_TRANSMITFLOWCONTROL_DISABLE;
  macinit.VLANTagComparison := ETH_VLANTAGCOMPARISON_16BIT;
  macinit.VLANTagIdentifier := $0;

  (*------------------------ ETHERNET MACCR Configuration --------------------*)
  (* Get the ETHERNET MACCR value *)
  tmpreg := heth.Instance^.MACCR;
  (* Clear WD, PCE, PS, TE and RE bits *)
  tmpreg := tmpreg and (ETH_MACCR_CLEAR_MASK);
  (* Set the WD bit according to ETH Watchdog value *)
  (* Set the JD: bit according to ETH Jabber value *)
  (* Set the IFG bit according to ETH InterFrameGap value *)
  (* Set the DCRS bit according to ETH CarrierSense value *)
  (* Set the FES bit according to ETH Speed value *)
  (* Set the DO bit according to ETH ReceiveOwn value *)
  (* Set the LM bit according to ETH LoopbackMode value *)
  (* Set the DM bit according to ETH Mode value *)
  (* Set the IPCO bit according to ETH ChecksumOffload value *)
  (* Set the DR bit according to ETH RetryTransmission value *)
  (* Set the ACS bit according to ETH AutomaticPadCRCStrip value *)
  (* Set the BL bit according to ETH BackOffLimit value *)
  (* Set the DC bit according to ETH DeferralCheck value *)
  tmpreg := tmpreg or (macinit.Watchdog or macinit.Jabber or macinit.InterFrameGap or macinit.CarrierSense or (heth.Init).Speed or macinit.ReceiveOwn or macinit.LoopbackMode or
    (heth.Init).DuplexMode or macinit.ChecksumOffload or macinit.RetryTransmission or macinit.AutomaticPadCRCStrip or macinit.BackOffLimit or macinit.DeferralCheck);

  (* Write to ETHERNET MACCR *)
  heth.Instance^.MACCR := tmpreg;

  (* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.MACCR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.MACCR := tmpreg;

  (*----------------------- ETHERNET MACFFR Configuration --------------------*)
  (* Set the RA bit according to ETH ReceiveAll value *)
  (* Set the SAF and SAIF bits according to ETH SourceAddrFilter value *)
  (* Set the PCF bit according to ETH PassControlFrames value *)
  (* Set the DBF bit according to ETH BroadcastFramesReception value *)
  (* Set the DAIF bit according to ETH DestinationAddrFilter value *)
  (* Set the PR bit according to ETH PromiscuousMode value *)
  (* Set the PM, HMC and HPF bits according to ETH MulticastFramesFilter value *)
  (* Set the HUC and HPF bits according to ETH UnicastFramesFilter value *)
  (* Write to ETHERNET MACFFR *)
  heth.Instance^.MACFFR := (macinit.ReceiveAll or macinit.SourceAddrFilter or macinit.PassControlFrames or macinit.BroadcastFramesReception or macinit.DestinationAddrFilter or macinit.PromiscuousMode or
    macinit.MulticastFramesFilter or macinit.UnicastFramesFilter);

   (* Wait until the write operation will be taken into account:
      at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.MACFFR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.MACFFR := tmpreg;

  (*--------------- ETHERNET MACHTHR and MACHTLR Configuration --------------*)
  (* Write to ETHERNET MACHTHR *)
  heth.Instance^.MACHTHR := macinit.HashTableHigh;

  (* Write to ETHERNET MACHTLR *)
  heth.Instance^.MACHTLR := macinit.HashTableLow;
  (*----------------------- ETHERNET MACFCR Configuration -------------------*)

  (* Get the ETHERNET MACFCR value *)
  tmpreg := heth.Instance^.MACFCR;
  (* Clear xx bits *)
  tmpreg := tmpreg and (ETH_MACFCR_CLEAR_MASK);

  (* Set the PT bit according to ETH PauseTime value *)
  (* Set the DZPQ bit according to ETH ZeroQuantaPause value *)
  (* Set the PLT bit according to ETH PauseLowThreshold value *)
  (* Set the UP bit according to ETH UnicastPauseFrameDetect value *)
  (* Set the RFE bit according to ETH ReceiveFlowControl value *)
  (* Set the TFE bit according to ETH TransmitFlowControl value *)
  tmpreg := tmpreg or ((macinit.PauseTime shl 16) or macinit.ZeroQuantaPause or macinit.PauseLowThreshold or macinit.UnicastPauseFrameDetect or macinit.ReceiveFlowControl or macinit.TransmitFlowControl);

  (* Write to ETHERNET MACFCR *)
  heth.Instance^.MACFCR := tmpreg;

   (* Wait until the write operation will be taken into account:
   at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.MACFCR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.MACFCR := tmpreg;

  (*----------------------- ETHERNET MACVLANTR Configuration ----------------*)
  (* Set the ETV bit according to ETH VLANTagComparison value *)
  (* Set the VL bit according to ETH VLANTagIdentifier value *)
  heth.Instance^.MACVLANTR := (macinit.VLANTagComparison or macinit.VLANTagIdentifier);

    (* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.MACVLANTR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.MACVLANTR := tmpreg;

  (* Ethernet DMA default initialization ************************************)
  dmainit.DropTCPIPChecksumErrorFrame := ETH_DROPTCPIPCHECKSUMERRORFRAME_ENABLE;
  dmainit.ReceiveStoreForward := ETH_RECEIVESTOREFORWARD_ENABLE;
  dmainit.FlushReceivedFrame := ETH_FLUSHRECEIVEDFRAME_ENABLE;
  dmainit.TransmitStoreForward := ETH_TRANSMITSTOREFORWARD_ENABLE;
  dmainit.TransmitThresholdControl := ETH_TRANSMITTHRESHOLDCONTROL_64BYTES;
  dmainit.ForwardErrorFrames := ETH_FORWARDERRORFRAMES_DISABLE;
  dmainit.ForwardUndersizedGoodFrames := ETH_FORWARDUNDERSIZEDGOODFRAMES_DISABLE;
  dmainit.ReceiveThresholdControl := ETH_RECEIVEDTHRESHOLDCONTROL_64BYTES;
  dmainit.SecondFrameOperate := ETH_SECONDFRAMEOPERARTE_ENABLE;
  dmainit.AddressAlignedBeats := ETH_ADDRESSALIGNEDBEATS_ENABLE;
  dmainit.FixedBurst := ETH_FIXEDBURST_ENABLE;
  dmainit.RxDMABurstLength := ETH_RXDMABURSTLENGTH_32BEAT;
  dmainit.TxDMABurstLength := ETH_TXDMABURSTLENGTH_32BEAT;
  dmainit.EnhancedDescriptorFormat := ETH_DMAENHANCEDDESCRIPTOR_ENABLE;
  dmainit.DescriptorSkipLength := $0;
  dmainit.DMAArbitration := ETH_DMAARBITRATION_ROUNDROBIN_RXTX_1_1;

  (* Get the ETHERNET DMAOMR value *)
  tmpreg := heth.Instance^.DMAOMR;
  (* Clear xx bits *)
  tmpreg := tmpreg and (ETH_DMAOMR_CLEAR_MASK);

  (* Set the DT bit according to ETH DropTCPIPChecksumErrorFrame value *)
  (* Set the RSF bit according to ETH ReceiveStoreForward value *)
  (* Set the DFF bit according to ETH FlushReceivedFrame value *)
  (* Set the TSF bit according to ETH TransmitStoreForward value *)
  (* Set the TTC bit according to ETH TransmitThresholdControl value *)
  (* Set the FEF bit according to ETH ForwardErrorFrames value *)
  (* Set the FUF bit according to ETH ForwardUndersizedGoodFrames value *)
  (* Set the RTC bit according to ETH ReceiveThresholdControl value *)
  (* Set the OSF bit according to ETH SecondFrameOperate value *)
  tmpreg := tmpreg or (dmainit.DropTCPIPChecksumErrorFrame or dmainit.ReceiveStoreForward or dmainit.FlushReceivedFrame or dmainit.TransmitStoreForward or dmainit.TransmitThresholdControl or
    dmainit.ForwardErrorFrames or dmainit.ForwardUndersizedGoodFrames or dmainit.ReceiveThresholdControl or dmainit.SecondFrameOperate);

  (* Write to ETHERNET DMAOMR *)
  heth.Instance^.DMAOMR := tmpreg;

    (* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.DMAOMR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.DMAOMR := tmpreg;

  (*----------------------- ETHERNET DMABMR Configuration ------------------*)
  (* Set the AAL bit according to ETH AddressAlignedBeats value *)
  (* Set the FB bit according to ETH FixedBurst value *)
  (* Set the RPBL and 4*PBL bits according to ETH RxDMABurstLength value *)
  (* Set the PBL and 4*PBL bits according to ETH TxDMABurstLength value *)
  (* Set the Enhanced DMA descriptors bit according to ETH EnhancedDescriptorFormat value*)
  (* Set the DSL bit according to ETH DesciptorSkipLength value *)
  (* Set the PR and DA bits according to ETH DMAArbitration value *)
  heth.Instance^.DMABMR := (dmainit.AddressAlignedBeats or dmainit.FixedBurst or dmainit.RxDMABurstLength or
    (* !! if 4xPBL is selected for Tx or Rx it is applied for the other *)
    dmainit.TxDMABurstLength or dmainit.EnhancedDescriptorFormat or (dmainit.DescriptorSkipLength shl 2) or dmainit.DMAArbitration or ETH_DMABMR_USP); (* Enable use of separate PBL for Rx and Tx *)

     (* Wait until the write operation will be taken into account:
        at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.DMABMR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.DMABMR := tmpreg;

  if ((heth.Init).RxMode = ETH_RXINTERRUPT_MODE) then
  begin
    (* Enable the Ethernet Rx Interrupt *)
    __HAL_ETH_DMA_ENABLE_IT((heth), ETH_DMA_IT_NIS or ETH_DMA_IT_R);
  end;

  (* Initialize MAC address in ethernet MAC *)
  ETH_MACAddressConfig(heth, ETH_MAC_ADDRESS0, heth.Init.MACAddr);
end;

(**
  * @brief  Configures the selected MAC address.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  MacAddr: The MAC address to configure
  *          This parameter can be one of the following values:
  *             @arg ETH_MAC_Address0: MAC Address0
  *             @arg ETH_MAC_Address1: MAC Address1
  *             @arg ETH_MAC_Address2: MAC Address2
  *             @arg ETH_MAC_Address3: MAC Address3
  * @param  Addr: Pointer to MAC address buffer data (6 bytes)
  * @retval HAL status
  *)
procedure ETH_MACAddressConfig(var heth: ETH_HandleTypeDef; MacAddr: longword; Addr: pbyte);
var
  tmpreg: longword;
begin
  (* Calculate the selected MAC address high register *)
  tmpreg := (Addr[5] shl 8) or Addr[4];
  (* Load the selected MAC address high register *)
  plongword(longword(ETH_MAC_ADDR_HBASE + MacAddr))^ := tmpreg;
  (* Calculate the selected MAC address low register *)
  tmpreg := (Addr[3] shl 24) or (Addr[2] shl 16) or (Addr[1] shl 8) or Addr[0];

  (* Load the selected MAC address low register *)
  plongword(longword(ETH_MAC_ADDR_LBASE + MacAddr))^ := tmpreg;
end;

(**
  * @brief  Enables the MAC transmission.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_MACTransmissionEnable(var heth: ETH_HandleTypeDef);
var
  tmpreg: longword;
begin
  (* Enable the MAC transmission *)
  heth.Instance^.MACCR := heth.Instance^.MACCR or ETH_MACCR_TE;

  (* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.MACCR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.MACCR := tmpreg;
end;

(**
  * @brief  Disables the MAC transmission.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_MACTransmissionDisable(var heth: ETH_HandleTypeDef);
var
  tmpreg: longword;
begin
  (* Disable the MAC transmission *)
  heth.Instance^.MACCR := heth.Instance^.MACCR and (not ETH_MACCR_TE);

  (* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.MACCR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.MACCR := tmpreg;
end;

(**
  * @brief  Enables the MAC reception.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_MACReceptionEnable(var heth: ETH_HandleTypeDef);
var
  tmpreg: longword;
begin
  (* Enable the MAC reception *)
  heth.Instance^.MACCR := heth.Instance^.MACCR or ETH_MACCR_RE;

  (* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.MACCR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.MACCR := tmpreg;
end;

(**
  * @brief  Disables the MAC reception.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_MACReceptionDisable(var heth: ETH_HandleTypeDef);
var
  tmpreg: longword;
begin
  (* Disable the MAC reception *)
  heth.Instance^.MACCR := heth.Instance^.MACCR and (not ETH_MACCR_RE);

  (* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.MACCR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.MACCR := tmpreg;
end;

(**
  * @brief  Enables the DMA transmission.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_DMATransmissionEnable(var heth: ETH_HandleTypeDef);
begin
  (* Enable the DMA transmission *)
  heth.Instance^.DMAOMR := heth.Instance^.DMAOMR or ETH_DMAOMR_ST;
end;

(**
  * @brief  Disables the DMA transmission.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_DMATransmissionDisable(var heth: ETH_HandleTypeDef);
begin
  (* Disable the DMA transmission *)
  heth.Instance^.DMAOMR := heth.Instance^.DMAOMR and (not ETH_DMAOMR_ST);
end;

(**
  * @brief  Enables the DMA reception.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_DMAReceptionEnable(var heth: ETH_HandleTypeDef);
begin
  (* Enable the DMA reception *)
  heth.Instance^.DMAOMR := heth.Instance^.DMAOMR or ETH_DMAOMR_SR;
end;

(**
  * @brief  Disables the DMA reception.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_DMAReceptionDisable(var heth: ETH_HandleTypeDef);
begin
  (* Disable the DMA reception *)
  heth.Instance^.DMAOMR := heth.Instance^.DMAOMR and (not ETH_DMAOMR_SR);
end;


(**
  * @brief  Clears the ETHERNET transmit FIFO.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  *)
procedure ETH_FlushTransmitFIFO(var heth: ETH_HandleTypeDef);
var
  tmpreg: longword;
begin
  (* Set the Flush Transmit FIFO bit *)
  heth.Instance^.DMAOMR := heth.Instance^.DMAOMR or ETH_DMAOMR_FTF;

  (* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles *)
  tmpreg := heth.Instance^.DMAOMR;
  HAL_Delay(ETH_REG_WRITE_DELAY);
  heth.Instance^.DMAOMR := tmpreg;
end;

function HAL_ETH_Init(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
  var
    tempreg, phyreg, hclk, tickstart, err: longword;
  begin
    tempreg := 0;
    phyreg := 0;
    hclk := 60000000;
    tickstart := 0;
    err := ETH_SUCCESS;

    if (heth.State = HAL_ETH_STATE_RESET) then
      begin
        (* Allocate lock resource and initialize it *)
        heth.Lock := HAL_UNLOCKED;
        (* Init the low level hardware : GPIO, CLOCK, NVIC. *)
        HAL_ETH_MspInit(heth);
      end;

    (* Enable SYSCFG Clock *)
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    (* Select MII or RMII Mode*)
    SYSCFG.PMC := SYSCFG.PMC and (not (SYSCFG_PMC_MII_RMII_SEL));
    SYSCFG.PMC := SYSCFG.PMC or heth.Init.MediaInterface;

    (* Ethernet Software reset *)
    (* Set the SWR bit: resets all MAC subsystem internal registers and logic *)
    (* After reset all the registers holds their respective reset values *)
    heth.Instance^.DMABMR := heth.Instance^.DMABMR or ETH_DMABMR_SR;

    (* Wait for software reset *)
    while ((heth.Instance^.DMABMR and ETH_DMABMR_SR) <> 0) do ;

    (*-------------------------------- MAC Initialization ----------------------*)
    (* Get the ETHERNET MACMIIAR value *)
    tempreg := heth.Instance^.MACMIIAR;
    (* Clear CSR Clock Range CR[2:0] bits *)
    tempreg := tempreg and (ETH_MACMIIAR_CR_MASK);

    (* Get hclk frequency value *)
    hclk := HAL_RCC_GetHCLKFreq();

    (* Set CR bits depending on hclk value *)
    if ((hclk >= 20000000) and (hclk < 35000000)) then
      (* CSR Clock Range between 20-35 MHz *)
      tempreg := tempreg or ETH_MACMIIAR_CR_Div16
    else if ((hclk >= 35000000) and (hclk < 60000000)) then
      (* CSR Clock Range between 35-60 MHz *)
      tempreg := tempreg or ETH_MACMIIAR_CR_Div26
    else if ((hclk >= 60000000) and (hclk < 100000000)) then
      (* CSR Clock Range between 60-100 MHz *)
      tempreg := tempreg or ETH_MACMIIAR_CR_Div42
    else if ((hclk >= 100000000) and (hclk < 150000000)) then
      (* CSR Clock Range between 100-150 MHz *)
      tempreg := tempreg or ETH_MACMIIAR_CR_Div62
    else (* ((hclk >= 150000000)and(hclk <:= 200000000)) *)
      (* CSR Clock Range between 150-216 MHz *)
      tempreg := tempreg or ETH_MACMIIAR_CR_Div102;

    (* Write to ETHERNET MAC MIIAR: Configure the ETHERNET CSR Clock Range *)
    heth.Instance^.MACMIIAR := tempreg;

    (*-------------------- PHY initialization and configuration ----------------*)
    (* Put the PHY in reset mode *)
    if ((HAL_ETH_WritePHYRegister(heth, PHY_BCR, PHY_RESET)) <> HAL_OK) then
      begin
        (* In case of write timeout *)
        err := ETH_ERROR;

        (* Config MAC and DMA *)
        ETH_MACDMAConfig(heth, err);

        (* Set the ETH peripheral state to READY *)
        heth.State := HAL_ETH_STATE_READY;

        (* Return HAL_ERROR *)
        exit(HAL_ERROR);
      end;

    (* Delay to assure PHY reset *)
    HAL_Delay(PHY_RESET_DELAY);

    if ((heth.Init).AutoNegotiation <> ETH_AUTONEGOTIATION_DISABLE) then
      begin
        (* Get tick *)
        tickstart := HAL_GetTick();

        (* We wait for linked status *)
        repeat
          HAL_ETH_ReadPHYRegister(heth, PHY_BSR, phyreg);

          (* Check for the Timeout *)
          if ((HAL_GetTick() - tickstart) > LINKED_STATE_TIMEOUT_VALUE) then
            begin
              (* In case of write timeout *)
              err := ETH_ERROR;

              (* Config MAC and DMA *)
              ETH_MACDMAConfig(heth, err);

              heth.State := HAL_ETH_STATE_READY;

              (* Process Unlocked *)
              __HAL_UNLOCK(heth.lock);

              exit(HAL_TIMEOUT);
            end;
        until (((phyreg and PHY_LINKED_STATUS) = PHY_LINKED_STATUS));


        (* Enable Auto-Negotiation *)
        if ((HAL_ETH_WritePHYRegister(heth, PHY_BCR, PHY_AUTONEGOTIATION)) <> HAL_OK) then
          begin
            (* In case of write timeout *)
            err := ETH_ERROR;

            (* Config MAC and DMA *)
            ETH_MACDMAConfig(heth, err);

            (* Set the ETH peripheral state to READY *)
            heth.State := HAL_ETH_STATE_READY;

            (* Return HAL_ERROR *)
            exit(HAL_ERROR);
          end;

        (* Get tick *)
        tickstart := HAL_GetTick();

        (* Wait until the auto-negotiation will be completed *)
        repeat
          HAL_ETH_ReadPHYRegister(heth, PHY_BSR, phyreg);

          (* Check for the Timeout *)
          if ((HAL_GetTick() - tickstart) > AUTONEGO_COMPLETED_TIMEOUT_VALUE) then
            begin
              (* In case of write timeout *)
              err := ETH_ERROR;

              (* Config MAC and DMA *)
              ETH_MACDMAConfig(heth, err);

              heth.State := HAL_ETH_STATE_READY;

              (* Process Unlocked *)
              __HAL_UNLOCK(heth.lock);

              exit(HAL_TIMEOUT);
            end;
        until (((phyreg and PHY_AUTONEGO_COMPLETE) = PHY_AUTONEGO_COMPLETE));

        (* Read the result of the auto-negotiation *)
        if ((HAL_ETH_ReadPHYRegister(heth, PHY_SR, phyreg)) <> HAL_OK) then
          begin
            (* In case of write timeout *)
            err := ETH_ERROR;

            (* Config MAC and DMA *)
            ETH_MACDMAConfig(heth, err);

            (* Set the ETH peripheral state to READY *)
            heth.State := HAL_ETH_STATE_READY;

            (* Return HAL_ERROR *)
            exit(HAL_ERROR);
          end;

        (* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process *)
        if ((phyreg and PHY_DUPLEX_STATUS) <> 0) then
          (* Set Ethernet duplex mode to Full-duplex following the auto-negotiation *)
          (heth.Init).DuplexMode := ETH_MODE_FULLDUPLEX
        else
          (* Set Ethernet duplex mode to Half-duplex following the auto-negotiation *)
          (heth.Init).DuplexMode := ETH_MODE_HALFDUPLEX;

        (* Configure the MAC with the speed fixed by the auto-negotiation process *)
        if ((phyreg and PHY_SPEED_STATUS) = PHY_SPEED_STATUS) then
          (* Set Ethernet speed to 10M following the auto-negotiation *)
          (heth.Init).Speed := ETH_SPEED_10M
        else
          (* Set Ethernet speed to 100M following the auto-negotiation *)
          (heth.Init).Speed := ETH_SPEED_100M;
      end
    else (* AutoNegotiation Disable *)
      begin
        (* Set MAC Speed and Duplex Mode *)
        if (HAL_ETH_WritePHYRegister(heth, PHY_BCR, (((heth.Init).DuplexMode shr 3) or ((heth.Init).Speed shr 1))) <> HAL_OK) then
          begin
            (* In case of write timeout *)
            err := ETH_ERROR;

            (* Config MAC and DMA *)
            ETH_MACDMAConfig(heth, err);

            (* Set the ETH peripheral state to READY *)
            heth.State := HAL_ETH_STATE_READY;

            (* Return HAL_ERROR *)
            exit(HAL_ERROR);
          end;

        (* Delay to assure PHY configuration *)
        HAL_Delay(PHY_CONFIG_DELAY);
      end;

    (* Config MAC and DMA *)
    ETH_MACDMAConfig(heth, err);

    (* Set ETH HAL State to Ready *)
    heth.State := HAL_ETH_STATE_READY;

    (* Return function status *)
    exit(HAL_OK);
  end;

function HAL_ETH_DeInit(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
  begin
    (* Set the ETH peripheral state to BUSY *)
    heth.State := HAL_ETH_STATE_BUSY;

    (* De-Init the low level hardware : GPIO, CLOCK, NVIC. *)
    HAL_ETH_MspDeInit(heth);

    (* Set ETH HAL state to Disabled *)
    heth.State := HAL_ETH_STATE_RESET;

    (* Release Lock *)
    __HAL_UNLOCK(heth.lock);

    (* Return function status *)
    exit(HAL_OK);
  end;

procedure HAL_ETH_MspInit_stub(var heth: ETH_HandleTypeDef); assembler; nostackframe; public name 'HAL_ETH_MspInit';
asm
  .weak HAL_ETH_MspInit
end;

procedure HAL_ETH_MspDeInit_stub(var heth: ETH_HandleTypeDef); assembler; nostackframe; public name 'HAL_ETH_MspDeInit';
asm
  .weak HAL_ETH_MspDeInit
end;

function HAL_ETH_DMATxDescListInit(var heth: ETH_HandleTypeDef; DMATxDescTab: PETH_DMADescTypeDef; var TxBuff; TxBuffCount: longword): HAL_StatusTypeDef;
var
  i: longword;
  dmatxdesc: ^ETH_DMADescTypeDef;
begin
  (* Process Locked *)
  __HAL_Lock(heth.lock);

  (* Set the ETH peripheral state to BUSY *)
  heth.State := HAL_ETH_STATE_BUSY;

  (* Set the DMATxDescToSet pointer with the first one of the DMATxDescTab list *)
  heth.TxDesc := @DMATxDescTab;

  (* Fill each DMATxDesc descriptor with the right values *)
  for i := 0 to txbuffcount - 1 do
    begin
      (* Get the pointer on the ith member of the Tx Desc list *)
      dmatxdesc := @DMATxDescTab[i];

      (* Set Second Address Chained bit *)
      dmatxdesc^.Status := ETH_DMATXDESC_TCH;

      (* Set Buffer1 address pointer *)
      dmatxdesc^.Buffer1Addr := @pbyte(@TxBuff)[i * ETH_TX_BUF_SIZE];

      if ((heth.Init).ChecksumMode = ETH_CHECKSUM_BY_HARDWARE) then
        (* Set the DMA Tx descriptors checksum insertion *)
        dmatxdesc^.Status := dmatxdesc^.Status or ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL;

      (* Initialize the next descriptor with the Next Descriptor Polling Enable *)
      if (i < (TxBuffCount - 1)) then
        (* Set next descriptor address register with next descriptor base address *)
        dmatxdesc^.Buffer2NextDescAddr := @DMATxDescTab[i + 1]
      else
        (* For last descriptor, set next descriptor address register equal to the first descriptor base address *)
        dmatxdesc^.Buffer2NextDescAddr := DMATxDescTab;
    end;

  (* Set Transmit Descriptor List Address Register *)
  heth.Instance^.DMATDLAR := ptruint(DMATxDescTab);

  (* Set ETH HAL State to Ready *)
  heth.State := HAL_ETH_STATE_READY;

  (* Process Unlocked *)
  __HAL_Unlock(heth.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ETH_DMARxDescListInit(var heth: ETH_HandleTypeDef; DMARxDescTab: PETH_DMADescTypeDef; var RxBuff; RxBuffCount: longword): HAL_StatusTypeDef;
var
  i: longword;
  DMARxDesc: ^ETH_DMADescTypeDef;
begin
  (* Process Locked *)
  __HAL_Lock(heth.lock);

  (* Set the ETH peripheral state to BUSY *)
  heth.State := HAL_ETH_STATE_BUSY;

  (* Set the Ethernet RxDesc pointer with the first one of the DMARxDescTab list *)
  heth.RxDesc := DMARxDescTab;

  (* Fill each DMARxDesc descriptor with the right values *)
  for i := 0 to RxBuffCount - 1 do
    begin
      (* Get the pointer on the ith member of the Rx Desc list *)
      DMARxDesc := @DMARxDescTab[i];

      (* Set Own bit of the Rx descriptor Status *)
      DMARxDesc^.Status := ETH_DMARXDESC_OWN;

      (* Set Buffer1 size and Second Address Chained bit *)
      DMARxDesc^.ControlBufferSize := ETH_DMARXDESC_RCH or ETH_RX_BUF_SIZE;

      (* Set Buffer1 address pointer *)
      DMARxDesc^.Buffer1Addr := @pbyte(@RxBuff)[i * ETH_RX_BUF_SIZE];

      if ((heth.Init).RxMode = ETH_RXINTERRUPT_MODE) then
        (* Enable Ethernet DMA Rx Descriptor interrupt *)
        DMARxDesc^.ControlBufferSize := DMARxDesc^.ControlBufferSize and (not ETH_DMARXDESC_DIC);

      (* Initialize the next descriptor with the Next Descriptor Polling Enable *)
      if (i < (RxBuffCount - 1)) then
        (* Set next descriptor address register with next descriptor base address *)
        DMARxDesc^.Buffer2NextDescAddr := @DMARxDescTab[i+1]
      else
        (* For last descriptor, set next descriptor address register equal to the first descriptor base address *)
        DMARxDesc^.Buffer2NextDescAddr := DMARxDescTab;
    end;

  (* Set Receive Descriptor List Address Register *)
  heth.Instance^.DMARDLAR := ptruint(DMARxDescTab);

  (* Set ETH HAL State to Ready *)
  heth.State := HAL_ETH_STATE_READY;

  (* Process Unlocked *)
  __HAL_Unlock(heth.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ETH_TransmitFrame(var heth: ETH_HandleTypeDef; FrameLength: longword): HAL_StatusTypeDef;
var
  bufcount, size, i: longword;
begin
  (* Process Locked *)
  __HAL_Lock(heth.lock);

  (* Set the ETH peripheral state to BUSY *)
  heth.State := HAL_ETH_STATE_BUSY;

  if (FrameLength = 0) then
    begin
      (* Set ETH HAL state to READY *)
      heth.State := HAL_ETH_STATE_READY;

      (* Process Unlocked *)
      __HAL_Unlock(heth.lock);

      exit(HAL_ERROR);
    end;

  (* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) *)
  if ((heth.TxDesc^.Status and ETH_DMATXDESC_OWN) <> 0) then
    begin
      (* OWN bit set *)
      heth.State := HAL_ETH_STATE_BUSY_TX;

      (* Process Unlocked *)
      __HAL_Unlock(heth.lock);

      exit(HAL_ERROR);
    end;

  (* Get the number of needed Tx buffers for the current frame *)
  if (FrameLength > ETH_TX_BUF_SIZE) then
    begin
      bufcount := FrameLength div ETH_TX_BUF_SIZE;

      if longword(FrameLength-longword(bufcount*longword(ETH_TX_BUF_SIZE)))<>0 then
        Inc(bufcount);
    end
  else
    bufcount := 1;

  if bufcount = 1 then
    begin
      (* Set LAST and FIRST segment *)
      heth.TxDesc^.Status := heth.TxDesc^.Status or ETH_DMATXDESC_FS or ETH_DMATXDESC_LS;
      (* Set frame size *)
      heth.TxDesc^.ControlBufferSize := (FrameLength and ETH_DMATXDESC_TBS1);
      (* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA *)
      heth.TxDesc^.Status := heth.TxDesc^.Status or ETH_DMATXDESC_OWN;
      (* Point to next descriptor *)
      heth.TxDesc := heth.TxDesc^.Buffer2NextDescAddr;
    end
  else
    begin
      for i := 0 to bufcount - 1 do
        begin
          (* Clear FIRST and LAST segment bits *)
          heth.TxDesc^.Status := heth.TxDesc^.Status and (not (ETH_DMATXDESC_FS or ETH_DMATXDESC_LS));

          if (i = 0) then
            (* Setting the first segment bit *)
            heth.TxDesc^.Status := heth.TxDesc^.Status or ETH_DMATXDESC_FS;

          (* Program size *)
          heth.TxDesc^.ControlBufferSize := (ETH_TX_BUF_SIZE and ETH_DMATXDESC_TBS1);

          if i = (bufcount - 1) then
            begin
              (* Setting the last segment bit *)
              heth.TxDesc^.Status := heth.TxDesc^.Status or ETH_DMATXDESC_LS;
              size := FrameLength - (bufcount - 1) * ETH_TX_BUF_SIZE;
              heth.TxDesc^.ControlBufferSize := size and ETH_DMATXDESC_TBS1;
            end;

          (* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA *)
          heth.TxDesc^.Status := heth.TxDesc^.Status or ETH_DMATXDESC_OWN;
          (* point to next descriptor *)
          heth.TxDesc := heth.TxDesc^.Buffer2NextDescAddr;
        end;
    end;

  (* When Tx Buffer unavailable flag is set: clear it and resume transmission *)
  if (heth.Instance^.DMASR and ETH_DMASR_TBUS) <> 0 then
    begin
      (* Clear TBUS ETHERNET DMA flag *)
      heth.Instance^.DMASR := ETH_DMASR_TBUS;
      (* Resume DMA transmission*)
      heth.Instance^.DMATPDR := 0;
    end;

  (* Set ETH HAL State to Ready *)
  heth.State := HAL_ETH_STATE_READY;

  (* Process Unlocked *)
  __HAL_Unlock(heth.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ETH_GetReceivedFrame(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
var
  framelength: longword;
begin
  (* Process Locked *)
  __HAL_Lock(heth.lock);

  (* Check the ETH state to BUSY *)
  heth.State := HAL_ETH_STATE_BUSY;

  (* Check if segment is not owned by DMA *)
  (* (((heth.RxDesc^.Status and ETH_DMARXDESC_OWN) = RESET) and ((heth.RxDesc^.Status and ETH_DMARXDESC_LS) <> RESET)) *)
  if (heth.RxDesc^.Status and ETH_DMARXDESC_OWN) = 0 then
    begin
      (* Check if last segment *)
      if (heth.RxDesc^.Status and ETH_DMARXDESC_LS) <> 0 then
        begin
          (* increment segment count *)
          Inc(heth.RxFrameInfos.SegCount);

          (* Check if last segment is first segment: one segment contains the frame *)
          if ((heth.RxFrameInfos).SegCount = 1) then
            (heth.RxFrameInfos).FSRxDesc := heth.RxDesc;

          heth.RxFrameInfos.LSRxDesc := heth.RxDesc;

          (* Get the Frame Length of the received packet: substruct 4 bytes of the CRC *)
          framelength := ((heth.RxDesc^.Status and ETH_DMARXDESC_FL) shr ETH_DMARXDESC_FRAMELENGTHSHIFT) - 4;
          heth.RxFrameInfos.length := framelength;

          (* Get the address of the buffer start address *)
          heth.RxFrameInfos.buffer := heth.RxFrameInfos.FSRxDesc^.Buffer1Addr;
          (* point to next descriptor *)
          heth.RxDesc := heth.RxDesc^.Buffer2NextDescAddr;

          (* Set HAL State to Ready *)
          heth.State := HAL_ETH_STATE_READY;

          (* Process Unlocked *)
          __HAL_Unlock(heth.lock);

          (* Return function status *)
          exit(HAL_OK);
        end
      (* Check if first segment *)
      else if (heth.RxDesc^.Status and ETH_DMARXDESC_FS) <> 0 then
        begin
          (heth.RxFrameInfos).FSRxDesc := heth.RxDesc;
          (heth.RxFrameInfos).LSRxDesc := nil;
          (heth.RxFrameInfos).SegCount := 1;
          (* Point to next descriptor *)
          heth.RxDesc := heth.RxDesc^.Buffer2NextDescAddr;
        end
      (* Check if intermediate segment *)
      else
        begin
          Inc(heth.RxFrameInfos.SegCount);
          (* Point to next descriptor *)
          heth.RxDesc := heth.RxDesc^.Buffer2NextDescAddr;
        end;
    end;

  (* Set ETH HAL State to Ready *)
  heth.State := HAL_ETH_STATE_READY;

  (* Process Unlocked *)
  __HAL_Unlock(heth.lock);

  (* Return function status *)
  exit(HAL_ERROR);
end;

function HAL_ETH_ReadPHYRegister(var heth: ETH_HandleTypeDef; PHYReg: word; var RegValue: longword): HAL_StatusTypeDef;
var
  tmpreg, tickstart: longword;
begin
  tmpreg := 0;
  tickstart := 0;

  (* Check the ETH peripheral state *)
  if (heth.State = HAL_ETH_STATE_BUSY_RD) then
    exit(HAL_BUSY);

  (* Set ETH HAL State to BUSY_RD *)
  heth.State := HAL_ETH_STATE_BUSY_RD;

  (* Get the ETHERNET MACMIIAR value *)
  tmpreg := heth.Instance^.MACMIIAR;

  (* Keep only the CSR Clock Range CR[2:0] bits value *)
  tmpreg := tmpreg and (not ETH_MACMIIAR_CR_MASK);

  (* Prepare the MII address register value *)
  tmpreg := tmpreg or ((heth.Init.PhyAddress shl 11) and ETH_MACMIIAR_PA); (* Set the PHY device address   *)
  tmpreg := tmpreg or ((PHYReg shl 6) and ETH_MACMIIAR_MR);                   (* Set the PHY register address *)
  tmpreg := tmpreg and (not ETH_MACMIIAR_MW);                                           (* Set the read mode            *)
  tmpreg := tmpreg or ETH_MACMIIAR_MB;                                            (* Set the MII Busy bit         *)

  (* Write the result value into the MII Address register *)
  heth.Instance^.MACMIIAR := tmpreg;

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Check for the Busy flag *)
  while ((tmpreg and ETH_MACMIIAR_MB) = ETH_MACMIIAR_MB) do
  begin
    (* Check for the Timeout *)
    if ((HAL_GetTick() - tickstart) > PHY_READ_TO) then
    begin
      heth.State := HAL_ETH_STATE_READY;

      (* Process Unlocked *)
      __HAL_Unlock(heth.lock);

      exit(HAL_TIMEOUT);
    end;

    tmpreg := heth.Instance^.MACMIIAR;
  end;

  (* Get MACMIIDR value *)
  RegValue := word(heth.Instance^.MACMIIDR);

  (* Set ETH HAL State to READY *)
  heth.State := HAL_ETH_STATE_READY;

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ETH_WritePHYRegister(var heth: ETH_HandleTypeDef; PHYReg: word; RegValue: longword): HAL_StatusTypeDef;
var
  tmpreg, tickstart: longword;
begin
  tmpreg := 0;
  tickstart := 0;

  (* Check the ETH peripheral state *)
  if (heth.State = HAL_ETH_STATE_BUSY_WR) then
    exit(HAL_BUSY);

  (* Set ETH HAL State to BUSY_WR *)
  heth.State := HAL_ETH_STATE_BUSY_WR;

  (* Get the ETHERNET MACMIIAR value *)
  tmpreg := heth.Instance^.MACMIIAR;

  (* Keep only the CSR Clock Range CR[2:0] bits value *)
  tmpreg := tmpreg and (not ETH_MACMIIAR_CR_MASK);

  (* Prepare the MII register address value *)
  tmpreg := tmpreg or ((heth.Init.PhyAddress shl 11) and ETH_MACMIIAR_PA); (* Set the PHY device address *)
  tmpreg := tmpreg or ((PHYReg shl 6) and ETH_MACMIIAR_MR);                 (* Set the PHY register address *)
  tmpreg := tmpreg or ETH_MACMIIAR_MW;                                          (* Set the write mode *)
  tmpreg := tmpreg or ETH_MACMIIAR_MB;                                          (* Set the MII Busy bit *)

  (* Give the value to the MII data register *)
  heth.Instance^.MACMIIDR := word(RegValue);

  (* Write the result value into the MII Address register *)
  heth.Instance^.MACMIIAR := tmpreg;

  (* Get tick *)
  tickstart := HAL_GetTick();

  (* Check for the Busy flag *)
  while ((tmpreg and ETH_MACMIIAR_MB) = ETH_MACMIIAR_MB) do
  begin
    (* Check for the Timeout *)
    if ((HAL_GetTick() - tickstart) > PHY_WRITE_TO) then
    begin
      heth.State := HAL_ETH_STATE_READY;

      (* Process Unlocked *)
      __HAL_Unlock(heth.lock);

      exit(HAL_TIMEOUT);
    end;

    tmpreg := heth.Instance^.MACMIIAR;
  end;

  (* Set ETH HAL State to READY *)
  heth.State := HAL_ETH_STATE_READY;

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ETH_GetReceivedFrame_IT(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
  var
    descriptorscancounter: longword;
  begin
    descriptorscancounter := 0;

    (* Process Locked *)
    __HAL_Lock(heth.lock);

    (* Set ETH HAL State to BUSY *)
    heth.State := HAL_ETH_STATE_BUSY;

    (* Scan descriptors owned by CPU *)
    while (((heth.RxDesc^.Status and ETH_DMARXDESC_OWN) = 0) and (descriptorscancounter < ETH_RXBUFNB)) do
      begin
        (* Just for security *)
        Inc(descriptorscancounter);

        (* Check if first segment in frame *)
        (* ((heth.RxDesc^.Status and ETH_DMARXDESC_FS) <> RESET) and ((heth.RxDesc^.Status and ETH_DMARXDESC_LS) = RESET)) *)
        if ((heth.RxDesc^.Status and (ETH_DMARXDESC_FS or ETH_DMARXDESC_LS)) = ETH_DMARXDESC_FS) then
          begin
            heth.RxFrameInfos.FSRxDesc := heth.RxDesc;
            heth.RxFrameInfos.SegCount := 1;
            (* Point to next descriptor *)
            heth.RxDesc := heth.RxDesc^.Buffer2NextDescAddr;
          end
        (* Check if intermediate segment *)
        (* ((heth.RxDesc^.Status and ETH_DMARXDESC_LS) = RESET)and ((heth.RxDesc^.Status and ETH_DMARXDESC_FS) = RESET)) *)
        else if ((heth.RxDesc^.Status and (ETH_DMARXDESC_LS or ETH_DMARXDESC_FS)) = 0) then
          begin
            (* Increment segment count *)
            Inc(heth.RxFrameInfos.SegCount);
            (* Point to next descriptor *)
            heth.RxDesc := heth.RxDesc^.Buffer2NextDescAddr;
          end
        (* Should be last segment *)
        else
          begin
            (* Last segment *)
            heth.RxFrameInfos.LSRxDesc := heth.RxDesc;

            (* Increment segment count *)
            Inc(heth.RxFrameInfos.SegCount);

            (* Check if last segment is first segment: one segment contains the frame *)
            if ((heth.RxFrameInfos.SegCount) = 1) then
              heth.RxFrameInfos.FSRxDesc := heth.RxDesc;

            (* Get the Frame Length of the received packet: substruct 4 bytes of the CRC *)
            heth.RxFrameInfos.length := (((heth.RxDesc)^.Status and ETH_DMARXDESC_FL) shr ETH_DMARXDESC_FRAMELENGTHSHIFT) - 4;

            (* Get the address of the buffer start address *)
            heth.RxFrameInfos.buffer := heth.RxFrameInfos.FSRxDesc^.Buffer1Addr;

            (* Point to next descriptor *)
            heth.RxDesc := heth.RxDesc^.Buffer2NextDescAddr;

            (* Set HAL State to Ready *)
            heth.State := HAL_ETH_STATE_READY;

            (* Process Unlocked *)
            __HAL_Unlock(heth.lock);

            (* Return function status *)
            exit(HAL_OK);
          end;
      end;

    (* Set HAL State to Ready *)
    heth.State := HAL_ETH_STATE_READY;

    (* Process Unlocked *)
    __HAL_Unlock(heth.lock);

    (* Return function status *)
    exit(HAL_ERROR);
  end;

procedure HAL_ETH_IRQHandler(var heth: ETH_HandleTypeDef);
  begin
    (* Frame received *)
    if (__HAL_ETH_DMA_GET_FLAG(heth, ETH_DMA_FLAG_R)) then
      begin
        (* Receive complete callback *)
        HAL_ETH_RxCpltCallback(heth);

        (* Clear the Eth DMA Rx IT pending bits *)
        __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_IT_R);

        (* Set HAL State to Ready *)
        heth.State := HAL_ETH_STATE_READY;

        (* Process Unlocked *)
        __HAL_Unlock(heth.lock);

      end
    (* Frame transmitted *)
    else if (__HAL_ETH_DMA_GET_FLAG(heth, ETH_DMA_FLAG_T)) then
      begin
        (* Transfer complete callback *)
        HAL_ETH_TxCpltCallback(heth);

        (* Clear the Eth DMA Tx IT pending bits *)
        __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_IT_T);

        (* Set HAL State to Ready *)
        heth.State := HAL_ETH_STATE_READY;

        (* Process Unlocked *)
        __HAL_Unlock(heth.lock);
      end;

    (* Clear the interrupt flags *)
    __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_IT_NIS);

    (* ETH DMA Error *)
    if (__HAL_ETH_DMA_GET_FLAG(heth, ETH_DMA_FLAG_AIS)) then
      begin
        (* Ethernet Error callback *)
        HAL_ETH_ErrorCallback(heth);

        (* Clear the interrupt flags *)
        __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_FLAG_AIS);

        (* Set HAL State to Ready *)
        heth.State := HAL_ETH_STATE_READY;

        (* Process Unlocked *)
        __HAL_Unlock(heth.lock);
      end;
  end;

procedure HAL_ETH_TxCpltCallback_stub(var heth: ETH_HandleTypeDef); assembler; nostackframe; public name 'HAL_ETH_TxCpltCallback';
  asm
    .weak HAL_ETH_TxCpltCallback
  end;

procedure HAL_ETH_RxCpltCallback_stub(var heth: ETH_HandleTypeDef); assembler; nostackframe; public name 'HAL_ETH_RxCpltCallback';
  asm
    .weak HAL_ETH_RxCpltCallback
  end;

procedure HAL_ETH_ErrorCallback_stub(var heth: ETH_HandleTypeDef); assembler; nostackframe; public name 'HAL_ETH_ErrorCallback';
  asm
    .weak HAL_ETH_ErrorCallback
  end;

function HAL_ETH_Start(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Process Locked *)
  __HAL_Lock(heth.lock);

  (* Set the ETH peripheral state to BUSY *)
  heth.State := HAL_ETH_STATE_BUSY;

  (* Enable transmit state machine of the MAC for transmission on the MII *)
  ETH_MACTransmissionEnable(heth);

  (* Enable receive state machine of the MAC for reception from the MII *)
  ETH_MACReceptionEnable(heth);

  (* Flush Transmit FIFO *)
  ETH_FlushTransmitFIFO(heth);

  (* Start DMA transmission *)
  ETH_DMATransmissionEnable(heth);

  (* Start DMA reception *)
  ETH_DMAReceptionEnable(heth);

  (* Set the ETH state to READY*)
  heth.State := HAL_ETH_STATE_READY;

  (* Process Unlocked *)
  __HAL_Unlock(heth.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ETH_Stop(var heth: ETH_HandleTypeDef): HAL_StatusTypeDef;
begin
  (* Process Locked *)
  __HAL_Lock(heth.lock);

  (* Set the ETH peripheral state to BUSY *)
  heth.State := HAL_ETH_STATE_BUSY;

  (* Stop DMA transmission *)
  ETH_DMATransmissionDisable(heth);

  (* Stop DMA reception *)
  ETH_DMAReceptionDisable(heth);

  (* Disable receive state machine of the MAC for reception from the MII *)
  ETH_MACReceptionDisable(heth);

  (* Flush Transmit FIFO *)
  ETH_FlushTransmitFIFO(heth);

  (* Disable transmit state machine of the MAC for transmission on the MII *)
  ETH_MACTransmissionDisable(heth);

  (* Set the ETH state*)
  heth.State := HAL_ETH_STATE_READY;

  (* Process Unlocked *)
  __HAL_Unlock(heth.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ETH_ConfigMAC(var heth: ETH_HandleTypeDef; macconf: PETH_MACInitTypeDef): HAL_StatusTypeDef;
var
  tmpreg: longword;
begin
  (* Process Locked *)
  __HAL_Lock(heth.lock);

  (* Set the ETH peripheral state to BUSY *)
  heth.State := HAL_ETH_STATE_BUSY;

  if (macconf <> nil) then
  begin
    (*------------------------ ETHERNET MACCR Configuration --------------------*)
    (* Get the ETHERNET MACCR value *)
    tmpreg := heth.Instance^.MACCR;
    (* Clear WD, PCE, PS, TE and RE bits *)
    tmpreg := tmpreg and (ETH_MACCR_CLEAR_MASK);

    tmpreg := tmpreg or (macconf^.Watchdog or macconf^.Jabber or macconf^.InterFrameGap or macconf^.CarrierSense or
      (heth.Init).Speed or macconf^.ReceiveOwn or macconf^.LoopbackMode or (heth.Init).DuplexMode or
      macconf^.ChecksumOffload or macconf^.RetryTransmission or macconf^.AutomaticPadCRCStrip or macconf^.BackOffLimit or
      macconf^.DeferralCheck);

    (* Write to ETHERNET MACCR *)
    heth.Instance^.MACCR := tmpreg;

    (* Wait until the write operation will be taken into account :
    at least four TX_CLK/RX_CLK clock cycles *)
    tmpreg := heth.Instance^.MACCR;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    heth.Instance^.MACCR := tmpreg;

    (*----------------------- ETHERNET MACFFR Configuration --------------------*)
    (* Write to ETHERNET MACFFR *)
    heth.Instance^.MACFFR := (macconf^.ReceiveAll or macconf^.SourceAddrFilter or macconf^.PassControlFrames or
      macconf^.BroadcastFramesReception or macconf^.DestinationAddrFilter or macconf^.PromiscuousMode or
      macconf^.MulticastFramesFilter or macconf^.UnicastFramesFilter);

     (* Wait until the write operation will be taken into account :
     at least four TX_CLK/RX_CLK clock cycles *)
    tmpreg := heth.Instance^.MACFFR;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    heth.Instance^.MACFFR := tmpreg;

    (*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*)
    (* Write to ETHERNET MACHTHR *)
    heth.Instance^.MACHTHR := macconf^.HashTableHigh;

    (* Write to ETHERNET MACHTLR *)
    heth.Instance^.MACHTLR := macconf^.HashTableLow;
    (*----------------------- ETHERNET MACFCR Configuration --------------------*)

    (* Get the ETHERNET MACFCR value *)
    tmpreg := heth.Instance^.MACFCR;
    (* Clear xx bits *)
    tmpreg := tmpreg and (ETH_MACFCR_CLEAR_MASK);

    tmpreg := tmpreg or ((macconf^.PauseTime shl 16) or macconf^.ZeroQuantaPause or macconf^.PauseLowThreshold or
      macconf^.UnicastPauseFrameDetect or macconf^.ReceiveFlowControl or macconf^.TransmitFlowControl);

    (* Write to ETHERNET MACFCR *)
    heth.Instance^.MACFCR := tmpreg;

     (* Wait until the write operation will be taken into account :
     at least four TX_CLK/RX_CLK clock cycles *)
    tmpreg := heth.Instance^.MACFCR;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    heth.Instance^.MACFCR := tmpreg;

    (*----------------------- ETHERNET MACVLANTR Configuration -----------------*)
    heth.Instance^.MACVLANTR := (macconf^.VLANTagComparison or macconf^.VLANTagIdentifier);

      (* Wait until the write operation will be taken into account :
      at least four TX_CLK/RX_CLK clock cycles *)
    tmpreg := heth.Instance^.MACVLANTR;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    heth.Instance^.MACVLANTR := tmpreg;
  end
  else (* macconf = nil : here we just configure Speed and Duplex mode *)
  begin
    (*------------------------ ETHERNET MACCR Configuration --------------------*)
    (* Get the ETHERNET MACCR value *)
    tmpreg := heth.Instance^.MACCR;

    (* Clear FES and DM bits *)
    tmpreg := tmpreg and (not ($00004800));

    tmpreg := tmpreg or (heth.Init.Speed or heth.Init.DuplexMode);

    (* Write to ETHERNET MACCR *)
    heth.Instance^.MACCR := tmpreg;

    (* Wait until the write operation will be taken into account:
    at least four TX_CLK/RX_CLK clock cycles *)
    tmpreg := heth.Instance^.MACCR;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    heth.Instance^.MACCR := tmpreg;
  end;

  (* Set the ETH state to Ready *)
  heth.State := HAL_ETH_STATE_READY;

  (* Process Unlocked *)
  __HAL_Unlock(heth.lock);

  (* Return function status *)
  exit(HAL_OK);
end;

function HAL_ETH_ConfigDMA(var heth: ETH_HandleTypeDef; dmaconf: PETH_DMAInitTypeDef): HAL_StatusTypeDef;
  var
    tmpreg: longword;
  begin
    (* Process Locked *)
    __HAL_Lock(heth.lock);

    (* Set the ETH peripheral state to BUSY *)
    heth.State := HAL_ETH_STATE_BUSY;

    (*----------------------- ETHERNET DMAOMR Configuration --------------------*)
    (* Get the ETHERNET DMAOMR value *)
    tmpreg := heth.Instance^.DMAOMR;
    (* Clear xx bits *)
    tmpreg := tmpreg and (ETH_DMAOMR_CLEAR_MASK);

    tmpreg := tmpreg or (dmaconf^.DropTCPIPChecksumErrorFrame or dmaconf^.ReceiveStoreForward or dmaconf^.FlushReceivedFrame or
      dmaconf^.TransmitStoreForward or dmaconf^.TransmitThresholdControl or dmaconf^.ForwardErrorFrames or
      dmaconf^.ForwardUndersizedGoodFrames or dmaconf^.ReceiveThresholdControl or dmaconf^.SecondFrameOperate);

    (* Write to ETHERNET DMAOMR *)
    heth.Instance^.DMAOMR := tmpreg;

    (* Wait until the write operation will be taken into account:
    at least four TX_CLK/RX_CLK clock cycles *)
    tmpreg := heth.Instance^.DMAOMR;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    heth.Instance^.DMAOMR := tmpreg;

    (*----------------------- ETHERNET DMABMR Configuration --------------------*)
    heth.Instance^.DMABMR := (dmaconf^.AddressAlignedBeats or dmaconf^.FixedBurst or dmaconf^.RxDMABurstLength or
      (* !! if 4xPBL is selected for Tx or Rx it is applied for the other *)
      dmaconf^.TxDMABurstLength or dmaconf^.EnhancedDescriptorFormat or (dmaconf^.DescriptorSkipLength shl 2) or
      dmaconf^.DMAArbitration or ETH_DMABMR_USP); (* Enable use of separate PBL for Rx and Tx *)

     (* Wait until the write operation will be taken into account:
        at least four TX_CLK/RX_CLK clock cycles *)
    tmpreg := heth.Instance^.DMABMR;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    heth.Instance^.DMABMR := tmpreg;

    (* Set the ETH state to Ready *)
    heth.State := HAL_ETH_STATE_READY;

    (* Process Unlocked *)
    __HAL_Unlock(heth.lock);

    (* Return function status *)
    exit(HAL_OK);
  end;

function HAL_ETH_GetState(var heth: ETH_HandleTypeDef): HAL_ETH_StateTypeDef;
  begin
    (* Return ETH state *)
    exit(heth.State);
  end;

end.
