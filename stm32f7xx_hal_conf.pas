(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_conf_template.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   HAL configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to stm32f7xx_hal_conf.h.
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
unit stm32f7xx_hal_conf;

interface

uses
  stm32f7xx_hal_eth;

(* ########################## HSE/HSI Values adaptation #####################  *)

(**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).
   *)

const
  HSE_VALUE = 25000000; (*!< Value of the External oscillator in Hz  *)
  HSE_STARTUP_TIMEOUT = 5000;  (*!< Time out for HSE start up, in ms  *)

  (**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
   *)
  HSI_VALUE = 16000000;  (*!< Value of the Internal oscillator in Hz *)

  (**
  * @brief Internal Low Speed oscillator (LSI) value.
   *)
  LSI_VALUE = 32000;  (*!< LSI Typical Value in Hz

                                             The real value may vary depending on the variations
                                             in voltage and temperature.  *)
(**
  * @brief External Low Speed oscillator (LSE) value.
   *)
  LSE_VALUE = 32768;  (*!< Value of the External Low Speed oscillator in Hz  *)

  (**
  * @brief External clock source for I2S peripheral
  *        This value is used by the I2S HAL module to compute the I2S clock source
  *        frequency, this source is inserted directly through I2S_CKIN pad.
   *)
  EXTERNAL_CLOCK_VALUE = 12288000;  (*!< Value of the Internal oscillator in Hz *)

  (* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor.  *)

  (* ########################### System Configuration #########################  *)

  (**
  * @brief This is the HAL system configuration section
   *)
  VDD_VALUE = 3300;  (*!< Value of VDD in mv  *)
  TICK_INT_PRIORITY = $0F;  (*!< tick interrupt priority  *)
  USE_RTOS = false;
  ART_ACCLERATOR_ENABLE = true;  (* To enable instruction cache and prefetch  *)
  (* ########################## Assert Selection ##############################  *)

  (**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
   *)
  (* #define USE_FULL_ASSERT    1  *)
  (* ################## Ethernet peripheral configuration #####################  *)

  (* Section 1 : Ethernet peripheral configuration  *)

  (* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5  *)
  MAC_ADDR0 = 2;
  MAC_ADDR1 = 0;
  MAC_ADDR2 = 0;
  MAC_ADDR3 = 0;
  MAC_ADDR4 = 0;
  MAC_ADDR5 = 0;
  (* Definition of the Ethernet driver buffers size and count  *)

  ETH_RX_BUF_SIZE = ETH_MAX_PACKET_SIZE;  (* buffer size for receive                *)
  ETH_TX_BUF_SIZE = ETH_MAX_PACKET_SIZE;  (* buffer size for transmit               *)
  ETH_RXBUFNB = (4);  (* 4 Rx buffers of size ETH_RX_BUF_SIZE   *)
  ETH_TXBUFNB = (4);  (* 4 Tx buffers of size ETH_TX_BUF_SIZE   *)
  (* Section 2: PHY configuration section  *)

  (* DP83848 PHY Address *)

  DP83848_PHY_ADDRESS = $01;
  (* PHY Reset delay these values are based on a 1 ms Systick interrupt *)

  PHY_RESET_DELAY = $000000FF;
  (* PHY Configuration delay  *)

  PHY_CONFIG_DELAY = $00000FFF;
  PHY_READ_TO = $0000FFFF;
  PHY_WRITE_TO = $0000FFFF;
  (* Section 3: Common PHY Registers  *)

  PHY_BCR = ($00);  (*!< Transceiver Basic Control Register    *)
  PHY_BSR = ($01);  (*!< Transceiver Basic Status Register     *)
  PHY_RESET = ($8000);  (*!< PHY Reset  *)
  PHY_LOOPBACK = ($4000);  (*!< Select loop-back mode  *)
  PHY_FULLDUPLEX_100M = ($2100);  (*!< Set the full-duplex mode at 100 Mb/s  *)
  PHY_HALFDUPLEX_100M = ($2000);  (*!< Set the half-duplex mode at 100 Mb/s  *)
  PHY_FULLDUPLEX_10M = ($0100);  (*!< Set the full-duplex mode at 10 Mb/s   *)
  PHY_HALFDUPLEX_10M = ($0000);  (*!< Set the half-duplex mode at 10 Mb/s   *)
  PHY_AUTONEGOTIATION = ($1000);  (*!< Enable auto-negotiation function      *)
  PHY_RESTART_AUTONEGOTIATION = ($0200);
  (*!< Restart auto-negotiation function     *)
  PHY_POWERDOWN = ($0800);  (*!< Select the power down mode            *)
  PHY_ISOLATE = ($0400);  (*!< Isolate PHY from MII                  *)
  PHY_AUTONEGO_COMPLETE = ($0020);  (*!< Auto-Negotiation process completed    *)
  PHY_LINKED_STATUS = ($0004);  (*!< Valid link established                *)
  PHY_JABBER_DETECTION = ($0002);  (*!< Jabber condition detected             *)
  (* Section 4: Extended PHY Registers  *)

  PHY_SR = ($10);  (*!< PHY status register Offset                       *)
  PHY_MICR = ($11);  (*!< MII Interrupt Control Register                   *)
  PHY_MISR = ($12);  (*!< MII Interrupt Status and Misc. Control Register  *)
  PHY_LINK_STATUS = ($0001);  (*!< PHY Link mask                                    *)
  PHY_SPEED_STATUS = ($0002);
  (*!< PHY Speed mask                                   *)
  PHY_DUPLEX_STATUS = ($0004);
  (*!< PHY Duplex mask                                  *)
  PHY_MICR_INT_EN = ($0002);  (*!< PHY Enable interrupts                            *)
  PHY_MICR_INT_OE = ($0001);  (*!< PHY Enable output interrupt events               *)
  PHY_MISR_LINK_INT_EN = ($0020);
  (*!< Enable Interrupt on change of link status        *)
  PHY_LINK_INTERRUPT = ($2000);

implementation

end.
