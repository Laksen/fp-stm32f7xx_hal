(*
  Ported to Pascal by:
    Jeppe Johansen - jeppe@j-software.dk

  *)

(**
  ******************************************************************************
  * @file    stm32f7xx_hal_gpio.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   Header file of GPIO HAL module.
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

unit stm32f7xx_hal_gpio;

interface

uses
  stm32f7xx_hal;

type
  GPIO_InitTypeDef = record
    Pin: longword;  (*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define  *)
    Mode: longword;  (*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define  *)
    Pull: longword;  (*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define  *)
    Speed: longword;  (*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define  *)
    Alternate: longword;  (*!< Peripheral to be connected to the selected pins.
                            This parameter can be a value of @ref GPIO_Alternate_function_selection  *)
  end;

  (**
  * @brief  GPIO Bit SET and Bit RESET enumeration
   *)
type
  GPIO_PinState = (GPIO_PIN_RESET, GPIO_PIN_SET);

const
  GPIO_PIN_0 = ($0001);  (* Pin 0 selected     *)
  GPIO_PIN_1 = ($0002);  (* Pin 1 selected     *)
  GPIO_PIN_2 = ($0004);  (* Pin 2 selected     *)
  GPIO_PIN_3 = ($0008);  (* Pin 3 selected     *)
  GPIO_PIN_4 = ($0010);  (* Pin 4 selected     *)
  GPIO_PIN_5 = ($0020);  (* Pin 5 selected     *)
  GPIO_PIN_6 = ($0040);  (* Pin 6 selected     *)
  GPIO_PIN_7 = ($0080);  (* Pin 7 selected     *)
  GPIO_PIN_8 = ($0100);  (* Pin 8 selected     *)
  GPIO_PIN_9 = ($0200);  (* Pin 9 selected     *)
  GPIO_PIN_10 = ($0400);  (* Pin 10 selected    *)
  GPIO_PIN_11 = ($0800);  (* Pin 11 selected    *)
  GPIO_PIN_12 = ($1000);  (* Pin 12 selected    *)
  GPIO_PIN_13 = ($2000);  (* Pin 13 selected    *)
  GPIO_PIN_14 = ($4000);  (* Pin 14 selected    *)
  GPIO_PIN_15 = ($8000);  (* Pin 15 selected    *)
  GPIO_PIN_All = ($FFFF);  (* All pins selected  *)
  GPIO_PIN_MASK = ($0000FFFF);  (* PIN mask for assert test  *)

(** @defgroup GPIO_mode_define GPIO mode define
  * @brief GPIO Configuration Mode
  *        Elements values convention: $X0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @begin
   *)

const
  GPIO_MODE_INPUT = ($00000000);  (*!< Input Floating Mode                    *)
  GPIO_MODE_OUTPUT_PP = ($00000001);  (*!< Output Push Pull Mode                  *)
  GPIO_MODE_OUTPUT_OD = ($00000011);  (*!< Output Open Drain Mode                 *)
  GPIO_MODE_AF_PP = ($00000002);  (*!< Alternate Function Push Pull Mode      *)
  GPIO_MODE_AF_OD = ($00000012);  (*!< Alternate Function Open Drain Mode     *)
  GPIO_MODE_ANALOG = ($00000003);  (*!< Analog Mode   *)
  GPIO_MODE_IT_RISING = ($10110000);  (*!< External Interrupt Mode with Rising edge trigger detection           *)
  GPIO_MODE_IT_FALLING = ($10210000);  (*!< External Interrupt Mode with Falling edge trigger detection          *)
  GPIO_MODE_IT_RISING_FALLING = ($10310000);  (*!< External Interrupt Mode with Rising/Falling edge trigger detection   *)
  GPIO_MODE_EVT_RISING = ($10120000);  (*!< External Event Mode with Rising edge trigger detection                *)
  GPIO_MODE_EVT_FALLING = ($10220000);  (*!< External Event Mode with Falling edge trigger detection               *)
  GPIO_MODE_EVT_RISING_FALLING = ($10320000);  (*!< External Event Mode with Rising/Falling edge trigger detection        *)
  (**
  * @end;
   *)

  (** @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @begin
   *)

  GPIO_SPEED_LOW = ($00000000);  (*!< Low speed      *)
  GPIO_SPEED_MEDIUM = ($00000001);  (*!< Medium speed   *)
  GPIO_SPEED_FAST = ($00000002);  (*!< Fast speed     *)
  GPIO_SPEED_HIGH = ($00000003);  (*!< High speed     *)
  (**
  * @end;
   *)

  (** @defgroup GPIO_pull_define GPIO pull define
   * @brief GPIO Pull-Up or Pull-Down Activation
   * @begin
    *)

  GPIO_NOPULL = ($00000000);  (*!< No Pull-up or Pull-down activation   *)
  GPIO_PULLUP = ($00000001);  (*!< Pull-up activation                   *)
  GPIO_PULLDOWN = ($00000002);
(*!< Pull-down activation                 *)
  (**
  * @end;
   *)

procedure HAL_GPIO_Init(var GPIOx:  GPIO_TypeDef; const GPIO_Init: GPIO_InitTypeDef);
procedure HAL_GPIO_DeInit(var GPIOx: GPIO_TypeDef; GPIO_Pin: longword);

function HAL_GPIO_ReadPin(var GPIOx: GPIO_TypeDef; GPIO_Pin: word): GPIO_PinState;
procedure HAL_GPIO_WritePin(var GPIOx: GPIO_TypeDef; GPIO_Pin: word; PinState: GPIO_PinState);
procedure HAL_GPIO_TogglePin(var GPIOx: GPIO_TypeDef; GPIO_Pin: word);
function HAL_GPIO_LockPin(var GPIOx: GPIO_TypeDef; GPIO_Pin: word): HAL_StatusTypeDef;

implementation

uses
  stm32f7xx_hal_rcc,
  stm32f7xx_hal_gpio_ex,
  stm32f7xx_defs;

const
  GPIO_NUMBER = 16;

  GPIO_MODE = ($00000003);
  EXTI_MODE = ($10000000);
  GPIO_MODE_IT = ($00010000);
  GPIO_MODE_EVT = ($00020000);
  RISING_EDGE = ($00100000);
  FALLING_EDGE = ($00200000);
  GPIO_OUTPUT_TYPE = ($00000010);

procedure HAL_GPIO_Init(var GPIOx: GPIO_TypeDef; const GPIO_Init: GPIO_InitTypeDef);
var
  position, ioposition, iocurrent, temp: longword;
begin
  position := $00;
  ioposition := $00;
  iocurrent := $00;
  temp := $00;

  (* Configure the port pins *)
  for position := 0 to GPIO_NUMBER - 1 do
  begin
    (* Get the IO position *)
    ioposition := ($01) shl position;
    (* Get the current IO position *)
    iocurrent := (GPIO_Init.Pin) and ioposition;

    if (iocurrent = ioposition) then
    begin
      (*--------------------- GPIO Mode Configuration ------------------------*)
      (* In case of Alternate function mode selection *)
      if ((GPIO_Init.Mode = GPIO_MODE_AF_PP) or (GPIO_Init.Mode = GPIO_MODE_AF_OD)) then
      begin
        (* Configure Alternate function mapped with the current IO *)
        temp := GPIOx.AFR[position shr 3];
        temp := temp and (not ($F shl ((position and $07) * 4)));
        temp := temp or ((GPIO_Init.Alternate) shl ((position and $07) * 4));
        GPIOx.AFR[position shr 3] := temp;
      end;

      (* Configure IO Direction mode (Input, Output, Alternate or Analog) *)
      temp := GPIOx.MODER;
      temp := temp and (not (GPIO_MODER_MODER0 shl (position * 2)));
      temp := temp or ((GPIO_Init.Mode and GPIO_MODE) shl (position * 2));
      GPIOx.MODER := temp;

      (* In case of Output or Alternate function mode selection *)
      if ((GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP) or (GPIO_Init.Mode = GPIO_MODE_AF_PP) or (GPIO_Init.Mode = GPIO_MODE_OUTPUT_OD) or (GPIO_Init.Mode = GPIO_MODE_AF_OD)) then
      begin
        (* Configure the IO Speed *)
        temp := GPIOx.OSPEEDR;
        temp := temp and (not (GPIO_OSPEEDER_OSPEEDR0 shl (position * 2)));
        temp := temp or (GPIO_Init.Speed shl (position * 2));
        GPIOx.OSPEEDR := temp;

        (* Configure the IO Output Type *)
        temp := GPIOx.OTYPER;
        temp := temp and (not (GPIO_OTYPER_OT_0 shl position));
        temp := temp or (((GPIO_Init.Mode and GPIO_OUTPUT_TYPE) shr 4) shl position);
        GPIOx.OTYPER := temp;
      end;

      (* Activate the Pull-up or Pull down resistor for the current IO *)
      temp := GPIOx.PUPDR;
      temp := temp and (not (GPIO_PUPDR_PUPDR0 shl (position * 2)));
      temp := temp or ((GPIO_Init.Pull) shl (position * 2));
      GPIOx.PUPDR := temp;

      (*--------------------- EXTI Mode Configuration ------------------------*)
      (* Configure the External Interrupt or event for the current IO *)
      if ((GPIO_Init.Mode and EXTI_MODE) = EXTI_MODE) then
      begin
        (* Enable SYSCFG Clock *)
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        temp := SYSCFG.EXTICR[position shr 2];
        temp := temp and (not (($0F) shl (4 * (position and $03))));
        temp := temp or ((GPIO_GET_INDEX(GPIOx)) shl (4 * (position and $03)));
        SYSCFG.EXTICR[position shr 2] := temp;

        (* Clear EXTI line configuration *)
        temp := EXTI.IMR;
        temp := temp and (not (iocurrent));
        if ((GPIO_Init.Mode and GPIO_MODE_IT) = GPIO_MODE_IT) then
        begin
          temp := temp or iocurrent;
        end;
        EXTI.IMR := temp;

        temp := EXTI.EMR;
        temp := temp and (not (iocurrent));
        if ((GPIO_Init.Mode and GPIO_MODE_EVT) = GPIO_MODE_EVT) then
        begin
          temp := temp or iocurrent;
        end;
        EXTI.EMR := temp;

        (* Clear Rising Falling edge configuration *)
        temp := EXTI.RTSR;
        temp := temp and (not (iocurrent));
        if ((GPIO_Init.Mode and RISING_EDGE) = RISING_EDGE) then
        begin
          temp := temp or iocurrent;
        end;
        EXTI.RTSR := temp;

        temp := EXTI.FTSR;
        temp := temp and (not (iocurrent));
        if ((GPIO_Init.Mode and FALLING_EDGE) = FALLING_EDGE) then
        begin
          temp := temp or iocurrent;
        end;
        EXTI.FTSR := temp;
      end;
    end;
  end;
end;

procedure HAL_GPIO_DeInit(var GPIOx: GPIO_TypeDef; GPIO_Pin: longword);
var
  position, ioposition, iocurrent, tmp: longword;
begin
  ioposition := $00;
  iocurrent := $00;
  tmp := $00;

  (* Configure the port pins *)
  for position := 0 to GPIO_NUMBER - 1 do
  begin
    (* Get the IO position *)
    ioposition := ($01) shl position;
    (* Get the current IO position *)
    iocurrent := (GPIO_Pin) and ioposition;

    if (iocurrent = ioposition) then
    begin
      (*------------------------- GPIO Mode Configuration --------------------*)
      (* Configure IO Direction in Input Floating Mode *)
      GPIOx.MODER := GPIOx.MODER and not (GPIO_MODER_MODER0 shl (position * 2));

      (* Configure the default Alternate Function in current IO *)
      GPIOx.AFR[position shr 3] := GPIOx.AFR[position shr 3] and not ($F shl ((position and $07) * 4));

      (* Configure the default value for IO Speed *)
      GPIOx.OSPEEDR := GPIOx.OSPEEDR and not (GPIO_OSPEEDER_OSPEEDR0 shl (position * 2));

      (* Configure the default value IO Output Type *)
      GPIOx.OTYPER := GPIOx.OTYPER and not (GPIO_OTYPER_OT_0 shl position);

      (* Deactivate the Pull-up and Pull-down resistor for the current IO *)
      GPIOx.PUPDR := GPIOx.PUPDR and not (GPIO_PUPDR_PUPDR0 shl (position * 2));

      (*------------------------- EXTI Mode Configuration --------------------*)
      tmp := SYSCFG.EXTICR[position shr 2];
      tmp := tmp and (($0F) shl (4 * (position and $03)));
      if (tmp = ((GPIO_GET_INDEX(GPIOx)) shl (4 * (position and $03)))) then
      begin
        (* Configure the External Interrupt or event for the current IO *)
        tmp := ($0F) shl (4 * (position and $03));
        SYSCFG.EXTICR[position shr 2] := SYSCFG.EXTICR[position shr 2] and not tmp;

        (* Clear EXTI line configuration *)
        EXTI.IMR := EXTI.IMR and not (iocurrent);
        EXTI.EMR := EXTI.EMR and not (iocurrent);

        (* Clear Rising Falling edge configuration *)
        EXTI.RTSR := EXTI.RTSR and not (iocurrent);
        EXTI.FTSR := EXTI.FTSR and not (iocurrent);
      end;
    end;
  end;
end;

function HAL_GPIO_ReadPin(var GPIOx: GPIO_TypeDef; GPIO_Pin: word): GPIO_PinState;
var
  bitstatus: GPIO_PinState;
begin
  if ((GPIOx.IDR and GPIO_Pin) <> 0) then
    bitstatus := GPIO_PIN_SET
  else
    bitstatus := GPIO_PIN_RESET;

  exit(bitstatus);
end;

procedure HAL_GPIO_WritePin(var GPIOx: GPIO_TypeDef; GPIO_Pin: word; PinState: GPIO_PinState);
begin
  if (PinState <> GPIO_PIN_RESET) then
    GPIOx.BSRR := GPIO_Pin
  else
    GPIOx.BSRR := longword(GPIO_Pin) shl 16;
end;

procedure HAL_GPIO_TogglePin(var GPIOx: GPIO_TypeDef; GPIO_Pin: word);
begin
  GPIOx.ODR := GPIOx.ODR xor GPIO_Pin;
end;

function HAL_GPIO_LockPin(var GPIOx: GPIO_TypeDef; GPIO_Pin: word): HAL_StatusTypeDef;
var
  tmp: word;
begin
  (* Apply lock key write sequence *)
  tmp := GPIO_LCKR_LCKK or GPIO_Pin;
  (* Set LCKx bit(s): LCKK:='1' + LCK[15-0] *)
  GPIOx.LCKR := tmp;
  (* Reset LCKx bit(s): LCKK:='0' + LCK[15-0] *)
  GPIOx.LCKR := GPIO_Pin;
  (* Set LCKx bit(s): LCKK:='1' + LCK[15-0] *)
  GPIOx.LCKR := tmp;
  (* Read LCKK bit*)
  tmp := GPIOx.LCKR;

  if ((GPIOx.LCKR and GPIO_LCKR_LCKK) <> 0) then
    exit(HAL_OK)
  else
    exit(HAL_ERROR);
end;

end.
