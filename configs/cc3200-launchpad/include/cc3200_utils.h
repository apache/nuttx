/************************************************************************************
 * configs/cc3200/include/cc3200_util.h
 *
 *   Copyright (C) 2014 Droidifi LLC. All rights reserved.
 *   Author: Jim Ewing <jim@droidifi.com>
 * 
 *   Adapted from code Copyright (C) 2014 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIGS_CC3200_INCLUDE_UTILS_H
#define __CONFIGS_CC3200_INCLUDE_UTILS_H 1

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define CONSOLE_BAUD_RATE   115200

#define PAD_CONFIG_BASE     0x4402E0A0

#define PIN_TYPE_STD        0x00000000
#define PIN_STRENGTH_2MA    0x00000020
#define PAD_MODE_MASK       0x0000000F
#define PAD_STRENGTH_MASK   0x000000E0
#define PAD_TYPE_MASK       0x00000310

#define PIN_MODE_0          0x00000000
#define PIN_MODE_1          0x00000001
#define PIN_MODE_2          0x00000002
#define PIN_MODE_3          0x00000003
#define PIN_MODE_4          0x00000004
#define PIN_MODE_5          0x00000005
#define PIN_MODE_6          0x00000006
#define PIN_MODE_7          0x00000007
#define PIN_MODE_8          0x00000008
#define PIN_MODE_9          0x00000009
#define PIN_MODE_10         0x0000000A
#define PIN_MODE_11         0x0000000B
#define PIN_MODE_12         0x0000000C
#define PIN_MODE_13         0x0000000D
#define PIN_MODE_14         0x0000000E
#define PIN_MODE_15         0x0000000F

#define PIN_STRENGTH_2MA    0x00000020
#define PIN_STRENGTH_4MA    0x00000040
#define PIN_STRENGTH_6MA    0x00000060

#define PIN_TYPE_STD        0x00000000
#define PIN_TYPE_STD_PU     0x00000100
#define PIN_TYPE_STD_PD     0x00000200

#define PIN_TYPE_OD         0x00000010
#define PIN_TYPE_OD_PU      0x00000110
#define PIN_TYPE_OD_PD      0x00000210
#define PIN_TYPE_ANALOG     0x10000000

#define PIN_01              0x00000000
#define PIN_02              0x00000001
#define PIN_03              0x00000002
#define PIN_04              0x00000003
#define PIN_05              0x00000004
#define PIN_06              0x00000005
#define PIN_07              0x00000006
#define PIN_08              0x00000007
#define PIN_11              0x0000000A
#define PIN_12              0x0000000B
#define PIN_13              0x0000000C
#define PIN_14              0x0000000D
#define PIN_15              0x0000000E
#define PIN_16              0x0000000F
#define PIN_17              0x00000010
#define PIN_18              0x00000011
#define PIN_19              0x00000012
#define PIN_20              0x00000013
#define PIN_21              0x00000014
#define PIN_45              0x0000002C
#define PIN_46              0x0000002D
#define PIN_47              0x0000002E
#define PIN_48              0x0000002F
#define PIN_49              0x00000030
#define PIN_50              0x00000031
#define PIN_52              0x00000033
#define PIN_53              0x00000034
#define PIN_55              0x00000036
#define PIN_56              0x00000037
#define PIN_57              0x00000038
#define PIN_58              0x00000039
#define PIN_59              0x0000003A
#define PIN_60              0x0000003B
#define PIN_61              0x0000003C
#define PIN_62              0x0000003D
#define PIN_63              0x0000003E
#define PIN_64              0x0000003F

#define GPIO_O_GPIO_DATA    0x00000000
#define GPIO_O_GPIO_DIR     0x00000400

#define GPIO_DIR_MODE_OUT   0x00000001
#define GPIO_DIR_MODE_IN    0x00000000

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void cc3200_print(char* str);
void cc3200_pin_config_set(uint32_t pin, uint32_t pin_strength, uint32_t pin_type);
void cc3200_pin_mode_set(uint32_t pin, uint32_t pin_mode);
void cc3200_pin_type_uart(uint32_t pin, uint32_t pin_mode);
void cc3200_get_gpio_port_pin(uint8_t pin, uint32_t *gpio_port, uint8_t *gpio_pin);
void cc3200_set_gpio(uint8_t pin, uint32_t gpio_port, uint8_t gpio_pin, uint8_t gpio_val);
void cc3200_set_gpio_dir(uint32_t port, uint8_t pins, uint32_t pin_io);
void cc3200_pin_type_gpio(uint32_t pin, uint32_t pin_mode, uint32_t open_drain);

#endif /* __CONFIGS_CC3200_INCLUDE_UTILS_H */
