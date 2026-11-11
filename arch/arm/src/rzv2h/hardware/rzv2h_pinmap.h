/****************************************************************************
 * arch/arm/src/rzv2h/hardware/rzv2h_pinmap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RZV2H_HARDWARE_RZV2H_PINMAP_H
#define __ARCH_ARM_SRC_RZV2H_HARDWARE_RZV2H_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "bsp_api.h"
#include "chip.h"
#include "r_ioport.h"

/****************************************************************************
 * Porting implementation
 ****************************************************************************/

/* Pin configuration encoding with support for input/output. */

#define RZV2H_PINMUX_CFG(port, pin, mode, cfg) \
  ((gpio_pinset_t) \
    { \
      (uint32_t)(((port) << 8) | (pin)), \
      IOPORT_CFG_PERIPHERAL_PIN | ((uint32_t)(mode) << 24) | \
      (cfg) \
    })

/* Pin configuration encoding. */

#define RZV2H_PINMUX(port, pin, mode) \
  RZV2H_PINMUX_CFG(port, pin, mode, \
                   IOPORT_CFG_DRIVE_B01 | IOPORT_CFG_SLEW_RATE_FAST)

#define RZV2H_PINMUX_INPUT(port, pin, mode) \
  ((gpio_pinset_t) \
    { \
      (uint32_t)(((port) << 8) | (pin)), \
      IOPORT_CFG_DRIVE_B01 | IOPORT_CFG_PERIPHERAL_PIN | \
      IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_ENABLE | \
      IOPORT_CFG_SLEW_RATE_FAST | ((uint32_t)(mode) << 24) \
    })

#define RZV2H_PINMUX_OUTPUT(port, pin, mode)  RZV2H_PINMUX(port, pin, mode)

/* SCI-B UART pin configurations. */

/* SCI-B channel 0. */

#define GPIO_TXD0_MOSI0_DA0_P5_0_M1   RZV2H_PINMUX(5, 0, 1)
#define GPIO_RXD0_MISO0_SCL0_P5_1_M1  RZV2H_PINMUX(5, 1, 1)

/* SCI-B channel 1. */

#define GPIO_TXD1_MOSI1_SDA1_P5_2_M1  RZV2H_PINMUX(5, 2, 1)
#define GPIO_RXD1_MISO1_SCL1_P5_3_M1  RZV2H_PINMUX(5, 3, 1)

/* SCI-B channel 2. */

#define GPIO_TXD2_MOSI2_SDA2_P5_4_M1  RZV2H_PINMUX(5, 4, 1)
#define GPIO_TXD2_MOSI2_SDA2_P6_0_M6  RZV2H_PINMUX(6, 0, 6)
#define GPIO_RXD2_MISO2_SCL2_P5_5_M1  RZV2H_PINMUX(5, 5, 1)
#define GPIO_RXD2_MISO2_SCL2_P6_1_M6  RZV2H_PINMUX(6, 1, 6)

/* SCI-B channel 3. */

#define GPIO_TXD3_MOSI3_SDA3_P5_6_M1  RZV2H_PINMUX(5, 6, 1)
#define GPIO_TXD3_MOSI3_SDA3_P6_2_M6  RZV2H_PINMUX(6, 2, 6)
#define GPIO_RXD3_MISO3_SCL3_P5_7_M1  RZV2H_PINMUX(5, 7, 1)
#define GPIO_RXD3_MISO3_SCL3_P6_3_M6  RZV2H_PINMUX(6, 3, 6)

/* SCI-B channel 4. */

#define GPIO_TXD4_MOSI1_SDA4_P4_0_M2  RZV2H_PINMUX(4, 0, 2)
#define GPIO_TXD4_MOSI4_SDA4_P6_4_M6  RZV2H_PINMUX(6, 4, 6)
#define GPIO_TXD4_MOSI4_SDA4_P7_0_M1  RZV2H_PINMUX(7, 0, 1)
#define GPIO_TXD4_MOSI4_SDA4_P8_4_M6  RZV2H_PINMUX(8, 4, 6)
#define GPIO_TXD4_MOSI8_SDA8_P8_0_M6  RZV2H_PINMUX(8, 0, 6)
#define GPIO_TXD4_MOSI9_SDA0_P9_2_M6  RZV2H_PINMUX(9, 2, 6)
#define GPIO_RXD4_MISO4_SCL4_P4_1_M2  RZV2H_PINMUX(4, 1, 2)
#define GPIO_RXD4_MISO4_SCL4_P7_1_M1  RZV2H_PINMUX(7, 1, 1)
#define GPIO_RXD4_MISO4_SCL4_P8_5_M6  RZV2H_PINMUX(8, 5, 6)
#define GPIO_RXD4_MISO6_SCL6_P6_5_M6  RZV2H_PINMUX(6, 5, 6)

/* SCI-B channel 5. */

#define GPIO_TXD5_MOSI5_SDA5_P4_4_M2  RZV2H_PINMUX(4, 4, 2)
#define GPIO_TXD5_MOSI5_SDA5_P7_2_M1  RZV2H_PINMUX(7, 2, 1)
#define GPIO_TXD5_MOSI5_SDA5_P8_6_M6  RZV2H_PINMUX(8, 6, 6)
#define GPIO_RXD5_MISO5_SCL5_P4_5_M2  RZV2H_PINMUX(4, 5, 2)
#define GPIO_RXD5_MISO5_SCL5_P7_3_M1  RZV2H_PINMUX(7, 3, 1)
#define GPIO_RXD5_MISO5_SCL5_P8_7_M6  RZV2H_PINMUX(8, 7, 6)

/* SCI-B channel 6. */

#define GPIO_TXD6_MOSI6_SDA6_P7_4_M1  RZV2H_PINMUX(7, 4, 1)
#define GPIO_RXD6_MISO6_SCL6_P7_5_M1  RZV2H_PINMUX(7, 5, 1)

/* SCI-B channel 7. */

#define GPIO_TXD7_MOSI7_SDA7_P6_6_M6  RZV2H_PINMUX(6, 6, 6)
#define GPIO_TXD7_MOSI7_SDA7_P7_6_M1  RZV2H_PINMUX(7, 6, 1)
#define GPIO_TXD7_MOSI7_SDA7_P9_4_M2  RZV2H_PINMUX(9, 4, 2)
#define GPIO_RXD7_MISO7_SCL7_P6_7_M6  RZV2H_PINMUX(6, 7, 6)
#define GPIO_RXD7_MISO7_SCL7_P7_7_M1  RZV2H_PINMUX(7, 7, 1)
#define GPIO_RXD7_MISO7_SCL7_P9_5_M2  RZV2H_PINMUX(9, 5, 2)

/* SCI-B channel 8. */

#define GPIO_TXD8_MOSI8_SDA6_P9_0_M2  RZV2H_PINMUX(9, 0, 2)
#define GPIO_TXD8_MOSI8_SDA8_PB_1_M2  RZV2H_PINMUX(11, 1, 2)
#define GPIO_RXD8_MISO8_SCL6_P9_1_M2  RZV2H_PINMUX(9, 1, 2)
#define GPIO_RXD8_MISO8_SCL8_P8_1_M6  RZV2H_PINMUX(8, 1, 6)
#define GPIO_RXD8_MISO8_SCL8_PB_2_M2  RZV2H_PINMUX(11, 2, 2)

/* SCI-B channel 9. */

#define GPIO_TXD9_MOSI9_SDA9_P8_2_M6  RZV2H_PINMUX(8, 2, 6)
#define GPIO_TXD9_MOSI9_SDA9_PB_4_M2  RZV2H_PINMUX(11, 4, 2)
#define GPIO_RXD9_MISO0_SCL0_P9_3_M6  RZV2H_PINMUX(9, 3, 6)
#define GPIO_RXD9_MISO9_SCL9_P8_3_M6  RZV2H_PINMUX(8, 3, 6)
#define GPIO_RXD9_MISO9_SCL9_PB_3_M2  RZV2H_PINMUX(11, 3, 2)

/* RIIC I2C pin configurations. */

/* RIIC channel 0: P31/SCL, P30/SDA, Mode 1 */

#define GPIO_SCL0_P3_1_M1   RZV2H_PINMUX(3, 1, 1)
#define GPIO_SDA0_P3_0_M1   RZV2H_PINMUX(3, 0, 1)

/* RIIC channel 1: P33/SCL, P32/SDA, Mode 1 */

#define GPIO_SCL1_P3_3_M1   RZV2H_PINMUX(3, 3, 1)
#define GPIO_SDA1_P3_2_M1   RZV2H_PINMUX(3, 2, 1)

/* RIIC channel 2: P21/SCL, P20/SDA, Mode 4 */

#define GPIO_SCL2_P2_1_M4   RZV2H_PINMUX(2, 1, 4)
#define GPIO_SDA2_P2_0_M4   RZV2H_PINMUX(2, 0, 4)

/* RIIC channel 3: P37/SCL, P36/SDA, Mode 1 */

#define GPIO_SCL3_P3_7_M1   RZV2H_PINMUX(3, 7, 1)
#define GPIO_SDA3_P3_6_M1   RZV2H_PINMUX(3, 6, 1)

/* RIIC channel 4: P41/SCL, P40/SDA, Mode 1 */

#define GPIO_SCL4_P4_1_M1   RZV2H_PINMUX(4, 1, 1)
#define GPIO_SDA4_P4_0_M1   RZV2H_PINMUX(4, 0, 1)

/* RIIC channel 5: P43/SCL, P42/SDA, Mode 1 */

#define GPIO_SCL5_P4_3_M1   RZV2H_PINMUX(4, 3, 1)
#define GPIO_SDA5_P4_2_M1   RZV2H_PINMUX(4, 2, 1)

/* RIIC channel 6: P45/SCL, P44/SDA, Mode 1 */

#define GPIO_SCL6_P4_5_M1   RZV2H_PINMUX(4, 5, 1)
#define GPIO_SDA6_P4_4_M1   RZV2H_PINMUX(4, 4, 1)

/* RIIC channel 7: P47/SCL, P46/SDA, Mode 1 */

#define GPIO_SCL7_P4_7_M1   RZV2H_PINMUX(4, 7, 1)
#define GPIO_SDA7_P4_6_M1   RZV2H_PINMUX(4, 6, 1)

/* RIIC channel 8: P07/SCL, P06/SDA, Mode 1 */

#define GPIO_SCL8_P0_7_M1   RZV2H_PINMUX(0, 7, 1)
#define GPIO_SDA8_P0_6_M1   RZV2H_PINMUX(0, 6, 1)

/* ADC_E external trigger pin configuration. */

#define GPIO_ADTRG_P0_4_M4 \
  RZV2H_PINMUX_CFG(0, 4, 4, IOPORT_CFG_PULLUP_ENABLE | \
                            IOPORT_CFG_SCHMITT_ENABLE)
#define GPIO_ADTRG_P0_5_M4 \
  RZV2H_PINMUX_CFG(0, 5, 4, IOPORT_CFG_PULLUP_ENABLE | \
                            IOPORT_CFG_SCHMITT_ENABLE)
#define GPIO_ADTRG_P1_4_M4 \
  RZV2H_PINMUX_CFG(1, 4, 4, IOPORT_CFG_PULLUP_ENABLE | \
                            IOPORT_CFG_SCHMITT_ENABLE)
#define GPIO_ADTRG_P1_5_M4 \
  RZV2H_PINMUX_CFG(1, 5, 4, IOPORT_CFG_PULLUP_ENABLE | \
                            IOPORT_CFG_SCHMITT_ENABLE)
#define GPIO_ADTRG_P9_7_M4 \
  RZV2H_PINMUX_CFG(9, 7, 4, IOPORT_CFG_PULLUP_ENABLE | \
                            IOPORT_CFG_SCHMITT_ENABLE)

/* SPI-B channel 0. */

#define GPIO_MISO0_P9_1_M1             RZV2H_PINMUX(9, 1, 1)
#define GPIO_MOSI0_P9_0_M1             RZV2H_PINMUX(9, 0, 1)
#define GPIO_RSPCK0_P9_2_M1            RZV2H_PINMUX(9, 2, 1)
#define GPIO_SSLA0_P0_4_M2_INPUT       RZV2H_PINMUX_INPUT(0, 4, 2)
#define GPIO_SSLA0_P0_4_M2_OUTPUT      RZV2H_PINMUX_OUTPUT(0, 4, 2)
#define GPIO_SSLA0_P3_4_M5_INPUT       RZV2H_PINMUX_INPUT(3, 4, 5)
#define GPIO_SSLA0_P3_4_M5_OUTPUT      RZV2H_PINMUX_OUTPUT(3, 4, 5)
#define GPIO_SSLA0_P9_3_M1_INPUT       RZV2H_PINMUX_INPUT(9, 3, 1)
#define GPIO_SSLA0_P9_3_M1_OUTPUT      RZV2H_PINMUX_OUTPUT(9, 3, 1)
#define GPIO_SSLA1_P0_5_M2_INPUT       RZV2H_PINMUX_INPUT(0, 5, 2)
#define GPIO_SSLA1_P0_5_M2_OUTPUT      RZV2H_PINMUX_OUTPUT(0, 5, 2)
#define GPIO_SSLA1_P3_5_M5_INPUT       RZV2H_PINMUX_INPUT(3, 5, 5)
#define GPIO_SSLA1_P3_5_M5_OUTPUT      RZV2H_PINMUX_OUTPUT(3, 5, 5)
#define GPIO_SSLA1_P9_4_M1_INPUT       RZV2H_PINMUX_INPUT(9, 4, 1)
#define GPIO_SSLA1_P9_4_M1_OUTPUT      RZV2H_PINMUX_OUTPUT(9, 4, 1)
#define GPIO_SSLA2_P1_4_M2_INPUT       RZV2H_PINMUX_INPUT(1, 4, 2)
#define GPIO_SSLA2_P1_4_M2_OUTPUT      RZV2H_PINMUX_OUTPUT(1, 4, 2)
#define GPIO_SSLA2_P3_6_M5_INPUT       RZV2H_PINMUX_INPUT(3, 6, 5)
#define GPIO_SSLA2_P3_6_M5_OUTPUT      RZV2H_PINMUX_OUTPUT(3, 6, 5)
#define GPIO_SSLA2_P9_5_M1_INPUT       RZV2H_PINMUX_INPUT(9, 5, 1)
#define GPIO_SSLA2_P9_5_M1_OUTPUT      RZV2H_PINMUX_OUTPUT(9, 5, 1)
#define GPIO_SSLA3_P1_5_M2_INPUT       RZV2H_PINMUX_INPUT(1, 5, 2)
#define GPIO_SSLA3_P1_5_M2_OUTPUT      RZV2H_PINMUX_OUTPUT(1, 5, 2)
#define GPIO_SSLA3_P3_7_M5_INPUT       RZV2H_PINMUX_INPUT(3, 7, 5)
#define GPIO_SSLA3_P3_7_M5_OUTPUT      RZV2H_PINMUX_OUTPUT(3, 7, 5)
#define GPIO_SSLA3_P9_6_M1_INPUT       RZV2H_PINMUX_INPUT(9, 6, 1)
#define GPIO_SSLA3_P9_6_M1_OUTPUT      RZV2H_PINMUX_OUTPUT(9, 6, 1)

/* SPI-B channel 1. */

#define GPIO_MISO0_PB_2_M4             RZV2H_PINMUX(11, 2, 4)
#define GPIO_MOSI0_PB_1_M4             RZV2H_PINMUX(11, 1, 4)
#define GPIO_RSPCK0_PB_0_M4            RZV2H_PINMUX(11, 0, 4)
#define GPIO_SSLB0_P3_4_M6_INPUT       RZV2H_PINMUX_INPUT(3, 4, 6)
#define GPIO_SSLB0_P3_4_M6_OUTPUT      RZV2H_PINMUX_OUTPUT(3, 4, 6)
#define GPIO_SSLB0_PA_4_M4_INPUT       RZV2H_PINMUX_INPUT(10, 4, 4)
#define GPIO_SSLB0_PA_4_M4_OUTPUT      RZV2H_PINMUX_OUTPUT(10, 4, 4)
#define GPIO_SSLB1_P3_6_M6_INPUT       RZV2H_PINMUX_INPUT(3, 6, 6)
#define GPIO_SSLB1_P3_6_M6_OUTPUT      RZV2H_PINMUX_OUTPUT(3, 6, 6)
#define GPIO_SSLB1_PA_5_M4_INPUT       RZV2H_PINMUX_INPUT(10, 5, 4)
#define GPIO_SSLB1_PA_5_M4_OUTPUT      RZV2H_PINMUX_OUTPUT(10, 5, 4)
#define GPIO_SSLB2_P0_4_M3_INPUT       RZV2H_PINMUX_INPUT(0, 4, 3)
#define GPIO_SSLB2_P0_4_M3_OUTPUT      RZV2H_PINMUX_OUTPUT(0, 4, 3)
#define GPIO_SSLB2_PA_6_M4_INPUT       RZV2H_PINMUX_INPUT(10, 6, 4)
#define GPIO_SSLB2_PA_6_M4_OUTPUT      RZV2H_PINMUX_OUTPUT(10, 6, 4)
#define GPIO_SSLB3_P1_4_M3_INPUT       RZV2H_PINMUX_INPUT(1, 4, 3)
#define GPIO_SSLB3_P1_4_M3_OUTPUT      RZV2H_PINMUX_OUTPUT(1, 4, 3)
#define GPIO_SSLB3_PA_7_M4_INPUT       RZV2H_PINMUX_INPUT(10, 7, 4)
#define GPIO_SSLB3_PA_7_M4_OUTPUT      RZV2H_PINMUX_OUTPUT(10, 7, 4)

/* SPI-B channel 2. */

#define GPIO_MISO0_PB_3_M5             RZV2H_PINMUX(11, 3, 5)
#define GPIO_MOSI0_PB_4_M5             RZV2H_PINMUX(11, 4, 5)
#define GPIO_RSPCK0_PB_5_M5            RZV2H_PINMUX(11, 5, 5)
#define GPIO_SSLC0_P3_5_M6_INPUT       RZV2H_PINMUX_INPUT(3, 5, 6)
#define GPIO_SSLC0_P3_5_M6_OUTPUT      RZV2H_PINMUX_OUTPUT(3, 5, 6)
#define GPIO_SSLC0_PA_7_M5_INPUT       RZV2H_PINMUX_INPUT(10, 7, 5)
#define GPIO_SSLC0_PA_7_M5_OUTPUT      RZV2H_PINMUX_OUTPUT(10, 7, 5)
#define GPIO_SSLC1_P3_7_M6_INPUT       RZV2H_PINMUX_INPUT(3, 7, 6)
#define GPIO_SSLC1_P3_7_M6_OUTPUT      RZV2H_PINMUX_OUTPUT(3, 7, 6)
#define GPIO_SSLC1_PA_6_M5_INPUT       RZV2H_PINMUX_INPUT(10, 6, 5)
#define GPIO_SSLC1_PA_6_M5_OUTPUT      RZV2H_PINMUX_OUTPUT(10, 6, 5)
#define GPIO_SSLC2_P0_5_M3_INPUT       RZV2H_PINMUX_INPUT(0, 5, 3)
#define GPIO_SSLC2_P0_5_M3_OUTPUT      RZV2H_PINMUX_OUTPUT(0, 5, 3)
#define GPIO_SSLC2_PA_5_M5_INPUT       RZV2H_PINMUX_INPUT(10, 5, 5)
#define GPIO_SSLC2_PA_5_M5_OUTPUT      RZV2H_PINMUX_OUTPUT(10, 5, 5)
#define GPIO_SSLC3_P1_5_M3_INPUT       RZV2H_PINMUX_INPUT(1, 5, 3)
#define GPIO_SSLC3_P1_5_M3_OUTPUT      RZV2H_PINMUX_OUTPUT(1, 5, 3)
#define GPIO_SSLC3_PA_4_M5_INPUT       RZV2H_PINMUX_INPUT(10, 4, 5)
#define GPIO_SSLC3_PA_4_M5_OUTPUT      RZV2H_PINMUX_OUTPUT(10, 4, 5)

/* GPT PWM — GTIOC pin configurations. */

/* GPT channel 0. */

#define GPIO_GTIOC0A_P4_0_M9    RZV2H_PINMUX(4, 0, 9)
#define GPIO_GTIOC0A_P7_0_M9    RZV2H_PINMUX(7, 0, 9)
#define GPIO_GTIOC0B_P4_1_M9    RZV2H_PINMUX(4, 1, 9)
#define GPIO_GTIOC0B_P7_1_M9    RZV2H_PINMUX(7, 1, 9)

/* GPT channel 1. */

#define GPIO_GTIOC1A_P4_2_M9    RZV2H_PINMUX(4, 2, 9)
#define GPIO_GTIOC1A_P8_0_M9    RZV2H_PINMUX(8, 0, 9)
#define GPIO_GTIOC1B_P4_3_M9    RZV2H_PINMUX(4, 3, 9)
#define GPIO_GTIOC1B_P8_1_M9    RZV2H_PINMUX(8, 1, 9)

/* GPT channel 2. */

#define GPIO_GTIOC2A_P4_4_M9    RZV2H_PINMUX(4, 4, 9)
#define GPIO_GTIOC2A_P7_2_M9    RZV2H_PINMUX(7, 2, 9)
#define GPIO_GTIOC2B_P4_5_M9    RZV2H_PINMUX(4, 5, 9)
#define GPIO_GTIOC2B_P7_3_M9    RZV2H_PINMUX(7, 3, 9)

/* GPT channel 3. */

#define GPIO_GTIOC3A_P4_6_M9    RZV2H_PINMUX(4, 6, 9)
#define GPIO_GTIOC3A_P8_2_M9    RZV2H_PINMUX(8, 2, 9)
#define GPIO_GTIOC3B_P4_7_M9    RZV2H_PINMUX(4, 7, 9)
#define GPIO_GTIOC3B_P8_3_M9    RZV2H_PINMUX(8, 3, 9)

/* GPT channel 4. */

#define GPIO_GTIOC4A_P3_0_M9    RZV2H_PINMUX(3, 0, 9)
#define GPIO_GTIOC4A_P7_4_M9    RZV2H_PINMUX(7, 4, 9)
#define GPIO_GTIOC4A_P9_4_M11   RZV2H_PINMUX(9, 4, 11)
#define GPIO_GTIOC4B_P3_1_M9    RZV2H_PINMUX(3, 1, 9)
#define GPIO_GTIOC4B_P7_5_M9    RZV2H_PINMUX(7, 5, 9)
#define GPIO_GTIOC4B_P9_5_M11   RZV2H_PINMUX(9, 5, 11)

/* GPT channel 5. */

#define GPIO_GTIOC5A_P3_2_M9    RZV2H_PINMUX(3, 2, 9)
#define GPIO_GTIOC5A_P8_4_M9    RZV2H_PINMUX(8, 4, 9)
#define GPIO_GTIOC5A_P9_6_M11   RZV2H_PINMUX(9, 6, 11)
#define GPIO_GTIOC5B_P3_3_M9    RZV2H_PINMUX(3, 3, 9)
#define GPIO_GTIOC5B_P8_5_M9    RZV2H_PINMUX(8, 5, 9)
#define GPIO_GTIOC5B_P9_7_M11   RZV2H_PINMUX(9, 7, 11)

/* GPT channel 6. */

#define GPIO_GTIOC6A_P3_4_M9    RZV2H_PINMUX(3, 4, 9)
#define GPIO_GTIOC6A_P7_6_M9    RZV2H_PINMUX(7, 6, 9)
#define GPIO_GTIOC6A_PA_4_M11   RZV2H_PINMUX(10, 4, 11)
#define GPIO_GTIOC6B_P3_5_M9    RZV2H_PINMUX(3, 5, 9)
#define GPIO_GTIOC6B_P7_7_M9    RZV2H_PINMUX(7, 7, 9)
#define GPIO_GTIOC6B_PA_5_M11   RZV2H_PINMUX(10, 5, 11)

/* GPT channel 7. */

#define GPIO_GTIOC7A_P3_6_M9    RZV2H_PINMUX(3, 6, 9)
#define GPIO_GTIOC7A_P8_6_M9    RZV2H_PINMUX(8, 6, 9)
#define GPIO_GTIOC7A_PA_6_M11   RZV2H_PINMUX(10, 6, 11)
#define GPIO_GTIOC7B_P3_7_M9    RZV2H_PINMUX(3, 7, 9)
#define GPIO_GTIOC7B_P8_7_M9    RZV2H_PINMUX(8, 7, 9)
#define GPIO_GTIOC7B_PA_7_M11   RZV2H_PINMUX(10, 7, 11)

/* GPT channel 8. */

#define GPIO_GTIOC8A_P5_0_M11   RZV2H_PINMUX(5, 0, 11)
#define GPIO_GTIOC8A_P9_4_M9    RZV2H_PINMUX(9, 4, 9)
#define GPIO_GTIOC8B_P5_1_M11   RZV2H_PINMUX(5, 1, 11)
#define GPIO_GTIOC8B_P9_5_M9    RZV2H_PINMUX(9, 5, 9)

/* GPT channel 9. */

#define GPIO_GTIOC9A_P6_0_M11   RZV2H_PINMUX(6, 0, 11)
#define GPIO_GTIOC9A_P9_6_M9    RZV2H_PINMUX(9, 6, 9)
#define GPIO_GTIOC9B_P6_1_M11   RZV2H_PINMUX(6, 1, 11)
#define GPIO_GTIOC9B_P9_7_M9    RZV2H_PINMUX(9, 7, 9)

/* GPT channel 10. */

#define GPIO_GTIOC10A_P5_2_M11   RZV2H_PINMUX(5, 2, 11)
#define GPIO_GTIOC10A_PA_4_M9    RZV2H_PINMUX(10, 4, 9)
#define GPIO_GTIOC10B_P5_3_M11   RZV2H_PINMUX(5, 3, 11)
#define GPIO_GTIOC10B_PA_5_M9    RZV2H_PINMUX(10, 5, 9)

/* GPT channel 11. */

#define GPIO_GTIOC11A_P6_2_M11   RZV2H_PINMUX(6, 2, 11)
#define GPIO_GTIOC11A_PA_6_M9    RZV2H_PINMUX(10, 6, 9)
#define GPIO_GTIOC11B_P6_3_M11   RZV2H_PINMUX(6, 3, 11)
#define GPIO_GTIOC11B_PA_7_M9    RZV2H_PINMUX(10, 7, 9)

/* GPT channel 12. */

#define GPIO_GTIOC12A_P3_0_M11   RZV2H_PINMUX(3, 0, 11)
#define GPIO_GTIOC12A_P5_4_M11   RZV2H_PINMUX(5, 4, 11)
#define GPIO_GTIOC12B_P3_1_M11   RZV2H_PINMUX(3, 1, 11)
#define GPIO_GTIOC12B_P5_5_M11   RZV2H_PINMUX(5, 5, 11)

/* GPT channel 13. */

#define GPIO_GTIOC13A_P3_2_M11   RZV2H_PINMUX(3, 2, 11)
#define GPIO_GTIOC13A_P6_4_M11   RZV2H_PINMUX(6, 4, 11)
#define GPIO_GTIOC13B_P3_3_M11   RZV2H_PINMUX(3, 3, 11)
#define GPIO_GTIOC13B_P6_5_M11   RZV2H_PINMUX(6, 5, 11)

/* GPT channel 14. */

#define GPIO_GTIOC14A_P3_4_M11   RZV2H_PINMUX(3, 4, 11)
#define GPIO_GTIOC14A_P5_6_M11   RZV2H_PINMUX(5, 6, 11)
#define GPIO_GTIOC14B_P3_5_M11   RZV2H_PINMUX(3, 5, 11)
#define GPIO_GTIOC14B_P5_7_M11   RZV2H_PINMUX(5, 7, 11)

/* GPT channel 15. */

#define GPIO_GTIOC15A_P3_6_M11   RZV2H_PINMUX(3, 6, 11)
#define GPIO_GTIOC15A_P6_6_M11   RZV2H_PINMUX(6, 6, 11)
#define GPIO_GTIOC15B_P3_7_M11   RZV2H_PINMUX(3, 7, 11)
#define GPIO_GTIOC15B_P6_7_M11   RZV2H_PINMUX(6, 7, 11)

/* xSPI0 (QSPI) pin configurations.
 *
 * Drive strength (IOPORT_CFG_DRIVE_B11) matches the HAL Renesas reference
 * pin table (RZV2HCR8_0/rzv_gen/pin_data.c), which sets it on every xSPI0
 * pin including CKP/CS0N/RESET0N -- IOPORT_CFG_SPECIAL_PURPOSE_PORT_
 * INPUT_DISABLE alone is 0x00000000 and leaves the IOLH drive-strength
 * field at its hardware reset value instead of the intended B11 (maximum)
 * setting.
 */

#define GPIO_XSPI0_CKN \
  (gpio_pinset_t){BSP_IO_XSPI0_CKN, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_DISABLE | IOPORT_CFG_DRIVE_B11)}
#define GPIO_XSPI0_CKP \
  (gpio_pinset_t){BSP_IO_XSPI0_CKP, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_DISABLE | IOPORT_CFG_DRIVE_B11)}
#define GPIO_XSPI0_CS0N \
  (gpio_pinset_t){BSP_IO_XSPI0_CS0N, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_DISABLE | IOPORT_CFG_DRIVE_B11)}
#define GPIO_XSPI0_RESET0N \
  (gpio_pinset_t){BSP_IO_XSPI0_RESET0N, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_DISABLE | IOPORT_CFG_DRIVE_B11)}

/* xSPI0_DS (Data Strobe): required for any DDR protocol (4S-4D-4D,
 * 8D-8D-8D) per the hardware manual's Figure 10.1-24 "DDR Transmission
 * and Reception Timing" -- the flash drives/samples this line to latch
 * data on both clock edges.  Not needed by (and not configured for) any
 * SDR-only protocol.  Bidirectional like IO0-3, hence INPUT_ENABLE
 * rather than CKP/CS0N/RESET0N's INPUT_DISABLE.
 */

#define GPIO_XSPI0_DS \
  (gpio_pinset_t){BSP_IO_XSPI0_DS, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_ENABLE | IOPORT_CFG_DRIVE_B11)}

/* xSPI0_CS1N: unlike CKP/CS0N/RESET0N/IO0-3 above, this signal is not
 * one of the xSPI0 peripheral's dedicated special-purpose pins -- it is
 * an ordinary PFC-muxed function on port pin P1_0 (P10, Func15 in the
 * hardware manual's pin function table).
 */

#define GPIO_XSPI0_CS1N \
  RZV2H_PINMUX_CFG(1, 0, 15, \
                   IOPORT_CFG_DRIVE_B11 | IOPORT_CFG_SLEW_RATE_FAST)

#define GPIO_XSPI0_IO0 \
  (gpio_pinset_t){BSP_IO_XSPI0_IO0, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_ENABLE | IOPORT_CFG_DRIVE_B11)}
#define GPIO_XSPI0_IO1 \
  (gpio_pinset_t){BSP_IO_XSPI0_IO1, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_ENABLE | IOPORT_CFG_DRIVE_B11)}
#define GPIO_XSPI0_IO2 \
  (gpio_pinset_t){BSP_IO_XSPI0_IO2, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_ENABLE | IOPORT_CFG_DRIVE_B11)}
#define GPIO_XSPI0_IO3 \
  (gpio_pinset_t){BSP_IO_XSPI0_IO3, \
    (IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_ENABLE | IOPORT_CFG_DRIVE_B11)}

#endif /* __ARCH_ARM_SRC_RZV2H_HARDWARE_RZV2H_PINMAP_H */
