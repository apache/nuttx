/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_pin.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_S32K3XX_PIN_H
#define __ARCH_ARM_SRC_S32K3XX_S32K3XX_PIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <hardware/s32k3xx_siul2.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to s32k3xx_pinconfig() *********************************/

/* General form (32 bits):
 *
 * oooo oooo iiii iiii ittt tsss pppn nnnn
 *         |           |    |  |   |     `--- n: Pin number (0-31)
 *         |           |    |  |   `--- p: Port (A-F)
 *         |           |    |  `--- s: Output SSS (Source Signal Select)
 *         |           |    `--- t: Input SSS (Source Signal Select)
 *         |           `--- i: IMCR (Input Multiplexing Config. Register)
 *         `--- o: Options (see below!)
 *
 * Note: IMCR field may instead contain the external WKPU source number, if
 *       Input SSS is set to 12.
 *
 * Options:
 * - Bit 24: PUE (Pull Up Enable)
 * - Bit 25: PUS (Pull Up Select)
 * - Bit 26: OBE (Output Buffer Enable)
 * - Bit 27: DSE (Drive Strength Enable)
 * - Bit 28: SRC (Slew Rate Control)
 * - Bit 29: GPIO Initial Value
 * - Bit 30: EIRQ/WKPU Enable Interrupt on Rising Edge
 * - Bit 31: EIRQ/WKPU Enable Interrupt on Falling Edge
 *
 * Not (yet) included/implemented:
 * - IBE (Input Buffer Enable)  --> IBE will be enabled for all configured
 *                                  pins.  No option needed.
 * - IFE (Input Filter Enable)  --> Only available for RESET pin.
 * - PKE (Pad Keeping Enable)   --> Keep I/O configuration when switching
 *                                  between Run and Standby modes.
 * - SMC (Safe Mode Control)    --> Output buffer will be disabled by default
 *                                  when chip enters safe mode.
 * - INV (Invert)               --> Invert all pad output.
 *
 * - Selection between DMA or interrupt request (currently only interrupts
 *   are supported).
 * - Configuration of interrupt glitch filter.
 */

/* Five bits are used to define the pin number:
 *
 * ---- ---- ---- ---- ---- ---- ---n nnnn
 */

#define _PIN_SHIFT                 (0)  /* Bits 0-4: Pin number (0-31) */
#define _PIN_MASK                  (0x1f << _PIN_SHIFT)

#define PIN(n)                     ((n) << _PIN_SHIFT)
#define PIN0                       (0 << _PIN_SHIFT)
#define PIN1                       (1 << _PIN_SHIFT)
#define PIN2                       (2 << _PIN_SHIFT)
#define PIN3                       (3 << _PIN_SHIFT)
#define PIN4                       (4 << _PIN_SHIFT)
#define PIN5                       (5 << _PIN_SHIFT)
#define PIN6                       (6 << _PIN_SHIFT)
#define PIN7                       (7 << _PIN_SHIFT)
#define PIN8                       (8 << _PIN_SHIFT)
#define PIN9                       (9 << _PIN_SHIFT)
#define PIN10                      (10 << _PIN_SHIFT)
#define PIN11                      (11 << _PIN_SHIFT)
#define PIN12                      (12 << _PIN_SHIFT)
#define PIN13                      (13 << _PIN_SHIFT)
#define PIN14                      (14 << _PIN_SHIFT)
#define PIN15                      (15 << _PIN_SHIFT)
#define PIN16                      (16 << _PIN_SHIFT)
#define PIN17                      (17 << _PIN_SHIFT)
#define PIN18                      (18 << _PIN_SHIFT)
#define PIN19                      (19 << _PIN_SHIFT)
#define PIN20                      (20 << _PIN_SHIFT)
#define PIN21                      (21 << _PIN_SHIFT)
#define PIN22                      (22 << _PIN_SHIFT)
#define PIN23                      (23 << _PIN_SHIFT)
#define PIN24                      (24 << _PIN_SHIFT)
#define PIN25                      (25 << _PIN_SHIFT)
#define PIN26                      (26 << _PIN_SHIFT)
#define PIN27                      (27 << _PIN_SHIFT)
#define PIN28                      (28 << _PIN_SHIFT)
#define PIN29                      (29 << _PIN_SHIFT)
#define PIN30                      (30 << _PIN_SHIFT)
#define PIN31                      (31 << _PIN_SHIFT)

/* Three bits are used to define the port number:
 *
 * ---- ---- ---- ---- ---- ---- ppp- ----
 */

#define _PIN_PORT_SHIFT            (5)  /* Bits 5-7: Port (A-F) */
#define _PIN_PORT_MASK             (0x07 << _PIN_PORT_SHIFT)

#define PIN_PORTA                  (S32K3XX_PORTA << _PIN_PORT_SHIFT)
#define PIN_PORTB                  (S32K3XX_PORTB << _PIN_PORT_SHIFT)
#define PIN_PORTC                  (S32K3XX_PORTC << _PIN_PORT_SHIFT)
#define PIN_PORTD                  (S32K3XX_PORTD << _PIN_PORT_SHIFT)
#define PIN_PORTE                  (S32K3XX_PORTE << _PIN_PORT_SHIFT)
#define PIN_PORTF                  (S32K3XX_PORTF << _PIN_PORT_SHIFT)
#define PIN_PORTG                  (S32K3XX_PORTG << _PIN_PORT_SHIFT)

/* Sixteen bits are used to define the input/output multiplexing:
 *
 * ---- ---- iiii iiii ittt tsss ---- ----
 */

#define _PIN_MODE_SHIFT            (8)  /* Bits 8-23: Pin Mode */
#define _PIN_MODE_MASK             (0xffff << _PIN_MODE_SHIFT)

/* Of which three bits are used to define the output multiplexing:
 *
 * ---- ---- ---- ---- ---- -sss ---- ----
 */

#define _PIN_OUTPUT_MODE_SHIFT     (8)  /* Bits 8-10: Pin Output Mode */
#define _PIN_OUTPUT_MODE_MASK      (0x07 << _PIN_OUTPUT_MODE_SHIFT)

#define _PIN_MODE_GPIO             (0x00) /* 000 GPIO */
#define _PIN_OUTPUT_MODE_ALT0      _PIN_MODE_GPIO
#define _PIN_OUTPUT_MODE_ALT1      (0x01) /* 001 Output Alternative 1 */
#define _PIN_OUTPUT_MODE_ALT2      (0x02) /* 010 Output Alternative 2 */
#define _PIN_OUTPUT_MODE_ALT3      (0x03) /* 011 Output Alternative 3 */
#define _PIN_OUTPUT_MODE_ALT4      (0x04) /* 100 Output Alternative 4 */
#define _PIN_OUTPUT_MODE_ALT5      (0x05) /* 101 Output Alternative 5 */
#define _PIN_OUTPUT_MODE_ALT6      (0x06) /* 110 Output Alternative 6 */
#define _PIN_OUTPUT_MODE_ALT7      (0x07) /* 111 Output Alternative 7 */

#define PIN_MODE_GPIO              (_PIN_MODE_GPIO        << _PIN_OUTPUT_MODE_SHIFT) /* 000 GPIO */
#define PIN_OUTPUT_MODE_ALT0       PIN_MODE_GPIO
#define PIN_OUTPUT_MODE_ALT1       (_PIN_OUTPUT_MODE_ALT1 << _PIN_OUTPUT_MODE_SHIFT) /* 001 Output Alternative 1 */
#define PIN_OUTPUT_MODE_ALT2       (_PIN_OUTPUT_MODE_ALT2 << _PIN_OUTPUT_MODE_SHIFT) /* 010 Output Alternative 2 */
#define PIN_OUTPUT_MODE_ALT3       (_PIN_OUTPUT_MODE_ALT3 << _PIN_OUTPUT_MODE_SHIFT) /* 011 Output Alternative 3 */
#define PIN_OUTPUT_MODE_ALT4       (_PIN_OUTPUT_MODE_ALT4 << _PIN_OUTPUT_MODE_SHIFT) /* 100 Output Alternative 4 */
#define PIN_OUTPUT_MODE_ALT5       (_PIN_OUTPUT_MODE_ALT5 << _PIN_OUTPUT_MODE_SHIFT) /* 101 Output Alternative 5 */
#define PIN_OUTPUT_MODE_ALT6       (_PIN_OUTPUT_MODE_ALT6 << _PIN_OUTPUT_MODE_SHIFT) /* 110 Output Alternative 6 */
#define PIN_OUTPUT_MODE_ALT7       (_PIN_OUTPUT_MODE_ALT7 << _PIN_OUTPUT_MODE_SHIFT) /* 111 Output Alternative 7 */

/* Four bits define the input multiplexing (together with the IMCR field):
 *
 * ---- ---- ---- ---- -ttt t--- ---- ----
 */

#define _PIN_INPUT_MODE_SHIFT      (11) /* Bits 11-14: Pin Input Mode */
#define _PIN_INPUT_MODE_MASK       (0x0f << _PIN_INPUT_MODE_SHIFT)

#define _PIN_INPUT_MODE_DIS        (0x00) /* 0000 Input Signal Disabled (- Does NOT disable the input buffer!) */
#define _PIN_INPUT_MODE_ALT0       _PIN_INPUT_MODE_DIS
#define _PIN_INPUT_MODE_ALT1       (0x01) /* 0001 Input Alternative 1 */
#define _PIN_INPUT_MODE_ALT2       (0x02) /* 0010 Input Alternative 2 */
#define _PIN_INPUT_MODE_ALT3       (0x03) /* 0011 Input Alternative 3 */
#define _PIN_INPUT_MODE_ALT4       (0x04) /* 0100 Input Alternative 4 */
#define _PIN_INPUT_MODE_ALT5       (0x05) /* 0101 Input Alternative 5 */
#define _PIN_INPUT_MODE_ALT6       (0x06) /* 0110 Input Alternative 6 */
#define _PIN_INPUT_MODE_ALT7       (0x07) /* 0111 Input Alternative 7 */
#define _PIN_INPUT_MODE_ALT8       (0x08) /* 1000 Input Alternative 8 */
#define _PIN_INPUT_MODE_ALT9       (0x09) /* 1001 Input Alternative 9 */
#define _PIN_INPUT_MODE_ALT10      (0x0a) /* 1010 Input Alternative 10 */
#define _PIN_INPUT_MODE_ALT11      (0x0b) /* 1011 Input Alternative 11 */
#define _PIN_INPUT_MODE_WKPU       (0x0c) /* 1100 Use Wakeup Unit for external interrupt */
#define _PIN_INPUT_MODE_ALT13      (0x0d) /* 1101 Unused input mode */
#define _PIN_INPUT_MODE_ALT14      (0x0e) /* 1110 Unused input mode */
#define _PIN_INPUT_MODE_ALT15      (0x0f) /* 1111 Unused input mode */

#define PIN_INPUT_MODE_DIS         (_PIN_INPUT_MODE_DIS   << _PIN_INPUT_MODE_SHIFT) /* 0000 Input Signal Disabled (- Does NOT disable the input buffer!) */
#define PIN_INPUT_MODE_ALT0        PIN_INPUT_MODE_DIS
#define PIN_INPUT_MODE_ALT1        (_PIN_INPUT_MODE_ALT1  << _PIN_INPUT_MODE_SHIFT) /* 0001 Input Alternative 1 */
#define PIN_INPUT_MODE_ALT2        (_PIN_INPUT_MODE_ALT2  << _PIN_INPUT_MODE_SHIFT) /* 0010 Input Alternative 2 */
#define PIN_INPUT_MODE_ALT3        (_PIN_INPUT_MODE_ALT3  << _PIN_INPUT_MODE_SHIFT) /* 0011 Input Alternative 3 */
#define PIN_INPUT_MODE_ALT4        (_PIN_INPUT_MODE_ALT4  << _PIN_INPUT_MODE_SHIFT) /* 0100 Input Alternative 4 */
#define PIN_INPUT_MODE_ALT5        (_PIN_INPUT_MODE_ALT5  << _PIN_INPUT_MODE_SHIFT) /* 0101 Input Alternative 5 */
#define PIN_INPUT_MODE_ALT6        (_PIN_INPUT_MODE_ALT6  << _PIN_INPUT_MODE_SHIFT) /* 0110 Input Alternative 6 */
#define PIN_INPUT_MODE_ALT7        (_PIN_INPUT_MODE_ALT7  << _PIN_INPUT_MODE_SHIFT) /* 0111 Input Alternative 7 */
#define PIN_INPUT_MODE_ALT8        (_PIN_INPUT_MODE_ALT8  << _PIN_INPUT_MODE_SHIFT) /* 1000 Input Alternative 8 */
#define PIN_INPUT_MODE_ALT9        (_PIN_INPUT_MODE_ALT9  << _PIN_INPUT_MODE_SHIFT) /* 1001 Input Alternative 9 */
#define PIN_INPUT_MODE_ALT10       (_PIN_INPUT_MODE_ALT10 << _PIN_INPUT_MODE_SHIFT) /* 1010 Input Alternative 10 */
#define PIN_INPUT_MODE_ALT11       (_PIN_INPUT_MODE_ALT11 << _PIN_INPUT_MODE_SHIFT) /* 1011 Input Alternative 11 */
#define PIN_INPUT_MODE_WKPU        (_PIN_INPUT_MODE_WKPU  << _PIN_INPUT_MODE_SHIFT) /* 1100 Use Wakeup Unit for external interrupt */
#define PIN_INPUT_MODE_ALT13       (_PIN_INPUT_MODE_ALT13 << _PIN_INPUT_MODE_SHIFT) /* 1101 Unused input mode */
#define PIN_INPUT_MODE_ALT14       (_PIN_INPUT_MODE_ALT14 << _PIN_INPUT_MODE_SHIFT) /* 1110 Unused input mode */
#define PIN_INPUT_MODE_ALT15       (_PIN_INPUT_MODE_ALT15 << _PIN_INPUT_MODE_SHIFT) /* 1111 Unused input mode */

/* Nine bits specify the IMCR number for the input multiplexing:
 *
 * ---- ---- iiii iiii i--- ---- ---- ----
 */

#define _IMCR_SHIFT                (15) /* Bits 15-23: IMCR */
#define _IMCR_MASK                 (0x01ff << _IMCR_SHIFT)
#  define IMCR(n)                  ((((n) - 512) << _IMCR_SHIFT) & _IMCR_MASK)

#define _WKPU_SHIFT                _IMCR_SHIFT /* WKPU number re-uses the IMCR field, only applicable if Input SSS = 12 */
#define _WKPU_MASK                 (0x3f << _WKPU_SHIFT)
#  define WPKU(n)                  (((n) << _WKPU_SHIFT) & _WKPU_MASK)

/* Eight bits are used to define various pin options:
 *
 * oooo oooo ---- ---- ---- ---- ---- ----
 */

#define _PIN_OPTIONS_SHIFT         (24) /* Bits 24-30: GPIO Pin Options */
#define _PIN_OPTIONS_MASK          (0x7f << _PIN_OPTIONS_SHIFT)

/* Of which two bits are used to define pull-up/pull-down behavior:
 *
 * ---- --oo ---- ---- ---- ---- ---- ----
 */

#define _PIN_INPUT_PULL_SHIFT      (24) /* Bits 24-25: Pull-Down/Pull-Up Behavior */
#define _PIN_INPUT_PULL_MASK       (0x03 << _PIN_INPUT_PULL_SHIFT)
#  define _PIN_INPUT_PULLENABLE    (0x01 << _PIN_INPUT_PULL_SHIFT) /* Pull Enable (PUE) */
#  define _PIN_INPUT_PULLSELECT    (0x02 << _PIN_INPUT_PULL_SHIFT) /* Pull Select (PUS) */
#  define PIN_INPUT_PULLDOWN       (0x01 << _PIN_INPUT_PULL_SHIFT) /* Pull-Down */
#  define PIN_INPUT_PULLUP         (0x03 << _PIN_INPUT_PULL_SHIFT) /* Pull-Up */

/* One bit enables the output buffer:
 *
 * ---- -o-- ---- ---- ---- ---- ---- ----
 */

#define _PIN_OUTPUT_BUFFER_SHIFT   (26) /* Bit 26: Output Buffer Enable */
#define _PIN_OUTPUT_BUFFER_MASK    (1 << _PIN_OUTPUT_BUFFER_SHIFT)
#  define PIN_OUTPUT_BUFFERDIS     (0 << _PIN_OUTPUT_BUFFER_SHIFT)
#  define PIN_OUTPUT_BUFFERENA     (1 << _PIN_OUTPUT_BUFFER_SHIFT)

/* One bit defines the drive strength:
 *
 * ---- o--- ---- ---- ---- ---- ---- ----
 */

#define _PIN_OUTPUT_DRIVE_SHIFT    (27) /* Bit 27: Drive Strength Enable */
#define _PIN_OUTPUT_DRIVE_MASK     (1 << _PIN_OUTPUT_DRIVE_SHIFT)
#  define PIN_OUTPUT_LOWDRIVE      (0 << _PIN_OUTPUT_DRIVE_SHIFT)
#  define PIN_OUTPUT_HIGHDRIVE     (1 << _PIN_OUTPUT_DRIVE_SHIFT)

/* One bit defines the slew rate:
 *
 * ---o ---- ---- ---- ---- ---- ---- ----
 */

#define _PIN_OUTPUT_SLEW_SHIFT     (28) /* Bit 28: Slew Rate Control */
#define _PIN_OUTPUT_SLEW_MASK      (1 << _PIN_OUTPUT_SLEW_SHIFT)
#  define PIN_OUTPUT_FASTSLEW      (0 << _PIN_OUTPUT_SLEW_SHIFT) /* 0: Fast Slew Rate */
#  define PIN_OUTPUT_SLOWSLEW      (1 << _PIN_OUTPUT_SLEW_SHIFT) /* 1: Slow (Limited) Slew Rate */

/* The initial value for GPIO (Alternative 0 outputs):
 *
 * --o- ---- ---- ---- ---- ---- ---- ----
 */

#define _PIN_OUTPUT_INVAL_SHIFT    (29) /* Bit 29: GPIO Inititial Value */
#define _PIN_OUTPUT_INVAL_MASK     (1 << _PIN_OUTPUT_INVAL_SHIFT)
#  define GPIO_OUTPUT_ZERO         (0 << _PIN_OUTPUT_INVAL_SHIFT) /* 0: Initial output value is 0 */
#  define GPIO_OUTPUT_ONE          (1 << _PIN_OUTPUT_INVAL_SHIFT) /* 1: Initial output value is 1 */

/* Two bits are used to encode interrupt options:
 *
 * oo- ---- ---- ---- ---- ---- ---- ----
 *
 * TO DO: May be expanded to include DMA options by reusing bits that are
 *        only relevant for outputs.
 */

#define _PIN_INT_SHIFT             (30)
#define _PIN_INT_MASK              (0x03 << _PIN_INT_SHIFT)
#  define PIN_INT_RISING           (0x01 << _PIN_INT_SHIFT) /* 01: Interrupt on rising edge */
#  define PIN_INT_FALLING          (0x02 << _PIN_INT_SHIFT) /* 10: Interrupt on falling edge */
#  define PIN_INT_BOTH             (0x03 << _PIN_INT_SHIFT) /* 11: Interrupt on both edges */

/* End-user pin modes and configurations.
 *
 * Notes:
 * (1) None of the digital options are available for analog functions, which
 * are handled in parallel and are configured elsewhere.
 * (2) Digital settings may be combined (OR'ed) provided that input-only
 * and output-only options are not intermixed.
 */

/* GPIO pins */

#define GPIO_INPUT                 PIN_MODE_GPIO /* Note: Input Buffer will always be enabled for all configured pins! */
#define GPIO_PULLDOWN              (PIN_MODE_GPIO | PIN_INPUT_PULLDOWN)
#define GPIO_PULLUP                (PIN_MODE_GPIO | PIN_INPUT_PULLUP)
#define GPIO_OUTPUT                (PIN_MODE_GPIO | PIN_OUTPUT_BUFFERENA)
#define GPIO_LOWDRIVE              (PIN_MODE_GPIO | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define GPIO_HIGHDRIVE             (PIN_MODE_GPIO | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define GPIO_FASTSLEW              (PIN_MODE_GPIO | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define GPIO_SLOWSLEW              (PIN_MODE_GPIO | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

/* Outputs */

#define PIN_OUTPUT_ALT0            (PIN_OUTPUT_MODE_ALT0 | PIN_OUTPUT_BUFFERENA)
#define PIN_OUTPUT_ALT0_LOWDRIVE   (PIN_OUTPUT_MODE_ALT0 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define PIN_OUTPUT_ALT0_HIGHDRIVE  (PIN_OUTPUT_MODE_ALT0 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define PIN_OUTPUT_ALT0_FASTSLEW   (PIN_OUTPUT_MODE_ALT0 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define PIN_OUTPUT_ALT0_SLOWSLEW   (PIN_OUTPUT_MODE_ALT0 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

#define PIN_OUTPUT_ALT1            (PIN_OUTPUT_MODE_ALT1 | PIN_OUTPUT_BUFFERENA)
#define PIN_OUTPUT_ALT1_LOWDRIVE   (PIN_OUTPUT_MODE_ALT1 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define PIN_OUTPUT_ALT1_HIGHDRIVE  (PIN_OUTPUT_MODE_ALT1 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define PIN_OUTPUT_ALT1_FASTSLEW   (PIN_OUTPUT_MODE_ALT1 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define PIN_OUTPUT_ALT1_SLOWSLEW   (PIN_OUTPUT_MODE_ALT1 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

#define PIN_OUTPUT_ALT2            (PIN_OUTPUT_MODE_ALT2 | PIN_OUTPUT_BUFFERENA)
#define PIN_OUTPUT_ALT2_LOWDRIVE   (PIN_OUTPUT_MODE_ALT2 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define PIN_OUTPUT_ALT2_HIGHDRIVE  (PIN_OUTPUT_MODE_ALT2 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define PIN_OUTPUT_ALT2_FASTSLEW   (PIN_OUTPUT_MODE_ALT2 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define PIN_OUTPUT_ALT2_SLOWSLEW   (PIN_OUTPUT_MODE_ALT2 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

#define PIN_OUTPUT_ALT3            (PIN_OUTPUT_MODE_ALT3 | PIN_OUTPUT_BUFFERENA)
#define PIN_OUTPUT_ALT3_LOWDRIVE   (PIN_OUTPUT_MODE_ALT3 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define PIN_OUTPUT_ALT3_HIGHDRIVE  (PIN_OUTPUT_MODE_ALT3 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define PIN_OUTPUT_ALT3_FASTSLEW   (PIN_OUTPUT_MODE_ALT3 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define PIN_OUTPUT_ALT3_SLOWSLEW   (PIN_OUTPUT_MODE_ALT3 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

#define PIN_OUTPUT_ALT4            (PIN_OUTPUT_MODE_ALT4 | PIN_OUTPUT_BUFFERENA)
#define PIN_OUTPUT_ALT4_LOWDRIVE   (PIN_OUTPUT_MODE_ALT4 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define PIN_OUTPUT_ALT4_HIGHDRIVE  (PIN_OUTPUT_MODE_ALT4 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define PIN_OUTPUT_ALT4_FASTSLEW   (PIN_OUTPUT_MODE_ALT4 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define PIN_OUTPUT_ALT4_SLOWSLEW   (PIN_OUTPUT_MODE_ALT4 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

#define PIN_OUTPUT_ALT5            (PIN_OUTPUT_MODE_ALT5 | PIN_OUTPUT_BUFFERENA)
#define PIN_OUTPUT_ALT5_LOWDRIVE   (PIN_OUTPUT_MODE_ALT5 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define PIN_OUTPUT_ALT5_HIGHDRIVE  (PIN_OUTPUT_MODE_ALT5 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define PIN_OUTPUT_ALT5_FASTSLEW   (PIN_OUTPUT_MODE_ALT5 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define PIN_OUTPUT_ALT5_SLOWSLEW   (PIN_OUTPUT_MODE_ALT5 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

#define PIN_OUTPUT_ALT6            (PIN_OUTPUT_MODE_ALT6 | PIN_OUTPUT_BUFFERENA)
#define PIN_OUTPUT_ALT6_LOWDRIVE   (PIN_OUTPUT_MODE_ALT6 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define PIN_OUTPUT_ALT6_HIGHDRIVE  (PIN_OUTPUT_MODE_ALT6 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define PIN_OUTPUT_ALT6_FASTSLEW   (PIN_OUTPUT_MODE_ALT6 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define PIN_OUTPUT_ALT6_SLOWSLEW   (PIN_OUTPUT_MODE_ALT6 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

#define PIN_OUTPUT_ALT7            (PIN_OUTPUT_MODE_ALT7 | PIN_OUTPUT_BUFFERENA)
#define PIN_OUTPUT_ALT7_LOWDRIVE   (PIN_OUTPUT_MODE_ALT7 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_LOWDRIVE)
#define PIN_OUTPUT_ALT7_HIGHDRIVE  (PIN_OUTPUT_MODE_ALT7 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_HIGHDRIVE)
#define PIN_OUTPUT_ALT7_FASTSLEW   (PIN_OUTPUT_MODE_ALT7 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_FASTSLEW)
#define PIN_OUTPUT_ALT7_SLOWSLEW   (PIN_OUTPUT_MODE_ALT7 | PIN_OUTPUT_BUFFERENA | PIN_OUTPUT_SLOWSLEW)

/* Inputs */

#define PIN_INPUT_ALT1             PIN_INPUT_MODE_ALT1
#define PIN_INPUT_ALT1_PULLDOWN    (PIN_INPUT_MODE_ALT1  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT1_PULLUP      (PIN_INPUT_MODE_ALT1  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT2             PIN_INPUT_MODE_ALT2
#define PIN_INPUT_ALT2_PULLDOWN    (PIN_INPUT_MODE_ALT2  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT2_PULLUP      (PIN_INPUT_MODE_ALT2  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT3             PIN_INPUT_MODE_ALT3
#define PIN_INPUT_ALT3_PULLDOWN    (PIN_INPUT_MODE_ALT3  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT3_PULLUP      (PIN_INPUT_MODE_ALT3  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT4             PIN_INPUT_MODE_ALT4
#define PIN_INPUT_ALT4_PULLDOWN    (PIN_INPUT_MODE_ALT4  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT4_PULLUP      (PIN_INPUT_MODE_ALT4  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT5             PIN_INPUT_MODE_ALT5
#define PIN_INPUT_ALT5_PULLDOWN    (PIN_INPUT_MODE_ALT5  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT5_PULLUP      (PIN_INPUT_MODE_ALT5  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT6             PIN_INPUT_MODE_ALT6
#define PIN_INPUT_ALT6_PULLDOWN    (PIN_INPUT_MODE_ALT6  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT6_PULLUP      (PIN_INPUT_MODE_ALT6  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT7             PIN_INPUT_MODE_ALT7
#define PIN_INPUT_ALT7_PULLDOWN    (PIN_INPUT_MODE_ALT7  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT7_PULLUP      (PIN_INPUT_MODE_ALT7  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT8             PIN_INPUT_MODE_ALT8
#define PIN_INPUT_ALT8_PULLDOWN    (PIN_INPUT_MODE_ALT8  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT8_PULLUP      (PIN_INPUT_MODE_ALT8  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT9             PIN_INPUT_MODE_ALT9
#define PIN_INPUT_ALT9_PULLDOWN    (PIN_INPUT_MODE_ALT9  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT9_PULLUP      (PIN_INPUT_MODE_ALT9  | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT10            PIN_INPUT_MODE_ALT10
#define PIN_INPUT_ALT10_PULLDOWN   (PIN_INPUT_MODE_ALT10 | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT10_PULLUP     (PIN_INPUT_MODE_ALT10 | PIN_INPUT_PULLUP)

#define PIN_INPUT_ALT11            PIN_INPUT_MODE_ALT11
#define PIN_INPUT_ALT11_PULLDOWN   (PIN_INPUT_MODE_ALT11 | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_ALT11_PULLUP     (PIN_INPUT_MODE_ALT11 | PIN_INPUT_PULLUP)

#define PIN_INPUT_WKPU             PIN_INPUT_MODE_WKPU
#define PIN_INPUT_WKPU_PULLDOWN    (PIN_INPUT_MODE_WKPU  | PIN_INPUT_PULLDOWN)
#define PIN_INPUT_WKPU_PULLUP      (PIN_INPUT_MODE_WKPU  | PIN_INPUT_PULLUP)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_pinconfig
 *
 * Description:
 *   Configure a pin based on a bit-encoded description of the pin.
 *
 ****************************************************************************/

int s32k3xx_pinconfig(uint32_t cfgset);

/****************************************************************************
 * Name: s32k3xx_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void s32k3xx_gpiowrite(uint32_t pinset, bool value);

/****************************************************************************
 * Name: s32k3xx_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool s32k3xx_gpioread(uint32_t pinset);

/****************************************************************************
 * Name: s32k3xx_pinirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_GPIOIRQ
void s32k3xx_pinirq_initialize(void);
#else
#  define s32k3xx_pinirq_initialize()
#endif

/****************************************************************************
 * Name: s32k3xx_pinirqattach
 *
 * Description:
 *   Attach a pin interrupt handler.  The normal initialization sequence is:
 *
 *   1. Call s32k3xx_pinconfig() to configure the interrupting pin
 *      (pin interrupts will be disabled.
 *   2. Call s32k3xx_pinirqattach() to attach the pin interrupt handling
 *      function.
 *   3. Call s32k3xx_pinirqenable() to enable interrupts on the pin.
 *
 * Input Parameters:
 *   pinset -  Pin configuration
 *   pinisr -  Pin interrupt service routine
 *   arg    -  An argument that will be provided to the interrupt service
 *             routine.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int s32k3xx_pinirqattach(uint32_t pinset, xcpt_t pinisr, void *arg);

/****************************************************************************
 * Name: s32k3xx_pinirqenable
 *
 * Description:
 *   Enable the interrupt for specified pin IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_GPIOIRQ
void s32k3xx_pinirqenable(uint32_t pinset);
#else
#  define s32k3xx_pinirqenable(pinset)
#endif

/****************************************************************************
 * Name: s32k3xx_pinirqdisable
 *
 * Description:
 *   Disable the interrupt for specified pin
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_GPIOIRQ
void s32k3xx_pinirqdisable(uint32_t pinset);
#else
#  define s32k3xx_pinirqdisable(pinset)
#endif

/****************************************************************************
 * Name: s32k3xx_pindmaenable
 *
 * Description:
 *   Enable DMA for specified pin
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_DMA
void s32k3xx_pindmaenable(uint32_t pinset);
#endif

/****************************************************************************
 * Name: s32k3xx_pindmadisable
 *
 * Description:
 *   Disable DMA for specified pin
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_DMA
void s32k3xx_pindmadisable(uint32_t pinset);
#endif

/****************************************************************************
 * Function:  s32k3xx_pindump
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
void s32k3xx_pindump(uint32_t pinset, const char *msg);
#else
#  define s32k3xx_pindump(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K3XX_S32K3XX_PIN_H */
