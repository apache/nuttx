/****************************************************************************
 * drivers/ioexpander/icjx.h
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

#ifndef __DRIVERS_IOEXPANDER_ICJX_H
#define __DRIVERS_IOEXPANDER_ICJX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mutex.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_ICJX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_SPI
 *     SPI support is required
 *   CONFIG_IOEXPANDER
 *     Enables I/O expander support
 *
 * CONFIG_IOEXPANDER_ICJX
 *   Enables support for the ICJX driver (Needs CONFIG_INPUT)
 * CONFIG_ICJX_MULTIPLE
 *   Can be defined to support multiple ICJX devices on board.
 */

#ifndef CONFIG_SPI
#  error "CONFIG_SPI is required by ICJX"
#endif

/* iC-JX Registers **********************************************************/

#define ICJX_INPUT_A            0x00
#define ICJX_INPUT_B            0x01
#define ICJX_CHNG_MSG_A         0x02
#define ICJX_CNNG_MSG_B         0x03
#define ICJX_INT_STATUS_A       0x04
#define ICJX_INT_STATUS_B       0x05
#define ICJX_OVERCURR_MSG_A     0x06
#define ICJX_OVERCURR_MSG_B     0x07
#define ICJX_OVERCURR_STATUS_A  0x08
#define ICJX_OVERCURR_STATUS_B  0x09
#define ICJX_ADC_DATA_1         0x0A
#define ICJX_ADC_DATA_2         0x0B
#define ICJX_OUTPUT_A           0x0C
#define ICJX_OUTPUT_B           0x0D
#define ICJX_FLASH_PULSE_A      0x0E
#define ICJX_FLASH_PULSE_B      0x0F
#define ICJX_CHNG_INT_EN_A      0x10
#define ICJX_CHNG_INT_EN_B      0x11
#define ICJX_OVERCURR_INT_EN_A  0x12
#define ICJX_OVERCURR_INT_EN_B  0x13
#define ICJX_CTRL_WORD_1_A      0x14
#define ICJX_CTRL_WORD_1_B      0x15
#define ICJX_CTRL_WORD_2_A      0x16
#define ICJX_CTRL_WORD_2_B      0x17
#define ICJX_CTRL_WORD_3_A      0x18
#define ICJX_CTRL_WORD_3_B      0x19
#define ICJX_CTRL_WORD_4        0x1A
#define ICJX_CTRL_WORD_5        0x1B
#define ICJX_CTRL_WORD_6        0x1C
#define ICJX_DEV_ID             0x1D
#define ICJX_TEST_1             0x1E
#define ICJX_TEST_2             0x1F

/* Control Word 1 */

#define ICJX_CTRL_WORD_1_BYP0   (1 << 3)
#define ICJX_CTRL_WORD_1_BYP1   (1 << 7)

/* Control Word 2 */

#define ICJX_CTRL_WORD_2_NIOL   (1 << 3)
#define ICJX_CTRL_WORD_2_NIOH   (1 << 7)

/* Control Word 4 */

#define ICJX_CTRL_WORD_4_EOI    (1 << 7)

/* Interrupt Status Register A */

#define ICJX_ISR_A_SCS   (1 << 0)
#define ICJX_ISR_A_ET1   (1 << 1)
#define ICJX_ISR_A_ET2   (1 << 2)
#define ICJX_ISR_A_ISCI  (1 << 4)
#define ICJX_ISR_A_IET1  (1 << 5)
#define ICJX_ISR_A_IET2  (1 << 6)
#define ICJX_ISR_A_DCHI  (1 << 7)

/* Interrupt Status Register B */

#define ICJX_ISR_B_USA   (1 << 0)
#define ICJX_ISR_B_USD   (1 << 1)
#define ICJX_ISR_B_EOC   (1 << 2)
#define ICJX_ISR_B_IUSA  (1 << 4)
#define ICJX_ISR_B_IUSD  (1 << 5)
#define ICJX_ISR_B_ISD   (1 << 6)
#define ICJX_ISR_B_IOEC  (1 << 7)

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_ICJX */
#endif /* __DRIVERS_IOEXPANDER_ICJX_H */
