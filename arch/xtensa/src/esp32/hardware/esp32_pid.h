/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_pid.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PID_H_
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PID_H_

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_PIDCTRL_BASE                        0x3ff1f000

/* Bits 1..7: 1 if interrupt will be triggering PID change */

#define PIDCTRL_INTERRUPT_ENABLE_REG            ((REG_PIDCTRL_BASE) + 0x000)

/* PIDCTRL_INTERRUPT_ENABLE : R/W; bitpos: [7:1]; default: 0;
 * These bits are used to enable interrupt identification and processing.
 */

#define PIDCTRL_INTERRUPT_ENABLE        0x0000007f
#define PIDCTRL_INTERRUPT_ENABLE_M      (PIDCTRL_INTERRUPT_ENABLE_V << PIDCTRL_INTERRUPT_ENABLE_S)
#define PIDCTRL_INTERRUPT_ENABLE_V      0x0000007f
#define PIDCTRL_INTERRUPT_ENABLE_S      1

/* Vectors for the various interrupt handlers */

#define PIDCTRL_INTERRUPT_ADDR_1_REG            ((REG_PIDCTRL_BASE) + 0x004)
#define PIDCTRL_INTERRUPT_ADDR_2_REG            ((REG_PIDCTRL_BASE) + 0x008)
#define PIDCTRL_INTERRUPT_ADDR_3_REG            ((REG_PIDCTRL_BASE) + 0x00c)
#define PIDCTRL_INTERRUPT_ADDR_4_REG            ((REG_PIDCTRL_BASE) + 0x010)
#define PIDCTRL_INTERRUPT_ADDR_5_REG            ((REG_PIDCTRL_BASE) + 0x014)
#define PIDCTRL_INTERRUPT_ADDR_6_REG            ((REG_PIDCTRL_BASE) + 0x018)
#define PIDCTRL_INTERRUPT_ADDR_7_REG            ((REG_PIDCTRL_BASE) + 0x01c)

/* Delay, in CPU cycles, before switching to new PID */

#define PIDCTRL_PID_DELAY_REG                   ((REG_PIDCTRL_BASE) + 0x020)

/* PIDCTRL_PID_DELAY : R/W; bitpos: [11:0]; default: 20;
 * Delay until newly assigned PID is valid.
 */

#define PIDCTRL_PID_DELAY        0x00001fff
#define PIDCTRL_PID_DELAY_M      (PIDCTRL_PID_DELAY_V << PIDCTRL_PID_DELAY_S)
#define PIDCTRL_PID_DELAY_V      0x00001fff
#define PIDCTRL_PID_DELAY_S      0

#define PIDCTRL_NMI_DELAY_REG                   ((REG_PIDCTRL_BASE) + 0x024)

/* PIDCTRL_NMI_DELAY : R/W; bitpos: [11:0]; default: 16;
 * Delay for disabling CPU NMI interrupt mask signal.
 */

#define PIDCTRL_NMI_DELAY        0x00001fff
#define PIDCTRL_NMI_DELAY_M      (PIDCTRL_NMI_DELAY_V << PIDCTRL_NMI_DELAY_S)
#define PIDCTRL_NMI_DELAY_V      0x00001fff
#define PIDCTRL_NMI_DELAY_S      0

/* Last detected interrupt. Set by hw on int. */

#define PIDCTRL_LEVEL_REG                       ((REG_PIDCTRL_BASE) + 0x028)

/* PID/prev int data for each int */

#define PIDCTRL_FROM_REG(i)         ((REG_PIDCTRL_BASE) + 0x028 + (0x4 * i))

#define PIDCTRL_FROM_1_REG                      ((REG_PIDCTRL_BASE) + 0x02c)
#define PIDCTRL_FROM_2_REG                      ((REG_PIDCTRL_BASE) + 0x030)
#define PIDCTRL_FROM_3_REG                      ((REG_PIDCTRL_BASE) + 0x034)
#define PIDCTRL_FROM_4_REG                      ((REG_PIDCTRL_BASE) + 0x038)
#define PIDCTRL_FROM_5_REG                      ((REG_PIDCTRL_BASE) + 0x03c)
#define PIDCTRL_FROM_6_REG                      ((REG_PIDCTRL_BASE) + 0x040)
#define PIDCTRL_FROM_7_REG                      ((REG_PIDCTRL_BASE) + 0x044)

/* PIDCTRL_FROM_INT : R/W; bitpos: [6:3]; default: 0;
 * Interrupt status of the system before the interrupt occurs.
 */

#define PIDCTRL_FROM_INT        0x0000000f
#define PIDCTRL_FROM_INT_M      (PIDCTRL_FROM_INT_V << PIDCTRL_FROM_INT_S)
#define PIDCTRL_FROM_INT_V      0x0000000f
#define PIDCTRL_FROM_INT_S      3

/* PIDCTRL_FROM_PID : R/W; bitpos: [2:0]; default: 0;
 * Process running on the CPU before the interrupt occurs.
 */

#define PIDCTRL_FROM_PID        0x00000007
#define PIDCTRL_FROM_PID_M      (PIDCTRL_FROM_PID_V << PIDCTRL_FROM_PID_S)
#define PIDCTRL_FROM_PID_V      0x00000007
#define PIDCTRL_FROM_PID_S      0

/* PID to be set after confirm routine */

#define PIDCTRL_PID_NEW_REG                     ((REG_PIDCTRL_BASE) + 0x048)

/* Write to kick off PID change */

#define PIDCTRL_PID_CONFIRM_REG                 ((REG_PIDCTRL_BASE) + 0x04c)

/* Write to mask NMI */

#define PIDCTRL_NMI_MASK_ENABLE_REG             ((REG_PIDCTRL_BASE) + 0x054)

/* Write to unmask NMI */

#define PIDCTRL_NMI_MASK_DISABLE_REG            ((REG_PIDCTRL_BASE) + 0x058)

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PID_H_ */
