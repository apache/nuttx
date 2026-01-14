/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_intctrl.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_INTCTRL_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_INTCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP23XX_TIMER0_IRQ_0         0
#define RP23XX_TIMER0_IRQ_1         1
#define RP23XX_TIMER0_IRQ_2         2
#define RP23XX_TIMER0_IRQ_3         3
#define RP23XX_TIMER1_IRQ_0         4
#define RP23XX_TIMER1_IRQ_1         5
#define RP23XX_TIMER1_IRQ_2         6
#define RP23XX_TIMER1_IRQ_3         7
#define RP23XX_PWM_IRQ_WRAP_0       8
#define RP23XX_PWM_IRQ_WRAP_1       9
#define RP23XX_DMA_IRQ_0            10
#define RP23XX_DMA_IRQ_1            11
#define RP23XX_DMA_IRQ_2            12
#define RP23XX_DMA_IRQ_3            13
#define RP23XX_USBCTRL_IRQ          14
#define RP23XX_PIO0_IRQ_0           15
#define RP23XX_PIO0_IRQ_1           16
#define RP23XX_PIO1_IRQ_0           17
#define RP23XX_PIO1_IRQ_1           18
#define RP23XX_PIO2_IRQ_0           19
#define RP23XX_PIO2_IRQ_1           20
#define RP23XX_IO_IRQ_BANK0         21
#define RP23XX_IO_IRQ_BANK0_NS      22
#define RP23XX_IO_IRQ_QSPI          23
#define RP23XX_IO_IRQ_QSPI_NS       24
#define RP23XX_SIO_IRQ_FIFO         25
#define RP23XX_SIO_IRQ_BELL         26
#define RP23XX_SIO_IRQ_FIFO_NS      27
#define RP23XX_SIO_IRQ_BELL_NS      28
#define RP23XX_SIO_IRQ_MTIMECMP     29
#define RP23XX_CLOCKS_IRQ           30
#define RP23XX_SPI0_IRQ             31
#define RP23XX_SPI1_IRQ             32
#define RP23XX_UART0_IRQ            33
#define RP23XX_UART1_IRQ            34
#define RP23XX_ADC_IRQ_FIFO         35
#define RP23XX_I2C0_IRQ             36
#define RP23XX_I2C1_IRQ             37
#define RP23XX_OTP_IRQ              38
#define RP23XX_TRNG_IRQ             39
#define RP23XX_PROC0_IRQ_CTI        40
#define RP23XX_PROC1_IRQ_CTI        41
#define RP23XX_PLL_SYS_IRQ          42
#define RP23XX_PLL_USB_IRQ          43
#define RP23XX_POWMAN_IRQ_POW       44
#define RP23XX_POWMAN_IRQ_TIMER     45
#define RP23XX_SPARE_IRQ_0          46
#define RP23XX_SPARE_IRQ_1          47
#define RP23XX_SPARE_IRQ_2          48
#define RP23XX_SPARE_IRQ_3          49
#define RP23XX_SPARE_IRQ_4          50
#define RP23XX_SPARE_IRQ_5          51
#define RP23XX_IRQ_COUNT            52

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_INTCTRL_H */
