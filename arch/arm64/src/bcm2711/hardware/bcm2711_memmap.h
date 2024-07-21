/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_memmap.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_MM_H
#define __ARCH_ARM64_SRC_BCM2711_MM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BCM2711 peripheral base address */

#if defined(CONFIG_BCM2711_LOW_PERIPHERAL)

/* Low peripheral addressing mode (default for RPi 4B boot) */

#define BCM_PERIPHERAL_BASEADDR 0x0fe000000

#elif defined(CONFIG_BCM2711_LEGACY_ADDR)

/* Legacy addressing mode */

#define BCM_PERIPHERAL_BASEADDR 0x07e000000

#else

/* 35-bit addressing mode */

#define BCM_PERIPHERAL_BASEADDR 0x47e000000

#endif // defined(CONFIG_BCM2711_LOW_PERIPHERAL)

/* Base addresses for chip registers */

#define BCM_ARMT_BASEADDR                                                    \
  (BCM_PERIPHERAL_BASEADDR + 0x0000b000) /* ARM timer */
#define BCM_AUX_BASEADDR                                                     \
  (BCM_PERIPHERAL_BASEADDR + 0x000215000) /* Auxilliary */
#define BCM_GPCLK_BASEADDR                                                   \
  (BCM_PERIPHERAL_BASEADDR + 0x000101000) /* General purpose clock */
#define BCM_GPIO_BASEADDR                                                    \
  (BCM_PERIPHERAL_BASEADDR + 0x000200000) /* GPIO */
#define BCM_PCM_BASEADDR                                                     \
  (BCM_PERIPHERAL_BASEADDR + 0x000203000) /* PCM */
#define BCM_SYST_BASEADDR                                                    \
  (BCM_PERIPHERAL_BASEADDR + 0x000003000) /* System timer */

/* SPI interface register base addresses */

#define BCM_SPI0_BASEADDR                                                    \
  (BCM_PERIPHERAL_BASEADDR + 0x000204000) /* SPI interface 0 */
#define BCM_SPI3_BASEADDR                                                    \
  (BCM_PERIPHERAL_BASEADDR + 0x000204600) /* SPI interface 3 */
#define BCM_SPI4_BASEADDR                                                    \
  (BCM_PERIPHERAL_BASEADDR + 0x000204800) /* SPI interface 4 */
#define BCM_SPI5_BASEADDR                                                    \
  (BCM_PERIPHERAL_BASEADDR + 0x000204a00) /* SPI interface 5 */
#define BCM_SPI6_BASEADDR                                                    \
  (BCM_PERIPHERAL_BASEADDR + 0x000204c00) /* SPI interface 6 */

/* UART interface base addresses */

#define BCM_UART0_BASEADDR                                                   \
  (BCM_PERIPHERAL_BASEADDR + 0x000201000) /* UART interface 0 */
#define BCM_UART2_BASEADDR                                                   \
  (BCM_PERIPHERAL_BASEADDR + 0x000201400) /* UART interface 2 */
#define BCM_UART3_BASEADDR                                                   \
  (BCM_PERIPHERAL_BASEADDR + 0x000201600) /* UART interface 3 */
#define BCM_UART4_BASEADDR                                                   \
  (BCM_PERIPHERAL_BASEADDR + 0x000201800) /* UART interface 4 */
#define BCM_UART5_BASEADDR                                                   \
  (BCM_PERIPHERAL_BASEADDR + 0x000201a00) /* UART interface 5 */

/* DMA channel base addresses */

#define BCM_DMA0_BASE                                                        \
  (BCM_PERIPHERAL_BASEADDR + 0x000007000) /* DMA Channel 0 */
#define BCM_DMA15_BASE                                                       \
  (BCM_PERIPHERAL_BASEADDR + 0x000e05000) /* DMA Channel 15 */

/* ARM_LOCAL base address */

#if defined(CONFIG_BCM2711_LOW_PERIPHERAL)
#define BCM_ARMLOCAL_BASEADDR 0xff800000
#else
#define BCM_ARMLOCAL_BASEADDR 0x4c0000000
#endif /* defined(CONFIG_BCM2711_LOW_PERIPHERAL) */

#endif /* __ARCH_ARM64_SRC_BCM2711_MM_H */
