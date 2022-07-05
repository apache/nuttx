/****************************************************************************
 * arch/arm/src/stm32wl5/stm32wl5_ipcc.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_IPCC_H
#define __ARCH_ARM_SRC_STM32WL5_IPCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/ipcc.h>
#include "hardware/stm32wl5_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Channels are sequential, that is, if channel 3 is enabled that means
 * channel 2 and 1 are also enabled. Kconfig makes sure there is no
 * situation where channel 3 is enabled while channel 2 is not.
 */

/* channel 1 configuration **************************************************/

#define IPCC_CHAN1_RX_SIZE   (CONFIG_STM32WL5_IPCC_CHAN1_RX_SIZE)
#define IPCC_CHAN1_TX_SIZE   (CONFIG_STM32WL5_IPCC_CHAN1_TX_SIZE)
#define IPCC_CHAN1_START     (IPCC_START)
#define IPCC_CHAN1_SIZE      (IPCC_CHAN1_RX_SIZE + IPCC_CHAN1_TX_SIZE)
#define IPCC_CHAN1           (1)

/* channel 2 configuration **************************************************/

#if defined(CONFIG_STM32WL5_IPCC_CHAN2)
#  define IPCC_CHAN2_RX_SIZE (CONFIG_STM32WL5_IPCC_CHAN2_RX_SIZE)
#  define IPCC_CHAN2_TX_SIZE (CONFIG_STM32WL5_IPCC_CHAN2_TX_SIZE)
#  define IPCC_CHAN2_START   (IPCC_CHAN1_START + IPCC_CHAN1_SIZE)
#  define IPCC_CHAN2_SIZE    (IPCC_CHAN2_RX_SIZE + IPCC_CHAN2_TX_SIZE)
#  define IPCC_CHAN2         (1)
#else
#  define IPCC_CHAN2_SIZE    (0)
#  define IPCC_CHAN2         (0)
#endif

/* channel 3 configuration **************************************************/

#if defined(CONFIG_STM32WL5_IPCC_CHAN3)
#  define IPCC_CHAN3_RX_SIZE (CONFIG_STM32WL5_IPCC_CHAN3_RX_SIZE)
#  define IPCC_CHAN3_TX_SIZE (CONFIG_STM32WL5_IPCC_CHAN3_TX_SIZE)
#  define IPCC_CHAN3_START   (IPCC_CHAN2_START + IPCC_CHAN2_SIZE)
#  define IPCC_CHAN3_SIZE    (IPCC_CHAN3_RX_SIZE + IPCC_CHAN3_TX_SIZE)
#  define IPCC_CHAN3         (1)
#else
#  define IPCC_CHAN3_SIZE    (0)
#  define IPCC_CHAN3         (0)
#endif

/* channel 4 configuration **************************************************/

#if defined(CONFIG_STM32WL5_IPCC_CHAN4)
#  define IPCC_CHAN4_RX_SIZE (CONFIG_STM32WL5_IPCC_CHAN4_RX_SIZE)
#  define IPCC_CHAN4_TX_SIZE (CONFIG_STM32WL5_IPCC_CHAN4_TX_SIZE)
#  define IPCC_CHAN4_START   (IPCC_CHAN3_START + IPCC_CHAN3_SIZE)
#  define IPCC_CHAN4_SIZE    (IPCC_CHAN4_RX_SIZE + IPCC_CHAN4_TX_SIZE)
#  define IPCC_CHAN4         (1)
#else
#  define IPCC_CHAN4_SIZE    (0)
#  define IPCC_CHAN4         (0)
#endif

/* channel 5 configuration **************************************************/

#if defined(CONFIG_STM32WL5_IPCC_CHAN5)
#  define IPCC_CHAN5_RX_SIZE (CONFIG_STM32WL5_IPCC_CHAN5_RX_SIZE)
#  define IPCC_CHAN5_TX_SIZE (CONFIG_STM32WL5_IPCC_CHAN5_TX_SIZE)
#  define IPCC_CHAN5_START   (IPCC_CHAN4_START + IPCC_CHAN4_SIZE)
#  define IPCC_CHAN5_SIZE    (IPCC_CHAN5_RX_SIZE + IPCC_CHAN5_TX_SIZE)
#  define IPCC_CHAN5         (1)
#else
#  define IPCC_CHAN5_SIZE    (0)
#  define IPCC_CHAN5         (0)
#endif

/* channel 6 configuration **************************************************/

#if defined(CONFIG_STM32WL5_IPCC_CHAN6)
#  define IPCC_CHAN6_RX_SIZE (CONFIG_STM32WL5_IPCC_CHAN6_RX_SIZE)
#  define IPCC_CHAN6_TX_SIZE (CONFIG_STM32WL5_IPCC_CHAN6_TX_SIZE)
#  define IPCC_CHAN6_START   (IPCC_CHAN5_START + IPCC_CHAN5_SIZE)
#  define IPCC_CHAN6_SIZE    (IPCC_CHAN6_RX_SIZE + IPCC_CHAN6_TX_SIZE)
#  define IPCC_CHAN6         (1)
#else
#  define IPCC_CHAN6_SIZE    (0)
#  define IPCC_CHAN6         (0)
#endif

/* ipcc general configuration ***********************************************/

/* Memory layout is:
 *
 *  +----------+
 *  | CHAN1_RX | CHAN1_START (IPCC_START)
 *  | CHAN1_TX | CHAN1_START + IPCC_CHAN1_RX_SIZE
 *  +----------+
 *  | CHAN2_RX | CHAN2_START
 *  | CHAN2_TX | CHAN2_START + IPCC_CHAN2_RX_SIZE
 *  |    .     |
 *  |    .     |
 *  |    .     |
 *  | CHAN6_RX | CHAN6_START
 *  | CHAN6_TX | CHAN6_START + IPCC_CHAN6_RX_SIZE
 *  +----------+
 */

/* IPCC needs continous memory of known address that is shared
 * between CPUs. Because of that we reserve memory at beginning
 * of SRAM2. SRAM2 region will be right after IPCC reserved memory
 */

#define IPCC_START      STM32WL5_SRAM2_BASE
#define IPCC_NCHAN     (IPCC_CHAN1 + IPCC_CHAN2 + IPCC_CHAN3 + \
                        IPCC_CHAN4 + IPCC_CHAN5 + IPCC_CHAN6)
#define IPCC_END       (IPCC_START + IPCC_CHAN1_SIZE + IPCC_CHAN2_SIZE + \
                        IPCC_CHAN3_SIZE + IPCC_CHAN4_SIZE + \
                        IPCC_CHAN5_SIZE + IPCC_CHAN6_SIZE)

struct ipcc_lower_s *stm32wl5_ipcc_init(int chan);

#endif /* __ARCH_ARM_SRC_STM32WL5_IPCC_H */
