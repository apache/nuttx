/****************************************************************************
 * arch/xtensa/src/esp32/chip.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_CHIP_H
#define __ARCH_XTENSA_SRC_ESP32_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip_macros.h"
#include "chip_memory.h"

#if defined(CONFIG_ESP32_OPENETH) && !defined(__ASSEMBLY__)
#include "hardware/esp32_soc.h"
#include "esp32_irq.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ESP32_OPENETH)
#define OPENETH_PERIPH_MAC   ESP32_PERIPH_EMAC
#define OPENETH_CPUINT_LEVEL ESP32_CPUINT_LEVEL
#define OPENETH_IRQ_MAC      ESP32_IRQ_EMAC
#define OPENETH_SETUP_IRQ    esp32_setup_irq
#define RX_BUF_COUNT CONFIG_ESP32_OPENETH_DMA_RX_BUFFER_NUM
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_CHIP_H */
