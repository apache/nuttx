/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_clockconfig.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_CLOCKCONFIG_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "esp32c3_attr.h"

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
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
 * Name: esp32c3_clockconfig
 ****************************************************************************/

void esp32c3_clockconfig(void);

/****************************************************************************
 * Name: esp32c3_clk_cpu_freq
 *
 * Description:
 *   Returns CPU frequency in Hz.
 *
 ****************************************************************************/

int esp32c3_clk_cpu_freq(void);

/****************************************************************************
 * Name: esp32c3_clk_apb_freq
 *
 * Description:
 *   Returns ABP frequency in Hertz.
 *
 ****************************************************************************/

int esp32c3_clk_apb_freq(void);

/****************************************************************************
 * Name: esp32c3_clk_crypto_freq
 *
 * Description:
 *   Returns crypto engine frequency in Hertz.
 *
 ****************************************************************************/

int esp32c3_clk_crypto_freq(void);

/****************************************************************************
 * Name: esp32c3_cpu_cycle_count
 *
 * Description:
 *   Get the current value of the internal counter that increments
 *   every processor-clock cycle.
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp32c3_cpu_cycle_count(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_CLOCKCONFIG_H */
