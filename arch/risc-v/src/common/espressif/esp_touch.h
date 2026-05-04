/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_touch.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TOUCH_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TOUCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdbool.h>
#include <sys/types.h>

#ifndef __ASSEMBLY__

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
 * Name: esp_configtouch
 *
 * Description:
 *   Configures touch pad channels.
 *
 * Input Parameters:
 *   button_num - Address to return number of buttons initialized
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_configtouch(int *button_num);

/****************************************************************************
 * Name: esp_touchread
 *
 * Description:
 *   Read a touch pad channel.
 *
 * Input Parameters:
 *   channel - The touch pad channel.
 *
 * Returned Value:
 *   0 if touch pad pressed, 1 if released.
 *
 ****************************************************************************/

bool esp_touchread(int channel);

/****************************************************************************
 * Name: esp_touchirqattach
 *
 * Description:
 *   Attach an interrupt handler to specified channel.
 *
 * Input Parameters:
 *   channel - Touch pad channel number
 *   handler - Interrupt handler function
 *   arg     - Argument to pass to the handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_TOUCH_IRQ
int esp_touchirqattach(int channel, xcpt_t handler, void *arg);
#else
#  define esp_touchirqattach(channel, handler, arg) (-EINVAL)
#endif

/****************************************************************************
 * Name: esp_touchirqdetach
 *
 * Description:
 *   Detach interrupt handler from specified channel.
 *
 * Input Parameters:
 *   channel - Touch pad channel number
 *
 * Returned Value:
 *   Zero (OK) is returned on success; -1 (ERROR) in failure
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_TOUCH_IRQ
int esp_touchirqdetach(int channel);
#else
#  define esp_touchirqdetach(channel) (-EINVAL)
#endif

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TOUCH_H */
