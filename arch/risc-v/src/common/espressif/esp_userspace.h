/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_userspace.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_USERSPACE_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_USERSPACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: esp_userspace
 *
 * Description:
 *   For the case of the separate user/kernel space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the userspace .data and .bss
 *   segments, but on this SoC it also means mapping the user image out of
 *   external flash and establishing the PMP regions that separate the two
 *   worlds.
 *
 *   Must be called from esp_start() immediately before nx_start().
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_userspace(void);

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_BUILD_PROTECTED */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_USERSPACE_H */
