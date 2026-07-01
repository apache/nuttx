/****************************************************************************
 * arch/arm/src/common/ameba/sdk_shim/os_wrapper_specific.h
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

/****************************************************************************
 * NuttX shim for the SDK's os_wrapper_specific.h.
 *
 * The stock SDK header (component/os/freertos/os_wrapper/include/) pulls in
 * FreeRTOS.h, which does not exist in the NuttX port (rtos_* are backed by
 * ameba_os_wrap.c).  Its only payload is the RTOS_CONVERT_MS_TO_TICKS
 * helper, reproduced here on NuttX's tick base.  This shim is placed ahead
 * of the SDK include path for the fwlib compile so SDK sources (e.g.
 * ameba_flash_ram.c)
 * resolve it without dragging in FreeRTOS.
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_SDK_SHIM_OS_WRAPPER_SPECIFIC_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_SDK_SHIM_OS_WRAPPER_SPECIFIC_H

/* This header is compiled with the scoped SDK fwlib include set (no NuttX
 * core headers), so the conversion is expressed self-contained.  The NuttX
 * port runs at CONFIG_USEC_PER_TICK = 1000 (1 ms / tick), so milliseconds
 * map 1:1 to ticks; 0xFFFFFFFF keeps its "wait forever" sentinel.
 */

#ifndef RTOS_CONVERT_MS_TO_TICKS
#  define RTOS_CONVERT_MS_TO_TICKS(MS) \
     (((MS) == 0xFFFFFFFFUL) ? 0xFFFFFFFFUL : (unsigned long)(MS))
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_SDK_SHIM_OS_WRAPPER_SPECIFIC_H */
