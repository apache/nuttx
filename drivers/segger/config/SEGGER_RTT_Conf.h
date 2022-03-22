/****************************************************************************
 * drivers/segger/config/SEGGER_RTT_Conf.h
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

#ifndef __DRIVERS_SEGGER_CONFIG_SEGGER_RTT_CONF_H
#define __DRIVERS_SEGGER_CONFIG_SEGGER_RTT_CONF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/spinlock.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Take in and set to correct values for Cortex systems with CPU cache */

/* Largest cache line size (in bytes) in the current system */

#define SEGGER_RTT_CPU_CACHE_LINE_SIZE  CONFIG_SEGGER_RTT_CPU_CACHE_LINE_SIZE

/* Address alias where RTT CB and buffers can be accessed uncached */

#define SEGGER_RTT_UNCACHED_OFF         CONFIG_SEGGER_RTT_UNCACHED_OFF

/* Number of up-buffers (T->H) available on this target */

#define SEGGER_RTT_MAX_NUM_UP_BUFFERS   CONFIG_SEGGER_RTT_MAX_NUM_UP_BUFFERS

/* Number of down-buffers (H->T) available on this target */

#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS CONFIG_SEGGER_RTT_MAX_NUM_DOWN_BUFFERS

/* Size of the buffer for terminal output of target, up to host */

#define BUFFER_SIZE_UP                  CONFIG_SEGGER_RTT_BUFFER_SIZE_UP

/* Size of the buffer for terminal input to target from host */

#define BUFFER_SIZE_DOWN                CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN

/* Mode for pre-initialized terminal channel */

#define SEGGER_RTT_MODE_DEFAULT         SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL

/* 0: Use memcpy/SEGGER_RTT_MEMCPY, 1: Use a simple byte-loop */

#define SEGGER_RTT_MEMCPY_USE_BYTELOOP  0

/* RTT lock configuration */

/* Lock RTT (nestable)   (i.e. disable interrupts) */

#define SEGGER_RTT_LOCK()               irqstate_t __flags = spin_lock_irqsave(NULL)

/* Unlock RTT (nestable) (i.e. enable previous interrupt lock state) */

#define SEGGER_RTT_UNLOCK()             spin_unlock_irqrestore(NULL, __flags)

/* Disable RTT SEGGER_RTT_WriteSkipNoLock */

#define RTT_USE_ASM                     0

#endif /* __DRIVERS_SEGGER_CONFIG_SEGGER_RTT_CONF_H */
