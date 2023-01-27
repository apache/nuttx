/****************************************************************************
 * arch/arm/include/rp2040/watchdog.h
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

#ifndef __ARCH_ARM_INCLUDE_RP2040_WATCHDOG_H
#define __ARCH_ARM_INCLUDE_RP2040_WATCHDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/watchdog.h>

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_WATCHDOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* The watchdog driver uses a standard character driver framework.  However,
 * since the watchdog driver is a device control interface and not a data
 * transfer interface, the majority of the functionality is implemented in
 * driver ioctl calls.
 *
 * See nuttx/timers/watchdog.h for the IOCTLs handled by the upper half.
 *
 * These are detected and handled by the "lower half" watchdog timer driver.
 *
 *  WDIOC_SET_SCRATCHn  - save a 32-bit "arg" value in a scratch register
 *                        that will be preserved over soft resets. A hard
 *                        reset sets all scratch values to zero.
 *
 *  WDIOC_GET_SCRATCHn  - fetch a 32-bit value from a scratch register
 *                        into a uint32_t pointed to by "arg".
 */

#define WDIOC_SET_SCRATCH0   _WDIOC(0x180)
#define WDIOC_SET_SCRATCH1   _WDIOC(0x181)
#define WDIOC_SET_SCRATCH2   _WDIOC(0x182)
#define WDIOC_SET_SCRATCH3   _WDIOC(0x183)
#define WDIOC_SET_SCRATCH4   _WDIOC(0x184)
#define WDIOC_SET_SCRATCH5   _WDIOC(0x185)
#define WDIOC_SET_SCRATCH6   _WDIOC(0x186)
#define WDIOC_SET_SCRATCH7   _WDIOC(0x187)

#define WDIOC_SET_SCRATCH(n) _WDIOC(0x180 + (n))

#define WDIOC_GET_SCRATCH0   _WDIOC(0x1f0)
#define WDIOC_GET_SCRATCH1   _WDIOC(0x1f1)
#define WDIOC_GET_SCRATCH2   _WDIOC(0x1f2)
#define WDIOC_GET_SCRATCH3   _WDIOC(0x1f3)
#define WDIOC_GET_SCRATCH4   _WDIOC(0x1f4)
#define WDIOC_GET_SCRATCH5   _WDIOC(0x1f5)
#define WDIOC_GET_SCRATCH6   _WDIOC(0x1f6)
#define WDIOC_GET_SCRATCH7   _WDIOC(0x1f7)

#define WDIOC_GET_SCRATCH(n) _WDIOC(0x1f0 + (n))

#endif /* CONFIG_WATCHDOG */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_RP2040_WATCHDOG_H */
