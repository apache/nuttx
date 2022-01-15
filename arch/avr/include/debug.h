/****************************************************************************
 * arch/avr/include/debug.h
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

#ifndef __ARCH_AVR_INCLUDE_DEBUG_H
#define __ARCH_AVR_INCLUDE_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <syslog.h>

#ifdef CONFIG_AVR_HAS_MEMX_PTR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Since format string that is passed to __arch_syslog() can be a
 * comma-separated list, we need some cpp trickery to handle it
 *
 * __dbg_first() helper macro accepts the format (that is potentially
 * a comma-separated) and substitutes to its first element.
 *
 * __dbg_subst() helper macro substitutes first element in the format
 * with prefix.
 */

#define __dbg_first(format, ...) format
#define __dbg_subst(prefix, format, ...) prefix, ##__VA_ARGS__

#define __dbg_expand(logger, prio, format, ...) \
  do \
    { \
     static const IOBJ char dbg_s[] = __dbg_first(format); \
     logger(prio, __dbg_subst(dbg_s, format), ##__VA_ARGS__); \
    } \
  while(0)

/* __arch_syslog() overrides the behavior of NuttX debug macros. They put
 * the format string into program memory and utilize IPTR (__memx) parameter
 * of syslog to take the format directly from program memory.  This reduces
 * amount of RAM held by the format strings used in debug statements.
 */

#define __arch_syslog(...) \
   __dbg_expand(syslog, ##__VA_ARGS__)

#endif /* CONFIG_AVR_HAS_MEMX_PTR */

#endif /* __ARCH_AVR_INCLUDE_DEBUG_H */
