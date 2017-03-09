/****************************************************************************
 * arch/avr/include/debug.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_ARCH_AVR_DEBUG_H
#define __INCLUDE_ARCH_AVR_DEBUG_H

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

#endif /* __INCLUDE_ARCH_DEBUG_H */
