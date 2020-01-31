/****************************************************************************
 * arch/z80/src/common/z80_internal.h
 *
 *   Copyright (C) 2007-2009, 2015, 2017-2018 Gregory Nutt. All rights
 *     reserved.
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

#ifndef __ARCH_Z80_SRC_COMMON_Z80_INTERNAL_H
#define __ARCH_Z80_SRC_COMMON_Z80_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include "chip.h"
#include "switch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#if defined(CONFIG_DEV_LOWCONSOLE)
#  undef USE_SERIALDRIVER
#  ifdef CONFIG_HAVE_LOWSERIALINIT
#    define USE_LOWSERIALINIT 1
#  else
#    undef USE_LOWSERIALINIT
#  endif
#elif !defined(CONFIG_DEV_CONSOLE)
#  undef  USE_SERIALDRIVER
#  undef  USE_LOWSERIALINIT
#  undef  CONFIG_DEV_LOWCONSOLE
#  undef  CONFIG_RAMLOG_CONSOLE
#else
#  undef  USE_LOWSERIALINIT
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  CONFIG_DEV_LOWCONSOLE
#  elif defined(CONFIG_CONSOLE_SYSLOG)
#    undef  USE_SERIALDRIVER
#    undef  CONFIG_DEV_LOWCONSOLE
#  elif defined(CONFIG_DEV_LOWCONSOLE)
#    undef  USE_SERIALDRIVER
#  else
#    define USE_SERIALDRIVER 1
#  endif
#endif

/* If some other device is used as the console, then the serial driver may
 * still be needed.  Let's assume that if the upper half serial driver is
 * built, then the lower half will also be needed.  There is no need for
 * the early serial initialization in this case.
 */

#if !defined(USE_SERIALDRIVER) && defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/* Supplied by chip- or board-specific logic */

void z80_irq_initialize(void);

#ifdef CONFIG_RTC_ALARM
void z80_rtc_irqinitialize(void);
#endif

#ifdef USE_LOWSERIALINIT
void z80_lowserial_initialize(void);
#endif

/* Defined in xyz_doirq.c */

FAR chipreg_t *z80_doirq(uint8_t irq, FAR chipreg_t *regs);

/* Define in zyz_sigdeliver */

void z80_sigdeliver(void);

#ifdef CONFIG_ARCH_ADDRENV
/* Defined in CPU-specific logic (only for Z180) */

int z80_mmu_initialize(void);
#endif

/* Defined in xyz_serial.c */

#ifdef USE_SERIALDRIVER
void z80_serial_initialize(void);
#else
# define z80_serial_initialize()
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#else
#  define rpmsg_serialinit()
#endif

/* Defined in drivers/lowconsole.c */

#ifdef CONFIG_DEV_LOWCONSOLE
void lowconsole_init(void);
#else
# define lowconsole_init()
#endif

/* Defined in drivers/syslog_console.c */

#ifdef CONFIG_CONSOLE_SYSLOG
void syslog_console_init();
#else
# define syslog_console_init()
#endif

/* Defined in drivers/ramlog.c */

#ifdef CONFIG_RAMLOG_CONSOLE
void ramlog_consoleinit(void);
#else
# define ramlog_consoleinit()
#endif

/* Low level string output */

void up_puts(const char *str);

/* Defined in xyz_timerisr.c */

void z80_timer_initialize(void);

/* Architecture specific hook into the timer interrupt handler */

#ifdef CONFIG_ARCH_TIMERHOOK
void up_timerhook(void);
#endif

/* Defined in board/up_network.c */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
int  up_netinitialize(void);
void up_netuninitialize(void);
# ifdef CONFIG_ARCH_MCFILTER
int up_multicastfilter(FAR struct net_driver_s *dev, FAR uint8_t *mac, bool enable);
# else
#   define up_multicastfilter(dev, mac, enable)
# endif
#else
# define up_netinitialize()
# define up_netuninitialize()
# define up_multicastfilter(dev, mac, enable)
#endif

/* Return the current value of the stack pointer (used in stack dump logic) */

uintptr_t z80_getsp(void);

/* Dump stack and registers */

#ifdef CONFIG_ARCH_STACKDUMP
void up_stackdump(void);
# define REGISTER_DUMP() _REGISTER_DUMP()
#else
# define up_stackdump()
# define REGISTER_DUMP()
#endif

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z80_SRC_COMMON_Z80_INTERNAL_H */
