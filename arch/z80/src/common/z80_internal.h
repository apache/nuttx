/****************************************************************************
 * arch/z80/src/common/z80_internal.h
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

#if !defined(CONFIG_DEV_CONSOLE)
#  undef  USE_SERIALDRIVER
#else
#  if defined(CONFIG_CONSOLE_SYSLOG)
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

/* The Z80 stack does not need to be aligned.  Here is is aligned at word
 * (4 byte) boundary.
 */

#define STACK_ALIGNMENT     4

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* Register access macros ***************************************************
 *
 * The register access mechanism provided in ez8.h differs from the useful in
 * other NuttX architectures. The following NuttX common macros will at least
 * make the access compatible at the source level (however, strict type check
 * is lost).
 */

#define getreg8(a)          (a)
#define putreg8(v,a)        ((a) = (v))
#define getreg16(a)         (a)
#define putreg16(v,a)       ((a) = (v))
#define getreg32(a)         (a)
#define putreg32(v,a)       ((a) = (v))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
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
#  define z80_serial_initialize()
#endif

/* Architecture specific hook into the timer interrupt handler */

#ifdef CONFIG_ARCH_TIMERHOOK
void up_timerhook(void);
#endif

/* Defined in board/up_network.c */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
int  up_netinitialize(void);
void up_netuninitialize(void);
# ifdef CONFIG_ARCH_MCFILTER
int up_multicastfilter(FAR struct net_driver_s *dev, FAR uint8_t *mac,
                       bool enable);
# else
#   define up_multicastfilter(dev, mac, enable)
# endif
#else
# define up_netinitialize()
# define up_netuninitialize()
# define up_multicastfilter(dev, mac, enable)
#endif

/* Dump stack and registers */

#ifdef CONFIG_ARCH_STACKDUMP
void z80_stackdump(void);
# define Z80_REGISTER_DUMP() _REGISTER_DUMP()
#else
# define z80_stackdump()
# define Z80_REGISTER_DUMP()
#endif

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z80_SRC_COMMON_Z80_INTERNAL_H */
