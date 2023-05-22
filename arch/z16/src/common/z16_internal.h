/****************************************************************************
 * arch/z16/src/common/z16_internal.h
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

#ifndef __ARCH_Z16_SRC_COMMON_Z16_INTERNAL_H
#define __ARCH_Z16_SRC_COMMON_Z16_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bring-up debug configurations.  These are here (vs defconfig)
 * because these should only be controlled during low level
 * board bring-up and not part of normal platform configuration.
 */

#undef  CONFIG_Z16_LOWPUTC            /* Support z16_lowputc for debug */
#undef  CONFIG_Z16_LOWGETC            /* support z16_lowgetc for debug */

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#if defined(CONFIG_Z16_LOWPUTC) || defined(CONFIG_Z16_LOWGETC)
#  define USE_LOWUARTINIT 1
#elif !defined(CONFIG_DEV_CONSOLE)
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#else
#  if defined(CONFIG_CONSOLE_SYSLOG)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  else
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
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

/* Align the stack to word (4 byte) boundaries.  This is probablya greater
 * alignment than is required.
 */

#define STACK_ALIGNMENT     4

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* Macros for portability */

#define IN_INTERRUPT             (g_current_regs != NULL)
#define SAVE_IRQCONTEXT(tcb)     z16_copystate((tcb)->xcp.regs, (FAR chipreg_t*)g_current_regs)
#define SET_IRQCONTEXT(tcb)      do { g_current_regs = (tcb)->xcp.regs; } while (0)
#define SAVE_USERCONTEXT(tcb)    up_saveusercontext((tcb)->xcp.regs)
#define RESTORE_USERCONTEXT(tcb) z16_restoreusercontext((tcb)->xcp.regs)
#define SIGNAL_RETURN(regs)      z16_restoreusercontext(regs)

/* Register access macros ***************************************************/

#define getreg8(a)              (*(uint8_t volatile _Near*)(a))
#define putreg8(v,a)            (*(uint8_t volatile _Near*)(a) = (v))
#define getreg16(a)             (*(uint16_t volatile _Near*)(a))
#define putreg16(v,a)           (*(uint16_t volatile _Near*)(a) = (v))
#define getreg32(a)             (*(uint32_t volatile _Near*)(a))
#define putreg32(v,a)           (*(uint32_t volatile _Near*)(a) = (v))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Defined in files with the same name as the function */

void z16_copystate(FAR chipreg_t *dest, FAR chipreg_t *src);
FAR chipreg_t *z16_doirq(int irq, FAR chipreg_t *regs);
void z16_restoreusercontext(FAR chipreg_t *regs);
void z16_sigdeliver(void);

#if defined(CONFIG_Z16_LOWPUTC) || defined(CONFIG_Z16_LOWGETC)
void z16_lowputc(char ch);
#else
#  define z16_lowputc(ch)
#endif

/* Defined in xyz_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void z16_addregion(void);
#endif

/* Defined in xyz_serial.c */

#ifdef USE_SERIALDRIVER
void z16_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void z16_earlyserialinit(void);
#endif

/* Defined in xyz_irq.c */

void z16_ack_irq(int irq);

/* Defined in board/xyz_network.c */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void z16_netinitialize(void);
#else
#  define z16_netinitialize()
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_Z16_SRC_COMMON_Z16_INTERNAL_H */
