/****************************************************************************
 * arch/avr/src/common/avr_internal.h
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

#ifndef __ARCH_AVR_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_AVR_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <nuttx/arch.h>
#  include <nuttx/irq.h>
#endif

#ifdef CONFIG_ARCH_FAMILY_AVR32
#  include "avr32.h"
#else
#  include "avr.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if an interrupt stack size is configured */

#ifndef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR    's'
#define INTSTACK_COLOR 's'
#define HEAP_COLOR     'h'

#define getreg8(a)     (*(volatile uint8_t *)(a))
#define putreg8(v,a)   (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)    (*(volatile uint16_t *)(a))
#define putreg16(v,a)  (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)    (*(volatile uint32_t *)(a))
#define putreg32(v,a)  (*(volatile uint32_t *)(a) = (v))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
extern uint8_t g_intstackalloc[];
extern uint8_t g_intstacktop[];
#endif

/* These symbols are setup by the linker script. */

extern uint8_t _stext[];           /* Start of .text */
extern uint8_t _etext[];           /* End_1 of .text + .rodata */
extern const uint8_t _eronly[];    /* End+1 of read only section (.text + .rodata) */
extern uint8_t _sdata[];           /* Start of .data */
extern uint8_t _edata[];           /* End+1 of .data */
extern uint8_t _sbss[];            /* Start of .bss */
extern uint8_t _ebss[];            /* End+1 of .bss */

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/* Defined in files with the same name as the function */

#ifdef CONFIG_ARCH_DMA
void weak_function avr_dma_initialize(void);
#endif
void avr_sigdeliver(void);
void avr_lowputc(char ch);
void avr_lowputs(const char *str);

/* Defined in common/avr_allocateheap.c or chip/xxx_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void avr_addregion(void);
#else
#  define avr_addregion()
#endif

/* Defined in chip/xxx_lowinit.c.  This function is called from the
 * head.S file just before jumping to nx_start().  This function
 * performs whatever very low level initialization that is needed
 * before the OS gets started (clocks, console, LEDs, etc.)
 */

void avr_lowinit(void);

/* Defined in chip/xxx_serial.c */

#ifdef CONFIG_DEV_CONSOLE
void avr_earlyserialinit(void);
void avr_serialinit(void);
#endif

/* Defined in chip/xxx_ethernet.c */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void avr_netinitialize(void);
#else
#  define avr_netinitialize()
#endif

/* Defined in chip/xxx_usbdev.c */

#ifdef CONFIG_USBDEV
void avr_usbinitialize(void);
void avr_usbuninitialize(void);
#else
#  define avr_usbinitialize()
#  define avr_usbuninitialize()
#endif

#ifdef CONFIG_STACK_COLORATION
size_t avr_stack_check(uintptr_t alloc, size_t size);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_COMMON_UP_INTERNAL_H */
