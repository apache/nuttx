/****************************************************************************
 * arch/x86/src/i486/up_irq.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/io.h>

#include "up_internal.h"
#include "qemu.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void idt_outb(uint8_t val, uint16_t addr) noinline_function;
static void up_remappic(void);
static void up_idtentry(unsigned int index, uint32_t base, uint16_t sel,
                        uint8_t flags);
static inline void up_idtinit(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct idt_entry_s idt_entries[256];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: idt_outb
 *
 * Description:
 *   A slightly slower version of outb
 *
 ****************************************************************************/

static void idt_outb(uint8_t val, uint16_t addr)
{
  outb(val, addr);
}

/****************************************************************************
 * Name: up_remappic
 *
 * Description:
 *   Remap the PIC.  The Programmable Interrupt Controller (PIC) is used to
 *   combine several sources of interrupt onto one or more CPU lines, while
 *   allowing priority levels to be assigned to its interrupt outputs. When
 *   the device has multiple interrupt outputs to assert, it will assert them
 *   in the order of their relative priority.
 *
 ****************************************************************************/

static void up_remappic(void)
{
  /* Mask interrupts from PIC */

  idt_outb(PIC1_IMR_ALL, PIC1_IMR);
  idt_outb(PIC2_IMR_ALL, PIC2_IMR);

  /* If the PIC has been reset, it must be initialized with 2 to 4
   * Initialization Command Words (ICW) before it will accept and process
   * Interrupt Requests. The following outlines the four possible
   * Initialization Command Words.
   */

  /* Remap the irq table for primary:
   *
   * ICW1 - We will be sending ICW4
   * ICW2 - Address
   * ICW3
   */

  idt_outb(PIC_ICW1_ICW4 | PIC_ICW1_ICW1, PIC1_ICW1);
  idt_outb(0x20,                        PIC1_ICW2);
  idt_outb(PIC1_ICW3_IRQ2,              PIC1_ICW3);
  idt_outb(PIC_ICW4_808XMODE,           PIC1_ICW4);

  /* Remap irq for slave */

  idt_outb(PIC_ICW1_ICW4 | PIC_ICW1_ICW1, PIC2_ICW1);
  idt_outb(0x28,                        PIC2_ICW2);
  idt_outb(PIC_ICW3_SID2,               PIC2_ICW3);
  idt_outb(PIC_ICW4_808XMODE,           PIC2_ICW4);

  /* Mask interrupts from PIC */

  idt_outb(PIC1_IMR_ALL, PIC1_IMR);
  idt_outb(PIC2_IMR_ALL, PIC2_IMR);
}

/****************************************************************************
 * Name: up_idtentry
 *
 * Description:
 *   Initialize one IDT entry.
 *
 ****************************************************************************/

static void up_idtentry(unsigned int index, uint32_t base, uint16_t sel,
                       uint8_t flags)
{
  struct idt_entry_s *entry = &idt_entries[index];

  entry->lobase = base & 0xffff;
  entry->hibase = (base >> 16) & 0xffff;

  entry->sel    = sel;
  entry->zero   = 0;

  /* We must uncomment the OR below when we get to using user-mode. It sets
   * the interrupt gate's privilege level to 3.
   */

  entry->flags  = flags /* | 0x60 */;
}

/****************************************************************************
 * Name: up_idtinit
 *
 * Description:
 *   Initialize the IDT. The Interrupt Descriptor Table (IDT) is a data
 *   structure used by the x86 architecture to implement an interrupt vector
 *   table. The IDT is used by the processor to determine the correct
 *   response to interrupts and exceptions.
 *
 ****************************************************************************/

static inline void up_idtinit(void)
{
  struct idt_ptr_s idt_ptr;

  idt_ptr.limit = sizeof(struct idt_entry_s) * 256 - 1;
  idt_ptr.base  = (uint32_t)&idt_entries;

  memset(&idt_entries, 0, sizeof(struct idt_entry_s)*256);

  /* Re-map the PIC */

  up_remappic();

  /* Set each ISR/IRQ to the appropriate vector with selector=8 and with
   * 32-bit interrupt gate.  Interrupt gate (vs. trap gate) will leave
   * interrupts enabled when the IRS/IRQ handler is entered.
   */

  up_idtentry(ISR0,  (uint32_t)vector_isr0 , 0x08, 0x8e);
  up_idtentry(ISR1,  (uint32_t)vector_isr1 , 0x08, 0x8e);
  up_idtentry(ISR2,  (uint32_t)vector_isr2 , 0x08, 0x8e);
  up_idtentry(ISR3,  (uint32_t)vector_isr3 , 0x08, 0x8e);
  up_idtentry(ISR4,  (uint32_t)vector_isr4 , 0x08, 0x8e);
  up_idtentry(ISR5,  (uint32_t)vector_isr5 , 0x08, 0x8e);
  up_idtentry(ISR6,  (uint32_t)vector_isr6 , 0x08, 0x8e);
  up_idtentry(ISR7,  (uint32_t)vector_isr7 , 0x08, 0x8e);
  up_idtentry(ISR8,  (uint32_t)vector_isr8 , 0x08, 0x8e);
  up_idtentry(ISR9,  (uint32_t)vector_isr9 , 0x08, 0x8e);
  up_idtentry(ISR10, (uint32_t)vector_isr10, 0x08, 0x8e);
  up_idtentry(ISR11, (uint32_t)vector_isr11, 0x08, 0x8e);
  up_idtentry(ISR12, (uint32_t)vector_isr12, 0x08, 0x8e);
  up_idtentry(ISR13, (uint32_t)vector_isr13, 0x08, 0x8e);
  up_idtentry(ISR14, (uint32_t)vector_isr14, 0x08, 0x8e);
  up_idtentry(ISR15, (uint32_t)vector_isr15, 0x08, 0x8e);
  up_idtentry(ISR16, (uint32_t)vector_isr16, 0x08, 0x8e);
  up_idtentry(ISR17, (uint32_t)vector_isr17, 0x08, 0x8e);
  up_idtentry(ISR18, (uint32_t)vector_isr18, 0x08, 0x8e);
  up_idtentry(ISR19, (uint32_t)vector_isr19, 0x08, 0x8e);
  up_idtentry(ISR20, (uint32_t)vector_isr20, 0x08, 0x8e);
  up_idtentry(ISR21, (uint32_t)vector_isr21, 0x08, 0x8e);
  up_idtentry(ISR22, (uint32_t)vector_isr22, 0x08, 0x8e);
  up_idtentry(ISR23, (uint32_t)vector_isr23, 0x08, 0x8e);
  up_idtentry(ISR24, (uint32_t)vector_isr24, 0x08, 0x8e);
  up_idtentry(ISR25, (uint32_t)vector_isr25, 0x08, 0x8e);
  up_idtentry(ISR26, (uint32_t)vector_isr26, 0x08, 0x8e);
  up_idtentry(ISR27, (uint32_t)vector_isr27, 0x08, 0x8e);
  up_idtentry(ISR28, (uint32_t)vector_isr28, 0x08, 0x8e);
  up_idtentry(ISR29, (uint32_t)vector_isr29, 0x08, 0x8e);
  up_idtentry(ISR30, (uint32_t)vector_isr30, 0x08, 0x8e);
  up_idtentry(ISR31, (uint32_t)vector_isr31, 0x08, 0x8e);

  up_idtentry(IRQ0,  (uint32_t)vector_irq0,  0x08, 0x8e);
  up_idtentry(IRQ1,  (uint32_t)vector_irq1,  0x08, 0x8e);
  up_idtentry(IRQ2,  (uint32_t)vector_irq2,  0x08, 0x8e);
  up_idtentry(IRQ3,  (uint32_t)vector_irq3,  0x08, 0x8e);
  up_idtentry(IRQ4,  (uint32_t)vector_irq4,  0x08, 0x8e);
  up_idtentry(IRQ5,  (uint32_t)vector_irq5,  0x08, 0x8e);
  up_idtentry(IRQ6,  (uint32_t)vector_irq6,  0x08, 0x8e);
  up_idtentry(IRQ7,  (uint32_t)vector_irq7,  0x08, 0x8e);
  up_idtentry(IRQ8,  (uint32_t)vector_irq8,  0x08, 0x8e);
  up_idtentry(IRQ9,  (uint32_t)vector_irq9,  0x08, 0x8e);
  up_idtentry(IRQ10, (uint32_t)vector_irq10, 0x08, 0x8e);
  up_idtentry(IRQ11, (uint32_t)vector_irq11, 0x08, 0x8e);
  up_idtentry(IRQ12, (uint32_t)vector_irq12, 0x08, 0x8e);
  up_idtentry(IRQ13, (uint32_t)vector_irq13, 0x08, 0x8e);
  up_idtentry(IRQ14, (uint32_t)vector_irq14, 0x08, 0x8e);
  up_idtentry(IRQ15, (uint32_t)vector_irq15, 0x08, 0x8e);

  /* Then program the IDT */

  idt_flush((uint32_t)&idt_ptr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Initialize the IDT */

  up_idtinit();

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_restore(X86_FLAGS_IF);
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  unsigned int regaddr;
  uint8_t      regbit;

  if (irq >= IRQ0)
    {
      /* Map the IRQ IMR register to a PIC and a bit number */

      if (irq <= IRQ7)
        {
          regaddr = PIC1_IMR;
          regbit  = (1 << (irq - IRQ0));
        }
      else if (irq <= IRQ15)
        {
          regaddr = PIC2_IMR;
          regbit  = (1 << (irq - IRQ8));
        }
      else
        {
          return;
        }

      /* Disable (mask) the interrupt */

      modifyreg8(regaddr, 0, regbit);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  unsigned int regaddr;
  uint8_t      regbit;

  if (irq >= IRQ0)
    {
      /* Map the IRQ IMR register to a PIC and a bit number */

      if (irq <= IRQ7)
        {
          regaddr = PIC1_IMR;
          regbit  = (1 << (irq - IRQ0));
        }
      else if (irq <= IRQ15)
        {
          regaddr = PIC2_IMR;
          regbit  = (1 << (irq - IRQ8));
        }
      else
        {
          return;
        }

      /* Enable (unmask) the interrupt */

      modifyreg8(regaddr, regbit, 0);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
#warning "Missing Logic"
  return OK;
}
#endif
