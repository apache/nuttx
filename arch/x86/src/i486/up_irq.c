/****************************************************************************
 * arch/x86/src/i486/up_irq.c
 * arch/x86/src/chip/up_irq.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/io.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"
#include "qemu_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void idt_outb(uint8_t val, uint16_t addr) __attribute__((noinline));
static void up_remappic(void);
static void up_idtentry(struct idt_entry_s *entry, uint32_t base,
                        uint16_t sel, uint8_t flags);
static inline void up_idtinit(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

uint32_t *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct idt_entry_s idt_entries[256];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name idt_outb
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
 * Name up_remappic
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

  /* If the PIC has been reset, it must be initialized with 2 to 4 Initialization
   * Command Words (ICW) before it will accept and process Interrupt Requests. The
   * following outlines the four possible Initialization Command Words. 
   */

  /* Remap the irq table for primary:
   *
   * ICW1 - We will be sending ICW4
   * ICW2 - Address
   * ICW3    */

  idt_outb(PIC_ICW1_ICW4|PIC_ICW1_ICW1, PIC1_ICW1);
  idt_outb(0x20,                        PIC1_ICW2);
  idt_outb(PIC1_ICW3_IRQ2,              PIC1_ICW3);
  idt_outb(PIC_ICW4_808xMODE,           PIC1_ICW4);

  /* Remap irq for slave */

  idt_outb(PIC_ICW1_ICW4|PIC_ICW1_ICW1, PIC2_ICW1);
  idt_outb(0x28,                        PIC2_ICW2);
  idt_outb(PIC_ICW3_SID2,               PIC2_ICW3);
  idt_outb(PIC_ICW4_808xMODE,           PIC2_ICW4);
}

/****************************************************************************
 * Name up_idtentry
 *
 * Description:
 *   Initialize one IDT entry. 
 *
 ****************************************************************************/

static void up_idtentry(struct idt_entry_s *entry, uint32_t base,
                        uint16_t sel, uint8_t flags)
{
  entry->lobase = base & 0xffff;
  entry->hibase = (base >> 16) & 0xffff;

  entry->sel    = sel;
  entry->zero   = 0;

  /* We must uncomment the OR below when we get to using user-mode. It sets the
   * interrupt gate's privilege level to 3.
   */

  entry->flags  = flags /* | 0x60 */;
}

/****************************************************************************
 * Name up_idtinit
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

  up_remappic();

  up_idtentry(&idt_entries[0],  (uint32_t)vector_isr0 , 0x08, 0x8e);
  up_idtentry(&idt_entries[1],  (uint32_t)vector_isr1 , 0x08, 0x8e);
  up_idtentry(&idt_entries[2],  (uint32_t)vector_isr2 , 0x08, 0x8e);
  up_idtentry(&idt_entries[3],  (uint32_t)vector_isr3 , 0x08, 0x8e);
  up_idtentry(&idt_entries[4],  (uint32_t)vector_isr4 , 0x08, 0x8e);
  up_idtentry(&idt_entries[5],  (uint32_t)vector_isr5 , 0x08, 0x8e);
  up_idtentry(&idt_entries[6],  (uint32_t)vector_isr6 , 0x08, 0x8e);
  up_idtentry(&idt_entries[7],  (uint32_t)vector_isr7 , 0x08, 0x8e);
  up_idtentry(&idt_entries[8],  (uint32_t)vector_isr8 , 0x08, 0x8e);
  up_idtentry(&idt_entries[9],  (uint32_t)vector_isr9 , 0x08, 0x8e);
  up_idtentry(&idt_entries[10], (uint32_t)vector_isr10, 0x08, 0x8e);
  up_idtentry(&idt_entries[11], (uint32_t)vector_isr11, 0x08, 0x8e);
  up_idtentry(&idt_entries[12], (uint32_t)vector_isr12, 0x08, 0x8e);
  up_idtentry(&idt_entries[13], (uint32_t)vector_isr13, 0x08, 0x8e);
  up_idtentry(&idt_entries[14], (uint32_t)vector_isr14, 0x08, 0x8e);
  up_idtentry(&idt_entries[15], (uint32_t)vector_isr15, 0x08, 0x8e);
  up_idtentry(&idt_entries[16], (uint32_t)vector_isr16, 0x08, 0x8e);
  up_idtentry(&idt_entries[17], (uint32_t)vector_isr17, 0x08, 0x8e);
  up_idtentry(&idt_entries[18], (uint32_t)vector_isr18, 0x08, 0x8e);
  up_idtentry(&idt_entries[19], (uint32_t)vector_isr19, 0x08, 0x8e);
  up_idtentry(&idt_entries[20], (uint32_t)vector_isr20, 0x08, 0x8e);
  up_idtentry(&idt_entries[21], (uint32_t)vector_isr21, 0x08, 0x8e);
  up_idtentry(&idt_entries[22], (uint32_t)vector_isr22, 0x08, 0x8e);
  up_idtentry(&idt_entries[23], (uint32_t)vector_isr23, 0x08, 0x8e);
  up_idtentry(&idt_entries[24], (uint32_t)vector_isr24, 0x08, 0x8e);
  up_idtentry(&idt_entries[25], (uint32_t)vector_isr25, 0x08, 0x8e);
  up_idtentry(&idt_entries[26], (uint32_t)vector_isr26, 0x08, 0x8e);
  up_idtentry(&idt_entries[27], (uint32_t)vector_isr27, 0x08, 0x8e);
  up_idtentry(&idt_entries[28], (uint32_t)vector_isr28, 0x08, 0x8e);
  up_idtentry(&idt_entries[29], (uint32_t)vector_isr29, 0x08, 0x8e);
  up_idtentry(&idt_entries[30], (uint32_t)vector_isr30, 0x08, 0x8e);
  up_idtentry(&idt_entries[31], (uint32_t)vector_isr31, 0x08, 0x8e);
  up_idtentry(&idt_entries[32], (uint32_t)vector_irq0,  0x08, 0x8e);
  up_idtentry(&idt_entries[33], (uint32_t)vector_irq1,  0x08, 0x8e);
  up_idtentry(&idt_entries[34], (uint32_t)vector_irq2,  0x08, 0x8e);
  up_idtentry(&idt_entries[35], (uint32_t)vector_irq3,  0x08, 0x8e);
  up_idtentry(&idt_entries[36], (uint32_t)vector_irq4,  0x08, 0x8e);
  up_idtentry(&idt_entries[37], (uint32_t)vector_irq5,  0x08, 0x8e);
  up_idtentry(&idt_entries[38], (uint32_t)vector_irq6,  0x08, 0x8e);
  up_idtentry(&idt_entries[39], (uint32_t)vector_irq7,  0x08, 0x8e);
  up_idtentry(&idt_entries[40], (uint32_t)vector_irq8,  0x08, 0x8e);
  up_idtentry(&idt_entries[41], (uint32_t)vector_irq9,  0x08, 0x8e);
  up_idtentry(&idt_entries[42], (uint32_t)vector_irq10, 0x08, 0x8e);
  up_idtentry(&idt_entries[43], (uint32_t)vector_irq11, 0x08, 0x8e);
  up_idtentry(&idt_entries[44], (uint32_t)vector_irq12, 0x08, 0x8e);
  up_idtentry(&idt_entries[45], (uint32_t)vector_irq13, 0x08, 0x8e);
  up_idtentry(&idt_entries[46], (uint32_t)vector_irq14, 0x08, 0x8e);
  up_idtentry(&idt_entries[47], (uint32_t)vector_irq15, 0x08, 0x8e);

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

  current_regs = NULL;

  /* Initialize the IDT */

  up_idtinit();

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqrestore(X86_FLAGS_IF);
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
 
  if ((unsigned)irq >= IRQ0)
    {
      /* Map the IRQ IMR regiser to a PIC and a bit number */

      if ((unsigned)irq <= IRQ7)
        {
          regaddr = PIC1_IMR;
          regbit  = (1 << (irq - IRQ0));
        }
      else if ((unsigned)irq <= IRQ15)
        {
          regaddr = PIC2_IMR;
          regbit  = (1 << (irq - IRQ8));
        }
      else
        {
          return;
        }

      /* Disable the interrupt */

      modifyreg8(regaddr, regbit, 0);
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
 
  if ((unsigned)irq >= IRQ0)
    {
      /* Map the IRQ IMR regiser to a PIC and a bit number */

      if ((unsigned)irq <= IRQ7)
        {
          regaddr = PIC1_IMR;
          regbit  = (1 << (irq - IRQ0));
        }
      else if ((unsigned)irq <= IRQ15)
        {
          regaddr = PIC2_IMR;
          regbit  = (1 << (irq - IRQ8));
        }
      else
        {
          return;
        }

      /* Enable the interrupt */

      modifyreg8(regaddr, 0, regbit);
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
