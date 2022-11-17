/****************************************************************************
 * arch/x86_64/src/intel64/intel64_irq.c
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
#include <arch/arch.h>
#include <arch/irq.h>
#include <arch/io.h>
#include <arch/board/board.h>

#include "x86_64_internal.h"
#include "intel64.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_BASE 0x3f8

#define IRQ_STACK_SIZE 0x2000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_apic_init(void);
static void up_idtentry(unsigned int index, uint64_t base, uint16_t sel,
                        uint8_t flags, uint8_t ist);
static inline void up_idtinit(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint64_t *g_current_regs;

uint8_t g_interrupt_stack[IRQ_STACK_SIZE] aligned_data(16);
uint8_t *g_interrupt_stack_end = g_interrupt_stack + IRQ_STACK_SIZE - 16;

uint8_t g_isr_stack[IRQ_STACK_SIZE] aligned_data(16);
uint8_t *g_isr_stack_end = g_isr_stack + IRQ_STACK_SIZE - 16;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct idt_entry_s idt_entries[256];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ioapic_read
 *
 * Description:
 *  Read a IOAPIC register
 *
 ****************************************************************************/

uint32_t up_ioapic_read(uint32_t reg)
{
  mmio_write32((void *)IOAPIC_BASE + IOAPIC_REG_INDEX, reg);
  return mmio_read32((void *)IOAPIC_BASE + IOAPIC_REG_DATA);
}

/****************************************************************************
 * Name: up_ioapic_write
 *
 * Description:
 *  Write a IOAPIC register
 *
 ****************************************************************************/

void up_ioapic_write(uint32_t reg, uint32_t data)
{
  mmio_write32((void *)IOAPIC_BASE + IOAPIC_REG_INDEX, reg);
  mmio_write32((void *)IOAPIC_BASE + IOAPIC_REG_DATA, data);
}

/****************************************************************************
 * Name: up_ioapic_pin_set_vector
 *
 * Description:
 *  Bind a hardware IRQ number to APIC IRQ number
 *
 ****************************************************************************/

void up_ioapic_pin_set_vector(unsigned int pin,
                              enum ioapic_trigger_mode trigger_mode,
                              unsigned int vector)
{
  up_ioapic_write(IOAPIC_REG_TABLE + pin * 2 + 1,
      up_apic_cpu_id() << (56 - 32));
  up_ioapic_write(IOAPIC_REG_TABLE + pin * 2,
      trigger_mode | vector);
}

/****************************************************************************
 * Name: up_ioapic_mask_pin
 *
 * Description:
 *  Mask an IO-APIC interrupt
 *
 ****************************************************************************/

void up_ioapic_mask_pin(unsigned int pin)
{
  uint32_t cur;

  cur = up_ioapic_read(IOAPIC_REG_TABLE + pin * 2);
  up_ioapic_write(IOAPIC_REG_TABLE + pin * 2,
      cur | IOAPIC_PIN_DISABLE);
}

/****************************************************************************
 * Name: up_ioapic_unmask_pin
 *
 * Description:
 *  Unmask an IO-APIC interrupt
 *
 ****************************************************************************/

void up_ioapic_unmask_pin(unsigned int pin)
{
  uint32_t cur;

  cur = up_ioapic_read(IOAPIC_REG_TABLE + pin * 2);
  up_ioapic_write(IOAPIC_REG_TABLE + pin * 2,
      cur & ~(IOAPIC_PIN_DISABLE));
}

/****************************************************************************
 * Name: up_init_ist
 *
 * Description:
 *  Initialize the Interrupt Stack Table
 *
 ****************************************************************************/

static void up_ist_init(void)
{
  struct gdt_entry_s tss_l;
  uint64_t           tss_h;

  memset(&tss_l, 0, sizeof(tss_l));
  memset(&tss_h, 0, sizeof(tss_h));

  tss_l.limit_low = (((104 - 1) & 0xffff));    /* Segment limit = TSS size - 1 */

  tss_l.base_low  = ((uintptr_t)ist64 & 0x00ffffff);          /* Low address 1 */
  tss_l.base_high = (((uintptr_t)ist64 & 0xff000000) >> 24);  /* Low address 2 */

  tss_l.P = 1;

  /* Set type as IST */

  tss_l.AC = 1;
  tss_l.EX = 1;

  tss_h = (((uintptr_t)ist64 >> 32) & 0xffffffff);  /* High address */

  gdt64[X86_GDT_ISTL_SEL_NUM] = tss_l;

  /* memcpy used to handle type punning compiler warning */

  memcpy((void *)&gdt64[X86_GDT_ISTH_SEL_NUM],
      (void *)&tss_h, sizeof(gdt64[0]));

  ist64->IST1 = (uintptr_t)g_interrupt_stack_end;
  ist64->IST2 = (uintptr_t)g_isr_stack_end;

  asm volatile ("mov $0x30, %%ax; ltr %%ax":::"memory", "rax");
}

/****************************************************************************
 * Name: up_deinit_8259
 *
 * Description:
 *  Initialize the Legacy 8259 PIC
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
static void up_deinit_8259(void)
{
  /* First do an initialization to for any pending interrupt to vanish */

  /* Starts the initialization sequence (in cascade mode) */

  outb(X86_PIC_INIT, X86_IO_PORT_PIC1_CMD);
  outb(X86_PIC_INIT, X86_IO_PORT_PIC2_CMD);

  /* Remap the IRQ to 32~ in case of a spurious interrupt */

  outb(IRQ0,     X86_IO_PORT_PIC1_DATA);  /* ICW2: Master PIC vector offset */
  outb(IRQ0 + 8, X86_IO_PORT_PIC2_DATA);  /* ICW2: Slave PIC vector offset */

  /* Tell the PICs they are in cascade */

  /* ICW3: tell Master PIC that there is a slave PIC at IRQ2 (0000 0100) */

  outb(X86_PIC1_CASCADE, X86_IO_PORT_PIC1_DATA);

  /* ICW3: tell Slave PIC its cascade identity (0000 0010) */

  outb(X86_PIC2_CASCADE, X86_IO_PORT_PIC2_DATA);

  /* Put the PICs into 8086 mode */

  outb(X86_PIC_8086, X86_IO_PORT_PIC1_DATA);
  outb(X86_PIC_8086, X86_IO_PORT_PIC2_DATA);

  /* Disable the 8259 PIC by masking all the interrupt and acking them */

  outb(0xff, X86_IO_PORT_PIC1_DATA);
  outb(0xff, X86_IO_PORT_PIC2_DATA);
  outb(X86_PIC_EOI, X86_IO_PORT_PIC1_CMD);
  outb(X86_PIC_EOI, X86_IO_PORT_PIC2_CMD);
}
#endif

/****************************************************************************
 * Name: up_init_apic
 *
 * Description:
 *  Initialize the APIC
 *
 ****************************************************************************/

static void up_apic_init(void)
{
  uint32_t ver;
  uint32_t icrl;
  uint32_t apic_base;

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  /* Enable the APIC in X2APIC MODE */

  apic_base = read_msr(MSR_IA32_APIC_BASE) & 0xfffff000;
  write_msr(MSR_IA32_APIC_BASE, apic_base | MSR_IA32_APIC_EN |
                                MSR_IA32_APIC_X2APIC | MSR_IA32_APIC_BSP);
#endif

  /* Enable the APIC and setup an spurious interrupt vector */

  write_msr(MSR_X2APIC_SPIV, MSR_X2APIC_SPIV_EN | IRQ_SPURIOUS);

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  /* Disable the LINT interrupt lines */

  write_msr(MSR_X2APIC_LINT0, MSR_X2APIC_MASKED);
  write_msr(MSR_X2APIC_LINT1, MSR_X2APIC_MASKED);

  /* Disable performance counter overflow interrupts on machines tha
   * provide that interrupt entry.
   */

  ver = read_msr(MSR_X2APIC_VER);
  if (((ver >> 16) & 0xff) >= 4)
    {
      write_msr(MSR_X2APIC_LVTPMR, MSR_X2APIC_MASKED);
    }

  /* Map error interrupt to IRQ_ERROR. */

  write_msr(MSR_X2APIC_LERR, MSR_X2APIC_MASKED);

  /* Clear error status register (requires back-to-back writes). */

  write_msr(MSR_X2APIC_ESR, 0);
  write_msr(MSR_X2APIC_ESR, 0);

  /* Ack any outstanding interrupts. */

  write_msr(MSR_X2APIC_EOI, 0);

  /* Send an Init Level De-Assert to synchronize arbitration ID's. */

  write_msr(MSR_X2APIC_ICR, MSR_X2APIC_ICR_BCAST | MSR_X2APIC_ICR_INIT |
                            MSR_X2APIC_ICR_LEVEL);
  do
    {
      icrl = read_msr(MSR_X2APIC_ICR);
    }
  while (icrl & MSR_X2APIC_ICR_DELIVS);

  /* Enable interrupts on the APIC (but not on the processor). */

  write_msr(MSR_X2APIC_TPR, 0);
#endif
}

/****************************************************************************
 * Name: legacy_pic_irq_handler
 *
 * Description:
 *  This function will capture will legacy 8259 PIC IRQ using virtual wire
 *  mode
 *
 ****************************************************************************/

static int unused_code
legacy_pic_irq_handler(int irq, uint32_t *regs, void *arg)
{
  return 0;
}

/****************************************************************************
 * Name: up_init_ioapic
 *
 * Description:
 *  Initialize the IOAPIC
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
static void up_ioapic_init(void)
{
  int i;

  up_map_region((void *)IOAPIC_BASE, HUGE_PAGE_SIZE,
                X86_PAGE_PRESENT | X86_PAGE_WR | X86_PAGE_NOCACHE);

  /* Setup the IO-APIC, remap the interrupt to 32~ */

  uint32_t maxintr = (up_ioapic_read(IOAPIC_REG_VER) >> 16) & 0xff;

  for (i = 0; i < maxintr; i++)
    {
      up_ioapic_pin_set_vector(i, TRIGGER_RISING_EDGE, IRQ0 + i);
      up_ioapic_mask_pin(i);
    }

  return;
}
#endif

/****************************************************************************
 * Name: up_idtentry
 *
 * Description:
 *   Initialize one IDT entry.
 *
 ****************************************************************************/

static void up_idtentry(unsigned int index, uint64_t base, uint16_t sel,
                        uint8_t flags, uint8_t ist)
{
  struct idt_entry_s *entry = &idt_entries[index];

  entry->lobase  = base & 0xffff;
  entry->hibase  = (base >> 16) & 0xffff;
  entry->xhibase = (base >> 32) & 0xffffffff;
  entry->ist     = ist & 0x7;
  entry->sel     = sel;
  entry->zero    = 0;

  /* We must uncomment the OR below when we get to using user-mode. It sets
   * the interrupt gate's privilege level to 3.
   */

  entry->flags  = flags; /* | 0x60 */
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

struct idt_ptr_s idt_ptr;

static inline void up_idtinit(void)
{
  memset(&idt_entries, 0, sizeof(struct idt_entry_s)*256);

  /* Set each ISR/IRQ to the appropriate vector with selector=8 and with
   * 32-bit interrupt gate.  Interrupt gate (vs. trap gate) will leave
   * interrupts enabled when the IRS/IRQ handler is entered.
   */

  up_idtentry(ISR0,  (uint64_t)vector_isr0 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR1,  (uint64_t)vector_isr1 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR2,  (uint64_t)vector_isr2 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR3,  (uint64_t)vector_isr3 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR4,  (uint64_t)vector_isr4 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR5,  (uint64_t)vector_isr5 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR6,  (uint64_t)vector_isr6 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR7,  (uint64_t)vector_isr7 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR8,  (uint64_t)vector_isr8 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR9,  (uint64_t)vector_isr9 , 0x08, 0x8e, 0x2);
  up_idtentry(ISR10, (uint64_t)vector_isr10, 0x08, 0x8e, 0x2);
  up_idtentry(ISR11, (uint64_t)vector_isr11, 0x08, 0x8e, 0x2);
  up_idtentry(ISR12, (uint64_t)vector_isr12, 0x08, 0x8e, 0x2);
  up_idtentry(ISR13, (uint64_t)vector_isr13, 0x08, 0x8e, 0x2);
  up_idtentry(ISR14, (uint64_t)vector_isr14, 0x08, 0x8e, 0x2);
  up_idtentry(ISR15, (uint64_t)vector_isr15, 0x08, 0x8e, 0x2);
  up_idtentry(ISR16, (uint64_t)vector_isr16, 0x08, 0x8e, 0x2);
  up_idtentry(ISR17, (uint64_t)vector_isr17, 0x08, 0x8e, 0x2);
  up_idtentry(ISR18, (uint64_t)vector_isr18, 0x08, 0x8e, 0x2);
  up_idtentry(ISR19, (uint64_t)vector_isr19, 0x08, 0x8e, 0x2);
  up_idtentry(ISR20, (uint64_t)vector_isr20, 0x08, 0x8e, 0x2);
  up_idtentry(ISR21, (uint64_t)vector_isr21, 0x08, 0x8e, 0x2);
  up_idtentry(ISR22, (uint64_t)vector_isr22, 0x08, 0x8e, 0x2);
  up_idtentry(ISR23, (uint64_t)vector_isr23, 0x08, 0x8e, 0x2);
  up_idtentry(ISR24, (uint64_t)vector_isr24, 0x08, 0x8e, 0x2);
  up_idtentry(ISR25, (uint64_t)vector_isr25, 0x08, 0x8e, 0x2);
  up_idtentry(ISR26, (uint64_t)vector_isr26, 0x08, 0x8e, 0x2);
  up_idtentry(ISR27, (uint64_t)vector_isr27, 0x08, 0x8e, 0x2);
  up_idtentry(ISR28, (uint64_t)vector_isr28, 0x08, 0x8e, 0x2);
  up_idtentry(ISR29, (uint64_t)vector_isr29, 0x08, 0x8e, 0x2);
  up_idtentry(ISR30, (uint64_t)vector_isr30, 0x08, 0x8e, 0x2);
  up_idtentry(ISR31, (uint64_t)vector_isr31, 0x08, 0x8e, 0x2);

  up_idtentry(IRQ0,  (uint64_t)vector_irq0,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ1,  (uint64_t)vector_irq1,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ2,  (uint64_t)vector_irq2,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ3,  (uint64_t)vector_irq3,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ4,  (uint64_t)vector_irq4,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ5,  (uint64_t)vector_irq5,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ6,  (uint64_t)vector_irq6,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ7,  (uint64_t)vector_irq7,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ8,  (uint64_t)vector_irq8,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ9,  (uint64_t)vector_irq9,  0x08, 0x8e, 0x1);
  up_idtentry(IRQ10, (uint64_t)vector_irq10, 0x08, 0x8e, 0x1);
  up_idtentry(IRQ11, (uint64_t)vector_irq11, 0x08, 0x8e, 0x1);
  up_idtentry(IRQ12, (uint64_t)vector_irq12, 0x08, 0x8e, 0x1);
  up_idtentry(IRQ13, (uint64_t)vector_irq13, 0x08, 0x8e, 0x1);
  up_idtentry(IRQ14, (uint64_t)vector_irq14, 0x08, 0x8e, 0x1);
  up_idtentry(IRQ15, (uint64_t)vector_irq15, 0x08, 0x8e, 0x1);

  /* Then program the IDT */

  setidt(&idt_entries, sizeof(struct idt_entry_s) * NR_IRQS - 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Initialize the IST */

  up_ist_init();

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  /* Disable 8259 PIC */

  up_deinit_8259();
#endif

  /* Initialize the APIC */

  up_apic_init();

#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  /* Initialize the IOAPIC */

  up_ioapic_init();
#endif

  /* Initialize the IDT */

  up_idtinit();

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_restore(X86_64_RFLAGS_IF);
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ line specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  if (irq >= IRQ0)
    {
      up_ioapic_mask_pin(irq - IRQ0);
    }
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ line specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
#ifndef CONFIG_ARCH_INTEL64_DISABLE_INT_INIT
  if (irq >= IRQ0)
    {
      up_ioapic_unmask_pin(irq - IRQ0);
    }
#endif
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
