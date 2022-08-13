/****************************************************************************
 * arch/x86_64/include/intel64/arch.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_X86_64_INCLUDE_INTEL64_ARCH_H
#define __ARCH_X86_64_INCLUDE_INTEL64_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X86_64_LOAD_OFFSET 0x100000000

/* RFLAGS bits */

#define X86_64_RFLAGS_CF          (1 << 0)  /* Bit 0:  Carry Flag */
                                            /* Bit 1:  Reserved */
#define X86_64_RFLAGS_PF          (1 << 2)  /* Bit 2:  Parity Flag */
                                            /* Bit 3:  Reserved */
#define X86_64_RFLAGS_AF          (1 << 4)  /* Bit 4:  Auxillary carry Flag */
                                            /* Bit 5:  Reserved */
#define X86_64_RFLAGS_ZF          (1 << 6)  /* Bit 6:  Zero Flag */
#define X86_64_RFLAGS_SF          (1 << 7)  /* Bit 7:  Sign Flag */
#define X86_64_RFLAGS_TF          (1 << 8)  /* Bit 8:  Trap Flag */
#define X86_64_RFLAGS_IF          (1 << 9)  /* Bit 9:  Interrupt Flag */
#define X86_64_RFLAGS_DF          (1 << 10) /* Bit 10: Direction Flag */
#define X86_64_RFLAGS_OF          (1 << 11) /* Bit 11: Overflow Flag */
#define X86_64_RFLAGS_IOPL_SHIFT  (12)      /* Bits 12-13: IOPL mask (286+ only)*/
#define X86_64_RFLAGS_IOPL_MASK   (3 << X86_64_RFLAGS_IOPL_SHIFT)
#define X86_64_RFLAGS_NT          (1 << 14) /* Bit 14: Nested Task */
                                            /* Bit 15: Reserved */
#define X86_64_RFLAGS_RF          (1 << 16) /* Bit 16: Resume Flag (386+ only) */
#define X86_64_RFLAGS_VM          (1 << 17) /* Bit 17: Virtual Mode (386+ only) */
#define X86_64_RFLAGS_AC          (1 << 18) /* Bit 18: Alignment Check (486SX+ only) */
#define X86_64_RFLAGS_VIF         (1 << 19) /* Bit 19: Virtual Interrupt Flag (Pentium+) */
#define X86_64_RFLAGS_VIP         (1 << 20) /* Bit 20: Virtual Interrupt Pending (Pentium+) */
#define X86_64_RFLAGS_ID          (1 << 21) /* Bit 21: CPUID detection flag (Pentium+) */

/* GDT Definitions */

/* Starting from third selector to confirm the syscall interface */

#define X86_GDT_ENTRY_SIZE      0x8

#define X86_GDT_CODE_SEL_NUM    1
# define X86_GDT_CODE_SEL       (X86_GDT_CODE_SEL_NUM * X86_GDT_ENTRY_SIZE)

#define X86_GDT_DATA_SEL_NUM    2
# define X86_GDT_DATA_SEL       (X86_GDT_DATA_SEL_NUM * X86_GDT_ENTRY_SIZE)

#define X86_GDT_ISTL_SEL_NUM    6
#define X86_GDT_ISTH_SEL_NUM    (X86_GDT_ISTL_SEL_NUM + 1)

#define X86_GDT_BASE      0x0000000000000000
#define X86_GDT_LIMIT     0x000f00000000ffff

#define X86_GDT_FLAG_LONG 0x0020000000000000

#define X86_GDT_ACC_PR    0x0000800000000000
#define X86_GDT_ACC_SEG   0x0000100000000000
#define X86_GDT_ACC_EX    0x0000080000000000
#define X86_GDT_ACC_WR    0x0000020000000000

#define X86_GDT_CODE64_ENTRY    (X86_GDT_BASE + X86_GDT_LIMIT + X86_GDT_FLAG_LONG + X86_GDT_ACC_PR + X86_GDT_ACC_SEG + X86_GDT_ACC_EX)
#define X86_GDT_CODE32_ENTRY    (X86_GDT_BASE + X86_GDT_LIMIT + X86_GDT_ACC_PR + X86_GDT_ACC_SEG + X86_GDT_ACC_EX)
#define X86_GDT_DATA_ENTRY      (X86_GDT_BASE + X86_GDT_LIMIT + X86_GDT_ACC_PR + X86_GDT_ACC_SEG + X86_GDT_ACC_WR)

/* CR0 Definitions */

#define X86_CR0_PE        0x00000001
#define X86_CR0_MP        0x00000002
#define X86_CR0_EM        0x00000004
#define X86_CR0_WP        0x00010000
#define X86_CR0_PG        0x80000000

/* CR4 Definitions */

#define X86_CR4_PAE      0x00000020
#define X86_CR4_PGE      0x00000080
#define X86_CR4_OSXFSR   0x00000200
#define X86_CR4_XMMEXCPT 0x00000400
#define X86_CR4_FGSBASE  0x00010000
#define X86_CR4_PCIDE    0x00020000

/* PAGE TABLE ENTRY Definitions */

#define X86_PAGE_PRESENT (1 << 0)
#define X86_PAGE_WR      (1 << 1)
#define X86_PAGE_USER    (1 << 2)
#define X86_PAGE_WRTHR   (1 << 3)
#define X86_PAGE_NOCACHE (1 << 4)
#define X86_PAGE_HUGE    (1 << 7)
#define X86_PAGE_GLOBAL  (1 << 8)
#define X86_PAGE_NX      (1 << 63)

#define X86_PAGE_ENTRY_SIZE 8
#define X86_NUM_PAGE_ENTRY (PAGE_SIZE / X86_PAGE_ENTRY_SIZE)

#define PAGE_SIZE        (0x1000)
# define PAGE_MASK        (~(PAGE_SIZE - 1))

#define HUGE_PAGE_SIZE   (0x200000)
# define HUGE_PAGE_MASK   (~(HUGE_PAGE_SIZE - 1))

/* CPUID Leaf Definitions */

#define X86_64_CPUID_CAP        0x01
# define X86_64_CPUID_01_SSE3   (1 << 0)
# define X86_64_CPUID_01_PCID   (1 << 17)
# define X86_64_CPUID_01_X2APIC (1 << 21)
# define X86_64_CPUID_01_TSCDEA (1 << 24)
# define X86_64_CPUID_01_XSAVE  (1 << 26)
# define X86_64_CPUID_01_RDRAND (1 << 30)
#define X86_64_CPUID_TSC        0x15

/* MSR Definitions */

#define MSR_FS_BASE             0xc0000100 /* 64bit FS base */

#define MSR_EFER                0xc0000080
# define EFER_LME               0x00000100

#define MSR_MTRR_DEF_TYPE       0x000002ff
# define MTRR_ENABLE            0x00000800

#define MSR_IA32_TSC_DEADLINE   0x6e0

#define MSR_IA32_APIC_BASE      0x01b
# define MSR_IA32_APIC_EN       0x800
# define MSR_IA32_APIC_X2APIC   0x400
# define MSR_IA32_APIC_BSP      0x100

#define MSR_X2APIC_ID           0x802
#define MSR_X2APIC_VER          0x803
#define MSR_X2APIC_TPR          0x808
#define MSR_X2APIC_PPR          0x80a
#define MSR_X2APIC_EOI          0x80b
#define MSR_X2APIC_LDR          0x80d

#define MSR_X2APIC_SPIV         0x80f
# define MSR_X2APIC_SPIV_EN     0x100

#define MSR_X2APIC_ISR0         0x810
#define MSR_X2APIC_ISR1         0x811
#define MSR_X2APIC_ISR2         0x812
#define MSR_X2APIC_ISR3         0x813
#define MSR_X2APIC_ISR4         0x814
#define MSR_X2APIC_ISR5         0x815
#define MSR_X2APIC_ISR6         0x816
#define MSR_X2APIC_ISR7         0x817

#define MSR_X2APIC_TMR0         0x818
#define MSR_X2APIC_TMR1         0x819
#define MSR_X2APIC_TMR2         0x81a
#define MSR_X2APIC_TMR3         0x81b
#define MSR_X2APIC_TMR4         0x81c
#define MSR_X2APIC_TMR5         0x81d
#define MSR_X2APIC_TMR6         0x81e
#define MSR_X2APIC_TMR7         0x81f

#define MSR_X2APIC_IRR0         0x820
#define MSR_X2APIC_IRR1         0x821
#define MSR_X2APIC_IRR2         0x822
#define MSR_X2APIC_IRR3         0x823
#define MSR_X2APIC_IRR4         0x824
#define MSR_X2APIC_IRR5         0x825
#define MSR_X2APIC_IRR6         0x826
#define MSR_X2APIC_IRR7         0x827

#define MSR_X2APIC_ESR          0x828
#define MSR_X2APIC_ICR          0x830
# define MSR_X2APIC_ICR_INIT           0x00000500  /* INIT/RESET */
# define MSR_X2APIC_ICR_STARTUP        0x00000600  /* Startup IPI */
# define MSR_X2APIC_ICR_DELIVS         0x00001000  /* Delivery status */
# define MSR_X2APIC_ICR_ASSERT         0x00004000  /* Assert interrupt (vs deassert) */
# define MSR_X2APIC_ICR_DEASSERT       0x00000000
# define MSR_X2APIC_ICR_LEVEL          0x00008000  /* Level triggered */
# define MSR_X2APIC_ICR_BCAST          0x00080000  /* Send to all APICs, including self. */
# define MSR_X2APIC_ICR_BUSY           0x00001000
# define MSR_X2APIC_ICR_FIXED          0x00000000
#define MSR_X2APIC_LVTT         0x832
# define MSR_X2APIC_LVTT_X1            0x0000000B  /* divide counts by 1 */
# define MSR_X2APIC_LVTT_PERIODIC      0x00020000  /* Periodic */
# define MSR_X2APIC_LVTT_TSC_DEADLINE  0x00040000  /* Enable TSC DEADLINE One-shot timer */
#define MSR_X2APIC_LVTTHER      0x833
#define MSR_X2APIC_LVTPMR       0x834
#define MSR_X2APIC_LINT0        0x835
#define MSR_X2APIC_LINT1        0x836
#define MSR_X2APIC_LERR         0x837
# define MSR_X2APIC_MASKED             0x00010000  /* Interrupt masked */
#define MSR_X2APIC_TMICT        0x838
#define MSR_X2APIC_TMCCT        0x839
#define MSR_X2APIC_TDCR         0x83e

/* IOAPIC related Definitions */

#define IOAPIC_BASE             0xfec00000
#define IOAPIC_REG_INDEX        0x00
#define IOAPIC_REG_DATA         0x10
# define IOAPIC_REG_ID          0x00       /* Register index: ID */
# define IOAPIC_REG_VER         0x01       /* Register index: version */
# define IOAPIC_REG_TABLE       0x10       /* Redirection table base */
# define IOAPIC_PIN_DISABLE     (1 << 16)  /* Disable */

/* PIC related Definitions */

#define X86_IO_PORT_PIC1_CMD   0x20
#define X86_IO_PORT_PIC1_DATA  (X86_IO_PORT_PIC1_CMD + 1)
#define X86_IO_PORT_PIC2_CMD   0xA0
#define X86_IO_PORT_PIC2_DATA  (X86_IO_PORT_PIC2_CMD + 2)

#define X86_PIC_INIT           0x11
#define X86_PIC1_CASCADE       4
#define X86_PIC2_CASCADE       2
#define X86_PIC_8086           1
#define X86_PIC_EOI            0x20

#define BITS_PER_LONG    64

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IDT data structures ******************************************************
 *
 * The Interrupt Descriptor Table (IDT) is a data structure used by the x86
 * architecture to implement an interrupt vector table. The IDT is used by
 * the processor to determine the correct response to interrupts and
 * exceptions.
 */

begin_packed_struct struct idt_entry_s
{
  uint16_t lobase;           /* Lower 16-bits of vector address for interrupt */
  uint16_t sel;              /* Kernel segment selector */
  uint8_t  ist;              /* 0..2 bits is Interrupt Stack Table offset, reset is zero */
  uint8_t  flags;            /* (See documentation) */
  uint16_t hibase;           /* Upper 16-bits of vector address for interrupt */
  uint32_t xhibase;          /* Top   32-bits of vector address for interrupt */
  uint32_t zero;             /* reserved */
} end_packed_struct;

/* A struct describing a pointer to an array of interrupt handlers.  This is
 * in a format suitable for giving to 'lidt'.
 */

begin_packed_struct struct idt_ptr_s
{
  uint16_t limit;
  uint64_t base;             /* The address of the first GDT entry */
} end_packed_struct;

/* GDT data structures ******************************************************
 *
 * The Global Descriptor Table (GDT) is a data structure used by the x86
 * architecture to implement segments and privilege levels. The GDT is used
 * by the processor to determine current privilege level and memory access
 * right.
 */

begin_packed_struct struct gdt_entry_s
{
  uint16_t limit_low;            /* Lower 16-bits of segment limit */
  uint32_t base_low:24;          /* Lower 24-bits of base address */
  uint8_t  AC:1;                 /* 1: CPU accessed this segement */
  uint8_t  RW:1;                 /* 1: Data Segement 0: Code Segement */
  uint8_t  DC:1;                 /* Direction bit/Conforming bit.  */
  uint8_t  EX:1;                 /* 1: Segment can be executed  */
  uint8_t  S:1;                  /* S: 0:TSS 1:Code/Data Segement */
  uint8_t  DPL:2;                /* DPL */
  uint8_t  P:1;                  /* Present? 1:Segment is preset */
  uint8_t  limit_high:4;         /* Upper 4-bits of segment limit */
  uint8_t  RESV:1;               /* Reserved */
  uint8_t  L:1;                  /* 1: Long Mode 0: IA32e Mode */
  uint8_t  SZ:1;                 /* 1: 32bit protected mode 0: 16bit protected mode  */
  uint8_t  GR:1;                 /* 0: Byte Granularity 1: 4KB Granularity  */
  uint32_t base_high:8;          /* Upper 8-bits of base address */
} end_packed_struct;

/* A struct describing a pointer to an array of global descriptors.  This is
 * in a format suitable for giving to 'lgdt'.
 */

begin_packed_struct struct gdt_ptr_s
{
  uint16_t limit;
  uint64_t base;             /* The address of the first GDT entry */
} end_packed_struct;

/* IST data structures ******************************************************
 *
 * The Interrupt Stack Table (GDT) is a data structure used by the x86-64
 * architecture to automatically switch stack on interrupt and privilege
 * change. It allows setting up to 7 different stack for interrupts.
 */

begin_packed_struct struct ist_s
{
  uint32_t reserved1;            /* reserved */
  uint64_t RSP0;                 /* Stack for Ring 0 */
  uint64_t RSP1;                 /* Stack for Ring 1 */
  uint64_t RSP2;                 /* Stack for Ring 2 */
  uint64_t reserved2;            /* reserved */
  uint64_t IST1;                 /* Interrupt Stack 1 */
  uint64_t IST2;                 /* Interrupt Stack 2 */
  uint64_t IST3;                 /* Interrupt Stack 3 */
  uint64_t IST4;                 /* Interrupt Stack 4 */
  uint64_t IST5;                 /* Interrupt Stack 5 */
  uint64_t IST6;                 /* Interrupt Stack 6 */
  uint64_t IST7;                 /* Interrupt Stack 7 */
  uint64_t reserved3;            /* reserved */
  uint64_t reserved4;            /* reserved */
  uint16_t reserved5;            /* reserved */
  uint16_t IOPB_OFFSET;          /* IOPB_offset */
} end_packed_struct;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These are defined in intel64_head.S */

extern volatile uint8_t pdpt_low;
extern volatile uint8_t pd_low;
extern volatile uint8_t pt_low;

extern volatile uint8_t ist64_low;
extern volatile uint8_t gdt64_low;
extern volatile uint8_t gdt64_ist_low;
extern volatile uint8_t gdt64_low_end;

/* The actual address of the page table and gdt/ist after mapping the kernel
 * in high address
 */

extern volatile uint64_t *pdpt;
extern volatile uint64_t *pd;
extern volatile uint64_t *pt;

extern volatile struct ist_s *ist64;
extern volatile struct gdt_entry_s *gdt64;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int up_map_region(void *base, int size, int flags);
void x86_64_check_and_enable_capability(void);

extern void __enable_sse_avx(void);
extern void __revoke_low_memory(void);
extern void __enable_pcid(void);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_INCLUDE_INTEL64_ARCH_H */
