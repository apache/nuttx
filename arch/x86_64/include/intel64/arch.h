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
#  include <stddef.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X86_64_LOAD_OFFSET 0x100000000

/* Page pool configuration for CONFIG_ARCH_PGPOOL_MAPPING=n */

#ifndef CONFIG_ARCH_X86_64_PGPOOL_SIZE
#  define X86_64_PGPOOL_SIZE      (0)
#else
#  if CONFIG_ARCH_X86_64_PGPOOL_SIZE % CONFIG_MM_PGSIZE != 0
#    error CONFIG_ARCH_X86_64_PGPOOL_SIZE must be multiple of page size
#  endif
#  define X86_64_PGPOOL_SIZE      (CONFIG_ARCH_X86_64_PGPOOL_SIZE)
#endif

#define X86_64_PGPOOL_BASE        (CONFIG_RAM_SIZE - X86_64_PGPOOL_SIZE)

/* RFLAGS bits */

#define X86_64_RFLAGS_CF          (1 << 0)  /* Bit 0:  Carry Flag */
                                            /* Bit 1:  Reserved */
#define X86_64_RFLAGS_PF          (1 << 2)  /* Bit 2:  Parity Flag */
                                            /* Bit 3:  Reserved */
#define X86_64_RFLAGS_AF          (1 << 4)  /* Bit 4:  Auxiliary carry Flag */
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

#define X86_GDT_ENTRY_SIZE        0x8

#define X86_GDT_CODE_SEL_NUM      1
#  define X86_GDT_CODE_SEL        (X86_GDT_CODE_SEL_NUM * X86_GDT_ENTRY_SIZE)

#define X86_GDT_DATA_SEL_NUM      2
#  define X86_GDT_DATA_SEL        (X86_GDT_DATA_SEL_NUM * X86_GDT_ENTRY_SIZE)

/* The first TSS entry */

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
#define X86_CR4_OSXSAVE  0x00040000

/* XCR0 */

#define X86_XCR0_X87       (1 << 0)
#define X86_XCR0_SSE       (1 << 1)
#define X86_XCR0_AVX       (1 << 2)
#define X86_XCR0_BNDREG    (1 << 3)
#define X86_XCR0_BNDCSR    (1 << 4)
#define X86_XCR0_OPMASK    (1 << 5)
#define X86_XCR0_HI256     (1 << 6)
#define X86_XCR0_HI16      (1 << 7)
#define X86_XCR0_PT        (1 << 8)
#define X86_XCR0_PKRU      (1 << 9)
#define X86_XCR0_PASID     (1 << 10)
#define X86_XCR0_CETU      (1 << 11)
#define X86_XCR0_CETS      (1 << 12)
#define X86_XCR0_HDC       (1 << 13)
#define X86_XCR0_UINTR     (1 << 14)
#define X86_XCR0_LBR       (1 << 15)
#define X86_XCR0_HWP       (1 << 16)
#define X86_XCR0_XTILECFG  (1 << 17)
#define X86_XCR0_XTILEDATA (1 << 18)
#define X86_XCR0_APX       (1 << 19)

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
#  define PAGE_MASK      (~(PAGE_SIZE - 1))

#define HUGE_PAGE_SIZE   (0x200000)
#  define HUGE_PAGE_MASK (~(HUGE_PAGE_SIZE - 1))

/* Kernel mapping - lower 1GB maps to 4GB-5GB */

#define X86_PDPT_KERNEL_MAP (X86_PAGE_GLOBAL | X86_PAGE_WR | \
                             X86_PAGE_PRESENT | X86_PAGE_HUGE)

/* CPUID Leaf Definitions */

#define X86_64_CPUID_VENDOR           0x00
#define X86_64_CPUID_CAP              0x01
#  define X86_64_CPUID_01_SSE3        (1 << 0)
#  define X86_64_CPUID_01_SSSE3       (1 << 9)
#  define X86_64_CPUID_01_FMA         (1 << 12)
#  define X86_64_CPUID_01_PCID        (1 << 17)
#  define X86_64_CPUID_01_SSE41       (1 << 19)
#  define X86_64_CPUID_01_SSE42       (1 << 20)
#  define X86_64_CPUID_01_X2APIC      (1 << 21)
#  define X86_64_CPUID_01_TSCDEA      (1 << 24)
#  define X86_64_CPUID_01_XSAVE       (1 << 26)
#  define X86_64_CPUID_01_AVX         (1 << 28)
#  define X86_64_CPUID_01_RDRAND      (1 << 30)
#  define X86_64_CPUID_01_APICID(ebx) ((ebx) >> 24)
#define X86_64_CPUID_EXTCAP           0x07
#  define X86_64_CPUID_07_AVX2        (1 << 5)
#  define X86_64_CPUID_07_AVX512F     (1 << 16)
#  define X86_64_CPUID_07_AVX512DQ    (1 << 17)
#  define X86_64_CPUID_07_SMAP        (1 << 20)
#  define X86_64_CPUID_07_AVX512IFMA  (1 << 21)
#  define X86_64_CPUID_07_CLWB        (1 << 24)
#  define X86_64_CPUID_07_AVX512PF    (1 << 26)
#  define X86_64_CPUID_07_AVX512ER    (1 << 27)
#  define X86_64_CPUID_07_AVX512CD    (1 << 28)
#  define X86_64_CPUID_07_AVX512BW    (1 << 30)
#  define X86_64_CPUID_07_AVX512VL    (1 << 31)
#define X86_64_CPUID_XSAVE            0x0d
#define X86_64_CPUID_TSC              0x15
#define X86_64_CPUID_EXTINFO          0x80000001
#  define X86_64_CPUID_EXTINFO_RDTSCP (1 << 27)

/* MSR Definitions */

#define MSR_FS_BASE             0xc0000100 /* 64bit FS base */

#define MSR_EFER                0xc0000080
#  define EFER_LME              0x00000100

#define MSR_MTRR_DEF_TYPE       0x000002ff
#  define MTRR_ENABLE           0x00000800

#define MSR_IA32_TSC_DEADLINE   0x6e0

#define MSR_IA32_APIC_BASE      0x01b
#  define MSR_IA32_APIC_EN      0x800
#  define MSR_IA32_APIC_X2APIC  0x400
#  define MSR_IA32_APIC_BSP     0x100

#define MSR_X2APIC_ID           0x802
#define MSR_X2APIC_VER          0x803
#define MSR_X2APIC_TPR          0x808
#define MSR_X2APIC_PPR          0x80a
#define MSR_X2APIC_EOI          0x80b
#define MSR_X2APIC_LDR          0x80d

#define MSR_X2APIC_SPIV         0x80f
#  define MSR_X2APIC_SPIV_EN    0x100

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
#  define MSR_X2APIC_ICR_INIT          0x00000500  /* INIT/RESET */
#  define MSR_X2APIC_ICR_STARTUP       0x00000600  /* Startup IPI */
#  define MSR_X2APIC_ICR_DELIVS        0x00001000  /* Delivery status */
#  define MSR_X2APIC_ICR_ASSERT        0x00004000  /* Assert interrupt (vs deassert) */
#  define MSR_X2APIC_ICR_DEASSERT      0x00000000
#  define MSR_X2APIC_ICR_LEVEL         0x00008000  /* Level triggered */
#  define MSR_X2APIC_ICR_BCAST         0x00080000  /* Send to all APICs, including self. */
#  define MSR_X2APIC_ICR_OTHERS        0x000c0000  /* Send to all APICs, excluding self. */
#  define MSR_X2APIC_ICR_BUSY          0x00001000
#  define MSR_X2APIC_ICR_FIXED         0x00000000
#  define MSR_X2APIC_DESTINATION(d)    ((d) << 32)
#define MSR_X2APIC_LVTT         0x832
#  define MSR_X2APIC_LVTT_X1           0x0000000B  /* divide counts by 1 */
#  define MSR_X2APIC_LVTT_PERIODIC     0x00020000  /* Periodic */
#  define MSR_X2APIC_LVTT_TSC_DEADLINE 0x00040000  /* Enable TSC DEADLINE One-shot timer */
#define MSR_X2APIC_LVTTHER      0x833
#define MSR_X2APIC_LVTPMR       0x834
#define MSR_X2APIC_LINT0        0x835
#define MSR_X2APIC_LINT1        0x836
#define MSR_X2APIC_LERR         0x837
#  define MSR_X2APIC_MASKED            0x00010000  /* Interrupt masked */
#define MSR_X2APIC_TMICT        0x838
#define MSR_X2APIC_TMCCT        0x839
#define MSR_X2APIC_TDCR         0x83e
#define MSR_IA32_XSS            0xda0

/* IOAPIC related Definitions */

#define IOAPIC_BASE             0xfec00000
#define IOAPIC_REG_INDEX        0x00
#define IOAPIC_REG_DATA         0x10
#  define IOAPIC_REG_ID         0x00       /* Register index: ID */
#  define IOAPIC_REG_VER        0x01       /* Register index: version */
#  define IOAPIC_REG_TABLE      0x10       /* Redirection table base */
#  define IOAPIC_PIN_DISABLE    (1 << 16)  /* Disable */

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

#define BITS_PER_LONG          64

/* Interrupt Stack Table size */

#define X86_IST_SIZE           104
#define X86_TSS_SIZE           (104 + 8)

/* Reset Control Register (RST_CNT) */

#define X86_RST_CNT_REG        0xcf9
#  define X86_RST_CNT_SYS_RST  0x02
#  define X86_RST_CNT_CPU_RST  0x04
#  define X86_RST_CNT_FULL_RST 0x08

/* XSAVE state component bitmap */

#define X86_XSAVE_X87           (1 << 0)  /* Bit 0: X87 state */
#define X86_XSAVE_SSE           (1 << 1)  /* Bit 1: SSE state (512 bytes) */
#define X86_XSAVE_AVX           (1 << 2)  /* Bit 2: AVX state (256 bytes) */
#define X86_XSAVE_MPX_BNDREGS   (1 << 3)  /* Bit 3: MPX BNDREGS (64 bytes) */
#define X86_XSAVE_MPX_BNDCSR    (1 << 4)  /* Bit 4: MPX BNDCSR (16 bytes) */
#define X86_XSAVE_AVX512_OPMASK (1 << 5)  /* Bit 5: AVX-512 opmask (64 bytes) */
#define X86_XSAVE_AVX512_HI256  (1 << 6)  /* Bit 6: AVX-512 ZMM_Hi256 (512 bytes) */
#define X86_XSAVE_AVX512_HI16   (1 << 7)  /* Bit 7: AVX-512 Hi16_ZMM (1024 bytes) */
#define X86_XSAVE_PT            (1 << 8)  /* Bit 8: PT (72 bytes) */
#define X86_XSAVE_PKRU          (1 << 9)  /* Bit 9: PKRU (4 bytes) */
#define X86_XSAVE_PASID         (1 << 10) /* Bit 10: PASID state */
#define X86_XSAVE_CET_U         (1 << 11) /* Bit 11: CET_U state */
#define X86_XSAVE_CET_S         (1 << 12) /* Bit 12: CET_S state */
#define X86_XSAVE_HDC           (1 << 13) /* Bit 13: HDC */
#define X86_XSAVE_UINTR         (1 << 14) /* Bit 14: UINTR state */
#define X86_XSAVE_LBR           (1 << 15) /* Bit 15: LBR state */
#define X86_XSAVE_HWP           (1 << 16) /* Bit 16: HWP state */
#define X86_XSAVE_AMX_TILECFG   (1 << 17) /* Bit 17: AMX TILECFG state (64 bytes) */
#define X86_XSAVE_AMX_TILEDATA  (1 << 18) /* Bit 18: AMX TILEDATA state (8192 bytes) */

/* XSAVE area size */

#define XSAVE_LEGACY_SIZE       (512) /* X87 + SSE */
#define XSAVE_HEADER_SIZE       (64)  /* XSAVE header */
#define XSAVE_AVX_SIZE          (256)
#define XSAVE_MXP_BNDREGS_SIZE  (64)
#define XSAVE_MXP_BNDCSR_SIZE   (16)
#define XSAVE_AVX512OPMASK_SIZE (64)
#define XSAVE_AVX512HI256_SIZE  (512)
#define XSAVE_AVX512HI16_SIZE   (1024)
#define XSAVE_PT_SIZE           (72)
#define XSAVE_PKRU_SIZE         (4)
#define XSAVE_HDC_SIZE          (8)

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
  uint8_t  AC:1;                 /* 1: CPU accessed this segment */
  uint8_t  RW:1;                 /* 1: Data Segment 0: Code Segment */
  uint8_t  DC:1;                 /* Direction bit/Conforming bit.  */
  uint8_t  EX:1;                 /* 1: Segment can be executed  */
  uint8_t  S:1;                  /* S: 0:TSS 1:Code/Data Segment */
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
  uint16_t reserved4;            /* reserved */
  uint16_t IOPB_OFFSET;          /* IOPB_offset */
} end_packed_struct;

/* TSS */

begin_packed_struct struct tss_s
{
  struct ist_s  ist;    /* IST  */
  void         *cpu;    /* CPU private data */
} end_packed_struct;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These are defined in intel64_head.S */

extern volatile uint8_t g_pdpt_low;
extern volatile uint8_t g_pd_low;
extern volatile uint8_t g_pt_low;

extern volatile uint8_t g_ist64_low;
extern volatile uint8_t g_gdt64_low;
extern volatile uint8_t g_gdt64_ist_low;
extern volatile uint8_t g_gdt64_low_end;

/* The actual address of the page table and gdt/ist after mapping the kernel
 * in high address
 */

extern volatile uint64_t *g_pdpt;
extern volatile uint64_t *g_pd;
extern volatile uint64_t *g_pt;

extern volatile struct ist_s       *g_ist64;
extern volatile struct gdt_entry_s *g_gdt64;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int up_map_region(void *base, size_t size, int flags);
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
