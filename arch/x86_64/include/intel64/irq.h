/****************************************************************************
 * arch/x86_64/include/intel64/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_X86_64_INCLUDE_INTEL64_IRQ_H
#define __ARCH_X86_64_INCLUDE_INTEL64_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  include <time.h>
#endif

#include <arch/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ISR and IRQ numbers */

#define ISR0     0 /* Division by zero exception */
#define ISR1     1 /* Debug exception */
#define ISR2     2 /* Non maskable interrupt */
#define ISR3     3 /* Breakpoint exception */
#define ISR4     4 /* 'Into detected overflow' */
#define ISR5     5 /* Out of bounds exception */
#define ISR6     6 /* Invalid opcode exception */
#define ISR7     7 /* No coprocessor exception */
#define ISR8     8 /* Double fault (pushes an error code) */
#define ISR9     9 /* Coprocessor segment overrun */
#define ISR10   10 /* Bad TSS (pushes an error code) */
#define ISR11   11 /* Segment not present (pushes an error code) */
#define ISR12   12 /* Stack fault (pushes an error code) */
#define ISR13   13 /* General protection fault (pushes an error code) */
#define ISR14   14 /* Page fault (pushes an error code) */
#define ISR15   15 /* Unknown interrupt exception */
#define ISR16   16 /* Coprocessor fault */
#define ISR17   17 /* Alignment check exception */
#define ISR18   18 /* Machine check exception */
#define ISR19   19 /* SIMD Float-Point Exception*/
#define ISR20   20 /* Virtualization Exception */
#define ISR21   21 /* Reserved */
#define ISR22   22 /* Reserved */
#define ISR23   23 /* Reserved */
#define ISR24   24 /* Reserved */
#define ISR25   25 /* Reserved */
#define ISR26   26 /* Reserved */
#define ISR27   27 /* Reserved */
#define ISR28   28 /* Reserved */
#define ISR29   29 /* Reserved */
#define ISR30   30 /* Security Exception */
#define ISR31   31 /* Reserved */

#define IRQ0    32 /* System timer (cannot be changed) */
#define IRQ1    33 /* Keyboard controller (cannot be changed) */
#define IRQ2    34 /* Cascaded signals from IRQs 8~15 */
#define IRQ3    35 /* Serial port controller for COM2/4 */
#define IRQ4    36 /* serial port controller for COM1/3 */
#define IRQ5    37 /* LPT port 2 or sound card */
#define IRQ6    38 /* Floppy disk controller */
#define IRQ7    39 /* LPT port 1 or sound card */
#define IRQ8    40 /* Real time clock (RTC) */
#define IRQ9    41 /* Open interrupt/available or SCSI host adapter */
#define IRQ10   42 /* Open interrupt/available or SCSI or NIC */
#define IRQ11   43 /* Open interrupt/available or SCSI or NIC */
#define IRQ12   44 /* Mouse on PS/2 connector */
#define IRQ13   45 /* Math coprocessor */
#define IRQ14   46 /* Primary ATA channel */
#define IRQ15   47 /* Secondary ATA channel */
#define IRQ16   48
#define IRQ17   49
#define IRQ18   50
#define IRQ19   51
#define IRQ20   52
#define IRQ21   53
#define IRQ22   54
#define IRQ23   55
#define IRQ24   56
#define IRQ25   57
#define IRQ26   58
#define IRQ27   59
#define IRQ28   60
#define IRQ29   61
#define IRQ30   62
#define IRQ31   63
#define IRQ32   64
#define IRQ33   65
#define IRQ34   66
#define IRQ35   67
#define IRQ36   68
#define IRQ37   69
#define IRQ38   70
#define IRQ39   71
#define IRQ40   72
#define IRQ41   73
#define IRQ42   74
#define IRQ43   75
#define IRQ44   76
#define IRQ45   77
#define IRQ46   78
#define IRQ47   79
#define IRQ48   80
#define IRQ49   81
#define IRQ50   82
#define IRQ51   83
#define IRQ52   84
#define IRQ53   85
#define IRQ54   86
#define IRQ55   87
#define IRQ56   88
#define IRQ57   89
#define IRQ58   90
#define IRQ59   91
#define IRQ60   92
#define IRQ61   93
#define IRQ62   94
#define IRQ63   95
#define IRQ64   96
#define IRQ65   97
#define IRQ66   98
#define IRQ67   99
#define IRQ68   100
#define IRQ69   101
#define IRQ70   102
#define IRQ71   103
#define IRQ72   104
#define IRQ73   105
#define IRQ74   106
#define IRQ75   107
#define IRQ76   108
#define IRQ77   109
#define IRQ78   110
#define IRQ79   111
#define IRQ80   112
#define IRQ81   113
#define IRQ82   114
#define IRQ83   115
#define IRQ84   116
#define IRQ85   117
#define IRQ86   118
#define IRQ87   119
#define IRQ88   120
#define IRQ89   121
#define IRQ90   122
#define IRQ91   123
#define IRQ92   124
#define IRQ93   125
#define IRQ94   126
#define IRQ95   127
#define IRQ96   128
#define IRQ97   129
#define IRQ98   130
#define IRQ99   131
#define IRQ100  132
#define IRQ101  133
#define IRQ102  134
#define IRQ103  135
#define IRQ104  136
#define IRQ105  137
#define IRQ106  138
#define IRQ107  139
#define IRQ108  140
#define IRQ109  141
#define IRQ110  142
#define IRQ111  143
#define IRQ112  144
#define IRQ113  145
#define IRQ114  146
#define IRQ115  147
#define IRQ116  148
#define IRQ117  149
#define IRQ118  150
#define IRQ119  151
#define IRQ120  152
#define IRQ121  153
#define IRQ122  154
#define IRQ123  155
#define IRQ124  156
#define IRQ125  157
#define IRQ126  158
#define IRQ127  159
#define IRQ128  160
#define IRQ129  161
#define IRQ130  162
#define IRQ131  163
#define IRQ132  164
#define IRQ133  165
#define IRQ134  166
#define IRQ135  167
#define IRQ136  168
#define IRQ137  169
#define IRQ138  170
#define IRQ139  171
#define IRQ140  172
#define IRQ141  173
#define IRQ142  174
#define IRQ143  175
#define IRQ144  176
#define IRQ145  177
#define IRQ146  178
#define IRQ147  179
#define IRQ148  180
#define IRQ149  181
#define IRQ150  182
#define IRQ151  183
#define IRQ152  184
#define IRQ153  185
#define IRQ154  186
#define IRQ155  187
#define IRQ156  188
#define IRQ157  189
#define IRQ158  190
#define IRQ159  191
#define IRQ160  192
#define IRQ161  193
#define IRQ162  194
#define IRQ163  195
#define IRQ164  196
#define IRQ165  197
#define IRQ166  198
#define IRQ167  199
#define IRQ168  200
#define IRQ169  201
#define IRQ170  202
#define IRQ171  203
#define IRQ172  204
#define IRQ173  205
#define IRQ174  206
#define IRQ175  207
#define IRQ176  208
#define IRQ177  209
#define IRQ178  210
#define IRQ179  211
#define IRQ180  212
#define IRQ181  213
#define IRQ182  214
#define IRQ183  215
#define IRQ184  216
#define IRQ185  217
#define IRQ186  218
#define IRQ187  219
#define IRQ188  220
#define IRQ189  221
#define IRQ190  222
#define IRQ191  223
#define IRQ192  224
#define IRQ193  225
#define IRQ194  226
#define IRQ195  227
#define IRQ196  228
#define IRQ197  229
#define IRQ198  230
#define IRQ199  231
#define IRQ200  232
#define IRQ201  233
#define IRQ202  234
#define IRQ203  235
#define IRQ204  236
#define IRQ205  237
#define IRQ206  238
#define IRQ207  239
#define IRQ208  240
#define IRQ209  241
#define IRQ210  242
#define IRQ211  243
#define IRQ212  244
#define IRQ213  245
#define IRQ214  246
#define IRQ215  247
#define IRQ216  248
#define IRQ217  249
#define IRQ218  250
#define IRQ219  251
#define IRQ220  252
#define IRQ221  253
#define IRQ222  254
#define IRQ223  255
#define IRQ224  256
#define IRQ225  257
#define IRQ226  258
#define IRQ227  259
#define IRQ228  260
#define IRQ229  261
#define IRQ230  262
#define IRQ231  263
#define IRQ232  264
#define IRQ233  265
#define IRQ234  266
#define IRQ235  267
#define IRQ236  268
#define IRQ237  269
#define IRQ238  270
#define IRQ239  271
#define IRQ240  272
#define IRQ241  273
#define IRQ242  274
#define IRQ243  275
#define IRQ244  276
#define IRQ245  277
#define IRQ246  278
#define IRQ247  279
#define IRQ248  280
#define IRQ249  281
#define IRQ250  282
#define IRQ251  283
#define IRQ252  284
#define IRQ253  285
#define IRQ254  286
#define IRQ255  287

#define NR_IRQS     288
#define MAX_NR_IRQS 255

#define IRQ_ERROR    51   /* APIC Error */
#define IRQ_SPURIOUS 0xff /* Spurious Interrupts */

/* Use legacy routing for HPET */

#define HPET0_IRQ    IRQ2
#define HPET1_IRQ    IRQ8

/* Use IRQ15 IRQ16 for SMP */

#define SMP_IPI_IRQ  IRQ15
#define SMP_IPI_ASYNC_IRQ  IRQ16

/* Common register save structure created by up_saveusercontext() and by
 * ISR/IRQ interrupt processing.
 */

#ifndef CONFIG_ARCH_X86_64_HAVE_XSAVE

/* Only legacy area if XSAVE not supported */

#  define XCPTCONTEXT_XMM_AREA_SIZE 512
#else

/* XSAVE state depneds on enabled features */

#  ifdef CONFIG_ARCH_X86_64_AVX
#    define XSTATE_AVX_STATE    X86_XSAVE_AVX
#    define XSTATE_AVX_SIZE     XSAVE_AVX_SIZE
#  else
#    define XSTATE_AVX_STATE    0
#    define XSTATE_AVX_SIZE     0
#  endif

#  ifdef CONFIG_ARCH_X86_64_AVX512
#    define XSTATE_AVX512_STATE (X86_XSAVE_AVX512_OPMASK |  \
                                 X86_XSAVE_AVX512_HI256 |   \
                                 X86_XSAVE_AVX512_HI16)
#    define XSTATE_AVX512_SIZE  (XSAVE_MXP_BNDREGS_SIZE +   \
                                 XSAVE_MXP_BNDCSR_SIZE +    \
                                 XSAVE_AVX512OPMASK_SIZE +  \
                                 XSAVE_AVX512HI256_SIZE +   \
                                 XSAVE_AVX512HI16_SIZE)
#  else
#    define XSTATE_AVX512_STATE 0
#    define XSTATE_AVX512_SIZE  0
#  endif

/* State component bitmap */

#  define XSAVE_STATE_COMPONENTS    (X86_XSAVE_X87 |    \
                                     X86_XSAVE_SSE |    \
                                     XSTATE_AVX_STATE | \
                                     XSTATE_AVX512_STATE)

/* Area for XSAVE - standard area format */

#  define XCPTCONTEXT_XMM_AREA_SIZE (XSAVE_LEGACY_SIZE +  \
                                     XSAVE_HEADER_SIZE +  \
                                     XSTATE_AVX_SIZE +    \
                                     XSTATE_AVX512_SIZE)
#endif

/* Align registers to 64-bytes */

#ifdef CONFIG_ARCH_X86_64_AVX512
#  define XMMAREA_REG_ALIGN (13)
#else
#  define XMMAREA_REG_ALIGN (7)
#endif

/* Register offset in XMMAREA */

#define XMMAREA_OFFSET     (XCPTCONTEXT_XMM_AREA_SIZE / 8)
#define XMMAREA_REG_OFFSET (XMMAREA_REG_ALIGN + XMMAREA_OFFSET)

/* Data segments */

#define REG_FS            (0 + XMMAREA_REG_OFFSET)  /* "    " "" "     " "" "       " "        " */
#define REG_GS            (1 + XMMAREA_REG_OFFSET)  /* "    " "" "     " "" "       " "        " */
#define REG_ES            (2 + XMMAREA_REG_OFFSET)  /* "    " "" "     " "" "       " "        " */
#define REG_DS            (3 + XMMAREA_REG_OFFSET)  /* Data segment selector */

/* Remaining regs */

#define REG_RAX           (4 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_RBX           (5 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_RBP           (6 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_R10           (7 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_R11           (8 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_R12           (9 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_R13          (10 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_R14          (11 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_R15          (12 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */

/* ABI calling convention */

#define REG_R9           (13 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_R8           (14 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_RCX          (15 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_RDX          (16 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_RSI          (17 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */
#define REG_RDI          (18 + XMMAREA_REG_OFFSET)  /* "   " "" "   " */

/* IRQ saved */

#define REG_ERRCODE      (19 + XMMAREA_REG_OFFSET)  /* Error code */
#define REG_RIP          (20 + XMMAREA_REG_OFFSET)  /* Pushed by process on interrupt processing */
#define REG_CS           (21 + XMMAREA_REG_OFFSET)  /* "    " "" "     " "" "       " "        " */
#define REG_RFLAGS       (22 + XMMAREA_REG_OFFSET)  /* "    " "" "     " "" "       " "        " */
#define REG_RSP          (23 + XMMAREA_REG_OFFSET)  /* "    " "" "     " "" "       " "        " */
#define REG_SS           (24 + XMMAREA_REG_OFFSET)  /* "    " "" "     " "" "       " "        " */

#define XMMAREA_REGS     (25)

/* NOTE 2: This is not really state data.  Rather, this is just a convenient
 *   way to pass parameters from the interrupt handler to C code.
 */

#define XCPTCONTEXT_REGS (XMMAREA_REGS + XMMAREA_REG_ALIGN + \
                          XCPTCONTEXT_XMM_AREA_SIZE / 8)

#define XCPTCONTEXT_SIZE (8 * XCPTCONTEXT_REGS)

/* Always align XCPTCONTEXT to 64-bytes to support XSAVE */

#define XCPTCONTEXT_ALIGN (64)

#define XCP_ALIGN_MASK    (XCPTCONTEXT_ALIGN - 1)
#define XCP_ALIGN_DOWN(a) ((a) & ~XCP_ALIGN_MASK)
#define XCP_ALIGN_UP(a)   (((a) + XCP_ALIGN_MASK) & ~XCP_ALIGN_MASK)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
enum ioapic_trigger_mode
{
  TRIGGER_RISING_EDGE = 0,
  TRIGGER_FALLING_EDGE = (1 << 13),
  TRIGGER_LEVEL_ACTIVE_HIGH = 1 << 15,
  TRIGGER_LEVEL_ACTIVE_LOW = (1 << 15) | (1 << 13),
};

/* This struct defines the way the registers are stored */

struct xcptcontext
{
  /* The following function pointer is non-zero if there are pending signals
   * to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of instruction pointer and EFLAGS used during
   * signal processing.
   */

  uint64_t saved_rip;
  uint64_t saved_rflags;
  uint64_t saved_rsp;

  /* Register save area - allocated from stack in up_initial_state() */

  uint64_t *regs;
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

static inline void setgdt(void *gdt, int size)
{
  struct gdt_ptr_s gdt_ptr;
  gdt_ptr.limit = size;
  gdt_ptr.base = (uintptr_t)gdt;

  __asm__ volatile ("lgdt %0"::"m"(gdt_ptr):"memory");
}

static inline void setidt(void *idt, int size)
{
  struct idt_ptr_s idt_ptr;
  idt_ptr.limit = size;
  idt_ptr.base = (uintptr_t)idt;

  __asm__ volatile ("lidt %0"::"m"(idt_ptr):"memory");
}

static inline uint64_t rdtscp(void)
{
  uint32_t lo;
  uint32_t hi;

  __asm__ volatile("rdtscp" : "=a" (lo), "=d" (hi)::"ecx", "memory");
  return (uint64_t)lo | (((uint64_t)hi) << 32);
}

static inline uint64_t rdtsc(void)
{
  uint32_t lo;
  uint32_t hi;

  __asm__ volatile("rdtsc" : "=a" (lo), "=d" (hi)::"memory");
  return (uint64_t)lo | (((uint64_t)hi) << 32);
}

static inline void set_pcid(uint64_t pcid)
{
  if (pcid < 4095)
    {
      __asm__ volatile("mov %%cr3, %%rbx; andq $-4096, %%rbx; or %0, "
                       "%%rbx; mov %%rbx, %%cr3;"
                       ::"g"(pcid):"memory", "rbx", "rax");
    }
}

static inline void set_cr3(uint64_t cr3)
{
  __asm__ volatile("mov %0, %%cr3" : "=rm"(cr3) : : "memory");
}

static inline uint64_t get_cr3(void)
{
  uint64_t cr3;
  __asm__ volatile("mov %%cr3, %0" : "=rm"(cr3) : : "memory");
  return cr3;
}

static inline uint64_t get_pml4(void)
{
  /* Aligned to a 4-KByte boundary */

  return get_cr3() & 0xfffffffffffff000;
}

static inline unsigned long read_msr(unsigned int msr)
{
  uint32_t low;
  uint32_t high;

  __asm__ volatile("rdmsr" : "=a" (low), "=d" (high) : "c" (msr));
  return low | ((unsigned long)high << 32);
}

static inline void write_msr(unsigned int msr, unsigned long val)
{
  __asm__ volatile("wrmsr"
                   : /* no output */
                   : "c" (msr), "a" (val), "d" (val >> 32)
                   : "memory");
}

static inline uint64_t read_fsbase(void)
{
  uint64_t val;
  __asm__ volatile("rdfsbase %0"
                   : "=r" (val)
                   : /* no output */
                   : "memory");

  return val;
}

static inline void write_fsbase(unsigned long val)
{
  __asm__ volatile("wrfsbase %0"
                   : /* no output */
                   : "r" (val)
                   : "memory");
}

static inline uint64_t read_gsbase(void)
{
  uint64_t val;
  __asm__ volatile("rdgsbase %0"
                   : "=r" (val)
                   : /* no output */
                   : "memory");

  return val;
}

static inline void write_gsbase(unsigned long val)
{
  __asm__ volatile("wrgsbase %0"
                   : /* no output */
                   : "r" (val)
                   : "memory");
}

/* Return stack pointer */

static inline uint64_t up_getsp(void)
{
  uint64_t regval;

  __asm__ volatile(
    "\tmovq %%rsp, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

/* Get segment registers */

static inline uint32_t up_getds(void)
{
  uint32_t regval;

  __asm__ volatile(
    "\tmov %%ds, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getcs(void)
{
  uint32_t regval;

  __asm__ volatile(
    "\tmov %%cs, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getss(void)
{
  uint32_t regval;

  __asm__ volatile(
    "\tmov %%ss, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getes(void)
{
  uint32_t regval;

  __asm__ volatile(
    "\tmov %%es, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getfs(void)
{
  uint32_t regval;

  __asm__ volatile(
    "\tmov %%fs, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getgs(void)
{
  uint32_t regval;

  __asm__ volatile(
    "\tmov %%gs, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Get the current FLAGS register contents */

static inline irqstate_t irqflags()
{
  irqstate_t flags;

  __asm__ volatile(
    "\tpushfq\n"
    "\tpopq %0\n"
    : "=rm" (flags)
    :
    : "memory");
  return flags;
}

/* Get a sample of the FLAGS register, determine if interrupts are disabled.
 * If the X86_FLAGS_IF is cleared by cli, then interrupts are disabled.  If
 * if the X86_FLAGS_IF is set by sti, then interrupts are enable.
 */

static inline bool up_irq_disabled(irqstate_t flags)
{
  return ((flags & X86_64_RFLAGS_IF) == 0);
}

static inline bool up_irq_enabled(irqstate_t flags)
{
  return ((flags & X86_64_RFLAGS_IF) != 0);
}

/* Disable interrupts unconditionally */

static inline void up_irq_disable(void)
{
  __asm__ volatile("cli": : :"memory");
}

/* Enable interrupts unconditionally */

static inline void up_irq_enable(void)
{
  __asm__ volatile("sti": : :"memory");
}

/* Disable interrupts, but return previous interrupt state */

static inline irqstate_t up_irq_save(void)
{
  irqstate_t flags = irqflags();
  up_irq_disable();
  return flags;
}

/* Conditionally disable interrupts */

static inline void up_irq_restore(irqstate_t flags)
{
  if (up_irq_enabled(flags))
    {
      up_irq_enable();
    }
}

static inline unsigned int up_apic_cpu_id(void)
{
  return read_msr(MSR_X2APIC_ID);
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void up_ioapic_pin_set_vector(unsigned int pin,
                              enum ioapic_trigger_mode trigger_mode,
                              unsigned int vector);

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
#endif /* __ARCH_X86_INCLUDE_I486_IRQ_H */

