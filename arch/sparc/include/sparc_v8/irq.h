/****************************************************************************
 * arch/sparc/include/sparc_v8/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_SPARC_INCLUDE_SPARC_V8_IRQ_H
#define __ARCH_SPARC_INCLUDE_SPARC_V8_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The global pointer (GP) does not need to be saved in the "normal," flat
 * NuttX build.  However, it would be necessary to save the GP if this is
 * a KERNEL build or if NXFLAT is supported.
 */

#undef SPARC_V8_SAVE_GP
#if defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NXFLAT)
#  define SPARC_V8_SAVE_GP 1
#endif

/* If this is a kernel build, how many nested system calls should support? */

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 2
#endif

/* Register save state structure ********************************************/

#define REG_R1              (3)   /* R1 */
#define REG_R2              (4)   /* R2 */
#define REG_R3              (5)   /* R3 */
#define REG_R4              (6)   /* R4 */
#define REG_R5              (7)   /* R5 */
#define REG_R6              (8)   /* R6 */
#define REG_R7              (9)   /* R7 */

#define REG_R8              (10)  /* R8 */
#define REG_R9              (11)  /* R9 */
#define REG_R10             (12)  /* R10 */
#define REG_R11             (13)  /* R11 */
#define REG_R12             (14)  /* R12 */
#define REG_R13             (15)  /* R13 */
#define REG_R14             (16)  /* R14 */
#define REG_R15             (17)  /* R15 */

#define REG_R16             (52)  /* R16 */
#define REG_R17             (53)  /* R17 */
#define REG_R18             (54)  /* R18 */
#define REG_R19             (55)  /* R19 */
#define REG_R20             (56)  /* R20 */
#define REG_R21             (57)  /* R21 */
#define REG_R22             (58)  /* R22 */
#define REG_R23             (59)  /* R23 */

#define REG_R24             (60)  /* R24 */
#define REG_R25             (61)  /* R25 */
#define REG_R26             (62)  /* R26 */
#define REG_R27             (63)  /* R27 */
#define REG_R28             (64)  /* R28 */
#define REG_R29             (65)  /* R29 */
#define REG_R30             (66)  /* R30 */
#define REG_R31             (67)  /* R31 */

#define REG_R32             (0)   /* R32 */
#define REG_R33             (1)   /* R33 */
#define REG_R34             (2)   /* R34 */

#define REG_R35             (18)   /* R35 */
#define REG_R36             (19)   /* R36 */

#define XCPTCONTEXT_REGS    (52)
/* Alternate register names *************************************************/

/* %psr: processor status register */
#define REG_PSR             (0)

/* %pc: pc register */
#define REG_PC              (1)

/* %npc: npc register */
#define REG_NPC             (2)

/* %g1: global 1 */
#define REG_G1              (3)

/* %g2: global 2 (reserved for application) */
#define REG_G2              (4)

/* %g3: global 3 (reserved for application) */
#define REG_G3              (5)

/* %g4: global 4 (reserved for application) */
#define REG_G4              (6)

/* %g5: global 5 (reserved for system) */
#define REG_G5              (7)

/* %g6: global 5 (reserved for system) */
#define REG_G6              (8)

/* %g7: global 5 (reserved for system) */
#define REG_G7              (9)

/* %i0: incoming param 0, outgoing return value */
#define REG_I0              (10)

/* %i1: incoming param 1 */
#define REG_I1              (11)

/* %i2: incoming param 2 */
#define REG_I2              (12)

/* %i3: incoming param 3 */
#define REG_I3              (13)

/* %i4: incoming param 4 */
#define REG_I4              (14)

/* %i5: incoming param 5 */
#define REG_I5              (15)

/* %i6: frame pointer */
#define REG_I6              (16)

/* %i7: return address - 8 */
#define REG_I7              (17)

#define REG_Y               (18)
#define REG_FSR             (19)

/* %l0: loacal 0 */
#define REG_L0              (52)

/* %l1: loacal 1 */
#define REG_L1              (53)

/* %l2: loacal 2 */
#define REG_L2              (54)

/* %l3: loacal 3 */
#define REG_L3              (55)

/* %l4: loacal 4 */
#define REG_L4              (56)

/* %l5: loacal 5 */
#define REG_L5              (57)

/* %l6: loacal 6 */
#define REG_L6              (58)

/* %l7: loacal 7 */
#define REG_L7              (59)

/* %o0: outgoing param 0, incoming return value */
#define REG_O0              (60)

/* %o1: outgoing param 1 */
#define REG_O1              (61)

/* %o2: outgoing param 2 */
#define REG_O2              (62)

/* %o3: outgoing param 3 */
#define REG_O3              (63)

/* %o4: outgoing param 4 */
#define REG_O4              (64)

/* %o5: outgoing param 5 */
#define REG_O5              (65)

/* %o6: stack pointer */
#define REG_O6              (66)

/* %o7: address of call instruction, temporary */
#define REG_O7              (67)

/* SPARC Software Trap number definitions */
#define SPARC_SWTRAP_SYSCALL 0
#define SPARC_SWTRAP_IRQDIS  9
#define SPARC_SWTRAP_IRQEN   10
/**
 * PSR masks and starting bit positions
 *
 * NOTE: Reserved bits are ignored.
 */

#define SPARC_PSR_CWP_MASK  0x00000007   /* bits  0 -  4 */

/** This constant is a mask for the ET bits in the PSR. */
#define SPARC_PSR_ET_MASK   0x00000020   /* bit   5 */

/** This constant is a mask for the PS bits in the PSR. */
#define SPARC_PSR_PS_MASK   0x00000040   /* bit   6 */

/** This constant is a mask for the S bits in the PSR. */
#define SPARC_PSR_S_MASK    0x00000080   /* bit   7 */

/** This constant is a mask for the PIL bits in the PSR. */
#define SPARC_PSR_PIL_MASK  0x00000F00   /* bits  8 - 11 */

/** This constant is a mask for the EF bits in the PSR. */
#define SPARC_PSR_EF_MASK   0x00001000   /* bit  12 */

/** This constant is a mask for the EC bits in the PSR. */
#define SPARC_PSR_EC_MASK   0x00002000   /* bit  13 */

/** This constant is a mask for the ICC bits in the PSR. */
#define SPARC_PSR_ICC_MASK  0x00F00000   /* bits 20 - 23 */

/** This constant is a mask for the VER bits in the PSR. */
#define SPARC_PSR_VER_MASK  0x0F000000   /* bits 24 - 27 */

/** This constant is a mask for the IMPL bits in the PSR. */
#define SPARC_PSR_IMPL_MASK 0xF0000000   /* bits 28 - 31 */

/** This constant is the starting bit position of the CWP in the PSR. */
#define SPARC_PSR_CWP_BIT_POSITION   0   /* bits  0 -  4 */

/** This constant is the starting bit position of the ET in the PSR. */
#define SPARC_PSR_ET_BIT_POSITION    5   /* bit   5 */

/** This constant is the starting bit position of the PS in the PSR. */
#define SPARC_PSR_PS_BIT_POSITION    6   /* bit   6 */

/** This constant is the starting bit position of the S in the PSR. */
#define SPARC_PSR_S_BIT_POSITION     7   /* bit   7 */

/** This constant is the starting bit position of the PIL in the PSR. */
#define SPARC_PSR_PIL_BIT_POSITION   8   /* bits  8 - 11 */

/** This constant is the starting bit position of the EF in the PSR. */
#define SPARC_PSR_EF_BIT_POSITION   12   /* bit  12 */

/** This constant is the starting bit position of the EC in the PSR. */
#define SPARC_PSR_EC_BIT_POSITION   13   /* bit  13 */

/** This constant is the starting bit position of the ICC in the PSR. */
#define SPARC_PSR_ICC_BIT_POSITION  20   /* bits 20 - 23 */

/** This constant is the starting bit position of the VER in the PSR. */
#define SPARC_PSR_VER_BIT_POSITION  24   /* bits 24 - 27 */

/** This constant is the starting bit position of the IMPL in the PSR. */
#define SPARC_PSR_IMPL_BIT_POSITION 28   /* bits 28 - 31 */

#define SPARC_NUMBER_OF_REGISTER_WINDOWS      8

#define CPU_STACK_FRAME_L0_OFFSET             0x00
#define CPU_STACK_FRAME_L2_OFFSET             0x08
#define CPU_STACK_FRAME_L4_OFFSET             0x10
#define CPU_STACK_FRAME_L6_OFFSET             0x18
#define CPU_STACK_FRAME_I0_OFFSET             0x20
#define CPU_STACK_FRAME_I2_OFFSET             0x28
#define CPU_STACK_FRAME_I4_OFFSET             0x30
#define CPU_STACK_FRAME_I6_FP_OFFSET          0x38
#define CPU_STRUCTURE_RETURN_ADDRESS_OFFSET   0x40
#define CPU_STACK_FRAME_SAVED_ARG0_OFFSET     0x44
#define CPU_STACK_FRAME_SAVED_ARG1_OFFSET     0x48
#define CPU_STACK_FRAME_SAVED_ARG2_OFFSET     0x4c
#define CPU_STACK_FRAME_SAVED_ARG3_OFFSET     0x50
#define CPU_STACK_FRAME_SAVED_ARG4_OFFSET     0x54
#define CPU_STACK_FRAME_SAVED_ARG5_OFFSET     0x58
#define CPU_STACK_FRAME_PAD0_OFFSET           0x5c

#define CPU_MINIMUM_STACK_FRAME_SIZE          0x60
#define ISF_STACK_FRAME_OFFSET 		      0x00

#define ISF_PSR_OFFSET         (CPU_MINIMUM_STACK_FRAME_SIZE + 0x00)
#define ISF_PC_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x04)
#define ISF_NPC_OFFSET         (CPU_MINIMUM_STACK_FRAME_SIZE + 0x08)
#define ISF_G1_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x0c)
#define ISF_G2_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x10)
#define ISF_G4_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x18)
#define ISF_G6_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x20)
#define ISF_I0_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x28)
#define ISF_I2_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x30)
#define ISF_I4_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x38)
#define ISF_I6_FP_OFFSET       (CPU_MINIMUM_STACK_FRAME_SIZE + 0x40)
#define ISF_Y_OFFSET           (CPU_MINIMUM_STACK_FRAME_SIZE + 0x48)

#define ISF_FSR_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x4c)

#define ISF_F0_OFFSET           (CPU_MINIMUM_STACK_FRAME_SIZE + 0x50)
#define ISF_F2_OFFSET           (CPU_MINIMUM_STACK_FRAME_SIZE + 0x58)
#define ISF_F4_OFFSET           (CPU_MINIMUM_STACK_FRAME_SIZE + 0x60)
#define ISF_F6_OFFSET           (CPU_MINIMUM_STACK_FRAME_SIZE + 0x68)

#define ISF_F8_OFFSET           (CPU_MINIMUM_STACK_FRAME_SIZE + 0x70)
#define ISF_F10_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x78)
#define ISF_F12_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x80)
#define ISF_F14_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x88)

#define ISF_F16_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x90)
#define ISF_F18_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0x98)
#define ISF_F20_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0xa0)
#define ISF_F22_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0xa8)

#define ISF_F24_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0xb0)
#define ISF_F26_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0xb8)
#define ISF_F28_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0xc0)
#define ISF_F30_OFFSET          (CPU_MINIMUM_STACK_FRAME_SIZE + 0xc8)

#define CONTEXT_CONTROL_INTERRUPT_FRAME_SIZE (CPU_MINIMUM_STACK_FRAME_SIZE + 0x50 + 32*4)

/*  Offsets of fields with Context_Control for assembly routines. */

/** This macro defines an offset into the context for use in assembly. */
#define PSR_OFFSET   0x00

/** This macro defines an offset into the context for use in assembly. */
#define PC_OFFSET    0x04
/** This macro defines an offset into the context for use in assembly. */
#define NPC_OFFSET   0x08

/** This macro defines an offset into the context for use in assembly. */
#define G1_OFFSET    0x0C
/** This macro defines an offset into the context for use in assembly. */
#define G2_OFFSET    0x10
/** This macro defines an offset into the context for use in assembly. */
#define G3_OFFSET    0x14
/** This macro defines an offset into the context for use in assembly. */
#define G4_OFFSET    0x18
/** This macro defines an offset into the context for use in assembly. */
#define G5_OFFSET    0x1C
/** This macro defines an offset into the context for use in assembly. */
#define G6_OFFSET    0x20
/** This macro defines an offset into the context for use in assembly. */
#define G7_OFFSET    0x24

/** This macro defines an offset into the context for use in assembly. */
#define O0_OFFSET    0x28
/** This macro defines an offset into the context for use in assembly. */
#define O1_OFFSET    0x2C
/** This macro defines an offset into the context for use in assembly. */
#define O2_OFFSET    0x30
/** This macro defines an offset into the context for use in assembly. */
#define O3_OFFSET    0x34
/** This macro defines an offset into the context for use in assembly. */
#define O4_OFFSET    0x38
/** This macro defines an offset into the context for use in assembly. */
#define O5_OFFSET    0x3C
/** This macro defines an offset into the context for use in assembly. */
#define O6_SP_OFFSET 0x40
/** This macro defines an offset into the context for use in assembly. */
#define O7_OFFSET    0x44

#define Y_OFFSET     0x48
#define FSR_OFFSET   0x4c

#define F0_OFFSET    0x50
#define F2_OFFSET    0x58
#define F4_OFFSET    0x60
#define F6_OFFSET    0x68

#define F8_OFFSET    0x70
#define F10_OFFSET   0x78
#define F12_OFFSET   0x80
#define F14_OFFSET   0x88

#define F16_OFFSET   0x90
#define F18_OFFSET   0x98
#define F20_OFFSET   0xa0
#define F22_OFFSET   0xa8

#define F24_OFFSET   0xb0
#define F26_OFFSET   0xb8
#define F28_OFFSET   0xc0
#define F30_OFFSET   0xc8

#if ( SPARC_HAS_FPU == 1 )
  /**
   * @brief Offset of the CPU_Per_CPU_control::fsr field relative to the
   * Per_CPU_Control begin.
   */
  #define SPARC_PER_CPU_FSR_OFFSET 4
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This structure represents the return state from a system call */

#ifdef CONFIG_BUILD_KERNEL
struct xcpt_syscall_s
{
  uint32_t sysreturn;   /* The return PC */
};
#endif

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

struct xcptcontext
{
  /* The following function pointer is non-NULL if there are pending signals
   * to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These additional register save locations are used to implement the
   * signal delivery trampoline.
   *
   * REVISIT:  Because there is only one copy of these save areas,
   * only a single signal handler can be active.  This precludes
   * queuing of signal actions.  As a result, signals received while
   * another signal handler is executing will be ignored!
   */

  uint32_t saved_pc;     /* Trampoline PC */
  uint32_t saved_npc;    /* Trampoline nPC */
  uint32_t saved_status; /* Status with interrupts disabled. */

#ifdef CONFIG_BUILD_KERNEL
  /* This is the saved address to use when returning from a user-space
   * signal handler.
   */

  uint32_t sigreturn;

#endif

#ifdef CONFIG_BUILD_KERNEL
  /* The following array holds information needed to return from each nested
   * system call.
   */

  uint8_t nsyscalls;
  struct xcpt_syscall_s syscall[CONFIG_SYS_NNEST];

#endif

  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS]
  __attribute__((aligned(8)));
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/**
 * @brief Macro to set the PSR.
 *
 * This macro sets the PSR register to the value in @a _psr.
 */
#define sparc_set_psr( _psr ) \
  do { \
    __asm__ volatile ( "mov  %0, %%psr " : "=r" ((_psr)) : "0" ((_psr)) ); \
    nop(); \
    nop(); \
    nop(); \
  } while ( 0 )

/**
 * @brief Macro to obtain the PSR.
 *
 * This macro returns the current contents of the PSR register in @a _psr.
 */
#define sparc_get_psr( _psr ) \
  do { \
     (_psr) = 0; \
     __asm__ volatile( "rd %%psr, %0" :  "=r" (_psr) : "0" (_psr) ); \
  } while ( 0 )

/**
 * This macro is a standard nop instruction.
 */
#define nop() \
  do { \
    __asm__ volatile ( "nop" ); \
  } while ( 0 )

/**
 * @brief Macro to obtain the TBR.
 *
 * This macro returns the current contents of the TBR register in @a _tbr.
 */
#define sparc_get_tbr( _tbr ) \
  do { \
     (_tbr) = 0; /* to avoid unitialized warnings */ \
     __asm__ volatile( "rd %%tbr, %0" :  "=r" (_tbr) : "0" (_tbr) ); \
  } while ( 0 )

/**
 * @brief Macro to set the TBR.
 *
 * This macro sets the TBR register to the value in @a _tbr.
 */
#define sparc_set_tbr( _tbr ) \
  do { \
     __asm__ volatile( "wr %0, 0, %%tbr" :  "=r" (_tbr) : "0" (_tbr) ); \
  } while ( 0 )

/**
 * @brief Macro to obtain the WIM.
 *
 * This macro returns the current contents of the WIM field in @a _wim.
 */
#define sparc_get_wim( _wim ) \
  do { \
    __asm__ volatile( "rd %%wim, %0" :  "=r" (_wim) : "0" (_wim) ); \
  } while ( 0 )

/**
 * @brief Macro to set the WIM.
 *
 * This macro sets the WIM field to the value in @a _wim.
 */
#define sparc_set_wim( _wim ) \
  do { \
    __asm__ volatile( "wr %0, %%wim" :  "=r" (_wim) : "0" (_wim) ); \
    nop(); \
    nop(); \
    nop(); \
  } while ( 0 )

/**
 * @brief Macro to obtain the Y register.
 *
 * This macro returns the current contents of the Y register in @a _y.
 */
#define sparc_get_y( _y ) \
  do { \
    __asm__ volatile( "rd %%y, %0" :  "=r" (_y) : "0" (_y) ); \
  } while ( 0 )

/**
 * @brief Macro to set the Y register.
 *
 * This macro sets the Y register to the value in @a _y.
 */
#define sparc_set_y( _y ) \
  do { \
    __asm__ volatile( "wr %0, %%y" :  "=r" (_y) : "0" (_y) ); \
  } while ( 0 )

/**
 * @brief Macro to obtain the asr17.
 *
 * This macro returns the current contents of the asr17 register in _asr17.
 */
#define sparc_get_asr17( _asr17 ) \
  do { \
     (_asr17) = 0; /* to avoid unitialized warnings */ \
     __asm__ volatile( "rd %%asr17, %0" :  "=r" (_asr17) : "0" (_asr17) ); \
  } while ( 0 )

/**
 * @brief SPARC disable processor interrupts.
 *
 * This method is invoked to disable all maskable interrupts.
 *
 * @return This method returns the entire PSR contents.
 */

static inline uint32_t sparc_disable_interrupts(void)
{
  register uint32_t psr __asm__("g1"); /* return value of trap handler */
  __asm__ volatile ("ta %1\n\t" : "=r" (psr) : "i" (SPARC_SWTRAP_IRQDIS));
  return psr;
}

/**
 * @brief SPARC enable processor interrupts.
 *
 * This method is invoked to enable all maskable interrupts.
 *
 * @param[in] psr is the PSR returned by @ref sparc_disable_interrupts.
 */

static inline void sparc_enable_interrupts(uint32_t psr)
{
  register uint32_t _psr __asm__("g1") = psr; /* input to trap handler */

  /* The trap instruction has a higher trap priority than the interrupts
   * according to "The SPARC Architecture Manual: Version 8", Table 7-1
   * "Exception and Interrupt Request Priority and tt Values".  Add a nop to
   * prevent a trap instruction right after the interrupt enable trap.
   */

  __asm__ volatile ("ta %0\nnop\n" :: "i" (SPARC_SWTRAP_IRQEN), "r" (_psr));
}

/**
 * @brief SPARC flash processor interrupts.
 *
 * This method is invoked to temporarily enable all maskable interrupts.
 *
 * @param[in] _psr is the PSR returned by @ref sparc_disable_interrupts.
 */

#define sparc_flash_interrupts( _psr ) \
  do { \
    sparc_enable_interrupts( (_psr) ); \
    _psr = sparc_disable_interrupts(); \
  } while ( 0 )

/**
 * @brief SPARC obtain interrupt level.
 *
 * This method is invoked to obtain the current interrupt disable level.
 *
 * @param[in] _level is the PSR returned by @ref sparc_disable_interrupts.
 */

#define sparc_get_interrupt_level( _level ) \
  do { \
    register uint32_t   _psr_level = 0; \
    \
    sparc_get_psr( _psr_level ); \
    (_level) = \
      (_psr_level & SPARC_PSR_PIL_MASK) >> SPARC_PSR_PIL_BIT_POSITION; \
  } while ( 0 )

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Save the current interrupt state and disable interrupts.
 *
 *   NOTE: This function should never be called from application code and,
 *   as a general rule unless you really know what you are doing, this
 *   function should not be called directly from operation system code :
 *   Typically, the wrapper functions, enter_critical_section() is probably
 *   what you really want.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Interrupt state prior to disabling interrupts.
 *
 ****************************************************************************/

irqstate_t up_irq_save(void);

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the previous interrupt state (i.e., the one previously returned
 *   by up_irq_save())
 *
 *   NOTE: This function should never be called from application code and,
 *   as a general rule unless you really know what you are doing, this
 *   function should not be called directly from operation system code :
 *   Typically, the wrapper functions, leave_critical_section() is probably
 *   what you really want.
 *
 * Input Parameters:
 *   state - The interrupt state to be restored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t irqtate);

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_irq_enable(void);

/****************************************************************************
 * Name: up_irq_disable
 *
 * Description:
 *   Disable interrupts
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   the entire PSR contents
 *
 ****************************************************************************/

uint32_t up_irq_disable(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY */
#endif /* __ARCH_SPARC_INCLUDE_SPARC_V8_IRQ_H */

