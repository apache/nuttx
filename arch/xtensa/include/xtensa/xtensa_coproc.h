/****************************************************************************
 * arch/xtensa/include/xtensa/xtensa_coproc.h
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic originally provided by Cadence Design Systems Inc.
 *
 *   Copyright (c) 2006-2015 Cadence Design Systems Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_INCLUDE_XTENSA_XTENSA_COPROC_H
#define __ARCH_XTENSA_INCLUDE_XTENSA_XTENSA_COPROC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/esp32/core-isa.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if XCHAL_CP_NUM > 0

/* Align a value up/down to nearest n-byte boundary, where n is a power of 2. */

#define _CP_MASK(n)           ((n) - 1)
#define _CP_ALIGNUP(n,val)   (((val) + _CP_MASK(n)) & ~_CP_MASK(n))
#define _CP_ALIGNDOWN(n,val) ((val) & ~_CP_MASK(n))

/* A set of all co-processors */

#define XTENSA_CP_ALLSET     ((1 << XCHAL_CP_NUM) - 1)

/* CO-PROCESSOR STATE SAVE AREA FOR A THREAD
 *
 * NuttX provides an area per thread to save the state of co-processors when
 * that thread does not have control.  Co-processors are context-switched
 * lazily (on demand) only when a new thread uses a co-processor instruction,
 * otherwise a thread retains ownership of the co-processor even when it
 * loses control of the processor. An Xtensa co-processor exception is
 * triggered when any co-processor instruction is executed by a thread that
 * is not the owner, and the context switch of that co-processor is then
 * peformed by the handler. Ownership represents which thread's state is
 * currently in the co-processor.
 *
 * Co-processors may not be used by interrupt or exception handlers. If an
 * co-processor instruction is executed by an interrupt or exception handler,
 * the co-processor exception handler will trigger a kernel panic and freeze.
 * This restriction is introduced to reduce the overhead of saving and
 * restoring co-processor state (which can be quite large) and in particular
 * remove that overhead from interrupt handlers.
 *
 * The co-processor state save area may be in any convenient per-thread
 * location such as in the thread control block or above the thread stack
 * area. It need not be in the interrupt stack frame since interrupts don't
 * use co-processors.
 *
 * Along with the save area for each co-processor, two bitmasks with flags
 * per co-processor (laid out as in the CPENABLE reg) help manage context-
 * switching co-processors as efficiently as possible:
 *
 * XTENSA_CPENABLE
 *   The contents of a non-running thread's CPENABLE register.
 *   It represents the co-processors owned (and whose state is still needed)
 *   by the thread. When a thread is preempted, its CPENABLE is saved here.
 *   When a thread solicits a context-swtich, its CPENABLE is cleared - the
 *   compiler has saved the (caller-saved) co-proc state if it needs to.
 *   When a non-running thread loses ownership of a CP, its bit is cleared.
 *   When a thread runs, it's XTENSA_CPENABLE is loaded into the CPENABLE reg.
 *   Avoids co-processor exceptions when no change of ownership is needed.
 *
 * XTENSA_CPSTORED
 *   A bitmask with the same layout as CPENABLE, a bit per co-processor.
 *   Indicates whether the state of each co-processor is saved in the state
 *   save area. When the state of a thread is saved, only the state of co-procs
 *   still enabled in CPENABLE is saved. When the co-processor state is
 *   is restored, the state is only resotred for a co-processor if this bit
 *   is set.  This bist set is cleared after after co-processor state has
 *   been restored.
 *
 * XTENSA_CPASA
 *   Pointer to the aligned save area.  Allows it to be aligned more than
 *   the overall save area (which might only be stack-aligned or TCB-aligned).
 *   Especially relevant for Xtensa cores configured with a very large data
 *   path that requires alignment greater than 16 bytes (ABI stack alignment).
 */

/* Offsets of each coprocessor save area within the 'aligned save area': */

#define XTENSA_CP0_SA     0
#define XTENSA_CP1_SA     _CP_ALIGNUP(XCHAL_CP1_SA_ALIGN, XTENSA_CP0_SA + XCHAL_CP0_SA_SIZE)
#define XTENSA_CP2_SA     _CP_ALIGNUP(XCHAL_CP2_SA_ALIGN, XTENSA_CP1_SA + XCHAL_CP1_SA_SIZE)
#define XTENSA_CP3_SA     _CP_ALIGNUP(XCHAL_CP3_SA_ALIGN, XTENSA_CP2_SA + XCHAL_CP2_SA_SIZE)
#define XTENSA_CP4_SA     _CP_ALIGNUP(XCHAL_CP4_SA_ALIGN, XTENSA_CP3_SA + XCHAL_CP3_SA_SIZE)
#define XTENSA_CP5_SA     _CP_ALIGNUP(XCHAL_CP5_SA_ALIGN, XTENSA_CP4_SA + XCHAL_CP4_SA_SIZE)
#define XTENSA_CP6_SA     _CP_ALIGNUP(XCHAL_CP6_SA_ALIGN, XTENSA_CP5_SA + XCHAL_CP5_SA_SIZE)
#define XTENSA_CP7_SA     _CP_ALIGNUP(XCHAL_CP7_SA_ALIGN, XTENSA_CP6_SA + XCHAL_CP6_SA_SIZE)
#define XTENSA_CP_SA_SIZE _CP_ALIGNUP(16, XTENSA_CP7_SA + XCHAL_CP7_SA_SIZE)

/* Offsets within the overall save area: */

#define XTENSA_CPENABLE   0  /* (2 bytes) coprocessors active for this thread */
#define XTENSA_CPSTORED   2  /* (2 bytes) coprocessors saved for this thread */
#define XTENSA_CPASA      4  /* (4 bytes) ptr to aligned save area */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct xtensa_cpstate_s
{
  uint16_t cpenable;  /* (2 bytes) Co-processors active for this thread */
  uint16_t cpstored;  /* (2 bytes) Co-processors saved for this thread */
  uint32_t *cpasa;    /* (4 bytes) Pointer to aligned save area */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* Return the current value of the CPENABLE register */

static inline uint32_t xtensa_get_cpenable(void)
{
  uint32_t cpenable;

  __asm__ __volatile__
  (
    "\trsr %0, CPENABLE\n"
    : "=r"(cpenable)
  );

  return cpenable;
}

/* Set the value of the CPENABLE register */

static inline void xtensa_set_cpenable(uint32_t cpenable)
{
  __asm__ __volatile__
  (
    "\twsr %0, CPENABLE\n"
    "\trsync\n"
    : : "r"(cpenable)
  );
}

#endif /* __ASSEMBLY__ */
#endif /* #if XCHAL_CP_NUM > 0 */
#endif /* __ARCH_XTENSA_INCLUDE_XTENSA_XTENSA_COPROC_H */
