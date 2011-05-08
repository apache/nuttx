/****************************************************************************
 * arch/mips/src/mips32/cp0.h
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

#ifndef __ARCH_MIPS_SRC_MIPS32_CP0_H
#define __ARCH_MIPS_SRC_MIPS32_CP0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* CP0 Registers ************************************************************/

/* Register Number: 0 Sel: 0 Name: Index
 * Function: Index into the TLB array
 */

/* Register Number: 1 Sel: 0 Name: Random
 * Function: Randomly generated index into the TLB array
 */

/* Register Number: 2 Sel: 0 Name: EntryLo0
 * Function: Low-order portion of the TLB entry foreven-numbered virtual
 *           pages
 */

/* Register Number: 3 Sel: 0 Name: EntryLo1
 * Function: Low-order portion of the TLB entry forodd-numbered virtual
 *           pages
 */

/* Register Number: 4 Sel: 0 Name: Context
 * Function: Pointer to page table entry in memory
 */

/* Register Number: 5 Sel: 0 Name: PageMask
 * Function: Control for variable page size in TLB entries
 */

/* Register Number: 6 Sel: 0 Name: Wired
 * Function: Controls the number of fixed (“wired”) TLB entries
 */

/* Register Number: 7 Sel: all (Reserved for future extensions)
/* Register Number: 8 Sel: 0 Name: BadVAddr
 * Function: Reports the address for the most recent address-related
 *           exception
 */

/* Register Number: 9 Sel: 0 Name: Count
 * Function: Processor cycle count
 */

/* Register Number: 9 Sel: 6-7 (Available for implementation dependent user) */

/* Register Number: 10 Sel: 0 Name: EntryHi
 * Function: High-order portion of the TLB entry
 */

/* Register Number: 11 Sel: 0 Name: Compare
 * Function: Timer interrupt control
 */

/* Register Number: 11 Sel: 6-7 (Available for implementation dependent user) */

/* Register Number: 12 Sel: 0 Name: Status
 * Function: Processor status and control
 */

/* Register Number: 13 Sel: 0 Name: Cause
 * Function: Cause of last general exception
 */

/* Register Number: 14 Sel: 0 Name: EPC
 * Function: Program counter at last exception
 */

/* Register Number: 15 Sel: 0 Name: PRId
 * Function: Processor identification and revision
 */

/* Register Number: 16 Sel: 0 Name: Config
 * Function: Configuration register
 */

/* Register Number: 16 Sel: 1 Name: Config1
 * Function: Configuration register 1
 */

/* Register Number: 16 Sel: 2 Name: Config2
 * Function: Configuration register 2
 */

/* Register Number: 16 Sel: 3 Name: Config3
 * Function: Configuration register 3
 */

/* Register Number: 16 Sel: 6-7 (Available for implementation dependent use) */

/* Register Number: 17 Sel: 0 Name: LLAddr
 * Function: Load linked address
 */

/* Register Number: 18 Sel: 0-n Name: WatchLo
 * Function: Watchpoint addr
 */

/* Register Number: 19 Sel: 0-n Name: WatchHi
 * Function: Watchpoint control
 */

/* Register Number: 20 Sel: 0 Name: XContext
 * Function: in 64-bit implementations
 */

/* Register Number: 21 Sel: all (Reserved for future extensions) */

/* Register Number: 22 Sel: all Available for implementation dependent use) */

/* Register Number: 23 Sel: 0 Name: Debug
 * Function: EJTAG Debug register
 */

/* Register Number: 24 Sel: 0 Name: DEPC
 * Function: Program counter at last EJTAG debug exception
 */

/* Register Number: 25 Sel: 0-n Name: PerfCnt
 * Function: Performance counter interface
 */

/* Register Number: 26 Sel: 0 Name: ErrCtl
 * Function: Parity/ECC error control and status
 */

/* Register Number: 27 Sel: 0-3 Name: CacheErr
 * Function: Cache parity error control and status
 */

/* Register Number: 28 Sel: 0 Name: TagLo
 * Function: Low-order portion of cache tag interface
 */

/* Register Number: 29 Sel: 0 Name: TagHi
 * Function: High-order portion of cache tag interface
 */

/* Register Number: 30 Sel: 0 Name: ErrorEPC
 * Function: Program counter at last error
 */

/* Register Number: 31 Sel: 0 Name: DESAVE
 * Function: EJTAG debug exception save register
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_MIPS32_CP0_H */
