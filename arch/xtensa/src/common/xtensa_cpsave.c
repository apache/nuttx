/****************************************************************************
 * arch/xtensa/src/common/xtensa_cpsave.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <arch/chip/core-isa.h>

#include "xtensa.h"

#if XCHAL_CP_NUM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_coproc_savestate
 *
 * Description:
 *   If there is a current thread and it has a coprocessor state save area,
 *   then save all callee-saved state into this area. xtensa_coproc_savestate()
 *   is simply a C wrapper around the assembly language call to
 *   _xtensa_coproc_savestate.
 *
 * Entry Conditions:
 *   - The thread being switched out is still the current thread.
 *   - CPENABLE state reflects which coprocessors are active.
 *
 * Exit conditions:
 *   - All necessary CP callee-saved state has been saved.
 *
 ****************************************************************************/

void xtensa_coproc_savestate(struct tcb_s *tcb)
{
  uint32_t cpsave = (uint32_t)((uintptr_t)&tcp->xcp.cpsave);

    __asm__ __volatile__
    (
      "mov a2, %0\n"
      "call0 _xtensa_coproc_savestate\n"
      :
      : "r" (cpsave)
      : "a0", "a2", "a3", "a4", "a5", "a6", "a7", "a13", "a14", "a15"
    )
}

/****************************************************************************
 * Name: xtensa_coproc_restorestate
 *
 * Description:
 *   Restore any callee-saved coprocessor state for the incoming thread.
 *   xtensa_coproc_restorestate() is simply a C wrapper around the assembly
 *   language call to _xtensa_coproc_restorestate.
 *
 * Entry Conditions:
 *   - CPENABLE is set up correctly for all required coprocessors.
 *
 * Exit conditions:
 *   - All necessary CP callee-saved state has been restored.
 *   - CPENABLE - unchanged.
 *
 ****************************************************************************/

void xtensa_coproc_restorestate(struct tcb_s *tcb)
{
  uint32_t cpsave = (uint32_t)((uintptr_t)&tcp->xcp.cpsave);

    __asm__ __volatile__
    (
      "mov a2, %0\n"
      "mov a3, %1\n"
      "call0 _xtensa_coproc_restorestate\n"
      :
      : "r" (cpmask) "r" (cpsave)
      : "a0", "a2", "a3", "a4", "a5", "a6", "a7", "a13", "a14", "a15"
    )
}

#endif /* XCHAL_CP_NUM */
