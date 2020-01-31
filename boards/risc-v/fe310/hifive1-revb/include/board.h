/****************************************************************************
 * boards/risc-v/fe310/hifive1-revb/include/board.h
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
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

#ifndef __BOARDS_RISCV_FE310_HIFIVE_REVB_INCLUDE_BOARD_H
#define __BOARDS_RISCV_FE310_HIFIVE_REVB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUM_BUTTONS       1

#define LED_STARTED       0  /* N/A */
#define LED_HEAPALLOCATE  1  /* N/A */
#define LED_IRQSENABLED   2  /* N/A */
#define LED_STACKCREATED  3  /* N/A */
#define LED_INIRQ         4  /* N/A */
#define LED_SIGNAL        5  /* N/A */
#define LED_ASSERTION     6  /* N/A */
#define LED_PANIC         7  /* N/A */
#define LED_CPU           8  /* LED */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: fe310_boardinitialize
 ****************************************************************************/

void fe310_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISC-V_FE310_HIFIVE-REVB_INCLUDE_BOARD_H */
