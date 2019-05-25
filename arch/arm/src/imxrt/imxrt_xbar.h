/**************************************************************************************************************************************************
 * arch/arm/src/imxrt/imxrt_abar.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 **************************************************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_XBAR_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_XBAR_H

/**************************************************************************************************************************************************
 * Included Files
 **************************************************************************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/imxrt_xbar.h"
#include "hardware/imxrt_memorymap.h"

/**************************************************************************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************************************************************************/

/* Cross Bars
 *
 * A cross bar is M muxes with I inputs
 *
 * There are 3 such cross bars switches:
 *
 *   | XBARA1  | XBARAB2  | XBARB3  |
 *   |---------+----------*---------|
 *   | M  | I  |  M  | I  | M  | I  |
 *   |132 | 88 |  16 | 64 | 16 | 64 |
 *
 *   Therefore there are M select fields that will be written to I values.
 *   The M fields are know as SELn, there are 2 selects fields per register.
 *
 *   A Input is wired to an output with a call to imxrt_xbar_connect(output, input)
 *
 */

/*    + ----------------- Which side of Mux Input or Output
 *    | ++++------------- XBAR  Index 0 - 2
 *    v vvvv  +++++++---- Index of input or output index
 * 000s xxxx  nnnnnnnn
 *
 * Where side is used to parameter check the passed value in output is an output
 * XBAR  is the index of the XBAR
 * input|output index - the index of the M mux (output) or I.
 *
 */

/* Input and Output Indexes  */

#define IMXRT_XBARA_IO_INDEX_SHIFTS                      0
#define IMXRT_XBARA_IO_INDEX_MASK                        (0xff << IMXRT_XBARA_IO_INDEX_SHIFTS)

/* Index for xbar addresses */

#define IMXRT_XBARA_INDEX_SHIFTS                         8
#define IMXRT_XBARA_INDEX_MASK                           (0xf << IMXRT_XBARA_INDEX_SHIFTS)
#  define IMXRT_XBARA1_INDEX                             (0 << IMXRT_XBARA_INDEX_SHIFTS)
#  define IMXRT_XBARA2_INDEX                             (1 << IMXRT_XBARA_INDEX_SHIFTS)
#  define IMXRT_XBARA3_INDEX                             (2 << IMXRT_XBARA_INDEX_SHIFTS)

/* Side of xbar  */

#define IMXRT_XBARA_SIDE_SHIFTS                          12
#define IMXRT_XBARA_SIDE_MASK                            (0x1 << IMXRT_XBARA_SIDE_SHIFTS)

#define XBAR_OUTPUT                                      (0x1 << IMXRT_XBARA_SIDE_SHIFTS)
#define XBAR_INPUT                                       (0x0 << IMXRT_XBARA_SIDE_SHIFTS)

/* xbar helpers */

#define IMXRT_XBARA1(side, select)                       ((uint16_t)((side) | IMXRT_XBARA1_INDEX | \
                                                         ((select) & 0xff) << IMXRT_XBARA_IO_INDEX_SHIFTS))
#define IMXRT_XBARA2(side, select)                       ((uint16_t)((side) | IMXRT_XBARA2_INDEX | \
                                                         ((select) & 0xff) << IMXRT_XBARA_IO_INDEX_SHIFTS))
#define IMXRT_XBARA3(side, select)                       ((uint16_t)((side) | IMXRT_XBARA3_INDEX | \
                                                         ((select) & 0xff) << IMXRT_XBARA_IO_INDEX_SHIFTS))

#define IMXRT_SEL(six)                                   ((six) & IMXRT_XBARA_IO_INDEX_MASK) >> IMXRT_XBARA_IO_INDEX_SHIFTS
#define IMXRT_XBAR(six)                                  ((six) & IMXRT_XBARA_INDEX_MASK) >> IMXRT_XBARA_INDEX_SHIFTS
#define IMXRT_SIDE(six)                                  ((six) & IMXRT_XBARA_SIDE_MASK)  >> IMXRT_XBARA_SIDE_SHIFTS

/* Collect correct XBAR definitions from chip file */
#include "hardware/imxrt_xbar.h"

/**************************************************************************************************************************************************
 * Public Functions
 **************************************************************************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/******************************************************************************************************************************************
 * Name: imxrt_xbar_connect
 *
 * Description:
 *   This function maps the input_index of the cross bar to the output.
 *
 * input_index Parameters:
 *   mux_index_out   - XBAR Output and mux_index choice.
 *   mux_index_input - XBAR Input and input_index choice.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ******************************************************************************************************************************************/

int imxrt_xbar_connect(uint16_t mux_out, uint16_t mux_input);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_XBAR_H */
