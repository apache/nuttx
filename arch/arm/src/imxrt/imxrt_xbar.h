/****************************************************************************
 * arch/arm/src/imxrt/imxrt_xbar.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_XBAR_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_XBAR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/imxrt_memorymap.h"
#include "imxrt_periphclks.h"

/* Collect correct XBAR definitions from chip file */

#include "hardware/imxrt_xbar.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cross Bars
 *
 * A cross bar is M muxes with I inputs
 *
 * There are 3 such cross bars switches:
 *
 *   | XBARA1  | XBARAB2  | XBARB3  |
 *   |---------+----------*---------|
 *   | M  | I  |  M  | I  | M  | I  |
 *   |132 | 88 |  16 | 64 | 16 | 64 | 1020, 1050, 1060 Family
 *   |177 |146 |  16 |100 | 16 |100 | 1170 Family
 *
 *   Therefore there are M select fields that will be written to I values.
 *   The M fields are know as SELn, there are 2 selects fields per register.
 *
 *   A Input is wired to an output with a call to imxrt_xbar_connect(output,
 *   input)
 *
 */

/*    + ----------------- Which side of Mux Input or Output
 *    | ++++------------- XBAR  Index 0 - 2
 *    v vvvv  +++++++---- Index of input or output index
 * 000s xxxx  nnnnnnnn
 *
 * Where side is used to parameter check the passed value in output is an
 * output XBAR  is the index of the XBAR
 * input|output index - the index of the M mux (output) or I.
 *
 */

/* Input and Output Indexes  */

#define IMXRT_XBARA_IO_INDEX_SHIFTS  0
#define IMXRT_XBARA_IO_INDEX_MASK    (0xff << IMXRT_XBARA_IO_INDEX_SHIFTS)

/* Index for xbar addresses */

#define IMXRT_XBARA_INDEX_SHIFTS     8
#define IMXRT_XBARA_INDEX_MASK       (0xf << IMXRT_XBARA_INDEX_SHIFTS)
#  define IMXRT_XBARA1_INDEX         (0 << IMXRT_XBARA_INDEX_SHIFTS)
#  define IMXRT_XBARA2_INDEX         (1 << IMXRT_XBARA_INDEX_SHIFTS)
#  define IMXRT_XBARA3_INDEX         (2 << IMXRT_XBARA_INDEX_SHIFTS)

/* Side of xbar  */

#define IMXRT_XBARA_SIDE_SHIFTS      12
#define IMXRT_XBARA_SIDE_MASK        (0x1 << IMXRT_XBARA_SIDE_SHIFTS)

#define XBAR_OUTPUT                  (0x1 << IMXRT_XBARA_SIDE_SHIFTS)
#define XBAR_INPUT                   (0x0 << IMXRT_XBARA_SIDE_SHIFTS)

/* xbar helpers */

#define IMXRT_XBARA1(side, select)   ((uint16_t)((side) | IMXRT_XBARA1_INDEX | \
                                     ((select) & 0xff) << IMXRT_XBARA_IO_INDEX_SHIFTS))
#define IMXRT_XBARA2(side, select)   ((uint16_t)((side) | IMXRT_XBARA2_INDEX | \
                                     ((select) & 0xff) << IMXRT_XBARA_IO_INDEX_SHIFTS))
#define IMXRT_XBARA3(side, select)   ((uint16_t)((side) | IMXRT_XBARA3_INDEX | \
                                     ((select) & 0xff) << IMXRT_XBARA_IO_INDEX_SHIFTS))

#define IMXRT_SEL(six)               ((six) & IMXRT_XBARA_IO_INDEX_MASK) >> IMXRT_XBARA_IO_INDEX_SHIFTS
#define IMXRT_XBAR(six)              ((six) & IMXRT_XBARA_INDEX_MASK) >> IMXRT_XBARA_INDEX_SHIFTS
#define IMXRT_SIDE(six)              ((six) & IMXRT_XBARA_SIDE_MASK)  >> IMXRT_XBARA_SIDE_SHIFTS

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
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
 ****************************************************************************/

int imxrt_xbar_connect(uint16_t mux_out, uint16_t mux_input);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_XBAR_H */
