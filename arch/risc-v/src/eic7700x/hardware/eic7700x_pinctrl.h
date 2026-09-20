/****************************************************************************
 * arch/risc-v/src/eic7700x/hardware/eic7700x_pinctrl.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_PINCTRL_H
#define __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_PINCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/eic7700x_pinctrl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Chip Level Mode Mux (CLMM), the pad multiplexing block.
 *
 * TRM part 4 section 12.1 page 369 describes the block, table 12-1 on page
 * 369 places it.  The base address is shared with the GPIO controller: GPIO
 * owns 0x51600000 to 0x5160007f and CLMM everything above, so the first pad
 * register sits at offset 0x80 rather than at zero.
 *
 * The pad registers run from 0x80 to 0x314 with no holes, one 32 bit
 * register per pad, which is why a pad id is an index and needs no lookup
 * table.  Registers past 0x314 up to the end of the 2 MiB window are not
 * described by the manual and are not touched here.
 */

#define EIC7700X_PINCTRL_BASE      0x51600000ul
#define EIC7700X_PINCTRL_PAD0      (EIC7700X_PINCTRL_BASE + 0x0080)
#define EIC7700X_PINCTRL_PAD(n)    (EIC7700X_PINCTRL_PAD0 + ((n) << 2))

/* Pad register layouts, TRM part 4 section 12.1.2.1 page 370 and the
 * register detail description in section 12.1.2.2 pages 371 to 410.
 *
 * Four layouts appear.  Every pad uses exactly one of them and the pad
 * table records which, because a field only means what it says below when
 * the pad has that layout: bit 8 is MS1 on the one RGMII style pad and part
 * of nothing at all on a general pad.
 */

/* Layout 1, general IO.  161 of the 166 pads. */

#define PINCTRL_GEN_FUNC_SHIFT     (16)      /* Bits 16-18: function select */
#define PINCTRL_GEN_FUNC_MASK      (7ul << PINCTRL_GEN_FUNC_SHIFT)
#define PINCTRL_GEN_FUNC(n)        ((uint32_t)(n) << PINCTRL_GEN_FUNC_SHIFT)
#define PINCTRL_GEN_SMT            (1ul << 7)  /* Bit 7:  Schmitt trigger   */
#define PINCTRL_GEN_DS_SHIFT       (3)         /* Bits 3-6: drive strength  */
#define PINCTRL_GEN_DS_MASK        (15ul << PINCTRL_GEN_DS_SHIFT)
#define PINCTRL_GEN_DS(n)          ((uint32_t)(n) << PINCTRL_GEN_DS_SHIFT)
#define PINCTRL_GEN_PD             (1ul << 2)  /* Bit 2:  pull down enable  */
#define PINCTRL_GEN_PU             (1ul << 1)  /* Bit 1:  pull up enable    */
#define PINCTRL_GEN_IE             (1ul << 0)  /* Bit 0:  input enable      */

/* Layout 2, RGMII style pad.  LPDDR_REF_CLK only.
 *
 * This pad has no function select at all.  The two voltage mode bits sit
 * where the general layout has nothing, and the manual forbids values of
 * {MS2,MS1} other than 2'b11 for 1.8 V and 2'b00 for 3.3 V.
 */

#define PINCTRL_RGMII_MS2          (1ul << 9)  /* Bit 9:  voltage select    */
#define PINCTRL_RGMII_MS1          (1ul << 8)  /* Bit 8:  voltage select    */
#define PINCTRL_RGMII_SMT          (1ul << 7)  /* Bit 7:  Schmitt trigger   */
#define PINCTRL_RGMII_DS_SHIFT     (3)         /* Bits 3-6: drive strength  */
#define PINCTRL_RGMII_DS_MASK      (15ul << PINCTRL_RGMII_DS_SHIFT)
#define PINCTRL_RGMII_PD           (1ul << 2)  /* Bit 2:  pull down enable  */
#define PINCTRL_RGMII_PU           (1ul << 1)  /* Bit 1:  pull up enable    */
#define PINCTRL_RGMII_IE           (1ul << 0)  /* Bit 0:  input enable      */

/* Layout 3, oscillator pad.  XIN and the pad at 0x98, which the register
 * table leaves unnamed but which table 2-4 in part 1 lists as XOUT_24M.
 *
 * Section 12.1.2.1 page 370 and the register detail description disagree
 * about the low four bits.  The prose calls bits 3-2 the damping resistor
 * RD and bits 1-0 the feedback resistor REF; the register table calls bits
 * 3-2 FRS and bits 1-0 RD.  The register table is followed here, being the
 * authoritative per register description, and neither field is written by
 * this driver.
 */

#define PINCTRL_OSC_DS_SHIFT       (4)         /* Bits 4-7: drive strength  */
#define PINCTRL_OSC_DS_MASK        (15ul << PINCTRL_OSC_DS_SHIFT)
#define PINCTRL_OSC_FRS_SHIFT      (2)         /* Bits 2-3: see note above  */
#define PINCTRL_OSC_FRS_MASK       (3ul << PINCTRL_OSC_FRS_SHIFT)
#define PINCTRL_OSC_RD_SHIFT       (0)         /* Bits 0-1: see note above  */
#define PINCTRL_OSC_RD_MASK        (3ul << PINCTRL_OSC_RD_SHIFT)

/* Layout 4, RGMII voltage mode select.  The two registers at 0x310 and
 * 0x314, which are not pads at all: they select the IO voltage for the
 * RGMII0 and RGMII1 groups as a whole.  They are in the pad address range
 * and so are described here, but they carry no pin.
 */

#define PINCTRL_MODESEL_MS1        (1ul << 1)  /* Bit 1:  voltage select    */
#define PINCTRL_MODESEL_MS2        (1ul << 0)  /* Bit 0:  voltage select    */

#endif /* __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_PINCTRL_H */
