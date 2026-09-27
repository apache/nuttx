/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_iomuxc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_IOMUXC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_IOMUXC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOMUXC register bit-fields (IMXRT1180RM chapter 17)
 *
 * SW_MUX_CTL_PAD register:
 *   [3:0] MUX_MODE  (alternate function 0..12)
 *   [4]   SION      (force input path)
 *
 * SW_PAD_CTL_PAD register:
 *   [1]   PDRV      (0 = high driver, 1 = normal driver)
 *   [3:2] PULL      (00 = keep, 01 = pull-up, 10 = pull-down, 11 = disabled)
 *   [4]   ODE       (open drain enable)
 *   [5]   HYS       (Schmitt trigger / hysteresis enable)
 *   [6]   FSEL_APC  (fast/slow slew select)
 *   [7]   DDR_MODE  (DDR input mode; only implemented on selected pads)
 */

/* MUX_CTL fields */

#define IOMUXC_MUX_MODE_SHIFT     (0)
#define IOMUXC_MUX_MODE_MASK      (0x0fu << IOMUXC_MUX_MODE_SHIFT)
#  define IOMUXC_MUX_MODE_ALT(n)  (((n) & 0x0fu) << IOMUXC_MUX_MODE_SHIFT)
#  define IOMUXC_MUX_MODE_ALT0    IOMUXC_MUX_MODE_ALT(0)
#  define IOMUXC_MUX_MODE_ALT1    IOMUXC_MUX_MODE_ALT(1)
#  define IOMUXC_MUX_MODE_ALT2    IOMUXC_MUX_MODE_ALT(2)
#  define IOMUXC_MUX_MODE_ALT3    IOMUXC_MUX_MODE_ALT(3)
#  define IOMUXC_MUX_MODE_ALT4    IOMUXC_MUX_MODE_ALT(4)
#  define IOMUXC_MUX_MODE_ALT5    IOMUXC_MUX_MODE_ALT(5)
#  define IOMUXC_MUX_MODE_ALT6    IOMUXC_MUX_MODE_ALT(6)
#  define IOMUXC_MUX_MODE_ALT7    IOMUXC_MUX_MODE_ALT(7)
#  define IOMUXC_MUX_MODE_ALT8    IOMUXC_MUX_MODE_ALT(8)
#  define IOMUXC_MUX_MODE_ALT9    IOMUXC_MUX_MODE_ALT(9)
#  define IOMUXC_MUX_MODE_ALT10   IOMUXC_MUX_MODE_ALT(10)
#  define IOMUXC_MUX_MODE_ALT11   IOMUXC_MUX_MODE_ALT(11)
#  define IOMUXC_MUX_MODE_ALT12   IOMUXC_MUX_MODE_ALT(12)

#define IOMUXC_MUX_SION_SHIFT     (4)
#define IOMUXC_MUX_SION_MASK      (0x01u << IOMUXC_MUX_SION_SHIFT)
#  define IOMUXC_MUX_SION_OFF     (0u << IOMUXC_MUX_SION_SHIFT)
#  define IOMUXC_MUX_SION_ON      (1u << IOMUXC_MUX_SION_SHIFT)

/* PAD_CTL fields */

#define IOMUXC_PAD_PDRV_SHIFT     (1)
#define IOMUXC_PAD_PDRV_MASK      (0x01u << IOMUXC_PAD_PDRV_SHIFT)
#  define IOMUXC_PAD_PDRV_HIGH    (0u << IOMUXC_PAD_PDRV_SHIFT)
#  define IOMUXC_PAD_PDRV_NORMAL  (1u << IOMUXC_PAD_PDRV_SHIFT)

#define IOMUXC_PAD_PULL_SHIFT     (2)
#define IOMUXC_PAD_PULL_MASK      (0x03u << IOMUXC_PAD_PULL_SHIFT)
#  define IOMUXC_PAD_PULL_KEEP    (0u << IOMUXC_PAD_PULL_SHIFT)
#  define IOMUXC_PAD_PULL_UP      (1u << IOMUXC_PAD_PULL_SHIFT)
#  define IOMUXC_PAD_PULL_DOWN    (2u << IOMUXC_PAD_PULL_SHIFT)
#  define IOMUXC_PAD_PULL_NONE    (3u << IOMUXC_PAD_PULL_SHIFT)

#define IOMUXC_PAD_ODE_SHIFT      (4)
#define IOMUXC_PAD_ODE_MASK       (0x01u << IOMUXC_PAD_ODE_SHIFT)
#  define IOMUXC_PAD_ODE_OFF      (0u << IOMUXC_PAD_ODE_SHIFT)
#  define IOMUXC_PAD_ODE_ON       (1u << IOMUXC_PAD_ODE_SHIFT)

#define IOMUXC_PAD_HYS_SHIFT      (5)
#define IOMUXC_PAD_HYS_MASK       (0x01u << IOMUXC_PAD_HYS_SHIFT)
#  define IOMUXC_PAD_HYS_OFF      (0u << IOMUXC_PAD_HYS_SHIFT)
#  define IOMUXC_PAD_HYS_ON       (1u << IOMUXC_PAD_HYS_SHIFT)

#define IOMUXC_PAD_FSEL_SHIFT     (6)
#define IOMUXC_PAD_FSEL_MASK      (0x01u << IOMUXC_PAD_FSEL_SHIFT)
#  define IOMUXC_PAD_FSEL_SLOW    (0u << IOMUXC_PAD_FSEL_SHIFT)
#  define IOMUXC_PAD_FSEL_FAST    (1u << IOMUXC_PAD_FSEL_SHIFT)

#define IOMUXC_PAD_DDR_SHIFT      (7)
#define IOMUXC_PAD_DDR_MASK       (0x01u << IOMUXC_PAD_DDR_SHIFT)
#  define IOMUXC_PAD_DDR_OFF      (0u << IOMUXC_PAD_DDR_SHIFT)
#  define IOMUXC_PAD_DDR_ON       (1u << IOMUXC_PAD_DDR_SHIFT)

/****************************************************************************
 * IMXRT_PADCFG packed encoding
 ****************************************************************************
 *
 * The (mux/pad/daisy) register addresses for every RT1180 pin/alt combo can
 * be derived from a single uint32_t using pure arithmetic, without any run-
 * time look-up tables.  The two IOMUXC instances on RT1180 (main IOMUXC at
 * 0x42a1_0000 and AON IOMUXC at 0x443c_0000) each expose:
 *
 *   MUX_CTL[i]  = base + ctl_first  + i * 4     (contiguous, verified)
 *   PAD_CTL[i]  = MUX_CTL[i] + pad_delta        (constant offset per bank)
 *   DAISY[j]    = base + dsy_first  + j * 4     (base offset differs; j
 *                                                may be sparse but is
 *                                                still expressible as an
 *                                                index into the daisy
 *                                                region)
 *
 * Constants per bank (from IMXRT1180RM Ch. 17):
 *
 *   +---------------+---------+-----------+-----------+-----------+
 *   | bank          | ctl_bit | ctl_first | pad_delta | dsy_first |
 *   +---------------+---------+-----------+-----------+-----------+
 *   | main IOMUXC   |    0    |  0x010    |  0x248    |  0x4A0    |
 *   | AON  IOMUXC   |    1    |  0x000    |  0x074    |  0x0E8    |
 *   +---------------+---------+-----------+-----------+-----------+
 *
 * PADCFG bit layout (LSB..MSB):
 *
 *   bits  0- 2  dsy_val    (3 b)  value written to DAISY register
 *   bits  3- 8  reserved   (6 b)  must be zero
 *   bits  9-17  dsy_idx    (9 b)  DAISY[j]; IMXRT_PADCFG_NO_DSYIDX = none
 *   bits 18-21  mux_mode   (4 b)  ALT 0..12
 *   bits 22-29  pad_idx    (8 b)  MUX_CTL[i] where i = pad_idx
 *   bit  30     dsy_bank   (1 b)  0 = main IOMUXC, 1 = AON IOMUXC
 *   bit  31     pad_bank   (1 b)  0 = main IOMUXC, 1 = AON IOMUXC
 *
 * Together with a two-entry bank descriptor (see imxrt_iomuxc.c) this
 * encoding suffices to compute all three register addresses at run time
 * with only shifts, masks, and a single indexed load of the descriptor
 * struct.  There is no per-pin ROM table.
 */

#define IMXRT_PADCFG_PADBANK_SHIFT   (31)
#define IMXRT_PADCFG_PADBANK_MASK    (0x1u << IMXRT_PADCFG_PADBANK_SHIFT)

#define IMXRT_PADCFG_DSYBANK_SHIFT   (30)
#define IMXRT_PADCFG_DSYBANK_MASK    (0x1u << IMXRT_PADCFG_DSYBANK_SHIFT)

#define IMXRT_PADCFG_PADIDX_SHIFT    (22)
#define IMXRT_PADCFG_PADIDX_MASK     (0xffu << IMXRT_PADCFG_PADIDX_SHIFT)

#define IMXRT_PADCFG_MUX_SHIFT       (18)
#define IMXRT_PADCFG_MUX_MASK        (0xfu << IMXRT_PADCFG_MUX_SHIFT)

#define IMXRT_PADCFG_DSYIDX_SHIFT    (9)
#define IMXRT_PADCFG_DSYIDX_MASK     (0x1ffu << IMXRT_PADCFG_DSYIDX_SHIFT)

/* Sentinel dsy_idx meaning "this pin/alt has no input daisy". */

#define IMXRT_PADCFG_NO_DSYIDX       (0x1ffu)

#define IMXRT_PADCFG_DSY_SHIFT       (0)
#define IMXRT_PADCFG_DSY_MASK        (0x7u << IMXRT_PADCFG_DSY_SHIFT)

/* Compile-time packer used by the auto-generated imxrt118x_pinmux.h. */

#define IMXRT_PADCFG(_padbank, _padidx, _mux, _dsybank, _dsyidx, _dsy)         \
  ((((uint32_t)(_padbank) & 0x1u)   << IMXRT_PADCFG_PADBANK_SHIFT) |           \
   (((uint32_t)(_dsybank) & 0x1u)   << IMXRT_PADCFG_DSYBANK_SHIFT) |           \
   (((uint32_t)(_padidx)  & 0xffu)  << IMXRT_PADCFG_PADIDX_SHIFT)  |           \
   (((uint32_t)(_mux)     & 0xfu)   << IMXRT_PADCFG_MUX_SHIFT)     |           \
   (((uint32_t)(_dsyidx)  & 0x1ffu) << IMXRT_PADCFG_DSYIDX_SHIFT)  |           \
   (((uint32_t)(_dsy)     & 0x7u)   << IMXRT_PADCFG_DSY_SHIFT))

/* Field extraction helpers (used by the runtime driver). */

#define IMXRT_PADCFG_PADBANK(_v)  \
  (((_v) & IMXRT_PADCFG_PADBANK_MASK) >> IMXRT_PADCFG_PADBANK_SHIFT)
#define IMXRT_PADCFG_DSYBANK(_v)  \
  (((_v) & IMXRT_PADCFG_DSYBANK_MASK) >> IMXRT_PADCFG_DSYBANK_SHIFT)
#define IMXRT_PADCFG_PADIDX(_v)   \
  (((_v) & IMXRT_PADCFG_PADIDX_MASK)  >> IMXRT_PADCFG_PADIDX_SHIFT)
#define IMXRT_PADCFG_MUX(_v)      \
  (((_v) & IMXRT_PADCFG_MUX_MASK)     >> IMXRT_PADCFG_MUX_SHIFT)
#define IMXRT_PADCFG_DSYIDX(_v)   \
  (((_v) & IMXRT_PADCFG_DSYIDX_MASK)  >> IMXRT_PADCFG_DSYIDX_SHIFT)
#define IMXRT_PADCFG_DSY(_v)      \
  (((_v) & IMXRT_PADCFG_DSY_MASK)     >> IMXRT_PADCFG_DSY_SHIFT)

/****************************************************************************
 * Bank descriptor type
 ****************************************************************************/

#define IMXRT_PADCFG_NBANKS       (2)
#define IMXRT_PADCFG_BANK_MAIN    (0)
#define IMXRT_PADCFG_BANK_AON     (1)

#ifndef __ASSEMBLY__

struct imxrt_iomux_bank_s
{
  uint32_t base;       /* IOMUXC instance base address       */
  uint16_t ctl_first;  /* Offset of MUX_CTL[0] within the bank */
  uint16_t pad_delta;  /* pad_reg = mux_reg + pad_delta        */
  uint16_t dsy_first;  /* Offset of DAISY[0] within the bank   */
};

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_IOMUXC_H */
