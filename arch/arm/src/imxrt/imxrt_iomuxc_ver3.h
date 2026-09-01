/****************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc_ver3.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER3_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/rt118x/imxrt118x_iomuxc.h"
#include "hardware/rt118x/imxrt118x_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* imxrt_pinset_t is a 64-bit descriptor packing all four sub-fields
 * needed to fully program a physical pad:
 *
 *  bits  0..31 : padcfg   IMXRT_PADCFG(...) constant (pad_bank,
 *                          pad_idx, ALT, dsy_bank, dsy_idx, dsy_val).
 *  bits 32..39 : pad_ctl   SW_PAD_CTL byte.
 *  bits 40..47 : mux_side  SW_MUX_CTL side-band bits in register order
 *                          (currently only SION at bit 4).
 *  bits 48..63 : gpio      16-bit gpio pinset (port | pin | mode |
 *                          output value | int cfg | int both).
 */

#define IMXRT_PINSET_PADCFG_SHIFT   (0)
#define IMXRT_PINSET_PADCTL_SHIFT   (32)
#define IMXRT_PINSET_MUXSIDE_SHIFT  (40)
#define IMXRT_PINSET_GPIO_SHIFT     (48)

#define IMXRT_PINSET_PADCFG(_p)     ((uint32_t)(_p))
#define IMXRT_PINSET_PADCTL(_p)     ((uint8_t)((_p) >> IMXRT_PINSET_PADCTL_SHIFT))
#define IMXRT_PINSET_MUXSIDE(_p)    ((uint8_t)((_p) >> IMXRT_PINSET_MUXSIDE_SHIFT))
#define IMXRT_PINSET_GPIO(_p)       ((uint16_t)((_p) >> IMXRT_PINSET_GPIO_SHIFT))

/* Compose a full 64-bit pinset from its four sub-fields.  See
 * IOMUX_PIN and IOMUX_GPIO below for the convenient wrappers.
 */

#define IOMUX_CFG(_padcfg, _pad_ctl, _mux_side, _gpio)                       \
  (((uint64_t)(uint32_t)(_padcfg))                                         | \
   (((uint64_t)(uint8_t)(_pad_ctl))  << IMXRT_PINSET_PADCTL_SHIFT)         | \
   (((uint64_t)(uint8_t)(_mux_side)) << IMXRT_PINSET_MUXSIDE_SHIFT)        | \
   (((uint64_t)(uint16_t)(_gpio))    << IMXRT_PINSET_GPIO_SHIFT))

/* Peripheral pin: no GPIO fields.  `mux_side' is OR'd into SW_MUX_CTL
 * verbatim; the only defined bit is IOMUXC_MUX_SION_ON.
 */

#define IOMUX_PIN(_padcfg, _pad_ctl, _mux_side) \
  IOMUX_CFG(_padcfg, _pad_ctl, _mux_side, 0)

/* GPIO pin: caller supplies pad_ctl and a 16-bit gpio pinset built from
 * GPIO_INPUT / GPIO_OUTPUT / GPIO_PORTn / GPIO_PINn / etc.  SION is 0
 * because a GPIO output only needs to sample the pad when the ALT is
 * routed elsewhere.
 */

#define IOMUX_GPIO(_padcfg, _pad_ctl, _gpio) \
  IOMUX_CFG(_padcfg, _pad_ctl, 0, _gpio)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Packed 64-bit pad + GPIO descriptor.  See IOMUX_CFG above for the
 * bit layout.  Peripheral drivers and board files should treat this
 * type as opaque and use the IOMUX_PIN / IOMUX_GPIO macros to build
 * values and the IMXRT_PINSET_* macros (or the driver entry points) to
 * consume them.
 */

typedef uint64_t imxrt_pinset_t;

/****************************************************************************
 * Public Function Prototypes
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
 * Name: imxrt_iomux_configure
 *
 * Description:
 *   Program a pad's SW_MUX_CTL, SW_PAD_CTL and (if applicable) input
 *   DAISY select register from a packed pinset.
 *
 * Input Parameters:
 *   pinset - the pad configuration built with IOMUX_PIN() or IOMUX_GPIO()
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int imxrt_iomux_configure(imxrt_pinset_t pinset);

/****************************************************************************
 * Name: imxrt_iomux_gpio
 *
 * Description:
 *   Forcibly set a pad to GPIO mode.  This overrides and disconnects any
 *   peripheral that was previously routed through the pad.  The GPIO ALT
 *   is selected automatically based on the pad (ALT5 for most pads, ALT0
 *   for dedicated GPIO_IO pads on the main IOMUXC).
 *
 * Input Parameters:
 *   pinset - the pad configuration.  Only the padcfg and pad_ctl fields
 *            are used to identify the pad and program SW_PAD_CTL.
 *   sion   - if true; sets SION, otherwise clears it.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int imxrt_iomux_gpio(imxrt_pinset_t pinset, bool sion);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER3_H */
