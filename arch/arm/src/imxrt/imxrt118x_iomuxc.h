/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_iomuxc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT118X_IOMUXC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT118X_IOMUXC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/rt118x/imxrt118x_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IMXRT118X pin encoding for imxrt_config_gpio():
 *
 *   Bit 31    - Peripheral tag (must be set for RT118x pinsets)
 *   Bits 0-7  - Index into the board's RT118x pad table
 *
 * The pad table is provided by board-specific code (see the board's
 * imxrt_boot.c) and holds the IOMUXC base, mux/pad register offsets, alt
 * and pad control value for each pin used.
 */

#define GPIO_RT118X_PERIPH        (1u << 31)
#define GPIO_RT118X_INDEX_MASK    (0xffu)
#define GPIO_RT118X_INDEX(n)      (GPIO_RT118X_PERIPH | ((n) & GPIO_RT118X_INDEX_MASK))

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct imxrt118x_iomux_s
{
  uintptr_t base;      /* IOMUXC instance base                  */
  uint16_t  mux_off;   /* SW_MUX_CTL offset                     */
  uint16_t  pad_off;   /* SW_PAD_CTL offset, 0 if not applied   */
  uint16_t  dsy_off;   /* Daisy register offset, 0 if unused    */
  uint8_t   alt;       /* Alternate mode (mux mode)             */
  uint8_t   dsy;       /* Daisy value                           */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Board-provided pad table indexed by GPIO_RT118X_INDEX(n) */

extern const struct imxrt118x_iomux_s g_imxrt118x_padtable[];
extern const uint32_t g_imxrt118x_padctltable[];
extern const unsigned int g_imxrt118x_padtable_size;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void imxrt118x_iomux_configure(const struct imxrt118x_iomux_s *cfg,
                               uint32_t padctl);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT118X_IOMUXC_H */
