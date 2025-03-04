/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_iomuxc.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_IOMUXC_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_IOMUXC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_IMX93)
#  include "hardware/imx93/imx93_iomux.h"
#else
#  error Unrecognized i.MX9 architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pad muxing */

#define IOMUXC_MUX_MODE_SHIFT   (0)   /* MODE: pin alternate function */
#define IOMUXC_MUX_MODE_MASK    (0x03 << IOMUXC_MUX_MODE_SHIFT)
#define IOMUXC_MUX_MODE_ALT0    (0 << IOMUXC_MUX_MODE_SHIFT)
#define IOMUXC_MUX_MODE_ALT1    (1 << IOMUXC_MUX_MODE_SHIFT)
#define IOMUXC_MUX_MODE_ALT2    (2 << IOMUXC_MUX_MODE_SHIFT)
#define IOMUXC_MUX_MODE_ALT3    (3 << IOMUXC_MUX_MODE_SHIFT)
#define IOMUXC_MUX_MODE_ALT4    (4 << IOMUXC_MUX_MODE_SHIFT)
#define IOMUXC_MUX_MODE_ALT5    (5 << IOMUXC_MUX_MODE_SHIFT)
#define IOMUXC_MUX_MODE_ALT6    (6 << IOMUXC_MUX_MODE_SHIFT)

#define IOMUXC_MUX_SION_SHIFT   (4)   /* SION: Force input path */
#define IPMUXC_MUX_SION_MASK    (0x01 << IOMUXC_MUX_SION_SHIFT)
#define IOMUXC_MUX_SION_OFF     (0 << IOMUXC_MUX_SION_SHIFT)
#define IOMUXC_MUX_SION_ON      (1 << IOMUXC_MUX_SION_SHIFT)

/* Pad control */

#define IOMUXC_PAD_DSE_SHIFT    (1)   /* DSE: Drive strength */
#define IOMUXC_PAD_DSE_MASK     (0x3f << IOMUXC_PAD_DSE_SHIFT)
#define IOMUXC_PAD_DSE_X0       (0x00 << IOMUXC_PAD_DSE_SHIFT)
#define IOMUXC_PAD_DSE_X1       (0x01 << IOMUXC_PAD_DSE_SHIFT)
#define IOMUXC_PAD_DSE_X2       (0x03 << IOMUXC_PAD_DSE_SHIFT)
#define IOMUXC_PAD_DSE_X3       (0x07 << IOMUXC_PAD_DSE_SHIFT)
#define IOMUXC_PAD_DSE_X4       (0x0f << IOMUXC_PAD_DSE_SHIFT)
#define IOMUXC_PAD_DSE_X5       (0x1f << IOMUXC_PAD_DSE_SHIFT)
#define IOMUXC_PAD_DSE_X6       (0x3f << IOMUXC_PAD_DSE_SHIFT)

#define IOMUXC_PAD_FSEL_SHIFT   (7)   /* FSEL: Slew rate control */
#define IOMUXC_PAD_FSEL_MASK    (0x02 << IOMUXC_PAD_FSEL_SHIFT)
#define IOMUXC_PAD_FSEL_SLOW    (0 << IOMUXC_PAD_FSEL_SHIFT)
#define IOMUXC_PAD_FSEL_SSLOW   (1 << IOMUXC_PAD_FSEL_SHIFT)   /* Slightly slow */
#define IOMUXC_PAD_FSEL_SFAST   (2 << IOMUXC_PAD_FSEL_SHIFT)   /* Slightly fast */
#define IOMUXC_PAD_FSEL_FAST    (3 << IOMUXC_PAD_FSEL_SHIFT)

#define IOMUXC_PAD_PU_SHIFT     (9)   /* PU: Pull-up */
#define IOMUXC_PAD_PU_MASK      (0x01 << IOMUXC_PAD_PU_SHIFT)
#define IOMUXC_PAD_PU_OFF       (0 << IOMUXC_PAD_PU_SHIFT)
#define IOMUXC_PAD_PU_ON        (1 << IOMUXC_PAD_PU_SHIFT)

#define IOMUXC_PAD_PD_SHIFT     (10)  /* PD: Pull-down */
#define IOMUXC_PAD_PD_MASK      (0x01 << IOMUXC_PAD_PD_SHIFT)
#define IOMUXC_PAD_PD_OFF       (0 << IOMUXC_PAD_PD_SHIFT)
#define IOMUXC_PAD_PD_ON        (1 << IOMUXC_PAD_PD_SHIFT)

#define IOMUXC_PAD_OD_SHIFT     (11)  /* OD: Open-drain */
#define IOMUXC_PAD_OD_MASK      (0x01 << IOMUXC_PAD_OD_SHIFT)
#define IOMUXC_PAD_OD_DISABE    (0 << IOMUXC_PAD_OD_SHIFT)
#define IOMUXC_PAD_OD_ENABLE    (1 << IOMUXC_PAD_OD_SHIFT)

#define IOMUXC_PAD_HYS_SHIFT    (12)  /* HYS: Enable schmitt-trigger on input */
#define IOMUXC_PAD_HYS_MASK     (0x01 << IOMUXC_PAD_HYS_SHIFT)
#define IOMUXC_PAD_HYS_ST_OFF   (0 << IOMUXC_PAD_HYS_SHIFT)   /* Schmitt-trigger off */
#define IOMUXC_PAD_HYS_ST_ON    (1 << IOMUXC_PAD_HYS_SHIFT)   /* Schmitt-trigger on */

#define IOMUXC_PAD_APC_SHIFT    (24)  /* APC: Access control */
#define IOMUXC_PAD_APC_MASK     (0xff << IOMUXC_PAD_APC_SHIFT)

/* Daisy chain control, 2 bits seems to be enough */

#define IOMUXC_DSY_SHIFT        (0)
#define IOMUXC_DSY_MASK         (0x03 << IOMUXC_DSY_SHIFT)

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_IOMUXC_H */
