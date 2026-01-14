/****************************************************************************
 * arch/arm/src/imx9/imx9_iomuxc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_IOMUXC_H
#define __ARCH_ARM_SRC_IMX9_IMX9_IOMUXC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/imx9_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IOMUXC_PAD_CONFIG_SHIFT 1

#define IOMUX_PADCFG(_ctlregoff, _mode, _dsyregoff, _dsy, _padregoff) \
  {                                                          \
    .ctlregoff = (_ctlregoff),                               \
    .padregoff = (_padregoff),                               \
    .dsyregoff = (_dsyregoff),                               \
    .mode   = (_mode),                                       \
    .dsy    = (_dsy),                                        \
  }

#define IOMUX_CFG(_padcfg, _pad, _sion)                      \
  (iomux_cfg_t)                                              \
  {                                                          \
    .padcfg = _padcfg,                                       \
    .pad    = (_pad) >> IOMUXC_PAD_CONFIG_SHIFT,             \
    .sion   = (_sion) >> IOMUXC_MUX_SION_SHIFT,              \
  }

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Information for the pad alternate function */

struct iomux_padcfg_s
{
  /* Register offsets for PAD
   * ALT(mode) configuration for ctlreg
   * input daisy(dsy) configuration for dsyreg
   */

  uint16_t mode       :  3;
  uint16_t ctlregoff  : 13;
  uint16_t padregoff  : 16;
  uint16_t dsy        :  3;
  uint16_t dsyregoff  : 13;
};

struct iomux_cfg_s
{
  struct iomux_padcfg_s padcfg;

  /* Register values */

  uint32_t pad  :31;
  uint32_t sion : 1;
};
typedef struct iomux_cfg_s iomux_cfg_t;

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
 * Name: imx9_iomux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 * Input Parameters:
 *   cfg - The IOMUX configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int imx9_iomux_configure(iomux_cfg_t cfg);

/****************************************************************************
 * Name: imx9_iomux_configure
 *
 * Description:
 *   This can be used to forcibly set a pad to GPIO mode. This overrides and
 *   disconnects any peripheral using the pin.
 *
 * Input Parameters:
 *   cfg - The IOMUX configuration.
 *   sion - if true; sets SION, otherwise clears it.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int imx9_iomux_gpio(iomux_cfg_t cfg, bool sion);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ARCH_ARM_SRC_IMX9_IMX9_IOMUXC_H */
