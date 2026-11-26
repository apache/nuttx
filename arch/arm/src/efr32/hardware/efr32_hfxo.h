/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_hfxo.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_HFXO_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_HFXO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efr32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define EFR32_HFXO_XTALCFG_OFFSET   0x0010  /* Crystal configuration */
#define EFR32_HFXO_XTALCTRL_OFFSET  0x0018  /* Crystal control */
#define EFR32_HFXO_CFG_OFFSET       0x0020  /* Configuration */
#define EFR32_HFXO_CTRL_OFFSET      0x0028  /* Control */
#define EFR32_HFXO_CMD_OFFSET       0x0050  /* Command */
#define EFR32_HFXO_STATUS_OFFSET    0x0058  /* Status */
#define EFR32_HFXO_LOCK_OFFSET      0x0080  /* Configuration lock */

/* Register addresses *******************************************************/

#define EFR32_HFXO0_XTALCFG    (EFR32_HFXO0_BASE + EFR32_HFXO_XTALCFG_OFFSET)
#define EFR32_HFXO0_XTALCTRL   (EFR32_HFXO0_BASE + EFR32_HFXO_XTALCTRL_OFFSET)
#define EFR32_HFXO0_CFG        (EFR32_HFXO0_BASE + EFR32_HFXO_CFG_OFFSET)
#define EFR32_HFXO0_CTRL       (EFR32_HFXO0_BASE + EFR32_HFXO_CTRL_OFFSET)
#define EFR32_HFXO0_CMD        (EFR32_HFXO0_BASE + EFR32_HFXO_CMD_OFFSET)
#define EFR32_HFXO0_STATUS     (EFR32_HFXO0_BASE + EFR32_HFXO_STATUS_OFFSET)
#define EFR32_HFXO0_LOCK       (EFR32_HFXO0_BASE + EFR32_HFXO_LOCK_OFFSET)

#define EFR32_HFXO0_XTALCTRL_SET (EFR32_HFXO0_XTALCTRL + EFR32_SET_OFFSET)
#define EFR32_HFXO0_CTRL_SET   (EFR32_HFXO0_CTRL + EFR32_SET_OFFSET)
#define EFR32_HFXO0_CTRL_CLR   (EFR32_HFXO0_CTRL + EFR32_CLR_OFFSET)

/* Register bit definitions *************************************************/

/* Crystal configuration */

#define HFXO_XTALCFG_COREBIASSTARTUPI_SHIFT (0)
#define HFXO_XTALCFG_COREBIASSTARTUPI_MASK  (0x3f << HFXO_XTALCFG_COREBIASSTARTUPI_SHIFT)
#  define HFXO_XTALCFG_COREBIASSTARTUPI(n)  ((uint32_t)(n) << HFXO_XTALCFG_COREBIASSTARTUPI_SHIFT)
#define HFXO_XTALCFG_COREBIASSTARTUP_SHIFT  (6)
#define HFXO_XTALCFG_COREBIASSTARTUP_MASK   (0x3f << HFXO_XTALCFG_COREBIASSTARTUP_SHIFT)
#  define HFXO_XTALCFG_COREBIASSTARTUP(n)   ((uint32_t)(n) << HFXO_XTALCFG_COREBIASSTARTUP_SHIFT)
#define HFXO_XTALCFG_CTUNEXISTARTUP_SHIFT   (12)
#define HFXO_XTALCFG_CTUNEXISTARTUP_MASK    (0xf << HFXO_XTALCFG_CTUNEXISTARTUP_SHIFT)
#  define HFXO_XTALCFG_CTUNEXISTARTUP(n)    ((uint32_t)(n) << HFXO_XTALCFG_CTUNEXISTARTUP_SHIFT)
#define HFXO_XTALCFG_CTUNEXOSTARTUP_SHIFT   (16)
#define HFXO_XTALCFG_CTUNEXOSTARTUP_MASK    (0xf << HFXO_XTALCFG_CTUNEXOSTARTUP_SHIFT)
#  define HFXO_XTALCFG_CTUNEXOSTARTUP(n)    ((uint32_t)(n) << HFXO_XTALCFG_CTUNEXOSTARTUP_SHIFT)
#define HFXO_XTALCFG_TIMEOUTSTEADY_SHIFT    (20)
#define HFXO_XTALCFG_TIMEOUTSTEADY_MASK     (0xf << HFXO_XTALCFG_TIMEOUTSTEADY_SHIFT)
#  define HFXO_XTALCFG_TIMEOUTSTEADY_T83US  (3 << HFXO_XTALCFG_TIMEOUTSTEADY_SHIFT)
#  define HFXO_XTALCFG_TIMEOUTSTEADY_T833US (12 << HFXO_XTALCFG_TIMEOUTSTEADY_SHIFT)
#define HFXO_XTALCFG_TIMEOUTCBLSB_SHIFT     (24)
#define HFXO_XTALCFG_TIMEOUTCBLSB_MASK      (0xf << HFXO_XTALCFG_TIMEOUTCBLSB_SHIFT)
#  define HFXO_XTALCFG_TIMEOUTCBLSB_T416US  (11 << HFXO_XTALCFG_TIMEOUTCBLSB_SHIFT)

/* Crystal control */

#define HFXO_XTALCTRL_COREBIASANA_SHIFT     (0)
#define HFXO_XTALCTRL_COREBIASANA_MASK      (0xff << HFXO_XTALCTRL_COREBIASANA_SHIFT)
#  define HFXO_XTALCTRL_COREBIASANA(n)      ((uint32_t)(n) << HFXO_XTALCTRL_COREBIASANA_SHIFT)
#define HFXO_XTALCTRL_CTUNEXIANA_SHIFT      (8)
#define HFXO_XTALCTRL_CTUNEXIANA_MASK       (0xff << HFXO_XTALCTRL_CTUNEXIANA_SHIFT)
#  define HFXO_XTALCTRL_CTUNEXIANA(n)       ((uint32_t)(n) << HFXO_XTALCTRL_CTUNEXIANA_SHIFT)
#define HFXO_XTALCTRL_CTUNEXOANA_SHIFT      (16)
#define HFXO_XTALCTRL_CTUNEXOANA_MASK       (0xff << HFXO_XTALCTRL_CTUNEXOANA_SHIFT)
#  define HFXO_XTALCTRL_CTUNEXOANA(n)       ((uint32_t)(n) << HFXO_XTALCTRL_CTUNEXOANA_SHIFT)
#define HFXO_XTALCTRL_CTUNEFIXANA_SHIFT     (24)
#define HFXO_XTALCTRL_CTUNEFIXANA_MASK      (0x3 << HFXO_XTALCTRL_CTUNEFIXANA_SHIFT)
#  define HFXO_XTALCTRL_CTUNEFIXANA_NONE    (0 << HFXO_XTALCTRL_CTUNEFIXANA_SHIFT)
#  define HFXO_XTALCTRL_CTUNEFIXANA_XI      (1 << HFXO_XTALCTRL_CTUNEFIXANA_SHIFT)
#  define HFXO_XTALCTRL_CTUNEFIXANA_XO      (2 << HFXO_XTALCTRL_CTUNEFIXANA_SHIFT)
#  define HFXO_XTALCTRL_CTUNEFIXANA_BOTH    (3 << HFXO_XTALCTRL_CTUNEFIXANA_SHIFT)
#define HFXO_XTALCTRL_COREDGENANA_SHIFT     (26)
#define HFXO_XTALCTRL_COREDGENANA_MASK      (0x3 << HFXO_XTALCTRL_COREDGENANA_SHIFT)
#  define HFXO_XTALCTRL_COREDGENANA_NONE    (0 << HFXO_XTALCTRL_COREDGENANA_SHIFT)
#define HFXO_XTALCTRL_SKIPCOREBIASOPT       (1 << 31)

/* Configuration */

#define HFXO_CFG_MODE_SHIFT                 (0)
#define HFXO_CFG_MODE_MASK                  (0x3 << HFXO_CFG_MODE_SHIFT)
#  define HFXO_CFG_MODE_XTAL                (0 << HFXO_CFG_MODE_SHIFT)
#  define HFXO_CFG_MODE_EXTCLK              (1 << HFXO_CFG_MODE_SHIFT)
#  define HFXO_CFG_MODE_EXTCLKPKDET         (2 << HFXO_CFG_MODE_SHIFT)
#define HFXO_CFG_ENXIDCBIASANA              (1 << 2)
#define HFXO_CFG_SQBUFSCHTRGANA             (1 << 3)

/* Control */

#define HFXO_CTRL_EM23ONDEMAND              (1 << 3)
#define HFXO_CTRL_FORCEXI2GNDANA            (1 << 4)
#define HFXO_CTRL_FORCEXO2GNDANA            (1 << 5)
#define HFXO_CTRL_FORCEEN                   (1 << 16)
#define HFXO_CTRL_DISONDEMAND               (1 << 24)
#define HFXO_CTRL_DISONDEMANDBUFOUT         (1 << 26)

/* Status */

#define HFXO_STATUS_RDY                     (1 << 0)
#define HFXO_STATUS_COREBIASOPTRDY          (1 << 1)
#define HFXO_STATUS_ENS                     (1 << 16)
#define HFXO_STATUS_SYNCBUSY                (1 << 30)
#define HFXO_STATUS_LOCK                    (1 << 31)

/* Configuration lock */

#define HFXO_LOCK_LOCKKEY_UNLOCK            0x580e

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_HFXO_H */
