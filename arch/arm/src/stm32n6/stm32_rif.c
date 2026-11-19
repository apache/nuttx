/****************************************************************************
 * arch/arm/src/stm32n6/stm32_rif.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/stm32n6xxx_rcc.h"
#include "stm32_rif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Master attributes: compartment 1 (the Cortex-M55), Secure, privileged */

#define RIF_MASTER_CID1_SEC_PRIV (RIFSC_RIMC_ATTR_MCID(1) | \
                                  RIFSC_RIMC_ATTR_MSEC | \
                                  RIFSC_RIMC_ATTR_MPRIV)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rif_setmaster
 ****************************************************************************/

static inline void rif_setmaster(unsigned int master, uint32_t attr)
{
  modifyreg32(STM32_RIFSC_RIMC_ATTR(master),
              RIFSC_RIMC_ATTR_MCID_MASK | RIFSC_RIMC_ATTR_MSEC |
              RIFSC_RIMC_ATTR_MPRIV, attr);
}

/****************************************************************************
 * Name: rif_setslave_secpriv
 ****************************************************************************/

static inline void rif_setslave_secpriv(unsigned int periph)
{
  modifyreg32(STM32_RIFSC_RISC_SECCFGR(RIFSC_PERIPH_REG(periph)), 0,
              RIFSC_PERIPH_BIT(periph));
  modifyreg32(STM32_RIFSC_RISC_PRIVCFGR(RIFSC_PERIPH_REG(periph)), 0,
              RIFSC_PERIPH_BIT(periph));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rifconfig
 ****************************************************************************/

void stm32_rifconfig(void)
{
  putreg32(RCC_AHB3ENR_RIFSCEN, STM32_RCC_AHB3ENSR);

#ifdef CONFIG_STM32_ETHMAC
  /* The ETH1 DMA reads and writes descriptors and buffers in Secure
   * AXISRAM
   */

  rif_setmaster(RIFSC_MASTER_ETH1, RIF_MASTER_CID1_SEC_PRIV);
  rif_setslave_secpriv(RIFSC_PERIPH_ETH1);
#endif

#ifdef CONFIG_STM32_LTDC
  /* Both LTDC layers fetch the framebuffer from Secure AXISRAM */

  rif_setmaster(RIFSC_MASTER_LTDC1, RIF_MASTER_CID1_SEC_PRIV);
  rif_setmaster(RIFSC_MASTER_LTDC2, RIF_MASTER_CID1_SEC_PRIV);
  rif_setslave_secpriv(RIFSC_PERIPH_LTDCL1);
  rif_setslave_secpriv(RIFSC_PERIPH_LTDCL2);
#endif

#ifdef CONFIG_STM32_SDMMC1
  /* The SDMMC IDMA reads and writes buffers in Secure AXISRAM */

  rif_setmaster(RIFSC_MASTER_SDMMC1, RIF_MASTER_CID1_SEC_PRIV);
  rif_setslave_secpriv(RIFSC_PERIPH_SDMMC1);
#endif

#ifdef CONFIG_STM32_SDMMC2
  rif_setmaster(RIFSC_MASTER_SDMMC2, RIF_MASTER_CID1_SEC_PRIV);
  rif_setslave_secpriv(RIFSC_PERIPH_SDMMC2);
#endif

#if defined(CONFIG_STM32_OTGHS) && defined(CONFIG_STM32N6_OTGHS_USB2)
  /* USB2 OTG HS is a bus master (its DMA mode is not used by the driver) */

  rif_setmaster(RIFSC_MASTER_OTG2, RIF_MASTER_CID1_SEC_PRIV);
  rif_setslave_secpriv(RIFSC_PERIPH_OTG2HS);
#elif defined(CONFIG_STM32_OTGHS)
  /* USB1 OTG HS is a bus master (its DMA mode is not used by the driver) */

  rif_setmaster(RIFSC_MASTER_OTG1, RIF_MASTER_CID1_SEC_PRIV);
  rif_setslave_secpriv(RIFSC_PERIPH_OTG1HS);
#endif
}
