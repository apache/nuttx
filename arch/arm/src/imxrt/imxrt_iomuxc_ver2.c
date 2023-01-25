/****************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc_ver2.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include "arm_internal.h"
#include "hardware/imxrt_ccm.h"
#include "imxrt_periphclks.h"
#include "imxrt_iomuxc_ver2.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This table is indexed by the Pad Mux register index and provides the index
 * to the corresponding Pad Control register.
 *
 * REVISIT:  This could be greatly simplified:  The Pad Control registers
 * map 1-to-1 with the Pad Mux registers except for two regions where
 * there are no corresponding Pad Mux registers.  The entire table could be
 * replaced to two range checks and the appropriate offset added to the Pad
 * Mux Register index.
 */

static const uint8_t g_mux2ctl_map[IMXRT_PADMUX_NREGISTERS] =
{
  /* The first mappings are simple 1-to-1 mappings.
   *  This may be a little wasteful
   */

  IMXRT_PADCTL_GPIO_EMC_B1_00_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_01_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_02_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_03_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_04_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_05_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_06_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_07_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_08_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_09_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_10_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_11_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_12_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_13_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_14_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_15_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_16_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_17_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_18_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_19_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_20_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_21_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_22_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_23_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_24_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_25_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_26_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_27_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_28_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_29_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_30_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_31_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_32_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_33_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_34_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_35_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_36_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_37_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_38_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_39_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_40_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B1_41_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_00_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_01_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_02_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_03_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_04_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_05_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_06_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_07_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_08_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_09_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_10_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_11_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_12_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_13_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_14_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_15_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_16_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_17_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_18_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_19_INDEX,
  IMXRT_PADCTL_GPIO_EMC_B2_20_INDEX,
  IMXRT_PADCTL_GPIO_AD_00_INDEX,
  IMXRT_PADCTL_GPIO_AD_01_INDEX,
  IMXRT_PADCTL_GPIO_AD_02_INDEX,
  IMXRT_PADCTL_GPIO_AD_03_INDEX,
  IMXRT_PADCTL_GPIO_AD_04_INDEX,
  IMXRT_PADCTL_GPIO_AD_05_INDEX,
  IMXRT_PADCTL_GPIO_AD_06_INDEX,
  IMXRT_PADCTL_GPIO_AD_07_INDEX,
  IMXRT_PADCTL_GPIO_AD_08_INDEX,
  IMXRT_PADCTL_GPIO_AD_09_INDEX,
  IMXRT_PADCTL_GPIO_AD_10_INDEX,
  IMXRT_PADCTL_GPIO_AD_11_INDEX,
  IMXRT_PADCTL_GPIO_AD_12_INDEX,
  IMXRT_PADCTL_GPIO_AD_13_INDEX,
  IMXRT_PADCTL_GPIO_AD_14_INDEX,
  IMXRT_PADCTL_GPIO_AD_15_INDEX,
  IMXRT_PADCTL_GPIO_AD_16_INDEX,
  IMXRT_PADCTL_GPIO_AD_17_INDEX,
  IMXRT_PADCTL_GPIO_AD_18_INDEX,
  IMXRT_PADCTL_GPIO_AD_19_INDEX,
  IMXRT_PADCTL_GPIO_AD_20_INDEX,
  IMXRT_PADCTL_GPIO_AD_21_INDEX,
  IMXRT_PADCTL_GPIO_AD_22_INDEX,
  IMXRT_PADCTL_GPIO_AD_23_INDEX,
  IMXRT_PADCTL_GPIO_AD_24_INDEX,
  IMXRT_PADCTL_GPIO_AD_25_INDEX,
  IMXRT_PADCTL_GPIO_AD_26_INDEX,
  IMXRT_PADCTL_GPIO_AD_27_INDEX,
  IMXRT_PADCTL_GPIO_AD_28_INDEX,
  IMXRT_PADCTL_GPIO_AD_29_INDEX,
  IMXRT_PADCTL_GPIO_AD_30_INDEX,
  IMXRT_PADCTL_GPIO_AD_31_INDEX,
  IMXRT_PADCTL_GPIO_AD_32_INDEX,
  IMXRT_PADCTL_GPIO_AD_33_INDEX,
  IMXRT_PADCTL_GPIO_AD_34_INDEX,
  IMXRT_PADCTL_GPIO_AD_35_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_00_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_01_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_02_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_03_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_04_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_05_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_00_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_01_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_02_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_03_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_04_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_05_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_06_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_07_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_08_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_09_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_10_INDEX,
  IMXRT_PADCTL_GPIO_SD_B2_11_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_00_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_01_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_02_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_03_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_04_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_05_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_06_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_07_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_08_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_09_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_10_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B1_11_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_00_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_01_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_02_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_03_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_04_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_05_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_06_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_07_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_08_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_09_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_10_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_11_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_12_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_13_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_14_INDEX,
  IMXRT_PADCTL_GPIO_DISP_B2_15_INDEX,
  IMXRT_PADCTL_WAKEUP_INDEX,
  IMXRT_PADCTL_PMIC_ON_REQ_INDEX,
  IMXRT_PADCTL_PMIC_STBY_REQ_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_00_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_01_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_02_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_03_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_04_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_05_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_06_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_07_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_08_INDEX,
  IMXRT_PADCTL_GPIO_SNVS_09_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_00_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_01_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_02_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_03_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_04_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_05_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_06_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_07_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_08_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_09_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_10_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_11_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_12_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_13_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_14_INDEX,
  IMXRT_PADCTL_GPIO_LPSR_15_INDEX
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_padmux_map
 *
 * Description:
 *   This function map a Pad Mux register index to the corresponding Pad
 *   Control register index.
 *
 ****************************************************************************/

unsigned int imxrt_padmux_map(unsigned int padmux)
{
  DEBUGASSERT(padmux < IMXRT_PADMUX_NREGISTERS);
  return (unsigned int)g_mux2ctl_map[padmux];
}

/****************************************************************************
 * Name: imxrt_iomux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 ****************************************************************************/

int imxrt_iomux_configure(uintptr_t padctl, iomux_pinset_t ioset)
{
  uint32_t regval = 0;

  /* Enable IOMUXC clock if it is not already enabled */

  imxrt_clockall_iomuxc();
#if 0
  imxrt_clockall_iomuxc_gpr();
#endif

#if 0 /* Are low-power domain, Secure Non-volatile Storage (SNVS) IOMUXC clocks needed? */
  imxrt_clockall_iomuxc_snvs();
  imxrt_clockall_iomuxc_snvs_gpr();
#endif

  DEBUGASSERT(padctl >= IMXRT_PADCTL_GPIO_EMC_B1_00);

  if (padctl <= IMXRT_PADCTL_GPIO_EMC_B1_41)
    {
      /* GPIO_EMC_B1 ********************************************************/

      /* Select drive strength */

      if ((ioset & IOMUX_DRIVE_MASK) == IOMUX_DRIVE_HIGHSTRENGTH)
        {
          regval &= ~PADCTL_EMC_B1_PDRV;
        }
      else
        {
          regval |= PADCTL_EMC_B1_PDRV;
        }

      /* Handle pull/keep selection */

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval |= PADCTL_EMC_B1_PULL_UP;
            break;

          case IOMUX_PULL_DOWN:
            regval |= PADCTL_EMC_B1_PULL_DOWN;
            break;

          case IOMUX_PULL_NONE:
            regval |= PADCTL_EMC_B1_PULL_NONE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_EMC_B1_ODE;
        }
      else
        {
          regval &= ~(PADCTL_EMC_B1_ODE);
        }
    }
  else if (padctl <= IMXRT_PADCTL_GPIO_EMC_B2_20)
    {
      /* GPIO_EMC_B2 ********************************************************/

      /* Select drive strength */

      if ((ioset & IOMUX_DRIVE_MASK) == IOMUX_DRIVE_HIGHSTRENGTH)
        {
          regval &= ~PADCTL_EMC_B2_PDRV;
        }
      else
        {
          regval |= PADCTL_EMC_B2_PDRV;
        }

      /* Handle pull/keep selection */

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval |= PADCTL_EMC_B2_PULL_UP;
            break;

          case IOMUX_PULL_DOWN:
            regval |= PADCTL_EMC_B2_PULL_DOWN;
            break;

          case IOMUX_PULL_NONE:
            regval |= PADCTL_EMC_B2_PULL_NONE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_EMC_B2_ODE;
        }
      else
        {
          regval &= ~(PADCTL_EMC_B2_ODE);
        }
    }
  else if (padctl <= IMXRT_PADCTL_GPIO_AD_35)
    {
      /* GPIO_AD ************************************************************/

      /* Select slow/fast slew rate */

      if ((ioset & IOMUX_SLEW_FAST) != 0)
        {
          regval |= PADCTL_AD_SRE;
        }
      else
        {
          regval &= ~(PADCTL_AD_SRE);
        }

      /* Select drive strength */

      if ((ioset & IOMUX_DRIVE_MASK) == IOMUX_DRIVE_HIGHSTRENGTH)
        {
          regval |= PADCTL_AD_DSE;
        }
      else
        {
          regval &= ~(PADCTL_AD_DSE);
        }

      /* Handle pull/keep selection */

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval &= ~(PADCTL_AD_PUS | PADCTL_AD_PUE);
            regval |= (PADCTL_AD_PUE | PADCTL_AD_PULL_UP);
            break;

          case IOMUX_PULL_DOWN:
            regval &= ~(PADCTL_AD_PUS | PADCTL_AD_PUE);
            regval |= (PADCTL_AD_PUE | PADCTL_AD_PULL_DOWN);
            break;

          case IOMUX_PULL_NONE:
            regval &= ~PADCTL_AD_PUE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_AD_ODE;
        }
      else
        {
          regval &= ~(PADCTL_AD_ODE);
        }
    }
  else if (padctl <= IMXRT_PADCTL_GPIO_SD_B1_05)
    {
      /* GPIO_SD_B1 *********************************************************/

      /* Select drive strength */

      if ((ioset & IOMUX_DRIVE_MASK) == IOMUX_DRIVE_HIGHSTRENGTH)
        {
          regval &= ~(PADCTL_SD_DISP_B1_PDRV);
        }
      else
        {
          regval |= PADCTL_SD_DISP_B1_PDRV;
        }

      /* Handle pull/keep selection */

      regval &= ~(PADCTL_SD_DISP_B1_PULL_MASK);

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval |= PADCTL_SD_DISP_B1_PULL_UP;
            break;

          case IOMUX_PULL_DOWN:
            regval |= PADCTL_SD_DISP_B1_PULL_DOWN;
            break;

          case IOMUX_PULL_NONE:
            regval |= PADCTL_SD_DISP_B1_PULL_NONE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_SD_DISP_B1_ODE;
        }
      else
        {
          regval &= ~(PADCTL_SD_DISP_B1_ODE);
        }
    }
  else if (padctl <= IMXRT_PADCTL_GPIO_SD_B2_11)
    {
      /* GPIO_SD_B2 *********************************************************/

      /* Select drive strength */

      if ((ioset & IOMUX_DRIVE_MASK) == IOMUX_DRIVE_HIGHSTRENGTH)
        {
          regval |= PADCTL_SD_B2_PDRV;
        }
      else
        {
          regval &= ~(PADCTL_SD_B2_PDRV);
        }

      /* Handle pull/keep selection */

      regval &= ~(PADCTL_SD_B2_PULL_MASK);

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval |= PADCTL_SD_B2_PULL_UP;
            break;

          case IOMUX_PULL_DOWN:
            regval |= PADCTL_SD_B2_PULL_DOWN;
            break;

          case IOMUX_PULL_NONE:
            regval |= PADCTL_SD_B2_PULL_NONE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_SD_B2_ODE;
        }
      else
        {
          regval &= ~(PADCTL_SD_B2_ODE);
        }
    }
  else if (padctl <= IMXRT_PADCTL_GPIO_DISP_B1_11)
    {
      /* GPIO_DISP_B1 *******************************************************/

      /* Select drive strength */

      if ((ioset & IOMUX_DRIVE_MASK) == IOMUX_DRIVE_HIGHSTRENGTH)
        {
          regval &= ~(PADCTL_SD_DISP_B1_PDRV);
        }
      else
        {
          regval |= PADCTL_SD_DISP_B1_PDRV;
        }

      /* Handle pull/keep selection */

      regval &= ~(PADCTL_SD_DISP_B1_PULL_MASK);

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval |= PADCTL_SD_DISP_B1_PULL_UP;
            break;

          case IOMUX_PULL_DOWN:
            regval |= PADCTL_SD_DISP_B1_PULL_DOWN;
            break;

          case IOMUX_PULL_NONE:
            regval |= PADCTL_SD_DISP_B1_PULL_NONE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_SD_DISP_B1_ODE;
        }
      else
        {
          regval &= ~(PADCTL_SD_DISP_B1_ODE);
        }
    }
  else if (padctl <= IMXRT_PADCTL_GPIO_DISP_B2_15)
    {
      /* GPIO_DISP_B2 *******************************************************/

      /* Select slow/fast slew rate */

      if ((ioset & IOMUX_SLEW_FAST) != 0)
        {
          regval |= PADCTL_DISP_B2_SRE;
        }
      else
        {
          regval &= ~(PADCTL_DISP_B2_SRE);
        }

      /* Select drive strength */

      if ((ioset & IOMUX_DRIVE_MASK) == IOMUX_DRIVE_HIGHSTRENGTH)
        {
          regval |= PADCTL_DISP_B2_DSE;
        }
      else
        {
          regval &= ~(PADCTL_DISP_B2_DSE);
        }

      /* Handle pull/keep selection */

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval &= ~(PADCTL_DISP_B2_PUE | PADCTL_DISP_B2_PUS);
            regval |= (PADCTL_DISP_B2_PUE | PADCTL_DISP_B2_PULL_UP);
            break;

          case IOMUX_PULL_DOWN:
            regval &= ~(PADCTL_DISP_B2_PUE | PADCTL_DISP_B2_PUS);
            regval |= (PADCTL_DISP_B2_PUE | PADCTL_DISP_B2_PULL_DOWN);
            break;

          case IOMUX_PULL_NONE:
            regval &= ~PADCTL_DISP_B2_PUE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_DISP_B2_ODE;
        }
      else
        {
          regval &= ~(PADCTL_DISP_B2_ODE);
        }
    }
  else if (padctl <= IMXRT_PADCTL_GPIO_LPSR_15)
    {
      DEBUGASSERT(padctl >= IMXRT_PADCTL_GPIO_LPSR_00);

      /* GPIO_LPSR **********************************************************/

      /* Select slow/fast slew rate */

      if ((ioset & IOMUX_SLEW_FAST) != 0)
        {
          regval |= PADCTL_LPSR_SRE;
        }
      else
        {
          regval &= ~(PADCTL_LPSR_SRE);
        }

      /* Select drive strength */

      if ((ioset & IOMUX_DRIVE_MASK) == IOMUX_DRIVE_HIGHSTRENGTH)
        {
          regval |= PADCTL_LPSR_DSE;
        }
      else
        {
          regval &= ~(PADCTL_LPSR_DSE);
        }

      /* Handle pull/keep selection */

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval &= ~(PADCTL_LPSR_PUE | PADCTL_LPSR_PUS);
            regval |= (PADCTL_LPSR_PUE | PADCTL_LPSR_PULL_UP);
            break;

          case IOMUX_PULL_DOWN:
            regval &= ~(PADCTL_LPSR_PUE | PADCTL_LPSR_PUS);
            regval |= (PADCTL_LPSR_PUE | PADCTL_LPSR_PULL_DOWN);
            break;

          case IOMUX_PULL_NONE:
            regval &= ~PADCTL_LPSR_PUE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_LPSR_ODE;
        }
      else
        {
          regval &= ~(PADCTL_LPSR_ODE);
        }
    }
  else if (padctl <= IMXRT_PADCTL_GPIO_SNVS_09)
    {
      DEBUGASSERT(padctl >= IMXRT_PADCTL_TEST_MODE);

      /* GPIO_SNVS **********************************************************/

      /* Handle pull/keep selection */

      switch (ioset & IOMUX_PULL_MASK)
        {
          case IOMUX_PULL_UP:
            regval &= ~(PADCTL_SNVS_PUE | PADCTL_SNVS_PUS);
            regval |= (PADCTL_SNVS_PUE | PADCTL_SNVS_PULL_UP);
            break;

          case IOMUX_PULL_DOWN:
            regval &= ~(PADCTL_SNVS_PUE | PADCTL_SNVS_PUS);
            regval |= (PADCTL_SNVS_PUE | PADCTL_SNVS_PULL_DOWN);
            break;

          case IOMUX_PULL_NONE:
            regval &= ~PADCTL_SNVS_PUE;
            break;

          default:
            break;
        }

      /* Select CMOS output or Open Drain output */

      if ((ioset & IOMUX_OPENDRAIN) != 0)
        {
          regval |= PADCTL_SNVS_ODE;
        }
      else
        {
          regval &= ~(PADCTL_SNVS_ODE);
        }
    }
  else
    {
      /* We should never get here, unless padctl is not a valid register */

      return ERROR;
    }

  /* Write the result to the specified Pad Control register */

  putreg32(regval, padctl);
  return OK;
}
