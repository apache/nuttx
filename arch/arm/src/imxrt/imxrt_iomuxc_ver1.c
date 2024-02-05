/****************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc_ver1.c
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
#include "imxrt_iomuxc.h"

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

#if (defined(CONFIG_ARCH_FAMILY_IMXRT105x) || defined(CONFIG_ARCH_FAMILY_IMXRT106x))
static const uint8_t g_mux2ctl_map[IMXRT_PADMUX_NREGISTERS] =
{
  /* The first mappings are simple 1-to-1 mappings.
   *  This may be a little wasteful
   */

  IMXRT_PADCTL_GPIO_EMC_00_INDEX,
  IMXRT_PADCTL_GPIO_EMC_01_INDEX,
  IMXRT_PADCTL_GPIO_EMC_02_INDEX,
  IMXRT_PADCTL_GPIO_EMC_03_INDEX,
  IMXRT_PADCTL_GPIO_EMC_04_INDEX,
  IMXRT_PADCTL_GPIO_EMC_05_INDEX,
  IMXRT_PADCTL_GPIO_EMC_06_INDEX,
  IMXRT_PADCTL_GPIO_EMC_07_INDEX,
  IMXRT_PADCTL_GPIO_EMC_08_INDEX,
  IMXRT_PADCTL_GPIO_EMC_09_INDEX,
  IMXRT_PADCTL_GPIO_EMC_10_INDEX,
  IMXRT_PADCTL_GPIO_EMC_11_INDEX,
  IMXRT_PADCTL_GPIO_EMC_12_INDEX,
  IMXRT_PADCTL_GPIO_EMC_13_INDEX,
  IMXRT_PADCTL_GPIO_EMC_14_INDEX,
  IMXRT_PADCTL_GPIO_EMC_15_INDEX,
  IMXRT_PADCTL_GPIO_EMC_16_INDEX,
  IMXRT_PADCTL_GPIO_EMC_17_INDEX,
  IMXRT_PADCTL_GPIO_EMC_18_INDEX,
  IMXRT_PADCTL_GPIO_EMC_19_INDEX,
  IMXRT_PADCTL_GPIO_EMC_20_INDEX,
  IMXRT_PADCTL_GPIO_EMC_21_INDEX,
  IMXRT_PADCTL_GPIO_EMC_22_INDEX,
  IMXRT_PADCTL_GPIO_EMC_23_INDEX,
  IMXRT_PADCTL_GPIO_EMC_24_INDEX,
  IMXRT_PADCTL_GPIO_EMC_25_INDEX,
  IMXRT_PADCTL_GPIO_EMC_26_INDEX,
  IMXRT_PADCTL_GPIO_EMC_27_INDEX,
  IMXRT_PADCTL_GPIO_EMC_28_INDEX,
  IMXRT_PADCTL_GPIO_EMC_29_INDEX,
  IMXRT_PADCTL_GPIO_EMC_30_INDEX,
  IMXRT_PADCTL_GPIO_EMC_31_INDEX,
  IMXRT_PADCTL_GPIO_EMC_32_INDEX,
  IMXRT_PADCTL_GPIO_EMC_33_INDEX,
  IMXRT_PADCTL_GPIO_EMC_34_INDEX,
  IMXRT_PADCTL_GPIO_EMC_35_INDEX,
  IMXRT_PADCTL_GPIO_EMC_36_INDEX,
  IMXRT_PADCTL_GPIO_EMC_37_INDEX,
  IMXRT_PADCTL_GPIO_EMC_38_INDEX,
  IMXRT_PADCTL_GPIO_EMC_39_INDEX,
  IMXRT_PADCTL_GPIO_EMC_40_INDEX,
  IMXRT_PADCTL_GPIO_EMC_41_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_00_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_01_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_02_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_03_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_04_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_05_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_06_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_07_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_08_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_09_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_10_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_11_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_12_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_13_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_14_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_15_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_00_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_01_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_02_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_03_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_04_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_05_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_06_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_07_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_08_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_09_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_10_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_11_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_12_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_13_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_14_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_15_INDEX,
  IMXRT_PADCTL_GPIO_B0_00_INDEX,
  IMXRT_PADCTL_GPIO_B0_01_INDEX,
  IMXRT_PADCTL_GPIO_B0_02_INDEX,
  IMXRT_PADCTL_GPIO_B0_03_INDEX,
  IMXRT_PADCTL_GPIO_B0_04_INDEX,
  IMXRT_PADCTL_GPIO_B0_05_INDEX,
  IMXRT_PADCTL_GPIO_B0_06_INDEX,
  IMXRT_PADCTL_GPIO_B0_07_INDEX,
  IMXRT_PADCTL_GPIO_B0_08_INDEX,
  IMXRT_PADCTL_GPIO_B0_09_INDEX,
  IMXRT_PADCTL_GPIO_B0_10_INDEX,
  IMXRT_PADCTL_GPIO_B0_11_INDEX,
  IMXRT_PADCTL_GPIO_B0_12_INDEX,
  IMXRT_PADCTL_GPIO_B0_13_INDEX,
  IMXRT_PADCTL_GPIO_B0_14_INDEX,
  IMXRT_PADCTL_GPIO_B0_15_INDEX,
  IMXRT_PADCTL_GPIO_B1_00_INDEX,
  IMXRT_PADCTL_GPIO_B1_01_INDEX,
  IMXRT_PADCTL_GPIO_B1_02_INDEX,
  IMXRT_PADCTL_GPIO_B1_03_INDEX,
  IMXRT_PADCTL_GPIO_B1_04_INDEX,
  IMXRT_PADCTL_GPIO_B1_05_INDEX,
  IMXRT_PADCTL_GPIO_B1_06_INDEX,
  IMXRT_PADCTL_GPIO_B1_07_INDEX,
  IMXRT_PADCTL_GPIO_B1_08_INDEX,
  IMXRT_PADCTL_GPIO_B1_09_INDEX,
  IMXRT_PADCTL_GPIO_B1_10_INDEX,
  IMXRT_PADCTL_GPIO_B1_11_INDEX,
  IMXRT_PADCTL_GPIO_B1_12_INDEX,
  IMXRT_PADCTL_GPIO_B1_13_INDEX,
  IMXRT_PADCTL_GPIO_B1_14_INDEX,
  IMXRT_PADCTL_GPIO_B1_15_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_00_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_01_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_02_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_03_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_04_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_05_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_00_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_01_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_02_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_03_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_04_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_05_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_06_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_07_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_08_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_09_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_10_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_11_INDEX,
  IMXRT_PADCTL_WAKEUP_INDEX,
  IMXRT_PADCTL_PMIC_ON_REQ_INDEX,
  IMXRT_PADCTL_PMIC_STBY_REQ_INDEX
};
#elif defined(CONFIG_ARCH_FAMILY_IMXRT102x)
static const uint8_t g_mux2ctl_map[IMXRT_PADMUX_NREGISTERS] =
{
  /* The first mappings are simple 1-to-1 mappings.
   *  This may be a little wasteful
   */

  IMXRT_PADCTL_GPIO_EMC_00_INDEX,
  IMXRT_PADCTL_GPIO_EMC_01_INDEX,
  IMXRT_PADCTL_GPIO_EMC_02_INDEX,
  IMXRT_PADCTL_GPIO_EMC_03_INDEX,
  IMXRT_PADCTL_GPIO_EMC_04_INDEX,
  IMXRT_PADCTL_GPIO_EMC_05_INDEX,
  IMXRT_PADCTL_GPIO_EMC_06_INDEX,
  IMXRT_PADCTL_GPIO_EMC_07_INDEX,
  IMXRT_PADCTL_GPIO_EMC_08_INDEX,
  IMXRT_PADCTL_GPIO_EMC_09_INDEX,
  IMXRT_PADCTL_GPIO_EMC_10_INDEX,
  IMXRT_PADCTL_GPIO_EMC_11_INDEX,
  IMXRT_PADCTL_GPIO_EMC_12_INDEX,
  IMXRT_PADCTL_GPIO_EMC_13_INDEX,
  IMXRT_PADCTL_GPIO_EMC_14_INDEX,
  IMXRT_PADCTL_GPIO_EMC_15_INDEX,
  IMXRT_PADCTL_GPIO_EMC_16_INDEX,
  IMXRT_PADCTL_GPIO_EMC_17_INDEX,
  IMXRT_PADCTL_GPIO_EMC_18_INDEX,
  IMXRT_PADCTL_GPIO_EMC_19_INDEX,
  IMXRT_PADCTL_GPIO_EMC_20_INDEX,
  IMXRT_PADCTL_GPIO_EMC_21_INDEX,
  IMXRT_PADCTL_GPIO_EMC_22_INDEX,
  IMXRT_PADCTL_GPIO_EMC_23_INDEX,
  IMXRT_PADCTL_GPIO_EMC_24_INDEX,
  IMXRT_PADCTL_GPIO_EMC_25_INDEX,
  IMXRT_PADCTL_GPIO_EMC_26_INDEX,
  IMXRT_PADCTL_GPIO_EMC_27_INDEX,
  IMXRT_PADCTL_GPIO_EMC_28_INDEX,
  IMXRT_PADCTL_GPIO_EMC_29_INDEX,
  IMXRT_PADCTL_GPIO_EMC_30_INDEX,
  IMXRT_PADCTL_GPIO_EMC_31_INDEX,
  IMXRT_PADCTL_GPIO_EMC_32_INDEX,
  IMXRT_PADCTL_GPIO_EMC_33_INDEX,
  IMXRT_PADCTL_GPIO_EMC_34_INDEX,
  IMXRT_PADCTL_GPIO_EMC_35_INDEX,
  IMXRT_PADCTL_GPIO_EMC_36_INDEX,
  IMXRT_PADCTL_GPIO_EMC_37_INDEX,
  IMXRT_PADCTL_GPIO_EMC_38_INDEX,
  IMXRT_PADCTL_GPIO_EMC_39_INDEX,
  IMXRT_PADCTL_GPIO_EMC_40_INDEX,
  IMXRT_PADCTL_GPIO_EMC_41_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_00_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_01_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_02_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_03_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_04_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_05_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_06_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_07_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_08_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_09_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_10_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_11_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_12_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_13_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_14_INDEX,
  IMXRT_PADCTL_GPIO_AD_B0_15_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_00_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_01_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_02_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_03_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_04_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_05_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_06_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_07_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_08_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_09_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_10_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_11_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_12_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_13_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_14_INDEX,
  IMXRT_PADCTL_GPIO_AD_B1_15_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_00_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_01_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_02_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_03_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_04_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_05_INDEX,
  IMXRT_PADCTL_GPIO_SD_B0_06_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_00_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_01_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_02_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_03_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_04_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_05_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_06_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_07_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_08_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_09_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_10_INDEX,
  IMXRT_PADCTL_GPIO_SD_B1_11_INDEX,
  IMXRT_PADCTL_WAKEUP_INDEX,
  IMXRT_PADCTL_PMIC_ON_REQ_INDEX,
  IMXRT_PADCTL_PMIC_STBY_REQ_INDEX
};
#else
#error Unrecognised IMXRT family
#endif

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
  uint32_t value;

  /* Enable IOMUXC clock if it is not already enabled */

  imxrt_clockall_iomuxc();
  imxrt_clockall_iomuxc_gpr();

#if 0 /* Are low-power domain, Secure Non-volatile Storage (SNVS) IOMUXC clocks needed? */
  imxrt_clockall_iomuxc_snvs();
  imxrt_clockall_iomuxc_snvs_gpr();
#endif

  /* Select CMOS input or Schmitt Trigger input */

  regval = 0;
  if ((ioset & IOMUX_SCHMITT_TRIGGER) != 0)
    {
      regval |= PADCTL_HYS;
    }

  /* Select drive strength */

  value = (ioset & IOMUX_DRIVE_MASK) >> IOMUX_DRIVE_SHIFT;
  regval |= PADCTL_DSE(value);

  /* Select spped */

  value = (ioset & IOMUX_SPEED_MASK) >> IOMUX_SPEED_SHIFT;
  regval |= PADCTL_SPEED(value);

  /* Select CMOS output or Open Drain outpout */

  if ((ioset & IOMUX_OPENDRAIN) != 0)
    {
      regval |= PADCTL_ODE;
    }

  /* Handle pull/keep selection */

  switch (ioset & _IOMUX_PULLTYPE_MASK)
    {
      default:
      case _IOMUX_PULL_NONE:
        break;

      case _IOMUX_PULL_KEEP:
        {
          regval |= PADCTL_PKE;
        }
        break;

      case _IOMUX_PULL_ENABLE:
        {
          regval |= (PADCTL_PKE | PADCTL_PUE);

          value   = (ioset & _IOMUX_PULLDESC_MASK) >> _IOMUX_PULLDESC_SHIFT;
          regval |= PADCTL_PUS(value);
        }
        break;
    }

  /* Select slow/fast slew rate */

  if ((ioset & IOMUX_SLEW_FAST) != 0)
    {
      regval |= PADCTL_SRE;
    }

  /* Write the result to the specified Pad Control register */

  putreg32(regval, padctl);
  return OK;
}
