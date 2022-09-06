/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_pmu.c
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
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "nvic.h"
#include "gd32f4xx_pmu.h"
#include "gd32f4xx.h"

#if defined(CONFIG_GD32F4_PMU)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t gd32_pmu_reg_snap[4];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pmu_lvd_select
 *
 * Description:
 *   Select low voltage detector threshold.
 *
 * Input Parameters:
 *   lvdt_n - PMU_CTL_LVDT(n), LVD threshold level
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_lvd_select(uint32_t lvdt_n)
{
  uint32_t regval;

  /* Disable LVD */

  modifyreg32(GD32_PMU_CTL, PMU_CTL_LVDEN, 0);

  regval = getreg32(GD32_PMU_CTL);

  /* Clear LVDT bits */

  regval &= ~PMU_CTL_LVDT_MASK;

  /* Set LVDT bits according to pmu_lvdt_n */

  regval |= lvdt_n;
  putreg32(regval, GD32_PMU_CTL);

  /* Enable LVD */

  modifyreg32(GD32_PMU_CTL, 0, PMU_CTL_LVDEN);
}

/****************************************************************************
 * Name: gd32_pmu_lvd_enable
 *
 * Description:
 *   Enable LVD
 *
 ****************************************************************************/

void gd32_pmu_lvd_enable(void)
{
  /* Enable LVD */

  modifyreg32(GD32_PMU_CTL, 0, PMU_CTL_LVDEN);
}

/****************************************************************************
 * Name: gd32_pmu_lvd_disable
 *
 * Description:
 *   Disable LVD
 *
 ****************************************************************************/

void gd32_pmu_lvd_disable(void)
{
  /* Disable LVD */

  modifyreg32(GD32_PMU_CTL, PMU_CTL_LVDEN, 0);
}

/****************************************************************************
 * Name: gd32_pmu_ldo_output_select
 *
 * Description:
 *   Select the LDO output voltage.  Set LDO output when the main PLL closed,
 *   and it takes effect when the main PLL enabled.
 *
 * Input Parameters:
 *   ldo_output - PMU_CTL_LDOVS(n), PMU LDO output voltage select
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_ldo_output_select(uint32_t ldo_output)
{
  uint32_t regval;

  regval = getreg32(GD32_PMU_CTL);
  regval &= ~PMU_CTL_LDOVS_MASK;
  regval |= ldo_output;
  putreg32(regval, GD32_PMU_CTL);
}

/****************************************************************************
 * Name: gd32_pmu_highdriver_mode_enable
 *
 * Description:
 *   Disable high-driver mode
 *
 ****************************************************************************/

void gd32_pmu_highdriver_mode_enable(void)
{
  modifyreg32(GD32_PMU_CTL, 0, PMU_CTL_HDEN);
}

/****************************************************************************
 * Name: gd32_pmu_highdriver_mode_disable
 *
 * Description:
 *   Disable high-driver mode
 *
 ****************************************************************************/

void gd32_pmu_highdriver_mode_disable(void)
{
  modifyreg32(GD32_PMU_CTL, PMU_CTL_HDEN, 0);
}

/****************************************************************************
 * Name: gd32_pmu_highdriver_switch_select
 *
 * Description:
 *   Switch high-driver mode.
 *
 * Input Parameters:
 *   highdr_switch - PMU high-driver mode
 *                 - PMU_HIGHDR_SWITCH_NONE
 *                 - PMU_HIGHDR_SWITCH_EN
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_highdriver_switch_select(uint32_t highdr_switch)
{
  uint32_t regval;

  while (getreg32(GD32_PMU_CS) & PMU_CS_HDRF);

  regval = getreg32(GD32_PMU_CTL);
  regval &= ~PMU_CTL_HDS;
  regval |= highdr_switch;
  putreg32(regval, GD32_PMU_CTL);
}

/****************************************************************************
 * Name: gd32_pmu_lowdriver_mode_enable
 *
 * Description:
 *   Enable low-driver mode in deep-sleep
 *
 ****************************************************************************/

void gd32_pmu_lowdriver_mode_enable(void)
{
  modifyreg32(GD32_PMU_CTL, 0, PMU_LOWDRIVER_ENABLE);
}

/****************************************************************************
 * Name: gd32_pmu_lowdriver_mode_disable
 *
 * Description:
 *   Disable low-driver mode in deep-sleep
 *
 ****************************************************************************/

void gd32_pmu_lowdriver_mode_disable(void)
{
  modifyreg32(GD32_PMU_CTL, PMU_LOWDRIVER_ENABLE, PMU_LOWDRIVER_DISABLE);
}

/****************************************************************************
 * Name: gd32_pmu_lowpower_driver_config
 *
 * Description:
 *   In deep-sleep mode, driver mode when use low power LDO.
 *
 * Input Parameters:
 *   mode - PMU low-driver mode
 *        - PMU_NORMALDR_LOWPWR:  normal driver when use low power LDO
 *        - PMU_LOWDR_LOWPWR:     low-driver mode enabled when LDEN is 11 and
 *                                use low power LDO
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_lowpower_driver_config(uint32_t mode)
{
  uint32_t regval;
  regval = getreg32(GD32_PMU_CTL);
  regval &= ~PMU_CTL_LDLP;
  regval |= mode;
  putreg32(regval, GD32_PMU_CTL);
}

/****************************************************************************
 * Name: gd32_pmu_normalpower_driver_config
 *
 * Description:
 *   In deep-sleep mode, driver mode when use normal power LDO.
 *
 * Input Parameters:
 *   mode - PMU low-driver mode
 *        - PMU_NORMALDR_NORMALPWR: normal driver when use normal power LDO
 *        - PMU_LOWDR_NORMALPWR: low-driver mode enabled when LDEN is 11 and
 *                               use normal power LDO
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_normalpower_driver_config(uint32_t mode)
{
  uint32_t regval;
  regval = getreg32(GD32_PMU_CTL);
  regval &= ~PMU_CTL_LDNP;
  regval |= mode;
  putreg32(regval, GD32_PMU_CTL);
}

/****************************************************************************
 * Name: gd32_pmu_to_sleepmode
 *
 * Description:
 *   PMU work in sleep mode.
 *
 * Input Parameters:
 *   sleepmodecmd - PMU command constants
 *        - WFI_CMD: use WFI command
 *        - WFE_CMD: use WFE command
 *   sleeponexit
 *        - true:  SLEEPONEXIT bit is set when the WFI instruction is
 *                 executed, the MCU enters Sleep mode as soon as it
 *                 exits the lowest priority ISR.
 *        - false: SLEEPONEXIT bit is cleared, the MCU enters Sleep
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_sleepmode(uint8_t sleepmodecmd, bool sleeponexit)
{
  uint32_t regval;

  /* Clear SLEEPDEEP bit of Cortex-M4 System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval &= ~NVIC_SYSCON_SLEEPDEEP;

  if (sleeponexit)
    {
      regval |= NVIC_SYSCON_SLEEPONEXIT;
    }
  else
    {
      regval &= ~NVIC_SYSCON_SLEEPONEXIT;
    }

  putreg32(regval, NVIC_SYSCON);

  /* Select WFI or WFE command to enter sleep mode */

  if (sleepmodecmd == WFI_CMD)
    {
      asm("wfi");
    }
  else
    {
      asm("wfe");
    }
}

/****************************************************************************
 * Name: gd32_pmu_to_deepsleepmode
 *
 * Description:
 *   PMU work in deep-sleep mode
 *
 * Input Parameters:
 *   ldo
 *       - PMU_LDO_NORMAL:   LDO normal work when pmu enter deep-sleep mode
 *       - PMU_LDO_LOWPOWER: LDO work at low power mode when pmu enter
 *                           deep-sleep mode
 *   lowdrive
 *       - PMU_LOWDRIVER_DISABLE: Low-driver mode disable in deep-sleep mode
 *       - PMU_LOWDRIVER_ENABLE:  Low-driver mode enable in deep-sleep mode
 *
 *   deepsleepmodecmdd - PMU command constants
 *       - WFI_CMD: use WFI command
 *       - WFE_CMD: use WFE command
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_deepsleepmode(uint32_t ldo, uint32_t lowdrive,
                               uint8_t deepsleepmodecmd)
{
  uint32_t regval;

  regval = getreg32(GD32_PMU_CTL);

  /* Clear stbmod and ldolp bits */

  regval &= ~(PMU_CTL_STBMOD | PMU_CTL_LDOLP | PMU_CTL_LDEN_MASK |
              PMU_CTL_LDNP | PMU_CTL_LDLP);

  /* Set ldolp bit according to pmu_ldo */

  regval |= ldo;

  /* Configure low drive mode in deep-sleep mode */

  if (lowdrive == PMU_LOWDRIVER_ENABLE)
    {
      if (ldo == PMU_LDO_NORMAL)
        {
          regval |= (PMU_LOWDRIVER_ENABLE | PMU_CTL_LDNP);
        }
      else
        {
          regval |= (PMU_LOWDRIVER_ENABLE | PMU_CTL_LDLP);
        }
    }

  putreg32(regval, GD32_PMU_CTL);

  /* Set SLEEPDEEP bit of Cortex-M4 System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  gd32_pmu_reg_snap[0] = getreg32(NVIC_SYSTICK_CTRL);
  gd32_pmu_reg_snap[1] = getreg32(NVIC_IRQ0_31_ENABLE);
  gd32_pmu_reg_snap[2] = getreg32(NVIC_IRQ32_63_ENABLE);
  gd32_pmu_reg_snap[3] = getreg32(NVIC_IRQ64_95_ENABLE);

  putreg32((0x00010004u & gd32_pmu_reg_snap[0]), NVIC_SYSTICK_CTRL);
  putreg32(0xff7ff831u, NVIC_IRQ0_31_CLEAR);
  putreg32(0xff7ff831u, NVIC_IRQ32_63_CLEAR);
  putreg32(0xff7ff831u, NVIC_IRQ64_95_CLEAR);

  /* Select WFI or WFE command to enter sleep mode */

  if (deepsleepmodecmd == WFI_CMD)
    {
      asm("wfi");
    }
  else
    {
      asm("sev");
      asm("wfe");
      asm("wfe");
    }

  putreg32(gd32_pmu_reg_snap[0], NVIC_SYSTICK_CTRL);
  putreg32(gd32_pmu_reg_snap[1], NVIC_IRQ0_31_ENABLE);
  putreg32(gd32_pmu_reg_snap[2], NVIC_IRQ32_63_ENABLE);
  putreg32(gd32_pmu_reg_snap[3], NVIC_IRQ64_95_ENABLE);

  /* Reset SLEEPDEEP bit of Cortex-M4 System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval &= ~NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);
}

/****************************************************************************
 * Name: gd32_pmu_to_standbymode
 *
 * Description:
 *   PMU work in standby mode
 *
 * Input Parameters:
 *   standbymodecmd - PMU command constants
 *                  WFI_CMD: use WFI command
 *                  WFE_CMD: use WFE command
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_standbymode(uint8_t standbymodecmd)
{
  uint32_t regval;

  /* Set SLEEPDEEP bit of Cortex-M4 System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  regval = getreg32(GD32_PMU_CTL);

  /* Set stbmod bit */

  regval |= PMU_CTL_STBMOD;

  /* Reset wakeup flag */

  regval |= PMU_CTL_WURST;
  putreg32(regval, GD32_PMU_CTL);

  /* Select WFI or WFE command to enter sleep mode */

  if (standbymodecmd == WFI_CMD)
    {
      asm("wfi");
    }
  else
    {
      asm("wfe");
      asm("wfe");
    }
}

/****************************************************************************
 * Name: gd32_pmu_wakeup_pin_enable
 *
 * Description:
 *   Enables PMU wakeup pin.
 *
 ****************************************************************************/

void gd32_pmu_wakeup_pin_enable(void)
{
  modifyreg32(GD32_PMU_CS, 0, PMU_CS_WUPEN);
}

/****************************************************************************
 * Name: gd32_pmu_wakeup_pin_disable
 *
 * Description:
 *   Disables PMU wakeup pin.
 *
 ****************************************************************************/

void gd32_pmu_wakeup_pin_disable(void)
{
  modifyreg32(GD32_PMU_CS, PMU_CS_WUPEN, 0);
}

/****************************************************************************
 * Name: gd32_pmu_backup_ldo_config
 *
 * Description:
 *   Enables the backup ldo, to open Backup SRAM LDO for data protection of
 *   backup SRAM when VDD shut down. When VDD shut down and this bit is
 *   cleared, the data in Backup SRAM will be lost.
 *   Once set, the application should wait that the Backup SRAM LDO flag
 *   (BLDORF) is set to indicate that the data written into the RAM will be
 *   maintained when VDD shut down.
 *
 * Input Parameters:
 *   bkp_ldo - state to set it to
 *           - PMU_BLDOON_OFF: backup SRAM LDO closed
 *           - PMU_BLDOON_ON: backup SRAM LDO closed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_backup_ldo_config(bool bkp_ldo)
{
  uint32_t regval;

  regval = getreg32(GD32_PMU_CS);
  regval &= ~ PMU_CS_BLDOON;
  regval |= bkp_ldo;
  putreg32(regval, GD32_PMU_CS);
}

/****************************************************************************
 * Name: gd32_pmu_backup_init
 *
 * Description:
 *   Insures the referenced count access to the backup domain
 *   (RTC registers, RTC backup data registers and backup SRAM is consistent
 *   with the hardware state without relying on a variable.
 *
 *   NOTE: This function should only be called by SoC Start up code.
 *
 * Input Parameters:
 *   writable - set the initial state of the enable or disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_backup_init(bool writable)
{
  uint32_t regval;

  /* Make the hardware not writable */

  regval = getreg32(GD32_PMU_CTL);
  regval &= ~PMU_CS_WUPEN;
  putreg32(regval, GD32_PMU_CTL);

  if (writable)
    {
      gd32_pmu_backup_write_enable();
    }
  else
    {
      gd32_pmu_backup_write_disable();
    }
}

/****************************************************************************
 * Name: gd32_pmu_backup_write_enable
 *
 * Description:
 *   Enableswrite access to the registers in backup domain
 *
 ****************************************************************************/

void gd32_pmu_backup_write_enable(void)
{
  modifyreg32(GD32_PMU_CTL, 0, PMU_CTL_BKPWEN);
}

/****************************************************************************
 * Name: gd32_pmu_backup_write_disable
 *
 * Description:
 *   DIsables write access to the registers in backup domain
 *
 ****************************************************************************/

void gd32_pmu_backup_write_disable(void)
{
  modifyreg32(GD32_PMU_CTL, PMU_CTL_BKPWEN, 0);
}

/****************************************************************************
 * Name: gd32_pmu_flag_get
 *
 * Description:
 *   Get flag state
 *
 * Input Parameters:
 *   flag - PMU_CS_WUF: wakeup flag
 *        - PMU_CS_STBF: standby flag
 *        - PMU_CS_LVDF: lvd flag
 *        - PMU_CS_BLDORF: backup SRAM LDO ready flag
 *        - PMU_CS_LDOVSRF: LDO voltage select ready flag
 *        - PMU_CS_HDRF: high-driver ready flag
 *        - PMU_CS_HDSRF: high-driver switch ready flag
 *        - PMU_CS_LDRF: low-driver mode ready flag
 *
 ****************************************************************************/

bool gd32_pmu_flag_get(uint32_t flag)
{
  if (getreg32(GD32_PMU_CS) & flag)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: gd32_pmu_backup_write_disable
 *
 * Description:
 *   Clear the flag
 *
 * Input Parameters:
 *   flag - PMU_FLAG_RESET_WAKEUP: reset wakeup flag
 *        - PMU_FLAG_RESET_STANDBY: reset standby flag
 *
 ****************************************************************************/

void gd32_pmu_flag_clear(uint32_t flag)
{
  uint32_t regval;

  regval = getreg32(GD32_PMU_CTL);

  switch (flag)
    {
      case PMU_FLAG_RESET_WAKEUP:

        /* Reset wakeup flag */

        regval |= PMU_CTL_WURST;
        putreg32(regval, GD32_PMU_CTL);
        break;
      case PMU_FLAG_RESET_STANDBY:

        /* Reset standby flag */

        regval |= PMU_CTL_STBRST;
        putreg32(regval, GD32_PMU_CTL);
        break;
      default:
        break;
    }
}

#endif /* CONFIG_GD32F4_PMU */
