/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_power.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include "chip.h"
#include "arm_internal.h"

#include <arch/chip/pm.h>
#include <arch/board/board.h>
#include "cxd56_pmic.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_ltlock = NXMUTEX_INITIALIZER;
static bool g_used_lna = false;
static bool g_used_tcxo = true;
#ifdef CONFIG_BOARDCTL_RESET
static struct pm_cpu_freqlock_s g_hv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('B', 'P', 0),
                      PM_CPUFREQLOCK_FLAG_HV);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_pmic_read
 *
 * Description:
 *   Read the value from the specified sub address
 *
 * Input Parameter:
 *   addr - sub address
 *   buf - pointer to read buffer
 *   size - byte count of read
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int board_pmic_read(uint8_t addr, void *buf, uint32_t size)
{
  return cxd56_pmic_read(addr, buf, size);
}

/****************************************************************************
 * Name: board_pmic_write
 *
 * Description:
 *   Write the value to the specified sub address
 *
 * Input Parameter:
 *   addr - sub address
 *   buf - pointer to write buffer
 *   size - byte count of write
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int board_pmic_write(uint8_t addr, void *buf, uint32_t size)
{
  return cxd56_pmic_write(addr, buf, size);
}

/****************************************************************************
 * Name: board_power_setup
 *
 * Description:
 *   Initial setup for board-specific power control
 *
 ****************************************************************************/

int board_power_setup(int status)
{
#ifdef CONFIG_BOARD_USB_DISABLE_IN_DEEP_SLEEPING
  int      ret;
  uint8_t  val = 0;
  uint32_t bootcause;

  /* Enable USB after wakeup from deep sleeping */

  bootcause = up_pm_get_bootcause();

  switch (bootcause)
    {
      case PM_BOOT_DEEP_WKUPL:
      case PM_BOOT_DEEP_WKUPS:
      case PM_BOOT_DEEP_RTC:
      case PM_BOOT_DEEP_OTHERS:
        ret = cxd56_pmic_read(PMIC_REG_CNT_USB2, &val, sizeof(val));
        if ((ret == 0) && (val & PMIC_SET_CHGOFF))
          {
            val &= ~PMIC_SET_CHGOFF;
            cxd56_pmic_write(PMIC_REG_CNT_USB2, &val, sizeof(val));
          }
        break;
      default:
        break;
    }
#endif

  /* Disable unused DDC/LDO permanently */

  board_power_control(POWER_DDC_ANA | POWER_LDO_PERI, false);

  /* Disable unnecessary load switch in boot-up stage */

  board_power_control(POWER_AUDIO_DVDD, false);

  /* Power off all of GPO switches (except for GPO0) in boot-up stage */

  board_power_control(PMIC_GPO(1) | PMIC_GPO(2) | PMIC_GPO(3) | PMIC_GPO(4) |
                      PMIC_GPO(5) | PMIC_GPO(6) | PMIC_GPO(7), false);

  /* Set GPO0 to Hi-Z */

  cxd56_pmic_set_gpo_hiz(PMIC_GET_CH(PMIC_GPO(0)));

  return 0;
}

/****************************************************************************
 * Name: board_power_control
 *
 * Description:
 *   Power on/off the device on the board.
 *
 ****************************************************************************/

int board_power_control(int target, bool en)
{
  int ret = 0;
  int (*pfunc)(uint8_t chset, bool en) = NULL;

  switch (PMIC_GET_TYPE(target))
    {
#ifdef CONFIG_CXD56_PMIC
    case PMIC_TYPE_LSW:
      pfunc = cxd56_pmic_set_loadswitch;
      break;
    case PMIC_TYPE_GPO:
      pfunc = cxd56_pmic_set_gpo;
      break;
    case PMIC_TYPE_DDCLDO:
      pfunc = cxd56_pmic_set_ddc_ldo;
      break;
#endif /* CONFIG_CXD56_PMIC */
    default:
      break;
    }

  if (pfunc)
    {
      ret = pfunc(PMIC_GET_CH(target), en);

      /* If RTC clock is unstable, delay 1 tick for PMIC GPO setting. */

      if (!g_rtc_enabled && (PMIC_GET_TYPE(target) == PMIC_TYPE_GPO))
        {
          usleep(1);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_power_control_tristate
 *
 * Description:
 *   Power on/off/HiZ the device on the board.
 *   (HiZ is available only for PMIC_TYPE_GPO.)
 *
 * Input Parameter:
 *   target : PMIC channel
 *   value : 1 (ON), 0 (OFF), -1(HiZ)
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

int board_power_control_tristate(int target, int value)
{
  int ret = 0;
  bool en;

  if ((PMIC_GET_TYPE(target) == PMIC_TYPE_GPO) && (value < 0))
    {
      /* set HiZ to PMIC GPO channel */

      ret = cxd56_pmic_set_gpo_hiz(PMIC_GET_CH(target));

      /* If RTC clock is unstable, delay 1 tick for PMIC setting. */

      if (!g_rtc_enabled)
        {
          usleep(1);
        }
    }
  else
    {
      en = value ? true : false;
      ret = board_power_control(target, en);
    }

  return ret;
}

/****************************************************************************
 * Name: board_power_monitor
 *
 * Description:
 *   Get status of Power on/off the device on the board.
 *
 ****************************************************************************/

bool board_power_monitor(int target)
{
  bool ret = false;
  bool (*pfunc)(uint8_t chset) = NULL;

  switch (PMIC_GET_TYPE(target))
    {
#ifdef CONFIG_CXD56_PMIC
    case PMIC_TYPE_LSW:
      pfunc = cxd56_pmic_get_loadswitch;
      break;
    case PMIC_TYPE_GPO:
      pfunc = cxd56_pmic_get_gpo;
      break;
    case PMIC_TYPE_DDCLDO:
      pfunc = cxd56_pmic_get_ddc_ldo;
      break;
#endif /* CONFIG_CXD56_PMIC */
    default:
      break;
    }

  if (pfunc)
    {
      ret = pfunc(PMIC_GET_CH(target));
    }

  return ret;
}

/****************************************************************************
 * Name: board_power_monitor_tristate
 *
 * Description:
 *   Get status of Power on/off/HiZ the device on the board.
 *
 * Input Parameter:
 *   target : PMIC channel
 *
 * Returned Value:
 *   1 (ON), 0 (OFF), -1(HiZ)
 *
 ****************************************************************************/

int board_power_monitor_tristate(int target)
{
  int ret = 0;
  bool en;

  if (PMIC_GET_TYPE(target) == PMIC_TYPE_GPO)
    {
      ret = cxd56_pmic_get_gpo_hiz(PMIC_GET_CH(target));
    }
  else
    {
      en = board_power_monitor(target);
      ret = en ? 1 : 0;
    }

  return ret;
}

/****************************************************************************
 * Name: board_flash_power_control
 *
 * Description:
 *   Power on/off the flash device on the board.
 *
 ****************************************************************************/

int board_flash_power_control(bool en)
{
  int ret = 0;

  if (en)
    {
      /* power on */

      board_power_control(POWER_FLASH, true);

      /* pin enable */

      CXD56_PIN_CONFIGS(PINCONFS_SPI1);
    }
  else
    {
      /* pin disable */

      CXD56_PIN_CONFIGS(PINCONFS_SPI1_GPIO);

      /* power off */

      board_power_control(POWER_FLASH, false);
    }

  return ret;
}

/****************************************************************************
 * Name: board_flash_power_monitor
 *
 * Description:
 *   Get status of Power on/off the flash device on the board.
 *
 ****************************************************************************/

bool board_flash_power_monitor(void)
{
  return board_power_monitor(POWER_FLASH);
}

/****************************************************************************
 * Name: board_xtal_power_control
 *
 * Description:
 *   Power on/off the Xtal device on the board.
 *
 ****************************************************************************/

int board_xtal_power_control(bool en)
{
  int ret = 0;

  /* Get exclusive access to the lna / tcxo power control */

  nxmutex_lock(&g_ltlock);

  if (en)
    {
      /* power on */

      board_power_control(POWER_TCXO, true);

      /* set used flag */

      g_used_tcxo = true;
    }
  else
    {
      /* power off */

      if (!g_used_lna)
        {
          board_power_control(POWER_TCXO, false);
        }

      /* unset used flag */

      g_used_tcxo = false;
    }

  nxmutex_unlock(&g_ltlock);
  return ret;
}

/****************************************************************************
 * Name: board_xtal_power_monitor
 *
 * Description:
 *   Get status of Power on/off the Xtal device on the board.
 *
 ****************************************************************************/

bool board_xtal_power_monitor(void)
{
  return board_power_monitor(POWER_TCXO);
}

/****************************************************************************
 * Name: board_lna_power_control
 *
 * Description:
 *   Power on/off the LNA device on the board.
 *
 ****************************************************************************/

int board_lna_power_control(bool en)
{
  int ret = 0;

  /* Get exclusive access to the lna / tcxo power control */

  nxmutex_lock(&g_ltlock);

  if (en)
    {
      /* power on */

      board_power_control(POWER_LNA, true);

      /* set used flag */

      g_used_lna = true;
    }
  else
    {
      /* power off */

      if (!g_used_tcxo)
        {
          board_power_control(POWER_LNA, false);
        }

      /* unset used flag */

      g_used_lna = false;
    }

  nxmutex_unlock(&g_ltlock);
  return ret;
}

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  This function may or may not be supported by a
 *   particular board architecture.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *     meaning of this status information is board-specific.  If not used by
 *     a board, the value zero may be provided in calls to board_reset.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 * Assumptions:
 *   Must not compile up_systemreset.c to avoid duplication symbol definition
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET
int board_reset(int status)
{
  /* Restore the original state for bootup after power cycle  */

  if (!up_interrupt_context())
    {
      board_xtal_power_control(true);
      board_flash_power_control(true);
      up_pm_acquire_freqlock(&g_hv_lock);
    }

  /* System reboot */

  up_pm_reboot(); /* this function never returns */

  return 0;
}
#endif

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Power off the board.
 *
 *   If this function returns, then it was not possible to power-off the
 *   board due to some other constraints.
 *
 * Input Parameters:
 *   status - Status information provided with the power off event.
 *            This status is used as the power shutdown level.
 *            0= Deep Sleep, 1= Cold Sleep
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_POWEROFF
int board_power_off(int status)
{
  enum pm_sleepmode_e mode;
  uint8_t val;

  /* Power off explicitly because GPOs are kept during deep sleeping */

  board_power_control(PMIC_GPO(0) | PMIC_GPO(1) | PMIC_GPO(2) | PMIC_GPO(3) |
                      PMIC_GPO(4) | PMIC_GPO(5) | PMIC_GPO(6) | PMIC_GPO(7),
                      false);

  /* Set DDC_ANA output to HiZ before sleeping for power saving */

  val = PMIC_PM_HIZ | PMIC_IOST_DEF | PMIC_IOMAX_DEF;
  cxd56_pmic_write(PMIC_REG_DDC_ANA1, &val, sizeof(val));

  if (BOARD_POWEROFF_COLD == status)
    {
      /* Flash power off */

      board_flash_power_control(false);

      /* Enter cold sleep mode */

      mode = PM_SLEEP_COLD;
    }
  else
    {
#ifdef CONFIG_BOARD_USB_DISABLE_IN_DEEP_SLEEPING
      /* Disable USB detection to enter deep sleep with USB attached */

      val = PMIC_SET_CHGOFF;
      cxd56_pmic_write(PMIC_REG_CNT_USB2, &val, sizeof(val));
#endif

      /* Enter deep sleep mode */

      mode = PM_SLEEP_DEEP;
    }

  /* this function never returns */

  up_pm_sleep(mode);

  return 0;
}
#endif
