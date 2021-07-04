/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_pmic.c
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
#include <debug.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include <arch/chip/pm.h>
#include "cxd56_pmic.h"

#ifdef CONFIG_CXD56_PMIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

enum pmic_cmd_type_e
{
  /* basic */

  PMIC_CMD_READ = 0x00,
  PMIC_CMD_WRITE,
  PMIC_CMD_GPO,
  PMIC_CMD_LOADSW,
  PMIC_CMD_DDCLDO,
  PMIC_CMD_INTSTATUS,
  PMIC_CMD_SETPREVSYS,
  PMIC_CMD_GETPREVSYS,
  PMIC_CMD_SETVSYS,
  PMIC_CMD_GETVSYS,

  /* charger */

  PMIC_CMD_GAUGE = 0x10,
  PMIC_CMD_GET_USB_PORT_TYPE,
  PMIC_CMD_GET_CHG_STATE,
  PMIC_CMD_SET_CHG_VOLTAGE,
  PMIC_CMD_GET_CHG_VOLTAGE,
  PMIC_CMD_SET_CHG_CURRENT,
  PMIC_CMD_GET_CHG_CURRENT,
  PMIC_CMD_SET_CHG_TEMPERATURE_MODE,
  PMIC_CMD_GET_CHG_TEMPERATURE_MODE,
  PMIC_CMD_SET_RECHG_VOLTAGE,
  PMIC_CMD_GET_RECHG_VOLTAGE,
  PMIC_CMD_PRESET_CHG_CURRENT,
  PMIC_CMD_CHG_START,
  PMIC_CMD_CHG_STOP,
  PMIC_CMD_CHG_PAUSE,
  PMIC_CMD_CHG_ENABLE,
  PMIC_CMD_CHG_DISABLE,

  /* power monitor */

  PMIC_CMD_POWER_MONITOR_ENABLE = 0x30,
  PMIC_CMD_POWER_MONITOR_STATUS,
  PMIC_CMD_POWER_MONITOR_SET,
  PMIC_CMD_POWER_MONITOR_GET,
  PMIC_CMD_AFE,
  PMIC_CMD_SET_CHG_IFIN,
  PMIC_CMD_GET_CHG_IFIN,
  PMIC_CMD_SET_CHG_TEMPERATURE_TABLE,
  PMIC_CMD_GET_CHG_TEMPERATURE_TABLE,
};

/* Register CNT_USB2 [1:0] USB_CUR_LIM constants */

#define PMIC_CUR_LIM_2_5MA  0
#define PMIC_CUR_LIM_100MA  1
#define PMIC_CUR_LIM_500MA  2

/* Register CNT_CHG1 [6:5] VO_CHG_DET4 constants */

#define PMIC_CHG_DET_MINUS400  0
#define PMIC_CHG_DET_MINUS350  1
#define PMIC_CHG_DET_MINUS300  2
#define PMIC_CHG_DET_MINUS250  3

/* Register CNT_CHG2 [7:5] SET_CHG_IFIN constants */

#define PMIC_CHG_IFIN_50   0
#define PMIC_CHG_IFIN_40   1
#define PMIC_CHG_IFIN_30   2
#define PMIC_CHG_IFIN_20   3
#define PMIC_CHG_IFIN_10   4

/* RTC Register */

#define PMIC_REG_RTC                   (0x40)
#define PMIC_REG_RRQ_TIME              (0x46)
#define PMIC_REG_RTC_ALM               (0x58)
#define PMIC_REG_LRQ_ALM               (0x5E)
#define PMIC_REG_RRQ_LRQ_STATUS        (0x60)

/* Register RRQ_LRQ_STATUS */

#define RRQ_TIME_STATE  (1 << 4)
#define LRQ_TIME_STATE  (1 << 3)
#define LRQ_OFST_STATE  (1 << 2)
#define LRQ_WU_STATE    (1 << 1)
#define LRQ_ALM_STATE   (1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* FarAPI interface structures */

struct pmic_afe_s
{
  int voltage;
  int current;
  int temperature;
};

struct pmic_temp_mode_s
{
  int low;
  int high;
};

extern int fw_pm_pmiccontrol(int cmd, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_CXD56_PMIC_INT
static pmic_notify_t g_pmic_notify[PMIC_NOTIFY_MAX];
static struct work_s g_irqwork;
#endif /* CONFIG_CXD56_PMIC_INT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_CXD56_PMIC_INT
/****************************************************************************
 * Name: is_notify_registerd
 *
 * Description:
 *   Return whether any notification is registered or not
 *
 ****************************************************************************/

static bool is_notify_registerd(void)
{
  int i;

  for (i = PMIC_NOTIFY_ALARM; i < PMIC_NOTIFY_MAX; i++)
    {
      if (g_pmic_notify[i])
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: pmic_int_worker
 *
 * Description:
 *   Work queue for pmic interrupt
 *
 ****************************************************************************/

static void pmic_int_worker(void *arg)
{
  int        i;
  uint8_t    stat;
  irqstate_t flags;
  int        irq = CXD56_IRQ_PMIC;

  /* Get interrupt cause with clear and call the registered callback */

  cxd56_pmic_get_interrupt_status(&stat);

  for (i = PMIC_NOTIFY_ALARM; i < PMIC_NOTIFY_MAX; i++)
    {
      if ((stat & (1 << i)) && g_pmic_notify[i])
        {
          g_pmic_notify[i](arg);
        }
    }

  /* Prevent from the race condition with up_pmic_set_notify() */

  flags = enter_critical_section();

  /* After processing of each notification, enable the pmic interrupt again */

  if (is_notify_registerd())
    {
      up_enable_irq(irq);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: pmic_int_handler
 *
 * Description:
 *   Interrupt handler for pmic interrupt
 *
 ****************************************************************************/

static int pmic_int_handler(int irq, void *context, void *arg)
{
  int ret;

  /* Schedule the callback to occur on the low-priority worker thread */

  DEBUGASSERT(work_available(&g_irqwork));
  ret = work_queue(LPWORK, &g_irqwork, pmic_int_worker, NULL, 0);
  if (ret < 0)
    {
      _err("ERROR: work_queue failed: %d\n", ret);
    }

  /* Disable any further pmic interrupts */

  up_disable_irq(irq);

  return OK;
}
#endif /* CONFIG_CXD56_PMIC_INT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_pmic_get_interrupt_status
 *
 * Description:
 *   Get Raw Interrupt Status register. And furthermore, if status is set,
 *   then clear the interrupt. Register's description is below:
 *
 *     7   6   5   4    3     2     1     0
 *   +---------------+-----+-----+-----+-----+
 *   | x   x   x   x |VSYS |WKUPL|WKUPS|ALARM| target of status
 *   +---------------+-----+-----+-----+-----+
 *
 * Input Parameter:
 *   status - a pointer to the polled value of interrupt status
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   status - return the value of interrupt status register
 *
 ****************************************************************************/

int cxd56_pmic_get_interrupt_status(uint8_t *status)
{
  return fw_pm_pmiccontrol(PMIC_CMD_INTSTATUS, status);
}

/****************************************************************************
 * Name: cxd56_pmic_set_gpo_reg
 *
 * Description:
 *   Set GPO register. Register's description is below:
 *
 *     7   6   5   4   3   2   1   0
 *   +---+---+---+---+---+---+---+---+
 *   |CH3|CH2|CH1|CH0|CH3|CH2|CH1|CH0| target of setbit0/clrbit0
 *   +---+---+---+---+---+---+---+---+
 *   +---+---+---+---+---+---+---+---+
 *   |CH7|CH6|CH5|CH4|CH7|CH6|CH5|CH4| target of setbit1/clrbit1
 *   +---+---+---+---+---+---+---+---+
 *   |<- 0: Hi-Z   ->|<- 0: Low    ->|
 *   |<- 1: Output ->|<- 1: High   ->|
 *
 * Input Parameter:
 *   setbitX - set bit that 1 is set
 *   clrbitX - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbitX - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo_reg(uint8_t *setbit0, uint8_t *clrbit0,
                           uint8_t *setbit1, uint8_t *clrbit1)
{
  struct pmic_gpo_arg_s
    {
      uint8_t *setbit0;
      uint8_t *clrbit0;
      uint8_t *setbit1;
      uint8_t *clrbit1;
    }

  arg =
    {
      .setbit0 = setbit0,
      .clrbit0 = clrbit0,
      .setbit1 = setbit1,
      .clrbit1 = clrbit1,
    };

  return fw_pm_pmiccontrol(PMIC_CMD_GPO, &arg);
}

/****************************************************************************
 * Name: cxd56_pmic_set_gpo
 *
 * Description:
 *   Set High/Low to the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset - GPO Channel number(s)
 *   value - true if output high, false if output low.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo(uint8_t chset, bool value)
{
  uint8_t setbit0 = 0;
  uint8_t clrbit0 = 0;
  uint8_t setbit1 = 0;
  uint8_t clrbit1 = 0;
  uint8_t set;

  /* Set GPO0~3 */

  set = chset & 0xf;
  if (set)
    {
      if (value)
        {
          setbit0 = (set << 4) | set;
        }
      else
        {
          setbit0 = (set << 4);
          clrbit0 = set;
        }
    }

  /* Set GPO4~7 */

  set = (chset >> 4) & 0xf;
  if (set)
    {
      if (value)
        {
          setbit1 = (set << 4) | set;
        }
      else
        {
          setbit1 = (set << 4);
          clrbit1 = set;
        }
    }

  return cxd56_pmic_set_gpo_reg(&setbit0, &clrbit0, &setbit1, &clrbit1);
}

/****************************************************************************
 * Name: cxd56_pmic_set_gpo_hiz
 *
 * Description:
 *   Set Hi-Z to the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset - GPO Channel number(s)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_gpo_hiz(uint8_t chset)
{
  uint8_t setbit0 = 0;
  uint8_t clrbit0 = 0;
  uint8_t setbit1 = 0;
  uint8_t clrbit1 = 0;
  uint8_t set;

  /* Set GPO0~3 */

  set = chset & 0xf;
  if (set)
    {
      clrbit0 = (set << 4) | set;
    }

  /* Set GPO4~7 */

  set = (chset >> 4) & 0xf;
  if (set)
    {
      clrbit1 = (set << 4) | set;
    }

  return cxd56_pmic_set_gpo_reg(&setbit0, &clrbit0, &setbit1, &clrbit1);
}

/****************************************************************************
 * Name: cxd56_pmic_get_gpo
 *
 * Description:
 *   Get the value from the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset : GPO Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are high. Else, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_gpo(uint8_t chset)
{
  uint8_t setbit0 = 0;
  uint8_t clrbit0 = 0;
  uint8_t setbit1 = 0;
  uint8_t clrbit1 = 0;
  uint8_t set;

  cxd56_pmic_set_gpo_reg(&setbit0, &clrbit0, &setbit1, &clrbit1);

  set = ((setbit1 & 0xf) << 4) | (setbit0 & 0xf);

  /* If all of the specified chset is high, return true */

  if ((set & chset) == chset)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: cxd56_pmic_get_gpo_hiz
 *
 * Description:
 *   Get the tristate value from the specified GPO channel(s)
 *
 * Input Parameter:
 *   chset : GPO Channel number(s)
 *
 * Returned Value:
 *   Return 0(off), 1(on) or -1(HiZ)
 *
 ****************************************************************************/

int cxd56_pmic_get_gpo_hiz(uint8_t chset)
{
  uint8_t setbit0 = 0;
  uint8_t clrbit0 = 0;
  uint8_t setbit1 = 0;
  uint8_t clrbit1 = 0;
  uint8_t set;
  uint8_t hiz;

  cxd56_pmic_set_gpo_reg(&setbit0, &clrbit0, &setbit1, &clrbit1);

  set = ((setbit1 & 0xf) << 4) | (setbit0 & 0xf);
  hiz = ((setbit1) & 0xf0) | ((setbit0 & 0xf0) >> 4);

  /* If all of the specified chset is hiz, return -1 */

  if ((hiz & chset) != chset)
    {
      return -1;
    }

  /* If all of the specified chset is high, return 1 */

  if ((set & chset) == chset)
    {
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: cxd56_pmic_set_loadswitch_reg
 *
 * Description:
 *   Set LoadSwitch register. Register's description is below:
 *
 *     7   6   5   4   3   2   1   0
 *   +---+---+---+---+---+---+---+---+
 *   | - | - | - |CH4|CH3|CH2| 1 | 1 | target of setbit/clrbit
 *   +---+---+---+---+---+---+---+---+
 *               |<- 0: Off->|
 *               |<- 1: On ->|
 *
 * Input Parameter:
 *   setbit - set bit that 1 is set
 *   clrbit - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbit - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_loadswitch_reg(uint8_t *setbit, uint8_t *clrbit)
{
  struct pmic_loadswitch_arg_s
    {
      uint8_t *setbit;
      uint8_t *clrbit;
    }

  arg =
    {
      .setbit = setbit,
      .clrbit = clrbit,
    };

  return fw_pm_pmiccontrol(PMIC_CMD_LOADSW, &arg);
}

/****************************************************************************
 * Name: cxd56_pmic_set_loadswitch
 *
 * Description:
 *   Set On/Off to the specified LoadSwitch channel(s)
 *
 * Input Parameter:
 *   chset - LoadSwitch Channel number(s)
 *   value - true if set on, false if set off.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_loadswitch(uint8_t chset, bool value)
{
  uint8_t setbit = 0;
  uint8_t clrbit = 0;

  if (value)
    {
      setbit = chset;
    }
  else
    {
      clrbit = chset;
    }

  return cxd56_pmic_set_loadswitch_reg(&setbit, &clrbit);
}

/****************************************************************************
 * Name: cxd56_pmic_get_loadswitch
 *
 * Description:
 *   Get the value from the specified LoadSwitch channel(s)
 *
 * Input Parameter:
 *   chset - LoadSwitch Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are on.
 *   Otherwise, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_loadswitch(uint8_t chset)
{
  uint8_t setbit = 0;
  uint8_t clrbit = 0;

  cxd56_pmic_set_loadswitch_reg(&setbit, &clrbit);

  return ((setbit & chset) == chset);
}

/****************************************************************************
 * Name: cxd56_pmic_set_ddc_ldo_reg
 *
 * Description:
 *   Set DDC/LDO register. Register's description is below:
 *
 *      7    6    5    4    3    2    1    0
 *   +----+----+----+----+----+----+----+----+
 *   |    |    |LDO |DDC |LDO |DDC |LDO |DDC | target of setbit/clrbit
 *   | -  | -  |PERI|CORE|ANA |ANA |EMMC|IO  |
 *   +----+----+----+----+----+----+----+----+
 *             |<-  0: Off                 ->|
 *             |<-  1: On                  ->|
 *
 * Input Parameter:
 *   setbit - set bit that 1 is set
 *   clrbit - clear bit that 1 is set
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *   setbit - return the current value of register
 *
 ****************************************************************************/

int cxd56_pmic_set_ddc_ldo_reg(uint8_t *setbit, uint8_t *clrbit)
{
  struct pmic_ddc_ldo_arg_s
    {
      uint8_t *setbit;
      uint8_t *clrbit;
    }

  arg =
    {
      .setbit = setbit,
      .clrbit = clrbit,
    };

  return fw_pm_pmiccontrol(PMIC_CMD_DDCLDO, &arg);
}

/****************************************************************************
 * Name: cxd56_pmic_set_ddc_ldo
 *
 * Description:
 *   Set On/Off to the specified DDC/LDO channel(s)
 *
 * Input Parameter:
 *   chset - DDC/LO Channel number(s)
 *   value - true if set on, false if set off.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_set_ddc_ldo(uint8_t chset, bool value)
{
  uint8_t setbit = 0;
  uint8_t clrbit = 0;

  if (value)
    {
      setbit = chset;
    }
  else
    {
      clrbit = chset;
    }

  return cxd56_pmic_set_ddc_ldo_reg(&setbit, &clrbit);
}

/****************************************************************************
 * Name: cxd56_pmic_get_ddc_ldo
 *
 * Description:
 *   Get the value from the specified DDC/LDO channel(s)
 *
 * Input Parameter:
 *   chset - DDC/LDO Channel number(s)
 *
 * Returned Value:
 *   Return true if all of the specified chset are on.
 *   Otherwise, return false
 *
 ****************************************************************************/

bool cxd56_pmic_get_ddc_ldo(uint8_t chset)
{
  uint8_t setbit = 0;
  uint8_t clrbit = 0;

  cxd56_pmic_set_ddc_ldo_reg(&setbit, &clrbit);

  return ((setbit & chset) == chset);
}

/****************************************************************************
 * Name: cxd56_pmic_get_rtc
 *
 * Description:
 *   Get the RTC value from PMIC
 *
 * Input Parameter:
 *   count - the pointer to the returned RTC counter value
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_get_rtc(uint64_t *count)
{
  int ret = 0;
  uint8_t data;
  uint8_t rtc[6];

  if (!count) return -EINVAL;

  data = 0x1;
  ret = cxd56_pmic_write(PMIC_REG_RRQ_TIME, &data, sizeof(data));
  if (ret) goto error;

  do
    {
      ret = cxd56_pmic_read(PMIC_REG_RRQ_LRQ_STATUS, &data, sizeof(data));
      if (ret) goto error;
    }
  while (!(RRQ_TIME_STATE & data));

  ret = cxd56_pmic_read(PMIC_REG_RTC, rtc, sizeof(rtc));
  if (ret) goto error;

  *count =
    (((uint64_t)((rtc[5] << 24) | (rtc[4] << 16) | (rtc[3] << 8) |
     rtc[2]) << 15) | ((rtc[1] << 8) | rtc[0]));

error:
  return ret;
}

/****************************************************************************
 * Name: cxd56_pmic_get_gauge
 *
 * Description:
 *   Get the set of values (voltage, current and temperature) from PMIC.
 *
 * Input Parameter:
 *   gauge - Set of gauge related values
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_get_gauge(FAR struct pmic_gauge_s *gauge)
{
  return fw_pm_pmiccontrol(PMIC_CMD_AFE, gauge);
}

/****************************************************************************
 * Name: cxd56_pmic_getlowervol
 *
 * Description:
 *   Get lower limit of voltage for system to be running.
 *
 * Input Parameter:
 *   voltage - Lower limit voltage (mv)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getlowervol(FAR int *voltage)
{
  return fw_pm_pmiccontrol(PMIC_CMD_GETVSYS, voltage);
}

/****************************************************************************
 * Name: cxd56_pmic_setlowervol
 *
 * Description:
 *   Set lower limit of voltage for system to be running.
 *
 * Input Parameter:
 *   voltage - Lower limit voltage (mv)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setlowervol(int voltage)
{
  return fw_pm_pmiccontrol(PMIC_CMD_SETVSYS, (void *)voltage);
}

/****************************************************************************
 * Name: cxd56_pmic_getnotifyvol
 *
 * Description:
 *   Get voltage for the low battery notification
 *
 * Input Parameter:
 *   voltage - Low battery voltage (mv)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getnotifyvol(FAR int *voltage)
{
  return fw_pm_pmiccontrol(PMIC_CMD_GETPREVSYS, voltage);
}

/****************************************************************************
 * Name: cxd56_pmic_setnotifyvol
 *
 * Description:
 *   Set voltage for the low battery notification
 *
 * Input Parameter:
 *   voltage - Low battery voltage (mv)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setnotifyvol(int voltage)
{
  return fw_pm_pmiccontrol(PMIC_CMD_SETPREVSYS, (void *)voltage);
}

/****************************************************************************
 * Name: cxd56_pmic_getchargevol
 *
 * Description:
 *   Get charge voltage
 *
 * Input Parameter:
 *   voltage - Possible values are every 50 between 4000 to 4400 (mv)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargevol(FAR int *voltage)
{
  int val;
  int ret;

  ret = fw_pm_pmiccontrol(PMIC_CMD_GET_CHG_VOLTAGE, &val);
  if (ret)
    {
      return -EIO;
    }

  val &= 0xf;

  /* Convert register value to actual voltage (mv) */

  if (val <= 8)
    {
      *voltage = 4000 + (val * 50);
    }
  else
    {
      *voltage = 4200;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_pmic_setchargevol
 *
 * Description:
 *   Set charge voltage
 *
 * Input Parameter:
 *   voltage - Available values are every 50 between 4000 to 4400 (mv)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setchargevol(int voltage)
{
  int val;

  /* Sanity check */

  if (voltage < 4000 || voltage > 4400)
    {
      return -EINVAL;
    }

  if (voltage % 50)
    {
      return -EINVAL;
    }

  /* Register setting values are every 50mv between 4.0V to 4.4V */

  val = (voltage - 4000) / 50;

  return fw_pm_pmiccontrol(PMIC_CMD_SET_CHG_VOLTAGE, (void *)val);
}

/****************************************************************************
 * Name: cxd56_pmic_getchargecurrent
 *
 * Description:
 *   Get charge current value
 *
 * Input Parameter:
 *   current - Possible values are 2, 100 and 500 (mA). However,
 *             2 means 2.5 mA actually.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargecurrent(FAR int *current)
{
  int val;
  int ret;

  ret = fw_pm_pmiccontrol(PMIC_CMD_GET_CHG_CURRENT, &val);
  if (ret)
    {
      return ret;
    }

  /* Convert register value to current */

  switch (val & 0x3)
    {
      case PMIC_CUR_LIM_2_5MA:
        *current = 2;
        break;

      case PMIC_CUR_LIM_100MA:
        *current = 100;
        break;

      case PMIC_CUR_LIM_500MA:
        *current = 500;
        break;

      default:
        return -EFAULT;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_pmic_setchargecurrent
 *
 * Description:
 *   Set charge current value
 *
 * Input Parameter:
 *   current - Available values are 2, 100 and 500 (mA). However, 2 means
 *             2.5 mA actually.
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setchargecurrent(int current)
{
  int val;

  /* Replace current values for CNT_USB2 [1:0] USB_CUR_LIM */

  switch (current)
    {
      case 2:
        val = PMIC_CUR_LIM_2_5MA;
        break;

      case 100:
        val = PMIC_CUR_LIM_100MA;
        break;

      case 500:
        val = PMIC_CUR_LIM_500MA;
        break;

      default:
        return -EFAULT;
    }

  return fw_pm_pmiccontrol(PMIC_CMD_SET_CHG_CURRENT, (void *)val);
}

/****************************************************************************
 * Name: cxd56_pmic_getporttype
 *
 * Description:
 *   Get USB port type
 *
 * Input Parameter:
 *   porttype - PMIC_PORTTYPE_SDP or PMIC_PORTTYPE_DCPCDP
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getporttype(FAR int *porttype)
{
  return fw_pm_pmiccontrol(PMIC_CMD_GET_USB_PORT_TYPE, porttype);
}

/****************************************************************************
 * Name: cxd56_pmic_getchargestate
 *
 * Description:
 *   Read charging status
 *
 * Input Parameter:
 *   state - Charging status (see PMIC_STAT_* definitions)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargestate(uint8_t *state)
{
  struct pmic_afe_s arg;
  int val;
  int ret;

  /* Update charge state */

  ret = fw_pm_pmiccontrol(PMIC_CMD_AFE, &arg);
  if (ret)
    {
      return ret;
    }

  /* Get actual charging state (CNT_USB1) */

  ret = fw_pm_pmiccontrol(PMIC_CMD_GET_CHG_STATE, &val);
  *state = val & 0xff;

  return ret;
}

/****************************************************************************
 * Name: cxd56_pmic_setrechargevol
 *
 * Description:
 *   Set threshold voltage against full charge for automatic restart
 *   charging.
 *
 * Input Parameter:
 *   mv - Available values are -400, -350, -300 and -250 (mv)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setrechargevol(int mv)
{
  int val;

  /* Convert voltage to register value */

  switch (mv)
    {
      case -400:
        val = PMIC_CHG_DET_MINUS400;
        break;

      case -350:
        val = PMIC_CHG_DET_MINUS350;
        break;

      case -300:
        val = PMIC_CHG_DET_MINUS300;
        break;

      case -250:
        val = PMIC_CHG_DET_MINUS250;
        break;

      default:
        return -EINVAL;
    }

  return fw_pm_pmiccontrol(PMIC_CMD_SET_RECHG_VOLTAGE, (void *)val);
}

/****************************************************************************
 * Name: cxd56_pmic_getrechargevol
 *
 * Description:
 *   Get threshold voltage against full charge for automatic restart
 *   charging.
 *
 * Input Parameter:
 *   mv - Possible values are -400, -350, -300 and -250 (mv)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getrechargevol(FAR int *mv)
{
  int val;
  int ret;

  ret = fw_pm_pmiccontrol(PMIC_CMD_GET_RECHG_VOLTAGE, &val);
  if (ret)
    {
      return ret;
    }

  /* Convert register value to voltage */

  switch (val)
    {
      case PMIC_CHG_DET_MINUS400:
        *mv = -400;
        break;

      case PMIC_CHG_DET_MINUS350:
        *mv = -350;
        break;

      case PMIC_CHG_DET_MINUS300:
        *mv = -300;
        break;

      case PMIC_CHG_DET_MINUS250:
        *mv = -250;
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_pmic_setchargecompcurrent
 *
 * Description:
 *   Set current value setting for determine fully charged.
 *
 * Input Parameter:
 *   current - Possible values are 50, 40, 30, 20 and 10 (mA)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setchargecompcurrent(int current)
{
  int val;

  /* Convert current (mA) to register value */

  switch (current)
    {
      case 50:
        val = PMIC_CHG_IFIN_50;
        break;

      case 40:
        val = PMIC_CHG_IFIN_40;
        break;

      case 30:
        val = PMIC_CHG_IFIN_30;
        break;

      case 20:
        val = PMIC_CHG_IFIN_20;
        break;

      case 10:
        val = PMIC_CHG_IFIN_10;
        break;

      default:
        return -EINVAL;
        break;
    }

  return fw_pm_pmiccontrol(PMIC_CMD_SET_CHG_IFIN, (void *)val);
}

/****************************************************************************
 * Name: cxd56_pmic_getchargecompcurrent
 *
 * Description:
 *   Get current value setting for determine fully charged.
 *
 * Input Parameter:
 *   current - Available values are 50, 40, 30, 20 and 10 (mA)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargecompcurrent(FAR int *current)
{
  int val;
  int ret;

  ret = fw_pm_pmiccontrol(PMIC_CMD_GET_CHG_IFIN, &val);
  if (ret)
    {
      return ret;
    }

  /* Convert register value to current (mA) */

  switch (val)
    {
      case PMIC_CHG_IFIN_50:
        *current = 50;
        break;

      case PMIC_CHG_IFIN_40:
        *current = 40;
        break;

      case PMIC_CHG_IFIN_30:
        *current = 30;
        break;

      case PMIC_CHG_IFIN_20:
        *current = 20;
        break;

      case PMIC_CHG_IFIN_10:
        *current = 10;
        break;

      default:
        *current = 0;
        ret = -EFAULT;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_pmic_gettemptable
 *
 * Description:
 *   Get temperature detect resistance table
 *
 * Input Parameter:
 *   table - Settings values for temperature detecting (see CXD5247GF
 *           specification for more detail)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_gettemptable(FAR struct pmic_temp_table_s *table)
{
  /* SET_T60 (70h) - SET_T0_2 (78h) */

  return fw_pm_pmiccontrol(PMIC_CMD_GET_CHG_TEMPERATURE_TABLE, table);
}

/****************************************************************************
 * Name: cxd56_pmic_settemptable
 *
 * Description:
 *   Set temperature detect resistance table
 *
 * Input Parameter:
 *   table - Settings values for temperature detecting (see CXD5247GF
 *           specification for more detail)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_settemptable(FAR struct pmic_temp_table_s *table)
{
  return fw_pm_pmiccontrol(PMIC_CMD_SET_CHG_TEMPERATURE_TABLE, table);
}

/****************************************************************************
 * Name: cxd56_pmic_setchargemode
 *
 * Description:
 *   Set charging mode in each low/high temperatures.
 *   In lower than 10 degrees Celsius, charging mode will be changed on/off
 *   and weak (half of charge current) according to setting.
 *   In higher than 45 degrees Celsius, charging mode will be charged on/off
 *   and weak (-0.15V from charge voltage) according to setting.
 *
 * Input Parameter:
 *   low  - Charging mode in low temperature (see PMIC_CHGMODE_*)
 *   high - Charging mode in high temperature (see PMIC_CHGMODE_*)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_setchargemode(int low, int high)
{
  struct pmic_temp_mode_s arg;

  /* CNT_CHG3
   * [3:2] SEL_CHG_THL:    <-> low
   * [1:0] SEL_CHG_THH:    <-> high
   */

  /* Sanity check */

  if (low == PMIC_CHGMODE_ON || low == PMIC_CHGMODE_OFF ||
      low == PMIC_CHGMODE_WEAK)
    {
      arg.low = low;
    }
  else
    {
      return -EINVAL;
    }

  if (high == PMIC_CHGMODE_ON || high == PMIC_CHGMODE_OFF ||
      high == PMIC_CHGMODE_WEAK)
    {
      arg.high = high;
    }
  else
    {
      return -EINVAL;
    }

  return fw_pm_pmiccontrol(PMIC_CMD_SET_CHG_TEMPERATURE_MODE, &arg);
}

/****************************************************************************
 * Name: cxd56_pmic_getchargemode
 *
 * Description:
 *   Get charging mode in each low/high temperatures.
 *   In lower than 10 degrees Celsius, charging mode will be changed on/off
 *   and weak (half of charge current) according to setting.
 *   In higher than 45 degrees Celsius, charging mode will be charged on/off
 *   and weak (-0.15V from charge voltage) according to setting.
 *
 * Input Parameter:
 *   low  - Charging mode in low temperature (see PMIC_CHG_*)
 *   high - Charging mode in high temperature (see PMIC_CHG_*)
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_pmic_getchargemode(FAR int *low, FAR int *high)
{
  struct pmic_temp_mode_s arg;
  int ret;

  ret = fw_pm_pmiccontrol(PMIC_CMD_GET_CHG_TEMPERATURE_MODE, &arg);
  if (ret)
    {
      return ret;
    }

  *low = arg.low;
  *high = arg.high;

  return OK;
}

#ifdef CONFIG_CXD56_PMIC_BATMONITOR
/****************************************************************************
 * Battery monitor for debug
 ****************************************************************************/

int cxd56_pmic_monitor_enable(FAR struct pmic_mon_s *ptr)
{
  return fw_pm_pmiccontrol(PMIC_CMD_POWER_MONITOR_ENABLE, ptr);
}

int cxd56_pmic_monitor_status(FAR struct pmic_mon_status_s *ptr)
{
  return fw_pm_pmiccontrol(PMIC_CMD_POWER_MONITOR_STATUS, ptr);
}

int cxd56_pmic_monitor_set(FAR struct pmic_mon_set_s *ptr)
{
  return fw_pm_pmiccontrol(PMIC_CMD_POWER_MONITOR_SET, ptr);
}

int cxd56_pmic_monitor_get(FAR struct pmic_mon_log_s *ptr)
{
  return fw_pm_pmiccontrol(PMIC_CMD_POWER_MONITOR_GET, ptr);
}
#endif

#ifdef CONFIG_CXD56_PMIC_INT
/****************************************************************************
 * Name: up_pmic_set_notify
 *
 * Description:
 *   Register a callback for pmic interrupt
 *
 * Input Parameter:
 *   kind - A kind of pmic interrupt defined as pmic_notify_e
 *   cb - A callback function for a kind of pmic interrupt
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int up_pmic_set_notify(int kind, pmic_notify_t cb)
{
  static int is_first = 1;
  irqstate_t flags;
  int        irq = CXD56_IRQ_PMIC;
  int        delayed_work = 0;

  if ((kind < PMIC_NOTIFY_ALARM) || (PMIC_NOTIFY_MAX <= kind))
    {
      return -EINVAL;
    }

  /* attach interrupt handler only for the first time */

  if (is_first)
    {
      irq_attach(irq, pmic_int_handler, NULL);
      is_first = 0;
    }

  flags = enter_critical_section();

  g_pmic_notify[kind] = cb;

  if (is_notify_registerd())
    {
      /* If up_enable_irq() is called when interrupt is pending,
       * an assertion may occur because workqueue is not prepared yet.
       * As the workaround, call worker directly and in the worker
       * both interrupt clear and enable processing are performed.
       */

      delayed_work = 1;
    }
  else
    {
      up_disable_irq(irq);
    }

  leave_critical_section(flags);

  if (delayed_work)
    {
      pmic_int_worker(NULL);
    }

  return 0;
}
#endif /* CONFIG_CXD56_PMIC_INT */

/****************************************************************************
 * Name: cxd56_pmic_read
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

int cxd56_pmic_read(uint8_t addr, void *buf, uint32_t size)
{
  struct pmic_trans_arg_s
    {
      uint8_t  addr;
      void     *buf;
      uint32_t size;
    }

  arg =
    {
      .addr = addr,
      .buf = buf,
      .size = size,
    };

  return fw_pm_pmiccontrol(PMIC_CMD_READ, &arg);
}

/****************************************************************************
 * Name: cxd56_pmic_write
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

int cxd56_pmic_write(uint8_t addr, void *buf, uint32_t size)
{
  struct pmic_trans_arg_s
    {
      uint8_t  addr;
      void     *buf;
      uint32_t size;
    }

  arg =
    {
      .addr = addr,
      .buf = buf,
      .size = size,
    };

  return fw_pm_pmiccontrol(PMIC_CMD_WRITE, &arg);
}

#endif /* CONFIG_CXD56_PMIC */
