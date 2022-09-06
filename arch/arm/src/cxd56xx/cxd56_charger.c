/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_charger.c
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

/* CXD5247GF is Li-Ion Battery Charger with Power-Path Management.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <debug.h>

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <math.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>

#include <arch/chip/battery_ioctl.h>

#include "cxd56_pmic.h"

#ifdef CONFIG_CXD56_CHARGER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#ifdef CONFIG_CXD56_CHARGER_TEMP_PRECISE
#  define USE_FLOAT_CONVERSION
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct charger_dev_s
{
  mutex_t batlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int charger_get_status(enum battery_status_e *status);
static int charger_get_health(enum battery_health_e *health);
static int charger_online(bool *online);
static int charger_get_temptable(struct battery_temp_table_s *table);
static int charger_set_temptable(struct battery_temp_table_s *table);

static ssize_t charger_read(struct file *filep, char *buffer,
                            size_t buflen);
static ssize_t charger_write(struct file *filep,
                             const char *buffer, size_t buflen);
static int charger_ioctl(struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_chargerops =
{
  NULL,           /* open */
  NULL,           /* close */
  charger_read,   /* read */
  charger_write,  /* write */
  NULL,           /* seek */
  charger_ioctl,  /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

static struct charger_dev_s g_chargerdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: charger_therm2temp
 *
 * Description:
 *   Convert temperature register value to degrees Celsius.
 *
 ****************************************************************************/

static int charger_therm2temp(int val)
{
#ifdef USE_FLOAT_CONVERSION
  float f1;
  float f2;
  float f3;
  float f4;
  float f5;
  float f6;

  f1 = (float)val;
  f2 = f1 / 4096.f;
  f3 = f2 * 100.f / (1.f - f2);
  f4 = f3 / 100.f;
  f5 = logf(f4);
  f6 = 1.f / (f5 / 4250.f + 1.f / 298.f) - 273.f;

  return (int)f6;
#else
  static short T[29] =
    {
      4020, /* -40,-35,..-20,-15,..,95,100 */
      3986,
      3939,
      3877,
      3759,
      3691,
      3562,
      3405,
      3222,
      3015, /* -40,.. */
      2787,
      2545,
      2296,
      2048,
      1808,
      1582,
      1374,
      1186,
      1020,
       874, /*  10,.. */
       747,
       639,
       546,
       467,
       400,
       343,
       295,
       254,
       220
    };      /*  60,..,100 */

  int i;
  int t0 = -45;
  int t1 = -40;
  int tt = -45;

  for (i = 0; i < 29; i++)
    {
      if (val > T[i])
        {
          break;
        }

      t0 += 5;
      t1 += 5;
    }

  if (i > 0)
    {
      int diff = T[i - 1] - T[i];
      tt = t1 - (val - T[i]) * 5 / diff; /* interpolation : not accurate */
    }

  return tt;
#endif
}

/****************************************************************************
 * Name: charger_get_status
 ****************************************************************************/

static int charger_get_status(enum battery_status_e *status)
{
  uint8_t state;
  int ret;

  if (status == NULL)
    {
      return -EINVAL;
    }

  *status = BATTERY_UNKNOWN;

  ret = cxd56_pmic_getchargestate(&state);
  if (ret < 0)
    {
      return -EIO;
    }

  switch (state)
    {
      /* Is there some fault in the battery? */

      case PMIC_STAT_BAT_UNUSUAL:
        *status = BATTERY_FAULT;
        break;

      /* Is the charging done? */

      case PMIC_STAT_CHG_COMPLETE:
        *status = BATTERY_FULL;
        break;

      /* Is the charging in progress? */

      case PMIC_STAT_GB_QCKCHARGE:
      case PMIC_STAT_GB_LOWCHARGE:
      case PMIC_STAT_GB_HIGHCHARGE:
        *status = BATTERY_CHARGING;
        break;

      /* Is the charging ready? */

      case PMIC_STAT_CHG_STOP:
        *status = BATTERY_DISCHARGING;
        break;

      default:
        _info("Charge state %d\n", state);
        *status = BATTERY_IDLE;
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: charger_get_health
 ****************************************************************************/

static int charger_get_health(enum battery_health_e *health)
{
  struct pmic_gauge_s gauge;
  uint8_t state;
  int temp;
  int ret;

  if (health == NULL)
    {
      return -EINVAL;
    }

  *health = BATTERY_HEALTH_UNKNOWN;

  ret = cxd56_pmic_get_gauge(&gauge);
  if (ret < 0)
    {
      return -EIO;
    }

  ret = cxd56_pmic_getchargestate(&state);
  if (ret < 0)
    {
      return -EIO;
    }

  /* Convert register value to degrees Celsius */

  temp = charger_therm2temp(gauge.temp);

  if (temp < 10)
    {
      *health = BATTERY_HEALTH_COLD;
    }
  else if (temp > 60)
    {
      *health = BATTERY_HEALTH_OVERHEAT;
    }
  else
    {
      *health = BATTERY_HEALTH_GOOD;
    }

  return OK;
}

/****************************************************************************
 * Name: charger_online
 ****************************************************************************/

static int charger_online(bool *online)
{
  if (online == NULL)
    {
      return -EINVAL;
    }

  *online = true;
  return OK;
}

/****************************************************************************
 * Name: charger_get_current
 ****************************************************************************/

static int charger_get_current(int *current)
{
  struct pmic_gauge_s gauge;
  int ret;

  ASSERT(current);

  *current = 0;

  ret = cxd56_pmic_get_gauge(&gauge);
  if (ret < 0)
    {
      return -EIO;
    }

  /* (Register value - 800h) / Current detection resistor (0.1 ohm)
   *    x 0.02929
   */

#ifdef USE_FLOAT_CONVERSION
  *current = (gauge.current - 0x800) / 0.1f * 0.02929f;
#else
  *current = (gauge.current - 0x800) * 2929 / 10000;
#endif

  return OK;
}

/****************************************************************************
 * Name: charger_get_voltage
 ****************************************************************************/

static int charger_get_voltage(int *voltage)
{
  struct pmic_gauge_s gauge;
  int ret;

  ASSERT(voltage);

  *voltage = 0;

  ret = cxd56_pmic_get_gauge(&gauge);
  if (ret < 0)
    {
      return -EIO;
    }

#ifdef USE_FLOAT_CONVERSION
  *voltage = gauge.voltage * 1.12f;
#else
  *voltage = gauge.voltage * 112 / 100;
#endif

  return OK;
}

/****************************************************************************
 * Name: charger_get_temptable
 ****************************************************************************/

static int charger_get_temptable(struct battery_temp_table_s *table)
{
  struct pmic_temp_table_s buf;
  int ret;

  ret = cxd56_pmic_gettemptable(&buf);
  if (ret < 0)
    {
      return -EIO;
    }

  table->T60 = buf.T60;
  table->T45 = buf.T45;
  table->T10 = buf.T10;
  table->T00 = buf.T00;

  return OK;
}

/****************************************************************************
 * Name: charger_set_temptable
 ****************************************************************************/

static int charger_set_temptable(struct battery_temp_table_s *table)
{
  struct pmic_temp_table_s buf;

  buf.T60 = table->T60;
  buf.T45 = table->T45;
  buf.T10 = table->T10;
  buf.T00 = table->T00;

  return cxd56_pmic_settemptable(&buf);
}

/****************************************************************************
 * Name: charger_read
 ****************************************************************************/

static ssize_t charger_read(struct file *filep, char *buffer,
                            size_t buflen)
{
  /* Return nothing read */

  return 0;
}

/****************************************************************************
 * Name: charger_write
 ****************************************************************************/

static ssize_t charger_write(struct file *filep,
                             const char *buffer, size_t buflen)
{
  /* Return nothing written */

  return 0;
}

/****************************************************************************
 * Name: charger_ioctl
 ****************************************************************************/

static int charger_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct charger_dev_s *priv = inode->i_private;
  int ret = -ENOTTY;

  nxmutex_lock(&priv->batlock);

  switch (cmd)
    {
      case BATIOC_STATE:
        {
          enum battery_status_e *status =
            (enum battery_status_e *)(uintptr_t)arg;
          ret = charger_get_status(status);
        }
        break;

      case BATIOC_HEALTH:
        {
          enum battery_health_e *health =
            (enum battery_health_e *)(uintptr_t)arg;
          ret = charger_get_health(health);
        }
        break;

      case BATIOC_ONLINE:
        {
          bool *online = (bool *)(uintptr_t)arg;
          ret = charger_online(online);
        }
        break;

      case BATIOC_VOLTAGE:
        {
          int *voltage = (int *)(uintptr_t)arg;
          ret = cxd56_pmic_setchargevol(*voltage);
        }
        break;

      case BATIOC_CURRENT:
        {
          /* Not supported */

          ret = OK;
        }
        break;

      case BATIOC_INPUT_CURRENT:
        {
          int *current = (int *)(uintptr_t)arg;
          ret = cxd56_pmic_setchargecurrent(*current);
        }
        break;

      case BATIOC_GET_CHGVOLTAGE:
        {
          int *voltage = (int *)(uintptr_t)arg;
          ret = cxd56_pmic_getchargevol(voltage);
        }
        break;

      case BATIOC_GET_CHGCURRENT:
        {
          int *current = (int *)(uintptr_t)arg;
          ret = cxd56_pmic_getchargecurrent(current);
        }
        break;

      case BATIOC_GET_RECHARGEVOL:
        {
          int *voltage = (int *)(uintptr_t)arg;
          ret = cxd56_pmic_getrechargevol(voltage);
        }
        break;

      case BATIOC_SET_RECHARGEVOL:
        {
          int *voltage = (int *)(uintptr_t)arg;
          ret = cxd56_pmic_setrechargevol(*voltage);
        }
        break;

      case BATIOC_GET_COMPCURRENT:
        {
          int *current = (int *)(uintptr_t)arg;
          ret = cxd56_pmic_getchargecompcurrent(current);
        }
        break;

      case BATIOC_GET_TEMPTABLE:
        {
          struct battery_temp_table_s *table =
            (struct battery_temp_table_s *)(uintptr_t)arg;
          ret = charger_get_temptable(table);
        }
        break;

      case BATIOC_SET_TEMPTABLE:
        {
          struct battery_temp_table_s *table =
            (struct battery_temp_table_s *)(uintptr_t)arg;
          ret = charger_set_temptable(table);
        }
        break;

      case BATIOC_SET_COMPCURRENT:
        {
          int *current = (int *)(uintptr_t)arg;
          ret = cxd56_pmic_setchargecompcurrent(*current);
        }
        break;

      case BATIOC_GET_CURRENT:
        {
          int *curr = (int *)(uintptr_t)arg;

          if (curr)
            {
              ret = charger_get_current(curr);
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      case BATIOC_GET_VOLTAGE:
        {
          int *vol = (int *)(uintptr_t)arg;

          if (vol)
            {
              ret = charger_get_voltage(vol);
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->batlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_charger_initialize
 *
 * Description:
 *   Initialize the CXD5247 battery charger driver.
 *
 * Input Parameters:
 *   devpath - Device file path
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_charger_initialize(const char *devpath)
{
  struct charger_dev_s *priv = &g_chargerdev;
  int ret;

  /* Initialize the CXD5247 device structure */

  nxmutex_init(&priv->batlock);

  /* Register battery driver */

  ret = register_driver(devpath, &g_chargerops, 0666, priv);
  if (ret < 0)
    {
      baterr("ERROR: register_driver failed: %d\n", ret);
      return -EFAULT;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_charger_uninitialize
 *
 * Description:
 *   Uninitialize the CXD5247 battery charger driver.
 *
 * Input Parameters:
 *   devpath - Device file path
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_charger_uninitialize(const char *devpath)
{
  unregister_driver(devpath);
  return OK;
}

#endif /* CONFIG_CXD56_CHARGER */
