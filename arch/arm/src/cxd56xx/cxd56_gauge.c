/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gauge.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* CXD5247GF is Li-Ion Battery Charger with Power-Path Management. */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
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
#include <nuttx/power/battery_gauge.h>
#include <nuttx/power/battery_ioctl.h>

#include <arch/chip/battery_ioctl.h>

#include "cxd56_pmic.h"

#ifdef CONFIG_CXD56_GAUGE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CXD56_GAUGE_DEBUG
#define baterr(fmt, ...) logerr(fmt, ## __VA_ARGS__)
#define batdbg(fmt, ...) logdebug(fmt, ## __VA_ARGS__)
#else
#define baterr(fmt, ...)
#define batdbg(fmt, ...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bat_gauge_dev_s
{
  sem_t batsem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gauge_open(FAR struct file *filep);
static int gauge_close(FAR struct file *filep);
static ssize_t gauge_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t gauge_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen);
static int gauge_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_gaugeops =
{
  gauge_open,   /* open */
  gauge_close,  /* close */
  gauge_read,   /* read */
  gauge_write,  /* write */
  0,            /* seek */
  gauge_ioctl   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL        /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

static struct bat_gauge_dev_s g_gaugedev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gauge_get_status
 ****************************************************************************/

static int gauge_get_status(FAR enum battery_gauge_status_e *status)
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

      /* Is the discharging */

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
 * Name: gauge_get_vol
 ****************************************************************************/

static int gauge_get_vol(FAR b16_t *voltage)
{
  struct pmic_gauge_s gauge;
  int ret;

  if (voltage == NULL)
    {
      return -EINVAL;
    }

  *voltage = 0;

  ret = cxd56_pmic_get_gauge(&gauge);
  if (ret < 0)
    {
      return -EIO;
    }

  *voltage = gauge.voltage * 112 / 100;
  return OK;
}

/****************************************************************************
 * Name: gauge_get_capacity
 ****************************************************************************/

static int gauge_get_capacity(FAR b16_t *capacity)
{
  b16_t vol;
  int lower;
  int upper;
  int ret;

  if (capacity == NULL)
    {
      return -EINVAL;
    }

  /* Get current battery voltage and upper/lower limit settings from PMIC. */

  ret = gauge_get_vol(&vol);
  if (ret < 0)
    {
      return -EIO;
    }

  ret = cxd56_pmic_getchargevol(&upper);
  if (ret < 0)
    {
      return -EIO;
    }

  ret = cxd56_pmic_getlowervol(&lower);
  if (ret < 0)
    {
      return -EIO;
    }

  /* Calculate capacity (0-100%)
   * Actually, battery voltage possible to be under lower limit voltage.
   *
   * NOTE: This logic is tentative, linear from lower to upper. But it
   * depends on the battery. Thus, user should be apply a voltage
   * characteristic for capacity calculation if you want to more accuracy.
   */

  if (vol > lower)
    {
      upper -= lower;
      vol -= lower;
      *capacity = (vol * 100) / upper;
    }
  else
    {
      *capacity = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: gauge_online
 ****************************************************************************/

static int gauge_online(FAR bool *online)
{
  if (online == NULL)
    {
      return -EINVAL;
    }

  *online = true;
  return OK;
}

/****************************************************************************
 * Name: gauge_open
 *
 * Description:
 *   This function is called whenever the battery device is opened.
 *
 ****************************************************************************/

static int gauge_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: gauge_close
 *
 * Description:
 *   This routine is called when the battery device is closed.
 *
 ****************************************************************************/

static int gauge_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: gauge_read
 ****************************************************************************/

static ssize_t gauge_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  /* Return nothing read */

  return 0;
}

/****************************************************************************
 * Name: gauge_write
 ****************************************************************************/

static ssize_t gauge_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen)
{
  /* Return nothing written */

  return 0;
}

/****************************************************************************
 * Name: gauge_ioctl
 ****************************************************************************/

static int gauge_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bat_gauge_dev_s *priv  = inode->i_private;
  int ret = -ENOTTY;

  nxsem_wait_uninterruptible(&priv->batsem);

  switch (cmd)
    {
      case BATIOC_STATE:
        {
          FAR enum battery_gauge_status_e *status =
            (FAR enum battery_gauge_status_e *)(uintptr_t)arg;
          ret = gauge_get_status(status);
        }
        break;

      case BATIOC_VOLTAGE:
        {
          FAR b16_t *voltage = (FAR b16_t *)(uintptr_t)arg;
          ret = gauge_get_vol(voltage);
        }
        break;

      case BATIOC_CAPACITY:
        {
          FAR b16_t *capacity = (FAR b16_t *)(uintptr_t)arg;
          ret = gauge_get_capacity(capacity);
        }
        break;

      case BATIOC_ONLINE:
        {
          FAR bool *online = (FAR bool *)(uintptr_t)arg;
          ret = gauge_online(online);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->batsem);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_gauge_initialize
 *
 * Description:
 *   Initialize the CXD5247 battery driver.
 *
 * Input Parameters:
 *   devpath - Device file path
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_gauge_initialize(FAR const char *devpath)
{
  FAR struct bat_gauge_dev_s *priv = &g_gaugedev;
  int ret;

  /* Initialize the CXD5247 device structure */

  nxsem_init(&priv->batsem, 0, 1);

  /* Register battery driver */

  ret = register_driver(devpath, &g_gaugeops, 0666, priv);
  if (ret < 0)
    {
      _err("ERROR: register_driver failed: %d\n", ret);
      return -EFAULT;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_gauge_uninitialize
 *
 * Description:
 *   Uninitialize the CXD5247 battery driver.
 *
 * Input Parameters:
 *   devpath - Device file path
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_gauge_uninitialize(FAR const char *devpath)
{
  unregister_driver(devpath);
  return OK;
}

#endif /* CONFIG_CXD56_GAUGE */
