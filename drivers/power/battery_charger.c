/****************************************************************************
 * drivers/power/battery_charger.c
 * Upper-half, character driver for batteries charger.
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>

/* This driver requires:
 *
 * CONFIG_BATTERY_CHARGER - Upper half battery driver support
 */

#if defined(CONFIG_BATTERY_CHARGER)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     bat_charger_open(FAR struct file *filep);
static int     bat_charger_close(FAR struct file *filep);
static ssize_t bat_charger_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t bat_charger_write(FAR struct file *filep,
                 FAR const char *buffer, size_t buflen);
static int     bat_charger_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_batteryops =
{
  bat_charger_open,
  bat_charger_close,
  bat_charger_read,
  bat_charger_write,
  NULL,
  bat_charger_ioctl,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bat_charger_open
 *
 * Description:
 *   This function is called whenever the battery device is opened.
 *
 ****************************************************************************/

static int bat_charger_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: bat_charger_close
 *
 * Description:
 *   This routine is called when the battery device is closed.
 *
 ****************************************************************************/

static int bat_charger_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: bat_charger_read
 ****************************************************************************/

static ssize_t bat_charger_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  /* Return nothing read */

  return 0;
}

/****************************************************************************
 * Name: bat_charger_write
 ****************************************************************************/

static ssize_t bat_charger_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen)
{
  /* Return nothing written */

  return 0;
}

/****************************************************************************
 * Name: bat_charger_ioctl
 ****************************************************************************/

static int bat_charger_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct battery_charger_dev_s *dev  = inode->i_private;
  int ret;

  /* Enforce mutually exclusive access to the battery driver */

  ret = nxsem_wait(&dev->batsem);
  if (ret < 0)
    {
      return ret; /* Probably -EINTR */
    }

  /* Process the IOCTL command */

  ret = -EINVAL;  /* Assume a bad argument */
  switch (cmd)
    {
      case BATIOC_STATE:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->state(dev, ptr);
            }
        }
        break;

      case BATIOC_HEALTH:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->health(dev, ptr);
            }
        }
        break;

      case BATIOC_ONLINE:
        {
          FAR bool *ptr = (FAR bool *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->online(dev, ptr);
            }
        }
        break;

      case BATIOC_VOLTAGE:
        {
          int volts;
          FAR int *voltsp = (FAR int *)((uintptr_t)arg);
          if (voltsp)
            {
              volts = *voltsp;
              ret = dev->ops->voltage(dev, volts);
            }
        }
        break;

      case BATIOC_CURRENT:
        {
          int amps;
          FAR int *ampsp = (FAR int *)((uintptr_t)arg);
          if (ampsp)
            {
              amps = *ampsp;
              ret = dev->ops->current(dev, amps);
            }
        }
        break;

      case BATIOC_INPUT_CURRENT:
        {
          int amps;
          FAR int *ampsp = (FAR int *)((uintptr_t)arg);
          if (ampsp)
            {
              amps = *ampsp;
              ret = dev->ops->input_current(dev, amps);
            }
        }
        break;

      case BATIOC_OPERATE:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->operate(dev, (uintptr_t)arg);
            }
        }
        break;

      default:
        _err("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&dev->batsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: battery_charger_register
 *
 * Description:
 *   Register a lower half battery driver with the common, upper-half
 *   battery driver.
 *
 * Input Parameters:
 *   devpath - The location in the pseudo-filesystem to create the driver.
 *     Recommended standard is "/dev/bat0", "/dev/bat1", etc.
 *   dev - An instance of the battery state structure .
 *
 * Returned Value:
 *    Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int battery_charger_register(FAR const char *devpath,
                             FAR struct battery_charger_dev_s *dev)
{
  int ret;

  /* Initialize the semaphore */

  nxsem_init(&dev->batsem, 0, 1);

  /* Register the character driver */

  ret = register_driver(devpath, &g_batteryops, 0555, dev);
  if (ret < 0)
    {
      _err("ERROR: Failed to register driver: %d\n", ret);
    }

  return ret;
}
#endif /* CONFIG_BATTERY_CHARGER */
