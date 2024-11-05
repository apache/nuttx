/****************************************************************************
 * drivers/pinctrl/pinctrl.c
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

#include <sys/types.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/pinctrl/pinctrl.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     pinctrl_open(FAR struct file *filep);
static int     pinctrl_close(FAR struct file *filep);
static ssize_t pinctrl_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t pinctrl_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     pinctrl_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pinctrl_drvrops =
{
  pinctrl_open,  /* open */
  pinctrl_close, /* close */
  pinctrl_read,  /* read */
  pinctrl_write, /* write */
  NULL,          /* seek */
  pinctrl_ioctl  /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinctrl_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int pinctrl_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: pinctrl_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int pinctrl_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: pinctrl_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t pinctrl_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: pinctrl_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t pinctrl_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return buflen;
}

/****************************************************************************
 * Name: pinctrl_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int pinctrl_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct pinctrl_dev_s *dev;
  FAR struct pinctrl_param_s *param;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  switch (cmd)
    {
     /* Command:     PINCTRLC_SETFUNCTION
      * Description: Set the mux function of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_param_s
      */

      case PINCTRLC_SETFUNCTION:
        {
          param = (FAR struct pinctrl_param_s *)((uintptr_t)arg);
          ret = PINCTRL_SETFUNCTION(dev, param->pin, param->para.function);
        }
      break;

     /* Command:     PINCTRLC_SETSTRENGTH
      * Description: Set the driver strength of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_param_s
      */

      case PINCTRLC_SETSTRENGTH:
        {
          param = (FAR struct pinctrl_param_s *)((uintptr_t)arg);
          ret = PINCTRL_SETSTRENGTH(dev, param->pin, param->para.strength);
        }
      break;

     /* Command:     PINCTRLC_SETDRIVER
      * Description: Set the driver type of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_param_s
      */

      case PINCTRLC_SETDRIVER:
        {
          param = (FAR struct pinctrl_param_s *)((uintptr_t)arg);
          ret = PINCTRL_SETDRIVER(dev, param->pin, param->para.type);
        }
      break;

     /* Command:     PINCTRLC_SETSLEWRATE
      * Description: Set slewrate of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_param_s
      */

      case PINCTRLC_SETSLEWRATE:
        {
          param = (FAR struct pinctrl_param_s *)((uintptr_t)arg);
          ret = PINCTRL_SETSLEWRATE(dev, param->pin, param->para.slewrate);
        }
      break;

     /* Command:     PINCTRLC_SELECTGPIO
      * Description: Select gpio function of pinctrl pin
      * Argument:    The uint32_t pinctrl number
      */

      case PINCTRLC_SELECTGPIO:
        {
          ret = PINCTRL_SELECTGPIO(dev, arg);
        }
      break;

      /* Unrecognized command */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinctrl_register
 *
 * Description:
 *   Register PINCTRL device driver.
 *
 ****************************************************************************/

int pinctrl_register(FAR struct pinctrl_dev_s *dev, int minor)
{
  char devname[32];

  snprintf(devname, 16, "/dev/pinctrl%u", (unsigned int)minor);
  return register_driver(devname, &g_pinctrl_drvrops, 0666, dev);
}

/****************************************************************************
 * Name: pinctrl_unregister
 *
 * Description:
 *   Unregister PINCTRL device driver.
 *
 ****************************************************************************/

void pinctrl_unregister(FAR struct pinctrl_dev_s *dev, int minor)
{
  char devname[32];

  snprintf(devname, 16, "/dev/pinctrl%u", (unsigned int)minor);
  (void)unregister_driver(devname);
}
