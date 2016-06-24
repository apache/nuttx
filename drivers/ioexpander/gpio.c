/****************************************************************************
 * drivers/ioexpander/gpio.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/ioexpander/gpio.h>

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     gpio_open(FAR struct file *filep);
static int     gpio_close(FAR struct file *filep);
static ssize_t gpio_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     gpio_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_gpio_input_ops =
{
  gpio_open,  /* open */
  gpio_close, /* close */
  gpio_read,  /* read */
  NULL,       /* write */
  NULL,       /* seek */
  gpio_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL      /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
};

static const struct file_operations g_gpio_output_ops =
{
  gpio_open,  /* open */
  gpio_close, /* close */
  NULL,       /* read */
  gpio_write, /* write */
  NULL,       /* seek */
  gpio_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL      /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int gpio_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: gpio_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int gpio_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: gpio_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: gpio_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return (ssize_t)buflen;
}

/****************************************************************************
 * Name: gpio_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int gpio_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct gpio_common_dev_s *dev;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  switch (cmd)
  {
  /* Command:     GPIO_WRITE
   * Description: Set the value of an output GPIO
   * Argument:    0=output a low value; 1=outut a high value
   */

  case GPIO_WRITE:
    if (dev->gp_output)
      {
        FAR struct gpio_output_dev_s *outdev =
          (FAR struct gpio_output_dev_s *)dev;

        DEBUGASSERT(outdev->gpout_write != NULL &&
                    ((arg == 0UL) || (arg == 1UL)));
        ret = outdev->gpout_write(outdev, (int)arg);
      }
    else
      {
        ret = -EACCES;
      }
    break;

  /* Command:     GPIO_READ
   * Description: Read the value of an input or output GPIO
   * Argument:    A pointer to an integer value to receive the result:
   *              0=low value; 1=high value.
   */

  case GPIO_READ:
    if (dev->gp_output)
      {
        FAR struct gpio_output_dev_s *outdev =
          (FAR struct gpio_output_dev_s *)dev;

        DEBUGASSERT(outdev->gpout_read != NULL);
        ret = outdev->gpout_read(outdev);
      }
    else
      {
        FAR struct gpio_input_dev_s *indev =
          (FAR struct gpio_input_dev_s *)dev;

        DEBUGASSERT(indev->gpin_read != NULL);
        ret = indev->gpin_read(indev);
      }
    break;
  }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_input_register
 *
 * Description:
 *   Register GPIO input pin device driver.
 *
 ****************************************************************************/

int gpio_input_register(FAR struct gpio_input_dev_s *dev, int minor)
{
  char devname[16];

  DEBUGASSERT(dev != NULL && !dev->gpin_output && dev->gpin_read != NULL &&
              (unsigned int)minor < 100);

  snprintf(devname, 16, "/dev/gpin%u", (unsigned int)minor);
  return register_driver(devname, &g_gpio_input_ops, 0444, dev);
}

/****************************************************************************
 * Name: gpio_output_register
 *
 * Description:
 *   Register GPIO output pin device driver.
 *
 ****************************************************************************/

int gpio_output_register(FAR struct gpio_output_dev_s *dev, int minor)
{
  char devname[16];

  DEBUGASSERT(dev != NULL && dev->gpout_output && dev->gpout_read != NULL &&
              dev->gpout_write != NULL &&(unsigned int)minor < 100);

  snprintf(devname, 16, "/dev/gpout%u", (unsigned int)minor);
  return register_driver(devname, &g_gpio_output_ops, 0222, dev);
}

#endif /* CONFIG_DEV_GPIO */
