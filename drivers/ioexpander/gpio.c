/****************************************************************************
 * drivers/ioexpander/gpio.c
 *
 *   Copyright (C) 2016, 2018-2019 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/ioexpander/gpio.h>

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     gpio_handler(FAR struct gpio_dev_s *dev, uint8_t pin);
static int     gpio_open(FAR struct file *filep);
static int     gpio_close(FAR struct file *filep);
static ssize_t gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static off_t   gpio_seek(FAR struct file *filep, off_t offset, int whence);
static int     gpio_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_gpio_drvrops =
{
  gpio_open,   /* open */
  gpio_close,  /* close */
  gpio_read,   /* read */
  gpio_write,  /* write */
  gpio_seek,   /* seek */
  gpio_ioctl,  /* ioctl */
  NULL         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL       /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_handler
 *
 * Description:
 *   GPIO interrupt callback function.
 *
 ****************************************************************************/

static int gpio_handler(FAR struct gpio_dev_s *dev, uint8_t pin)
{
  int i;

  DEBUGASSERT(dev != NULL);

  for (i = 0; i < CONFIG_DEV_GPIO_NSIGNALS; i++)
    {
      FAR struct gpio_signal_s *signal = &dev->gp_signals[i];

      if (signal->gp_pid == 0)
        {
          break;
        }

      nxsig_notification(signal->gp_pid, &signal->gp_event,
                         SI_QUEUE, &signal->gp_work);
    }

  return OK;
}

/****************************************************************************
 * Name: gpio_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int gpio_open(FAR struct file *filep)
{
  filep->f_pos = 0;
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
  FAR struct inode *inode;
  FAR struct gpio_dev_s *dev;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  DEBUGASSERT(buffer != NULL);
  if (buflen == 0)
    {
      return 0;  /* Zero will be interpreted as the End-of-File. */
    }

  /* Check for End-of-File.
   *
   * REVISIT:  Returning End-of-File after one byte has been written permits
   * you to cat or echo the GPIO.  This, however, is an un-natural use of a
   * file position since there is no file position associated with a GPIO.
   * It also makes the read() method difficult to use programmatically.
   */

  if (filep->f_pos > 0)
    {
      return 0;
    }

  /* Read the GPIO value */

  ret = dev->gp_ops->go_read(dev, (FAR bool *)&buffer[0]);
  if (ret < 0)
    {
      return ret;
    }

  /* Convert the GPIO value to ASCII and increment the file position */

  buffer[0]    += '0';
  filep->f_pos  = 1;
  return 1;
}

/****************************************************************************
 * Name: gpio_write
 *
 * Description:
 *   Standard character driver write method.
 *
 *   REVISIT:  The read() method obeys the semantics of a file position and
 *   requires re-opening the driver or seeking to address 0.  The write()
 *   method does not.  This is an inconsistency.
 *
 ****************************************************************************/

static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct gpio_dev_s *dev;
  int ret;
  bool val;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  DEBUGASSERT(buffer != NULL);

  /* Check if this pin is write-able */

  if (dev->gp_pintype != GPIO_OUTPUT_PIN &&
      dev->gp_pintype != GPIO_OUTPUT_PIN_OPENDRAIN)
    {
      return -EACCES;
    }

  /* Verfy that a buffer containing data was provided */

  DEBUGASSERT(buffer != NULL);
  if (buflen != 0)
    {
      /* Only values '0' and '1' can be written */

      if (buffer[0] == '0')
        {
          val = 0;
        }
      else if (buffer[0] == '1')
        {
          val = 1;
        }
      else
        {
          return -EINVAL;
        }

      /* Write the GPIO value */

      ret = dev->gp_ops->go_write(dev, val);
      if (ret < 0)
        {
          return ret;
        }

      /* One byte written */

      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: gpio_seek
 *
 * Description:
 *   Reset read flag on seek to 0
 *
 *   REVISIT:  Seeking address zero is required to return addition GPIO
 *   values from read().  This, however, is an un-natural use of a file
 *   position since there is no file position associated with a GPIO.  It
 *   also makes the read() method difficult to use programmatically.
 *
 ****************************************************************************/

static off_t gpio_seek(FAR struct file *filep, off_t offset, int whence)
{
  /* Only SEEK_SET is supported, return ENOSYS for other valid options */

  if (whence == SEEK_CUR || whence == SEEK_END)
    {
      return -ENOSYS;
    }

  /* Only Offset zero makes sense,  POSIX permits setting the file position
   * beyond the end of the file, but that makes little sense here.
   */

  if (whence == SEEK_SET && offset == 0)
    {
      filep->f_pos = 0;
      return 0;
    }

  return -EINVAL;
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
  FAR struct gpio_dev_s *dev;
  irqstate_t flags;
  pid_t pid;
  int ret;
  int i;
  int j = 0;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  switch (cmd)
    {
      /* Command:     GPIOC_WRITE
       * Description: Set the value of an output GPIO
       * Argument:    0=output a low value; 1=outut a high value
       */

      case GPIOC_WRITE:
        if (dev->gp_pintype == GPIO_OUTPUT_PIN ||
            dev->gp_pintype == GPIO_OUTPUT_PIN_OPENDRAIN)
          {
            DEBUGASSERT(arg == 0ul || arg == 1ul);
            ret = dev->gp_ops->go_write(dev, (bool)arg);
          }
        else
          {
            ret = -EACCES;
          }
        break;

      /* Command:     GPIOC_READ
       * Description: Read the value of an input or output GPIO
       * Argument:    A pointer to an bool value to receive the result:
       *              false=low value; true=high value.
       */

      case GPIOC_READ:
        {
          FAR bool *ptr = (FAR bool *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          ret = dev->gp_ops->go_read(dev, ptr);
          DEBUGASSERT(ret < 0 || *ptr == 0 || *ptr == 1);
        }
        break;

      /* Command:     GPIOC_PINTYPE
       * Description: Return the GPIO pin type.
       * Argument:    A pointer to an instance of type enum gpio_pintype_e
       */

      case GPIOC_PINTYPE:
        {
          FAR enum gpio_pintype_e *ptr =
            (FAR enum gpio_pintype_e *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = dev->gp_pintype;
          ret = OK;
        }
        break;

      /* Command:     GPIOC_REGISTER
       * Description: Register to receive a signal whenever there an
       *              interrupt is received on an input gpio pin.  This
       *              feature, of course, depends upon interrupt GPIO
       *              support from the platform.
       * Argument:    The number of signal to be generated when the
       *              interrupt occurs.
       */

      case GPIOC_REGISTER:
        if (arg && dev->gp_pintype >= GPIO_INTERRUPT_PIN)
          {
            pid = getpid();
            flags = spin_lock_irqsave();
            for (i = 0; i < CONFIG_DEV_GPIO_NSIGNALS; i++)
              {
                FAR struct gpio_signal_s *signal = &dev->gp_signals[i];

                if (signal->gp_pid == 0 || signal->gp_pid == pid)
                  {
                    memcpy(&signal->gp_event, (FAR void *)arg,
                           sizeof(signal->gp_event));
                    signal->gp_pid = pid;
                    ret = OK;
                    break;
                  }
              }

            spin_unlock_irqrestore(flags);

            if (i == 0)
              {
                /* Register our handler */

                ret = dev->gp_ops->go_attach(dev,
                                             (pin_interrupt_t)gpio_handler);
                if (ret >= 0)
                  {
                    /* Enable pin interrupts */

                    ret = dev->gp_ops->go_enable(dev, true);
                  }
              }
            else if (i == CONFIG_DEV_GPIO_NSIGNALS)
              {
                ret = -EBUSY;
              }
          }
        else
          {
            ret = -EACCES;
          }
        break;

      /* Command:     GPIOC_UNREGISTER
       * Description: Stop receiving signals for pin interrupts.
       * Argument:    None.
       */

      case GPIOC_UNREGISTER:
        if (dev->gp_pintype >= GPIO_INTERRUPT_PIN)
          {
            pid = getpid();
            flags = spin_lock_irqsave();
            for (i = 0; i < CONFIG_DEV_GPIO_NSIGNALS; i++)
              {
                if (pid == dev->gp_signals[i].gp_pid)
                  {
                    for (j = i + 1; j < CONFIG_DEV_GPIO_NSIGNALS; j++)
                      {
                        if (dev->gp_signals[j].gp_pid == 0)
                          {
                            break;
                          }
                      }

                    if (i != --j)
                      {
                        memcpy(&dev->gp_signals[i], &dev->gp_signals[j],
                               sizeof(dev->gp_signals[i]));
                      }

                    dev->gp_signals[j].gp_pid = 0;
                    nxsig_cancel_notification(&dev->gp_signals[j].gp_work);
                    ret = OK;
                    break;
                  }
                }

            spin_unlock_irqrestore(flags);

            if (i == 0 && j == 0)
              {
                /* Make sure that the pin interrupt is disabled */

                ret = dev->gp_ops->go_enable(dev, false);
                if (ret >= 0)
                  {
                    /* Detach the handler */

                    ret = dev->gp_ops->go_attach(dev, NULL);
                  }
              }
            else if (i == CONFIG_DEV_GPIO_NSIGNALS)
              {
                ret = -EINVAL;
              }
          }
        else
          {
            ret = -EACCES;
          }
        break;

      /* Command:     GPIOC_SETPINTYPE
       * Description: Set the GPIO pin type.
       * Argument:    The enum gpio_pintype_e type.
       */

      case GPIOC_SETPINTYPE:
        {
          ret = dev->gp_ops->go_setpintype(dev, arg);
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
 * Name: gpio_pin_register
 *
 * Description:
 *   Register GPIO pin device driver.
 *
 *   - Input pin types will be registered at /dev/gpinN
 *   - Output pin types will be registered at /dev/gpoutN
 *   - Interrupt pin types will be registered at /dev/gpintN
 *
 *   Where N is the provided minor number in the range of 0-99.
 *
 ****************************************************************************/

int gpio_pin_register(FAR struct gpio_dev_s *dev, int minor)
{
  FAR const char *fmt;
  char devname[16];
  int ret;

  DEBUGASSERT(dev != NULL && dev->gp_ops != NULL &&
              (unsigned int)minor < 100);

  switch (dev->gp_pintype)
    {
      case GPIO_INPUT_PIN:
      case GPIO_INPUT_PIN_PULLUP:
      case GPIO_INPUT_PIN_PULLDOWN:
        {
          DEBUGASSERT(dev->gp_ops->go_read != NULL);
          fmt = "/dev/gpin%u";
        }
        break;

      case GPIO_OUTPUT_PIN:
      case GPIO_OUTPUT_PIN_OPENDRAIN:
        {
          DEBUGASSERT(dev->gp_ops->go_read != NULL &&
                     dev->gp_ops->go_write != NULL);
          fmt = "/dev/gpout%u";
        }
        break;

      default:
        {
          DEBUGASSERT(dev->gp_ops->go_read != NULL &&
                      dev->gp_ops->go_attach != NULL &&
                      dev->gp_ops->go_enable != NULL);

          /* Make sure that the pin interrupt is disabled */

          ret = dev->gp_ops->go_enable(dev, false);
          if (ret < 0)
            {
              return ret;
            }

          fmt = "/dev/gpint%u";
        }
        break;
    }

  snprintf(devname, 16, fmt, (unsigned int)minor);
  gpioinfo("Registering %s\n", devname);

  return register_driver(devname, &g_gpio_drvrops, 0666, dev);
}

/****************************************************************************
 * Name: gpio_pin_unregister
 *
 * Description:
 *   Unregister GPIO pin device driver.
 *
 *   - Input pin types will be registered at /dev/gpinN
 *   - Output pin types will be registered at /dev/gpoutN
 *   - Interrupt pin types will be registered at /dev/gpintN
 *
 *   Where N is the provided minor number in the range of 0-99.
 *
 *
 ****************************************************************************/

void gpio_pin_unregister(FAR struct gpio_dev_s *dev, int minor)
{
  FAR const char *fmt;
  char devname[16];

  switch (dev->gp_pintype)
    {
      case GPIO_INPUT_PIN:
      case GPIO_INPUT_PIN_PULLUP:
      case GPIO_INPUT_PIN_PULLDOWN:
        {
          fmt = "/dev/gpin%u";
        }
        break;

      case GPIO_OUTPUT_PIN:
      case GPIO_OUTPUT_PIN_OPENDRAIN:
        {
          fmt = "/dev/gpout%u";
        }
        break;

      default:
        {
          fmt = "/dev/gpint%u";
        }
        break;
    }

  snprintf(devname, 16, fmt, (unsigned int)minor);
  gpioinfo("Unregistering %s\n", devname);

  unregister_driver(devname);
}

#endif /* CONFIG_DEV_GPIO */
