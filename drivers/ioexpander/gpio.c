/****************************************************************************
 * drivers/ioexpander/gpio.c
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
#include <string.h>
#include <signal.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>

#include <nuttx/fs/fs.h>
#include <nuttx/spinlock.h>
#include <nuttx/ioexpander/gpio.h>

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     gpio_handler(FAR struct gpio_dev_s *dev, uint8_t pin);
static int     gpio_open(FAR struct file *filep);
static ssize_t gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static off_t   gpio_seek(FAR struct file *filep, off_t offset, int whence);
static int     gpio_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int     gpio_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_gpio_drvrops =
{
  gpio_open,   /* open */
  NULL,        /* close */
  gpio_read,   /* read */
  gpio_write,  /* write */
  gpio_seek,   /* seek */
  gpio_ioctl,  /* ioctl */
  NULL,        /* mmap */
  NULL,        /* truncate */
  gpio_poll,   /* poll */
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
#if CONFIG_DEV_GPIO_NSIGNALS > 0
  int i;
#endif

  DEBUGASSERT(dev != NULL);

  dev->int_count++;

#if CONFIG_DEV_GPIO_NPOLLWAITERS > 0
  poll_notify(dev->fds, CONFIG_DEV_GPIO_NPOLLWAITERS, POLLIN);
#endif

#if CONFIG_DEV_GPIO_NSIGNALS > 0
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
#endif

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
  FAR struct inode *inode;
  FAR struct gpio_dev_s *dev;

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  filep->f_priv = (FAR void *)dev->int_count;
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

  /* Update interrupt count and read the GPIO value */

  filep->f_priv = (FAR void *)dev->int_count;

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

  /* Verify that a buffer containing data was provided */

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
  int ret = OK;
#if CONFIG_DEV_GPIO_NSIGNALS > 0
  pid_t pid;
  int i;
  int j;
#endif

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;
  DEBUGASSERT(dev->gp_ops != NULL);

  switch (cmd)
    {
      /* Command:     GPIOC_WRITE
       * Description: Set the value of an output GPIO
       * Argument:    0=output a low value; 1=output a high value
       */

      case GPIOC_WRITE:
        if (dev->gp_pintype == GPIO_OUTPUT_PIN ||
            dev->gp_pintype == GPIO_OUTPUT_PIN_OPENDRAIN)
          {
            DEBUGASSERT(arg == 0ul || arg == 1ul);
            DEBUGASSERT(dev->gp_ops->go_write != NULL);
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

          filep->f_priv = (FAR void *)dev->int_count;

          DEBUGASSERT(dev->gp_ops->go_read != NULL);
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

          *ptr = (enum gpio_pintype_e)dev->gp_pintype;
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
        if (dev->gp_pintype >= GPIO_INTERRUPT_PIN)
          {
            flags = enter_critical_section();
#if CONFIG_DEV_GPIO_NSIGNALS > 0
            if (arg)
              {
                pid = nxsched_getpid();
                for (i = 0; i < CONFIG_DEV_GPIO_NSIGNALS; i++)
                  {
                    FAR struct gpio_signal_s *signal = &dev->gp_signals[i];

                    if (signal->gp_pid == 0 || signal->gp_pid == pid)
                      {
                        memcpy(&signal->gp_event, (FAR void *)arg,
                               sizeof(signal->gp_event));
                        signal->gp_pid = pid;
                        break;
                      }
                  }

                if (i == CONFIG_DEV_GPIO_NSIGNALS)
                  {
                    leave_critical_section(flags);
                    ret = -EBUSY;
                    break;
                  }
              }
#endif

            if (dev->register_count++ > 0)
              {
                leave_critical_section(flags);
                break;
              }

            leave_critical_section(flags);

            /* Register our handler */

            DEBUGASSERT(dev->gp_ops->go_attach != NULL);
            ret = dev->gp_ops->go_attach(dev,
                                         (pin_interrupt_t)gpio_handler);
            if (ret >= 0)
              {
                /* Enable pin interrupts */

                DEBUGASSERT(dev->gp_ops->go_enable != NULL);
                ret = dev->gp_ops->go_enable(dev, true);
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
            flags = enter_critical_section();
#if CONFIG_DEV_GPIO_NSIGNALS > 0
            pid = nxsched_getpid();
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
                    break;
                  }
              }
#endif

            if (--dev->register_count > 0)
              {
                leave_critical_section(flags);
                break;
              }

            leave_critical_section(flags);

            /* Make sure that the pin interrupt is disabled */

            DEBUGASSERT(dev->gp_ops->go_enable != NULL);
            ret = dev->gp_ops->go_enable(dev, false);
            if (ret >= 0)
              {
                /* Detach the handler */

                DEBUGASSERT(dev->gp_ops->go_attach != NULL);
                ret = dev->gp_ops->go_attach(dev, NULL);
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
          enum gpio_pintype_e pintype = (enum gpio_pintype_e)arg;

          /* Check if the argument is a valid pintype */

          if (pintype >= GPIO_NPINTYPES)
            {
              ret = -EINVAL;
              break;
            }

          /* Check if the pintype actually needs to be changed */

          if (dev->gp_pintype == pintype)
            {
              /* Pintype remains the same, no need to change anything */

              break;
            }

          /* Disable interrupt if pin had an interrupt pintype previously */

          if (dev->gp_pintype >= GPIO_INTERRUPT_PIN)
            {
              DEBUGASSERT(dev->gp_ops->go_enable != NULL);
              ret = dev->gp_ops->go_enable(dev, false);
              if (ret < 0)
                {
                  break;
                }
            }

          /* Change pintype */

          DEBUGASSERT(dev->gp_ops->go_setpintype != NULL);
          ret = dev->gp_ops->go_setpintype(dev, pintype);
          if (ret < 0)
            {
              break;
            }

          /* Additional DEBUGASSERTs to make sure the right operations are
           * available after the pintype has been changed.
           */

          DEBUGASSERT(dev->gp_pintype == pintype);
          DEBUGASSERT(dev->gp_ops != NULL);
          DEBUGASSERT(dev->gp_ops->go_read != NULL);
          DEBUGASSERT(dev->gp_ops->go_setpintype != NULL);

          if (pintype >= GPIO_OUTPUT_PIN && pintype < GPIO_INTERRUPT_PIN)
            {
              DEBUGASSERT(dev->gp_ops->go_write != NULL);
            }
          else if (pintype >= GPIO_INTERRUPT_PIN)
            {
              DEBUGASSERT(dev->gp_ops->go_attach != NULL);
              DEBUGASSERT(dev->gp_ops->go_enable != NULL);
            }
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
 * Name: gpio_poll
 *
 * Description:
 *   Poll method for gpio device.
 *
 ****************************************************************************/

static int gpio_poll(FAR struct file *filep,
                     FAR struct pollfd *fds, bool setup)
{
#if CONFIG_DEV_GPIO_NPOLLWAITERS > 0
  FAR struct inode *inode = filep->f_inode;
  FAR struct gpio_dev_s *dev = inode->i_private;
  int i;
#endif

  irqstate_t flags;
  int ret = OK;

  /* Are we setting up the poll?  Or tearing it down? */

  flags = enter_critical_section();
  if (setup)
    {
#if CONFIG_DEV_GPIO_NPOLLWAITERS > 0
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_DEV_GPIO_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (dev->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              dev->fds[i] = fds;
              fds->priv   = &dev->fds[i];

              /* Report if a event is pending */

              if (dev->int_count != (uintptr_t)(filep->f_priv))
                {
                  poll_notify(&fds, 1, POLLIN);
                }

              break;
            }
        }

      if (i >= CONFIG_DEV_GPIO_NPOLLWAITERS)
#endif
        {
          fds->priv = NULL;
          ret       = -EBUSY;
        }
    }
  else if (fds->priv != NULL)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_pin_register
 *
 * Description:
 *   Register GPIO pin device driver at /dev/gpioN, where N is the provided
 *   minor number.
 *
 * Input Parameters:
 *   dev    - A pointer to a gpio_dev_s
 *   minor  - An integer value to be concatenated with '/dev/gpio'
 *            to form the device name.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *   all error values returned by inode_reserve:
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int gpio_pin_register(FAR struct gpio_dev_s *dev, int minor)
{
  char devname[16];

  snprintf(devname, sizeof(devname), "gpio%u", (unsigned int)minor);
  return gpio_pin_register_byname(dev, devname);
}

/****************************************************************************
 * Name: gpio_pin_register_byname
 *
 * Description:
 *   Register GPIO pin device driver with it's pin name.
 *
 * Input Parameters:
 *   dev      - A pointer to a gpio_dev_s
 *   pinname  - A pointer to the name to be concatenated with '/dev/'
 *              to form the device name.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *   all error values returned by inode_reserve:
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int gpio_pin_register_byname(FAR struct gpio_dev_s *dev,
                             FAR const char *pinname)
{
  char devname[32];
  int ret;

  DEBUGASSERT(dev != NULL && dev->gp_ops != NULL && pinname != NULL);

  switch (dev->gp_pintype)
    {
      case GPIO_INPUT_PIN:
      case GPIO_INPUT_PIN_PULLUP:
      case GPIO_INPUT_PIN_PULLDOWN:
        {
          DEBUGASSERT(dev->gp_ops->go_read != NULL);
        }
        break;

      case GPIO_OUTPUT_PIN:
      case GPIO_OUTPUT_PIN_OPENDRAIN:
        {
          DEBUGASSERT(dev->gp_ops->go_read != NULL &&
                      dev->gp_ops->go_write != NULL);
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
        }
        break;
    }

  snprintf(devname, sizeof(devname), "/dev/%s", pinname);

  gpioinfo("Registering %s\n", devname);

  return register_driver(devname, &g_gpio_drvrops, 0666, dev);
}

/****************************************************************************
 * Name: gpio_pin_unregister
 *
 * Description:
 *   Unregister GPIO pin device driver at /dev/gpioN, where N is the provided
 *   minor number.
 *
 * Input Parameters:
 *   dev    - A pointer to a gpio_dev_s
 *   minor  - An integer value to be concatenated with '/dev/gpio'
 *            to form the device name.
 *
 * Returned Value:
 *   Zero on success; A negated value is returned on a failure
 *   (all error values returned by inode_remove):
 *
 *   ENOENT - path does not exist.
 *   EBUSY  - Ref count is not 0;
 *
 ****************************************************************************/

int gpio_pin_unregister(FAR struct gpio_dev_s *dev, int minor)
{
  char devname[16];
  snprintf(devname, sizeof(devname), "gpio%u", (unsigned int)minor);
  return gpio_pin_unregister_byname(dev, devname);
}

/****************************************************************************
 * Name: gpio_pin_unregister_byname
 *
 * Description:
 *   Unregister GPIO pin device driver at /dev/pinname.
 *
 * Input Parameters:
 *   dev      - A pointer to a gpio_dev_s
 *   pinname  - A pointer to the name to be concatenated with '/dev/'
 *              to form the device name.
 *
 *
 * Returned Value:
 *   Zero on success; A negated value is returned on a failure
 *   (all error values returned by inode_remove):
 *
 *   ENOENT - path does not exist.
 *   EBUSY  - Ref count is not 0;
 ****************************************************************************/

int gpio_pin_unregister_byname(FAR struct gpio_dev_s *dev,
                               FAR const char *pinname)
{
  char devname[32];

  snprintf(devname, sizeof(devname), "/dev/%s", pinname);

  gpioinfo("Unregistering %s\n", devname);

  return unregister_driver(devname);
}

#endif /* CONFIG_DEV_GPIO */
