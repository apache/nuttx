/****************************************************************************
 * drivers/lcd/ft80x.c
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

/* References:
 *  - Document No.: FT_000792, "FT800 Embedded Video Engine", Datasheet
 *    Version 1.1, Clearance No.: FTDI# 334, Future Technology Devices
 *    International Ltd.
 *  - Document No.: FT_000986, "FT801 Embedded Video Engine Datasheet",
 *    Version 1.0, Clearance No.: FTDI#376, Future Technology Devices
 *    International Ltd.
 *  - Application Note AN_240AN_240, "FT800 From the Ground Up", Version
 *    1.1, Issue Date: 2014-06-09, Future Technology Devices International
 *    Ltd.
 *  - "FT800 Series Programmer Guide Guide", Version 2.1, Issue Date:
 *    2016-09-19, Future Technology Devices International Ltd.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ft80x.h>

#include <arch/irq.h>

#include "ft80x.h"

#ifdef CONFIG_LCD_FT80X

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHIPID         0x7c
#define ROMID_MASK     0x0000ffff
#define VERSION_MASK   0xffff0000
#if defined(CONFIG_LCD_FT800)
#  define DEVNAME      "/dev/ft800"
#  define ROM_CHIPID   0x00010008  /* Byte [0:1] Chip ID "0800" BCD
                                    * Byte [2:3] Version "0100" BCD */
#elif defined(CONFIG_LCD_FT801)
#  define DEVNAME      "/dev/ft801"
#  define ROM_CHIPID   0x00010108  /* Byte [0:1] Chip ID "0801" BCD
                                    * Byte [2:3] Version "0100" BCD */
#else
#  error No FT80x device configured
#endif

#define ROMID          (ROM_CHIPID & ROMID_MASK)
#define VERSION        (ROM_CHIPID & VERSION_MASK)

#define MIN_FADE_DELAY 10          /* Milliseconds */
#define MAX_FADE_DELAY 16700       /* Milliseconds */
#define FADE_STEP_MSEC 10          /* Milliseconds */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  ft80x_fade(FAR struct ft80x_dev_s *priv,
              FAR const struct ft80x_fade_s *fade);

static void ft80x_notify(FAR struct ft80x_dev_s *priv,
              enum ft80x_notify_e id, int value);
static void ft80x_interrupt_work(FAR void *arg);
static int  ft80x_interrupt(int irq, FAR void *context, FAR void *arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void ft80x_destroy(FAR struct ft80x_dev_s *priv);
#endif

/* Character driver methods */

static int  ft80x_open(FAR struct file *filep);
static int  ft80x_close(FAR struct file *filep);

static ssize_t ft80x_read(FAR struct file *filep, FAR char *buffer,
              size_t buflen);
static ssize_t ft80x_write(FAR struct file *filep, FAR const char *buffer,
              size_t buflen);
static int  ft80x_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int  ft80x_unlink(FAR struct inode *inode);
#endif

/* Initialization */

static int  ft80x_initialize(FAR struct ft80x_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ft80x_fops =
{
  ft80x_open,    /* open */
  ft80x_close,   /* close */
  ft80x_read,    /* read */
  ft80x_write,   /* write */
  NULL,          /* seek */
  ft80x_ioctl,   /* ioctl */
  NULL           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , ft80x_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft80x_forcetake
 *
 * Description:
 *   This is a wrapper around but nxsem_wait_uninterruptible().  The wrapper
 *   continues to wait even if the thread is canceled.  This must be done in
 *   certain conditions where were must continue in order to clean-up
 *   resources.
 *
 ****************************************************************************/

static void ft80x_forcetake(FAR sem_t *sem)
{
  int ret;

  do
    {
      ret = nxsem_wait_uninterruptible(sem);

      /* The only expected error would -ECANCELED meaning that the
       * parent thread has been canceled.  We have to continue and
       * terminate the poll in this case.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: ft80x_fade
 *
 * Description:
 *   Change the backlight intensity with a controllable fade.
 *
 ****************************************************************************/

static int ft80x_fade(FAR struct ft80x_dev_s *priv,
                      FAR const struct ft80x_fade_s *fade)
{
  clock_t start;
  clock_t elapsed;
  int32_t delay;
  int32_t duty;
  int16_t endduty;
  int16_t delta;

  /* 0% corresponds to the value 0, but 100% corresponds to the value 128 */

  endduty = (uint16_t)((uint16_t)fade->duty << 7) / 100;

  /* Get the change in duty from the current to the terminal duty. */

  duty  = (int32_t)(ft80x_read_byte(priv, FT80X_REG_PWM_DUTY) & 0x7f);
  delta = endduty - (int16_t)duty;

  /* The "smoothness" of the steps will depend on the resolution of the
   * system timer.  The minimum delay is <= 2 * system_clock_period.
   *
   * We will try for a FADE_STEP_MSEC delay, but we will try to adapt to
   * whatever we get is we are working close the system time resolution.
   * For human factors reasons, any delay less than 100 MS or so should
   * appear more or less smooth.
   *
   * The delay calculation should never overflow:
   *
   *   Max delay:        16,700 msec (MAX_FADE_DELAY)
   *   Min clock period: 1 usec
   *   Max delay:        16,700,000 ticks
   *   INT32_MAX         2,147,483,647
   */

  delay = MSEC2TICK((int32_t)fade->delay);
  if (delay <= 0)
    {
      delay = 1;
    }

  start = clock_systime_ticks();

  do
    {
      /* Wait for FADE_STEP_MSEC msec (or whatever we get) */

      nxsig_usleep(FADE_STEP_MSEC * 1000);

      /* Get the elapsed time */

      elapsed = clock_systime_ticks() - start;
      if (elapsed > INT32_MAX || (int32_t)elapsed >= delay)
        {
          duty = endduty;
        }
      else
        {
          /* Interpolate to get the next PWM duty in the fade.  This
           * calculation should never overflow:
           *
           *   Max delta:       128
           *   Max elapsed:     16,700,000 ticks
           *   Max numerator:   2,137,600,000
           *   Min denominator: 1
           *   Max duty:        2,137,600,000
           *   INT32_MAX        2,147,483,647
           */

          duty += ((int32_t)delta * (int32_t)elapsed) / delay;
          if (duty > 128)
            {
              duty = 128;
            }
          else if (duty < 0)
            {
              duty = 0;
            }
        }

      /* The set the new backlight PWM duty */

      ft80x_write_byte(priv, FT80X_REG_PWM_DUTY, (uint8_t)duty);
    }
  while (duty != endduty);

  return OK;
}

/****************************************************************************
 * Name: ft80x_notify
 *
 * Description:
 *   Notify any registered client of the FT80x event
 *
 ****************************************************************************/

static void ft80x_notify(FAR struct ft80x_dev_s *priv,
                         enum ft80x_notify_e id, int value)
{
  FAR struct ft80x_eventinfo_s *info = &priv->notify[id];

  /* Are notifications enabled for this event? */

  if (info->enable)
    {
      DEBUGASSERT(info->pid > 0);

      /* Yes.. Signal the client */

      info->event.sigev_value.sival_int = value;
      nxsig_notification(info->pid, &info->event, SI_QUEUE, &info->work);
    }
}

/****************************************************************************
 * Name: ft80x_interrupt_work
 *
 * Description:
 *   Back end handling for FT80x interrupts
 *
 ****************************************************************************/

static void ft80x_interrupt_work(FAR void *arg)
{
  FAR struct ft80x_dev_s *priv = (FAR struct ft80x_dev_s *)arg;
  uint32_t intflags;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the device structures */

  ft80x_forcetake(&priv->exclsem);

  /* Get the set of pending interrupts.  Note that simply reading this
   * register is sufficient to clear all pending interrupts.
   */

  intflags = ft80x_read_word(priv, FT80X_REG_INT_FLAGS);

  /* And process each pending interrupt.
   *
   * REVISIT:  No interrupt sources are ever enabled in the current
   * implementation.
   */

  if ((intflags & FT80X_INT_SWAP) != 0)
    {
      /* Display swap occurred */

      lcdinfo("Display swap occurred\n");
      ft80x_notify(priv, FT80X_NOTIFY_SWAP, 0);
    }

  if ((intflags & FT80X_INT_TOUCH) != 0)
    {
      /* Touch-screen touch detected */

      lcdinfo("Touch-screen touch detected\n");
      ft80x_notify(priv, FT80X_NOTIFY_TOUCH, 0);
    }

  if ((intflags & FT80X_INT_TAG) != 0)
    {
      /* Touch-screen tag value change */

      lcdinfo("Touch-screen tag value change\n");
#ifdef CONFIG_LCD_FT800
      regval = ft80x_read_word(priv, FT80X_REG_TOUCH_TAG);
#else
      regval = ft80x_read_word(priv, FT80X_REG_CTOUCH_TAG);
#endif
      ft80x_notify(priv, FT80X_NOTIFY_TAG, (int)(regval & TOUCH_TAG_MASK));
    }

  if ((intflags & FT80X_INT_SOUND) != 0)
    {
      /*  Sound effect ended */

      lcdinfo(" Sound effect ended\n");
      ft80x_notify(priv, FT80X_NOTIFY_SOUND, 0);
    }

  if ((intflags & FT80X_INT_PLAYBACK) != 0)
    {
      /* Audio playback ended */

      lcdinfo("Audio playback ended\n");
      ft80x_notify(priv, FT80X_NOTIFY_PLAYBACK, 0);
    }

  if ((intflags & FT80X_INT_CMDEMPTY) != 0)
    {
      /* Command FIFO empty */

      lcdinfo("Command FIFO empty\n");
      ft80x_notify(priv, FT80X_NOTIFY_CMDEMPTY, 0);
    }

  if ((intflags & FT80X_INT_CMDFLAG) != 0)
    {
      /* Command FIFO flag */

      lcdinfo("Command FIFO flag\n");
      ft80x_notify(priv, FT80X_NOTIFY_CMDFLAG, 0);
    }

  if ((intflags & FT80X_INT_CONVCOMPLETE) != 0)
    {
      /* Touch-screen conversions completed */

      lcdinfo(" Touch-screen conversions completed\n");
      ft80x_notify(priv, FT80X_NOTIFY_CONVCOMPLETE, 0);
    }

  /* Re-enable interrupts */

  DEBUGASSERT(priv->lower != NULL && priv->lower->enable != NULL);
  priv->lower->enable(priv->lower, true);
  nxsem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: ft80x_interrupt
 *
 * Description:
 *   FT80x interrupt handler
 *
 ****************************************************************************/

static int ft80x_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ft80x_dev_s *priv = (FAR struct ft80x_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Perform the interrupt work on the high priority work queue. */

  work_queue(HPWORK, &priv->intwork, ft80x_interrupt_work, priv, 0);

  /* Disable further interrupts for the GPIO interrupt source.
   * REVISIT:  This assumes that GPIO interrupts will pend until re-enabled.
   * In certain implementations, that assumption is not true and could cause
   * a loss of interrupts.
   */

  DEBUGASSERT(priv->lower != NULL && priv->lower->enable != NULL);
  priv->lower->enable(priv->lower, false);
  return OK;
}

/****************************************************************************
 * Name: ft80x_destroy
 *
 * Description:
 *   The driver has been unlinked... clean up as best we can.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void ft80x_destroy(FAR struct ft80x_dev_s *priv)
{
  /* If the lower half driver provided a destroy method, then call that
   * method now in order order to clean up resources used by the lower-half
   * driver.
   */

  DEBUGASSERT(priv != NULL && priv->lower != NULL);
  if (priv->lower->destroy != NULL)
    {
      priv->lower->destroy(priv->lower);
    }

  /* Then free our container */

  nxsem_destroy(&priv->exclsem);
  kmm_free(priv);
}
#endif

/****************************************************************************
 * Name: ft80x_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ****************************************************************************/

static int ft80x_open(FAR struct file *filep)
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  uint8_t tmp;
  int ret;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  lcdinfo("crefs: %d\n", priv->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Increment the count of references to the device */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Save the new open count */

  priv->crefs = tmp;
  ret = OK;

errout_with_sem:
  nxsem_post(&priv->exclsem);

errout:
  return ret;
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: ft80x_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ****************************************************************************/

static int ft80x_close(FAR struct file *filep)
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  int ret;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  lcdinfo("crefs: %d\n", priv->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Will the count decrement to zero? */

  if (priv->crefs <= 1)
    {
      /* Yes.. if the driver has been unlinked, then we need to destroy the
       * driver instance.
       */

      priv->crefs = 0;
      if (priv->unlinked)
        {
          ft80x_destroy(priv);
          return OK;
        }
    }
  else
    {
      /* NO.. decrement the number of references to the driver. */

      priv->crefs--;
    }

  ret = OK;
  nxsem_post(&priv->exclsem);

errout:
  return ret;
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: ft80x_read
 ****************************************************************************/

static ssize_t ft80x_read(FAR struct file *filep, FAR char *buffer,
                          size_t len)
{
  /* Reading from the FT80X is an undefined operation and not support */

  lcdinfo("buffer: %p len %lu\n", buffer, (unsigned long)len);
  return 0;  /* Return EOF */
}

/****************************************************************************
 * Name: ft80x_write
 ****************************************************************************/

static ssize_t ft80x_write(FAR struct file *filep, FAR const char *buffer,
                           size_t len)
{
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  int ret;

  lcdinfo("buffer: %p len %lu\n", buffer, (unsigned long)len);
  DEBUGASSERT(buffer != NULL && ((uintptr_t)buffer & 3) == 0 &&
              len > 0 && (len & 3) == 0 && len <= FT80X_RAM_DL_SIZE);

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  if (buffer == NULL || ((uintptr_t)buffer & 3) != 0 ||
      len == 0 || (len & 3) != 0 || (len + filep->f_pos) > FT80X_RAM_DL_SIZE)
    {
      return -EINVAL;
    }

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Note that there is no check if the driver was opened read-only.  That
   * would be a silly thing to do.
   */

  /* The write method is functionally equivalent to the FT80X_IOC_CREATEDL
   * IOCTL command:  It simply copies the display list in the user buffer to
   * the FT80x display list memory.
   */

  ft80x_write_memory(priv, FT80X_RAM_DL + filep->f_pos, buffer, len);
  filep->f_pos += len;

  nxsem_post(&priv->exclsem);
  return len;
}

/****************************************************************************
 * Name: ft80x_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *
 ****************************************************************************/

static int ft80x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  int ret;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  lcdinfo("cmd: %d arg: %lu\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* FT80X_IOC_CREATEDL:
       *   Description:  Write a display list to the FT80x display list
       *                 memory
       *   Description:  Write a display list to the FT80x display list
       *                 memory starting at offset zero.  This may or may
       *                 not be the entire display list.  Display lists may
       *                 be created incrementally, starting with
       *                 FT80X_IOC_CREATEDL and finishing the display list
       *                 using FT80XIO_APPENDDL
       *   Argument:     A reference to a display list structure instance.
       *                 See struct ft80x_displaylist_s.
       *   Returns:      None
       */

      case FT80X_IOC_CREATEDL:

        /* Set the file position to zero and fall through to "append" the new
         * display list data at offset 0.
         */

        filep->f_pos = 0;

        /* FALLTHROUGH */

      /* FT80X_IOC_APPENDDL:
       *   Description:  Write additional display list entries to the FT80x
       *                 display list memory at the current display list
       *                 offset.  This IOCTL command permits display lists
       *                 to be completed incrementally, starting with
       *                 FT80X_IOC_CREATEDL and finishing the display list
       *                 using FT80XIO_APPENDDL.
       *   Argument:     A reference to a display list structure instance.
       *                 See struct ft80x_displaylist_s.
       *   Returns:      None
       */

      case FT80X_IOC_APPENDDL:
        {
          FAR struct ft80x_displaylist_s *dl =
            (FAR struct ft80x_displaylist_s *)((uintptr_t)arg);

          if (dl == NULL || ((uintptr_t)&dl->cmd & 3) != 0 ||
              (dl->dlsize & 3) != 0 ||
              dl->dlsize + filep->f_pos > FT80X_RAM_DL_SIZE)
            {
              ret = -EINVAL;
            }
          else
            {
              /* Check if there is a display list.  It might be useful for
               * the application to issue FT80X_IOC_CREATEDL with no data in
               * order to initialize the display list, then form all of the
               * list entries with FT80X_IOC_APPENDDL.
               */

              if (dl->dlsize > 0)
                {
                  /* This IOCTL command simply copies the display list
                   * provided into the FT80x display list memory.
                   */

                  ft80x_write_memory(priv, FT80X_RAM_DL + filep->f_pos,
                                     &dl->cmd, dl->dlsize);
                  filep->f_pos += dl->dlsize;
                }

              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_GETRAMDL:
       *   Description:  Read a 32-bit value from the display list.
       *   Argument:     A reference to an instance of struct ft80x_relmem_s.
       *   Returns:      The 32-bit value read from the display list.
       */

      case FT80X_IOC_GETRAMDL:
        {
          FAR struct ft80x_relmem_s *ramdl =
            (FAR struct ft80x_relmem_s *)((uintptr_t)arg);

          if (ramdl == NULL || ((uintptr_t)ramdl->offset & 3) != 0 ||
              ramdl->offset >= FT80X_RAM_DL_SIZE)
            {
              ret = -EINVAL;
            }
          else
            {
              ft80x_read_memory(priv, FT80X_RAM_DL + ramdl->offset,
                                ramdl->value, ramdl->nbytes);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_PUTRAMG
       *   Description:  Write byte data to FT80x graphics memory (RAM_G)
       *   Argument:     A reference to an instance of struct ft80x_relmem_s.
       *   Returns:      None.
       */

      case FT80X_IOC_PUTRAMG:
        {
          FAR struct ft80x_relmem_s *ramg =
            (FAR struct ft80x_relmem_s *)((uintptr_t)arg);

          if (ramg == NULL ||
             (ramg->offset + ramg->nbytes) >= FT80X_RAM_G_SIZE)
            {
              ret = -EINVAL;
            }
          else
            {
              ft80x_write_memory(priv, FT80X_RAM_G + ramg->offset,
                                 ramg->value, ramg->nbytes);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_PUTRAMCMD
       *   Description:  Write 32-bit aligned data to FT80x FIFO (RAM_CMD)
       *   Argument:     A reference to an instance of struct ft80x_relmem_s.
       *   Returns:      None.
       */

      case FT80X_IOC_PUTRAMCMD:
        {
          FAR struct ft80x_relmem_s *ramcmd =
            (FAR struct ft80x_relmem_s *)((uintptr_t)arg);

          if (ramcmd == NULL || ((uintptr_t)ramcmd->offset & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              ft80x_write_memory(priv, FT80X_RAM_CMD + ramcmd->offset,
                                ramcmd->value, ramcmd->nbytes);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_GETREG8:
       *   Description:  Read an 8-bit register value from the FT80x.
       *   Argument:     A reference to an instance of struct
       *                 ft80x_register_s.
       *   Returns:      The 8-bit value read from the register.
       */

      case FT80X_IOC_GETREG8:
        {
          FAR struct ft80x_register_s *reg =
            (FAR struct ft80x_register_s *)((uintptr_t)arg);

          if (reg == NULL || ((uintptr_t)reg->addr & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              reg->value.u8 = ft80x_read_byte(priv, reg->addr);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_GETREG16:
       *   Description:  Read a 16-bit register value from the FT80x.
       *   Argument:     A reference to an instance of struct
       *                 ft80x_register_s.
       *   Returns:      The 16-bit value read from the register.
       */

      case FT80X_IOC_GETREG16:
        {
          FAR struct ft80x_register_s *reg =
            (FAR struct ft80x_register_s *)((uintptr_t)arg);

          if (reg == NULL || ((uintptr_t)reg->addr & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              reg->value.u16 = ft80x_read_hword(priv, reg->addr);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_GETREG32:
       *   Description:  Read a 32-bit register value from the FT80x.
       *   Argument:     A reference to an instance of struct
       *                 ft80x_register_s.
       *   Returns:      The 32-bit value read from the register.
       */

      case FT80X_IOC_GETREG32:
        {
          FAR struct ft80x_register_s *reg =
            (FAR struct ft80x_register_s *)((uintptr_t)arg);

          if (reg == NULL || ((uintptr_t)reg->addr & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              reg->value.u32 = ft80x_read_word(priv, reg->addr);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_GETREGS:
       *   Description:  Read multiple 32-bit register values from the FT80x.
       *   Argument:     A reference to an instance of struct
       *                 ft80x_registers_s.
       *   Returns:      The 32-bit values read from the consecutive
       *                 registers .
       */

      case FT80X_IOC_GETREGS:
        {
          FAR struct ft80x_registers_s *regs =
            (FAR struct ft80x_registers_s *)((uintptr_t)arg);

          if (regs == NULL || ((uintptr_t)regs->addr & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              ft80x_read_memory(priv, regs->addr, regs->value,
                                (size_t)regs->nregs << 2);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_PUTREG8:
       *   Description:  Write an 8-bit register value to the FT80x.
       *   Argument:     A reference to an instance of struct
       *                 ft80x_register_s.
       *   Returns:      None.
       */

      case FT80X_IOC_PUTREG8:
        {
          FAR struct ft80x_register_s *reg =
            (FAR struct ft80x_register_s *)((uintptr_t)arg);

          if (reg == NULL || ((uintptr_t)reg->addr & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              ft80x_write_byte(priv, reg->addr, reg->value.u8);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_PUTREG16:
       *   Description:  Write a 16-bit  register value to the FT80x.
       *   Argument:     A reference to an instance of struct
       *                 ft80x_register_s.
       *   Returns:      None.
       */

      case FT80X_IOC_PUTREG16:
        {
          FAR struct ft80x_register_s *reg =
            (FAR struct ft80x_register_s *)((uintptr_t)arg);

          if (reg == NULL || ((uintptr_t)reg->addr & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              ft80x_write_hword(priv, reg->addr, reg->value.u16);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_PUTREG32:
       *   Description:  Write a 32-bit  register value to the FT80x.
       *   Argument:     A reference to an instance of struct
       *                 ft80x_register_s.
       *   Returns:      None.
       */

      case FT80X_IOC_PUTREG32:
        {
          FAR struct ft80x_register_s *reg =
            (FAR struct ft80x_register_s *)((uintptr_t)arg);

          if (reg == NULL || ((uintptr_t)reg->addr & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              ft80x_write_word(priv, reg->addr, reg->value.u32);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_PUTREGS:
       *   Description:  Write multiple 32-bit register values to the FT80x.
       *   Argument:     A reference to an instance of struct
       *                 ft80x_registers_s.
       *   Returns:      None.
       */

      case FT80X_IOC_PUTREGS:
        {
          FAR struct ft80x_registers_s *regs =
            (FAR struct ft80x_registers_s *)((uintptr_t)arg);

          if (regs == NULL || ((uintptr_t)regs->addr & 3) != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              ft80x_write_memory(priv, regs->addr, regs->value,
                                 (size_t)regs->nregs << 2);
              ret = OK;
            }
        }
        break;

      /* FT80X_IOC_EVENTNOTIFY:
       *   Description:  Setup to receive a signal when an event occurs.
       *   Argument:     A reference to an instance of struct ft80x_notify_s.
       *   Returns:      None
       */

      case FT80X_IOC_EVENTNOTIFY:
        {
          FAR struct ft80x_notify_s *notify =
            (FAR struct ft80x_notify_s *)((uintptr_t)arg);

          if (notify == NULL || notify->pid < 0 ||
              (unsigned int)notify->id >= FT80X_INT_NEVENTS)
            {
              ret = -EINVAL;
            }
          else
            {
              FAR struct ft80x_eventinfo_s *info = &priv->notify[notify->id];
              uint32_t regval;

              /* Are we enabling or disabling */

              if (notify->enable)
                {
                  /* Make sure that arguments are valid for the enable */

                  if (notify->pid == 0)
                    {
                      ret = -EINVAL;
                    }
                  else
                    {
                      /* Setup the new notification information */

                      info->event  = notify->event;
                      info->pid    = notify->pid;
                      info->enable = true;

                      /* Enable interrupts associated with the event */

                      regval  = ft80x_read_word(priv, FT80X_REG_INT_MASK);
                      regval |= (1 << notify->id);
                      ft80x_write_word(priv, FT80X_REG_INT_MASK, regval);
                      ret = OK;
                    }
                }
              else
                {
                  /* Disable the notification */

                  info->pid    = 0;
                  info->enable = false;

                  /* Disable interrupts associated with the event */

                  regval  = ft80x_read_word(priv, FT80X_REG_INT_MASK);
                  regval &= ~(1 << notify->id);
                  ft80x_write_word(priv, FT80X_REG_INT_MASK, regval);

                  /* Cancel any pending notification */

                  nxsig_cancel_notification(&info->work);
                  ret = OK;
                }
            }
        }
        break;

       /* FT80X_IOC_FADE:
        *   Description:  Change the backlight intensity with a controllable
        *                 fade.
        *   Argument:     A reference to an instance of struct ft80x_fade_s.
        *   Returns:      None.
        */

       case FT80X_IOC_FADE:
        {
          FAR const struct ft80x_fade_s *fade =
            (FAR const struct ft80x_fade_s *)((uintptr_t)arg);

          if (fade == NULL || fade->duty > 100 ||
              fade->delay < MIN_FADE_DELAY || fade->delay > MAX_FADE_DELAY)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = ft80x_fade(priv, fade);
            }
        }
        break;

       /* FT80X_IOC_AUDIO:
        *   Description:  Enable/disable an external audio amplifier.
        *   Argument:     0=disable; 1=enable.
        *   Returns:      None.
        */

       case FT80X_IOC_AUDIO:
        {
#if defined(CONFIG_LCD_FT80X_AUDIO_MCUSHUTDOWN)
          /* Amplifier is controlled by an MCU GPIO pin */

          DEBUGASSERT(priv->lower->attach != NULL &&
                      priv->lower->audio != NULL);
          DEBUGASSERT(arg == 0 || arg == 1);

          priv->lower->audio(priv->lower, (arg != 0));
          ret = OK;

#elif defined(CONFIG_LCD_FT80X_AUDIO_GPIOSHUTDOWN)
          /* Amplifier is controlled by an FT80x GPIO pin */

          uint8_t regval8;

          DEBUGASSERT(arg == 0 || arg == 1);

          regval8  = ft80x_read_byte(priv, FT80X_REG_GPIO);

          /* Active low logic assumed */

          if (arg == 0)
            {
              regval8 |= (1 << CONFIG_LCD_FT80X_AUDIO_GPIO);
            }
          else
            {
              regval8 &= ~(1 << CONFIG_LCD_FT80X_AUDIO_GPIO);
            }

          ft80x_write_byte(priv, FT80X_REG_GPIO, regval8);
          ret = OK;

#else
          /* Amplifier is not controllable. */

          DEBUGASSERT(arg == 0 || arg == 1);
          return OK;
#endif
        }
        break;

      /* Unrecognized IOCTL command */

      default:
        lcderr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: ft80x_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int ft80x_unlink(FAR struct inode *inode)
{
  FAR struct ft80x_dev_s *priv;

  /* Get the reference to our internal state structure from the inode
   * structure.
   */

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Indicate that the driver has been unlinked */

  priv->unlinked = true;

  /* If there are no further open references to the driver, then commit
   * Hara-Kiri now.
   */

  if (priv->crefs == 0)
    {
      ft80x_destroy(priv);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ft80x_initialize
 *
 * Description:
 *  Initialize the FT80x
 *
 ****************************************************************************/

static int ft80x_initialize(FAR struct ft80x_dev_s *priv)
{
  uint32_t timeout;
  uint32_t regval32;
  uint8_t regval8;

  /* To configure the display, load the timing control registers with values
   * for the particular display. These registers control horizontal timing:
   *
   *   - FT80X_REG_PCLK
   *   - FT80X_REG_PCLK_POL
   *   - FT80X_REG_HCYCLE
   *   - FT80X_REG_HOFFSET
   *   - FT80X_REG_HSIZE
   *   - FT80X_REG_HSYNC0
   *   - FT80X_REG_HSYNC1
   *
   * These registers control vertical timing:
   *
   *   - FT80X_REG_VCYCLE
   *   - FT80X_REG_VOFFSET
   *   - FT80X_REG_VSIZE
   *   - FT80X_REG_VSYNC0
   *   - FT80X_REG_VSYNC1
   *
   * And the FT80X_REG_CSPREAD register changes color clock timing to reduce
   * system noise.
   *
   * GPIO bit 7 is used for the display enable pin of the LCD module. By
   * setting the direction of the GPIO bit to out direction, the display can
   * be enabled by writing value of 1 into GPIO bit 7 or the display can be
   * disabled by writing a value of 0 into GPIO bit 7. By default GPIO bit 7
   * direction is output and the value is 0.
   */

  /* Initialization Sequence from Power Down using PD_N pin:
   *
   * 1. Drive the PD_N pin high
   * 2. Wait for at least 20ms
   * 3. Execute "Initialization Sequence during the Boot up" from steps 1
   *    to 9
   *
   * Initialization Sequence from Sleep Mode:
   *
   * 1. Send Host command "ACTIVE" to enable clock to FT800
   * 2. Wait for at least 20ms
   * 3. Execute "Initialization Sequence during Boot Up" from steps 5 to 8
   *
   * Initialization sequence from standby mode:
   *
   * Execute all the steps mentioned in "Initialization Sequence from Sleep
   * Mode" except waiting for at least 20ms in step 2.
   */

  DEBUGASSERT(priv->lower != NULL && priv->lower->pwrdown != NULL);
  priv->lower->pwrdown(priv->lower, false);
  up_mdelay(20);

  /* Initialization Sequence during the boot up:
   *
   * 1. Use MCU SPI clock not more than 11MHz
   * 2. Send Host command CLKEXT to FT800 to enable PLL input from oscillator
   *    or external clock.  Should default to 48MHz PLL output.
   * 3. Send Host command ACTIVE to enable clock and wake up the FT80x.
   * 4. Configure video timing registers, except FT80X_REG_PCLK
   * 5. Write first display list
   * 6. Write FT80X_REG_DLSWAP, FT800 swaps display list immediately
   * 7. Enable back light control for display
   * 8. Write FT80X_REG_PCLK, video output begins with the first display list
   * 9. Use MCU SPI clock not more than 30MHz
   */

  /* 1. Select the initial SPI frequency */

  DEBUGASSERT(priv->lower->init_frequency <= 11000000);
  priv->frequency = priv->lower->init_frequency;

  /* 2. Send Host command CLKEXT to FT800 to enable PLL input from oscillator
   *    or external clock.
   */

  ft80x_host_command(priv, FT80X_CMD_CLKEXT);
  up_mdelay(10);

#if 0 /* Un-necessary? */
  /* Switch PLL output to 48MHz (should be the default) */

  ft80x_host_command(priv, FT80X_CMD_CLK48M);
  up_mdelay(10);
#endif

  /* 3. Send Host command ACTIVE to enable clock and wake up the FT80x. */

  ft80x_host_command(priv, FT80X_CMD_ACTIVE);
  up_mdelay(10);

#if 0 /* Un-necessary? */
  /* Do a core reset for safer */

  ft80x_host_command(priv, FT80X_CMD_CORERST);
#endif

  /* Verify the chip ID.  Read repeatedly until FT80x is ready. */

  timeout = 0;
  for (; ; )
    {
      /* Read the Chip ID */

      regval8 = ft80x_read_byte(priv, FT80X_REG_ID);
      if (regval8 == CHIPID)
        {
          /* Chip ID verify so FT80x is ready */

          break;
        }

      /* Initial Chip ID read may fail because the chip is not yet ready. */

      if (++timeout > 100000)
        {
          lcderr("ERROR: Bad chip ID: %02x\n", regval8);
          return -ENODEV;
        }
    }

  regval32 = ft80x_read_word(priv, FT80X_ROM_CHIPID);
  if ((regval32 & ROMID_MASK) != ROMID)
    {
      lcderr("ERROR: Bad ROM chip ID: %08lx\n", (unsigned long)regval32);
      return -ENODEV;
    }

  /* 4. Configure video timing registers, except FT80X_REG_PCLK
   *
   * Once the FT800 is awake and the internal clock set and Device ID
   * checked, the next task is to configure the LCD display parameters for
   * the chosen display with the values determined in Section 2.3.3 above.
   *
   * a. Set FT80X_REG_PCLK to zero - This disables the pixel clock output
   *    while the LCD and other system parameters are configured
   * b. Set the following registers with values for the chosen display.
   *    Typical WQVGA and QVGA values are shown:
   *
   *    Register            Description                    WQVGA     QVGA
   *                                                      480x272  320x240
   *    FT80X_REG_PCLK_POL  Pixel Clock Polarity             1        0
   *    FT80X_REG_HSIZE     Image width in pixels            480      320
   *    FT80X_REG_HCYCLE    Total number of clocks per line  548      408
   *    FT80X_REG_HOFFSET   Horizontal image start           43       70
   *                        (pixels from left)
   *    FT80X_REG_HSYNC0    Start of HSYNC pulse             0        0
   *                        (falling edge)
   *    FT80X_REG_HSYNC1    End of HSYNC pulse               41       10
   *                        (rising edge)
   *    FT80X_REG_VSIZE     Image height in pixels           272      240
   *    FT80X_REG_VCYCLE    Total number of lines per screen 292      263
   *    FT80X_REG_VOFFSET   Vertical image start             12       13
   *                        (lines from top)
   *    FT80X_REG_VSYNC0    Start of VSYNC pulse             0        0
   *                        (falling edge)
   *    FT80X_REG_VSYNC1    End of VSYNC pulse               10       2
   *                        (rising edge)
   *
   * c. Enable or disable FT80X_REG_CSPREAD with a value of 01h or 00h,
   *    respectively.  Enabling FT80X_REG_CSPREAD will offset the R, G and B
   *    output bits so all they do not all change at the same time.
   */

  ft80x_write_byte(priv, FT80X_REG_PCLK, 0);

#if defined(CONFIG_LCD_FT80X_WQVGA)
  ft80x_write_hword(priv, FT80X_REG_HCYCLE, 548);
  ft80x_write_hword(priv, FT80X_REG_HOFFSET, 43);
  ft80x_write_hword(priv, FT80X_REG_HSYNC0, 0);
  ft80x_write_hword(priv, FT80X_REG_HSYNC1, 41);
  ft80x_write_hword(priv, FT80X_REG_VCYCLE, 292);
  ft80x_write_hword(priv, FT80X_REG_VOFFSET, 12);
  ft80x_write_hword(priv, FT80X_REG_VSYNC0, 0);
  ft80x_write_hword(priv, FT80X_REG_VSYNC1, 10);
  ft80x_write_byte(priv, FT80X_REG_SWIZZLE, 0);
  ft80x_write_byte(priv, FT80X_REG_PCLK_POL, 1);
  ft80x_write_byte(priv, FT80X_REG_CSPREAD, 1);
  ft80x_write_hword(priv, FT80X_REG_HSIZE, 480);
  ft80x_write_hword(priv, FT80X_REG_VSIZE, 272);

#elif defined(CONFIG_LCD_FT80X_QVGA)
  ft80x_write_hword(priv, FT80X_REG_HCYCLE, 408);
  ft80x_write_hword(priv, FT80X_REG_HOFFSET, 70);
  ft80x_write_hword(priv, FT80X_REG_HSYNC0, 0);
  ft80x_write_hword(priv, FT80X_REG_HSYNC1, 10);
  ft80x_write_hword(priv, FT80X_REG_VCYCLE, 263);
  ft80x_write_hword(priv, FT80X_REG_VOFFSET, 13);
  ft80x_write_hword(priv, FT80X_REG_VSYNC0, 0);
  ft80x_write_hword(priv, FT80X_REG_VSYNC1, 2);
  ft80x_write_byte(priv, FT80X_REG_SWIZZLE, 2);
  ft80x_write_byte(priv, FT80X_REG_PCLK_POL, 0);
  ft80x_write_byte(priv, FT80X_REG_CSPREAD, 1);
  ft80x_write_hword(priv, FT80X_REG_HSIZE, 320);
  ft80x_write_hword(priv, FT80X_REG_VSIZE, 240);

#else
#  error Unknown display size
#endif

  /* 5. Write first display list */

  ft80x_write_word(priv, FT80X_RAM_DL + 0, FT80X_CLEAR_COLOR_RGB(0, 0, 0));
  ft80x_write_word(priv, FT80X_RAM_DL + 4, FT80X_CLEAR(1, 1, 1));
  ft80x_write_word(priv, FT80X_RAM_DL + 8, FT80X_DISPLAY());

  /* 6. Write FT80X_REG_DLSWAP, FT800 swaps display list immediately */

  ft80x_write_byte(priv, FT80X_REG_DLSWAP, DLSWAP_FRAME);

  /* GPIO bit 7 is used for the display enable pin of the LCD module. By
   * setting the direction of the GPIO bit to out direction, the display can
   * be enabled by writing value of 1 into GPIO bit 7 or the display can be
   * disabled by writing a value of 0 into GPIO bit 7. By default GPIO bit 7
   * direction is output and the value is 0.
   *
   * If an external audio amplified is controlled by an FT80x GPIO, then
   * configure that GPIO as well.  Active low logic is assumed so that the
   * amplifier is initially in the shutdown state.
   */

  regval8  = ft80x_read_byte(priv, FT80X_REG_GPIO_DIR);
  regval8 |= (1 << 7);
#ifdef CONFIG_LCD_FT80X_AUDIO_GPIOSHUTDOWN
  regval8 |= (1 << CONFIG_LCD_FT80X_AUDIO_GPIO);
#endif
  ft80x_write_byte(priv, FT80X_REG_GPIO_DIR, regval8);

  regval8  = ft80x_read_byte(priv, FT80X_REG_GPIO);
  regval8 |= (1 << 7);
#ifdef CONFIG_LCD_FT80X_AUDIO_GPIOSHUTDOWN
  regval8 |= (1 << CONFIG_LCD_FT80X_AUDIO_GPIO);
#endif
  ft80x_write_byte(priv, FT80X_REG_GPIO, regval8);

  /* 7. Enable back light control for display */
#warning Missing logic

  /* 8. Write FT80X_REG_PCLK, video output with the first display list */

#if defined(CONFIG_LCD_FT80X_WQVGA)
  ft80x_write_byte(priv, FT80X_REG_PCLK, 5);
#elif defined(CONFIG_LCD_FT80X_QVGA)
  ft80x_write_byte(priv, FT80X_REG_PCLK, 8);
#else
#  error Unknown display size
#endif

  /* 9. Use MCU SPI clock not more than 30MHz */

  DEBUGASSERT(priv->lower->op_frequency <= 30000000);
  priv->frequency = priv->lower->op_frequency;

  /* Configure touch mode.  Using default touch mode of FRAME_SYNC (~60Hz) */

  ft80x_write_byte(priv, FT80X_REG_TOUCH_MODE, TOUCH_MODE_FRAMESYNC);

#if defined(CONFIG_LCD_FT800)
  /* Configure the touch threshold.  The value 1200 may need to be tweaked
   * for your application.
   */

  ft80x_write_hword(priv, FT80X_REG_TOUCH_RZTHRESH, 1200);

#elif defined(CONFIG_LCD_FT801)
#ifdef CONFIG_LCD_FT801_MULTITOUCH
  /* Selected extended mode */

  ft80x_write_byte(priv, FT80X_REG_CTOUCH_EXTENDED, 0);
#else
  /* Selected compatibility mode */

  ft80x_write_byte(priv, FT80X_REG_CTOUCH_EXTENDED, 1);
#endif

#else
#  error No FT80x device configured
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft80x_register
 *
 * Description:
 *   Configure the ADS7843E to use the provided SPI device instance.  This
 *   will register the driver as /dev/ft800 or /dev/ft801, depending upon
 *   the configuration.
 *
 * Input Parameters:
 *   spi   - An SPI driver instance
 *   i2c   - An I2C master driver instance
 *   lower - Persistent board configuration data / lower half interface
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_LCD_FT80X_SPI)
int ft80x_register(FAR struct spi_dev_s *spi,
                   FAR const struct ft80x_config_s *lower)
#elif defined(CONFIG_LCD_FT80X_I2C)
int ft80x_register(FAR struct i2c_master_s *i2c,
                   FAR const struct ft80x_config_s *lower)
#endif
{
  FAR struct ft80x_dev_s *priv;
  int ret;

#if defined(CONFIG_LCD_FT80X_SPI)
  DEBUGASSERT(spi != NULL && lower != NULL);
#elif defined(CONFIG_LCD_FT80X_I2C)
  DEBUGASSERT(i2c != NULL && lower != NULL);
#endif

  /* Allocate the driver state structure */

  priv = (FAR struct ft80x_dev_s *)kmm_zalloc(sizeof(struct ft80x_dev_s));
  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate state structure\n");
      return -ENOMEM;
    }

  /* Save the lower level interface and configuration information */

  priv->lower = lower;

#ifdef CONFIG_LCD_FT80X_SPI
  /* Remember the SPI configuration */

  priv->spi = spi;
#else
  /* Remember the I2C configuration */

  priv->i2c = i2c;
#endif

  /* Initialize the mutual exclusion semaphore */

  nxsem_init(&priv->exclsem, 0, 1);

  /* Initialize the FT80x */

  ret = ft80x_initialize(priv);
  if (ret < 0)
    {
      goto errout_with_sem;
    }

  /* Attach our interrupt handler */

  DEBUGASSERT(lower->attach != NULL && lower->enable != NULL);
  ret = lower->attach(lower, ft80x_interrupt, priv);
  if (ret < 0)
    {
      goto errout_with_sem;
    }

  /* Disable all interrupt sources, but enable interrupts both in the lower
   * half driver and in the FT80x.
   */

  ft80x_write_word(priv, FT80X_REG_INT_MASK, 0);
  ft80x_write_word(priv, FT80X_REG_INT_EN, FT80X_INT_ENABLE);
  lower->enable(lower, true);

  /* Register the FT80x character driver */

  ret = register_driver(DEVNAME, &g_ft80x_fops, 0666, priv);
  if (ret < 0)
    {
      goto errout_with_interrupts;
    }

  return OK;

errout_with_interrupts:
  lower->enable(lower, false);
  ft80x_write_word(priv, FT80X_REG_INT_EN, FT80X_INT_DISABLE);
  lower->attach(lower, NULL, NULL);

errout_with_sem:
  nxsem_destroy(&priv->exclsem);
  return ret;
}

#endif /* CONFIG_LCD_FT80X */
