/****************************************************************************
 * drivers/input/spq10kbd.c
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

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>

#include <nuttx/input/spq10kbd.h>
#include <nuttx/input/djoystick.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This format is used to construct the /dev/kbd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/kbd%c"
#define DEV_NAMELEN         11

/* Number of Joystick discretes */

#define DJOY_NGPIOS  5

/* Bitset of supported Joystick discretes */

#define DJOY_SUPPORTED (DJOY_UP_BIT | DJOY_DOWN_BIT | DJOY_LEFT_BIT | \
                        DJOY_RIGHT_BIT | DJOY_BUTTON_1_BIT | \
                        DJOY_BUTTON_2_BIT | DJOY_BUTTON_3_BIT | \
                        DJOY_BUTTON_4_BIT)

/* Registers */

#define SPQ10KBD_VER  0x01
#define SPQ10KBD_CFG  0x02
#define SPQ10KBD_INT  0x03
#define SPQ10KBD_KEY  0x04
#define SPQ10KBD_BKL  0x05
#define SPQ10KBD_DEB  0x06
#define SPQ10KBD_FRQ  0x07
#define SPQ10KBD_RST  0x08
#define SPQ10KBD_FIF  0x09

/* VER */

#define SPQ10KBD_VER_MAJOR_SHIFT    4
#define SPQ10KBD_VER_MAJOR_MASK     (0xf << SPQ10KBD_VER_MAJOR_SHIFT)
#define SPQ10KBD_VER_MINOR_SHIFT    0
#define SPQ10KBD_VER_MINOR_MASK     (0xf << SPQ10KBD_VER_MINOR_SHIFT)

#define SPQ10KBD_VER_00_02          0x0002

/* CFG */

#define SPQ10KBD_CFG_OVERFLOW_ON    (1 << 0)
#define SPQ10KBD_CFG_OVERFLOW_INT   (1 << 1)
#define SPQ10KBD_CFG_CAPSLOCK_INT   (1 << 2)
#define SPQ10KBD_CFG_NUMLOCK_INT    (1 << 3)
#define SPQ10KBD_CFG_KEY_INT        (1 << 4)
#define SPQ10KBD_CFG_PANIC_INT      (1 << 5)
#define SPQ10KBD_CFG_REPORT_MODS    (1 << 6)
#define SPQ10KBD_CFG_USE_MODS       (1 << 7)

/* INT */

#define SPQ10KBD_INT_OVERFLOW       (1 << 0)
#define SPQ10KBD_INT_CAPSLOCK       (1 << 1)
#define SPQ10KBD_INT_NUMLOCK        (1 << 2)
#define SPQ10KBD_INT_KEY            (1 << 3)
#define SPQ10KBD_INT_PANIC          (1 << 4)

/* KEY */

#define SPQ10KBD_KEY_COUNT_SHIFT    0
#define SPQ10KBD_KEY_COUNT_MASK     (0xf << SPQ10KBD_KEY_COUNT_SHIFT)
#define SPQ10KBD_KEY_CAPSLOCK       (1 << 5)
#define SPQ10KBD_KEY_NUMLOCK        (1 << 6)

/* FIF */

#define SPQ10KBD_FIF_STATE_SHIFT    0
#define SPQ10KBD_FIF_STATE_MASK     (0xff << SPQ10KBD_FIF_STATE_SHIFT)
#define SPQ10KBD_FIF_KEY_SHIFT      8
#define SPQ10KBD_FIF_KEY_MASK       (0xff << SPQ10KBD_FIF_KEY_SHIFT)

#define KEY_PRESS       0x01
#define KEY_PRESS_HOLD  0x02
#define KEY_RELEASE     0x03

/* Special Key Encodings */

#define KEY_BUTTON_FIRST 0x01  /* Start of the button region */
#define KEY_JOY_UP       0x01
#define KEY_JOY_DOWN     0x02
#define KEY_JOY_LEFT     0x03
#define KEY_JOY_RIGHT    0x04
#define KEY_JOY_CENTER   0x05
#define KEY_BTN_LEFT1    0x06
#define KEY_BTN_RIGHT1   0x07
#define KEY_BACKSPACE    0x08  /* Normal ASCII */
#define KEY_TAB          0x09  /* Normal ASCII */
#define KEY_NL           0x0a  /* Normal ASCII */
#define KEY_BTN_LEFT2    0x11
#define KEY_BTN_RIGHT2   0x12
#define KEY_BUTTON_LAST  0x12  /* End of the button region */

/* Key to joystick mapping */

#ifdef CONFIG_SPQ10KBD_DJOY
static const djoy_buttonset_t joystick_map[] =
{
  0,                  /* No Key */
  DJOY_UP_BIT,        /* KEY_JOY_UP */
  DJOY_DOWN_BIT,      /* KEY_JOY_DOWN */
  DJOY_LEFT_BIT,      /* KEY_JOY_LEFT */
  DJOY_RIGHT_BIT,     /* KEY_JOY_RIGHT */
  0,                  /* KEY_JOY_CENTER */
  DJOY_BUTTON_1_BIT,  /* KEY_BTN_LEFT1 */
  DJOY_BUTTON_3_BIT,  /* KEY_BTN_RIGHT1 */
  0,                  /* KEY_BACKSPACE */
  0,                  /* KEY_TAB */
  0,                  /* KEY_NL */
  0,                  /* No Key */
  0,                  /* No Key */
  0,                  /* No Key */
  0,                  /* No Key */
  0,                  /* No Key */
  0,                  /* No Key */
  DJOY_BUTTON_2_BIT,  /* KEY_BTN_LEFT2 */
  DJOY_BUTTON_4_BIT,  /* KEY_BTN_RIGHT2 */
};
#endif  /* CONFIG_SPQ10KBD_DJOY */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spq10kbd_dev_s
{
  FAR const struct spq10kbd_config_s *config;  /* Board configuration data */
  FAR struct i2c_master_s            *i2c;     /* Saved I2C driver instance */

#ifdef CONFIG_SPQ10KBD_DJOY
  struct djoy_lowerhalf_s   djoylower;        /* Digital joystick */
  djoy_interrupt_t          djoyhandle;       /* Joystick handler func */
  FAR void                 *djoycbarg;        /* Joystick callback arg */
  djoy_buttonset_t          djoypressmask;    /* Joystick press evts */
  djoy_buttonset_t          djoyreleasemask;  /* Joystick release evts */
  djoy_buttonset_t          djoystate;        /* Joystick button state */
#endif  /* CONFIG_SPQ10KBD_DJOY */

  mutex_t lock;         /* Exclusive access to dev */
  sem_t   waitsem;      /* Signal waiting thread */
  bool    waiting;      /* Waiting for keyboard data */
  struct work_s work;   /* Supports the interrupt handling "bottom half" */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_SPQ10KBD_NPOLLWAITERS];

  /* Buffer used to collect and buffer incoming keyboard characters */

  uint16_t  headndx;      /* Buffer head index */
  uint16_t  tailndx;      /* Buffer tail index */
  uint8_t   kbdbuffer[CONFIG_SPQ10KBD_BUFSIZE];

  uint8_t   crefs;        /* Reference count on the driver instance */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static int spq10kbd_interrupt(int irq, FAR void *context, FAR void *arg);
static void spq10kbd_worker(FAR void *arg);
static void spq10kbd_putreg8(FAR struct spq10kbd_dev_s *priv,
                             uint8_t regaddr, uint8_t regval);
static uint8_t spq10kbd_getreg8(FAR struct spq10kbd_dev_s *priv,
                                uint8_t regaddr);
static uint16_t spq10kbd_getreg16(FAR struct spq10kbd_dev_s *priv,
                                  uint8_t regaddr);
static int spq10kbd_checkver(FAR struct spq10kbd_dev_s *priv);
static int spq10kbd_reset(FAR struct spq10kbd_dev_s *priv);
static void spq10kbd_putbuffer(FAR struct spq10kbd_dev_s *priv,
                               uint8_t keycode);

/* Digital Joystick methods */

#ifdef CONFIG_SPQ10KBD_DJOY
static djoy_buttonset_t djoy_supported(
  FAR const struct djoy_lowerhalf_s *lower);
static djoy_buttonset_t djoy_sample(
  FAR const struct djoy_lowerhalf_s *lower);
static void djoy_enable(FAR const struct djoy_lowerhalf_s *lower,
                        djoy_buttonset_t press, djoy_buttonset_t release,
                        djoy_interrupt_t handler, FAR void *arg);
#endif  /* CONFIG_SPQ10KBD_DJOY */

/* Driver methods. We export the keyboard as a standard character driver */

static int  spq10kbd_open(FAR struct file *filep);
static int  spq10kbd_close(FAR struct file *filep);
static ssize_t spq10kbd_read(FAR struct file *filep,
                             FAR char *buffer, size_t len);
static ssize_t spq10kbd_write(FAR struct file *filep,
                              FAR const char *buffer, size_t len);
static int  spq10kbd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hidkbd_fops =
{
  spq10kbd_open,             /* open */
  spq10kbd_close,            /* close */
  spq10kbd_read,             /* read */
  spq10kbd_write,            /* write */
  NULL,                      /* seek */
  NULL,                      /* ioctl */
  spq10kbd_poll              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                     /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SPQ10KBD_DJOY

/****************************************************************************
 * Name: djoy_supported
 *
 * Description:
 *   Return the set of buttons supported on the discrete joystick device
 *
 ****************************************************************************/

static djoy_buttonset_t djoy_supported(
  FAR const struct djoy_lowerhalf_s *lower)
{
  iinfo("Supported: %02x\n", DJOY_SUPPORTED);
  return (djoy_buttonset_t)DJOY_SUPPORTED;
}

/****************************************************************************
 * Name: djoy_sample
 *
 * Description:
 *   Return the current state of all discrete joystick buttons
 *
 ****************************************************************************/

static djoy_buttonset_t djoy_sample(
  FAR const struct djoy_lowerhalf_s *lower)
{
  FAR struct spq10kbd_dev_s *priv =
    (FAR struct spq10kbd_dev_s *)(lower->config);
  return priv->djoystate;
}

/****************************************************************************
 * Name: djoy_enable
 *
 * Description:
 *   Enable interrupts on the selected set of joystick buttons.  An empty
 *   set will disable all interrupts.
 *
 ****************************************************************************/

static void djoy_enable(FAR const struct djoy_lowerhalf_s *lower,
                        djoy_buttonset_t press, djoy_buttonset_t release,
                        djoy_interrupt_t handler, FAR void *arg)
{
  FAR struct spq10kbd_dev_s *priv =
    (FAR struct spq10kbd_dev_s *)(lower->config);
  priv->djoypressmask    = press;
  priv->djoyreleasemask  = release;
  priv->djoyhandle       = handler;
  priv->djoycbarg        = arg;
}
#endif  /* CONFIG_SPQ10KBD_DJOY */

/****************************************************************************
 * Name: spq10kbd_worker
 ****************************************************************************/

static void spq10kbd_worker(FAR void *arg)
{
  FAR struct spq10kbd_dev_s *priv = (FAR struct spq10kbd_dev_s *)arg;
  uint16_t                   regval;
  uint8_t                    key;
  uint8_t                    state;
  int                        ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return;
    }

  regval = spq10kbd_getreg8(priv, SPQ10KBD_INT);
  if (regval & SPQ10KBD_INT_KEY)
    {
      /* There is a keypress in the FIFO */

      while (spq10kbd_getreg8(priv, SPQ10KBD_KEY) & SPQ10KBD_KEY_COUNT_MASK)
        {
          regval = spq10kbd_getreg16(priv, SPQ10KBD_FIF);
          state = (regval & SPQ10KBD_FIF_STATE_MASK) >> \
                  SPQ10KBD_FIF_STATE_SHIFT;
          key = (regval & SPQ10KBD_FIF_KEY_MASK) >> SPQ10KBD_FIF_KEY_SHIFT;
          if (key <= KEY_BUTTON_LAST &&
              !(key == KEY_BACKSPACE ||
                key == KEY_TAB ||
                key == KEY_NL))
            {
#ifdef CONFIG_SPQ10KBD_DJOY
              if (joystick_map[key] == 0)
                {
                  /* Key is not mapped, skip */

                  iinfo("Skipping unmapped key %02x\n", key);
                }

              switch (state)
                {
                  case KEY_PRESS:
                    iinfo("Button Press: %02x\n", key);
                    priv->djoystate |= joystick_map[key];
                    if (priv->djoypressmask & joystick_map[key])
                      {
                        priv->djoyhandle(&priv->djoylower,
                                         priv->djoycbarg);
                      }

                    break;
                  case KEY_RELEASE:
                    iinfo("Button Release: %02x\n", key);
                    priv->djoystate &= ~joystick_map[key];
                    if (priv->djoypressmask & joystick_map[key])
                      {
                        priv->djoyhandle(&priv->djoylower,
                                         priv->djoycbarg);
                      }

                    break;
                }

              iinfo("Stored state: %02x\n", priv->djoystate);
#else
              iinfo("Button Ignored. No joystick interface.\n");
#endif  /* CONFIG_SPQ10KBD_DJOY */
            }
          else if(state == KEY_PRESS)
            {
              /* key is a normal ascii character */

              spq10kbd_putbuffer(priv, key);

              /* Notify waiting reads */

              if (priv->waiting == true)
                {
                  priv->waiting = false;
                  nxsem_post(&priv->waitsem);
                }
            }
        }
    }

  /* Clear interrupt status register */

  spq10kbd_putreg8(priv, SPQ10KBD_INT, 0);
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: spq10kbd_interrupt
 ****************************************************************************/

static int spq10kbd_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct spq10kbd_dev_s *priv = (FAR struct spq10kbd_dev_s *)arg;
  int                        ret;

  /* Let the event worker know that it has an interrupt event to handle
   * It is possbile that we will already have work scheduled from a
   * previous interrupt event.  That is OK we will service all the events
   * in the same work job.
   */

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, spq10kbd_worker, priv, 0);
      if (ret != 0)
        {
          ierr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: spq10kbd_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int spq10kbd_open(FAR struct file *filep)
{
  FAR struct inode          *inode;
  FAR struct spq10kbd_dev_s *priv;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Increment the reference count on the driver */

  priv->crefs++;

  return OK;
}

/****************************************************************************
 * Name: spq10kbd_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int spq10kbd_close(FAR struct file *filep)
{
  FAR struct inode          *inode;
  FAR struct spq10kbd_dev_s *priv;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Decrement the reference count on the driver */

  DEBUGASSERT(priv->crefs >= 1);

  priv->crefs--;

  return OK;
}

/****************************************************************************
 * Name: spq10kbd_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t spq10kbd_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  FAR struct inode          *inode;
  FAR struct spq10kbd_dev_s *priv;
  size_t                     nbytes;
  uint16_t                   tail;
  int                        ret;

  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Read data from our internal buffer of received characters */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  while (priv->tailndx == priv->headndx)
    {
      /* No.. were we open non-blocking? */

      if (filep->f_oflags & O_NONBLOCK)
        {
          /* Yes.. then return a failure */

          ret = -EAGAIN;
          goto errout;
        }
      else
        {
          priv->waiting = true;
          nxmutex_unlock(&priv->lock);
          ret = nxsem_wait_uninterruptible(&priv->waitsem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxmutex_lock(&priv->lock);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  for (tail = priv->tailndx, nbytes = 0;
       tail != priv->headndx && nbytes < len;
       nbytes++)
    {
      /* Copy the next keyboard character into the user buffer */

      *buffer++ = priv->kbdbuffer[tail];

      /* Handle wrap-around of the tail index */

      if (++tail >= CONFIG_SPQ10KBD_BUFSIZE)
        {
          tail = 0;
        }
    }

  ret = nbytes;

  /* Update the tail index (perhaps marking the buffer empty) */

  priv->tailndx = tail;

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: spq10kbd_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t spq10kbd_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  /* We won't try to write to the keyboard */

  return -ENOSYS;
}

/****************************************************************************
 * Name: spq10kbd_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

static int spq10kbd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct inode          *inode;
  FAR struct spq10kbd_dev_s *priv;
  int                        ret;
  int                        i;

  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv);
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_SPQ10KBD_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_SPQ10KBD_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is buffered keyboard data.
       */

      if (priv->headndx != priv->tailndx)
        {
          poll_notify(priv->fds, CONFIG_SPQ10KBD_NPOLLWAITERS, POLLIN);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: spq10kbd_putbuffer
 *
 * Description:
 *   Add one character to the user buffer.
 *   Expectation is that we already have exclusive use of the device.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   keycode - The value to add to the user buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spq10kbd_putbuffer(FAR struct spq10kbd_dev_s *priv,
                               uint8_t keycode)
{
  uint16_t head;
  uint16_t tail;

  DEBUGASSERT(priv);

  /* Copy the next keyboard character into the user buffer. */

  head = priv->headndx;
  priv->kbdbuffer[head] = keycode;

  /* Increment the head index */

  if (++head >= CONFIG_SPQ10KBD_BUFSIZE)
    {
      head = 0;
    }

  /* If the buffer is full, then increment the tail index to make space.
   * Drop old unread key presses.
   */

  tail = priv->tailndx;
  if (tail == head)
    {
      if (++tail >= CONFIG_SPQ10KBD_BUFSIZE)
        {
          tail = 0;
        }

      /* Save the updated tail index */

      priv->tailndx = tail;
    }

  /* Save the updated head index */

  priv->headndx = head;
}

/****************************************************************************
 * Name: spq10kbd_checkver
 *
 * Description:
 *   Read and verify the Q10 Keyboard Controller Version
 *
 ****************************************************************************/

static int spq10kbd_checkver(FAR struct spq10kbd_dev_s *priv)
{
  uint8_t version;

  /* Read device version  */

  version = spq10kbd_getreg8(priv, SPQ10KBD_VER);
  iinfo("version: %02x\n", version);

  if (version != SPQ10KBD_VER_00_02)
    {
      /* Version is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: spq10kbd_reset
 *
 * Description:
 *   Reset the Q10 Keyboard Controller Version
 *
 ****************************************************************************/

static int spq10kbd_reset(FAR struct spq10kbd_dev_s *priv)
{
  spq10kbd_putreg8(priv, SPQ10KBD_RST, 0xff);
  return OK;
}

/****************************************************************************
 * Name: spq10kbd_getreg8
 *
 * Description:
 *   Read from an 8-bit Q10 Keyboard register
 *
 ****************************************************************************/

static uint8_t spq10kbd_getreg8(FAR struct spq10kbd_dev_s *priv,
                                uint8_t regaddr)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - Q10_Reg_Address -
   *    Repeated_Start - I2C_Read_Address  - Q10_Read_Data - STOP
   */

  struct i2c_msg_s msg[2];
  uint8_t          regval;
  int              ret;

  /* Setup 8-bit Q10 Keyboard address write message */

  msg[0].frequency = priv->config->frequency;  /* I2C frequency */
  msg[0].addr      = priv->config->address;    /* 7-bit address */
  msg[0].flags     = 0;                        /* Write transaction, beginning with START */
  msg[0].buffer    = &regaddr;                 /* Transfer from this address */
  msg[0].length    = 1;                        /* Send one byte following the address
                                                * (no STOP) */

  /* Set up the 8-bit Q10 Keyboard data read message */

  msg[1].frequency = priv->config->frequency;  /* I2C frequency */
  msg[1].addr      = priv->config->address;    /* 7-bit address */
  msg[1].flags     = I2C_M_READ;               /* Read transaction, beginning with Re-START */
  msg[1].buffer    = &regval;                  /* Transfer to this address */
  msg[1].length    = 1;                        /* Receive one byte following the address
                                                * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

#ifdef CONFIG_SPQ10KBD_REGDBG
  _err("%02x->%02x\n", regaddr, regval);
#endif
  return regval;
}

/****************************************************************************
 * Name: spq10kbd_getreg16
 *
 * Description:
 *   Read from an 8-bit Q10 Keyboard register
 *
 ****************************************************************************/

static uint16_t spq10kbd_getreg16(FAR struct spq10kbd_dev_s *priv,
                                  uint8_t regaddr)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - Q10_Reg_Address -
   *    Repeated_Start - I2C_Read_Address  - Q10_Read_Data - STOP
   */

  struct i2c_msg_s msg[2];
  uint8_t          regval[2];
  uint16_t              ret;

  /* Setup 8-bit Q10 Keyboard address write message */

  msg[0].frequency = priv->config->frequency;  /* I2C frequency */
  msg[0].addr      = priv->config->address;    /* 7-bit address */
  msg[0].flags     = 0;                        /* Write transaction, beginning with START */
  msg[0].buffer    = &regaddr;                 /* Transfer from this address */
  msg[0].length    = 1;                        /* Send one byte following the address
                                                * (no STOP) */

  /* Set up the 8-bit Q10 Keyboard data read message */

  msg[1].frequency = priv->config->frequency;  /* I2C frequency */
  msg[1].addr      = priv->config->address;    /* 7-bit address */
  msg[1].flags     = I2C_M_READ;               /* Read transaction, beginning with Re-START */
  msg[1].buffer    = regval;                   /* Transfer to this address */
  msg[1].length    = 2;                        /* Receive two bytes following the address
                                                * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  ret = (regval[1] << 8) | regval[0];
#ifdef CONFIG_SPQ10KBD_REGDBG
  _err("%02x->%04x\n", regaddr, ret);
#endif
  return ret;
}

/****************************************************************************
 * Name: spq10kbd_putreg8
 *
 * Description:
 *   Write a value to an 8-bit Q10 Keyboard register
 *
 ****************************************************************************/

static void spq10kbd_putreg8(FAR struct spq10kbd_dev_s *priv,
                             uint8_t regaddr, uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - Q10_Reg_Address - Q10_Write_Data - STOP
   */

  struct i2c_msg_s msg;
  uint8_t          txbuffer[2];
  int              ret;

#ifdef CONFIG_SPQ10KBD_REGDBG
  _err("%02x<-%02x\n", regaddr, regval);
#endif

  /* Setup to the data to be transferred.  Two bytes:  The Q10 Keyboard
   * register address followed by one byte of data.
   */

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  /* Setup 8-bit Q10 Keyboard address write message */

  msg.frequency = priv->config->frequency;  /* I2C frequency */
  msg.addr      = priv->config->address;    /* 7-bit address */
  msg.flags     = 0;                        /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                 /* Transfer from this address */
  msg.length    = 2;                        /* Send two byte following the address
                                             * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spq10kbd_register
 *
 * Description:
 *   Configure the Solder Party Q10 Keyboard to use the provided I2C device
 *   instance.  This will register the driver as /dev/kbdN where N is the
 *   minor device number, as well as a joystick at joydevname
 *
 * Input Parameters:
 *   i2c         - An I2C driver instance
 *   config      - Persistent board configuration data
 *   kbdminor    - The keyboard input device minor number
 *   joydevname  - The name of the joystick device /dev/djoyN
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPQ10KBD_DJOY
int spq10kbd_register(FAR struct i2c_master_s *i2c,
                      FAR const struct spq10kbd_config_s *config,
                      char kbdminor, char *joydevname)
#else
int spq10kbd_register(FAR struct i2c_master_s *i2c,
                      FAR const struct spq10kbd_config_s *config,
                      char kbdminor)
#endif
{
  FAR struct spq10kbd_dev_s *priv;
  char                       kbddevname[DEV_NAMELEN];
  int                        ret;

  /* Debug Sanity Checks */

  DEBUGASSERT(i2c != NULL && config != NULL);
  DEBUGASSERT(config->attach != NULL && config->enable != NULL &&
              config->clear  != NULL);

  priv = (FAR struct spq10kbd_dev_s *)kmm_zalloc(
    sizeof(struct spq10kbd_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_zalloc(%d) failed\n", sizeof(struct spq10kbd_dev_s));
      return -ENOMEM;
    }

  /* Initialize the device driver instance */

  priv->i2c       = i2c;     /* Save the I2C device handle */
  priv->config    = config;  /* Save the board configuration */
  priv->tailndx   = 0;       /* Reset keypress buffer state */
  priv->headndx   = 0;
  priv->crefs     = 0;       /* Reset reference count to 0 */
  priv->waiting   = false;

#ifdef CONFIG_SPQ10KBD_DJOY
  priv->djoylower.config       = priv;
  priv->djoylower.dl_supported = djoy_supported;
  priv->djoylower.dl_sample    = djoy_sample;
  priv->djoylower.dl_enable    = djoy_enable;
  priv->djoyhandle             = NULL;
  priv->djoystate              = 0;
#endif  /* CONFIG_SPQ10KBD_DJOY */

  nxmutex_init(&priv->lock);   /* Initialize device mutex */
  nxsem_init(&priv->waitsem, 0, 0);

  config->clear(config);
  config->enable(config, false);

  /* Attach the interrupt handler */

  ret = config->attach(config, spq10kbd_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  ret = spq10kbd_checkver(priv);

  if (ret != OK)
    {
      /* Did not find a supported device on the bus */

      return ret;
    }

  spq10kbd_reset(priv);

  /* Start servicing events */

  priv->config->enable(priv->config, true);

  snprintf(kbddevname, sizeof(kbddevname), DEV_FORMAT, kbdminor);
  iinfo("Registering %s\n", kbddevname);
  ret = register_driver(kbddevname, &g_hidkbd_fops, 0666, priv);

#ifdef CONFIG_SPQ10KBD_DJOY
  iinfo("Registering %s\n", joydevname);
  ret = djoy_register(joydevname, &priv->djoylower);
#endif  /* CONFIG_SPQ10KBD_DJOY */

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->lock);
  nxsem_destroy(&priv->waitsem);
  kmm_free(priv);
  return ret;
}
