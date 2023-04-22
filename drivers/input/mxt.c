/****************************************************************************
 * drivers/input/mxt.c
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

/* Suppress verbose debug output so that we don't swamp the system */

#ifdef CONFIG_MXT_DISABLE_CONFIG_DEBUG_INFO
#  undef CONFIG_DEBUG_INFO
#endif

#include <sys/types.h>

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>

#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/mxt.h>

#include "mxt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* This is a value for the threshold that guarantees a big difference on the
 * first pendown (but can't overflow).
 */

#define INVALID_POSITION 0x1000

/* Maximum number of retries */

#define MAX_RETRIES  3

/* Get a 16-bit value in little endian order (not necessarily aligned).  The
 * source data is in little endian order.  The host byte order does not
 * matter in this case.
 */

#define MXT_GETUINT16(p) \
  (((uint16_t)(((FAR uint8_t *)(p))[1]) << 8) | \
    (uint16_t)(((FAR uint8_t *)(p))[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration describes the state of one contact.
 *
 *                  |
 *                  v
 *             CONTACT_NONE            (1) Touch
 *           / (1)        ^ (3)        (2) Release
 *          v              \           (3) Event reported
 *     CONTACT_NEW     CONTACT_LOST
 *          \ (3)          ^ (2)
 *           v            /
 *           CONTACT_REPORT
 *             \ (1)    ^ (3)
 *              v      /
 *            CONTACT_MOVE
 *
 * NOTE: This state transition diagram is simplified.  There are a few other
 * sneaky transitions to handle unexpected conditions.
 */

enum mxt_contact_e
{
  CONTACT_NONE = 0,                /* No contact */
  CONTACT_NEW,                     /* New contact */
  CONTACT_MOVE,                    /* Same contact, possibly different position */
  CONTACT_REPORT,                  /* Contact reported */
  CONTACT_LOST,                    /* Contact lost */
};

/* This structure describes the results of one MXT sample */

struct mxt_sample_s
{
  uint8_t  id;                     /* Sampled touch point ID */
  uint8_t  contact;                /* Contact state (see enum mxt_contact_e) */
  bool     valid;                  /* True: x,y,pressure contain valid, sampled data */
  uint16_t x;                      /* Measured X position */
  uint16_t y;                      /* Measured Y position */
  uint16_t lastx;                  /* Last reported X position */
  uint16_t lasty;                  /* Last reported Y position */
  uint8_t  area;                   /* Contact area */
  uint8_t  pressure;               /* Contact pressure */
};

/* This 7-bit 'info' data read from the MXT and that describes the
 * characteristics of the particular maXTouch chip
 */

struct mxt_info_s
{
  uint8_t family;                  /* MXT family ID */
  uint8_t variant;                 /* MXT variant ID */
  uint8_t version;                 /* MXT version number */
  uint8_t build;                   /* MXT build number */
  uint8_t xsize;                   /* Matrix X size */
  uint8_t ysize;                   /* Matrix Y size */
  uint8_t nobjects;                /* Number of objects */
};
#define MXT_INFO_SIZE 7

/* Describes the state of the MXT driver */

struct mxt_dev_s
{
  /* These are the retained references to the I2C device and to the
   * lower half configuration data.
   */

  FAR struct i2c_master_s *i2c;
  FAR const struct mxt_lower_s *lower;

  /* This is the allocated array of object information */

  FAR struct mxt_object_s *objtab;

  /* This is an allocated array of sample data, one for each possible touch */

  FAR struct mxt_sample_s *sample; /* Last sampled touch point data */

  uint8_t nwaiters;                /* Number of threads waiting for MXT data */
  uint8_t id;                      /* Current touch point ID */
  uint8_t nslots;                  /* Number of slots */
  uint8_t crefs;                   /* Reference count */

  /* Cached parameters from object table */

#ifdef MXT_SUPPORT_T6
  uint8_t t6id;                    /* T6 report ID */
#endif
  uint8_t t9idmin;                 /* T9 touch event report IDs */
  uint8_t t9idmax;
#ifdef CONFIG_MXT_BUTTONS
  uint8_t t19id;                   /* T19 button report ID */
#endif

  volatile bool event;             /* True: An unreported event is buffered */
  mutex_t devlock;                 /* Manages exclusive access to this structure */
  sem_t waitsem;                   /* Used to wait for the availability of data */
  uint32_t frequency;              /* Current I2C frequency */

  char phys[64];                   /* Device physical location */
  struct mxt_info_s info;          /* Configuration info read from device */
  struct work_s work;              /* Supports the interrupt handling "bottom half" */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_MXT_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MXT register access */

static int  mxt_getreg(FAR struct mxt_dev_s *priv, uint16_t regaddr,
              FAR uint8_t *buffer, size_t buflen);
static int  mxt_putreg(FAR struct mxt_dev_s *priv, uint16_t regaddr,
              FAR const uint8_t *buffer, size_t buflen);

/* MXT object/message access */

static FAR struct mxt_object_s *mxt_object(FAR struct mxt_dev_s *priv,
              uint8_t type);
static int mxt_getmessage(FAR struct mxt_dev_s *priv,
              FAR struct mxt_msg_s *msg);
static int mxt_putobject(FAR struct mxt_dev_s *priv, uint8_t type,
              uint8_t offset, uint8_t value);
#if 0 /* Not used */
static int mxt_getobject(FAR struct mxt_dev_s *priv, uint8_t type,
              uint8_t offset, FAR uint8_t *value);
#endif
static int  mxt_flushmsgs(FAR struct mxt_dev_s *priv);

/* Poll support */

static void mxt_notify(FAR struct mxt_dev_s *priv);

/* Touch event waiting */

static inline int  mxt_checksample(FAR struct mxt_dev_s *priv);
static inline int  mxt_waitsample(FAR struct mxt_dev_s *priv);

/* Interrupt handling/position sampling */

#ifdef CONFIG_MXT_BUTTONS
static void mxt_button_event(FAR struct mxt_dev_s *priv,
              FAR struct mxt_msg_s *msg);
#endif
static void mxt_touch_event(FAR struct mxt_dev_s *priv,
              FAR struct mxt_msg_s *msg, int ndx);
static void mxt_worker(FAR void *arg);
static int  mxt_interrupt(FAR const struct mxt_lower_s *lower,
              FAR void *context);

/* Character driver methods */

static int  mxt_open(FAR struct file *filep);
static int  mxt_close(FAR struct file *filep);
static ssize_t mxt_read(FAR struct file *filep, FAR char *buffer,
              size_t len);
static int  mxt_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  mxt_poll(FAR struct file *filep, struct pollfd *fds, bool setup);

/* Initialization */

static int  mxt_getinfo(struct mxt_dev_s *priv);
static int  mxt_getobjtab(FAR struct mxt_dev_s *priv);
static int  mxt_hwinitialize(FAR struct mxt_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_mxt_fops =
{
  mxt_open,    /* open */
  mxt_close,   /* close */
  mxt_read,    /* read */
  NULL,        /* write */
  NULL,        /* seek */
  mxt_ioctl,   /* ioctl */
  NULL,        /* mmap */
  NULL,        /* truncate */
  mxt_poll     /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mxt_getreg
 ****************************************************************************/

static int mxt_getreg(FAR struct mxt_dev_s *priv, uint16_t regaddr,
                      FAR uint8_t *buffer, size_t buflen)
{
  struct i2c_msg_s msg[2];
  uint8_t addrbuf[2];
  int retries;
  int ret;

  /* Try up to three times to read the register */

  for (retries = 1; retries <= MAX_RETRIES; retries++)
    {
      iinfo("retries=%d regaddr=%04x buflen=%d\n", retries, regaddr, buflen);

      /* Set up to write the address */

      addrbuf[0]       = regaddr & 0xff;
      addrbuf[1]       = (regaddr >> 8) & 0xff;

      msg[0].frequency = priv->frequency;
      msg[0].addr      = priv->lower->address;
      msg[0].flags     = 0;
      msg[0].buffer    = addrbuf;
      msg[0].length    = 2;

      /* Followed by the read data */

      msg[1].frequency = priv->frequency;
      msg[1].addr      = priv->lower->address;
      msg[1].flags     = I2C_M_READ;
      msg[1].buffer    = buffer;
      msg[1].length    = buflen;

      /* Read the register data.  The returned value is the number messages
       * completed.
       */

      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up?  Try to shake the bus free.
           * Don't bother with the reset if this was the last attempt.
           */

          if (retries < MAX_RETRIES)
            {
              iwarn("WARNING: I2C_TRANSFER failed: %d ... Resetting\n", ret);

              ret = I2C_RESET(priv->i2c);
              if (ret < 0)
                {
                  ierr("ERROR: I2C_RESET failed: %d\n", ret);
                  break;
                }
            }
#else
          ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
#endif
        }
      else
        {
          /* The I2C transfer was successful... break out of the loop and
           * return the success indication.
           */

          break;
        }
    }

  /* Return the last status returned by I2C_TRANSFER */

  return ret;
}

/****************************************************************************
 * Name: mxt_putreg
 ****************************************************************************/

static int mxt_putreg(FAR struct mxt_dev_s *priv, uint16_t regaddr,
                      FAR const uint8_t *buffer, size_t buflen)
{
  struct i2c_msg_s msg[2];
  uint8_t addrbuf[2];
  int retries;
  int ret;

  /* Try up to three times to read the register */

  for (retries = 1; retries <= MAX_RETRIES; retries++)
    {
      iinfo("retries=%d regaddr=%04x buflen=%d\n", retries, regaddr, buflen);

      /* Set up to write the address */

      addrbuf[0]       = regaddr & 0xff;
      addrbuf[1]       = (regaddr >> 8) & 0xff;

      msg[0].frequency = priv->frequency;
      msg[0].addr      = priv->lower->address;
      msg[0].flags     = 0;
      msg[0].buffer    = addrbuf;
      msg[0].length    = 2;

      /* Followed by the write data (with no repeated start) */

      msg[1].frequency = priv->frequency;
      msg[1].addr      = priv->lower->address;
      msg[1].flags     = I2C_M_NOSTART;
      msg[1].buffer    = (FAR uint8_t *)buffer;
      msg[1].length    = buflen;

      /* Write the register data.  The returned value is the number messages
       * completed.
       */

      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up?  Try to shake the bus free.
           * Don't bother with the reset if this was the last attempt.
           */

          if (retries < MAX_RETRIES)
            {
              iwarn("WARNING: I2C_TRANSFER failed: %d ... Resetting\n", ret);

              ret = I2C_RESET(priv->i2c);
              if (ret < 0)
                {
                  ierr("ERROR: I2C_RESET failed: %d\n", ret);
                }
            }
#else
          ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
#endif
        }
      else
        {
          /* The I2C transfer was successful... break out of the loop and
           * return the success indication.
           */

          break;
        }
    }

  /* Return the last status returned by I2C_TRANSFER */

  return ret;
}

/****************************************************************************
 * Name: mxt_object
 ****************************************************************************/

static FAR struct mxt_object_s *mxt_object(FAR struct mxt_dev_s *priv,
                                           uint8_t type)
{
  struct mxt_object_s *object;
  int i;

  /* Search the object table for the entry matching the type */

  for (i = 0; i < priv->info.nobjects; i++)
    {
      object = &priv->objtab[i];
      if (object->type == type)
        {
          /* Found it.. return the pointer to the object structure */

          return object;
        }
    }

  ierr("ERROR: Invalid object type: %d\n", type);
  return NULL;
}

/****************************************************************************
 * Name: mxt_getmessage
 ****************************************************************************/

static int mxt_getmessage(FAR struct mxt_dev_s *priv,
                          FAR struct mxt_msg_s *msg)
{
  struct mxt_object_s *object;
  uint16_t regaddr;

  object = mxt_object(priv, MXT_GEN_MESSAGE_T5);
  if (object == NULL)
    {
      ierr("ERROR: mxt_object failed\n");
      return -EINVAL;
    }

  regaddr = MXT_GETUINT16(object->addr);
  return mxt_getreg(priv, regaddr, (FAR uint8_t *)msg,
                    sizeof(struct mxt_msg_s));
}

/****************************************************************************
 * Name: mxt_putobject
 ****************************************************************************/

static int mxt_putobject(FAR struct mxt_dev_s *priv, uint8_t type,
                         uint8_t offset, uint8_t value)
{
  FAR struct mxt_object_s *object;
  uint16_t regaddr;

  object = mxt_object(priv, type);
  if (object == NULL || offset >= object->size + 1)
    {
      return -EINVAL;
    }

  regaddr = MXT_GETUINT16(object->addr);
  return mxt_putreg(priv, regaddr + offset, &value, 1);
}

/****************************************************************************
 * Name: mxt_getobject
 ****************************************************************************/

#if 0 /* Not used */
static int mxt_getobject(FAR struct mxt_dev_s *priv, uint8_t type,
                         uint8_t offset, FAR uint8_t *value)
{
  FAR struct mxt_object_s *object;
  uint16_t regaddr;

  object = mxt_object(priv, type);
  if (object == NULL || offset >= object->size + 1)
    {
      return -EINVAL;
    }

  regaddr = MXT_GETUINT16(object->addr);
  return mxt_getreg(priv, regaddr + offset, value, 1);
}
#endif

/****************************************************************************
 * Name: mxt_flushmsgs
 *
 *   Clear any pending messages be reading messages until there are no
 *   pending messages.  This will force the CHG pin to the high state and
 *   prevent spurious initial interrupts.
 *
 ****************************************************************************/

static int mxt_flushmsgs(FAR struct mxt_dev_s *priv)
{
  struct mxt_msg_s msg;
  int retries = 16;
  int ret;

  /* Read dummy message until there are no more to read (or until we have
   * tried 10 times).
   */

  do
    {
      ret = mxt_getmessage(priv, &msg);
      if (ret < 0)
        {
          ierr("ERROR: mxt_getmessage failed: %d\n", ret);
          return ret;
        }
    }
  while (msg.id != 0xff && --retries > 0);

  /* Complain if we exceed the retry limit */

  if (retries <= 0)
    {
      ierr("ERROR: Failed to clear messages: ID=%02x\n", msg.id);
      return -EBUSY;
    }

  return OK;
}

/****************************************************************************
 * Name: mxt_notify
 ****************************************************************************/

static void mxt_notify(FAR struct mxt_dev_s *priv)
{
  /* If there are threads waiting on poll() for maXTouch data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  poll_notify(priv->fds, CONFIG_MXT_NPOLLWAITERS, POLLIN);

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the maXTouch
       * is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: mxt_checksample
 *
 * Description:
 *   This function implements a test and clear of the priv->event flag.
 *   Called only from mxt_waitsample.
 *
 * Assumptions:
 *   - Scheduler must be locked to prevent the worker thread from running
 *     while this thread runs.  The sample data is, of course, updated from
 *     the worker thread.
 *  -  Interrupts must be disabled when this is called to (1) prevent posting
 *     of semaphores from interrupt handlers, and (2) to prevent sampled data
 *     from changing until it has been reported.
 *
 ****************************************************************************/

static inline int mxt_checksample(FAR struct mxt_dev_s *priv)
{
  /* Is there new maXTouch sample data available? */

  if (priv->event)
    {
      /* Yes.. clear the flag and return success */

      priv->event = false;
      return OK;
    }

  /* No.. return failure */

  return -EAGAIN;
}

/****************************************************************************
 * Name: mxt_waitsample
 *
 * Wait until sample data is available.  Called only from mxt_read.
 *
 * Assumptions:
 *   - Scheduler must be locked to prevent the worker thread from running
 *     while this thread runs.  The sample data is, of course, updated from
 *     the worker thread.
 *
 ****************************************************************************/

static inline int mxt_waitsample(FAR struct mxt_dev_s *priv)
{
  irqstate_t flags;
  int ret;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = enter_critical_section();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  nxmutex_unlock(&priv->devlock);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (mxt_checksample(priv) < 0)
    {
      /* Wait for a change in the maXTouch state */

      priv->nwaiters++;
      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          goto errout;
        }
    }

  /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure.  We may have to wait here.  But we have our
   * sample.  Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = nxmutex_lock(&priv->devlock);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: mxt_button_event
 ****************************************************************************/

#ifdef CONFIG_MXT_BUTTONS
static void mxt_button_event(FAR struct mxt_dev_s *priv,
                             FAR struct mxt_msg_s *msg)
{
  bool button;
  int i;

  /* REVISIT: Button inputs are currently ignored */

  /* Buttons are active low and determined by the GPIO bit
   * settings in byte 0 of the message data: A button is
   * pressed if the corresponding bit is zero.
   */

  for (i = 0; i < priv->lower->nbuttons; i++)
    {
      uint8_t bit = (MXT_GPIO0_MASK << i);

      /* Does this implementation support the button? */

      if ((priv->lower->bmask & bit) != 0)
        {
          /* Yes.. get the button state */

          button = (msg->body[0] & mask) == 0;

          /* Now what? */

          UNUSED(button);
        }
    }
}
#endif

/****************************************************************************
 * Name: mxt_touch_event
 ****************************************************************************/

static void mxt_touch_event(FAR struct mxt_dev_s *priv,
                            FAR struct mxt_msg_s *msg, int ndx)
{
  FAR struct mxt_sample_s *sample;
  uint16_t x;
  uint16_t y;
  uint8_t  area;
  uint8_t  pressure;
  uint8_t  status;

  /* Extract the 12-bit X and Y positions */

  x = ((uint16_t)msg->body[1] << 4) |
      (((uint16_t)msg->body[3] >> 4) & 0x0f);
  y = ((uint16_t)msg->body[2] << 4) |
      (((uint16_t)msg->body[3] & 0x0f));

  /* Swap X/Y as necessary */

  if (priv->lower->swapxy)
    {
      uint16_t tmp = x;
      y = x;
      x = tmp;
    }

  /* Extract area pressure and status */

  area     = msg->body[4];
  pressure = msg->body[5];

  status = msg->body[0];
  iinfo("ndx=%u status=%02x pos(%u,%u) area=%u pressure=%u\n",
        ndx, status, x, y, area, pressure);

  /* The normal sequence that we would see for a touch would be something
   * like:
   *
   *   1. MXT_DETECT + MXT_PRESS
   *   2. MXT_DETECT + MXT_AMP
   *   3. MXT_DETECT + MXT_MOVE + MXT_AMP
   *   4. MXT_RELEASE
   *
   * So we really only need to check MXT_DETECT to drive this state machine.
   */

  /* Is this a loss of contact? */

  sample = &priv->sample[ndx];
  if ((status & MXT_DETECT) == 0)
    {
      /* Ignore the event if there was no contact to be lost:
       *
       *   CONTACT_NONE = No touch and loss-of-contact already reported
       *   CONTACT_LOST = No touch and unreported loss-of-contact.
       */

      if (sample->contact == CONTACT_NONE)
        {
          /* Return without posting any event */

          return;
        }

      /* State is one of CONTACT_NEW, CONTACT_MOVE, CONTACT_REPORT or
       * CONTACT_LOST.
       *
       * NOTE: Here we do not check for these other states because there is
       * not much that can be done anyway.  The transition to CONTACK_LOST
       * really only makes sense if the preceding state was CONTACT_REPORT.
       * If we were in (unreported) CONTACT_NEW or CONTACT_MOVE states, then
       * this will overwrite that event and it will not be reported.  This
       * opens the possibility for contact lost reports when no contact was
       * ever reported.
       *
       * We could improve this be leaving the unreported states in place,
       * remembering that the contact was lost, and then reporting the loss-
       * of-contact after touch state is reported.
       */

      sample->contact = CONTACT_LOST;

      /* Reset the last position so that we guarantee that the next position
       * will pass the thresholding test.
       */

      sample->lastx = INVALID_POSITION;
      sample->lasty = INVALID_POSITION;
    }
  else
    {
      /* It is a touch event.  If the last loss-of-contact event has not
       * been processed yet, then have to bump up the touch identifier and
       * hope that the client is smart enough to infer the loss-of-contact
       * event for the preceding touch.
       */

      if (sample->contact == CONTACT_LOST)
        {
           priv->id++;
        }

      /* Save the measurements */

      sample->x        = x;
      sample->y        = y;
      sample->area     = area;
      sample->pressure = pressure;
      sample->valid    = true;

      add_ui_randomness((x << 16) ^ y ^ (area << 9) ^ (pressure << 1));

      /* If this is not the first touch report, then report it as a move:
       * Same contact, same ID, but with a new, updated position.
       * The CONTACT_REPORT state means that a contacted has been detected,
       * but all contact events have been successfully reported.
       */

      if (sample->contact == CONTACT_REPORT)
        {
          uint16_t xdiff;
          uint16_t ydiff;

          /* Not a new contact.  Check if the new measurements represent a
           * non-trivial change in position.  A trivial change is detected
           * by comparing the change in position since the last report
           * against configurable threshold values.
           *
           * REVISIT:  Should a large change in pressure also generate a
           * event?
           */

          xdiff = x > sample->lastx ? (x - sample->lastx) :
                                      (sample->lastx - x);
          ydiff = y > sample->lasty ? (y - sample->lasty) :
                                      (sample->lasty - y);

          /* Check the thresholds */

          if (xdiff >= CONFIG_MXT_THRESHX || ydiff >= CONFIG_MXT_THRESHY)
            {
              /* Report a contact move event.  This state will be set back
               * to CONTACT_REPORT after it been reported.
               */

              sample->contact = CONTACT_MOVE;

              /* Update the last position for next threshold calculations */

              sample->lastx   = x;
              sample->lasty   = y;
            }
          else
            {
              /* Bail without reporting anything for this event */

              return;
            }
        }

      /* If we have seen this contact before but it has not yet been
       * reported, then do nothing other than overwrite the positional
       * data.
       *
       * This the state must be one of CONTACT_NONE or CONTACT_LOST (see
       * above) and we have a new contact with a new ID.
       */

      else if (sample->contact != CONTACT_NEW &&
               sample->contact != CONTACT_MOVE)
        {
          /* First contact.  Save the contact event and assign a new
           * ID to the contact.
           */

          sample->contact = CONTACT_NEW;
          sample->id      = priv->id++;

          /* Update the last position for next threshold calculations */

          sample->lastx   = x;
          sample->lasty   = y;

          /* This state will be set to CONTACT_REPORT after it
           * been reported.
           */
        }
    }

  /* Indicate the availability of new sample data for this ID and notify
   * any waiters that new maXTouch data is available
   */

  priv->event = true;
  mxt_notify(priv);
}

/****************************************************************************
 * Name: mxt_worker
 ****************************************************************************/

static void mxt_worker(FAR void *arg)
{
  FAR struct mxt_dev_s *priv = (FAR struct mxt_dev_s *)arg;
  FAR const struct mxt_lower_s *lower;
  struct mxt_msg_s msg;
  uint8_t id;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  lower = priv->lower;
  DEBUGASSERT(lower != NULL);

  /* Get exclusive access to the MXT driver data structure */

  nxmutex_lock(&priv->devlock);

  /* Loop, processing each message from the maXTouch */

  retries = 0;
  do
    {
      /* Retrieve the next message from the maXTouch */

      ret = mxt_getmessage(priv, &msg);
      if (ret < 0)
        {
          ierr("ERROR: mxt_getmessage failed: %d\n", ret);
          goto errout_with_lock;
        }

      id = msg.id;

#ifdef MXT_SUPPORT_T6
      /* Check for T6 */

      if (id == priv->t6id)
        {
          uint32_t chksum;
          int status;

          status = msg.body[0];
          chksum = (uint32_t)msg.body[1] |
                  ((uint32_t)msg.body[2] << 8) |
                  ((uint32_t)msg.body[3] << 16);

          iinfo("T6: status: %02x checksum: %06lx\n",
                status, (unsigned long)chksum);

          retries = 0;
        }
      else
#endif

      /* Check for T9 */

      if (id >= priv->t9idmin && id <= priv->t9idmax)
        {
          mxt_touch_event(priv, &msg, id - priv->t9idmin);
          retries = 0;
        }

#ifdef CONFIG_MXT_BUTTONS
      /* Check for T19 */

      else if (msg.id == priv->t19id)
        {
          mxt_button_event(priv, &msg);
          retries = 0;
        }
#endif

      /* 0xff marks the end of the messages; any other message IDs are
       * ignored (after complaining a little).
       */

      else if (msg.id != 0xff)
        {
          iinfo("Ignored: id=%u message="
                "{%02x %02x %02x %02x %02x %02x %02x}\n",
                msg.id,
                msg.body[0], msg.body[1], msg.body[2],
                msg.body[3], msg.body[4], msg.body[5],
                msg.body[6]);

          retries++;
        }
    }
  while (id != 0xff && retries < 16);

errout_with_lock:

  /* Release our lock on the MXT device */

  nxmutex_unlock(&priv->devlock);

  /* Acknowledge and re-enable maXTouch interrupts */

  MXT_CLEAR(lower);
  MXT_ENABLE(lower);
}

/****************************************************************************
 * Name: mxt_interrupt
 ****************************************************************************/

static int mxt_interrupt(FAR const struct mxt_lower_s *lower, FAR void *arg)
{
  FAR struct mxt_dev_s *priv = (FAR struct mxt_dev_s *)arg;
  int ret;

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  DEBUGASSERT(lower != NULL && priv != NULL);

  /* Disable further interrupts */

  MXT_DISABLE(lower);

  /* Transfer processing to the worker thread.  Since maXTouch interrupts are
   * disabled while the work is pending, no special action should be required
   * to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, mxt_worker, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  /* Clear any pending interrupts and return success */

  lower->clear(lower);
  return OK;
}

/****************************************************************************
 * Name: mxt_open
 ****************************************************************************/

static int mxt_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct mxt_dev_s *priv;
  uint8_t tmp;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mxt_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ierr("ERROR: Too many opens: %d\n", priv->crefs);
      ret = -EMFILE;
      goto errout_with_lock;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */

  if (tmp == 1)
    {
      /* Touch enable */

      ret = mxt_putobject(priv, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0x83);
      if (ret < 0)
        {
          ierr("ERROR: Failed to enable touch: %d\n", ret);
          goto errout_with_lock;
        }

      /* Clear any pending messages by reading all messages.  This will
       * force the CHG interrupt pin to the high state and prevent spurious
       * interrupts when they are enabled.
       */

      ret = mxt_flushmsgs(priv);
      if (ret < 0)
        {
          ierr("ERROR: mxt_flushmsgs failed: %d\n", ret);
          mxt_putobject(priv, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0);
          goto errout_with_lock;
        }

      /* Enable touch interrupts */

      MXT_ENABLE(priv->lower);
    }

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_lock:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: mxt_close
 ****************************************************************************/

static int mxt_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct mxt_dev_s *priv;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mxt_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count unless it would decrement a negative
   * value.  When the count decrements to zero, there are no further
   * open references to the driver.
   */

  if (priv->crefs >= 1)
    {
      if (--priv->crefs < 1)
        {
          /* Disable touch interrupts */

          MXT_ENABLE(priv->lower);

          /* Touch disable */

          ret = mxt_putobject(priv, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0);
          if (ret < 0)
            {
              ierr("ERROR: Failed to disable touch: %d\n", ret);
            }
        }
    }

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: mxt_read
 ****************************************************************************/

static ssize_t mxt_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct mxt_dev_s *priv;
  ssize_t samplesize;
  int ncontacts;
  int ret;
  int i;
  int j;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mxt_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Locking the scheduler will prevent the worker thread from running
   * until we finish here.
   */

  sched_lock();

  /* Try to read sample data. */

  ret = mxt_checksample(priv);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      /* Wait for sample data */

      ret = mxt_waitsample(priv);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          goto errout;
        }
    }

  /* In any event, we now have sampled maXTouch data that we can report
   * to the caller.  First, count the number of valid contacts.
   */

  samplesize = 0;
  ncontacts  = 0;

  for (i = 0; i < priv->nslots; i++)
    {
      FAR struct mxt_sample_s *sample = &priv->sample[i];

      /* Do we need to report this?  We need to report the event if
       * it is CONTACT_LOST or CONTACT_REPORT or CONTACT_MOVE (with new,
       * valid positional data).
       */

      if (sample->contact == CONTACT_LOST ||
          sample->contact == CONTACT_NEW ||
          sample->contact == CONTACT_MOVE)
        {
          int newcount    = ncontacts + 1;
          ssize_t newsize = SIZEOF_TOUCH_SAMPLE_S(newcount);

          /* Would this sample exceed the buffer size provided by the
           * caller?
           */

          if (newsize > len)
            {
              /* Yes.. break out of the loop using the previous size and
               * count.
               */

              break;
            }

          /* Save the new size and count */

          ncontacts  = newcount;
          samplesize = newsize;
        }
    }

  /* Did we find any valid samples? */

  if (ncontacts > 0)
    {
      FAR struct touch_sample_s *report =
        (FAR struct touch_sample_s *)buffer;

      /* Yes, copy the sample data into the user buffer */

      memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(ncontacts));
      report->npoints = ncontacts;

      for (i = 0, j = 0; i < priv->nslots && j < ncontacts; i++)
        {
          FAR struct mxt_sample_s *sample = &priv->sample[i];

          /* Do we need to report this?  We need to report the event if
           * it is CONTACT_LOST or CONTACT_REPORT or CONTACT_MOVE (with new,
           * valid positional data).
           */

          if (sample->contact == CONTACT_LOST ||
              sample->contact == CONTACT_NEW ||
              sample->contact == CONTACT_MOVE)
            {
              /* Yes.. transfer the sample data */

              FAR struct touch_point_s *point = &report->point[j];
              j++;

              /* REVISIT:  height and width are not set, area is
               * not used.
               */

              point->id        = sample->id;
              point->x         = sample->x;
              point->y         = sample->y;
              point->pressure  = sample->pressure;

              /* Report the appropriate flags */

              if (sample->contact == CONTACT_LOST)
                {
                  /* The contact was lost.  Is the positional data
                   * valid?  This is important to know because the release
                   * will be sent to the window based on its last positional
                   * data.
                   */

                  if (sample->valid)
                    {
                      point->flags  = TOUCH_UP | TOUCH_ID_VALID |
                                      TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
                    }
                  else
                    {
                      point->flags  = TOUCH_UP | TOUCH_ID_VALID;
                    }

                  /* Change to CONTACT_NONE to indicate that the sample
                   * has been reported.  From here it can change only
                   * to CONTACT_REPORT (with a new ID).
                   */

                  sample->contact = CONTACT_NONE;
                }
              else
                {
                  /* We have contact.  Is it the first contact? */

                  if (sample->contact == CONTACT_NEW)
                    {
                      /* Yes.. first contact. */

                      point->flags = TOUCH_DOWN | TOUCH_ID_VALID |
                                     TOUCH_POS_VALID;
                    }
                  else /* if (sample->contact == CONTACT_MOVE) */
                    {
                      /* No.. then it must be movement of the same contact */

                      point->flags = TOUCH_MOVE | TOUCH_ID_VALID |
                                     TOUCH_POS_VALID;
                    }

                  /* Change to CONTACT_REPORT to indicate that the sample
                   * has been reported.  From here is can change to
                   * CONTACT_LOST (same ID), CONTACT_MOVE (same ID) or back
                   * to CONTACT_NEW (new ID).
                   */

                  sample->contact = CONTACT_REPORT;

                  /* A pressure measurement of zero means that pressure is
                   * not available.
                   */

                  if (point->pressure != 0)
                    {
                      point->flags  |= TOUCH_PRESSURE_VALID;
                    }
                }

              /* In any case, the sample data has been reported and is no
               * longer valid.
               */

              sample->valid = false;
            }
        }
    }

  ret = samplesize;

errout:
  sched_unlock();
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: mxt_ioctl
 ****************************************************************************/

static int mxt_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct mxt_dev_s *priv;
  int                       ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mxt_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      case TSIOC_SETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->lower != NULL && ptr != NULL);

          priv->frequency = *ptr;
        }
        break;

      case TSIOC_GETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->lower != NULL && ptr != NULL);
          *ptr = priv->frequency;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: mxt_poll
 ****************************************************************************/

static int mxt_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode         *inode;
  FAR struct mxt_dev_s *priv;
  int                       ret;
  int                       i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mxt_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ierr("ERROR: Missing POLLIN: revents: %08" PRIx32 "\n",
               fds->revents);
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_MXT_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_MXT_NPOLLWAITERS)
        {
          ierr("ERROR: No available slot found: %d\n", i);
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->event)
        {
          mxt_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: mxt_getinfo
 ****************************************************************************/

static int mxt_getinfo(struct mxt_dev_s *priv)
{
  int ret;

  /* Read 7-byte information block starting at address MXT_INFO */

  ret = mxt_getreg(priv, MXT_INFO, (FAR uint8_t *)&priv->info,
                   sizeof(struct mxt_info_s));
  if (ret < 0)
    {
      ierr("ERROR: mxt_getreg failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: mxt_getobjtab
 ****************************************************************************/

static int mxt_getobjtab(FAR struct mxt_dev_s *priv)
{
  FAR struct mxt_object_s *object;
  size_t tabsize;
  uint8_t idmin;
  uint8_t idmax;
  uint8_t id;
  int ret;
  int i;

  /* Read the size of the object table */

  tabsize = priv->info.nobjects * sizeof(struct mxt_object_s);
  ret = mxt_getreg(priv, MXT_OBJECT_START, (FAR uint8_t *)priv->objtab,
                   tabsize);
  if (ret < 0)
    {
      ierr("ERROR: Failed to object table size: %d\n", ret);
      return ret;
    }

  /* Search through the object table.  Find the values associated with
   * certain object types and save those ID.Valid report IDs start at ID=1.
   */

  for (i = 0, id = 1; i < priv->info.nobjects; i++)
    {
      object = &priv->objtab[i];
      if (object->nids > 0)
        {
          idmin  = id;
          id    += object->nids * (object->ninstances + 1);
          idmax  = id - 1;
        }
      else
        {
          idmin  = 0;
          idmax  = 0;
        }

      iinfo("%2d. type %2d addr %04x size: %d instances: %d IDs: %u-%u\n",
            i, object->type, MXT_GETUINT16(object->addr), object->size + 1,
            object->ninstances + 1, idmin, idmax);

      switch (object->type)
        {
#ifdef MXT_SUPPORT_T6
        case MXT_GEN_COMMAND_T6:
          priv->t6id = idmin;
          break;
#endif

        case MXT_TOUCH_MULTI_T9:
          priv->t9idmin = idmin;
          priv->t9idmax = idmax;
          break;

#ifdef CONFIG_MXT_BUTTONS
        case MXT_SPT_GPIOPWM_T19:
          priv->t19id = idmin;
          break;
#endif
        default:
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mxt_hwinitialize
 ****************************************************************************/

static int mxt_hwinitialize(FAR struct mxt_dev_s *priv)
{
  struct mxt_info_s *info = &priv->info;
  unsigned int nslots;
  uint8_t regval;
  int ret;

  /* Set the selected I2C frequency */

  priv->frequency = priv->lower->frequency;

  /* Read the info registers from the device */

  ret = mxt_getinfo(priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to read info registers: %d\n", ret);
      return ret;
    }

  /* Allocate memory for the object table */

  priv->objtab = kmm_zalloc(info->nobjects * sizeof(struct mxt_object_s));
  if (priv->objtab == NULL)
    {
      ierr("ERROR: Failed to allocate object table\n");
      return -ENOMEM;
    }

  /* Get object table information */

  ret = mxt_getobjtab(priv);
  if (ret < 0)
    {
      goto errout_with_objtab;
    }

  /* Perform a soft reset */

  ret = mxt_putobject(priv, MXT_GEN_COMMAND_T6, MXT_COMMAND_RESET, 1);
  if (ret < 0)
    {
      ierr("ERROR: Soft reset failed: %d\n", ret);
      goto errout_with_objtab;
    }

  nxsig_usleep(MXT_RESET_TIME);

  /* Update matrix size in the info structure */

  ret = mxt_getreg(priv, MXT_MATRIX_X_SIZE, (FAR uint8_t *)&regval, 1);
  if (ret < 0)
    {
      ierr("ERROR: Failed to get X size: %d\n", ret);
      goto errout_with_objtab;
    }

  info->xsize = regval;

  ret = mxt_getreg(priv, MXT_MATRIX_Y_SIZE, (FAR uint8_t *)&regval, 1);
  if (ret < 0)
    {
      ierr("ERROR: Failed to get Y size: %d\n", ret);
      goto errout_with_objtab;
    }

  info->ysize = regval;

  iinfo("Family: %u variant: %u version: %u.%u.%02x\n",
        info->family, info->variant, info->version >> 4,
        info->version & 0x0f, info->build);
  iinfo("Matrix size: (%u,%u) objects: %u\n",
        info->xsize, info->ysize, info->nobjects);

  /* How many multi touch "slots" */

  nslots = priv->t9idmax - priv->t9idmin + 1;
  DEBUGASSERT(nslots > 0 && nslots < 256);

  priv->nslots = nslots;

  /* Allocate a place to hold sample data for each slot */

  priv->sample = (FAR struct mxt_sample_s *)
    kmm_zalloc(nslots * sizeof(struct mxt_sample_s));
  if (priv->sample == NULL)
    {
      ierr("ERROR: Failed to allocate object table\n");
      ret = -ENOMEM;
      goto errout_with_objtab;
    }

  return OK;

  /* Error exits */

errout_with_objtab:
  kmm_free(priv->objtab);
  priv->objtab = NULL;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mxt_register
 *
 * Description:
 *   Configure the maXTouch to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   lower   - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mxt_register(FAR struct i2c_master_s *i2c,
                 FAR const struct mxt_lower_s * const lower, int minor)
{
  FAR struct mxt_dev_s *priv;
  char devname[DEV_NAMELEN];
  int ret;

  iinfo("i2c: %p minor: %d\n", i2c, minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(i2c != NULL && lower != NULL && minor >= 0 && minor < 100);

  /* Create and initialize a maXTouch device driver instance */

  priv = (FAR struct mxt_dev_s *)kmm_zalloc(sizeof(struct mxt_dev_s));
  if (priv == NULL)
    {
      ierr("ERROR: Failed allocate device structure\n");
      return -ENOMEM;
    }

  /* Initialize the ADS7843E device driver instance */

  memset(priv, 0, sizeof(struct mxt_dev_s));
  priv->i2c   = i2c;              /* Save the SPI device handle */
  priv->lower = lower;            /* Save the board configuration */

  /* Initialize mutex & semaphores */

  nxmutex_init(&priv->devlock);     /* Initialize device mutex */
  nxsem_init(&priv->waitsem, 0, 0); /* Initialize event wait semaphore */

  /* Make sure that interrupts are disabled */

  MXT_CLEAR(lower);
  MXT_DISABLE(lower);

  /* Attach the interrupt handler */

  ret = MXT_ATTACH(lower, mxt_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  /* Configure the MXT hardware */

  ret = mxt_hwinitialize(priv);
  if (ret < 0)
    {
      ierr("ERROR: mxt_hwinitialize failed: %d\n", ret);
      goto errout_with_irq;
    }

  /* Register the device as an input device */

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  iinfo("Registering %s\n", devname);

  ret = register_driver(devname, &g_mxt_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_hwinit;
    }

  /* And return success.  MXT interrupts will not be enable until the
   * MXT device has been opened (see mxt_open).
   */

  return OK;

  /* Error clean-up exits */

errout_with_hwinit:
  kmm_free(priv->objtab);
  kmm_free(priv->sample);
errout_with_irq:
  MXT_DETACH(lower);
errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  nxsem_destroy(&priv->waitsem);
  kmm_free(priv);
  return ret;
}
