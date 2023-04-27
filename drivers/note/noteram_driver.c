/****************************************************************************
 * drivers/note/noteram_driver.c
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
#include <sched.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <string.h>

#include <nuttx/spinlock.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct noteram_driver_s
{
  struct note_driver_s driver;
  FAR uint8_t *ni_buffer;
  size_t ni_bufsize;
  unsigned int ni_overwrite;
  volatile unsigned int ni_head;
  volatile unsigned int ni_tail;
  volatile unsigned int ni_read;
  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int noteram_open(FAR struct file *filep);
static ssize_t noteram_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen);
static int noteram_ioctl(struct file *filep, int cmd, unsigned long arg);
static void noteram_add(FAR struct note_driver_s *drv,
                        FAR const void *note, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_noteram_fops =
{
  noteram_open,  /* open */
  NULL,          /* close */
  noteram_read,  /* read */
  NULL,          /* write */
  NULL,          /* seek */
  noteram_ioctl, /* ioctl */
};

static uint8_t g_ramnote_buffer[CONFIG_DRIVERS_NOTERAM_BUFSIZE];

static const struct note_driver_ops_s g_noteram_ops =
{
  noteram_add
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct noteram_driver_s g_noteram_driver =
{
  {&g_noteram_ops},
  g_ramnote_buffer,
  CONFIG_DRIVERS_NOTERAM_BUFSIZE,
#ifdef CONFIG_DRIVERS_NOTERAM_DEFAULT_NOOVERWRITE
  NOTERAM_MODE_OVERWRITE_DISABLE
#else
  NOTERAM_MODE_OVERWRITE_ENABLE
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noteram_buffer_clear
 *
 * Description:
 *   Clear all contents of the circular buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void noteram_buffer_clear(FAR struct noteram_driver_s *drv)
{
  drv->ni_tail = drv->ni_head;
  drv->ni_read = drv->ni_head;

  if (drv->ni_overwrite == NOTERAM_MODE_OVERWRITE_OVERFLOW)
    {
      drv->ni_overwrite = NOTERAM_MODE_OVERWRITE_DISABLE;
    }
}

/****************************************************************************
 * Name: noteram_next
 *
 * Description:
 *   Return the circular buffer index at offset from the specified index
 *   value, handling wraparound
 *
 * Input Parameters:
 *   ndx - Old circular buffer index
 *
 * Returned Value:
 *   New circular buffer index
 *
 ****************************************************************************/

static inline unsigned int noteram_next(FAR struct noteram_driver_s *drv,
                                        unsigned int ndx,
                                        unsigned int offset)
{
  ndx += offset;
  if (ndx >= drv->ni_bufsize)
    {
      ndx -= drv->ni_bufsize;
    }

  return ndx;
}

/****************************************************************************
 * Name: noteram_length
 *
 * Description:
 *   Length of data currently in circular buffer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Length of data currently in circular buffer.
 *
 ****************************************************************************/

static unsigned int noteram_length(FAR struct noteram_driver_s *drv)
{
  unsigned int head = drv->ni_head;
  unsigned int tail = drv->ni_tail;

  if (tail > head)
    {
      head += drv->ni_bufsize;
    }

  return head - tail;
}

/****************************************************************************
 * Name: noteram_unread_length
 *
 * Description:
 *   Length of unread data currently in circular buffer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Length of unread data currently in circular buffer.
 *
 ****************************************************************************/

static unsigned int noteram_unread_length(FAR struct noteram_driver_s *drv)
{
  unsigned int head = drv->ni_head;
  unsigned int read = drv->ni_read;

  if (read > head)
    {
      head += drv->ni_bufsize;
    }

  return head - read;
}

/****************************************************************************
 * Name: noteram_remove
 *
 * Description:
 *   Remove the variable length note from the tail of the circular buffer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

static void noteram_remove(FAR struct noteram_driver_s *drv)
{
  unsigned int tail;
  unsigned int length;

  /* Get the tail index of the circular buffer */

  tail = drv->ni_tail;
  DEBUGASSERT(tail < drv->ni_bufsize);

  /* Get the length of the note at the tail index */

  length = drv->ni_buffer[tail];
  DEBUGASSERT(length <= noteram_length(drv));

  /* Increment the tail index to remove the entire note from the circular
   * buffer.
   */

  if (drv->ni_read == drv->ni_tail)
    {
      /* The read index also needs increment. */

      drv->ni_read = noteram_next(drv, tail, length);
    }

  drv->ni_tail = noteram_next(drv, tail, length);
}

/****************************************************************************
 * Name: noteram_get
 *
 * Description:
 *   Get the next note from the read index of the circular buffer.
 *
 * Input Parameters:
 *   buffer - Location to return the next note
 *   buflen - The length of the user provided buffer.
 *
 * Returned Value:
 *   On success, the positive, non-zero length of the return note is
 *   provided.  Zero is returned only if the circular buffer is empty.  A
 *   negated errno value is returned in the event of any failure.
 *
 ****************************************************************************/

static ssize_t noteram_get(FAR struct noteram_driver_s *drv,
                           FAR uint8_t *buffer, size_t buflen)
{
  FAR struct note_common_s *note;
  unsigned int remaining;
  unsigned int read;
  ssize_t notelen;
  size_t circlen;

  DEBUGASSERT(buffer != NULL);

  /* Verify that the circular buffer is not empty */

  circlen = noteram_unread_length(drv);
  if (circlen <= 0)
    {
      return 0;
    }

  /* Get the read index of the circular buffer */

  read = drv->ni_read;
  DEBUGASSERT(read < drv->ni_bufsize);

  /* Get the length of the note at the read index */

  note = (FAR struct note_common_s *)&drv->ni_buffer[read];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

  /* Is the user buffer large enough to hold the note? */

  if (buflen < notelen)
    {
      /* Skip the large note so that we do not get constipated. */

      drv->ni_read = noteram_next(drv, read, notelen);

      /* and return an error */

      return -EFBIG;
    }

  /* Loop until the note has been transferred to the user buffer */

  remaining = (unsigned int)notelen;
  while (remaining > 0)
    {
      /* Copy the next byte at the read index */

      *buffer++ = drv->ni_buffer[read];

      /* Adjust indices and counts */

      read = noteram_next(drv, read, 1);
      remaining--;
    }

  drv->ni_read = read;

  return notelen;
}

/****************************************************************************
 * Name: noteram_size
 *
 * Description:
 *   Return the size of the next note at the read index of the circular
 *   buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero is returned if the circular buffer is empty.  Otherwise, the size
 *   of the next note is returned.
 *
 ****************************************************************************/

static ssize_t noteram_size(FAR struct noteram_driver_s *drv)
{
  FAR struct note_common_s *note;
  unsigned int read;
  ssize_t notelen;
  size_t circlen;

  /* Verify that the circular buffer is not empty */

  circlen = noteram_unread_length(drv);
  if (circlen <= 0)
    {
      return 0;
    }

  /* Get the read index of the circular buffer */

  read = drv->ni_read;
  DEBUGASSERT(read < drv->ni_bufsize);

  /* Get the length of the note at the read index */

  note    = (FAR struct note_common_s *)&drv->ni_buffer[read];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

  return notelen;
}

/****************************************************************************
 * Name: noteram_open
 ****************************************************************************/

static int noteram_open(FAR struct file *filep)
{
  FAR struct noteram_driver_s *drv =
    (FAR struct noteram_driver_s *)filep->f_inode->i_private;

  /* Reset the read index of the circular buffer */

  filep->f_priv = drv;
  drv->ni_read = drv->ni_tail;

  return OK;
}

/****************************************************************************
 * Name: noteram_read
 ****************************************************************************/

static ssize_t noteram_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen)
{
  ssize_t notelen;
  ssize_t retlen ;
  irqstate_t flags;
  FAR struct noteram_driver_s *drv =
    (FAR struct noteram_driver_s *)filep->f_priv;

  DEBUGASSERT(filep != 0 && buffer != NULL && buflen > 0);

  /* Then loop, adding as many notes as possible to the user buffer. */

  retlen = 0;
  flags = spin_lock_irqsave_wo_note(&drv->lock);
  do
    {
      /* Get the next note (removing it from the buffer) */

      notelen = noteram_get(drv, (FAR uint8_t *)buffer, buflen);
      if (notelen < 0)
        {
          /* We were unable to read the next note, probably because it will
           * not fit into the user buffer.
           */

          if (retlen == 0)
            {
              /* If nothing was read then report the error.  Otherwise,
               * just silently drop the note.
               */

              retlen = notelen;
            }

          break;
        }

      /* Update pointers from the note that was transferred */

      retlen += notelen;
      buffer += notelen;
      buflen -= notelen;

      /* Will the next note fit?  There is a race here and even if the next
       * note will fit, it may fail still when noteram_get() is called.
       *
       * It won't fit (or an error occurred).  Return what we have without
       * trying to get the next note (which would cause it to be deleted).
       */

      notelen = noteram_size(drv);
    }
  while (notelen > 0 && notelen <= buflen);

  spin_unlock_irqrestore_wo_note(&drv->lock, flags);
  return retlen;
}

/****************************************************************************
 * Name: noteram_ioctl
 ****************************************************************************/

static int noteram_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOSYS;
  FAR struct noteram_driver_s *drv =
    (FAR struct noteram_driver_s *)filep->f_priv;
  irqstate_t flags = spin_lock_irqsave_wo_note(&drv->lock);

  /* Handle the ioctl commands */

  switch (cmd)
    {
      /* NOTERAM_CLEAR
       *      - Clear all contents of the circular buffer
       *        Argument: Ignored
       */

      case NOTERAM_CLEAR:
        noteram_buffer_clear(drv);
        ret = OK;
        break;

      /* NOTERAM_GETMODE
       *      - Get overwrite mode
       *        Argument: A writable pointer to unsigned int
       */

      case NOTERAM_GETMODE:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            *(unsigned int *)arg = drv->ni_overwrite;
            ret = OK;
          }
        break;

      /* NOTERAM_SETMODE
       *      - Set overwrite mode
       *        Argument: A read-only pointer to unsigned int
       */

      case NOTERAM_SETMODE:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            drv->ni_overwrite = *(unsigned int *)arg;
            ret = OK;
          }
        break;

#ifdef NOTERAM_GETTASKNAME
      /* NOTERAM_GETTASKNAME
       *      - Get task name string
       *        Argument: A writable pointer to struct note_get_taskname_s
       *        Result:   If -ESRCH, the corresponding task name doesn't
       *                  exist.
       */

      case NOTERAM_GETTASKNAME:
        {
          FAR struct noteram_get_taskname_s *param;

          if (arg == 0)
            {
              ret = -EINVAL;
              break;
            }

          param = (FAR struct noteram_get_taskname_s *)arg;
          ret = note_get_taskname(param->pid, param->taskname);
        }
        break;
#endif

      default:
          break;
    }

  spin_unlock_irqrestore_wo_note(&drv->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: noteram_add
 *
 * Description:
 *   Add the variable length note to the transport layer
 *
 * Input Parameters:
 *   note    - The note buffer
 *   notelen - The buffer length
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

static void noteram_add(FAR struct note_driver_s *driver,
                        FAR const void *note, size_t notelen)
{
  FAR const char *buf = note;
  FAR struct noteram_driver_s *drv = (FAR struct noteram_driver_s *)driver;
  unsigned int head;
  unsigned int remain;
  unsigned int space;
  irqstate_t flags;

  flags = spin_lock_irqsave_wo_note(&drv->lock);

  if (drv->ni_overwrite == NOTERAM_MODE_OVERWRITE_OVERFLOW)
    {
      spin_unlock_irqrestore_wo_note(&drv->lock, flags);
      return;
    }

  DEBUGASSERT(note != NULL && notelen < drv->ni_bufsize);
  remain = drv->ni_bufsize - noteram_length(drv);

  if (remain < notelen)
    {
      if (drv->ni_overwrite == NOTERAM_MODE_OVERWRITE_DISABLE)
        {
          /* Stop recording if not in overwrite mode */

          drv->ni_overwrite = NOTERAM_MODE_OVERWRITE_OVERFLOW;
          spin_unlock_irqrestore_wo_note(&drv->lock, flags);
          return;
        }

      /* Remove the note at the tail index , make sure there is enough space
       */

      do
        {
          noteram_remove(drv);
          remain = drv->ni_bufsize - noteram_length(drv);
        }
      while (remain < notelen);
    }

  head = drv->ni_head;
  space = drv->ni_bufsize - head;
  space = space < notelen ? space : notelen;
  memcpy(drv->ni_buffer + head, note, space);
  memcpy(drv->ni_buffer, buf + space, notelen - space);
  drv->ni_head = noteram_next(drv, head, notelen);
  spin_unlock_irqrestore_wo_note(&drv->lock, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noteram_register
 *
 * Description:
 *   Register a serial driver at /dev/note/ram that can be used by an
 *   application to read data from the circular note buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int noteram_register(void)
{
  return register_driver("/dev/note/ram", &g_noteram_fops, 0666,
                         &g_noteram_driver);
}
