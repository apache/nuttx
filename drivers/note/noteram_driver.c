/****************************************************************************
 * drivers/note/noteram_driver.c
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
#include <sched.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct noteram_info_s
{
  volatile unsigned int ni_head;
  volatile unsigned int ni_tail;
  volatile unsigned int ni_read;
  unsigned int ni_overwrite;
  uint8_t ni_buffer[CONFIG_DRIVER_NOTERAM_BUFSIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int noteram_open(FAR struct file *filep);
static ssize_t noteram_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen);
static int noteram_ioctl(struct file *filep, int cmd, unsigned long arg);

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
  NULL           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0            /* unlink */
#endif
};

static struct noteram_info_s g_noteram_info =
{
#ifdef CONFIG_DRIVER_NOTERAM_DEFAULT_NOOVERWRITE
  .ni_overwrite = NOTERAM_MODE_OVERWRITE_DISABLE
#else
  .ni_overwrite = NOTERAM_MODE_OVERWRITE_ENABLE
#endif
};

#ifdef CONFIG_SMP
static volatile spinlock_t g_noteram_lock;
#endif

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

static void noteram_buffer_clear(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  g_noteram_info.ni_tail = g_noteram_info.ni_head;
  g_noteram_info.ni_read = g_noteram_info.ni_head;

  if (g_noteram_info.ni_overwrite == NOTERAM_MODE_OVERWRITE_OVERFLOW)
    {
      g_noteram_info.ni_overwrite = NOTERAM_MODE_OVERWRITE_DISABLE;
    }

  leave_critical_section(flags);
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

static inline unsigned int noteram_next(unsigned int ndx,
                                        unsigned int offset)
{
  ndx += offset;
  if (ndx >= CONFIG_DRIVER_NOTERAM_BUFSIZE)
    {
      ndx -= CONFIG_DRIVER_NOTERAM_BUFSIZE;
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

#ifdef CONFIG_DEBUG_ASSERTIONS
static unsigned int noteram_length(void)
{
  unsigned int head = g_noteram_info.ni_head;
  unsigned int tail = g_noteram_info.ni_tail;

  if (tail > head)
    {
      head += CONFIG_DRIVER_NOTERAM_BUFSIZE;
    }

  return head - tail;
}
#endif

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

static unsigned int noteram_unread_length(void)
{
  unsigned int head = g_noteram_info.ni_head;
  unsigned int read = g_noteram_info.ni_read;

  if (read > head)
    {
      head += CONFIG_DRIVER_NOTERAM_BUFSIZE;
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

static void noteram_remove(void)
{
  FAR struct note_common_s *note;
  unsigned int tail;
  unsigned int length;

  /* Get the tail index of the circular buffer */

  tail = g_noteram_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_DRIVER_NOTERAM_BUFSIZE);

  /* Get the length of the note at the tail index */

  note   = (FAR struct note_common_s *)&g_noteram_info.ni_buffer[tail];
  length = note->nc_length;
  DEBUGASSERT(length <= noteram_length());

  /* Increment the tail index to remove the entire note from the circular
   * buffer.
   */

  if (g_noteram_info.ni_read == g_noteram_info.ni_tail)
    {
      /* The read index also needs increment. */

      g_noteram_info.ni_read = noteram_next(tail, length);
    }

  g_noteram_info.ni_tail = noteram_next(tail, length);
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

static ssize_t noteram_get(FAR uint8_t *buffer, size_t buflen)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int remaining;
  unsigned int read;
  ssize_t notelen;
  size_t circlen;

  DEBUGASSERT(buffer != NULL);
  flags = enter_critical_section();

  /* Verify that the circular buffer is not empty */

  circlen = noteram_unread_length();
  if (circlen <= 0)
    {
      notelen = 0;
      goto errout_with_csection;
    }

  /* Get the read index of the circular buffer */

  read    = g_noteram_info.ni_read;
  DEBUGASSERT(read < CONFIG_DRIVER_NOTERAM_BUFSIZE);

  /* Get the length of the note at the read index */

  note    = (FAR struct note_common_s *)&g_noteram_info.ni_buffer[read];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

  /* Is the user buffer large enough to hold the note? */

  if (buflen < notelen)
    {
      /* Skip the large note so that we do not get constipated. */

      g_noteram_info.ni_read = noteram_next(read, notelen);

      /* and return an error */

      notelen = -EFBIG;
      goto errout_with_csection;
    }

  /* Loop until the note has been transferred to the user buffer */

  remaining = (unsigned int)notelen;
  while (remaining > 0)
    {
      /* Copy the next byte at the read index */

      *buffer++ = g_noteram_info.ni_buffer[read];

      /* Adjust indices and counts */

      read = noteram_next(read, 1);
      remaining--;
    }

  g_noteram_info.ni_read = read;

errout_with_csection:
  leave_critical_section(flags);
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

static ssize_t noteram_size(void)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int read;
  ssize_t notelen;
  size_t circlen;

  flags = enter_critical_section();

  /* Verify that the circular buffer is not empty */

  circlen = noteram_unread_length();
  if (circlen <= 0)
    {
      notelen = 0;
      goto errout_with_csection;
    }

  /* Get the read index of the circular buffer */

  read = g_noteram_info.ni_read;
  DEBUGASSERT(read < CONFIG_DRIVER_NOTERAM_BUFSIZE);

  /* Get the length of the note at the read index */

  note    = (FAR struct note_common_s *)&g_noteram_info.ni_buffer[read];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

errout_with_csection:
  leave_critical_section(flags);
  return notelen;
}

/****************************************************************************
 * Name: noteram_open
 ****************************************************************************/

static int noteram_open(FAR struct file *filep)
{
  /* Reset the read index of the circular buffer */

  g_noteram_info.ni_read = g_noteram_info.ni_tail;

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

  DEBUGASSERT(filep != 0 && buffer != NULL && buflen > 0);

  /* Then loop, adding as many notes as possible to the user buffer. */

  retlen = 0;
  sched_lock();
  do
    {
      /* Get the next note (removing it from the buffer) */

      notelen = noteram_get((FAR uint8_t *)buffer, buflen);
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

      notelen = noteram_size();
    }
  while (notelen > 0 && notelen <= buflen);

  sched_unlock();
  return retlen;
}

/****************************************************************************
 * Name: noteram_ioctl
 ****************************************************************************/

static int noteram_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOSYS;

  /* Handle the ioctl commands */

  switch (cmd)
    {
      /* NOTERAM_CLEAR
       *      - Clear all contents of the circular buffer
       *        Argument: Ignored
       */

      case NOTERAM_CLEAR:
        noteram_buffer_clear();
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
            *(unsigned int *)arg = g_noteram_info.ni_overwrite;
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
            g_noteram_info.ni_overwrite = *(unsigned int *)arg;
            ret = OK;
          }
        break;

      default:
          break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_add
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

void sched_note_add(FAR const void *note, size_t notelen)
{
  FAR const char *buf = note;
  unsigned int head;
  unsigned int next;
  irqstate_t flags;

  flags = up_irq_save();
#ifdef CONFIG_SMP
  spin_lock_wo_note(&g_noteram_lock);
#endif

  if (g_noteram_info.ni_overwrite == NOTERAM_MODE_OVERWRITE_OVERFLOW)
    {
#ifdef CONFIG_SMP
      spin_unlock_wo_note(&g_noteram_lock);
#endif
      up_irq_restore(flags);
      return;
    }

  /* Get the index to the head of the circular buffer */

  DEBUGASSERT(note != NULL && notelen < CONFIG_DRIVER_NOTERAM_BUFSIZE);
  head = g_noteram_info.ni_head;

  /* Loop until all bytes have been transferred to the circular buffer */

  while (notelen > 0)
    {
      /* Get the next head index.  Would it collide with the current tail
       * index?
       */

      next = noteram_next(head, 1);
      if (next == g_noteram_info.ni_tail)
        {
          if (g_noteram_info.ni_overwrite == NOTERAM_MODE_OVERWRITE_DISABLE)
            {
              /* Stop recording if not in overwrite mode */

              g_noteram_info.ni_overwrite = NOTERAM_MODE_OVERWRITE_OVERFLOW;

#ifdef CONFIG_SMP
              spin_unlock_wo_note(&g_noteram_lock);
#endif
              up_irq_restore(flags);
              return;
            }

          /* Yes, then remove the note at the tail index */

          noteram_remove();
        }

      /* Save the next byte at the head index */

      g_noteram_info.ni_buffer[head] = *buf++;

      head = next;
      notelen--;
    }

  g_noteram_info.ni_head = head;

#ifdef CONFIG_SMP
  spin_unlock_wo_note(&g_noteram_lock);
#endif
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: noteram_register
 *
 * Description:
 *   Register a serial driver at /dev/note that can be used by an
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
  return register_driver("/dev/note", &g_noteram_fops, 0666, NULL);
}
