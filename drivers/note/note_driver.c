/****************************************************************************
 * drivers/note/note_driver.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct note_info_s
{
  volatile unsigned int ni_head;
  volatile unsigned int ni_tail;
  uint8_t ni_buffer[CONFIG_SCHED_NOTE_BUFSIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t note_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations note_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  note_read,     /* read */
  NULL,          /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
  NULL           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0            /* unlink */
#endif
};

static struct note_info_s g_note_info;

#ifdef CONFIG_SMP
static volatile spinlock_t g_note_lock;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: note_next
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

static inline unsigned int note_next(unsigned int ndx, unsigned int offset)
{
  ndx += offset;
  if (ndx >= CONFIG_SCHED_NOTE_BUFSIZE)
    {
      ndx -= CONFIG_SCHED_NOTE_BUFSIZE;
    }

  return ndx;
}

/****************************************************************************
 * Name: note_length
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

static unsigned int note_length(void)
{
  unsigned int head = g_note_info.ni_head;
  unsigned int tail = g_note_info.ni_tail;

  if (tail > head)
    {
      head += CONFIG_SCHED_NOTE_BUFSIZE;
    }

  return head - tail;
}

/****************************************************************************
 * Name: note_remove
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

static void note_remove(void)
{
  FAR struct note_common_s *note;
  unsigned int tail;
  unsigned int length;

  /* Get the tail index of the circular buffer */

  tail = g_note_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_SCHED_NOTE_BUFSIZE);

  /* Get the length of the note at the tail index */

  note   = (FAR struct note_common_s *)&g_note_info.ni_buffer[tail];
  length = note->nc_length;
  DEBUGASSERT(length <= note_length());

  /* Increment the tail index to remove the entire note from the circular
   * buffer.
   */

  g_note_info.ni_tail = note_next(tail, length);
}

/****************************************************************************
 * Name: sched_note_get
 *
 * Description:
 *   Remove the next note from the tail of the circular buffer.  The note
 *   is also removed from the circular buffer to make room for further notes.
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

static ssize_t sched_note_get(FAR uint8_t *buffer, size_t buflen)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int remaining;
  unsigned int tail;
  ssize_t notelen;
  size_t circlen;

  DEBUGASSERT(buffer != NULL);
  flags = enter_critical_section();

  /* Verify that the circular buffer is not empty */

  circlen = note_length();
  if (circlen <= 0)
    {
      notelen = 0;
      goto errout_with_csection;
    }

  /* Get the index to the tail of the circular buffer */

  tail    = g_note_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_SCHED_NOTE_BUFSIZE);

  /* Get the length of the note at the tail index */

  note    = (FAR struct note_common_s *)&g_note_info.ni_buffer[tail];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

  /* Is the user buffer large enough to hold the note? */

  if (buflen < notelen)
    {
      /* Remove the large note so that we do not get constipated. */

      note_remove();

      /* and return an error */

      notelen = -EFBIG;
      goto errout_with_csection;
    }

  /* Loop until the note has been transferred to the user buffer */

  remaining = (unsigned int)notelen;
  while (remaining > 0)
    {
      /* Copy the next byte at the tail index */

      *buffer++ = g_note_info.ni_buffer[tail];

      /* Adjust indices and counts */

      tail = note_next(tail, 1);
      remaining--;
    }

  g_note_info.ni_tail = tail;

errout_with_csection:
  leave_critical_section(flags);
  return notelen;
}

/****************************************************************************
 * Name: sched_note_size
 *
 * Description:
 *   Return the size of the next note at the tail of the circular buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero is returned if the circular buffer is empty.  Otherwise, the size
 *   of the next note is returned.
 *
 ****************************************************************************/

static ssize_t sched_note_size(void)
{
  FAR struct note_common_s *note;
  irqstate_t flags;
  unsigned int tail;
  ssize_t notelen;
  size_t circlen;

  flags = enter_critical_section();

  /* Verify that the circular buffer is not empty */

  circlen = note_length();
  if (circlen <= 0)
    {
      notelen = 0;
      goto errout_with_csection;
    }

  /* Get the index to the tail of the circular buffer */

  tail = g_note_info.ni_tail;
  DEBUGASSERT(tail < CONFIG_SCHED_NOTE_BUFSIZE);

  /* Get the length of the note at the tail index */

  note    = (FAR struct note_common_s *)&g_note_info.ni_buffer[tail];
  notelen = note->nc_length;
  DEBUGASSERT(notelen <= circlen);

errout_with_csection:
  leave_critical_section(flags);
  return notelen;
}

/****************************************************************************
 * Name: note_read
 ****************************************************************************/

static ssize_t note_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
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

      notelen = sched_note_get((FAR uint8_t *)buffer, buflen);
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
       * note will fit, it may fail still when sched_note_get() is called.
       *
       * It won't fit (or an error occurred).  Return what we have without
       * trying to get the next note (which would cause it to be deleted).
       */

      notelen = sched_note_size();
    }
  while (notelen > 0 && notelen <= buflen);

  sched_unlock();
  return retlen;
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

#ifdef CONFIG_SMP
  irqstate_t flags = up_irq_save();
  spin_lock_wo_note(&g_note_lock);
#endif

  /* Get the index to the head of the circular buffer */

  DEBUGASSERT(note != NULL && notelen < CONFIG_SCHED_NOTE_BUFSIZE);
  head = g_note_info.ni_head;

  /* Loop until all bytes have been transferred to the circular buffer */

  while (notelen > 0)
    {
      /* Get the next head index.  Would it collide with the current tail
       * index?
       */

      next = note_next(head, 1);
      if (next == g_note_info.ni_tail)
        {
          /* Yes, then remove the note at the tail index */

          note_remove();
        }

      /* Save the next byte at the head index */

      g_note_info.ni_buffer[head] = *buf++;

      head = next;
      notelen--;
    }

  g_note_info.ni_head = head;

#ifdef CONFIG_SMP
  spin_unlock_wo_note(&g_note_lock);
  up_irq_restore(flags);
#endif
}

/****************************************************************************
 * Name: note_register
 *
 * Description:
 *   Register a serial driver at /dev/note that can be used by an
 *   application to read data from the circular note buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero is returned if the circular buffer is empty.  Otherwise, a negated
 *   errno value is returned.
 *
 ****************************************************************************/

int note_register(void)
{
  return register_driver("/dev/note", &note_fops, 0666, NULL);
}
