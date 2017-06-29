/****************************************************************************
 * drivers/syslog/note_driver.c
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

#include <nuttx/sched_note.h>
#include <nuttx/fs/fs.h>

#if defined(CONFIG_SCHED_INSTRUMENTATION_BUFFER) && \
    defined(CONFIG_DRIVER_NOTE)

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
  0,             /* open */
  0,             /* close */
  note_read,     /* read */
  0,             /* write */
  0,             /* seek */
  0              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0            /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 * Name: note_register
 *
 * Description:
 *   Register a serial driver at /dev/note that can be used by an
 *   application to read data from the circular not buffer.
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

#endif /* CONFIG_SCHED_INSTRUMENTATION_BUFFER && CONFIG_DRIVER_NOTE */