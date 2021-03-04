/****************************************************************************
 * drivers/syslog/syslog_device.c
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

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/syslog/syslog.h>

#include "syslog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Open the device/file write-only, try to create (file) it if it doesn't
 * exist, if the file that already exists, then append the new log data to
 * end of the file.
 */

#define SYSLOG_OFLAGS (O_WRONLY | O_CREAT | O_APPEND)

/* An invalid thread ID */

#define NO_HOLDER     ((pid_t)-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration represents the state of the SYSLOG device interface */

enum syslog_dev_state
{
  SYSLOG_UNINITIALIZED = 0, /* SYSLOG has not been initialized */
  SYSLOG_INITIALIZING,      /* SYSLOG is being initialized */
  SYSLOG_REOPEN,            /* SYSLOG open failed... try again later */
  SYSLOG_FAILURE,           /* SYSLOG open failed... don't try again */
  SYSLOG_OPENED,            /* SYSLOG device is open and ready to use */
};

/* This structure contains all SYSLOGing state information */

struct syslog_dev_s
{
  uint8_t      sl_state;    /* See enum syslog_dev_state */
  uint8_t      sl_oflags;   /* Saved open mode (for re-open) */
  uint16_t     sl_mode;     /* Saved open flags (for re-open) */
  sem_t        sl_sem;      /* Enforces mutually exclusive access */
  pid_t        sl_holder;   /* PID of the thread that holds the semaphore */
  struct file  sl_file;     /* The syslog file structure */
  FAR char    *sl_devpath;  /* Full path to the character device */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the device structure for the console or syslogging function. */

static struct syslog_dev_s g_syslog_dev;
static const uint8_t g_syscrlf[2] =
{
  '\r', '\n'
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_dev_takesem
 ****************************************************************************/

static inline int syslog_dev_takesem(void)
{
  pid_t me = getpid();
  int ret;

  /* Does this thread already hold the semaphore?  That could happen if
   * we were called recursively, i.e., if the logic kicked off by
   * file_write() where to generate more debug output.  Return an
   * error in that case.
   */

  if (g_syslog_dev.sl_holder == me)
    {
      /* Return an error (instead of deadlocking) */

      return -EWOULDBLOCK;
    }

  /* Either the semaphore is available or is currently held by another
   * thread.  Wait for it to become available.
   */

  ret = nxsem_wait(&g_syslog_dev.sl_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* We hold the semaphore.  We can safely mark ourself as the holder
   * of the semaphore.
   */

  g_syslog_dev.sl_holder = me;
  return OK;
}

/****************************************************************************
 * Name: syslog_dev_givesem
 ****************************************************************************/

static inline void syslog_dev_givesem(void)
{
#ifdef CONFIG_DEBUG_ASSERTIONS
  pid_t me = getpid();
  DEBUGASSERT(g_syslog_dev.sl_holder == me);
#endif

  /* Relinquish the semaphore */

  g_syslog_dev.sl_holder = NO_HOLDER;
  nxsem_post(&g_syslog_dev.sl_sem);
}

/****************************************************************************
 * Name: syslog_dev_outputready
 *
 * Description:
 * Ignore any output:
 *
 * (1) Before the SYSLOG device has been initialized.  This could happen
 *     from debug output that occurs early in the boot sequence before
 *     syslog_dev_initialize() is called (SYSLOG_UNINITIALIZED).
 * (2) While the device is being initialized.  The case could happen if
 *     debug output is generated while syslog_dev_initialize() executes
 *     (SYSLOG_INITIALIZING).
 * (3) While we are generating SYSLOG output.  The case could happen if
 *     debug output is generated while syslog_dev_putc() executes
 *     (This case is actually handled inside of syslog_semtake()).
 * (4) Any debug output generated from interrupt handlers.  A disadvantage
 *     of using the generic character device for the SYSLOG is that it
 *     cannot handle debug output generated from interrupt level handlers.
 * (5) Any debug output generated from the IDLE loop.  The character
 *     driver interface is blocking and the IDLE thread is not permitted
 *     to block.
 * (6) If an irrecoverable failure occurred during initialization.  In
 *     this case, we won't ever bother to try again (ever).
 *
 * NOTE: That the third case is different.  It applies only to the thread
 * that currently holds the sl_sem semaphore.  Other threads should wait.
 * that is why that case is handled in syslog_semtake().
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int syslog_dev_outputready(void)
{
  int ret;

  /* Cases (4) and (5) */

  if (up_interrupt_context() || getpid() == 0)
    {
      return -ENOSYS;
    }

  /* We can save checks in the usual case:  That after the SYSLOG device
   * has been successfully opened.
   */

  if (g_syslog_dev.sl_state != SYSLOG_OPENED)
    {
      /* Case (1) and (2) */

      if (g_syslog_dev.sl_state == SYSLOG_UNINITIALIZED ||
          g_syslog_dev.sl_state == SYSLOG_INITIALIZING)
        {
          return -EAGAIN; /* Can't access the SYSLOG now... maybe next time? */
        }

      /* Case (6) */

      if (g_syslog_dev.sl_state == SYSLOG_FAILURE)
        {
          return -ENXIO;  /* There is no SYSLOG device */
        }

      /* syslog_dev_initialize() is called as soon as enough of the operating
       * system is in place to support the open operation... but it is
       * possible that the SYSLOG device is not yet registered at that time.
       * In this case, we know that the system is sufficiently initialized
       * to support an attempt to re-open the SYSLOG device.
       *
       * NOTE that the scheduler is locked.  That is because we do not have
       * fully initialized semaphore capability until the SYSLOG device is
       * successfully initialized.
       */

      sched_lock();
      if (g_syslog_dev.sl_state == SYSLOG_REOPEN)
        {
          /* Try again to initialize the device.  We may do this repeatedly
           * because the log device might be something that was not ready
           * the first time that syslog_dev_initialize() was called (such as
           * a USB serial device that has not yet been connected or a file in
           * an NFS mounted file system that has not yet been mounted).
           */

          DEBUGASSERT(g_syslog_dev.sl_devpath != NULL);
          ret = syslog_dev_initialize(g_syslog_dev.sl_devpath,
                                      (int)g_syslog_dev.sl_oflags,
                                      (int)g_syslog_dev.sl_mode);
          if (ret < 0)
            {
              sched_unlock();
              return ret;
            }
        }

      sched_unlock();
      DEBUGASSERT(g_syslog_dev.sl_state == SYSLOG_OPENED);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_dev_initialize
 *
 * Description:
 *   Initialize to use the character device (or file) at
 *   CONFIG_SYSLOG_DEVPATH as the SYSLOG sink.
 *
 *   One power up, the SYSLOG facility is non-existent or limited to very
 *   low-level output.  This function may be called later in the
 *   initialization sequence after full driver support has been initialized.
 *   (via syslog_initialize())  It installs the configured SYSLOG drivers
 *   and enables full SYSLOGing capability.
 *
 *   NOTE that this implementation excludes using a network connection as
 *   SYSLOG device.  That would be a good extension.
 *
 * Input Parameters:
 *   devpath - The full path to the character device to be used.
 *   oflags  - File open flags
 *   mode    - File open mode (only if oflags include O_CREAT)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int syslog_dev_initialize(FAR const char *devpath, int oflags, int mode)
{
  int ret;

  /* At this point, the only expected states are SYSLOG_UNINITIALIZED or
   * SYSLOG_REOPEN.  Not SYSLOG_INITIALIZING, SYSLOG_FAILURE, SYSLOG_OPENED.
   */

  DEBUGASSERT(g_syslog_dev.sl_state == SYSLOG_UNINITIALIZED ||
              g_syslog_dev.sl_state == SYSLOG_REOPEN);

  /* Save the path to the device in case we have to re-open it.
   * If we get here and sl_devpath is not equal to NULL, that is a clue
   * that we are re-opening the file.
   */

  if (g_syslog_dev.sl_state == SYSLOG_REOPEN)
    {
      /* Re-opening: Then we should already have a copy of the path to the
       * device. But that may be for a different device if we revert back
       * to old syslog destination after the previous attempt failed.
       */

      DEBUGASSERT(g_syslog_dev.sl_devpath != NULL);
    }
  else
    {
      /* Initializing. We do not have the device path yet. */

      DEBUGASSERT(g_syslog_dev.sl_devpath == NULL);
    }

  /* Copy the device path so that we can use it if we
   * have to re-open the file.
   */

  g_syslog_dev.sl_oflags  = oflags;
  g_syslog_dev.sl_mode    = mode;
  if (g_syslog_dev.sl_devpath != devpath)
    {
      if (g_syslog_dev.sl_devpath != NULL)
        {
          kmm_free(g_syslog_dev.sl_devpath);
        }

      g_syslog_dev.sl_devpath = strdup(devpath);
    }

  DEBUGASSERT(g_syslog_dev.sl_devpath != NULL);

  g_syslog_dev.sl_state = SYSLOG_INITIALIZING;

  /* Open the device driver. */

  ret = file_open(&g_syslog_dev.sl_file, devpath, oflags, mode);
  if (ret < 0)
    {
      /* We failed to open the file. Perhaps it does exist?  Perhaps it
       * exists, but is not ready because it depends on insertion of a
       * removable device?
       *
       * In any case we will attempt to re-open the device repeatedly.
       * The assumption is that the device path is valid but that the
       * driver has not yet been registered or a removable device has
       * not yet been installed.
       */

      g_syslog_dev.sl_state = SYSLOG_REOPEN;
      return ret;
    }

  /* The SYSLOG device is open and ready for writing. */

  nxsem_init(&g_syslog_dev.sl_sem, 0, 1);
  g_syslog_dev.sl_holder = NO_HOLDER;
  g_syslog_dev.sl_state  = SYSLOG_OPENED;
  return OK;
}

/****************************************************************************
 * Name: syslog_dev_uninitialize
 *
 * Description:
 *   Called to disable the last device/file channel in preparation to use
 *   a different SYSLOG device. Currently only used for CONFIG_SYSLOG_FILE.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   The caller has already switched the SYSLOG source to some safe channel
 *   (the default channel).
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_FILE /* Currently only used in this configuration */
void syslog_dev_uninitialize(void)
{
  /* Check if the system is ready */

  if (syslog_dev_outputready() < 0)
    {
      return;
    }

  /* Attempt to flush any buffered data */

  sched_lock();
  syslog_dev_flush();

  /* Close the detached file instance */

  g_syslog_dev.sl_state = SYSLOG_UNINITIALIZED;
  file_close(&g_syslog_dev.sl_file);

  /* Free the device path */

  if (g_syslog_dev.sl_devpath != NULL)
    {
      kmm_free(g_syslog_dev.sl_devpath);
    }

  /* Destroy the semaphore */

  nxsem_destroy(&g_syslog_dev.sl_sem);

  /* Reset the state structure */

  memset(&g_syslog_dev, 0, sizeof(struct syslog_dev_s));
  sched_unlock();
}
#endif /* CONFIG_SYSLOG_FILE */

/****************************************************************************
 * Name: syslog_dev_write
 *
 * Description:
 *   This is the low-level, multiple byte, system logging interface provided
 *   for the character driver interface.
 *
 * Input Parameters:
 *   buffer - The buffer containing the data to be output
 *   buflen - The number of bytes in the buffer
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller. A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

ssize_t syslog_dev_write(FAR const char *buffer, size_t buflen)
{
  FAR const char *endptr;
  ssize_t nwritten;
  size_t writelen;
  size_t remaining;
  int ret;

  /* Check if the system is ready to do output operations */

  ret = syslog_dev_outputready();
  if (ret < 0)
    {
      return ret;
    }

  /* The syslog device is ready for writing */

  ret = syslog_dev_takesem();
  if (ret < 0)
    {
      /* We probably already hold the semaphore and were probably
       * re-entered by the logic kicked off by file_write().
       * We might also have been interrupted by a signal.  Either
       * way, we are outta here.
       */

      return ret;
    }

  /* Loop until we have output all characters */

  for (endptr = buffer, remaining = buflen;
       remaining > 0;
       endptr++, remaining--)
    {
      /* Check for carriage return or line feed */

      if (*endptr == '\r' || *endptr == '\n')
        {
          /* Check for pre-formatted CR-LF sequence */

          if (remaining > 1 &&
              ((endptr[0] == '\r' && endptr[1] == '\n') ||
               (endptr[0] == '\n' && endptr[1] == '\r')))
            {
              /* Just skip over pre-formatted CR-LF or LF-CR sequence */

              endptr++;
              remaining--;
            }
          else
            {
              /* Write everything up to the position of the special
               * character.
               *
               * - buffer points to next byte to output.
               * - endptr points to the special character.
               */

              writelen = (size_t)((uintptr_t)endptr - (uintptr_t)buffer);
              if (writelen > 0)
                {
                  nwritten = file_write(&g_syslog_dev.sl_file,
                                        buffer, writelen);
                  if (nwritten < 0)
                    {
                      ret = (int)nwritten;
                      goto errout_with_sem;
                    }
                }

              /* Ignore the carriage return, but for the linefeed, output
               * both a carriage return and a linefeed.
               */

              if (*endptr == '\n')
                {
                  nwritten = file_write(&g_syslog_dev.sl_file, g_syscrlf, 2);
                  if (nwritten < 0)
                    {
                      ret = (int)nwritten;
                      goto errout_with_sem;
                    }
                }

              /* Adjust pointers */

               writelen++;         /* Skip the special character */
               buffer += writelen; /* Points past the special character */
            }
        }
    }

  /* Write any unterminated data at the end of the buffer.
   *
   * - buffer points to next byte to output.
   * - endptr points to the end of the buffer plus 1.
   */

  writelen = (size_t)((uintptr_t)endptr - (uintptr_t)buffer);
  if (writelen > 0)
    {
      nwritten = file_write(&g_syslog_dev.sl_file, buffer, writelen);
      if (nwritten < 0)
        {
          ret = (int)nwritten;
          goto errout_with_sem;
        }
    }

  syslog_dev_givesem();
  return buflen;

errout_with_sem:
  syslog_dev_givesem();
  return ret;
}

/****************************************************************************
 * Name: syslog_dev_putc
 *
 * Description:
 *   This is the low-level, single character, system logging interface
 *   provided for the character driver interface.
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller. A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int syslog_dev_putc(int ch)
{
  ssize_t nbytes;
  uint8_t uch;
  int ret;

  /* Check if the system is ready to do output operations */

  ret = syslog_dev_outputready();
  if (ret < 0)
    {
      return ret;
    }

  /* Ignore carriage returns */

  if (ch == '\r')
    {
      return ch;
    }

  /* The syslog device is ready for writing and we have something of
   * value to write.
   */

  ret = syslog_dev_takesem();
  if (ret < 0)
    {
      /* We probably already hold the semaphore and were probably
       * re-entered by the logic kicked off by file_write().
       * We might also have been interrupted by a signal.  Either
       * way, we are outta here.
       */

      return ret;
    }

  /* Pre-pend a newline with a carriage return. */

  if (ch == '\n')
    {
      /* Write the CR-LF sequence */

      nbytes = file_write(&g_syslog_dev.sl_file, g_syscrlf, 2);

      /* Synchronize the file when each CR-LF is encountered (i.e.,
       * implements line buffering always).
       */

#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (nbytes > 0)
        {
          syslog_dev_flush();
        }
#endif
    }
  else
    {
      /* Write the non-newline character (and don't flush) */

      uch = (uint8_t)ch;
      nbytes = file_write(&g_syslog_dev.sl_file, &uch, 1);
    }

  syslog_dev_givesem();

  /* Check if the write was successful.  If not, nbytes will be
   * a negated errno value.
   */

  if (nbytes < 0)
    {
      return (int)nbytes;
    }

  return ch;
}

/****************************************************************************
 * Name: syslog_dev_flush
 *
 * Description:
 *   Flush any buffer data in the file system to media.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int syslog_dev_flush(void)
{
#if defined(CONFIG_SYSLOG_FILE) && !defined(CONFIG_DISABLE_MOUNTPOINT)
  /* Ignore return value, always return success.  file_fsync() could fail
   * because the file is not open, the inode is not a mountpoint, or the
   * mountpoint does not support the sync() method.
   */

  file_fsync(&g_syslog_dev.sl_file);
#endif

  return OK;
}
