/****************************************************************************
 * drivers/syslog/syslog_filechannel.c
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

#include <sys/stat.h>
#include <sched.h>
#include <fcntl.h>

#include <nuttx/syslog/syslog.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_FILE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPEN_FLAGS (O_WRONLY | O_CREAT | O_APPEND)
#define OPEN_MODE  (S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SYSLOG channel methods */

static int syslog_file_force(int ch);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the SYSLOG channel */

static const struct syslog_channel_s g_syslog_file_channel =
{
#ifdef CONFIG_SYSLOG_WRITE
  syslog_dev_write,
#endif
  syslog_dev_putc,
  syslog_file_force,
  syslog_dev_flush,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_file_force
 *
 * Description:
 *   A dummy FORCE method
 *
 ****************************************************************************/

static int syslog_file_force(int ch)
{
  return ch;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_file_channel
 *
 * Description:
 *   Configure to use a file in a mounted file system at 'devpath' as the
 *   SYSLOG channel.
 *
 *   This tiny function is simply a wrapper around syslog_dev_initialize()
 *   and syslog_channel().  It calls syslog_dev_initialize() to configure
 *   the character file at 'devpath then calls syslog_channel() to use that
 *   device as the SYSLOG output channel.
 *
 *   File SYSLOG channels differ from other SYSLOG channels in that they
 *   cannot be established until after fully booting and mounting the target
 *   file system.  This function would need to be called from board-specific
 *   bring-up logic AFTER mounting the file system containing 'devpath'.
 *
 *   SYSLOG data generated prior to calling syslog_file_channel will, of
 *   course, not be included in the file.
 *
 *   NOTE interrupt level SYSLOG output will be lost in this case unless
 *   the interrupt buffer is used.
 *
 * Input Parameters:
 *   devpath - The full path to the file to be used for SYSLOG output.
 *     This may be an existing file or not.  If the file exists,
 *     syslog_file_channel() will append new SYSLOG data to the end of the
 *     file.  If it does not, then syslog_file_channel() will create the
 *     file.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int syslog_file_channel(FAR const char *devpath)
{
  FAR const struct syslog_channel_s *saved_channel;
  int ret;

  /* Reset the default SYSLOG channel so that we can safely modify the
   * SYSLOG device.  This is an atomic operation and we should be safe
   * after the default channel has been selected.
   *
   * We disable pre-emption only so that we are not suspended and a lot of
   * important debug output is lost while we futz with the channels.
   */

  sched_lock();
  saved_channel = g_syslog_channel;
  ret = syslog_channel(&g_default_channel);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  /* Uninitialize any driver interface that may have been in place */

  ret = syslog_dev_uninitialize();
  if (ret < 0)
    {
      /* Nothing fatal has happened yet, we can restore the last channel
       * since it was not uninitialized (was it?)
       */

      (void)syslog_channel(saved_channel);
      goto errout_with_lock;
    }

  /* Then initialize the file interface */

  ret = syslog_dev_initialize(devpath, OPEN_FLAGS, OPEN_MODE);
  if (ret < 0)
    {
      /* We should still be able to back-up and re-initialized everything */

      (void)syslog_initialize(SYSLOG_INIT_EARLY);
      (void)syslog_initialize(SYSLOG_INIT_LATE);
      goto errout_with_lock;
    }

  /* Use the file as the SYSLOG channel. If this fails we are pretty much
   * screwed.
   */

  ret = syslog_channel(&g_syslog_file_channel);

errout_with_lock:
  sched_unlock();
  return ret;
}

#endif /* CONFIG_SYSLOG_FILE */
