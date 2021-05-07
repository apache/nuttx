/****************************************************************************
 * drivers/syslog/syslog_filechannel.c
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

#include <sys/stat.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/syslog/syslog.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_FILE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPEN_FLAGS (O_WRONLY | O_CREAT | O_APPEND)
#define OPEN_MODE  (S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Handle to the SYSLOG channel */

FAR static struct syslog_channel_s *g_syslog_file_channel;

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
  int ret;

  /* Reset the default SYSLOG channel so that we can safely modify the
   * SYSLOG device.  This is an atomic operation and we should be safe
   * after the default channel has been selected.
   *
   * We disable pre-emption only so that we are not suspended and a lot of
   * important debug output is lost while we futz with the channels.
   */

  sched_lock();

  /* Uninitialize any driver interface that may have been in place */

  if (g_syslog_file_channel != NULL)
    {
      syslog_dev_uninitialize(g_syslog_file_channel);
    }

  /* Then initialize the file interface */

  g_syslog_file_channel = syslog_dev_initialize(devpath, OPEN_FLAGS,
                                                OPEN_MODE);
  if (g_syslog_file_channel == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Use the file as the SYSLOG channel. If this fails we are pretty much
   * screwed.
   */

  ret = syslog_channel(g_syslog_file_channel);

errout_with_lock:
  sched_unlock();
  return ret;
}

#endif /* CONFIG_SYSLOG_FILE */
