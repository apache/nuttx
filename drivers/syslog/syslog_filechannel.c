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
#include <unistd.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/syslog/syslog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_FILE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPEN_FLAGS (O_WRONLY | O_CREAT | O_APPEND)
#define OPEN_MODE  (S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_FILE_SEPARATE
static void log_separate(FAR const char *log_file)
{
  struct file fp;

  if (file_open(&fp, log_file, (O_WRONLY | O_APPEND)) < 0)
    {
      return;
    }

  file_write(&fp, "\n\n", 2);

  file_close(&fp);
}
#endif

#if CONFIG_SYSLOG_FILE_ROTATIONS > 0
static void log_rotate(FAR const char *log_file)
{
  int i;
  off_t size;
  struct stat f_stat;
  size_t name_size;
  FAR char *rotate_to;
  FAR char *rotate_from;

  /* Get the size of the current log file. */

  if (stat(log_file, &f_stat) < 0)
    {
      return;
    }

  size = f_stat.st_size;

  /* If it does not exceed the limit we are OK. */

  if (size < CONFIG_SYSLOG_FILE_SIZE_LIMIT)
    {
      return;
    }

  /* Rotated file names. */

  name_size = strlen(log_file) + 8;
  rotate_to = kmm_malloc(name_size);
  rotate_from = kmm_malloc(name_size);
  if ((rotate_to == NULL) || (rotate_from == NULL))
    {
      goto end;
    }

  /* Rotate the logs. */

  for (i = (CONFIG_SYSLOG_FILE_ROTATIONS - 1); i > 0; i--)
    {
      snprintf(rotate_to, name_size, "%s.%d", log_file, i);
      snprintf(rotate_from, name_size, "%s.%d", log_file, i - 1);

      rename(rotate_from, rotate_to);
    }

  snprintf(rotate_to, name_size, "%s.0", log_file);

  rename(log_file, rotate_to);

end:

  kmm_free(rotate_to);
  kmm_free(rotate_from);
}
#endif

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
 *   A pointer to the new SYSLOG channel; NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct syslog_channel_s *syslog_file_channel(FAR const char *devpath)
{
  FAR struct syslog_channel_s *file_channel;

  /* Reset the default SYSLOG channel so that we can safely modify the
   * SYSLOG device.  This is an atomic operation and we should be safe
   * after the default channel has been selected.
   *
   * We disable pre-emption only so that we are not suspended and a lot of
   * important debug output is lost while we futz with the channels.
   */

  sched_lock();

  /* Rotate the log file, if needed. */

#if CONFIG_SYSLOG_FILE_ROTATIONS > 0
  log_rotate(devpath);
#endif

  /* Separate the old log entries. */

#ifdef CONFIG_SYSLOG_FILE_SEPARATE
  log_separate(devpath);
#endif

  /* Then initialize the file interface */

  file_channel = syslog_dev_initialize(devpath, OPEN_FLAGS, OPEN_MODE);
  if (file_channel == NULL)
    {
      goto errout_with_lock;
    }

  /* Use the file as the SYSLOG channel. If this fails we are pretty much
   * screwed.
   */

  if (syslog_channel(file_channel) != OK)
    {
      syslog_dev_uninitialize(file_channel);
      file_channel = NULL;
    }

errout_with_lock:
  sched_unlock();
  return file_channel;
}

#endif /* CONFIG_SYSLOG_FILE */
