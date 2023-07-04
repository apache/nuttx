/****************************************************************************
 * drivers/syslog/syslog_chardev.c
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
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/syslog/syslog.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_CHARDEV

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t syslog_chardev_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen);
#ifdef CONFIG_SYSLOG_IOCTL
static int syslog_chardev_ioctl(FAR struct file *filep,
                                int cmd, unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_syslog_fops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  NULL,                 /* read */
  syslog_chardev_write, /* write */
  NULL,                 /* seek */
#ifdef CONFIG_SYSLOG_IOCTL
  syslog_chardev_ioctl, /* ioctl */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_chardev_write
 ****************************************************************************/

static ssize_t syslog_chardev_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t len)
{
  syslog(LOG_INFO, "%.*s", (int)len, buffer);
  return len;
}

#ifdef CONFIG_SYSLOG_IOCTL
static int syslog_chardev_ioctl(FAR struct file *filep,
                                int cmd, unsigned long arg)
{
  FAR struct syslog_channel_info_s *info;
  FAR struct syslog_channel_s *channel = NULL;
  int i;

  if (arg == 0)
    {
      return -EINVAL;
    }

  if (cmd == SYSLOGIOC_GETCHANNELS)
    {
      info = (FAR struct syslog_channel_info_s *)arg;

      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          channel = g_syslog_channel[i];
          if (channel == NULL || channel->sc_name[0] == '\0')
            {
              break;
            }

          strlcpy(info[i].sc_name, channel->sc_name,
                  sizeof(info[i].sc_name));
          info[i].sc_disable = channel->sc_disable;
        }
    }
  else if (cmd == SYSLOGIOC_SETFILTER)
    {
      info = (FAR struct syslog_channel_info_s *)arg;

      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          if (strncmp(g_syslog_channel[i]->sc_name, info->sc_name,
                      sizeof(info->sc_name)) == 0)
            {
              channel = g_syslog_channel[i];
              break;
            }
        }

      if (channel == NULL)
        {
          return -ENOENT;
        }

      channel->sc_disable = info->sc_disable;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_register
 *
 * Description:
 *   Register a simple character driver at /dev/log whose write() method
 *   will transfer data to the SYSLOG device.  This can be useful if, for
 *   example, you want to redirect the output of a program to the SYSLOG.
 *
 *   NOTE that unlike other syslog output, this data is unformatted raw
 *   byte output with no time-stamping or any other SYSLOG features
 *   supported.
 *
 ****************************************************************************/

void syslog_register(void)
{
  register_driver("/dev/log", &g_syslog_fops, 0222, NULL);
}

#endif /* CONFIG_SYSLOG_CHARDEV */
