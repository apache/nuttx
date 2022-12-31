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

#include <nuttx/fs/fs.h>
#include <syslog.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_CHARDEV

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t syslog_chardev_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations syslog_fops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  NULL,                 /* read */
  syslog_chardev_write, /* write */
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
  register_driver("/dev/log", &syslog_fops, 0222, NULL);
}

#endif /* CONFIG_SYSLOG_CHARDEV */
