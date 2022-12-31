/****************************************************************************
 * drivers/syslog/syslog_console.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/syslog/syslog.h>

#include "syslog.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t syslog_console_read(FAR struct file *filep, FAR char *buffer,
                                   size_t buflen);
static ssize_t syslog_console_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_consoleops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  syslog_console_read,  /* read */
  syslog_console_write, /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_console_read
 ****************************************************************************/

static ssize_t syslog_console_read(FAR struct file *filep, FAR char *buffer,
                                   size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: syslog_console_write
 ****************************************************************************/

static ssize_t syslog_console_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen)
{
  return syslog_write(buffer, buflen);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_console_init
 ****************************************************************************/

void syslog_console_init(void)
{
  register_driver("/dev/console", &g_consoleops, 0666, NULL);
}
