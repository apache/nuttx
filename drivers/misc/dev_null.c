/****************************************************************************
 * drivers/misc/dev_null.c
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

#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t devnull_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t devnull_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     devnull_poll(FAR struct file *filep, FAR struct pollfd *fds,
                 bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations devnull_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  devnull_read,  /* read */
  devnull_write, /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
  NULL,          /* truncate */
  NULL,          /* mmap */
  devnull_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devnull_read
 ****************************************************************************/

static ssize_t devnull_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  UNUSED(filep);
  UNUSED(buffer);
  UNUSED(len);

  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: devnull_write
 ****************************************************************************/

static ssize_t devnull_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  UNUSED(filep);
  UNUSED(buffer);

  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: devnull_poll
 ****************************************************************************/

static int devnull_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  UNUSED(filep);

  if (setup)
    {
      poll_notify(&fds, 1, POLLIN | POLLOUT);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devnull_register
 *
 * Description:
 *   Register /dev/null
 *
 ****************************************************************************/

void devnull_register(void)
{
  register_driver("/dev/null", &devnull_fops, 0666, NULL);
}
