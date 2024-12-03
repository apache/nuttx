/****************************************************************************
 * drivers/misc/dev_zero.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

static ssize_t devzero_readv(FAR struct file *filep,
                             FAR const struct uio *uio);
static ssize_t devzero_writev(FAR struct file *filep,
                              FAR const struct uio *uio);
static int     devzero_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_devzero_fops =
{
  NULL,           /* open */
  NULL,           /* close */
  NULL,           /* read */
  NULL,           /* write */
  NULL,           /* seek */
  NULL,           /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  devzero_poll,   /* poll */
  devzero_readv,  /* readv */
  devzero_writev  /* writev */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devzero_read
 ****************************************************************************/

static ssize_t devzero_readv(FAR struct file *filep,
                             FAR const struct uio *uio)
{
  ssize_t total =  uio_total_len(uio);
  FAR const struct iovec *iov = uio->uio_iov;
  int iovcnt = uio->uio_iovcnt;
  int i;

  UNUSED(filep);

  if (total < 0)
    {
      return total;
    }

  for (i = 0; i < iovcnt; i++)
    {
      memset(iov[i].iov_base, 0, iov[i].iov_len);
    }

  return total;
}

/****************************************************************************
 * Name: devzero_write
 ****************************************************************************/

static ssize_t devzero_writev(FAR struct file *filep,
                              FAR const struct uio *uio)
{
  UNUSED(filep);

  return uio_total_len(uio);
}

/****************************************************************************
 * Name: devzero_poll
 ****************************************************************************/

static int devzero_poll(FAR struct file *filep, FAR struct pollfd *fds,
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
 * Name: devzero_register
 *
 * Description:
 *   Register /dev/zero
 *
 ****************************************************************************/

void devzero_register(void)
{
  register_driver("/dev/zero", &g_devzero_fops, 0666, NULL);
}
