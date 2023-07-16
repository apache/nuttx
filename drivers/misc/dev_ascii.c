/****************************************************************************
 * drivers/misc/dev_ascii.c
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

#include <sys/types.h>
#include <poll.h>

#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRINTABLE_FIRST 0x20
#define PRINTABLE_COUNT (0x7f - 0x20)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t devascii_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t devascii_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static int devascii_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_devascii_fops =
{
  NULL,           /* open */
  NULL,           /* close */
  devascii_read,  /* read */
  devascii_write, /* write */
  NULL,           /* seek */
  NULL,           /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  devascii_poll   /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devascii_read
 ****************************************************************************/

static ssize_t devascii_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  size_t i;
  for (i = 0; i < len; i++)
    {
      buffer[i] = PRINTABLE_FIRST + (filep->f_pos + i) % PRINTABLE_COUNT;

      /* Replace the space character with a newline */

      if (buffer[i] == PRINTABLE_FIRST)
        {
          buffer[i] = '\n';
        }
    }

  filep->f_pos += len;
  return len;
}

/****************************************************************************
 * Name: devascii_write
 ****************************************************************************/

static ssize_t devascii_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  UNUSED(filep);
  UNUSED(buffer);

  return len;
}

/****************************************************************************
 * Name: devascii_poll
 ****************************************************************************/

static int devascii_poll(FAR struct file *filep, FAR struct pollfd *fds,
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
 * Name: devascii_register
 *
 * Description:
 *   Register /dev/ascii
 *
 ****************************************************************************/

void devascii_register(void)
{
  register_driver("/dev/ascii", &g_devascii_fops, 0666, NULL);
}
