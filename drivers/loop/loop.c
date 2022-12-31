/****************************************************************************
 * drivers/loop/loop.c
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
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/loop.h>

#ifdef CONFIG_DEV_LOOP

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t loop_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t loop_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     loop_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_loop_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  loop_read,     /* read */
  loop_write,    /* write */
  NULL,          /* seek */
  loop_ioctl,    /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: loop_read
 ****************************************************************************/

static ssize_t loop_read(FAR struct file *filep, FAR char *buffer,
                         size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: loop_write
 ****************************************************************************/

static ssize_t loop_write(FAR struct file *filep, FAR const char *buffer,
                          size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: loop_ioctl
 ****************************************************************************/

static int loop_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret;

  switch (cmd)
    {
    /* Command:      LOOPIOC_SETUP
     * Description:  Setup the loop device
     * Argument:     A pointer to a read-only instance of struct losetup_s.
     * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
     */

    case LOOPIOC_SETUP:
      {
         FAR struct losetup_s *setup =
           (FAR struct losetup_s *)((uintptr_t)arg);

        if (setup == NULL)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = losetup(setup->devname, setup->filename, setup->sectsize,
                          setup->offset, setup->readonly);
          }
      }
      break;

    /* Command:      LOOPIOC_TEARDOWN
     * Description:  Teardown a loop device previously setup via
                     LOOPIOC_SETUP
     * Argument:     A read-able pointer to the path of the device to be
     *               torn down
     * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
     */

    case LOOPIOC_TEARDOWN:
      {
        FAR const char *devname = (FAR const char *)((uintptr_t)arg);

        if (devname == NULL)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = loteardown(devname);
          }
       }
       break;

     default:
       ret = -ENOTTY;
       break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: loop_register
 *
 * Description:
 *   Register /dev/null
 *
 ****************************************************************************/

void loop_register(void)
{
  register_driver("/dev/loop", &g_loop_fops, 0666, NULL);
}

#endif /* CONFIG_DEV_LOOP */
