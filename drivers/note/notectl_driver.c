/****************************************************************************
 * drivers/note/notectl_driver.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/note/notectl_driver.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int notectl_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_notectl_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  NULL,          /* read */
  NULL,          /* write */
  NULL,          /* seek */
  notectl_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notectl_ioctl
 ****************************************************************************/

static int notectl_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOSYS;

  /* Handle the ioctl commands */

  switch (cmd)
    {
      /* NOTECTL_GETMODE
       *      - Get note filter mode
       *        Argument: A writable pointer to struct note_filter_mode_s
       */

      case NOTECTL_GETMODE:
        {
          struct note_filter_mode_s *mode = (struct note_filter_mode_s *)arg;

          if (mode == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              sched_note_filter_mode(mode, NULL);
              ret = OK;
            }
        }
        break;

      /* NOTECTL_SETMODE
       *      - Set note filter mode
       *        Argument: A read-only pointer to struct note_filter_mode_s
       */

      case NOTECTL_SETMODE:
        {
          struct note_filter_mode_s *mode = (struct note_filter_mode_s *)arg;

          if (mode == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              sched_note_filter_mode(NULL, mode);
              ret = OK;
            }
        }
        break;

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
      /* NOTECTL_GETSYSCALLFILTER
       *      - Get syscall filter setting
       *        Argument: A writable pointer to struct note_filter_syscall_s
       */

      case NOTECTL_GETSYSCALLFILTER:
        {
          struct note_filter_syscall_s *filter;
          filter = (struct note_filter_syscall_s *)arg;

          if (filter == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              sched_note_filter_syscall(filter, NULL);
              ret = OK;
            }
        }
        break;

      /* NOTECTL_SETSYSCALLFILTER
       *      - Set syscall filter setting
       *        Argument: A read-only pointer to struct note_filter_syscall_s
       */

      case NOTECTL_SETSYSCALLFILTER:
        {
          struct note_filter_syscall_s *filter;
          filter = (struct note_filter_syscall_s *)arg;

          if (filter == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              sched_note_filter_syscall(NULL, filter);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
      /* NOTECTL_GETIRQFILTER
       *      - Get IRQ filter setting
       *        Argument: A writable pointer to struct note_filter_irq_s
       */

      case NOTECTL_GETIRQFILTER:
        {
          struct note_filter_irq_s *filter;
          filter = (struct note_filter_irq_s *)arg;

          if (filter == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              sched_note_filter_irq(filter, NULL);
              ret = OK;
            }
        }
        break;

      /* NOTECTL_SETIRQFILTER
       *      - Set IRQ filter setting
       *        Argument: A read-only pointer to struct
       *                  note_filter_irq_s
       */

      case NOTECTL_SETIRQFILTER:
        {
          struct note_filter_irq_s *filter;
          filter = (struct note_filter_irq_s *)arg;

          if (filter == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              sched_note_filter_irq(NULL, filter);
              ret = OK;
            }
        }
        break;
#endif

      default:
          break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notectl_register
 *
 * Description:
 *   Register a driver at /dev/notectl that can be used by an application to
 *   control the note filter.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int notectl_register(void)
{
  return register_driver("/dev/notectl", &g_notectl_fops, 0666, NULL);
}
