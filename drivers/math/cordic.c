/****************************************************************************
 * drivers/math/cordic.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include <nuttx/math/math_ioctl.h>
#include <nuttx/math/cordic.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct cordic_upperhalf_s
{
  FAR struct cordic_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t cordic_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t cordic_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     cordic_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cordicops =
{
  NULL,         /* open */
  NULL,         /* close */
  cordic_read,  /* read */
  cordic_write, /* write */
  NULL,         /* seek */
  cordic_ioctl, /* ioctl */
  NULL          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cordic_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t cordic_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: cordic_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t cordic_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: cordic_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the cordic timer
 *   work is done.
 *
 ****************************************************************************/

static int cordic_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode              *inode = filep->f_inode;
  FAR struct cordic_upperhalf_s *upper = NULL;
  FAR struct cordic_lowerhalf_s *lower = NULL;
  int                            ret   = 0;
  irqstate_t                     flags;

  _info("cmd: %d arg: %lu\n", cmd, arg);
  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  flags = enter_critical_section();

  switch (cmd)
    {
      /* CORDIC calulcate */

      case MATHIOC_CORDIC_CALC:
        {
          FAR struct cordic_calc_s *calc =
            (FAR struct cordic_calc_s *)((uintptr_t)arg);

          ret = lower->ops->calc(lower, calc);

          break;
        }

      /* Not supported */

      default:
        {
          ret = -ENOTTY;

          break;
        }
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cordic_register
 *
 * Description:
 *   Register a CORDIC driver.
 *
 ****************************************************************************/

int cordic_register(FAR const char *path,
                    FAR struct cordic_lowerhalf_s *lower)
{
  FAR struct cordic_upperhalf_s *upper = NULL;
  int                            ret   = OK;

  DEBUGASSERT(path);
  DEBUGASSERT(lower);

  /* Allocate the upper-half data structure */

  upper = (FAR struct cordic_upperhalf_s *)
    kmm_zalloc(sizeof(struct cordic_upperhalf_s));
  if (!upper)
    {
      _err("Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the CORDIC device structure */

  upper->lower = lower;

  /* Register the cordic timer device */

  ret = register_driver(path, &g_cordicops, 0666, upper);
  if (ret < 0)
    {
      _err("register_driver failed: %d\n", ret);
      kmm_free(upper);
    }

errout:
  return ret;
}
