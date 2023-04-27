/****************************************************************************
 * drivers/efuse/efuse.c
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
#include <nuttx/mutex.h>
#include <nuttx/efuse/efuse.h>

#ifdef CONFIG_EFUSE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct efuse_upperhalf_s
{
  mutex_t   lock;     /* Supports mutual exclusion */
  FAR char *path;     /* Registration path */

  /* The contained lower-half driver */

  FAR struct efuse_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t efuse_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t efuse_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     efuse_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_efuseops =
{
  NULL,        /* open */
  NULL,        /* close */
  efuse_read,  /* read */
  efuse_write, /* write */
  NULL,        /* seek */
  efuse_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efuse_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t efuse_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: efuse_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t efuse_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: efuse_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the efuse timer
 *   work is done.
 *
 ****************************************************************************/

static int efuse_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode             *inode = filep->f_inode;
  FAR struct efuse_upperhalf_s *upper;
  FAR struct efuse_lowerhalf_s *lower;
  int                           ret;

  minfo("cmd: %d arg: %lu\n", cmd, arg);
  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
    /* cmd:         EFUSEIOC_READ_FIELD
     * Description: Read a field
     * Argument:    A pointer to a struct efuse_param.
     *              Where the field is an array.
     *              Each item in the array is a pointer to an efuse_desc_t
     *              variable.
     *              An efuse_desc_t variable contains an offset to a field
     *              or subfield and its size in bits.
     *              The size param is a redundancy, and it is the sum
     *              of all subfields sizes from efuse_desc_t in bits.
     *              The data is a pointer to a pre-allocated space
     *              where the driver will load the data read from efuse.
     */

    case EFUSEIOC_READ_FIELD:
      {
        FAR struct efuse_param_s *param =
                   (FAR struct efuse_param_s *)((uintptr_t)arg);

        /* Read the efuse */

        DEBUGASSERT(lower->ops->read_field); /* Required */
        ret = lower->ops->read_field(lower,
                                     param->field,
                                     param->data,
                                     param->size);
      }
      break;

    /* cmd:         EFUSEIOC_WRITE_FIELD
     * Description: Write a field
     * Argument: A pointer to a struct efuse_param.
     *           Where the field is an array.
     *           Each item in the array is a pointer to an efuse_desc_t
     *           variable.
     *           An efuse_desc_t variable contains an offset to a field
     *           or subfield and its size in bits.
     *           The size param is a redundancy, and it is the sum
     *           of all subfields sizes from efuse_desc_t in bits.
     *           The data is a pointer to a pre-allocated space
     *           where the user wrote the value that he wants
     *           to write in a field or subfield.
     */

    case EFUSEIOC_WRITE_FIELD:
      {
        FAR struct efuse_param_s *param =
                   (FAR struct efuse_param_s *)((uintptr_t)arg);

        /* Write the efuse */

        DEBUGASSERT(lower->ops->write_field); /* Required */
        ret = lower->ops->write_field(lower,
                                      param->field,
                                      param->data,
                                      param->size);
      }
      break;

    default:
      {
        minfo("Forwarding unrecognized cmd: %d arg: %lu\n", cmd, arg);

        /* Ioctls commands that are not recognized by the "upper-half"
         * driver are forwarded to the lower half driver through this
         * method.
         */

        if (lower->ops->ioctl) /* Optional */
          {
            ret = lower->ops->ioctl(lower, cmd, arg);
          }
        else
          {
            ret = -ENOTTY;
          }
      }
      break;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efuse_register
 *
 * Description:
 *   This function binds an instance of a "lower half" efuse driver with
 *   the "upper half" efuse device and registers that device so that can
 *   be used by application code.
 *
 * Input Parameters:
 *   dev path - The full path to the driver to be registered in the NuttX
 *     pseudo-filesystem.  The recommended convention is to name all
 *     efuse drivers as "/dev/efuse".
 *   lower - A pointer to an instance of lower half efuse driver.  This
 *     instance is bound to the efuse driver and must persists as long as
 *     the driver persists.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned to the caller.  In the event
 *   of any failure, a NULL value is returned.
 *
 ****************************************************************************/

FAR void *efuse_register(FAR const char *path,
                         FAR struct efuse_lowerhalf_s *lower)
{
  FAR struct efuse_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(path && lower);
  minfo("Entry: path=%s\n", path);

  /* Allocate the upper-half data structure */

  upper = (FAR struct efuse_upperhalf_s *)
          kmm_zalloc(sizeof(struct efuse_upperhalf_s));
  if (!upper)
    {
      merr("Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the efuse timer device structure (it was already zeroed
   * by kmm_zalloc()).
   */

  nxmutex_init(&upper->lock);
  upper->lower = lower;

  /* Copy the registration path */

  upper->path = strdup(path);
  if (!upper->path)
    {
      merr("Path allocation failed\n");
      goto errout_with_upper;
    }

  /* Register the efuse timer device */

  ret = register_driver(path, &g_efuseops, 0666, upper);
  if (ret < 0)
    {
      merr("register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

  return (FAR void *)upper;

errout_with_path:
  kmm_free(upper->path);

errout_with_upper:
  nxmutex_destroy(&upper->lock);
  kmm_free(upper);

errout:
  return NULL;
}

/****************************************************************************
 * Name: efuse_unregister
 *
 * Description:
 *   This function can be called to disable and unregister the efuse
 *   device driver.
 *
 * Input Parameters:
 *   handle - This is the handle that was returned by efuse_register()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void efuse_unregister(FAR void *handle)
{
  FAR struct efuse_upperhalf_s *upper;
  FAR struct efuse_lowerhalf_s *lower;

  /* Recover the pointer to the upper-half driver state */

  upper = (FAR struct efuse_upperhalf_s *)handle;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  minfo("Unregistering: %s\n", upper->path);

  /* Unregister the efuse timer device */

  unregister_driver(upper->path);

  /* Then free all of the driver resources */

  kmm_free(upper->path);
  nxmutex_destroy(&upper->lock);
  kmm_free(upper);
}

#endif /* CONFIG_EFUSE */
