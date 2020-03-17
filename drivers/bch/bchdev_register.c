/****************************************************************************
 * drivers/bch/bchdev_register.c
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

#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include "bch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bchdev_register
 *
 * Description:
 *   Setup so that it exports the block driver referenced by 'blkdev' as a
 *   character device 'chardev'
 *
 ****************************************************************************/

int bchdev_register(FAR const char *blkdev, FAR const char *chardev,
                    bool readonly)
{
  FAR void *handle;
  int ret;

  finfo("blkdev=\"%s\" chardev=\"%s\" readonly=%c\n",
        blkdev, chardev, readonly ? 'T' : 'F');

  /* Setup the BCH lib functions */

  ret = bchlib_setup(blkdev, readonly, &handle);
  if (ret < 0)
    {
      ferr("ERROR: bchlib_setup failed: %d\n", -ret);
      return ret;
    }

  /* Then setup the character device */

  ret = register_driver(chardev, &bch_fops, 0666, handle);
  if (ret < 0)
    {
      ferr("ERROR: register_driver failed: %d\n", -ret);
      bchlib_teardown(handle);
      handle = NULL;
    }

  return ret;
}
