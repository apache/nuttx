/****************************************************************************
 * include/nuttx/fs/loop.h
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

#ifndef __INCLUDE_NUTTX_FS_LOOP_H
#define __INCLUDE_NUTTX_FS_LOOP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEV_LOOP
/* Loop device IOCTL commands */

/* Command:      LOOPIOC_SETUP
 * Description:  Setup the loop device
 * Argument:     A pointer to a read-only instance of struct losetup_s.
 * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
 */

/* Command:      LOOPIOC_TEARDOWN
 * Description:  Teardown a loop device previously setup vis LOOPIOC_SETUP
 * Argument:     A read-able pointer to the path of the device to be
 *               torn down
 * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
 */

#define LOOPIOC_SETUP     _LOOPIOC(0x0001)
#define LOOPIOC_TEARDOWN  _LOOPIOC(0x0002)

#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

#ifdef CONFIG_DEV_LOOP
/* This is the structure referred to in the argument to the LOOPIOC_SETUP
 * IOCTL command.
 */

struct losetup_s
{
  FAR const char *devname;   /* The loop block device to be created */
  FAR const char *filename;  /* The file or character device to use */
  uint16_t sectsize;         /* The sector size to use with the block device */
  off_t offset;              /* An offset that may be applied to the device */
  bool readonly;             /* True: Read access will be supported only */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __KERNEL__
/* These are internal OS interface and are not available to applications */

/****************************************************************************
 * Name: loop_register
 *
 * Description:
 *   Register /dev/loop
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_LOOP
void loop_register(void);
#endif

/****************************************************************************
 * Name: losetup
 *
 * Description:
 *   Setup the loop device so that it exports the file referenced by
 *   'filename' as a block device.
 *
 ****************************************************************************/

int losetup(FAR const char *devname, FAR const char *filename,
            uint16_t sectsize, off_t offset, bool readonly);

/****************************************************************************
 * Name: loteardown
 *
 * Description:
 *   Undo the setup performed by losetup
 *
 ****************************************************************************/

int loteardown(FAR const char *devname);
#endif /* __KERNEL __ */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_FS_LOOP_H */
