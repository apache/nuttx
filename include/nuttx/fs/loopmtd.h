/****************************************************************************
 * include/nuttx/fs/loopmtd.h
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

#ifndef __INCLUDE_NUTTX_FS_LOOPMTD_H
#define __INCLUDE_NUTTX_FS_LOOPMTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MTD_LOOP
/* Loop device IOCTL commands */

/* Command:      MTD_LOOPIOC_SETUP
 * Description:  Setup the loop device
 * Argument:     A pointer to a read-only instance of struct losetup_s.
 * Dependencies: The loop device must be enabled (CONFIG_MTD_LOOP=y)
 */

/* Command:      MTD_LOOPIOC_TEARDOWN
 * Description:  Teardown a loop device previously setup vis LOOPIOC_SETUP
 * Argument:     A read-able pointer to the path of the device to be
 *               torn down
 * Dependencies: The loop device must be enabled (CONFIG_MTD_LOOP=y)
 */

#define MTD_LOOPIOC_SETUP     _LOOPIOC(0x0001)
#define MTD_LOOPIOC_TEARDOWN  _LOOPIOC(0x0002)

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_MTD_LOOP
/* This is the structure referred to in the argument to the LOOPIOC_SETUP
 * IOCTL command.
 */

struct mtd_losetup_s
{
  FAR const char *devname;      /* The loop mtd device to be created */
  FAR const char *filename;     /* The file or character device to use */
  size_t          erasesize;    /* The erase size to use on the file */
  size_t          sectsize;     /* The sector / page size of the file */
  off_t           offset;       /* An offset that may be applied to the device */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

#ifdef CONFIG_MTD_LOOP
int mtd_loop_register(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_FS_LOOPMTD_H */
