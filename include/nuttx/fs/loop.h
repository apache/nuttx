/****************************************************************************
 * include/nuttx/fs/loop.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 *   Setup the loop device so that it exports the file referenced by 'filename'
 *   as a block device.
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
