/****************************************************************************
 * include/nuttx/drivers/drivers.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_DRIVERS_H
#define __INCLUDE_NUTTX_DRIVERS_DRIVERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: drivers_early_initialize
 *
 * Description:
 *   Performs one-time, early driver initialization that doesn't rely on OS
 *   resources being ready.
 *
 ****************************************************************************/

void drivers_early_initialize(void);

/****************************************************************************
 * Name: drivers_initialize
 *
 * Description:
 *   Initialize chip and board independent general driver
 *
 ****************************************************************************/

void drivers_initialize(void);

/****************************************************************************
 * Name: devnull_register
 *
 * Description:
 *   Register /dev/null
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void devnull_register(void);

/****************************************************************************
 * Name: devascii_register
 *
 * Description:
 *   Register /dev/ascii
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_ASCII
void devascii_register(void);
#endif

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void);
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM
void devurandom_register(void);
#endif

/****************************************************************************
 * Name: devcrypto_register
 *
 * Description:
 *   Register /dev/crypto
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void devcrypto_register(void);

/****************************************************************************
 * Name: devmem_register
 *
 * Description:
 *   Register devmem driver
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_MEM
int devmem_register(void);
#endif

/****************************************************************************
 * Name: devzero_register
 *
 * Description:
 *   Register /dev/zero
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void devzero_register(void);

/****************************************************************************
 * Name: bchdev_register
 *
 * Description:
 *   Setup so that it exports the block driver referenced by 'blkdev' as a
 *   character device 'chardev'
 *
 ****************************************************************************/

int bchdev_register(FAR const char *blkdev, FAR const char *chardev,
                    bool readonly);

/****************************************************************************
 * Name: bchdev_unregister
 *
 * Description:
 *   Unregister character driver access to a block device that was created
 *   by a previous call to bchdev_register().
 *
 ****************************************************************************/

int bchdev_unregister(FAR const char *chardev);

/* Low level, direct access. NOTE: low-level access and character driver
 * access are incompatible. One and only one access method should be
 * implemented.
 */

/****************************************************************************
 * Name: bchlib_setup
 *
 * Description:
 *   Setup so that the block driver referenced by 'blkdev' can be accessed
 *   similar to a character device.
 *
 ****************************************************************************/

int bchlib_setup(FAR const char *blkdev, bool readonly, FAR void **handle);

/****************************************************************************
 * Name: bchlib_teardown
 *
 * Description:
 *   Setup so that the block driver referenced by 'blkdev' can be accessed
 *   similar to a character device.
 *
 ****************************************************************************/

int bchlib_teardown(FAR void *handle);

/****************************************************************************
 * Name: bchlib_open
 *
 * Description:
 *   Opens a block device and increments the reference count for tracking
 *   the number of active users. Ensures no more than the maximum allowed
 *   open count (`MAX_OPENCNT`) is exceeded.
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *
 * Return:
 *   On success, returns `OK`.
 *   On failure, returns a negative error code such as:
 *     -EMFILE: Maximum open count exceeded.
 *     Other error codes may reflect mutex lock issues.
 ****************************************************************************/

int bchlib_open(FAR void *handle);

/****************************************************************************
 * Name: bchlib_close
 *
 * Description:
 *   Closes the block device by decrementing the reference count and
 *   performing cleanup operations like flushing dirty pages in the cache.
 *   If all references are released and the device is marked as unlinked,
 *   it tears down the BCH device.
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *
 * Return:
 *   On success, returns `OK`.
 *   On failure, returns a negative error code such as:
 *     -EIO: Reference count is already zero.
 *     Other error codes may reflect mutex lock issues.
 ****************************************************************************/

int bchlib_close(FAR void *handle);

/****************************************************************************
 * Name: bchlib_read
 *
 * Description:
 *   Read from the block device set-up by bchlib_setup as if it were a
 *   character device.
 *
 ****************************************************************************/

ssize_t bchlib_read(FAR void *handle, FAR char *buffer, off_t offset,
                    size_t len);

/****************************************************************************
 * Name: bchlib_write
 *
 * Description:
 *   Write to the block device set-up by bchlib_setup as if it were a
 *   character device.
 *
 ****************************************************************************/

ssize_t bchlib_write(FAR void *handle, FAR const char *buffer, off_t offset,
                     size_t len);

/****************************************************************************
 * Name: bchlib_ioctl
 *
 * Description:
 *   Handles I/O control requests (IOCTLs) for the block device. Implements
 *   device-specific commands like retrieving private data, flushing the
 *   cache, or setting encryption keys (if encryption is enabled).
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *   cmd    - Command indicating the requested operation.
 *   arg    - Argument for the command.
 *
 * Return:
 *   On success, returns `OK` or a command-specific result value.
 *   On failure, returns a negative error code:
 *     -ENOTTY: Command not supported.
 *     -EINVAL: Invalid argument.
 *     Other error codes may reflect underlying implementation issues.
 ****************************************************************************/

int bchlib_ioctl(FAR void *handle, int cmd, unsigned long arg);

/****************************************************************************
 * Name: bchlib_seek
 *
 * Description:
 *   Adjusts the current position within the block device based on the
 *   provided offset and the specified seek mode (whence).
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *   offset - The offset value used to calculate the new position.
 *   whence - Specifies the base position for seeking. It can take one of
 *            the following values:
 *            - SEEK_SET: Seek from the start of the device.
 *            - SEEK_CUR: Seek from the current position.
 *            - SEEK_END: Seek from the end of the device.
 *   pos    - Pointer to the position value that will be updated to reflect
 *            the new position after seeking.
 *
 * Return:
 *   On success, returns the new position within the block device.
 *   On failure, returns a negative error code, such as:
 *     -EINVAL: Invalid value for `whence` or if the resulting position is
 *              negative.
 *     Other error codes may reflect mutex lock issues.
 ****************************************************************************/

off_t bchlib_seek(FAR void *handle, off_t offset, int whence, off_t *pos);

/****************************************************************************
 * Name: bchlib_unlink
 *
 * Description:
 *   Marks the block device as unlinked (no longer actively accessible). If
 *   there are no remaining open references, tears down the BCH device.
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *
 * Return:
 *   On success, returns `OK`.
 *   On failure, returns a negative error code indicating issues like mutex
 *   lock errors.
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
int bchlib_unlink(FAR void *handle);
#endif

/****************************************************************************
 * Name: lwlconsole_init
 *
 * Description:
 *   Register /dev/console
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lwlconsole_init(void);

/****************************************************************************
 * Name: rpmsg_serialinit
 *
 * Description:
 *   Register rpmsg serial driver
 *
 ****************************************************************************/

void rpmsg_serialinit(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_DRIVERS_H */
