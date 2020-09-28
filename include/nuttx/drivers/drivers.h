/****************************************************************************
 * include/nuttx/fs/drivers.h
 *
 *   Copyright (C) 2007-2009, 2011-2013, 2015-2016 Gregory Nutt.
 *   All rights reserved.
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

#ifndef __INCLUDE_NUTTX_FS_DRIVERS_H
#define __INCLUDE_NUTTX_FS_DRIVERS_H

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
 * Name: bchlib_read
 *
 * Description:
 *   Read from the block device set-up by bchlib_setup as if it were a
 *   character device.
 *
 ****************************************************************************/

ssize_t bchlib_read(FAR void *handle, FAR char *buffer, size_t offset,
                    size_t len);

/****************************************************************************
 * Name: bchlib_write
 *
 * Description:
 *   Write to the block device set-up by bchlib_setup as if it were a
 *   character device.
 *
 ****************************************************************************/

ssize_t bchlib_write(FAR void *handle, FAR const char *buffer, size_t offset,
                     size_t len);

/****************************************************************************
 * Name: nx_pipe
 *
 * Description:
 *   nx_pipe() creates a pair of file descriptors, pointing to a pipe inode,
 *   and  places them in the array pointed to by 'fd'. fd[0] is for reading,
 *   fd[1] is for writing.
 *
 *   NOTE: nx_pipe is a special, non-standard, NuttX-only interface.  Since
 *   the NuttX FIFOs are based in in-memory, circular buffers, the ability
 *   to control the size of those buffers is critical for system tuning.
 *
 * Input Parameters:
 *   fd[2] - The user provided array in which to catch the pipe file
 *   descriptors
 *   bufsize - The size of the in-memory, circular buffer in bytes.
 *   flags - The file status flags.
 *
 * Returned Value:
 *   0 is returned on success; otherwise, the negative error code return
 *   appropriately.
 *
 ****************************************************************************/

#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0
int nx_pipe(int fd[2], size_t bufsize, int flags);
#endif

/****************************************************************************
 * Name: nx_mkfifo
 *
 * Description:
 *   nx_mkfifo() makes a FIFO device driver file with name 'pathname.' Unlike
 *   Linux, a NuttX FIFO is not a special file type but simply a device
 *   driver instance.  'mode' specifies the FIFO's permissions.
 *
 *   Once the FIFO has been created by nx_mkfifo(), any thread can open it
 *   for reading or writing, in the same way as an ordinary file. However, it
 *   must have been opened from both reading and writing before input or
 *   output can be performed.  This FIFO implementation will block all
 *   attempts to open a FIFO read-only until at least one thread has opened
 *   the FIFO for  writing.
 *
 *   If all threads that write to the FIFO have closed, subsequent calls to
 *   read() on the FIFO will return 0 (end-of-file).
 *
 *   NOTE: nx_mkfifo is a special, non-standard, NuttX-only interface.  Since
 *   the NuttX FIFOs are based in in-memory, circular buffers, the ability
 *   to control the size of those buffers is critical for system tuning.
 *
 * Input Parameters:
 *   pathname - The full path to the FIFO instance to attach to or to create
 *     (if not already created).
 *   mode - Ignored for now
 *   bufsize - The size of the in-memory, circular buffer in bytes.
 *
 * Returned Value:
 *   0 is returned on success; otherwise, the negative error code return
 *   appropriately.
 *
 ****************************************************************************/

#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0
int nx_mkfifo(FAR const char *pathname, mode_t mode, size_t bufsize);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_FS_DRIVERS_H */
