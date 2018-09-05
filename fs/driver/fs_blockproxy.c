/****************************************************************************
 * fs/driver/fs_blockproxy.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/drivers/drivers.h>

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    !defined(CONFIG_DISABLE_PSEUDOFS_OPERATIONS)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_devno;
static sem_t g_devno_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unique_chardev
 *
 * Description:
 *   Create a unique temporary device name in the /dev/ directory of the
 *   pseudo-file system.  We cannot use mktemp for this because it will
 *   attempt to open() the file.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The allocated path to the device.  This must be released by the caller
 *   to prevent memory links.  NULL will be returned only the case where
 *   we fail to allocate memory.
 *
 ****************************************************************************/

static FAR char *unique_chardev(void)
{
  struct stat statbuf;
  char devbuf[16];
  uint32_t devno;
  int ret;

  /* Loop until we get a unique device name */

  for (; ; )
    {
      /* Get the semaphore protecting the path number */

      do
        {
          ret = nxsem_wait(&g_devno_sem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);

      /* Get the next device number and release the semaphore */

      devno = ++g_devno;
      nxsem_post(&g_devno_sem);

      /* Construct the full device number */

      devno &= 0xffffff;
      snprintf(devbuf, 16, "/dev/tmp%06lx", (unsigned long)devno);

      /* Make sure that file name is not in use */

      ret = stat(devbuf, &statbuf);
      if (ret < 0)
        {
          DEBUGASSERT(errno == ENOENT);
          return strdup(devbuf);
        }

      /* It is in use, try again */
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: block_proxy
 *
 * Description:
 *   Create a temporary char driver using drivers/bch to mediate character
 *   oriented accessed to the block driver.
 *
 * Input Parameters:
 *   blkdev - The path to the block driver
 *   oflags - Character driver open flags
 *
 * Returned Value:
 *   If positive, non-zero file descriptor is returned on success.  This
 *   is the file descriptor of the nameless character driver that mediates
 *   accesses to the block driver.
 *
 *   Errors that may be returned:
 *
 *     ENOMEM - Failed to create a temporay path name.
 *
 *   Plus:
 *
 *     - Errors reported from bchdev_register()
 *     - Errors reported from open() or unlink()
 *
 ****************************************************************************/

int block_proxy(FAR const char *blkdev, int oflags)
{
  FAR char *chardev;
  bool readonly;
  int ret;
  int fd;

  DEBUGASSERT(blkdev);

  /* Create a unique temporary file name for the character device */

  chardev = unique_chardev();
  if (chardev == NULL)
    {
      ferr("ERROR: Failed to create temporary device name\n");
      return -ENOMEM;
    }

  /* Should this character driver be read-only? */

  readonly = ((oflags & O_WROK) == 0);

  /* Wrap the block driver with an instance of the BCH driver */

  ret = bchdev_register(blkdev, chardev, readonly);
  if (ret < 0)
    {
      ferr("ERROR: bchdev_register(%s, %s) failed: %d\n",
           blkdev, chardev, ret);

      goto errout_with_chardev;
    }

  /* Open the newly created character driver */

  oflags &= ~(O_CREAT | O_EXCL | O_APPEND | O_TRUNC);
  fd = open(chardev, oflags);
  if (fd < 0)
    {
      ret = -errno;
      ferr("ERROR: Failed to open %s: %d\n", chardev, ret);
      goto errout_with_bchdev;
    }

  /* Unlink the character device name.  The driver instance will persist,
   * provided that CONFIG_DISABLE_PSEUDOFS_OPERATIONS=y (otherwise, we have
   * a problem here!)
   */

  ret = unlink(chardev);
  if (ret < 0)
    {
      ret = -errno;
      ferr("ERROR: Failed to unlink %s: %d\n", chardev, ret);
    }

  /* Free the allocate character driver name and return the open file
   * descriptor.
   */

  kmm_free(chardev);
  return fd;

errout_with_bchdev:
  (void)unlink(chardev);

errout_with_chardev:
  kmm_free(chardev);
  return ret;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && !CONFIG_DISABLE_PSEUDOFS_OPERATIONS */
