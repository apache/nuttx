/****************************************************************************
 * binfmt/builtin.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include <sys/ioctl.h>

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/lib/builtin.h>

#ifdef CONFIG_FS_BINFS

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int builtin_loadbinary(FAR struct binary_s *binp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_builtin_binfmt =
{
  NULL,               /* next */
  builtin_loadbinary, /* load */
  NULL,               /* unload */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: builtin_loadbinary
 *
 * Description:
 *   Verify that the file is an builtin binary.
 *
 ****************************************************************************/

static int builtin_loadbinary(struct binary_s *binp)
{
  FAR const char *filename;
  FAR const struct builtin_s *builtin;
  int fd;
  int index;
  int ret;

  binfo("Loading file: %s\n", binp->filename);

  /* Open the binary file for reading (only) */

  fd = nx_open(binp->filename, O_RDONLY);
  if (fd < 0)
    {
      berr("ERROR: Failed to open binary %s: %d\n", binp->filename, fd);
      return fd;
    }

  /* If this file is a BINFS file system, then we can recover the name of
   * the file using the FIOC_FILENAME ioctl() call.
   */

  ret = ioctl(fd, FIOC_FILENAME, (unsigned long)((uintptr_t)&filename));
  if (ret < 0)
    {
      int errval = get_errno();
      berr("ERROR: FIOC_FILENAME ioctl failed: %d\n", errval);
      close(fd);
      return -errval;
    }

  /* Other file systems may also support FIOC_FILENAME, so the real proof
   * is that we can look up the index to this name in g_builtins[].
   */

  index = builtin_isavail(filename);
  if (index < 0)
    {
      berr("ERROR: %s is not a builtin application\n", filename);
      close(fd);
      return index;
    }

  /* Return the load information.  NOTE: that there is no way to configure
   * the priority.  That is a bug and needs to be fixed.
   */

  builtin         = builtin_for_index(index);
  binp->entrypt   = builtin->main;
  binp->stacksize = builtin->stacksize;
  binp->priority  = builtin->priority;
  close(fd);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: builtin_initialize
 *
 * Description:
 *   In order to use the builtin binary format, this function must be called
 *   during system initialize to register the builtin binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int builtin_initialize(void)
{
  int ret;

  /* Register ourselves as a binfmt loader */

  binfo("Registering Builtin Loader\n");

  ret = register_binfmt(&g_builtin_binfmt);
  if (ret != 0)
    {
      berr("Failed to register binfmt: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: builtin_uninitialize
 *
 * Description:
 *   Unregister the builtin binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void builtin_uninitialize(void)
{
  unregister_binfmt(&g_builtin_binfmt);
}

#endif /* CONFIG_FS_BINFS */
