/****************************************************************************
 * binfmt/pcode.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <fcntl.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/poff.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/pcode.h>

#ifdef CONFIG_PCODE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pcode_loadbinary(FAR struct binary_s *binp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_pcode_binfmt =
{
  NULL,              /* next */
  pcode_loadbinary,  /* load */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcode_proxy
 *
 * Description:
 *   This is the proxy program that runs and starts the P-Code interpreter.
 *
 ****************************************************************************/

#ifndef CONFIG_NUTTX_KERNEL
static int pcode_proxy(int argc, char **argv)
{
  /* REVISIT:  There are issues here when CONFIG_NUTTX_KERNEL is selected. */

  bdbg("ERROR: Not implemented");
  return EXIT_FAILURE;
}
#else
#  error Missing logic for the case of CONFIG_NUTTX_KERNEL
#endif

/****************************************************************************
 * Name: pcode_loadbinary
 *
 * Description:
 *   Verify that the file is an pcode binary.
 *
 ****************************************************************************/

static int pcode_loadbinary(struct binary_s *binp)
{
  FAR struct poff_fileheader_s hdr;
  FAR const struct pcode_s *b;
  FAR uint8_t *ptr;
  size_t remaining;
  ssize_t nread;
  int fd;
  int ret;

  bvdbg("Loading file: %s\n", binp->filename);

  /* Open the binary file for reading (only) */

  fd = open(binp->filename, O_RDONLY);
  if (fd < 0)
    {
      int errval = errno;
      bdbg("ERROR: Failed to open binary %s: %d\n", binp->filename, errval);
      return -errval;
    }

  /* Read the POFF file header */

  for (remaining = sizeof(struct poff_fileheader_s), ptr = (FAR uint8_t *)&hdr;
       remaining > 0; )
    {
      /* Read the next GULP */

      nread = read(fd, ptr, remaining);
      if (nread < 0)
        {
          /* If errno is EINTR, then this is not an error; the read() was
           * simply interrupted by a signal.
           */

          int errval = errno;
          DEBUGASSERT(errval > 0);

          if (errval != EINTR)
            {
              bdbg("ERROR: read failed: %d\n", errval);
              ret = -errval;
              goto errout_with_fd;
            }

          bdbg("Interrupted by a signal\n");
        }
      else
        {
          /* Set up for the next gulp */

          DEBUASSERT(nread > 0 && nread <=remaining);
          remaining -= nread;
          ptr += nread;
        } 
    }

#ifdef CONFIG_PCODE_DUMPBUFFER
  lib_dumpbuffer("POFF File Header", &hdr, sizeof(poff_fileheader_s));
#endif

  /* Verify that the file is a P-Code executable */

  if (memcmp(&hdr.fh_ident, FHI_POFF_MAG, 4) != 0 || hdr.fh_type != FHT_EXEC)
    {
      dbg("ERROR: File is not a P-code executable: %d\n");
      ret = -ENOEXEC;
      goto errout_with_fd;
    }

  /* Return the load information.
   * REVISIT:  There are issues here when CONFIG_NUTTX_KERNEL is selected.
   */

  binp->entrypt   = pcode_proxy;
  binp->stacksize = CONFIG_PCODE_STACKSIZE;
  binp->priority  = CONFIG_PCODE_PRIORITY;

  /* Successfully identified a p-code binary */

  ret = OK;

errout_with_fd:
  close(fd);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcode_initialize
 *
 * Description:
 *   P-code support is built based on the configuration.  However, in order
 *   to use this binary format, this function must be called during system
 *   initialization in order to register the P-Code binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int pcode_initialize(void)
{
  int ret;

  /* Register ourselves as a binfmt loader */

  bvdbg("Registering P-Code Loader\n");

  ret = register_binfmt(&g_pcode_binfmt);
  if (ret != 0)
    {
      bdbg("Failed to register binfmt: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: pcode_uninitialize
 *
 * Description:
 *   Unregister the pcode binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pcode_uninitialize(void)
{
  unregister_binfmt(&g_pcode_binfmt);
}

#endif /* CONFIG_PCODE */

