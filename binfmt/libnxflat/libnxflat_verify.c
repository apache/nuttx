/****************************************************************************
 * nxflat/lib/nxflat_stack.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <arpa/inet.h>
#include <nuttx/nxflat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define V_MAP   (load_info->vtbl->map)
#define V_UNMAP (load_info->vtbl->unmap)
#define V_ALLOC (load_info->vtbl->alloc)
#define V_FREE  (load_info->vtbl->free)
#define V_OPEN  (load_info->vtbl->open)
#define V_READ  (load_info->vtbl->read)
#define V_CLOSE (load_info->vtbl->close)

#define XFLT_HDR_SIZE   sizeof(struct nxflat_hdr_s)

#ifndef MAX
#  define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_verifyheader
 ****************************************************************************/

int nxflat_verifyheader(const struct nxflat_hdr_s *header)
{
  uint16 revision;

  if (!header)
    {
      dbg("NULL NXFLAT header!");
      return -ENOEXEC;
    }

  /* Check the FLT header -- magic number and revision.
   * 
   * If the the magic number does not match.  Just return
   * silently.  This is not our binary.
   */
  
  if (strncmp(header->h_magic, "NXFLAT", 4) != 0)
    {
      dbg("Unrecognized magic=\"%c%c%c%c\"",
	  header->h_magic[0], header->h_magic[1],
	  header->h_magic[2], header->h_magic[3]);
      return -ENOEXEC;
    }

  /* Complain a little more if the version does not match. */

  revision = ntohs(header->h_rev);
  if (revision != NXFLAT_VERSION_CURRENT)
    {
      dbg("Unsupported NXFLAT version=%d\n", revision);
      return -ENOEXEC;
    }
  return 0;
}

