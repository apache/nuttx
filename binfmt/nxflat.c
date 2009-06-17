/****************************************************************************
 * binfmt/nxflat.c
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
#include <nxflat.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/binfmt.h>
#include <nuttx/nxflat.h>

#ifdef CONFIG_NXFLAT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nxflat_loadbinary(struct binary_s *binp);
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT)
static void nxflat_dumpmemory(void *addr, int nbytes);
static void nxflat_dumploadinfo(struct nxflat_loadinfo_s *loadinfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_nxflatbinfmt =
{
  NULL,                /* next */
  nxflat_loadbinary,   /* load */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nnxflat_dumpmemory
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT)
static void nxflat_dumpmemory(void *addr, int nbytes)
{
  ubyte *ptr;

  bdbg("  ADDRESS    VALUE\n");
  for (ptr = (ubyte*)addr; nbytes > 0; ptr += 4, nbytes -= 4)
    {
      bdbg("  %p: %02x %02x %02x %02x\n", ptr, ptr[0], ptr[1], ptr[2], ptr[3]);
    }
}
#else /* CONFIG_XFLAT_DEBUG */
# define nnxflat_dumpmemory(a,n)
#endif /* CONFIG_XFLAT_DEBUG */

/****************************************************************************
 * Name: nxflat_dumploadinfo
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT)
static void nxflat_dumploadinfo(struct nxflat_loadinfo_s *loadinfo)
{
  unsigned long dsize = loadinfo->datasize + loadinfo->bsssize;

  bdbg("LOAD_INFO:\n");
  bdbg("  ISPACE:\n");
  bdbg("    ispace:       %08lx\n", loadinfo->ispace);
  bdbg("    entryoffs:    %08lx\n", loadinfo->entryoffs);
  bdbg("    isize:        %08lx\n", loadinfo->isize);

  bdbg("  DSPACE:\n");
  bdbg("    dspace:       %08lx\n", loadinfo->dspace);
  bdbg("    datasize:     %08lx\n", loadinfo->datasize);
  bdbg("    bsssize:      %08lx\n", loadinfo->bsssize);
  bdbg("      (pad):      %08lx\n", loadinfo->dsize - dsize);
  bdbg("    stacksize:    %08lx\n", loadinfo->stacksize);
  bdbg("    dsize:        %08lx\n", loadinfo->dsize);

  bdbg("  RELOCS:\n");
  bdbg("    relocstart:   %08lx\n", loadinfo->relocstart);
  bdbg("    reloccount:   %08lx\n", loadinfo->reloccount);

  bdbg("  HANDLES:\n");
  bdbg("    filfd:        %d\n",    loadinfo->filfd);

  bdbg("  NXFLT HEADER:");
  bdbg("    header:       %p\n",    loadinfo->header);
}
#else /* CONFIG_XFLAT_DEBUG */
# define nxflat_dumploadinfo(i)
#endif /* CONFIG_XFLAT_DEBUG */

/****************************************************************************
 * Name: nxflat_loadbinary
 *
 * Description:
 *   Verify that the file is an NXFLAT binary and, if so, load the NXFLAT
 *   binary into memory
 *
 ****************************************************************************/

static int nxflat_loadbinary(struct binary_s *binp)
{
  struct nxflat_hdr_s      header;    /* Just allocated memory */
  struct nxflat_loadinfo_s loadinfo;  /* Contains globals for libnxflat */
  int                      ret;

  bvdbg("Loading file: %s\n", binp->filename);

  /* Initialize the xflat library to load the program binary. */

  ret = nxflat_init(binp->filename, &header, &loadinfo);
  nxflat_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      bdbg("Failed to initialize for load of NXFLT program: %d\n", ret);
      return ret;
    }

  /* Load the program binary */

  ret = nxflat_load(&loadinfo);
  nxflat_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      bdbg("Failed to load NXFLT program binary: %d\n", ret);
      nxflat_uninit(&loadinfo);
      return ret;
    }

  /* Return the load information */

  binp->entrypt   = (main_t)(loadinfo.ispace + loadinfo.entryoffs);
  binp->ispace    = (void*)loadinfo.ispace;
  binp->dspace    = (void*)loadinfo.dspace;
  binp->isize     = loadinfo.isize;
  binp->stacksize = loadinfo.stacksize;

  bvdbg("ENTRY CODE:\n");
  nxflat_dumpmemory(binp->entrypt, 16*sizeof(uint32));
  nxflat_uninit(&loadinfo);
  return OK;
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************
 * Name: nxflat_initialize
 *
 * Description:
 *   NXFLAT support is built unconditionally.  However, it order to
 *   use this binary format, this function must be called during system
 *   format in order to register the NXFLAT binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ***********************************************************************/

int nxflat_initialize(void)
{
  int ret;

  /* Register ourselves as a binfmt loader */

  bvdbg("Registering NXFLAT\n");
  ret = register_binfmt(&g_nxflatbinfmt);
  if (ret != 0)
    {
      bdbg("Failed to register binfmt: %d\n", ret);
    }
  return ret;
}

/****************************************************************************
 * Name: nxflat_uninitialize
 *
 * Description:
 *   Unregister the NXFLAT binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxflat_uninitialize(void)
{
  unregister_binfmt(&g_nxflatbinfmt);
}

#endif /* CONFIG_NXFLAT */

