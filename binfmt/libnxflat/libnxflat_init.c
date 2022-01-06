/****************************************************************************
 * binfmt/libnxflat/libnxflat_init.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/stat.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <nxflat.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/fs/fs.h>
#include <nuttx/binfmt/nxflat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO, and CONFIG_DEBUG_BINFMT have to
 * be defined or CONFIG_NXFLAT_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_INFO) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_NXFLAT_DUMPBUFFER
#endif

#ifdef CONFIG_NXFLAT_DUMPBUFFER
# define nxflat_dumpbuffer(m,b,n) binfodumpbuffer(m,b,n)
#else
# define nxflat_dumpbuffer(m,b,n)
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
 * Name: nxflat_init
 *
 * Description:
 *   This function is called to configure the library to process an NXFLAT
 *   program binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_init(const char *filename, struct nxflat_loadinfo_s *loadinfo)
{
  uint32_t datastart;
  uint32_t dataend;
  uint32_t bssend;
  int      ret;

  binfo("filename: %s loadinfo: %p\n", filename, loadinfo);

  /* Clear the load info structure */

  memset(loadinfo, 0, sizeof(struct nxflat_loadinfo_s));

  /* Open the binary file */

  ret = file_open(&loadinfo->file, filename, O_RDONLY);
  if (ret < 0)
    {
      berr("ERROR: Failed to open NXFLAT binary %s: %d\n", filename, ret);
      return ret;
    }

  /* Read the NXFLAT header from offset 0 */

  ret = nxflat_read(loadinfo, (FAR char *)&loadinfo->header,
                    sizeof(struct nxflat_hdr_s), 0);
  if (ret < 0)
    {
      berr("ERROR: Failed to read NXFLAT header: %d\n", ret);
      file_close(&loadinfo->file);
      return ret;
    }

  nxflat_dumpbuffer("NXFLAT header", (FAR const uint8_t *)&loadinfo->header,
                    sizeof(struct nxflat_hdr_s));

  /* Verify the NXFLAT header */

  if (nxflat_verifyheader(&loadinfo->header) != 0)
    {
      /* This is not an error because we will be called to attempt loading
       * EVERY binary.  Returning -ENOEXEC simply informs the system that
       * the file is not an NXFLAT file.  Besides, if there is something
       * worth complaining about, nnxflat_verifyheader() has already
       * done so.
       */

      berr("ERROR: Bad NXFLAT header\n");
      file_close(&loadinfo->file);
      return -ENOEXEC;
    }

  /* Save all of the input values in the loadinfo structure
   * and extract some additional information from the xflat
   * header.  Note that the information in the xflat header is in
   * network order.
   */

  datastart             = ntohl(loadinfo->header.h_datastart);
  dataend               = ntohl(loadinfo->header.h_dataend);
  bssend                = ntohl(loadinfo->header.h_bssend);

  /* And put this information into the loadinfo structure as well.
   *
   * Note that:
   *
   *   isize       = the address range from 0 up to datastart.
   *   datasize   = the address range from datastart up to dataend
   *   bsssize    = the address range from dataend up to bssend.
   */

  loadinfo->entryoffs   = ntohl(loadinfo->header.h_entry);
  loadinfo->isize       = datastart;

  loadinfo->datasize    = dataend - datastart;
  loadinfo->bsssize     = bssend - dataend;
  loadinfo->stacksize   = ntohl(loadinfo->header.h_stacksize);

  /* This is the initial dspace size.  We'll re-calculate this later
   * after the memory has been allocated.
   */

  loadinfo->dsize       = bssend - datastart;

  /* Get the offset to the start of the relocations (we'll relocate
   * this later).
   */

  loadinfo->relocstart  = ntohl(loadinfo->header.h_relocstart);
  loadinfo->reloccount  = ntohs(loadinfo->header.h_reloccount);

  return 0;
}
