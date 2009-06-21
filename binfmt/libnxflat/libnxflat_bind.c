/****************************************************************************
 * binfmt/libnxflat/libnxflat_bind.c
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

#include <nxflat.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>
#include <nuttx/nxflat.h>
#include <nuttx/symtab.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/***********************************************************************
 * Name: nxflat_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by 'symtab'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ***********************************************************************/

int nxflat_bind(FAR struct nxflat_loadinfo_s *loadinfo,
                FAR const struct symtab_s *exports, int nexports)
{
  FAR struct nxflat_import_s *imports;
  FAR struct nxflat_hdr_s    *hdr;
  FAR const struct symtab_s *symbol;

  char   *symname;
  uint32  offset;
  uint16  nimports;
  int     i;

  /* Get the ISpace load address of the module.  The NXFLAT header is the
   * first thing at the beginning of the ISpace.
   */

  hdr = (FAR struct nxflat_hdr_s*)loadinfo->ispace;

  /* From this, we can get the offset to the list of symbols imported by
   * this module and the number of symbols imported by this module.
   */

  offset   = ntohl(hdr->h_importsymbols);
  nimports = ntohs(hdr->h_importcount);

  /* Verify that this module requires imported symbols */

  if (offset != 0 && nimports > 0)
    {
      /* It does.. make sure that exported symbols are provided */

      DEBUGASSERT(symtab && nexports > 0);

      /* If non-zero, the value of the imported symbol list that we get
       * from the header is a file offset.  We will have to convert this
       * to an offset into the DSpace segment to get the pointer to the
       * beginning of the imported symbol list.
       */

      DEBUGASSERT(offset >= loadinfo->isize &&
                  offset < loadinfo->isize + loadinfo->dsize);

      imports = (struct nxflat_import_s*)
	(offset - loadinfo->isize + loadinfo->dspace);

      /* Now, search the list of imported symbols and attempt to bind
       * each symbol to the value exported by from the exported symbol
       * table.
       */

      for (i = 0; i < nimports; i++)
	{
	  /* Get a pointer to the imported symbol name.  The name itself
	   * lies in the TEXT segment.  But the reference to the name
	   * lies in DATA segment.  Therefore, the name reference should
	   * have been relocated when the module was loaded.
	   */

          offset = imports[i].i_funcname;
          DEBUGASSERT(offset < loadinfo->isize);

	  symname = (char*)(offset + loadinfo->ispace);

	  /* Find the exported symbol value for this this symbol name. */

#ifdef CONFIG_SYMTAB_ORDEREDBYNAME
          symbol = symtab_findorderedbyname(exports, symname, nexports);
#else
          symbol = symtab_findbyname(exports, symname, nexports);
#endif
	  if (!symbol)
	    {
	      bdbg("Exported symbol \"%s\" not found\n", symname);
              return -ENOENT;
	    }

	  /* And put this into the module's import structure. */

	  imports[i].i_funcaddress =  (uint32)symbol->sym_value;

	  bvdbg("Bound imported function '%s' to address %08x\n",
	        symname, imports[i].function_address);
	}
    }

  return OK;
}

