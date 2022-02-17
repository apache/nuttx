/****************************************************************************
 * binfmt/libnxflat/libnxflat_bind.c
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
#include <nuttx/compiler.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <nxflat.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/binfmt/nxflat.h>
#include <nuttx/binfmt/symtab.h>

#include "libnxflat.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_bindrel32i
 *
 * Description:
 *   Perform the NXFLAT_RELOC_TYPE_REL32I binding:
 *
 *   Meaning: Object file contains a 32-bit offset into I-Space at the
 *            offset.
 *   Fixup:   Add mapped I-Space address to the offset.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int nxflat_bindrel32i(FAR struct nxflat_loadinfo_s *loadinfo,
                                    uint32_t offset)
{
  FAR uint32_t *addr;

  binfo("NXFLAT_RELOC_TYPE_REL32I Offset: %08" PRIx32
        " I-Space: %" PRIxPTR "\n",
        offset, loadinfo->ispace + sizeof(struct nxflat_hdr_s));

  if (offset < loadinfo->dsize)
    {
      addr = (FAR uint32_t *)(offset + loadinfo->dspace->region);
      binfo("  Before: %08" PRIx32 "\n", *addr);
     *addr += (uint32_t)(loadinfo->ispace + sizeof(struct nxflat_hdr_s));
      binfo("  After: %08" PRIx32 "\n", *addr);
      return OK;
    }
  else
    {
      berr("Offset: %08" PRIx32 " does not lie in "
           "D-Space size: %08" PRIx32 "\n",
           offset, loadinfo->dsize);
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: nxflat_bindrel32d
 *
 * Description:
 *   Perform the NXFLAT_RELOC_TYPE_REL32D binding:
 *
 *   Meaning: Object file contains a 32-bit offset into D-Space at the
 *            offset.
 *   Fixup:   Add allocated D-Space address to the offset.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int nxflat_bindrel32d(FAR struct nxflat_loadinfo_s *loadinfo,
                                    uint32_t offset)
{
  FAR uint32_t *addr;

  binfo("NXFLAT_RELOC_TYPE_REL32D Offset: %08" PRIx32 " D-Space: %p\n",
        offset, loadinfo->dspace->region);

  if (offset < loadinfo->dsize)
    {
      addr = (FAR uint32_t *)(offset + loadinfo->dspace->region);
      binfo("  Before: %08" PRIx32 "\n", *addr);
     *addr += (uint32_t)(loadinfo->dspace->region);
      binfo("  After: %08" PRIx32 "\n", *addr);
      return OK;
    }
  else
    {
      berr("Offset: %08" PRIx32 " does not lie in "
           "D-Space size: %08" PRIx32 "\n",
           offset, loadinfo->dsize);
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: nxflat_bindrel32id
 *
 * Description:
 *   Perform the NXFLAT_RELOC_TYPE_REL32ID binding:
 *
 *   Meaning: Object file contains a 32-bit offset into I-Space at the offset
 *            that will unfortunately be references relative to the GOT
 *   Fixup:   Add allocated the mapped I-Space address MINUS the allocated
 *            D-Space address to the offset.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef NXFLAT_RELOC_TYPE_REL32ID
static inline int nxflat_bindrel32id(FAR struct nxflat_loadinfo_s *loadinfo,
                                     uint32_t offset)
{
  FAR uint32_t *addr;

  binfo("NXFLAT_RELOC_TYPE_REL32D Offset: %08x D-Space: %p\n",
        offset, loadinfo->dspace->region);

  if (offset < loadinfo->dsize)
    {
      addr  = (FAR uint32_t *)(offset + loadinfo->dspace->region);
      binfo("  Before: %08x\n", *addr);

     *addr += ((uint32_t)loadinfo->ispace -
               (uint32_t)(loadinfo->dspace->region));
      binfo("  After: %08x\n", *addr);

      return OK;
    }
  else
    {
      berr("Offset: %08 does not lie in D-Space size: %08x\n",
           offset, loadinfo->dsize);
      return -EINVAL;
    }
}
#endif

/****************************************************************************
 * Name: nxflat_gotrelocs
 *
 * Description:
 *   Bind all of the GOT relocations in the loaded module described by
 *   'loadinfo'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int nxflat_gotrelocs(FAR struct nxflat_loadinfo_s *loadinfo)
{
  FAR struct nxflat_reloc_s *relocs;
  FAR struct nxflat_reloc_s  reloc;
  FAR struct nxflat_hdr_s   *hdr;
  uint32_t offset;
  uint16_t nrelocs;
  int      ret;
  int      result;
  int      i;

  /* The NXFLAT header is the first thing at the beginning of the ISpace. */

  hdr = (FAR struct nxflat_hdr_s *)loadinfo->ispace;

  /* From this, we can get the offset to the list of relocation entries */

  offset  = NTOHL(hdr->h_relocstart);
  nrelocs = NTOHS(hdr->h_reloccount);
  binfo("offset: %08lx nrelocs: %d\n", (long)offset, nrelocs);

  /* The value of the relocation list that we get from the header is a
   * file offset.  We will have to convert this to an offset into the
   * DSpace segment to get the pointer to the beginning of the relocation
   * list.
   */

  DEBUGASSERT(offset >= loadinfo->isize);
  DEBUGASSERT(offset + nrelocs * sizeof(struct nxflat_reloc_s)
              <= (loadinfo->isize + loadinfo->dsize));

  relocs = (FAR struct nxflat_reloc_s *)
        (offset - loadinfo->isize + loadinfo->dspace->region);
  binfo("isize: %08lx dpsace: %p relocs: %p\n",
        (long)loadinfo->isize, loadinfo->dspace->region, relocs);

  /* All relocations are performed within the D-Space allocation.  If
   * CONFIG_ARCH_ADDRENV=y, then that D-Space allocation lies in an address
   * environment that may not be in place.  So, in that case, we must call
   * nxflat_addrenv_select to temporarily instantiate that address space
   * before the relocations can be performed.
   */

#ifdef CONFIG_ARCH_ADDRENV
  ret = nxflat_addrenv_select(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_select() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Now, traverse the relocation list of and bind each GOT relocation. */

  ret = OK; /* Assume success */
  for (i = 0; i < nrelocs; i++)
    {
      /* Handle the relocation by the relocation type */

      reloc  = *relocs++;
      result = OK;

      switch (NXFLAT_RELOC_TYPE(reloc.r_info))
        {
        /* NXFLAT_RELOC_TYPE_REL32I  Meaning: Object file contains a 32-bit
         *                                    offset into I-Space at the
         *                                    offset.
         *                           Fixup:   Add mapped I-Space address
         *                                    to the offset.
         */

        case NXFLAT_RELOC_TYPE_REL32I:
          {
            result = nxflat_bindrel32i(loadinfo,
                                       NXFLAT_RELOC_OFFSET(reloc.r_info));
          }
          break;

        /* NXFLAT_RELOC_TYPE_REL32D  Meaning: Object file contains a 32-bit
         *                                    offset into D-Space at the
         *                                    offset.
         *                           Fixup:   Add allocated D-Space address
         *                                    to the offset.
         */

        case NXFLAT_RELOC_TYPE_REL32D:
          {
            result = nxflat_bindrel32d(loadinfo,
                                       NXFLAT_RELOC_OFFSET(reloc.r_info));
          }
          break;

        /* NXFLAT_RELOC_TYPE_REL32ID Meaning: Object file contains a 32-bit
         *                                    offset into I-Space at the
         *                                    offset that will unfortunately
         *                                    be references relative to the
         *                                    GOT
         *                           Fixup:   Add allocated the mapped
         *                                    I-Space address MINUS the
         *                                    allocated D-Space address to
         *                                    the offset.
         */

#ifdef NXFLAT_RELOC_TYPE_REL32ID
        case NXFLAT_RELOC_TYPE_REL32ID:
          {
            result = nxflat_bindrel32id(loadinfo,
                                        NXFLAT_RELOC_OFFSET(reloc.r_info));
          }
          break;
#endif

        default:
          {
            berr("ERROR: Unrecognized relocation type: %" PRId32 "\n",
                 (uint32_t)NXFLAT_RELOC_TYPE(reloc.r_info));
            result = -EINVAL;
          }
          break;
        }

      /* Check for failures */

      if (result < 0 && ret == OK)
        {
          ret = result;
        }
    }

  /* Dump the relocation got */

#ifdef CONFIG_NXFLAT_DUMPBUFFER
  if (ret == OK && nrelocs > 0)
    {
      relocs = (FAR struct nxflat_reloc_s *)
        (offset - loadinfo->isize + loadinfo->dspace->region);
      nxflat_dumpbuffer("GOT", (FAR const uint8_t *)relocs,
                        nrelocs * sizeof(struct nxflat_reloc_s));
    }
#endif

  /* Restore the original address environment */

#ifdef CONFIG_ARCH_ADDRENV
  ret = nxflat_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_restore() failed: %d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nxflat_bindimports
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by 'symtab'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int nxflat_bindimports(FAR struct nxflat_loadinfo_s *loadinfo,
                                     FAR const struct symtab_s *exports,
                                     int nexports)
{
  FAR struct nxflat_import_s *imports;
  FAR struct nxflat_hdr_s    *hdr;
  FAR const struct symtab_s  *symbol;

  char    *symname;
  uint32_t offset;
  uint16_t nimports;
#ifdef CONFIG_ARCH_ADDRENV
  int      ret;
#endif
  int      i;

  /* The NXFLAT header is the first thing at the beginning of the ISpace. */

  hdr = (FAR struct nxflat_hdr_s *)loadinfo->ispace;

  /* From this, we can get the offset to the list of symbols imported by
   * this module and the number of symbols imported by this module.
   */

  offset   = NTOHL(hdr->h_importsymbols);
  nimports = NTOHS(hdr->h_importcount);
  binfo("Imports offset: %08" PRIx32 " nimports: %d\n", offset, nimports);

  /* The import[] table resides within the D-Space allocation.  If
   * CONFIG_ARCH_ADDRENV=y, then that D-Space allocation lies in an address
   * environment that may not be in place.  So, in that case, we must call
   * nxflat_addrenv_select to temporarily instantiate that address space
   * before the import[] table can be modified.
   */

#ifdef CONFIG_ARCH_ADDRENV
  ret = nxflat_addrenv_select(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_select() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Verify that this module requires imported symbols */

  if (offset != 0 && nimports > 0)
    {
      /* It does.. make sure that exported symbols are provided */

      DEBUGASSERT(exports && nexports > 0);

      /* If non-zero, the value of the imported symbol list that we get
       * from the header is a file offset.  We will have to convert this
       * to an offset into the DSpace segment to get the pointer to the
       * beginning of the imported symbol list.
       */

      DEBUGASSERT(offset >= loadinfo->isize &&
                  offset < loadinfo->isize + loadinfo->dsize);

      imports = (FAR struct nxflat_import_s *)
        (offset - loadinfo->isize + loadinfo->dspace->region);

      /* Now, traverse the list of imported symbols and attempt to bind
       * each symbol to the value exported by from the exported symbol
       * table.
       */

      for (i = 0; i < nimports; i++)
        {
          binfo("Import[%d] (%p) "
                "offset: %08" PRIx32 " func: %08" PRIx32 "\n",
                i, &imports[i], imports[i].i_funcname,
                imports[i].i_funcaddress);

          /* Get a pointer to the imported symbol name.  The name itself
           * lies in the TEXT segment.  But the reference to the name
           * lies in DATA segment.  Therefore, the name reference should
           * have been relocated when the module was loaded.
           */

          offset = imports[i].i_funcname;
          DEBUGASSERT(offset < loadinfo->isize);

          symname = (FAR char *)
            (offset + loadinfo->ispace + sizeof(struct nxflat_hdr_s));

          /* Find the exported symbol value for this symbol name. */

          symbol = symtab_findbyname(exports, symname, nexports);
          if (!symbol)
            {
              berr("Exported symbol \"%s\" not found\n", symname);
#ifdef CONFIG_ARCH_ADDRENV
              nxflat_addrenv_restore(loadinfo);
#endif
              return -ENOENT;
            }

          /* And put this into the module's import structure. */

          imports[i].i_funcaddress =  (uint32_t)symbol->sym_value;

          binfo("Bound import[%d] (%p) to export '%s' (%08" PRIx32 ")\n",
                i, &imports[i], symname, imports[i].i_funcaddress);
        }
    }

  /* Dump the relocation import table */

#ifdef CONFIG_NXFLAT_DUMPBUFFER
  if (nimports > 0)
    {
      nxflat_dumpbuffer("Imports", (FAR const uint8_t *)imports,
                        nimports * sizeof(struct nxflat_import_s));
    }
#endif

  /* Restore the original address environment */

#ifdef CONFIG_ARCH_ADDRENV
  ret = nxflat_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_restore() failed: %d\n", ret);
    }

  return ret;
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: nxflat_clearbss
 *
 * Description:
 *   Clear uninitialized .bss memory
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int nxflat_clearbss(FAR struct nxflat_loadinfo_s *loadinfo)
{
#ifdef CONFIG_ARCH_ADDRENV
  int ret;
#endif

  /* .bss resides within the D-Space allocation.  If CONFIG_ARCH_ADDRENV=y,
   * then that D-Space allocation lies in an address environment that may
   * not be in place.  So, in that case, we must call nxflat_addrenv_select
   * to temporarily instantiate that address space before the .bss can be
   * accessed.
   */

#ifdef CONFIG_ARCH_ADDRENV
  ret = nxflat_addrenv_select(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_select() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Zero the BSS area */

  memset((FAR void *)(loadinfo->dspace->region + loadinfo->datasize), 0,
         loadinfo->bsssize);

  /* Restore the original address environment */

#ifdef CONFIG_ARCH_ADDRENV
  ret = nxflat_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_restore() failed: %d\n", ret);
    }

  return ret;
#else
  return OK;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by 'symtab'.
 *   After binding the module, clear the BSS region (which held the
 *   relocation data) in preparation for execution.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_bind(FAR struct nxflat_loadinfo_s *loadinfo,
                FAR const struct symtab_s *exports, int nexports)
{
  /* Bind the imported symbol, absolute relocations separately.  This is done
   * before the standard relocations because that logic may modify the
   * import list (for the better hopefully, but we don't want to depend on
   * it).
   */

  int ret = nxflat_bindimports(loadinfo, exports, nexports);
  if (ret == OK)
    {
      /* Then bind all GOT relocations */

      ret = nxflat_gotrelocs(loadinfo);
      if (ret == OK)
        {
          /* Zero the BSS area, trashing the relocations that lived in that
           * space in the loaded file.
           */

          ret = nxflat_clearbss(loadinfo);
        }
    }

  return ret;
}
