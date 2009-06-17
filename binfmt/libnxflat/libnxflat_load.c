/****************************************************************************
 * binfmt/libnxflat/libnxflat_load.c
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
#include <sys/mman.h>

#include <stdlib.h>
#include <string.h>
#include <nxflat.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/nxflat.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef MAX
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

#if defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_BINFMT)
static const char g_textsegment[] = "TEXT";
static const char g_datasegment[] = "DATA";
static const char g_bsssegment[]  = "BSS";
static const char g_unksegment[]  = "UNKNOWN";

static const char *g_segment[] =
{
  g_textsegment,
  g_datasegment,
  g_bsssegment,
  g_unksegment
};

#  define SEGNAME(rl) g_segment[NXFLAT_RELOC_TYPE(rl)]
#else
#  define SEGNAME(rl) "(no name)"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_reloc
 ****************************************************************************/

static void nxflat_reloc(struct nxflat_loadinfo_s *loadinfo, uint32 rl)
{
  uint32 *ptr;
  uint32 datastart;

  /* We only support relocations in the data sections. Verify that the
   * relocation address lies in the data section of the file image.
   */

  if (NXFLAT_RELOC_OFFSET(rl) > loadinfo->datasize)
    {
      bdbg("ERROR: Relocation at 0x%08x invalid -- "
	  "does not lie in the data segment, size=0x%08x\n",
	  NXFLAT_RELOC_OFFSET(rl), loadinfo->datasize);
      bdbg("       Relocation not performed!\n");
    }
  else if ((NXFLAT_RELOC_OFFSET(rl) & 0x00000003) != 0)
    {
      bdbg("ERROR: Relocation at 0x%08x invalid -- "
	  "Improperly aligned\n",
	  NXFLAT_RELOC_OFFSET(rl));
    }
  else
    {
      /* Get a reference to the "real" start of data.  It is
       * offset slightly from the beginning of the allocated
       * DSpace to hold information needed by ld.so at run time.
       */

      datastart = (uint32)loadinfo->dspace->region;

      /* Get a pointer to the value that needs relocation in
       * DSpace.
       */
      
      ptr = (uint32*)(datastart + NXFLAT_RELOC_OFFSET(rl));

      bvdbg("Relocation of variable at DATASEG+0x%08x "
	  "(address 0x%p, currently 0x%08x) into segment %s\n",
	  NXFLAT_RELOC_OFFSET(rl), ptr, *ptr, SEGNAME(rl));
	
      switch (NXFLAT_RELOC_TYPE(rl))
	{
	  /* TEXT is located at an offset of sizeof(struct nxflat_hdr_s) from
	   * the allocated/mapped ISpace region.
	   */

	case NXFLAT_RELOC_TYPE_TEXT:
	  *ptr += loadinfo->ispace + sizeof(struct nxflat_hdr_s);
	  break;

	  /* DATA and BSS are always contiguous regions.  DATA
	   * begins at the beginning of the allocated data segment.
	   * BSS is positioned after DATA, unrelocated references
	   * to BSS include the data offset.
	   *
	   * In other contexts, is it necessary to add the datasize
	   * to get the BSS offset like:
	   *
	   *   *ptr += datastart + loadinfo->datasize;
	   */

	case NXFLAT_RELOC_TYPE_DATA:
	case NXFLAT_RELOC_TYPE_BSS:
	  *ptr += datastart;
	  break;

	  /* This case happens normally if the symbol is a weak
	   * undefined symbol.  We permit these.
	   */

	case NXFLAT_RELOC_TYPE_NONE:
	  bdbg("NULL relocation!\n");
	  break;

	default:
	  bdbg("ERROR: Unknown relocation type=%d\n", NXFLAT_RELOC_TYPE(rl));
	  break;
	}

      bvdbg("Relocation became 0x%08x\n", *ptr);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_load
 ****************************************************************************/

int nxflat_load(struct nxflat_loadinfo_s *loadinfo)
{
  off_t   doffset;     /* Offset to .data in the NXFLAT file */
  uint32 *reloctab;    /* Address of the relocation table */
  uint32  dreadsize;   /* Total number of bytes of .data to be read */
  uint32  ret = OK;
  int     i;

  /* Calculate the extra space we need to allocate.  This extra space will be
   * the size of the BSS section.  This extra space will also be used
   * temporarily to hold relocation information.  So the allocated size of this
   * region will either be the size of .data + size of.bss section OR, the
   * size of .data + the relocation entries, whichever is larger
   */

  {
    uint32 relocsize;
    uint32 extrasize;

    /* This is the amount of memory that we have to have to hold the
     * relocations.
     */

    relocsize  = loadinfo->reloccount * sizeof(uint32);

    /* In the file, the relocations should lie at the same offset as BSS.
     * The additional amount that we allocate have to be either (1) the
     * BSS size, or (2) the size of the relocation records, whicher is
     * larger.
     */

    extrasize = MAX(loadinfo->bsssize, relocsize);

    /* Use this addtional amount to adjust the total size of the dspace
     * region.
     */

    loadinfo->dsize = loadinfo->datasize + extrasize;

    /* The number of bytes of data that we have to read from the file is
     * the data size plus the size of the relocation table.
     */

    dreadsize = loadinfo->datasize + relocsize;
  }

  /* We'll need this a few times as well. */

  doffset = loadinfo->isize;

  /* We will make two mmap calls create an address space for the executable.
   * We will attempt to map the file to get the ISpace address space and
   * to allocate RAM to get the DSpace address space.  If the filesystem does
   * not support file mapping, the map() implementation should do the
   * right thing.
   */

  /* The following call will give as a pointer to the mapped file ISpace.
   * This may be in ROM, RAM, Flash, ... We don't really care where the memory
   * resides as long as it is fully initialized and ready to execute.
   */

  loadinfo->ispace = (uint32)mmap(NULL, loadinfo->isize, PROT_READ,
                                  MAP_SHARED|MAP_FILE, loadinfo->filfd, 0);
  if (loadinfo->ispace == (uint32)MAP_FAILED)
    {
      bdbg("Failed to map NXFLAT ISpace: %d\n", errno);
      return -errno;
    }

  bvdbg("Mapped ISpace (%d bytes) at 0x%08x\n", loadinfo->isize, loadinfo->ispace);

  /* The following call will give a pointer to the allocated but
   * uninitialized ISpace memory.
   */

  loadinfo->dspace = (struct dspace_s *)malloc(SIZEOF_DSPACE_S(loadinfo->dsize));
  if (loadinfo->dspace == 0)
    {
      bdbg("Failed to allocate DSpace\n");
      ret = -ENOMEM;
      goto errout;
    }
  loadinfo->dspace->crefs = 1;

  bvdbg("Allocated DSpace (%d bytes) at %p\n", loadinfo->dsize, loadinfo->dspace);

  /* Now, read the data into allocated DSpace at doffset into the
   * allocated DSpace memory.
   */

  ret = nxflat_read(loadinfo, (char*)loadinfo->dspace->region, dreadsize, doffset);
  if (ret < 0)
    {
      bdbg("Failed to read .data section: %d\n", ret);
      goto errout;
    }
       
  bvdbg("TEXT=0x%x Entry point offset=0x%08x, datastart is 0x%08x\n",
      loadinfo->ispace, loadinfo->entryoffs, doffset);

  /* Resolve the address of the relocation table.  In the file, the
   * relocations should lie at the same offset as BSS.  The current
   * value of relocstart is the offset from the beginning of the file.
   * The following adjustment will convert it to an address in dspace->
   */

  reloctab = (uint32*)(loadinfo->relocstart + (uint32)loadinfo->dspace->region - loadinfo->isize);

  bvdbg("Relocation table at 0x%p, reloccount=%d\n",
      reloctab, loadinfo->reloccount);

  /* Now run through the relocation entries. */

  for (i=0; i < loadinfo->reloccount; i++)
    {
      nxflat_reloc(loadinfo, htonl(reloctab[i]));
    }

  /* Zero the BSS area, trashing the relocations that lived in space
   * in the file.
   */

  memset((void*)(loadinfo->dspace->region + loadinfo->datasize),
	          0, loadinfo->bsssize);
  return OK;

errout:
  (void)nxflat_unload(loadinfo);
  return ret;
}

