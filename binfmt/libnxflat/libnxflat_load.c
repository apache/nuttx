/****************************************************************************
 * libnxflat/lib/libnxflat_load.c
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
#include <nuttx/nxflat.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define V_MAP   (load_info->vtbl->map)
#define V_UNMAP (load_info->vtbl->unmap)
#define V_ALLOC (load_info->vtbl->alloc)
#define V_FREE  (load_info->vtbl->free)
#define V_OPEN  (load_info->vtbl->open)
#define V_READ  (load_info->vtbl->read)
#define V_CLOSE (load_info->vtbl->close)

#define NXFLAT_HDR_SIZE sizeof(struct nxflat_hdr_s)

#ifndef MAX
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

#ifdef CONFIG_NXFLAT_DEBUG
static const char text_segment[] = "TEXT";
static const char data_segment[] = "DATA";
static const char bss_segment[]  = "BSS";
static const char unknown[]      = "UNKNOWN";

static const char *segment[] =
{
  text_segment,
  data_segment,
  bss_segment,
  unknown
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_swap32
 ****************************************************************************/

#ifdef __BIG_ENDIAN
static inline uint32 nxflat_swap32(uint32 little)
{
    uint32 big =
	((little >> 24) & 0xff) | 
	(((little >> 16) & 0xff) << 8) |
	(((little >> 8) & 0xff) << 16) |
	((little & 0xff) << 24);
    return big;
}
#endif

/****************************************************************************
 * Name: nxflat_reloc
 ****************************************************************************/

static void nxflat_reloc(struct nxflat_loadinfo_s *load_info, uint32 rl)
{
  union
  {
    uint32 l;
    struct nxflat_reloc_s s;
  } reloc;
  uint32 *ptr;
  uint32 datastart;

  /* Force the long value into a union so that we can strip off some
   * bit-encoded values.
   */

  reloc.l = rl;

  /* We only support relocations in the data sections.
   * Verify that the the relocation address lies in the data
   * section of the file image.
   */

  if (reloc.s.r_offset > load_info->data_size)
    {
      dbg("ERROR: Relocation at 0x%08x invalid -- "
	  "does not lie in the data segment, size=0x%08x\n",
	  reloc.s.r_offset, load_info->data_size);
      dbg("       Relocation not performed!\n");
    }
  else if ((reloc.s.r_offset & 0x00000003) != 0)
    {
      dbg("ERROR: Relocation at 0x%08x invalid -- "
	  "Improperly aligned\n",
	  reloc.s.r_offset);
    }
  else
    {
      /* Get a reference to the "real" start of data.  It is
       * offset slightly from the beginning of the allocated
       * DSpace to hold information needed by ld.so at run time.
       */

      datastart = load_info->dspace + NXFLAT_DATA_OFFSET;

      /* Get a pointer to the value that needs relocation in
       * DSpace.
       */
      
      ptr = (uint32*)(datastart + reloc.s.r_offset);

      vdbg("Relocation of variable at DATASEG+0x%08x "
	  "(address 0x%p, currently 0x%08x) into segment %s\n",
	  reloc.s.r_offset, ptr, *ptr, segment[reloc.s.r_type]);
	
      switch (reloc.s.r_type)
	{
	  /* TEXT is located at an offset of NXFLAT_HDR_SIZE from
	   * the allocated/mapped ISpace region.
	   */

	case NXFLAT_RELOC_TYPE_TEXT:
	  *ptr += load_info->ispace + NXFLAT_HDR_SIZE;
	  break;

	  /* DATA and BSS are always contiguous regions.  DATA
	   * begins at an offset of NXFLAT_DATA_OFFSET from
	   * the beginning of the allocated data segment.
	   * BSS is positioned after DATA, unrelocated references
	   * to BSS include the data offset.
	   *
	   * In other contexts, is it necessary to add the data_size
	   * to get the BSS offset like:
	   *
	   *   *ptr += datastart + load_info->data_size;
	   */

	case NXFLAT_RELOC_TYPE_DATA:
	case NXFLAT_RELOC_TYPE_BSS:
	  *ptr += datastart;
	  break;

	  /* This case happens normally if the symbol is a weak
	   * undefined symbol.  We permit these.
	   */

	case NXFLAT_RELOC_TYPE_NONE:
	  dbg("NULL relocation!\n");
	  break;

	default:
	  dbg("ERROR: Unknown relocation type=%d\n", reloc.s.r_type);
	  break;
	}

      vdbg("Relocation became 0x%08x\n", *ptr);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_init
 ****************************************************************************/

int nxflat_init(bin_handle_t bin_handle,  file_handle_t file_handle,
	        const struct nxflat_hdr_s *header,  const struct nxflat_vtbl_s *vtbl,
	        struct nxflat_loadinfo_s *load_info)
{
  uint32  datastart;
  uint32  dataend;
  uint32  bssstart;
  uint32  bssend;

  vdbg("bin_handle=0x%p header=0x%p load_info=0x%p\n",
      bin_handle, header, load_info);

  /* Clear the load info structure */

  memset(load_info, 0, sizeof(struct nxflat_loadinfo_s));

  /* Verify the xFLT header */

  if (nxflat_verifyheader(header) != 0)
    {
      /* This is not an error because we will be called
       * to attempt loading EVERY binary.  Returning -ENOEXEC
       * simply informs the system that the file is not
       * an xFLT file.  Besides, if there is something worth
       * complaining about, nnxflat_verifyheader() has already
       * done so.
       */

      dbg("Bad xFLT header\n");
      return -ENOEXEC;
    }

  /* Save all of the input values in the load_info structure */

  load_info->bin_handle  = bin_handle;
  load_info->file_handle = file_handle;
  load_info->header      = header;
  load_info->vtbl        = vtbl;

  /* And extract some additional information from the xflat
   * header.  Note that the information in the xflat header is in
   * network order.
   */

  datastart             = ntohl(header->h_datastart);
  dataend               = ntohl(header->h_dataend);
  bssstart              = dataend;
  bssend                = ntohl(header->h_bssend);

  /* And put this information into the load_info structure as well.
   *
   * Note that:
   *
   *   ispace_size = the address range from 0 up to datastart.
   *   data_size   = the address range from datastart up to dataend
   *   bss_size    = the address range from dataend up to bssend.
   */

  load_info->entry_offset = ntohl(header->h_entry);
  load_info->ispace_size  = datastart;

  load_info->data_size    = dataend - datastart;
  load_info->bss_size     = bssend - dataend;
  load_info->stack_size   = ntohl(header->h_stacksize);

  /* This is the initial dspace size.  We'll recaculate this later
   * after the memory has been allocated.  So that the caller can feel
   * free to modify dspace_size values from now until then.
   */

  load_info->dspace_size  =      /* Total DSpace Size is: */
    (NXFLAT_DATA_OFFSET +         /*   Memory set aside for ldso */
     bssend - datastart +      /*   Data and bss segment sizes */
     load_info->stack_size);     /*   (Current) stack size */

  /* Get the offset to the start of the relocations (we'll relocate
   * this later).
   */

  load_info->reloc_start  = ntohl(header->h_relocstart);
  load_info->reloc_count  = ntohl(header->h_reloccount);

  return 0;
}

/****************************************************************************
 * Name: nxflat_uninit
 ****************************************************************************/

int nxflat_uninit(struct nxflat_loadinfo_s *load_info)
{
  if (load_info->file_handle)
    {
      V_CLOSE(load_info->file_handle);
    }
  return 0;
}
		      
/****************************************************************************
 * Name: nxflat_load
 ****************************************************************************/

int nxflat_load(struct nxflat_loadinfo_s *load_info)
{
  uint32 dspace_read_size;
  uint32 data_offset;
  uint32 *reloc_tab;
  uint32 result;
  int           i;

  /* Calculate the extra space we need to map in.  This region
   * will be the BSS segment and the stack.  It will also be used
   * temporarily to hold relocation information.  So the size of this
   * region will either be the size of the BSS section and the
   * stack OR, it the size of the relocation entries, whichever
   * is larger
   */

  {
    uint32 extra_alloc;
    uint32 reloc_size;

    /* This is the amount of memory that we have to have to hold
     * the relocations.
     */

    reloc_size  = load_info->reloc_count * sizeof(uint32);

    /* In the file, the relocations should lie at the same offset
     * as BSS.  The additional amount that we allocate have to
     * be either (1) the BSS size + the stack size, or (2) the
     * size of the relocation records, whicher is larger.
     */

    extra_alloc = MAX(load_info->bss_size + load_info->stack_size,
		      reloc_size);

    /* Use this addtional amount to adjust the total size of the
     * dspace region. */

    load_info->dspace_size =
      NXFLAT_DATA_OFFSET +      /* Memory used by ldso */
      load_info->data_size +   /* Initialized data */
      extra_alloc;             /* bss+stack/relocs */

    /* The number of bytes of data that we have to read from the
     * file is the data size plus the size of the relocation table.
     */

    dspace_read_size = load_info->data_size + reloc_size;
  }

  /* We'll need this a few times as well. */

  data_offset = load_info->ispace_size;

  /* We will make two mmap calls create an address space for
   * the executable.  We will attempt to map the file to get
   * the ISpace address space and to allocate RAM to get the
   * DSpace address space.  If the system does not support
   * file mapping, the V_MAP() implementation should do the
   * right thing.
   */

  /* The following call will give as a pointer to the mapped
   * file ISpace.  This may be in ROM, RAM, Flash, ...
   * We don't really care where the memory resides as long
   * as it is fully initialized and ready to execute.
   * However, the memory should be share-able between processes;
   * otherwise, we don't really have shared libraries.
   */

  load_info->ispace = (uint32)V_MAP(load_info->file_handle,
				       load_info->ispace_size);
      
  if (load_info->ispace >= (uint32) -4096)
    {
      dbg("Failed to map xFLT ISpace, error=%d\n", -load_info->ispace);
      return load_info->ispace;
    }

  vdbg("Mapped ISpace (%d bytes) at 0x%08x\n",
      load_info->ispace_size, load_info->ispace);

  /* The following call will give a pointer to the allocated
   * but uninitialized ISpace memory.
   */

  load_info->dspace = (uint32)V_ALLOC(load_info->dspace_size);

  if (load_info->dspace >= (uint32) -4096)
    {
      dbg("Failed to allocate DSpace, error=%d\n",
	  -load_info->ispace);
      (void)nxflat_unload(load_info);
      return load_info->ispace;
    }

  vdbg("Allocated DSpace (%d bytes) at 0x%08x\n",
      load_info->dspace_size, load_info->dspace);

  /* Now, read the data into allocated DSpace at an offset into
   * the allocated DSpace memory.  This offset provides a small
   * amount of BSS for use by the loader.
   */

  result = V_READ(load_info->bin_handle,
		  load_info->file_handle,
		  (char*)(load_info->dspace + NXFLAT_DATA_OFFSET),
		  dspace_read_size,
		  data_offset);

  if (result >= (uint32) -4096)
    {
      dbg("Unable to read DSpace, errno %d\n", -result);
      (void)nxflat_unload(load_info);
      return result;
    }

  /* Save information about the allocation. */

  load_info->alloc_start = load_info->dspace;
  load_info->alloc_size  = load_info->dspace_size;

  vdbg("TEXT=0x%x Entry point offset=0x%08x, datastart is 0x%08x\n",
      load_info->ispace, load_info->entry_offset, data_offset);

  /* Resolve the address of the relocation table.  In the file, the
   * relocations should lie at the same offset as BSS.  The current
   * value of reloc_start is the offset from the beginning of the file.
   * The following adjustment will convert it to an address in DSpace.
   */

  reloc_tab = (uint32*)
    (load_info->reloc_start     /* File offset to reloc records */
     + load_info->dspace        /* + Allocated DSpace memory */
     + NXFLAT_DATA_OFFSET        /* + Offset for ldso usage */
     - load_info->ispace_size); /* - File offset to DSpace */

  vdbg("Relocation table at 0x%p, reloc_count=%d\n",
      reloc_tab, load_info->reloc_count);

  /* Now run through the relocation entries. */

  for (i=0; i < load_info->reloc_count; i++)
    {
#ifdef __BIG_ENDIAN
      nxflat_reloc(load_info, nxflat_swap32(reloc_tab[i]));
#else
      nxflat_reloc(load_info, reloc_tab[i]);
#endif
    }

  /* Zero the BSS, BRK and stack areas, trashing the relocations
   * that lived in the corresponding space in the file. */

  memset((void*)(load_info->dspace + NXFLAT_DATA_OFFSET + load_info->data_size),
	       0,
	       (load_info->dspace_size - NXFLAT_DATA_OFFSET -
		load_info->data_size));

  return 0;
}

/****************************************************************************
 * Name: nxflat_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially
 *   undoes the actions of nxflat_load.
 *
 ****************************************************************************/

int nxflat_unload(struct nxflat_loadinfo_s *load_info)
{
  /* Reset the contents of the info structure. */

  /* Nothing is allocated */

  load_info->alloc_start = 0;
  load_info->alloc_size  = 0;

  /* Release the memory segments */

  if (load_info->ispace)
    {
      V_UNMAP((void*)load_info->ispace, load_info->ispace_size);
      load_info->ispace = 0;
    }

  if (load_info->dspace)
    {
      V_FREE((void*)load_info->dspace, load_info->dspace_size);
      load_info->dspace = 0;
    }

  return 0;
}

