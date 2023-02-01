/****************************************************************************
 * binfmt/libnxflat/libnxflat_load.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <sys/mman.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <nxflat.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/binfmt/nxflat.h>

#include "libnxflat.h"

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
 * Name: nxflat_load
 *
 * Description:
 *   Loads the binary specified by nxflat_init into memory, mapping
 *   the I-space executable regions, allocating the D-Space region,
 *   and initializing the data segment (relocation information is
 *   temporarily loaded into the BSS region.  BSS will be cleared
 *   by nxflat_bind() after the relocation data has been processed).
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_load(struct nxflat_loadinfo_s *loadinfo)
{
  off_t    doffset;     /* Offset to .data in the NXFLAT file */
  uint32_t dreadsize;   /* Total number of bytes of .data to be read */
  uint32_t relocsize;   /* Memory needed to hold relocations */
  uint32_t extrasize;   /* MAX(BSS size, relocsize) */
  int      ret = OK;

  /* Calculate the extra space we need to allocate.  This extra space will be
   * the size of the BSS section.  This extra space will also be used
   * temporarily to hold relocation information.  So the allocated size of
   * this region will either be the size of .data + size of.bss section OR,
   * the size of .data + the relocation entries, whichever is larger
   *
   * This is the amount of memory that we have to have to hold the
   * relocations.
   */

  relocsize  = loadinfo->reloccount * sizeof(struct nxflat_reloc_s);

  /* In the file, the relocations should lie at the same offset as BSS.
   * The additional amount that we allocate have to be either (1) the
   * BSS size, or (2) the size of the relocation records, whicher is
   * larger.
   */

  extrasize = MAX(loadinfo->bsssize, relocsize);

  /* Use this additional amount to adjust the total size of the dspace
   * region.
   */

  loadinfo->dsize = loadinfo->datasize + extrasize;

  /* The number of bytes of data that we have to read from the file is
   * the data size plus the size of the relocation table.
   */

  dreadsize = loadinfo->datasize + relocsize;

  /* We'll need this a few times. */

  doffset = loadinfo->isize;

  /* We will make two mmap calls create an address space for the executable.
   * We will attempt to map the file to get the ISpace address space and
   * to allocate RAM to get the DSpace address space.  If the filesystem does
   * not support file mapping, the map() implementation should do the
   * right thing.
   */

  /* The following call will give as a pointer to the mapped file ISpace.
   * This may be in ROM, RAM, Flash, ... We don't really care where the
   * memory resides as long as it is fully initialized and ready to execute.
   */

  ret = file_mmap(&loadinfo->file, NULL, loadinfo->isize, PROT_READ,
                  MAP_SHARED | MAP_FILE, 0, (FAR void **)&loadinfo->ispace);
  if (ret < 0)
    {
      berr("Failed to map NXFLAT ISpace: %d\n", ret);
      return ret;
    }

  binfo("Mapped ISpace (%" PRId32 " bytes) at %08x\n",
        loadinfo->isize, loadinfo->ispace);

  /* The following call allocate D-Space memory and will provide a pointer
   * to the allocated (but still uninitialized) D-Space memory.
   */

  ret = nxflat_addrenv_alloc(loadinfo, loadinfo->dsize);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_alloc() failed: %d\n", ret);
      return ret;
    }

  binfo("Allocated DSpace (%" PRId32 " bytes) at %p\n",
        loadinfo->dsize, loadinfo->dspace->region);

  /* If CONFIG_ARCH_ADDRENV=y, then the D-Space allocation lies in an address
   * environment that may not be in place.  So, in that case, we must call
   * nxflat_addrenv_select to temporarily instantiate that address space
   * it can be initialized.
   */

#ifdef CONFIG_ARCH_ADDRENV
  ret = nxflat_addrenv_select(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_select() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Now, read the data into allocated DSpace at doffset into the allocated
   * DSpace memory.
   */

  ret = nxflat_read(loadinfo, (FAR char *)loadinfo->dspace->region,
                    dreadsize, doffset);
  if (ret < 0)
    {
      berr("Failed to read .data section: %d\n", ret);
      goto errout;
    }

  binfo("TEXT: %08x Entry point offset: %08" PRIx32 " Data offset: %08jx\n",
        loadinfo->ispace, loadinfo->entryoffs, (intmax_t)doffset);

  /* Restore the original address environment */

#ifdef CONFIG_ARCH_ADDRENV
  ret = nxflat_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: nxflat_addrenv_restore() failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;

errout:
#ifdef CONFIG_ARCH_ADDRENV
  nxflat_addrenv_restore(loadinfo);
#endif
  nxflat_unload(loadinfo);
  return ret;
}
