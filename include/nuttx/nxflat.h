/****************************************************************************
 * include/nuttx/nxflat.h
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

#ifndef __INCLUDE_NUTTX_NXFLAT_H
#define __INCLUDE_NUTTX_NXFLAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nxflat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* When DSpace is allocated, space is reserved at the beginning to
 * hold ldso-specific information.  The following structure defines
 * that information.  This structure can be referenced at run-time
 * using a negative offset from the PID base address.
 */

struct nxflat_ldso_info
{
  uint32 dspace; /* The beginning of ldso DSpace */
};
#define NXFLAT_DATA_OFFSET sizeof(struct nxflat_ldso_info)

/* This struct provides a desciption of the currently loaded
 * instantiation of an xflat binary.
 */

struct nxflat_loadinfo_s
{
  /* Instruction Space (ISpace):  This region contains the flat
   * file header plus everything from the text section.  Ideally,
   * will have only one text section instance in the system.
   */

  uint32 ispace;       /* Address where hdr/text is loaded */
                       /* 1st: struct nxflat_hdr_s */
                       /* 2nd: text section */
  uint32 entry_offset; /* Offset from ispace to entry point */
  uint32 ispace_size;  /* Size of ispace. */

  /* Data Space (DSpace): This region contains all information that
   * in referenced as data.  There will be a unique instance of
   * DSpace for each instance of a process.
   */

  uint32 dspace;       /* Address where data/bss/stack/etc. is loaded */
                          /* 1st: Memory set aside for ldso */
  uint32 data_size;    /* 2nd: Size of data segment in dspace */
  uint32 bss_size;     /* 3rd: Size of bss segment in dspace */
                          /* 4th: Potential padding from relocs/mm/etc. */
  uint32 stack_size;   /* 5th: Size of stack in dspace */
  uint32 dspace_size;  /* Size of dspace (may be large than parts) */

  /* Program arguments (addresses in dspace) */

  uint32 arg_start;    /* Beginning of program arguments */
  uint32 env_start;    /* End of argments, beginning of env */
  uint32 env_end;      /* End(+4) of env */

  /* This is temporary memory where relocation records will be loaded. */

  uint32 reloc_start;  /* Start of array of struct flat_reloc */
  uint32 reloc_count;  /* Number of elements in reloc array */

  /* File descriptors */

  int    filfd;        /* Descriptor for the file being loaded */

  const struct nxflat_hdr_s  *header; /* A reference to the flat file header */

  /* At most one memory allocation will be made.  These describe that
   * allocation.
   */

  uint32 alloc_start;  /* Start of the allocation */
  uint32 alloc_size;   /* Size of the allocation */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Given the header from a possible NXFLAT executable, verify that it
 * is an NXFLAT executable.
 */

EXTERN int nxflat_verifyheader(const struct nxflat_hdr_s *header);

/* This function is called to configure the library to process an NXFLAT
 * program binary.
 */

EXTERN int nxflat_init(const char *filename,
                       struct nxflat_hdr_s *header,
	               struct nxflat_loadinfo_s *loadinfo);

/* Releases any resources committed by nxflat_init().  This essentially
 * undoes the actions of nxflat_init.
 */

EXTERN int nxflat_uninit(struct nxflat_loadinfo_s *loadinfo);

/* Loads the binary specified by nxflat_init into memory,
 * Completes all relocations, and clears BSS.
 */

EXTERN int nxflat_load(struct nxflat_loadinfo_s *loadinfo);

/* Read 'readsize' bytes from the object file at 'offset' */

EXTERN int nxflat_read(struct nxflat_loadinfo_s *loadinfo, char *buffer,
                       int readsize, int offset);

/* This function unloads the object from memory. This essentially
 * undoes the actions of nxflat_load.
 */

EXTERN int nxflat_unload(struct nxflat_loadinfo_s *loadinfo);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NXFLAT_H */
