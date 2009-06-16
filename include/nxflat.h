/****************************************************************************
 * include/nxflat.h
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

#ifndef __INCLUDE_NXFLAT_H
#define __INCLUDE_NXFLAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Maximum Sizes
 ****************************************************************************/

#define NXFLAT_MAX_STRING_SIZE 64 /* Largest size of string (w/zterminator) */

/****************************************************************************
 * The NXFLAT file header
 *
 * The elements within this structure are stored in network order (i.e.,
 * ntohs() and ntohl() should be used to access fields within the
 * header.
 ****************************************************************************/

struct nxflat_hdr_s
{
  /* The "magic number identifying the file type.  This field should contain
   * "NxFT"
   */

  char  h_magic[4];

  /* NXFLAT revision number number. */

  uint16 h_rev;
  uint16 h_pad;

  /* The following fields provide the memory map for the nxflat binary.
   *
   * h_entry      - Offset to the the first executable insruction from
   *                the beginning of the file.
   * h_datastart  - Offset to the beginning of the data segment from
   *                the beginning of the file.  This field can also
   *                interpreted as the size of the ISpace segment.
   * h_dataend    - Offset to the end of the data segment from the
   *                beginning of  the file.
   * h_bssend     - Offset to the end of bss segment from the beginning
   *                of the file.
   *
   * The text segment can be considered to be the contiguous (unrelocated)
   * address space range from address zero through (but not including)
   * h_datastart.
   *
   * The size of the data/bss segment includes (as a minimum) the data
   * and bss regions (bss_end - data_start) as well as the size of the
   * stack.  At run time, this region will also include program arguments
   * and environement variables.
   * 
   * The bss segment is data_end through bss_end.
   */

  uint32 h_entry;
  uint32 h_datastart;
  uint32 h_dataend;
  uint32 h_bssend;

  /* Size of stack, in bytes */

  uint32 h_stacksize;

  /* Relocation entries
   * h_relocstart - Offset to the beginning of an array of relocation
   *                records (struct nxflat_reloc).  The offset is
   *                relative to the start of the file
   * h_reloccount - The number of relocation records in the arry
   */

  uint32 h_relocstart;       /* Offset of relocation records */
  uint32 h_reloccount;       /* Number of relocation records */

  /* Imported and exported symbol tables
   *
   * h_importsymbols - Offset to the beginning of an array of imported
   *                   symbol structures (struct nxflat_import).  The
   *                   h_importsymbols offset is relative to the
   *                   beginning of the file.  Each entry of the
   *                   array contains an uint32 offset (again from
   *                   the beginning of the file) to the name of
   *                   a symbol string.  This string is null-terminated.
   * h_importcount   - The number of records in the h_exportsymbols array.
   * h_exportsymbols - Offset to the beginning of an array of export
   *                   symbol structures (struct nxflat_export).  The
   *                   h_importsymbols offset is relative to the
   *                   beginning of the file.  Each entry of the
   *                   array contains an uint32 offset (again from
   *                   the beginning of the file) to the name of
   *                   a symbol string.  This string is null-terminated.
   * h_exportcount   - The number of records in the h_exportsymbols array.
   *
   * NOTE:  All of the arrays referenced in the header reside in the
   * the .text section.  This is possible because these arrays are
   * read-only and are only referenced by the load.  Residing in text
   * also guarantees that only one copy of the array is required.
   *
   * An exception is the h_importsymbols array with will lie
   * in .data.  This array contains write-able data and must have
   * a single instance per process.  NOTE:  The string offset contained
   * within nxflat_import still refers to strings residing in the text
   * section.
   */

  uint32  h_importsymbols;   /* Offset to list of imported symbols */
  uint32  h_exportsymbols;   /* Offset to list of exported symbols */
  uint16  h_importcount;     /* Number of imported symbols */
  uint16  h_exportcount;     /* Number of imported symbols */
};

/* Legal values for the version field.  */

#define NXFLAT_VERSION_NONE     0      /* Invalid NXFLAT version */
#define NXFLAT_VERSION_CURRENT  1      /* Current version */
#define NXFLAT_VERSION_NUM      2

/****************************************************************************
 * NXFLAT Relocation types.
 *
 * The relocation records are an array of the following type.  The fields
 * in each element are stored in native machine order.
 ****************************************************************************/

#define NXFLAT_RELOC_TYPE_NONE  0      /* Invalid relocation type */
#define NXFLAT_RELOC_TYPE_TEXT  1      /* Symbol lies in .text region */
#define NXFLAT_RELOC_TYPE_DATA  2      /* Symbol lies in .data region */
#define NXFLAT_RELOC_TYPE_BSS   3      /* Symbol lies in .bss region */
#define NXFLAT_RELOC_TYPE_NUM   4

struct nxflat_reloc_s
{
#ifdef CONFIG_ENDIAN_BIG
  uint32 r_type   : 2; 
  sint32 r_offset : 30;
#else
  sint32 r_offset : 30;
  uint32 r_type   : 2; 
#endif
};

/****************************************************************************
 * NXFLAT Imported symbol type 
 *
 * The imported symbols are an array of the following type.  The fields
 * in each element are stored in native machine order.
 ****************************************************************************/

struct nxflat_import_s
{
  uint32 i_funcname;    /* Offset to name of imported function */
  uint32 i_funcaddress; /* Resolved address of imported function */
};

/****************************************************************************
 * Exported symbol type
 *
 * The exported symbols are an array of the following type.  The fields
 * in each element are stored in native machine order.
 ****************************************************************************/

struct nxflat_export_s
{
  uint32 x_funcname;    /* Offset to name of exported function */
  uint32 x_funcaddress; /* Address of exported function */
};

#endif /* __INCLUDE_NXFLAT_H */
