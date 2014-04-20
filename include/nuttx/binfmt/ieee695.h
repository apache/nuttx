/************************************************************************************
 * include/nuttx/binfmt/ieee695.h
 *
 *   Copyright (C) 2014 Gregory Nutt.  All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_BINFMT_IEEE695_H
#define __INCLUDE_NUTTX_BINFMT_IEEE695_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IEEE695 Record Types */

#define IEEE695_RECTYPE_THEADR       0x80  /* Translator Header Record */
#define IEEE695_RECTYPE_LHEADR       0x82  /* Library Module Header Record */
#define IEEE695_RECTYPE_COMENT       0x88  /* Comment Record */
#define IEEE695_RECTYPE_MODEND_EVEN  0x8a  /* Module End Record */
#define IEEE695_RECTYPE_MODEND_ODD   0x8B  /* Module End Record */
#define IEEE695_RECTYPE_EXTDEF       0x8C  /* External Names Definition Record */
#define IEEE695_RECTYPE_PUBDEF_EVEN  0x90  /* Public Names Definition Record */
#define IEEE695_RECTYPE_PUBDEF_ODD   0x91  /* Public Names Definition Record */
#define IEEE695_RECTYPE_LINNUM_EVEN  0x94  /* Line Numbers Record */
#define IEEE695_RECTYPE_LINNUM_ODD   0x95  /* Line Numbers Record */
#define IEEE695_RECTYPE_LNAMES       0x96  /* List of Names Record */
#define IEEE695_RECTYPE_SEGDEF_EVEN  0x98  /* Segment Definition Record */
#define IEEE695_RECTYPE_SEGDEF_ODD   0x99  /* Segment Definition Record */
#define IEEE695_RECTYPE_GRPDEF       0x9a  /* Group Definition Record */
#define IEEE695_RECTYPE_FIXUPP_EVEN  0x9c  /* Fix-up Record */
#define IEEE695_RECTYPE_FIXUPP_ODD   0x9d  /* Fix-up Record */
#define IEEE695_RECTYPE_LEDATA_EVEN  0xa0  /* Logical Enumerated Data Record */
#define IEEE695_RECTYPE_LEDATA_ODD   0xa1  /* Logical Enumerated Data Record */
#define IEEE695_RECTYPE_LIDATA_EVEN  0xa2  /* Logical Iterated Data Record */
#define IEEE695_RECTYPE_LIDATA_ODD   0xa3  /* Logical Iterated Data Record */
#define IEEE695_RECTYPE_COMDEF       0xb0  /* Communal Names Definition Record */
#define IEEE695_RECTYPE_BAKPAT_EVEN  0xb2  /* Back-patch Record */
#define IEEE695_RECTYPE_BAKPAT_ODD   0xb3  /* Back-patch Record */
#define IEEE695_RECTYPE_LEXTDEF      0xb4  /* Local External Names Definition Record */
#define IEEE695_RECTYPE_LPUBDEF_EVEN 0xb6  /* Local Public Names Definition Record */
#define IEEE695_RECTYPE_LPUBDEF_ODD  0xb7  /* Local Public Names Definition Record */
#define IEEE695_RECTYPE_LCOMDEF      0xb8  /* Local Communal Names Definition Record */
#define IEEE695_RECTYPE_CEXTDEF      0xbc  /* COMDAT External Names Definition Record */
#define IEEE695_RECTYPE_COMDAT_EVEN  0xc2  /* Initialized Communal Data Record */
#define IEEE695_RECTYPE_COMDAT_ODD   0xc3  /* Initialized Communal Data Record */
#define IEEE695_RECTYPE_LINSYM_EVEN  0xc4  /* Symbol Line Numbers Record */
#define IEEE695_RECTYPE_LINSYM_ODD   0xc5  /* Symbol Line Numbers Record */
#define IEEE695_RECTYPE_ALIAS        0xc6  /* Alias Definition Record */
#define IEEE695_RECTYPE_NBKPAT_EVEN  0xc8  /* Named Back-patch Record */
#define IEEE695_RECTYPE_NBKPAT_ODD   0xc9  /* Named Back-patch Record */
#define IEEE695_RECTYPE_LLNAMES      0xca  /* Local Logical Names Definition Record */
#define IEEE695_RECTYPE_VERNUM       0xcc  /* OMF Version Number Record */
#define IEEE695_RECTYPE_VENDEXT      0xce  /* Vendor-specific OMF Extension Record */
#define IEEE695_RECTYPE_LIBHDR       0xf0  /* Library Header Record */
#define IEEE695_RECTYPE_LIBEND       0xf1  /* Library End Record */

/* Comment type */

#define IEEE695_COMENT_NP            0x80  /* No purge bit */
#define IEEE695_COMENT_NL            0x40  /* No list bit */

/* Comment class */

#define IEEE695_COMENT_TRANSLATOR    0x00  /* Translator (may name the source language or translator) */
#define IEEE695_COMENT_INTELCOPY     0x01  /* Intel copyright (ignored) */
                                           /* 0x2–0x9b Intel reserved */
#define IEEE695_COMENT_LIBSPEC       0x81  /* Library specifier (Replaced by comment class 9f) */
#define IEEE695_COMENT_MSDOSVER      0x9c  /* MS-DOS version (obsolete) */
#define IEEE695_COMENT_MEMMODEL      0x9d  /* Memory model */
#define IEEE695_COMENT_DOSSEG        0x9e  /* Sets Microsoft LINK's DOSSEG switch */
#define IEEE695_COMENT_LIBSRCH       0x9f  /* Default library search name */
#define IEEE695_COMENT_OMFEXT        0xa0  /* OMF extensions (see below) */
#define IEEE695_COMENT_NEWOMF        0xa1  /* "New OMF" extension. Indicates symbolic debug information version. */
#define IEEE695_COMENT_LPSEP         0xa2  /* Link Pass Separator (see below) */
#define IEEE695_COMENT_LIBMOD        0xa3  /* Library module comment record */
#define IEEE695_COMENT_EXESTR        0xa4  /* Executable string */
#define IEEE695_COMENT_INCERR        0xa6  /* Incremental compilation error */
#define IEEE695_COMENT_NOPAD         0xa7  /* No segment padding */
#define IEEE695_COMENT_WKEXT         0xa8  /* Weak Extern record */
#define IEEE695_COMENT_LZEXT         0xa9  /* Lazy Extern record */
#define IEEE695_COMENT_RANDOM        0xda  /* Comment for random comment */
#define IEEE695_COMENT_COMPILER      0xdb  /* For pragma comment(compiler); version number */
#define IEEE695_COMENT_DATE          0xdc  /* For pragma comment(date stamp) */
#define IEEE695_COMENT_TIME          0xdd  /* For pragma comment(timestamp) */
#define IEEE695_COMENT_USER          0xdf  /* For pragma comment(user). Sometimes used for copyright notices */
#define IEEE695_COMENT_DEPENDENCY    0xe9  /* Dependency file (Borland) */
#define IEEE695_COMENT_CMDLINE       0xff  /* Command line (Microsoft QuickC) */
                                           /* 0xc0-0xff Reserved for user-defined comment classes */

/* OMF extensions to the comment class.  This class consists of a set of records,
 * identified by subtype (first byte of commentary string).
 */

#define IEEE695_OMFEXT_IMPDEF        0x01  /* Import definition record */
#define IEEE695_OMFEXT_EXPDEF        0x02  /* Export definition record */
#define IEEE695_OMFEXT_INCDEF        0x03  /* Incremental compilation record */
#define IEEE695_OMFEXT_PROTECTED     0x04  /* Protected memory library */
#define IEEE695_OMFEXT_LNKDIR        0x05  /* Microsoft C++ linker directives record */
#define IEEE695_OMFEXT_BIGENDIAN     0x06  /* Big-endian */
#define IEEE695_OMFEXT_PRECOMP       0x07 
                                           /* 0x08-0xff Reserved */

/* Link pass separator.  This record conveys information to the linker about the
 * organization of the file. The value of the first byte of the commentary string
 * specifies the comment subtype. Currently, a single subtype is defined:
 *
 * Note: This comment class may become obsolete with the advent of COMDAT records.
 */

#define IEEE695_COMENT_PASS2         0x01  /* Records generated from Pass 2 of the linker */

/* These macros extract un-aligned, little endian values from the object file */

#define IEEE695_GETUINT16(p) \
  (((uint16_t)(*(FAR uint8_t *)(p)) << 8) | \
    (uint16_t)(*((FAR uint8_t *)(p) + 1)))

#define IEEE695_GETUINT32(p) \
  (((uint32_t)(*(FAR uint8_t *)(p)) << 24) | \
   ((uint32_t)(*((FAR uint8_t *)(p) + 1)) << 16) | \
   ((uint32_t)(*((FAR uint8_t *)(p) + 2)) << 8) | \
    (uint32_t)(*((FAR uint8_t *)(p) + 3)))

/* This macro extracts the 16-bit length from the IEEE 695 record header */

#define IEEE695_RECLEN(rec) \
   IEEE695_GETUINT16(((FAR struct ieee695_record_s *)(rec))->len)

/* The maximum record size of the entire record (for most record types) */

#define IEEE695_MAX_RECLEN 1024

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* This structure describes one IEEE 605 record.  The actual size of the
 * data[] array is the value of the 'len' field plus one to account to a
 * terminating checksum (or 0)
 */

struct ieee695_record_s
{
  uint8_t type;          /* Record type */
  uint8_t len[2];        /* Data length (N+1) (little endian) */
  uint8_t data[1];       /* Data begins here */
};

#define SIZEOF_IEEE695_RECORD(len) (sizeof(struct ieee695_record_s)+(len))

/* 80H THEADR—Translator Header Record */

struct ieee695_theadr_s
{
  uint8_t type;            /* Record type */
  uint8_t len[2];          /* Data length (N+1) (little endian) */
  uint8_t nstlen;          /* Name string length */
  uint8_t name[1];         /* Name string data begins here */
};

/* 82H LHEADR—Library Module Header Record */

struct ieee695_lheadr_s
{
  uint8_t type;            /* Record type */
  uint8_t len[2];          /* Data length (N+1) (little endian) */
  uint8_t nstlen;          /* Name string length */
  uint8_t name[1];         /* Name string data begins here */
};

/* 88H COMENT—Comment Record */

struct ieee695_coment_s
{
  uint8_t type;            /* Record type */
  uint8_t len[2];          /* Data length (N+1) (little endian) */
  uint8_t ctype;           /* Comment type */
  uint8_t cclass;          /* Comment class */
  uint8_t stringlen;       /* String length */
  uint8_t cstring[1];      /* Commentary Byte String (optional) */
};

/* 8AH or 8BH MODEND—Module End Record */

struct ieee695_modend_s
{
  uint8_t type;          /* Record type */
  uint8_t len[2];        /* Data length (N+1) (little endian) */
  uint8_t mtype;         /* Module type */
  uint8_t mdata[1];      /* Module data begins here */
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_IEEE695_H */

