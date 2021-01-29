/************************************************************************************
 * include/nuttx/binfmt/ieee695.h
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

/* IEEE695 Record Types *************************************************************/

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

/* Obsolete records */

#define IEEE695_RECTYPE_RHEADR       0x6e  /* R-Module Header Record */
#define IEEE695_RECTYPE_REGINT       0x70  /* Register Initialization Record */
#define IEEE695_RECTYPE_REDATA       0x72  /* Relocatable Enumerated Data Record */
#define IEEE695_RECTYPE_RIDATA       0x74  /* Relocatable Iterated Data Record */
#define IEEE695_RECTYPE_OVLDEF       0x76  /* Overlay Definition Record */
#define IEEE695_RECTYPE_ENDREC       0x78  /* End Record */
#define IEEE695_RECTYPE_BLKDEF       0x7a  /* Block Definition Record */
#define IEEE695_RECTYPE_BLKEND       0x7c  /* Block End Record */
#define IEEE695_RECTYPE_DEBSYM       0x7e  /* Debug Symbols Record */
#define IEEE695_RECTYPE_PEDATA       0x84  /* Physical Enumerated Data Record */
#define IEEE695_RECTYPE_PIDATA       0x86  /* Physical Iterated Data Record */
#define IEEE695_RECTYPE_TYPDEF       0x8e  /* Type of data represented by a name */
#define IEEE695_RECTYPE_LOCSYM       0x92  /* Local Symbols Record */
#define IEEE695_RECTYPE_NONAME       0x9e  /* Unnamed record */
#define IEEE695_RECTYPE_LIBHED       0xa4  /* Library Header Record */
#define IEEE695_RECTYPE_LIBNAM       0xa6  /* Library Module Names Record */
#define IEEE695_RECTYPE_LIBLOC       0xa8  /* Library Module Locations Record */
#define IEEE695_RECTYPE_LIBDIC       0xaa  /* Library Dictionary Record */

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

/* Standard Functions, Identifiers and Commands *************************************/

/* Standard functions */

#define IEEE695_FUNC_F               0xa0
#define IEEE695_FUNC_T               0xa1
#define IEEE695_FUNC_ABS             0xa2
#define IEEE695_FUNC_NEG             0xa3
#define IEEE695_FUNC_NOT             0xa4
#define IEEE695_FUNC_PLUS            0xa5
#define IEEE695_FUNC_MINUS           0xa6
#define IEEE695_FUNC_DIV             0xa7
#define IEEE695_FUNC_MUL             0xa8
#define IEEE695_FUNC_MAX             0xa9
#define IEEE695_FUNC_MIN             0xaa
#define IEEE695_FUNC_MOD             0xab
#define IEEE695_FUNC_LT              0xac
#define IEEE695_FUNC_GT              0xad
#define IEEE695_FUNC_EQU             0xae
#define IEEE695_FUNC_NEQ             0xaf
#define IEEE695_FUNC_AND             0xb0
#define IEEE695_FUNC_OR              0xb1
#define IEEE695_FUNC_XOR             0xb2
#define IEEE695_FUNC_EXT             0xb3
#define IEEE695_FUNC_INS             0xb4
#define IEEE695_FUNC_ERR             0xb5
#define IEEE695_FUNC_IF              0xb6
#define IEEE695_FUNC_ELSE            0xb7
#define IEEE695_FUNC_END             0xb8
#define IEEE695_FUNC_ESCAPE          0xb9
#define IEEE695_FUNC_LPSIGNED        0xba
#define IEEE695_FUNC_RPSIGNED        0xbb
#define IEEE695_FUNC_LPUNSIGNED      0xbc
#define IEEE695_FUNC_RPUNSIGNED      0xbd
#define IEEE695_FUNC_LPEITHER        0xbe
#define IEEE695_FUNC_RPEITHER        0xbf

/* Standard Identifiers */

#define IEEE695_IDENT_A              0xc1
#define IEEE695_IDENT_B              0xc2
#define IEEE695_IDENT_C              0xc3
#define IEEE695_IDENT_D              0xc4
#define IEEE695_IDENT_E              0xc5
#define IEEE695_IDENT_F              0xc6
#define IEEE695_IDENT_G              0xc7
#define IEEE695_IDENT_H              0xc8
#define IEEE695_IDENT_I              0xc9
#define IEEE695_IDENT_J              0xca
#define IEEE695_IDENT_K              0xcb
#define IEEE695_IDENT_L              0xcc
#define IEEE695_IDENT_M              0xcd
#define IEEE695_IDENT_N              0xce
#define IEEE695_IDENT_O              0xcf
#define IEEE695_IDENT_P              0xd0
#define IEEE695_IDENT_Q              0xd1
#define IEEE695_IDENT_R              0xd2
#define IEEE695_IDENT_S              0xd3
#define IEEE695_IDENT_T              0xd4
#define IEEE695_IDENT_U              0xd5
#define IEEE695_IDENT_V              0xd6
#define IEEE695_IDENT_W              0xd7
#define IEEE695_IDENT_X              0xd8
#define IEEE695_IDENT_Y              0xd9
#define IEEE695_IDENT_Z              0xda

/* Standard Commands */

#define IEEE695_CMD_MB               0xe0 /* Module Begin (MB) */
#define IEEE695_CMD_ME               0xe1
#define IEEE695_CMD_AS               0xe2 /* Assign Value to variable (ASvn) */
#define IEEE695_CMD_IR               0xe3
#define IEEE695_CMD_LR               0xe4
#define IEEE695_CMD_SB               0xe5 /* Set Current Section (SB) */
#define IEEE695_CMD_ST               0xe6 /* Section Type (ST) */
#define IEEE695_CMD_SA               0xe7 /* Section Alignment (SA) */
#define IEEE695_CMD_NI               0xe8 /* Public (External) Symbol (NI) */
#define IEEE695_CMD_NX               0xe9 /* External Reference Name (NX) */
#define IEEE695_CMD_CO               0xea
#define IEEE695_CMD_DT               0xeb
#define IEEE695_CMD_AD               0xec /* Address Descriptor (AD) */
#define IEEE695_CMD_LD               0xed
#define IEEE695_CMD_CSSUM            0xee
#define IEEE695_CMD_CS               0xef
#define IEEE695_CMD_NN               0xf0
#define IEEE695_CMD_AT               0xf1 /* Assign Attribute (ATN) */
#define IEEE695_CMD_TY               0xf2
#define IEEE695_CMD_RI               0xf3
#define IEEE695_CMD_WX               0xf4 /* Weak External Reference (WX) */
#define IEEE695_CMD_LI               0xf5
#define IEEE695_CMD_LX               0xf6
#define IEEE695_CMD_RE               0xf7
#define IEEE695_CMD_BB               0xf8 /* Block Begin (BB) */
#define IEEE695_CMD_BE               0xf9 /* Block End (BE) */
#define IEEE695_CMD_LT               0xfa
#define IEEE695_CMD_NC               0xfb /* Define Context (NC) */

/* Extended commands */

#define IEEE695_EXTCMD_ISDEF         {0x01, 0xb9}
#define IEEE695_EXTCMD_TRANS         {0x02, 0xb9}
#define IEEE695_EXTCMD_SPLIT         {0x03, 0xb9}
#define IEEE695_EXTCMD_INBLOCK       {0x04, 0xb9}
#define IEEE695_EXTCMD_CALLOPT       {0x05, 0xb9}

/* Attribute Definitions */

#define IEEE695_ATTR_VERSION         37 /* Object format version number, 2 bytes follow */
#define IEEE695_ATTR_OBJFORMT        38 /* Object format type, 1 byte follows */
#define IEEE695_ATTR_CASESENSITIVE   39 /* Case sensitivity, 1 byte follows */
#define IEEE695_ATTR_MEMORYMODEL     40 /* Memory model, 1 byte follows */

#define IEEE695_ATTR_CREATIONTIME    50 /* Creation date/time, 6 bytes follows */
#define IEEE695_ATTR_CMDLINE         51 /* Command line text, string follows */
#define IEEE695_ATTR_EXECSTATUS      52 /* Execution status, 1 byte follows */
#define IEEE695_ATTR_HOSTENVIRON     53 /* Host environment, 1 byte follows */
#define IEEE695_ATTR_TOOLVERSION     54 /* Tool and version number used to create the module, 3 bytes follows */
#define IEEE695_ATTR_COMMENTS        55 /* Comments, string follows */
#define IEEE695_ATTR_UNKNOWN         56 /* What is this? 1 byte follows */

#define IEEE695_OBJFORMT_ABSOLUTE    1  /* Absolute (not relinkable) */
#define IEEE695_OBJFORMT_RELOCATABLE 2  /* Relocatable */
#define IEEE695_OBJFORMT_LOADABLE    3  /* Loadable */
#define IEEE695_OBJFORMT_LIBRARY     4  /* Library */

#define IEEE695_CASE_SENSITIVE       1  /* Treat all symbols as if they were upper case */
#define IEEE695_CASE_INSENSITIVE     2  /* Do not change the case of symbols */

#define IEEE695_MEMORYMODEL_TINY     0  /* Code and data: Same single 64K segment */
#define IEEE695_MEMORYMODEL_SMALL    1  /* Code and data: Each have a single 64K segment */
#define IEEE695_MEMORYMODEL_MEDIUM   2  /* Data: Single 64K segment; Code: Multiple 64K segments */
#define IEEE695_MEMORYMODEL_COMPACT  3  /* Data: Multiple 64K segments; Code: Single 64K segment */
#define IEEE695_MEMORYMODEL_LARGE    4  /* Data and code: Both have multiple 64K segments */
#define IEEE695_MEMORYMODEL_BIG      5  /* Code has multiple 64K segments with common "near" data area */
#define IEEE695_MEMORYMODEL_HUGE     6  /* All large arrays and structures are in their own section */

#define IEEE695_EXECSTATUS_SUCCESS   0  /* Success */
#define IEEE695_EXECSTATUS_WARNINGS  1  /* Warning(s) */
#define IEEE695_EXECSTATUS_ERRORS    2  /* Error(s) */
#define IEEE695_EXECSTATUS_FATAL     3  /* Fatal error(s) */

#define IEEE695_HOSTENVIRON_UNKNOWN  0
#define IEEE695_HOSTENVIRON_VMS      1
#define IEEE695_HOSTENVIRON_MSDOS    2
#define IEEE695_HOSTENVIRON_UNIX     3
#define IEEE695_HOSTENVIRON_HPUX     4

/* Symbol Type */

#define IEEE695_SYMTYPE_UNKNOWN      0  /* Unspecified */
#define IEEE695_SYMTYPE_BYTE         3  /* 8-bit data byte */
#define IEEE695_SYMTYPE_SHORT        5  /* 16-bit short data word */
#define IEEE695_SYMTYPE_LONG         7  /* 32-bit long data word */
#define IEEE695_SYMTYPE_FLOAT        10 /* 32-bit floating point */
#define IEEE695_SYMTYPE_DOUBLE       11 /* 64-bit floating point */
#define IEEE695_SYMTYPE_FLOAT12      12 /* 10 or 12 byte floating point */
#define IEEE695_SYMTYPE_ADDRESS      15 /* Instruction address */

/* Symbol Attributes */

#define IEEE695_SYMATTR_GLOBAL       8  /* Global compiler symbol */
#define IEEE695_SYMATTR_CONSTANT     16 /* Constant (see additional attributes below */
#define IEEE695_SYMATTR_STATIC       19 /* Static symbol generated by assembler */

/* Constant Symbol Attributes.  Constant symbols have four additional arguments:
 *
 * 1. Symbol class
 * 2. Public/local indicator (optional).  One means public.
 * 3. Numeric value (optional)
 * 4. String value (optional)
 *
 * Either 3 or 4 should be present, but not both.
 */

#define IEEE695_SYMCLASS_UNKNOWN     0  /* Unknown class */
#define IEEE695_SYMCLASS_EQU         1  /* EQU constant */
#define IEEE695_SYMCLASS_SET         2  /* SET constant */
#define IEEE695_SYMCLASS_PASCAL      3  /* Pascal CONST constant */
#define IEEE695_SYMCLASS_DEFINE      4  /* C #define constant */

/* Helper Macros ********************************************************************/

/* These macros extract un-aligned, little-endian values from the object file */

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
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions Definitions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_IEEE695_H */
