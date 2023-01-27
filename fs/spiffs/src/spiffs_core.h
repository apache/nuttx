/****************************************************************************
 * fs/spiffs/src/spiffs_core.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of version 0.3.7 of SPIFFS by Peter Andersion.  That
 * version was originally released under the MIT license but is here re-
 * released under the NuttX BSD license.
 *
 *   Copyright (c) 2013-2017 Peter Andersson (pelleplutt1976@gmail.com)
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

#ifndef __FS_SPIFFS_SRC_SPIFFS_NUCLEIUS_H
#define __FS_SPIFFS_SRC_SPIFFS_NUCLEIUS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdbool.h>

#include "spiffs.h"
#include "spiffs_mtd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPIFFS layout
 *
 * spiffs is designed for following spi flash characteristics:
 *   - only big areas of data (blocks) can be erased
 *   - erasing resets all bits in a block to ones
 *   - writing pulls ones to zeroes
 *   - zeroes cannot be pulled to ones, without erase
 *   - wear leveling
 *
 * spiffs is also meant to be run on embedded, memory constraint devices.
 *
 * Entire area is divided in blocks. Entire area is also divided in pages.
 * Each block contains same number of pages. A page cannot be erased, but a
 * block can be erased.
 *
 * Entire area must be block_size * x
 * page_size must be block_size / (2^y) where y > 2
 *
 * ex: area = 1024*1024 bytes, block size = 65536 bytes,
 * page size = 256 bytes
 *
 * BLOCK 0  PAGE 0       object lookup 1
 *          PAGE 1       object lookup 2
 *          ...
 *          PAGE n-1     object lookup n
 *          PAGE n       object data 1
 *          PAGE n+1     object data 2
 *          ...
 *          PAGE n+m-1   object data m
 *
 * BLOCK 1  PAGE n+m     object lookup 1
 *          PAGE n+m+1   object lookup 2
 *          ...
 *          PAGE 2n+m-1  object lookup n
 *          PAGE 2n+m    object data 1
 *          PAGE 2n+m    object data 2
 *          ...
 *          PAGE 2n+2m-1 object data m
 * ...
 *
 * n is number of object lookup pages, which is number of pages needed to
 * index all pages in a block by object objid
 *   : block_size / page_size * sizeof(objid) / page_size
 * m is number data pages, which is number of pages in block minus number of
 * lookup pages
 *   : block_size / page_size - block_size / page_size * sizeof(objid) /
 *     page_size
 * thus, n+m is total number of pages in a block
 *   : block_size / page_size
 *
 * ex: n = 65536/256*2/256 = 2, m = 65536/256 - 2 = 254 =>
 *     n+m = 65536/256 = 256
 *
 * Object lookup pages contain object objid entries. Each entry represent the
 * corresponding data page.
 * Assuming a 16 bit object objid, an object objid being 0xffff represents a
 * free page.
 * An object objid being 0x0000 represents a deleted page.
 *
 * ex: page 0 : lookup : 0008 0001 0aaa ffff ffff ffff ffff ffff ..
 *     page 1 : lookup : ffff ffff ffff ffff ffff ffff ffff ffff ..
 *     page 2 : data   : data for object objid 0008
 *     page 3 : data   : data for object objid 0001
 *     page 4 : data   : data for object objid 0aaa
 *     ...
 *
 *
 * Object data pages can be either object index pages or object content.
 * All object data pages contains a data page header, containing object objid
 * and span index.
 * The span index denotes the object page ordering amongst data pages with
 * same object objid.
 * This applies to both object index pages (when index spans more than one
 * page of entries), and object data pages.
 * An object index page contains page entries pointing to object content
 * page. The entry index in a object index page correlates to the span index
 * in the actual object data page.
 * The first object index page (span index 0) is called object index header
 * page, and also contains object flags (directory/file), size, object name
 * etc.
 *
 * ex:
 *  BLOCK 1
 *    PAGE 256: object lookup page 1
 *      [*123] [ 123] [ 123] [ 123]
 *      [ 123] [*123] [ 123] [ 123]
 *      [free] [free] [free] [free] ...
 *    PAGE 257: object lookup page 2
 *      [free] [free] [free] [free] ...
 *    PAGE 258: object index page (header)
 *      obj.objid:0123 span.ndx:0000 flags:INDEX
 *      size:1600 name:ex.txt type:file
 *      [259] [260] [261] [262]
 *    PAGE 259: object data page
 *      obj.objid:0123 span.ndx:0000 flags:DATA
 *    PAGE 260: object data page
 *      obj.objid:0123 span.ndx:0001 flags:DATA
 *    PAGE 261: object data page
 *      obj.objid:0123 span.ndx:0002 flags:DATA
 *    PAGE 262: object data page
 *      obj.objid:0123 span.ndx:0003 flags:DATA
 *    PAGE 263: object index page
 *      obj.objid:0123 span.ndx:0001 flags:INDEX
 *      [264] [265] [fre] [fre]
 *      [fre] [fre] [fre] [fre]
 *    PAGE 264: object data page
 *      obj.objid:0123 span.ndx:0004 flags:DATA
 *    PAGE 265: object data page
 *      obj.objid:0123 span.ndx:0005 flags:DATA
 */

/* Internal error numbers.
 *
 * REVISIT:  These should all be converted to standard errors as defined in
 * errno.h.  -EFTYPE might be a good choice for most.  Only three are
 * actually referenced from within SPIFFS a require special naming.
  */

#define SPIFFS_ERR_INTERNAL             (-256)

#define SPIFFS_ERR_DELETED              (SPIFFS_ERR_INTERNAL - 1)
#define SPIFFS_ERR_NOT_FINALIZED        (SPIFFS_ERR_INTERNAL - 2)
#define SPIFFS_ERR_NOT_INDEX            (SPIFFS_ERR_INTERNAL - 3)
#define SPIFFS_ERR_IS_INDEX             (SPIFFS_ERR_INTERNAL - 4)
#define SPIFFS_ERR_IS_FREE              (SPIFFS_ERR_INTERNAL - 5)
#define SPIFFS_ERR_INDEX_SPAN_MISMATCH  (SPIFFS_ERR_INTERNAL - 6)
#define SPIFFS_ERR_DATA_SPAN_MISMATCH   (SPIFFS_ERR_INTERNAL - 7)
#define SPIFFS_ERR_INDEX_REF_FREE       (SPIFFS_ERR_INTERNAL - 8)
#define SPIFFS_ERR_INDEX_REF_LU         (SPIFFS_ERR_INTERNAL - 9)
#define SPIFFS_ERR_INDEX_REF_INVALID    (SPIFFS_ERR_INTERNAL - 10)
#define SPIFFS_ERR_INDEX_FREE           (SPIFFS_ERR_INTERNAL - 11)
#define SPIFFS_ERR_INDEX_LU             (SPIFFS_ERR_INTERNAL - 12)
#define SPIFFS_ERR_INDEX_INVALID        (SPIFFS_ERR_INTERNAL - 13)

/* These are not errors, but live in the same number space */

/* Visitor result, continue searching */

#define SPIFFS_VIS_COUNTINUE            (SPIFFS_ERR_INTERNAL - 14)

/* Visitor result, continue searching after reloading lu buffer */

#define SPIFFS_VIS_COUNTINUE_RELOAD     (SPIFFS_ERR_INTERNAL - 15)

/* Visitor result, stop searching */

#define SPIFFS_VIS_END                  (SPIFFS_ERR_INTERNAL - 16)

/* Events */

#define SPIFFS_EV_NDXUPD                (0)  /* Updating object index contents */
#define SPIFFS_EV_NDXNEW                (1)  /* Creating new object index */
#define SPIFFS_EV_NDXDEL                (2)  /* Deleting object index */
#define SPIFFS_EV_NDXMOV                (3)  /* Moving abject index without
                                              * updating contents */
#define SPIFFS_EV_NDXUPD_HDR            (4)  /* Updating object index header
                                              * data only (not the table itself */

#define SPIFFS_OBJID_NDXFLAG            ((int16_t)(1 << (8 * sizeof(int16_t) - 1)))

#define SPIFFS_UNDEFINED_LEN            (uint32_t)(-1)

#define SPIFFS_OBJID_DELETED            ((int16_t)0)
#define SPIFFS_OBJID_FREE               ((int16_t)-1)

/* Number of object lookup pages per block */

#define SPIFFS_OBJ_LOOKUP_PAGES(fs) \
  (MAX(1, (SPIFFS_GEO_PAGES_PER_BLOCK(fs) * sizeof(int16_t)) / SPIFFS_GEO_PAGE_SIZE(fs)))

/* Checks if page index belongs to object lookup */

#define SPIFFS_IS_LOOKUP_PAGE(fs,pgndx)     \
  (((pgndx) % SPIFFS_GEO_PAGES_PER_BLOCK(fs)) < SPIFFS_OBJ_LOOKUP_PAGES(fs))

/* Number of object lookup entries in all object lookup pages */

#define SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs) \
  (SPIFFS_GEO_PAGES_PER_BLOCK(fs)-SPIFFS_OBJ_LOOKUP_PAGES(fs))

/* Converts a block to physical address */

#define SPIFFS_BLOCK_TO_PADDR(fs, block)       ((block) * SPIFFS_GEO_BLOCK_SIZE(fs))

/* Converts a object lookup entry to page index */

#define SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, block, entry) \
  ((block)*SPIFFS_GEO_PAGES_PER_BLOCK(fs) + (SPIFFS_OBJ_LOOKUP_PAGES(fs) + entry))

/* Converts a object lookup entry to physical address of corresponding page */

#define SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, block, entry) \
  (SPIFFS_BLOCK_TO_PADDR(fs, block) + (SPIFFS_OBJ_LOOKUP_PAGES(fs) + entry) * SPIFFS_GEO_PAGE_SIZE(fs))

/* Converts a page to physical address */

#define SPIFFS_PAGE_TO_PADDR(fs, page)         ((page) * SPIFFS_GEO_PAGE_SIZE(fs))

/* Converts a physical address to page */

#define SPIFFS_PADDR_TO_PAGE(fs, addr)         ((addr) / SPIFFS_GEO_PAGE_SIZE(fs))

/* Gives index in page for a physical address */

#define SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr)  ((addr) % SPIFFS_GEO_PAGE_SIZE(fs))

/* Returns containing block for given page */

#define SPIFFS_BLOCK_FOR_PAGE(fs, page)        ((page) / SPIFFS_GEO_PAGES_PER_BLOCK(fs))

/* Returns starting page for block */

#define SPIFFS_PAGE_FOR_BLOCK(fs, block)       ((block) * SPIFFS_GEO_PAGES_PER_BLOCK(fs))

/* Converts page to entry in object lookup page */

#define SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, page) \
  ((page) % SPIFFS_GEO_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs))

/* Returns data size in a data page */

#define SPIFFS_DATA_PAGE_SIZE(fs) \
    (SPIFFS_GEO_PAGE_SIZE(fs) - sizeof(struct spiffs_page_header_s))

/* Returns physical address for block's erase count,
 * always in the physical last entry of the last object lookup page
 */

#define SPIFFS_ERASE_COUNT_PADDR(fs, blkndx) \
  (SPIFFS_BLOCK_TO_PADDR(fs, blkndx) + SPIFFS_OBJ_LOOKUP_PAGES(fs) * SPIFFS_GEO_PAGE_SIZE(fs) - sizeof(int16_t))

/* Checks if there is any room for magic in the object luts */

#define SPIFFS_CHECK_MAGIC_POSSIBLE(fs) \
  ((SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs) % (SPIFFS_GEO_PAGE_SIZE(fs)/sizeof(int16_t))) * sizeof(int16_t) \
    <= (SPIFFS_GEO_PAGE_SIZE(fs)-sizeof(int16_t)*2))

/* Entries in an object header page index */

#define SPIFFS_OBJHDR_NDXLEN(fs) \
  ((SPIFFS_GEO_PAGE_SIZE(fs) - sizeof(struct spiffs_pgobj_ndxheader_s))/sizeof(int16_t))

/* Entries in an object page index */

#define SPIFFS_OBJNDX_LEN(fs) \
  ((SPIFFS_GEO_PAGE_SIZE(fs) - sizeof(struct spiffs_page_objndx_s))/sizeof(int16_t))

/* Object index entry for given data span index */

#define SPIFFS_OBJNDX_ENTRY(fs, spndx) \
  ((spndx) < SPIFFS_OBJHDR_NDXLEN(fs) ? (spndx) : (((spndx)-SPIFFS_OBJHDR_NDXLEN(fs))%SPIFFS_OBJNDX_LEN(fs)))

/* Object index span index number for given data span index or entry */

#define SPIFFS_OBJNDX_ENTRY_SPNDX(fs, spndx) \
  ((spndx) < SPIFFS_OBJHDR_NDXLEN(fs) ? 0 : (1+((spndx)-SPIFFS_OBJHDR_NDXLEN(fs))/SPIFFS_OBJNDX_LEN(fs)))

/* Get data span index for object index span index */

#define SPIFFS_DATA_SPNDX_FOR_OBJNDX_SPNDX(fs, spndx) \
  ((spndx) == 0 ? 0 : (SPIFFS_OBJHDR_NDXLEN(fs) + (((spndx)-1) * SPIFFS_OBJNDX_LEN(fs))))

#define SPIFFS_OP_T_OBJ_LU    (0 << 0)
#define SPIFFS_OP_T_OBJ_LU2   (1 << 0)
#define SPIFFS_OP_T_OBJNDX    (2 << 0)
#define SPIFFS_OP_T_OBJ_DA    (3 << 0)
#define SPIFFS_OP_C_DELE      (0 << 2)
#define SPIFFS_OP_C_UPDT      (1 << 2)
#define SPIFFS_OP_C_MOVS      (2 << 2)
#define SPIFFS_OP_C_MOVD      (3 << 2)
#define SPIFFS_OP_C_FLSH      (4 << 2)
#define SPIFFS_OP_C_READ      (5 << 2)
#define SPIFFS_OP_C_WRTHRU    (6 << 2)

#define SPIFFS_OP_TYPE_MASK   (3 << 0)
#define SPIFFS_OP_COM_MASK    (7 << 2)

/* if 0, this page is written to, else clean */

#define SPIFFS_PH_FLAG_USED   (1<<0)

/* if 0, writing is finalized, else under modification */

#define SPIFFS_PH_FLAG_FINAL  (1<<1)

/* if 0, this is an index page, else a data page */

#define SPIFFS_PH_FLAG_INDEX  (1<<2)

/* if 0, page is deleted, else valid */

#define SPIFFS_PH_FLAG_DELET  (1<<7)

/* if 0, this index header is being deleted */

#define SPIFFS_PH_FLAG_NDXDELE (1<<6)

/* Check objid, only visit matching object ids */

#define SPIFFS_VIS_CHECK_ID     (1<<0)

/* report argument object objid to visitor - else object lookup objid is
 * reported
 */

#define SPIFFS_VIS_CHECK_PH     (1<<1)

/* Stop searching at end of all look up pages */

#define SPIFFS_VIS_NO_WRAP      (1<<2)

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* object structs */

/* page header, part of each page except object lookup pages
 * NB: this is always aligned when the data page is an object index,
 * as in this case struct spiffs_page_objndx_s is used
 */

begin_packed_struct struct spiffs_page_header_s
{
  int16_t objid;                        /* object objid */
  int16_t spndx;                        /* object span index */
  uint8_t flags;                        /* flags */
} end_packed_struct;

/* Object index header page header */

#ifdef CONFIG_SPIFFS_LEADING_SLASH
#define	SPIFFS_LEADING_SLASH_SIZE	1
#else
#define	SPIFFS_LEADING_SLASH_SIZE	0
#endif

begin_packed_struct struct spiffs_pgobj_ndxheader_s
{
  struct spiffs_page_header_s phdr;     /* common page header */
#ifndef CONFIG_SPIFFS_COMPAT_OLD_NUTTX
  uint8_t _align[4 - ((sizeof(struct spiffs_page_header_s) & 3) ==
                 0 ? 4 : (sizeof(struct spiffs_page_header_s) & 3))];
#endif
  uint32_t size;                        /* size of object */
  uint8_t type;                         /* type of object */
  uint8_t name[SPIFFS_LEADING_SLASH_SIZE +
               CONFIG_SPIFFS_NAME_MAX]; /* name of object */
} end_packed_struct;

/* Object index page header */

begin_packed_struct struct spiffs_page_objndx_s
{
  struct spiffs_page_header_s phdr;
  uint8_t _align[4 - ((sizeof(struct spiffs_page_header_s) & 3) ==
                 0 ? 4 : (sizeof(struct spiffs_page_header_s) & 3))];
} end_packed_struct;

/* callback func for object lookup visitor */

typedef int (*spiffs_callback_t)(FAR struct spiffs_s *fs, int16_t objid,
                                 int16_t blkndx, int entry,
                                 FAR const void *user_const,
                                 FAR void *user_var);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int     spiffs_validate_objndx(FAR struct spiffs_page_header_s *ph,
                               int16_t objid, int16_t spndx);
int     spiffs_phys_cpy(FAR struct spiffs_s *fs,
          int16_t objid, uint32_t dest, uint32_t src, uint32_t len);
int     spiffs_foreach_objlu(FAR struct spiffs_s *fs, int16_t starting_block,
          int starting_lu_entry, uint8_t flags, int16_t objid,
          spiffs_callback_t v, FAR const void *user_const,
          FAR void *user_var, FAR int16_t *blkndx, int *lu_entry);
int     spiffs_erase_block(FAR struct spiffs_s *fs, int16_t blkndx);
int     spiffs_objlu_scan(FAR struct spiffs_s *fs);
int     spiffs_objlu_find_free_objid(FAR struct spiffs_s *fs,
          int16_t *objid, FAR const uint8_t *conflicting_name);
int     spiffs_objlu_find_free(FAR struct spiffs_s *fs,
          int16_t starting_block, int starting_lu_entry,
          FAR int16_t *blkndx, FAR int *lu_entry);
int     spiffs_objlu_find_id(FAR struct spiffs_s *fs,
          int16_t starting_block, int starting_lu_entry, int16_t objid,
          FAR int16_t *blkndx, FAR int *lu_entry);
int     spiffs_objlu_find_id_and_span(FAR struct spiffs_s *fs,
          int16_t objid, int16_t spndx, int16_t exclusion_pgndx,
          FAR int16_t *pgndx);
int     spiffs_objlu_find_id_and_span_byphdr(FAR struct spiffs_s *fs,
          int16_t objid, int16_t spndx, int16_t exclusion_pgndx,
          FAR int16_t *pgndx);
int     spiffs_page_allocate_data(FAR struct spiffs_s *fs,
          int16_t objid, FAR struct spiffs_page_header_s *ph,
          FAR uint8_t *data, uint32_t len, uint32_t page_offs,
          bool finalize, FAR int16_t *pgndx);
int     spiffs_page_move(FAR struct spiffs_s *fs,
          int16_t objid, FAR uint8_t *page_data, int16_t ndx,
          FAR struct spiffs_page_header_s *page_hdr, int16_t src_pgndx,
          FAR int16_t *dst_pgndx);
int     spiffs_page_delete(FAR struct spiffs_s *fs, int16_t pgndx);
int     spiffs_fobj_create(FAR struct spiffs_s *fs,
          int16_t objid, const uint8_t name[], uint8_t type,
          FAR int16_t *objhdr_pgndx);
int     spiffs_fobj_update_ndxhdr(FAR struct spiffs_s *fs,
          FAR struct spiffs_file_s *fobj, int16_t objid,
          int16_t objhdr_pgndx,
          FAR uint8_t *new_objhdr_data, const uint8_t name[],
          uint32_t size, FAR int16_t *new_pgndx);
void    spiffs_fobj_event(FAR struct spiffs_s *fs,
          FAR struct spiffs_page_objndx_s * objndx, int ev, int16_t objid,
          int16_t spndx, int16_t new_pgndx, uint32_t new_size);
int     spiffs_fobj_open_bypage(FAR struct spiffs_s *fs,
          int16_t pgndx, FAR struct spiffs_file_s *f);
ssize_t spiffs_fobj_append(FAR struct spiffs_s *fs,
          FAR struct spiffs_file_s *fobj, off_t offset, FAR uint8_t *data,
          size_t len);
ssize_t spiffs_object_read(FAR struct spiffs_s *fs, FAR
          FAR struct spiffs_file_s *fobj, off_t offset, size_t len,
          FAR uint8_t *dest);
int     spiffs_fobj_truncate(FAR struct spiffs_s *fs,
          FAR struct spiffs_file_s *fobj, off_t new_size, bool remove_full);
ssize_t spiffs_fobj_modify(FAR struct spiffs_s *fs,
          FAR struct spiffs_file_s *fobj, off_t offset, FAR uint8_t *data,
          size_t len);
int     spiffs_find_objhdr_pgndx(FAR struct spiffs_s *fs,
          const uint8_t name[CONFIG_SPIFFS_NAME_MAX], FAR int16_t *pgndx);

#endif /* __FS_SPIFFS_SRC_SPIFFS_NUCLEIUS_H */
