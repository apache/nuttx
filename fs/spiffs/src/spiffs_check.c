/****************************************************************************
 * fs/spiffs.h/spiffs_check.c
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

/* Contains functionality for checking file system consistency
 * and mending problems.
 * Three levels of consistency checks are implemented:
 *
 * Look up consistency
 *   Checks if indices in lookup pages are coherent with page headers
 * Object index consistency
 *   Checks if there are any orphaned object indices (missing object index
 *     headers).
 *   If an object index is found but not its header, the object index is
 *     deleted.
 *   This is critical for the following page consistency check.
 * Page consistency
 *   Checks for pages that ought to be indexed, ought not to be indexed, are
 *     multiple indexed
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <debug.h>

#include "spiffs.h"
#include "spiffs_core.h"
#include "spiffs_cache.h"
#include "spiffs_check.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_check_get_data_pgndx
 *
 * Description:
 *   Searches in the object indices and returns the referenced page index
 *   given the object ID and the data span index destroys fs->lu_work
 *
 * Input Parameters:
 *   fs             - A reference to the SPIFFS volume object instance
 *   objid          - Object ID
 *   data_spndx     - Data span index
 *   pgnx           - Page index
 *   objndx_pgndx   - Object index page index
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int spiffs_check_get_data_pgndx(FAR struct spiffs_s *fs,
                                       int16_t objid, int16_t data_spndx,
                                       FAR int16_t *pgndx,
                                       FAR int16_t *objndx_pgndx)
{
  uint32_t addr;
  int16_t objndx_spndx;
  int ret;

  /* Calculate object index span index for given data page span index */

  objndx_spndx = SPIFFS_OBJNDX_ENTRY_SPNDX(fs, data_spndx);

  /* Find the object index for the object ID and span index */

  ret = spiffs_objlu_find_id_and_span(fs, objid | SPIFFS_OBJID_NDXFLAG,
                                      objndx_spndx, 0, objndx_pgndx);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n", ret);
      return ret;
    }

  /* Load the object index entry */

  addr = SPIFFS_PAGE_TO_PADDR(fs, *objndx_pgndx);
  if (objndx_spndx == 0)
    {
      /* Get the referenced page from object index header */

      addr += sizeof(struct spiffs_pgobj_ndxheader_s) +
              data_spndx * sizeof(int16_t);
    }
  else
    {
      /* Get the referenced page from object index */

      addr += sizeof(struct spiffs_page_objndx_s) +
              SPIFFS_OBJNDX_ENTRY(fs, data_spndx) *
              sizeof(int16_t);
    }

  /* Read the page from FLASH (or the cache) */

  ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ, 0,
                          addr, sizeof(int16_t), (FAR uint8_t *)pgndx);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_check_rewrite_page
 *
 * Description:
 *   Copies page contents to a new page
 *
 * Input Parameters:
 *   fs        - A reference to the SPIFFS volume object instance
 *   cur_pgndx - Current page index
 *   pghdr     - Reference to page header
 *   new_pgndx - Location to return the new page index
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int spiffs_check_rewrite_page(FAR struct spiffs_s *fs,
                                     int16_t cur_pgndx,
                                     FAR struct spiffs_page_header_s *pghdr,
                                     FAR int16_t *new_pgndx)
{
  int ret;

  ret = spiffs_page_allocate_data(fs, pghdr->objid, pghdr, 0, 0, 0, 0,
                                  new_pgndx);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_page_allocate_data() failed: %d\n", ret);
      return ret;
    }

  ret = spiffs_phys_cpy(fs, 0,
                        SPIFFS_PAGE_TO_PADDR(fs, *new_pgndx) +
                        sizeof(struct spiffs_page_header_s),
                        SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                        sizeof(struct spiffs_page_header_s),
                        SPIFFS_DATA_PAGE_SIZE(fs));
  if (ret < 0)
    {
      ferr("ERROR: spiffs_phys_cpy() failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_check_rewrite_index
 *
 * Description:
 *   Rewrites the object index for given object ID and replaces the
 *   data page index to a new page index
 *
 * Input Parameters:
 *   fs             - A reference to the SPIFFS volume object instance
 *   objid          - Object ID
 *   data_spndx     - Data span index
 *   pgnx           - Page index
 *   objndx_pgndx   - Object index page index
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int spiffs_check_rewrite_index(FAR struct spiffs_s *fs,
                                      int16_t objid, int16_t data_spndx,
                                      int16_t new_data_pgndx,
                                      int16_t objndx_pgndx)
{
  FAR struct spiffs_page_header_s *objndx_phdr;
  int16_t blkndx;
  int16_t free_pgndx;
  int entry;
  int ret;

  objid |= SPIFFS_OBJID_NDXFLAG;

  /* Find free entry */

  ret = spiffs_objlu_find_free(fs, fs->free_blkndx,
                               fs->free_entry, &blkndx, &entry);
  if (ret < 0)
    {
      fwarn("WARNING: spiffs_objlu_find_free() failed: %d\n", ret);
      return ret;
    }

  free_pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);

  /* Calculate object index span index for given data page span index */

  int16_t objndx_spndx = SPIFFS_OBJNDX_ENTRY_SPNDX(fs, data_spndx);
  if (objndx_spndx == 0)
    {
      /* Calculate index in index header */

      entry = data_spndx;
    }
  else
    {
      /* Calculate entry in index */

      entry = SPIFFS_OBJNDX_ENTRY(fs, data_spndx);
    }

  /* Load index */

  ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                          0, SPIFFS_PAGE_TO_PADDR(fs, objndx_pgndx),
                          SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
      return ret;
    }

  objndx_phdr = (FAR struct spiffs_page_header_s *)fs->lu_work;

  /* Be ultra safe, double check header against provided data.  Return
   * -EFAULT to indicate this condition.
   */

  if (objndx_phdr->objid != objid)
    {
      spiffs_page_delete(fs, free_pgndx);
      return -EFAULT;
    }

  if (objndx_phdr->spndx != objndx_spndx)
    {
      spiffs_page_delete(fs, free_pgndx);
      return -EFAULT;
    }

  if ((objndx_phdr->flags & (SPIFFS_PH_FLAG_USED | SPIFFS_PH_FLAG_NDXDELE |
                             SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_FINAL |
                             SPIFFS_PH_FLAG_DELET)) !=
      (SPIFFS_PH_FLAG_NDXDELE | SPIFFS_PH_FLAG_DELET))
    {
      spiffs_page_delete(fs, free_pgndx);
      return -EFAULT;
    }

  /* Rewrite in memory */

  if (objndx_spndx == 0)
    {
      ((FAR int16_t *)((FAR uint8_t *)fs->lu_work +
        sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx] =
          new_data_pgndx;
    }
  else
    {
      ((FAR int16_t *)((FAR uint8_t *)fs->lu_work +
        sizeof(struct spiffs_page_objndx_s)))
          [SPIFFS_OBJNDX_ENTRY(fs, data_spndx)] = new_data_pgndx;
    }

  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT, 0,
                           SPIFFS_PAGE_TO_PADDR(fs, free_pgndx),
                           SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT, 0,
          SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs, free_pgndx)) +
          SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, free_pgndx) * sizeof(int16_t),
          sizeof(int16_t), (FAR uint8_t *)&objid);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  ret = spiffs_page_delete(fs, objndx_pgndx);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_check_delobj_lazy
 *
 * Description:
 *   Deletes an object just by marking object index header as deleted
 *
 * Input Parameters:
 *   fs             - A reference to the SPIFFS volume object instance
 *   objid          - Object ID to be deleted
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int spiffs_check_delobj_lazy(FAR struct spiffs_s *fs, int16_t objid)
{
  int16_t objhdr_pgndx;
  uint8_t flags = 0xff;
  int ret;

  ret = spiffs_objlu_find_id_and_span(fs, objid, 0, 0, &objhdr_pgndx);
  if (ret == -ENOENT)
    {
      return OK;
    }
  else if (ret < 0)
    {
      ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_SPIFFS_NO_BLIND_WRITES
  /* Perform a read-modify-write */

  ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, 0,
                   SPIFFS_PAGE_TO_PADDR(fs, objhdr_pgndx) +
                   offsetof(struct spiffs_page_header_s, flags),
                   sizeof(flags), &flags);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Clear the deleted flag in FLASH to mark the page deleted */

  flags &= ~SPIFFS_PH_FLAG_NDXDELE;
  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT, 0,
                           SPIFFS_PAGE_TO_PADDR(fs, objhdr_pgndx) +
                           offsetof(struct spiffs_page_header_s, flags),
                           sizeof(flags), &flags);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_check_rewrite_index
 *
 * Description:
 *   Validates the given look up entry
 *
 * Input Parameters:
 *   fs          - A reference to the SPIFFS volume object instance
 *   objid       - Object ID
 *   pghdr       - Page header
 *   cur_pgndx   - Current page index
 *   cur_block   - Current block
 *   cur_entry   - Current entry
 *   reload_lu   - Reload lookup entry (returned)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int
spiffs_check_luentry_validate(FAR struct spiffs_s *fs,
                              int16_t lu_objid,
                              FAR struct spiffs_page_header_s *pghdr,
                              int16_t cur_pgndx,
                              int16_t cur_block,
                              int cur_entry,
                              FAR bool *reload_lu)
{
  int16_t objndx_pgndx;
  int16_t ref_pgndx;
  bool delete_page = false;
  int ret = OK;

  /* Check validity, take actions */

  if (((lu_objid == SPIFFS_OBJID_DELETED) &&
       (pghdr->flags & SPIFFS_PH_FLAG_DELET)) ||
      ((lu_objid == SPIFFS_OBJID_FREE) &&
       (pghdr->flags & SPIFFS_PH_FLAG_USED) == 0))
    {
      /* Look up entry deleted / free but used in page header */

      spiffs_checkinfo("pgndx=%04x deleted/free in lu but not on page\n",
                       cur_pgndx);

      *reload_lu  = true;
      delete_page = true;

      if (pghdr->flags & SPIFFS_PH_FLAG_INDEX)
        {
          /* Header says data page data page can be removed if not
           * referenced by some object index
           */

          ret = spiffs_check_get_data_pgndx(fs, pghdr->objid,
                                            pghdr->spndx,
                                            &ref_pgndx, &objndx_pgndx);
          if (ret == -ENOENT)
            {
              /* No object with this objid, so remove page safely */

              ret = OK;
            }
          else if (ret < 0)
            {
              ferr("ERROR: spiffs_check_get_data_pgndx() failed: %d\n", ret);
              return ret;
            }
          else if (ref_pgndx == cur_pgndx)
            {
              int16_t new_pgndx;

              /* Data page referenced by object index but deleted in lu copy
               * page to new place and re-write the object index to new place
               */

              ret = spiffs_check_rewrite_page(fs, cur_pgndx, pghdr,
                                              &new_pgndx);

              spiffs_checkinfo("Data page not found elsewhere, rewriting "
                               "%04x to new page %04x\n",
                               cur_pgndx, new_pgndx);

              if (ret < 0)
                {
                  ferr("ERROR: spiffs_check_rewrite_page() failed: %d\n",
                       ret);
                  return ret;
                }

              *reload_lu = true;

              spiffs_checkinfo("Page %04x rewritten to %04x, "
                               "affected objndx_pgndx %04x\n",
                               cur_pgndx, new_pgndx, objndx_pgndx);

              ret = spiffs_check_rewrite_index(fs,
                                               pghdr->objid, pghdr->spndx,
                                               new_pgndx, objndx_pgndx);
              if (ret == -EFAULT)
                {
                  int ret2;

                  /* Index bad also, cannot mend this file */

                  spiffs_checkinfo("Index bad %d, cannot mend!\n", ret);

                  ret2 = spiffs_page_delete(fs, new_pgndx);
                  if (ret2 < 0)
                    {
                      ferr("ERROR: spiffs_page_delete() failed: %d\n", ret2);
                      return ret2;
                    }

                  ret2 = spiffs_check_delobj_lazy(fs, pghdr->objid);
                  if (ret2 < 0)
                    {
                      ferr("ERROR: spiffs_check_delobj_lazy() failed: %d\n",
                           ret2);
                      return ret2;
                    }
                }

              if (ret < 0)
                {
                  ferr("ERROR: spiffs_check_rewrite_index() failed: %d\n",
                       ret);
                  return ret;
                }
            }
        }
      else
        {
          /* Header says index page index page can be removed if other index
           * with same objid and span index is found
           */

          ret = spiffs_objlu_find_id_and_span(fs,
                                         pghdr->objid | SPIFFS_OBJID_NDXFLAG,
                                         pghdr->spndx, cur_pgndx, 0);
          if (ret == -ENOENT)
            {
              /* No such index page found, check for a data page amongst page
               * headers.  lu cannot be trusted
               */

              ret =
                spiffs_objlu_find_id_and_span_byphdr(fs,
                                                     pghdr->objid |
                                                     SPIFFS_OBJID_NDXFLAG,
                                                     0, 0, 0);
              if (ret >= 0)
                {
                  int16_t new_pgndx;

                  /* Ignore other errors.  Got a data page also, assume lu
                   * corruption only, rewrite to new page
                   */

                  ret = spiffs_check_rewrite_page(fs, cur_pgndx, pghdr,
                                                  &new_pgndx);

                  spiffs_checkinfo("Index page with data not found, "
                                   "rewriting %04x to new page %04x\n",
                                   cur_pgndx, new_pgndx);

                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_check_rewrite_page() failed: %d\n",
                           ret);
                      return ret;
                    }

                  *reload_lu = true;
                }
            }
          else if (ret < 0)
            {
              ferr("ERROR: spiffs_objlu_find_id_and_span_byphdr(): %d\n",
                   ret);
              return ret;
            }
        }
    }

  if (lu_objid != SPIFFS_OBJID_FREE && lu_objid != SPIFFS_OBJID_DELETED)
    {
      /* look up entry used */

      if ((pghdr->objid | SPIFFS_OBJID_NDXFLAG) !=
          (lu_objid | SPIFFS_OBJID_NDXFLAG))
        {
          spiffs_checkinfo("pgndx %04x differ in objid lu="
                           "%04x ph:%04x\n", cur_pgndx, lu_objid,
                           pghdr->objid);
          delete_page = true;
          if ((pghdr->flags & SPIFFS_PH_FLAG_DELET) == 0 ||
              (pghdr->flags & SPIFFS_PH_FLAG_FINAL) ||
              (pghdr->flags & (SPIFFS_PH_FLAG_INDEX |
                               SPIFFS_PH_FLAG_NDXDELE)) == 0)
            {
              /* Page deleted or not finalized, just remove it */
            }
          else if (pghdr->flags & SPIFFS_PH_FLAG_INDEX)
            {
              /* if data page, check for reference to this page */

              ret = spiffs_check_get_data_pgndx(fs,
                                                pghdr->objid,
                                                pghdr->spndx,
                                                &ref_pgndx,
                                                &objndx_pgndx);
              if (ret == -ENOENT)
                {
                  /* no object with this objid, so remove page safely */

                  ret = OK;
                }
              else if (ret < 0)
                {
                  ferr("ERROR: spiffs_check_get_data_pgndx() failed: %d\n",
                       ret);
                  return ret;
                }

              /* if found, rewrite page with object ID, update index, and
               * delete current
               */

              else if (ref_pgndx == cur_pgndx)
                {
                  int16_t new_pgndx;

                  ret = spiffs_check_rewrite_page(fs, cur_pgndx, pghdr,
                                                  &new_pgndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_check_rewrite_page() failed: %d\n",
                           ret);
                      return ret;
                    }

                  ret = spiffs_check_rewrite_index(fs, pghdr->objid,
                                                   pghdr->spndx, new_pgndx,
                                                   objndx_pgndx);
                  if (ret == -EFAULT)
                    {
                      int ret2;

                      /* Index bad also, cannot mend this file */

                      spiffs_checkinfo("Index bad %d, cannot mend!\n", ret);

                      ret2 = spiffs_page_delete(fs, new_pgndx);
                      if (ret2 < 0)
                        {
                          ferr("ERROR: spiffs_page_delete() failed: %d\n",
                               ret2);
                          return ret2;
                        }

                      ret2 = spiffs_check_delobj_lazy(fs, pghdr->objid);
                      if (ret2 < 0)
                        {
                          ferr("ERROR: spiffs_check_delobj_lazy(): %d\n",
                               ret2);
                          return ret2;
                        }

                      *reload_lu = true;
                    }

                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_check_rewrite_index(): %d\n",
                           ret);
                      return ret;
                    }
                }
            }
          else
            {
              int16_t objndx_pgndx_lu;
              int16_t objndx_pgndx_ph;

              /* Else if index, check for other pages with both ID's and
               * span index
               *
               * See if other object index page exists for lookup objid
               * and span index
               */

              ret = spiffs_objlu_find_id_and_span(fs,
                                             lu_objid | SPIFFS_OBJID_NDXFLAG,
                                             pghdr->spndx, 0,
                                             &objndx_pgndx_lu);
              if (ret == -ENOENT)
                {
                  ret = OK;
                  objndx_pgndx_lu = 0;
                }
              else if (ret < 0)
                {
                  ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                       ret);
                  return ret;
                }

              /* See if other object index exists for page header objid and
               * span index
               */

              ret = spiffs_objlu_find_id_and_span(fs,
                                         pghdr->objid | SPIFFS_OBJID_NDXFLAG,
                                         pghdr->spndx, 0, &objndx_pgndx_ph);
              if (ret == -ENOENT)
                {
                  ret = OK;
                  objndx_pgndx_ph = 0;
                }
              else if (ret < 0)
                {
                  ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                       ret);
                  return ret;
                }

              /* If both ID's found, just delete current */

              if (objndx_pgndx_ph == 0 || objndx_pgndx_lu == 0)
                {
                  struct spiffs_page_header_s new_ph;
                  int16_t data_pgndx_lu;
                  int16_t data_pgndx_ph;
                  int16_t new_pgndx;

                  /* Otherwise try finding first corresponding data pages.
                   *
                   * See if other data page exists for look up objid and
                   * span index
                   */

                  ret =
                    spiffs_objlu_find_id_and_span(fs,
                                            lu_objid & ~SPIFFS_OBJID_NDXFLAG,
                                            0, 0, &data_pgndx_lu);
                  if (ret == -ENOENT)
                    {
                      ret = OK;
                      objndx_pgndx_lu = 0;
                    }
                  else if (ret < 0)
                    {
                      ferr("ERROR: spiffs_objlu_find_id_and_span(): %d\n",
                           ret);
                      return ret;
                    }

                  /* See if other data page exists for page header objid
                   * and span index
                   */

                  ret =
                    spiffs_objlu_find_id_and_span(fs,
                                                  pghdr->objid &
                                                  ~SPIFFS_OBJID_NDXFLAG,
                                                  0, 0, &data_pgndx_ph);
                  if (ret == -ENOENT)
                    {
                      ret = OK;
                      objndx_pgndx_ph = 0;
                    }
                  else if (ret < 0)
                    {
                      ferr("ERROR: spiffs_objlu_find_id_and_span(): %d\n",
                           ret);
                      return ret;
                    }

                  new_ph.flags = 0xff & ~(SPIFFS_PH_FLAG_USED |
                                          SPIFFS_PH_FLAG_INDEX |
                                          SPIFFS_PH_FLAG_FINAL);
                  new_ph.spndx = pghdr->spndx;

                  if ((objndx_pgndx_lu != 0 && data_pgndx_lu != 0 &&
                       data_pgndx_ph != 0 && objndx_pgndx_ph == 0) ||
                      (objndx_pgndx_lu == 0 && data_pgndx_ph &&
                       objndx_pgndx_ph == 0))
                    {
                      /* Got a data page for page header objid rewrite as
                       * objid_ph
                       */

                      new_ph.objid = pghdr->objid | SPIFFS_OBJID_NDXFLAG;
                      ret = spiffs_check_rewrite_page(fs, cur_pgndx, &new_ph,
                                                      &new_pgndx);

                      spiffs_checkinfo(
                        "Rewrite page %04x as %04x to pgndx %04x\n",
                        cur_pgndx, new_ph.objid, new_pgndx);

                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_check_rewrite_page(): %d\n",
                               ret);
                          return ret;
                        }

                      *reload_lu = true;
                    }
                  else if ((objndx_pgndx_ph != 0 && data_pgndx_ph != 0 &&
                            data_pgndx_lu != 0 && objndx_pgndx_lu == 0) ||
                           (objndx_pgndx_ph == 0 && data_pgndx_lu &&
                            objndx_pgndx_lu == 0))
                    {
                      /* Got a data page for look up objid rewrite as
                       * objid_lu
                       */

                      new_ph.objid = lu_objid | SPIFFS_OBJID_NDXFLAG;

                      spiffs_checkinfo("Rewrite page %04x as %04x\n",
                                       cur_pgndx, new_ph.objid);

                      ret = spiffs_check_rewrite_page(fs, cur_pgndx, &new_ph,
                                                      &new_pgndx);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_check_rewrite_page(): %d\n",
                               ret);
                          return ret;
                        }

                      *reload_lu = true;
                    }
                  else
                    {
                      /* Cannot safely do anything */

                      spiffs_checkinfo("Nothing to do, just delete\n");
                    }
                }
            }
        }
      else if (((lu_objid & SPIFFS_OBJID_NDXFLAG) != 0 &&
                (pghdr->flags & SPIFFS_PH_FLAG_INDEX) != 0) ||
               ((lu_objid & SPIFFS_OBJID_NDXFLAG) == 0 &&
                (pghdr->flags & SPIFFS_PH_FLAG_INDEX) == 0))
        {
          int16_t data_pgndx;
          int16_t objndx_pgndx_d;

          spiffs_checkinfo("%04x lu/page index marking differ\n", cur_pgndx);

          /* see if other data page exists for given objid and span index */

          ret = spiffs_objlu_find_id_and_span(fs,
                                       lu_objid & ~SPIFFS_OBJID_NDXFLAG,
                                       pghdr->spndx, cur_pgndx, &data_pgndx);
          if (ret == -ENOENT)
            {
              ret = OK;
              data_pgndx = 0;
            }
          else if (ret < 0)
            {
              ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                   ret);
              return ret;
            }

          /* See if other object index exists for given objid and span
           * index
           */

          ret = spiffs_objlu_find_id_and_span(fs,
                                             lu_objid | SPIFFS_OBJID_NDXFLAG,
                                             pghdr->spndx, cur_pgndx,
                                             &objndx_pgndx_d);
          if (ret == -ENOENT)
            {
              ret = OK;
              objndx_pgndx_d = 0;
            }
          else if (ret < 0)
            {
              ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                   ret);
              return ret;
            }

          delete_page = true;

          /* If other data page exists and object index exists, just delete
           * page
           */

          if (data_pgndx != 0 && objndx_pgndx_d != 0)
            {
              spiffs_checkinfo(
                "Other index and data page exists, simply remove\n");
            }

          /* If only data page exists, make this page index */

          else if (data_pgndx && objndx_pgndx_d == 0)
            {
              struct spiffs_page_header_s new_ph;
              int16_t new_pgndx;

              spiffs_checkinfo("Other data page exists, make this index\n");

              new_ph.flags = 0xff & ~(SPIFFS_PH_FLAG_USED |
                                      SPIFFS_PH_FLAG_FINAL |
                                      SPIFFS_PH_FLAG_INDEX);
              new_ph.objid = lu_objid | SPIFFS_OBJID_NDXFLAG;
              new_ph.spndx = pghdr->spndx;

              ret = spiffs_page_allocate_data(fs, new_ph.objid, &new_ph,
                                              0, 0, 0, 1, &new_pgndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_page_allocate_data() failed: %d\n",
                       ret);
                  return ret;
                }

              ret = spiffs_phys_cpy(fs, 0,
                                    SPIFFS_PAGE_TO_PADDR(fs, new_pgndx) +
                                    sizeof(struct spiffs_page_header_s),
                                    SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                                    sizeof(struct spiffs_page_header_s),
                                    SPIFFS_GEO_PAGE_SIZE(fs) -
                                    sizeof(struct spiffs_page_header_s));
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_phys_cpy() failed: %d\n", ret);
                  return ret;
                }
            }

          /* If only index exists, make data page */

          else if (data_pgndx == 0 && objndx_pgndx_d)
            {
              struct spiffs_page_header_s new_ph;
              int16_t new_pgndx;

              spiffs_checkinfo("Other index page exists, make this data\n");

              new_ph.flags = 0xff & ~(SPIFFS_PH_FLAG_USED |
                                      SPIFFS_PH_FLAG_FINAL);
              new_ph.objid = lu_objid & ~SPIFFS_OBJID_NDXFLAG;
              new_ph.spndx = pghdr->spndx;

              ret = spiffs_page_allocate_data(fs, new_ph.objid, &new_ph,
                                              0, 0, 0, 1, &new_pgndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_page_allocate_data() failed: %d\n",
                       ret);
                  return ret;
                }

              ret = spiffs_phys_cpy(fs, 0,
                                    SPIFFS_PAGE_TO_PADDR(fs, new_pgndx) +
                                    sizeof(struct spiffs_page_header_s),
                                    SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                                    sizeof(struct spiffs_page_header_s),
                                    SPIFFS_GEO_PAGE_SIZE(fs) -
                                    sizeof(struct spiffs_page_header_s));
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_phys_cpy() failed: %d\n", ret);
                  return ret;
                }
            }
          else
            {
              /* If nothing exists, we cannot safely make a decision -
               * delete
               */
            }
        }
      else if ((pghdr->flags & SPIFFS_PH_FLAG_DELET) == 0)
        {
          spiffs_checkinfo("pgndx=%04x busy in lu but deleted on page\n",
                           cur_pgndx);
          delete_page = 1;
        }
      else if ((pghdr->flags & SPIFFS_PH_FLAG_FINAL))
        {
          spiffs_checkinfo("pgndx=%04x busy but not final\n",
                           cur_pgndx);

          /* Page can be removed if not referenced by object index */

          *reload_lu = true;
          ret = spiffs_check_get_data_pgndx(fs, lu_objid, pghdr->spndx,
                                            &ref_pgndx, &objndx_pgndx);
          if (ret == -ENOENT)
            {
              /* No object with this ID, so remove page safely */

              ret = OK;
              delete_page = true;
            }
          else if (ret < 0)
            {
              ferr("ERROR: spiffs_phys_cpy() failed: %d\n", ret);
              return ret;
            }
          else if (ref_pgndx != cur_pgndx)
            {
              spiffs_checkinfo(
                "Other finalized page is referred, just delete\n");
              delete_page = true;
            }
          else
            {
              uint8_t flags = 0xff;

              /* page referenced by object index but not final
               * just finalize
               */

              spiffs_checkinfo("Unfinalized page is referred, finalizing\n");

#ifdef CONFIG_SPIFFS_NO_BLIND_WRITES
              ret = spiffs_cache_read(fs,
                                SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_READ,
                                0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                                offsetof(struct spiffs_page_header_s, flags),
                                sizeof(flags), &flags);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
                  return ret;
                }
#endif

              flags &= ~SPIFFS_PH_FLAG_FINAL;
              ret = spiffs_cache_write(fs,
                                SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                                0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                                offsetof(struct spiffs_page_header_s, flags),
                                sizeof(flags), &flags);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
                  return ret;
                }
            }
        }
    }

  if (delete_page)
    {
      spiffs_checkinfo("Deleting page %04x\n", cur_pgndx);
      ret = spiffs_page_delete(fs, cur_pgndx);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_lucheck_callback
 *
 * Description:
 *    This is a callback from spiffs_foreach_objlu().  It is part of the
 *    logic of spiffs_check_luconsistency().  It checks the page page
 *    header for each entry for validity.
 *
 * Input Parameters:
 *   fs           - A reference to the SPIFFS volume object instance
 *   objid        - Object ID
 *   cur_block    - Current block
 *   cur_entry    - Current entry
 *   user_const - User provided constant data
 *   user_var   - User provided variable data
 *
 * Returned Value:
 *   Returns SPIFFS_VIS_COUNTINUE_RELOAD, SPIFFS_VIS_COUNTINUE, or a
 *   negated errno value in the event of a failure.
 *
 ****************************************************************************/

static int spiffs_lucheck_callback(FAR struct spiffs_s *fs, int16_t objid,
                                   int16_t cur_block, int cur_entry,
                                   FAR const void *user_const,
                                   FAR void *user_var)
{
  struct spiffs_page_header_s pghdr;
  int16_t cur_pgndx;
  int ret = OK;
  bool reload_lu = false;

  cur_pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, cur_block, cur_entry);

  /* Load header */

  ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                          0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                          sizeof(struct spiffs_page_header_s),
                          (FAR uint8_t *)&pghdr);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
      return ret;
    }

  ret = spiffs_check_luentry_validate(fs, objid, &pghdr, cur_pgndx,
                                      cur_block, cur_entry, &reload_lu);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_check_luentry_validate() failed: %d\n", ret);
      return ret;
    }

  return reload_lu ? SPIFFS_VIS_COUNTINUE_RELOAD : SPIFFS_VIS_COUNTINUE;
}

/****************************************************************************
 * Name: spifss_check_objndx_search
 *
 * Description:
 *   Searches for given object ID in temporary object ID index.
 *
 * Input Parameters:
 *   fs    - A reference to the SPIFFS volume object instance
 *   objid - The Object ID
 *
 * Returned Value:
 *   The index associated with the objid is returned on success.  -ENOENT
 *   is resutled if the objid was not found.
 *
 ****************************************************************************/

static int spifss_check_objndx_search(FAR struct spiffs_s *fs, int16_t objid)
{
  FAR int16_t *obj_table = (FAR int16_t *)fs->work;
  int i;

  objid &= ~SPIFFS_OBJID_NDXFLAG;
  for (i = 0; i < SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t); i++)
    {
      if ((obj_table[i] & ~SPIFFS_OBJID_NDXFLAG) == objid)
        {
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: spiffs_check_objidconsistency_callback
 *
 * Description:
 *   Check object index consistency.  This is callback from
 *   spiffs_foreach_objlu() and logically a part of
 *   spiffs_check_objidconsistency()
 *
 * Input Parameters:
 *   fs           - A reference to the SPIFFS volume object instance
 *   objid        - Object ID
 *   cur_block    - Current block
 *   cur_entry    - Current entry
 *   user_const - User provided constant data
 *   user_var   - User provided variable data
 *
 * Returned Value:
 *   Returns SPIFFS_VIS_COUNTINUE_RELOAD, SPIFFS_VIS_COUNTINUE, or a
 *   negated errno value in the event of a failure.
 *
 ****************************************************************************/

static int spiffs_check_objidconsistency_callback(FAR struct spiffs_s *fs,
                                                  int16_t objid,
                                                  int16_t cur_block,
                                                  int cur_entry,
                                                  FAR const void *user_const,
                                                  FAR void *user_var)
{
  FAR uint32_t *log_ndx = (FAR uint32_t *)user_var;
  FAR int16_t *obj_table = (FAR int16_t *)fs->work;
  int retc = SPIFFS_VIS_COUNTINUE;
  int ret = OK;

  if (objid != SPIFFS_OBJID_FREE && objid != SPIFFS_OBJID_DELETED &&
      (objid & SPIFFS_OBJID_NDXFLAG) != 0)
    {
      struct spiffs_page_header_s pghdr;
      int16_t cur_pgndx;

      cur_pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, cur_block, cur_entry);

      /* Load header */

      ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                     0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                     sizeof(struct spiffs_page_header_s), (uint8_t *)&pghdr);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
          return ret;
        }

      if (pghdr.spndx == 0 &&
          (pghdr.flags & (SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_FINAL |
                          SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_NDXDELE)) ==
          (SPIFFS_PH_FLAG_DELET))
        {
          spiffs_checkinfo("pgndx=%04x, objid=%04x spndx=%04x "
                           "header not fully deleted - deleting\n",
                           cur_pgndx, objid, pghdr.spndx);

          ret = spiffs_page_delete(fs, cur_pgndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
              return ret;
            }

          return retc;
        }

      if ((pghdr.flags & (SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_FINAL |
                          SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_NDXDELE)) ==
          (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_NDXDELE))
        {
          return retc;
        }

      if (pghdr.spndx == 0)
        {
          int ret2;

          /* objndx header page, register objid as reachable */

          ret2 = spifss_check_objndx_search(fs, objid);
          if (ret2 < 0)
            {
              /* Not registered, do it */

              obj_table[*log_ndx] = objid & ~SPIFFS_OBJID_NDXFLAG;
              (*log_ndx)++;
              if (*log_ndx >= SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t))
                {
                  *log_ndx = 0;
                }
            }
        }
      else
        {
          bool delete = false;
          int ret2;

          /* Span index
           * objndx page, see if header can be found
           */

          ret2 = spifss_check_objndx_search(fs, objid);
          if (ret2 < 0)
            {
              int16_t objhdr_pgndx;

              /* Not in temporary index, try finding it */

              ret = spiffs_objlu_find_id_and_span(fs,
                                                objid | SPIFFS_OBJID_NDXFLAG,
                                                0, 0, &objhdr_pgndx);
              retc = SPIFFS_VIS_COUNTINUE_RELOAD;

              if (ret >= 0)
                {
                  /* Found, register as reachable */

                  obj_table[*log_ndx] = objid & ~SPIFFS_OBJID_NDXFLAG;
                }
              else if (ret == -ENOENT)
                {
                  /* Not found, register as unreachable */

                  delete = true;
                  obj_table[*log_ndx] = objid | SPIFFS_OBJID_NDXFLAG;
                }
              else if (ret < 0)
                {
                  ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                       ret);
                  return ret;
                }

              (*log_ndx)++;
              if (*log_ndx >= SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t))
                {
                  *log_ndx = 0;
                }
            }
          else
            {
              /* In temporary index, check reachable flag */

              if ((obj_table[ret2] & SPIFFS_OBJID_NDXFLAG))
                {
                  /* Registered as unreachable */

                  delete = true;
                }
            }

          if (delete)
            {
              spiffs_checkinfo("pgndx=%04x objid=%04x spndx:%04x"
                               " is orphan index - deleting\n",
                               cur_pgndx, objid, pghdr.spndx);

              ret = spiffs_page_delete(fs, cur_pgndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
                  return ret;
                }
            }
        }
    }

  return retc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_check_luconsistency
 *
 * Description:
 *   Scans all object look up. For each entry, corresponding page header is
 *   checked for validity.  If an object index header page is found, this is
 *   also checked
 *
 * Input Parameters:
 *   fs - A reference to the SPIFFS volume object instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int spiffs_check_luconsistency(FAR struct spiffs_s *fs)
{
  int ret = OK;

  ret = spiffs_foreach_objlu(fs, 0, 0, 0, 0, spiffs_lucheck_callback,
                             0, 0, 0, 0);
  if (ret == SPIFFS_VIS_END)
    {
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_check_pgconsistency
 *
 * Description:
 *   Checks consistency amongst all pages and fixes irregularities
 *   Scans all pages (except lu pages), reserves 4 bits in working memory
 *   for each page
 *
 *     bit 0: 0 == FREE|DELETED, 1 == USED
 *     bit 1: 0 == UNREFERENCED, 1 == REFERENCED
 *     bit 2: 0 == NOT_INDEX,    1 == INDEX
 *     bit 3: unused
 *
 *   A consistent file system will have only pages being
 *
 *     - x000 free, unreferenced, not index
 *      - x011 used, referenced only once, not index
 *      - x101 used, unreferenced, index
 *
 *   The working memory might not fit all pages so several scans might be
 *   needed
 *
 * Input Parameters:
 *   fs - A reference to the SPIFFS volume object instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int spiffs_check_pgconsistency(FAR struct spiffs_s *fs)
{
  const uint32_t bits = 4;
  const int16_t pages_per_scan = SPIFFS_GEO_PAGE_SIZE(fs) * 8 / bits;
  int16_t pgndx_offset = 0;
  int ret = OK;

  /* For each range of pages fitting into work memory */

  while (pgndx_offset < SPIFFS_GEO_PAGES_PER_BLOCK(fs) *
                        SPIFFS_GEO_BLOCK_COUNT(fs))
    {
      int16_t cur_block = 0;
      bool restart = false;

      memset(fs->work, 0, SPIFFS_GEO_PAGE_SIZE(fs));

      /* Build consistency bitmap for ID range traversing all blocks */

      while (!restart && cur_block < SPIFFS_GEO_BLOCK_COUNT(fs))
        {
          /* Traverse each page except for lookup pages */

          int16_t cur_pgndx = SPIFFS_OBJ_LOOKUP_PAGES(fs) +
                              SPIFFS_GEO_PAGES_PER_BLOCK(fs) * cur_block;

          while (!restart && cur_pgndx <
                 SPIFFS_GEO_PAGES_PER_BLOCK(fs) * (cur_block + 1))
            {
              struct spiffs_page_header_s pghdr;
              uint32_t pgndx_bytendx;
              uint8_t pgndx_bitndx;
              bool within_range;

              /* read header */

              ret =
                spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                  0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                  sizeof(struct spiffs_page_header_s),
                                  (FAR uint8_t *)&pghdr);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
                  return ret;
                }

              within_range  = (cur_pgndx >= pgndx_offset &&
                               cur_pgndx < pgndx_offset + pages_per_scan);
              pgndx_bytendx = (cur_pgndx - pgndx_offset) / (8 / bits);
              pgndx_bitndx  = (cur_pgndx & ((8 / bits) - 1)) * bits;

              if (within_range &&
                  (pghdr.flags & SPIFFS_PH_FLAG_DELET) &&
                  (pghdr.flags & SPIFFS_PH_FLAG_USED) == 0)
                {
                  /* Used */

                  fs->work[pgndx_bytendx] |= (1 << (pgndx_bitndx + 0));
                }

              if ((pghdr.flags & SPIFFS_PH_FLAG_DELET) &&
                  (pghdr.flags & SPIFFS_PH_FLAG_NDXDELE) &&
                  (pghdr.flags & (SPIFFS_PH_FLAG_INDEX |
                                  SPIFFS_PH_FLAG_USED)) == 0)
                {
                  FAR struct spiffs_page_header_s *objndx_phdr;
                  FAR int16_t *object_page_index;
                  int16_t data_spndx_offset;
                  int entries;
                  int i;

                  /* Found non-deleted index */

                  if (within_range)
                    {
                      fs->work[pgndx_bytendx] |= (1 << (pgndx_bitndx + 2));
                    }

                  /* Load non-deleted index */

                  ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                      0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                      SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
                      return ret;
                    }

                  /* traverse index for referenced pages */

                  objndx_phdr =
                    (FAR struct spiffs_page_header_s *)fs->lu_work;

                  if (pghdr.spndx == 0)
                    {
                      /* object header page index */

                      entries           = SPIFFS_OBJHDR_NDXLEN(fs);
                      data_spndx_offset = 0;
                      object_page_index =
                        (FAR int16_t *)((FAR uint8_t *)fs->lu_work +
                          sizeof(struct spiffs_pgobj_ndxheader_s));
                    }
                  else
                    {
                      /* Object page index */

                      entries           = SPIFFS_OBJNDX_LEN(fs);
                      data_spndx_offset = SPIFFS_OBJHDR_NDXLEN(fs) +
                                          SPIFFS_OBJNDX_LEN(fs) *
                                          (pghdr.spndx - 1);
                      object_page_index =
                        (FAR int16_t *)((FAR uint8_t *) fs->lu_work +
                          sizeof(struct spiffs_page_objndx_s));
                    }

                  /* For all entries in index */

                  for (i = 0; !restart && i < entries; i++)
                    {
                      int16_t rpgndx = object_page_index[i];
                      bool rpgndx_within_range;

                      rpgndx_within_range = (rpgndx >= pgndx_offset &&
                                             rpgndx < pgndx_offset +
                                             pages_per_scan);

                      if ((rpgndx != (int16_t) - 1 &&
                           rpgndx > SPIFFS_GEO_PAGE_COUNT(fs)) ||
                           (rpgndx_within_range &&
                            SPIFFS_IS_LOOKUP_PAGE(fs, rpgndx)))
                        {
                          int16_t data_pgndx;

                          /* Bad reference */

                          spiffs_checkinfo("pgndx=%04x bad pgndx / LU "
                                           "referenced from page %04x\n",
                                           rpgndx, cur_pgndx);

                          /* Check for data page elsewhere */

                          ret = spiffs_objlu_find_id_and_span(fs,
                                                       objndx_phdr->objid &
                                                       ~SPIFFS_OBJID_NDXFLAG,
                                                       data_spndx_offset + i,
                                                       0, &data_pgndx);
                          if (ret == -ENOENT)
                            {
                              ret = OK;
                              data_pgndx = 0;
                            }
                          else if (ret < 0)
                            {
                              ferr("ERR: spiffs_objlu_find_id_and_span %d\n",
                                    ret);
                              return ret;
                            }

                          if (data_pgndx == 0)
                            {
                              struct spiffs_page_header_s new_ph;

                              /* If not, allocate free page */

                              new_ph.flags = 0xff & ~(SPIFFS_PH_FLAG_USED |
                                                      SPIFFS_PH_FLAG_FINAL);
                              new_ph.objid = objndx_phdr->objid &
                                             ~SPIFFS_OBJID_NDXFLAG;
                              new_ph.spndx = data_spndx_offset + i;

                              ret = spiffs_page_allocate_data(fs,
                                                         new_ph.objid,
                                                         &new_ph, 0, 0, 0, 1,
                                                         &data_pgndx);
                              if (ret < 0)
                                {
                                  ferr("ERR: spiffs_page_allocate_data %d\n",
                                       ret);
                                  return ret;
                                }

                              spiffs_checkinfo("Found no existing data page,"
                                               " created new @ %04x\n",
                                               data_pgndx);
                            }

                          /* Remap index */

                          spiffs_checkinfo("Rewriting index pgndx=%04x\n",
                                           cur_pgndx);

                          ret =
                            spiffs_check_rewrite_index(fs,
                                                      objndx_phdr->objid |
                                                      SPIFFS_OBJID_NDXFLAG,
                                                      data_spndx_offset + i,
                                                      data_pgndx, cur_pgndx);
                          if (ret == -EFAULT)
                            {
                              /* Index bad also, cannot mend this file */

                              spiffs_checkinfo("Index bad %d, cannot mend - "
                                               "delete object\n",
                                               ret);

                              /* Delete file */

                              ret = spiffs_page_delete(fs, cur_pgndx);
                              if (ret < 0)
                                {
                                  ferr("ERROR: spiffs_page_delete(): %d\n",
                                       ret);
                                  return ret;
                                }
                            }
                          else if (ret < 0)
                            {
                              ferr("ERR: spiffs_check_rewrite_index(): %d\n",
                                   ret);
                              return ret;
                            }

                          restart = true;
                        }
                      else if (rpgndx_within_range)
                        {
                          /* Valid reference. read referenced page header */

                          struct spiffs_page_header_s rphdr;
                          ret =
                            spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                      0, SPIFFS_PAGE_TO_PADDR(fs, rpgndx),
                                      sizeof(struct spiffs_page_header_s),
                                      (FAR uint8_t *)&rphdr);
                          if (ret < 0)
                            {
                              ferr("ERROR: spiffs_cache_read() failed: %d\n",
                                   ret);
                              return ret;
                            }

                          /* Cross reference page header check */

                          if (rphdr.objid != (pghdr.objid &
                                              ~SPIFFS_OBJID_NDXFLAG) ||
                              rphdr.spndx != data_spndx_offset + i ||
                              (rphdr.flags & (SPIFFS_PH_FLAG_DELET |
                                              SPIFFS_PH_FLAG_INDEX |
                                              SPIFFS_PH_FLAG_USED)) !=
                              (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_INDEX))
                            {
                              int16_t data_pgndx;

                              spiffs_checkinfo(
                                "pgndx=%04x has inconsistent page header "
                                "index objid/span:%04x/%04x, "
                                "ref objid/span:%04x/%04x flags=%02x\n",
                                rpgndx, pghdr.objid & ~SPIFFS_OBJID_NDXFLAG,
                                data_spndx_offset + i, rphdr.objid,
                                rphdr.spndx, rphdr.flags);

                              /* Try finding correct page */

                              ret =
                                spiffs_objlu_find_id_and_span(fs,
                                                       pghdr.objid &
                                                       ~SPIFFS_OBJID_NDXFLAG,
                                                       data_spndx_offset + i,
                                                       rpgndx, &data_pgndx);
                              if (ret == -ENOENT)
                                {
                                  ret = OK;
                                  data_pgndx = 0;
                                }
                              else if (ret < 0)
                                {
                                  ferr("spiffs_objlu_find_id_and_span: %d\n",
                                       ret);
                                  return ret;
                                }

                              if (data_pgndx == 0)
                                {
                                  /* Not found, this index is badly borked */

                                  spiffs_checkinfo(
                                    "Index bad, delete object objid %04x\n",
                                    pghdr.objid);

                                  ret = spiffs_check_delobj_lazy(fs,
                                                                pghdr.objid);
                                  if (ret < 0)
                                    {
                                      ferr("spiffs_check_delobj_lazy: %d\n",
                                           ret);
                                      return ret;
                                    }

                                  break;
                                }
                              else
                                {
                                  /* Found it, so rewrite index */

                                  spiffs_checkinfo(
                                    "Found correct data pgndx=%04x, "
                                    "rewrite index pgndx=%04x objid=%04x\n",
                                    data_pgndx, cur_pgndx, pghdr.objid);

                                  ret =
                                    spiffs_check_rewrite_index(
                                      fs, pghdr.objid, data_spndx_offset + i,
                                      data_pgndx, cur_pgndx);
                                  if (ret == -EFAULT)
                                    {
                                      /* Index bad, cannot mend this file */

                                      spiffs_checkinfo(
                                        "Index bad %d, cannot mend!\n", ret);

                                      ret = spiffs_check_delobj_lazy(
                                              fs, pghdr.objid);
                                    }
                                  else if (ret < 0)
                                    {
                                      ferr("spiffs_check_rewrite_index %d\n",
                                           ret);
                                      return ret;
                                    }

                                  restart = true;
                                }
                            }
                          else
                            {
                              /* Mark rpgndx as referenced */

                              const uint32_t rpgndx_byte_ix =
                                (rpgndx - pgndx_offset) / (8 / bits);
                              const uint8_t rpgndx_bit_ix =
                                (rpgndx & ((8 / bits) - 1)) * bits;

                              if ((fs->work[rpgndx_byte_ix] &
                                   (1 << (rpgndx_bit_ix + 1))) != 0)
                                {
                                  spiffs_checkinfo(
                                    "pgndx=%04x multiple referenced "
                                    "from page %04x\n",
                                    rpgndx, cur_pgndx);

                                  /* Here, we should have fixed all broken
                                   * references - getting this means there
                                   * must be multiple files with same object
                                   * ID. Only solution is to delete
                                   * the object which is referring to this
                                   * page
                                   */

                                  spiffs_checkinfo("Removing objid=%04x and"
                                                   "page=%04x\n",
                                                   pghdr.objid, cur_pgndx);

                                  ret = spiffs_check_delobj_lazy(
                                          fs, pghdr.objid);
                                  if (ret < 0)
                                    {
                                      ferr("spiffs_check_delobj_lazy: %d\n",
                                           ret);
                                      return ret;
                                    }

                                  /* Precaution, delete this page also */

                                  ret = spiffs_page_delete(fs, cur_pgndx);
                                  if (ret < 0)
                                    {
                                      ferr("ERR: spiffs_page_delete(): %d\n",
                                           ret);
                                      return ret;
                                    }

                                  restart = true;
                                }

                              fs->work[rpgndx_byte_ix] |=
                                (1 << (rpgndx_bit_ix + 1));
                            }
                        }
                    }
                }

              /* Next page */

              cur_pgndx++;
            }

          /* Next block */

          cur_block++;
        }

      /* Check consistency bitmap */

      if (!restart)
        {
          uint32_t byte_ndx;
          int16_t objndx_pgndx;
          int16_t rpgndx;
          uint8_t bit_ndx;

          for (byte_ndx = 0;
               !restart && byte_ndx < SPIFFS_GEO_PAGE_SIZE(fs);
               byte_ndx++)
            {
              for (bit_ndx = 0; !restart && bit_ndx < 8 / bits; bit_ndx++)
                {
                  uint8_t bitmask;
                  int16_t cur_pgndx;

                  bitmask   = (fs->work[byte_ndx] >> (bit_ndx * bits)) & 0x7;
                  cur_pgndx = pgndx_offset + byte_ndx * (8 / bits) + bit_ndx;

                  /* 000 ok - free, unreferenced, not index */

                  if (bitmask == 0x1)
                    {
                      struct spiffs_page_header_s pghdr;
                      bool rewrite_ndx_to_this = false;
                      bool delete_page = false;

                      /* 001 */

                      spiffs_checkinfo(
                        "pgndx=%04x USED, UNREFERENCED, not index\n",
                        cur_pgndx);

                      /* Check corresponding object index entry */

                      ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                      0,
                                      SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                      sizeof(struct spiffs_page_header_s),
                                      (FAR uint8_t *)&pghdr);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_cache_read() failed: %d\n",
                               ret);
                          return ret;
                        }

                      ret = spiffs_check_get_data_pgndx(fs, pghdr.objid,
                                                        pghdr.spndx, &rpgndx,
                                                        &objndx_pgndx);
                      if (ret >= 0)
                        {
                          if (((rpgndx == (int16_t) - 1 ||
                                rpgndx > SPIFFS_GEO_PAGE_COUNT(fs)) ||
                               (SPIFFS_IS_LOOKUP_PAGE(fs, rpgndx))))
                            {
                              /* Pointing to a bad page altogether, rewrite
                               * index to this
                               */

                              rewrite_ndx_to_this = true;

                              spiffs_checkinfo(
                                "Corresponding ref is bad: "
                                "%04x, rewrite to this %04x\n",
                                rpgndx, cur_pgndx);
                            }
                          else
                            {
                              struct spiffs_page_header_s rphdr;

                              /* Pointing to something else, check what */

                              ret =
                                spiffs_cache_read(fs,
                                         SPIFFS_OP_T_OBJ_LU2 |
                                         SPIFFS_OP_C_READ,
                                         0,
                                         SPIFFS_PAGE_TO_PADDR(fs, rpgndx),
                                         sizeof(struct spiffs_page_header_s),
                                         (FAR uint8_t *)&rphdr);
                              if (ret < 0)
                                {
                                  ferr("ERROR: spiffs_cache_read(): %d\n",
                                       ret);
                                  return ret;
                                }

                              if (((pghdr.objid & ~SPIFFS_OBJID_NDXFLAG) ==
                                   rphdr.objid) &&
                                  ((rphdr.flags & (SPIFFS_PH_FLAG_INDEX |
                                                    SPIFFS_PH_FLAG_DELET |
                                                    SPIFFS_PH_FLAG_USED |
                                                    SPIFFS_PH_FLAG_FINAL)) ==
                                                  (SPIFFS_PH_FLAG_INDEX |
                                                    SPIFFS_PH_FLAG_DELET)))
                                {
                                  /* Pointing to something else valid, just
                                   * delete this page then
                                   */

                                  spiffs_checkinfo(
                                    "Corresponding ref is good but "
                                    "different: %04x, delete this %04x\n",
                                    rpgndx, cur_pgndx);

                                  delete_page = true;
                                }

                              /* Pointing to something weird, update index
                               * to point to this page instead
                               */

                              else if (rpgndx != cur_pgndx)
                                {
                                  spiffs_checkinfo
                                    ("PA: corresponding ref is weird: "
                                     "%04x %s%s%s%s, rewrite this "
                                      "%04x\n", rpgndx,
                                      (rphdr.flags & SPIFFS_PH_FLAG_INDEX) ?
                                       "" : "INDEX ",
                                      (rphdr.flags & SPIFFS_PH_FLAG_DELET) ?
                                       "" : "DELETED ",
                                      (rphdr.flags & SPIFFS_PH_FLAG_USED) ?
                                       "NOTUSED " : "",
                                      (rphdr.flags & SPIFFS_PH_FLAG_FINAL) ?
                                       "NOTFINAL " : "", cur_pgndx);

                                  rewrite_ndx_to_this = true;
                                }
                              else
                                {
                                  /* Should not happen, destined for fubar */
                                }
                            }
                        }
                      else if (ret == -ENOENT)
                        {
                          spiffs_checkinfo("Corresponding ref not found, "
                                           "delete %04x\n",
                                           cur_pgndx);

                          delete_page = true;
                          ret = OK;
                        }

                      if (rewrite_ndx_to_this)
                        {
                          /* If pointing to invalid page, redirect index to
                           * this page
                           */

                          spiffs_checkinfo(
                            "Rewrite index objid=%04x data spndx=%04x"
                            " to point to this pgndx: %04x\n",
                            pghdr.objid, pghdr.spndx, cur_pgndx);

                          ret = spiffs_check_rewrite_index(fs, pghdr.objid,
                                  pghdr.spndx, cur_pgndx, objndx_pgndx);
                          if (ret == -EFAULT)
                            {
                              int ret2;

                              /* Index bad also, cannot mend this file */

                              spiffs_checkinfo("PA: FIXUP: index bad %d"
                                               ", cannot mend!\n", ret);

                              ret2 = spiffs_page_delete(fs, cur_pgndx);
                              if (ret2 < 0)
                                {
                                  ferr("ERROR: spiffs_page_delete(): %d\n",
                                       ret2);
                                  return ret2;
                                }

                              ret2 = spiffs_check_delobj_lazy(fs,
                                                              pghdr.objid);
                              if (ret2 < 0)
                                {
                                  ferr(
                                    "ERR: spiffs_check_delobj_lazy(): %d\n",
                                    ret2);
                                  return ret2;
                                }
                            }
                          else if (ret < 0)
                            {
                              ferr("ERR: spiffs_check_rewrite_index(): %d\n",
                                   ret);
                              return ret;
                            }

                          restart = true;
                          continue;
                        }
                      else if (delete_page)
                        {
                          spiffs_checkinfo("Deleting page %04x\n",
                                           cur_pgndx);

                          ret = spiffs_page_delete(fs, cur_pgndx);
                          if (ret < 0)
                            {
                              ferr("ERROR: spiffs_page_delete(): %d\n", ret);
                              return ret;
                            }
                        }
                    }

                  if (bitmask == 0x2)
                    {
                      /* 010 */

                      spiffs_checkinfo(
                        "pgndx=%04x  FREE, REFERENCED, not index\n",
                        cur_pgndx);

                      /* No op, this should be taken care of when checking
                       * valid references
                       */
                    }

                  /* 011 OK - busy, referenced, not index */

                  if (bitmask == 0x4)
                    {
                      /* 100 */

                      spiffs_checkinfo(
                        "pgndx=%04x FREE, unreferenced, INDEX\n", cur_pgndx);

                      /* This should never happen, major fubar */
                    }

                  /* 101 OK - busy, unreferenced, index */

                  if (bitmask == 0x6)
                    {
                      /* 110 */

                      spiffs_checkinfo(
                        "pgndx=%04x FREE, REFERENCED, INDEX\n", cur_pgndx);

                      /* No op, this should be taken care of when checking
                       * valid references
                       */
                    }

                  if (bitmask == 0x7)
                    {
                      /* 111 */

                      spiffs_checkinfo(
                        "pgndx=%04x USED, REFERENCED, INDEX\n", cur_pgndx);

                      /* No op, this should be taken care of when checking
                       * valid references
                       */
                    }
                }
            }
        }

      spiffs_checkinfo("Processed %04x, restart %d\n",
                       pgndx_offset, restart);

      /* next page range */

      if (!restart)
        {
          pgndx_offset += pages_per_scan;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_check_pgconsistency
 *
 * Description:
 *   Removes orphaned and partially deleted index pages.
 *   Scans for index pages. When an index page is found, corresponding index
 *   header is searched for.   If no such page exists, the index page cannot
 *   be reached as no index header exists and must be deleted.
 *
 * Input Parameters:
 *   fs - A reference to the SPIFFS volume object instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int spiffs_check_objidconsistency(FAR struct spiffs_s *fs)
{
  uint32_t objid_logndx = 0;
  int ret = OK;

  /* Implementation not:
   * fs->work is used for a temporary object index memory, listing found
   * object ids and indicating whether they can be reached or not. Acting
   * as a FIFO if object ids cannot fit.  In the temporary object index
   * memory, SPIFFS_OBJID_NDXFLAG bit is used to indicate a reachable/
   * unreachable object ID.
   */

  memset(fs->work, 0, SPIFFS_GEO_PAGE_SIZE(fs));

  ret = spiffs_foreach_objlu(fs, 0, 0, 0, 0,
                             spiffs_check_objidconsistency_callback, 0,
                             &objid_logndx, 0, 0);
  if (ret == SPIFFS_VIS_END)
    {
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_dump
 *
 * Description:
 *   Dump logical flash content
 *
 * Input Parameters:
 *   fs - A reference to the volume structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPIFFS_DUMP
int spiffs_dump(FAR struct spiffs_s *fs)
{
  FAR int16_t *objlu_buf = (FAR int16_t *)fs->lu_work;
  uint32_t pages_per_block;
  uint32_t blocks;
  uint32_t obj_lupages;
  uint32_t data_pgsize;
  uint32_t ndata_pages;
  int16_t pgndx = 0;
  int16_t erase_count;
  char buffer[80];
  int entries_per_page;
  int len = 0;
  int ret = OK;

  entries_per_page = (SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t));

  while (pgndx < SPIFFS_GEO_PAGE_COUNT(fs))
    {
      /* Check each object lookup page */

      int obj_lookup_page = 0;
      int cur_entry = 0;

      while (ret >= 0 && obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset = obj_lookup_page * entries_per_page;

          ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                  0, pgndx * SPIFFS_GEO_PAGE_SIZE(fs) +
                                  SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                                  SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);

          /* Check each entry */

          while (ret >= 0 &&
                 cur_entry - entry_offset < entries_per_page &&
                 cur_entry < (int)(SPIFFS_GEO_PAGES_PER_BLOCK(fs) -
                 SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
              int16_t objid = objlu_buf[cur_entry - entry_offset];

              if (cur_entry == 0)
                {
                  len += snprintf(&buffer[len], 80 - len, "%04x ", pgndx);
                }
              else if ((cur_entry & 0x3f) == 0)
                {
                  len += snprintf(&buffer[len], 80 - len, "     ");
                }

              if ((objid == SPIFFS_OBJID_FREE) != 0)
                {
                  len += snprintf(&buffer[len], 80 - len, ".");
                }
              else if ((objid == SPIFFS_OBJID_DELETED) != 0)
                {
                  len += snprintf(&buffer[len], 80 - len, "x");
                }
              else if ((objid & SPIFFS_OBJID_NDXFLAG) != 0)
                {
                  len += snprintf(&buffer[len], 80 - len, "I");
                }
              else
                {
                  len += snprintf(&buffer[len], 80 - len, "D");
                }

              cur_entry++;

              if ((cur_entry & 0x3f) == 0)
                {
                  len += snprintf(&buffer[len], 80 - len, "\n");
                  spiffs_checkinfo("%s", buffer);
                  len = 0;
                }
            }

          obj_lookup_page++;
        }

      ret = spiffs_cache_read(fs, SPIFFS_OP_C_READ | SPIFFS_OP_T_OBJ_LU2, 0,
                              SPIFFS_ERASE_COUNT_PADDR(fs, pgndx),
                              sizeof(int16_t), (FAR uint8_t *)&erase_count);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_mtd_read() failed: %d\n", ret);
          return ret;
        }

      if (erase_count != (int16_t)-1)
        {
          len += snprintf(&buffer[len], 80 - len,
                          "  era_cnt=%d\n", erase_count);
        }
      else
        {
          len += snprintf(&buffer[len], 80 - len, "  era_cnt (N/A)\n");
        }

      spiffs_checkinfo("%s", buffer);
      len = 0;

      pgndx++;
    }

  spiffs_checkinfo("era_cnt_max: %d\n", fs->max_erase_count);
  spiffs_checkinfo("blocks:      %d\n", SPIFFS_GEO_PAGE_COUNT(fs));
  spiffs_checkinfo("free_blocks: %d\n", fs->free_blocks);
  spiffs_checkinfo("page_alloc:  %d\n", fs->alloc_pages);
  spiffs_checkinfo("page_delet:  %d\n", fs->deleted_pages);

  /* The following duplicates some logic from spiffs_statfs().
   * -2 for  spare blocks, +1 for emergency page.
   */

  pages_per_block  = SPIFFS_GEO_PAGES_PER_BLOCK(fs);
  blocks           = SPIFFS_GEO_BLOCK_COUNT(fs);
  obj_lupages      = SPIFFS_OBJ_LOOKUP_PAGES(fs);
  data_pgsize      = SPIFFS_DATA_PAGE_SIZE(fs);
  ndata_pages      = (blocks - 2) * (pages_per_block - obj_lupages) + 1;

  spiffs_checkinfo("used:        %ld of %ld\n",
                   (long)(fs->alloc_pages * data_pgsize),
                   (long)(ndata_pages * data_pgsize));
  return OK;
}
#endif
