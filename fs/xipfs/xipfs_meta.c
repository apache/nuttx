/****************************************************************************
 * fs/xipfs/xipfs_meta.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 *
 * Power-loss atomicity of the metadata commit lives here.  The scheme is a
 * ping-pong ring of whole generations: each commit erases the next ring
 * slot, writes the dirent body, and finally writes the header block that
 * carries the sequence number and the CRC over header plus body.
 *
 * That final header write is the commit point, and it is a single
 * read/write block program -- the smallest unit the media can tear at.  A
 * power loss before it leaves a slot whose header is still erased, so the
 * generation simply does not exist.  A power loss during it leaves a header
 * that fails its own CRC.  Either way mount falls back to the previous
 * generation, which is untouched because it lives in a different slot.
 *
 * The workload is tens of modules, so a whole directory fits in one erase
 * block.  Rewriting all of it per commit is what buys the single-write
 * commit point and removes any need for journal replay logic.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <nuttx/crc32.h>
#include <nuttx/kmalloc.h>

#include "xipfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_meta_bodyblocks
 *
 * Description:
 *   Number of read/write blocks the dirent body of a generation occupies.
 *
 ****************************************************************************/

static uint32_t xipfs_meta_bodyblocks(FAR struct xipfs_mount_s *fs,
                                      uint32_t nentries)
{
  if (nentries == 0)
    {
      return 0;
    }

  return (nentries + fs->entries_per_blk - 1) / fs->entries_per_blk;
}

/****************************************************************************
 * Name: xipfs_meta_verify
 *
 * Description:
 *   Read the generation in ring slot 'slot' and verify it.  On success
 *   returns OK and reports the sequence number and entry count.
 *
 ****************************************************************************/

static int xipfs_meta_verify(FAR struct xipfs_mount_s *fs, uint32_t slot,
                             FAR uint32_t *seq, FAR uint32_t *nentries)
{
  FAR struct xipfs_genhdr_s *hdr;
  uint32_t hdrfields[3];
  uint32_t running;
  uint32_t nbody;
  uint32_t crc;
  uint32_t i;
  int ret;

  ret = xipfs_flash_read(fs, fs->meta_start + slot, 0, fs->metabuf,
                         fs->geo.blocksize);
  if (ret < 0)
    {
      return ret;
    }

  hdr = (FAR struct xipfs_genhdr_s *)fs->metabuf;
  if (hdr->magic != XIPFS_GEN_MAGIC)
    {
      return -EINVAL;
    }

  if (hdr->nentries > fs->max_entries)
    {
      return -EINVAL;
    }

  /* Snapshot the header before the body reads reuse the buffer */

  hdrfields[0] = hdr->magic;
  hdrfields[1] = hdr->seq;
  hdrfields[2] = hdr->nentries;
  crc          = hdr->crc;

  /* Recompute the CRC over the header fields followed by every body block.
   * An empty generation is legitimate and simply has no body blocks.
   */

  running = crc32part((FAR const uint8_t *)hdrfields, sizeof(hdrfields), 0);
  nbody   = xipfs_meta_bodyblocks(fs, hdrfields[2]);

  for (i = 0; i < nbody; i++)
    {
      ret = xipfs_flash_read(fs, fs->meta_start + slot,
                             (i + 1) * fs->geo.blocksize, fs->metabuf,
                             fs->geo.blocksize);
      if (ret < 0)
        {
          return ret;
        }

      running = crc32part(fs->metabuf, fs->geo.blocksize, running);
    }

  if (running != crc)
    {
      return -EINVAL;
    }

  *seq      = hdrfields[1];
  *nentries = hdrfields[2];
  return OK;
}

/****************************************************************************
 * Name: xipfs_meta_validate
 *
 * Description:
 *   Check that the entries just loaded form a tree.  Two identities must not
 *   collide, every parent must name a live directory, and following parents
 *   must reach the root -- a cycle on the medium would otherwise hang the
 *   path walk rather than merely returning the wrong answer.  Bounding the
 *   climb by the entry count is what makes that terminate.
 *
 ****************************************************************************/

static int xipfs_meta_validate(FAR struct xipfs_mount_s *fs)
{
  FAR struct xipfs_extent_s *ext;
  FAR struct xipfs_extent_s *other;
  FAR struct xipfs_extent_s *up;
  uint32_t steps;

  for (ext = fs->extents; ext != NULL; ext = ext->flink)
    {
      for (other = ext->flink; other != NULL; other = other->flink)
        {
          if (other->id == ext->id)
            {
              ferr("ERROR: Duplicate id %u ('%s' and '%s')\n",
                   ext->id, ext->name, other->name);
              return -EFTYPE;
            }

          if (other->parent == ext->parent &&
              strcmp(other->name, ext->name) == 0)
            {
              ferr("ERROR: Duplicate name '%s' in directory %u\n",
                   ext->name, ext->parent);
              return -EFTYPE;
            }
        }

      up    = ext;
      steps = 0;

      while (up->parent != XIPFS_ROOT_ID)
        {
          if (++steps > fs->nextents)
            {
              ferr("ERROR: Directory cycle above '%s'\n", ext->name);
              return -EFTYPE;
            }

          for (other = fs->extents; other != NULL; other = other->flink)
            {
              if (other->id == up->parent)
                {
                  break;
                }
            }

          if (other == NULL || !other->isdir)
            {
              ferr("ERROR: '%s' has no parent directory %u\n",
                   up->name, up->parent);
              return -EFTYPE;
            }

          up = other;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: xipfs_meta_load
 *
 * Description:
 *   Build the in-RAM entry list from the generation in ring slot 'slot'.
 *
 ****************************************************************************/

static int xipfs_meta_load(FAR struct xipfs_mount_s *fs, uint32_t slot,
                           uint32_t nentries)
{
  FAR struct xipfs_dirent_s *dirent;
  FAR struct xipfs_extent_s *ext;
  uint32_t nbody;
  uint32_t idx = 0;
  uint32_t i;
  uint32_t j;
  int ret;

  xipfs_meta_freeextents(fs);

  nbody = xipfs_meta_bodyblocks(fs, nentries);

  for (i = 0; i < nbody; i++)
    {
      ret = xipfs_flash_read(fs, fs->meta_start + slot,
                             (i + 1) * fs->geo.blocksize, fs->metabuf,
                             fs->geo.blocksize);
      if (ret < 0)
        {
          return ret;
        }

      for (j = 0; j < fs->entries_per_blk && idx < nentries; j++, idx++)
        {
          dirent = (FAR struct xipfs_dirent_s *)
                   (fs->metabuf + j * XIPFS_DIRENT_SIZE);

          bool isdir = (dirent->flags & XIPFS_DIRENT_DIR) != 0;

          /* The mount path is exactly where a corrupted medium must be
           * caught rather than trusted.  Every entry needs an identity of
           * its own, a directory must carry no extent, and a file's extent
           * must lie inside the data region.
           */

          if (dirent->id == XIPFS_ROOT_ID ||
              dirent->name[0] == '\0' ||
              strchr(dirent->name, '/') != NULL)
            {
              ferr("ERROR: Corrupt dirent, id %u name '%s'\n",
                   dirent->id, dirent->name);
              return -EFTYPE;
            }

          if (isdir)
            {
              if (dirent->start_block != 0 || dirent->nblocks != 0 ||
                  dirent->size != 0)
                {
                  ferr("ERROR: Directory '%s' carries an extent\n",
                       dirent->name);
                  return -EFTYPE;
                }
            }
          else if (dirent->start_block < fs->data_start ||
                   dirent->nblocks == 0 ||
                   dirent->start_block + dirent->nblocks >
                   fs->data_start + fs->data_nblocks)
            {
              ferr("ERROR: Corrupt dirent '%s' at %" PRIu32 "+%" PRIu32 "\n",
                   dirent->name, dirent->start_block, dirent->nblocks);
              return -EFTYPE;
            }

          ext = kmm_zalloc(sizeof(struct xipfs_extent_s));
          if (ext == NULL)
            {
              return -ENOMEM;
            }

          strlcpy(ext->name, dirent->name, sizeof(ext->name));
          ext->fs          = fs;
          ext->id          = dirent->id;
          ext->parent      = dirent->parent;
          ext->isdir       = isdir;
          ext->size        = dirent->size;
          ext->start_block = dirent->start_block;
          ext->nblocks     = dirent->nblocks;

          ext->flink   = fs->extents;
          fs->extents  = ext;
          fs->nextents++;
        }
    }

  return xipfs_meta_validate(fs);
}

/****************************************************************************
 * Name: xipfs_meta_newid
 *
 * Description:
 *   The lowest identity not in use.  Identities are stored on the medium
 *   because parent references them, so they have to survive a remount; they
 *   are reused once nothing refers to them.  Zero is reserved for the root.
 *
 ****************************************************************************/

static uint16_t xipfs_meta_newid(FAR struct xipfs_mount_s *fs)
{
  FAR struct xipfs_extent_s *ext;
  uint16_t id;

  for (id = 1; id != 0; id++)
    {
      for (ext = fs->extents; ext != NULL; ext = ext->flink)
        {
          if (ext->id == id)
            {
              break;
            }
        }

      if (ext == NULL)
        {
          return id;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_meta_freeextents
 ****************************************************************************/

void xipfs_meta_freeextents(FAR struct xipfs_mount_s *fs)
{
  FAR struct xipfs_extent_s *ext;
  FAR struct xipfs_extent_s *next;

  for (ext = fs->extents; ext != NULL; ext = next)
    {
      next = ext->flink;
      kmm_free(ext);
    }

  fs->extents  = NULL;
  fs->nextents = 0;
}

/****************************************************************************
 * Name: xipfs_meta_commit
 *
 * Description:
 *   Write the current in-RAM directory as a new generation into the next
 *   ring slot.  Returns only after the commit point has landed, so a
 *   successful return means the new state is durable and a failure means
 *   the previous generation is still the live one.
 *
 ****************************************************************************/

int xipfs_meta_commit(FAR struct xipfs_mount_s *fs)
{
  FAR struct xipfs_genhdr_s *hdr;
  FAR struct xipfs_extent_s *ext;
  FAR struct xipfs_dirent_s *dirent;
  uint32_t hdrfields[3];
  uint32_t nentries = 0;
  uint32_t slot;
  uint32_t nbody;
  uint32_t crc;
  uint32_t i;
  uint32_t j;
  int ret;

  /* Count the entries that will be committed.  Unlinked extents are
   * already detached from the list, so they are simply absent.
   */

  for (ext = fs->extents; ext != NULL; ext = ext->flink)
    {
      nentries++;
    }

  if (nentries > fs->max_entries)
    {
      return -ENOSPC;
    }

  slot  = (fs->meta_slot + 1) % fs->meta_nblocks;
  nbody = xipfs_meta_bodyblocks(fs, nentries);

  /* Erase the target slot.  Until the header write below lands, this slot
   * holds nothing valid and the live generation remains the old one.
   */

  ret = xipfs_flash_erase(fs, fs->meta_start + slot, 1);
  if (ret < 0)
    {
      return ret;
    }

  hdrfields[0] = XIPFS_GEN_MAGIC;
  hdrfields[1] = fs->meta_seq + 1;
  hdrfields[2] = nentries;
  crc = crc32part((FAR const uint8_t *)hdrfields, sizeof(hdrfields), 0);

  /* Write the body first, accumulating the CRC as we go */

  ext = fs->extents;
  for (i = 0; i < nbody; i++)
    {
      memset(fs->metabuf, 0xff, fs->geo.blocksize);

      for (j = 0; j < fs->entries_per_blk && ext != NULL; j++)
        {
          dirent = (FAR struct xipfs_dirent_s *)
                   (fs->metabuf + j * XIPFS_DIRENT_SIZE);

          memset(dirent, 0, sizeof(*dirent));
          strlcpy(dirent->name, ext->name, sizeof(dirent->name));
          dirent->size        = ext->size;
          dirent->start_block = ext->start_block;
          dirent->nblocks     = ext->nblocks;
          dirent->flags       = ext->isdir ? XIPFS_DIRENT_DIR : 0;
          dirent->id          = ext->id;
          dirent->parent      = ext->parent;

          ext = ext->flink;
        }

      crc = crc32part(fs->metabuf, fs->geo.blocksize, crc);

      ret = xipfs_flash_write(fs, fs->meta_start + slot,
                              (i + 1) * fs->geo.blocksize, fs->metabuf,
                              fs->geo.blocksize);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* The commit point: a single read/write block program */

  memset(fs->metabuf, 0xff, fs->geo.blocksize);
  hdr = (FAR struct xipfs_genhdr_s *)fs->metabuf;
  hdr->magic    = hdrfields[0];
  hdr->seq      = hdrfields[1];
  hdr->nentries = hdrfields[2];
  hdr->crc      = crc;

  ret = xipfs_flash_write(fs, fs->meta_start + slot, 0, fs->metabuf,
                          fs->geo.blocksize);
  if (ret < 0)
    {
      return ret;
    }

  fs->meta_slot = slot;
  fs->meta_seq  = hdrfields[1];
  return OK;
}

/****************************************************************************
 * Name: xipfs_meta_format
 *
 * Description:
 *   Lay down a fresh filesystem: erase the volume, write both superblock
 *   copies, then commit an empty generation.
 *
 ****************************************************************************/

int xipfs_meta_format(FAR struct xipfs_mount_s *fs)
{
  FAR struct xipfs_sblk_s *sblk;
  int ret;

  ret = xipfs_flash_erase(fs, 0, fs->geo.neraseblocks);
  if (ret < 0)
    {
      return ret;
    }

  memset(fs->metabuf, 0xff, fs->geo.blocksize);
  sblk = (FAR struct xipfs_sblk_s *)fs->metabuf;

  memset(sblk, 0, sizeof(*sblk));
  sblk->magic        = XIPFS_SBLK_MAGIC;
  sblk->version      = XIPFS_VERSION;
  sblk->erasesize    = fs->geo.erasesize;
  sblk->blocksize    = fs->geo.blocksize;
  sblk->nblocks      = fs->geo.neraseblocks;
  sblk->meta_start   = XIPFS_META_START;
  sblk->meta_nblocks = XIPFS_META_NBLOCKS;
  sblk->data_start   = XIPFS_DATA_START;
  sblk->data_nblocks = fs->geo.neraseblocks - XIPFS_DATA_START;
  sblk->crc = crc32((FAR const uint8_t *)sblk,
                    offsetof(struct xipfs_sblk_s, crc));

  ret = xipfs_flash_write(fs, XIPFS_SBLK_A, 0, fs->metabuf,
                          fs->geo.blocksize);
  if (ret < 0)
    {
      return ret;
    }

  ret = xipfs_flash_write(fs, XIPFS_SBLK_B, 0, fs->metabuf,
                          fs->geo.blocksize);
  if (ret < 0)
    {
      return ret;
    }

  /* Adopt the geometry we just wrote, then commit generation 1 */

  fs->meta_start   = XIPFS_META_START;
  fs->meta_nblocks = XIPFS_META_NBLOCKS;
  fs->data_start   = XIPFS_DATA_START;
  fs->data_nblocks = fs->geo.neraseblocks - XIPFS_DATA_START;
  fs->meta_slot    = fs->meta_nblocks - 1;
  fs->meta_seq     = 0;

  xipfs_meta_freeextents(fs);

  return xipfs_meta_commit(fs);
}

/****************************************************************************
 * Name: xipfs_meta_mount
 *
 * Description:
 *   Read the superblock, scan the metadata ring and adopt the last fully
 *   valid generation.
 *
 ****************************************************************************/

int xipfs_meta_mount(FAR struct xipfs_mount_s *fs)
{
  struct xipfs_sblk_s sblk;
  uint32_t best_slot = 0;
  uint32_t best_seq = 0;
  uint32_t best_nentries = 0;
  bool found = false;
  uint32_t copy;
  uint32_t slot;
  uint32_t seq;
  uint32_t nentries;
  int ret;

  /* Superblock: try copy A, fall back to copy B */

  ret = -EFTYPE;
  for (copy = XIPFS_SBLK_A; copy <= XIPFS_SBLK_B; copy++)
    {
      ret = xipfs_flash_read(fs, copy, 0, fs->metabuf, fs->geo.blocksize);
      if (ret < 0)
        {
          continue;
        }

      memcpy(&sblk, fs->metabuf, sizeof(sblk));

      if (sblk.magic != XIPFS_SBLK_MAGIC ||
          sblk.version != XIPFS_VERSION)
        {
          ret = -EFTYPE;
          continue;
        }

      if (crc32((FAR const uint8_t *)&sblk,
                offsetof(struct xipfs_sblk_s, crc)) != sblk.crc)
        {
          fwarn("xipfs: superblock copy %" PRIu32 " failed CRC\n", copy);
          ret = -EFTYPE;
          continue;
        }

      /* The geometry recorded at format time must still match the medium,
       * otherwise every block index in the directory means something
       * different from what it meant when it was written.
       */

      if (sblk.erasesize != fs->geo.erasesize ||
          sblk.blocksize != fs->geo.blocksize ||
          sblk.nblocks != fs->geo.neraseblocks)
        {
          ferr("ERROR: Superblock geometry does not match the medium\n");
          return -EFTYPE;
        }

      ret = OK;
      break;
    }

  if (ret < 0)
    {
      return ret;
    }

  fs->meta_start   = sblk.meta_start;
  fs->meta_nblocks = sblk.meta_nblocks;
  fs->data_start   = sblk.data_start;
  fs->data_nblocks = sblk.data_nblocks;

  if (fs->meta_nblocks < 2 ||
      fs->meta_start + fs->meta_nblocks > fs->data_start ||
      fs->data_start + fs->data_nblocks > fs->geo.neraseblocks)
    {
      ferr("ERROR: Superblock describes an inconsistent layout\n");
      return -EFTYPE;
    }

  /* Scan the ring for the highest sequence number that fully verifies */

  for (slot = 0; slot < fs->meta_nblocks; slot++)
    {
      if (xipfs_meta_verify(fs, slot, &seq, &nentries) < 0)
        {
          continue;
        }

      if (!found || seq > best_seq)
        {
          found         = true;
          best_seq      = seq;
          best_slot     = slot;
          best_nentries = nentries;
        }
    }

  if (!found)
    {
      ferr("ERROR: No valid metadata generation found\n");
      return -EFTYPE;
    }

  fs->meta_slot = best_slot;
  fs->meta_seq  = best_seq;

  finfo("xipfs: mounted generation %" PRIu32 " from slot %" PRIu32
        " with %" PRIu32 " entries\n", best_seq, best_slot, best_nentries);

  return xipfs_meta_load(fs, best_slot, best_nentries);
}

/****************************************************************************
 * Name: xipfs_meta_find
 ****************************************************************************/

FAR struct xipfs_extent_s *xipfs_meta_find(FAR struct xipfs_mount_s *fs,
                                           uint16_t parent,
                                           FAR const char *name,
                                           size_t namelen)
{
  FAR struct xipfs_extent_s *ext;

  for (ext = fs->extents; ext != NULL; ext = ext->flink)
    {
      if (ext->parent == parent &&
          strncmp(ext->name, name, namelen) == 0 &&
          ext->name[namelen] == '\0')
        {
          return ext;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: xipfs_meta_path
 *
 * Description:
 *   Compose an entry's path relative to the mountpoint by climbing its
 *   parents.  Written backwards from the end of the buffer, so a path too
 *   long to fit loses its leading components rather than the name that
 *   identifies it.  The climb is bounded by the entry count, which mount has
 *   already proved cycle-free, so this cannot spin.
 *
 *   The caller must hold the lock.
 *
 ****************************************************************************/

void xipfs_meta_path(FAR struct xipfs_mount_s *fs,
                     FAR struct xipfs_extent_s *ext,
                     FAR char *buf, size_t buflen)
{
  FAR struct xipfs_extent_s *up = ext;
  FAR struct xipfs_extent_s *cur;
  uint32_t steps = 0;
  size_t pos;

  DEBUGASSERT(buflen > 0);

  pos      = buflen - 1;
  buf[pos] = '\0';

  for (; ; )
    {
      size_t len = strlen(up->name);

      if (len > pos)
        {
          break;
        }

      pos -= len;
      memcpy(&buf[pos], up->name, len);

      if (up->parent == XIPFS_ROOT_ID || pos == 0)
        {
          break;
        }

      for (cur = fs->extents; cur != NULL; cur = cur->flink)
        {
          if (cur->id == up->parent)
            {
              break;
            }
        }

      if (cur == NULL || ++steps > fs->nextents)
        {
          break;
        }

      buf[--pos] = '/';
      up = cur;
    }

  if (pos > 0)
    {
      memmove(buf, &buf[pos], buflen - pos);
    }
}

/****************************************************************************
 * Name: xipfs_meta_haschildren
 *
 * Description:
 *   Whether anything names 'dirid' as its parent, which is what stops rmdir
 *   removing a directory that still holds something.
 *
 ****************************************************************************/

bool xipfs_meta_haschildren(FAR struct xipfs_mount_s *fs, uint16_t dirid)
{
  FAR struct xipfs_extent_s *ext;

  for (ext = fs->extents; ext != NULL; ext = ext->flink)
    {
      if (ext->parent == dirid)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: xipfs_meta_add
 *
 * Description:
 *   Attach a new extent to the in-RAM directory.  The caller commits.
 *
 ****************************************************************************/

FAR struct xipfs_extent_s *xipfs_meta_add(FAR struct xipfs_mount_s *fs,
                                          uint16_t parent,
                                          FAR const char *name,
                                          size_t namelen, bool isdir,
                                          uint32_t start_block,
                                          uint32_t nblocks, uint32_t size)
{
  FAR struct xipfs_extent_s *ext;
  uint16_t id;

  if (namelen > XIPFS_NAME_MAX)
    {
      return NULL;
    }

  id = xipfs_meta_newid(fs);
  if (id == 0)
    {
      return NULL;
    }

  ext = kmm_zalloc(sizeof(struct xipfs_extent_s));
  if (ext == NULL)
    {
      return NULL;
    }

  memcpy(ext->name, name, namelen);
  ext->name[namelen] = '\0';

  ext->fs          = fs;
  ext->id          = id;
  ext->parent      = parent;
  ext->isdir       = isdir;
  ext->start_block = start_block;
  ext->nblocks     = nblocks;
  ext->size        = size;

  ext->flink  = fs->extents;
  fs->extents = ext;
  fs->nextents++;

  return ext;
}

/****************************************************************************
 * Name: xipfs_meta_detach
 *
 * Description:
 *   Remove an extent from the in-RAM directory without freeing it.  The
 *   object stays alive while opens or pins reference it; the caller frees
 *   it once the last reference goes away.
 *
 ****************************************************************************/

void xipfs_meta_detach(FAR struct xipfs_mount_s *fs,
                       FAR struct xipfs_extent_s *ext)
{
  FAR struct xipfs_extent_s **prev;

  for (prev = &fs->extents; *prev != NULL; prev = &(*prev)->flink)
    {
      if (*prev == ext)
        {
          *prev = ext->flink;
          ext->flink = NULL;
          ext->unlinked = true;
          fs->nextents--;
          return;
        }
    }
}
