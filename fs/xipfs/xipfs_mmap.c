/****************************************************************************
 * fs/xipfs/xipfs_mmap.c
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
 * This is what distinguishes xipfs from littlefs, FAT or SPIFFS: the mmap
 * operation hands back a direct pointer into memory mapped flash rather
 * than a RAM copy, so a module's text and rodata are executed and read
 * where they already live.
 *
 * Two properties make that safe.
 *
 * First, the mapping takes a pin on the extent, and the pin lives on the
 * extent object rather than on the file descriptor.  Three running
 * instances of one module therefore produce a pin count of three, and the
 * extent only becomes movable again when the last of them goes away.
 *
 * Second, the pin is acquired under the same lock the defragmenter holds
 * while it decides whether an extent is movable.  Without that, a new
 * mapping could slip in between the compactor observing "pin count zero"
 * and the compactor starting the move, and the module would be executing
 * out of blocks that were being erased underneath it.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mman.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/mm/map.h>
#include <nuttx/sched.h>

#include "xipfs.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int xipfs_munmap(FAR struct task_group_s *group,
                        FAR struct mm_map_entry_s *entry,
                        FAR void *start, size_t length);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_munmap
 *
 * Description:
 *   Release an XIP mapping and drop its pin.
 *
 *   This runs in two situations: an explicit munmap by the module loader,
 *   and the task teardown walk that mm_map_destroy performs when a task
 *   dies.  The second case is the one that matters most -- a module that
 *   faults or is killed without unmapping must still release its pin, or
 *   the extent stays immovable forever and the defragmenter slowly stops
 *   being able to reclaim anything.
 *
 *   Because it can run from teardown, this function must not consult
 *   this_task()->group: the group is being dismantled and is passed as
 *   NULL.  Everything it needs is reachable from the entry itself.
 *
 ****************************************************************************/

static int xipfs_munmap(FAR struct task_group_s *group,
                        FAR struct mm_map_entry_s *entry,
                        FAR void *start, size_t length)
{
  FAR struct xipfs_extent_s *ext;
  FAR struct xipfs_mount_s *fs;
  int ret;

  DEBUGASSERT(entry != NULL && entry->priv.p != NULL);

  /* Only whole mappings can be released.  A partial unmap of an XIP window
   * has no meaning -- there is no allocation to shrink, only a refcount --
   * and accepting one would leave the pin count wrong in a way that is
   * very hard to trace back later.
   */

  if (start != entry->vaddr || length != entry->length)
    {
      ferr("ERROR: Partial unmap of an XIP mapping is not supported\n");
      return -EINVAL;
    }

  ext = (FAR struct xipfs_extent_s *)entry->priv.p;
  fs  = ext->fs;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      /* Failing here would leak the pin permanently, and the caller has no
       * way to retry from the teardown path.  Drop the count anyway: the
       * lock only orders us against defrag, and an unpinned-but-still-
       * mapped extent cannot arise because the mapping is going away.
       */

      fwarn("xipfs: unmap could not take the lock; dropping pin anyway\n");
    }

  DEBUGASSERT(ext->pincount > 0);
  ext->pincount--;

  /* An extent that was unlinked while mapped is freed once the last
   * reference goes away.
   */

  if (ext->unlinked && ext->pincount == 0 && ext->openrefs == 0)
    {
      xipfs_free(fs, ext->start_block, ext->nblocks);
      kmm_free(ext);
    }

  if (ret >= 0)
    {
      xipfs_unlock(fs);
    }

  return mm_map_remove(get_group_mm(group), entry);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_mmap
 *
 * Description:
 *   Map a range of a file directly onto the underlying memory mapped media.
 *
 *   The error returned when that is impossible is significant.  The core
 *   mmap path falls back to copying the file into RAM only when the file
 *   system answers -ENOTTY, so returning anything else suppresses the
 *   fallback.  A caller that passed MAP_XIP_STRICT -- a module loader, for
 *   which a silent RAM copy would defeat the entire point of executing in
 *   place -- gets -ENXIO instead, which it can turn into "defragment and
 *   retry" or into a refusal to load.  Ordinary data readers that did not
 *   ask for strict behaviour still get the convenience of the copy.
 *
 ****************************************************************************/

int xipfs_mmap(FAR struct file *filep, FAR struct mm_map_entry_s *map)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *xf;
  FAR struct xipfs_extent_s *ext;
  FAR uint8_t *addr;
  bool strict;
  int ret;

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  xf     = filep->f_priv;
  fs     = filep->f_inode->i_private;
  strict = (map->flags & MAP_XIP_STRICT) != 0;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  ext = xf->ext;

  /* A file still being written has no stable contents to map yet */

  if (ext->writing)
    {
      ret = strict ? -ENXIO : -ENOTTY;
      goto errout_with_lock;
    }

  if (map->offset < 0 || map->length == 0 ||
      (uint64_t)map->offset + map->length > ext->size)
    {
      ret = -EINVAL;
      goto errout_with_lock;
    }

  addr = xipfs_flash_addr(fs, ext->start_block);
  if (addr == NULL)
    {
      /* The media cannot be addressed directly, so there is no in-place
       * mapping to be had at any price.
       */

      ret = strict ? -ENXIO : -ENOTTY;
      goto errout_with_lock;
    }

  map->vaddr = addr + map->offset;

  /* Take the pin before releasing the lock.  From this moment the extent
   * is frozen in place: the defragmenter will skip it, so a mapping can
   * never be relocated out from under a running module.
   */

  ext->pincount++;

  /* Record what kind of mapping this is now, rather than trying to work it
   * out again at unmap time.  priv.p carries the pinned extent and offset
   * carries the mount, which is all the teardown path can rely on.
   */

  map->priv.p = ext;
  map->offset = (off_t)(uintptr_t)fs;
  map->munmap = xipfs_munmap;
  map->msync  = NULL;

  ret = mm_map_add(get_current_mm(), map);
  if (ret < 0)
    {
      ext->pincount--;
      goto errout_with_lock;
    }

  finfo("xipfs: XIP map '%s' at %p len %zu pins %" PRIu32 "\n",
        ext->name, map->vaddr, map->length, ext->pincount);

  xipfs_unlock(fs);
  return OK;

errout_with_lock:
  xipfs_unlock(fs);
  return ret;
}
