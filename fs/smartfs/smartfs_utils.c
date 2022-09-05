/****************************************************************************
 * fs/smartfs/smartfs_utils.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "smartfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WORKBUFFER_SIZE 256

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_SMARTFS_MULTI_ROOT_DIRS) || \
  (defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS))
static struct smartfs_mountpt_s *g_mounthead = NULL;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smartfs_semtake
 ****************************************************************************/

int smartfs_semtake(struct smartfs_mountpt_s *fs)
{
  return nxsem_wait_uninterruptible(fs->fs_sem);
}

/****************************************************************************
 * Name: smartfs_semgive
 ****************************************************************************/

void smartfs_semgive(struct smartfs_mountpt_s *fs)
{
  nxsem_post(fs->fs_sem);
}

/****************************************************************************
 * Name: smartfs_rdle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

uint16_t smartfs_rdle16(FAR const void *val)
{
  return (uint16_t)((FAR const uint8_t *)val)[1] << 8 |
    (uint16_t)((FAR const uint8_t *)val)[0];
}

/****************************************************************************
 * Name: smartfs_wrle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void smartfs_wrle16(FAR void *dest, uint16_t val)
{
  ((FAR uint8_t *) dest)[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  ((FAR uint8_t *) dest)[1] = val >> 8;
}

/****************************************************************************
 * Name: smartfs_rdle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint32_t representing the whole 32-bit integer value
 *
 ****************************************************************************/

uint32_t smartfs_rdle32(FAR const void *val)
{
  /* Little endian means LS halfword first in byte stream */

  return (uint32_t)smartfs_rdle16(&((FAR const uint8_t *)val)[2]) << 16 |
    (uint32_t)smartfs_rdle16(val);
}

/****************************************************************************
 * Name: smartfs_wrle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void smartfs_wrle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  smartfs_wrle16(dest, (uint16_t)(val & 0xffff));
  smartfs_wrle16(dest + 2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: smartfs_mount
 *
 * Description: This function is called only when the mountpoint is first
 *   established.  It initializes the mountpoint structure and verifies
 *   that a valid SMART filesystem is provided by the block driver.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int smartfs_mount(struct smartfs_mountpt_s *fs, bool writeable)
{
  FAR struct inode *inode;
  struct geometry geo;
  int ret = OK;
#if defined(CONFIG_SMARTFS_MULTI_ROOT_DIRS)
  struct smartfs_mountpt_s *nextfs;
#endif

  /* Assume that the mount is not successful */

  fs->fs_mounted = false;

  /* Check if there is media available */

  inode = fs->fs_blkdriver;
  if (!inode || !inode->u.i_bops || !inode->u.i_bops->geometry ||
      inode->u.i_bops->geometry(inode, &geo) != OK || !geo.geo_available)
    {
      ret = -ENODEV;
      goto errout;
    }

  /* Make sure that that the media is write-able
   * (if write access is needed)
   */

  if (writeable && !geo.geo_writeenabled)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Get the SMART low-level format information to validate the device has
   * been formatted and scan properly for logical to physical sector mapping.
   */

  ret = FS_IOCTL(fs, BIOC_GETFORMAT, (unsigned long) &fs->fs_llformat);
  if (ret != OK)
    {
      ferr("ERROR: Error getting device low level format: %d\n", ret);
      goto errout;
    }

  /* Validate the low-level format is valid */

  if (!(fs->fs_llformat.flags & SMART_FMT_ISFORMATTED))
    {
      ferr("ERROR: No low-level format found\n");
      ret = -ENODEV;
      goto errout;
    }

  /* Allocate a read/write buffer */

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  /* Scan linked list of mounted file systems to find another FS with
   * the same blockdriver.  We will reuse the buffers.
   */

  nextfs = g_mounthead;
  while (nextfs != NULL)
    {
      /* Test if this FS uses the same block driver */

      if (nextfs->fs_blkdriver == fs->fs_blkdriver)
        {
          /* Yep, it's the same block driver.  Reuse the buffers.
           * we can do this because we are protected by the same
           * semaphore.
           */

          fs->fs_rwbuffer = nextfs->fs_rwbuffer;
          fs->fs_workbuffer = nextfs->fs_workbuffer;
          break;
        }

      /* Advance to next FS */

      nextfs = nextfs->fs_next;
    }

  /* If we didn't find a FS above, then allocate some buffers */

  if (nextfs == NULL)
    {
      fs->fs_rwbuffer = (char *) kmm_malloc(fs->fs_llformat.availbytes);
      fs->fs_workbuffer = (char *) kmm_malloc(WORKBUFFER_SIZE);
    }

  /* Now add ourselves to the linked list of SMART mounts */

  fs->fs_next = g_mounthead;
  g_mounthead = fs;

  /* Set our root directory sector based on the directory entry
   * reported by the block driver (based on which device is
   * associated with this mount.
   */

  fs->fs_rootsector = SMARTFS_ROOT_DIR_SECTOR + fs->fs_llformat.rootdirnum;

#else  /* CONFIG_SMARTFS_MULTI_ROOT_DIRS */
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  /* Now add ourselves to the linked list of SMART mounts */

  fs->fs_next = g_mounthead;
  g_mounthead = fs;
#endif

  fs->fs_rwbuffer = (char *) kmm_malloc(fs->fs_llformat.availbytes);
  fs->fs_workbuffer = (char *) kmm_malloc(WORKBUFFER_SIZE);
  fs->fs_rootsector = SMARTFS_ROOT_DIR_SECTOR;

#endif /* CONFIG_SMARTFS_MULTI_ROOT_DIRS */

  /* We did it! */

  fs->fs_mounted = TRUE;

  finfo("SMARTFS:\n");
  finfo("\t    Sector size:     %d\n", fs->fs_llformat.sectorsize);
  finfo("\t    Bytes/sector     %d\n", fs->fs_llformat.availbytes);
  finfo("\t    Num sectors:     %d\n", fs->fs_llformat.nsectors);
  finfo("\t    Free sectors:    %d\n", fs->fs_llformat.nfreesectors);
  finfo("\t    Max filename:    %d\n", CONFIG_SMARTFS_MAXNAMLEN);
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  finfo("\t    RootDirEntries:  %d\n", fs->fs_llformat.nrootdirentries);
#endif
  finfo("\t    RootDirSector:   %d\n", fs->fs_rootsector);

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_unmount
 *
 * Description: This function is called only when the mountpoint is being
 *   unbound.  If we are serving multiple directories, then we have to
 *   remove ourselves from the mount linked list, and potentially free
 *   the shared buffers.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int smartfs_unmount(struct smartfs_mountpt_s *fs)
{
  int           ret = OK;
  struct inode *inode;
#if defined(CONFIG_SMARTFS_MULTI_ROOT_DIRS) || \
  (defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS))
  struct smartfs_mountpt_s *nextfs;
  struct smartfs_mountpt_s *prevfs;
  int           count = 0;
  int           found = FALSE;
#endif

#if defined(CONFIG_SMARTFS_MULTI_ROOT_DIRS) || \
  (defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS))
  /* Start at the head of the mounts and search for our entry.  Also
   * count the number of entries that match our blkdriver.
   */

  nextfs = g_mounthead;
  prevfs = NULL;
  while (nextfs != NULL)
    {
      /* Test if this FS's blkdriver matches ours (it could be us) */

      if (nextfs->fs_blkdriver == fs->fs_blkdriver)
        count++;

      /* Test if this entry is our's */

      if (nextfs == fs)
        {
          found = TRUE;
        }

      /* Keep track of the previous entry until our's is found */

      if (!found)
        {
          /* Save this entry as the previous entry */

          prevfs = nextfs;
        }

      /* Advance to the next entry */

      nextfs = nextfs->fs_next;
    }

  /* Ensure we found our FS */

  if (!found)
    {
      /* Our entry not found!  Invalid unmount or bug somewhere */

      return -EINVAL;
    }

  /* If the count is only one, then we need to delete the shared
   * buffers because we are the last ones.
   */

  if (count == 1)
    {
      /* Close the block driver */

      if (fs->fs_blkdriver)
        {
          inode = fs->fs_blkdriver;
          if (inode)
            {
              if (inode->u.i_bops && inode->u.i_bops->close)
                {
                  inode->u.i_bops->close(inode);
                }
            }
        }

      /* Free the buffers */

      kmm_free(fs->fs_rwbuffer);
      kmm_free(fs->fs_workbuffer);

      /* Set the buffer's to invalid value to catch program bugs */

      fs->fs_rwbuffer = (char *) 0xdeadbeef;
      fs->fs_workbuffer = (char *) 0xdeadbeef;
    }

  /* Now removed ourselves from the linked list */

  if (fs == g_mounthead)
    {
      /* We were the first ones.  Set a new head */

      g_mounthead = fs->fs_next;
    }
  else
    {
      /* Remove from the middle of the list somewhere */

      prevfs->fs_next = fs->fs_next;
    }
#else
  if (fs->fs_blkdriver)
    {
     inode = fs->fs_blkdriver;
      if (inode)
        {
          if (inode->u.i_bops && inode->u.i_bops->close)
            {
              inode->u.i_bops->close(inode);
            }
        }
    }

  /* Release the mountpoint private data */

  kmm_free(fs->fs_rwbuffer);
  kmm_free(fs->fs_workbuffer);
#endif

  return ret;
}

/****************************************************************************
 * Name: smartfs_finddirentry
 *
 * Description: Finds an entry in the filesystem as specified by relpath.
 *              If found, the direntry will be populated with information
 *              for accessing the entry.
 *
 *              If the final directory segment of relpath just before the
 *              last segment (the target file/dir) is valid, then the
 *              parentdirsector will indicate the logical sector number of
 *              the parent directory where a new entry should be created,
 *              and the filename pointer will point to the final segment
 *              (i.e. the "filename").
 *
 ****************************************************************************/

int smartfs_finddirentry(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *direntry, const char *relpath,
        uint16_t *parentdirsector, const char **filename)
{
  int ret = -ENOENT;
  const char *segment;
  const char *ptr;
  uint16_t    seglen;
  uint16_t    depth = 0;
  uint16_t    dirstack[CONFIG_SMARTFS_DIRDEPTH];
  uint16_t    dirsector;
  uint16_t    entrysize;
  uint16_t    offset;
  struct      smartfs_chain_header_s *header;
  struct      smart_read_write_s readwrite;
  struct      smartfs_entry_header_s *entry;

  /* Set the initial value of the output */

  *parentdirsector = 0xffff;
  *filename = NULL;

  /* Initialize directory level zero as the root sector */

  dirstack[0] = fs->fs_rootsector;
  entrysize = sizeof(struct smartfs_entry_header_s) +
              fs->fs_llformat.namesize;

  /* Test if this is a request for the root directory */

  if (*relpath == '\0')
    {
      direntry->firstsector = fs->fs_rootsector;
      direntry->flags = SMARTFS_DIRENT_TYPE_DIR | 0777;
      direntry->utc = 0;
      direntry->dsector = 0;
      direntry->doffset = 0;
      direntry->dfirst = fs->fs_rootsector;
      direntry->name = NULL;
      direntry->datlen = 0;

      *parentdirsector = 0;    /* Our parent is the format sector I guess */
      return OK;
    }

  /* Parse through each segment of relpath */

  segment = relpath;
  while (segment != NULL && *segment != '\0')
    {
      /* Find the end of this segment.  It will be '/' or NULL. */

      ptr = segment;
      seglen = 0;
      while (*ptr != '/' && *ptr != '\0')
        {
          seglen++;
          ptr++;
        }

      /* Check to avoid buffer overflow */

      if (seglen >= WORKBUFFER_SIZE)
        {
          ret = -ENAMETOOLONG;
          goto errout;
        }

      strlcpy(fs->fs_workbuffer, segment, seglen + 1);

      /* Search for "." and ".." as segment names */

      if (strcmp(fs->fs_workbuffer, ".") == 0)
        {
          /* Just ignore this segment.  Advance ptr if not on NULL */

          if (*ptr == '/')
            {
              ptr++;
            }

          segment = ptr;
          continue;
        }
      else if (strcmp(fs->fs_workbuffer, "..") == 0)
        {
          /* Up one level */

          if (depth == 0)
            {
              /* We went up one level past our mount point! */

              goto errout;
            }

          /* "Pop" to the previous directory level */

          depth--;
          if (*ptr == '/')
            {
              ptr++;
            }

          segment = ptr;
          continue;
        }
      else
        {
          /* Search for the entry in the current directory */

          dirsector = dirstack[depth];

          /* Read the directory */

          offset = 0xffff;

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
          while (dirsector != 0xffff)
#else
          while (dirsector != 0)
#endif
            {
              /* Read the next directory in the chain */

              readwrite.logsector = dirsector;
              readwrite.count = fs->fs_llformat.availbytes;
              readwrite.buffer = (uint8_t *)fs->fs_rwbuffer;
              readwrite.offset = 0;
              ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
              if (ret < 0)
                {
                  goto errout;
                }

              /* Point to next sector in chain */

              header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;
              dirsector = SMARTFS_NEXTSECTOR(header);

              /* Search for the entry */

              offset = sizeof(struct smartfs_chain_header_s);
              entry = (struct smartfs_entry_header_s *)
                &fs->fs_rwbuffer[offset];
              while (offset < readwrite.count)
                {
                  /* Test if this entry is valid and active */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                  if (((smartfs_rdle16(&entry->flags) &
                        SMARTFS_DIRENT_EMPTY) ==
                      (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) ||
                      ((smartfs_rdle16(&entry->flags)
                        & SMARTFS_DIRENT_ACTIVE) !=
                      (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#else
                  if (((entry->flags & SMARTFS_DIRENT_EMPTY) ==
                      (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) ||
                      ((entry->flags & SMARTFS_DIRENT_ACTIVE) !=
                      (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#endif
                    {
                      /* This entry isn't valid, skip it */

                      offset += entrysize;
                      entry = (struct smartfs_entry_header_s *)
                        &fs->fs_rwbuffer[offset];

                      continue;
                    }

                  /* Test if the name matches */

                  if (strncmp(entry->name, fs->fs_workbuffer,
                      fs->fs_llformat.namesize) == 0)
                    {
                      /* We found it!  If this is the last segment entry,
                       * then report the entry.  If it isn't the last
                       * entry, then validate it is a directory entry and
                       * open it and continue searching.
                       */

                      if (*ptr == '\0')
                        {
                          /* We are at the last segment.  Report the entry */

                          /* Fill in the entry */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                          direntry->firstsector =
                            smartfs_rdle16(&entry->firstsector);
                          direntry->flags = smartfs_rdle16(&entry->flags);
                          direntry->utc = smartfs_rdle32(&entry->utc);
#else
                          direntry->firstsector = entry->firstsector;
                          direntry->flags = entry->flags;
                          direntry->utc = entry->utc;
#endif
                          direntry->dsector = readwrite.logsector;
                          direntry->doffset = offset;
                          direntry->dfirst = dirstack[depth];
                          if (direntry->name == NULL)
                            {
                              direntry->name = (FAR char *)
                                kmm_malloc(fs->fs_llformat.namesize + 1);
                            }

                          strlcpy(direntry->name, entry->name,
                                  fs->fs_llformat.namesize + 1);
                          direntry->datlen = 0;

                          /* Scan the file's sectors to calculate the length
                           * and perform a rudimentary check.
                           */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                          if ((smartfs_rdle16(&entry->flags) &
                                SMARTFS_DIRENT_TYPE) ==
                              SMARTFS_DIRENT_TYPE_FILE)
#else
                          if ((entry->flags & SMARTFS_DIRENT_TYPE) ==
                              SMARTFS_DIRENT_TYPE_FILE)
#endif
                            {
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                              dirsector =
                                smartfs_rdle16(&entry->firstsector);
#else
                              dirsector = entry->firstsector;
#endif
                              readwrite.count =
                                sizeof(struct smartfs_chain_header_s);
                              readwrite.buffer = (uint8_t *)fs->fs_rwbuffer;
                              readwrite.offset = 0;

                              while (dirsector != SMARTFS_ERASEDSTATE_16BIT)
                                {
                                  /* Read the next sector of the file */

                                  readwrite.logsector = dirsector;
                                  ret = FS_IOCTL(fs, BIOC_READSECT,
                                                 (unsigned long) &readwrite);
                                  if (ret < 0)
                                    {
                                      ferr("ERROR: Error in sector"
                                           " chain at %d!\n", dirsector);
                                      break;
                                    }

                                  /* Add used bytes to the total and point
                                   * to next sector
                                   */

                                  if (SMARTFS_USED(header) !=
                                      SMARTFS_ERASEDSTATE_16BIT)
                                    {
                                      direntry->datlen +=
                                        SMARTFS_USED(header);
                                    }

                                  dirsector = SMARTFS_NEXTSECTOR(header);
                                }
                            }

                          *parentdirsector = dirstack[depth];
                          *filename = segment;
                          ret = OK;
                          goto errout;
                        }
                      else
                        {
                          /* Validate it's a directory */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                          if ((smartfs_rdle16(&entry->flags) &
                               SMARTFS_DIRENT_TYPE) !=
                              SMARTFS_DIRENT_TYPE_DIR)
#else
                          if ((entry->flags & SMARTFS_DIRENT_TYPE) !=
                              SMARTFS_DIRENT_TYPE_DIR)
#endif
                            {
                              /* Not a directory!  Report the error */

                              ret = -ENOTDIR;
                              goto errout;
                            }

                          /* "Push" the directory and continue searching */

                          if (depth >= CONFIG_SMARTFS_DIRDEPTH - 1)
                            {
                              /* Directory depth too big */

                              ret = -ENAMETOOLONG;
                              goto errout;
                            }

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                          dirstack[++depth] =
                            smartfs_rdle16(&entry->firstsector);
#else
                          dirstack[++depth] = entry->firstsector;
#endif
                          segment = ptr + 1;
                          break;
                        }
                    }

                  /* Not this entry.  Skip to the next one */

                  offset += entrysize;
                  entry = (struct smartfs_entry_header_s *)
                    &fs->fs_rwbuffer[offset];
                }

              /* Test if a directory entry was found and break if it was */

              if (offset < readwrite.count)
                {
                  break;
                }
            }

          /* If we found a dir entry, then continue searching */

          if (offset < readwrite.count)
            {
              /* Update the segment pointer */

              if (*ptr != '\0')
                {
                  ptr++;
                }

              segment = ptr;
              continue;
            }

          /* Entry not found!  Report the error.  Also, if this is the last
           * segment, then report the parent directory sector.
           */

          if (*ptr == '\0')
            {
              *parentdirsector = dirstack[depth];
              *filename = segment;
            }
          else
            {
              *parentdirsector = 0xffff;
              *filename = NULL;
            }

          ret = -ENOENT;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_createentry
 *
 * Description: Creates a new entry in the specified parent directory, using
 *              the specified type and name.  If the given sectorno is
 *              0xffff, then a new sector is allocated for the new entry,
 *              otherwise the supplied sectorno is used.
 *
 ****************************************************************************/

int smartfs_createentry(FAR struct smartfs_mountpt_s *fs,
                        uint16_t parentdirsector,
                        FAR const char *filename,
                        uint16_t type, mode_t mode,
                        FAR struct smartfs_entry_s *direntry,
                        uint16_t sectorno,
                        FAR struct smartfs_ofile_s *sf)
{
  struct    smart_read_write_s readwrite;
  int       ret;
  uint16_t  psector;
  uint16_t  nextsector;
  uint16_t  offset;
  uint16_t  found;
  uint16_t  entrysize;
  struct    smartfs_entry_header_s *entry;
  struct    smartfs_chain_header_s *chainheader;
  int       update_chain = 0;
  struct    smart_read_write_s     update_readwrite;
  struct    smartfs_chain_header_s update_header;

  /* Start at the 1st sector in the parent directory */

  psector = parentdirsector;
  found = FALSE;
  entrysize = sizeof(struct smartfs_entry_header_s) +
              fs->fs_llformat.namesize;

  /* Validate the name isn't too long */

  if (strlen(filename) > fs->fs_llformat.namesize)
    {
      return -ENAMETOOLONG;
    }

  /* Read the parent directory sector and find a place to insert
   * the new entry.
   */

  while (1)
    {
      /* Read the next sector */

      readwrite.logsector = psector;
      readwrite.count = fs->fs_llformat.availbytes;
      readwrite.offset = 0;
      readwrite.buffer = (uint8_t *) fs->fs_rwbuffer;
      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          goto errout;
        }

      /* Get the next chained sector */

      chainheader = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;
      nextsector = SMARTFS_NEXTSECTOR(chainheader);

      /* Search for an empty entry in this sector */

      offset = sizeof(struct smartfs_chain_header_s);
      entry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
      while (offset + entrysize < readwrite.count)
        {
          /* Check if this entry is available */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
          if ((smartfs_rdle16(&entry->flags) == SMARTFS_ERASEDSTATE_16BIT) ||
              ((smartfs_rdle16(&entry->flags) &
#else
          if ((entry->flags == SMARTFS_ERASEDSTATE_16BIT) ||
              ((entry->flags &
#endif
                (SMARTFS_DIRENT_EMPTY | SMARTFS_DIRENT_ACTIVE)) ==
               (~SMARTFS_ERASEDSTATE_16BIT &
                (SMARTFS_DIRENT_EMPTY | SMARTFS_DIRENT_ACTIVE))))
            {
              /* We found an empty entry.  Use it. */

              found = TRUE;
              break;
            }

          /* Not available.  Skip to next entry */

          offset += entrysize;
          entry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
        }

      /* If we found an entry, stop the search */

      if (found)
        {
          break;
        }

      /* If there are no more sectors, then we need to add one to make
       * room for the new entry.
       */

      if (nextsector == SMARTFS_ERASEDSTATE_16BIT)
        {
          /* Allocate a new sector and chain it to the last one */

          ret = FS_IOCTL(fs, BIOC_ALLOCSECT, 0xffff);
          if (ret < 0)
            {
              goto errout;
            }

          nextsector = (uint16_t) ret;

          /* Chain the next sector into this sector. */

          *((uint16_t *)update_header.nextsector) = nextsector;
          update_readwrite.logsector = psector;
          update_readwrite.offset = offsetof(struct smartfs_chain_header_s,
                                             nextsector);
          update_readwrite.count = sizeof(uint16_t);
          update_readwrite.buffer = update_header.nextsector;
          update_chain = 1;
        }

      /* Now update to the next sector */

      psector = nextsector;
    }

  /* We found an insertion point.  Create the entry at sector,offset */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&entry->flags, (uint16_t) (SMARTFS_DIRENT_ACTIVE |
        SMARTFS_DIRENT_DELETING | SMARTFS_DIRENT_RESERVED | type | (mode &
        SMARTFS_DIRENT_MODE)));
#else
  entry->flags = (uint16_t) (SMARTFS_DIRENT_ACTIVE |
        SMARTFS_DIRENT_DELETING | SMARTFS_DIRENT_RESERVED | type | (mode &
        SMARTFS_DIRENT_MODE));
#endif
#else   /* CONFIG_SMARTFS_ERASEDSTATE == 0xff */
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&entry->flags, (uint16_t) (SMARTFS_DIRENT_EMPTY | type |
        (mode & SMARTFS_DIRENT_MODE)));
#else
  entry->flags = (uint16_t) (SMARTFS_DIRENT_EMPTY | type |
        (mode & SMARTFS_DIRENT_MODE));
#endif
#endif /* CONFIG_SMARTFS_ERASEDSTATE == 0xff */

  if (sectorno == 0xffff)
    {
      /* Allocate a new sector for the file / dir */

      ret = FS_IOCTL(fs, BIOC_ALLOCSECT, 0xffff);
      if (ret < 0)
        {
          goto errout;
        }

      nextsector = (uint16_t) ret;

      /* Set the newly allocated sector's type (file or dir) */

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER
      if (sf)
        {
          /* Using sector buffer and we have an open file context.
           * Just update the sector buffer in the open file context.
           */

          memset(sf->buffer, CONFIG_SMARTFS_ERASEDSTATE,
                 fs->fs_llformat.availbytes);
          chainheader = (struct smartfs_chain_header_s *) sf->buffer;
          chainheader->type = SMARTFS_SECTOR_TYPE_FILE;
          sf->bflags = SMARTFS_BFLAG_DIRTY | SMARTFS_BFLAG_NEWALLOC;
        }
      else
#endif
        {
          if ((type & SMARTFS_DIRENT_TYPE) == SMARTFS_DIRENT_TYPE_DIR)
            {
              chainheader->type = SMARTFS_SECTOR_TYPE_DIR;
            }
          else
            {
              chainheader->type = SMARTFS_SECTOR_TYPE_FILE;
            }

          readwrite.count = 1;
          readwrite.offset = offsetof(struct smartfs_chain_header_s, type);
          readwrite.buffer = (uint8_t *) &chainheader->type;
          readwrite.logsector = nextsector;
          ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
          if (ret < 0)
            {
              ferr("ERROR: Error %d setting new sector type for sector %d\n",
                   ret, nextsector);
              goto errout;
            }
        }
    }
  else
    {
      /* Use the provided sector number */

      nextsector = sectorno;
    }

  /* Create the directory entry to be written in the parent's sector */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&entry->firstsector, nextsector);
  smartfs_wrle16(&entry->utc, time(NULL));
#else
  entry->firstsector = nextsector;
  entry->utc = time(NULL);
#endif
  memset(entry->name, 0, fs->fs_llformat.namesize);
  strlcpy(entry->name, filename, fs->fs_llformat.namesize);

  /* Now write the new entry to the parent directory sector */

  readwrite.logsector = psector;
  readwrite.offset = offset;
  readwrite.count = entrysize;
  readwrite.buffer = (uint8_t *) &fs->fs_rwbuffer[offset];
  ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
  if (ret < 0)
    {
      goto errout;
    }

  if (update_chain)
    {
      /* Update chain header after the next sector was written */

      ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &update_readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error chaining sector %d\n",
               update_readwrite.logsector);
          goto errout;
        }
    }

  /* Now fill in the entry */

  direntry->firstsector = nextsector;
  direntry->dsector = psector;
  direntry->doffset = offset;
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  direntry->flags = smartfs_rdle16(&entry->flags);
  direntry->utc = smartfs_rdle32(&entry->utc);
#else
  direntry->flags = entry->flags;
  direntry->utc = entry->utc;
#endif
  direntry->datlen = 0;
  if (direntry->name == NULL)
    {
      direntry->name = (FAR char *)kmm_malloc(fs->fs_llformat.namesize + 1);
    }

  memset(direntry->name, 0, fs->fs_llformat.namesize + 1);
  strlcpy(direntry->name, filename, fs->fs_llformat.namesize);

  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_deleteentry
 *
 * Description: Deletes an entry from the filesystem (file or dir) by
 *              freeing all the entry's sectors and then marking it inactive
 *              in it's parent's directory sector.  For a directory, it
 *              does not validate the directory is empty, nor does it do
 *              a recursive delete.
 *
 ****************************************************************************/

int smartfs_deleteentry(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *entry)
{
  int                             ret;
  uint16_t                        nextsector;
  uint16_t                        sector;
  uint16_t                        count;
  uint16_t                        entrysize;
  uint16_t                        offset;
  struct smartfs_entry_header_s  *direntry;
  struct smartfs_chain_header_s  *header;
  struct smart_read_write_s       readwrite;

  /* Okay, delete the file.  Loop through each sector and release them
   *
   * TODO:  We really should walk the list backward to avoid lost
   *        sectors in the event we lose power. However this requires
   *        allocating a buffer to build the sector list since we don't
   *        store a doubly-linked list of sectors on the device.  We
   *        could test if the sector data buffer is big enough and
   *        just use that, and only allocate a new buffer if the
   *        sector buffer isn't big enough.  Do do this, however, we
   *        need to change the code below as it is using the a few
   *        bytes of the buffer to read in header info.
   */

  nextsector = entry->firstsector;
  header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;
  readwrite.offset = 0;
  readwrite.count = sizeof(struct smartfs_chain_header_s);
  readwrite.buffer = (uint8_t *) fs->fs_rwbuffer;
  while (nextsector != SMARTFS_ERASEDSTATE_16BIT)
    {
      /* Read the next sector into our buffer */

      sector = nextsector;
      readwrite.logsector = sector;
      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error reading sector %d\n", nextsector);
          break;
        }

      /* Release this sector */

      nextsector = SMARTFS_NEXTSECTOR(header);
      ret = FS_IOCTL(fs, BIOC_FREESECT, sector);
    }

  /* Remove the entry from the directory tree */

  readwrite.logsector = entry->dsector;
  readwrite.offset = 0;
  readwrite.count = fs->fs_llformat.availbytes;
  readwrite.buffer = (uint8_t *) fs->fs_rwbuffer;
  ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
  if (ret < 0)
    {
      ferr("ERROR: Error reading directory info at sector %d\n",
           entry->dsector);
      goto errout;
    }

  /* Mark this entry as inactive */

  direntry = (struct smartfs_entry_header_s *)
    &fs->fs_rwbuffer[entry->doffset];
#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&direntry->flags,
                 smartfs_rdle16(&direntry->flags) & ~SMARTFS_DIRENT_ACTIVE);
#else
  direntry->flags &= ~SMARTFS_DIRENT_ACTIVE;
#endif
#else   /* CONFIG_SMARTFS_ERASEDSTATE == 0xff */
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&direntry->flags,
                 smartfs_rdle16(&direntry->flags) | SMARTFS_DIRENT_ACTIVE);
#else
  direntry->flags |= SMARTFS_DIRENT_ACTIVE;
#endif
#endif /* CONFIG_SMARTFS_ERASEDSTATE == 0xff */

  /* Write the updated flags back to the sector */

  readwrite.offset = entry->doffset;
  readwrite.count = sizeof(uint16_t);
  readwrite.buffer = (uint8_t *) &direntry->flags;
  ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
  if (ret < 0)
    {
      ferr("ERROR: Error marking entry inactive at sector %d\n",
           entry->dsector);
      goto errout;
    }

  /* Test if any entries in this sector are being used */

  if ((entry->dsector != fs->fs_rootsector) &&
      (entry->dsector != entry->dfirst))
    {
      /* Scan the sector and count used entries */

      count = 0;
      offset = sizeof(struct smartfs_chain_header_s);
      entrysize = sizeof(struct smartfs_entry_header_s) +
                  fs->fs_llformat.namesize;
      while (offset + entrysize < fs->fs_llformat.availbytes)
        {
          /* Test the next entry */

          direntry = (struct smartfs_entry_header_s *)
            &fs->fs_rwbuffer[offset];
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
          if (((smartfs_rdle16(&direntry->flags) & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((smartfs_rdle16(&direntry->flags) & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#else
          if (((direntry->flags & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((direntry->flags & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#endif
            {
              /* Count this entry */

              count++;
            }

          /* Advance to next entry */

          offset += entrysize;
        }

      /* Test if the count it zero.
       * If it is, then we will release the sector
       */

      if (count == 0)
        {
          /* Okay, to release the sector, we must find the sector that we
           * are chained to and remove ourselves from the chain.  First
           * save our nextsector value so we can "unchain" ourselves.
           */

          nextsector = SMARTFS_NEXTSECTOR(header);

          /* Now loop through the dir sectors to find ourselves in the
           * chain
           */

          sector = entry->dfirst;
          readwrite.offset = 0;
          readwrite.count = sizeof(struct smartfs_chain_header_s);
          readwrite.buffer = (uint8_t *) fs->fs_rwbuffer;
          while (sector != SMARTFS_ERASEDSTATE_16BIT)
            {
              /* Read the header for the next sector */

              readwrite.logsector = sector;
              ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
              if (ret < 0)
                {
                  ferr("ERROR: Error reading sector %d\n", nextsector);
                  break;
                }

              /* Test if this sector "points" to us */

              if (SMARTFS_NEXTSECTOR(header) == entry->dsector)
                {
                  /* We found ourselves in the chain.  Update the chain. */

                  SMARTFS_SET_NEXTSECTOR(header, nextsector);
                  readwrite.offset = offsetof(struct smartfs_chain_header_s,
                                              nextsector);
                  readwrite.count  = sizeof(uint16_t);
                  readwrite.buffer = header->nextsector;

                  ret = FS_IOCTL(fs, BIOC_WRITESECT,
                                 (unsigned long)&readwrite);
                  if (ret < 0)
                    {
                      ferr("ERROR: Error unchaining sector (%d)\n",
                            nextsector);
                      goto errout;
                    }

                  /* Now release our sector */

                  ret = FS_IOCTL(fs, BIOC_FREESECT,
                                 (unsigned long)entry->dsector);
                  if (ret < 0)
                    {
                      ferr("ERROR: Error freeing sector %d\n",
                            entry->dsector);
                      goto errout;
                    }

                  /* Break out of the loop, we are done! */

                  break;
                }

              /* Chain to the next sector */

              sector = SMARTFS_NEXTSECTOR(header);
            }
        }
    }

  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_countdirentries
 *
 * Description: Counts the number of items in the specified directory entry.
 *              This routine assumes you have validated the entry you are
 *              passing is in fact a directory sector, though it checks
 *              just in case you were stupid :-)
 *
 ****************************************************************************/

int smartfs_countdirentries(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *entry)
{
  int                             ret;
  uint16_t                        nextsector;
  uint16_t                        offset;
  uint16_t                        entrysize;
  int                             count;
  struct smartfs_entry_header_s  *direntry;
  struct smartfs_chain_header_s  *header;
  struct smart_read_write_s       readwrite;

  /* Walk through the directory's sectors and count entries */

  count = 0;
  nextsector = entry->firstsector;
  while (nextsector != SMARTFS_ERASEDSTATE_16BIT)
    {
      /* Read the next sector into our buffer */

      readwrite.logsector = nextsector;
      readwrite.offset = 0;
      readwrite.count = fs->fs_llformat.availbytes;
      readwrite.buffer = (uint8_t *) fs->fs_rwbuffer;
      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error reading sector %d\n", nextsector);
          break;
        }

      /* Validate this is a directory type sector */

      header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;
      if (header->type != SMARTFS_SECTOR_TYPE_DIR)
        {
          ferr("ERROR: Sector %d is not a DIR sector!\n", nextsector);
          goto errout;
        }

      /* Loop for all entries in this sector and count them */

      offset = sizeof(struct smartfs_chain_header_s);
      entrysize = sizeof(struct smartfs_entry_header_s) +
        fs->fs_llformat.namesize;
      direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
      while (offset + entrysize < readwrite.count)
        {
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
          if (((smartfs_rdle16(&direntry->flags) & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((smartfs_rdle16(&direntry->flags) & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#else
          if (((direntry->flags & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((direntry->flags & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#endif
            {
              /* Count this entry */

              count++;
            }

          offset += entrysize;
          direntry = (struct smartfs_entry_header_s *)
            &fs->fs_rwbuffer[offset];
        }

      /* Get the next sector from the header */

      nextsector = SMARTFS_NEXTSECTOR(header);
    }

  ret = count;

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_sync_internal
 *
 * Description:
 *   Synchronize the file state on disk to match internal, in-memory state.
 *
 ****************************************************************************/

int smartfs_sync_internal(FAR struct smartfs_mountpt_s *fs,
                          FAR struct smartfs_ofile_s *sf)
{
  FAR struct smartfs_chain_header_s *header;
  struct smart_read_write_s readwrite;
  int ret = OK;

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER
  if (sf->bflags & SMARTFS_BFLAG_DIRTY)
    {
      /* Update the header with the number of bytes written */

      header = (struct smartfs_chain_header_s *)sf->buffer;

      if (SMARTFS_USED(header) == SMARTFS_ERASEDSTATE_16BIT)
        {
          SMARTFS_SET_USED(header, sf->byteswritten);
        }
      else
        {
          SMARTFS_SET_USED(header, SMARTFS_USED(header)
                                   + sf->byteswritten);
        }

      /* Write the entire sector to FLASH */

      readwrite.logsector = sf->currsector;
      readwrite.offset    = 0;
      readwrite.count     = fs->fs_llformat.availbytes;
      readwrite.buffer    = sf->buffer;

      ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error %d writing used bytes for sector %d\n",
               ret, sf->currsector);
          goto errout;
        }

      sf->byteswritten = 0;
      sf->bflags = 0;
    }
#else  /* CONFIG_SMARTFS_USE_SECTOR_BUFFER */

  /* Test if we have written bytes to the current sector that
   * need to be recorded in the chain header's used bytes field.
   */

  if (sf->byteswritten > 0)
    {
      finfo("Syncing sector %d\n", sf->currsector);

      /* Read the existing sector used bytes value */

      readwrite.logsector = sf->currsector;
      readwrite.offset    = 0;
      readwrite.buffer    = (uint8_t *) fs->fs_rwbuffer;
      readwrite.count     = sizeof(struct smartfs_chain_header_s);

      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error %d reading sector %d data\n",
               ret, sf->currsector);
          goto errout;
        }

      /* Add new byteswritten to existing value */

      header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;

      if (SMARTFS_USED(header) == SMARTFS_ERASEDSTATE_16BIT)
        {
          SMARTFS_SET_USED(header, sf->byteswritten);
        }
      else
        {
          SMARTFS_SET_USED(header, SMARTFS_USED(header)
                                   + sf->byteswritten);
        }

      readwrite.offset = offsetof(struct smartfs_chain_header_s, used);
      readwrite.count  = sizeof(uint16_t);
      readwrite.buffer = (uint8_t *) &fs->fs_rwbuffer[readwrite.offset];

      ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error %d writing used bytes for sector %d\n",
               ret, sf->currsector);
          goto errout;
        }

      sf->byteswritten = 0;
    }
#endif /* CONFIG_SMARTFS_USE_SECTOR_BUFFER */

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_seek_internal
 *
 * Description:
 *   Performs the logic of the seek function.  This is an internal function
 *   because it does not provide semaphore protection and therefore must be
 *   called from one of the other public interface routines (open, seek,
 *   etc.).
 *
 ****************************************************************************/

off_t smartfs_seek_internal(FAR struct smartfs_mountpt_s *fs,
                            FAR struct smartfs_ofile_s *sf,
                            off_t offset, int whence)
{
  FAR struct smartfs_chain_header_s *header;
  struct smart_read_write_s readwrite;
  off_t newpos;
  off_t sectorstartpos;
  int ret;

  /* Test if this is a seek to get the current file pos */

  if ((whence == SEEK_CUR) && (offset == 0))
    {
      return sf->filepos;
    }

  /* Test if we need to sync the file */

  if (sf->byteswritten > 0)
    {
      /* Perform a sync */

      smartfs_sync_internal(fs, sf);
    }

  /* Calculate the file position to seek to based on current position */

  switch (whence)
  {
    case SEEK_SET:
    default:
      newpos = offset;
      break;

    case SEEK_CUR:
      newpos = sf->filepos + offset;
      break;

    case SEEK_END:
      newpos = sf->entry.datlen + offset;
      break;
  }

  /* Ensure newpos is in range */

  if (newpos < 0)
    {
      newpos = 0;
    }

  if (newpos > sf->entry.datlen)
    {
      newpos = sf->entry.datlen;
    }

  /* Now perform the seek.  Test if we are seeking within the current
   * sector and can skip the search to save time.
   */

  sectorstartpos = sf->filepos - (sf->curroffset - sizeof(struct
        smartfs_chain_header_s));

  if (newpos >= sectorstartpos && newpos < sectorstartpos +
      fs->fs_llformat.availbytes - sizeof(struct smartfs_chain_header_s))
    {
      /* Seeking within the current sector.  Just update the offset */

      sf->curroffset = sizeof(struct smartfs_chain_header_s) +
                       newpos - sectorstartpos;
      sf->filepos    = newpos;

      return newpos;
    }

  /* Nope, we have to search for the sector and offset.  If the new pos is
   * greater than the current pos, then we can start from the beginning of
   * the current sector, otherwise we have to start from the beginning of
   * the file.
   */

  if (newpos > sf->filepos)
    {
      sf->filepos = sectorstartpos;
    }
  else
    {
      sf->currsector = sf->entry.firstsector;
      sf->filepos = 0;
    }

  header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;
  while ((sf->currsector != SMARTFS_ERASEDSTATE_16BIT) &&
      (sf->filepos + fs->fs_llformat.availbytes -
      sizeof(struct smartfs_chain_header_s) < newpos))
    {
      /* Read the sector's header */

      readwrite.logsector = sf->currsector;
      readwrite.offset    = 0;
      readwrite.count     = sizeof(struct smartfs_chain_header_s);
      readwrite.buffer    = (uint8_t *) fs->fs_rwbuffer;

      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error %d reading sector %d header\n",
               ret, sf->currsector);
          goto errout;
        }

      /* Point to next sector and update filepos */

      sf->currsector = SMARTFS_NEXTSECTOR(header);
      sf->filepos += SMARTFS_USED(header);
    }

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER

  /* When using sector buffering, we must read in the last buffer to our
   * sf->buffer in case any changes are made.
   */

  if (sf->currsector != SMARTFS_ERASEDSTATE_16BIT)
    {
      readwrite.logsector = sf->currsector;
      readwrite.offset = 0;
      readwrite.count = fs->fs_llformat.availbytes;
      readwrite.buffer = (uint8_t *) sf->buffer;
      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error %d reading sector %d header\n",
               ret, sf->currsector);
          goto errout;
        }
    }
#endif

  /* Now calculate the offset */

  sf->curroffset = sizeof(struct smartfs_chain_header_s) + newpos -
                   sf->filepos;
  sf->filepos = newpos;
  return newpos;

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_shrinkfile
 *
 * Description:
 *   Shrink the size of an existing file to the specified length
 *
 ****************************************************************************/

int smartfs_shrinkfile(FAR struct smartfs_mountpt_s *fs,
                       FAR struct smartfs_ofile_s *sf, off_t length)
{
  FAR struct smartfs_chain_header_s *header;
  FAR struct smartfs_entry_s *entry;
  FAR uint8_t *dest;
  struct smart_read_write_s readwrite;
  uint16_t nextsector;
  uint16_t sector;
  off_t remaining;
  off_t destsize;
  off_t available;
  off_t offset;
  int ret;

  /* Walk through the directory's sectors and count entries */

  entry      = &sf->entry;
  nextsector = entry->firstsector;
  header     = (struct smartfs_chain_header_s *)fs->fs_rwbuffer;
  remaining  = length;
  available  = fs->fs_llformat.availbytes -
               sizeof(struct smartfs_chain_header_s);

  while (nextsector != SMARTFS_ERASEDSTATE_16BIT)
    {
      /* Read the next sector into our buffer */

      readwrite.logsector = nextsector;
      readwrite.offset    = 0;
      readwrite.count     = fs->fs_llformat.availbytes;
      readwrite.buffer    = (FAR uint8_t *)fs->fs_rwbuffer;

      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          ferr("ERROR: Error reading sector %d header\n", nextsector);
          return ret;
        }

      /* Get the next chained sector */

      sector = SMARTFS_NEXTSECTOR(header);

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER
      /* When we have a sector buffer in use, simply skip the first sector.
       * It will be handled below.
       */

      if (nextsector == entry->firstsector)
        {
          if (remaining > available)
            {
              remaining -= available;
            }
          else
            {
              remaining = 0;
            }
        }
#endif

      /* Are we retaining the sector it its entirety? */

      if (remaining >= available)
        {
          /* Yes... skip to the next sector */

          remaining -= available;
        }

      /* Are we removing the sector it its entirety? */

      else if (remaining <= 0 && nextsector != entry->firstsector)
        {
          /* Yes.. just release the sector */

          ret = FS_IOCTL(fs, BIOC_FREESECT, (unsigned long)nextsector);
          if (ret < 0)
            {
              ferr("ERROR: Error freeing sector %d\n", nextsector);
              return ret;
            }
        }
      else
        {
          /* No.. Fill our buffer with erased data, retaining any still-
           * valid bytes at the beginning of the buffer.
           *
           * Because of the preceding tests we know that
           * 0 <= remaining < available.  A special case is remaining == 0
           * and nextsector == firstsector.  In that case, we need to
           * overwrite the sector data with the erased state value.  The
           * underlying SMART block driver will detect this and release the
           * old sector and create a new one with the new (blank) data.
           *
           * Otherwise, we need to preserve the header and overwrite some of
           * the data.
           */

          if (remaining == 0)
            {
              dest       = (FAR uint8_t *)fs->fs_rwbuffer;
              destsize   = fs->fs_llformat.availbytes;
            }
          else
            {
              offset     = sizeof(struct smartfs_chain_header_s) + remaining;
              dest       = (FAR uint8_t *)&fs->fs_rwbuffer[offset];
              destsize   = fs->fs_llformat.availbytes - offset;

              SMARTFS_SET_USED(header, remaining);
              SMARTFS_SET_NEXTSECTOR(header, SMARTFS_ERASEDSTATE_16BIT);
              remaining  = 0;
            }

          memset(dest, CONFIG_SMARTFS_ERASEDSTATE, destsize);
          header->type = SMARTFS_SECTOR_TYPE_FILE;

          /* Now write the new sector data */

          readwrite.count = fs->fs_llformat.availbytes;

          ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long)&readwrite);
          if (ret < 0)
            {
              ferr("ERROR: Error blanking 1st sector (%d) of file\n",
                    nextsector);
              return ret;
            }
        }

      /* Now move on to the next sector */

      nextsector = sector;
    }

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER
  /* Now deal with the first sector in the event we are using a sector buffer
   * like we would be if CRC is enabled.
   *
   * Using sector buffer and we have an open file context.  Just update the
   * sector buffer in the open file context.
   */

  if (length < fs->fs_llformat.availbytes)
    {
      /* Read the entire sector */

      readwrite.logsector = entry->firstsector;
      readwrite.offset    = 0;
      readwrite.count     = fs->fs_llformat.availbytes;
      readwrite.buffer    = (uint8_t *)sf->buffer;

      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long)&readwrite);
      if (ret < 0)
        {
          return ret;
        }

      /* Retain any valid data at the beginning of the sector, including the
       * header.  Special case length == 0
       */

      if (length == 0)
        {
          dest       = (FAR uint8_t *)sf->buffer;
          destsize   = fs->fs_llformat.availbytes;
        }
      else
        {
          offset     = sizeof(struct smartfs_chain_header_s) + length;
          dest       = (FAR uint8_t *)&sf->buffer[offset];
          destsize   = fs->fs_llformat.availbytes - offset;

          header     = (struct smartfs_chain_header_s *)sf->buffer;

          SMARTFS_SET_USED(header, length);
          SMARTFS_SET_NEXTSECTOR(header, SMARTFS_ERASEDSTATE_16BIT);
        }

      memset(dest, CONFIG_SMARTFS_ERASEDSTATE, destsize);

      header        = (struct smartfs_chain_header_s *)sf->buffer;
      header->type  = SMARTFS_SECTOR_TYPE_FILE;
      sf->bflags    = SMARTFS_BFLAG_DIRTY;
    }
#endif

  entry->datlen = length;
  return OK;
}

/****************************************************************************
 * Name: smartfs_extendfile
 *
 * Description:
 *   Zero-extend the length of a regular file to 'length'.
 *
 ****************************************************************************/

int smartfs_extendfile(FAR struct smartfs_mountpt_s *fs,
                       FAR struct smartfs_ofile_s *sf, off_t length)
{
  struct smart_read_write_s readwrite;
  FAR struct smartfs_chain_header_s *header;
#ifndef CONFIG_SMARTFS_USE_SECTOR_BUFFER
  FAR uint8_t *buffer;
#endif
  off_t remaining;
  off_t savepos;
  off_t oldsize;
  int ret;

  /* We are zero-extending the file.  This is essentially the same as a
   * write except that (1) we write zeros and (2) we don't update the file
   * position.
   */

#ifndef CONFIG_SMARTFS_USE_SECTOR_BUFFER
  /* In order to perform the writes we will have to have a sector buffer.  If
   * SmartFS is not configured with a sector buffer then we will, then we
   * will, unfortunately, need to allocate one.
   */

  buffer = (FAR uint8_t *)kmm_malloc(SMARTFS_TRUNCBUFFER_SIZE);
  if (buffer == NULL)
    {
      return -ENOMEM;
    }
#endif

  /* Loop until either (1) the file has been fully extended with zeroed data
   * or (2) an error occurs.  We assume we start with the current sector in
   * cache (ff_currentsector).
   */

  oldsize   = sf->entry.datlen;
  remaining = length - oldsize;
  DEBUGASSERT(length > oldsize);

  /* Seek to the end of the file for the append/write operation, remembering
   * the current file position.  It will be restored before returning; the
   * truncate operation must not alter the file position.
   */

  savepos = sf->filepos;
  smartfs_seek_internal(fs, sf, 0, SEEK_END);

  while (remaining > 0)
    {
      /* We will fill up the current sector. Write data to the current
       * sector first.
       */

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER
      readwrite.count = fs->fs_llformat.availbytes - sf->curroffset;
      if (readwrite.count > remaining)
        {
          readwrite.count = remaining;
        }

      memset(&sf->buffer[sf->curroffset], 0, readwrite.count);
      sf->bflags |= SMARTFS_BFLAG_DIRTY;

#else  /* CONFIG_SMARTFS_USE_SECTOR_BUFFER */
      readwrite.offset    = sf->curroffset;
      readwrite.logsector = sf->currsector;
      readwrite.buffer    = buffer;

      /* Select max size that available in the current sector */

      readwrite.count     = fs->fs_llformat.availbytes - sf->curroffset;
      if (readwrite.count > remaining)
        {
          /* Limit the write to the size for our smaller working buffer */

          readwrite.count = SMARTFS_TRUNCBUFFER_SIZE;
        }

      if (readwrite.count > remaining)
        {
          /* Further limit the write to the remaining bytes to write */

          readwrite.count = remaining;
        }

      /* Perform the write */

      if (readwrite.count > 0)
        {
          ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
          if (ret < 0)
            {
              ferr("ERROR: Error %d writing sector %d data\n",
                   ret, sf->currsector);
              goto errout_with_buffer;
            }
        }
#endif /* CONFIG_SMARTFS_USE_SECTOR_BUFFER */

      /* Update our control variables */

      sf->entry.datlen += readwrite.count;
      sf->byteswritten += readwrite.count;
      sf->curroffset   += readwrite.count;
      remaining        -= readwrite.count;

      /* Test if we wrote a full sector of data */

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER
      if (sf->curroffset == fs->fs_llformat.availbytes && remaining)
        {
          /* First get a new chained sector */

          ret = FS_IOCTL(fs, BIOC_ALLOCSECT, 0xffff);
          if (ret < 0)
            {
              ferr("ERROR: Error %d allocating new sector\n", ret);
              goto errout_with_buffer;
            }

          /* Copy the new sector to the old one and chain it */

          header = (struct smartfs_chain_header_s *) sf->buffer;
          SMARTFS_SET_NEXTSECTOR(header, ret);

          /* Now sync the file to write this sector out */

          ret = smartfs_sync_internal(fs, sf);
          if (ret != OK)
            {
              goto errout_with_buffer;
            }

          /* Record the new sector in our tracking variables and reset the
           * offset to "zero".
           */

          if (sf->currsector == SMARTFS_NEXTSECTOR(header))
            {
              /* Error allocating logical sector! */

              ferr("ERROR: Duplicate logical sector %d\n", sf->currsector);
            }

          sf->bflags     = SMARTFS_BFLAG_DIRTY;
          sf->currsector = SMARTFS_NEXTSECTOR(header);
          sf->curroffset = sizeof(struct smartfs_chain_header_s);
          memset(sf->buffer, CONFIG_SMARTFS_ERASEDSTATE,
                 fs->fs_llformat.availbytes);
          header->type   = SMARTFS_DIRENT_TYPE_FILE;
        }
#else  /* CONFIG_SMARTFS_USE_SECTOR_BUFFER */

      if (sf->curroffset == fs->fs_llformat.availbytes)
        {
          /* Sync the file to write this sector out */

          ret = smartfs_sync_internal(fs, sf);
          if (ret != OK)
            {
              goto errout_with_buffer;
            }

          /* Allocate a new sector if needed */

          if (remaining > 0)
            {
              /* Allocate a new sector */

              ret = FS_IOCTL(fs, BIOC_ALLOCSECT, 0xffff);
              if (ret < 0)
                {
                  ferr("ERROR: Error %d allocating new sector\n", ret);
                  goto errout_with_buffer;
                }

              /* Copy the new sector to the old one and chain it */

              header = (struct smartfs_chain_header_s *)fs->fs_rwbuffer;
              SMARTFS_SET_NEXTSECTOR(header, ret);

              readwrite.offset = offsetof(struct smartfs_chain_header_s,
                                          nextsector);
              readwrite.buffer = (FAR uint8_t *)header->nextsector;
              readwrite.count  = sizeof(uint16_t);

              ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
              if (ret < 0)
                {
                  ferr("ERROR: Error %d writing next sector\n", ret);
                  goto errout_with_buffer;
                }

              /* Record the new sector in our tracking variables and
               * reset the offset to "zero".
               */

              if (sf->currsector == SMARTFS_NEXTSECTOR(header))
                {
                  /* Error allocating logical sector! */

                  ferr("ERROR: Duplicate logical sector %d\n",
                        sf->currsector);
                }

              sf->currsector = SMARTFS_NEXTSECTOR(header);
              sf->curroffset = sizeof(struct smartfs_chain_header_s);
            }
        }
#endif /* CONFIG_SMARTFS_USE_SECTOR_BUFFER */
    }

  /* The file was successfully extended with zeros */

  ret = OK;

errout_with_buffer:
#ifndef CONFIG_SMARTFS_USE_SECTOR_BUFFER
  /* Release the allocated buffer */

  kmm_free(buffer);
#endif
  /* Restore the original file position */

  smartfs_seek_internal(fs, sf, savepos, SEEK_SET);
  return ret;
}

/****************************************************************************
 * Name: smartfs_get_first_mount
 *
 * Description: Returns a pointer to the first mounted smartfs volume.
 *
 ****************************************************************************/

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
FAR struct smartfs_mountpt_s *smartfs_get_first_mount(void)
{
  return g_mounthead;
}
#endif
