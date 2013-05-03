/****************************************************************************
 * fs/smartfs/smartfs_utils.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "smartfs.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
static struct smartfs_mountpt_s* g_mounthead = NULL;
#endif

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smartfs_semtake
 ****************************************************************************/

void smartfs_semtake(struct smartfs_mountpt_s *fs)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(fs->fs_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

/****************************************************************************
 * Name: smartfs_semgive
 ****************************************************************************/

void smartfs_semgive(struct smartfs_mountpt_s *fs)
{
   sem_post(fs->fs_sem);
}

/****************************************************************************
 * Name: smartfs_mount
 *
 * Desciption: This function is called only when the mountpoint is first
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
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
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

  /* Make sure that that the media is write-able (if write access is needed) */

  if (writeable && !geo.geo_writeenabled)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Get the SMART low-level format information to validate the device has been
   * formatted and scan properly for logical to physical sector mapping.
   */

  ret = FS_IOCTL(fs, BIOC_GETFORMAT, (unsigned long) &fs->fs_llformat);
  if (ret != OK)
    {
      fdbg("Error getting device low level format: %d\n", ret);
      goto errout;
    }

  /* Validate the low-level format is valid */

  if (!(fs->fs_llformat.flags & SMART_FMT_ISFORMATTED))
    {
      fdbg("No low-level format found\n");
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
      fs->fs_rwbuffer = (char *) kmalloc(fs->fs_llformat.availbytes);
      fs->fs_workbuffer = (char *) kmalloc(256);
    }

  /* Now add ourselves to the linked list of SMART mounts */

  fs->fs_next = g_mounthead;
  g_mounthead = fs;

  /* Set our root directory sector based on the directory entry
   * reported by the block driver (based on which device is
   * associated with this mount.
   */

  fs->fs_rootsector = SMARTFS_ROOT_DIR_SECTOR + fs->fs_llformat.rootdirnum;

#else
  fs->fs_rwbuffer = (char *) kmalloc(fs->fs_llformat.availbytes);
  fs->fs_workbuffer = (char *) kmalloc(256);
  fs->fs_rootsector = SMARTFS_ROOT_DIR_SECTOR;
#endif

  /* We did it! */

  fs->fs_mounted = TRUE;

  fdbg("SMARTFS:\n");
  fdbg("\t    Sector size:     %d\n", fs->fs_llformat.sectorsize);
  fdbg("\t    Bytes/sector     %d\n", fs->fs_llformat.availbytes);
  fdbg("\t    Num sectors:     %d\n", fs->fs_llformat.nsectors);
  fdbg("\t    Free sectors:    %d\n", fs->fs_llformat.nfreesectors);
  fdbg("\t    Max filename:    %d\n", CONFIG_SMARTFS_MAXNAMLEN);
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  fdbg("\t    RootDirEntries:  %d\n", fs->fs_llformat.nrootdirentries);
#endif
  fdbg("\t    RootDirSector:   %d\n", fs->fs_rootsector);

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_unmount
 *
 * Desciption: This function is called only when the mountpoint is being
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
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  struct smartfs_mountpt_s *nextfs;
  struct smartfs_mountpt_s *prevfs;
  int           count = 0;
  int           found = FALSE;
#endif

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
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
                  (void)inode->u.i_bops->close(inode);
                }
            }
        }

      /* Free the buffers */

      kfree(fs->fs_rwbuffer);
      kfree(fs->fs_workbuffer);

      /* Set the buffer's to invalid value to catch program bugs */

      fs->fs_rwbuffer = (char *) 0xDEADBEEF;
      fs->fs_workbuffer = (char *) 0xDEADBEEF;
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
              (void)inode->u.i_bops->close(inode);
            }
        }
    }

  /* Release the mountpoint private data */

  kfree(fs->fs_rwbuffer);
  kfree(fs->fs_workbuffer);
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
 *              If the final directory segement of relpath just before the
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

  /* Initialize directory level zero as the root sector */

  dirstack[0] = fs->fs_rootsector;
  entrysize = sizeof(struct smartfs_entry_header_s) + fs->fs_llformat.namesize;

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

      strncpy(fs->fs_workbuffer, segment, seglen);
      fs->fs_workbuffer[seglen] = '\0';

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

          offset = 0xFFFF;

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
          while (dirsector != 0xFFFF)
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
              entry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
              while (offset < readwrite.count)
                {
                  /* Test if this entry is valid and active */

                  if (((entry->flags & SMARTFS_DIRENT_EMPTY) ==
                      (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) ||
                      ((entry->flags & SMARTFS_DIRENT_ACTIVE) !=
                      (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
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

                          direntry->firstsector = entry->firstsector;
                          direntry->flags = entry->flags;
                          direntry->utc = entry->utc;
                          direntry->dsector = readwrite.logsector;
                          direntry->doffset = offset;
                          direntry->dfirst = dirstack[depth];
                          if (direntry->name == NULL)
                            {
                              direntry->name = (char *) kmalloc(fs->fs_llformat.namesize+1);
                            }

                          memset(direntry->name, 0, fs->fs_llformat.namesize + 1);
                          strncpy(direntry->name, entry->name, fs->fs_llformat.namesize);
                          direntry->datlen = 0;

                          /* Scan the file's sectors to calculate the length and perform
                           * a rudamentary check.
                           */

                          if ((entry->flags & SMARTFS_DIRENT_TYPE) == SMARTFS_DIRENT_TYPE_FILE)
                            {
                              dirsector = entry->firstsector;
                              readwrite.count = sizeof(struct smartfs_chain_header_s);
                              readwrite.buffer = (uint8_t *)fs->fs_rwbuffer;
                              readwrite.offset = 0;

                              while (dirsector != SMARTFS_ERASEDSTATE_16BIT)
                                {
                                  /* Read the next sector of the file */

                                  readwrite.logsector = dirsector;
                                  ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
                                  if (ret < 0)
                                    {
                                      fdbg("Error in sector chain at %d!\n", dirsector);
                                      break;
                                    }

                                  /* Add used bytes to the total and point to next sector */

                                  if (*((uint16_t *) header->used) != SMARTFS_ERASEDSTATE_16BIT)
                                    {
                                      direntry->datlen += *((uint16_t *) header->used);
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

                          if ((entry->flags & SMARTFS_DIRENT_TYPE) !=
                              SMARTFS_DIRENT_TYPE_DIR)
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

                          dirstack[++depth] = entry->firstsector;
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
              *parentdirsector = 0xFFFF;
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
 *              0xFFFF, then a new sector is allocated for the new entry,
 *              otherwise the supplied sectorno is used.
 *
 ****************************************************************************/

int smartfs_createentry(struct smartfs_mountpt_s *fs,
        uint16_t parentdirsector, const char* filename,
        uint16_t type,
        mode_t mode, struct smartfs_entry_s *direntry,
        uint16_t sectorno)
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

          if ((entry->flags == SMARTFS_ERASEDSTATE_16BIT) ||
              ((entry->flags &
                (SMARTFS_DIRENT_EMPTY | SMARTFS_DIRENT_ACTIVE) ) ==
               (~SMARTFS_ERASEDSTATE_16BIT &
                (SMARTFS_DIRENT_EMPTY | SMARTFS_DIRENT_ACTIVE) )))
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

          ret = FS_IOCTL(fs, BIOC_ALLOCSECT, 0xFFFF);
          if (ret < 0)
            {
              goto errout;
            }

          nextsector = (uint16_t) ret;

          /* Chain the next sector into this sector sector */

          *((uint16_t *) chainheader->nextsector) = nextsector;
          readwrite.offset = offsetof(struct smartfs_chain_header_s,
              nextsector);
          readwrite.count = sizeof(uint16_t);
          readwrite.buffer = chainheader->nextsector;
          ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
          if (ret < 0)
            {
              fdbg("Error chaining sector %d\n", nextsector);
              goto errout;
            }
        }

      /* Now update to the next sector */

      psector = nextsector;
    }

  /* We found an insertion point.  Create the entry at sector,offset */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  entry->flags = (uint16_t) (SMARTFS_DIRENT_ACTIVE | SMARTFS_DIRENT_DELETING |
          SMARTFS_DIRENT_RESERVED | type | (mode & SMARTFS_DIRENT_MODE));
#else
  entry->flags = (uint16_t) (SMARTFS_DIRENT_EMPTY | type | (mode & SMARTFS_DIRENT_MODE));
#endif

  if (sectorno == 0xFFFF)
    {
      /* Allocate a new sector for the file / dir */

      ret = FS_IOCTL(fs, BIOC_ALLOCSECT, 0xFFFF);
      if (ret < 0)
        {
          goto errout;
        }

      nextsector = ret;

      /* Set the newly allocated sector's type (file or dir) */

      nextsector = (uint16_t) ret;
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
          fdbg("Error %d setting new sector type for sector %d\n",ret,  nextsector);
          goto errout;
        }
    }
  else
    {
      /* Use the provided sector number */

      nextsector = sectorno;
    }

  /* Create the directory entry to be written in the parent's sector */

  entry->firstsector = nextsector;
  entry->utc = 0;
  memset(entry->name, 0, fs->fs_llformat.namesize);
  strncpy(entry->name, filename, fs->fs_llformat.namesize);

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

  /* Now fill in the entry */

  direntry->firstsector = nextsector;
  direntry->dsector = psector;
  direntry->doffset = offset;
  direntry->flags = entry->flags;
  direntry->utc = 0;
  direntry->datlen = 0;
  if (direntry->name == NULL)
    {
      direntry->name = (FAR char *) kmalloc(fs->fs_llformat.namesize+1);
    }

  memset(direntry->name, 0, fs->fs_llformat.namesize+1);
  strncpy(direntry->name, filename, fs->fs_llformat.namesize);

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

   * TODO:  We really should walk the list backward to avoid lost
   *        sectors in the event we lose power. However this requires
   *        allocting a buffer to build the sector list since we don't
   *        store a doubly-linked list of sectors on the deivce.  We
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
          fdbg("Error reading sector %d\n", nextsector);
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
      fdbg("Error reading directory info at sector %s\n", entry->dsector);
      goto errout;
    }

  /* Mark this entry as inactive */

  direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[entry->doffset];
#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  direntry->flags &= ~SMARTFS_DIRENT_ACTIVE;
#else
  direntry->flags |= SMARTFS_DIRENT_ACTIVE;
#endif

  /* Write the updated flags back to the sector */

  readwrite.offset = entry->doffset;
  readwrite.count = sizeof(uint16_t);
  readwrite.buffer = (uint8_t *) &direntry->flags;
  ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
  if (ret < 0)
    {
      fdbg("Error marking entry inactive at sector %s\n", entry->dsector);
      goto errout;
    }

  /* Test if any entries in this sector are being used */

  if ((entry->dsector != fs->fs_rootsector) &&
      (entry->dsector != entry->dfirst))
    {
      /* Scan the sector and count used entries */

      count = 0;
      offset = sizeof(struct smartfs_chain_header_s);
      entrysize = sizeof(struct smartfs_entry_header_s) + fs->fs_llformat.namesize;
      while (offset + entrysize < fs->fs_llformat.availbytes)
        {
          /* Test the next entry */

          direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
          if (((direntry->flags & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((direntry->flags & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
            {
              /* Count this entry */
              count++;
            }

          /* Advance to next entry */

          offset += entrysize;
        }

      /* Test if the count it zero.  If it is, then we will release the sector */

      if (count == 0)
        {
          /* Okay, to release the sector, we must find the sector that we
           * are chained to and remove ourselves from the chain.  First
           * save our nextsector value so we can "unchain" ourselves.
           */

          nextsector = SMARTFS_NEXTSECTOR(header);

          /* Now loop through the dir sectors to find ourselves in the chain */

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
                  fdbg("Error reading sector %d\n", nextsector);
                  break;
                }

              /* Test if this sector "points" to us */

              if (SMARTFS_NEXTSECTOR(header) == entry->dsector)
                {
                  /* We found ourselves in the chain.  Update the chain. */

                  SMARTFS_NEXTSECTOR(header) = nextsector;
                  readwrite.offset = offsetof(struct smartfs_chain_header_s, nextsector);
                  readwrite.count = sizeof(uint16_t);
                  readwrite.buffer = header->nextsector;
                  ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
                  if (ret < 0)
                    {
                      fdbg("Error unchaining sector (%d)\n", nextsector);
                      goto errout;
                    }

                  /* Now release our sector */

                  ret = FS_IOCTL(fs, BIOC_FREESECT, (unsigned long) entry->dsector);
                  if (ret < 0)
                    {
                      fdbg("Error freeing sector %d\n", entry->dsector);
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
          fdbg("Error reading sector %d\n", nextsector);
          break;
        }

      /* Validate this is a directory type sector */

      header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;
      if (header->type != SMARTFS_SECTOR_TYPE_DIR)
        {
          fdbg("Sector %d is not a DIR sector!\n", nextsector);
          goto errout;
        }

      /* Loop for all entries in this sector and count them */

      offset = sizeof(struct smartfs_chain_header_s);
      entrysize = sizeof(struct smartfs_entry_header_s) + fs->fs_llformat.namesize;
      direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
      while (offset + entrysize < readwrite.count)
        {
          if (((direntry->flags & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((direntry->flags & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
            {
              /* Count this entry */
              count++;
            }

          offset += entrysize;
          direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
        }

      /* Get the next sector from the header */

      nextsector = SMARTFS_NEXTSECTOR(header);
    }

  ret = count;

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_truncatefile
 *
 * Description: Truncates an existing file on the device so that it occupies
 *              zero bytes and can be completely re-written.
 *
 ****************************************************************************/

int smartfs_truncatefile(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *entry)
{
  int                             ret;
  uint16_t                        nextsector;
  uint16_t                        sector;
  struct smartfs_chain_header_s  *header;
  struct smart_read_write_s       readwrite;

  /* Walk through the directory's sectors and count entries */

  nextsector = entry->firstsector;
  header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;

  while (nextsector != SMARTFS_ERASEDSTATE_16BIT)
    {
      /* Read the next sector's header into our buffer */

      readwrite.logsector = nextsector;
      readwrite.offset = 0;
      readwrite.count = sizeof(struct smartfs_chain_header_s);
      readwrite.buffer = (uint8_t *) fs->fs_rwbuffer;
      ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
      if (ret < 0)
        {
          fdbg("Error reading sector %d header\n", nextsector);
          goto errout;
        }

      /* Get the next chained sector */

      sector = SMARTFS_NEXTSECTOR(header);

      /* If this is the 1st sector of the file, then just overwrite
       * the sector data with the erased state value.  The underlying
       * SMART block driver will detect this and release the old
       * sector and create a new one with the new (blank) data.
       */

      if (nextsector == entry->firstsector)
        {
          /* Fill our buffer with erased data */

          memset(fs->fs_rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, fs->fs_llformat.availbytes);
          header->type = SMARTFS_SECTOR_TYPE_FILE;

          /* Now write the new sector data */

          readwrite.count = fs->fs_llformat.availbytes;
          ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
          if (ret < 0)
            {
              fdbg("Error blanking 1st sector (%d) of file\n", nextsector);
              goto errout;
            }

          /* Set the entry's data length to zero ... we just truncated */

          entry->datlen = 0;
        }
      else
        {
          /* Not the 1st sector -- release it */

          ret = FS_IOCTL(fs, BIOC_FREESECT, (unsigned long) nextsector);
          if (ret < 0)
            {
              fdbg("Error freeing sector %d\n", nextsector);
              goto errout;
            }
        }

      /* Now move on to the next sector */

      nextsector = sector;
    }

  ret = OK;

errout:
  return ret;
}
