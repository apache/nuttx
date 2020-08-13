/****************************************************************************
 * rm/romfs/fs_romfsutil.c
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

/* References: Linux/Documentation/filesystems/romfs.txt */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/mtd/mtd.h>

#include "fs_romfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINK_NOT_FOLLOWED 0
#define LINK_FOLLOWED     1

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: romfs_devread32
 *
 * Description:
 *   Read the big-endian 32-bit value from the mount device buffer
 *
 * Assumption:
 *   All values are aligned to 32-bit boundaries
 *
 ****************************************************************************/

static uint32_t romfs_devread32(struct romfs_mountpt_s *rm, int ndx)
{
  /* This should not read past the end of the sector since the directory
   * entries are aligned at 16-byte boundaries.
   */

  return ((((uint32_t)rm->rm_buffer[ndx]     & 0xff) << 24) |
          (((uint32_t)rm->rm_buffer[ndx + 1] & 0xff) << 16) |
          (((uint32_t)rm->rm_buffer[ndx + 2] & 0xff) << 8) |
           ((uint32_t)rm->rm_buffer[ndx + 3] & 0xff));
}

/****************************************************************************
 * Name: romfs_checkentry
 *
 * Description:
 *   Check if the entry at offset is a directory or file path segment
 *
 ****************************************************************************/

static inline int romfs_checkentry(struct romfs_mountpt_s *rm,
                                   uint32_t offset, const char *entryname,
                                   int entrylen,
                                   struct romfs_dirinfo_s *dirinfo)
{
  char name[NAME_MAX + 1];
  uint32_t linkoffset;
  uint32_t next;
  uint32_t info;
  uint32_t size;
  int ret;

  /* Parse the directory entry at this offset (which may be re-directed
   * to some other entry if HARLINKED).
   */

  ret = romfs_parsedirentry(rm, offset, &linkoffset, &next, &info, &size);
  if (ret < 0)
    {
      return ret;
    }

  /* Now we are pointing to the real entry of interest. Is it a
   * directory? Or a file?
   */

  if (IS_DIRECTORY(next) || IS_FILE(next))
    {
      /* Get the name of the directory entry. */

      ret = romfs_parsefilename(rm, offset, name);
      if (ret < 0)
        {
          return ret;
        }

      /* Then check if this the name segment we are looking for.  The
       * string comparison is awkward because there is no terminator
       * on entryname (there is a terminator on name, however)
       */

      if (memcmp(entryname, name, entrylen) == 0 &&
          strlen(name) == entrylen)
        {
          /* Found it -- save the component info and return success */

          if (IS_DIRECTORY(next))
            {
              dirinfo->rd_dir.fr_firstoffset = info;
              dirinfo->rd_dir.fr_curroffset  = info;
              dirinfo->rd_size               = 0;
            }
          else
            {
              dirinfo->rd_dir.fr_curroffset  = offset;
              dirinfo->rd_size               = size;
            }

          dirinfo->rd_next                   = next;
          return OK;
        }
    }

  /* The entry is not a directory or it does not have the matching name */

  return -ENOENT;
}

/****************************************************************************
 * Name: romfs_devcacheread
 *
 * Description:
 *   Read the specified sector for specified offset into the sector cache.
 *   Return the index into the sector corresponding to the offset
 *
 ****************************************************************************/

int16_t romfs_devcacheread(struct romfs_mountpt_s *rm, uint32_t offset)
{
  uint32_t sector;
  int      ret;

  /* rm->rm_cachesector holds the current sector that is buffer in or
   * referenced by rm->tm_buffer. If the requested sector is the same as this
   * this then we do nothing.
   */

  sector = SEC_NSECTORS(rm, offset);
  if (rm->rm_cachesector != sector)
    {
      /* Check the access mode */

      if (rm->rm_xipbase)
        {
          /* In XIP mode, rf_buffer is just an offset pointer into the device
           * address space.
           */

          rm->rm_buffer = rm->rm_xipbase + SEC_ALIGN(rm, offset);
        }
      else
        {
          /* In non-XIP mode, we will have to read the new sector. */

          ret = romfs_hwread(rm, rm->rm_buffer, sector, 1);
          if (ret < 0)
            {
              return (int16_t)ret;
            }
        }

      /* Update the cached sector number */

      rm->rm_cachesector = sector;
    }

  /* Return the offset */

  return offset & SEC_NDXMASK(rm);
}

/****************************************************************************
 * Name: romfs_followhardlinks
 *
 * Description:
 *   Given the offset to a file header, check if the file is a hardlink.
 *   If so, traverse the hard links until the terminal, non-linked header
 *   so found and return that offset.
 *
 * Return value:
 *   < 0  :  An error occurred
 *     0  :  No link followed
 *     1  :  Link followed, poffset is the new volume offset
 *
 ****************************************************************************/

static int romfs_followhardlinks(struct romfs_mountpt_s *rm, uint32_t offset,
                                 uint32_t *poffset)
{
  uint32_t next;
  int16_t  ndx;
  int      i;
  int      ret = LINK_NOT_FOLLOWED;

  /* Loop while we are redirected by hardlinks */

  for (i = 0; i < ROMF_MAX_LINKS; i++)
    {
      /* Read the sector containing the offset into memory */

      ndx = romfs_devcacheread(rm, offset);
      if (ndx < 0)
        {
          return ndx;
        }

      /* Check if this is a hard link */

      next = romfs_devread32(rm, ndx + ROMFS_FHDR_NEXT);
      if (!IS_HARDLINK(next))
        {
          *poffset = offset;
          return ret;
        }

      /* Follow the hard-link.  Set return to indicate that we followed a
       * link and that poffset was set to the link offset is valid.
       */

      offset = romfs_devread32(rm, ndx + ROMFS_FHDR_INFO);
      ret    = LINK_FOLLOWED;
    }

  return -ELOOP;
}

/****************************************************************************
 * Name: romfs_searchdir
 *
 * Description:
 *   This is part of the romfs_finddirentry log.  Search the directory
 *   beginning at dirinfo->fr_firstoffset for entryname.
 *
 ****************************************************************************/

static inline int romfs_searchdir(struct romfs_mountpt_s *rm,
                                  const char *entryname, int entrylen,
                                  struct romfs_dirinfo_s *dirinfo)
{
  uint32_t offset;
  uint32_t next;
  int16_t  ndx;
  int      ret;

  /* Then loop through the current directory until the directory
   * with the matching name is found.  Or until all of the entries
   * the directory have been examined.
   */

  offset = dirinfo->rd_dir.fr_firstoffset;
  do
    {
      /* Read the sector into memory (do this before calling
       * romfs_checkentry() so we won't have to read the sector
       * twice in the event that the offset refers to a hardlink).
       */

      ndx = romfs_devcacheread(rm, offset);
      if (ndx < 0)
        {
          return ndx;
        }

      /* Because everything is chunked and aligned to 16-bit boundaries,
       * we know that most the basic node info fits into the sector.
       */

      next = romfs_devread32(rm, ndx + ROMFS_FHDR_NEXT) & RFNEXT_OFFSETMASK;

      /* Check if the name this entry is a directory with the matching
       * name
       */

      ret = romfs_checkentry(rm, offset, entryname, entrylen, dirinfo);
      if (ret == OK)
        {
          /* Its a match! Return success */

          return OK;
        }

      /* No match... select the offset to the next entry */

      offset = next;
    }
  while (next != 0);

  /* There is nothing in this directory with that name */

  return -ENOENT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: romfs_semtake
 ****************************************************************************/

int romfs_semtake(struct romfs_mountpt_s *rm)
{
  return nxsem_wait_uninterruptible(&rm->rm_sem);
}

/****************************************************************************
 * Name: romfs_semgive
 ****************************************************************************/

void romfs_semgive(struct romfs_mountpt_s *rm)
{
  nxsem_post(&rm->rm_sem);
}

/****************************************************************************
 * Name: romfs_hwread
 *
 * Description: Read the specified sector into the sector buffer
 *
 ****************************************************************************/

int romfs_hwread(struct romfs_mountpt_s *rm, uint8_t *buffer,
                 uint32_t sector, unsigned int nsectors)
{
  int ret = OK;

  /* Check the access mode */

  if (rm->rm_xipbase)
    {
      /* In XIP mode, we just copy the requested data */

      memcpy(buffer,
             rm->rm_xipbase + sector*rm->rm_hwsectorsize,
             nsectors*rm->rm_hwsectorsize);
    }
  else
    {
      /* In non-XIP mode, we have to read the data from the device */

      struct inode *inode = rm->rm_blkdriver;
      ssize_t nsectorsread = -ENODEV;

      DEBUGASSERT(inode);
      if (INODE_IS_MTD(inode))
        {
          nsectorsread =
            MTD_BREAD(inode->u.i_mtd, sector, nsectors, buffer);
        }
      else if (inode->u.i_bops->read)
        {
          nsectorsread =
            inode->u.i_bops->read(inode, buffer, sector, nsectors);
        }

      if (nsectorsread == (ssize_t)nsectors)
        {
          ret = OK;
        }
      else if (nsectorsread < 0)
        {
          ret = nsectorsread;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: romfs_filecacheread
 *
 * Description:
 *   Read the specified sector into the sector cache
 *
 ****************************************************************************/

int romfs_filecacheread(struct romfs_mountpt_s *rm, struct romfs_file_s *rf,
                        uint32_t sector)
{
  int ret;

  finfo("sector: %d cached: %d sectorsize: %d XIP base: %p buffer: %p\n",
        sector, rf->rf_cachesector, rm->rm_hwsectorsize,
        rm->rm_xipbase, rf->rf_buffer);

  /* rf->rf_cachesector holds the current sector that is buffer in or
   * referenced by rf->rf_buffer. If the requested sector is the same as this
   * sector then we do nothing.
   */

  if (rf->rf_cachesector != sector)
    {
      /* Check the access mode */

      if (rm->rm_xipbase)
        {
          /* In XIP mode, rf_buffer is just an offset pointer into the device
           * address space.
           */

          rf->rf_buffer = rm->rm_xipbase + sector * rm->rm_hwsectorsize;
          finfo("XIP buffer: %p\n", rf->rf_buffer);
        }
      else
        {
          /* In non-XIP mode, we will have to read the new sector. */

          finfo("Calling romfs_hwread\n");
          ret = romfs_hwread(rm, rf->rf_buffer, sector, 1);
          if (ret < 0)
            {
              ferr("ERROR: romfs_hwread failed: %d\n", ret);
              return ret;
            }
        }

      /* Update the cached sector number */

      rf->rf_cachesector = sector;
    }

  return OK;
}

/****************************************************************************
 * Name: romfs_hwconfigure
 *
 * Description:
 *   This function is called as part of the ROMFS mount operation.
 *   It configures the ROMFS filestem for use on this block driver.  This
 *   include the accounting for the geometry of the device, setting up any
 *   XIP modes of operation, and/or allocating any cache buffers.
 *
 ****************************************************************************/

int romfs_hwconfigure(struct romfs_mountpt_s *rm)
{
  struct inode *inode = rm->rm_blkdriver;
  int ret;

  /* Get the underlying device geometry */

#ifdef CONFIG_DEBUG_FEATURES
  if (inode == NULL)
    {
      return -ENODEV;
    }
#endif

  if (INODE_IS_MTD(inode))
    {
      struct mtd_geometry_s mgeo;

      ret = MTD_IOCTL(inode->u.i_mtd, MTDIOC_GEOMETRY,
                      (unsigned long)&mgeo);
      if (ret != OK)
        {
          return ret;
        }

      /* Save that information in the mount structure */

      rm->rm_hwsectorsize = mgeo.blocksize;
      rm->rm_hwnsectors   = mgeo.neraseblocks *
                            (mgeo.erasesize / mgeo.blocksize);
    }
  else
    {
      struct geometry geo;

      ret = inode->u.i_bops->geometry(inode, &geo);
      if (ret != OK)
        {
          return ret;
        }

      if (!geo.geo_available)
        {
          return -EBUSY;
        }

      /* Save that information in the mount structure */

      rm->rm_hwsectorsize = geo.geo_sectorsize;
      rm->rm_hwnsectors   = geo.geo_nsectors;
    }

  /* Determine if block driver supports the XIP mode of operation */

  rm->rm_cachesector  = (uint32_t)-1;

  if (INODE_IS_MTD(inode))
    {
      ret = MTD_IOCTL(inode->u.i_mtd, MTDIOC_XIPBASE,
                      (unsigned long)&rm->rm_xipbase);
    }
  else if (inode->u.i_bops->ioctl != NULL)
    {
      ret = inode->u.i_bops->ioctl(inode, BIOC_XIPBASE,
                                   (unsigned long)&rm->rm_xipbase);
    }
  else
    {
      ret = -ENOTSUP;
    }

  if (ret == OK && rm->rm_xipbase)
    {
      /* Yes.. Then we will directly access the media (vs.
       * copying into an allocated sector buffer.
       */

      rm->rm_buffer      = rm->rm_xipbase;
      rm->rm_cachesector = 0;
      return OK;
    }

  /* Allocate the device cache buffer for normal sector accesses */

  rm->rm_buffer = (FAR uint8_t *)kmm_malloc(rm->rm_hwsectorsize);
  if (!rm->rm_buffer)
    {
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Name: romfs_fsconfigure
 *
 * Description:
 *   This function is called as part of the ROMFS mount operation   It
 *   sets up the mount structure to include configuration information
 *   contained in the ROMFS header.  This is the place where we actually
 *   determine if the media contains a ROMFS filesystem.
 *
 ****************************************************************************/

int romfs_fsconfigure(struct romfs_mountpt_s *rm)
{
  const char *name;
  int16_t     ndx;

  /* Then get information about the ROMFS filesystem on the devices managed
   * by this block driver. Read sector zero which contains the volume header.
   */

  ndx = romfs_devcacheread(rm, 0);
  if (ndx < 0)
    {
      return ndx;
    }

  /* Verify the magic number at that identifies this as a ROMFS filesystem */

  if (memcmp(rm->rm_buffer, ROMFS_VHDR_MAGIC, 8) != 0)
    {
      return -EINVAL;
    }

  /* Then extract the values we need from the header and return success */

  rm->rm_volsize    = romfs_devread32(rm, ROMFS_VHDR_SIZE);

  /* The root directory entry begins right after the header */

  name              = (FAR const char *)&rm->rm_buffer[ROMFS_VHDR_VOLNAME];
  rm->rm_rootoffset = ROMFS_ALIGNUP(ROMFS_VHDR_VOLNAME + strlen(name) + 1);

  /* and return success */

  rm->rm_mounted    = true;
  return OK;
}

/****************************************************************************
 * Name: romfs_fileconfigure
 *
 * Description:
 *   This function is called as part of the ROMFS file open operation   It
 *   sets up the file structure to handle buffer appropriately, depending
 *   upon XIP mode or not.
 *
 ****************************************************************************/

int romfs_fileconfigure(struct romfs_mountpt_s *rm, struct romfs_file_s *rf)
{
  /* Check if XIP access mode is supported.  If so, then we do not need
   * to allocate anything.
   */

  if (rm->rm_xipbase)
    {
      /* We'll put a valid address in rf_buffer just in case. */

      rf->rf_cachesector = 0;
      rf->rf_buffer      = rm->rm_xipbase;
    }
  else
    {
      /* Nothing in the cache buffer */

      rf->rf_cachesector = (uint32_t)-1;

      /* Create a file buffer to support partial sector accesses */

      rf->rf_buffer = (FAR uint8_t *)kmm_malloc(rm->rm_hwsectorsize);
      if (!rf->rf_buffer)
        {
          return -ENOMEM;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: romfs_checkmount
 *
 * Description: Check if the mountpoint is still valid.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int romfs_checkmount(struct romfs_mountpt_s *rm)
{
  struct inode *inode;
  struct geometry geo;
  int ret;

  /* If the fs_mounted flag is false, then we have already handled the loss
   * of the mount.
   */

  DEBUGASSERT(rm && rm->rm_blkdriver);
  if (rm->rm_mounted)
    {
      /* We still think the mount is healthy.  Check an see if this is
       * still the case
       */

      inode = rm->rm_blkdriver;
      if (INODE_IS_MTD(inode))
        {
          /* It is impossible to remove MTD device */

          return OK;
        }
      else if (inode->u.i_bops->geometry)
        {
          ret = inode->u.i_bops->geometry(inode, &geo);
          if (ret == OK && geo.geo_available && !geo.geo_mediachanged)
            {
              return OK;
            }
        }

      /* If we get here, the mount is NOT healthy */

      rm->rm_mounted = false;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: romfs_finddirentry
 *
 * Description:
 *   Given a path to something that may or may not be in the file system,
 *   return the directory entry of the item.
 *
 ****************************************************************************/

int romfs_finddirentry(struct romfs_mountpt_s *rm,
                       struct romfs_dirinfo_s *dirinfo, const char *path)
{
  const char *entryname;
  const char *terminator;
  int entrylen;
  int ret;

  /* Start with the first element after the root directory */

  dirinfo->rd_dir.fr_firstoffset = rm->rm_rootoffset;
  dirinfo->rd_dir.fr_curroffset  = rm->rm_rootoffset;
  dirinfo->rd_next               = RFNEXT_DIRECTORY;
  dirinfo->rd_size               = 0;

  /* The root directory is a special case */

  if (!path || path[0] == '\0')
    {
      return OK;
    }

  /* Then loop for each directory/file component in the full path */

  entryname    = path;
  terminator = NULL;

  for (; ; )
    {
      /* Find the start of the next path component */

      while (*entryname == '/') entryname++;

      /* Find the end of the next path component */

      terminator = strchr(entryname, '/');
      if (!terminator)
        {
          entrylen = strlen(entryname);
        }
      else
        {
          entrylen = terminator - entryname;
        }

      if (entrylen == 0)
        {
          return OK;
        }

      /* Long path segment names will be truncated to NAME_MAX */

      if (entrylen > NAME_MAX)
        {
          entrylen = NAME_MAX;
        }

      /* Then find the entry in the current directory with the
       * matching name.
       */

      ret = romfs_searchdir(rm, entryname, entrylen, dirinfo);
      if (ret < 0)
        {
          return ret;
        }

      /* Was that the last path component? */

      if (!terminator)
        {
          /* Yes.. return success */

          return OK;
        }

      /* No... If that was not the last path component, then it had
       * better have been a directory
       */

      if (!IS_DIRECTORY(dirinfo->rd_next))
        {
          return -ENOTDIR;
        }

      /* Setup to search the next directory for the next component
       * of the path
       */

      entryname = terminator;
    }

  return ERROR; /* Won't get here */
}

/****************************************************************************
 * Name: romfs_parsedirentry
 *
 * Description:
 *   Return the directory entry at this offset.  If rf is NULL, then the
 *   mount device resources are used.  Otherwise, file resources are used.
 *
 ****************************************************************************/

int romfs_parsedirentry(struct romfs_mountpt_s *rm, uint32_t offset,
                        uint32_t *poffset, uint32_t *pnext, uint32_t *pinfo,
                        uint32_t *psize)
{
  uint32_t save;
  uint32_t next;
  int16_t  ndx;
  int      ret;

  /* Read the sector into memory */

  ndx = romfs_devcacheread(rm, offset);
  if (ndx < 0)
    {
      return ndx;
    }

  /* Yes.. Save the first 'next' value.  That has the offset needed to
   * traverse the parent directory.  But we may need to change the type
   * after we follow the hard links.
   */

  save = romfs_devread32(rm, ndx + ROMFS_FHDR_NEXT);

  /* Traverse hardlinks as necessary to get to the real file header */

  ret = romfs_followhardlinks(rm, offset, poffset);
  if (ret < 0)
    {
      return ret;
    }
  else if (ret > 0)
    {
      /* The link was followed */

      ndx = romfs_devcacheread(rm, *poffset);
      if (ndx < 0)
        {
          return ndx;
        }
    }

  /* Because everything is chunked and aligned to 16-bit boundaries,
   * we know that most the basic node info fits into the sector.  The
   * associated name may not, however.
   *
   * NOTE:  Since ROMFS directory entries are aligned to 16-byte boundaries,
   * we are assured that ndx + ROMFS_FHDR_INFO/SIZE will lie wholly within
   * the sector buffer.
   */

  next   = romfs_devread32(rm, ndx + ROMFS_FHDR_NEXT);
  *pnext = (save & RFNEXT_OFFSETMASK) | (next & RFNEXT_ALLMODEMASK);
  *pinfo = romfs_devread32(rm, ndx + ROMFS_FHDR_INFO);
  *psize = romfs_devread32(rm, ndx + ROMFS_FHDR_SIZE);

  return OK;
}

/****************************************************************************
 * Name: romfs_parsefilename
 *
 * Description:
 *   Return the filename from directory entry at this offset
 *
 ****************************************************************************/

int romfs_parsefilename(struct romfs_mountpt_s *rm, uint32_t offset,
                        char *pname)
{
  int16_t  ndx;
  uint16_t namelen;
  uint16_t chunklen;
  bool     done;

  /* Loop until the whole name is obtained or until NAME_MAX characters
   * of the name have been parsed.
   */

  offset += ROMFS_FHDR_NAME;
  for (namelen = 0, done = false; namelen < NAME_MAX && !done; )
    {
      /* Read the sector into memory */

      ndx = romfs_devcacheread(rm, offset + namelen);
      if (ndx < 0)
        {
          return ndx;
        }

      /* Is the name terminated in this 16-byte block */

      if (rm->rm_buffer[ndx + 15] == '\0')
        {
          /* Yes.. then this chunk is less than 16 */

          chunklen = strlen((FAR char *)&rm->rm_buffer[ndx]);
          done     = true;
        }
      else
        {
          /* No.. then this chunk is 16 bytes in length */

          chunklen = 16;
        }

      /* Check if we would exceed the NAME_MAX */

      if (namelen + chunklen > NAME_MAX)
        {
          chunklen = NAME_MAX - namelen;
          done     = true;
        }

      /* Copy the chunk */

      memcpy(&pname[namelen], &rm->rm_buffer[ndx], chunklen);
      namelen += chunklen;
    }

  /* Terminate the name (NAME_MAX+1 chars total) and return success */

  pname[namelen] = '\0';
  return OK;
}

/****************************************************************************
 * Name: romfs_datastart
 *
 * Description:
 *   Given the offset to a file header, return the offset to the start of
 *   the file data
 *
 ****************************************************************************/

int romfs_datastart(struct romfs_mountpt_s *rm, uint32_t offset,
                    uint32_t *start)
{
  int16_t ndx;
  int     ret;

  /* Traverse hardlinks as necessary to get to the real file header */

  ret = romfs_followhardlinks(rm, offset, &offset);
  if (ret < 0)
    {
      return ret;
    }

  /* Loop until the header size is obtained. */

  offset += ROMFS_FHDR_NAME;
  for (; ; )
    {
      /* Read the sector into memory */

      ndx = romfs_devcacheread(rm, offset);
      if (ndx < 0)
        {
          return ndx;
        }

      /* Get the offset to the next chunk */

      offset += 16;
      if (offset >= rm->rm_volsize)
        {
          return -EIO;
        }

      /* Is the name terminated in this 16-byte block */

      if (rm->rm_buffer[ndx + 15] == '\0')
        {
          /* Yes.. then the data starts at the next chunk */

          *start = offset;
          return OK;
        }
    }

  return -EINVAL; /* Won't get here */
}
