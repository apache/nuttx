/****************************************************************************
 * rm/romfs/fs_romfsutil.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References: Linux/Documentation/filesystems/romfs.txt
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "fs_romfs.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: romfs_swap32
 *
 * Desciption:
 *   Convert the 32-bit big endian value to little endian
 *
 ****************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
static inline uint32 romfs_swap32(uint32 value)
{
  return ((((value) & 0x000000ff) << 24) | (((value) & 0x0000ff00) << 8) |
          (((value) & 0x00ff0000) >>  8) | (((value) & 0xff000000) >> 24));
}
#endif

/****************************************************************************
 * Name: romfs_devread32
 *
 * Desciption:
 *   Read the big-endian 32-bit value from the mount device buffer 
 *
 * Assumption:
 *   All values are aligned to 32-bit boundaries
 *
 ****************************************************************************/

static uint32 romfs_devread32(struct romfs_mountpt_s *rm, int ndx)
{
  /* Extract the value */

  uint32 value = *(uint32*)&rm->rm_buffer[ndx];

  /* Value is begin endian -- return the native host endian-ness. */
#ifdef CONFIG_ENDIAN_BIG
  return value;
#else
  return romfs_swap32(value);
#endif
}

/****************************************************************************
 * Name: romfs_checkentry
 *
 * Desciption:
 *   Check if the entry at offset is a directory or file path segment
 *
 ****************************************************************************/

static inline int romfs_checkentry(struct romfs_mountpt_s *rm, uint32 offset,
                                   const char *entryname, int entrylen,
                                   struct romfs_dirinfo_s *dirinfo)
{
  char name[NAME_MAX+1];
  uint32 linkoffset;
  uint32 next;
  uint32 info;
  uint32 size;
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
              dirinfo->rd_dir.fr_diroffset   = linkoffset;
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
 * Name: romfs_searchdir
 *
 * Desciption:
 *   This is part of the romfs_finddirentry log.  Search the directory
 *   beginning at dirinfo->fr_firstoffset for entryname.
 *
 ****************************************************************************/

static inline int romfs_searchdir(struct romfs_mountpt_s *rm,
                                  const char *entryname, int entrylen,
                                  struct romfs_dirinfo_s *dirinfo)
{
  uint32 offset;
  uint32 sector;
  uint32 next;
  uint16 ndx;
  int    ret;

  /* Then loop through the current directory until the directory
   * with the matching name is found.
   */

  offset = dirinfo->rd_dir.fr_firstoffset;
  for (;;)
    {
      /* Convert the offset into sector + index */

      sector = SEC_NSECTORS(rm, offset);
      ndx    = offset & SEC_NDXMASK(rm);

      /* Read the sector into memory (do this before calling
       * romfs_checkentry() so we won't have to read the sector
       * twice in the event that the offset refers to a hardlink).
       */

      ret = romfs_devcacheread(rm, sector);
      if (ret < 0)
        {
          return ret;
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

      offset += next;
    }
   while (next != 0)

   /* There is nothing in this directoy with that name */

   return -ENOENT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: romfs_semtake
 ****************************************************************************/

void romfs_semtake(struct romfs_mountpt_s *rm)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&rm->rm_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

/****************************************************************************
 * Name: romfs_semgive
 ****************************************************************************/

void romfs_semgive(struct romfs_mountpt_s *rm)
{
   sem_post(&rm->rm_sem);
}

/****************************************************************************
 * Name: romfs_hwread
 *
 * Desciption: Read the specified sector into the sector buffer
 *
 ****************************************************************************/

int romfs_hwread(struct romfs_mountpt_s *rm, ubyte *buffer, uint32 sector,
                 unsigned int nsectors)
{
  struct inode *inode;;
  ssize_t nsectorsread;
  int ret = -ENODEV;

  if (rm && rm->rm_blkdriver)
    {
      inode = rm->rm_blkdriver;
      if (inode && inode->u.i_bops && inode->u.i_bops->read)
        {
          nsectorsread =
            inode->u.i_bops->read(inode, buffer, sector, nsectors);

          if (nsectorsread == (ssize_t)nsectors)
            {
              ret = OK;
            }
          else if (nsectorsread < 0)
            {
              ret = nsectorsread;
            }
        }
    }
  return ret;
}

/****************************************************************************
 * Name: romfs_devcacheread
 *
 * Desciption:
 *   Read the specified sector into the sector cache
 *
 ****************************************************************************/

int romfs_devcacheread(struct romfs_mountpt_s *rm, uint32 sector)
{
  int ret;

  /* rm->rm_cachesector holds the current sector that is buffered in
   * rm->tm_buffer. If the requested sector is the same as this sector, then
   * we do nothing. Otherwise, we will have to read the new sector.
   */

  if (rm->rm_cachesector != sector)
      {
        ret = romfs_hwread(rm, rm->rm_buffer, sector, 1);
        if (ret < 0)
          {
            return ret;
          }

        /* Update the cached sector number */

        rm->rm_cachesector = sector;
    }
    return OK;
}

/****************************************************************************
 * Name: romfs_filecacheread
 *
 * Desciption:
 *   Read the specified sector into the sector cache
 *
 ****************************************************************************/

int romfs_filecacheread(struct romfs_mountpt_s *rm, struct romfs_file_s *rf, uint32 sector)
{
  int ret;

  /* rf->rf_cachesector holds the current sector that is buffered in
   * rf->rf_buffer. If the requested sector is the same as this sector, then
   * we do nothing. Otherwise, we will have to read the new sector.
   */

  if (rf->rf_cachesector != sector)
      {
        ret = romfs_hwread(rm, rf->rf_buffer, sector, 1);
        if (ret < 0)
          {
            return ret;
          }

        /* Update the cached sector number */

        rf->rf_cachesector = sector;
    }
    return OK;
}

/****************************************************************************
 * Name: romfs_getgeometry
 *
 * Desciption:
 *   Get the geometry of the device (part of the mount operation)
 *
 ****************************************************************************/

int romfs_getgeometry(struct romfs_mountpt_s *rm)
{
  struct inode *inode = rm->rm_blkdriver;
  struct geometry geo;
  int ret;

  /* Get the underlying device geometry */

  if (!inode || !inode->u.i_bops || !inode->u.i_bops->geometry)
    {
      return -ENODEV;
    }

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
  return OK;
}

/****************************************************************************
 * Name: romfs_mount
 *
 * Desciption:
 *   Setup ROMFS on the block driver
 *
 ****************************************************************************/

int romfs_mount(struct romfs_mountpt_s *rm)
{
  const char *name;
  int ret;

  /* Then get information about the ROMFS filesystem on the devices managed
   * by this block driver.  Read sector zero which contains the volume header.
   */

  ret = romfs_devcacheread(rm, 0);
  if (ret != 0)
    {
      return ret;
    }

  /* Verify the magic number at that identifies this as a ROMFS filesystem */

  if (memcmp(rm->rm_buffer, ROMFS_VHDR_MAGIC, 8) != 0)
    {
      return -EINVAL;
    }

  /* Then extract the values we need from the header and return success */

  rm->rm_volsize    = romfs_devread32(rm, ROMFS_VHDR_SIZE);

  /* The root directory entry begins right after the header */

  name              = (const char*)&rm->rm_buffer[ROMFS_VHDR_VOLNAME];
  rm->rm_rootoffset = ROMFS_ALIGNUP(ROMFS_VHDR_VOLNAME + strlen(name) + 1);

  /* and return success */

  rm->rm_mounted      = TRUE;
  return OK;
}

/****************************************************************************
 * Name: romfs_checkmount
 *
 * Desciption: Check if the mountpoint is still valid.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int romfs_checkmount(struct romfs_mountpt_s *rm)
{
  /* If the fs_mounted flag is FALSE, then we have already handled the loss
   * of the mount.
   */

  if (rm && rm->rm_mounted)
    {
      struct romfs_file_s *file;

      /* We still think the mount is healthy.  Check an see if this is
       * still the case
       */

      if (rm->rm_blkdriver)
        {
          struct inode *inode = rm->rm_blkdriver;
          if (inode && inode->u.i_bops && inode->u.i_bops->geometry)
            {
              struct geometry geo;
              int errcode = inode->u.i_bops->geometry(inode, &geo);
              if (errcode == OK && geo.geo_available && !geo.geo_mediachanged)
                {
                  return OK;
                }
            }
        }

      /* If we get here, the mount is NOT healthy */

      rm->rm_mounted = FALSE;

      /* Make sure that this is flagged in every opened file */

      for (file = rm->rm_head; file; file = file->rf_next)
        {
          file->rf_open = FALSE;
        }
    }
  return -ENODEV;
}

/****************************************************************************
 * Name: romfs_finddirentry
 *
 * Desciption:
 *   Given a path to something that may or may not be in the file system,
 *   return the directory entry of the item.
 *
 ****************************************************************************/

int romfs_finddirentry(struct romfs_mountpt_s *rm, struct romfs_dirinfo_s *dirinfo,
                       const char *path)
{
  const char *entryname;
  const char *terminator;
  int entrylen;
  int ret;

  /* Start with the first element after the root directory */

  dirinfo->rd_dir.fr_diroffset   = 0;
  dirinfo->rd_dir.fr_firstoffset = rm->rm_rootoffset;
  dirinfo->rd_dir.fr_curroffset  = rm->rm_rootoffset;
  dirinfo->rd_next               = 0;
  dirinfo->rd_size               = 0;

  /* Then loop for each directory/file component in the full path */

  entryname    = path;
  terminator = NULL;

  for (;;)
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
 * Desciption:
 *   Return the directory entry at this offset.  If rf is NULL, then the
 *   mount device resources are used.  Otherwise, file resources are used.
 *
 ****************************************************************************/

int romfs_parsedirentry(struct romfs_mountpt_s *rm, uint32 offset, uint32 *poffset,
                        uint32 *pnext, uint32 *pinfo, uint32 *psize)
{
  uint32 sector;
  uint32 save;
  uint32 next;
  uint32 info;
  uint32 size;
  uint16 ndx;
  int ret;
  int i;

  /* Loop while we are redirected by hardlinks */

  for (i = 0; i < ROMF_MAX_LINKS; i++)
    {
      /* Convert the offset into sector + index */

      sector = SEC_NSECTORS(rm, offset);
      ndx    = offset & SEC_NDXMASK(rm);

      /* Read the sector into memory */

      ret    = romfs_devcacheread(rm, sector);
      if (ret < 0)
        {
          return ret;
        }

      /* Because everything is chunked and aligned to 16-bit boundaries,
       * we know that most the basic node info fits into the sector.  The
       * associated name may not, however.
       */

      next = romfs_devread32(rm, ndx + ROMFS_FHDR_NEXT);
      info = romfs_devread32(rm, ndx + ROMFS_FHDR_INFO);
      size = romfs_devread32(rm, ndx + ROMFS_FHDR_SIZE);

      /* Is this the first offset we have tried? */

      if (i == 0)
        {
          /* Yes.. Save the first 'next' value.  That has the offset needed to
           * traverse the parent directory.
           */

          save = next;
      }

      /* Is this a hardlink? */

      if (IS_HARDLINK(next))
        {
          /* Yes.. then the info field is the offset to the actual entry
           * that we are interested in.
           */

          offset = info;
        }
      else
        {
          /* No... then break returning the directory information.
           * Retaining the original offset
           */

          *poffset = offset;
          *pnext   = (save & RFNEXT_OFFSETMASK) | (next & RFNEXT_MODEMASK);
          *pinfo   = info;
          *psize   = size;
          return OK;
        }
    }

  /* Too many hard links -- probably an infinite loop */

  return -ELOOP;
}

/****************************************************************************
 * Name: romfs_parsefilename
 *
 * Desciption:
 *   Return the filename from directory entry at this offset
 *
 ****************************************************************************/

int romfs_parsefilename(struct romfs_mountpt_s *rm, uint32 offset, char *pname)
{
  uint32 sector;
  uint16 ndx;
  uint16 namelen;
  uint16 chunklen;
  boolean done;
  int ret;

  /* Loop until the whole name is obtained or until NAME_MAX characters
   * of the name have been parsed.
   */

  offset += ROMFS_FHDR_NAME;
  for (namelen = 0, done = FALSE; namelen < NAME_MAX && !done;)
    {
      /* Convert the offset into sector + index */

      sector = SEC_NSECTORS(rm, offset);
      ndx    = offset & SEC_NDXMASK(rm);

      /* Read the sector into memory */

      ret    = romfs_devcacheread(rm, sector);
      if (ret < 0)
        {
          return ret;
        }

      /* Is the name terminated in this 16-byte block */

      if (rm->rm_buffer[ndx + 15] == '\0')
        {
          /* Yes.. then this chunk is less than 16 */

          chunklen = strlen((char*)&rm->rm_buffer[ndx]);
          done     = TRUE;
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
          done     = TRUE;
        }

      /* Copy the chunk */

      memcpy(&pname[namelen], &rm->rm_buffer[ndx], chunklen);
      namelen += chunklen;
    }

  /* Terminate the name (NAME_MAX+1 chars total) and return success */

  pname[namelen] = '\0';
  return OK;
}
