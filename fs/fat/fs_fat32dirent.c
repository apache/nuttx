/****************************************************************************
 * fs/fat/fs_fat32dirent.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * NOTE:  If CONFIG_FAT_LFN is defined, then there may be some legal, patent
 * issues. The following was extracted from the entry "File Allocation Table
 * from Wikipedia, the free encyclopedia:
 *
 * "On December 3, 2003 Microsoft announced it would be offering licenses
 *  for use of its FAT specification and 'associated intellectual property',
 *  at the cost of a US$0.25 royalty per unit sold, with a $250,000 maximum
 *  royalty per license agreement.
 *
 *  o "U.S. Patent 5,745,902 (http://www.google.com/patents?vid=5745902) -
 *     Method and system for accessing a file using file names having
 *     different file name formats. ...
 *  o "U.S. Patent 5,579,517 (http://www.google.com/patents?vid=5579517) -
 *     Common name space for long and short filenames. ...
 *  o "U.S. Patent 5,758,352 (http://www.google.com/patents?vid=5758352) -
 *     Common name space for long and short filenames. ...
 *  o "U.S. Patent 6,286,013 (http://www.google.com/patents?vid=6286013) -
 *     Method and system for providing a common name space for long and
 *     short file names in an operating system. ...
 *
 * "Many technical commentators have concluded that these patents only cover
 *  FAT implementations that include support for long filenames, and that
 *  removable solid state media and consumer devices only using short names
 *  would be unaffected. ..."
 *
 * So you have been forewarned:  Use the long filename at your own risk!
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
#include <ctype.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>
#include <nuttx/fat.h>

#include "fs_internal.h"
#include "fs_fat32.h"

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

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_path2dirname
 *
 * Desciption:  Convert a user filename into a properly formatted FAT
 *   (short) filname as it would appear in a directory entry.  Here are the
 *    rules for the 11 byte name in the directory:
 *
 *   The first byte:
 *   - 0xe5 = The directory is free
 *   - 0x00 = This directory and all following directories are free
 *   - 0x05 = Really 0xe5
 *   - 0x20 = May NOT be ' '
 *
 *   Any bytes
 *     0x00-0x1f = (except for 0x00 and 0x05 in the first byte)
 *     0x22      = '"'
 *     0x2a-0x2c = '*', '+', ','
 *     0x2e-0x2f = '.', '/'
 *     0x3a-0x3f = ':', ';', '<', '=', '>', '?'
 *     0x5b-0x5d = '[', '\\', ;]'
 *     0x7c      = '|'
 *
 *   Upper case characters are not allowed in directory names (without some
 *   poorly documented operatgions on the NTRes directory byte).  Lower case
 *   codes may represent different characters in other character sets ("DOS
 *   code pages".  The logic below does not, at present, support any other
 *   character sets.
 *
 ****************************************************************************/

static inline int fat_path2dirname(const char **path, struct fat_dirinfo_s *dirinfo,
                                   char *terminator)
{
#ifdef CONFIG_FAT_LCNAMES
    unsigned int ntlcenable = FATNTRES_LCNAME | FATNTRES_LCEXT;
    unsigned int ntlcfound  = 0;
#endif
    const char *node = *path;
    int endndx;
    uint8_t ch;
    int ndx = 0;

    /* Initialized the name with all spaces */

    memset(dirinfo->fd_name, ' ', 8+3);
 
    /* Loop until the name is successfully parsed or an error occurs */

    endndx  = 8;
    for (;;)
      {
        /* Get the next byte from the path */

        ch = *node++;

        /* Check if this the last byte in this node of the name */

        if ((ch == '\0' || ch == '/') && ndx != 0 )
          {
            /* Return the accumulated NT flags and the terminating character */
#ifdef CONFIG_FAT_LCNAMES
            dirinfo->fd_ntflags = ntlcfound & ntlcenable;
#endif
            *terminator = ch;
            *path       = node;
            return OK;
          }

        /* Accept only the printable character set.  Note the first byte
         * of the name could be 0x05 meaning that is it 0xe5, but this is
         * not a printable character in this character in either case.
         */

        else if (!isgraph(ch))
          {
            goto errout;
          }

        /* Check for transition from name to extension */

        else if (ch == '.')
          {
            /* Starting the extension */

            ndx    = 8;
            endndx = 11;
            continue;
          }

        /* Reject printable characters forbidden by FAT */

        else if (ch == '"'  ||  (ch >= '*' && ch <= ',') ||
                 ch == '.'  ||   ch == '/'               ||
                (ch >= ':'  &&   ch <= '?')              ||
                (ch >= '['  &&   ch <= ']')              ||
                (ch == '|'))
          {
            goto errout;
          }

        /* Check for upper case charaters */

#ifdef CONFIG_FAT_LCNAMES
        else if (isupper(ch))
          {
            /* Some or all of the characters in the name or extension
             * are upper case. Force all of the characters to be interpreted
             * as upper case.
             */

              if ( endndx == 8)
                {
                  /* Clear lower case name bit in mask*/
                  ntlcenable &= FATNTRES_LCNAME;
                }
              else
                {
                  /* Clear lower case extension in mask */
                  ntlcenable &= FATNTRES_LCNAME;
                }
          }
#endif

        /* Check for lower case characters */

        else if (islower(ch))
          {
            /* Convert the character to upper case */

            ch = toupper(ch);

            /* Some or all of the characters in the name or extension
             * are lower case.  They can be interpreted as lower case if
             * only if all of the characters in the name or extension are
             * lower case.
             */

#ifdef CONFIG_FAT_LCNAMES
            if ( endndx == 8)
              {
                /* Set lower case name bit */
                ntlcfound |= FATNTRES_LCNAME;
              }
            else
              {
                /* Set lower case extension bit */
                ntlcfound |= FATNTRES_LCNAME;
              }
#endif
          }

        /* Check if the file name exceeds the size permitted (without
         * long file name support
         */

        if (ndx >= endndx)
          {
            goto errout;
          }

        /* Save next character in the accumulated name */

        dirinfo->fd_name[ndx++] = ch;
      }

 errout:
    return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_finddirentry
 *
 * Desciption: Given a path to something that may or may not be in the file
 *   system, return the directory entry of the item.
 *
 ****************************************************************************/

int fat_finddirentry(struct fat_mountpt_s *fs, struct fat_dirinfo_s *dirinfo,
                     const char *path)
{
  off_t  cluster;
  uint8_t *direntry = NULL;
  char terminator;
  int ret;

  /* Initialize to traverse the chain.  Set it to the cluster of
   * the root directory
   */

  cluster = fs->fs_rootbase;
  if (fs->fs_type == FSTYPE_FAT32)
    {
      /* For FAT32, the root directory is variable sized and is a
       * cluster chain like any other directory.  fs_rootbase holds
       * the first cluster of the root directory.
       */

      dirinfo->dir.fd_startcluster = cluster;
      dirinfo->dir.fd_currcluster  = cluster;
      dirinfo->dir.fd_currsector   = fat_cluster2sector(fs, cluster);
    }
  else
    {
      /* For FAT12/16, the first sector of the root directory is a sector
       * relative to the first sector of the fat volume.
       */

      dirinfo->dir.fd_startcluster = 0;
      dirinfo->dir.fd_currcluster  = 0;
      dirinfo->dir.fd_currsector   = cluster;
    }

  /* fd_index is the index into the current directory table */

  dirinfo->dir.fd_index = 0;

  /* If no path was provided, then the root directory must be exactly
   * what the caller is looking for.
   */

  if (*path == '\0')
    {
      dirinfo->fd_entry = NULL;
      return OK;
    }

  /* Otherwise, loop until the path is found */

  for (;;)
    {
      /* Convert the next the path segment name into the kind of
       * name that we would see in the directory entry.
       */

      ret = fat_path2dirname(&path, dirinfo, &terminator);
      if (ret < 0)
        {
          /* ERROR:  The filename contains invalid characters or is
           * too long.
           */

          return ret;
        }

      /* Now search the current directory entry for an entry with this
       * matching name.
       */

      for (;;)
        {
          /* Read the next sector into memory */

          ret = fat_fscacheread(fs, dirinfo->dir.fd_currsector);
          if (ret < 0)
            {
              return ret;
            }

          /* Get a pointer to the directory entry */

          direntry = &fs->fs_buffer[DIRSEC_BYTENDX(fs, dirinfo->dir.fd_index)];

          /* Check if we are at the end of the directory */

          if (direntry[DIR_NAME] == DIR0_ALLEMPTY)
            {
              return -ENOENT;
            }

          /* Check if we have found the directory entry that we are looking for */

          if (direntry[DIR_NAME] != DIR0_EMPTY &&
              !(DIR_GETATTRIBUTES(direntry) & FATATTR_VOLUMEID) &&
              !memcmp(&direntry[DIR_NAME], dirinfo->fd_name, 8+3) )
            {
              /* Yes.. break out of the loop */

              break;
            }

          /* No... get the next directory index and try again */

          if (fat_nextdirentry(fs, &dirinfo->dir) != OK)
            {
              return -ENOENT;
            }
        }

      /* We get here only if we have found a directory entry that matches
       * the path element that we are looking for.
       *
       * If the terminator character in the path was the end of the string
       * then we have successfully found the directory entry that describes
       * the path.
       */

      if (!terminator)
        {
          /* Return the pointer to the matching directory entry */

          dirinfo->fd_entry = direntry;
          return OK;
        }

      /* No.. then we have found one of the intermediate directories on
       * the way to the final path target.  In this case, make sure
       * the thing that we found is, indeed, a directory.
       */

      if (!(DIR_GETATTRIBUTES(direntry) & FATATTR_DIRECTORY))
        {
          /* Ooops.. we found something else */

          return -ENOTDIR;
        }

      /* Get the cluster number of this directory */

      cluster =
          ((uint32_t)DIR_GETFSTCLUSTHI(direntry) << 16) |
          DIR_GETFSTCLUSTLO(direntry);

      /* The restart scanning at the new directory */

      dirinfo->dir.fd_currcluster = dirinfo->dir.fd_startcluster = cluster;
      dirinfo->dir.fd_currsector  = fat_cluster2sector(fs, cluster);
      dirinfo->dir.fd_index       = 2;
    }
}

/****************************************************************************
 * Name: fat_allocatedirentry
 *
 * Desciption: Find a free directory entry
 *
 ****************************************************************************/

int fat_allocatedirentry(struct fat_mountpt_s *fs, struct fat_dirinfo_s *dirinfo)
{
  int32_t cluster;
  off_t   sector;
  uint8_t *direntry;
  uint8_t ch;
  int     ret;
  int     i;

  /* Re-initialize directory object */

  cluster = dirinfo->dir.fd_startcluster;
  if (cluster)
    {
     /* Cluster chain can be extended */

      dirinfo->dir.fd_currcluster = cluster;
      dirinfo->dir.fd_currsector  = fat_cluster2sector(fs, cluster);
    }
  else
    {
      /* Fixed size FAT12/16 root directory is at fixxed offset/size */

      dirinfo->dir.fd_currsector = fs->fs_rootbase;
    }
  dirinfo->dir.fd_index = 0;

  for (;;)
    {
      unsigned int dirindex;

      /* Read the directory sector into fs_buffer */

      ret = fat_fscacheread(fs, dirinfo->dir.fd_currsector);
      if (ret < 0)
        {
          return ret;
        }

      /* Get a pointer to the entry at fd_index */

      dirindex = (dirinfo->dir.fd_index & DIRSEC_NDXMASK(fs)) * DIR_SIZE;
      direntry = &fs->fs_buffer[dirindex];

      /* Check if this directory entry is empty */

      ch = direntry[DIR_NAME];
      if (ch == DIR0_ALLEMPTY || ch == DIR0_EMPTY)
        {
          /* It is empty -- we have found a directory entry */

          dirinfo->fd_entry = direntry;
          return OK;
        }

      ret = fat_nextdirentry(fs, &dirinfo->dir);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* If we get here, then we have reached the end of the directory table
   * in this sector without finding a free directory enty.
   *
   * It this is a fixed size dirctory entry, then this is an error.
   * Otherwise, we can try to extend the directory cluster chain to
   * make space for the new directory entry.
   */

  if (!cluster)
    {
      /* The size is fixed */
      return -ENOSPC;
    }

  /* Try to extend the cluster chain for this directory */

  cluster = fat_extendchain(fs, dirinfo->dir.fd_currcluster);
  if (cluster < 0)
    {
      return cluster;
    }

 /* Flush out any cached date in fs_buffer.. we are going to use
  * it to initialize the new directory cluster.
  */

  ret = fat_fscacheflush(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear all sectors comprising the new directory cluster */

  fs->fs_currentsector = fat_cluster2sector(fs, cluster);
  memset(fs->fs_buffer, 0, fs->fs_hwsectorsize);

  sector = sector;
  for (i = fs->fs_fatsecperclus; i; i--)
    {
      ret = fat_hwwrite(fs, fs->fs_buffer, sector, 1);
      if ( ret < 0)
        {
          return ret;
        }
      sector++;
    }

  dirinfo->fd_entry = fs->fs_buffer;
  return OK;
}

/****************************************************************************
 * Name: fat_freedirentry
 *
 * Desciption:  Free the directory entry.
 *
 * Assumptions: (1) the directory enty is in the cache and (2) direntry
 *   points to the directory entry to be deleted.  This obvioulsy needs
 *   to be re-designed to support long file names!
 *
 ****************************************************************************/

int fat_freedirentry(struct fat_mountpt_s *fs, FAR uint8_t *direntry)
{
  direntry[DIR_NAME] = DIR0_EMPTY;
  fs->fs_dirty = true;
  return OK;
}

/****************************************************************************
 * Name: fat_dirname2path
 *
 * Desciption:  Convert a filename in a raw directory entry into a user
 *    filename.  This is essentially the inverse operation of that performed
 *    by fat_path2dirname.  See that function for more details.
 *
 ****************************************************************************/

int fat_dirname2path(char *path, uint8_t *direntry)
{
#ifdef CONFIG_FAT_LCNAMES
    uint8_t ntflags;
#endif
    int  ch;
    int  ndx;

    /* Check if we will be doing upper to lower case conversions */

#ifdef CONFIG_FAT_LCNAMES
    ntflags = DIR_GETNTRES(direntry);
#endif

    /* Get the 8-byte filename */

    for (ndx = 0; ndx < 8; ndx++)
      {
        /* Get the next filename character from the directory entry */

        ch = direntry[ndx];

        /* Any space (or ndx==8) terminates the filename */

        if (ch == ' ')
          {
            break;
          }

        /* In this version, we never write 0xe5 in the directoryfilenames
         * (because we do not handle any character sets where 0xe5 is valid
         * in a filaname), but we could encounted this in a filesystem
         * written by some other system
         */

        if (ndx == 0 && ch == DIR0_E5)
          {
            ch = 0xe5;
          }

        /* Check if we should perform upper to lower case conversion
         * of the (whole) filename.
         */

#ifdef CONFIG_FAT_LCNAMES
        if (ntflags & FATNTRES_LCNAME && isupper(ch))
          {
            ch = tolower(ch);
          }
#endif
        /* Copy the next character into the filename */

        *path++ = ch;
      }

    /* Check if there is an extension */

    if (direntry[8] != ' ')
      {
        /* Yes, output the dot before the extension */

        *path++ = '.';

        /* Then output the (up to) 3 character extension */

        for (ndx = 8; ndx < 11; ndx++)
          {
            /* Get the next extensions character from the directory entry */

            ch = direntry[DIR_NAME + ndx];

            /* Any space (or ndx==11) terminates the extension */

            if (ch == ' ')
              {
                break;
              }

            /* Check if we should perform upper to lower case conversion
             * of the (whole) filename.
             */

#ifdef CONFIG_FAT_LCNAMES
            if (ntflags & FATNTRES_LCEXT && isupper(ch))
              {
                ch = tolower(ch);
              }
#endif
        /* Copy the next character into the filename */

            *path++ = ch;
          }
      }

    /* Put a null terminator at the end of the filename */

    *path = '\0';
    return OK;
}

/****************************************************************************
 * Name: fat_dirnamewrite
 *
 * Desciption: Write the (possibly long) directory entry name.
 *
 * Assumption:  The directory sector is in the cache.
 *
 ****************************************************************************/

int fat_dirnamewrite(struct fat_mountpt_s *fs, struct fat_dirinfo_s *dirinfo)
{
  uint8_t *direntry = dirinfo->fd_entry;
 
  memcpy(&direntry[DIR_NAME], dirinfo->fd_name, 8+3);
#ifdef CONFIG_FLAT_LCNAMES
  DIR_PUTNTRES(direntry, dirinfo->fd_ntflags);
#else
  DIR_PUTNTRES(direntry, 0);
#endif
  fs->fs_dirty = true;
  return OK;
}

/****************************************************************************
 * Name: fat_dirwrite
 *
 * Desciption: Write a directory entry, possibly with a long file name
 *
 * Assumption:  The directory sector is in the cache.  The caller will write
 *   sector information.
 *
 ****************************************************************************/

int fat_dirwrite(struct fat_mountpt_s *fs, struct fat_dirinfo_s *dirinfo,
                 uint8_t attributes, uint32_t fattime)
{
  uint8_t *direntry;

  /* Initialize the 32-byte directory entry */

  direntry = dirinfo->fd_entry;
  memset(direntry, 0, DIR_SIZE);

  /* Directory name info */

  (void)fat_dirnamewrite(fs, dirinfo);

  /* Set the attribute attribute, write time, creation time */

  DIR_PUTATTRIBUTES(direntry, attributes);

  /* Set the time information */

  DIR_PUTWRTTIME(direntry, fattime & 0xffff);
  DIR_PUTCRTIME(direntry, fattime & 0xffff);
  DIR_PUTWRTDATE(direntry, fattime >> 16);
  DIR_PUTCRDATE(direntry, fattime >> 16);

  fs->fs_dirty = true;
  return OK;
}

/****************************************************************************
 * Name: fat_dircreate
 *
 * Desciption: Create a directory entry for a new file
 *
 ****************************************************************************/

int fat_dircreate(struct fat_mountpt_s *fs, struct fat_dirinfo_s *dirinfo)
{
  uint32_t fattime;
  int ret;

  /* Allocate a directory entry entry */

  ret = fat_allocatedirentry(fs, dirinfo);
  if (ret != OK)
    {
      /* Failed to set up directory entry */

      return ret;
    }

  /* Write the directory entry with the current time and the ARCHIVE attribute */

  fattime = fat_systime2fattime();
  return fat_dirwrite(fs, dirinfo, FATATTR_ARCHIVE, fattime);
}

/****************************************************************************
 * Name: fat_remove
 *
 * Desciption: Remove a directory or file from the file system.  This
 *   implements both rmdir() and unlink().
 *
 ****************************************************************************/

int fat_remove(struct fat_mountpt_s *fs, const char *relpath, bool directory)
{
  struct fat_dirinfo_s dirinfo;
  uint32_t  dircluster;
  off_t   dirsector;
  int     ret;

  /* Find the directory entry referring to the entry to be deleted */

  ret = fat_finddirentry(fs, &dirinfo, relpath);
  if (ret != OK)
    {
      /* No such path */

      return -ENOENT;
    }

  /* Check if this is a FAT12/16 root directory */

  if (dirinfo.fd_entry == NULL)
    {
      /* The root directory cannot be removed */

      return -EPERM;
    }

  /* The object has to have write access to be deleted */

  if ((DIR_GETATTRIBUTES(dirinfo.fd_entry) & FATATTR_READONLY) != 0)
    {
      /* It is a read-only entry */

      return -EACCES;
    }

  /* Get the directory sector and cluster containing the
   * entry to be deleted
   */

  dirsector  = fs->fs_currentsector;
  dircluster =
      ((uint32_t)DIR_GETFSTCLUSTHI(dirinfo.fd_entry) << 16) |
      DIR_GETFSTCLUSTLO(dirinfo.fd_entry);

  /* Is this entry a directory? */

  if (DIR_GETATTRIBUTES(dirinfo.fd_entry) & FATATTR_DIRECTORY)
    {
      /* It is a sub-directory. Check if we are be asked to remove
       * a directory or a file.
       */

      if (!directory)
        {
          /* We are asked to delete a file */

          return -EISDIR;
        }

      /* We are asked to delete a directory. Check if this
       * sub-directory is empty
       */

      dirinfo.dir.fd_currcluster = dircluster;
      dirinfo.dir.fd_currsector  = fat_cluster2sector(fs, dircluster);
      dirinfo.dir.fd_index       = 2;

      /* Loop until either (1) an entry is found in the directory
       * (error), (2) the directory is found to be empty, or (3) some
       * error occurs.
       */

      for (;;)
        {
          unsigned int subdirindex;
          uint8_t     *subdirentry;

          /* Make sure that the sector containing the of the
           * subdirectory sector is in the cache
           */

          ret = fat_fscacheread(fs, dirinfo.dir.fd_currsector);
          if (ret < 0)
            {
              return ret;
            }

          /* Get a reference to the next entry in the directory */

          subdirindex = (dirinfo.dir.fd_index & DIRSEC_NDXMASK(fs)) * DIR_SIZE;
          subdirentry = &fs->fs_buffer[subdirindex];

          /* Is this the last entry in the direcory? */

          if (subdirentry[DIR_NAME] == DIR0_ALLEMPTY)
            {
              /* Yes then the directory is empty.  Break out of the
               * loop and delete the directory.
               */

              break;
            }

          /* Check if the next entry refers to a file or directory */

          if (subdirentry[DIR_NAME] != DIR0_EMPTY &&
              !(DIR_GETATTRIBUTES(subdirentry) & FATATTR_VOLUMEID))
            {
              /* The directory is not empty */

              return -ENOTEMPTY;
            }

          /* Get the next directgory entry */

          ret = fat_nextdirentry(fs, &dirinfo.dir);
          if (ret < 0)
            {
              return ret;
            }
        }
    }
  else
    {
      /* It is a file. Check if we are be asked to remove a directory
       * or a file.
       */

      if (directory)
        {
          /* We are asked to remove a directory */

          return -ENOTDIR;
        }
    }

  /* Make sure that the directory containing the entry to be deleted is
   * in the cache.
   */

  ret = fat_fscacheread(fs, dirsector);
  if (ret < 0)
    {
      return ret;
    }

  /* Mark the directory entry 'deleted' */

  ret = fat_freedirentry(fs, dirinfo.fd_entry);
  if (ret < 0)
    {
      return ret;
    }

  /* And remove the cluster chain making up the subdirectory */

  ret = fat_removechain(fs, dircluster);
  if (ret < 0)
    {
      return ret;
    }

  /* Update the FSINFO sector (FAT32) */

  ret = fat_updatefsinfo(fs);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}
