/****************************************************************************
 * fs/cromfs/fs_cromfs.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <lzf.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/binfmt/builtin.h>

#include "cromfs.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_CROMFS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents an open, regular file */

struct cromfs_file_s
{
  FAR const struct cromfs_node_s *ff_node;  /* The open file node */
  uint32_t ff_offset;                       /* Cached block offset (zero means none) */
  uint16_t ff_ulen;                         /* Length of decompressed data in cache */
  FAR uint8_t *ff_buffer;                   /* Cached, decompressed data */
};

/* This is the form of the callback from cromfs_foreach_node(): */

typedef CODE int (*cromfs_foreach_t)(FAR const struct cromfs_volume_s *fs,
                                     FAR const struct cromfs_node_s *node,
                                     FAR void *arg);

/* This is the form of the argument provided to the cromfs_comparenode()
 * callback.
 */

struct cromfs_comparenode_s
{
  FAR const struct cromfs_node_s **node;   /* Location to return the node */
  FAR const char *relpath;                 /* Full relative path */
  FAR const char *segment;                 /* Reference to start of the
                                            * path segment. */
  uint16_t seglen;                         /* Length of the next path segment */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static FAR void *cromfs_offset2addr(FAR const struct cromfs_volume_s *fs,
                                    uint32_t offset);
static uint32_t cromfs_addr2offset(FAR const struct cromfs_volume_s *fs,
                                   FAR const void *addr);
static int      cromfs_foreach_node(FAR const struct cromfs_volume_s *fs,
                                    FAR const struct cromfs_node_s *node,
                                    cromfs_foreach_t callback, FAR void *arg);
static uint16_t cromfs_seglen(FAR const char *relpath);
static int      cromfs_comparenode(FAR const struct cromfs_volume_s *fs,
                                   FAR const struct cromfs_node_s *node,
                                   FAR void *arg);
static int      cromfs_findnode(FAR const struct cromfs_volume_s *fs,
                                FAR const struct cromfs_node_s **node,
                                FAR const char *relpath);

/* Common file system methods */

static int      cromfs_open(FAR struct file *filep, const char *relpath,
                            int oflags, mode_t mode);
static int      cromfs_close(FAR struct file *filep);
static ssize_t  cromfs_read(FAR struct file *filep, char *buffer, size_t buflen);
static int      cromfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int      cromfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int      cromfs_fstat(FAR const struct file *filep, FAR struct stat *buf);

static int      cromfs_opendir(struct inode *mountpt, const char *relpath,
                               struct fs_dirent_s *dir);
static int      cromfs_readdir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);
static int      cromfs_rewinddir(FAR struct inode *mountpt,
                                 FAR struct fs_dirent_s *dir);

static int      cromfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                            FAR void **handle);
static int      cromfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                              unsigned int flags);
static int      cromfs_statfs(FAR struct inode *mountpt,
                              FAR struct statfs *buf);

static int      cromfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                            FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations cromfs_operations =
{
  cromfs_open,       /* open */
  cromfs_close,      /* close */
  cromfs_read,       /* read */
  NULL,              /* write */
  NULL,              /* seek */
  cromfs_ioctl,      /* ioctl */

  NULL,              /* sync */
  cromfs_dup,        /* dup */
  cromfs_fstat,      /* fstat */
  NULL,              /* truncate */

  cromfs_opendir,    /* opendir */
  NULL,              /* closedir */
  cromfs_readdir,    /* readdir */
  cromfs_rewinddir,  /* rewinddir */

  cromfs_bind,       /* bind */
  cromfs_unbind,     /* unbind */
  cromfs_statfs,     /* statfs */

  NULL,              /* unlink */
  NULL,              /* mkdir */
  NULL,              /* rmdir */
  NULL,              /* rename */
  cromfs_stat        /* stat */
};

/* The CROMFS uses a global, in-memory instance of the file system image
 * rather than a ROMDISK as does, same the ROMFS file system.  This is
 * primarily because the compression logic needs contiguous, in-memory
 * data.  One consequence of this is that there can only only be a single
 * CROMFS file system in the build.
 *
 * This is the address of the single CROMFS file system image:
 */

extern const struct cromfs_volume_s g_cromfs_image;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cromfs_offset2addr
 ****************************************************************************/

static FAR void *cromfs_offset2addr(FAR const struct cromfs_volume_s *fs,
                                    uint32_t offset)
{
  /* Zero offset is a specials case:  It corresponds to a NULL address */

  if (offset == 0)
    {
      return NULL;
    }

  /* The root node lies at the beginning of the CROMFS image so we can
   * convert the offset into the image by simply adding the the address
   * of the root node.
   */

  DEBUGASSERT(offset < fs->cv_fsize);
  return (FAR void *)((FAR uint8_t *)fs + offset);
}

/****************************************************************************
 * Name: cromfs_offset2addr
 ****************************************************************************/

static uint32_t cromfs_addr2offset(FAR const struct cromfs_volume_s *fs,
                                   FAR const void *addr)
{
  uintptr_t start;
  uintptr_t target;
  uint32_t offset;

  /* NULL is a specials case:  It corresponds to offset zero */

  if (addr == NULL)
    {
      return 0;
    }

  /* Make sure that the address is "after" the start of file system image. */

  start  = (uintptr_t)fs;
  target = (uintptr_t)addr;
  DEBUGASSERT(target >= start);

  /* Get the non-zero offset and make sure that the offset is within the file
   * system image.
   */

  offset = target - start;
  DEBUGASSERT(offset < fs->cv_fsize);
  return offset;
}

/****************************************************************************
 * Name: cromfs_foreach_node
 ****************************************************************************/

static int cromfs_foreach_node(FAR const struct cromfs_volume_s *fs,
                               FAR const struct cromfs_node_s *node,
                               cromfs_foreach_t callback, FAR void *arg)
{
  int ret = OK;

  /* Traverse all entries in this directory (i.e., following the 'peer'
   * links).
   */

  while (node != NULL)
    {
      ret = callback(fs, node, arg);
      if (ret != OK)
        {
          return ret;
        }

      node = (FAR const struct cromfs_node_s *)
             cromfs_offset2addr(fs, node->cn_peer);
    }

  return ret;
}

/****************************************************************************
 * Name: cromfs_seglen
 ****************************************************************************/

static uint16_t cromfs_seglen(FAR const char *relpath)
{
  FAR char *delimiter;
  int len;

  delimiter = strchr(relpath, '/');
  if (delimiter == NULL)
    {
      len = strlen(relpath);
    }
  else
    {
      len = (int)((uintptr_t)delimiter - (uintptr_t)relpath);
    }

  DEBUGASSERT((unsigned int)len <= UINT16_MAX);
  return (uint16_t)len;
}

/****************************************************************************
 * Name: cromfs_comparenode
 ****************************************************************************/

static int cromfs_comparenode(FAR const struct cromfs_volume_s *fs,
                              FAR const struct cromfs_node_s *node,
                              FAR void *arg)
{
  FAR struct cromfs_comparenode_s *cpnode;
  FAR const struct cromfs_node_s *child;
  FAR char *name;
  int namlen;

  DEBUGASSERT(fs != NULL && node != NULL && arg != NULL);
  cpnode = (FAR struct cromfs_comparenode_s *)arg;

  /* Get the name of the node */

  name   = (FAR char *)cromfs_offset2addr(fs, node->cn_name);
  namlen = strlen(name);

  finfo("Compare %s to %s[0-%u]\n", name, cpnode->segment, cpnode->seglen);

  /* If the lengths of the name does not match the length of the next path
   * segment, then this is not the node we are looking for.
   */

  if (namlen != cpnode->seglen)
    {
      return 0;  /* Keep looking */
    }

  /* The are the same length... are they the same? */

  if (strncmp(name, cpnode->segment, namlen) == 0)
    {
      FAR const char *segment = cpnode->segment;

      /* Got it!  Was this the last segment of the path?  If so, then
       * the segment length is equal to the length of the remaining
       * relpath and we should find a NUL terminator at that location.
       */

      if (segment[namlen] == '\0')
        {
          /* We have it.  Save the terminal node with the final match
           * and return 1 to stop the traversal.
           */

#if 1 /* REVISIT:  This seems to work, but I don't fully follow the logic. */
          if (S_ISDIR(node->cn_mode))
            {
              *cpnode->node = (FAR const struct cromfs_node_s *)
                              cromfs_offset2addr(fs, node->u.cn_child);
            }
          else
            {
              *cpnode->node = node;
            }
#else
          *cpnode->node = (FAR const struct cromfs_node_s *)
                          cromfs_offset2addr(fs, node->u.cn_child);
#endif
          return 1;
        }

      /* A special case is if the path ends in "/".  In this case I suppose
       * we need to interpret the as matching as long as it is a directory?
       */

      if (segment[namlen] == '/' && segment[namlen = 1] == '\0')
        {
          *cpnode->node = node;
          return S_ISDIR(node->cn_mode) ? 1 : -ENOENT;
        }

      /* If this is a valid, non-terminal segment on the path, then it must
       * be a directory.
       */

      if (!S_ISDIR(node->cn_mode))
        {
          /* Terminate the traversal with an error */

          return -ENOTDIR;
        }

      /* Set up to continue the traversal in the sub-directory.  NOTE that
       * this recurses and could potentially eat up a lot of stack.
       */

      child   = (FAR const struct cromfs_node_s *)
                 cromfs_offset2addr(fs, node->u.cn_child);
      segment = cpnode->segment + cpnode->seglen;

      /* Skip over any '/' delimiter */

      while (*segment == '/')
        {
          segment++;
        }

      cpnode->segment = segment;
      cpnode->seglen  = cromfs_seglen(segment);
      DEBUGASSERT(cpnode->seglen > 0);

      /* Then recurse */

      return cromfs_foreach_node(fs, child, cromfs_comparenode, cpnode);
    }
  else
    {
      return 0;  /* Keep looking in this directory */
    }
}

/****************************************************************************
 * Name: cromfs_findnode
 ****************************************************************************/

static int cromfs_findnode(FAR const struct cromfs_volume_s *fs,
                           FAR const struct cromfs_node_s **node,
                           FAR const char *relpath)
{
  struct cromfs_comparenode_s cpnode;
  FAR const struct cromfs_node_s *root;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Get the root node */

  root = (FAR const struct cromfs_node_s *)
          cromfs_offset2addr(fs, fs->cv_root);

  /* NULL or empty string refers to the root node */

  if (relpath == NULL || relpath[0] == '\0')
    {
      *node = root;
      return OK;
    }

  /* Not the root directory.  Relative so it should not begin with '/'. */

  if (relpath[0] == '/')
    {
      return -EINVAL;
    }

  /* Set up for the traversal */

  cpnode.node    = node;
  cpnode.relpath = relpath;
  cpnode.segment = relpath;
  cpnode.seglen  = (uint16_t)cromfs_seglen(relpath);

  ret = cromfs_foreach_node(fs, root, cromfs_comparenode, &cpnode);
  if (ret > 0)
    {
      return OK;
    }
  else if (ret == OK)
    {
      return -ENOENT;
    }
  else
    {
      return ret;
    }
}

/****************************************************************************
 * Name: cromfs_open
 ****************************************************************************/

static int cromfs_open(FAR struct file *filep, FAR const char *relpath,
                       int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR const struct cromfs_volume_s *fs;
  FAR const struct cromfs_node_s *node;
  FAR struct cromfs_file_s *ff;
  int ret;

  finfo("Open: %s\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* CROMFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

 /* Locate the node for this relative path */

  node = NULL;
  ret  = cromfs_findnode(fs, &node, relpath);
  if (ret < 0)
    {
      /* Nothing exists at that relative path (or a really bad error occurred) */

      return ret;
    }

  DEBUGASSERT(node != NULL);

  /* Verify that the node is a regular file */

  if (!S_ISREG(node->cn_mode))
    {
      return -EISDIR;
    }

  /* Create an instance of the file private date to describe the opened
   * file.
   */

  ff = (FAR struct cromfs_file_s *)kmm_zalloc(sizeof(struct cromfs_file_s));
  if (ff == NULL)
    {
      return -ENOMEM;
    }

  /* Create a file buffer to support partial sector accesses */

  ff->ff_buffer = (FAR uint8_t *)kmm_malloc(fs->cv_bsize);
  if (!ff->ff_buffer)
    {
      kmm_free(ff);
      return -ENOMEM;
    }

  /* Save the node in the open file instance */

  ff->ff_node = node;

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)ff;
  return OK;
}

/****************************************************************************
 * Name: cromfs_close
 ****************************************************************************/

static int cromfs_close(FAR struct file *filep)
{
  FAR struct cromfs_file_s *ff;

  finfo("Closing\n");
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the open file instance from the file structure */

  ff = filep->f_priv;
  DEBUGASSERT(ff->ff_node != NULL && ff->ff_buffer != NULL);

  /* Free all resources consumed by the opened file */

  kmm_free(ff->ff_buffer);
  kmm_free(ff);

  return OK;
}

/****************************************************************************
 * Name: cromfs_read
 ****************************************************************************/

static ssize_t cromfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR const struct cromfs_volume_s *fs;
  FAR struct cromfs_file_s *ff;
  FAR struct lzf_header_s *currhdr;
  FAR struct lzf_header_s *nexthdr;
  FAR uint8_t *dest;
  FAR const uint8_t *src;
  off_t fpos;
  size_t remaining;
  uint32_t blkoffs;
  uint16_t ulen;
  uint16_t clen;
  unsigned int copysize;
  unsigned int copyoffs;

  finfo("Read %d bytes from offset %d\n", buflen, filep->f_pos);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get the open file instance from the file structure */

  ff = (FAR struct cromfs_file_s *)filep->f_priv;
  DEBUGASSERT(ff->ff_node != NULL && ff->ff_buffer != NULL);

  /* Check for a read past the end of the file */

  if (filep->f_pos > ff->ff_node->cn_size)
    {
      /* Start read position is past the end of file.  Return the end-of-
       * file indication.
       */

      return 0;
    }
  else if ((filep->f_pos + buflen) > ff->ff_node->cn_size)
    {
      /* The final read position is past the end of file.  Truncate the
       * read length.
       */

      buflen = ff->ff_node->cn_size - filep->f_pos;
    }

  /* Find the compressed block containing the current offset, f_pos */

  dest      = (FAR uint8_t *)buffer;
  remaining = buflen;
  fpos      = filep->f_pos;
  blkoffs   = 0;
  ulen      = 0;
  nexthdr   = (FAR struct lzf_header_s *)
               cromfs_offset2addr(fs, ff->ff_node->u.cn_blocks);

  /* Look until we find the compressed block containing the start of the
   * requested data.
   */

  while (remaining > 0)
    {
      /* Search for the next block containing the fpos file offset.  This is
       * real search on the first time through but the remaining blocks should
       * be contiguous so that the logic should not loop.
       *
       */

      do
        {
          uint32_t blksize;

          /* Go to the next block */

          currhdr  = nexthdr;
          blkoffs += ulen;

          if (currhdr->lzf_type == LZF_TYPE0_HDR)
            {
              FAR struct lzf_type0_header_s *hdr0 =
                (FAR struct lzf_type0_header_s *)currhdr;

              ulen    = (uint16_t)hdr0->lzf_len[0] << 8 |
                        (uint16_t)hdr0->lzf_len[1];
              blksize = (uint32_t)ulen + LZF_TYPE0_HDR_SIZE;
            }
          else
            {
              FAR struct lzf_type1_header_s * hdr1 =
                (FAR struct lzf_type1_header_s *)currhdr;

              ulen    = (uint16_t)hdr1->lzf_ulen[0] << 8 |
                        (uint16_t)hdr1->lzf_ulen[1];
              clen    = (uint16_t)hdr1->lzf_clen[0] << 8 |
                        (uint16_t)hdr1->lzf_clen[1];
              blksize = (uint32_t)clen + LZF_TYPE1_HDR_SIZE;
            }

          nexthdr  = (FAR struct lzf_header_s *)
                     ((FAR uint8_t *)currhdr + blksize);
        }
      while (fpos >= (blkoffs + ulen));

      /* Check if we need to decompress the next block into the user buffer. */

      if (currhdr->lzf_type == LZF_TYPE0_HDR)
        {
          /* Just copy the uncompressed data copy data from image to the
           * user buffer.
           */

          copyoffs = (blkoffs >= filep->f_pos) ? 0 : filep->f_pos - blkoffs;
          DEBUGASSERT(ulen > copyoffs);
          copysize = ulen - copyoffs;

          if (copysize > remaining)  /* Clip to the size really needed */
            {
              copysize = remaining;
            }

          src = (FAR const uint8_t *)currhdr + LZF_TYPE0_HDR_SIZE;
          memcpy(dest, &src[copyoffs], copysize);

          finfo("blkoffs=%lu ulen=%u copysize=%u\n",
                (unsigned long)blkoffs, ulen, copysize);
        }
      else
        {
          /* If the source of the data is at the beginning of the compressed
           * data buffer and if the uncompressed data would not overrun the
           * buffer, then we can decompress directly into the user buffer.
           */

          if (filep->f_pos <= blkoffs && ulen <= remaining)
            {
              uint32_t voloffs;

              copyoffs = 0;
              copysize = ulen;

              /* Get the address and offset in the CROMFS image to obtain
               * the data.  Check if we already have this offset in the
               * cache.
               */

              src     = (FAR const uint8_t *)currhdr + LZF_TYPE1_HDR_SIZE;
              voloffs = cromfs_addr2offset(fs, src);
              if (voloffs != ff->ff_offset)
                {
                  unsigned int decomplen;

                  decomplen = lzf_decompress(src, clen, dest, fs->cv_bsize);

                  ff->ff_offset = voloffs;
                  ff->ff_ulen   = decomplen;
                }

              finfo("voloffs=%lu blkoffs=%lu ulen=%u ff_offset=%u copysize=%u\n",
                    (unsigned long)voloffs, (unsigned long)blkoffs, ulen,
                    ff->ff_offset, copysize);
              DEBUGASSERT(ff->ff_ulen >= copysize);
            }
          else
            {
              uint32_t voloffs;

              /* No, we will need to decompress into the our intermediate
               * decompression buffer.
               */

              copyoffs = (blkoffs >= filep->f_pos) ? 0 : filep->f_pos - blkoffs;
              DEBUGASSERT(ulen > copyoffs);
              copysize = ulen - copyoffs;

              if (copysize > remaining)  /* Clip to the size really needed */
                {
                  copysize = remaining;
                }

              DEBUGASSERT((copyoffs + copysize) <=  fs->cv_bsize);

              src = (FAR const uint8_t *)currhdr + LZF_TYPE1_HDR_SIZE;
              voloffs = cromfs_addr2offset(fs, src);
              if (voloffs != ff->ff_offset)
                {
                  unsigned int decomplen;

                  decomplen = lzf_decompress(src, clen, ff->ff_buffer,
                                             fs->cv_bsize);

                  ff->ff_offset = voloffs;
                  ff->ff_ulen   = decomplen;
                }

              finfo("voloffs=%lu blkoffs=%lu ulen=%u clen=%u ff_offset=%u "
                    "copyoffs=%u copysize=%u\n",
                    (unsigned long)voloffs, (unsigned long)blkoffs, ulen,
                    clen, ff->ff_offset, copyoffs, copysize);
              DEBUGASSERT(ff->ff_ulen >= (copyoffs + copysize));

              /* Then copy to user buffer */

              memcpy(dest, &ff->ff_buffer[copyoffs], copysize);
            }
        }

      /* Adjust pointers counts and offset */

      dest      += copysize;
      remaining -= copysize;
      fpos      += copysize;
    }

  /* Update the file pointer */

  filep->f_pos = fpos;
  return buflen;
}

/****************************************************************************
 * Name: cromfs_ioctl
 ****************************************************************************/

static int cromfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  finfo("cmd: %d arg: %08lx\n", cmd, arg);

  /* No IOCTL commands yet supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: cromfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int cromfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct cromfs_volume_s *fs;
  FAR struct cromfs_file_s *oldff;
  FAR struct cromfs_file_s *newff;

  finfo("Dup %p->%p\n", oldp, newp);
  DEBUGASSERT(oldp->f_priv != NULL && oldp->f_inode != NULL &&
              newp->f_priv == NULL && newp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  fs = (FAR struct cromfs_volume_s *)oldp->f_inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get the open file instance from the file structure */

  oldff = oldp->f_priv;
  DEBUGASSERT(oldff->ff_node != NULL && oldff->ff_buffer != NULL);

  /* Allocate and initialize an new open file instance referring to the
   * same node.
   */

  newff = (FAR struct cromfs_file_s *)kmm_zalloc(sizeof(struct cromfs_file_s));
  if (newff == NULL)
    {
      return -ENOMEM;
    }

  /* Create a file buffer to support partial sector accesses */

  newff->ff_buffer = (FAR uint8_t *)kmm_malloc(fs->cv_bsize);
  if (newff->ff_buffer == NULL)
    {
      kmm_free(newff);
      return -ENOMEM;
    }

  /* Save the node in the open file instance */

  newff->ff_node = oldff->ff_node;

  /* Copy the index from the old to the new file structure */

  newp->f_priv = newff;
  return OK;
}

/****************************************************************************
 * Name: cromfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int cromfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  FAR struct cromfs_volume_s *fs;
  FAR struct cromfs_file_s *ff;
  uint32_t fsize;
  uint32_t bsize;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume private data from the inode structure
   */

  ff              = filep->f_priv;
  DEBUGASSERT(ff->ff_node != NULL && ff->ff_buffer != NULL);

  inode           = filep->f_inode;
  fs              = inode->i_private;

  /* Return the stat info */

  fsize           = ff->ff_node->cn_size;
  bsize           = fs->cv_bsize;

  buf->st_mode    = ff->ff_node->cn_mode;
  buf->st_size    = fsize;
  buf->st_blksize = bsize;
  buf->st_blocks  = (fsize + (bsize - 1)) / bsize;
  buf->st_atime   = 0;
  buf->st_mtime   = 0;
  buf->st_ctime   = 0;

  return OK;
}

/****************************************************************************
 * Name: cromfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int cromfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s *dir)
{
  FAR const struct cromfs_volume_s *fs;
  FAR const struct cromfs_node_s *node;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

 /* Locate the node for this relative path */

  node = NULL;
  ret  = cromfs_findnode(fs, &node, relpath);
  if (ret < 0)
    {
      /* Nothing exists at that relative path (or a really bad error occurred) */

      return ret;
    }

  DEBUGASSERT(node != NULL);

  /* Verify that the node is a directory */

  if (!S_ISDIR(node->cn_mode))
    {
      return -ENOTDIR;
    }

  /* Set the start node and next node to the first entry in the directory */

  dir->u.cromfs.cr_firstoffset = cromfs_addr2offset(fs, node);
  dir->u.cromfs.cr_curroffset  = dir->u.cromfs.cr_firstoffset;
  return OK;
}

/****************************************************************************
 * Name: cromfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int cromfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  FAR const struct cromfs_volume_s *fs;
  FAR const struct cromfs_node_s *node;
  FAR char *name;
  uint32_t offset;

  finfo("mountpt: %p dir: %p\n", mountpt, dir);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Have we reached the end of the directory */

  offset = dir->u.cromfs.cr_curroffset;
  if (offset == 0)
    {
      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      finfo("Entry %d: End of directory\n", offset);
      return -ENOENT;
    }

  /* Convert the offset to a node address (assuming that everything is in-
   * memory)
   */

  node = (FAR const struct cromfs_node_s *)cromfs_offset2addr(fs, offset);

  /* Save the filename and file type */

  name = (FAR char *)cromfs_offset2addr(fs, node->cn_name);
  finfo("Entry %lu: %s\n", (unsigned long)offset, name);
  strncpy(dir->fd_dir.d_name, name, NAME_MAX + 1);

  switch (node->cn_mode & s_IFTGT)
    {
      case S_IFDIR:  /* Directory */
        dir->fd_dir.d_type = DTYPE_DIRECTORY;
        break;

      case S_IFREG:  /* Regular file */
        dir->fd_dir.d_type = DTYPE_FILE;
        break;

      case S_IFIFO:  /* FIFO */
      case S_IFCHR:  /* Character driver */
      case S_IFBLK:  /* Block driver */
   /* case S_IFSOCK:    Socket */
      case S_IFMQ:   /* Message queue */
      case S_IFSEM:  /* Semaphore */
      case S_IFSHM:  /* Shared memory */
      default:
        DEBUGPANIC();
        dir->fd_dir.d_type = DTYPE_UNKNOWN;
        break;
    }

  /* Set up the next directory entry offset.  NOTE that we could use the
   * standard f_pos instead of our own private fb_index.
   */

  dir->u.cromfs.cr_curroffset = node->cn_peer;
  return OK;
}

/****************************************************************************
 * Name: cromfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int cromfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  finfo("mountpt: %p dir: %p\n", mountpt, dir);

  dir->u.cromfs.cr_curroffset  = dir->u.cromfs.cr_firstoffset;
  return OK;
}

/****************************************************************************
 * Name: cromfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int cromfs_bind(FAR struct inode *blkdriver, const void *data,
                      void **handle)
{
  finfo("blkdriver: %p data: %p handle: %p\n", blkdriver, data, handle);

  DEBUGASSERT(blkdriver == NULL && handle != NULL);
  DEBUGASSERT(g_cromfs_image.cv_magic == CROMFS_MAGIC);

  /* Return the new file system handle */

  *handle = (FAR void *)&g_cromfs_image;
  return OK;
}

/****************************************************************************
 * Name: cromfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int cromfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                        unsigned int flags)
{
  finfo("handle: %p blkdriver: %p flags: %02x\n",
        handle, blkdriver, flags);
  return OK;
}

/****************************************************************************
 * Name: cromfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int cromfs_statfs(struct inode *mountpt, struct statfs *buf)
{
  FAR struct cromfs_volume_s *fs;

  finfo("mountpt: %p buf: %p\n", mountpt, buf);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs             = mountpt->i_private;

  /* Fill in the statfs info. */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = CROMFS_MAGIC;
  buf->f_namelen = NAME_MAX;
  buf->f_bsize   = fs->cv_bsize;
  buf->f_blocks  = fs->cv_nblocks;
  buf->f_files   = fs->cv_nnodes;
  return OK;
}

/****************************************************************************
 * Name: cromfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int cromfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct stat *buf)
{
  FAR const struct cromfs_volume_s *fs;
  FAR const struct cromfs_node_s *node;
  int ret;

  finfo("mountpt: %p relpath: %s buf: %p\n", mountpt, relpath, buf);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && buf != NULL );
  memset(buf, 0, sizeof(struct stat));

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Locate the node for this relative path */

  node = NULL;
  ret  = cromfs_findnode(fs, &node, relpath);
  if (ret >= 0)
    {
      DEBUGASSERT(node != NULL);

      /* Return the struct stat info associate with this node */

      buf->st_mode    = node->cn_mode;
      buf->st_size    = node->cn_size;
      buf->st_blksize = fs->cv_bsize;
      buf->st_blocks  = (node->cn_size + (fs->cv_bsize - 1)) / fs->cv_bsize;
      ret             = OK;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_CROMFS */
