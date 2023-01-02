/****************************************************************************
 * fs/cromfs/fs_cromfs.c
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
#include <sys/statfs.h>
#include <sys/stat.h>

#include <inttypes.h>
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
#include <nuttx/fs/ioctl.h>

#include "cromfs.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_CROMFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CROMFS_MAX_LINKS 64

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cromfs_dir_s
{
  struct fs_dirent_s cr_base; /* VFS directory structure */
  uint32_t cr_firstoffset;    /* Offset to the first entry in the directory */
  uint32_t cr_curroffset;     /* Current offset into the directory contents */
};

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
                                     uint32_t offset, FAR void *arg);

/* The cromfs_nodeinfo_s structure is an abbreviated version of
 * cromfs_node_s structure.
 */

struct cromfs_nodeinfo_s
{
  uint16_t ci_mode;      /* File type, attributes, and access mode bits */
  uint32_t ci_size;      /* Size of the uncompressed data (in bytes) */
  uint32_t ci_child;     /* Value associated with the directory file type */
};

/* This is the form of the argument provided to the cromfs_compare_node()
 * callback.
 */

struct cromfs_comparenode_s
{
  FAR struct cromfs_nodeinfo_s *info;   /* Location to return the node info */
  FAR const char *relpath;              /* Full relative path */
  FAR const char *segment;              /* Reference to start of the
                                         * path segment. */
  uint32_t offset;                      /* Physical offset in ROM */
  uint16_t seglen;                      /* Length of the next path segment */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static FAR void *cromfs_offset2addr(FAR const struct cromfs_volume_s *fs,
                  uint32_t offset);
static uint32_t cromfs_addr2offset(FAR const struct cromfs_volume_s *fs,
                  FAR const void *addr);
static int      cromfs_follow_link(FAR const struct cromfs_volume_s *fs,
                  FAR const struct cromfs_node_s **ppnode, bool follow,
                  FAR struct cromfs_node_s *newnode);
static int      cromfs_foreach_node(FAR const struct cromfs_volume_s *fs,
                  FAR const struct cromfs_node_s *node,
                  bool follow, cromfs_foreach_t callback, FAR void *arg);
static uint16_t cromfs_seglen(FAR const char *relpath);
static int      cromfs_child_node(FAR const struct cromfs_volume_s *fs,
                  FAR const struct cromfs_node_s *node,
                  FAR struct cromfs_nodeinfo_s *info);
static int      cromfs_compare_node(FAR const struct cromfs_volume_s *fs,
                  FAR const struct cromfs_node_s *node, uint32_t offset,
                  FAR void *arg);
static int      cromfs_find_node(FAR const struct cromfs_volume_s *fs,
                  FAR const char *relpath,
                  FAR struct cromfs_nodeinfo_s *info,
                  FAR uint32_t *offset);

/* Common file system methods */

static int      cromfs_open(FAR struct file *filep, const char *relpath,
                  int oflags, mode_t mode);
static int      cromfs_close(FAR struct file *filep);
static ssize_t  cromfs_read(FAR struct file *filep,
                  char *buffer, size_t buflen);
static int      cromfs_ioctl(FAR struct file *filep,
                  int cmd, unsigned long arg);

static int      cromfs_dup(FAR const struct file *oldp,
                  FAR struct file *newp);
static int      cromfs_fstat(FAR const struct file *filep,
                  FAR struct stat *buf);

static int      cromfs_opendir(FAR struct inode *mountpt,
                  FAR const char *relpath, FAR struct fs_dirent_s **dir);
static int      cromfs_closedir(FAR struct inode *mountpt,
                  FAR struct fs_dirent_s *dir);
static int      cromfs_readdir(FAR struct inode *mountpt,
                  FAR struct fs_dirent_s *dir,
                  FAR struct dirent *entry);
static int      cromfs_rewinddir(FAR struct inode *mountpt,
                  FAR struct fs_dirent_s *dir);

static int      cromfs_bind(FAR struct inode *blkdriver,
                  FAR const void *data, FAR void **handle);
static int      cromfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                  unsigned int flags);
static int      cromfs_statfs(FAR struct inode *mountpt,
                  FAR struct statfs *buf);

static int      cromfs_stat(FAR struct inode *mountpt,
                  FAR const char *relpath, FAR struct stat *buf);

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
  NULL,              /* truncate */
  NULL,              /* mmap */

  NULL,              /* sync */
  cromfs_dup,        /* dup */
  cromfs_fstat,      /* fstat */
  NULL,              /* fchstat */

  cromfs_opendir,    /* opendir */
  cromfs_closedir,   /* closedir */
  cromfs_readdir,    /* readdir */
  cromfs_rewinddir,  /* rewinddir */

  cromfs_bind,       /* bind */
  cromfs_unbind,     /* unbind */
  cromfs_statfs,     /* statfs */

  NULL,              /* unlink */
  NULL,              /* mkdir */
  NULL,              /* rmdir */
  NULL,              /* rename */
  cromfs_stat,       /* stat */
  NULL               /* chstat */
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
 *
 * Description:
 *   Convert an offset into an address in the CROMFS flat memory image.
 *
 ****************************************************************************/

static FAR void *cromfs_offset2addr(FAR const struct cromfs_volume_s *fs,
                                    uint32_t offset)
{
  /* Zero offset is a special case:  It corresponds to a NULL address */

  if (offset == 0 || offset >= fs->cv_fsize)
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
 * Name: cromfs_addr2offset
 *
 * Description:
 *   Convert a CROMFS flat image address into the file system offset.
 *
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
 * Name: cromfs_follow_link
 *
 * Description:
 *   If the node it a hardlink, then follow the node to the final target
 *   node which will be a directory or a file.
 *
 ****************************************************************************/

static int cromfs_follow_link(FAR const struct cromfs_volume_s *fs,
                              FAR const struct cromfs_node_s **ppnode,
                              bool follow,
                              FAR struct cromfs_node_s *newnode)
{
  FAR const struct cromfs_node_s *linknode;
  FAR const struct cromfs_node_s *node = *ppnode;
  FAR const char *name;
  int i;

  /* Loop while we are redirected by hardlinks */

  for (i = 0; i < CROMFS_MAX_LINKS; i++)
    {
      /* Check for a hard link */

      if ((node->cn_mode & S_IFMT) != S_IFLNK)
        {
          return OK;
        }

      /* Get the link target node */

      linknode = (FAR const struct cromfs_node_s *)
                  cromfs_offset2addr(fs, node->u.cn_link);
      DEBUGASSERT(linknode != NULL);

      /* Special case:  Don't follow either "." or ".."  These will generate
       * loops in both cases.
       *
       * REVISIT: This kludge is necessary due to an issue in gencromfs:
       * The "." entry and ".." refer to the first entry in the directory
       * list, ".", instead of to the directory entry itself.  Hence, we
       * cannot traverse "." or ".." to determine that these are
       * directories.  NOTE also that there is no root directory entry for
       * the top "." to refer to.
       */

      name = (FAR const char *)cromfs_offset2addr(fs, node->cn_name);
      if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0)
        {
          /* We assume this is a "." directory opener.  Create a directory
           * node on the stack.
           */

          newnode->cn_mode    = S_IFDIR | (node->cn_mode & ~S_IFMT);
          newnode->cn_pad     = 0;
          newnode->cn_name    = node->cn_name;
          newnode->cn_size    = 0;
          newnode->cn_peer    = node->cn_peer;
          newnode->u.cn_child = node->u.cn_child;

          /* Switch from the original read-only in ROM to the writable copy
           * in on the stack.
           */

          *ppnode             = newnode;
          return OK;
        }

      /* Copy the origin node file name into the writable node copy */

      newnode->cn_name   = node->cn_name;
      newnode->cn_pad    = 0;

      /* Copy all attributes of the target node, but retain the hard link
       * file name and, possibly, the peer node reference.
       */

      newnode->cn_mode   = linknode->cn_mode;
      newnode->cn_size   = linknode->cn_size;
      newnode->u.cn_link = linknode->u.cn_link;

      /* Copy the peer node offset, changing the peer only if we are
       * following the hard link in the traversal.
       */

      newnode->cn_peer   = follow ? linknode->cn_peer : node->cn_peer;

      /* Switch from the original read-only in ROM to the writable copy in
       * on the stack.
       */

      *ppnode            = newnode;
      node               = newnode;
    }

  return -ELOOP;
}

/****************************************************************************
 * Name: cromfs_foreach_node
 *
 * Description:
 *   Visit each node in the file system, performing the requested callback
 *   for each node.  The attributes of the hard link are replaced with the
 *   attributes of the target node.  Optionally, traversal can be forced to
 *   follow the hard link paths.
 *
 ****************************************************************************/

static int cromfs_foreach_node(FAR const struct cromfs_volume_s *fs,
                               FAR const struct cromfs_node_s *node,
                               bool follow, cromfs_foreach_t callback,
                               FAR void *arg)
{
  FAR const struct cromfs_node_s *pnode;
  struct cromfs_node_s newnode;
  uint32_t offset;
  int ret = OK;

  /* Traverse all entries in this directory (i.e., following the 'peer'
   * links).
   */

  pnode  = node;
  offset = cromfs_addr2offset(fs, node);

  while (pnode != NULL)
    {
      /* Follow any hard links */

      ret = cromfs_follow_link(fs, &pnode, follow, &newnode);
      if (ret < 0)
        {
          return ret;
        }

      /* Perform the callback for the node */

      ret = callback(fs, pnode, offset, arg);
      if (ret != OK)
        {
          return ret;
        }

      offset = pnode->cn_peer;
      pnode  = (FAR const struct cromfs_node_s *)
               cromfs_offset2addr(fs, offset);
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
 * Name: cromfs_child_node
 ****************************************************************************/

static int cromfs_child_node(FAR const struct cromfs_volume_s *fs,
                             FAR const struct cromfs_node_s *node,
                             FAR struct cromfs_nodeinfo_s *info)
{
  FAR const struct cromfs_node_s *pnode;
  FAR const struct cromfs_node_s *child;
  struct cromfs_node_s newnode;
  uint32_t offset;
  int ret;

  /* Get the child node referred by the directory entry */

  offset = node->u.cn_child;
  child  = (FAR const struct cromfs_node_s *)cromfs_offset2addr(fs, offset);

  /* Get the attributes of the child node by following the hard link.  This
   * first node under the directory will be the hard link ".".
   */

  pnode  = child;
  ret    = cromfs_follow_link(fs, &pnode, false, &newnode);
  if (ret < 0)
    {
      return ret;
    }

  info->ci_mode  = pnode->cn_mode;
  info->ci_size  = pnode->cn_size;
  info->ci_child = offset;
  return OK;
}

/****************************************************************************
 * Name: cromfs_compare_node
 ****************************************************************************/

static int cromfs_compare_node(FAR const struct cromfs_volume_s *fs,
                               FAR const struct cromfs_node_s *node,
                               uint32_t offset, FAR void *arg)
{
  FAR struct cromfs_comparenode_s *cpnode;
  FAR const struct cromfs_node_s *child;
  FAR char *name;
  int namlen;
  int ret;

  DEBUGASSERT(fs != NULL && node != NULL && arg != NULL);
  cpnode = (FAR struct cromfs_comparenode_s *)arg;

  /* Get the name of the node */

  name   = (FAR char *)cromfs_offset2addr(fs, node->cn_name);
  namlen = strlen(name);

  finfo("Compare %s to %s[0-%" PRIu16 "]\n", name, cpnode->segment,
        cpnode->seglen);

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
              /* This first node under the directory will be the hard
               * link ".".
               */

              ret = cromfs_child_node(fs, node, cpnode->info);
              if (ret < 0)
                {
                  return ret;
                }
            }
          else
            {
              cpnode->info->ci_mode  = node->cn_mode;
              cpnode->info->ci_size  = node->cn_size;
              cpnode->info->ci_child = node->u.cn_child;
            }
#else
            {
              /* This first node under the directory will be the hard
               * link ".".
               */

              ret = cromfs_child_node(fs, node, cpnode->info);
              if (ret < 0)
                {
                  return ret;
                }
            }
#endif

          cpnode->offset = offset;
          return 1;
        }

      /* A special case is if the path ends in "/".  In this case I suppose
       * we need to interpret the as matching as long as it is a directory?
       */

      if (segment[namlen] == '/' && segment[namlen = 1] == '\0')
        {
          cpnode->info->ci_mode  = node->cn_mode;
          cpnode->info->ci_size  = node->cn_size;
          cpnode->info->ci_child = node->u.cn_child;
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

      return cromfs_foreach_node(fs, child, true, cromfs_compare_node,
                                 cpnode);
    }

  return 0;  /* Keep looking in this directory */
}

/****************************************************************************
 * Name: cromfs_find_node
 *
 * Description:
 *   Find the CROMFS node at the provide mountpoint relative path.
 *
 ****************************************************************************/

static int cromfs_find_node(FAR const struct cromfs_volume_s *fs,
                            FAR const char *relpath,
                            FAR struct cromfs_nodeinfo_s *info,
                            FAR uint32_t *offset)
{
  struct cromfs_comparenode_s cpnode;
  FAR const struct cromfs_node_s *root;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Get the root node.  The root is the entry "." which is a hard link. */

  root = (FAR const struct cromfs_node_s *)
          cromfs_offset2addr(fs, fs->cv_root);

  /* NULL or empty string refers to the root node */

  if (relpath == NULL || relpath[0] == '\0')
    {
      struct cromfs_node_s newnode;

      /* Get the attributes of the root node by following the hard link.
       * We do this even though the attributes of the root node are well
       * defined.
       */

      ret = cromfs_follow_link(fs, &root, false, &newnode);
      if (ret < 0)
        {
          return ret;
        }

      info->ci_mode  = root->cn_mode;
      info->ci_size  = root->cn_size;
      info->ci_child = root->u.cn_child;
      *offset        = fs->cv_root;
      return OK;
    }

  /* Not the root directory.  Relative so it should not begin with '/'. */

  if (relpath[0] == '/')
    {
      return -EINVAL;
    }

  /* Set up for the traversal */

  cpnode.info    = info;
  cpnode.relpath = relpath;
  cpnode.segment = relpath;
  cpnode.offset  = fs->cv_root;
  cpnode.seglen  = (uint16_t)cromfs_seglen(relpath);

  ret = cromfs_foreach_node(fs, root, false, cromfs_compare_node, &cpnode);
  if (ret > 0)
    {
      *offset = cpnode.offset;
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
  struct cromfs_nodeinfo_s info;
  FAR struct cromfs_file_s *ff;
  uint32_t offset;
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

  ret = cromfs_find_node(fs, relpath, &info, &offset);
  if (ret < 0)
    {
      /* Nothing exists at that relative path (or a really bad error
       * occurred)
       */

      return ret;
    }

  /* Verify that the node is a regular file */

  if (!S_ISREG(info.ci_mode))
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

  ff->ff_node = (FAR const struct cromfs_node_s *)
    cromfs_offset2addr(fs, offset);

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

  finfo("Read %zu bytes from offset %jd\n", buflen, (intmax_t)filep->f_pos);
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
       * real search on the first time through but the remaining blocks
       * should be contiguous so that the logic should not loop.
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

      /* Check if we need to decompress the next block into the user
       * buffer.
       */

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

          finfo("blkoffs=%" PRIu32 " ulen=%" PRIu16 " copysize=%u\n",
                blkoffs, ulen, copysize);
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

              finfo("voloffs=%" PRIu32 " blkoffs=%" PRIu32
                    " ulen=%" PRIu16 " ff_offset=%" PRIu32 " copysize=%u\n",
                    voloffs, blkoffs, ulen, ff->ff_offset, copysize);
              DEBUGASSERT(ff->ff_ulen >= copysize);
            }
          else
            {
              uint32_t voloffs;

              /* No, we will need to decompress into the our intermediate
               * decompression buffer.
               */

              copyoffs = (blkoffs >= filep->f_pos) ?
                            0 : filep->f_pos - blkoffs;
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

              finfo("voloffs=%" PRIu32 " blkoffs=%" PRIu32 " ulen=%" PRIu16
                    " clen=%" PRIu16 " ff_offset=%" PRIu32
                    "  copyoffs=%u copysize=%u\n",
                    voloffs, blkoffs, ulen, clen, ff->ff_offset,
                    copyoffs, copysize);
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

  newff = kmm_zalloc(sizeof(struct cromfs_file_s));
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
                          FAR struct fs_dirent_s **dir)
{
  FAR const struct cromfs_volume_s *fs;
  FAR struct cromfs_dir_s *cdir;
  FAR struct cromfs_nodeinfo_s info;
  uint32_t offset;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Locate the node for this relative path */

  ret = cromfs_find_node(fs, relpath, &info, &offset);
  if (ret < 0)
    {
      /* Nothing exists at that relative path (or a really bad error
       * occurred)
       */

      return ret;
    }

  /* Verify that the node is a directory */

  if (!S_ISDIR(info.ci_mode))
    {
      return -ENOTDIR;
    }

  cdir = kmm_zalloc(sizeof(*cdir));
  if (cdir == NULL)
    {
      return -ENOMEM;
    }

  /* Set the start node and next node to the first entry in the directory */

  cdir->cr_firstoffset = info.ci_child;
  cdir->cr_curroffset  = info.ci_child;
  *dir = &cdir->cr_base;
  return OK;
}

/****************************************************************************
 * Name: cromfs_closedir
 *
 * Description: close directory.
 *
 ****************************************************************************/

static int cromfs_closedir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  DEBUGASSERT(mountpt != NULL);
  kmm_free(dir);
  return 0;
}

/****************************************************************************
 * Name: cromfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int cromfs_readdir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir,
                          FAR struct dirent *entry)
{
  FAR const struct cromfs_volume_s *fs;
  FAR const struct cromfs_node_s *node;
  FAR struct cromfs_dir_s *cdir;
  struct cromfs_node_s newnode;
  FAR char *name;
  uint32_t offset;
  int ret;

  finfo("mountpt: %p dir: %p\n", mountpt, dir);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;
  cdir = (FAR struct cromfs_dir_s *)dir;

  /* Have we reached the end of the directory */

  offset = cdir->cr_curroffset;
  if (offset == 0)
    {
      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      finfo("Entry %" PRIu32 ": End of directory\n", offset);
      return -ENOENT;
    }

  /* Convert the offset to a node address (assuming that everything is in-
   * memory)
   */

  node = (FAR const struct cromfs_node_s *)cromfs_offset2addr(fs, offset);
  if (node == NULL)
    {
      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      finfo("Entry %" PRIu32 ": End of directory\n", offset);
      return -ENOENT;
    }

  /* Get the attributes of the node by following the hard link. */

  ret = cromfs_follow_link(fs, &node, false, &newnode);
  if (ret < 0)
    {
      return ret;
    }

  /* Save the filename and file type */

  name = (FAR char *)cromfs_offset2addr(fs, node->cn_name);
  finfo("Entry %" PRIu32 ": %s\n", offset, name);
  strlcpy(entry->d_name, name, sizeof(entry->d_name));

  switch (node->cn_mode & S_IFMT)
    {
      case S_IFDIR:  /* Directory */
        entry->d_type = DTYPE_DIRECTORY;
        break;

      case S_IFREG:  /* Regular file */
        entry->d_type = DTYPE_FILE;
        break;

      case S_IFIFO:  /* FIFO */
        entry->d_type = DTYPE_FIFO;
        break;

      case S_IFCHR:  /* Character driver */
        entry->d_type = DTYPE_CHR;
        break;

      case S_IFBLK:  /* Block driver */
        entry->d_type = DTYPE_BLK;
        break;

      case S_IFMQ:   /* Message queue */
        entry->d_type = DTYPE_MQ;
        break;

      case S_IFSEM:  /* Semaphore */
        entry->d_type = DTYPE_SEM;
        break;

      case S_IFSHM:  /* Shared memory */
        entry->d_type = DTYPE_SHM;
        break;

      case S_IFMTD:  /* MTD driver */
        entry->d_type = DTYPE_MTD;
        break;

      case S_IFSOCK: /* Socket */
        entry->d_type = DTYPE_SOCK;
        break;

      default:
        DEBUGPANIC();
        entry->d_type = DTYPE_UNKNOWN;
        break;
    }

  /* Set up the next directory entry offset.  NOTE that we could use the
   * standard f_pos instead of our own private fb_index.
   */

  cdir->cr_curroffset = node->cn_peer;
  return OK;
}

/****************************************************************************
 * Name: cromfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int cromfs_rewinddir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct cromfs_dir_s *cdir;

  finfo("mountpt: %p dir: %p\n", mountpt, dir);

  cdir = (FAR struct cromfs_dir_s *)dir;
  cdir->cr_curroffset = cdir->cr_firstoffset;
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
  struct cromfs_nodeinfo_s info;
  uint32_t offset;
  int ret;

  finfo("mountpt: %p relpath: %s buf: %p\n", mountpt, relpath, buf);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && buf != NULL);
  memset(buf, 0, sizeof(struct stat));

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Locate the node for this relative path */

  ret  = cromfs_find_node(fs, relpath, &info, &offset);
  if (ret >= 0)
    {
      /* Return the struct stat info associate with this node */

      buf->st_mode    = info.ci_mode;
      buf->st_size    = info.ci_size;
      buf->st_blksize = fs->cv_bsize;
      buf->st_blocks  = (info.ci_size + (fs->cv_bsize - 1)) / fs->cv_bsize;
      ret             = OK;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_CROMFS */
