/****************************************************************************
 * fs/vfs/fs_rename.c
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

#include <sys/stat.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <libgen.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef FS_HAVE_RENAME
#if !defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
#  define FS_HAVE_RENAME 1
#endif

#ifdef FS_HAVE_RENAME

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pseudorename
 *
 * Description:
 *   Rename an inode in the pseudo file system
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pseudorename(FAR const char *oldpath, FAR struct inode *oldinode,
                        FAR const char *newpath)
{
  struct inode_search_s newdesc;
  FAR struct inode *newinode;
  FAR char *subdir = NULL;
  int ret;

  /* According to POSIX, any old inode at this path should be removed
   * first, provided that it is not a directory.
   */

next_subdir:
  SETUP_SEARCH(&newdesc, newpath, true);
  ret = inode_find(&newdesc);
  if (ret >= 0)
    {
      /* We found it.  Get the search results */

      newinode = newdesc.node;
      DEBUGASSERT(newinode != NULL);

      /* If the old and new inodes are the same, then this is an attempt to
       * move the directory entry onto itself.  Let's not but say we did.
       */

      if (oldinode == newinode)
        {
          inode_release(newinode);
          ret = OK;
          goto errout; /* Bad naming, this is not an error case. */
        }

#ifndef CONFIG_DISABLE_MOUNTPOINT
      /* Make sure that the old path does not lie on a mounted volume. */

      if (INODE_IS_MOUNTPT(newinode))
        {
          inode_release(newinode);
          ret = -EXDEV;
          goto errout;
        }
#endif

      /* We found it and it appears to be a "normal" inode.  Is it a
       * directory (i.e, an operation-less inode or an inode with children)?
       */

      if (newinode->u.i_ops == NULL || newinode->i_child != NULL)
        {
          FAR char *subdirname;
          FAR char *tmp;

          inode_release(newinode);

          /* Yes.. In this case, the target of the rename must be a
           * subdirectory of newinode, not the newinode itself.  For
           * example: mv b a/ must move b to a/b.
           */

          subdirname = basename((FAR char *)oldpath);
          tmp        = subdir;
          subdir     = NULL;

          asprintf(&subdir, "%s/%s", newpath, subdirname);

          if (tmp != NULL)
            {
              kmm_free(tmp);
            }

          if (subdir == NULL)
            {
              ret = -ENOMEM;
              goto errout;
            }

          newpath = subdir;

          /* This can be a recursive case, another inode may already exist
           * at oldpth/subdirname.  In that case, we need to do this all
           * over again.  A nasty goto is used because I am lazy.
           */

          RELEASE_SEARCH(&newdesc);
          goto next_subdir;
        }
      else
        {
          /* Not a directory... remove it.  It may still be something
           * important (like a driver), but we will just have to suffer
           * the consequences.
           *
           * NOTE (1) that we not bother to check the error.  If we
           * failed to remove the inode for some reason, then
           * inode_reserve() will complain below, and (2) the inode
           * won't really be removed until we call inode_release();
           */

          inode_remove(newpath);
        }

      inode_release(newinode);
    }

  /* Create a new, empty inode at the destination location.
   * NOTE that the new inode will be created with a reference count
   * of  zero.
   */

  ret = inode_semtake();
  if (ret < 0)
    {
      goto errout;
    }

  ret = inode_reserve(newpath, 0777, &newinode);
  if (ret < 0)
    {
      /* It is an error if a node at newpath already exists in the tree
       * OR if we fail to allocate memory for the new inode (and possibly
       * any new intermediate path segments).
       */

      ret = -EEXIST;
      goto errout_with_sem;
    }

  /* Copy the inode state from the old inode to the newly allocated inode */

  newinode->i_child   = oldinode->i_child;   /* Link to lower level inode */
  newinode->i_flags   = oldinode->i_flags;   /* Flags for inode */
  newinode->u.i_ops   = oldinode->u.i_ops;   /* Inode operations */
#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  newinode->i_mode    = oldinode->i_mode;    /* Access mode flags */
  newinode->i_owner   = oldinode->i_owner;   /* Owner */
  newinode->i_group   = oldinode->i_group;   /* Group */
  newinode->i_atime   = oldinode->i_atime;   /* Time of last access */
  newinode->i_mtime   = oldinode->i_mtime;   /* Time of last modification */
  newinode->i_ctime   = oldinode->i_ctime;   /* Time of last status change */
#endif
  newinode->i_private = oldinode->i_private; /* Per inode driver private data */

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  /* Prevent the link target string from being deallocated.  The pointer to
   * the allocated link target path was copied above (under the guise of
   * u.i_ops).  Now we must nullify the u.i_link pointer so that it is not
   * deallocated when inode_free() is (eventually called.
   */

  oldinode->u.i_link  = NULL;
#endif

  /* We now have two copies of the inode.  One with a reference count of
   * zero (the new one), and one that may have multiple references
   * including one by this logic (the old one)
   *
   * Remove the old inode.  Because we hold a reference count on the
   * inode, it will not be deleted now.  It will be deleted when all of
   * the references to the inode have been released (perhaps when
   * inode_release() is called in remove()).  inode_remove() should return
   * -EBUSY to indicate that the inode was not deleted now.
   */

  ret = inode_remove(oldpath);
  if (ret < 0 && ret != -EBUSY)
    {
      /* Remove the new node we just recreated */

      inode_remove(newpath);
      goto errout_with_sem;
    }

  /* Remove all of the children from the unlinked inode */

  oldinode->i_child  = NULL;
  oldinode->i_parent = NULL;
  ret = OK;

errout_with_sem:
  inode_semgive();

errout:
  RELEASE_SEARCH(&newdesc);

  if (subdir != NULL)
    {
      kmm_free(subdir);
    }

  return ret;
}

#endif /* CONFIG_DISABLE_PSEUDOFS_OPERATIONS */

/****************************************************************************
 * Name: mountptrename
 *
 * Description:
 *   Rename a file residing on a mounted volume.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static int mountptrename(FAR const char *oldpath, FAR struct inode *oldinode,
                         FAR const char *oldrelpath, FAR const char *newpath)
{
  struct inode_search_s newdesc;
  FAR struct inode *newinode;
  FAR const char *newrelpath;
  FAR char *subdir = NULL;
  int ret;

  DEBUGASSERT(oldinode->u.i_mops);

  /* If the file system does not support the rename() method, then bail now.
   * As of this writing, only NXFFS does not support the rename method.  A
   * good fallback might be to copy the oldrelpath to the correct location,
   * then unlink it.
   */

  if (oldinode->u.i_mops->rename == NULL)
    {
      return -ENOSYS;
    }

  /* Get an inode for the new relpath -- it should lie on the same
   * mountpoint
   */

  SETUP_SEARCH(&newdesc, newpath, true);

  ret = inode_find(&newdesc);
  if (ret < 0)
    {
      /* There is no mountpoint that includes in this path */

      goto errout_with_newsearch;
    }

  /* Get the search results */

  newinode   = newdesc.node;
  newrelpath = newdesc.relpath;
  DEBUGASSERT(newinode != NULL && newrelpath != NULL);

  /* Verify that the two paths lie on the same mountpoint inode */

  if (oldinode != newinode)
    {
      ret = -EXDEV;
      goto errout_with_newinode;
    }

  /* Does a directory entry already exist at the 'rewrelpath'?  And is it
   * not the same directory entry that we are moving?
   *
   * If the directory entry at the newrelpath is a regular file, then that
   * file should be removed first.
   *
   * If the directory entry at the target is a directory, then the source
   * file should be moved "under" the directory, i.e., if newrelpath is a
   * directory, then rename(b,a) should use move the olrelpath should be
   * moved as if rename(b,a/basename(b)) had been called.
   */

  if (oldinode->u.i_mops->stat != NULL &&
      strcmp(oldrelpath, newrelpath) != 0)
    {
      struct stat buf;

next_subdir:
      /* Something exists for this directory entry.  Do nothing in the
       * degenerate case where a directory or file is being moved to
       * itself.
       */

      if (strcmp(oldrelpath, newrelpath) != 0)
        {
          ret = oldinode->u.i_mops->stat(oldinode, newrelpath, &buf);
          if (ret >= 0)
            {
              /* Is the directory entry a directory? */

              if (S_ISDIR(buf.st_mode))
                {
                  FAR char *subdirname;

                  /* Yes.. In this case, the target of the rename must be a
                   * subdirectory of newinode, not the newinode itself.  For
                   * example: mv b a/ must move b to a/b.
                   */

                  subdirname = basename((FAR char *)oldrelpath);

                  /* Special case the root directory */

                  if (*newrelpath == '\0')
                    {
                      if (subdir != NULL)
                        {
                           kmm_free(subdir);
                           subdir = NULL;
                        }

                      newrelpath = subdirname;
                    }
                  else
                    {
                      FAR char *tmp = subdir;

                      subdir = NULL;
                      asprintf(&subdir, "%s/%s", newrelpath,
                               subdirname);

                      if (tmp != NULL)
                        {
                          kmm_free(tmp);
                        }

                      if (subdir == NULL)
                        {
                          ret = -ENOMEM;
                          goto errout_with_newinode;
                        }

                      newrelpath = subdir;
                    }

                  /* This can be a recursive, another directory may already
                   * exist at the newrelpath.  In that case, we need to
                   * do this all over again.  A nasty goto is used because
                   * I am lazy.
                   */

                  goto next_subdir;
                }
              else
                {
                  /* No.. newrelpath must refer to a regular file.  Make sure
                   * that the file at the olrelpath actually exists before
                   * performing any further actions with newrelpath
                   */

                  ret = oldinode->u.i_mops->stat(oldinode, oldrelpath, &buf);
                  if (ret < 0)
                    {
                      goto errout_with_newinode;
                    }

                  if (oldinode->u.i_mops->unlink)
                    {
                      /* Attempt to remove the file before doing the rename.
                       *
                       * NOTE that errors are not handled here.  If we failed
                       * to remove the file, then the file system 'rename'
                       * method should check that.
                       */

                       oldinode->u.i_mops->unlink(oldinode, newrelpath);
                    }
                }
            }
        }
    }

  /* Just declare success of the oldrepath and the newrelpath point to
   * the same directory entry.  That directory entry should have been
   * stat'ed above to assure that it exists.
   */

  ret = OK;
  if (strcmp(oldrelpath, newrelpath) != 0)
    {
      /* Perform the rename operation using the relative paths at the common
       * mountpoint.
       */

      ret = oldinode->u.i_mops->rename(oldinode, oldrelpath, newrelpath);
    }

errout_with_newinode:
  inode_release(newinode);

errout_with_newsearch:
  RELEASE_SEARCH(&newdesc);

  if (subdir != NULL)
    {
      kmm_free(subdir);
    }

  return ret;
}
#endif /* CONFIG_DISABLE_MOUNTPOINT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rename
 *
 * Description:
 *   Rename a file or directory.
 *
 ****************************************************************************/

int rename(FAR const char *oldpath, FAR const char *newpath)
{
  struct inode_search_s olddesc;
  FAR struct inode *oldinode;
  int ret;

  /* Ignore paths that are interpreted as the root directory which has no
   * name and cannot be moved
   */

  if (!oldpath || *oldpath == '\0' ||
      !newpath || *newpath == '\0')
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get an inode that includes the oldpath */

  SETUP_SEARCH(&olddesc, oldpath, true);

  ret = inode_find(&olddesc);
  if (ret < 0)
    {
      /* There is no inode that includes in this path */

      goto errout_with_oldsearch;
    }

  /* Get the search results */

  oldinode = olddesc.node;
  DEBUGASSERT(oldinode != NULL);

#ifndef CONFIG_DISABLE_MOUNTPOINT
  /* Verify that the old inode is a valid mountpoint. */

  if (INODE_IS_MOUNTPT(oldinode) && *olddesc.relpath != '\0')
    {
      ret = mountptrename(oldpath, oldinode, olddesc.relpath, newpath);
    }
  else
#endif /* CONFIG_DISABLE_MOUNTPOINT */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    {
      ret = pseudorename(oldpath, oldinode, newpath);
    }
#else
    {
      ret = -ENXIO;
    }
#endif

  inode_release(oldinode);

errout_with_oldsearch:
  RELEASE_SEARCH(&olddesc);

errout:
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

#endif /* FS_HAVE_RENAME */
