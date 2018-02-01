/****************************************************************************
 * fs/inode/inode.h
 *
 *   Copyright (C) 2007, 2009, 2012, 2014, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __FS_INODE_H
#define __FS_INODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <dirent.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PSEUDOFS_SOFTLINKS

#  define SETUP_SEARCH(d,p,n) \
    do \
      { \
        (d)->path     = (p); \
        (d)->node     = NULL; \
        (d)->peer     = NULL; \
        (d)->parent   = NULL; \
        (d)->relpath  = NULL; \
        (d)->linktgt  = NULL; \
        (d)->buffer   = NULL; \
        (d)->nofollow = (n); \
      } \
    while (0)

#  define RELEASE_SEARCH(d) \
     if ((d)->buffer != NULL) \
       { \
         kmm_free((d)->buffer); \
         (d)->buffer  = NULL; \
       }

#else

#  define SETUP_SEARCH(d,p,n) \
    do \
      { \
        (d)->path     = (p); \
        (d)->node     = NULL; \
        (d)->peer     = NULL; \
        (d)->parent   = NULL; \
        (d)->relpath  = NULL; \
      } \
    while (0)

#  define RELEASE_SEARCH(d)

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the argument to inode_search().
 *
 *  path     - INPUT:  Path of inode to find
 *             OUTPUT: Residual part of path not traversed
 *  node     - INPUT:  (not used)
 *             OUTPUT: On success, holds the pointer to the inode found.
 *  peer     - INPUT:  (not used)
 *             OUTPUT: The inode to the "left" of the inode found.
 *  parent   - INPUT:  (not used)
 *             OUTPUT: The inode to the "above" of the inode found.
 *  relpath  - INPUT:  (not used)
 *             OUTPUT: If the returned inode is a mountpoint, this is the
 *                     relative path from the mountpoint.
 *  linktgt  - INPUT:  (not used)
 *             OUTPUT: If a symobolic link into a mounted file system is
 *                     detected while traversing the path, then the link
 *                     will be converted to a mountpoint inode if the
 *                     mountpoint link is in an intermediate node of the
 *                     path or at the final node of the path with nofollow=true.
 *  nofollow - INPUT:  true: terminal node is returned; false: if the
 *                     terminal is a soft link, then return the inode of
 *                     the link target.
 *           - OUTPUT: (not used)
 *  buffer   - INPUT:  Not used
 *           - OUTPUT: May hold an allocated intermediate path which is
 *                     probably of no interest to the caller unless it holds
 *                     the relpath.
 */

struct inode_search_s
{
  FAR const char *path;      /* Path of inode to find */
  FAR struct inode *node;    /* Pointer to the inode found */
  FAR struct inode *peer;    /* Node to the "left" for the found inode */
  FAR struct inode *parent;  /* Node "above" the found inode */
  FAR const char *relpath;   /* Relative path into the mountpoint */
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  FAR const char *linktgt;   /* Target of symbolic link if linked to a directory */
  FAR char *buffer;          /* Path expansion buffer */
  bool nofollow;             /* true: Don't follow terminal soft link */
#endif
};

/* Callback used by foreach_inode to traverse all inodes in the pseudo-
 * file system.
 */

typedef int (*foreach_inode_t)(FAR struct inode *node,
                               FAR char dirpath[PATH_MAX],
                               FAR void *arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN FAR struct inode *g_root_inode;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: inode_initialize
 *
 * Description:
 *   This is called from the OS initialization logic to configure the file
 *   system.
 *
 ****************************************************************************/

void inode_initialize(void);

/****************************************************************************
 * Name: inode_semtake
 *
 * Description:
 *   Get exclusive access to the in-memory inode tree (tree_sem).
 *
 ****************************************************************************/

void inode_semtake(void);

/****************************************************************************
 * Name: inode_semgive
 *
 * Description:
 *   Relinquish exclusive access to the in-memory inode tree (tree_sem).
 *
 ****************************************************************************/

void inode_semgive(void);

/****************************************************************************
 * Name: inode_search
 *
 * Description:
 *   Find the inode associated with 'path' returning the inode references
 *   and references to its companion nodes.
 *
 *   If a mountpoint is encountered in the search prior to encountering the
 *   terminal node, the search will terminate at the mountpoint inode.  That
 *   inode and the relative path from the mountpoint, 'relpath' will be
 *   returned.
 *
 *   inode_search will follow soft links in path leading up to the terminal
 *   node.  Whether or no inode_search() will deference that terminal node
 *   depends on the 'nofollow' input.
 *
 *   If a soft link is encountered that is not the terminal node in the path,
 *   that link WILL be deferenced unconditionally.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

int inode_search(FAR struct inode_search_s *desc);

/****************************************************************************
 * Name: inode_find
 *
 * Description:
 *   This is called from the open() logic to get a reference to the inode
 *   associated with a path.  This is accomplished by calling inode_search().
 *   inode_find() is a simple wrapper around inode_search().  The primary
 *   difference between inode_find() and inode_search is that inode_find()
 *   will lock the inode tree and increment the reference count on the inode.
 *
 ****************************************************************************/

int inode_find(FAR struct inode_search_s *desc);

/****************************************************************************
 * Name: inode_stat
 *
 * Description:
 *   The inode_stat() function will obtain information about an 'inode' in
 *   the pseudo file system and will write it to the area pointed to by 'buf'.
 *
 *   The 'buf' argument is a pointer to a stat structure, as defined in
 *   <sys/stat.h>, into which information is placed concerning the file.
 *
 * Input Parameters:
 *   inode - The indoe of interest
 *   buf   - The caller provide location in which to return information about
 *           the inode.
 *
 * Returned Value:
 *   Zero (OK) returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

struct stat;  /* Forward reference */
int inode_stat(FAR struct inode *inode, FAR struct stat *buf);

/****************************************************************************
 * Name: inode_free
 *
 * Description:
 *   Free resources used by an inode
 *
 ****************************************************************************/

void inode_free(FAR struct inode *node);

/****************************************************************************
 * Name: inode_nextname
 *
 * Description:
 *   Given a path with node names separated by '/', return the next node
 *   name.
 *
 ****************************************************************************/

const char *inode_nextname(FAR const char *name);

/****************************************************************************
 * Name: inode_reserve
 *
 * Description:
 *   Reserve an (initialized) inode the pseudo file system.
 *
 *   NOTE: Caller must hold the inode semaphore
 *
 * Input Parameters:
 *   path - The path to the inode to create
 *   inode - The location to return the inode pointer
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on failure:
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int inode_reserve(FAR const char *path, FAR struct inode **inode);

/****************************************************************************
 * Name: inode_unlink
 *
 * Description:
 *   Given a path, remove a the node from the in-memory, inode tree that the
 *   path refers to.  This is normally done in preparation to removing or
 *   moving an inode.
 *
 * Assumptions/Limitations:
 *   The caller must hold the inode semaphore
 *
 ****************************************************************************/

FAR struct inode *inode_unlink(FAR const char *path);

/****************************************************************************
 * Name: inode_remove
 *
 * Description:
 *   Given a path, remove a the node from the in-memory, inode tree that the
 *   path refers to and free all resources related to the inode.  If the
 *   inode is in-use, then it will be unlinked, but will not be freed until
 *   the last reference to the inode is released.
 *
 * Assumptions/Limitations:
 *   The caller must hold the inode semaphore
 *
 ****************************************************************************/

int inode_remove(FAR const char *path);

/****************************************************************************
 * Name: inode_addref
 *
 * Description:
 *   Increment the reference count on an inode (as when a file descriptor
 *   is dup'ed).
 *
 ****************************************************************************/

void inode_addref(FAR struct inode *inode);

/****************************************************************************
 * Name: inode_release
 *
 * Description:
 *   This is called from close() logic when it no longer refers to the inode.
 *
 ****************************************************************************/

void inode_release(FAR struct inode *inode);

/****************************************************************************
 * Name: foreach_inode
 *
 * Description:
 *   Visit each inode in the pseudo-file system.  The traversal is terminated
 *   when the callback 'handler' returns a non-zero value, or when all of
 *   the inodes have been visited.
 *
 *   NOTE 1: Use with caution... The pseudo-file system is locked throughout
 *   the traversal.
 *   NOTE 2: The search algorithm is recursive and could, in principle, use
 *   an indeterminant amount of stack space.  This will not usually be a
 *   real work issue.
 *
 ****************************************************************************/

int foreach_inode(foreach_inode_t handler, FAR void *arg);

/****************************************************************************
 * Name: files_initialize
 *
 * Description:
 *   This is called from the FS initialization logic to configure the files.
 *
 ****************************************************************************/

void weak_function files_initialize(void);

/****************************************************************************
 * Name: files_allocate
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode instance.
 *   Returns the file descriptor == index into the files array.
 *
 ****************************************************************************/

int files_allocate(FAR struct inode *inode, int oflags, off_t pos, int minfd);

/****************************************************************************
 * Name: files_close
 *
 * Description:
 *   Close an inode (if open)
 *
 * Assumuptions:
 *   Caller holds the list semaphore because the file descriptor will be freed.
 *
 ****************************************************************************/

int files_close(int fd);

/****************************************************************************
 * Name: files_release
 *
 * Assumuptions:
 *   Similar to files_close().  Called only from open() logic on error
 *   conditions.
 *
 ****************************************************************************/

void files_release(int fd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_INODE_H */
