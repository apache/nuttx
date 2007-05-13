/************************************************************
 * fs_inodefinddir.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <string.h>
#include <errno.h>
#include <nuttx/fs.h>
#include "fs_internal.h"

#if CONFIG_NFILE_DESCRIPTORS >0

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Public Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name: inode_finddir
 *
 * Description:
 *   This is called from the opendir() logic to get a reference
 *   to the inode associated with a directory.  There are no
 *   real directories in this design; For our purposes, a
 *   directory inode is simply one that has children.
 *
 ************************************************************/

FAR struct inode *inode_finddir(const char *path)
{
  FAR struct inode *node;
  FAR struct inode *child = NULL;

  /* If we are given 'nothing' then we will interpret this as
   * request for the root inode.
   */

  if (!path || *path == 0 || strcmp(path, "/") == 0)
    {
       return root_inode;
    }

  /* We don't know what to do with relative pathes */

  if (*path != '/')
    {
      return NULL;
    }

  /* Find the node matching the path. */

  inode_semtake();

  /* Handle some special cases */

  node = inode_search(&path, (FAR void*)NULL, (FAR void*)NULL, (FAR const char**)NULL);
  if (node)
    {
       /* Does the inode have a child?  If so that the child
        * would be the 'head' of a list of nodes under the
        * directory.
        */

       child = node->i_child;
       if (child)
         {
           /* If found, then  increment the count of
            * references on the child node.
             */

           child->i_crefs++;
         }
    }
  inode_semgive();
  return child;
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
