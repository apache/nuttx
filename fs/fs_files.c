/************************************************************
 * fs_files.c
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
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/fs.h>
#include <nuttx/kmalloc.h>
#include "fs_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Public Types
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

#if CONFIG_NFILE_DESCRIPTORS >0

static void _files_semtake(FAR struct filelist *list)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&list->fl_sem) != 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

#define _files_semgive(list) sem_post(&list->fl_sem)

/************************************************************
 * Pulblic Functions
 ************************************************************/

/* This is called from the FS initialization logic to configure
 * the files.
 */

void files_initialize(void)
{
}

/* Allocate a list of files for a new task */

FAR struct filelist *files_alloclist(void)
{
  FAR struct filelist *list;
  list = (FAR struct filelist*)kzmalloc(sizeof(struct filelist));
  if (list)
    {
       /* Start with a reference count of one */

       list->fl_crefs = 1;

       /* Initialize the list access mutex */

      (void)sem_init(&list->fl_sem, 0, 1);
    }
  return list;
}

/* Increase the reference count on a file list */

int files_addreflist(FAR struct filelist *list)
{
  if (list)
    {
       /* Increment the reference count on the list.
        * NOTE: that we disable interrupts to do this
        * (vs. taking the list semaphore).  We do this
        * because file cleanup operations often must be
        * done from the IDLE task which cannot wait
        * on semaphores.
        */

       register irqstate_t flags = irqsave();
       list->fl_crefs++;
       irqrestore(flags);
    }
  return OK;
}

/* Release a reference to the file list */

int files_releaselist(FAR struct filelist *list)
{
  int crefs;
  if (list)
    {
       /* Decrement the reference count on the list.
        * NOTE: that we disable interrupts to do this
        * (vs. taking the list semaphore).  We do this
        * because file cleanup operations often must be
        * done from the IDLE task which cannot wait
        * on semaphores.
        */

       register irqstate_t flags = irqsave();
       crefs = --(list->fl_crefs);
       irqrestore(flags);

       /* If the count decrements to zero, then there is no reference
        * to the structure and it should be deallocated.  Since there
        * are references, it would be an error if any task still held
        * a reference to the list's semaphore.
        */

       if (crefs <= 0)
          {
            int i;

            /* close each file descriptor */

            for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
              {
                FAR struct inode *inode = list->fl_files[i].f_inode;
                if (inode)
                  {
                    inode_release(inode);
                  }
              }

             /* Destroy the semaphore and release the filelist */

             (void)sem_destroy(&list->fl_sem);
             sched_free(list);
          }
    }
  return OK;
}

/* Assign an inode to a specific files structure.  this is the
 * heart of dup2.
 */

int files_dup(FAR struct file *filep1, FAR struct file *filep2)
{
  FAR struct filelist *list;

  if (!filep1 || !filep1->f_inode || !filep2)
    {
      *get_errno_ptr() = EBADF;
      return ERROR;
    }

  list = sched_getfiles();
  if (!list)
    {
      *get_errno_ptr() = EMFILE;
      return ERROR;
    }

  _files_semtake(list);

  /* If there is already an inode contained in the new file structure,
   * release it (effectively closing the file).
   */

  if (filep2->f_inode)
    {
      inode_release(filep2->f_inode);
    }

  /* Increment the reference count on the contained inode */

  inode_addref(filep1->f_inode);

  /* Then clone the file structure */

  filep2->f_oflags = filep1->f_oflags;
  filep2->f_pos    = filep1->f_pos;
  filep2->f_inode  = filep1->f_inode;
  _files_semgive(list);
  return OK;
}

/* Allocate a struct files instance and associate it with an
 * inode instance.  Returns the file descriptor == index into
 * the files array.
 */

int files_allocate(FAR struct inode *inode, int oflags, off_t pos)
{
  FAR struct filelist *list;
  int i;

  list = sched_getfiles();
  if (list)
    {
      _files_semtake(list);
      for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          if (!list->fl_files[i].f_inode)
            {
               list->fl_files[i].f_oflags = oflags;
               list->fl_files[i].f_pos    = pos;
               list->fl_files[i].f_inode  = inode;
               _files_semgive(list);
               return i;
            }
        }
      _files_semgive(list);
    }
  return ERROR;
}

void files_release(int filedes)
{
  FAR struct filelist *list;

  list = sched_getfiles();
  if (list)
    {
      if (filedes >=0 && filedes < CONFIG_NFILE_DESCRIPTORS)
        {
          _files_semtake(list);
          list->fl_files[filedes].f_oflags  = 0;
          list->fl_files[filedes].f_pos     = 0;
          list->fl_files[filedes].f_inode = NULL;
          _files_semgive(list);
        }
    }
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
