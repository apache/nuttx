/************************************************************
 * lib_fopen.c
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
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>

#if CONFIG_NFILE_STREAMS > 0

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "lib_internal.h"

/************************************************************
 * Private Functions
 ************************************************************/

static int lib_mode2oflags(const char *mode)
{
  int oflags = 0;
  if (mode)
    {
      while(*mode)
        {
          switch (*mode)
            {
             /* Open for read access */

             case 'r' :
               if (*(mode + 1) == '+')
                 {
                   /* Open for read/write access */
                   oflags |= O_RDWR;
                   mode++;
                 }
               else
                 {
                   /* Open for read access */
                   oflags |= O_RDOK;
                 }
               break;

             /* Open for write access? */

             case 'w' :
               if (*(mode + 1) == '+')
                 {
                   /* Open for write read/access, truncating any existing file */
                   oflags |= O_RDWR|O_CREAT|O_TRUNC;
                   mode++;
                 }
               else
                 {
                   /* Open for write access, truncating any existing file */
                   oflags |= O_WROK|O_CREAT|O_TRUNC;
                 }
               break;

             /* Open for write/append access? */

             case 'a' :
               if (*(mode + 1) == '+')
                 {
                   /* Read from the beginning of the file; write to the end */
                   oflags |= O_RDWR|O_CREAT;
                   mode++;
                 }
               else
                 {
                   /* Write to the end of the file */
                   oflags |= O_WROK|O_CREAT;
                 }
               break;

             /* Open for binary access? */

             case 'b' :
             default:
               break;
            }
          mode++;
        }
    }
  return oflags;
}

/************************************************************
 * Public Functions
 ************************************************************/

FAR struct file_struct *lib_fdopen(int fd, const char *mode,
                                   FAR struct filelist *flist,
                                   FAR struct streamlist *slist)
{
  FAR struct inode *inode = flist->fl_files[fd].f_inode;
  FILE             *stream;
  int               oflags = lib_mode2oflags(mode);
  int               i;

  if (fd < 0 || !flist || !slist)
    {
      *get_errno_ptr() = EBADF;
      return NULL;
    }

  /* Make sure that the inode supports the requested access.  In
   * the case of fdopen, we are not actually creating the file -- in
   * particular w and w+ do not truncate the file and any files have
   * already been created.
   */

  if (inode_checkflags(inode, oflags) != OK)
    {
      return NULL;
    }

  /* Find an unallocated FILE structure */

  stream_semtake(slist);
  for (i = 0 ; i < CONFIG_NFILE_STREAMS; i++)
    {
      stream = &slist->sl_streams[i];
      if (stream->fs_filedes < 0)
        {
          /* Zero the structure */
#if CONFIG_STDIO_BUFFER_SIZE > 0
          memset(stream, 0, sizeof(FILE));
#elif CONFIG_NUNGET_CHARS > 0
          stream->fs_nungotten = 0;
#endif

#if CONFIG_STDIO_BUFFER_SIZE > 0
          /* Initialize the semaphore the manages access to the buffer */

          (void)sem_init(&stream->fs_sem, 0, 1);

          /* Allocate the IO buffer */

          stream->fs_bufstart = malloc(CONFIG_STDIO_BUFFER_SIZE);
          if (!stream)
            {
              *get_errno_ptr() = ENOMEM;
              return NULL;
            }

          /* Set up pointers */

          stream->fs_bufend  = &stream->fs_bufstart[CONFIG_STDIO_BUFFER_SIZE];
          stream->fs_bufpos  = stream->fs_bufstart;
          stream->fs_bufpos  = stream->fs_bufstart;
          stream->fs_bufread = stream->fs_bufstart;
          stream_semgive(slist);
#endif
          /* Save the file description and open flags.  Setting the
           * file descriptor locks this stream.
           */

          stream->fs_filedes = fd;
          stream->fs_oflags  = oflags;

          return stream;
        }
    }
  stream_semgive(slist);
  return NULL;
}

FILE *fdopen(int fd, const char *mode)
{
  FAR struct filelist   *flist = sched_getfiles();
  FAR struct streamlist *slist = sched_getstreams();
  return lib_fdopen(fd, mode, flist, slist);
}

FILE *fopen(const char *path, const char *mode)
{
  FAR struct filelist   *flist = sched_getfiles();
  FAR struct streamlist *slist = sched_getstreams();
  int oflags = lib_mode2oflags(mode);
  int fd     = open(path, oflags, 0666);

  FILE *ret = lib_fdopen(fd, mode, flist, slist);
  if (!ret)
    {
      (void)close(fd);
    }
  return ret;
}

#endif /* CONFIG_NFILE_STREAMS */
