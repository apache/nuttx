/****************************************************************************
 * fs/vfs/fs_fcntl.c
 *
 *   Copyright (C) 2009, 2012-2014, 2016-2017 Gregory Nutt. All rights
 *     reserved.
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

#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_vfcntl
 *
 * Description:
 *   Similar to the standard vfcntl function except that is accepts a struct
 *   struct file instance instead of a file descriptor.
 *
 * Input Parameters:
 *   filep - Instance for struct file for the opened file.
 *   cmd   - Identifies the operation to be performed.
 *   ap    - Variable argument following the command.
 *
 * Returned Value:
 *   The nature of the return value depends on the command.  Non-negative
 *   values indicate success.  Failures are reported as negated errno
 *   values.
 *
 ****************************************************************************/

int file_vfcntl(FAR struct file *filep, int cmd, va_list ap)
{
  int ret = -EINVAL;

  /* Was this file opened ? */

  if (!filep->f_inode)
    {
      return -EBADF;
    }

  switch (cmd)
    {
      case F_DUPFD:
        /* Return a new file descriptor which shall be the lowest numbered
         * available (that is, not already open) file descriptor greater than
         * or equal to the third argument, arg, taken as an integer of type
         * int. The new file descriptor shall refer to the same open file
         * description as the original file descriptor, and shall share any
         * locks.  The FD_CLOEXEC flag associated  with the new file
         * descriptor shall be cleared to keep the file open across calls to
         * one of the exec functions.
         */

        {
          /* Does not set the errno variable in the event of a failure */

          ret = file_dup(filep, va_arg(ap, int));
        }
        break;

      case F_GETFD:
        /* Get the file descriptor flags defined in <fcntl.h> that are
         * associated with the file descriptor fd.  File descriptor flags are
         * associated with a single file descriptor and do not affect other
         * file descriptors that refer to the same file.
         */

        {
          ret = filep->f_oflags & O_CLOEXEC ? FD_CLOEXEC : 0;
        }
        break;

      case F_SETFD:
        /* Set the file descriptor flags defined in <fcntl.h>, that are
         * associated with fd, to the third argument, arg, taken as type int.
         * If the FD_CLOEXEC flag in the third argument is 0, the file shall
         * remain open across the exec functions; otherwise, the file shall
         * be closed upon successful execution of one of the exec functions.
         */

        {
          int oflags = va_arg(ap, int);

          if (oflags & ~FD_CLOEXEC)
            {
              ret = -ENOSYS;
              break;
            }

          if (oflags & FD_CLOEXEC)
            {
              filep->f_oflags |= O_CLOEXEC;
            }
          else
            {
              filep->f_oflags &= ~O_CLOEXEC;
            }

          ret = OK;
        }
        break;

      case F_GETFL:
        /* Get the file status flags and file access modes, defined in
         * <fcntl.h>, for the file description associated with fd. The file
         * access modes can be extracted from the return value using the
         * mask O_ACCMODE, which is defined  in <fcntl.h>. File status flags
         * and file access modes are associated with the file description
         * and do not affect other file descriptors that refer to the same
         * file with different open file descriptions.
         */

        {
          ret = filep->f_oflags;
        }
        break;

      case F_SETFL:
        /* Set the file status flags, defined in <fcntl.h>, for the file
         * description associated with fd from the corresponding  bits in
         * the third argument, arg, taken as type int. Bits corresponding
         * to the file access mode and the file creation flags, as defined
         * in <fcntl.h>, that are set in arg shall be ignored. If any bits
         * in arg other than those mentioned here are changed by the
         * application, the result is unspecified.
         */

        {
          int oflags = va_arg(ap, int);

          oflags          &=  FFCNTL;
          filep->f_oflags &= ~FFCNTL;
          filep->f_oflags |=  oflags;
          ret              =  OK;
        }
        break;

      case F_GETOWN:
        /* If fd refers to a socket, get the process or process group ID
         * specified to receive SIGURG signals when out-of-band data is
         * available. Positive values indicate a process ID; negative
         * values, other than -1, indicate a process group ID. If fd does
         * not refer to a socket, the results are unspecified.
         */

      case F_SETOWN:
        /* If fd refers to a socket, set the process or process group ID
         * specified to receive SIGURG signals when out-of-band data is
         * available, using the value of the third argument, arg, taken as
         * type int. Positive values indicate a process ID; negative values,
         * other than -1, indicate a process group ID.  If fd does not refer
         * to a socket, the results are unspecified.
         */

        ret = -EBADF; /* Only valid on socket descriptors */
        break;

      case F_GETLK:
        /* Get the first lock which blocks the lock description pointed to
         * by the third argument, arg, taken as a pointer to type struct
         * flock, defined in <fcntl.h>.  The information retrieved shall
         * overwrite the information passed to fcntl() in the structure
         * flock. If no lock is found that would prevent this lock from
         * being created, then the structure shall be left unchanged except
         * for the lock type which shall be set to F_UNLCK.
         */

      case F_SETLK:
        /* Set or clear a file segment lock according to the lock
         * description pointed to by the third argument, arg, taken as a
         * pointer to type struct flock, defined in <fcntl.h>. F_SETLK can
         * establish shared (or read) locks (F_RDLCK) or exclusive (or
         * write) locks (F_WRLCK), as well  as to remove either type of lock
         * (F_UNLCK).  F_RDLCK, F_WRLCK, and F_UNLCK are defined in
         * <fcntl.h>. If a shared or exclusive lock cannot be set, fcntl()
         * shall return immediately with a return value of -1.
         */

      case F_SETLKW:
        /* This command shall be equivalent to F_SETLK except that if a
         * shared or exclusive lock is blocked by other locks, the thread
         * shall wait until the request can be satisfied. If a signal that
         * is to be caught is received while fcntl() is waiting for a
         * region, fcntl() shall be interrupted. Upon return from the signal
         * handler, fcntl() shall return -1 with errno set to [EINTR], and
         * the lock operation shall not be done.
         */

        ret = -ENOSYS; /* Not implemented */
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: file_fcntl
 *
 * Description:
 *   Similar to the standard fcntl function except that is accepts a struct
 *   struct file instance instead of a file descriptor.
 *
 * Input Parameters:
 *   filep - Instance for struct file for the opened file.
 *   cmd   - Identifies the operation to be performed.  Command specific
 *           arguments may follow.
 *
 * Returned Value:
 *   The nature of the return value depends on the command.  Non-negative
 *   values indicate success.  Failures are reported as negated errno
 *   values.
 *
 ****************************************************************************/

int file_fcntl(FAR struct file *filep, int cmd, ...)
{
  va_list ap;
  int ret;

  /* Setup to access the variable argument list */

  va_start(ap, cmd);

  /* Let file_vfcntl() do the real work.  The errno is not set on
   * failures.
   */

  ret = file_vfcntl(filep, cmd, ap);

  va_end(ap);
  return ret;
}

/****************************************************************************
 * Name: nx_fcntl and nx_vfcntl
 *
 * Description:
 *   nx_fcntl() is similar to the standard 'fcntl' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_vfcntl() is identical except that it accepts a va_list as an argument
 *   versus taking a variable length list of arguments.
 *
 *   nx_fcntl() and nx_vfcntl are internal NuttX interface and should not be
 *   called from applications.
 *
 * Returned Value:
 *   Returns a non-negative number on success;  A negated errno value is
 *   returned on any failure (see comments fcntl() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

int nx_vfcntl(int fd, int cmd, va_list ap)
{
  FAR struct file *filep;
  int ret;

  /* Did we get a valid file descriptor? */

  if (fd < CONFIG_NFILE_DESCRIPTORS)
    {
      /* Get the file structure corresponding to the file descriptor. */

      ret = fs_getfilep(fd, &filep);
      if (ret >= 0)
        {
          DEBUGASSERT(filep != NULL);

          /* Let file_vfcntl() do the real work.  The errno is not set on
           * failures.
           */

          ret = file_vfcntl(filep, cmd, ap);
        }
    }
  else
    {
      /* No... check for operations on a socket descriptor */

#ifdef CONFIG_NET
      if (fd < (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS))
        {
          /* Yes.. defer socket descriptor operations to net_vfcntl(). The
           * errno is not set on failures.
           */

          ret = net_vfcntl(fd, cmd, ap);
        }
      else
#endif
        {
          /* No.. this descriptor number is out of range */

          ret = -EBADF;
        }
    }

  return ret;
}

int nx_fcntl(int fd, int cmd, ...)
{
  va_list ap;
  int ret;

  /* Setup to access the variable argument list */

  va_start(ap, cmd);

  /* Let nx_vfcntl() do the real work.  The errno is not set on
   * failures.
   */

  ret = nx_vfcntl(fd, cmd, ap);

  va_end(ap);
  return ret;
}

/****************************************************************************
 * Name: fcntl
 *
 * Description:
 *   fcntl() will perform the operation specified by 'cmd' on an open file.
 *
 * Input Parameters:
 *   fd  - File descriptor of the open file
 *   cmd - Identifies the operation to be performed.  Command specific
 *         arguments may follow.
 *
 * Returned Value:
 *   The returned value depends on the nature of the command but for all
 *   commands the return value of -1 (ERROR) indicates that an error has
 *   occurred and, in this case, the errno variable will be set
 *   appropriately
 *
 ****************************************************************************/

int fcntl(int fd, int cmd, ...)
{
  va_list ap;
  int ret;

  /* fcntl() is a cancellation point */

  enter_cancellation_point();

  /* Setup to access the variable argument list */

  va_start(ap, cmd);

  /* Let nx_vfcntl() do the real work.  The errno is not set on
   * failures.
   */

  ret = nx_vfcntl(fd, cmd, ap);

  va_end(ap);

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
