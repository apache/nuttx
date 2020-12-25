/****************************************************************************
 * net/socket/net_vfcntl.c
 *
 *   Copyright (C) 2009, 2012-2015, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <stdint.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/irq.h>
#include <nuttx/net/net.h>
#include "socket/socket.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_vfcntl
 *
 * Description:
 *   Performs fcntl operations on socket
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   cmd   - The fcntl command.
 *   ap    - Command-specific arguments
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int psock_vfcntl(FAR struct socket *psock, int cmd, va_list ap)
{
  int ret = -EINVAL;

  ninfo("sockfd=%p cmd=%d\n", psock, cmd);

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Interrupts must be disabled in order to perform operations on socket
   * structures
   */

  net_lock();
  switch (cmd)
    {
      case F_DUPFD:
        /* Return a new file descriptor which shall be the lowest numbered
         * available (that is, not already open) file descriptor greater than
         * or equal to the third argument, arg, taken as an integer of type
         * int. The new file descriptor shall refer to the same open file
         * description as the original file descriptor, and shall share any
         * locks.  The FD_CLOEXEC flag associated  with the new file
         * descriptor shall be cleared to keep the file open across calls
         * to one of the exec functions.
         */

        {
          /* Does not set the errno value on failure */

          ret = psock_dup(psock, va_arg(ap, int));
        }
        break;

      case F_GETFD:
        /* Get the file descriptor flags defined in <fcntl.h> that are
         * associated with the file descriptor fd.  File descriptor flags
         * are associated with a single file descriptor and do not affect
         * other file descriptors that refer to the same file.
         */

        {
          ret = _SS_ISCLOEXEC(psock->s_flags) ? FD_CLOEXEC : 0;
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
              psock->s_flags |= _SF_CLOEXEC;
            }
          else
            {
              psock->s_flags &= ~_SF_CLOEXEC;
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
          sockcaps_t sockcaps;

          /* This summarizes the behavior of all NuttX sockets */

          ret = O_RDWR | O_SYNC | O_RSYNC;

          /* Unix domain sockets may be non-blocking.  TCP/IP and UDP/IP
           * sockets may also be non-blocking if read-ahead is enabled
           */

          DEBUGASSERT(psock->s_sockif != NULL &&
                      psock->s_sockif->si_sockcaps != NULL);
          sockcaps = psock->s_sockif->si_sockcaps(psock);

          if ((sockcaps & SOCKCAP_NONBLOCKING) != 0 &&
              _SS_ISNONBLOCK(psock->s_flags))
            {
              ret |= O_NONBLOCK;
            }
        }
        break;

      case F_SETFL:
        /* Set the file status flags, defined in <fcntl.h>, for the file
         * description associated with fd from the corresponding bits in
         * the third argument, arg, taken as type int. Bits corresponding
         * to the file access mode and the file creation flags, as defined
         * in <fcntl.h>, that are set in arg will be ignored. If any bits
         * in arg other than those mentioned here are changed by the
         * application, the result is unspecified.
         */

        {
           /* Non-blocking is the only configurable option.  And it applies
            * only Unix domain sockets and to read operations on TCP/IP
            * and UDP/IP sockets when read-ahead is enabled.
            */

          int mode =  va_arg(ap, int);
          sockcaps_t sockcaps;

          DEBUGASSERT(psock->s_sockif != NULL &&
                      psock->s_sockif->si_sockcaps != NULL);
          sockcaps = psock->s_sockif->si_sockcaps(psock);

          if ((sockcaps & SOCKCAP_NONBLOCKING) != 0)
            {
               if ((mode & O_NONBLOCK) != 0)
                 {
                   psock->s_flags |= _SF_NONBLOCK;
                 }
               else
                 {
                   psock->s_flags &= ~_SF_NONBLOCK;
                 }

               ret = OK;
            }
          else
            {
              nerr("ERROR: Non-blocking not supported for this socket\n");
              ret = -ENOSYS;
            }
        }
        break;

      case F_GETOWN:
        /* If fd refers to a socket, get the process or process group ID
         * specified to receive SIGURG signals when out-of-band data is
         * available.  Positive values indicate a process ID; negative
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
         * write) locks (F_WRLCK), as well as to remove either type of
         * lock  (F_UNLCK).  F_RDLCK, F_WRLCK, and F_UNLCK are defined in
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

         ret = -ENOSYS; /* F_GETOWN, F_SETOWN, F_GETLK, F_SETLK, F_SETLKW */
         break;

      default:
         break;
    }

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: psock_fcntl
 *
 * Description:
 *   Similar to the standard fcntl function except that is accepts a struct
 *   struct socket instance instead of a file descriptor.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   cmd   - Identifies the operation to be performed.  Command specific
 *           arguments may follow.
 *
 * Returned Value:
 *   The nature of the return value depends on the command.  Non-negative
 *   values indicate success.  Failures are reported as negated errno
 *   values.
 *
 ****************************************************************************/

int psock_fcntl(FAR struct socket *psock, int cmd, ...)
{
  va_list ap;
  int ret;

  /* Setup to access the variable argument list */

  va_start(ap, cmd);

  /* Let psock_vfcntl() do the real work.  The errno is not set on
   * failures.
   */

  ret = psock_vfcntl(psock, cmd, ap);

  va_end(ap);
  return ret;
}

/****************************************************************************
 * Name: net_vfcntl
 *
 * Description:
 *   Performs fcntl operations on socket
 *
 * Input Parameters:
 *   sockfd - Socket descriptor of the socket to operate on
 *   cmd    - The fcntl command.
 *   ap     - Command-specific arguments
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int net_vfcntl(int sockfd, int cmd, va_list ap)
{
  return psock_vfcntl(sockfd_socket(sockfd), cmd, ap);
}
