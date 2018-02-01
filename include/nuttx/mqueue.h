/****************************************************************************
 * include/nuttx/mqueue.h
 *
 *   Copyright (C) 2007, 2009, 2011, 2014-2017 Gregory Nutt. All rights
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

#ifndef ___INCLUDE_NUTTX_MQUEUE_H
#define ___INCLUDE_NUTTX_MQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <mqueue.h>
#include <queue.h>
#include <signal.h>

#if CONFIG_MQ_MAXMSGSIZE > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Most internal nxmq_* interfaces are not available in the user space in
 * PROTECTED and KERNEL builds.  In that context, the application message
 * queu interfaces must be used.  The differences between the two sets of
 * interfaces are:  (1) the nxmq_* interfaces do not cause cancellation
 * points and (2) they do not modify the errno variable.
 *
 * This is only important when compiling libraries (libc or libnx) that are
 * used both by the OS (libkc.a and libknx.a) or by the applications
 * (libuc.a and libunx.a).  The that case, the correct interface must be
 * used for the build context.
 *
 * The interfaces sigtimedwait(), sigwait(), sigwaitinfo(), sleep(),
 * nanosleep(), and usleep()  are cancellation points.
 *
 * REVISIT:  The fact that these interfaces are cancellation points is an
 * issue and may cause violations:  It use of these internally will cause
 * the calling function to become a cancellation points!
 */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
#  define _MQ_SEND(d,m,l,p)           nxmq_send(d,m,l,p)
#  define _MQ_TIMEDSEND(d,m,l,p,t)    nxmq_timedsend(d,m,l,p,t)
#  define _MQ_RECEIVE(d,m,l,p)        nxmq_receive(d,m,l,p)
#  define _MQ_TIMEDRECIEVE(d,m,l,p,t) nxmq_timedreceive(d,m,l,p,t)
#  define _MQ_GETERRNO(r)             (-(r))
#  define _MQ_SETERRNO(r)             set_errno(-(r))
#  define _MQ_GETERRVAL(r)            (r)
#else
#  define _MQ_SEND(d,m,l,p)           mq_send(d,m,l,p)
#  define _MQ_TIMEDSEND(d,m,l,p,t)    mq_timedsend(d,m,l,p,t)
#  define _MQ_RECEIVE(d,m,l,p)        mq_receive(d,m,l,p)
#  define _MQ_TIMEDRECIEVE(d,m,l,p,t) mq_timedreceive(d,m,l,p,t)
#  define _MQ_GETERRNO(r)             errno
#  define _MQ_SETERRNO(r)
#  define _MQ_GETERRVAL(r)            (-errno)
#endif

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* This structure defines a message queue */

struct mq_des; /* forward reference */

struct mqueue_inode_s
{
  FAR struct inode *inode;    /* Containing inode */
  sq_queue_t msglist;         /* Prioritized message list */
  int16_t maxmsgs;            /* Maximum number of messages in the queue */
  int16_t nmsgs;              /* Number of message in the queue */
  int16_t nwaitnotfull;       /* Number tasks waiting for not full */
  int16_t nwaitnotempty;      /* Number tasks waiting for not empty */
#if CONFIG_MQ_MAXMSGSIZE < 256
  uint8_t maxmsgsize;         /* Max size of message in message queue */
#else
  uint16_t maxmsgsize;        /* Max size of message in message queue */
#endif
#ifndef CONFIG_DISABLE_SIGNALS
  FAR struct mq_des *ntmqdes; /* Notification: Owning mqdes (NULL if none) */
  pid_t ntpid;                /* Notification: Receiving Task's PID */
  struct sigevent ntevent;    /* Notification description */
#endif
};

/* This describes the message queue descriptor that is held in the
 * task's TCB
 */

struct mq_des
{
  FAR struct mq_des *flink;        /* Forward link to next message descriptor */
  FAR struct mqueue_inode_s *msgq; /* Pointer to associated message queue */
  int oflags;                      /* Flags set when message queue was opened */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct tcb_s;         /* Forward reference */
struct mq_attr;       /* Forward reference */
struct timespec;      /* Forward reference */
struct task_group_s;  /* Forward reference */

/****************************************************************************
 * Name: nxmq_send
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mqdes).  This is an internal OS interface.  It is functionally
 *   equivalent to mq_send() except that:
 *
 *   - It is not a cancellaction point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_send() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mqdes  - Message queue descriptor
 *   msg    - Message to send
 *   msglen - The length of the message in bytes
 *   prio   - The priority of the message
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_send() for the list list valid return values).
 *
 ****************************************************************************/

int nxmq_send(mqd_t mqdes, FAR const char *msg, size_t msglen, int prio);

/****************************************************************************
 * Name: nxmq_timedsend
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mqdes).  nxmq_timedsend() behaves just like mq_send(), except that if
 *   the queue is full and the O_NONBLOCK flag is not enabled for the
 *   message queue description, then abstime points to a structure which
 *   specifies a ceiling on the time for which the call will block.
 *
 *   nxmq_timedsend() is functionally equivalent to mq_timedsend() except
 *   that:
 *
 *   - It is not a cancellaction point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_timedsend() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mqdes   - Message queue descriptor
 *   msg     - Message to send
 *   msglen  - The length of the message in bytes
 *   prio    - The priority of the message
 *   abstime - the absolute time to wait until a timeout is decleared
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_timedsend() for the list list valid return values).
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int nxmq_timedsend(mqd_t mqdes, FAR const char *msg, size_t msglen, int prio,
                   FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxmq_receive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages
 *   from the message queue specified by "mqdes."  This is an internal OS
 *   interface.  It is functionally equivalent to mq_receive except that:
 *
 *   - It is not a cancellaction point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_receive() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mqdes  - Message Queue Descriptor
 *   msg    - Buffer to receive the message
 *   msglen - Size of the buffer in bytes
 *   prio   - If not NULL, the location to store message priority.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_receive() for the list list valid return values).
 *
 ****************************************************************************/

ssize_t nxmq_receive(mqd_t mqdes, FAR char *msg, size_t msglen,
                     FAR int *prio);

/****************************************************************************
 * Name: nxmq_timedreceive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages from
 *   the message queue specified by "mqdes."  If the message queue is empty
 *   and O_NONBLOCK was not set, nxmq_timedreceive() will block until a
 *   message is added to the message queue (or until a timeout occurs).
 *
 *   nxmq_timedreceive() is an internal OS interface.  It is functionally
 *   equivalent to mq_timedreceive() except that:
 *
 *   - It is not a cancellaction point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_timedreceive() for a more complete description of
 *  the behavior of this function
 *
 * Input Parameters:
 *   mqdes   - Message Queue Descriptor
 *   msg     - Buffer to receive the message
 *   msglen  - Size of the buffer in bytes
 *   prio    - If not NULL, the location to store message priority.
 *   abstime - the absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_timedreceive() for the list list valid return values).
 *
 ****************************************************************************/

ssize_t nxmq_timedreceive(mqd_t mqdes, FAR char *msg, size_t msglen,
                        FAR int *prio, FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxmq_free_msgq
 *
 * Description:
 *   This function deallocates an initialized message queue structure.
 *   First, it deallocates all of the queued messages in the message
 *   queue.  It is assumed that this message is fully unlinked and
 *   closed so that no thread will attempt access it while it is being
 *   deleted.
 *
 * Input Parameters:
 *   msgq - Named essage queue to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmq_free_msgq(FAR struct mqueue_inode_s *msgq);

/****************************************************************************
 * Name: nxmq_alloc_msgq
 *
 * Description:
 *   This function implements a part of the POSIX message queue open logic.
 *   It allocates and initializes a structu mqueue_inode_s structure.
 *
 * Input Parameters:
 *   mode   - mode_t value is ignored
 *   attr   - The mq_maxmsg attribute is used at the time that the message
 *            queue is created to determine the maximum number of
 *             messages that may be placed in the message queue.
 *
 * Returned Value:
 *   The allocated and initalized message queue structure or NULL in the
 *   event of a failure.
 *
 ****************************************************************************/

FAR struct mqueue_inode_s *nxmq_alloc_msgq(mode_t mode,
                                           FAR struct mq_attr *attr);

/****************************************************************************
 * Name: nxmq_create_des
 *
 * Description:
 *   Create a message queue descriptor for the specified TCB
 *
 * Input Parameters:
 *   TCB - task that needs the descriptor.
 *   msgq - Named message queue containing the message
 *   oflags - access rights for the descriptor
 *
 * Returned Value:
 *   On success, the message queue descriptor is returned.  NULL is returned
 *   on a failure to allocate.
 *
 ****************************************************************************/

mqd_t nxmq_create_des(FAR struct tcb_s *mtcb,
                      FAR struct mqueue_inode_s *msgq, int oflags);

/****************************************************************************
 * Name: nxmq_close_group
 *
 * Description:
 *   This function is used to indicate that all threads in the group are
 *   finished with the specified message queue mqdes.  nxmq_close_group()
 *   deallocates any system resources allocated by the system for use by
 *   this task for its message queue.
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor.
 *   group - Group that has the open descriptor.
 *
 * Returned Value:
 *   Zero (OK) if the message queue is closed successfully.  Otherwise, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

int nxmq_close_group(mqd_t mqdes, FAR struct task_group_s *group);

/****************************************************************************
 * Name: nxmq_desclose_group
 *
 * Description:
 *   This function performs the portion of the mq_close operation related
 *   to freeing resource used by the message queue descriptor itself.
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor.
 *   group - Group that has the open descriptor.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 * - Called only from mq_close() with the scheduler locked.
 *
 ****************************************************************************/

void nxmq_desclose_group(mqd_t mqdes, FAR struct task_group_s *group);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* ___INCLUDE_NUTTX_MQUEUE_H */

