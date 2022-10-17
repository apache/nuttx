/****************************************************************************
 * include/nuttx/mqueue.h
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

#ifndef ___INCLUDE_NUTTX_MQUEUE_H
#define ___INCLUDE_NUTTX_MQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/signal.h>
#include <nuttx/list.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <mqueue.h>
#include <poll.h>

#if CONFIG_MQ_MAXMSGSIZE > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Most internal nxmq_* interfaces are not available in the user space in
 * PROTECTED and KERNEL builds.  In that context, the application message
 * queue interfaces must be used.  The differences between the two sets of
 * interfaces are:  (1) the nxmq_* interfaces do not cause cancellation
 * points and (2) they do not modify the errno variable.
 *
 * This is only important when compiling libraries (libc or libnx) that are
 * used both by the OS (libkc.a and libknx.a) or by the applications
 * (libc.a and libnx.a).  In that case, the correct interface must be
 * used for the build context.
 *
 * REVISIT:  In the flat build, the same functions must be used both by
 * the OS and by applications.  We have to use the normal user functions
 * in this case or we will fail to set the errno or fail to create the
 * cancellation point.
 */

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
#  define _MQ_OPEN                    nxmq_open
#  define _MQ_CLOSE(d)                nxmq_close(d)
#  define _MQ_UNLINK(n)               nxmq_unlink(n)
#  define _MQ_SEND(d,m,l,p)           nxmq_send(d,m,l,p)
#  define _MQ_TIMEDSEND(d,m,l,p,t)    nxmq_timedsend(d,m,l,p,t)
#  define _MQ_RECEIVE(d,m,l,p)        nxmq_receive(d,m,l,p)
#  define _MQ_TIMEDRECEIVE(d,m,l,p,t) nxmq_timedreceive(d,m,l,p,t)
#else
#  define _MQ_OPEN                    mq_open
#  define _MQ_CLOSE(d)                mq_close(d)
#  define _MQ_UNLINK(n)               mq_unlink(n)
#  define _MQ_SEND(d,m,l,p)           mq_send(d,m,l,p)
#  define _MQ_TIMEDSEND(d,m,l,p,t)    mq_timedsend(d,m,l,p,t)
#  define _MQ_RECEIVE(d,m,l,p)        mq_receive(d,m,l,p)
#  define _MQ_TIMEDRECEIVE(d,m,l,p,t) mq_timedreceive(d,m,l,p,t)
#endif

#if CONFIG_FS_MQUEUE_NPOLLWAITERS > 0
# define nxmq_pollnotify(msgq, eventset) \
  poll_notify(msgq->fds, CONFIG_FS_MQUEUE_NPOLLWAITERS, eventset)
#else
# define nxmq_pollnotify(msgq, eventset)
#endif

# define MQ_WNELIST(cmn)              (&((cmn).waitfornotempty))
# define MQ_WNFLIST(cmn)              (&((cmn).waitfornotfull))

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* Common prologue of all message queue structures. */

struct mqueue_cmn_s
{
  dq_queue_t waitfornotempty; /* Task list waiting for not empty */
  dq_queue_t waitfornotfull;  /* Task list waiting for not full */
  int16_t nwaitnotfull;       /* Number tasks waiting for not full */
  int16_t nwaitnotempty;      /* Number tasks waiting for not empty */
};

/* This structure defines a message queue */

struct mqueue_inode_s
{
  struct mqueue_cmn_s cmn;    /* Common prologue */
  FAR struct inode *inode;    /* Containing inode */
  struct list_node msglist;   /* Prioritized message list */
  int16_t maxmsgs;            /* Maximum number of messages in the queue */
  int16_t nmsgs;              /* Number of message in the queue */
#if CONFIG_MQ_MAXMSGSIZE < 256
  uint8_t maxmsgsize;         /* Max size of message in message queue */
#else
  uint16_t maxmsgsize;        /* Max size of message in message queue */
#endif
#ifndef CONFIG_DISABLE_MQUEUE_NOTIFICATION
  pid_t ntpid;                /* Notification: Receiving Task's PID */
  struct sigevent ntevent;    /* Notification description */
  struct sigwork_s ntwork;    /* Notification work */
#endif
  FAR struct pollfd *fds[CONFIG_FS_MQUEUE_NPOLLWAITERS];
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
 * Name: nxmq_open
 *
 * Description:
 *   This function establish a connection between a named message queue and
 *   the calling task. This is an internal OS interface.  It is functionally
 *   equivalent to mq_open() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_open() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the queue to open
 *   oflags - open flags
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *   parameters are expected:
 *
 *     1. mode_t mode, and
 *     2. struct mq_attr *attr.  The mq_maxmsg attribute
 *        is used at the time that the message queue is
 *        created to determine the maximum number of
 *        messages that may be placed in the message queue.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success, mqdes point to the new message queue descriptor.
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

mqd_t nxmq_open(FAR const char *mq_name, int oflags, ...);

/****************************************************************************
 * Name: nxmq_close
 *
 * Description:
 *   This is an internal OS interface.  It is functionally equivalent to
 *   mq_close() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_close() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxmq_close(mqd_t mqdes);

/****************************************************************************
 * Name: nxmq_unlink
 *
 * Description:
 *   This is an internal OS interface.  It is functionally equivalent to
 *   mq_unlink() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_unlink() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the message queue
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxmq_unlink(FAR const char *mq_name);

/****************************************************************************
 * Name: nxmq_send
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mqdes).  This is an internal OS interface.  It is functionally
 *   equivalent to mq_send() except that:
 *
 *   - It is not a cancellation point, and
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

int nxmq_send(mqd_t mqdes, FAR const char *msg, size_t msglen,
              unsigned int prio);

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
 *   - It is not a cancellation point, and
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

int nxmq_timedsend(mqd_t mqdes, FAR const char *msg, size_t msglen,
                   unsigned int prio, FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxmq_receive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages
 *   from the message queue specified by "mqdes."  This is an internal OS
 *   interface.  It is functionally equivalent to mq_receive except that:
 *
 *   - It is not a cancellation point, and
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
                     FAR unsigned int *prio);

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
 *   - It is not a cancellation point, and
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
                          FAR unsigned int *prio,
                          FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxmq_free_msgq
 *
 * Description:
 *   This function deallocates an initialized message queue structure.
 *   First, it deallocates all of the queued messages in the message
 *   queue.  It is assumed that this message queue is fully unlinked
 *   and closed so that no thread will attempt to access it while it
 *   is being deleted.
 *
 * Input Parameters:
 *   msgq - Named message queue to be freed
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
 *   It allocates and initializes a struct mqueue_inode_s structure.
 *
 * Input Parameters:
 *   attr   - The mq_maxmsg attribute is used at the time that the message
 *            queue is created to determine the maximum number of
 *            messages that may be placed in the message queue.
 *   pmsgq  - This parameter is a address of a pointer
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 *   EINVAL    attr is NULL or either attr->mq_mqssize or attr->mq_maxmsg
 *             have an invalid value
 *   ENOSPC    There is insufficient space for the creation of the new
 *             message queue
 *
 ****************************************************************************/

int nxmq_alloc_msgq(FAR struct mq_attr *attr,
                    FAR struct mqueue_inode_s **pmsgq);

/****************************************************************************
 * Name: file_mq_open
 *
 * Description:
 *   This function establish a connection between a named message queue and
 *   the calling task. This is an internal OS interface.  It is functionally
 *   equivalent to mq_open() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_open() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the queue to open
 *   oflags  - open flags
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *   parameters are expected:
 *
 *     1. mode_t mode, and
 *     2. struct mq_attr *attr.  The mq_maxmsg attribute
 *        is used at the time that the message queue is
 *        created to determine the maximum number of
 *        messages that may be placed in the message queue.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   NOT NULL Message queue descriptor, NULL failed.
 *
 ****************************************************************************/

int file_mq_open(FAR struct file *mq,
                 FAR const char *mq_name, int oflags, ...);

/****************************************************************************
 * Name: file_mq_close
 *
 * Description:
 *   This is an internal OS interface.  It is functionally equivalent to
 *   mq_close() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_close() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq - Message queue descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int file_mq_close(FAR struct file *mq);

/****************************************************************************
 * Name: file_mq_unlink
 *
 * Description:
 *   This is an internal OS interface.  It is functionally equivalent to
 *   mq_unlink() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_unlink() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the message queue
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int file_mq_unlink(FAR const char *mq_name);

/****************************************************************************
 * Name: file_mq_send
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mq).  This is an internal OS interface.  It is functionally
 *   equivalent to mq_send() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_send() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq     - Message queue descriptor
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

int file_mq_send(FAR struct file *mq, FAR const char *msg, size_t msglen,
                 unsigned int prio);

/****************************************************************************
 * Name: file_mq_timedsend
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mq).  file_mq_timedsend() behaves just like mq_send(), except that if
 *   the queue is full and the O_NONBLOCK flag is not enabled for the
 *   message queue description, then abstime points to a structure which
 *   specifies a ceiling on the time for which the call will block.
 *
 *   file_mq_timedsend() is functionally equivalent to mq_timedsend() except
 *   that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_timedsend() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq      - Message queue descriptor
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
 *            message queue description referred to by mq.
 *   EINVAL   Either msg or mq is NULL or the value of prio is invalid.
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int file_mq_timedsend(FAR struct file *mq, FAR const char *msg,
                      size_t msglen, unsigned int prio,
                      FAR const struct timespec *abstime);

/****************************************************************************
 * Name: file_mq_receive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages
 *   from the message queue specified by "mq."  This is an internal OS
 *   interface.  It is functionally equivalent to mq_receive except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_receive() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq     - Message Queue Descriptor
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

ssize_t file_mq_receive(FAR struct file *mq, FAR char *msg, size_t msglen,
                        FAR unsigned int *prio);

/****************************************************************************
 * Name: file_mq_timedreceive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages from
 *   the message queue specified by "mq."  If the message queue is empty
 *   and O_NONBLOCK was not set, file_mq_timedreceive() will block until a
 *   message is added to the message queue (or until a timeout occurs).
 *
 *   file_mq_timedreceive() is an internal OS interface.  It is functionally
 *   equivalent to mq_timedreceive() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_timedreceive() for a more complete description of
 *  the behavior of this function
 *
 * Input Parameters:
 *   mq      - Message Queue Descriptor
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

ssize_t file_mq_timedreceive(FAR struct file *mq, FAR char *msg,
                             size_t msglen, FAR unsigned int *prio,
                             FAR const struct timespec *abstime);

/****************************************************************************
 * Name:  file_mq_setattr
 *
 * Description:
 *   This function sets the attributes associated with the
 *   specified message queue "mq".  Only the "O_NONBLOCK"
 *   bit of the "mq_flags" can be changed.
 *
 *   If "oldstat" is non-null, mq_setattr() will store the
 *   previous message queue attributes at that location (just
 *   as would have been returned by file_mq_getattr()).
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor
 *   mq_stat - New attributes
 *   oldstate - Old attributes
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int file_mq_setattr(FAR struct file *mq, FAR const struct mq_attr *mq_stat,
                    FAR struct mq_attr *oldstat);

/****************************************************************************
 * Name:  file_mq_getattr
 *
 * Description:
 *   This functions gets status information and attributes
 *   associated with the specified message queue.
 *
 * Input Parameters:
 *   mq      - Message queue descriptor
 *   mq_stat - Buffer in which to return attributes
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int file_mq_getattr(FAR struct file *mq, FAR struct mq_attr *mq_stat);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* ___INCLUDE_NUTTX_MQUEUE_H */
