/****************************************************************************
 * include/sys/msg.h
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

#ifndef __INCLUDE_SYS_MSG_H
#define __INCLUDE_SYS_MSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ipc.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The MSG_NOERROR identifier value, the msqid_ds struct and the msg struct
 * are as defined by the SV API Intel 386 Processor Supplement.
 */

#define MSG_NOERROR 010000  /* No error if message is too big */
#define MSG_EXCEPT  020000  /* Recv any msg except of specified type.*/
#define MSG_COPY    040000  /* Copy (not remove) all queue messages */

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

typedef unsigned long msgqnum_t;
typedef unsigned long msglen_t;

struct msqid_ds
{
  struct ipc_perm msg_perm;   /* Ownership and permissions */
  time_t          msg_stime;  /* Time of last msgsnd(2) */
  time_t          msg_rtime;  /* Time of last msgrcv(2) */
  time_t          msg_ctime;  /* Time of last change */
  unsigned long   msg_cbytes; /* Current number of bytes in
                               * queue (nonstandard) */
  msgqnum_t       msg_qnum;   /* Current number of messages
                               * in queue */
  msglen_t        msg_qbytes; /* Maximum number of bytes
                               * allowed in queue */
  pid_t           msg_lspid;  /* PID of last msgsnd(2) */
  pid_t           msg_lrpid;  /* PID of last msgrcv(2) */
};

/* Structure describing a message.  The SVID doesn't suggest any
 * particular name for this structure.  There is a reference in the
 * msgop man page that reads "The structure mymsg is an example of what
 * this user defined buffer might look like, and includes the following
 * members:".  This sentence is followed by two lines equivalent
 * to the mtype and mtext field declarations below.  It isn't clear
 * if "mymsg" refers to the name of the structure type or the name of an
 * instance of the structure...
 */

struct mymsg
{
  long  mtype;    /* message type (+ve integer) */
  char  mtext[1]; /* message body */
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: msgctl
 *
 * Description:
 *   System V message control operations.
 *   msgctl() performs the control operation specified by cmd on the
 *   System V message queue with identifier msqid.
 *
 * Input Parameters:
 *   msqid    - System V message queue identifier
 *   cmd      - Command operations
 *   msqid_ds - Defines a message queue
 *
 * Returned Value:
 *   On success, IPC_STAT, IPC_SET, and IPC_RMID return 0.  A
 *   successful IPC_INFO or MSG_INFO operation returns the index of
 *   the highest used entry in the kernel's internal array recording
 *   information about all message queues.  (This information can be
 *   used with repeated MSG_STAT or MSG_STAT_ANY operations to obtain
 *   information about all queues on the system.)  A successful
 *   MSG_STAT or MSG_STAT_ANY operation returns the identifier of the
 *   queue whose index was given in msqid.
 *
 *   On failure, -1 is returned and errno is set to indicate the error.
 *
 ****************************************************************************/

int msgctl(int msqid, int cmd, FAR struct msqid_ds *buf);

/****************************************************************************
 * Name: msgget
 *
 * Description:
 *   Get a System V message queue identifier
 *   The msgget() system call returns the System V message queue
 *   identifier associated with the value of the key argument.  It may
 *   be used either to obtain the identifier of a previously created
 *   message queue (when msgflg is zero and key does not have the
 *   value IPC_PRIVATE), or to create a new set.
 *
 * Input Parameters:
 *   key    - Key associated with the message queue
 *   msgflg - Operations and permissions flag
 *
 * Returned Value:
 *   On success, msgget() returns the message queue identifier (a
 *   nonnegative integer).  On failure, -1 is returned, and errno is
 *   set to indicate the error.
 *
 ****************************************************************************/

int msgget(key_t key, int msgflg);

/****************************************************************************
 * Name: msgsnd
 *
 * Description:
 *   The msgsnd() function is used to send a message to the queue
 *   associated with the message queue identifier specified by msqid.
 *   The msgp argument points to a user-defined buffer that must contain
 *   first a field of type long int that will specify the type of the
 *   message, and then a data portion that will hold the data bytes of
 *   the message.
 *
 * Input Parameters:
 *   msqid  - Message queue identifier
 *   msgp   - Pointer to a buffer with the message to be sent
 *   msgsz  - Length of the data part of the message to be sent
 *   msgflg - Operations flags
 *
 * Returned Value:
 *   On success, mq_send() returns 0 (OK); on error, -1 (ERROR)
 *   is returned, with errno set to indicate the error:
 *
 *   EAGAIN   The queue was full and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int msgsnd(int msqid, FAR const void *msgp, size_t msgsz, int msgflg);

/****************************************************************************
 * Name: msgrcv
 *
 * Description:
 *   The msgrcv() function reads a message from the message queue specified
 *   by the msqid parameter and places it in the user-defined buffer
 *   pointed to by the *msgp parameter.
 *
 * Input Parameters:
 *   msqid  - Message queue identifier
 *   msgp   - Pointer to a buffer in which the received message will be
 *            stored
 *   msgsz  - Length of the data part of the buffer
 *   msgtyp - Type of message to be received.
 *   msgflg - Operations flags.
 *
 * Returned Value:
 *   On success, msgrcv() returns the number of bytes actually copied
 *   into the mtext array.
 *   On failure, both functions return -1, and set errno to indicate
 *   the error.
 *
 ****************************************************************************/

ssize_t msgrcv(int msqid, FAR void *msgp,
               size_t msgsz, long msgtyp, int msgflg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SYS_MSG_H */
