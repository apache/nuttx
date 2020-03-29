/****************************************************************************
 * include/nuttx/semaphore.h
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

#ifndef __INCLUDE_NUTTX_SEMAPHORE_H
#define __INCLUDE_NUTTX_SEMAPHORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <errno.h>
#include <semaphore.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for protocol attribute */

#define SEM_PRIO_NONE             0
#define SEM_PRIO_INHERIT          1
#define SEM_PRIO_PROTECT          2

/* Most internal nxsem_* interfaces are not available in the user space in
 * PROTECTED and KERNEL builds.  In that context, the application semaphore
 * interfaces must be used.  The differences between the two sets of
 * interfaces are:  (1) the nxsem_* interfaces do not cause cancellation
 * points and (2) they do not modify the errno variable.
 *
 * This is only important when compiling libraries (libc or libnx) that are
 * used both by the OS (libkc.a and libknx.a) or by the applications
 * (libuc.a and libunx.a).  In that case, the correct interface must be
 * used for the build context.
 *
 * REVISIT:  In the flat build, the same functions must be used both by
 * the OS and by applications.  We have to use the normal user functions
 * in this case or we will fail to set the errno or fail to create the
 * cancellation point.
 */

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
#  define _SEM_INIT(s,p,c)      nxsem_init(s,p,c)
#  define _SEM_DESTROY(s)       nxsem_destroy(s)
#  define _SEM_WAIT(s)          nxsem_wait(s)
#  define _SEM_TRYWAIT(s)       nxsem_trywait(s)
#  define _SEM_TIMEDWAIT(s,t)   nxsem_timedwait(s,t)
#  define _SEM_POST(s)          nxsem_post(s)
#  define _SEM_GETVALUE(s)      nxsem_getvalue(s)
#  define _SEM_GETPROTOCOL(s,p) nxsem_getprotocol(s,p)
#  define _SEM_SETPROTOCOL(s,p) nxsem_setprotocol(s,p)
#  define _SEM_ERRNO(r)         (-(r))
#  define _SEM_ERRVAL(r)        (r)
#else
#  define _SEM_INIT(s,p,c)      sem_init(s,p,c)
#  define _SEM_DESTROY(s)       sem_destroy(s)
#  define _SEM_WAIT(s)          sem_wait(s)
#  define _SEM_TRYWAIT(s)       sem_trywait(s)
#  define _SEM_TIMEDWAIT(s,t)   sem_timedwait(s,t)
#  define _SEM_GETVALUE(s,v)    sem_getvalue(s,v)
#  define _SEM_POST(s)          sem_post(s)
#  define _SEM_GETPROTOCOL(s,p) sem_getprotocol(s,p)
#  define _SEM_SETPROTOCOL(s,p) sem_setprotocol(s,p)
#  define _SEM_ERRNO(r)         errno
#  define _SEM_ERRVAL(r)        (-errno)
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_NAMED_SEMAPHORES
/* This is the named semaphore inode */

struct inode;
struct nsem_inode_s
{
  /* This must be the first element of the structure.  In sem_close() this
   * structure must be cast compatible with sem_t.
   */

  sem_t ns_sem;                     /* The contained semaphore */

  /* Inode payload unique to named semaphores. */

  FAR struct inode *ns_inode;       /* Containing inode */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: nxsem_init
 *
 * Description:
 *   This function initializes the UNNAMED semaphore sem. Following a
 *   successful call to nxsem_init(), the semaphore may be used in subsequent
 *   calls to nxsem_wait(), nxsem_post(), and nxsem_trywait().  The semaphore
 *   remains usable until it is destroyed.
 *
 *   Only sem itself may be used for performing synchronization. The result
 *   of referring to copies of sem in calls to sem_wait(), sem_trywait(),
 *   sem_post(), and sem_destroy() is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be initialized
 *   pshared - Process sharing (not used)
 *   value - Semaphore initialization value
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_init(FAR sem_t *sem, int pshared, unsigned int value);

/****************************************************************************
 * Name: nxsem_destroy
 *
 * Description:
 *   This function is used to destroy the un-named semaphore indicated by
 *   'sem'.  Only a semaphore that was created using nxsem_init() may be
 *   destroyed using nxsem_destroy(); the effect of calling nxsem_destroy()
 *   with a named semaphore is undefined.  The effect of subsequent use of
 *   the semaphore sem is undefined until sem is re-initialized by another
 *   call to nxsem_init().
 *
 *   The effect of destroying a semaphore upon which other processes are
 *   currently blocked is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be destroyed.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_destroy(FAR sem_t *sem);

/****************************************************************************
 * Name: nxsem_wait
 *
 * Description:
 *   This function attempts to lock the semaphore referenced by 'sem'.  If
 *   the semaphore value is (<=) zero, then the calling task will not return
 *   until it successfully acquires the lock.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     EINVAL - Invalid attempt to get the semaphore
 *     EINTR  - The wait was interrupted by the receipt of a signal.
 *
 ****************************************************************************/

int nxsem_wait(FAR sem_t *sem);

/****************************************************************************
 * Name: nxsem_trywait
 *
 * Description:
 *   This function locks the specified semaphore only if the semaphore is
 *   currently not locked.  Otherwise, it locks the semaphore.  In either
 *   case, the call returns without blocking.
 *
 * Input Parameters:
 *   sem - the semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     EINVAL - Invalid attempt to get the semaphore
 *     EAGAIN - The semaphore is not available.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsem_trywait(FAR sem_t *sem);

/****************************************************************************
 * Name: nxsem_timedwait
 *
 * Description:
 *   This function will lock the semaphore referenced by sem as in the
 *   sem_wait() function. However, if the semaphore cannot be locked without
 *   waiting for another process or thread to unlock the semaphore by
 *   performing a sem_post() function, this wait will be terminated when the
 *   specified timeout expires.
 *
 *   The timeout will expire when the absolute time specified by abstime
 *   passes, as measured by the clock on which timeouts are based (that is,
 *   when the value of that clock equals or exceeds abstime), or if the
 *   absolute time specified by abstime has already been passed at the
 *   time of the call.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   That may be one of:
 *
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   EINTR     A signal interrupted this function.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

struct timespec; /* Forward reference */
int nxsem_timedwait(FAR sem_t *sem, FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxsem_tickwait
 *
 * Description:
 *   This function is a lighter weight version of sem_timedwait().  It is
 *   non-standard and intended only for use within the RTOS.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   start   - The system time that the delay is relative to.  If the
 *             current time is not the same as the start time, then the
 *             delay will be adjust so that the end time will be the same
 *             in any event.
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to sem_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure:
 *
 *     -ETIMEDOUT is returned on the timeout condition.
 *     -ECANCELED may be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int nxsem_tickwait(FAR sem_t *sem, clock_t start, uint32_t delay);

/****************************************************************************
 * Name: nxsem_post
 *
 * Description:
 *   When a kernel thread has finished with a semaphore, it will call
 *   nxsem_post().  This function unlocks the semaphore referenced by sem
 *   by performing the semaphore unlock operation on that semaphore.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to sem_wait().
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int nxsem_post(FAR sem_t *sem);

/****************************************************************************
 * Name:  nxsem_getvalue
 *
 * Description:
 *   This function updates the location referenced by 'sval' argument to
 *   have the value of the semaphore referenced by 'sem' without effecting
 *   the state of the semaphore.  The updated value represents the actual
 *   semaphore value that occurred at some unspecified time during the call,
 *   but may not reflect the actual value of the semaphore when it is
 *   returned to the calling task.
 *
 *   If 'sem' is locked, the value return by nxsem_getvalue() will either be
 *   zero or a negative number whose absolute value represents the number
 *   of tasks waiting for the semaphore.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *   sval - Buffer by which the value is returned
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_getvalue(FAR sem_t *sem, FAR int *sval);

/****************************************************************************
 * Name: nxsem_reset
 *
 * Description:
 *   Reset a semaphore count to a specific value.  This is similar to part
 *   of the operation of nxsem_init().  But nxsem_reset() may need to wake up
 *   tasks waiting on a count.  This kind of operation is sometimes required
 *   within the OS (only) for certain error handling conditions.
 *
 * Input Parameters:
 *   sem   - Semaphore descriptor to be reset
 *   count - The requested semaphore count
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_reset(FAR sem_t *sem, int16_t count);

/****************************************************************************
 * Name: nxsem_getprotocol
 *
 * Description:
 *    Return the value of the semaphore protocol attribute.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               queried.
 *    protocol - The user provided location in which to store the protocol
 *               value.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

#define nxsem_getprotocol(s,p) sem_getprotocol(s,p)

/****************************************************************************
 * Name: sem_getprotocol
 *
 * Description:
 *    Return the value of the semaphore protocol attribute.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               queried.
 *    protocol - The user provided location in which to store the protocol
 *               value.
 *
 * Returned Value:
 *   This function is exposed as a non-standard application interface.  It
 *   returns zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_getprotocol(FAR sem_t *sem, FAR int *protocol);

/****************************************************************************
 * Name: nxsem_setprotocol
 *
 * Description:
 *    Set semaphore protocol attribute.
 *
 *    One particularly important use of this function is when a semaphore
 *    is used for inter-task communication like:
 *
 *      TASK A                 TASK B
 *      sem_init(sem, 0, 0);
 *      sem_wait(sem);
 *                             sem_post(sem);
 *      Awakens as holder
 *
 *    In this case priority inheritance can interfere with the operation of
 *    the semaphore.  The problem is that when TASK A is restarted it is a
 *    holder of the semaphore.  However, it never calls sem_post(sem) so it
 *    becomes *permanently* a holder of the semaphore and may have its
 *    priority boosted when any other task tries to acquire the semaphore.
 *
 *    The fix is to call nxsem_setprotocol(SEM_PRIO_NONE) immediately after
 *    the sem_init() call so that there will be no priority inheritance
 *    operations on this semaphore.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               modified
 *    protocol - The new protocol to use
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_setprotocol(FAR sem_t *sem, int protocol);

/****************************************************************************
 * Name: sem_setprotocol
 *
 * Description:
 *    Set semaphore protocol attribute.
 *
 *    One particularly important use of this function is when a semaphore
 *    is used for inter-task communication like:
 *
 *      TASK A                 TASK B
 *      sem_init(sem, 0, 0);
 *      sem_wait(sem);
 *                             sem_post(sem);
 *      Awakens as holder
 *
 *    In this case priority inheritance can interfere with the operation of
 *    the semaphore.  The problem is that when TASK A is restarted it is a
 *    holder of the semaphore.  However, it never calls sem_post(sem) so it
 *    becomes *permanently* a holder of the semaphore and may have its
 *    priority boosted when any other task tries to acquire the semaphore.
 *
 *    The fix is to call sem_setprotocol(SEM_PRIO_NONE) immediately after
 *    the sem_init() call so that there will be no priority inheritance
 *    operations on this semaphore.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               modified
 *    protocol - The new protocol to use
 *
 * Returned Value:
 *   This function is exposed as a non-standard application interface.  It
 *   returns zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_setprotocol(FAR sem_t *sem, int protocol);

/****************************************************************************
 * Name: nxsem_wait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_wait(), which is
 *   uninterruptible and convenient for use.
 *
 * Parameters:
 *   sem - Semaphore descriptor.
 *
 * Return Value:
 *   Zero(OK) - On success
 *   EINVAL - Invalid attempt to get the semaphore
 *
 ****************************************************************************/

#ifdef CONFIG_HAVE_INLINE
static inline int nxsem_wait_uninterruptible(FAR sem_t *sem)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(sem);
    }
  while (ret == -EINTR || ret == -ECANCELED);

  return ret;
}
#else
int nxsem_wait_uninterruptible(FAR sem_t *sem);
#endif

/****************************************************************************
 * Name: nxsem_timedwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_timedwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

#ifdef CONFIG_HAVE_INLINE
static inline int
  nxsem_timedwait_uninterruptible(FAR sem_t *sem,
                                  FAR const struct timespec *abstime)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_timedwait(sem, abstime);
    }
  while (ret == -EINTR);

  return ret;
}
#else
int nxsem_timedwait_uninterruptible(FAR sem_t *sem,
                                    FAR const struct timespec *abstime);
#endif

/****************************************************************************
 * Name: nxsem_tickwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_tickwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   start   - The system time that the delay is relative to.  If the
 *             current time is not the same as the start time, then the
 *             delay will be adjust so that the end time will be the same
 *             in any event.
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to sem_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure:
 *
 *     -ETIMEDOUT is returned on the timeout condition.
 *     -ECANCELED may be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

#ifdef CONFIG_HAVE_INLINE
static inline int
  nxsem_tickwait_uninterruptible(FAR sem_t *sem, clock_t start,
                                 uint32_t delay)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_tickwait(sem, start, delay);
    }
  while (ret == -EINTR);

  return ret;
}
#else
int nxsem_tickwait_uninterruptible(FAR sem_t *sem, clock_t start,
                                   uint32_t delay);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_SEMAPHORE_H */
