/****************************************************************************
 * include/nuttx/signal.h
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

#ifndef __INCLUDE_NUTTX_SIGNAL_H
#define __INCLUDE_NUTTX_SIGNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <signal.h>

#include <nuttx/wqueue.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SIG_EVTHREAD
  #define sigwork_init(work) memset(work, 0, sizeof(*work));
#else
  #define sigwork_init(work) (void)(work)
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct sigwork_s
{
  struct work_s work;           /* Work queue structure */
  union sigval value;           /* Data passed with notification */
  sigev_notify_function_t func; /* Notification function */
};

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Most internal nxsig_* interfaces are not available in the user space in
 * PROTECTED and KERNEL builds.  In that context, the application signal
 * interfaces must be used.  The differences between the two sets of
 * interfaces are:  (1) the nxsig_* interfaces do not cause cancellation
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
 *
 * The interfaces sigtimedwait(), sigwait(), sigwaitinfo(), sleep(),
 * nanosleep(), and usleep()  are cancellation points.
 *
 * REVISIT:  The fact that these interfaces are cancellation points is an
 * issue and may cause violations:  It use of these internally will cause
 * the calling function to become a cancellation points!
 */

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
#  define _SIG_PROCMASK(h,s,o)  nxsig_procmask(h,s,o)
#  define _SIG_SIGACTION(s,a,o) nxsig_action(s,a,o,false)
#  define _SIG_QUEUE(p,s,v)     nxsig_queue(p,s,v)
#  define _SIG_KILL(p,s)        nxsig_kill(p,s);
#  define _SIG_WAITINFO(s,i)    nxsig_timedwait(s,i,NULL)
#  define _SIG_NANOSLEEP(r,a)   nxsig_nanosleep(r,a)
#  define _SIG_SLEEP(s)         nxsig_sleep(s)
#  define _SIG_USLEEP(u)        nxsig_usleep(u)
#  define _SIG_ERRNO(r)         (-(r))
#  define _SIG_ERRVAL(r)        (r)
#else
#  define _SIG_PROCMASK(h,s,o)  sigprocmask(h,s,o)
#  define _SIG_SIGACTION(s,a,o) sigaction(s,a,o)
#  define _SIG_QUEUE(p,s,v)     sigqueue(p,s,v)
#  define _SIG_KILL(p,s)        kill(p,s);
#  define _SIG_WAITINFO(s,i)    sigwaitinfo(s,i)
#  define _SIG_NANOSLEEP(r,a)   nanosleep(r,a)
#  define _SIG_SLEEP(s)         sleep(s)
#  define _SIG_USLEEP(u)        usleep(u)
#  define _SIG_ERRNO(r)         errno
#  define _SIG_ERRVAL(r)        (-errno)
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct timespec;  /* Forward reference */

/****************************************************************************
 * Name: nxsig_ismember
 *
 * Description:
 *   This function tests whether the signal specified by signo is a member
 *   of the set specified by set.
 *
 * Input Parameters:
 *   set - Signal set to test
 *   signo - Signal to test for
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   On success, it returns 0 if the signal is not a member, 1 if the signal
 *   is a member of the set.
 *   A negated errno value is returned on failure.
 *
 *    EINVAL - The signo argument is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsig_ismember(FAR const sigset_t *set, int signo);

/****************************************************************************
 * Name: nxsig_addset
 *
 * Description:
 *   This function adds the signal specified by signo to the signal set
 *   specified by set.
 *
 * Input Parameters:
 *   set - Signal set to add signal to
 *   signo - Signal to add
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EINVAL - The signo argument is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsig_addset(FAR sigset_t *set, int signo);

/****************************************************************************
 * Name: nxsig_delset
 *
 * Description:
 *   This function deletes the signal specified by signo from the signal
 *   set specified by the 'set' argument.
 *
 * Input Parameters:
 *   set - Signal set to delete the signal from
 *   signo - Signal to delete
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EINVAL - The signo argument is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsig_delset(FAR sigset_t *set, int signo);

/****************************************************************************
 * Name: nxsig_pendingset
 *
 * Description:
 *   Convert the list of pending signals into a signal set
 *
 * Input Parameters:
 *   stcb - The specific tcb of return pending set.
 *
 * Returned Value:
 *   Return the pending signal set.
 *
 * Assumptions:
 *
 ****************************************************************************/

sigset_t nxsig_pendingset(FAR struct tcb_s *stcb);

/****************************************************************************
 * Name: nxsig_procmask
 *
 * Description:
 *   This function allows the calling process to examine and/or change its
 *   signal mask.  If the 'set' is not NULL, then it points to a set of
 *   signals to be used to change the currently blocked set.  The value of
 *   'how' indicates the manner in which the set is changed.
 *
 *   If there any pending unblocked signals after the call to
 *   nxsig_procmask(), those signals will be delivered before
 *    nxsig_procmask() returns.
 *
 *   If nxsig_procmask() fails, the signal mask of the process is not changed
 *   by this function call.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sigprocmask() except that it does not modify the errno value.
 *
 * Input Parameters:
 *   how - How the signal mask will be changed:
 *         SIG_BLOCK   - The resulting set is the union of the current set
 *                       and the signal set pointed to by 'set'.
 *         SIG_UNBLOCK - The resulting set is the intersection of the current
 *                       set and the complement of the signal set pointed to
 *                       by 'set'.
 *         SIG_SETMASK - The resulting set is the signal set pointed to by
 *                       'set'.
 *   set  - Location of the new signal mask
 *   oset - Location to store the old signal mask
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EINVAL - The 'how' argument is invalid.
 *
 ****************************************************************************/

int nxsig_procmask(int how, FAR const sigset_t *set, FAR sigset_t *oset);

/****************************************************************************
 * Name: nxsig_action
 *
 * Description:
 *   This function allows the calling process to examine and/or specify the
 *   action to be associated with a specific signal.  This is a non-standard,
 *   OS internal version of the standard sigaction() function. nxsig_action()
 *   adds an additional parameter, force, that is used to set default signal
 *   actions (which may not normally be settable).  nxsig_action() does not
 *   alter the errno variable.
 *
 * Input Parameters:
 *   sig   - Signal of interest
 *   act   - Location of new handler
 *   oact  - Location to store only handler
 *   force - Force setup of the signal handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int nxsig_action(int signo, FAR const struct sigaction *act,
                 FAR struct sigaction *oact, bool force);

/****************************************************************************
 * Name: nxsig_queue
 *
 * Description:
 *   This function sends the signal specified by signo with the signal
 *   parameter value to the process specified by pid.
 *
 *   If the receiving process has the signal blocked via the sigprocmask,
 *   the signal will pend until it is unmasked. Only one pending signal (per
 *   signo) is retained.  This is consistent with POSIX which states, "If
 *   a subsequent occurrence of a pending signal is generated, it is
 *   implementation defined as to whether the signal is delivered more than
 *   once.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sigqueue() except that it does not modify the errno value.
 *
 * Input Parameters:
 *   pid   - Process ID of task to receive signal
 *   signo - Signal number
 *   value - Value to pass to task with signal
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EGAIN  - The limit of signals which may be queued has been reached.
 *    EINVAL - sig was invalid.
 *    EPERM  - The  process  does  not  have  permission to send the
 *             signal to the receiving process.
 *    ESRCH  - No process has a PID matching pid.
 *
 ****************************************************************************/

int nxsig_queue(int pid, int signo, union sigval value);

/****************************************************************************
 * Name: nxsig_kill
 *
 * Description:
 *   The nxsig_kill() system call can be used to send any signal to any task.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   the POSIX standard kill() function but does not modify the application
 *   errno variable.
 *
 *   Limitation: Sending of signals to 'process groups' is not
 *   supported in NuttX
 *
 * Input Parameters:
 *   pid - The id of the task to receive the signal.  The POSIX nxsig_kill
 *     specification encodes process group information as zero and
 *     negative pid values.  Only positive, non-zero values of pid are
 *     supported by this implementation.
 *   signo - The signal number to send.  If signo is zero, no signal is
 *     sent, but all error checking is performed.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EINVAL An invalid signal was specified.
 *    EPERM  The process does not have permission to send the
 *           signal to any of the target processes.
 *    ESRCH  The pid or process group does not exist.
 *    ENOSYS Do not support sending signals to process groups.
 *
 ****************************************************************************/

int nxsig_kill(pid_t pid, int signo);

/****************************************************************************
 * Name: nxsig_waitinfo
 *
 * Description:
 *   This function is equivalent to nxsig_timedwait with a NULL timeout
 *   parameter.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sigwaitinfo() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   set - The pending signal set
 *   info - The returned value
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

#define nxsig_waitinfo(s,i) nxsig_timedwait(s,i,NULL)

/****************************************************************************
 * Name: nxsig_timedwait
 *
 * Description:
 *   This function selects the pending signal set specified by the argument
 *   set.  If multiple signals are pending in set, it will remove and return
 *   the lowest numbered one.  If no signals in set are pending at the time
 *   of the call, the calling process will be suspended until one of the
 *   signals in set becomes pending, OR until the process is interrupted by
 *   an unblocked signal, OR until the time interval specified by timeout
 *   (if any), has expired. If timeout is NULL, then the timeout interval
 *   is forever.
 *
 *   If the info argument is non-NULL, the selected signal number is stored
 *   in the si_signo member and the cause of the signal is stored in the
 *   si_code member.  The content of si_value is only meaningful if the
 *   signal was generated by sigqueue().
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sigtimedwait() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   set     - The pending signal set.
 *   info    - The returned value (may be NULL).
 *   timeout - The amount of time to wait (may be NULL)
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *   EAGAIN - No signal specified by set was generated within the specified
 *            timeout period.
 *   EINTR  - The wait was interrupted by an unblocked, caught signal.
 *
 ****************************************************************************/

int nxsig_timedwait(FAR const sigset_t *set, FAR struct siginfo *info,
                    FAR const struct timespec *timeout);

/****************************************************************************
 * Name: nxsig_nanosleep
 *
 * Description:
 *   The nxsig_nanosleep() function causes the current thread to be
 *   suspended from execution until either the time interval specified by
 *   the rqtp argument has elapsed or a signal is delivered to the calling
 *   thread and its action is to invoke a signal-catching function or to
 *   terminate the process. The suspension time may be longer than requested
 *   because the argument value is rounded up to an integer multiple of the
 *   sleep resolution or because of the scheduling of other activity by the
 *   system. But, except for the case of being interrupted by a signal, the
 *   suspension time will not be less than the time specified by rqtp, as
 *   measured by the system clock, CLOCK_REALTIME.
 *
 *   The use of the nxsig_nanosleep() function has no effect on the action
 *   or blockage of any signal.
 *
 * Input Parameters:
 *   rqtp - The amount of time to be suspended from execution.
 *   rmtp - If the rmtp argument is non-NULL, the timespec structure
 *          referenced by it is updated to contain the amount of time
 *          remaining in the interval (the requested time minus the time
 *          actually slept)
 *
 * Returned Value:
 *   If the nxsig_nanosleep() function returns because the requested time
 *   has elapsed, its return value is zero.
 *
 *   If the nxsig_nanosleep() function returns because it has been
 *   interrupted by a signal, the function returns a negated errno value
 *   indicate the interruption. If the rmtp argument is non-NULL, the
 *   timespec structure referenced by it is updated to contain the amount
 *   of time remaining in the interval (the requested time minus the time
 *   actually slept). If the rmtp argument is NULL, the remaining time is
 *   not returned.
 *
 *   If nxsig_nanosleep() fails, it returns a negated errno indicating the
 *   cause of the failure. The nxsig_nanosleep() function will fail if:
 *
 *     EINTR - The nxsig_nanosleep() function was interrupted by a signal.
 *     EINVAL - The rqtp argument specified a nanosecond value less than
 *       zero or greater than or equal to 1000 million.
 *     ENOSYS - The nxsig_nanosleep() function is not supported by this
 *       implementation.
 *
 ****************************************************************************/

int nxsig_nanosleep(FAR const struct timespec *rqtp,
                    FAR struct timespec *rmtp);

/****************************************************************************
 * Name: nxsig_sleep
 *
 * Description:
 *   The nxsig_sleep() function will cause the calling thread to be
 *   suspended from execution until either the number of real-time seconds
 *   specified by the argument 'seconds' has elapsed or a signal is
 *   delivered to the calling thread.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   the standard sleep() application interface except that:
 *
 *   - It is not a cancellation point, and
 *   - There is no check that the action of the signal is to invoke a
 *     signal-catching function or to terminate the process.
 *
 *   See the description of sleep() for additional information that is not
 *   duplicated here.
 *
 * Input Parameters:
 *   seconds - The number of seconds to sleep
 *
 * Returned Value:
 *   If nxsig_sleep() returns because the requested time has elapsed, the
 *   value returned will be zero (OK). If nxsig_sleep() returns because of
 *   premature arousal due to delivery of a signal, the return value will
 *   be the "unslept" amount (the requested time minus the time actually
 *   slept) in seconds.
 *
 ****************************************************************************/

unsigned int nxsig_sleep(unsigned int seconds);

/****************************************************************************
 * Name: nxsig_usleep
 *
 * Description:
 *   The nxsig_usleep() function will cause the calling thread to be
 *   suspended from execution until either the number of real-time
 *   microseconds specified by the argument 'usec' has elapsed or a signal
 *   is delivered to the calling thread. The suspension time may be longer
 *   than requested due to the scheduling of other activity by the system.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   the standard nxsig_usleep() application interface except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *   See the description of usleep() for additional information that is not
 *   duplicated here.
 *
 * Input Parameters:
 *   usec - the number of microseconds to wait.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsig_usleep(useconds_t usec);

/****************************************************************************
 * Name: nxsig_notification
 *
 * Description:
 *   Notify a client an event via either a signal or a function call
 *   base on the sigev_notify field.
 *
 * Input Parameters:
 *   pid   - The task/thread ID a the client thread to be signaled.
 *   event - The instance of struct sigevent that describes how to signal
 *           the client.
 *   code  - Source: SI_USER, SI_QUEUE, SI_TIMER, SI_ASYNCIO, or SI_MESGQ
 *   work  - The work structure to queue.  Must be non-NULL if
 *           event->sigev_notify == SIGEV_THREAD.  Ignored if
 *           CONFIG_SIG_EVTHREAD is not defined.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsig_notification(pid_t pid, FAR struct sigevent *event,
                       int code, FAR struct sigwork_s *work);

/****************************************************************************
 * Name: nxsig_cancel_notification
 *
 * Description:
 *   Cancel the notification if it doesn't send yet.
 *
 * Input Parameters:
 *   work  - The work structure to cancel
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_EVTHREAD
void nxsig_cancel_notification(FAR struct sigwork_s *work);
#else
  #define nxsig_cancel_notification(work) (void)(work)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SIGNAL_H */
