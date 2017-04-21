/****************************************************************************
 * include/nuttx/semaphore.h
 *
 *   Copyright (C) 2014-2016 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_SEMAPHORE_H
#define __INCLUDE_NUTTX_SEMAPHORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for protocol attribute */

#define SEM_PRIO_NONE             0
#define SEM_PRIO_INHERIT          1
#define SEM_PRIO_PROTECT          2

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
 * Name: sem_tickwait
 *
 * Description:
 *   This function is a lighter weight version of sem_timedwait().  It is
 *   non-standard and intended only for use within the RTOS.
 *
 * Parameters:
 *   sem     - Semaphore object
 *   start   - The system time that the delay is relative to.  If the
 *             current time is not the same as the start time, then the
 *             delay will be adjust so that the end time will be the same
 *             in any event.
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to sem_trywait().
 *
 * Return Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.  -ETIMEDOUT is returned on the timeout condition.
 *
 ****************************************************************************/

int sem_tickwait(FAR sem_t *sem, systime_t start, uint32_t delay);

/****************************************************************************
 * Name: sem_reset
 *
 * Description:
 *   Reset a semaphore count to a specific value.  This is similar to part
 *   of the operation of sem_init().  But sem_reset() may need to wake up
 *   tasks waiting on a count.  This kind of operation is sometimes required
 *   within the OS (only) for certain error handling conditions.
 *
 * Parameters:
 *   sem   - Semaphore descriptor to be reset
 *   count - The requested semaphore count
 *
 * Return Value:
 *   0 (OK) or a negated errno value if unsuccessful
 *
 ****************************************************************************/

int sem_reset(FAR sem_t *sem, int16_t count);

/****************************************************************************
 * Name: sem_getprotocol
 *
 * Description:
 *    Return the value of the semaphore protocol attribute.
 *
 * Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               queried.
 *    protocol - The user provided location in which to store the protocol
 *               value.
 *
 * Return Value:
 *   0 if successful.  Otherwise, -1 is returned and the errno value is set
 *   appropriately.
 *
 ****************************************************************************/

int sem_getprotocol(FAR sem_t *sem, FAR int *protocol);

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
 * Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               modified
 *    protocol - The new protocol to use
 *
 * Return Value:
 *   0 if successful.  Otherwise, -1 is returned and the errno value is set
 *   appropriately.
 *
 ****************************************************************************/

int sem_setprotocol(FAR sem_t *sem, int protocol);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_SEMAPHORE_H */
