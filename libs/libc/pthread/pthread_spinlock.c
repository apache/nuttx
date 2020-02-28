/****************************************************************************
 * libs/libc/pthread/pthread_spinlock.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/boardctl.h>

#include <pthread.h>
#include <sched.h>

/* The architecture specific spinlock.h header file must provide the
 * following:
 *
 *   SP_LOCKED    - A definition of the locked state value (usually 1)
 *   SP_UNLOCKED  - A definition of the unlocked state value (usually 0)
 *   spinlock_t   - The type of a spinlock memory object (usually uint8_t).
 */

#include <arch/spinlock.h>

#ifdef CONFIG_PTHREAD_SPINLOCKS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMPOSSIBLE_THREAD ((pthread_t)-1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_spin_init
 *
 * Description:
 *   The pthread_spin_init() function will allocate any resources required
 *   to use the spin lock referenced by lock and initialize the lock to an
 *   unlocked state.
 *
 *   The results are undefined if pthread_spin_init() is called specifying
 *   an already initialized spin lock. The results are undefined if a spin
 *   lock is used without first being initialized.
 *
 *   If the pthread_spin_init() function fails, the lock is not initialized
 *   and the contents of lock are undefined.
 *
 *   Only the object referenced by lock may be used for performing
 *   synchronization.  The result of referring to copies of that object in
 *   calls to pthread_spin_destroy(), pthread_spin_lock(),
 *   pthread_spin_trylock(), or pthread_spin_unlock() is undefined.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to be initialized.
 *   pshared - Unused
 *
 * Returned Value:
 *   Zero (OK) is returned if a valid lock value is provided.
 *
 * POSIX Compatibility:
 *   Not supported: "If the Thread Process-Shared Synchronization option is
 *   supported and the value of pshared is PTHREAD_PROCESS_SHARED, the
 *   implementation shall permit the spin lock to be operated upon by any
 *   thread that has access to the memory where the spin lock is allocated,
 *   even if it is allocated in memory that is shared by multiple
 *   processes."
 *
 ****************************************************************************/

int pthread_spin_init(FAR pthread_spinlock_t *lock, int pshared)
{
  int ret = EINVAL;

  DEBUGASSERT(lock != NULL);
  if (lock != NULL)
    {
      lock->sp_lock   = SP_UNLOCKED;
      lock->sp_holder = IMPOSSIBLE_THREAD;
      ret             = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: pthread_spin_destroy
 *
 * Description:
 *   The pthread_spin_destroy() function will destroy the spin lock
 *   referenced by lock and release any resources used by the lock.  The
 *   effect of subsequent use of the lock is undefined until the lock is
 *   reinitialized by another call to pthread_spin_init(). The results are
 *   undefined if pthread_spin_destroy() is called when a thread holds the
 *   lock, or if this function is called with an uninitialized thread spin
 *   lock.
 *
 * Input Parameters:
 *   lock    - A reference to the spinlock object to be initialized.
 *   pshared - Unused
 *
 * Returned Value:
 *   Zero (OK) is always returned.
 *
 * POSIX Compatibility:
 *   Not supported: "If the Thread Process-Shared Synchronization option is
 *   supported and the value of pshared is PTHREAD_PROCESS_SHARED, the
 *   implementation shall permit the spin lock to be operated upon by any
 *   thread that has access to the memory where the spin lock is allocated,
 *   even if it is allocated in memory that is shared by multiple
 *   processes."
 *
 ****************************************************************************/

int pthread_spin_destroy(pthread_spinlock_t *lock)
{
  return OK;
}

/****************************************************************************
 * Name: pthread_spin_lock
 *
 * Description:
 *   The pthread_spin_lock() function will lock the spin lock referenced by
 *   lock. The calling thread will acquire the lock if it is not held by
 *   another thread. Otherwise, the thread will spin (that is, will not
 *   return from the pthread_spin_lock() call) until the lock becomes
 *   available.
 *
 *   If it is determined the value specified by the lock argument to
 *   pthread_spin_lock() refers to a spin lock object for which the calling
 *   thread already holds the lock, it is recommended that the function will
 *   fail and report an EDEADLK error.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   Zero is returned if the lock was successfully acquired.  Otherwise one
 *   of the following errno values are returned:
 *
 *   EINVAL    - 'lock' does not refer to a valid spinlock object
 *   EDEADLOCK - The caller already holds the spinlock
 *
 ****************************************************************************/

int pthread_spin_lock(pthread_spinlock_t *lock)
{
  pthread_t me = pthread_self();
  int ret;

  DEBUGASSERT(lock != NULL);
  if (lock == NULL)
    {
      return EINVAL;
    }
  else if (lock->sp_holder == me)
    {
      return EDEADLOCK;
    }

  /* Loop until we successfully take the spinlock (i.e., until the previous
   * state of the spinlock was SP_UNLOCKED).  NOTE that the test/set operaion
   * is performed via boardctl() to avoid a variety of issues.  An option
   * might be to move the implementation of up_testset() to libs/libc/machine.
   */

  do
    {
      ret = boardctl(BOARDIOC_TESTSET, (uintptr_t)&lock->sp_lock);
    }
  while (ret == 1);

  /* Check for success (previous state was SP_UNLOCKED) */

  if (ret == 0)
    {
      lock->sp_holder = me;
    }
  else
    {
      /* An error of some kind is the only other possibility */

      DEBUGASSERT(ret < 0);
      ret = -ret;
    }

  return ret;
}

/****************************************************************************
 * Name: pthread_spin_trylock
 *
 * Description:
 *   The pthread_spin_trylock() function will lock the spin lock referenced
 *   by lock. The calling thread will acquire the lock if it is not held by
 *   another thread. Otherwise, The pthread_spin_trylock() will return a
 *   failure.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   Zero is returned if the lock was successfully acquired.  Otherwise one
 *   of the following errno values are returned:
 *
 *   EINVAL - 'lock' does not refer to a valid spinlock object
 *   EBUSY  - The spinlock is held by another thread
 *
 ****************************************************************************/

int pthread_spin_trylock(pthread_spinlock_t *lock)
{
  pthread_t me = pthread_self();
  int ret;

  DEBUGASSERT(lock != NULL);
  if (lock == NULL)
    {
      ret = EINVAL;
    }
  else if (lock->sp_holder == me)
    {
      ret = OK;
    }
  else
    {
      /* Perform the test/set operation via boardctl() */

      ret = boardctl(BOARDIOC_TESTSET, (uintptr_t)&lock->sp_lock);
      switch (ret)
        {
          case 0:  /* Previously unlocked.  We hold the spinlock */
            lock->sp_holder = me;
            break;

          case 1:  /* Previously locked.  We did not get the spinlock  */
            ret = EBUSY;
            break;

          default:
            DEBUGASSERT(ret < 0);
            ret = -ret;
            break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pthread_spin_unlock
 *
 * Description:
 *   The pthread_spin_unlock() function will release the spin lock
 *   referenced by lock which was locked via the pthread_spin_lock() or
 *   pthread_spin_trylock() functions.
 *
 *   The results are undefined if the lock is not held by the calling thread.
 *
 *   If there are threads spinning on the lock when pthread_spin_unlock() is
 *   called, the lock becomes available and an unspecified spinning thread
 *   will acquire the lock.
 *
 *   The results are undefined if this function is called with an
 *   uninitialized thread spin lock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   Zero is returned if the lock was successfully released.  Otherwise one
 *   of the following errno values are returned:
 *
 *   EINVAL    - 'lock' does not refer to a valid spinlock object
 *   EPERM     - The caller does not hold the spinlock or it is not locked
 *
 ****************************************************************************/

int pthread_spin_unlock(pthread_spinlock_t *lock)
{
  pthread_t me = pthread_self();

  DEBUGASSERT(lock != NULL &&
              lock->sp_lock == SP_LOCKED &&
              lock->sp_holder == me);

  if (lock == NULL)
    {
      return EINVAL;
    }
  else if (lock->sp_lock != SP_LOCKED || lock->sp_holder != me)
    {
      return EPERM;
    }

  /* Release the lock */

  lock->sp_holder = IMPOSSIBLE_THREAD;
  lock->sp_lock   = SP_UNLOCKED;
  return OK;
}

#endif /* CONFIG_PTHREAD_SPINLOCKS */
