/****************************************************************************
 * sched/sched/sched_gettcb.c
 *
#if CONFIG_NFILE_STREAMS > 0

 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#include <sched.h>

#include "nuttx/irq.h"

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_tcb
 *
 * Description:
 *   Given a task ID, this function will return the a pointer to the
 *   corresponding TCB (or NULL if there is no such task ID).
 *
 *   NOTE:  This function holds a critical section while examining TCB data
 *   data structures but releases that critical section before returning.
 *   When it is released, the TCB may become unstable.  If the caller
 *   requires absolute stability while using the TCB, then the caller
 *   should establish the critical section BEFORE calling this function and
 *   hold that critical section as long as necessary.
 *
 ****************************************************************************/

FAR struct tcb_s *nxsched_get_tcb(pid_t pid)
{
  FAR struct tcb_s *ret = NULL;
  irqstate_t flags;
  int hash_ndx;

  /* Verify that the PID is within range */

  if (pid >= 0)
    {
      /* Get the hash_ndx associated with the pid */

      hash_ndx = PIDHASH(pid);

      /* The test and the return setup should be atomic.  This still does
       * not provide proper protection if the recipient of the TCB does not
       * also protect against the task associated with the TCB from
       * terminating asynchronously.
       */

      flags = enter_critical_section();

      /* Verify that the correct TCB was found. */

      if (pid == g_pidhash[hash_ndx].pid)
        {
          /* Return the TCB associated with this pid (if any) */

          ret = g_pidhash[hash_ndx].tcb;
        }

      leave_critical_section(flags);
    }

  /* Return the TCB. */

  return ret;
}
