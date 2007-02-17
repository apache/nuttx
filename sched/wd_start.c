/************************************************************
 * wd_start.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <wdog.h>
#include <unistd.h>
#include <sched.h>
#include "wd_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  wd_start
 *
 * Description:
 *   This function adds a watchdog to the timer queue.  The 
 *   specified watchdog function will be called from the
 *   interrupt level after the specified number of ticks has
 *   elapsed. Watchdog timers may be started from the
 *   interrupt level.
 *
 *   Watchdog timers execute in the address enviroment that
 *   was in effect when wd_start() is called.
 *
 *   Watchdog timers execute only once.
 *
 *   To replace either the timeout delay or the function to
 *   be executed, call wd_start again with the same wdId; only
 *   the most recent wdStart() on a given watchdog ID has
 *   any effect.
 *
 * Parameters:
 *   wdId = watchdog ID
 *   delay = Delay count in clock ticks
 *   wdentry = function to call on timeout
 *   parm1..4 = parameters to pass to wdentry
 *
 * Return Value:
 *   OK or ERROR
 *
 * Assumptions:
 *   The watchdog routine runs in the context of the timer interrupt
 *   handler and is subject to all ISR restrictions.
 *
 ************************************************************/

STATUS wd_start(WDOG_ID wdId, int delay, wdentry_t wdentry,
                int parm1, int parm2, int parm3, int parm4)
{
  wdog_t *curr;
  wdog_t *prev;
  wdog_t *next;
  sint32  now;
  sint32  saved_state;

  /* Verify the wdId */

  if (!wdId)
    {
      return ERROR;
    }

  /* Check if the watchdog has been started. If so, stop it.
   * NOTE:  There is a race condition here... the caller may receive
   * the watchdog between the time that wd_start is called and
   * the critical section is established.
   */

  saved_state = irqsave();
  if (wdId->active)
    {
      wd_cancel(wdId);
    }

  /* Save the data in the watchdog structure */

  wdId->func    = wdentry;         /* Function to execute when delay expires */
  wdId->parm[0] = parm1;           /* Same as the parameter to pass */
  wdId->parm[1] = parm2;           /* 2nd parameter not used */
  wdId->parm[2] = parm3;           /* 3rd parameter not used */
  wdId->parm[3] = parm4;           /* 4th parameter not used */

  /* Calculate delay+1, forcing the delay into a range that we can handle */

  if (delay <= 0)
    {
      delay = 1;
    }
  else if (++delay <= 0)
    {
      delay--;
    }

  /* Do the easy case first -- when the watchdog timer queue is empty. */

  if (g_wdactivelist.head == NULL)
    {
      sq_addlast((sq_entry_t*)wdId,&g_wdactivelist);
    }

  /* There are other active watchdogs in the timer queue */

  else
    {
      now = 0;
      prev = curr = (wdog_t*)g_wdactivelist.head;

      /* Advance to positive time */

      while ((now += curr->lag) < 0 && curr->next)
        {
          prev = curr;
          curr = curr->next;
        }

      /* Advance past shorter delays */

      while (now <= delay && curr->next)
       {
         prev = curr;
         curr = curr->next;
         now += curr->lag;
       }

      /* Check if the new wdId must be inserted before the curr. */

      if (delay < now)
        {
          /* The relative delay time is smaller or equal to the current delay
           * time, so decrement the current delay time by the new relative
           * delay time.
           */

          delay -= (now - curr->lag);
          curr->lag -= delay;

          /* Insert the new watchdog in the list */

          if (curr == (wdog_t*)g_wdactivelist.head)
            {
              sq_addfirst((sq_entry_t*)wdId, &g_wdactivelist);
            }
          else
            {
              sq_addafter((sq_entry_t*)prev, (sq_entry_t*)wdId,
                          &g_wdactivelist);
            }
        }

      /* The new watchdog delay time is greater than the curr delay time,
       * so the new wdId must be inserted after the curr. This only occurs
       * if the wdId is to be added to the end of the list.
       */

      else
        {
          delay -= now;
          if (!curr->next)
            {
              sq_addlast((sq_entry_t*)wdId, &g_wdactivelist);
            }
          else
            {
              next = curr->next;
              next->lag -= delay;
              sq_addafter((sq_entry_t*)curr, (sq_entry_t*)wdId,
                          &g_wdactivelist);
            }
        }
    }

  /* Put the lag into the watchdog structure and mark it as active. */

  wdId->lag = delay;
  wdId->active = TRUE;

  irqrestore(saved_state);
  return OK;
}

/************************************************************
 * Function:  wd_timer
 *
 * Description:
 *   This function is called from the timer interrupt
 *   handler to determine if it is time to execute a watchdog
 *   function.  If so, the watchdog function will be executed
 *   in the context of the timer interrupt handler.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ************************************************************/

void wd_timer(void)
{
  pid_t   pid;
  wdog_t *wdog;

  /* Check if there are any active watchdogs to process */

  if (g_wdactivelist.head)
    {
      /* There are.  Decrement the lag counter */

      --(((wdog_t*)g_wdactivelist.head)->lag);

      /* Check if the watchdog at the head of the list is ready to run */

      if (((wdog_t*)g_wdactivelist.head)->lag <= 0)
        {
          /* Process the watchdog at the head of the list as well as any
           * other watchdogs that became ready to run at this time
           */

          while (g_wdactivelist.head &&
                 ((wdog_t*)g_wdactivelist.head)->lag <= 0)
            {
              /* Remove the watchdog from the head of the list */

              wdog = (wdog_t*)sq_remfirst(&g_wdactivelist);

              /* If there is another watchdog behind this one, update its
               * its lag (this shouldn't be necessary).
               */

              if (g_wdactivelist.head)
                {
                  ((wdog_t*)g_wdactivelist.head)->lag += wdog->lag;
                }

              /* Indicate that the watchdog is no longer activer. */

              wdog->active = FALSE;

              /* Get the current task's process ID.  We'll need this later to
               * see if the watchdog function caused a context switch.
               */

              pid = getpid();

              /* Execute the watchdog function */

              (*wdog->func)(wdog->parm[0], wdog->parm[1],
                            wdog->parm[2] ,wdog->parm[3]);
            }
        }
    }
}
