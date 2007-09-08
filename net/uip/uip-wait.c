/************************************************************
 * uip-wait.c
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <semaphore.h>
#include <arch/irq.h>
#include <net/uip/uip.h>

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Data
 ************************************************************/

/* Threads and tasks can both wait a semaphore.  This
 * initializer uses internal, non-portable knowledge of the\
 * structure of sem_t to initialize it to the value 0.
 */

static sem_t  uip_waitsem   = { 0 }; /* Semaphore to wait on */
static uint16 uip_waitflags = 0;     /* UIP flags to wait for */

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Global Functions
 ************************************************************/

/* This function is called user code to set up the wait */

int uip_event_wait(uint16 waitflags)
{
  /* Prevent conflicts with the interrupt level operation of
   * uip_event_signal().
   */
  irqstate_t save = irqsave();

  /* At present, we support only a single waiter. If uip_waitflags
   * is non-zero on entry, then there is a problem.
   */

  if ( uip_waitflags == 0)
    {
      /* If the requested event bits are not set in the uip_flags,
       * then wait.  We will be awakened as soon as an interrupt is
       * received that sets these bits.
       */

      goto errout_with_irqdisabled;
    }

  /* Loop until the requested bits are set in the flags */

  while ((waitflags & uip_flags) != 0)
    {
      /* Set the event flags that the caller is waiting on */

      uip_waitflags = waitflags;

      /* Wait for the event to occur */

      if (sem_wait(&uip_waitsem) != 0)
        {
          goto errout_with_irqdisabled;
        }
    }

    irqrestore(save);
    return OK;

errout_with_irqdisabled:
  irqrestore(save);
  return ERROR;
}

/* This function is called from uip_interrupt() to wake up any
 * waiting threads/tasks.
 */

void uip_event_signal(void)
{
  /* If any bits in uip_waitflags match bits set in uip_flags, then
   * there must be a thread/task waiting for this event to occur.
   */

    if ((uip_waitflags & uip_flags) != 0)
      {
        sem_post(&uip_waitsem); /* Post the event */
        uip_waitflags = 0;      /* Prohibit posting the event multiple times */
      }
}
