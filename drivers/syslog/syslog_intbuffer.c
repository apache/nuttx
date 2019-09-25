/****************************************************************************
 * drivers/syslog/syslog_intbuffer.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/syslog/syslog.h>
#include <nuttx/irq.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_INTBUFFER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Extend the size of the interrupt buffer so that a "[truncated]\n"
 * indication can be append to the end.
 *
 * The usable capacity of the interrupt buffer is (CONFIG_SYSLOG_INTBUFSIZE - 1).
 */

#define SYSLOG_BUFOVERRUN_MESSAGE  "[truncated]\n"
#define SYSLOG_BUFOVERRUN_SIZE     13

#if CONFIG_SYSLOG_INTBUFSIZE > (65535 - SYSLOG_BUFOVERRUN_SIZE)
#  undef  CONFIG_SYSLOG_INTBUFSIZE
#  define CONFIG_SYSLOG_INTBUFSIZE (65535 - SYSLOG_BUFOVERRUN_SIZE)
#  define SYSLOG_INTBUFSIZE        65535
#else
#  define SYSLOG_INTBUFSIZE \
     (CONFIG_SYSLOG_INTBUFSIZE + SYSLOG_BUFOVERRUN_SIZE)
#endif

#define USABLE_INTBUFSIZE          (CONFIG_SYSLOG_INTBUFSIZE - 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure encapsulates the interrupt buffer state */

struct g_syslog_intbuffer_s
{
  volatile uint16_t si_inndx;
  volatile uint16_t si_outndx;
  uint8_t si_buffer[SYSLOG_INTBUFSIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct g_syslog_intbuffer_s g_syslog_intbuffer;
static const char g_overrun_msg[SYSLOG_BUFOVERRUN_SIZE] = SYSLOG_BUFOVERRUN_MESSAGE;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_remove_intbuffer
 *
 * Description:
 *   Extract any characters that may have been added to the interrupt buffer
 *   to the SYSLOG device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, the extracted character is returned.  EOF is returned if
 *   the interrupt buffer is empty.
 *
 * Assumptions:
 *   Interrupts may or may not be disabled.
 *
 ****************************************************************************/

int syslog_remove_intbuffer(void)
{
  irqstate_t flags;
  uint32_t outndx;
  int ret = EOF;

  /* Extraction of the character and adjustment of the circular buffer
   * indices must be performed in a critical section to protect from
   * concurrent modification from interrupt handlers.
   */

  flags = enter_critical_section();

  /* Check if the interrupt buffer is empty */

  outndx = (uint32_t)g_syslog_intbuffer.si_outndx;
  if (outndx != (uint32_t)g_syslog_intbuffer.si_inndx)
    {
      /* Not empty.. Take the next character from the interrupt buffer */

      ret = g_syslog_intbuffer.si_buffer[outndx];

      /* Increment the OUT index, handling wrap-around */

      if (++outndx >= SYSLOG_INTBUFSIZE)
        {
          outndx -= SYSLOG_INTBUFSIZE;
        }

      g_syslog_intbuffer.si_outndx = (uint16_t)outndx;
    }

  leave_critical_section(flags);

  /* Now we can send the extracted character to the SYSLOG device */

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_add_intbuffer
 *
 * Description:
 *   Add one more character to the interrupt buffer.  In the event of
 *   buffer overflowed, the character will be dropped.  The indication
 *   "[truncated]\n" will be appended to the end of the interrupt buffer.
 *
 * Input Parameters:
 *   ch - The character to add to the interrupt buffer (must be positive).
 *
 * Returned Value:
 *   Zero success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 * Assumptions:
 *   - Called either from (1) interrupt handling logic with interrupts
 *     disabled or from an IDLE thread with interrupts enabled.
 *   - Requires caution because there may be an interrupted execution of
 *     syslog_flush_intbuffer():  Only the outndx can be modified.
 *
 ****************************************************************************/

int syslog_add_intbuffer(int ch)
{
  irqstate_t flags;
  uint32_t inndx;
  uint32_t outndx;
  uint32_t endndx;
  unsigned int inuse;
  int ret;
  int i;

  /* Disable concurrent modification from interrupt handling logic */

  flags = enter_critical_section();

  /* How much space is left in the intbuffer? */

  inndx  = (uint32_t)g_syslog_intbuffer.si_inndx;
  outndx = (uint32_t)g_syslog_intbuffer.si_outndx;

  endndx = inndx;
  if (endndx < outndx)
    {
      endndx += SYSLOG_INTBUFSIZE;
    }

  inuse = (unsigned int)(endndx - outndx);

  /* Is there space for another character (reserving space for the overrun
   * message)?
   */

  if (inuse == USABLE_INTBUFSIZE)
    {
      /* Copy the truncated message one character at a time, handing index
       * wrap-around on each character.
       */

      for (i = 0; i < SYSLOG_BUFOVERRUN_SIZE; i++)
        {
          /* Copy one character */

          g_syslog_intbuffer.si_buffer[inndx] = (uint8_t)g_overrun_msg[i];

          /* Increment the IN index, handling wrap-around */

          if (++inndx >= SYSLOG_INTBUFSIZE)
            {
              inndx -= SYSLOG_INTBUFSIZE;
            }

          DEBUGASSERT(inndx != outndx);
        }

      g_syslog_intbuffer.si_inndx = (uint16_t)inndx;
      ret = -ENOSPC;
    }
  else if (inuse < USABLE_INTBUFSIZE)
    {
       /* Copy one character */

       g_syslog_intbuffer.si_buffer[inndx] = (uint8_t)ch;

       /* Increment the IN index, handling wrap-around */

       if (++inndx >= SYSLOG_INTBUFSIZE)
         {
           inndx -= SYSLOG_INTBUFSIZE;
         }

      g_syslog_intbuffer.si_inndx = (uint16_t)inndx;
      ret = OK;
    }
  else
    {
      /* This character goes to the bit bucket.  We have already copied
       * the overrun message so there is nothing else to do.
       */

      ret = -ENOSPC;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: syslog_flush_intbuffer
 *
 * Description:
 *   Flush any characters that may have been added to the interrupt buffer
 *   to the SYSLOG device.
 *
 * Input Parameters:
 *   channel - The syslog channel to use in performing the flush operation.
 *   force   - Use the force() method of the channel vs. the putc() method.
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 * Assumptions:
 *   Interrupts may or may not be disabled.
 *
 ****************************************************************************/

int syslog_flush_intbuffer(FAR const struct syslog_channel_s *channel,
                           bool force)
{
  syslog_putc_t putfunc;
  int ch;
  int ret = OK;

  /* Select which putc function to use for this flush */

  putfunc = force ? channel->sc_putc : channel->sc_force;

  /* This logic is performed with the scheduler disabled to protect from
   * concurrent modification by other tasks.
   */

  sched_lock();
  do
    {
      /* Transfer one character to time.  This is inefficient, but is
       * done in this way to: (1) Deal with concurrent modification of
       * the interrupt buffer from interrupt activity, (2) Avoid keeper
       * interrupts disabled for a long time, and (3) to handler
       * wraparound of the circular buffer indices.
       */

      ch = syslog_remove_intbuffer();
      if (ch != EOF)
        {
          ret = putfunc(ch);
        }
    }
  while (ch != EOF && ret >= 0);

  sched_unlock();
  return ret;
}

#endif /* CONFIG_SYSLOG_INTBUFFER */
