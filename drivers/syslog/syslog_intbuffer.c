/****************************************************************************
 * drivers/syslog/syslog_intbuffer.c
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
 * The usable capacity of the interrupt buffer is
 * (CONFIG_SYSLOG_INTBUFSIZE - 1).
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
static const char g_overrun_msg[SYSLOG_BUFOVERRUN_SIZE] =
                                            SYSLOG_BUFOVERRUN_MESSAGE;

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

int syslog_flush_intbuffer(bool force)
{
  syslog_putc_t putfunc;
  int ch;
  int i;

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
       * wrap-around of the circular buffer indices.
       */

      ch = syslog_remove_intbuffer();

      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          if ((g_syslog_channel[i] == NULL) || (ch == EOF))
            {
              break;
            }

          /* Select which putc function to use for this flush */

          putfunc = force ? g_syslog_channel[i]->sc_ops->sc_force :
                    g_syslog_channel[i]->sc_ops->sc_putc;

          putfunc(g_syslog_channel[i], ch);
        }
    }
  while (ch != EOF);

  sched_unlock();

  return ch;
}

#endif /* CONFIG_SYSLOG_INTBUFFER */
