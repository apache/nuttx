/****************************************************************************
 * drivers/syslog/syslog_channel.c
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_RAMLOG_SYSLOG
#  include <nuttx/syslog/ramlog.h>
#elif defined(CONFIG_ARCH_LOWPUTC)
#  include <nuttx/arch.h>
#endif

#include "syslog.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_ARCH_LOWPUTC
static int syslog_default_putc(int ch);
#endif
static int syslog_default_flush(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_RAMLOG_SYSLOG)
static const struct syslog_channel_s g_default_channel =
{
  ramlog_putc,
  ramlog_putc,
  syslog_default_flush
};
#elif defined(CONFIG_ARCH_LOWPUTC)
static const struct syslog_channel_s g_default_channel =
{
  up_putc,
  up_putc,
  syslog_default_flush
};
#else
static const struct syslog_channel_s g_default_channel =
{
  syslog_default_putc,
  syslog_default_putc,
  syslog_default_flush
};
#endif

/* This is the current syslog channel in use */

static FAR const struct syslog_channel_s *g_syslog_channel = &g_default_channel;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_default_putc and syslog_default_flush
 *
 * Description:
 *   Dummy, no-nothing channel interface methods
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LOWPUTC
static int syslog_default_putc(int ch)
{
  return ch;
}
#endif

static int syslog_default_flush(void)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_channel
 *
 * Description:
 *   Configure the SYSLOGging function to use the provided channel to
 *   generate SYSLOG output.
 *
 * Input Parameters:
 *   channel - Provides the interface to the channel to be used.
 *
 * Returned Value:
 *   Zero (OK)is returned on  success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int syslog_channel(FAR const struct syslog_channel_s *channel)
{
  DEBUGASSERT(channel != NULL);

  if (channel != NULL)
    {
      DEBUGASSERT(channel->sc_putc != NULL && channel->sc_force != NULL &&
                  channel->sc_flush != NULL);

      g_syslog_channel = channel;
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: syslog_putc
 *
 * Description:
 *   This is the low-level system logging interface.
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

int syslog_putc(int ch)
{
  DEBUGASSERT(g_syslog_channel != NULL);

  /* Is this an attempt to do SYSLOG output from an interrupt handler? */

  if (up_interrupt_context() || sched_idletask())
    {
#if defined(CONFIG_SYSLOG_INTBUFFER)
      /* Buffer the character in the interrupt buffer.  The interrupt buffer
       * will be flushed before the next normal, non-interrupt SYSLOG output.
       */

      return syslog_add_intbuffer(ch);
#else
      /* Force the character to the SYSLOG device immediately (if possible).
       * This means that the interrupt data may not be in synchronization
       * with output data that may have been buffered by sc_putc().
       */

      DEBUGASSERT(g_syslog_channel->sc_force != NULL);

      return g_syslog_channel->sc_force(ch);
#endif
    }
  else
    {
      DEBUGASSERT(g_syslog_channel->sc_putc != NULL);

#ifdef CONFIG_SYSLOG_INTBUFFER
      /* Flush any characters that may have been added to the interrupt
       * buffer.
       */

      (void)syslog_flush_intbuffer(g_syslog_channel, false);
#endif

      return g_syslog_channel->sc_putc(ch);
    }
}

/****************************************************************************
 * Name: syslog_force
 *
 * Description:
 *   This is the low-level system logging interface.  This version forces
 *   the output and is only used in emergency situations (e.g., in assertion
 *   handling).
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

int syslog_force(int ch)
{
  DEBUGASSERT(g_syslog_channel != NULL && g_syslog_channel->sc_force != NULL);

#ifdef CONFIG_SYSLOG_INTBUFFER
  /* Flush any characters that may have been added to the interrupt
   * buffer through the emergency channel
   */

  (void)syslog_flush_intbuffer(g_syslog_channel, true);
#endif

  /* Then send the character to the emergency channel */

  return g_syslog_channel->sc_force(ch);
}

/****************************************************************************
 * Name: syslog_flush
 *
 * Description:
 *   This is called by system crash-handling logic.  It must flush any
 *   buffered data to the SYSLOG device.
 *
 *   Interrupts are disabled at the time of the crash and this logic must
 *   perform the flush using low-level, non-interrupt driven logic.
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   Zero (OK)is returned on  success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#if 0
/* REVISIT: (1) Not yet integrated into assertion handlers and (2) there is
 * an implementation problem in that if a character driver is the underlying
 * device, then there is no mechanism to flush the data buffered in the
 * driver with interrupts disabled.
 */

int syslog_flush(void)
{
  DEBUGASSERT(g_syslog_channel != NULL && g_syslog_channel->sc_flush != NULL);

#ifdef CONFIG_SYSLOG_INTBUFFER
  /* Flush any characters that may have been added to the interrupt
   * buffer.
   */

  (void)syslog_flush_intbuffer(g_syslog_channel, true);
#endif

  /* Then flush all of the buffered output to the SYSLOG device */

  return g_syslog_channel->sc_flush();
}
#endif
