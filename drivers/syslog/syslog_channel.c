/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>

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

static int syslog_force(int ch)
{
  DEBUGASSERT(g_syslog_channel != NULL && g_syslog_channel->sc_force != NULL);

  return g_syslog_channel->sc_force(ch);
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
 * Input buffer:
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

  if (up_interrupt_context())
    {
#ifdef CONFIG_SYSLOG_INTBUFFER
      /* Buffer the character in the interrupt buffer.  The interrupt buffer
       * will be flushed before the next normal, non-interrupt SYSLOG output.
       */

      return syslog_add_intbuffer(ch);
#else
      /* Force the character to the SYSLOG device immediately (if possible).
       * This means that the interrupt data may not be in synchronization
       * with output data that may have been buffered by sc_putc().
       */

      return syslog_force(ch);
#endif
    }
  else
    {
      DEBUGASSERT(g_syslog_channel->sc_putc != NULL);

#ifdef CONFIG_SYSLOG_INTBUFFER
      /* Flush any characters that may have been added to the interrupt
       * buffer.
       */

      (void)syslog_flush_intbuffer(&g_syslog_channel, false);
#endif

      return g_syslog_channel->sc_putc(ch);
    }
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

int syslog_flush(void)
{
  DEBUGASSERT(g_syslog_channel != NULL && g_syslog_channel->sc_flush != NULL);

#ifdef CONFIG_SYSLOG_INTBUFFER
  /* Flush any characters that may have been added to the interrupt
   * buffer.
   */

  (void)syslog_flush_intbuffer(&g_syslog_channel, true);
#endif

  /* Then flush all of the buffered output to the SYSLOG device */

  return g_syslog_channel->sc_flush();
}
