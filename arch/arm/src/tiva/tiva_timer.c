/****************************************************************************
 * arch/arm/src/tiva/tiva_timer.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip/tiva_syscontrol.h"
#include "chip/tiva_timer.h"

#include "tiva_enableclks.h"
#include "tiva_enablepwr.h"
#include "tiva_periphrdy.h"
#include "tiva_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure retains the fixed, well-known attributes of a GPTM module */

struct tiva_gptmattr_s
{
  uintptr_t base;              /* Register base address */
  int irq[2];                  /* Timer A/B interrupt numbers */
  xcpt_t handler32;            /* Handler for 32-bit timer interrupts */
  xcpt_t handler16[2];         /* Handlers for 16-bit timer A/B interrupts */
};

/* This structure represents the state of a GPTM module */

struct tiva_gptmstate_s
{
  /* Constant timer attributes and configuration */

  const struct tiva_gptmattr_s   *attr;
  const struct tiva_gptmconfig_s *config;

  /* Variable state values */

  uint32_t clkin;              /* Frequency of the input clock */
  uint32_t imr;                /* Interrupt mask value. Zero if no interrupts */

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Register level debug */

   bool wrlast;                /* Last was a write */
   uintptr_t addrlast;         /* Last address */
   uint32_t vallast;           /* Last value */
   int ntimes;                 /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register Access */

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
static bool tiva_timer_checkreg(struct tiva_gptmstate_s *priv, bool wr,
                                uint32_t regval, uintptr_t regaddr);
#endif
static uint32_t tiva_getreg(struct tiva_gptmstate_s *priv, unsigned int offset);
static void tiva_putreg(struct tiva_gptmstate_s *priv, unsigned int offset,
              uint32_t regval);

/* Interrupt handling */

static int  tiva_timer32_interrupt(struct tiva_gptmstate_s *priv);
#ifdef CONFIG_TIVA_TIMER0
static int  tiva_gptm0_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER1
static int  tiva_gptm1_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER2
static int  tiva_gptm2_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER3
static int  tiva_gptm3_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER4
static int  tiva_gptm4_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER5
static int  tiva_gptm5_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER6
static int  tiva_gptm6_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER7
static int  tiva_gptm7_interrupt(int irq, FAR void *context);
#endif

static int  tiva_timer16_interrupt(struct tiva_gptmstate_s *priv,
              int tmndx);
#ifdef CONFIG_TIVA_TIMER0
static int  tiva_timer0a_interrupt(int irq, FAR void *context);
static int  tiva_timer0b_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER1
static int  tiva_timer1a_interrupt(int irq, FAR void *context);
static int  tiva_timer1b_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER2
static int  tiva_timer2a_interrupt(int irq, FAR void *context);
static int  tiva_timer2b_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER3
static int  tiva_timer3a_interrupt(int irq, FAR void *context);
static int  tiva_timer3b_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER4
static int  tiva_timer4a_interrupt(int irq, FAR void *context);
static int  tiva_timer4b_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER5
static int  tiva_timer5a_interrupt(int irq, FAR void *context);
static int  tiva_timer5b_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER6
static int  tiva_timer6a_interrupt(int irq, FAR void *context);
static int  tiva_timer6b_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_TIVA_TIMER7
static int  tiva_timer7a_interrupt(int irq, FAR void *context);
static int  tiva_timer7b_interrupt(int irq, FAR void *context);
#endif

/* Timer initialization and configuration */

static int  tiva_oneshot_periodic_mode32(struct tiva_gptmstate_s *priv,
              const struct tiva_timer32config_s *timer);
static int  tiva_oneshot_periodic_mode16(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);
static int  tiva_rtc_mode32(struct tiva_gptmstate_s *priv,
              const struct tiva_timer32config_s *timer);
static int  tiva_input_edgecount_mode16(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);
static int  tiva_input_time_mode16(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);
static int  tiva_pwm_mode16(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);

static int  tiva_timer32_configure(struct tiva_gptmstate_s *priv,
              const struct tiva_timer32config_s *timer);
static int  tiva_timer16_configure(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER0
static const struct tiva_gptmattr_s g_gptm0_attr =
{
  .base      = TIVA_TIMER0_BASE,
  .irq       = { TIVA_IRQ_TIMER0A, TIVA_IRQ_TIMER0B },
  .handler32 = tiva_gptm0_interrupt,
  .handler16 = { tiva_timer0a_interrupt, tiva_timer0b_interrupt },
};

static struct tiva_gptmstate_s g_gptm0_state;
#endif

#ifdef CONFIG_TIVA_TIMER1
static const struct tiva_gptmattr_s g_gptm1_attr =
{
  .base      = TIVA_TIMER1_BASE,
  .irq       = { TIVA_IRQ_TIMER1A, TIVA_IRQ_TIMER1B },
  .handler32 = tiva_gptm1_interrupt,
  .handler16 = { tiva_timer1a_interrupt, tiva_timer1b_interrupt },
};

static struct tiva_gptmstate_s g_gptm1_state;
#endif

#ifdef CONFIG_TIVA_TIMER2
static const struct tiva_gptmattr_s g_gptm2_attr =
{
  .base      = TIVA_TIMER2_BASE,
  .irq       = { TIVA_IRQ_TIMER2A, TIVA_IRQ_TIMER2B },
  .handler32 = tiva_gptm2_interrupt,
  .handler16 = { tiva_timer2a_interrupt, tiva_timer2b_interrupt },
};

static struct tiva_gptmstate_s g_gptm2_state;
#endif

#ifdef CONFIG_TIVA_TIMER3
static const struct tiva_gptmattr_s g_gptm3_attr =
{
  .base      = TIVA_TIMER3_BASE,
  .irq       = { TIVA_IRQ_TIMER3A, TIVA_IRQ_TIMER3B },
  .handler32 = tiva_gptm3_interrupt,
  .handler16 = { tiva_timer3a_interrupt, tiva_timer3b_interrupt },
};

static struct tiva_gptmstate_s g_gptm3_state;
#endif

#ifdef CONFIG_TIVA_TIMER4
static const struct tiva_gptmattr_s g_gptm4_attr =
{
  .base      = TIVA_TIMER4_BASE,
  .irq       = { TIVA_IRQ_TIMER4A, TIVA_IRQ_TIMER4B },
  .handler32 = tiva_gptm4_interrupt,
  .handler16 = { tiva_timer4a_interrupt, tiva_timer4b_interrupt },
};

static struct tiva_gptmstate_s g_gptm4_state;
#endif

#ifdef CONFIG_TIVA_TIMER5
static const struct tiva_gptmattr_s g_gptm5_attr =
{
  .base      = TIVA_TIMER5_BASE,
  .irq       = { TIVA_IRQ_TIMER5A, TIVA_IRQ_TIMER5B },
  .handler32 = tiva_gptm5_interrupt,
  .handler16 = { tiva_timer5a_interrupt, tiva_timer5b_interrupt },
};

static struct tiva_gptmstate_s g_gptm5_state;
#endif

#ifdef CONFIG_TIVA_TIMER6
static const struct tiva_gptmattr_s g_gptm6_attr =
{
  .base      = TIVA_TIMER6_BASE,
  .irq       = { TIVA_IRQ_TIMER6A, TIVA_IRQ_TIMER6B },
  .handler32 = tiva_gptm6_interrupt,
  .handler16 = { tiva_timer6a_interrupt, tiva_timer6b_interrupt },
};

static struct tiva_gptmstate_s g_gptm6_state;
#endif

#ifdef CONFIG_TIVA_TIMER7
static const struct tiva_gptmattr_s g_gptm7_attr =
{
  .base      = TIVA_TIMER7_BASE,
  .irq       = { TIVA_IRQ_TIMER7A, TIVA_IRQ_TIMER7B },
  .handler32 = tiva_gptm7_interrupt,
  .handler16 = { tiva_timer7a_interrupt, tiva_timer7b_interrupt },
};

static struct tiva_gptmstate_s g_gptm7_state;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
static bool tiva_timer_checkreg(struct tiva_gptmstate_s *priv, bool wr,
                                uint32_t regval, uintptr_t regaddr)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      regaddr == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = regaddr;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: tiva_getreg
 *
 * Description:
 *   Read one 32-bit GPTM register
 *
 ****************************************************************************/

static uint32_t tiva_getreg(struct tiva_gptmstate_s *priv, unsigned int offset)
{
  uintptr_t regaddr = priv->attr->base + offset;
  uint32_t regval =  getreg32(regaddr);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  if (tiva_timer_checkreg(priv, false, regval, regaddr))
    {
      lldbg("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: tiva_putreg
 *
 * Description:
 *   Write one 32-bit GPTM register
 *
 ****************************************************************************/

static void tiva_putreg(struct tiva_gptmstate_s *priv, unsigned int offset,
                        uint32_t regval)
{
  uintptr_t regaddr = priv->attr->base + offset;

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  if (tiva_timer_checkreg(priv, true, regval, regaddr))
    {
       lldbg("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: tiva_modifyreg
 *
 * Description:
 *   This function permits atomic of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
 *
 ****************************************************************************/

static void tiva_modifyreg(struct tiva_gptmstate_s *priv, unsigned int offset,
                           uint32_t clrbits, uint32_t setbits)
{
#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  irqstate_t flags;
  uint32_t regval;

  flags = irqsave();
  regval = tiva_getreg(priv, offset);
  regval &= ~clrbits;
  regval |= setbits;
  tiva_putreg(priv, offset, regval);
  irqrestore(flags);
#else
  uintptr_t regaddr = priv->attr->base + offset;
  modifyreg32(regaddr, clrbits, setbits);
#endif
}

/****************************************************************************
 * Name: tiva_timer32_interrupt
 *
 * Description:
 *   Common interrupt handler for 32-bit timers
 *
 ****************************************************************************/

static int tiva_timer32_interrupt(struct tiva_gptmstate_s *priv)
{
  const struct tiva_gptm32config_s *config32;
  const struct tiva_timer32config_s *timer32;
  uint32_t status;

  DEBUGASSERT(priv && priv->attr && priv->config);

  /* Get the set of pending interrupts from the mask interrupt status
   * register.
   */

  status = tiva_getreg(priv, TIVA_TIMER_MIS_OFFSET);
  DEBUGASSERT(status != 0);

  if (status != 0)
    {
      tiva_putreg(priv, TIVA_TIMER_ICR_OFFSET, status);

      /* If this was a match interrupt, then disable further match
       * interrupts.
       */

      if ((status & TIMER_INT_TAM) != 0)
        {
          priv->imr &= ~TIMER_INT_TAM;
          tiva_putreg(priv, TIVA_TIMER_IMR_OFFSET, priv->imr);
        }

      /* Get the timer configuration from the private state structure */

      config32 = (const struct tiva_gptm32config_s *)priv->config;
      timer32  = &config32->config;

      /* Forward the interrupt to the handler.  There should always be a non-
       * NULL handler in this case.
       */

      DEBUGASSERT(timer32->handler);
      if (timer32->handler)
        {
          timer32->handler((TIMER_HANDLE)priv, config32, status);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: tiva_gptmN_interrupt, N=0..7
 *
 * Description:
 *   Individual interrupt handlers for each 32-bit timer
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER0
static int tiva_gptm0_interrupt(int irq, FAR void *context)
{
  return tiva_timer32_interrupt(&g_gptm0_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER1
static int tiva_gptm1_interrupt(int irq, FAR void *context)
{
  return tiva_timer32_interrupt(&g_gptm1_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER2
static int tiva_gptm2_interrupt(int irq, FAR void *context)
{
  return tiva_timer32_interrupt(&g_gptm2_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER3
static int tiva_gptm3_interrupt(int irq, FAR void *context)
{
  return tiva_timer32_interrupt(&g_gptm3_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER4
static int tiva_gptm4_interrupt(int irq, FAR void *context)
{
  return tiva_timer32_interrupt(&g_gptm4_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER5
static int tiva_gptm5_interrupt(int irq, FAR void *context)
{
  return tiva_timer32_interrupt(&g_gptm5_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER6
static int tiva_gptm6_interrupt(int irq, FAR void *context)
{
  return tiva_timer32_interrupt(&g_gptm6_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER7
static int tiva_gptm7_interrupt(int irq, FAR void *context)
{
  return tiva_timer32_interrupt(&g_gptm7_state);
}
#endif

/****************************************************************************
 * Name: tiva_timer16_interrupt
 *
 * Description:
 *   Common interrupt handler for 16-bit timers
 *
 ****************************************************************************/

static int tiva_timer16_interrupt(struct tiva_gptmstate_s *priv, int tmndx)
{
  const struct tiva_gptm16config_s  *config16;
  const struct tiva_timer16config_s *timer16;
  uint32_t intmask;
  uint32_t status;

  DEBUGASSERT(priv && priv->attr && priv->config && (unsigned)tmndx < 2);

  /* Read the masked interrupt status, masking out bits only for this timer. */

  intmask = tmndx ? TIMERB_INTS : TIMERA_INTS;
  status  = tiva_getreg(priv, TIVA_TIMER_MIS_OFFSET) & intmask;
  DEBUGASSERT(status != 0);

  if (status != 0)
    {
      /* Clear all pending interrupts for this timer */

      tiva_putreg(priv, TIVA_TIMER_ICR_OFFSET, status);

      /* If this was a match interrupt, then disable further match
       * interrupts.
       */

      if ((status & (TIMER_INT_TAM | TIMER_INT_TBM)) != 0)
        {
          priv->imr &= ~((TIMER_INT_TAM | TIMER_INT_TBM) & intmask);
          tiva_putreg(priv, TIVA_TIMER_IMR_OFFSET, priv->imr);
        }

      /* Get the timer configuration from the private state structure */

      config16 = (const struct tiva_gptm16config_s *)priv->config;
      timer16  = &config16->config[tmndx];

      /* Forward the interrupt to the handler.  There should always be a
       * non-NULL handler in this context.
       */

      DEBUGASSERT(timer16->handler);
      if (timer16->handler)
        {
          timer16->handler((TIMER_HANDLE)priv, config16, status, tmndx);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: tiva_timerNa_interrupt, tiva_timerNb_interrupt, N=0..7
 *
 * Description:
 *   Individual interrupt handlers for each 16-bit timer
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER0
static int tiva_timer0a_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm0_state, TIMER16A);
}

static int tiva_timer0b_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm0_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER1
static int tiva_timer1a_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm1_state, TIMER16A);
}

static int tiva_timer1b_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm1_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER2
static int tiva_timer2a_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm2_state, TIMER16A);
}

static int tiva_timer2b_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm2_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER3
static int tiva_timer3a_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm3_state, TIMER16A);
}

static int tiva_timer3b_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm3_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER4
static int tiva_timer4a_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm4_state, TIMER16A);
}

static int tiva_timer4b_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm4_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER5
static int tiva_timer5a_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm5_state, TIMER16A);
}

static int tiva_timer5b_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm5_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER6
static int tiva_timer6a_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm6_state, TIMER16A);
}

static int tiva_timer6b_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm6_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER7
static int tiva_timer7a_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm7_state, TIMER16A);
}

static int tiva_timer7b_interrupt(int irq, FAR void *context)
{
  return tiva_timer16_interrupt(&g_gptm7_state, TIMER16B);
}
#endif

/****************************************************************************
 * Name: tiva_oneshot_periodic_mode32
 *
 * Description:
 *   Configure a 32-bit timer to operate in one-short or periodic mode
 *
 ****************************************************************************/

static int tiva_oneshot_periodic_mode32(struct tiva_gptmstate_s *priv,
                                        const struct tiva_timer32config_s *timer)
{
  uint32_t regval;

  /* The GPTM is configured for One-Shot and Periodic modes by the following
   * sequence:
   *
   * 1. Ensure the timer is disabled (the TAEN bit in the GPTMCTL register
   *    is cleared) before making any changes.
   *
   *    NOTE: The TAEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 32-bit
   *    operation.
   */

   tiva_putreg(priv, TIVA_TIMER_CFG_OFFSET, TIMER_CFG_CFG_32);

  /* 3. Configure the TAMR field in the GPTM Timer n Mode Register
   *   (GPTMTAMR):
   *
   *    a. Write a value of 0x1 for One-Shot mode.
   *    b. Write a value of 0x2 for Periodic mode.
   *
   *    When Timer A and TimerB are concatenated, the GPTMTBMR register is
   *    ignored and GPTMTAMR controls the modes for both Timer A and Timer B
   */

  regval  = tiva_getreg(priv, TIVA_TIMER_TAMR_OFFSET);
  regval &= ~TIMER_TnMR_TnMR_MASK;

  if (priv->config->mode == TIMER32_MODE_ONESHOT)
    {
      regval |= TIMER_TnMR_TnMR_ONESHOT;
    }
  else /* if (priv->config->mode == TIMER32_MODE_PERIODIC) */
    {
      regval |= TIMER_TnMR_TnMR_PERIODIC;
    }

  tiva_putreg(priv, TIVA_TIMER_TAMR_OFFSET, regval);

  /* 4. Optionally configure the TASNAPS, TAWOT, TAMINTD, and TACDIR bits in
   *    the GPTMTAMR register to select whether to capture the value of the
   *    free-running timer at time-out, use an external trigger to start
   *    counting, configure an additional trigger or interrupt, and count up
   *    or down.
   *
   *    TASNAPS - GPTM Timer A Snap-Shot Mode
   *              0: Snap-shot mode is disabled (default)
   *              1: If the 32-bit timeris configured in the periodic mode,
   *                 the actual free-running, capture or snapshot value of
   *                 the timer is loaded at the time-out event/capture or
   *                 snapshot event into the concatenated GPTM Timer A
   *                 (GPTMTAR) register.
   *    TAWOT   - GPTM Timer A Wait-on-Trigger
   *              0: The 32-bit timer begins counting as soon as it is
   *                 enabled (default).
   *              1: If the 32-bit timer is enabled, it does not begin
   *                 counting until it receives a trigger from the timer
   *                 in the previous position in the daisy chain.
   *    TAINTD  - One-shot/Periodic Interrupt Disable
   *              0: Time-out interrupt functions as normal.
   *              1: Time-out interrupt are disabled (default).
   *    TACDIR  - GPTM Timer A Count Direction
   *              0: The timer counts down (default).
   *              1: The timer counts up. When counting up, the timer
   *                 starts from a value of 0.
   */

  /* Setup defaults */

  regval &= (TIMER_TnMR_TnCDIR | TIMER_TnMR_TnWOT | TIMER_TnMR_TnCDIR);
  regval |= TIMER_TnMR_TnCINTD;

  /* Enable snapshot mode?
   *
   *   In periodic, snap-shot mode (TnMR field is 0x2 and the TnSNAPS bit is
   *   set in the GPTMTnMR register), the value of the timer at the time-out
   *   event is loaded into the GPTMTnR register and the value of the
   *   prescaler is loaded into the GPTMTnPS register. The free-running
   *   counter value is shown in the GPTMTnV register. In this manner,
   *   software can determine the time elapsed from the interrupt assertion
   *   to the ISR entry by examining the snapshot values and the current value
   *   of the free-running timer. Snapshot mode is not available when the
   *   timer is configured in one-shot mode.
   *
   * TODO: Not implemented
   */
#warning Missing Logic

   /* Enable wait-on-trigger?
    *
    * TODO: Not implemented
    */
#warning Missing Logic

  /* Enable one-shot/periodic interrupts?  Enable interrupts only if an
   * interrupt handler was provided.
   */

  if (timer->handler)
    {
      /* Select the interrupt mask that will enable the timer interrupt.
       * Any non-zero value of imr indicates that interrupts are expected.
       */

      priv->imr = TIMER_INT_TATO;

      /* Clearing the TACINTD bit allows the time-out interrupt to be
       * generated as normal
       */
      /* REVISIT: When will interrupts be enabled? */

      regval &= ~TIMER_TnMR_TnCINTD;
    }

  /* Enable count down? */

  if (timer->countup)
    {
      regval |= TIMER_TnMR_TnCDIR_UP;
    }

  tiva_putreg(priv, TIVA_TIMER_TAMR_OFFSET, regval);

  /* In addition, if using CCP pins, the TCACT field can be programmed to
   * configure the compare action.
   */
#warning Missing Logic

  /* 5. Load the start value into the GPTM Timer n Interval Load Register
   *    (GPTMTAILR).
   *
   *   When a GPTM is configured to one of the 32-bit modes, GPTMTAILR
   *   appears as a 32-bit register; the upper 16-bits correspond to bits
   *   15:0 of the GPTM Timer B Interval Load (GPTMTBILR) register.
   *   Writes to GPTMTBILR are ignored.
   */

  regval = timer->u.periodic.interval;
  tiva_putreg(priv, TIVA_TIMER_TAILR_OFFSET, regval);

  /* Preload the timer counter register by setting the timer value register.
   * The timer value will be copied to the timer counter register on the
   * next clock cycle.
   */

  if (timer->countup)
    {
      /* Count up from zero */

      tiva_putreg(priv, TIVA_TIMER_TAV_OFFSET, 0);
    }
  else
    {
      /* Count down from the timer reload value */

      tiva_putreg(priv, TIVA_TIMER_TAV_OFFSET, regval);
    }

  /* 6. If interrupts are required, set the appropriate bits in the GPTM
   *    Interrupt Mask Register (GPTMIMR).
   *
   *    NOTE: Interrupts are still disabled at the NVIC.  Interrupts will
   *    be enabled at the NVIC after the timer is started.
   */

  tiva_putreg(priv, TIVA_TIMER_IMR_OFFSET, priv->imr);

  /* 7. Set the TAEN bit in the GPTMCTL register to enable the timer and
   *    start counting.
   * 8. Poll the GPTMRIS register or wait for the interrupt to be generated
   *    (if enabled). In both cases, the status flags are cleared by writing
   *    a 1 to the appropriate bit of the GPTM Interrupt Clear Register
   *   (GPTMICR).
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_oneshot_periodic_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in one-short or periodic mode
 *
 ****************************************************************************/

static int tiva_oneshot_periodic_mode16(struct tiva_gptmstate_s *priv,
                                       const struct tiva_timer16config_s *timer,
                                       int tmndx)
{
  unsigned int regoffset;
  uint32_t regval;

  /* The GPTM is configured for One-Shot and Periodic modes by the following
   * sequence:
   *
   * 1. Ensure the timer is disabled (the TnEN bit in the GPTMCTL register
   *    is cleared) before making any changes.
   *
   *    NOTE: The TnEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
   *    operation.
   *
   *    NOTE: 16-bit mode of operation was already selected in
   *    tiva_gptm_configure() before this function was called.
   *
   * 3. Configure the TnMR field in the GPTM Timer n Mode Register
   *   (GPTMTnMR):
   *    a. Write a value of 0x1 for One-Shot mode.
   *    b. Write a value of 0x2 for Periodic mode.
   */

  regoffset = tmndx ? TIVA_TIMER_TBMR_OFFSET : TIVA_TIMER_TAMR_OFFSET;
  regval    = tiva_getreg(priv, regoffset);
  regval   &= ~TIMER_TnMR_TnMR_MASK;

  if (timer->mode == TIMER16_MODE_ONESHOT)
    {
      regval |= TIMER_TnMR_TnMR_ONESHOT;
    }
  else /* if (timer->mode == TIMER16_MODE_PERIODIC) */
    {
      regval |= TIMER_TnMR_TnMR_PERIODIC;
    }

  tiva_putreg(priv, regoffset, regval);

  /* 4. Optionally configure the TnSNAPS, TnWOT, TnMINTD, and TnCDIR bits in
   *    the GPTMTnMR register to select whether to capture the value of the
   *    free-running timer at time-out, use an external trigger to start
   *    counting, configure an additional trigger or interrupt, and count up
   *    or down. In addition, if using CCP pins, the TCACT field can be
   *    programmed to configure the compare action.
   *
   *    TnSNAPS - GPTM Timer A/B Snap-Shot Mode
   *              0: Snap-shot mode is disabled (default)
   *              1: If the 16-bit timer is configured in the periodic mode,
   *                 the actual free-running, capture or snapshot value of
   *                 the timer is loaded at the time-out event/capture or
   *                 snapshot event into the GPTM Timer A/B (GPTMTnR)
   *                 register. If the timer prescaler is used, the prescaler
   *                 snapshot is loaded into the GPTM Timer A/B (GPTMTnPR).
   *    TnWOT   - GPTM Timer A/B Wait-on-Trigger
   *              0: The 16-bit begins counting as soon as it is enabled (default).
   *              1: If the 16-bit timer is enabled, it does not begin counting
   *                 until it receives a trigger from the timer in the
   *                 previous position in the daisy chain.
   *    TnINTD  - One-shot/Periodic Interrupt Disable
   *              0: Time-out interrupt functions as normal.
   *              1: Time-out interrupt are disabled (default).
   *    TnCDIR  - GPTM Timer A/B Count Direction
   *              0: The timer counts down (default).
   *              1: The timer counts up. When counting up, the timer
   *                 starts from a value of 0.
   */

  /* Setup defaults */

  regval &= (TIMER_TnMR_TnCDIR | TIMER_TnMR_TnWOT | TIMER_TnMR_TnCDIR);
  regval |= TIMER_TnMR_TnCINTD;

  /* Enable snapshot mode?
   *
   *   In periodic, snap-shot mode (TnMR field is 0x2 and the TnSNAPS bit is
   *   set in the GPTMTnMR register), the value of the timer at the time-out
   *   event is loaded into the GPTMTnR register and the value of the
   *   prescaler is loaded into the GPTMTnPS register. The free-running
   *   counter value is shown in the GPTMTnV register. In this manner,
   *   software can determine the time elapsed from the interrupt assertion
   *   to the ISR entry by examining the snapshot values and the current value
   *   of the free-running timer. Snapshot mode is not available when the
   *   timer is configured in one-shot mode.
   *
   * TODO: Not implemented
   */
#warning Missing Logic

  /* Enable wait-on-trigger?
   *
   * TODO: Not implemented
   */
#warning Missing Logic

  /* Enable one-shot/periodic interrupts?  Enable interrupts only if an
   * interrupt handler was provided.
   */

  if (timer->handler)
    {
      /* Select the interrupt mask that will enable the timer interrupt.
       * Any non-zero value of 'imr' indicates that interrupts are expected.
       */

      priv->imr |= tmndx ? TIMER_INT_TBTO : TIMER_INT_TATO;

      /* Clearing the TnCINTD bit allows the time-out interrupt to be
       * generated as normal
       */
      /* REVISIT: When will interrupts be enabled? */

      regval &= ~TIMER_TnMR_TnCINTD;
    }

  /* Enable count down? */

  if (timer->countup)
    {
      regval |= TIMER_TnMR_TnCDIR_UP;
    }

  tiva_putreg(priv, regoffset, regval);

  /* In addition, if using CCP pins, the TCACT field can be programmed to
   * configure the compare action.
   */
#warning Missing Logic

  /* Set the input clock pre-scaler value */

  regoffset = tmndx ? TIVA_TIMER_TBPR_OFFSET : TIVA_TIMER_TAPR_OFFSET;
  tiva_putreg(priv, regoffset, (uint32_t)timer->u.periodic.prescaler);

  /* 5. Load the start value into the GPTM Timer n Interval Load Register
   *    (GPTMTnILR).
   */

  regval    = (uint32_t)timer->u.periodic.interval;
  regoffset = tmndx ? TIVA_TIMER_TBILR_OFFSET : TIVA_TIMER_TAILR_OFFSET;
  tiva_putreg(priv, regoffset, regval);

  /* Preload the timer counter register by setting the timer value register.
   * The timer value will be copied to the timer counter register on the
   * next clock cycle.
   */

  regoffset = tmndx ? TIVA_TIMER_TBV_OFFSET : TIVA_TIMER_TAV_OFFSET;
  if (timer->countup)
    {
      /* Count up from zero */

      tiva_putreg(priv, regoffset, 0);
    }
  else
    {
      /* Count down from the timer reload value */

      tiva_putreg(priv, regoffset, regval);
    }

  /* 6. If interrupts are required, set the appropriate bits in the GPTM
   *    Interrupt Mask Register (GPTMIMR).
   *
   *    NOTE: Interrupts are still disabled at the NVIC.  Interrupts will
   *    be enabled at the NVIC after the timer is started.
   */

  tiva_putreg(priv, TIVA_TIMER_IMR_OFFSET, priv->imr);

  /* 7. Set the TnEN bit in the GPTMCTL register to enable the timer and
   *    start counting.
   * 8. Poll the GPTMRIS register or wait for the interrupt to be generated
   *    (if enabled). In both cases, the status flags are cleared by writing
   *    a 1 to the appropriate bit of the GPTM Interrupt Clear Register
   *   (GPTMICR).
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_rtc_mode32
 *
 * Description:
 *   Configure a 32-bit timer to operate in RTC mode
 *
 ****************************************************************************/

static int tiva_rtc_mode32(struct tiva_gptmstate_s *priv,
                           const struct tiva_timer32config_s *timer)
{
  /* To use the RTC mode, the timer must have a 32.768-KHz input signal on
   * an even CCP input. To enable the RTC feature, follow these steps:
   *
   * 1. Ensure the timer is disabled (the TAEN bit is cleared) before making
   *    any changes.
   *
   *    NOTE: The TAEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. If the timer has been operating in a different mode prior to this,
   *    clear any residual set bits in the GPTM Timer n Mode (GPTMTnMR)
   *    register before reconfiguring.
   *
   *    NOTE: The TAMR and TBMR registers were cleared when the timer
   *    was reset prior to calling this function.
   *
   * 3. Write the GPTM Configuration Register (GPTMCFG) with a to select
   *    the 32-bit RTC mode.
   */

   tiva_putreg(priv, TIVA_TIMER_CFG_OFFSET, TIMER_CFG_CFG_RTC);

  /* 4. Write the match value to the GPTM Timer n Match Register
   *    (GPTMTnMATCHR).
   */

  /* 5. Set/clear the RTCEN and TnSTALL bit in the GPTM Control Register
   *    (GPTMCTL) as needed.
   */

  /* 6. If interrupts are required, set the RTCIM bit in the GPTM Interrupt
   *    Mask Register (GPTMIMR).
   */

  /* 7. Set the TAEN bit in the GPTMCTL register to enable the timer and
   *    start counting.
   *
   * When the timer count equals the value in the GPTMTnMATCHR register,
   * the GPTM asserts the RTCRIS bit in the GPTMRIS register and continues
   * counting until Timer A is disabled or a hardware reset. The interrupt
   * is cleared by writing the RTCCINT bit in the GPTMICR register. Note
   * that if the GPTMTnILR register is loaded with a new value, the timer
   * begins counting at this new value and continues until it reaches
   * 0xFFFF.FFFF, at which point it rolls over.
   *
   * NOTE: The timer will not be enabled until tiva_gptm_enable() is called.
   */
#warning Missing Logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_input_edgecount_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in Input Edge-Count mode
 *
 ****************************************************************************/

static int tiva_input_edgecount_mode16(struct tiva_gptmstate_s *priv,
                                       const struct tiva_timer16config_s *timer,
                                       int tmndx)
{
  /* A timer is configured to Input Edge-Count mode by the following sequence:
   *
   * 1. Ensure the timer is disabled (the TnEN bit is cleared) before making
   *    any changes.
   *
   *    NOTE: The TnEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
   *    operation.
   *
   *    NOTE: 16-bit mode of operation was already selected in
   *    tiva_gptm_configure() before this function was called.
   *
   * 3. In the GPTM Timer Mode (GPTMTnMR) register, write the TnCMR field to
   *    0x0 and the TnMR field to 0x3.
   */

  /* 4. Configure the type of event(s) that the timer captures by writing
   *    the TnEVENT field of the GPTM Control (GPTMCTL) register.
   */

  /* 5. Program registers according to count direction:
   *   - In down-count mode, the GPTMTnMATCHR and GPTMTnPMR registers are
   *    configured so that the difference between the value in the GPTMTnILR
   *    and GPTMTnPR registers and the GPTMTnMATCHR and GPTMTnPMR registers
   *    equals the number of edge events that must be counted.
   *   - In up-count mode, the timer counts from 0x0 to the value in the
   *    GPTMTnMATCHR and GPTMTnPMR registers. Note that when executing an
   *    up-count, the value of the GPTMTnPR and GPTMTnILR must be greater
   *    than the value of GPTMTnPMR and GPTMTnMATCHR.
   */

  /* 6. If interrupts are required, set the CnMIM bit in the GPTM Interrupt
   *    Mask (GPTMIMR) register.
   */
#warning Missing Logic

  /* 7. Set the TnEN bit in the GPTMCTL register to enable the timer and
   *     begin waiting for edge events.
   * 8. Poll the CnMRIS bit in the GPTMRIS register or wait for the
   *    interrupt to be generated (if enabled). In both cases, the status
   *    flags are cleared by writing a 1 to the CnMCINT bit of the GPTM
   *    Interrupt Clear (GPTMICR) register.
   *
   * When counting down in Input Edge-Count Mode, the timer stops after the
   * programmed number of edge events has been detected. To re-enable the
   * timer, ensure that the TnEN bit is cleared and repeat steps 4 through 8.
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_input_time_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in Input Time mode
 *
 ****************************************************************************/

static int tiva_input_time_mode16(struct tiva_gptmstate_s *priv,
                                  const struct tiva_timer16config_s *timer,
                                  int tmndx)
{
  /* A timer is configured to Input Edge Time mode by the following sequence:
   *
   * 1. Ensure the timer is disabled (the TnEN bit is cleared) before making
   *    any changes.
   *
   *    NOTE: The TnEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
   *    operation.
   *
   *    NOTE: 16-bit mode of operation was already selected in
   *    tiva_gptm_configure() before this function was called.
   *
   * 3. In the GPTM Timer Mode (GPTMTnMR) register, write the TnCMR field to
   *    0x1 and the TnMR field to 0x3 and select a count direction by
   *    programming the TnCDIR bit.
   */

  /* 4. Configure the type of event that the timer captures by writing the
   *    TnEVENT field of the GPTM Control (GPTMCTL) register.
   */

  /* 5. If a prescaler is to be used, write the prescale value to the GPTM
   *    Timer n Prescale Register (GPTMTnPR).
   */

  /* 6. Load the timer start value into the GPTM Timer n Interval Load
   *    (GPTMTnILR) register.
   */

  /* 7. If interrupts are required, set the CnEIM bit in the GPTM Interrupt
   *    Mask (GPTMIMR) register.
   */
#warning Missing Logic

  /* 8. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable the
   *    timer and start counting.
   * 9. Poll the CnERIS bit in the GPTMRIS register or wait for the interrupt
   *    to be generated (if enabled). In both cases, the status flags are
   *    cleared by writing a 1 to the CnECINT bit of the GPTM Interrupt
   *    Clear (GPTMICR) register. The time at which the event happened can
   *    be obtained by reading the GPTM Timer n (GPTMTnR) register.
   *
   * In Input Edge Timing mode, the timer continues running after an edge
   * event has been detected, but the timer interval can be changed at any
   * time by writing the GPTMTnILR register and clearing the TnILD bit in
   * the GPTMTnMR register. The change takes effect at the next cycle after
   * the write.
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_pwm_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in PWM mode
 *
 ****************************************************************************/

static int tiva_pwm_mode16(struct tiva_gptmstate_s *priv,
                           const struct tiva_timer16config_s *timer,
                           int tmndx)
{
  /* A timer is configured to PWM mode using the following sequence:
   *
   * 1. Ensure the timer is disabled (the TnEN bit is cleared) before making
   *    any changes.
   *
   *    NOTE: The TnEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
   *    operation.
   *
   *    NOTE: 16-bit mode of operation was already selected in
   *    tiva_gptm_configure() before this function was called.
   *
   * 3. In the GPTM Timer Mode (GPTMTnMR) register, set the TnAMS bit to
   *    0x1, the TnCMR bit to 0x0, and the TnMR field to 0x2.
   */

  /* 4. Configure the output state of the PWM signal (whether or not it is
   *    inverted) in the TnPWML field of the GPTM Control (GPTMCTL) register.
   */

  /* 5. If a prescaler is to be used, write the prescale value to the GPTM
   *    Timer n Prescale Register (GPTMTnPR).
   */

  /* 6. If PWM interrupts are used, configure the interrupt condition in the
   *    TnEVENT field in the GPTMCTL register and enable the interrupts by
   *    setting the TnPWMIE bit in the GPTMTnMR register. Note that edge
   *    detect interrupt behavior is reversed when the PWM output is
   *    inverted.
   */

  /* 7. Load the timer start value into the GPTM Timer n Interval Load
   *    (GPTMTnILR) register.
   */

  /* 8. Load the GPTM Timer n Match (GPTMTnMATCHR) register with the match
   *    value.
   */
#warning Missing Logic

  /* 9. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable
   *    the timer and begin generation of the output PWM signal.
   *
   * In PWM Time mode, the timer continues running after the PWM signal has
   * been generated. The PWM period can be adjusted at any time by writing
   * the GPTMTnILR register, and the change takes effect at the next cycle
   * after the write.
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_timer16_configure
 *
 * Description:
 *   Configure the 32-bit timer to operate in the provided mode.
 *
 ****************************************************************************/

static int tiva_timer32_configure(struct tiva_gptmstate_s *priv,
                                  const struct tiva_timer32config_s *timer)
{
  switch (priv->config->mode)
    {
    case TIMER32_MODE_ONESHOT:       /* 32-bit programmable one-shot timer */
    case TIMER32_MODE_PERIODIC:      /* 32-bit programmable periodic timer */
      return tiva_oneshot_periodic_mode32(priv, timer);

    case TIMER32_MODE_RTC:           /* 32-bit RTC with external 32.768-KHz
                                      * input */
      return tiva_rtc_mode32(priv, timer);

    default:
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: tiva_timer16_configure
 *
 * Description:
 *   Configure 16-bit timer A or B to operate in the provided mode.
 *
 ****************************************************************************/

static int tiva_timer16_configure(struct tiva_gptmstate_s *priv,
                                  const struct tiva_timer16config_s *timer,
                                  int tmndx)
{
  /* Configure the timer per the selected mode */

  switch (timer->mode)
    {
    case TIMER16_MODE_NONE:
      return OK;

    case TIMER16_MODE_ONESHOT:       /* 16-bit programmable one-shot timer */
    case TIMER16_MODE_PERIODIC:      /* 16-bit programmable periodic timer */
      return tiva_oneshot_periodic_mode16(priv, timer, tmndx);

    case TIMER16_MODE_COUNT_CAPTURE: /* 16-bit input-edge count-capture
                                      * mode w/8-bit prescaler */
      return tiva_input_edgecount_mode16(priv, timer, tmndx);

    case TIMER16_MODE_TIME_CAPTURE:  /* 16-bit input-edge time-capture
                                      * mode w/8-bit prescaler */
      return tiva_input_time_mode16(priv, timer, tmndx);

    case TIMER16_MODE_PWM:           /* 16-bit PWM output mode w/8-bit
                                      * prescaler */
      return tiva_pwm_mode16(priv, timer, tmndx);

    default:
      return -EINVAL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_gptm_configure
 *
 * Description:
 *   Configure a general purpose timer module to operate in the provided
 *   modes.
 *
 * Input Parameters:
 *   gptm - Describes the configure of the GPTM timer resources
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that can be used with the
 *   other timer interfaces.  NULL is returned on any failure to initialize
 *   the timer.
 *
 ****************************************************************************/

TIMER_HANDLE tiva_gptm_configure(const struct tiva_gptmconfig_s *config)
{
  static const struct tiva_gptmattr_s *attr;
  static struct tiva_gptmstate_s *priv;
  uint32_t regval;
  int ret;

  DEBUGASSERT(config);

  /* Select the GPTM module. */

  switch (config->gptm)
    {
#ifdef CONFIG_TIVA_TIMER0
    case 0:
      /* Enable GPTM0 clocking and power */

      
      attr = &g_gptm0_attr;
      priv = &g_gptm0_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER1
    case 1:
      attr = &g_gptm1_attr;
      priv = &g_gptm1_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER2
    case 2:
      attr = &g_gptm2_attr;
      priv = &g_gptm2_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER3
    case 3:
      attr = &g_gptm3_attr;
      priv = &g_gptm3_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER4
    case 4:
      attr = &g_gptm4_attr;
      priv = &g_gptm4_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER5
    case 5:
      attr = &g_gptm5_attr;
      priv = &g_gptm5_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER6
    case 6:
      attr = &g_gptm6_attr;
      priv = &g_gptm6_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER7
    case 7:
      attr = &g_gptm7_attr;
      priv = &g_gptm7_state;
      break;
#endif

    default:
      return (TIMER_HANDLE)NULL;
    }

  /* Initialize the state structure */

  memset(priv, 0, sizeof(struct tiva_gptmstate_s));
  priv->attr   = attr;
  priv->config = config;

  /* Disable and detach all interrupt handlers */

  up_disable_irq(attr->irq[TIMER16A]);
  up_disable_irq(attr->irq[TIMER16B]);

  (void)irq_detach(attr->irq[TIMER16A]);
  (void)irq_detach(attr->irq[TIMER16B]);

  /* Enable power and clocking to the GPTM module
   *
   * - Enable Power (TM4C129 family only):  Applies power (only) to the GPTM
   *   module.  This is not an essential step since enabling clocking
   *   will also apply power.  The only significance is that the GPTM state
   *   will be retained if the GPTM clocking is subsequently disabled.
   * - Enable Clocking (All families):  Applies both power and clocking to
   *   the GPTM module, bringing it a fully functional state.
   */

  tiva_gptm_enableclk(config->gptm);
  tiva_gptm_enablepwr(config->gptm);

  /* Wait for the gptm to become ready before modifying its registers */

  while (!tiva_gpio_periphrdy(config->gptm));

  /* Reset the time to be certain that it is in the disabled state */

  regval  = tiva_getreg(priv, TIVA_SYSCON_SRTIMER);
  regval |= SYSCON_SRTIMER(config->gptm);
  tiva_putreg(priv, TIVA_SYSCON_SRTIMER, regval);

  regval &= ~SYSCON_SRTIMER(config->gptm);
  tiva_putreg(priv, TIVA_SYSCON_SRTIMER, regval);

  /* Wait for the reset to complete */

  while (!tiva_emac_periphrdy());
  up_udelay(250);

  /* Select the alternate timer clock source is so requested.  The general
   * purpose timer has the capability of being clocked by either the system
   * clock or an alternate clock source. By setting the ALTCLK bit in the
   * GPTM Clock Configuration (GPTMCC) register, software can selects an
   * alternate clock source as programmed in the Alternate Clock
   * Configuration (ALTCLKCFG) register in the System Control Module. The
   * alternate clock source options available are PIOSC, RTCOSC and LFIOSC.
   *
   * NOTE: The actual alternate clock source selection is a global property
   * and cannot be configure on a timer-by-timer basis here.  That selection
   * must be done by common logic earlier in the initialization sequence.
   *
   * NOTE: Both the frequency of the SysClk (SYSCLK_FREQUENCY) and of the
   * alternate clock (ALTCLK_FREQUENCY) must be provided in the board.h
   * header file.
   */

  if (config->alternate)
    {
      /* Enable the alternate clock source */

      regval  = tiva_getreg(priv, TIVA_TIMER_CC_OFFSET);
      regval |= TIMER_CC_ALTCLK;
      tiva_putreg(priv, TIVA_TIMER_CC_OFFSET, regval);

      /* Remember the frequency of the input clock */

      priv->clkin = ALTCLK_FREQUENCY;
    }
  else
    {
      /* Remember the frequency of the input clock */

      priv->clkin = SYSCLK_FREQUENCY;
    }

  /* Then [re-]configure the timer into the new configuration */

  if (config->mode != TIMER16_MODE)
    {
      const struct tiva_gptm32config_s *config32 =
        (const struct tiva_gptm32config_s *)config;

      /* Attach the 32-bit timer interrupt handler (but do not yet enable
       * the interrupt).
       */

      ret = irq_attach(attr->irq[TIMER32], attr->handler32);
      if (ret == OK)
        {
          /* Configure the 32-bit timer */

          ret = tiva_timer32_configure(priv, &config32->config);
        }
    }
  else
    {
      const struct tiva_gptm16config_s *config16 =
        (const struct tiva_gptm16config_s *)config;

      /* Attach the 16-bit timer interrupt handlers (but do not yet enable
       * the interrupts).
       */

      ret = irq_attach(attr->irq[TIMER16A], attr->handler16[TIMER16A]);
      if (ret == OK)
        {
          ret = irq_attach(attr->irq[TIMER16B], attr->handler16[TIMER16B]);
        }

      if (ret == OK)
        {
          /* Write the GPTM Configuration Register (GPTMCFG) to select 16-
           * bit operation.
           */

          tiva_putreg(priv, TIVA_TIMER_CFG_OFFSET, TIMER_CFG_CFG_16);

          /* Configure 16-bit timer A */

          ret = tiva_timer16_configure(priv, &config16->config[TIMER16A],
                                       TIMER16A);
        }

      /* Configure 16-bit timer B */

      if (ret == OK)
        {
          ret = tiva_timer16_configure(priv, &config16->config[TIMER16B],
                                       TIMER16B);
        }
    }

  /* Return the timer handler if successfully configured */

  return ret < 0 ? (TIMER_HANDLE)NULL : (TIMER_HANDLE)priv;
}

/****************************************************************************
 * Name: tiva_gptm_putreg
 *
 * Description:
 *   This function permits setting of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   offset - The offset to the timer register to be written
 *   value  - The value to write to the timer register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_gptm_putreg(TIMER_HANDLE handle, unsigned int offset, uint32_t value)
{
  DEBUGASSERT(handle);
  tiva_putreg((struct tiva_gptmstate_s *)handle, offset, value);
}

/****************************************************************************
 * Name: tiva_gptm_getreg
 *
 * Description:
 *   This function permits reading of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   offset - The offset to the timer register to be written
 *
 * Returned Value:
 *   The 32-bit value read at the provided offset into the timer register base
 *   address.
 *
 ****************************************************************************/

uint32_t tiva_gptm_getreg(TIMER_HANDLE handle, unsigned int offset)
{
  DEBUGASSERT(handle);
  return tiva_getreg((struct tiva_gptmstate_s *)handle, offset);
}

/****************************************************************************
 * Name: tiva_gptm_modifyreg
 *
 * Description:
 *   This function permits atomic of any timer register by its offset into
 *   the timer block.  Its primary purpose is to support inline functions
 *   defined in this header file.
 *
 * Input Parameters:
 *   handle  - The handle value returned  by tiva_gptm_configure()
 *   offset  - The offset to the timer register to be written
 *   clrbits - The collection of bits to be cleared in the register
 *   setbits - The collection of bits to be set in the register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_gptm_modifyreg(TIMER_HANDLE handle, unsigned int offset,
                         uint32_t clrbits, uint32_t setbits)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;

  DEBUGASSERT(priv && priv->attr);
  tiva_modifyreg(priv, offset, clrbits, setbits);
}

/****************************************************************************
 * Name: tiva_timer32_start
 *
 * Description:
 *   After tiva_gptm_configure() has been called to configure a 32-bit timer,
 *   this function must be called to start the timer(s).
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_timer32_start(TIMER_HANDLE handle)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;

  DEBUGASSERT(priv && priv->attr);

  /* Set the TAEN bit in the GPTMCTL register to enable the 32-bit timer and
   * start counting
   */

  tiva_modifyreg(priv, TIVA_TIMER_CTL_OFFSET, 0, TIMER_CTL_TAEN);

  /* Enable interrupts at the NVIC if interrupts are expected */

  if (priv->imr)
    {
      up_enable_irq(priv->attr->irq[TIMER32]);
    }
}

/****************************************************************************
 * Name: tiva_timer16_start
 *
 * Description:
 *   After tiva_gptm_configure() has been called to configure 16-bit timer(s),
 *   this function must be called to start one 16-bit timer.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_timer16_start(TIMER_HANDLE handle, int tmndx)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  uint32_t setbits;
  uint32_t intmask;

  DEBUGASSERT(priv && priv->attr && (unsigned)tmndx < 2);

  /* Set the TnEN bit in the GPTMCTL register to enable the 16-bit timer and
   * start counting
   */

  setbits = tmndx ? TIMER_CTL_TBEN : TIMER_CTL_TAEN;
  tiva_modifyreg(priv, TIVA_TIMER_CTL_OFFSET, 0, setbits);

  /* Enable interrupts at the NVIC if interrupts are expected */

  intmask = tmndx ? TIMERB_INTS : TIMERA_INTS;
  if ((priv->imr & intmask) != 0)
    {
      up_enable_irq(priv->attr->irq[tmndx]);
    }
}

/****************************************************************************
 * Name: tiva_timer32_stop
 *
 * Description:
 *   After tiva_timer32_start() has been called to start a 32-bit timer,
 *   this function may be called to stop the timer.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_timer32_stop(TIMER_HANDLE handle)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;

  DEBUGASSERT(priv && priv->attr);

  /* Disable interrupts at the NVIC */

  up_disable_irq(priv->attr->irq[TIMER32]);

  /* Clear the TAEN bit in the GPTMCTL register to disable the 16-bit timer */

  tiva_modifyreg(priv, TIVA_TIMER_CTL_OFFSET, TIMER_CTL_TAEN, 0);
}

/****************************************************************************
 * Name: tiva_timer16_stop
 *
 * Description:
 *   After tiva_timer32_start() has been called to start a 16-bit timer,
 *   this function may be called to stop the timer.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_timer16_stop(TIMER_HANDLE handle, int tmndx)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  uint32_t clrbits;

  DEBUGASSERT(priv && priv->attr && (unsigned)tmndx < 2);

  /* Disable interrupts at the NVIC */

  up_disable_irq(priv->attr->irq[tmndx]);

  /* Clear the TnEN bit in the GPTMCTL register to disable the 16-bit timer */

  clrbits = tmndx ? TIMER_CTL_TBEN : TIMER_CTL_TAEN;
  tiva_gptm_modifyreg(handle, TIVA_TIMER_CTL_OFFSET, clrbits, 0);
}

/****************************************************************************
 * Name: tiva_timer32_relmatch
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   match value of a 32-bit timer.  This function sets the match register
 *   to the current timer counter register value PLUS the relative value
 *   provided.  The relative value then is some the offset to some timer
 *   counter value in the future.
 *
 *   If an interrupt handler is provided, then the match interrupt will also
 *   be enabled.  A single match interrupt will be generated; further match
 *   interrupts will be disabled.
 *
 *   NOTE: Use of this function is only meaningful for a free-runnning,
 *   periodic timer.
 *
 *   WARNING: For free-running timers, the relative match value should be
 *   sufficiently far in the future to avoid race conditions.
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   relmatch - The value to write to the timer match register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_timer32_relmatch(TIMER_HANDLE handle, uint32_t relmatch)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_timer32config_s *config;
  uintptr_t base;
  irqstate_t flags;
  uint32_t counter;
  uint32_t match;

  DEBUGASSERT(priv && priv->attr && priv->config &&
              priv->config->mode != TIMER16_MODE);

  /* Update the saved IMR if an interrupt will be needed */

  config = (const struct tiva_timer32config_s *)priv->config;
  if (config->handler)
    {
      /* Enable the match interrupt */

      priv->imr |= TIMER_INT_TAM;
    }

  /* This must be done without interrupt or context switches to minimize
   * race conditions with the free-running timer.  Note that we also
   * by-pass the normal register accesses to keep the latency to a
   * minimum.
   */

  base  = priv->attr->base;
  flags = irqsave();

  /* Set the match register to the current value of the timer counter plus
   * the provided relative match value.
   *
   * NOTE that the prescale match is not used with the 32-bit timer.
   */

  counter = getreg32(base + TIVA_TIMER_TAR_OFFSET);
  match   = counter + relmatch;
  putreg32(match, base + TIVA_TIMER_TAMATCHR_OFFSET);

  /* Enable interrupts as necessary */

  putreg32(priv->imr, base + TIVA_TIMER_IMR_OFFSET);
  irqrestore(flags);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Generate low-level debug output outside of the critical section */

  lldbg("%08x->%08x\n", base + TIVA_TIMER_TAR_OFFSET, counter);
  lldbg("%08x<-%08x\n", base + TIVA_TIMER_TAMATCHR_OFFSET, match);
  lldbg("%08x<-%08x\n", base + TIVA_TIMER_IMR_OFFSET, priv->imr);
#endif
}

/****************************************************************************
 * Name: tiva_timer16_relmatch
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   match value of a 16-bit timer.  This function sets the match register
 *   to the current timer counter register value PLUS the relative value
 *   provided.  The relative value then is some the offset to some timer
 *   counter value in the future.
 *
 *   If an interrupt handler is provided, then the match interrupt will also
 *   be enabled.  A single match interrupt will be generated; further match
 *   interrupts will be disabled.
 *
 *   NOTE: Use of this function is only meaningful for a free-runnning,
 *   periodic timer.
 *
 *   NOTE: The relmatch input is a really a 24-bit value; it is the 16-bit
 *   match counter match value AND the 8-bit prescaler value.  From the
 *   callers point of view the match value is the 24-bit time to match
 *   driven at the timer input clock frequency.
 *
 *   When counting down in periodic modes, the prescaler contains the
 *   least-significant bits of the count. When counting up, the prescaler
 *   holds the most-significant bits of the count.  But the caller is
 *   protected from this complexity.
 *
 *   WARNING: For free-running timers, the relative match value should be
 *   sufficiently far in the future to avoid race conditions.
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   relmatch - The value to write to the timer match register
 *   tmndx    - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_timer16_relmatch(TIMER_HANDLE handle, uint32_t relmatch, int tmndx)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_gptm16config_s *gptm;
  const struct tiva_timer16config_s *config;
  irqstate_t flags;
  uintptr_t base;
  uintptr_t timerr;
  uintptr_t prescr;
  uintptr_t matchr;
  uintptr_t prematchr;
  uintptr_t imr;
  uint32_t timerv;
  uint32_t prescv;
  uint32_t matchv;
  uint32_t prematchv;
  uint32_t counter;
  bool countup;

  DEBUGASSERT(priv && priv->attr &&  priv->config &&
              priv->config->mode == TIMER16_MODE && (unsigned)tmndx < 2);


  /* Precalculate as much as possible before entering the critical section */

  gptm = (const struct tiva_gptm16config_s *)priv->config;
  base = priv->attr->base;

  if (tmndx)
    {
      /* Update the saved IMR if an interrupt will be needed */

      config = &gptm->config[TIMER16B];
      if (config->handler)
        {
          /* Enable the Timer B match interrupt */

          priv->imr |= TIMER_INT_TBM;
        }

      /* Get Timer B register addresses */

      timerr    = base + TIVA_TIMER_TBR_OFFSET;
      prescr    = base + TIVA_TIMER_TBPR_OFFSET;
      matchr    = base + TIVA_TIMER_TBMATCHR_OFFSET;
      prematchr = base + TIVA_TIMER_TBPMR_OFFSET;
    }
  else
    {
      /* Update the saved IMR if an interrupt will be needed */

      config = &gptm->config[TIMER16A];
      if (config->handler)
        {
          /* Enable the Timer A match interrupt */

          priv->imr |= TIMER_INT_TAM;
        }

      /* Get Timer B register addresses */

      timerr    = base + TIVA_TIMER_TAR_OFFSET;
      prescr    = base + TIVA_TIMER_TAPR_OFFSET;
      matchr    = base + TIVA_TIMER_TAMATCHR_OFFSET;
      prematchr = base + TIVA_TIMER_TAPMR_OFFSET;
    }

  imr      = base + TIVA_TIMER_IMR_OFFSET;
  countup  = config->countup;

  /* This must be done without interrupt or context switches to minimize
   * race conditions with the free-running timer.  Note that we also
   * by-pass the normal register accesses to keep the latency to a
   * minimum.
   */

  flags = irqsave();
  timerv = getreg32(timerr) & 0xffff;
  prescv = getreg32(prescr) & 0xff;

  /* Are we counting up or down? */

  if (countup)
    {
      /* When counting up in one-shot or periodic modes, the prescaler
       * acts as a timer extension and holds the most-significant bits
       * of the count
       */

      counter   = prescv << 16 | timerv;
      counter  += relmatch;
      matchv    = counter & 0xffff;
      prematchv = (counter >> 8) & 0xff;
    }
  else
    {
      /* When counting down in one-shot or periodic modes, the prescaler
       * acts as a true prescaler and contains the least-significant bits
       * of the count.
       */

      counter   = timerv << 8 | prescv;
      counter  += relmatch;
      matchv    = (counter >> 8) & 0xffff;
      prematchv = counter & 0xff;
    }

  /* Set the match and prescacle match registers */

  putreg32(matchv, matchr);
  putreg32(prematchv, prematchr);

  /* Enable interrupts as necessary */

  putreg32(priv->imr, imr);
  irqrestore(flags);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Generate low-level debug output outside of the critical section */

  lldbg("%08x->%08x\n", timerr, timerv);
  lldbg("%08x->%08x\n", prescr, prescv);
  lldbg("%08x<-%08x\n", matchr, matchv);
  lldbg("%08x<-%08x\n", prematchr, prematchv);
  lldbg("%08x<-%08x\n", imr, priv->imr);
#endif
}
