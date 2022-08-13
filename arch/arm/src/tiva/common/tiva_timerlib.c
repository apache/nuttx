/****************************************************************************
 * arch/arm/src/tiva/common/tiva_timerlib.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/tiva_sysctrl.h"
#include "hardware/tiva_timer.h"

#include "tiva_enableclks.h"
#include "tiva_enablepwr.h"
#include "tiva_periphrdy.h"
#include "tiva_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DEBUG_TIMER_INFO
#  undef CONFIG_TIVA_TIMER_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure retains the fixed, well-known attributes of a GPTM module */

struct tiva_gptmattr_s
{
  uintptr_t base;              /* Register base address */
  uint16_t irq[2];             /* Timer A/B interrupt numbers */
#ifdef CONFIG_TIVA_TIMER_32BIT
  xcpt_t handler32;            /* Handler for 32-bit timer interrupts */
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  xcpt_t handler16[2];         /* Handlers for 16-bit timer A/B interrupts */
#endif
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
static uint32_t tiva_getreg(struct tiva_gptmstate_s *priv,
                            unsigned int offset);
static void tiva_putreg(struct tiva_gptmstate_s *priv, unsigned int offset,
              uint32_t regval);

/* Interrupt handling */

#ifdef CONFIG_TIVA_TIMER_32BIT
static int  tiva_timer32_interrupt(struct tiva_gptmstate_s *priv);
#  ifdef CONFIG_TIVA_TIMER0
static int  tiva_gptm0_interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_TIVA_TIMER1
static int  tiva_gptm1_interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_TIVA_TIMER2
static int  tiva_gptm2_interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_TIVA_TIMER3
static int  tiva_gptm3_interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_TIVA_TIMER4
static int  tiva_gptm4_interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_TIVA_TIMER5
static int  tiva_gptm5_interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_TIVA_TIMER6
static int  tiva_gptm6_interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_TIVA_TIMER7
static int  tiva_gptm7_interrupt(int irq, void *context, void *arg);
#endif
#endif /* CONFIG_TIVA_TIMER_32BIT */

#ifdef CONFIG_TIVA_TIMER_16BIT
static int  tiva_timer16_interrupt(struct tiva_gptmstate_s *priv,
              int tmndx);
#ifdef CONFIG_TIVA_TIMER0
static int  tiva_timer0a_interrupt(int irq, void *context,
                                   void *arg);
static int  tiva_timer0b_interrupt(int irq, void *context,
                                   void *arg);
#endif
#ifdef CONFIG_TIVA_TIMER1
static int  tiva_timer1a_interrupt(int irq, void *context,
                                   void *arg);
static int  tiva_timer1b_interrupt(int irq, void *context,
                                   void *arg);
#endif
#ifdef CONFIG_TIVA_TIMER2
static int  tiva_timer2a_interrupt(int irq, void *context,
                                   void *arg);
static int  tiva_timer2b_interrupt(int irq, void *context,
                                   void *arg);
#endif
#ifdef CONFIG_TIVA_TIMER3
static int  tiva_timer3a_interrupt(int irq, void *context,
                                   void *arg);
static int  tiva_timer3b_interrupt(int irq, void *context,
                                   void *arg);
#endif
#ifdef CONFIG_TIVA_TIMER4
static int  tiva_timer4a_interrupt(int irq, void *context,
                                   void *arg);
static int  tiva_timer4b_interrupt(int irq, void *context,
                                   void *arg);
#endif
#ifdef CONFIG_TIVA_TIMER5
static int  tiva_timer5a_interrupt(int irq, void *context,
                                   void *arg);
static int  tiva_timer5b_interrupt(int irq, void *context,
                                   void *arg);
#endif
#ifdef CONFIG_TIVA_TIMER6
static int  tiva_timer6a_interrupt(int irq, void *context,
                                   void *arg);
static int  tiva_timer6b_interrupt(int irq, void *context,
                                   void *arg);
#endif
#ifdef CONFIG_TIVA_TIMER7
static int  tiva_timer7a_interrupt(int irq, void *context,
                                   void *arg);
static int  tiva_timer7b_interrupt(int irq, void *context,
                                   void *arg);
#endif
#endif /* CONFIG_TIVA_TIMER_16BIT */

/* Timer initialization and configuration */

#ifdef CONFIG_TIVA_TIMER32_PERIODIC
static int  tiva_oneshot_periodic_mode32(struct tiva_gptmstate_s *priv,
              const struct tiva_timer32config_s *timer);
#endif /* CONFIG_TIVA_TIMER32_PERIODIC */
#ifdef CONFIG_TIVA_TIMER16_PERIODIC
static int  tiva_oneshot_periodic_mode16(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);
#endif
#ifdef CONFIG_TIVA_TIMER32_RTC
static int  tiva_rtc_mode32(struct tiva_gptmstate_s *priv,
              const struct tiva_timer32config_s *timer);
#endif
#ifdef CONFIG_TIVA_TIMER16_EDGECOUNT
static int  tiva_input_edgecount_mode16(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);
#endif
#ifdef CONFIG_TIVA_TIMER16_TIMECAP
static int  tiva_input_time_mode16(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);
#endif
#ifdef CONFIG_TIVA_TIMER16_PWM
static uint32_t
tiva_pwm16_sel_event(struct tiva_gptmstate_s *priv,
                     const struct tiva_timer16config_s *timer,
                     int tmndx);
#endif
#ifdef CONFIG_TIVA_TIMER16_PWM
static int  tiva_pwm_mode16(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);

#endif
#ifdef CONFIG_TIVA_TIMER_32BIT
static int tiva_timer32_configure(struct tiva_gptmstate_s *priv,
              const struct tiva_timer32config_s *timer);
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
static int  tiva_timer16_configure(struct tiva_gptmstate_s *priv,
              const struct tiva_timer16config_s *timer, int tmndx);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER0
static const struct tiva_gptmattr_s g_gptm0_attr =
{
  .base      = TIVA_TIMER0_BASE,
  .irq       =
    {
      TIVA_IRQ_TIMER0A, TIVA_IRQ_TIMER0B
    },
#ifdef CONFIG_TIVA_TIMER_32BIT
  .handler32 = tiva_gptm0_interrupt,
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  .handler16 =
    {
      tiva_timer0a_interrupt, tiva_timer0b_interrupt
    },
#endif
};

static struct tiva_gptmstate_s g_gptm0_state;
#endif

#ifdef CONFIG_TIVA_TIMER1
static const struct tiva_gptmattr_s g_gptm1_attr =
{
  .base      = TIVA_TIMER1_BASE,
  .irq       =
    {
      TIVA_IRQ_TIMER1A, TIVA_IRQ_TIMER1B
    },
#ifdef CONFIG_TIVA_TIMER_32BIT
  .handler32 = tiva_gptm1_interrupt,
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  .handler16 =
    {
      tiva_timer1a_interrupt, tiva_timer1b_interrupt
    },
#endif
};

static struct tiva_gptmstate_s g_gptm1_state;
#endif

#ifdef CONFIG_TIVA_TIMER2
static const struct tiva_gptmattr_s g_gptm2_attr =
{
  .base      = TIVA_TIMER2_BASE,
  .irq       =
    {
      TIVA_IRQ_TIMER2A, TIVA_IRQ_TIMER2B
    },
#ifdef CONFIG_TIVA_TIMER_32BIT
  .handler32 = tiva_gptm2_interrupt,
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  .handler16 =
    {
      tiva_timer2a_interrupt, tiva_timer2b_interrupt
    },
#endif
};

static struct tiva_gptmstate_s g_gptm2_state;
#endif

#ifdef CONFIG_TIVA_TIMER3
static const struct tiva_gptmattr_s g_gptm3_attr =
{
  .base      = TIVA_TIMER3_BASE,
  .irq       =
    {
      TIVA_IRQ_TIMER3A, TIVA_IRQ_TIMER3B
    },
#ifdef CONFIG_TIVA_TIMER_32BIT
  .handler32 = tiva_gptm3_interrupt,
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  .handler16 =
    {
       tiva_timer3a_interrupt, tiva_timer3b_interrupt
    },
#endif
};

static struct tiva_gptmstate_s g_gptm3_state;
#endif

#ifdef CONFIG_TIVA_TIMER4
static const struct tiva_gptmattr_s g_gptm4_attr =
{
  .base      = TIVA_TIMER4_BASE,
  .irq       =
    {
      TIVA_IRQ_TIMER4A, TIVA_IRQ_TIMER4B
    },
#ifdef CONFIG_TIVA_TIMER_32BIT
  .handler32 = tiva_gptm4_interrupt,
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  .handler16 =
    {
       tiva_timer4a_interrupt, tiva_timer4b_interrupt
    },
#endif
};

static struct tiva_gptmstate_s g_gptm4_state;
#endif

#ifdef CONFIG_TIVA_TIMER5
static const struct tiva_gptmattr_s g_gptm5_attr =
{
  .base      = TIVA_TIMER5_BASE,
  .irq       =
    {
      TIVA_IRQ_TIMER5A, TIVA_IRQ_TIMER5B
    },
#ifdef CONFIG_TIVA_TIMER_32BIT
  .handler32 = tiva_gptm5_interrupt,
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  .handler16 =
    {
      tiva_timer5a_interrupt, tiva_timer5b_interrupt
    },
#endif
};

static struct tiva_gptmstate_s g_gptm5_state;
#endif

#ifdef CONFIG_TIVA_TIMER6
static const struct tiva_gptmattr_s g_gptm6_attr =
{
  .base      = TIVA_TIMER6_BASE,
  .irq       =
    {
      TIVA_IRQ_TIMER6A, TIVA_IRQ_TIMER6B
    },
#ifdef CONFIG_TIVA_TIMER_32BIT
  .handler32 = tiva_gptm6_interrupt,
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  .handler16 =
    {
       tiva_timer6a_interrupt, tiva_timer6b_interrupt
    },
#endif
};

static struct tiva_gptmstate_s g_gptm6_state;
#endif

#ifdef CONFIG_TIVA_TIMER7
static const struct tiva_gptmattr_s g_gptm7_attr =
{
  .base      = TIVA_TIMER7_BASE,
  .irq       =
    {
       TIVA_IRQ_TIMER7A, TIVA_IRQ_TIMER7B
    },
#ifdef CONFIG_TIVA_TIMER_32BIT
  .handler32 = tiva_gptm7_interrupt,
#endif
#ifdef CONFIG_TIVA_TIMER_16BIT
  .handler16 =
    {
       tiva_timer7a_interrupt, tiva_timer7b_interrupt
    },
#endif
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

          tmrinfo("...[Repeats %d times]...\n", priv->ntimes);
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

static uint32_t tiva_getreg(struct tiva_gptmstate_s *priv,
                            unsigned int offset)
{
  uintptr_t regaddr = priv->attr->base + offset;
  uint32_t regval =  getreg32(regaddr);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  if (tiva_timer_checkreg(priv, false, regval, regaddr))
    {
      tmrinfo("%08x->%08x\n", regaddr, regval);
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
       tmrinfo("%08x<-%08x\n", regaddr, regval);
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

static void tiva_modifyreg(struct tiva_gptmstate_s *priv,
                           unsigned int offset,
                           uint32_t clrbits, uint32_t setbits)
{
#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  irqstate_t flags;
  uint32_t regval;

  flags   = enter_critical_section();
  regval  = tiva_getreg(priv, offset);
  regval &= ~clrbits;
  regval |= setbits;
  tiva_putreg(priv, offset, regval);
  leave_critical_section(flags);
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

#ifdef CONFIG_TIVA_TIMER_32BIT
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
      /* Acknowledge (clear) the interrupt */

      tiva_putreg(priv, TIVA_TIMER_ICR_OFFSET, status);

      /* If this was a match (or RTC match) interrupt, then disable further
       * match interrupts.
       */

      if ((status & (TIMER_INT_TAM | TIMER_INT_RTC)) != 0)
        {
          status &= ~(TIMER_INT_TAM | TIMER_INT_RTC);
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
          timer32->handler((TIMER_HANDLE)priv, timer32->arg, status);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: tiva_gptmN_interrupt, N=0..7
 *
 * Description:
 *   Individual interrupt handlers for each 32-bit timer
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_32BIT
#ifdef CONFIG_TIVA_TIMER0
static int tiva_gptm0_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer32_interrupt(&g_gptm0_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER1
static int tiva_gptm1_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer32_interrupt(&g_gptm1_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER2
static int tiva_gptm2_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer32_interrupt(&g_gptm2_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER3
static int tiva_gptm3_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer32_interrupt(&g_gptm3_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER4
static int tiva_gptm4_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer32_interrupt(&g_gptm4_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER5
static int tiva_gptm5_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer32_interrupt(&g_gptm5_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER6
static int tiva_gptm6_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer32_interrupt(&g_gptm6_state);
}
#endif

#ifdef CONFIG_TIVA_TIMER7
static int tiva_gptm7_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer32_interrupt(&g_gptm7_state);
}
#endif
#endif

/****************************************************************************
 * Name: tiva_timer16_interrupt
 *
 * Description:
 *   Common interrupt handler for 16-bit timers
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
static int tiva_timer16_interrupt(struct tiva_gptmstate_s *priv, int tmndx)
{
  const struct tiva_gptm16config_s  *config16;
  const struct tiva_timer16config_s *timer16;
  uint32_t intmask;
  uint32_t status;

  DEBUGASSERT(priv && priv->attr && priv->config && (unsigned)tmndx < 2);

  /* Read the masked interrupt status,
   * masking out bits only for this timer.
   */

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
          timer16->handler((TIMER_HANDLE)priv, timer16->arg, status, tmndx);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: tiva_timerNa_interrupt, tiva_timerNb_interrupt, N=0..7
 *
 * Description:
 *   Individual interrupt handlers for each 16-bit timer
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
#ifdef CONFIG_TIVA_TIMER0
static int tiva_timer0a_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm0_state, TIMER16A);
}

static int tiva_timer0b_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm0_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER1
static int tiva_timer1a_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm1_state, TIMER16A);
}

static int tiva_timer1b_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm1_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER2
static int tiva_timer2a_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm2_state, TIMER16A);
}

static int tiva_timer2b_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm2_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER3
static int tiva_timer3a_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm3_state, TIMER16A);
}

static int tiva_timer3b_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm3_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER4
static int tiva_timer4a_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm4_state, TIMER16A);
}

static int tiva_timer4b_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm4_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER5
static int tiva_timer5a_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm5_state, TIMER16A);
}

static int tiva_timer5b_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm5_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER6
static int tiva_timer6a_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm6_state, TIMER16A);
}

static int tiva_timer6b_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm6_state, TIMER16B);
}
#endif

#ifdef CONFIG_TIVA_TIMER7
static int tiva_timer7a_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm7_state, TIMER16A);
}

static int tiva_timer7b_interrupt(int irq, void *context, void *arg)
{
  return tiva_timer16_interrupt(&g_gptm7_state, TIMER16B);
}
#endif
#endif

/****************************************************************************
 * Name: tiva_oneshot_periodic_mode32
 *
 * Description:
 *   Configure a 32-bit timer to operate in one-shot or periodic mode
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER32_PERIODIC
static int
tiva_oneshot_periodic_mode32(struct tiva_gptmstate_s *priv,
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
  regval &= ~TIMER_TNMR_TNMR_MASK;

  if (priv->config->mode == TIMER32_MODE_ONESHOT)
    {
      regval |= TIMER_TNMR_TNMR_ONESHOT;
    }
  else /* if (priv->config->mode == TIMER32_MODE_PERIODIC) */
    {
      regval |= TIMER_TNMR_TNMR_PERIODIC;
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
   *
   *    NOTE: one-shot/periodic timeout interrupts remain disabled until
   *    tiva_timer32_setinterval is called.
   */

  /* Setup defaults */

  regval &= (TIMER_TNMR_TNCDIR | TIMER_TNMR_TNWOT | TIMER_TNMR_TNCDIR);
#ifdef CONFIG_ARCH_CHIP_TM4C129
  regval |= TIMER_TNMR_TNCINTD;
#endif

  /* Enable snapshot mode?
   *
   * In periodic, snap-shot mode (TnMR field is 0x2 and the TnSNAPS bit is
   * set in the GPTMTnMR register), the value of the timer at the time-out
   * event is loaded into the GPTMTnR register and the value of the
   * prescaler is loaded into the GPTMTnPS register. The free-running
   * counter value is shown in the GPTMTnV register. In this manner,
   * software can determine the time elapsed from the interrupt assertion
   * to the ISR entry by examining the snapshot values and the current value
   * of the free-running timer. Snapshot mode is not available when the
   * timer is configured in one-shot mode.
   *
   * TODO: Not implemented
   */
#warning Missing Logic

  /* Enable wait-on-trigger?
   *
   * TODO: Not implemented
   */
#warning Missing Logic

  /* Enable count down? */

  if (TIMER_ISCOUNTUP(timer))
    {
      regval |= TIMER_TNMR_TNCDIR_UP;
    }

  tiva_putreg(priv, TIVA_TIMER_TAMR_OFFSET, regval);

  /* Enable and configure ADC trigger outputs */

  if (TIMER_ISADCTIMEOUT(timer) || TIMER_ISADCMATCH(timer))
    {
#ifdef CONFIG_ARCH_CHIP_TM4C129
      /* Enable timeout triggers now (match triggers will be
       * enabled when the first match value is set).
       */

      if (TIMER_ISADCTIMEOUT(timer))
        {
          tiva_putreg(priv, TIVA_TIMER_ADCEV_OFFSET, TIMER_ADCEV_TATOADCEN);
        }
#endif

      /* Enable ADC trigger outputs by setting the TAOTE bit in the
       * control register.
       */

      regval = tiva_getreg(priv, TIVA_TIMER_CTL_OFFSET);
      regval |= TIMER_CTL_TAOTE;
      tiva_putreg(priv, TIVA_TIMER_CTL_OFFSET, regval);
    }

  /* In addition, if using CCP pins, the TCACT field can be programmed to
   * configure the compare action.
   */
#warning Missing Logic

  /* TODO: Enable and configure uDMA trigger outputs */

  /* 5. Load the start value into the GPTM Timer n Interval Load Register
   *    (GPTMTAILR).
   *
   *    When a GPTM is configured to one of the 32-bit modes, GPTMTAILR
   *    appears as a 32-bit register; the upper 16-bits correspond to bits
   *    15:0 of the GPTM Timer B Interval Load (GPTMTBILR) register.
   *    Writes to GPTMTBILR are ignored.
   *
   *    NOTE: The default is a free-running timer.  The timer interval
   *    reload register is clear here.  It can be set to any value
   *    desired by calling tiva_timer32_setinterval().
   */

  tiva_putreg(priv, TIVA_TIMER_TAILR_OFFSET, 0);

  /* Preload the timer counter register by setting the timer value register.
   * The timer value will be copied to the timer counter register on the
   * next clock cycle.
   */

  if (TIMER_ISCOUNTUP(timer))
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
   * NOTE: This timer is not started until tiva_timer32_start() is called.
   */

  return OK;
}
#endif

/****************************************************************************
 * Name: tiva_oneshot_periodic_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in one-short or periodic mode
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PERIODIC
static int
tiva_oneshot_periodic_mode16(struct tiva_gptmstate_s *priv,
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
  regval   &= ~TIMER_TNMR_TNMR_MASK;

  if (timer->mode == TIMER16_MODE_ONESHOT)
    {
      regval |= TIMER_TNMR_TNMR_ONESHOT;
    }
  else /* if (timer->mode == TIMER16_MODE_PERIODIC) */
    {
      regval |= TIMER_TNMR_TNMR_PERIODIC;
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
   *              0: The 16-bit begins counting as soon as it is enabled
   *                 (default).
   *              1: If the 16-bit timer is enabled, it does not begin
   *                 counting until it receives a trigger from the timer in
   *                 the previous position in the daisy chain.
   *    TnINTD  - One-shot/Periodic Interrupt Disable
   *              0: Time-out interrupt functions as normal.
   *              1: Time-out interrupt are disabled (default).
   *    TnCDIR  - GPTM Timer A/B Count Direction
   *              0: The timer counts down (default).
   *              1: The timer counts up. When counting up, the timer
   *                 starts from a value of 0.
   *
   *    NOTE: one-shot/periodic timeout interrupts remain disabled until
   *    tiva_timer32_setinterval is called.
   */

  /* Setup defaults */

  regval &= (TIMER_TNMR_TNCDIR | TIMER_TNMR_TNWOT | TIMER_TNMR_TNCDIR);
#ifdef CONFIG_ARCH_CHIP_TM4C129
  regval |= TIMER_TNMR_TNCINTD;
#endif

  /* Enable snapshot mode?
   *
   *   In periodic, snap-shot mode (TnMR field is 0x2 and the TnSNAPS bit is
   *   set in the GPTMTnMR register), the value of the timer at the time-out
   *   event is loaded into the GPTMTnR register and the value of the
   *   prescaler is loaded into the GPTMTnPS register. The free-running
   *   counter value is shown in the GPTMTnV register. In this manner,
   *   software can determine the time elapsed from the interrupt assertion
   *   to the ISR entry by examining the snapshot values and the current
   *   value of the free-running timer. Snapshot mode is not available when
   *   the timer is configured in one-shot mode.
   *
   * TODO: Not implemented
   */
#warning Missing Logic

  /* Enable wait-on-trigger?
   *
   * TODO: Not implemented
   */
#warning Missing Logic

  /* Enable count down? */

  if (TIMER_ISCOUNTUP(timer))
    {
      regval |= TIMER_TNMR_TNCDIR_UP;
    }

  tiva_putreg(priv, regoffset, regval);

  /* Enable and configure ADC trigger outputs */

  if (TIMER_ISADCTIMEOUT(timer) || TIMER_ISADCMATCH(timer))
    {
      /* Enable ADC trigger outputs by setting the TnOTE bit in the
       * control register.
       */

      regval = tiva_getreg(priv, TIVA_TIMER_CTL_OFFSET);
      regval |= tmndx ? TIMER_CTL_TBOTE : TIMER_CTL_TAOTE;
      tiva_putreg(priv, TIVA_TIMER_CTL_OFFSET, regval);

#ifdef CONFIG_ARCH_CHIP_TM4C129
      /* Enable timeout triggers now (match triggers will be
       * enabled when the first match value is set).
       */

      if (TIMER_ISADCTIMEOUT(timer))
        {
          regval = tmndx ? TIMER_ADCEV_TBTOADCEN : TIMER_ADCEV_TATOADCEN;
          tiva_putreg(priv, TIVA_TIMER_ADCEV_OFFSET, regval);
        }
#endif /* CONFIG_ARCH_CHIP_TM4C129 */
    }

  /* TODO: Enable and configure uDMA trigger outputs */

  /* In addition, if using CCP pins, the TCACT field can be programmed to
   * configure the compare action.
   */
#warning Missing Logic

  /* 5. Load the start value into the GPTM Timer n Interval Load Register
   *    (GPTMTnILR).
   *
   *    NOTE: The default is a free-running timer.  The timer interval
   *    reload and prescale registers are cleared here.  They can be set to
   *    any value desired by calling tiva_timer32_setinterval().
   */

  regoffset = tmndx ? TIVA_TIMER_TBPR_OFFSET : TIVA_TIMER_TAPR_OFFSET;
  tiva_putreg(priv, regoffset, 0);

  regoffset = tmndx ? TIVA_TIMER_TBILR_OFFSET : TIVA_TIMER_TAILR_OFFSET;
  tiva_putreg(priv, regoffset, 0);

  /* Preload the timer counter register by setting the timer value register.
   * The timer value will be copied to the timer counter register on the
   * next clock cycle.
   */

  regoffset = tmndx ? TIVA_TIMER_TBV_OFFSET : TIVA_TIMER_TAV_OFFSET;
  if (TIMER_ISCOUNTUP(timer))
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
   * NOTE: This timer is not started until tiva_timer16_start() is called.
   */

  return OK;
}
#endif

/****************************************************************************
 * Name: tiva_rtc_mode32
 *
 * Description:
 *   Configure a 32-bit timer to operate in RTC mode
 *
 *   The input clock on a CCP0 input is required to be 32.768 KHz in RTC
 *   mode. The clock signal is then divided down to a 1-Hz rate and is
 *   passed along to the input of the counter.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER32_RTC
static int tiva_rtc_mode32(struct tiva_gptmstate_s *priv,
                           const struct tiva_timer32config_s *timer)
{
  uint32_t regval;

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
   *
   *    NOTE: The match register is not set until tiva_rtc_setalarm() is
   *    called.
   *
   * 5. Set/clear the RTCEN and TnSTALL bit in the GPTM Control Register
   *    (GPTMCTL) as needed.
   *
   *      RTCEN   - 1: RTC counting continues while the processor is
   *                halted by the debugger
   *      TASTALL - 1: Timer A freezes counting while the processor is
   *                halted by the debugger.
   */

  regval = tiva_getreg(priv, TIVA_TIMER_CTL_OFFSET);
#ifdef CONFIG_DEBUG_SYMBOLS
  regval |=  (TIMER_CTL_RTCEN | TIMER_CTL_TASTALL);
#else
  regval &= ~(TIMER_CTL_RTCEN | TIMER_CTL_TASTALL);
#endif
  tiva_putreg(priv, TIVA_TIMER_CTL_OFFSET, regval);

  /* 6. If interrupts are required, set the RTCIM bit in the GPTM Interrupt
   *    Mask Register (GPTMIMR).
   *
   *    NOTE: RTC interrupts are not enabled until tiva_rtc_setalarm() is
   *    called.
   *
   * 7. Set the TAEN bit in the GPTMCTL register to enable the timer and
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
   * NOTE: The RTC timer will not be enabled until tiva_gptm_enableclk() is
   * called.
   */

  return OK;
}
#endif

/****************************************************************************
 * Name: tiva_input_edgecount_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in Input Edge-Count mode
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_EDGECOUNT
static int
tiva_input_edgecount_mode16(struct tiva_gptmstate_s *priv,
                            const struct tiva_timer16config_s *timer,
                            int tmndx)
{
  /* A timer is configured to Input Edge-Count mode by the following
   * sequence:
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
   *
   *   - In down-count mode, the GPTMTnMATCHR and GPTMTnPMR registers are
   *     configured so that the difference between the value in the GPTMTnILR
   *     and GPTMTnPR registers and the GPTMTnMATCHR and GPTMTnPMR registers
   *     equals the number of edge events that must be counted.
   *
   *   - In up-count mode, the timer counts from 0x0 to the value in the
   *     GPTMTnMATCHR and GPTMTnPMR registers. Note that when executing an
   *     up-count, the value of the GPTMTnPR and GPTMTnILR must be greater
   *     than the value of GPTMTnPMR and GPTMTnMATCHR.
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
   * NOTE: This timer is not started until tiva_timer16_start() is called.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: tiva_input_time_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in Input Time mode
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_TIMECAP
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
   *
   *    REVISIT:  When the ALTCLK bit is set in the GPTMCC register to enable
   *    using the alternate clock source, the synchronization imposes
   *    restrictions on the starting count value (down-count), terminal value
   *    (up-count) and the match value. This restriction applies to all modes
   *    of operation. Each event must be spaced by 4 Timer (ALTCLK) clock
   *    periods + 2 system clock periods. If some events do not meet this
   *    requirement, then it is possible that the timer block may need to be
   *    reset for correct functionality to be restored.
   *
   *      Example: ALTCLK= TPIOSC = 62.5ns (16Mhz Trimmed)
   *      Thclk = 1us (1Mhz)
   *      4*62.5ns + 2*1us = 2.25us 2.25us/62.5ns = 36 or 0x23
   *
   *    The minimum values for the periodic or one-shot with a match
   *    interrupt enabled are: GPTMTAMATCHR = 0x23 GPTMTAILR = 0x46"
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
   * NOTE: This timer is not started until tiva_timer16_start() is called.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: tiva_pwm16_sel_event
 *
 * Description:
 *   Select which event to program into the TnEVENT field in the GPTMCTL
 *   register based on configured flags. Note that the caller must first
 *   clear the bit field (using either TIMER_CTL_TAEVENT_MASK or
 *   TIMER_CTL_TBEVENT_MASK depending on whether Timer A or Timer B is
 *   used) before writing the value returned by this function.
 *
 * Input Parameters:
 *   handle - The handle value returned by tiva_gptm_configure()
 *   timer  - The timer A or B configuration structure. This is located
 *            within the configuration passed to tiva_gptm_configure().
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   Value to be programmed into the TnEVENT field in the GPTMCTL register
 *   after clearing the bit field as described above.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PWM
static uint32_t
tiva_pwm16_sel_event(struct tiva_gptmstate_s *priv,
                     const struct tiva_timer16config_s *timer,
                     int tmndx)
{
  /* For PWM interrupt edge selection, we can interrupt on positive edge
   * (TIMER_CTL_TnEVENT_POS), negative edge (TIMER_CTL_TnEVENT_NEG), or both
   * edges (TIMER_CTL_TnEVENT_BOTH). If PWM output is inverted, then edge
   * detect interrupt behavior is reversed by the hardware. We will normalize
   * this so that higher level logic won't have to.
   */

  if (TIMER_ISPWMINTBOTH(timer))
    {
      /* When interrupting on both edges, it doesn't matter if PWM output
       * is inverted.
       */

      return tmndx ? TIMER_CTL_TBEVENT_BOTH : TIMER_CTL_TAEVENT_BOTH;
    }

  if (TIMER_ISPWMINTPOS(timer))
    {
      if (TIMER_ISPWMINVERT(timer))
        {
          return tmndx ? TIMER_CTL_TBEVENT_NEG : TIMER_CTL_TAEVENT_NEG;
        }
      else
        {
          return tmndx ? TIMER_CTL_TBEVENT_POS : TIMER_CTL_TAEVENT_POS;
        }
    }

  if (TIMER_ISPWMINTNEG(timer))
    {
      if (TIMER_ISPWMINVERT(timer))
        {
          return tmndx ? TIMER_CTL_TBEVENT_POS : TIMER_CTL_TAEVENT_POS;
        }
      else
        {
          return tmndx ? TIMER_CTL_TBEVENT_NEG : TIMER_CTL_TAEVENT_NEG;
        }
    }

  /* Not interrupting on any edge
   */

  return 0;
}
#endif

/****************************************************************************
 * Name: tiva_pwm_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in PWM mode. The timer is not
 *   started until tiva_timer16_start() is called.
 *
 * Input Parameters:
 *   handle - The handle value returned by tiva_gptm_configure()
 *   timer  - The timer A or B configuration structure. This is located
 *            within the configuration passed to tiva_gptm_configure().
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PWM
static int tiva_pwm_mode16(struct tiva_gptmstate_s *priv,
                           const struct tiva_timer16config_s *timer,
                           int tmndx)
{
  unsigned int regoffset;
  uint32_t regval;
  uint32_t clrbits;
  uint32_t setbits;

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
   *    0x1 (enables PWM), the TnCMR bit to 0x0 (edge count mode), and the
   *    TnMR field to 0x2 (periodic timer mode).
   */

  regoffset = tmndx ? TIVA_TIMER_TBMR_OFFSET : TIVA_TIMER_TAMR_OFFSET;
  clrbits = TIMER_TNMR_TNMR_MASK | TIMER_TNMR_TNCMR | TIMER_TNMR_TNAMS;
  setbits = TIMER_TNMR_TNMR_PERIODIC | TIMER_TNMR_TNCMR_EDGECOUNT |
            TIMER_TNMR_TNAMS_PWM;
  tiva_modifyreg(priv, regoffset, clrbits, setbits);

  /* 4. Configure the output state of the PWM signal (whether or not it is
   *    inverted) in the TnPWML field of the GPTM Control (GPTMCTL) register.
   */

  regval = tmndx ? TIMER_CTL_TBPWML : TIMER_CTL_TAPWML;

  if (TIMER_ISPWMINVERT(timer))
    {
      tiva_modifyreg(priv, TIVA_TIMER_CTL_OFFSET, 0, regval);
    }
  else
    {
      tiva_modifyreg(priv, TIVA_TIMER_CTL_OFFSET, regval, 0);
    }

  /* 5. If PWM interrupts are used, configure which signal edge(s) trigger
   *    the interrupt by setting the TnEVENT field in the GPTMCTL register.
   *    This can be the positive edge (TIMER_CTL_TnEVENT_POS), the negative
   *    edge (TIMER_CTL_TnEVENT_NEG), or both edges (TIMER_CTL_TnEVENT_BOTH).
   *    Note that if the PWM output is inverted (see above), then edge detect
   *    interrupt behavior is reversed. We will normalize this so that higher
   *    level logic won't have to.
   *
   *    Enable the interrupts by setting the TnPWMIE bit in the GPTMTnMR
   *    register. Note that edge detect interrupt behavior is reversed when
   *    the PWM output is inverted.
   */

  clrbits = tmndx ? TIMER_CTL_TBEVENT_MASK : TIMER_CTL_TAEVENT_MASK;
  setbits = tiva_pwm16_sel_event(priv, timer, tmndx);
  tiva_modifyreg(priv, TIVA_TIMER_CTL_OFFSET, clrbits, setbits);

  regoffset = tmndx ? TIVA_TIMER_TBMR_OFFSET : TIVA_TIMER_TAMR_OFFSET;
  tiva_modifyreg(priv, regoffset, 0, TIMER_TNMR_TNPWMIE);

  /* 6. Set PWM period: This is a 24-bit value. Put the high byte (bits 16
   *    through 23) in the prescaler register (TIVA_TIMER_TnPR_OFFSET).
   *    Put the low word (bits 0 through 15) in the interval load register
   *    (TIVA_TIMER_TnILR_OFFSET).
   *
   *    NOTE:
   *    This is done when tiva_timer16pwm_setperiodduty() is called.  That
   *    must be done by other logic, prior to starting the clock running.
   *
   *    The following note was here before implementation of this function
   *    was written:
   *
   *    REVISIT:  When the ALTCLK bit is set in the GPTMCC register to enable
   *    using the alternate clock source, the synchronization imposes
   *    restrictions on the starting count value (down-count), terminal value
   *    (up-count) and the match value. This restriction applies to all modes
   *    of operation. Each event must be spaced by 4 Timer (ALTCLK) clock
   *    periods + 2 system clock periods. If some events do not meet this
   *    requirement, then it is possible that the timer block may need to be
   *    reset for correct functionality to be restored.
   *
   *      Example: ALTCLK= TPIOSC = 62.5ns (16Mhz Trimmed)
   *      Thclk = 1us (1Mhz)
   *      4*62.5ns + 2*1us = 2.25us 2.25us/62.5ns = 36 or 0x23
   *
   *    The minimum values for the periodic or one-shot with a match
   *    interrupt enabled are: GPTMTAMATCHR = 0x23 GPTMTAILR = 0x46"
   */

  /* 7. Set PWM duty cycle: This is a 24-bit value. Put the high byte (bits
   *    16 through 23) in the prescale match register
   *    (TIVA_TIMER_TnPMR_OFFSET). Put the low word (bits 0 through 16) in
   *    the match register (TIVA_TIMER_TnMATCHR_OFFSET).
   *
   *    NOTE:
   *    This is done when tiva_timer16pwm_setperiodduty() is called. That
   *    must be done by other logic, prior to starting the clock running.
   *    Once the period and initial duty cycle are set, the duty cycle can
   *    be changed at any time by calling tiva_timer16pwm_setduty().
   */

  /* 8. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable
   *    the timer and begin generation of the output PWM signal.
   *
   *    This is done when tiva_timer16_start() is called.
   *
   *    In PWM Time mode, the timer continues running after the PWM signal
   *    has been generated. The PWM period can be adjusted at any time by
   *    writing the GPTMTnILR register, and the change takes effect at the
   *    next cycle after the write.
   */

  return OK;
}
#endif

/****************************************************************************
 * Name: tiva_timer32_configure
 *
 * Description:
 *   Configure the 32-bit timer to operate in the provided mode.
 *   The timer is not started until tiva_timer32_start() is called.
 *
 ****************************************************************************/
#ifdef CONFIG_TIVA_TIMER_32BIT
static int tiva_timer32_configure(struct tiva_gptmstate_s *priv,
                                  const struct tiva_timer32config_s *timer)
{
  switch (priv->config->mode)
    {
#ifdef CONFIG_TIVA_TIMER32_PERIODIC
    case TIMER32_MODE_ONESHOT:       /* 32-bit programmable one-shot timer */
    case TIMER32_MODE_PERIODIC:      /* 32-bit programmable periodic timer */
      return tiva_oneshot_periodic_mode32(priv, timer);
#endif /* CONFIG_TIVA_TIMER32_PERIODIC */

#ifdef CONFIG_TIVA_TIMER32_RTC
    case TIMER32_MODE_RTC:           /* 32-bit RTC with external 32.768-KHz
                                      * input */
      return tiva_rtc_mode32(priv, timer);
#endif

    default:
      return -EINVAL;
    }
}
#endif

/****************************************************************************
 * Name: tiva_timer16_configure
 *
 * Description:
 *   Configure 16-bit timer A or B to operate in the provided mode.
 *   The timer is not started until tiva_timer16_start() is called.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
static int tiva_timer16_configure(struct tiva_gptmstate_s *priv,
                                  const struct tiva_timer16config_s *timer,
                                  int tmndx)
{
  /* Configure the timer per the selected mode */

  switch (timer->mode)
    {
    case TIMER16_MODE_NONE:
      return OK;

#ifdef CONFIG_TIVA_TIMER16_PERIODIC
    case TIMER16_MODE_ONESHOT:       /* 16-bit programmable one-shot timer */
    case TIMER16_MODE_PERIODIC:      /* 16-bit programmable periodic timer */
      return tiva_oneshot_periodic_mode16(priv, timer, tmndx);
#endif

#ifdef CONFIG_TIVA_TIMER16_EDGECOUNT
    case TIMER16_MODE_COUNT_CAPTURE: /* 16-bit input-edge count-capture
                                      * mode w/8-bit prescaler */
      return tiva_input_edgecount_mode16(priv, timer, tmndx);
#endif

#ifdef CONFIG_TIVA_TIMER16_TIMECAP
    case TIMER16_MODE_TIME_CAPTURE:  /* 16-bit input-edge time-capture
                                      * mode w/8-bit prescaler */
      return tiva_input_time_mode16(priv, timer, tmndx);
#endif

#ifdef CONFIG_TIVA_TIMER16_PWM
    case TIMER16_MODE_PWM:           /* 24-bit PWM output mode with upper
                                      * 8-bits in prescaler register */
      return tiva_pwm_mode16(priv, timer, tmndx);
#endif

    default:
      return -EINVAL;
    }
}
#endif

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
  const struct tiva_gptmattr_s *attr;
  struct tiva_gptmstate_s *priv;
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

  irq_detach(attr->irq[TIMER16A]);
  irq_detach(attr->irq[TIMER16B]);

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

  while (!tiva_gptm_periphrdy(config->gptm));

  /* Reset the timer to be certain that it is in the disabled state */

  regval  = getreg32(TIVA_SYSCON_SRTIMER);
  regval |= SYSCON_SRTIMER(config->gptm);
  putreg32(regval, TIVA_SYSCON_SRTIMER);

  regval &= ~SYSCON_SRTIMER(config->gptm);
  putreg32(regval, TIVA_SYSCON_SRTIMER);

  /* Wait for the reset to complete */

  while (!tiva_gptm_periphrdy(config->gptm));
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
#ifdef CONFIG_ARCH_CHIP_TM4C129
      /* Enable the alternate clock source */

      regval  = tiva_getreg(priv, TIVA_TIMER_CC_OFFSET);
      regval |= TIMER_CC_ALTCLK;
      tiva_putreg(priv, TIVA_TIMER_CC_OFFSET, regval);

      /* Remember the frequency of the input clock */

      priv->clkin = ALTCLK_FREQUENCY;
#else
      tmrinfo("tiva_gptm_configure:");
      tmrinfo(" Error: alternate clock only available on TM4C129 devices\n");
      return (TIMER_HANDLE)NULL;
#endif /* CONFIG_ARCH_CHIP_TM4C129 */
    }
  else
    {
      /* Remember the frequency of the input clock */

      priv->clkin = SYSCLK_FREQUENCY;
    }

  /* Then [re-]configure the timer into the new configuration */

  if (config->mode != TIMER16_MODE)
    {
#ifdef CONFIG_TIVA_TIMER_32BIT
      const struct tiva_gptm32config_s *config32 =
        (const struct tiva_gptm32config_s *)config;

      /* Attach the 32-bit timer interrupt handler (but do not yet enable
       * the interrupt).
       */

      ret = irq_attach(attr->irq[TIMER32], attr->handler32, NULL);
      if (ret == OK)
        {
          /* Configure the 32-bit timer */

          ret = tiva_timer32_configure(priv, &config32->config);
        }
#else
       return (TIMER_HANDLE)NULL;
#endif
    }
  else
    {
#ifdef CONFIG_TIVA_TIMER_16BIT
      const struct tiva_gptm16config_s *config16 =
        (const struct tiva_gptm16config_s *)config;

      /* Attach the 16-bit timer interrupt handlers (but do not yet enable
       * the interrupts).
       */

      ret = irq_attach(attr->irq[TIMER16A], attr->handler16[TIMER16A], NULL);
      if (ret == OK)
        {
          ret = irq_attach(attr->irq[TIMER16B],
                           attr->handler16[TIMER16B], NULL);
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
#else
       return (TIMER_HANDLE)NULL;
#endif
    }

  /* Return the timer handler if successfully configured */

  return ret < 0 ? (TIMER_HANDLE)NULL : (TIMER_HANDLE)priv;
}

/****************************************************************************
 * Name: tiva_gptm_release
 *
 * Description:
 *   Release resources held by the timer instance.  After this function is
 *   called, the timer handle is invalid and must not be used further.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tiva_gptm_release(TIMER_HANDLE handle)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_gptmconfig_s *config;
  const struct tiva_gptmattr_s *attr;
  uint32_t regval;

  DEBUGASSERT(priv && priv->attr && priv->config);
  config = priv->config;
  attr   = priv->attr;

  /* Disable and detach interrupt handlers */

  up_disable_irq(attr->irq[TIMER16A]);
  up_disable_irq(attr->irq[TIMER16B]);

  irq_detach(attr->irq[TIMER16A]);
  irq_detach(attr->irq[TIMER16B]);

  /* Reset the time to be certain that it is in the disabled state */

  regval  = getreg32(TIVA_SYSCON_SRTIMER);
  regval |= SYSCON_SRTIMER(config->gptm);
  putreg32(regval, TIVA_SYSCON_SRTIMER);

  regval &= ~SYSCON_SRTIMER(config->gptm);
  putreg32(regval, TIVA_SYSCON_SRTIMER);

  /* Wait for the reset to complete */

  while (!tiva_gptm_periphrdy(config->gptm));
  up_udelay(250);

  /* Disable power and clocking to the GPTM module */

  tiva_gptm_disableclk(config->gptm);
  tiva_gptm_disablepwr(config->gptm);

  /* Un-initialize the state structure */

  memset(priv, 0, sizeof(struct tiva_gptmstate_s));
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

void tiva_gptm_putreg(TIMER_HANDLE handle,
                      unsigned int offset, uint32_t value)
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
 *   The 32-bit value read at the provided offset into the timer register
 *   base address.
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

#ifdef CONFIG_TIVA_TIMER_32BIT
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
#endif

/****************************************************************************
 * Name: tiva_timer16_start
 *
 * Description:
 *   After tiva_gptm_configure() has been called to configure 16-bit
 *   timer(s), this function must be called to start one 16-bit timer.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
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
#endif

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

#ifdef CONFIG_TIVA_TIMER_32BIT
void tiva_timer32_stop(TIMER_HANDLE handle)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;

  DEBUGASSERT(priv && priv->attr);

  /* Disable interrupts at the NVIC */

  up_disable_irq(priv->attr->irq[TIMER32]);

  /* Clear the TAEN bit in the GPTMCTL register to disable the 16-bit timer */

  tiva_modifyreg(priv, TIVA_TIMER_CTL_OFFSET, TIMER_CTL_TAEN, 0);
}
#endif

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

#ifdef CONFIG_TIVA_TIMER_16BIT
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
#endif

/****************************************************************************
 * Name: tiva_timer16_counter
 *
 * Description:
 *   Return the current 24-bit counter value of the 16-bit timer.
 *
 *   The timer 24-bit value is the 16-bit counter value AND the 8-bit
 *   prescaler value.  From the caller's point of view the match value is
 *   the 24-bit timer at the timer input clock frequency.
 *
 *   When counting down in periodic modes, the prescaler contains the
 *   least-significant bits of the count. When counting up, the prescaler
 *   holds the most-significant bits of the count.  But the caller is
 *   protected from this complexity.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   The current 24-bit counter value.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
uint32_t tiva_timer16_counter(TIMER_HANDLE handle, int tmndx)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_gptm16config_s *gptm;
  const struct tiva_timer16config_s *config;
  irqstate_t flags;
  uintptr_t base;
  uintptr_t timerr;
  uintptr_t prescr;
  uint32_t timerv;
  uint32_t prescv;
  uint32_t checkv;
  uint32_t counter;

  DEBUGASSERT(priv && priv->attr &&  priv->config &&
              priv->config->mode == TIMER16_MODE && (unsigned)tmndx < 2);

  /* Get settings common to both 16-bit timers */

  gptm = (const struct tiva_gptm16config_s *)priv->config;
  base = priv->attr->base;

  /* Get settings unique to one of the 16-bit timers */

  if (tmndx)
    {
      /* Get the Timer B configuration */

      config = &gptm->config[TIMER16B];

      /* Get Timer B register addresses */

      timerr = base + TIVA_TIMER_TBR_OFFSET;
      prescr = base + TIVA_TIMER_TBPR_OFFSET;
    }
  else
    {
      /* Get the Timer A configuration */

      config = &gptm->config[TIMER16A];

      /* Get Timer A register addresses */

      timerr = base + TIVA_TIMER_TAR_OFFSET;
      prescr = base + TIVA_TIMER_TAPR_OFFSET;
    }

  /* Are we counting up or down? */

  if (TIMER_ISCOUNTUP(config))
    {
      /* We are counting up. The prescaler holds the most-significant bits of
       * the count.  Sample these registers until we are assured that there
       * is no roll-over from the counter to the prescaler register.
       */

      do
        {
          flags  = enter_critical_section();
          checkv = getreg32(prescr);
          timerv = getreg32(timerr);
          prescv = getreg32(prescr);
          leave_critical_section(flags);
        }
      while (checkv != prescv);

      /* Then form the 32-bit counter value with the prescaler as the most
       * significant 8-bits.
       */

      counter = (prescv & 0xff) << 16 | (timerv & 0xffff);
    }
  else
    {
      /* We are counting down.
       * The prescaler contains the least-significant bits of the count.
       * Sample these registers until we are assured that there is no
       * roll-over from the counter to the counter register.
       */

      do
        {
          flags  = enter_critical_section();
          checkv = getreg32(timerr);
          prescv = getreg32(prescr);
          timerv = getreg32(timerr);
          leave_critical_section(flags);
        }
      while (checkv != timerv);

      /* Then form the 32-bit counter value with the counter as the most
       * significant 8-bits.
       */

      counter = (timerv & 0xffff) << 8 | (prescv & 0xff);
    }

  /* Return the counter value */

  return counter;
}
#endif

/****************************************************************************
 * Name: tiva_timer32_setinterval
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   load value of a 32-bit timer.
 *
 *   It the timer is configured as a 32-bit one-shot or periodic timer, then
 *   then function will also enable timeout interrupts.
 *
 *   NOTE: As of this writing, there is no interface to disable the timeout
 *   interrupts once they have been enabled.
 *
 *   REVISIT:  When the ALTCLK bit is set in the GPTMCC register to enable
 *   using the alternate clock source, the synchronization imposes
 *   restrictions on the starting count value (down-count), terminal value
 *   (up-count) and the match value. This restriction applies to all modes
 *   of operation. Each event must be spaced by 4 Timer (ALTCLK) clock
 *   periods + 2 system clock periods. If some events do not meet this
 *   requirement, then it is possible that the timer block may need to be
 *   reset for correct functionality to be restored.
 *
 *     Example: ALTCLK= TPIOSC = 62.5ns (16Mhz Trimmed)
 *     Thclk = 1us (1Mhz)
 *     4*62.5ns + 2*1us = 2.25us 2.25us/62.5ns = 36 or 0x23
 *
 *   The minimum values for the periodic or one-shot with a match
 *   interrupt enabled are: GPTMTAMATCHR = 0x23 GPTMTAILR = 0x46"
 *
 * Input Parameters:
 *   handle   - The handle value returned by tiva_gptm_configure()
 *   interval - The value to write to the timer interval load register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_32BIT
void tiva_timer32_setinterval(TIMER_HANDLE handle, uint32_t interval)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_gptm32config_s *config;
  const struct tiva_timer32config_s *timer;
  irqstate_t flags;
  uintptr_t base;
  uintptr_t loadr;
  uintptr_t imrr;
#ifdef CONFIG_ARCH_CHIP_TM4C129
  uintptr_t moder;
  uint32_t modev1;
  uint32_t modev2;
#endif /* CONFIG_ARCH_CHIP_TM4C129 */
  bool toints;

  DEBUGASSERT(priv);
  DEBUGASSERT(priv->attr);
  DEBUGASSERT(priv->config);
  config = (const struct tiva_gptm32config_s *)priv->config;

  DEBUGASSERT(config->cmn.mode != TIMER16_MODE);
  timer  = &config->config;

  /* Do we need to enable timeout interrupts?  Interrupts are only enabled
   * if (1) the user has provided a handler, and (2) the timer timer is
   * configure as a one-short or periodic timer.
   */

  base   = priv->attr->base;
  toints = false;

  if (timer->handler &&
     (config->cmn.mode == TIMER32_MODE_ONESHOT ||
      config->cmn.mode == TIMER32_MODE_PERIODIC))
    {
      toints = true;
    }

  loadr = base + TIVA_TIMER_TAILR_OFFSET;
  imrr  = base + TIVA_TIMER_IMR_OFFSET;

  /* Make the following atomic */

  flags = enter_critical_section();

  /* Set the new timeout interval */

  putreg32(interval, loadr);

  /* Enable/disable timeout interrupts */

  if (toints)
    {
#ifdef CONFIG_ARCH_CHIP_TM4C129
      /* Clearing the TACINTD bit allows the time-out interrupt to be
       * generated as normal
       */

      moder = base + TIVA_TIMER_TAMR_OFFSET;
      modev1 = getreg32(moder);
      modev2 = modev1 & ~TIMER_TNMR_TNCINTD;
      putreg32(modev2, moder);
#endif /* CONFIG_ARCH_CHIP_TM4C129 */

      /* Set the new interrupt mask */

      priv->imr |= TIMER_INT_TATO;
      putreg32(priv->imr, imrr);
    }

  leave_critical_section(flags);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Generate low-level debug output outside of the critical section */

  tmrinfo("%08x<-%08x\n", loadr, interval);
  if (toints)
    {
#  ifdef CONFIG_ARCH_CHIP_TM4C129
      tmrinfo("%08x->%08x\n", moder, modev1);
      tmrinfo("%08x<-%08x\n", moder, modev2);
#  endif /* CONFIG_ARCH_CHIP_TM4C129 */
      tmrinfo("%08x<-%08x\n", imrr, priv->imr);
    }
#endif
}
#endif

/****************************************************************************
 * Name: tiva_timer16_setinterval
 *
 * Description:
 *   This function may be called at any time to change the timer interval
 *   load value of a 16-bit timer.
 *
 *   It the timer is configured as a 16-bit one-shot or periodic timer, then
 *   then function will also enable timeout interrupts.
 *
 *   NOTE: As of this writing, there is no interface to disable the timeout
 *   interrupts once they have been enabled.
 *
 *   REVISIT:  When the ALTCLK bit is set in the GPTMCC register to enable
 *   using the alternate clock source, the synchronization imposes
 *   restrictions on the starting count value (down-count), terminal value
 *   (up-count) and the match value. This restriction applies to all modes
 *   of operation. Each event must be spaced by 4 Timer (ALTCLK) clock
 *   periods + 2 system clock periods. If some events do not meet this
 *   requirement, then it is possible that the timer block may need to be
 *   reset for correct functionality to be restored.
 *
 *     Example: ALTCLK= TPIOSC = 62.5ns (16Mhz Trimmed)
 *     Thclk = 1us (1Mhz)
 *     4*62.5ns + 2*1us = 2.25us 2.25us/62.5ns = 36 or 0x23
 *
 *   The minimum values for the periodic or one-shot with a match
 *   interrupt enabled are: GPTMTAMATCHR = 0x23 GPTMTAILR = 0x46"
 *
 * Input Parameters:
 *   handle   - The handle value returned  by tiva_gptm_configure()
 *   interval - The value to write to the timer interval load register
 *   tmndx    - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER_16BIT
void tiva_timer16_setinterval(TIMER_HANDLE handle,
                              uint16_t interval, int tmndx)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_gptm16config_s *config;
  const struct tiva_timer16config_s *timer;
  irqstate_t flags;
  uintptr_t base;
  uintptr_t loadr;
  uintptr_t imrr;
#ifdef CONFIG_ARCH_CHIP_TM4C129
  uintptr_t moder;
  uint32_t modev1;
  uint32_t modev2;
#endif /* CONFIG_ARCH_CHIP_TM4C129 */
  uint32_t intbit;
  bool toints;

  DEBUGASSERT(priv && priv->attr &&  priv->config &&
              priv->config->mode == TIMER16_MODE && (unsigned)tmndx < 2);

  config = (const struct tiva_gptm16config_s *)priv->config;
  timer  = &config->config[tmndx];

  /* Pre-calculate as much as possible outside of the critical section */

  base = priv->attr->base;
  if (tmndx)
    {
      intbit = TIMER_INT_TBTO;
      loadr  = base + TIVA_TIMER_TBILR_OFFSET;
    }
  else
    {
      intbit = TIMER_INT_TATO;
      loadr  = base + TIVA_TIMER_TAILR_OFFSET;
    }

  imrr = base + TIVA_TIMER_IMR_OFFSET;

  /* Do we need to enable timeout interrupts?  Interrupts are only enabled
   * if (1) the user has provided a handler, and (2) the timer is
   * configured as a one-shot or periodic timer.
   */

  toints = false;

  if (timer->handler &&
     (timer->mode == TIMER16_MODE_ONESHOT ||
      timer->mode == TIMER16_MODE_PERIODIC))
    {
       toints = true;
    }

  /* Make the following atomic */

  flags = enter_critical_section();

  /* Set the new timeout interval */

  putreg32(interval, loadr);

  /* Enable/disable timeout interrupts */

  if (toints)
    {
#ifdef CONFIG_ARCH_CHIP_TM4C129
      if (tmndx)
        {
          moder  = base + TIVA_TIMER_TBMR_OFFSET;
        }
      else
        {
          moder  = base + TIVA_TIMER_TAMR_OFFSET;
        }

      /* Clearing the TACINTD bit allows the time-out interrupt to be
       * generated as normal
       */

      modev1 = getreg32(moder);
      modev2 = modev1 & ~TIMER_TNMR_TNCINTD;
      putreg32(modev2, moder);
#endif /* CONFIG_ARCH_CHIP_TM4C129 */

      /* Set the new interrupt mask */

      priv->imr |= intbit;
      putreg32(priv->imr, imrr);
    }

  leave_critical_section(flags);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Generate low-level debug output outside of the critical section */

  tmrinfo("%08x<-%08x\n", loadr, interval);
  if (toints)
    {
#ifdef CONFIG_ARCH_CHIP_TM4C129
      tmrinfo("%08x->%08x\n", moder, modev1);
      tmrinfo("%08x<-%08x\n", moder, modev2);
#endif
      tmrinfo("%08x<-%08x\n", imrr, priv->imr);
    }
#endif
}
#endif

/****************************************************************************
 * Name: tiva_timer32_remaining
 *
 * Description:
 *   Get the time remaining before a one-shot or periodic 32-bit timer
 *   expires.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure().
 *
 * Returned Value:
 *   Time remaining until the next timeout interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER32_PERIODIC
uint32_t tiva_timer32_remaining(TIMER_HANDLE handle)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_gptm32config_s *config;
  irqstate_t flags;
  uint32_t counter;
  uint32_t status;
  uint32_t interval;
  uint32_t remaining;

  tmrinfo("Entry\n");

  DEBUGASSERT(priv && priv->attr && priv->config &&
              priv->config->mode != TIMER16_MODE);

  config = (const struct tiva_gptm32config_s *)priv->config;
  DEBUGASSERT(config->cmn.mode == TIMER32_MODE_ONESHOT ||
              config->cmn.mode == TIMER32_MODE_PERIODIC);

  /* These values can be modified if a timer interrupt were to occur.  Best
   * to do this is a critical section.
   */

  flags = enter_critical_section();

  /* Get the time remaining until the timer expires (in clock ticks).
   * Since we have selected a count-up timer timer and the interval will
   * expire when the count-up timer equals the timeout value, the
   * difference between the current count value and the timeout is the
   * time remaining.
   *
   * There is a race condition here.  What if the timer expires and
   * counter rolls over between the time that we disabled interrupts
   * above and the time that we read the counter below?
   */

  counter = tiva_getreg(priv, TIVA_TIMER_TAR_OFFSET);

  /* If the timer rolled over, there would be a pending timer interrupt.  In
   * that case, the time remaining time is zero.
   */

  status = tiva_getreg(priv, TIVA_TIMER_MIS_OFFSET);
  if ((status & TIMER_INT_TATO) != 0)
    {
      remaining = 0;
    }
  else
    {
      /* Is this a count-up or a count-down timer? */

      if (TIMER_ISCOUNTUP(&config->config))
        {
          /* Counting up.. When the timer is counting up and it reaches the
           * timeout event (the value in the GPTMTAILR, the timer reloads
           * with zero.
           *
           * Get the current timer interval value
           */

           interval  = tiva_getreg(priv, TIVA_TIMER_TAILR_OFFSET);

          /* The time remaining is the current interval reload value minus
           * the above sampled counter value.
           *
           * REVISIT: Or the difference +1?
           */

          DEBUGASSERT(interval == 0 || interval >= counter);
          remaining = interval - counter;
        }
      else
        {
          /* Counting down:  When the timer is counting down and it reaches
           * the timeout event (0x0), the timer reloads its start value
           * from the GPTMTAILR register on the next cycle.
           *
           * The time remaining it then just the value of the counter
           * register.
           *
           * REVISIT: Or the counter value +1?
           */

          remaining = counter;
        }
    }

  leave_critical_section(flags);
  return remaining;
}
#endif /* CONFIG_TIVA_TIMER32_PERIODIC */

/****************************************************************************
 * Name: tiva_rtc_setalarm
 *
 * Description:
 *   Setup to receive an interrupt when the RTC counter equals a match time
 *   value.  This function sets the match register to the current timer
 *   counter register value PLUS the relative value provided.  The relative
 *   value then is an offset in seconds from the current time.
 *
 *   NOTE: Use of this function is only meaningful for a a 32-bit RTC time.
 *
 * Input Parameters:
 *   handle - The handle value returned  by tiva_gptm_configure()
 *   delay  - A relative time in the future (seconds)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER32_RTC
void tiva_rtc_setalarm(TIMER_HANDLE handle, uint32_t delay)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_timer32config_s *config;
  irqstate_t flags;
  uintptr_t base;
  uint32_t counter;
  uint32_t match;
#ifdef CONFIG_ARCH_CHIP_TM4C129
  uint32_t adcev;
  uint32_t adcbits;
#endif

  DEBUGASSERT(priv && priv->attr && priv->config &&
              priv->config->mode == TIMER32_MODE_RTC);

  /* Update the saved IMR if an interrupt will be needed */

  config = (const struct tiva_timer32config_s *)priv->config;
  if (config->handler)
    {
      /* Update the saved IMR to enable the RTC match interrupt */

      priv->imr |= TIMER_INT_RTC;
    }

  /* This must be done without interrupt or context switches to minimize
   * race conditions with the free-running timer.  Note that we also
   * by-pass the normal register accesses to keep the latency to a
   * minimum.
   */

  base = priv->attr->base;

  flags = enter_critical_section();

  /* Set the match register to the current value of the timer counter plus
   * the provided relative delay value.
   */

  counter = getreg32(base + TIVA_TIMER_TAR_OFFSET);
  match   = counter + delay;
  putreg32(match, base + TIVA_TIMER_TAMATCHR_OFFSET);

#ifdef CONFIG_ARCH_CHIP_TM4C129
  /* Enable ADC trigger (if selected).  NOTE the TAOTE bit was already
   * selected in the GPTMCTL register when the timer was configured.
   */

  adcev = getreg32(base + TIVA_TIMER_ADCEV_OFFSET);
  adcbits = TIMER_ISADCRTCM(config) ? TIMER_ADCEV_RTCADCEN : 0;
  putreg32(adcev | adcbits, base + TIVA_TIMER_ADCEV_OFFSET);
#endif /* CONFIG_ARCH_CHIP_TM4C129 */

  /* TODO: Set uDMA trigger in the same manner */

  /* Enable interrupts as necessary */

  putreg32(priv->imr, base + TIVA_TIMER_IMR_OFFSET);
  leave_critical_section(flags);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Generate low-level debug output outside of the critical section */

  tmrinfo("%08x->%08x\n", base + TIVA_TIMER_TAR_OFFSET, counter);
  tmrinfo("%08x<-%08x\n", base + TIVA_TIMER_TAMATCHR_OFFSET, match);
#ifdef CONFIG_ARCH_CHIP_TM4C129
  tmrinfo("%08x->%08x\n", base + TIVA_TIMER_ADCEV_OFFSET, adcev);
  tmrinfo("%08x<-%08x\n", base + TIVA_TIMER_ADCEV_OFFSET, adcev | adcbits);
#endif /* CONFIG_ARCH_CHIP_TM4C129 */
  tmrinfo("%08x<-%08x\n", base + TIVA_TIMER_IMR_OFFSET, priv->imr);
#endif
}
#endif

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
 *   NOTE: Use of this function is only meaningful for a free-running,
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

#ifdef CONFIG_TIVA_TIMER32_PERIODIC
void tiva_timer32_relmatch(TIMER_HANDLE handle, uint32_t relmatch)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_timer32config_s *config;
  uintptr_t base;
  irqstate_t flags;
  uint32_t counter;
  uint32_t match;
#ifdef CONFIG_ARCH_CHIP_TM4C129
  uint32_t adcev;
  uint32_t adcbits;
#endif

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

  base    = priv->attr->base;
  flags = enter_critical_section();

  /* Set the match register to the current value of the timer counter plus
   * the provided relative match value.
   *
   * NOTE that the prescale match is not used with the 32-bit timer.
   */

  counter = getreg32(base + TIVA_TIMER_TAR_OFFSET);
  match   = counter + relmatch;
  putreg32(match, base + TIVA_TIMER_TAMATCHR_OFFSET);

#ifdef CONFIG_ARCH_CHIP_TM4C129
  /* Enable ADC trigger (if selected).  NOTE the TAOTE bit was already
   * selected in the GPTMCTL register when the timer was configured.
   */

  adcev   = getreg32(base + TIVA_TIMER_ADCEV_OFFSET);
  adcbits = TIMER_ISADCMATCH(config) ? TIMER_ADCEV_CAMADCEN : 0;
  putreg32(adcev | adcbits, base + TIVA_TIMER_ADCEV_OFFSET);
#endif /* CONFIG_ARCH_CHIP_TM4C129 */

  /* Enable interrupts as necessary */

  putreg32(priv->imr, base + TIVA_TIMER_IMR_OFFSET);
  leave_critical_section(flags);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Generate low-level debug output outside of the critical section */

  tmrinfo("%08x->%08x\n", base + TIVA_TIMER_TAR_OFFSET, counter);
  tmrinfo("%08x<-%08x\n", base + TIVA_TIMER_TAMATCHR_OFFSET, match);
#ifdef CONFIG_ARCH_CHIP_TM4C129
  tmrinfo("%08x->%08x\n", base + TIVA_TIMER_ADCEV_OFFSET, adcev);
  tmrinfo("%08x<-%08x\n", base + TIVA_TIMER_ADCEV_OFFSET, adcev | adcbits);
#endif /* CONFIG_ARCH_CHIP_TM4C129 */
  tmrinfo("%08x<-%08x\n", base + TIVA_TIMER_IMR_OFFSET, priv->imr);
#endif /* CONFIG_TIVA_TIMER_REGDEBUG */
}
#endif /* CONFIG_TIVA_TIMER32_PERIODIC */

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
 *   NOTE: Use of this function is only meaningful for a free-running,
 *   periodic timer.
 *
 *   NOTE: The relmatch input is a really a 24-bit value; it is the 16-bit
 *   match counter match value AND the 8-bit prescaler match value.  From
 *   the caller's point of view the match value is the 24-bit time to match
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

#ifdef CONFIG_TIVA_TIMER16_PERIODIC
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
#ifdef CONFIG_ARCH_CHIP_TM4C129
  uintptr_t adcevr;
  uint32_t adcevv;
  uint32_t adcbits;
#endif
  uint32_t timerv;
  uint32_t prescv;
  uint32_t matchv;
  uint32_t prematchv;
  uint32_t counter;
  bool countup;

  DEBUGASSERT(priv && priv->attr &&  priv->config &&
              priv->config->mode == TIMER16_MODE && (unsigned)tmndx < 2);

  /* Pre-calculate as much as possible before entering the critical section */

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

#ifdef CONFIG_ARCH_CHIP_TM4C129
      /* Do we need to enable ADC trigger on the match? */

      adcbits = TIMER_ISADCMATCH(config) ? TIMER_ADCEV_CBMADCEN : 0;
#endif /* CONFIG_ARCH_CHIP_TM4C129 */
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

      /* Get Timer A register addresses */

      timerr    = base + TIVA_TIMER_TAR_OFFSET;
      prescr    = base + TIVA_TIMER_TAPR_OFFSET;
      matchr    = base + TIVA_TIMER_TAMATCHR_OFFSET;
      prematchr = base + TIVA_TIMER_TAPMR_OFFSET;

#ifdef CONFIG_ARCH_CHIP_TM4C129
      /* Do we need to enable ADC trigger on the match? */

      adcbits = TIMER_ISADCMATCH(config) ? TIMER_ADCEV_CAMADCEN : 0;
#endif /* CONFIG_ARCH_CHIP_TM4C129 */
    }

#ifdef CONFIG_ARCH_CHIP_TM4C129
  adcevr   = base + TIVA_TIMER_ADCEV_OFFSET;
#endif /* CONFIG_ARCH_CHIP_TM4C129 */

  imr      = base + TIVA_TIMER_IMR_OFFSET;
  countup  = TIMER_ISCOUNTUP(config);

  /* This must be done without interrupt or context switches to minimize
   * race conditions with the free-running timer.  Note that we also
   * by-pass the normal register accesses to keep the latency to a
   * minimum.
   */

  flags  = enter_critical_section();
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

#ifdef CONFIG_ARCH_CHIP_TM4C129
  /* Enable ADC trigger (if selected).  NOTE the TnOTE bit was already
   * selected in the GPTMCTL register when the timer was configured.
   */

  adcevv = getreg32(adcevr);
  putreg32(adcevv | adcbits, adcevr);
#endif

  /* Enable interrupts as necessary */

  putreg32(priv->imr, imr);
  leave_critical_section(flags);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Generate low-level debug output outside of the critical section */

  tmrinfo("%08x->%08x\n", timerr, timerv);
  tmrinfo("%08x->%08x\n", prescr, prescv);
  tmrinfo("%08x<-%08x\n", matchr, matchv);
  tmrinfo("%08x<-%08x\n", prematchr, prematchv);
#ifdef CONFIG_ARCH_CHIP_TM4C129
  tmrinfo("%08x->%08x\n", adcevr, adcevv);
  tmrinfo("%08x<-%08x\n", adcevr, adcevv | adcbits);
#endif
  tmrinfo("%08x<-%08x\n", imr, priv->imr);
#endif
}
#endif

/****************************************************************************
 * Name: tiva_timer16pwm_setperiodduty
 *
 * Description:
 *   Set the period and initial duty cycle for a 16-bit timer operating in
 *   PWM mode. Also, enable interrupts if a handler is provided. The timer
 *   is not started until tiva_timer16_start() is called.
 *
 * Input Parameters:
 *   handle - The handle value returned by tiva_gptm_configure()
 *   period - The PWM period, a 24-bit value.
 *   duty   - The initial PWM duty cycle, a 24-bit value.
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PWM
void tiva_timer16pwm_setperiodduty(TIMER_HANDLE handle, uint32_t period,
                                   uint32_t duty, int tmndx)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  const struct tiva_gptm16config_s *config;
  const struct tiva_timer16config_s *timer;
  irqstate_t flags;
  uintptr_t base;
  bool toints;
  uint32_t imr;
  uint32_t periodhi;
  uint32_t dutyhi;

  DEBUGASSERT(priv && priv->attr &&  priv->config &&
              priv->config->mode == TIMER16_MODE &&
              (period & 0x00ffffff) && (duty & 0x00ffffff) &&
              (unsigned)tmndx < 2);

  config = (const struct tiva_gptm16config_s *)priv->config;
  timer  = &config->config[tmndx];
  base = priv->attr->base;

  /* Do we need to enable timeout interrupts?  Interrupts are only enabled
   * if (1) the user has provided a handler, and (2) the timer is
   * configured for PWM.
   */

  toints = false;
  imr = 0;

  if (timer->handler && timer->mode == TIMER16_MODE_PWM)
    {
       toints = true;
    }

  /* To set PWM period:
   * Put high byte (8 bits) in prescaler register.
   * Put low word (16 bits) in interval load register.
   *
   * To set PWM duty cycle:
   * Put high byte (8 bits) in prescale match register.
   * Put low word (16 bits) in match register.
   */

  periodhi = (period >> 16) & 0xff;
  period &= 0xffff;
  dutyhi = (duty >> 16) & 0xff;
  duty &= 0xffff;

  /* Make the following atomic */

  flags = enter_critical_section();

  if (tmndx)
    {
      putreg32(periodhi, base + TIVA_TIMER_TBPR_OFFSET);
      putreg32(period, base + TIVA_TIMER_TBILR_OFFSET);

      putreg32(dutyhi, base + TIVA_TIMER_TBPMR_OFFSET);
      putreg32(duty, base + TIVA_TIMER_TBMATCHR_OFFSET);

      imr |= TIMER_INT_CBE;
    }
  else
    {
      putreg32(periodhi, base + TIVA_TIMER_TAPR_OFFSET);
      putreg32(period, base + TIVA_TIMER_TAILR_OFFSET);

      putreg32(dutyhi, base + TIVA_TIMER_TAPMR_OFFSET);
      putreg32(duty, base + TIVA_TIMER_TAMATCHR_OFFSET);

      imr |= TIMER_INT_CAE;
    }

  /* Enable the capture mode event interrupt at the timer peripheral.
   * The interrupt will not fire until enabled at the NVIC. That will be
   * done when tiva_timer16_start() is called.
   */

  if (toints)
    {
      priv->imr = imr;
      putreg32(priv->imr, base + TIVA_TIMER_IMR_OFFSET);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: tiva_timer16pwm_setduty
 *
 * Description:
 *   Update the duty cycle for a 16-bit timer operating in PWM mode.
 *
 * Input Parameters:
 *   handle - The handle value returned by tiva_gptm_configure()
 *   duty   - The initial PWM duty cycle, a 24-bit value.
 *   tmndx  - Either TIMER16A or TIMER16B to select the 16-bit timer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER16_PWM
void tiva_timer16pwm_setduty(TIMER_HANDLE handle, uint32_t duty, int tmndx)
{
  struct tiva_gptmstate_s *priv = (struct tiva_gptmstate_s *)handle;
  irqstate_t flags;
  uintptr_t base;
  uintptr_t tnpmr;
  uintptr_t tnmatchr;
  uint32_t dutyhi;

  DEBUGASSERT(priv && priv->attr &&  priv->config &&
              priv->config->mode == TIMER16_MODE &&
              (duty & 0x00ffffff) && (unsigned)tmndx < 2);

  base = priv->attr->base;

  /* To set PWM duty cycle:
   * Put high byte (8 bits) in prescale match register.
   * Put low word (16 bits) in match register.
   */

  if (tmndx)
    {
      tnpmr = base + TIVA_TIMER_TBPMR_OFFSET;
      tnmatchr = base + TIVA_TIMER_TBMATCHR_OFFSET;
    }
  else
    {
      tnpmr = base + TIVA_TIMER_TAPMR_OFFSET;
      tnmatchr = base + TIVA_TIMER_TAMATCHR_OFFSET;
    }

  dutyhi = (duty >> 16) & 0xff;
  duty &= 0xffff;

  /* Make the following atomic */

  flags = enter_critical_section();

  putreg32(dutyhi, tnpmr);
  putreg32(duty, tnmatchr);

  leave_critical_section(flags);
}
#endif
