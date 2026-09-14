/****************************************************************************
 * arch/arm/src/am67/am67_i2c.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "am67_pinmux.h"
#include "am67_i2c.h"
#include "am67_i2c_hw.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_AM67_I2C0) || defined(CONFIG_AM67_I2C1) || \
    defined(CONFIG_AM67_WKUP_I2C0)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM67_I2C_SCLK             (96000000) /* 96 MHz */

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_AM67_I2CTIMEOSEC) && !defined(CONFIG_AM67_I2CTIMEOMS)
#  define CONFIG_AM67_I2CTIMEOSEC 0
#  define CONFIG_AM67_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_AM67_I2CTIMEOSEC)
#  define CONFIG_AM67_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_AM67_I2CTIMEOMS)
#  define CONFIG_AM67_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_AM67_I2CTIMEOTICKS
#  define CONFIG_AM67_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_AM67_I2CTIMEOSEC) + MSEC2TICK(CONFIG_AM67_I2CTIMEOMS))
#endif

#ifndef CONFIG_AM67_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_AM67_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_AM67_I2CTIMEOTICKS)
#endif

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define am67_i2c_tracereset(p)
#  define am67_i2c_tracenew(p,s)
#  define am67_i2c_traceevent(p,e,a)
#  define am67_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

#ifdef CONFIG_I2C_SLAVE
#  error I2C slave logic is not supported yet for AM67
#endif

#define I2C_MASTER      1
#define I2C_SLAVE       2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum am67_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum am67_trace_e
{
  I2CEVENT_NONE = 0,      /* No events have occurred with this status */
  I2CEVENT_SENDADDR,      /* Start/Master bit set and address sent, param = msgc */
  I2CEVENT_SENDBYTE,      /* Send byte, param = dcnt */
  I2CEVENT_RCVBYTE,       /* Read more dta, param = dcnt */
  I2CEVENT_NOSTART,       /* BTF on last byte with no restart, param = msgc */
  I2CEVENT_STARTRESTART,  /* Last byte sent, re-starting, param = msgc */
  I2CEVENT_STOP,          /* Last byte sten, send stop, param = 0 */
  I2CEVENT_ERROR          /* Error occurred, param = 0 */
};

/* Trace data */

struct am67_trace_s
{
  uint32_t status;              /* IRQSTATUS at the time of the event */
  uint32_t count;               /* Interrupt count when status change */
  enum am67_trace_e event;      /* Last event that occurred with this status */
  uint32_t parm;                /* Parameter associated with the event */
  clock_t time;                 /* First of event or first status */
};

/* I2C Device hardware configuration */

struct am67_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint8_t mode;               /* Master or Slave mode */
#ifndef CONFIG_I2C_POLLED
  uint32_t irq;               /* Event IRQ */
#endif
};

/* I2C Device Private Data */

struct am67_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct am67_i2c_config_s *config;

  int refs;                    /* Reference count */
  bool inited;                 /* HW brought up lazily on the first transfer */
  mutex_t lock;                /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum am67_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  uint32_t frequency;          /* Current I2C frequency */
  int dcnt;                    /* Current message length */
  uint16_t flags;              /* Current message flags */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct am67_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer IRQSTATUS */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t am67_i2c_getreg(struct am67_i2c_priv_s *priv,
                                         uint16_t offset);
static inline void am67_i2c_putreg(struct am67_i2c_priv_s *priv,
                                     uint16_t offset, uint32_t value);
static inline void am67_i2c_modifyreg(struct am67_i2c_priv_s *priv,
                                        uint16_t offset, uint32_t clearbits,
                                        uint32_t setbits);

#ifdef CONFIG_AM67_I2C_DYNTIMEO
static uint32_t am67_i2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_AM67_I2C_DYNTIMEO */

static inline int
am67_i2c_sem_waitdone(struct am67_i2c_priv_s *priv);
static inline bool
am67_i2c_sem_waitstop(struct am67_i2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void am67_i2c_tracereset(struct am67_i2c_priv_s *priv);
static void am67_i2c_tracenew(struct am67_i2c_priv_s *priv,
                                uint32_t status);
static void am67_i2c_traceevent(struct am67_i2c_priv_s *priv,
                                  enum am67_trace_e event, uint32_t parm);
static void am67_i2c_tracedump(struct am67_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void am67_i2c_setclock(struct am67_i2c_priv_s *priv,
                                uint32_t frequency);
static void am67_i2c_startmsg(struct am67_i2c_priv_s *priv);
static inline void am67_i2c_sendstop(struct am67_i2c_priv_s *priv);
static inline uint32_t
am67_i2c_getstatus(struct am67_i2c_priv_s *priv);

static int am67_i2c_isr_process(struct am67_i2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
static int am67_i2c_isr(int irq, void *context, void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int am67_i2c_init(struct am67_i2c_priv_s *priv);
static int am67_i2c_deinit(struct am67_i2c_priv_s *priv);
static int am67_i2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Trace events strings */

#ifdef CONFIG_I2C_TRACE
static const char *g_trace_names[] =
{
  "NONE           ",
  "SENDADDR       ",
  "SENDBYTE       ",
  "RCVBYTE        ",
  "NOSTART        ",
  "START/RESTART  ",
  "STOP           ",
  "ERROR          "
};
#endif

/* I2C interface */

static const struct i2c_ops_s am67_i2c_ops =
{
  .transfer = am67_i2c_transfer
};

/* I2C device structures */

#ifdef CONFIG_AM67_I2C0
static const struct am67_i2c_config_s am67_i2c0_config =
{
  .base       = AM67_I2C0_VADDR,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq        = AM67_IRQ_I2C0,
#endif
};

static struct am67_i2c_priv_s am67_i2c0_priv =
{
  .ops        = &am67_i2c_ops,
  .config     = &am67_i2c0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

#ifdef CONFIG_AM67_WKUP_I2C0
static const struct am67_i2c_config_s am67_i2c2_config =
{
  .base       = AM67_WKUP_I2C0_VADDR,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq        = AM67_IRQ_WKUP_I2C0,
#endif
};

static struct am67_i2c_priv_s am67_i2c2_priv =
{
  .ops        = &am67_i2c_ops,
  .config     = &am67_i2c2_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_i2c_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t am67_i2c_getreg(struct am67_i2c_priv_s *priv,
                                         uint16_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: am67_i2c_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void am67_i2c_putreg(struct am67_i2c_priv_s *priv,
                                     uint16_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: am67_i2c_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void am67_i2c_modifyreg(struct am67_i2c_priv_s *priv,
                                        uint16_t offset, uint32_t clearbits,
                                        uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: am67_i2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_AM67_I2C_DYNTIMEO
static uint32_t am67_i2c_toticks(int msgc, struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor.
   */

  return USEC2TICK(CONFIG_AM67_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: am67_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int am67_i2c_sem_waitdone(struct am67_i2c_priv_s *priv)
{
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  /* Signal the interrupt handler that we are waiting, then start the
   * first segment.  startmsg() arms exactly the interrupts the segment
   * needs; the ISR sequences all remaining segments on ARDY.  NOTE:
   * Interrupts are currently disabled but will be temporarily re-enabled
   * below when nxsem_tickwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  am67_i2c_startmsg(priv);

  do
    {
      /* Wait until either the transfer is complete or the timeout expires */

#ifdef CONFIG_AM67_I2C_DYNTIMEO
      ret = nxsem_tickwait(&priv->sem_isr,
                           am67_i2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait(&priv->sem_isr,
                           CONFIG_AM67_I2CTIMEOTICKS);
#endif
      if (ret < 0 && ret != -EINTR)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by nxsem_tickwait.
           * NOTE that we try again if we are awakened by a signal (EINTR).
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  am67_i2c_putreg(priv, AM67_I2C_IRQ_EN_CLR_OFFSET, I2C_ICR_CLEARMASK);

  leave_critical_section(flags);
  return ret;
}
#else
static inline int am67_i2c_sem_waitdone(struct am67_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_AM67_I2C_DYNTIMEO
  timeout = am67_i2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_AM67_I2CTIMEOTICKS;
#endif

  priv->intstate = INTSTATE_WAITING;

  /* Start the first segment; the polling loop below sequences the rest */

  am67_i2c_startmsg(priv);

  start = clock_systime_ticks();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      am67_i2c_isr_process(priv);
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cinfo("intstate: %d elapsed: %ld threshold: %ld status: %08" PRIx32
          "\n", priv->intstate, (long)elapsed, (long)timeout, priv->status);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/****************************************************************************
 * Name: am67_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ****************************************************************************/

static inline bool
am67_i2c_sem_waitstop(struct am67_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t regval;

  /* Select a timeout */

#ifdef CONFIG_AM67_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_AM67_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_AM67_I2CTIMEOTICKS;
#endif

  /* Wait as stop might still be in progress; but stop might also
   * be set because of a timeout error: "The [STOP] bit is set and
   * cleared by software, cleared by hardware when a Stop condition is
   * detected, set by hardware when a timeout error is detected."
   */

  start = clock_systime_ticks();
  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Check for Bus Free condition */

      regval = am67_i2c_getreg(priv, AM67_I2C_IRQ_STAT_RAW_OFFSET);
      if ((regval & I2C_IRQ_BB) == 0)
        {
          return true;
        }
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.
   */

  i2cinfo("Timeout with Status Register: %" PRIx32 "\n", regval);
  return false;
}

/****************************************************************************
 * Name: am67_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void am67_i2c_traceclear(struct am67_i2c_priv_s *priv)
{
  struct am67_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void am67_i2c_tracereset(struct am67_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  am67_i2c_traceclear(priv);
}

static void am67_i2c_tracenew(struct am67_i2c_priv_s *priv,
                                uint32_t status)
{
  struct am67_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?  Has the status changed? */

  if (trace->count == 0 || status != trace->status)
    {
      /* Yes.. Was it the status changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index (unless out of trace entries) */

          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              i2cerr("ERROR: Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      am67_i2c_traceclear(priv);
      trace->status = status;
      trace->count  = 1;
      trace->time   = clock_systime_ticks();
    }
  else
    {
      /* Just increment the count of times that we have seen this status */

      trace->count++;
    }
}

static void am67_i2c_traceevent(struct am67_i2c_priv_s *priv,
                                  enum am67_trace_e event, uint32_t parm)
{
  struct am67_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event = event;
      trace->parm  = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
        {
          i2cerr("ERROR: Trace table overflow\n");
          return;
        }

      priv->tndx++;
      am67_i2c_traceclear(priv);
    }
}

static void am67_i2c_tracedump(struct am67_i2c_priv_s *priv)
{
  struct am67_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %ld\n",
         (long)(clock_systime_ticks() - priv->start_time));

  for (i = 0; i < priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. STATUS: %08x COUNT: %3d EVENT: %s(%2d) PARM: %08x "
             "TIME: %d\n",
             i + 1, trace->status, trace->count,
             g_trace_names[trace->event],
             trace->event, trace->parm, trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: am67_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void am67_i2c_setclock(struct am67_i2c_priv_s *priv,
                                uint32_t frequency)
{
  uint32_t src_freq = AM67_I2C_SCLK;
  uint32_t men;
  uint32_t prescale = 0;
  uint32_t scl = 0;
  uint32_t scl_low = 0;
  uint32_t scl_hi = 0;
  uint32_t best_prescale = 0;
  uint32_t best_scl_low = 0;
  uint32_t best_scl_hi = 0;
  uint32_t abs_error = 0;
  uint32_t best_error = 0xffffffff;
  uint32_t computed_rate;

  /* Has the I2C bus frequency changed? */

  if (priv->config->mode == I2C_MASTER)
    {
      if (frequency != priv->frequency)
        {
          /* Disable the selected I2C peripheral to configure the new
           * clock if it is enabled.
           */

          men = am67_i2c_getreg(priv, AM67_I2C_CON_OFFSET) & I2C_CON_EN;
          if (men)
            {
              am67_i2c_modifyreg(priv, AM67_I2C_CON_OFFSET,
                                   I2C_CON_EN, 0);
            }

          /* I2C bus clock is:
           * Source Clock (Hz) / ((psc + 1) * (scll + 7 + sclh + 5))
           */

          for (scl = 14; scl < 522; scl += 2)
            {
              for (prescale = 3; prescale < 256; prescale++)
                {
                  scl_low = (scl / 2) - 7;
                  scl_hi = (scl / 2) - 5;

                  computed_rate = src_freq / (prescale + 1);
                  computed_rate /= scl_low + 7 + scl_hi + 5;

                  if (frequency > computed_rate)
                    {
                      abs_error = frequency - computed_rate;
                    }
                  else
                    {
                      abs_error = computed_rate - frequency;
                    }

                  if (abs_error < best_error)
                    {
                      best_prescale = prescale;
                      best_scl_low = scl_low;
                      best_scl_hi = scl_hi;
                      best_error = abs_error;

                      if (abs_error == 0)
                        {
                          scl = 522;
                          break;
                        }
                    }
                }
            }

          am67_i2c_putreg(priv, AM67_I2C_PSC_OFFSET, best_prescale);

          am67_i2c_putreg(priv, AM67_I2C_SCLL_OFFSET, best_scl_low);

          am67_i2c_putreg(priv, AM67_I2C_SCLH_OFFSET, best_scl_hi);

          /* Re-enable I2C if it was enabled previously */

          if (men)
            {
              am67_i2c_modifyreg(priv, AM67_I2C_CON_OFFSET,
                                   0, I2C_CON_EN);
            }

          /* Save the new I2C frequency */

          priv->frequency = frequency;
        }
    }
}

/****************************************************************************
 * Name: am67_i2c_startmsg
 *
 * Description:
 *   Program the hardware for the next message segment and issue a
 *   (repeated) START.  Must only be called while the module is register-
 *   accessible: before any transfer has started, or from the ISR when
 *   ARDY is set.
 *
 ****************************************************************************/

static void am67_i2c_startmsg(struct am67_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = priv->msgv;
  uint32_t regval;

  /* Bookkeeping: this segment's buffer/count/flags, advance the list */

  priv->ptr   = msg->buffer;
  priv->dcnt  = msg->length;
  priv->flags = msg->flags;
  priv->msgv++;
  priv->msgc--;

  /* SA and DCOUNT before anything starts.  DCOUNT is written exactly
   * once per segment and never touched again while it runs.
   */

  am67_i2c_putreg(priv, AM67_I2C_SA_OFFSET, msg->addr);
  am67_i2c_putreg(priv, AM67_I2C_CNT_OFFSET, msg->length);

  /* Direction + master mode.  STP is deliberately NOT pre-armed here:
   * when priming from the ARDY window the master is holding an open
   * transaction, and writing STP=1 there generates an immediate STOP
   * (this is why the TI SDK never sets STOP in its restart path).  Every
   * segment therefore ends with the bus held, and the ISR issues the
   * final STOP explicitly once the last segment's data is done.
   */

  regval = I2C_CON_EN | I2C_CON_MST;
  if ((priv->flags & I2C_M_READ) == 0)
    {
      regval |= I2C_CON_TRX;
    }

  if ((priv->flags & I2C_M_TEN) != 0)
    {
      regval |= I2C_CON_XSA;
    }

  am67_i2c_putreg(priv, AM67_I2C_CON_OFFSET, regval);

#ifndef CONFIG_I2C_POLLED
  /* Arm this segment's interrupts: errors, ARDY (segment done), and the
   * data direction actually in use.  BF stays off until we need it.
   */

  am67_i2c_putreg(priv, AM67_I2C_IRQ_EN_CLR_OFFSET,
                  I2C_IRQ_XRDY | I2C_IRQ_RRDY | I2C_IRQ_BF);
  am67_i2c_putreg(priv, AM67_I2C_IRQ_EN_SET_OFFSET,
                  I2C_IRQ_AL | I2C_IRQ_NACK | I2C_IRQ_AERR | I2C_IRQ_ARDY |
                  (((priv->flags & I2C_M_READ) != 0) ?
                   I2C_IRQ_RRDY : I2C_IRQ_XRDY));
#endif

  /* Everything is described — now press go */

  if ((priv->flags & I2C_M_NOSTART) == 0)
    {
      am67_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgc);
      am67_i2c_modifyreg(priv, AM67_I2C_CON_OFFSET, 0, I2C_CON_STT);
    }
  else
    {
      am67_i2c_traceevent(priv, I2CEVENT_NOSTART, priv->msgc);
    }
}

/****************************************************************************
 * Name: am67_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ****************************************************************************/

static inline void am67_i2c_sendstop(struct am67_i2c_priv_s *priv)
{
  am67_i2c_modifyreg(priv, AM67_I2C_CON_OFFSET, 0, I2C_CON_STP);
}

/****************************************************************************
 * Name: am67_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status
 *
 ****************************************************************************/

static inline uint32_t
am67_i2c_getstatus(struct am67_i2c_priv_s *priv)
{
#ifndef CONFIG_I2C_POLLED
  return am67_i2c_getreg(priv, AM67_I2C_IRQ_STAT_OFFSET);
#else
  return am67_i2c_getreg(priv, AM67_I2C_IRQ_STAT_RAW_OFFSET);
#endif
}

/****************************************************************************
 * Name: am67_i2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int am67_i2c_isr_process(struct am67_i2c_priv_s *priv)
{
  uint32_t status = am67_i2c_getstatus(priv);
  uint32_t raw    = am67_i2c_getreg(priv, AM67_I2C_IRQ_STAT_RAW_OFFSET);
  uint32_t fatal  = raw & (I2C_IRQ_NACK | I2C_IRQ_AL | I2C_IRQ_AERR);

  /* Check for new trace setup */

  am67_i2c_tracenew(priv, status);

  /* Rung 1: fatal errors preempt everything.  Checked in the RAW status
   * so an error can never hide behind interrupt masking.
   */

  if (fatal != 0)
    {
      am67_i2c_traceevent(priv, I2CEVENT_ERROR, fatal);

      /* Release the bus and reset the transaction engine, mirroring the
       * TI SDK fatal-error path.  Without this a single NACK leaves the
       * master mid-transaction and every later transfer times out.
       */

      am67_i2c_sendstop(priv);

      am67_i2c_putreg(priv, AM67_I2C_IRQ_STAT_OFFSET, I2C_STS_CLEARMASK);
      am67_i2c_putreg(priv, AM67_I2C_BUF_OFFSET,
                      I2C_BUF_TXFIFO_CLR | I2C_BUF_RXFIFO_CLR);
      am67_i2c_putreg(priv, AM67_I2C_CNT_OFFSET, 0);

      priv->status = status | fatal;
      priv->msgv   = NULL;
      priv->dcnt   = 0;
      priv->ptr    = NULL;

#ifndef CONFIG_I2C_POLLED
      am67_i2c_putreg(priv, AM67_I2C_IRQ_EN_CLR_OFFSET, I2C_ICR_CLEARMASK);

      if (priv->intstate == INTSTATE_WAITING)
        {
          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#else
      priv->intstate = INTSTATE_DONE;
#endif
      return OK;
    }

  /* Rung 2: a byte arrived — drain it BEFORE any phase decision so a
   * pending restart/stop cannot strand it in the FIFO.
   */

  if ((status & I2C_IRQ_RRDY) != 0)
    {
      if (priv->dcnt > 0 && priv->ptr != NULL)
        {
          am67_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);
          *priv->ptr++ = am67_i2c_getreg(priv, AM67_I2C_DATA_OFFSET) &
                         I2C_DATA_MASK;
          priv->dcnt--;
        }
      else
        {
          /* Unexpected byte: consume it so the FIFO cannot wedge */

          am67_i2c_getreg(priv, AM67_I2C_DATA_OFFSET);
        }

      am67_i2c_putreg(priv, AM67_I2C_IRQ_STAT_OFFSET, I2C_IRQ_RRDY);
    }

  /* Rung 3: the transmitter wants a byte */

  if ((status & I2C_IRQ_XRDY) != 0)
    {
      if (priv->dcnt > 0 && priv->ptr != NULL)
        {
          am67_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
          am67_i2c_putreg(priv, AM67_I2C_DATA_OFFSET, *priv->ptr++);
          priv->dcnt--;
        }
#ifndef CONFIG_I2C_POLLED
      else
        {
          /* Nothing left in this segment's buffer */

          am67_i2c_putreg(priv, AM67_I2C_IRQ_EN_CLR_OFFSET, I2C_IRQ_XRDY);
        }
#endif

      am67_i2c_putreg(priv, AM67_I2C_IRQ_STAT_OFFSET, I2C_IRQ_XRDY);
    }

  /* Rung 4: segment finished (ARDY), or bus released (BF).  Only act if
   * this segment's data really is done — in polled mode the raw status
   * contains incidental BF/ARDY bits from past bus activity.
   */

  if ((status & (I2C_IRQ_ARDY | I2C_IRQ_BF)) != 0)
    {
      am67_i2c_putreg(priv, AM67_I2C_IRQ_STAT_OFFSET,
                      status & (I2C_IRQ_ARDY | I2C_IRQ_BF));

      if (priv->dcnt <= 0 && priv->msgv != NULL)
        {
          if (priv->msgc > 0)
            {
              /* More segments: reprogram now — ARDY means the counter is
               * idle and the hardware is holding the bus for us.
               */

              am67_i2c_traceevent(priv, I2CEVENT_STARTRESTART, priv->msgc);
              am67_i2c_startmsg(priv);
            }
          else if ((raw & I2C_IRQ_BB) != 0 &&
                   (status & I2C_IRQ_BF) == 0 &&
                   (priv->flags & I2C_M_NOSTOP) == 0)
            {
              /* All data done but the bus is still held: issue the final
               * STOP (legal here — this is the ARDY window) and wait for
               * the bus-free event.  Skip STP if a STOP is already in
               * flight; hardware clears the bit when the STOP completes.
               */

              if ((am67_i2c_getreg(priv, AM67_I2C_CON_OFFSET) &
                   I2C_CON_STP) == 0)
                {
                  am67_i2c_sendstop(priv);
                }

#ifndef CONFIG_I2C_POLLED
              am67_i2c_putreg(priv, AM67_I2C_IRQ_EN_SET_OFFSET, I2C_IRQ_BF);
#endif
            }
          else
            {
              /* Transfer complete */

              am67_i2c_traceevent(priv, I2CEVENT_STOP, 0);

#ifndef CONFIG_I2C_POLLED
              am67_i2c_putreg(priv, AM67_I2C_IRQ_EN_CLR_OFFSET,
                              I2C_ICR_CLEARMASK);

              if (priv->intstate == INTSTATE_WAITING)
                {
                  nxsem_post(&priv->sem_isr);
                  priv->intstate = INTSTATE_DONE;
                }
#else
              priv->intstate = INTSTATE_DONE;
#endif
              priv->msgv = NULL;
            }
        }
    }

  priv->status = status;
  return OK;
}

/****************************************************************************
 * Name: am67_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int am67_i2c_isr(int irq, void *context, void *arg)
{
  struct am67_i2c_priv_s *priv = (struct am67_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return am67_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: am67_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int am67_i2c_init(struct am67_i2c_priv_s *priv)
{
  /* Configure pins */

  am67_i2c_pinmux_init();

  /* Disable I2C module before reset */

  am67_i2c_putreg(priv, AM67_I2C_CON_OFFSET, 0);

  /* Apply soft reset */

  am67_i2c_putreg(priv, AM67_I2C_SYSC_OFFSET, I2C_SYSC_SRST);

  /* TI I2C requires CON.EN=1 for the internal reset to complete;
   * without it RST_DONE never asserts.
   */

  am67_i2c_modifyreg(priv, AM67_I2C_CON_OFFSET, 0, I2C_CON_EN);

  while (!(am67_i2c_getreg(priv, AM67_I2C_SYSS_OFFSET) &
           I2C_SYSS_RST_DONE))
    {
    }

  /* No-idle mode + keep both clocks active so the WKUP domain power
   * controller cannot gate WKUP_I2C0 clocks between transfers when the
   * CPU enters WFI.  Without this the peripheral resets between NSH
   * commands and the second transfer always fails.
   */

  am67_i2c_putreg(priv, AM67_I2C_SYSC_OFFSET,
                  I2C_SYSC_IDLE_NO | I2C_SYSC_CLK_BOTH);

  /* Force a frequency update; setclock temporarily disables EN to write
   * PSC/SCLL/SCLH, then re-enables it.
   */

  priv->frequency = 0;
  am67_i2c_setclock(priv, 100000);

#ifndef CONFIG_I2C_POLLED
  /* Attach ISRs */

  irq_attach(priv->config->irq, am67_i2c_isr, priv);
  up_enable_irq(priv->config->irq);
#endif

  /* Ensure module is enabled after setclock may have re-enabled it */

  am67_i2c_modifyreg(priv, AM67_I2C_CON_OFFSET, 0, I2C_CON_EN);

  /* Enable all wakeup sources.  The TI Linux driver does this explicitly
   * "to stop I2C freezing on WFI instruction" on all AM6x/J7 class
   * devices.  Without WE_ALL the I2C interrupt cannot wake the CPU from
   * WFI, so the second transfer hangs until sem_waitdone times out.
   */

  am67_i2c_putreg(priv, AM67_I2C_WE_OFFSET,
                  I2C_WE_AL   | I2C_WE_NACK | I2C_WE_ARDY | I2C_WE_DRDY |
                  I2C_WE_GC   | I2C_WE_STC  | I2C_WE_BF   | I2C_WE_AAS  |
                  I2C_WE_XUDF | I2C_WE_ROVR | I2C_WE_RDR  | I2C_WE_XDR);

  /* Free-running mode keeps the module active under a JTAG halt */

  am67_i2c_modifyreg(priv, AM67_I2C_SYSTEST_OFFSET, 0, I2C_SYSTEST_FREE);

  return OK;
}

/****************************************************************************
 * Name: am67_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int am67_i2c_deinit(struct am67_i2c_priv_s *priv)
{
  /* Disable I2C module */

  am67_i2c_putreg(priv, AM67_I2C_CON_OFFSET, 0);

  /* Apply soft reset */

  am67_i2c_putreg(priv, AM67_I2C_SYSC_OFFSET, I2C_SYSC_SRST);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  irq_detach(priv->config->irq);
#endif

  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: am67_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int am67_i2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count)
{
  struct am67_i2c_priv_s *priv = (struct am67_i2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Bring the hardware up on first use.  The bus is registered early during
   * board bring-up (before the console), but the I2C functional clock is
   * only enabled by the DM afterwards, so the reset is deferred to here.
   */

  if (!priv->inited)
    {
      am67_i2c_init(priv);
      priv->inited = true;
    }

  /* Wait for any STOP in progress */

  ret = -EBUSY;
  if (am67_i2c_sem_waitstop(priv))
    {
      /* Clear TX and RX FIFOs before each transfer.  Stale FIFO data from
       * the previous transfer can cause XRDY/RRDY to fire at unexpected
       * points in the state machine.  The TI Linux driver does this too.
       */

      am67_i2c_putreg(priv, AM67_I2C_BUF_OFFSET,
                      I2C_BUF_TXFIFO_CLR | I2C_BUF_RXFIFO_CLR);

      /* Clear any pending error interrupts */

      am67_i2c_putreg(priv, AM67_I2C_IRQ_STAT_OFFSET,
                        I2C_STS_CLEARMASK);
      am67_i2c_putreg(priv, AM67_I2C_IRQ_EN_CLR_OFFSET,
                        I2C_ICR_CLEARMASK);

      /* Old transfers are done */

      /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
       * overwrite stale data.
       */

      priv->dcnt  = 0;
      priv->ptr   = NULL;

      priv->msgv  = msgs;
      priv->msgc  = count;

      i2cinfo("Flags %x, len %d\n", msgs->flags, msgs->length);

      /* Reset I2C trace logic */

      am67_i2c_tracereset(priv);

      /* Set I2C clock frequency (only reprograms the divisors when the
       * requested frequency actually changes).
       */

      am67_i2c_setclock(priv, msgs->frequency);

      priv->status = 0;

      /* Wait for an ISR, if there was a timeout, fetch latest status to get
       * the BUSY flag.
       */

      if (am67_i2c_sem_waitdone(priv) < 0)
        {
          ret = -ETIMEDOUT;

          i2cerr("ERROR: Timed out: IRQ_RAW: status: 0x%" PRIx32 "\n",
                 priv->status);
        }

      /* Check for error status conditions */

      else if ((priv->status & I2C_IRQ_ERRORMASK) != 0)
        {
          /* I2C_IRQ_ERRORMASK is the OR of the following individual bits: */

          if (priv->status & I2C_IRQ_AL)
            {
              /* Arbitration Lost (master mode) */

              i2cerr("Arbitration lost\n");
              ret = -EAGAIN;
            }
          else if (priv->status & I2C_IRQ_NACK)
            {
              /* Acknowledge Failure */

              i2cerr("Ack failure\n");
              ret = -ENXIO;
            }
          else if (priv->status & (I2C_IRQ_XUDF | I2C_IRQ_ROVR))
            {
              /* Overrun/Underrun */

              i2cerr("Overrun/Underrun status\n");
              ret = -EIO;
            }
          else if (priv->status & I2C_IRQ_AERR)
            {
              /* Access Error in reception or transmission */

              i2cerr("Access Error\n");
              ret = -EPROTO;
            }
          else if (priv->status & I2C_IRQ_BB)
            {
              /* Bus busy Error */

              i2cerr("Bus busy error\n");
              ret = -EIO;
            }
          else
            {
              i2cerr("Unspecified error\n");
              ret = -EINTR;
            }
        }
      else
        {
          ret = OK;
        }

      /* Dump the trace result */

      am67_i2c_tracedump(priv);

      /* Ensure ISR happening after we finish can't overwrite any user data */

      priv->dcnt = 0;
      priv->ptr = NULL;
    }
  else
    {
      i2cerr("ERROR: Bus busy, raw status: 0x%" PRIx32 "\n",
             am67_i2c_getreg(priv, AM67_I2C_IRQ_STAT_RAW_OFFSET));
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *am67_i2cbus_initialize(int port)
{
  struct am67_i2c_priv_s *priv = NULL;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_AM67_I2C0
      case 0:
        priv = (struct am67_i2c_priv_s *)&am67_i2c0_priv;
        break;
#endif
#ifdef CONFIG_AM67_WKUP_I2C0
      case 2:
        priv = (struct am67_i2c_priv_s *)&am67_i2c2_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Reference count only.  The hardware is brought up lazily on the first
   * transfer (see am67_i2c_transfer): bring-up registers the bus early,
   * before the console, but the I2C functional clock is enabled by the DM
   * later, so resetting the module here would spin on RST_DONE forever and
   * hang boot.
   */

  nxmutex_lock(&priv->lock);
  priv->refs++;
  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: am67_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int am67_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct am67_i2c_priv_s *priv = (struct am67_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs > 0)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Disable power and other HW resource (GPIO's), only if it was ever
   * brought up (lazy init may never have run).
   */

  if (priv->inited)
    {
      am67_i2c_deinit(priv);

      /* Re-arm lazy bring-up so the next transfer re-runs am67_i2c_init(). */

      priv->inited = false;
    }

  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_AM67_I2C0 || CONFIG_AM67_I2C1 || CONFIG_AM67_WKUP_I2C0 */
