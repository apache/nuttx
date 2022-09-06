/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_i2c.c
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

#include "mips_internal.h"
#include "hardware/pic32mz_i2c.h"
#include "pic32mz_i2c.h"

/* At least one I2C peripheral must be enabled */

#ifdef CONFIG_PIC32MZ_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_PIC32MZ_I2CTIMEOSEC) && !defined(CONFIG_PIC32MZ_I2CTIMEOMS)
#  define CONFIG_PIC32MZ_I2CTIMEOSEC 0
#  define CONFIG_PIC32MZ_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_PIC32MZ_I2CTIMEOSEC)
#  define CONFIG_PIC32MZ_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_PIC32MZ_I2CTIMEOMS)
#  define CONFIG_PIC32MZ_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_PIC32MZ_I2CTIMEOTICKS
#  define CONFIG_PIC32MZ_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_PIC32MZ_I2CTIMEOSEC) +\
     MSEC2TICK(CONFIG_PIC32MZ_I2CTIMEOMS))
#endif

#ifndef CONFIG_PIC32MZ_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_PIC32MZ_I2C_DYNTIMEO_STARTSTOP \
            TICK2USEC(CONFIG_PIC32MZ_I2CTIMEOTICKS)
#endif

/* Debug ********************************************************************/

/* I2C event trace logic. */

#ifndef CONFIG_I2C_TRACE
#  define pic32mz_i2c_tracereset(p)
#  define pic32mz_i2c_tracenew(p,s)
#  define pic32mz_i2c_traceevent(p,e,a)
#  define pic32mz_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

#if defined(CONFIG_I2C_TRACE) && defined(CONFIG_I2C_POLLED)
#  warning Default trace events might not be enough during polling.
#endif

#ifdef CONFIG_I2C_SLAVE
#  error I2C slave logic is not supported yet for PIC32MZ
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum pic32mz_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Process state */

enum pic32mz_process_state_e
{
  PROCESS_STATE_SEND_ADDR = 0,
  PROCESS_STATE_SEND_DATA,
  PROCESS_STATE_ENABLE_READ,
  PROCESS_STATE_READ_DATA,
  PROCESS_STATE_FETCH_NEXT,
  PROCESS_STATE_TRANSFERT_DONE
};

/* Trace events */

enum pic32mz_trace_e
{
  I2CEVENT_NONE = 0,      /* No events have occurred with this status */
  I2CEVENT_SENDADDR,      /* Start/Master bit set and address sent, param = msgc */
  I2CEVENT_SENDBYTE,      /* Send byte, param = dcnt */
  I2CEVENT_RCVMODEEN,     /* Receive mode enabled, param = 0 */
  I2CEVENT_RCVBYTE,       /* Read more data, param = dcnt */
  I2CEVENT_NOSTART,       /* BTF on last byte with no restart, param = dcnt */
  I2CEVENT_STARTRESTART,  /* Last byte sent, re-starting, param = 0 */
  I2CEVENT_STOP,          /* Last byte sten, send stop, param = 0 */
  I2CEVENT_WAKEUP,        /* Everything is done, wake up caller, param = 0 */
  I2CEVENT_ERROR          /* Bus collision error, param = 0 */
};

/* Trace data */

struct pic32mz_trace_s
{
  uint32_t status;             /* I2C 32-bit SR2|SR1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum pic32mz_trace_e event;  /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  clock_t time;                /* First of event or first status */
};

/* I2C Device hardware configuration */

struct pic32mz_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
  uint8_t mode;               /* Master or Slave mode */
#ifndef CONFIG_I2C_POLLED
  uint32_t ev_irq;            /* Event IRQ */
  uint32_t er_irq;            /* Error IRQ */
#endif
};

/* I2C Device Private Data */

struct pic32mz_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct pic32mz_i2c_config_s *config;

  int refs;                       /* Reference count */
  mutex_t lock;                   /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;                  /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;      /* Interrupt handshake (see enum
                                   * pic32mz_intstate_e) */
  volatile uint8_t process_state; /* State of the isr process */

  uint8_t msgc;                   /* Message count */
  struct i2c_msg_s *msgv;         /* Message list */
  uint8_t *ptr;                   /* Current message buffer */
  uint32_t frequency;             /* Current I2C frequency */
  int dcnt;                       /* Current message length */
  uint16_t flags;                 /* Current message flags */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct pic32mz_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t
pic32mz_i2c_getreg(struct pic32mz_i2c_priv_s *priv,
                   uint8_t offset);
static inline void pic32mz_i2c_putreg(struct pic32mz_i2c_priv_s *priv,
                                      uint8_t offset, uint32_t value);
static inline void pic32mz_i2c_modifyreg(struct pic32mz_i2c_priv_s *priv,
                                         uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits);

#ifdef CONFIG_PICM32MZ_I2C_DYNTIMEO
static uint32_t pic32mz_i2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_PIC32MZ_I2C_DYNTIMEO */

static inline int
pic32mz_i2c_sem_waitdone(struct pic32mz_i2c_priv_s *priv);
static inline void
pic32mz_i2c_sem_waitidle(struct pic32mz_i2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void pic32mz_i2c_tracereset(struct pic32mz_i2c_priv_s *priv);
static void pic32mz_i2c_tracenew(struct pic32mz_i2c_priv_s *priv,
                                 uint32_t status);
static void
pic32mz_i2c_traceevent(struct pic32mz_i2c_priv_s *priv,
                       enum pic32mz_trace_e event, uint32_t parm);
static void pic32mz_i2c_tracedump(struct pic32mz_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static inline int
pic32mz_i2c_setbaudrate(struct pic32mz_i2c_priv_s *priv,
                        uint32_t frequency);
static inline void
pic32mz_i2c_send_start(struct pic32mz_i2c_priv_s *priv);
static inline void
pic32mz_i2c_send_stop(struct pic32mz_i2c_priv_s *priv);
static inline void
pic32mz_i2c_send_repeatedstart(struct pic32mz_i2c_priv_s *priv);
static inline void pic32mz_i2c_send_ack(struct pic32mz_i2c_priv_s *priv,
                                        bool ack);
static inline void pic32mz_i2c_transmitbyte(struct pic32mz_i2c_priv_s *priv,
                                            uint8_t data);
static inline uint32_t
pic32mz_i2c_receivebyte(struct pic32mz_i2c_priv_s *priv);

static inline uint32_t
pic32mz_i2c_getstatus(struct pic32mz_i2c_priv_s *priv);
static inline bool
pic32mz_i2c_master_inactive(struct pic32mz_i2c_priv_s *priv);

static int pic32mz_i2c_isr_process(struct pic32mz_i2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
static int pic32mz_i2c_isr(int irq, void *context, void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int pic32mz_i2c_init(struct pic32mz_i2c_priv_s *priv);
static int pic32mz_i2c_deinit(struct pic32mz_i2c_priv_s *priv);

static int pic32mz_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int pic32mz_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Trace events strings */

#ifdef CONFIG_I2C_TRACE
static const char *g_trace_names[] =
{
  "NONE       ",
  "SENDADDR   ",
  "SENDBYTE   ",
  "RCVMODEEN  ",
  "RCVBYTE    ",
  "NOSTART    ",
  "RESTART    ",
  "STOP       ",
  "WAKEUP     ",
  "ERROR      "
};
#endif

/* I2C interface */

static const struct i2c_ops_s pic32mz_i2c_ops =
{
  .transfer      = pic32mz_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset         = pic32mz_i2c_reset,
#endif
};

/* I2C device structures */

#ifdef CONFIG_PIC32MZ_I2C1
static const struct pic32mz_i2c_config_s pic32mz_i2c1_config =
{
  .base          = PIC32MZ_I2C1_K1BASE,
  .scl_pin       = GPIO_I2C1_SCL,
  .sda_pin       = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = PIC32MZ_IRQ_I2C1M,
  .er_irq        = PIC32MZ_IRQ_I2C1COL
#endif
};

static struct pic32mz_i2c_priv_s pic32mz_i2c1_priv =
{
  .ops           = &pic32mz_i2c_ops,
  .config        = &pic32mz_i2c1_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_PIC32MZ_I2C2
static const struct pic32mz_i2c_config_s pic32mz_i2c2_config =
{
  .base          = PIC32MZ_I2C2_K1BASE,
  .scl_pin       = GPIO_I2C2_SCL,
  .sda_pin       = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = PIC32MZ_IRQ_I2C2M,
  .er_irq        = PIC32MZ_IRQ_I2C2COL
#endif
};

static struct pic32mz_i2c_priv_s pic32mz_i2c2_priv =
{
  .ops           = &pic32mz_i2c_ops,
  .config        = &pic32mz_i2c2_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_PIC32MZ_I2C3
static const struct pic32mz_i2c_config_s pic32mz_i2c3_config =
{
  .base          = PIC32MZ_I2C3_K1BASE,
  .scl_pin       = GPIO_I2C3_SCL,
  .sda_pin       = GPIO_I2C3_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = PIC32MZ_IRQ_I2C3M,
  .er_irq        = PIC32MZ_IRQ_I2C3COL
#endif
};

static struct pic32mz_i2c_priv_s pic32mz_i2c3_priv =
{
  .ops           = &pic32mz_i2c_ops,
  .config        = &pic32mz_i2c3_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_PIC32MZ_I2C4
static const struct pic32mz_i2c_config_s pic32mz_i2c4_config =
{
  .base          = PIC32MZ_I2C4_K1BASE,
  .scl_pin       = GPIO_I2C4_SCL,
  .sda_pin       = GPIO_I2C4_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = PIC32MZ_IRQ_I2C4M,
  .er_irq        = PIC32MZ_IRQ_I2C4COL
#endif
};

static struct pic32mz_i2c_priv_s pic32mz_i2c4_priv =
{
  .ops           = &pic32mz_i2c_ops,
  .config        = &pic32mz_i2c4_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_PIC32MZ_I2C5
static const struct pic32mz_i2c_config_s pic32mz_i2c5_config =
{
  .base          = PIC32MZ_I2C5_K1BASE,
  .scl_pin       = GPIO_I2C5_SCL,
  .sda_pin       = GPIO_I2C5_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = PIC32MZ_IRQ_I2C5M,
  .er_irq        = PIC32MZ_IRQ_I2C5COL
#endif
};

static struct pic32mz_i2c_priv_s pic32mz_i2c5_priv =
{
  .ops           = &pic32mz_i2c_ops,
  .config        = &pic32mz_i2c5_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void pic32mz_i2c_traceclear(struct pic32mz_i2c_priv_s *priv)
{
  struct pic32mz_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void pic32mz_i2c_tracereset(struct pic32mz_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  pic32mz_i2c_traceclear(priv);
}

static void pic32mz_i2c_tracenew(struct pic32mz_i2c_priv_s *priv,
                                uint32_t status)
{
  struct pic32mz_trace_s *trace = &priv->trace[priv->tndx];

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

      pic32mz_i2c_traceclear(priv);
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

static void pic32mz_i2c_traceevent(struct pic32mz_i2c_priv_s *priv,
                                   enum pic32mz_trace_e event, uint32_t parm)
{
  struct pic32mz_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event  = event;
      trace->parm   = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
        {
          i2cerr("ERROR: Trace table overflow\n");
          return;
        }

      priv->tndx++;
      pic32mz_i2c_traceclear(priv);
    }
}

static void pic32mz_i2c_tracedump(struct pic32mz_i2c_priv_s *priv)
{
  struct pic32mz_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %ld\n",
         (long)(clock_systime_ticks() - priv->start_time));

  for (i = 0; i < priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. STATUS: %04x COUNT: %3d EVENT: %s(%2d) "
             "PARM: %08x TIME: %d\n",
             i + 1, trace->status, trace->count, g_trace_names[trace->event],
             trace->event, trace->parm, trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: pic32mz_i2c_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t
pic32mz_i2c_getreg(struct pic32mz_i2c_priv_s *priv, uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: pic32mz_i2c_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void pic32mz_i2c_putreg(struct pic32mz_i2c_priv_s *priv,
                                      uint8_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: pic32mz_i2c_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void pic32mz_i2c_modifyreg(struct pic32mz_i2c_priv_s *priv,
                                         uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: pic32mz_i2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_I2C_DYNTIMEO
static uint32_t pic32mz_i2c_toticks(int msgc, struct i2c_msg_s *msgs)
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

  return USEC2TICK(CONFIG_PIC32MZ_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: pic32mz_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 * There are two versions of this function.  The first is included when using
 * interrupts while the second is used if polling (CONFIG_I2C_POLLED=y).
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int
pic32mz_i2c_sem_waitdone(struct pic32mz_i2c_priv_s *priv)
{
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  /* Signal the interrupt handler that we are waiting */

  priv->intstate = INTSTATE_WAITING;

  do
    {
      /* Wait until either the transfer is complete or the timeout expires */

#ifdef CONFIG_PIC32MZ_I2C_DYNTIMEO
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                       pic32mz_i2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_PIC32MZ_I2CTIMEOTICKS);
#endif
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by
           * nxsem_tickwait_uninterruptible.
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to idle. */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts. */

  up_disable_irq(priv->config->ev_irq);
  up_disable_irq(priv->config->er_irq);

  leave_critical_section(flags);

  return ret;
}
#else
static inline int
pic32mz_i2c_sem_waitdone(struct pic32mz_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_PIC32MZ_I2C_DYNTIMEO
  timeout = pic32mz_i2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_PIC32MZ_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_tickwait_uninterruptible() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  start = clock_systime_ticks();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      pic32mz_i2c_isr_process(priv);
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cinfo("intstate: %d elapsed: %ld threshold: %ld status: %04x\n",
          priv->intstate, (long)elapsed, (long)timeout, priv->status);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/****************************************************************************
 * Name: pic32mz_i2c_sem_waitidle
 *
 * Description:
 *   Wait for the bus to be in idle.
 *
 ****************************************************************************/

static inline void
pic32mz_i2c_sem_waitidle(struct pic32mz_i2c_priv_s *priv)
{
  uint32_t timeout;
  uint32_t start;
  uint32_t elapsed;
  uint32_t con;
  uint32_t stat;

  /* Select a timeout */

#ifdef CONFIG_PIC32MZ_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_PIC32MZ_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_PIC32MZ_I2CTIMEOTICKS;
#endif

  start = clock_systime_ticks();
  do
    {
      elapsed = clock_systime_ticks() - start;

      /* The bus is idle if the five least significant bits of I2CxCON
       * are cleared and the I2CxSTAT<TRSTAT> flag is cleared.
       */

      con = pic32mz_i2c_getreg(priv, PIC32MZ_I2C_CON_OFFSET);
      stat = pic32mz_i2c_getreg(priv, PIC32MZ_I2C_STAT_OFFSET);
      if (((con & I2C_CON_IDLEMASK) == 0) && ((stat & I2C_STAT_TRSTAT) == 0))
        {
          return;
        }
    }
  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the bus still in idle */

  i2cinfo("Timeout with I2CxCON: %04x I2CxSTAT: %04x\n", con, stat);
}

/****************************************************************************
 * Name: pic32mz_i2c_isr_process
 *
 * Description:
 *   Common interrupt service routine
 *
 ****************************************************************************/

static int pic32mz_i2c_isr_process(struct pic32mz_i2c_priv_s *priv)
{
  uint32_t status;

  status = pic32mz_i2c_getstatus(priv);

  /* Check for new trace setup */

  pic32mz_i2c_tracenew(priv, status);

  switch (priv->process_state)
    {
    /* The process starts from this state after a call to i2c_transfer.
     * It may return here in the case of a write/read transaction,
     * to send the address with the READ bit set.
     */

    case PROCESS_STATE_SEND_ADDR:

      pic32mz_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgc);

      if (priv->msgc > 0 && priv->msgv != NULL)
        {
          priv->ptr = priv->msgv->buffer;
          priv->dcnt = priv->msgv->length;
          priv->flags = priv->msgv->flags;

          /* Send the address byte and set the next state to either
           * read or transmit the data.
           */

          if (priv->flags & I2C_M_READ)
            {
              pic32mz_i2c_transmitbyte(priv, I2C_M_READ | priv->msgv->addr);

              priv->process_state = PROCESS_STATE_ENABLE_READ;
            }
          else
            {
              pic32mz_i2c_transmitbyte(priv, priv->msgv->addr);

              priv->process_state = PROCESS_STATE_SEND_DATA;
            }
        }
      else
        {
#ifndef CONFIG_I2C_POLLED
          up_clrpend_irq(priv->config->ev_irq);
#endif
        }
      break;

    /* This state is reached either after sending the address to the slave,
     * or, in the case of multi-byte buffer, after sending a byte.
     * We should first check that the previous transmission is not in
     * progress, and that the slave had acknowledged it.
     */

    case PROCESS_STATE_SEND_DATA:

      pic32mz_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);

      /* No transmission is in progress. */

      if ((status & I2C_STAT_TRSTAT) == 0)
        {
          /* ACK received from the slave. */

          if ((status & I2C_STAT_ACKSTAT) == 0)
            {
              /* We need to keep one byte to send before we leave this state.
               * This way we can trigger an interrupt and move to the next
               * state.
               */

              if (priv->dcnt > 1)
                {
                  pic32mz_i2c_transmitbyte(priv, *priv->ptr++);

                  priv->dcnt--;
                }
              else
                {
                  pic32mz_i2c_transmitbyte(priv, *priv->ptr++);

                  priv->dcnt--;

                  priv->process_state = PROCESS_STATE_FETCH_NEXT;
                }
            }
        }
      break;

    /* This state is reached after sending the address to the slave with
     * the read bit set, or, in the case of multi-byte transfer,
     * after reading the first byte.
     * We should first check that the previous transmission is not in
     * progress, and that the slave had acknowledged it.
     */

    case PROCESS_STATE_ENABLE_READ:

      pic32mz_i2c_traceevent(priv, I2CEVENT_RCVMODEEN, 0);

      /* No transmit is in progress. */

      if ((status & I2C_STAT_TRSTAT) == 0)
        {
          /* ACK received from the slave. */

          if ((status & I2C_STAT_ACKSTAT) == 0)
            {
              /* The master logic should be inactive before
               * attempting to enable receive mode.
               */

              if (pic32mz_i2c_master_inactive(priv))
                {
                  pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONSET_OFFSET,
                                     I2C_CON_RCEN);

                  priv->process_state = PROCESS_STATE_READ_DATA;
                }
            }
        }
      break;

    /* This state reads a byte from the receive buffer.
     * If there are more than one byte to read,
     * it should go back to the previous state to enable
     * the receive mode.
     */

    case PROCESS_STATE_READ_DATA:

      pic32mz_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

      /* Is data available in the receiver buffer? */

      if ((status & I2C_STAT_RBF) != 0)
        {
          /* Read and send an ACK */

          if (priv->dcnt > 1)
            {
#ifdef CONFIG_I2C_POLLED
              irqstate_t flags = enter_critical_section();
#endif

              *priv->ptr++ = pic32mz_i2c_receivebyte(priv);

              priv->dcnt--;

              /* The master logic should be inactive before
               * attempting to issue an ACK.
               */

              if (pic32mz_i2c_master_inactive(priv))
                {
                  pic32mz_i2c_send_ack(priv, true);
                }

#ifdef CONFIG_I2C_POLLED
              leave_critical_section(flags);
#endif
              /* Go back and re-enable read mode to handle the rest of
               * the data.
               * It is cleared by the hardware at the end of the eighth bit.
               */

              priv->process_state = PROCESS_STATE_ENABLE_READ;
            }

          /* Last byte, read and send a NACK */

          else
            {
#ifdef CONFIG_I2C_POLLED
              irqstate_t flags = enter_critical_section();
#endif
              *priv->ptr++ = pic32mz_i2c_receivebyte(priv);

              priv->dcnt--;

              /* The master logic should be inactive before
               * attempting to issue a NACK.
               */

              if (pic32mz_i2c_master_inactive(priv))
                {
                  pic32mz_i2c_send_ack(priv, false);
                }

#ifdef CONFIG_I2C_POLLED
              leave_critical_section(flags);
#endif
              priv->process_state = PROCESS_STATE_FETCH_NEXT;
            }
        }

      break;

    /* In this state we fetch the next mssage.
     * Increment to next pointer and decrement message count.
     * If we have an other set of data we will:
     *      - Issue a repeated start (I2C_M_NOSTOP flag set).
     *      - Continue with no start (I2C_M_NOSTART flag set).
     *      - Issue a start (No flag set).
     * If no more data to send, issue a stop.
     */

    case PROCESS_STATE_FETCH_NEXT:

      priv->msgv++;
      priv->msgc--;

      if (priv->msgc > 0 && priv->msgv != NULL)
        {
          /* If the previous message had the I2C_M_NOSTOP flag set,
           * this implies that we should issue a repeated start.
           * (Note: priv->flags still has the previous flags.)
           */

          if (priv->flags & I2C_M_NOSTOP)
            {
              pic32mz_i2c_traceevent(priv, I2CEVENT_STARTRESTART, 0);

              /* The bus should be in idle before issuing a repeated start. */

              if ((pic32mz_i2c_master_inactive(priv)) &&
                  (status & I2C_STAT_TRSTAT) == 0)
                {
                  pic32mz_i2c_send_repeatedstart(priv);
                  priv->process_state = PROCESS_STATE_SEND_ADDR;
                }
            }

          /* If the new message has the I2C_M_NOSTART flag set,
           * this means that it's a continuation of the same transfer.
           * We can't just move back to SEND_DATA as we need an interrupt.
           * So one byte must be sent from here first.
           */

          else if (priv->msgv->flags & I2C_M_NOSTART)
            {
              priv->ptr = priv->msgv->buffer;
              priv->dcnt = priv->msgv->length;
              priv->flags = priv->msgv->flags;

              pic32mz_i2c_traceevent(priv, I2CEVENT_NOSTART, priv->dcnt);

              if ((status & I2C_STAT_TRSTAT) == 0)
                {
                  if ((status & I2C_STAT_ACKSTAT) == 0)
                    {
                      /* We have more than one byte.
                       * Send the first one, this will trigger an interrupt
                       * the rest will get sent later.
                       */

                      if (priv->dcnt > 1)
                        {
                          pic32mz_i2c_transmitbyte(priv, *priv->ptr++);

                          priv->dcnt--;

                          priv->process_state = PROCESS_STATE_SEND_DATA;
                        }

                      /* Send the only byte we have and stay in this state
                       * to fetch the next message.
                       */

                      else
                        {
                          pic32mz_i2c_transmitbyte(priv, *priv->ptr++);

                          priv->dcnt--;
                        }
                    }
                }
            }

          /* If neither the I2C_M_NOSTOP nor the I2C_M_NOSTART is set,
           * just issue a start and let the isr process the data.
           */

          else
            {
              /* The bus should be in idle before issuing a start. */

              if ((pic32mz_i2c_master_inactive(priv)) &&
                  (status & I2C_STAT_TRSTAT) == 0)
                {
                  pic32mz_i2c_send_start(priv);

                  priv->process_state = PROCESS_STATE_SEND_ADDR;
                }
            }
        }
      else
        {
          /* The stop should be initiated here,
           * as there is no other way to trigger an interrupt.
           */

          pic32mz_i2c_traceevent(priv, I2CEVENT_STOP, 0);

          /* The master logic should be inactive before
           * attempting to issue a STOP.
           */

          if (pic32mz_i2c_master_inactive(priv))
            {
              pic32mz_i2c_send_stop(priv);

              priv->process_state = PROCESS_STATE_TRANSFERT_DONE;
            }
        }
      break;

    /* Arriving here, the transfer is complete.
     * Wake up any thread that has been waiting for this event.
     */

    case PROCESS_STATE_TRANSFERT_DONE:

      pic32mz_i2c_traceevent(priv, I2CEVENT_WAKEUP, 0);

      if (priv->msgv)
        {
          /* Is there a thread waiting for this event (there should be) */

          if (priv->intstate == INTSTATE_WAITING)
          {
#ifndef CONFIG_I2C_POLLED
            nxsem_post(&priv->sem_isr);
#endif
            priv->intstate = INTSTATE_DONE;
          }

          /* Mark that we have stopped with this transaction. */

          priv->msgv = NULL;
        }

      break;

    default:

      /* Nothing goes here! */

      break;
    }

  /* Clear the master interrupt flag. */

#ifndef CONFIG_I2C_POLLED
  if (up_pending_irq(priv->config->ev_irq))
    {
      up_clrpend_irq(priv->config->ev_irq);
    }
#endif

  /* If an error interrupt has accured. */

#ifndef CONFIG_I2C_POLLED
  if (up_pending_irq(priv->config->er_irq))
    {
      pic32mz_i2c_traceevent(priv, I2CEVENT_ERROR, 0);

      up_clrpend_irq(priv->config->er_irq);
    }
#endif

  priv->status = status;

  return OK;
}

/****************************************************************************
 * Name: pic32mz_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int pic32mz_i2c_isr(int irq, void *context, void *arg)
{
  struct pic32mz_i2c_priv_s *priv = (struct pic32mz_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return pic32mz_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: pic32mz_i2c_setbaudrate
 *
 * Description:
 *   Calculates the value of the baudrate.
 *
 ****************************************************************************/

static inline int
pic32mz_i2c_setbaudrate(struct pic32mz_i2c_priv_s *priv,
                        uint32_t frequency)
{
  uint32_t baudrate;

  if (frequency != priv->frequency)
    {
      /* BOARD_PBCLK and frequency are both given in Hz. */

      baudrate = (uint32_t)(((BOARD_PBCLK2 / (2 * frequency)) -
                        (BOARD_PBCLK2 / 10000000) - 2));

      /* Values of 0x0 and 0x1 are prohibited. */

      if (baudrate == 0x0 || baudrate == 0x1)
        {
          return ERROR;
        }
      else
        {
          pic32mz_i2c_putreg(priv, PIC32MZ_I2C_BRG_OFFSET, baudrate);
          priv->frequency = frequency;

          /* Enable Slew Rate Control when operating on High Speed mode. */

          if (frequency == 400000)
            {
              pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONCLR_OFFSET,
                                 I2C_CON_DISSLW);
            }
          else
            {
              pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONSET_OFFSET,
                                 I2C_CON_DISSLW);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mz_i2c_send_start
 *
 * Description:
 *   Initiate a start condition.
 *
 ****************************************************************************/

static inline void
pic32mz_i2c_send_start(struct pic32mz_i2c_priv_s *priv)
{
  pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONSET_OFFSET, I2C_CON_SEN);

  /* To avoid bus collision during polling. */

#ifdef CONFIG_I2C_POLLED
  while ((pic32mz_i2c_getreg(priv, PIC32MZ_I2C_CON_OFFSET) &
          I2C_CON_SEN) != 0);
#endif
}

/****************************************************************************
 * Name: pic32mz_i2c_send_stop
 *
 * Description:
 *   Initiate a stop condition.
 *
 ****************************************************************************/

static inline void pic32mz_i2c_send_stop(struct pic32mz_i2c_priv_s *priv)
{
  pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONSET_OFFSET, I2C_CON_PEN);

  /* To avoid bus collision during polling. */

#ifdef CONFIG_I2C_POLLED
  while ((pic32mz_i2c_getreg(priv, PIC32MZ_I2C_CON_OFFSET) &
          I2C_CON_PEN) != 0);
#endif
}

/****************************************************************************
 * Name: pic32mz_i2c_send_repeatedstart
 *
 * Description:
 *   Initiate a repeated start condition.
 *
 ****************************************************************************/

static inline void
pic32mz_i2c_send_repeatedstart(struct pic32mz_i2c_priv_s *priv)
{
  pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONSET_OFFSET, I2C_CON_RSEN);

  /* To avoid bus collision during polling. */

#ifdef CONFIG_I2C_POLLED
  while ((pic32mz_i2c_getreg(priv, PIC32MZ_I2C_CON_OFFSET) &
          I2C_CON_RSEN) != 0);
#endif
}

/****************************************************************************
 * Name: pic32mz_i2c_send_ack
 *
 * Description:
 *   Issue an ACK or a NACK.
 *
 ****************************************************************************/

static inline void pic32mz_i2c_send_ack(struct pic32mz_i2c_priv_s *priv,
                                        bool ack)
{
  if (ack)
    {
      pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONCLR_OFFSET, I2C_CON_ACKDT);
    }
  else
    {
      pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONSET_OFFSET, I2C_CON_ACKDT);
    }

  pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONSET_OFFSET, I2C_CON_ACKEN);

  /* To avoid bus collision during polling. */

#ifdef CONFIG_I2C_POLLED
  while ((pic32mz_i2c_getreg(priv, PIC32MZ_I2C_CON_OFFSET) &
          I2C_CON_ACKEN) != 0);
#endif
}

/****************************************************************************
 * Name: pic32mz_i2c_transmitbyte
 *
 * Description:
 *   Transmit a byte.
 *
 ****************************************************************************/

static inline void pic32mz_i2c_transmitbyte(struct pic32mz_i2c_priv_s *priv,
                                            uint8_t data)
{
  pic32mz_i2c_putreg(priv, PIC32MZ_I2C_TRN_OFFSET, data);

  /* To avoid bus collision during polling. */

#ifdef CONFIG_I2C_POLLED
  while ((pic32mz_i2c_getreg(priv, PIC32MZ_I2C_STAT_OFFSET) &
          I2C_STAT_TRSTAT) != 0);
#endif
}

/****************************************************************************
 * Name: pic32mz_i2c_receivebyte
 *
 * Description:
 *   Receive a byte.
 *
 ****************************************************************************/

static inline uint32_t
pic32mz_i2c_receivebyte(struct pic32mz_i2c_priv_s *priv)
{
  uint32_t val;

  /* To avoid bus collision during polling. */

#ifdef CONFIG_I2C_POLLED
  while ((pic32mz_i2c_getreg(priv, PIC32MZ_I2C_STAT_OFFSET) &
          I2C_CON_RCEN) != 0);
#endif

  val = pic32mz_i2c_getreg(priv, PIC32MZ_I2C_RCV_OFFSET);

  return val;
}

/****************************************************************************
 * Name: pic32mz_i2c_master_inactive
 *
 * Description:
 *   Check if the bus is inactive.
 *   No start, stop, ACK is in progress.
 *
 ****************************************************************************/

static inline bool
pic32mz_i2c_master_inactive(struct pic32mz_i2c_priv_s *priv)
{
  uint32_t con;

  con = pic32mz_i2c_getreg(priv, PIC32MZ_I2C_CON_OFFSET);

  return ((con & I2C_CON_IDLEMASK) ? false:true);
}

/****************************************************************************
 * Name: pic32mz_i2c_getstatus
 *
 * Description:
 *   Get the STAT register.
 *
 ****************************************************************************/

static inline uint32_t
pic32mz_i2c_getstatus(struct pic32mz_i2c_priv_s *priv)
{
  return pic32mz_i2c_getreg(priv, PIC32MZ_I2C_STAT_OFFSET);
}

/****************************************************************************
 * Name: pic32mz_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int pic32mz_i2c_init(struct pic32mz_i2c_priv_s *priv)
{
  /* Force a frequency update */

  priv->frequency = 0;
  pic32mz_i2c_setbaudrate(priv, 100000);

  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->ev_irq, pic32mz_i2c_isr, priv);
  irq_attach(priv->config->er_irq, pic32mz_i2c_isr, priv);
#endif

  /* Enable the I2C hardware.
   * The I2C hardware, when enabled, takes control over the pins.
   * The module overrides the port state and direction.
   * No need to configure the pins here.
   */

  pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONSET_OFFSET, I2C_CON_ON);

  return OK;
}

/****************************************************************************
 * Name: pic32mz_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int pic32mz_i2c_deinit(struct pic32mz_i2c_priv_s *priv)
{
  /* Disable I2C */

  pic32mz_i2c_putreg(priv, PIC32MZ_I2C_CONCLR_OFFSET, I2C_CON_ON);

  /* Disable and detach ISRs */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->ev_irq);
  up_disable_irq(priv->config->er_irq);
  irq_detach(priv->config->ev_irq);
  irq_detach(priv->config->er_irq);
#endif

  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int pic32mz_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count)
{
  struct pic32mz_i2c_priv_s *priv = (struct pic32mz_i2c_priv_s *)dev;
  uint32_t status = 0;
  int ret;

  /* Acquire the mutex. */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for the bus to be in an idle state. */

  pic32mz_i2c_sem_waitidle(priv);

  /* Clear any pending error interrupts. */

#ifndef CONFIG_I2C_POLLED
  up_clrpend_irq(priv->config->er_irq);
#endif

  /* Old transfers are done */

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  pic32mz_i2c_tracereset(priv);

  /* Set the baudrate. */

  pic32mz_i2c_setbaudrate(priv, priv->frequency);

#ifndef CONFIG_I2C_POLLED

  /* Enable interrupts here so when we send the start condition
   * below the ISR will fire if the data was sent.
   */

  up_enable_irq(priv->config->ev_irq);
  up_enable_irq(priv->config->er_irq);
#endif

  priv->status = 0;

  pic32mz_i2c_send_start(priv);

  /* Indicate to the process where to start. */

  priv->process_state = PROCESS_STATE_SEND_ADDR;

  if (pic32mz_i2c_sem_waitdone(priv) < 0)
    {
      status = pic32mz_i2c_getstatus(priv);
      ret = -ETIMEDOUT;

      i2cerr("ERROR: Timed out: CON: 0x%04x status: 0x%04x\n",
             pic32mz_i2c_getreg(priv, PIC32MZ_I2C_CON_OFFSET), status);
    }
  else
    {
      status = pic32mz_i2c_getstatus(priv);
    }

  /* Check for errors. */

  if ((status & I2C_STAT_BCL))
    {
      ret = -EIO;
    }

  else if ((status & I2C_STAT_IWCOL) != 0)
    {
      ret = -EIO;
    }

  else if ((status & I2C_STAT_I2COV) != 0)
    {
      ret = -EIO;
    }

  /* Dump the trace result */

  pic32mz_i2c_tracedump(priv);

  /* Ensure ISR happening after we finish can't overwrite any user data */

  priv->dcnt = 0;
  priv->ptr = NULL;

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: pic32mz_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int pic32mz_i2c_reset(struct i2c_master_s *dev)
{
  struct pic32mz_i2c_priv_s *priv = (struct pic32mz_i2c_priv_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t frequency;
  int ret = ERROR;

  DEBUGASSERT(dev);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  pic32mz_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  pic32mz_configgpio(priv->scl);
  pic32mz_configgpio(priv->sda);

  /* Let SDA go high */

  pic32mz_gpiowrite(priv->sda, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!pic32mz_gpioread(priv->sda))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!pic32mz_gpioread(priv->scl))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      pic32mz_gpiowrite(priv->scl, 0);
      up_udelay(10);

      /* Drive SCL high again */

      pic32mz_gpiowrite(priv->scl, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  pic32mz_gpiowrite(priv->sda, 0);
  up_udelay(10);
  pic32mz_gpiowrite(priv->scl, 0);
  up_udelay(10);
  pic32mz_gpiowrite(priv->scl, 1);
  up_udelay(10);
  pic32mz_gpiowrite(priv->sda, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  pic32mz_unconfiggpio(priv->sda);
  pic32mz_unconfiggpio(priv->scl);

  /* Re-init the port */

  pic32mz_i2c_init(priv);

  /* Restore the frequency */

  pic32mz_i2c_setclock(priv, frequency);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  nxmutex_unlock(&priv->lock);
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *pic32mz_i2cbus_initialize(int port)
{
  struct pic32mz_i2c_priv_s * priv = NULL;
  irqstate_t flags;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_PIC32MZ_I2C1
    case 1:
      priv = (struct pic32mz_i2c_priv_s *)&pic32mz_i2c1_priv;
      break;
#endif

#ifdef CONFIG_PIC32MZ_I2C2
    case 2:
      priv = (struct pic32mz_i2c_priv_s *)&pic32mz_i2c2_priv;
      break;
#endif

#ifdef CONFIG_PIC32MZ_I2C3
    case 3:
      priv = (struct pic32mz_i2c_priv_s *)&pic32mz_i2c3_priv;
      break;
#endif
#ifdef CONFIG_PIC32MZ_I2C4
    case 4:
      priv = (struct pic32mz_i2c_priv_s *)&pic32mz_i2c4_priv;
      break;
#endif

#ifdef CONFIG_PIC32MZ_I2C5
    case 5:
      priv = (struct pic32mz_i2c_priv_s *)&pic32mz_i2c5_priv;
      break;
#endif

    default:
      return NULL;
    }

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  flags = enter_critical_section();

  if ((volatile int)priv->refs++ == 0)
    {
      pic32mz_i2c_init(priv);
    }

  leave_critical_section(flags);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: pic32mz_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int pic32mz_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct pic32mz_i2c_priv_s *priv = (struct pic32mz_i2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  /* Disable I2C hardware */

  pic32mz_i2c_deinit(priv);

  return OK;
}

#endif /* CONFIG_PIC32MZ_I2C */
