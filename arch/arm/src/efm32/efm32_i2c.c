/****************************************************************************
 * arch/arm/src/efm32/efm32_i2c.c
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

/* Supports:
 *  - Master operation, 100 kHz (standard) and 400 kHz (full speed)
 * TODO:
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver,
 *     obtained by the i2c_init(); it extends the Device structure from the
 *     nuttx/i2c/i2c.h; Instance points to OPS, to common I2C Hardware
 *     private data and contains its own private data, as frequency,
 *     address, mode of operation (in the future)
 *  - Private: Private data of an I2C Hardware
 *
 */

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

#include "arm_internal.h"
#include "efm32_gpio.h"
#include "hardware/efm32_cmu.h"
#include "hardware/efm32_i2c.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_EFM32_I2C0) || defined(CONFIG_EFM32_I2C1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

#if !defined(CONFIG_EFM32_EFM32GG)
#  warning "Tested only on EFM32GG family"
#endif

#ifndef CONFIG_I2C_POLLED
#  warning "Not tested with interrupt"
#endif

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_EFM32_I2CTIMEOSEC) && !defined(CONFIG_EFM32_I2CTIMEOMS)
#  define CONFIG_EFM32_I2CTIMEOSEC 0
#  define CONFIG_EFM32_I2CTIMEOMS  500/* Default is 500 milliseconds */
#elif !defined(CONFIG_EFM32_I2CTIMEOSEC)
#  define CONFIG_EFM32_I2CTIMEOSEC 0  /* User provided milliseconds */
#elif !defined(CONFIG_EFM32_I2CTIMEOMS)
#  define CONFIG_EFM32_I2CTIMEOMS  0  /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_EFM32_I2CTIMEOTICKS
#  define CONFIG_EFM32_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_EFM32_I2CTIMEOSEC) + MSEC2TICK(CONFIG_EFM32_I2CTIMEOMS))
#endif

#ifndef CONFIG_EFM32_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_EFM32_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_EFM32_I2CTIMEOTICKS)
#endif

/* Macros to convert a I2C pin to a GPIO output */

#define I2C_OUTPUT (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_CNF_OUTOD)

#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define efm32_i2c_tracereset(p)
#  define efm32_i2c_tracenew(p)
#  define efm32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/* Error flags indicating I2C transfer has failed somehow.
 * Notice that I2C_IF_TXOF (transmit overflow) is not really possible with
 * this SW supporting master mode. Likewise for I2C_IF_RXUF (receive
 * underflow).  RXUF is only likely to occur with this SW if using a
 * debugger peeking into RXDATA register. Thus, we ignore those types of
 * fault.
 */

#define I2C_IF_ERRORS    (I2C_IF_BUSERR | I2C_IF_ARBLOST)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C result state */

enum efm32_result_e
{
  I2CRESULT_SWFAULT = -5,     /* SW fault. */
  I2CRESULT_USAGEFAULT = -4,  /* Usage fault. */
  I2CRESULT_ARBLOST = -3,     /* Arbitration lost during transfer. */
  I2CRESULT_BUSERR = -2,      /* Bus error during transfer (misplaced
                               * START/STOP). */
  I2CRESULT_NACK = -1,        /* NACK received during transfer. */
  I2CRESULT_INPROGRESS = 0,   /* Transfer in progress. */
  I2CRESULT_DONE = 1,         /* Transfer completed successfully. */
  I2CRESULT_NONE = 2          /* Nothing. */
};

enum efm32_i2cstate_e
{
  I2CSTATE_NONE = 0,          /* Send start + (first part of) address. */
  I2CSTATE_STARTADDRSEND,     /* Send start + (first part of) address. */
  I2CSTATE_ADDRWFACKNACK,     /* Wait for ACK/NACK on (first part of)
                               * address. */
  I2CSTATE_ADDRWF2NDACKNACK,  /* Wait for ACK/NACK on second part of 10 bit
                               * address. */
  I2CSTATE_RSTARTADDRSEND,    /* Send repeated start + (first part of)
                               * address. */
  I2CSTATE_RADDRWFACKNACK,    /* Wait for ACK/NACK on address sent after
                               * repeated start. */
  I2CSTATE_DATASEND,          /* Send data. */
  I2CSTATE_DATAWFACKNACK,     /* Wait for ACK/NACK on data sent. */
  I2CSTATE_WFDATA,            /* Wait for data. */
  I2CSTATE_WFSTOPSENT,        /* Wait for STOP to have been transmitted. */
  I2CSTATE_DONE               /* Transfer completed successfully. */
};

/* Trace data */

struct efm32_trace_s
{
  uint32_t i2c_state;         /* I2C state machine current state */
  uint32_t i2c_reg_state;     /* I2C register I2Cx_STATES */
  uint32_t i2c_reg_if;        /* I2C register I2Cx_IF */
  uint32_t count;             /* Interrupt count when status change */
  clock_t time;               /* First of event or first status */
  int dcnt;                   /* Interrupt count when status change */
};

/* I2C Device hardware configuration */

struct efm32_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t route;             /* route location of I2C */
  uint32_t reset_bit;         /* Reset bit */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  uint32_t irq;               /* Event IRQ */
#endif
};

/* I2C Device Private Data */

struct efm32_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct efm32_i2c_config_s *config;

  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif

  volatile int8_t result;      /* result of transfer */

  uint8_t i2c_state;           /* i2c state machine */
  uint32_t i2c_reg_if;         /* Current state of I2Cx_IF register. */
  uint32_t i2c_reg_state;      /* Current state of I2Cx_STATES register. */

  int addr;                    /* Message address */
  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  int dcnt;                    /* Current message length */
  uint32_t flags;              /* Current message flags */
  uint32_t frequency;          /* Current I2C frequency */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                   /* Trace array index */
  clock_t start_time;         /* Time when the trace was started */

  /* The actual trace data */

  struct efm32_trace_s trace[CONFIG_I2C_NTRACE];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t efm32_i2c_getreg(struct efm32_i2c_priv_s *priv,
                                        uint8_t offset);
static inline void efm32_i2c_putreg(struct efm32_i2c_priv_s *priv,
                                    uint8_t offset, uint32_t value);
static inline void efm32_i2c_modifyreg(struct efm32_i2c_priv_s *priv,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits);

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
static uint32_t efm32_i2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_EFM32_I2C_DYNTIMEO */

static inline int efm32_i2c_sem_waitdone(struct efm32_i2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void efm32_i2c_tracereset(struct efm32_i2c_priv_s *priv);
static void efm32_i2c_tracenew(struct efm32_i2c_priv_s *priv);
static void efm32_i2c_tracedump(struct efm32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void efm32_i2c_setclock(struct efm32_i2c_priv_s *priv,
                               uint32_t frequency);

static int efm32_i2c_isr_process(struct efm32_i2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
static int efm32_i2c_isr(int irq, void *context, void *arg);
#endif /* !CONFIG_I2C_POLLED */

static void efm32_i2c_hwreset(struct efm32_i2c_priv_s *priv);
static int efm32_i2c_init(struct efm32_i2c_priv_s *priv);
static int efm32_i2c_deinit(struct efm32_i2c_priv_s *priv);
static int efm32_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int efm32_i2c_reset(struct i2c_master_s *dev);
#endif

#ifdef CONFIG_I2C_TRACE
static const char *efm32_i2c_state_str(int i2c_state);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s efm32_i2c_ops =
{
  .transfer = efm32_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = efm32_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_EFM32_I2C0
static const struct efm32_i2c_config_s efm32_i2c0_config =
{
  .base = EFM32_I2C0_BASE,
  .clk_bit = CMU_HFPERCLKEN0_I2C0,
  .route = BOARD_I2C1_ROUTE_LOCATION,
  .scl_pin = BOARD_I2C0_SCL,
  .sda_pin = BOARD_I2C0_SDA,
#ifndef CONFIG_I2C_POLLED
  .irq = EFM32_IRQ_I2C0
#endif
};

static struct efm32_i2c_priv_s efm32_i2c0_priv =
{
  .ops = &efm32_i2c_ops,
  .config = &efm32_i2c0_config,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .result = I2CRESULT_NONE,
  .msgc = 0,
  .msgv = NULL,
  .ptr = NULL,
  .dcnt = 0,
  .flags = 0,
};
#endif

#ifdef CONFIG_EFM32_I2C1
static const struct efm32_i2c_config_s efm32_i2c1_config =
{
  .base = EFM32_I2C1_BASE,
  .clk_bit = CMU_HFPERCLKEN0_I2C1,
  .route = BOARD_I2C1_ROUTE_LOCATION,
  .scl_pin = BOARD_I2C1_SCL,
  .sda_pin = BOARD_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .irq = EFM32_IRQ_I2C1
#endif
};

static struct efm32_i2c_priv_s efm32_i2c1_priv =
{
  .ops = &efm32_i2c_ops,
  .config = &efm32_i2c1_config,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .result = I2CRESULT_NONE,
  .msgc = 0,
  .msgv = NULL,
  .ptr = NULL,
  .dcnt = 0,
  .flags = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t efm32_i2c_getreg(struct efm32_i2c_priv_s *priv,
                                        uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: efm32_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void efm32_i2c_putreg(struct efm32_i2c_priv_s *priv,
                                    uint8_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: efm32_i2c_modifyreg
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void efm32_i2c_modifyreg(struct efm32_i2c_priv_s *priv,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: efm32_i2c_state_str
 *
 * Description:
 *   Convert i2c_state into corresponding text.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static const char *efm32_i2c_state_str(int i2c_state)
{
  switch (i2c_state)
    {
    case I2CSTATE_NONE:
      return "Send start + (first part of) address.";
    case I2CSTATE_STARTADDRSEND:
      return "Send start + (first part of) address.";
    case I2CSTATE_ADDRWFACKNACK:
      return "Wait for ACK/NACK on (first part of) address.";
    case I2CSTATE_ADDRWF2NDACKNACK:
      return "Wait for ACK/NACK on second part of 10 bit address.";
    case I2CSTATE_RSTARTADDRSEND:
      return "Send repeated start + (first part of) address.";
    case I2CSTATE_RADDRWFACKNACK:
      return "Wait for ACK/NACK on address sent after repeated start.";
    case I2CSTATE_DATASEND:
      return "Send data.";
    case I2CSTATE_DATAWFACKNACK:
      return "Wait for ACK/NACK on data sent.";
    case I2CSTATE_WFDATA:
      return "Wait for data.";
    case I2CSTATE_WFSTOPSENT:
      return "Wait for STOP to have been transmitted.";
    case I2CSTATE_DONE:
      return "Transfer completed successfully.";
    default:
      return "Unknown state!";
    }
}
#endif

/****************************************************************************
 * Name: efm32_i2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
static uint32_t efm32_i2c_toticks(int msgc, struct i2c_msg_s *msgs)
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

  return USEC2TICK(CONFIG_EFM32_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: efm32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int efm32_i2c_sem_waitdone(struct efm32_i2c_priv_s *priv)
{
  int ret;

  do
    {
      /* Enable I2C interrupts */

      efm32_i2c_putreg(priv, EFM32_I2C_IEN_OFFSET, I2C_IF_NACK | I2C_IF_ACK |
                       I2C_IF_MSTOP | I2C_IF_RXDATAV | I2C_IF_ERRORS);

      /* Wait until either the transfer is complete or the timeout expires */

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                         efm32_i2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_EFM32_I2CTIMEOTICKS);
#endif

      /* Disable I2C interrupts */

      efm32_i2c_putreg(priv, EFM32_I2C_IEN_OFFSET, 0);

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

  while (priv->result == I2CRESULT_INPROGRESS);

  i2cinfo("result: %s elapsed: %d threshold: %d i2c_state %s "
          "I2Cx_STATES: %08x I2Cx_IF: %08x\n",
          efm32_i2c_result_str(priv->result), elapsed, timeout,
          efm32_i2c_state_str(priv->i2c_state), priv->i2c_reg_state,
          priv->i2c_reg_if);

  return ret;
}
#else
static inline int efm32_i2c_sem_waitdone(struct efm32_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;

  /* Get the timeout value */

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
  timeout = efm32_i2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_EFM32_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE: Interrupts are
   * currently disabled but will be temporarily re-enabled below when
   * nxsem_tickwait_uninterruptible() sleeps.
   */

  start = clock_systime_ticks();

  do
    {
      /* Poll by simply calling the timer interrupt handler until it reports
       * that it is done.
       */

      efm32_i2c_isr_process(priv);

      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;
    }

  /* Loop until the transfer is complete. */

  while ((priv->result == I2CRESULT_INPROGRESS) && elapsed < timeout);

  i2cinfo("result: %s elapsed: %d threshold: %d i2c_state %s "
          "I2Cx_STATES: %08x I2Cx_IF: %08x\n",
          efm32_i2c_result_str(priv->result), elapsed, timeout,
          efm32_i2c_state_str(priv->i2c_state), priv->i2c_reg_state,
          priv->i2c_reg_if);

  if (elapsed < timeout)
    {
      return OK;
    }

  return -1;
}
#endif

/****************************************************************************
 * Name: efm32_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void efm32_i2c_traceclear(struct efm32_i2c_priv_s *priv)
{
  struct efm32_trace_s *trace = &priv->trace[priv->tndx];

  trace->i2c_state = I2CSTATE_NONE; /* I2C current state of state machine */
  trace->i2c_reg_if = 0;            /* I2C I2Cx_IF register */
  trace->i2c_reg_state = 0;         /* I2C I2Cx_STATES register */
  trace->count = 0;                 /* Interrupt count when status change */
  trace->time = 0;                  /* Time of first status or event */
}

static void efm32_i2c_tracereset(struct efm32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx = 0;
  priv->start_time = clock_systime_ticks();
  efm32_i2c_traceclear(priv);
}

static void efm32_i2c_tracenew(struct efm32_i2c_priv_s *priv)
{
  struct efm32_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized? Has the states changed? */

  if ((trace->count == 0) ||
      (priv->i2c_reg_if != trace->i2c_reg_if) ||
      (priv->i2c_reg_state != trace->i2c_reg_state) ||
      (priv->dcnt != trace->dcnt) ||
      (priv->i2c_state != trace->i2c_state))
    {
      /* Yes.. Was it the states changed? */

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

      efm32_i2c_traceclear(priv);
      trace->i2c_state = priv->i2c_state;
      trace->i2c_reg_state = priv->i2c_reg_state;
      trace->i2c_reg_if = priv->i2c_reg_if;
      trace->count = 1;
      trace->dcnt = priv->dcnt;
      trace->time = clock_systime_ticks();
    }
  else
    {
      /* Just increment the count of times that we have seen this states */

      trace->count++;
    }
}

static void efm32_i2c_tracedump(struct efm32_i2c_priv_s *priv)
{
  struct efm32_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %ld\n",
         (long)(clock_systime_ticks() - priv->start_time));

  for (i = 0; i < priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. I2Cx_STATE: %08x I2Cx_PENDING: %08x dcnt %3d COUNT: %3d "
             "STATE: %s(%2d) TIME: %ld\n",
             i + 1, trace->i2c_reg_state, trace->i2c_reg_if, trace->dcnt,
             trace->count, efm32_i2c_state_str(trace->i2c_state),
             trace->i2c_state, (long)(trace->time - priv->start_time));
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: efm32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void efm32_i2c_setclock(struct efm32_i2c_priv_s *priv,
                               uint32_t frequency)
{
  uint32_t div;

  /* Has the I2C bus frequency changed? */

  if (priv->frequency != frequency)
    {
      /* Set the CLHR (clock low to high ratio). */

      efm32_i2c_modifyreg(priv, EFM32_I2C_CTRL_OFFSET, _I2C_CTRL_CLHR_MASK,
#if defined(CONFIG_EFM32_I2C_CLHR_FAST)
                          _I2C_CTRL_CLHR_FAST       /* Ratio is 11:3 */
#elif defined(CONFIG_EFM32_I2C_CLHR_ASYMMETRIC)
                          _I2C_CTRL_CLHR_ASYMMETRIC /* Ratio is 6:3 */
#else                                /* CLHR STANDARD */
                          _I2C_CTRL_CLHR_STANDARD   /* Ratio is 4:4 */
#endif
                          << _I2C_CTRL_CLHR_SHIFT);

      /* Frequency is given by:
       *
       *   fSCL = fHFPERCLK/((Nlow + Nhigh)(DIV + 1) + 4),
       *
       * thus
       *
       *   DIV = ((fHFPERCLK - 4fSCL)/((Nlow + Nhigh)fSCL)) - 1
       */

#if defined(CONFIG_EFM32_I2C_CLHR_FAST)
#  define n (11 + 6)          /* Ratio is 11:3 */
#elif defined(CONFIG_EFM32_I2C_CLHR_ASYMMETRIC)
#  define n (6 + 3)           /* Ratio is 6:3 */
#else                         /* CLHR STANDARD */
#  define n ( 4 + 4)          /* Ratio is 4:4 */
#endif

      div = (BOARD_HFPERCLK_FREQUENCY - (4 * frequency)) / (n * frequency);

      /* Clock divisor must be at least 1 in slave mode according to
       * reference manual (in which case there is normally no need to set
       * bus frequency).
       */

      if ((efm32_i2c_getreg(priv, EFM32_I2C_CTRL_OFFSET) & I2C_CTRL_SLAVE) &&
          !div)
        {
          div = 1;
        }

      DEBUGASSERT(div <= (_I2C_CLKDIV_DIV_MASK >> _I2C_CLKDIV_DIV_SHIFT));

      efm32_i2c_putreg(priv, EFM32_I2C_CLKDIV_OFFSET, div);

      /* Save the new I2C frequency */

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: efm32_i2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int efm32_i2c_isr_process(struct efm32_i2c_priv_s *priv)
{
  for (; ; )
    {
      int regval;

      priv->i2c_reg_if = efm32_i2c_getreg(priv, EFM32_I2C_IF_OFFSET);
      priv->i2c_reg_state = efm32_i2c_getreg(priv, EFM32_I2C_STATE_OFFSET);

      /* Check for new trace setup */

      efm32_i2c_tracenew(priv);

      /* If some sort of fault, abort transfer. */

      if (priv->i2c_reg_if & I2C_IF_ERRORS)
        {
          if (priv->i2c_reg_if & I2C_IF_ARBLOST)
            {
              /* If arbitration fault, it indicates either a slave device
               * not responding as expected, or other master which is not
               * supported by this SW.
               */

              priv->result = I2CRESULT_ARBLOST;
            }
          else if (priv->i2c_reg_if & I2C_IF_BUSERR)
            {
              /* A bus error indicates a misplaced start or stop, which
               * should not occur in master mode controlled by this SW.
               */

              priv->result = I2CRESULT_BUSERR;
            }

          /* If error situation occurred, it is difficult to know exact
           * cause and how to resolve. It will be up to a wrapper to
           * determine how to handle a fault/recovery if possible.
           */

          priv->i2c_state = I2CSTATE_DONE;
          goto done;
        }

      switch (priv->i2c_state)
        {
        /* Send first start+address (first byte if 10 bit) */

        case I2CSTATE_STARTADDRSEND:
          if (priv->flags & I2C_M_TEN)
            {
              if (priv->flags & I2C_M_READ)
                {
                  regval = I2C_READADDR10H(priv->addr);
                }
              else
                {
                  regval = I2C_WRITEADDR10H(priv->addr);
                }
            }
          else
            {
              if (priv->flags & I2C_M_READ)
                {
                  regval = I2C_READADDR8(priv->addr);
                }
              else
                {
                  regval = I2C_WRITEADDR8(priv->addr);
                }
            }

          if ((priv->flags & I2C_M_READ) && (priv->dcnt == 1))
            {
              efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_NACK);
            }

          /* Data not transmitted until START sent */

          efm32_i2c_putreg(priv, EFM32_I2C_TXDATA_OFFSET, regval);
          efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_START);
          priv->i2c_state = I2CSTATE_ADDRWFACKNACK;

          goto done;

        /* Wait for ACK/NACK on address (first byte if 10 bit) */

        case I2CSTATE_ADDRWFACKNACK:
          if (priv->i2c_reg_if & I2C_IF_NACK)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_NACK);
              efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_STOP);
              priv->result = I2CRESULT_NACK;
              priv->i2c_state = I2CSTATE_WFSTOPSENT;
            }
          else if (priv->i2c_reg_if & I2C_IF_ACK)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_ACK);

              /* If 10 bit address, send 2nd byte of address. */

              if (priv->flags & I2C_M_TEN)
                {
                  if (priv->flags & I2C_M_READ)
                    {
                      regval = I2C_READADDR10L(priv->addr);
                    }
                  else
                    {
                      regval = I2C_WRITEADDR10L(priv->addr);
                    }

                  efm32_i2c_putreg(priv, EFM32_I2C_TXDATA_OFFSET, regval);
                  priv->i2c_state = I2CSTATE_ADDRWF2NDACKNACK;
                }
              else
                {
                  /* Determine whether receiving or sending data */

                  if (priv->flags & I2C_M_READ)
                    {
                      priv->i2c_state = I2CSTATE_WFDATA;
                    }
                  else
                    {
                      priv->i2c_state = I2CSTATE_DATASEND;
                      continue;
                    }
                }
            }

          goto done;

        /* Wait for ACK/NACK on second byte of 10 bit address */

        case I2CSTATE_ADDRWF2NDACKNACK:
          if (priv->i2c_reg_if & I2C_IF_NACK)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_NACK);
              efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_STOP);
              priv->result = I2CRESULT_NACK;
              priv->i2c_state = I2CSTATE_WFSTOPSENT;
            }
          else if (priv->i2c_reg_if & I2C_IF_ACK)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_ACK);

              /* If using plain read sequence with 10 bit address, switch to
               * send repeated start.
               */

              if (priv->flags & I2C_M_READ)
                {
                  priv->i2c_state = I2CSTATE_RSTARTADDRSEND;
                }

              /* Otherwise expected to write 0 or more bytes */

              else
                {
                  priv->i2c_state = I2CSTATE_DATASEND;
                }

              continue;
            }

          goto done;

        /* Send repeated start+address */

        case I2CSTATE_RSTARTADDRSEND:
          if (priv->flags & I2C_M_TEN)
            {
              if (priv->flags & I2C_M_READ)
                {
                  regval = I2C_READADDR10H(priv->addr);
                }
              else
                {
                  regval = I2C_WRITEADDR10H(priv->addr);
                }
            }
          else
            {
              if (priv->flags & I2C_M_READ)
                {
                  regval = I2C_READADDR8(priv->addr);
                }
              else
                {
                  regval = I2C_WRITEADDR8(priv->addr);
                }
            }

          if ((priv->flags & I2C_M_READ) && (priv->dcnt == 1))
            {
              efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_NACK);
            }

          priv->i2c_state = I2CSTATE_RADDRWFACKNACK;

          /* We have to write START cmd first since repeated start, otherwise
           * data would be sent first.
           */

          efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_START);
          efm32_i2c_putreg(priv, EFM32_I2C_TXDATA_OFFSET, regval);
          goto done;

        /* Wait for ACK/NACK on repeated start+address
         * (first byte if 10 bit)
         */

        case I2CSTATE_RADDRWFACKNACK:
          if (priv->i2c_reg_if & I2C_IF_NACK)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_NACK);
              efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_STOP);
              priv->result = I2CRESULT_NACK;
              priv->i2c_state = I2CSTATE_WFSTOPSENT;
            }
          else if (priv->i2c_reg_if & I2C_IF_ACK)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_ACK);

              /* Determine whether receiving or sending data */

              if (priv->flags & I2C_M_READ)
                {
                  priv->i2c_state = I2CSTATE_WFDATA;
                }
              else
                {
                  priv->i2c_state = I2CSTATE_DATASEND;
                  continue;
                }
            }

          goto done;

        /* Send a data byte to slave */

        case I2CSTATE_DATASEND:

          /* Reached end of data buffer? */

          if (priv->dcnt == 0)
            {
              if (priv->msgc == 0)
                {
                  /* No more message part */

                  efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_STOP);
                  priv->i2c_state = I2CSTATE_WFSTOPSENT;
                  goto done;
                }

              /* Move to next message part */

              priv->ptr = priv->msgv->buffer;
              priv->dcnt = priv->msgv->length;
              priv->flags = priv->msgv->flags;
              priv->addr = priv->msgv->addr;
              priv->msgv++;
              priv->msgc--;

              /* Send byte continue with/without restart ? */

              if (!(priv->flags & I2C_M_NOSTART))
                {
                  priv->i2c_state = I2CSTATE_RSTARTADDRSEND;
                  continue;
                }
            }

          /* Send byte */

          efm32_i2c_putreg(priv, EFM32_I2C_TXDATA_OFFSET, *priv->ptr++);
          priv->dcnt--;
          priv->i2c_state = I2CSTATE_DATAWFACKNACK;
          goto done;

        /* Wait for ACK/NACK from slave after sending data to it */

        case I2CSTATE_DATAWFACKNACK:
          if (priv->i2c_reg_if & I2C_IF_NACK)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_NACK);
              efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_STOP);
              priv->result = I2CRESULT_NACK;
              priv->i2c_state = I2CSTATE_WFSTOPSENT;
            }
          else if (priv->i2c_reg_if & I2C_IF_ACK)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_ACK);
              priv->i2c_state = I2CSTATE_DATASEND;
              continue;
            }

          goto done;

        /* Wait for data from slave */

        case I2CSTATE_WFDATA:
          if (priv->i2c_reg_if & I2C_IF_RXDATAV)
            {
              /* Must read out data in order to not block further progress */

              regval = efm32_i2c_getreg(priv, EFM32_I2C_RXDATA_OFFSET);

              /* Make sure not storing beyond end of buffer just in case */

              if (priv->dcnt > 0)
                {
                  *(priv->ptr++) = regval;
                  priv->dcnt--;
                }

              /* If we have read all requested data, then the sequence should
               * end
               */

              if (priv->dcnt == 0)
                {
                  priv->i2c_state = I2CSTATE_WFSTOPSENT;
                  efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_STOP);
                }
              else
                {
                  /* Send ACK and wait for next byte */

                  efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_ACK);

                  if (priv->dcnt == 1)
                    {
                      /* If there is more than one byte to receive and this
                       * is the next to last byte we need to transmit the
                       * NAK now, before receiving the last byte.
                       */

                      efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET,
                                       I2C_CMD_NACK);
                    }
                }
            }

          goto done;

        /* Wait for STOP to have been sent */

        case I2CSTATE_WFSTOPSENT:
          if (priv->i2c_reg_if & I2C_IF_MSTOP)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_MSTOP);
              priv->i2c_state = I2CSTATE_DONE;
            }

          goto done;

        /* Unexpected state, SW fault */

        default:
          priv->result = I2CRESULT_SWFAULT;
          priv->i2c_state = I2CSTATE_DONE;
          goto done;
        }
    }

done:
  if (priv->i2c_state == I2CSTATE_DONE)
    {
#ifndef CONFIG_I2C_POLLED
      nxsem_post(&priv->sem_isr);
#endif
      /* Disable interrupt sources when done */

      efm32_i2c_putreg(priv, EFM32_I2C_IEN_OFFSET, 0);

      /* Update result unless some fault already occurred */

      if (priv->result == I2CRESULT_INPROGRESS)
        {
          priv->result = I2CRESULT_DONE;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: efm32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int efm32_i2c_isr(int irq, void *context, void *arg)
{
  struct efm32_i2c_priv_s *priv = (struct efm32_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return efm32_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: efm32_i2c_hwreset
 *
 * Description:
 *   Reset I2C to same state as after a HW reset.
 *
 ****************************************************************************/

static void efm32_i2c_hwreset(struct efm32_i2c_priv_s *priv)
{
  efm32_i2c_putreg(priv, EFM32_I2C_CTRL_OFFSET,
                   _I2C_CTRL_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_CLKDIV_OFFSET,
                   _I2C_CLKDIV_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_SADDR_OFFSET,
                   _I2C_SADDR_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_SADDRMASK_OFFSET,
                   _I2C_SADDRMASK_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_IEN_OFFSET,
                   _I2C_IEN_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET,
                   _I2C_IFC_MASK);

  /* Do not reset route register, setting should be done independently */
}

/****************************************************************************
 * Name: efm32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *   Prepare and start an I2C transfer (single master mode only).
 *
 ****************************************************************************/

static int efm32_i2c_init(struct efm32_i2c_priv_s *priv)
{
  int regval;

  /* Power-up and configure GPIOs */

  /* Enable power and reset the peripheral */

  /* Enable I2C Clock */

  modifyreg32(EFM32_CMU_HFPERCLKEN0, 0, priv->config->clk_bit);

  /* Eeset all register */

  efm32_i2c_hwreset(priv);

  /* Configure pins */

  if (efm32_configgpio(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (efm32_configgpio(priv->config->sda_pin) < 0)
    {
      return ERROR;
    }

  /* Set Route */

  regval = priv->config->route << _I2C_ROUTE_LOCATION_SHIFT;

  /* and enable pins */

  regval |= I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN;

  /* Apply it */

  efm32_i2c_putreg(priv, EFM32_I2C_ROUTE_OFFSET, regval);

  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->irq, efm32_i2c_isr, priv);
  up_enable_irq(priv->config->irq);
#endif

  efm32_i2c_setclock(priv, 100000);

  /* Enable I2C */

  efm32_i2c_putreg(priv, EFM32_I2C_CTRL_OFFSET, I2C_CTRL_EN);

  /* Send abort CMD to be done after reset if i2c peripheral timeout is not
   * used
   */

  efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_ABORT);

  return OK;
}

/****************************************************************************
 * Name: efm32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int efm32_i2c_deinit(struct efm32_i2c_priv_s *priv)
{
  /* Disable I2C */

  efm32_i2c_hwreset(priv);

  /* Unconfigure GPIO pins */

  efm32_unconfiggpio(priv->config->scl_pin);
  efm32_configgpio(priv->config->sda_pin);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  irq_detach(priv->config->irq);
#endif

  /* Disable clocking */

  modifyreg32(EFM32_CMU_HFPERCLKEN0, priv->config->clk_bit, 0);
  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int efm32_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct efm32_i2c_priv_s *priv = (struct efm32_i2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(count > 0);

  if (count <= 0 || msgs == NULL)
    {
      return -EINVAL;
    }

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt = 0;
  priv->ptr  = NULL;
  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  efm32_i2c_tracereset(priv);

  /* Set I2C clock frequency (on change it toggles I2C_CR1_PE !).
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  efm32_i2c_setclock(priv, msgs->frequency);

  /* Prepare for a transfer */

  priv->result = I2CRESULT_INPROGRESS;
  priv->i2c_state = I2CSTATE_STARTADDRSEND;
  priv->i2c_reg_if = 0;
  priv->i2c_reg_state = 0;

  /* Get run-time data */

  priv->ptr = priv->msgv->buffer;
  priv->dcnt = priv->msgv->length;
  priv->flags = priv->msgv->flags;
  priv->addr = priv->msgv->addr;
  priv->msgv++;
  priv->msgc--;

  /* Ensure buffers are empty */

  efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET,
                   I2C_CMD_CLEARPC | I2C_CMD_CLEARTX);
  if (efm32_i2c_getreg(priv, EFM32_I2C_IF_OFFSET) & I2C_IF_RXDATAV)
    {
      efm32_i2c_getreg(priv, EFM32_I2C_RXDATA_OFFSET);
    }

  /* Clear all pending interrupts prior to starting transfer. */

  efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, _I2C_IFC_MASK);

  /* Call once isr to start state machine. I2C interrupt are disabled and
   * will be enabled in efm32_i2c_sem_waitdone if CONFIG_I2C_POLLED is NOT
   * defined.
   */

  efm32_i2c_isr_process(priv);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get the
   * BUSY flag.
   */

  if (efm32_i2c_sem_waitdone(priv) < 0)
    {
      ret = -ETIMEDOUT;

      i2cerr("ERROR: Timed out: I2Cx_STATE: 0x%04x I2Cx_STATUS: 0x%08x\n",
              efm32_i2c_getreg(priv, EFM32_I2C_STATE_OFFSET),
              efm32_i2c_getreg(priv, EFM32_I2C_STATUS_OFFSET));

      /* Abort */

      efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_ABORT);
    }
  else
    {
      /* Check for error status conditions */

      switch (priv->result)
        {
          /* Arbitration lost during transfer. */

          case I2CRESULT_ARBLOST:
              ret = -EAGAIN;
              break;

          /* NACK received during transfer. */

          case I2CRESULT_NACK:
              ret = -ENXIO;
              break;

          /* SW fault. */

          case I2CRESULT_SWFAULT:
              ret = -EIO;
              break;

          /* Usage fault. */

          case I2CRESULT_USAGEFAULT:
              ret = -EINTR;
              break;

          /* Bus error during transfer (misplaced START/STOP).
           * I2C Bus is for some reason busy
           */

          case I2CRESULT_BUSERR:
              ret = -EBUSY;
              break;
        }
    }

  /* Dump the trace result */

  efm32_i2c_tracedump(priv);

  /* Ensure that any ISR happening after we finish can't overwrite any user
   * data
   */

  priv->result = I2CRESULT_NONE;
  priv->dcnt = 0;
  priv->ptr = NULL;

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: efm32_i2c_reset
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
int efm32_i2c_reset(struct i2c_master_s *dev)
{
  struct efm32_i2c_priv_s *priv = (struct efm32_i2c_priv_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  int ret;

  DEBUGASSERT(dev);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* De-init the port */

  efm32_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  efm32_configgpio(scl_gpio);
  efm32_configgpio(sda_gpio);

  /* Let SDA go high */

  efm32_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!efm32_gpioread(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished. If the bus
       * never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!efm32_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      efm32_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      efm32_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave state machines. */

  efm32_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  efm32_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  efm32_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  efm32_gpiowrite(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  efm32_unconfiggpio(sda_gpio);
  efm32_unconfiggpio(scl_gpio);

  /* Re-init the port */

  efm32_i2c_init(priv);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *efm32_i2cbus_initialize(int port)
{
  struct efm32_i2c_priv_s *priv = NULL;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_EFM32_I2C0
    case 0:
      priv = (struct efm32_i2c_priv_s *)&efm32_i2c0_priv;
      break;
#endif

#ifdef CONFIG_EFM32_I2C1
    case 1:
      priv = (struct efm32_i2c_priv_s *)&efm32_i2c1_priv;
      break;
#endif

    default:
      return NULL;
    }

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      efm32_i2c_init(priv);
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: efm32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int efm32_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct efm32_i2c_priv_s *priv = (struct efm32_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Disable power and other HW resource (GPIO's) */

  efm32_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_EFM32_I2C0 || CONFIG_EFM32_I2C1 */
