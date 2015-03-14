/************************************************************************************
 * arch/arm/src/efm32/efm32_i2c.c
 * EFM32 I2C Hardware Layer - Device Driver
 *
 *   Copyright (C) 2015 Pierre-noel Bouteville . All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
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
 ******************************************************************************/

/* Supports:
 *  - Master operation, 100 kHz (standard) and 400 kHz (full speed)
 * TODO:
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained by
 *     the i2c_init(); it extends the Device structure from the nuttx/i2c/i2c.h;
 *     Instance points to OPS, to common I2C Hardware private data and contains
 *     its own private data, as frequency, address, mode of operation (in the
 *     future)
 *  - Private: Private data of an I2C Hardware
 *
 */

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "efm32_gpio.h"
#include "chip/efm32_cmu.h"
#include "chip/efm32_i2c.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_EFM32_I2C0) || defined(CONFIG_EFM32_I2C1)

/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Configuration **************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.  Instead,
 * CPU-intensive polling will be used.
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

#define I2C_OUTPUT (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_CNF_OUTOD )

#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

/* Debug ****************************************************************************/

/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#  define i2cdbg dbg
#  define i2cvdbg vdbg
#else
#  define i2cdbg(x...)
#  define i2cvdbg(x...)
#endif

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other debug
 * is enabled.
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
 * this SW supporting master mode. Likewise for I2C_IF_RXUF (receive underflow)
 * RXUF is only likely to occur with this SW if using a debugger peeking into
 * RXDATA register. Thus, we ignore those types of fault.
 */

#define I2C_IF_ERRORS    (I2C_IF_BUSERR | I2C_IF_ARBLOST)

/************************************************************************************
 * Private Types
 ************************************************************************************/

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
  uint32_t time;              /* First of event or first status */
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
  int (*isr) (int, void *);   /* Interrupt handler */
  uint32_t irq;               /* Event IRQ */
#endif
};

/* I2C Device Private Data */

struct efm32_i2c_priv_s
{
  const struct efm32_i2c_config_s *config;    /* Port configuration */
  int refs;                   /* Referernce count */
  sem_t sem_excl;             /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;              /* Interrupt wait semaphore */
#endif

  volatile uint8_t result;    /* result of transfer */

  uint8_t i2c_state;          /* i2c state machine */
  uint32_t i2c_reg_if;        /* Current state of I2Cx_IF register. */
  uint32_t i2c_reg_state;     /* Current state of I2Cx_STATES register. */

  int addr;                   /* Message address */
  uint8_t msgc;               /* Message count */
  struct i2c_msg_s *msgv;     /* Message list */
  uint8_t *ptr;               /* Current message buffer */
  int dcnt;                   /* Current message length */
  uint32_t flags;             /* Current message flags */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                   /* Trace array index */
  uint32_t start_time;        /* Time when the trace was started */

  /* The actual trace data */

  struct efm32_trace_s trace[CONFIG_I2C_NTRACE];
#endif
};

/* I2C Device, Instance */

struct efm32_i2c_inst_s
{
  const struct i2c_ops_s *ops;        /* Standard I2C operations */
  struct efm32_i2c_priv_s *priv;      /* Common driver private data structure 
                                       */

  uint32_t frequency;         /* Frequency used in this instantiation */
  int address;                /* Address used in this instantiation */
  uint32_t flags;             /* Flags used in this instantiation */
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static inline uint32_t efm32_i2c_getreg(FAR struct efm32_i2c_priv_s *priv,
                                        uint8_t offset);
static inline void efm32_i2c_putreg(FAR struct efm32_i2c_priv_s *priv,
                                    uint8_t offset, uint32_t value);
static inline void efm32_i2c_modifyreg(FAR struct efm32_i2c_priv_s *priv,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits);
static inline void efm32_i2c_sem_wait(FAR struct i2c_dev_s *dev);

#ifdef CONFIG_EFM32_I2C_DYNTIMEOUT
static useconds_t efm32_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs);
#endif /* CONFIG_EFM32_I2C_DYNTIMEOUT */

static inline int efm32_i2c_sem_waitdone(FAR struct efm32_i2c_priv_s *priv);
static inline void efm32_i2c_sem_post(FAR struct i2c_dev_s *dev);
static inline void efm32_i2c_sem_init(FAR struct i2c_dev_s *dev);
static inline void efm32_i2c_sem_destroy(FAR struct i2c_dev_s *dev);

#ifdef CONFIG_I2C_TRACE
static void efm32_i2c_tracereset(FAR struct efm32_i2c_priv_s *priv);
static void efm32_i2c_tracenew(FAR struct efm32_i2c_priv_s *priv);
static void efm32_i2c_tracedump(FAR struct efm32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void efm32_i2c_setclock(FAR struct efm32_i2c_priv_s *priv,
                               uint32_t frequency);

static int efm32_i2c_isr(struct efm32_i2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
#ifdef CONFIG_EFM32_I2C0
static int efm32_i2c0_isr(int irq, void *context);
#endif
#ifdef CONFIG_EFM32_I2C1
static int efm32_i2c1_isr(int irq, void *context);
#endif
#endif /* !CONFIG_I2C_POLLED */

static void efm32_i2c_reset(FAR struct efm32_i2c_priv_s *priv);
static int efm32_i2c_init(FAR struct efm32_i2c_priv_s *priv, int frequency);
static int efm32_i2c_deinit(FAR struct efm32_i2c_priv_s *priv);
static uint32_t efm32_i2c_setfrequency(FAR struct i2c_dev_s *dev,
                                       uint32_t frequency);
static int efm32_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int efm32_i2c_process(FAR struct i2c_dev_s *dev,
                             FAR struct i2c_msg_s *msgs, int count);
static int efm32_i2c_write(FAR struct i2c_dev_s *dev, const uint8_t * buffer,
                           int buflen);
static int efm32_i2c_read(FAR struct i2c_dev_s *dev, uint8_t * buffer,
                          int buflen);

#ifdef CONFIG_I2C_WRITEREAD
static int efm32_i2c_writeread(FAR struct i2c_dev_s *dev,
                               const uint8_t * wbuffer, int wbuflen,
                               uint8_t * buffer, int buflen);
#endif /* CONFIG_I2C_WRITEREAD */

#ifdef CONFIG_I2C_TRANSFER
static int efm32_i2c_transfer(FAR struct i2c_dev_s *dev,
                              FAR struct i2c_msg_s *msgs, int count);
#endif /* CONFIG_I2C_TRANSFER */

static const char *efm32_i2c_state_str(int i2c_state);

/*******************************************************************************
 * Private Data
 ******************************************************************************/

/* Trace events strings */

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
  .isr = efm32_i2c0_isr,
  .irq = EFM32_IRQ_I2C0
#endif
};

static struct efm32_i2c_priv_s efm32_i2c0_priv =
{
  .config = &efm32_i2c0_config,
  .refs = 0,
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
  .isr = efm32_i2c1_isr,
  .irq = EFM32_IRQ_I2C1
#endif
};

static struct efm32_i2c_priv_s efm32_i2c1_priv =
{
  .config = &efm32_i2c1_config,
  .refs = 0,
  .result = I2CRESULT_NONE,
  .msgc = 0,
  .msgv = NULL,
  .ptr = NULL,
  .dcnt = 0,
  .flags = 0,
};
#endif

/* Device Structures, Instantiation */

static const struct i2c_ops_s efm32_i2c_ops =
{
  .setfrequency = efm32_i2c_setfrequency,
  .setaddress = efm32_i2c_setaddress,
  .write = efm32_i2c_write,
  .read = efm32_i2c_read
#ifdef CONFIG_I2C_WRITEREAD
    ,.writeread = efm32_i2c_writeread
#endif
#ifdef CONFIG_I2C_TRANSFER
    ,.transfer = efm32_i2c_transfer
#endif
#ifdef CONFIG_I2C_SLAVE
    ,.setownaddress = efm32_i2c_setownaddress,
  .registercallback = efm32_i2c_registercallback
#endif
};

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*******************************************************************************
 * Name: efm32_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ******************************************************************************/

static inline uint32_t efm32_i2c_getreg(FAR struct efm32_i2c_priv_s *priv,
                                        uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/*******************************************************************************
 * Name: efm32_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ******************************************************************************/

static inline void efm32_i2c_putreg(FAR struct efm32_i2c_priv_s *priv,
                                    uint8_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/*******************************************************************************
 * Name: efm32_i2c_modifyreg
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ******************************************************************************/

static inline void efm32_i2c_modifyreg(FAR struct efm32_i2c_priv_s *priv,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/*******************************************************************************
 * Name: efm32_i2c_state_str
 *
 * Description:
 *   Convert i2c_state into corresponding text.
 *
 ******************************************************************************/

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

/*******************************************************************************
 * Name: efm32_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ******************************************************************************/

static inline void efm32_i2c_sem_wait(FAR struct i2c_dev_s *dev)
{
  while (sem_wait(&((struct efm32_i2c_inst_s *)dev)->priv->sem_excl) != OK)
    {
      ASSERT(errno == EINTR);
    }
}

/*******************************************************************************
 * Name: efm32_i2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ******************************************************************************/

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
static useconds_t efm32_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor. */

  return (useconds_t) (CONFIG_EFM32_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/*******************************************************************************
 * Name: efm32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ******************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int efm32_i2c_sem_waitdone(FAR struct efm32_i2c_priv_s *priv)
{
  struct timespec abstime;
  int ret;

  do
    {
      /* Get the current time */

      (void)clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_EFM32_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_EFM32_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
      abstime.tv_nsec += 1000 * efm32_i2c_tousecs(priv->msgc, priv->msgv);
      if (abstime.tv_nsec > 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_EFM32_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_EFM32_I2CTIMEOMS * 1000 * 1000;
      if (abstime.tv_nsec > 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }
#endif

      /* Enable I2C interrupts */

      efm32_i2c_putreg(priv, EFM32_I2C_IEN_OFFSET, I2C_IF_NACK | I2C_IF_ACK |
                       I2C_IF_MSTOP | I2C_IF_RXDATAV | I2C_IF_ERRORS);

      /* Wait until either the transfer is complete or the timeout expires */

      ret = sem_timedwait(&priv->sem_isr, &abstime);

      /* Disable I2C interrupts */

      efm32_i2c_putreg(priv, EFM32_I2C_IEN_OFFSET, 0);

      if (ret != OK && errno != EINTR)
        {
          /* Break out of the loop on irrecoverable errors.  This would include 
           * timeouts and mystery errors reported by sem_timedwait. NOTE that
           * we try again if we are awakened by a signal (EINTR).
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->result == I2CRESULT_INPROGRESS);

  i2cvdbg("result: %s elapsed: %d threshold: %d i2c_state %s "
          "I2Cx_STATES: %08x I2Cx_IF: %08x\n",
          efm32_i2c_result_str(priv->result), elapsed, timeout,
          efm32_i2c_state_str(priv->i2c_state), priv->i2c_reg_state,
          priv->i2c_reg_if);

  return ret;
}
#else
static inline int efm32_i2c_sem_waitdone(FAR struct efm32_i2c_priv_s *priv)
{
  uint32_t timeout;
  uint32_t start;
  uint32_t elapsed;

  /* Get the timeout value */

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
  timeout = USEC2TICK(efm32_i2c_tousecs(priv->msgc, priv->msgv));
#else
  timeout = CONFIG_EFM32_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE: Interrupts are
   * currently disabled but will be temporarily re-enabled below when
   * sem_timedwait() sleeps.
   */

  start = clock_systimer();

  do
    {
      /* Poll by simply calling the timer interrupt handler until it reports
       * that it is done.
       */

      efm32_i2c_isr(priv);

      /* Calculate the elapsed time */

      elapsed = clock_systimer() - start;
    }

  /* Loop until the transfer is complete. */

  while ((priv->result == I2CRESULT_INPROGRESS) && elapsed < timeout);

  i2cvdbg("result: %s elapsed: %d threshold: %d i2c_state %s "
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

/*******************************************************************************
 * Name: efm32_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ******************************************************************************/

static inline void efm32_i2c_sem_post(FAR struct i2c_dev_s *dev)
{
  sem_post(&((struct efm32_i2c_inst_s *)dev)->priv->sem_excl);
}

/*******************************************************************************
 * Name: efm32_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ******************************************************************************/

static inline void efm32_i2c_sem_init(FAR struct i2c_dev_s *dev)
{
  sem_init(&((struct efm32_i2c_inst_s *)dev)->priv->sem_excl, 0, 1);
#ifndef CONFIG_I2C_POLLED
  sem_init(&((struct efm32_i2c_inst_s *)dev)->priv->sem_isr, 0, 0);
#endif
}

/*******************************************************************************
 * Name: efm32_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ******************************************************************************/

static inline void efm32_i2c_sem_destroy(FAR struct i2c_dev_s *dev)
{
  sem_destroy(&((struct efm32_i2c_inst_s *)dev)->priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  sem_destroy(&((struct efm32_i2c_inst_s *)dev)->priv->sem_isr);
#endif
}

/*******************************************************************************
 * Name: efm32_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ******************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void efm32_i2c_traceclear(FAR struct efm32_i2c_priv_s *priv)
{
  struct efm32_trace_s *trace = &priv->trace[priv->tndx];

  trace->i2c_state = I2CSTATE_NONE;     /* I2C current state of state machine */
  trace->i2c_reg_if = 0;        /* I2C I2Cx_IF register */
  trace->i2c_reg_state = 0;     /* I2C I2Cx_STATES register */
  trace->count = 0;             /* Interrupt count when status change */
  trace->time = 0;              /* Time of first status or event */
}

static void efm32_i2c_tracereset(FAR struct efm32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx = 0;
  priv->start_time = clock_systimer();
  efm32_i2c_traceclear(priv);
}

static void efm32_i2c_tracenew(FAR struct efm32_i2c_priv_s *priv)
{
  struct efm32_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized? Has the states changed? */

  if ((trace->count == 0) ||
      (priv->i2c_reg_if != trace->i2c_reg_if) ||
      (priv->i2c_reg_state != trace->i2c_reg_state) ||
      (priv->i2c_state != trace->i2c_state))
    {
      /* Yes.. Was it the states changed? */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index (unless we are out of trace entries) */

          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              i2cdbg("Trace table overflow\n");
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
      trace->time = clock_systimer();
    }
  else
    {
      /* Just increment the count of times that we have seen this states */

      trace->count++;
    }
}

static void efm32_i2c_tracedump(FAR struct efm32_i2c_priv_s *priv)
{
  struct efm32_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %d\n", clock_systimer() - priv->start_time);

  for (i = 0; i <= priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. I2Cx_STATE: %08x I2Cx_PENDING: %08x COUNT: %3d "
             "STATE: %s(%2d) PARM: %08x TIME: %d\n",
             i + 1, trace->i2c_reg_state, trace->i2c_reg_if, trace->count,
             efm32_i2c_state_str(trace->i2c_state), trace->i2c_state,
             trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/*******************************************************************************
 * Name: efm32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ******************************************************************************/

static void efm32_i2c_setclock(FAR struct efm32_i2c_priv_s *priv,
                               uint32_t frequency)
{
  uint32_t div;

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

  /* Frequency is given by fSCL = fHFPERCLK/((Nlow + Nhigh)(DIV + 1) + 4), thus
   * DIV = ((fHFPERCLK - 4fSCL)/((Nlow + Nhigh)fSCL)) - 1
   */

#if defined(CONFIG_EFM32_I2C_CLHR_FAST)
#  define n (11 + 6)          /* Ratio is 11:3 */
#elif defined(CONFIG_EFM32_I2C_CLHR_ASYMMETRIC)
#  define n ( 6 + 3)          /* Ratio is 6:3 */
#else                                /* CLHR STANDARD */
#  define n ( 4 + 4)          /* Ratio is 4:4 */
#endif

  div = (BOARD_HFPERCLK_FREQUENCY - (4 * frequency)) / (n * frequency);

  /* Clock divisor must be at least 1 in slave mode according to reference */
  /* manual (in which case there is normally no need to set bus frequency). */

  if ((efm32_i2c_getreg(priv, EFM32_I2C_CTRL_OFFSET) & I2C_CTRL_SLAVE) && !div)
    {
      div = 1;
    }

  DEBUGASSERT(div <= (_I2C_CLKDIV_DIV_MASK >> _I2C_CLKDIV_DIV_SHIFT));

  efm32_i2c_putreg(priv, EFM32_I2C_CLKDIV_OFFSET, div);
}

/*******************************************************************************
 * Name: efm32_i2c_isr
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ******************************************************************************/

static int efm32_i2c_isr(struct efm32_i2c_priv_s *priv)
{
  for (;;)
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
              /* If arbitration fault, it indicates either a slave device not
               * responding as expected, or other master which is not supported 
               * by this SW. */

              priv->result = I2CRESULT_ARBLOST;
            }
          else if (priv->i2c_reg_if & I2C_IF_BUSERR)
            {
              /* A bus error indicates a misplaced start or stop, which should
               * not occur in master mode controlled by this SW.
               */

              priv->result = I2CRESULT_BUSERR;
            }

          /* If error situation occurred, it is difficult to know exact cause
           * and how to resolve. It will be up to a wrapper to determine how to 
           * handle a fault/recovery if possible.
           */

          priv->i2c_state = I2CSTATE_DONE;
          goto done;
        }

      switch (priv->i2c_state)
        {
        /***************************************************
         * Send first start+address (first byte if 10 bit)
         */

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

        /*******************************************************
         * Wait for ACK/NACK on address (first byte if 10 bit)
         */

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

        /******************************************************
         * Wait for ACK/NACK on second byte of 10 bit address
         */

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

        /*******************************
         * Send repeated start+address
         */

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

        /***********************************************
         * Wait for ACK/NACK on repeated start+address
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

        /*****************************
         * Send a data byte to slave
         */

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

              if (!(priv->flags & I2C_M_NORESTART))
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

        /*********************************************************
         * Wait for ACK/NACK from slave after sending data to it
         */
                 
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

        /****************************
         * Wait for data from slave
         */

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
              else if (priv->dcnt == 1)
                {
                  /* If there is only one byte to receive we need to transmit
                   * the NACK now, before the stop.
                   */

                  efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_NACK);
                }
              else
                {
                  /* Send ACK and wait for next byte */

                  efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_ACK);
                }
            }
          goto done;

        /***********************************
         * Wait for STOP to have been sent
         */

        case I2CSTATE_WFSTOPSENT:
          if (priv->i2c_reg_if & I2C_IF_MSTOP)
            {
              efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, I2C_IFC_MSTOP);
              priv->i2c_state = I2CSTATE_DONE;
            }

          goto done;

        /******************************/
        /* Unexpected state, SW fault */
        /******************************/

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
      sem_post(&priv->sem_isr);
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

#ifndef CONFIG_I2C_POLLED

/*******************************************************************************
 * Name: efm32_i2c0_isr
 *
 * Description:
 *   I2C0 interrupt service routine
 *
 ******************************************************************************/

#ifdef CONFIG_EFM32_I2C0
static int efm32_i2c0_isr(int irq, void *context)
{
  return efm32_i2c_isr(&efm32_i2c0_priv);
}
#endif

/*******************************************************************************
 * Name: efm32_i2c1_isr
 *
 * Description:
 *   I2C1 interrupt service routine
 *
 ******************************************************************************/

#ifdef CONFIG_EFM32_I2C1
static int efm32_i2c1_isr(int irq, void *context)
{
  return efm32_i2c_isr(&efm32_i2c1_priv);
}
#endif

#endif

/*******************************************************************************
 * Private Initialization and Deinitialization
 ******************************************************************************/

/*******************************************************************************
 * Name: efm32_i2c_reset
 *
 * Description:
 *   Reset I2C to same state as after a HW reset.
 *
 ******************************************************************************/

static void efm32_i2c_reset(FAR struct efm32_i2c_priv_s *priv)
{
  efm32_i2c_putreg(priv, EFM32_I2C_CTRL_OFFSET, _I2C_CTRL_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_CLKDIV_OFFSET, _I2C_CLKDIV_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_SADDR_OFFSET, _I2C_SADDR_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_SADDRMASK_OFFSET, _I2C_SADDRMASK_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_IEN_OFFSET, _I2C_IEN_RESETVALUE);
  efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, _I2C_IFC_MASK);

  /* Do not reset route register, setting should be done independently */
}

/*******************************************************************************
 * Name: efm32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *   Prepare and start an I2C transfer (single master mode only).
 *
 ******************************************************************************/

static int efm32_i2c_init(FAR struct efm32_i2c_priv_s *priv, int frequency)
{
  int regval;

  /* Power-up and configure GPIOs */

  /* Enable power and reset the peripheral */

  /* Enable I2C Clock */

  modifyreg32(EFM32_CMU_HFPERCLKEN0, 0, priv->config->clk_bit);

  /* Eeset all resgister */

  efm32_i2c_reset(priv);

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
  irq_attach(priv->config->irq, priv->config->isr);
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

/*******************************************************************************
 * Name: efm32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ******************************************************************************/

static int efm32_i2c_deinit(FAR struct efm32_i2c_priv_s *priv)
{
  /* Disable I2C */

  efm32_i2c_reset(priv);

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

/*******************************************************************************
 * Device Driver Operations
 ******************************************************************************/

/*******************************************************************************
 * Name: efm32_i2c_setfrequency
 *
 * Description:
 *   Set the I2C frequency
 *
 ******************************************************************************/

static uint32_t efm32_i2c_setfrequency(FAR struct i2c_dev_s *dev,
                                       uint32_t frequency)
{
  efm32_i2c_sem_wait(dev);

  ((struct efm32_i2c_inst_s *)dev)->frequency = frequency;

  efm32_i2c_sem_post(dev);
  return frequency;
}

/*******************************************************************************
 * Name: efm32_i2c_setaddress
 *
 * Description:
 *   Set the I2C slave address
 *
 ******************************************************************************/

static int efm32_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
  efm32_i2c_sem_wait(dev);

  ((struct efm32_i2c_inst_s *)dev)->address = addr;
  ((struct efm32_i2c_inst_s *)dev)->flags = (nbits == 10) ? I2C_M_TEN : 0;

  efm32_i2c_sem_post(dev);
  return OK;
}

/*******************************************************************************
 * Name: efm32_i2c_process
 *
 * Description:
 *   Common I2C transfer logic
 *
 ******************************************************************************/

static int efm32_i2c_process(FAR struct i2c_dev_s *dev,
                             FAR struct i2c_msg_s *msgs, int count)
{
  struct efm32_i2c_inst_s *inst = (struct efm32_i2c_inst_s *)dev;
  FAR struct efm32_i2c_priv_s *priv = inst->priv;
  int errval = 0;

  ASSERT(count);

  if (count == 0 || msgs == NULL)
    {
      return -1;
    }

  /* Ensure that address or flags don't change meanwhile */

  efm32_i2c_sem_wait(dev);

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  efm32_i2c_tracereset(priv);

  /* Set I2C clock frequency (on change it toggles I2C_CR1_PE !) */

  efm32_i2c_setclock(priv, inst->frequency);

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
      (void)efm32_i2c_getreg(priv, EFM32_I2C_RXDATA_OFFSET);
    }

  /* Clear all pending interrupts prior to starting transfer. */

  efm32_i2c_putreg(priv, EFM32_I2C_IFC_OFFSET, _I2C_IFC_MASK);

  /* Call once isr to start state machine. I2C interrupt are disabled and will
   * be enabled in efm32_i2c_sem_waitdone if CONFIG_I2C_POLLED is NOT defined
   */

  efm32_i2c_isr(priv);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get the
   * BUSY flag.
   */

  if (efm32_i2c_sem_waitdone(priv) < 0)
    {
      errval = ETIMEDOUT;

      i2cdbg("Timed out: I2Cx_STATE: 0x%04x I2Cx_STATUS: 0x%08x\n",
             efm32_i2c_getreg(priv, EFM32_I2C_STATE_OFFSET),
             efm32_i2c_getreg(priv, EFM32_I2C_STATUS_OFFSET));

      /* Abort */

      efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_ABORT);

    }

  /* Check for error status conditions */

#if 0
  /* Arbitration Lost (master mode) */
  errval = EAGAIN;
  /* Acknowledge Failure */
  errval = ENXIO;
  /* Overrun/Underrun */
  errval = EIO;
  /* PEC Error in reception */
  errval = EPROTO;
  /* Timeout or Tlow Error */
  errval = ETIME;
  /* I2C Bus is for some reason busy */
  errval = EBUSY;
#endif

  /* Dump the trace result */

  efm32_i2c_tracedump(priv);

  /* Ensure that any ISR happening after we finish can't overwrite any user
   * data 
   */

  priv->result = I2CRESULT_NONE;
  priv->dcnt = 0;
  priv->ptr = NULL;

  efm32_i2c_sem_post(dev);

  return -errval;
}

/*******************************************************************************
 * Name: efm32_i2c_write
 *
 * Description:
 *   Write I2C data
 *
 ******************************************************************************/

static int efm32_i2c_write(FAR struct i2c_dev_s *dev, const uint8_t * buffer,
                           int buflen)
{
  struct i2c_msg_s msgv =
  {
    .addr = ((struct efm32_i2c_inst_s *)dev)->address,
    .flags = ((struct efm32_i2c_inst_s *)dev)->flags,
    .buffer = (uint8_t *) buffer,
    .length = buflen
  };

  return efm32_i2c_process(dev, &msgv, 1);
}

/*******************************************************************************
 * Name: efm32_i2c_read
 *
 * Description:
 *   Read I2C data
 *
 ******************************************************************************/

int efm32_i2c_read(FAR struct i2c_dev_s *dev, uint8_t * buffer, int buflen)
{
  struct i2c_msg_s msgv =
  {
    .addr = ((struct efm32_i2c_inst_s *)dev)->address,
    .flags = ((struct efm32_i2c_inst_s *)dev)->flags | I2C_M_READ,
    .buffer = buffer,
    .length = buflen
  };

  return efm32_i2c_process(dev, &msgv, 1);
}

/*******************************************************************************
 * Name: efm32_i2c_writeread
 *
 * Description:
 *  Read then write I2C data
 *
 ******************************************************************************/

#ifdef CONFIG_I2C_WRITEREAD
static int efm32_i2c_writeread(FAR struct i2c_dev_s *dev,
                               const uint8_t * wbuffer, int wbuflen,
                               uint8_t * buffer, int buflen)
{
  struct i2c_msg_s msgv[2] =
  {
    {
     .addr = ((struct efm32_i2c_inst_s *)dev)->address,
     .flags = ((struct efm32_i2c_inst_s *)dev)->flags,
     .buffer = (uint8_t *) wbuffer,     /* This is really ugly, sorry const ... */
     .length = wbuflen},
    {
     .addr = ((struct efm32_i2c_inst_s *)dev)->address,
     .flags = ((struct efm32_i2c_inst_s *)dev)->flags |
     ((buflen > 0) ? I2C_M_READ : I2C_M_NORESTART),
     .buffer = buffer,
     .length = (buflen > 0) ? buflen : -buflen}
  };

  return efm32_i2c_process(dev, msgv, 2);
}
#endif

/*******************************************************************************
 * Name: efm32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ******************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
static int efm32_i2c_transfer(FAR struct i2c_dev_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  return efm32_i2c_process(dev, msgs, count);
}
#endif

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*******************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ******************************************************************************/

FAR struct i2c_dev_s *up_i2cinitialize(int port)
{
  struct efm32_i2c_priv_s *priv = NULL; /* Private data of device with multiple 
                                         * instances */
  struct efm32_i2c_inst_s *inst = NULL; /* Device, single instance */
  int irqs;

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

  /* Allocate instance */

  if (!(inst = kmm_malloc(sizeof(struct efm32_i2c_inst_s))))
    {
      return NULL;
    }

  /* Initialize instance */

  inst->ops = &efm32_i2c_ops;
  inst->priv = priv;
  inst->frequency = 100000;
  inst->address = 0;
  inst->flags = 0;

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  irqs = irqsave();

  if ((volatile int)priv->refs++ == 0)
    {
      efm32_i2c_sem_init((struct i2c_dev_s *)inst);
      efm32_i2c_init(priv, inst->frequency);
    }

  irqrestore(irqs);
  return (struct i2c_dev_s *)inst;
}

/*******************************************************************************
 * Name: up_i2cuninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ******************************************************************************/

int up_i2cuninitialize(FAR struct i2c_dev_s *dev)
{
  int irqs;

  ASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (((struct efm32_i2c_inst_s *)dev)->priv->refs == 0)
    {
      return ERROR;
    }

  irqs = irqsave();

  if (--((struct efm32_i2c_inst_s *)dev)->priv->refs)
    {
      irqrestore(irqs);
      kmm_free(dev);
      return OK;
    }

  irqrestore(irqs);

  /* Disable power and other HW resource (GPIO's) */

  efm32_i2c_deinit(((struct efm32_i2c_inst_s *)dev)->priv);

  /* Release unused resources */

  efm32_i2c_sem_destroy((struct i2c_dev_s *)dev);

  kmm_free(dev);
  return OK;
}

/*******************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ******************************************************************************/

#ifdef CONFIG_I2C_RESET
int up_i2creset(FAR struct i2c_dev_s *dev)
{
  struct efm32_i2c_priv_s *priv;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  int ret = ERROR;

  ASSERT(dev);

  /* Get I2C private structure */

  priv = ((struct efm32_i2c_inst_s *)dev)->priv;

  /* Our caller must own a ref */

  ASSERT(priv->refs > 0);

  /* Lock out other clients */

  efm32_i2c_sem_wait(dev);

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

  efm32_i2c_init(priv, ((struct efm32_i2c_inst_s *)dev)->frequency);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  efm32_i2c_sem_post(dev);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

#endif /* CONFIG_EFM32_I2C0 || CONFIG_EFM32_I2C1 */
