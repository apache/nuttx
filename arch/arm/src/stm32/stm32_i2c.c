/****************************************************************************
 * arch/arm/src/stm32/stm32_i2c.c
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
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained
 *     by the i2c_init(); it extends the Device structure from the
 *     nuttx/i2c/i2c.h;
 *     Instance points to OPS, to common I2C Hardware private data and
 *     contains its own private data, as frequency, address, mode of
 *     operation (in the future)
 *  - Private: Private data of an I2C Hardware
 *
 * TODO
 *  - Check for all possible deadlocks (as BUSY='1' I2C needs to be reset in
 *    HW using the I2C_CR1_SWRST)
 *  - SMBus support (hardware layer timings are already supported) and add
 *    SMBA gpio pin
 *  - Slave support with multiple addresses (on multiple instances):
 *      - 2 x 7-bit address or
 *      - 1 x 10 bit addresses + 1 x 7 bit address (?)
 *      - plus the broadcast address (general call)
 *  - Multi-master support
 *  - DMA (to get rid of too many CPU wake-ups and interventions)
 *  - Be ready for IPMI
 */

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
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_rcc.h"
#include "stm32_i2c.h"
#include "stm32_waste.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_STM32_I2C1) || defined(CONFIG_STM32_I2C2) || \
    defined(CONFIG_STM32_I2C3)

/* This implementation is for the STM32 F1, F2, and F4 only.
 * Experimentally enabled for STM32L15XX.
 */

#if defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F10XX) || \
    defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_STM32_I2CTIMEOSEC) && !defined(CONFIG_STM32_I2CTIMEOMS)
#  define CONFIG_STM32_I2CTIMEOSEC 0
#  define CONFIG_STM32_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_STM32_I2CTIMEOSEC)
#  define CONFIG_STM32_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_STM32_I2CTIMEOMS)
#  define CONFIG_STM32_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_STM32_I2CTIMEOTICKS
#  define CONFIG_STM32_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_STM32_I2CTIMEOSEC) + MSEC2TICK(CONFIG_STM32_I2CTIMEOMS))
#endif

#ifndef CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_STM32_I2CTIMEOTICKS)
#endif

/* On the STM32F103ZE, there is an internal conflict between I2C1 and FSMC.
 * In that case, it is necessary to disable FSMC before each I2C1 access
 * and re-enable FSMC when the I2C access completes.
 */

#undef I2C1_FSMC_CONFLICT
#if defined(CONFIG_STM32_STM32F10XX) && defined(CONFIG_STM32_FSMC) && defined(CONFIG_STM32_I2C1)
#  define I2C1_FSMC_CONFLICT
#endif

/* Macros to convert a I2C pin to a GPIO output */

#if defined(CONFIG_STM32_STM32L15XX)
#  define I2C_OUTPUT (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_OPENDRAIN | \
                      GPIO_SPEED_40MHz)
#elif defined(CONFIG_STM32_STM32F10XX)
#  define I2C_OUTPUT (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_CNF_OUTOD | \
                      GPIO_MODE_50MHz)
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define I2C_OUTPUT (GPIO_OUTPUT | GPIO_FLOAT | GPIO_OPENDRAIN |\
                      GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#endif

#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define stm32_i2c_tracereset(p)
#  define stm32_i2c_tracenew(p,s)
#  define stm32_i2c_traceevent(p,e,a)
#  define stm32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum stm32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum stm32_trace_e
{
  I2CEVENT_NONE = 0,      /* No events have occurred with this status */
  I2CEVENT_SENDADDR,      /* Start/Master bit set and address sent, param = msgc */
  I2CEVENT_SENDBYTE,      /* Send byte, param = dcnt */
  I2CEVENT_ITBUFEN,       /* Enable buffer interrupts, param = 0 */
  I2CEVENT_RCVBYTE,       /* Read more dta, param = dcnt */
  I2CEVENT_REITBUFEN,     /* Re-enable buffer interrupts, param = 0 */
  I2CEVENT_DISITBUFEN,    /* Disable buffer interrupts, param = 0 */
  I2CEVENT_BTFNOSTART,    /* BTF on last byte with no restart, param = msgc */
  I2CEVENT_BTFRESTART,    /* Last byte sent, re-starting, param = msgc */
  I2CEVENT_BTFSTOP,       /* Last byte sten, send stop, param = 0 */
  I2CEVENT_ERROR          /* Error occurred, param = 0 */
};

/* Trace data */

struct stm32_trace_s
{
  uint32_t status;             /* I2C 32-bit SR2|SR1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum stm32_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  clock_t time;                /* First of event or first status */
};

/* I2C Device hardware configuration */

struct stm32_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t reset_bit;         /* Reset bit */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  uint32_t ev_irq;            /* Event IRQ */
  uint32_t er_irq;            /* Error IRQ */
#endif
};

/* I2C Device Private Data */

struct stm32_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct stm32_i2c_config_s *config;

  int refs;                    /* Reference count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum stm32_intstate_e) */

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

  struct stm32_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint16_t stm32_i2c_getreg(struct stm32_i2c_priv_s *priv,
                                        uint8_t offset);
static inline void stm32_i2c_putreg(struct stm32_i2c_priv_s *priv,
                                    uint8_t offset, uint16_t value);
static inline void stm32_i2c_modifyreg(struct stm32_i2c_priv_s *priv,
                                       uint8_t offset, uint16_t clearbits,
                                       uint16_t setbits);

#ifdef CONFIG_STM32_I2C_DYNTIMEO
static uint32_t stm32_i2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_STM32_I2C_DYNTIMEO */

static inline int  stm32_i2c_sem_waitdone(struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sem_waitstop(struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sem_post(struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sem_init(struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sem_destroy(struct stm32_i2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void stm32_i2c_tracereset(struct stm32_i2c_priv_s *priv);
static void stm32_i2c_tracenew(struct stm32_i2c_priv_s *priv,
                               uint32_t status);
static void stm32_i2c_traceevent(struct stm32_i2c_priv_s *priv,
                                 enum stm32_trace_e event, uint32_t parm);
static void stm32_i2c_tracedump(struct stm32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void stm32_i2c_setclock(struct stm32_i2c_priv_s *priv,
                               uint32_t frequency);
static inline void stm32_i2c_sendstart(struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_clrstart(struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sendstop(struct stm32_i2c_priv_s *priv);
static inline
uint32_t stm32_i2c_getstatus(struct stm32_i2c_priv_s *priv);

#ifdef I2C1_FSMC_CONFLICT
static inline
uint32_t stm32_i2c_disablefsmc(struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_enablefsmc(uint32_t ahbenr);
#endif /* I2C1_FSMC_CONFLICT */

static int stm32_i2c_isr_process(struct stm32_i2c_priv_s * priv);

#ifndef CONFIG_I2C_POLLED
static int stm32_i2c_isr(int irq, void *context, void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int stm32_i2c_init(struct stm32_i2c_priv_s *priv);
static int stm32_i2c_deinit(struct stm32_i2c_priv_s *priv);
static int stm32_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int stm32_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Trace events strings */

#ifdef CONFIG_I2C_TRACE
static const char *g_trace_names[] =
{
  "NONE      ",
  "SENDADDR  ",
  "SENDBYTE  ",
  "ITBUFEN   ",
  "RCVBYTE   ",
  "REITBUFEN ",
  "DISITBUFEN",
  "BTFNOSTART",
  "BTFRESTART",
  "BTFSTOP   ",
  "ERROR     "
};
#endif

/* I2C interface */

static const struct i2c_ops_s stm32_i2c_ops =
{
  .transfer = stm32_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = stm32_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_STM32_I2C1
static const struct stm32_i2c_config_s stm32_i2c1_config =
{
  .base       = STM32_I2C1_BASE,
  .clk_bit    = RCC_APB1ENR_I2C1EN,
  .reset_bit  = RCC_APB1RSTR_I2C1RST,
  .scl_pin    = GPIO_I2C1_SCL,
  .sda_pin    = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq     = STM32_IRQ_I2C1EV,
  .er_irq     = STM32_IRQ_I2C1ER
#endif
};

static struct stm32_i2c_priv_s stm32_i2c1_priv =
{
  .ops        = &stm32_i2c_ops,
  .config     = &stm32_i2c1_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

#ifdef CONFIG_STM32_I2C2
static const struct stm32_i2c_config_s stm32_i2c2_config =
{
  .base       = STM32_I2C2_BASE,
  .clk_bit    = RCC_APB1ENR_I2C2EN,
  .reset_bit  = RCC_APB1RSTR_I2C2RST,
  .scl_pin    = GPIO_I2C2_SCL,
  .sda_pin    = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq     = STM32_IRQ_I2C2EV,
  .er_irq     = STM32_IRQ_I2C2ER
#endif
};

static struct stm32_i2c_priv_s stm32_i2c2_priv =
{
  .ops        = &stm32_i2c_ops,
  .config     = &stm32_i2c2_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

#ifdef CONFIG_STM32_I2C3
static const struct stm32_i2c_config_s stm32_i2c3_config =
{
  .base       = STM32_I2C3_BASE,
  .clk_bit    = RCC_APB1ENR_I2C3EN,
  .reset_bit  = RCC_APB1RSTR_I2C3RST,
  .scl_pin    = GPIO_I2C3_SCL,
  .sda_pin    = GPIO_I2C3_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq     = STM32_IRQ_I2C3EV,
  .er_irq     = STM32_IRQ_I2C3ER
#endif
};

static struct stm32_i2c_priv_s stm32_i2c3_priv =
{
  .ops        = &stm32_i2c_ops,
  .config     = &stm32_i2c3_config,
  .refs       = 0,
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
 * Name: stm32_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t stm32_i2c_getreg(struct stm32_i2c_priv_s *priv,
                                        uint8_t offset)
{
  return getreg16(priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32_i2c_putreg(struct stm32_i2c_priv_s *priv,
                                    uint8_t offset, uint16_t value)
{
  putreg16(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32_i2c_modifyreg
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32_i2c_modifyreg(struct stm32_i2c_priv_s *priv,
                                       uint8_t offset, uint16_t clearbits,
                                       uint16_t setbits)
{
  modifyreg16(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: stm32_i2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_I2C_DYNTIMEO
static uint32_t stm32_i2c_toticks(int msgc, struct i2c_msg_s *msgs)
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

  return USEC2TICK(CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: stm32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int stm32_i2c_sem_waitdone(struct stm32_i2c_priv_s *priv)
{
  irqstate_t flags;
  uint32_t regval;
  int ret;

  flags = enter_critical_section();

  /* Enable I2C interrupts */

  regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  regval |= (I2C_CR2_ITERREN | I2C_CR2_ITEVFEN);
  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_tickwait_uninterruptible() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Wait until either the transfer is complete or the timeout expires */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                         stm32_i2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_STM32_I2CTIMEOTICKS);
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

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  regval &= ~I2C_CR2_ALLINTS;
  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);

  leave_critical_section(flags);
  return ret;
}
#else
static inline int stm32_i2c_sem_waitdone(struct stm32_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
  timeout = stm32_i2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_STM32_I2CTIMEOTICKS;
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

      stm32_i2c_isr_process(priv);
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cinfo("intstate: %d elapsed: %ld threshold: %ld status: %08" PRIx32 "\n",
          priv->intstate, (long)elapsed, (long)timeout, priv->status);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ****************************************************************************/

static inline void stm32_i2c_sem_waitstop(struct stm32_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t cr1;
  uint32_t sr1;

  /* Select a timeout */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_STM32_I2CTIMEOTICKS;
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

      /* Check for STOP condition */

      cr1 = stm32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET);
      if ((cr1 & I2C_CR1_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      sr1 = stm32_i2c_getreg(priv, STM32_I2C_SR1_OFFSET);
      if ((sr1 & I2C_SR1_TIMEOUT) != 0)
        {
          return;
        }
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.
   */

  i2cinfo("Timeout with CR1: %04" PRIx32 " SR1: %04" PRIx32 "\n", cr1, sr1);
}

/****************************************************************************
 * Name: stm32_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

static inline void stm32_i2c_sem_post(struct stm32_i2c_priv_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/****************************************************************************
 * Name: stm32_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ****************************************************************************/

static inline void stm32_i2c_sem_init(struct stm32_i2c_priv_s *priv)
{
  nxsem_init(&priv->sem_excl, 0, 1);

#ifndef CONFIG_I2C_POLLED
  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->sem_isr, 0, 0);
  nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);
#endif
}

/****************************************************************************
 * Name: stm32_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ****************************************************************************/

static inline void stm32_i2c_sem_destroy(struct stm32_i2c_priv_s *priv)
{
  nxsem_destroy(&priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&priv->sem_isr);
#endif
}

/****************************************************************************
 * Name: stm32_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void stm32_i2c_traceclear(struct stm32_i2c_priv_s *priv)
{
  struct stm32_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void stm32_i2c_tracereset(struct stm32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  stm32_i2c_traceclear(priv);
}

static void stm32_i2c_tracenew(struct stm32_i2c_priv_s *priv,
                               uint32_t status)
{
  struct stm32_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?  Has the status changed? */

  if (trace->count == 0 || status != trace->status)
    {
      /* Yes.. Was it the status changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index
           * (unless we are out of trace entries)
           */

          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              i2cerr("ERROR: Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      stm32_i2c_traceclear(priv);
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

static void stm32_i2c_traceevent(struct stm32_i2c_priv_s *priv,
                                 enum stm32_trace_e event, uint32_t parm)
{
  struct stm32_trace_s *trace;

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
      stm32_i2c_traceclear(priv);
    }
}

static void stm32_i2c_tracedump(struct stm32_i2c_priv_s *priv)
{
  struct stm32_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %ld\n",
         (long)(clock_systime_ticks() - priv->start_time));

  for (i = 0; i < priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
         "%2d. STATUS: %08x COUNT: %3d EVENT: %s(%2d) PARM: %08x TIME: %d\n",
             i + 1, trace->status, trace->count, g_trace_names[trace->event],
             trace->event, trace->parm, trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: stm32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void stm32_i2c_setclock(struct stm32_i2c_priv_s *priv,
                               uint32_t frequency)
{
  uint16_t cr1;
  uint16_t ccr;
  uint16_t trise;
  uint16_t freqmhz;
  uint16_t speed;

  /* Has the I2C bus frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Disable the selected I2C peripheral to configure TRISE */

      cr1 = stm32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET);
      stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, cr1 & ~I2C_CR1_PE);

      /* Update timing and control registers */

      freqmhz = (uint16_t)(STM32_PCLK1_FREQUENCY / 1000000);
      ccr = 0;

      /* Configure speed in standard mode */

      if (frequency <= 100000)
        {
          /* Standard mode speed calculation */

          speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency << 1));

          /* The CCR fault must be >= 4 */

          if (speed < 4)
            {
              /* Set the minimum allowed value */

              speed = 4;
            }

          ccr |= speed;

          /* Set Maximum Rise Time for standard mode */

          trise = freqmhz + 1;
        }

      /* Configure speed in fast mode */

      else /* (frequency <= 400000) */
        {
          /* Fast mode speed calculation with Tlow/Thigh = 16/9 */

#ifdef CONFIG_STM32_I2C_DUTY16_9
          speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency * 25));

          /* Set DUTY and fast speed bits */

          ccr |= (I2C_CCR_DUTY | I2C_CCR_FS);
#else
          /* Fast mode speed calculation with Tlow/Thigh = 2 */

          speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency * 3));

          /* Set fast speed bit */

          ccr |= I2C_CCR_FS;
#endif

          /* Verify that the CCR speed value is nonzero */

          if (speed < 1)
            {
              /* Set the minimum allowed value */

              speed = 1;
            }

          ccr |= speed;

          /* Set Maximum Rise Time for fast mode */

          trise = (uint16_t)(((freqmhz * 300) / 1000) + 1);
        }

      /* Write the new values of the CCR and TRISE registers */

      stm32_i2c_putreg(priv, STM32_I2C_CCR_OFFSET, ccr);
      stm32_i2c_putreg(priv, STM32_I2C_TRISE_OFFSET, trise);

      /* Bit 14 of OAR1 must be configured and kept at 1 */

      stm32_i2c_putreg(priv, STM32_I2C_OAR1_OFFSET, I2C_OAR1_ONE);

      /* Re-enable the peripheral (or not) */

      stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, cr1);

      /* Save the new I2C frequency */

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: stm32_i2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ****************************************************************************/

static inline void stm32_i2c_sendstart(struct stm32_i2c_priv_s *priv)
{
  /* Disable ACK on receive by default and generate START */

  stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                      I2C_CR1_ACK, I2C_CR1_START);
}

/****************************************************************************
 * Name: stm32_i2c_clrstart
 *
 * Description:
 *   Clear the STOP, START or PEC condition on certain error recovery steps.
 *
 ****************************************************************************/

static inline void stm32_i2c_clrstart(struct stm32_i2c_priv_s *priv)
{
  /* "Note: When the STOP, START or PEC bit is set, the software must
   *  not perform any write access to I2C_CR1 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PEC request."
   *
   * "The [STOP] bit is set and cleared by software, cleared by hardware
   *  when a Stop condition is detected, set by hardware when a timeout
   *  error is detected.
   *
   * "This [START] bit is set and cleared by software and cleared by hardware
   *  when start is sent or PE=0."  The bit must be cleared by software if
   *  the START is never sent.
   *
   * "This [PEC] bit is set and cleared by software, and cleared by hardware
   *  when PEC is transferred or by a START or Stop condition or when PE=0."
   */

  stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                      I2C_CR1_START | I2C_CR1_STOP | I2C_CR1_PEC, 0);
}

/****************************************************************************
 * Name: stm32_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ****************************************************************************/

static inline void stm32_i2c_sendstop(struct stm32_i2c_priv_s *priv)
{
  stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, I2C_CR1_STOP);
}

/****************************************************************************
 * Name: stm32_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status (SR1 and SR2 combined)
 *
 ****************************************************************************/

static inline uint32_t stm32_i2c_getstatus(struct stm32_i2c_priv_s *priv)
{
  uint32_t status = stm32_i2c_getreg(priv, STM32_I2C_SR1_OFFSET);
  status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);
  return status;
}

/****************************************************************************
 * Name: stm32_i2c_disablefsmc
 *
 * Description:
 *   FSMC must be disable while accessing I2C1 because it uses a common
 *   resource (LBAR)
 *
 *  NOTE:
 *  This is an issue with the STM32F103ZE, but may not be an issue with other
 *  STM32s.  You may need to experiment
 *
 ****************************************************************************/

#ifdef I2C1_FSMC_CONFLICT
static inline
uint32_t stm32_i2c_disablefsmc(struct stm32_i2c_priv_s *priv)
{
  uint32_t ret = 0;
  uint32_t regval;

  /* Is this I2C1 */

#if defined(CONFIG_STM32_I2C2) || defined(CONFIG_STM32_I2C3)
  if (priv->config->base == STM32_I2C1_BASE)
#endif
    {
      /* Disable FSMC unconditionally */

      ret    = getreg32(STM32_RCC_AHBENR);
      regval = ret & ~RCC_AHBENR_FSMCEN;
      putreg32(regval, STM32_RCC_AHBENR);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_i2c_enablefsmc
 *
 * Description:
 *   Re-enable the FSMC
 *
 ****************************************************************************/

static inline void stm32_i2c_enablefsmc(uint32_t ahbenr)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC only if it was previously enabled. */

  if ((ahbenr & RCC_AHBENR_FSMCEN) != 0)
    {
      regval  = getreg32(STM32_RCC_AHBENR);
      regval |= RCC_AHBENR_FSMCEN;
      putreg32(regval, STM32_RCC_AHBENR);
    }
}
#else
#  define stm32_i2c_disablefsmc(priv) (0)
#  define stm32_i2c_enablefsmc(ahbenr)
#endif /* I2C1_FSMC_CONFLICT */

/****************************************************************************
 * Name: stm32_i2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int stm32_i2c_isr_process(struct stm32_i2c_priv_s *priv)
{
  uint32_t status = stm32_i2c_getstatus(priv);

  /* Check for new trace setup */

  stm32_i2c_tracenew(priv, status);

  /* Was start bit sent */

  if ((status & I2C_SR1_SB) != 0)
    {
      stm32_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgc);

      /* We check for msgc > 0 here as an unexpected interrupt with
       * I2C_SR1_SB set due to noise on the I2C cable can otherwise cause
       * msgc to wrap causing memory overwrite
       */

      if (priv->msgc > 0 && priv->msgv != NULL)
        {
          /* Get run-time data */

          priv->ptr   = priv->msgv->buffer;
          priv->dcnt  = priv->msgv->length;
          priv->flags = priv->msgv->flags;

          /* Send address byte and define addressing mode */

          stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET,
                           (priv->flags & I2C_M_TEN) ?
                           0 : ((priv->msgv->addr << 1) |
                           (priv->flags & I2C_M_READ)));

          /* Set ACK for receive mode */

          if (priv->dcnt > 1 && (priv->flags & I2C_M_READ) != 0)
            {
              stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                                  0, I2C_CR1_ACK);
            }

          /* Increment to next pointer and decrement message count */

          priv->msgv++;
          priv->msgc--;
        }
      else
        {
          /* Clear ISR by writing to DR register */

          stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, 0);
        }
    }

  /* In 10-bit addressing mode, was first byte sent */

  else if ((status & I2C_SR1_ADD10) != 0)
    {
      /* TODO: Finish 10-bit mode addressing.
       *
       * For now just clear ISR by writing to DR register. As we don't do
       * 10 bit addressing this must be a spurious ISR
       */

       stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, 0);
    }

  /* Was address sent, continue with either sending or reading data */

  else if ((priv->flags & I2C_M_READ) == 0 &&
           (status & (I2C_SR1_ADDR | I2C_SR1_TXE)) != 0)
    {
      if (priv->dcnt > 0)
        {
          /* Send a byte */

          stm32_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
          stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, *priv->ptr++);
          priv->dcnt--;
        }
    }

  else if ((priv->flags & I2C_M_READ) != 0 && (status & I2C_SR1_ADDR) != 0)
    {
      /* Enable RxNE and TxE buffers in order to receive one or multiple
       * bytes
       */

#ifndef CONFIG_I2C_POLLED
      stm32_i2c_traceevent(priv, I2CEVENT_ITBUFEN, 0);
      stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
#endif
    }

  /* More bytes to read */

  else if ((status & I2C_SR1_RXNE) != 0)
    {
      /* Read a byte, if dcnt goes < 0, then read dummy bytes to ack ISRs */

      if (priv->dcnt > 0)
        {
          stm32_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

          /* No interrupts or context switches may occur in the following
           * sequence.  Otherwise, additional bytes may be sent by the
           * device.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t flags = enter_critical_section();
#endif
          /* Receive a byte */

          *priv->ptr++ = stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);

          /* Disable acknowledge when last byte is to be received */

          priv->dcnt--;
          if (priv->dcnt == 1)
            {
              stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                                  I2C_CR1_ACK, 0);
            }

#ifdef CONFIG_I2C_POLLED
          leave_critical_section(flags);
#endif
        }
      else
        {
          /* Throw away the unexpected byte */

          stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);
        }
    }
  else if (status & I2C_SR1_TXE)
    {
      /* This should never happen, but it does happen occasionally with lots
       * of noise on the bus. It means the peripheral is expecting more data
       * bytes, but we don't have any to give.
       */

      stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, 0);
    }
  else if (status & I2C_SR1_BTF)
    {
      /* We should have handled all cases where this could happen above, but
       * just to ensure it gets ACKed, lets clear it here
       */

      stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);
    }
  else if (status & I2C_SR1_STOPF)
    {
      /* We should never get this, as we are a master not a slave. Write CR1
       * with its current value to clear the error
       */

      stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, 0, 0);
    }

  /* Do we have more bytes to send, enable/disable buffer interrupts
   * (these ISRs could be replaced by DMAs)
   */

#ifndef CONFIG_I2C_POLLED
  if (priv->dcnt > 0)
    {
      stm32_i2c_traceevent(priv, I2CEVENT_REITBUFEN, 0);
      stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
    }
  else if (priv->dcnt == 0)
    {
      stm32_i2c_traceevent(priv, I2CEVENT_DISITBUFEN, 0);
      stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);
    }
#endif

  /* Was last byte received or sent?  Hmmm... the F2 and F4 seems to differ
   * from the F1 in that BTF is not set after data is received (only RXNE).
   */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX) || \
    defined(CONFIG_STM32_STM32L15XX)
  if (priv->dcnt <= 0 && (status & (I2C_SR1_BTF | I2C_SR1_RXNE)) != 0)
#else
  if (priv->dcnt <= 0 && (status & I2C_SR1_BTF) != 0)
#endif
    {
      stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);    /* ACK ISR */

      /* Do we need to terminate or restart after this byte?
       * If there are more messages to send, then we may:
       *
       *  - continue with repeated start
       *  - or just continue sending writeable part
       *  - or we close down by sending the stop bit
       */

      if (priv->msgc > 0 && priv->msgv != NULL)
        {
          if (priv->msgv->flags & I2C_M_NOSTART)
            {
              stm32_i2c_traceevent(priv, I2CEVENT_BTFNOSTART, priv->msgc);
              priv->ptr   = priv->msgv->buffer;
              priv->dcnt  = priv->msgv->length;
              priv->flags = priv->msgv->flags;
              priv->msgv++;
              priv->msgc--;

              /* Restart this ISR! */

#ifndef CONFIG_I2C_POLLED
              stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET,
                                  0, I2C_CR2_ITBUFEN);
#endif
            }
          else
            {
              stm32_i2c_traceevent(priv, I2CEVENT_BTFRESTART, priv->msgc);
              stm32_i2c_sendstart(priv);
            }
        }
      else if (priv->msgv)
        {
          stm32_i2c_traceevent(priv, I2CEVENT_BTFSTOP, 0);
          stm32_i2c_sendstop(priv);

          /* Is there a thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
          if (priv->intstate == INTSTATE_WAITING)
            {
              /* Yes.. inform the thread that the transfer is complete
               * and wake it up.
               */

              nxsem_post(&priv->sem_isr);
              priv->intstate = INTSTATE_DONE;
            }
#else
          priv->intstate = INTSTATE_DONE;
#endif

          /* Mark that we have stopped with this transaction */

          priv->msgv = NULL;
        }
    }

  /* Check for errors, in which case, stop the transfer and return
   * Note that in master reception mode AF becomes set on last byte
   * since ACK is not returned. We should ignore this error.
   */

  if ((status & I2C_SR1_ERRORMASK) != 0)
    {
      stm32_i2c_traceevent(priv, I2CEVENT_ERROR, 0);

      /* Clear interrupt flags */

      stm32_i2c_putreg(priv, STM32_I2C_SR1_OFFSET, 0);

      /* Is there a thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
      if (priv->intstate == INTSTATE_WAITING)
        {
          /* Yes.. inform the thread that the transfer is complete
           * and wake it up.
           */

          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#else
      priv->intstate = INTSTATE_DONE;
#endif
    }

  priv->status = status;
  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int stm32_i2c_isr(int irq, void *context, void *arg)
{
  struct stm32_i2c_priv_s *priv = (struct stm32_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return stm32_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: stm32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int stm32_i2c_init(struct stm32_i2c_priv_s *priv)
{
  /* Power-up and configure GPIOs */

  /* Enable power and reset the peripheral */

  modifyreg32(STM32_RCC_APB1ENR, 0, priv->config->clk_bit);
  modifyreg32(STM32_RCC_APB1RSTR, 0, priv->config->reset_bit);
  modifyreg32(STM32_RCC_APB1RSTR, priv->config->reset_bit, 0);

  /* Configure pins */

  if (stm32_configgpio(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (stm32_configgpio(priv->config->sda_pin) < 0)
    {
      stm32_unconfiggpio(priv->config->scl_pin);
      return ERROR;
    }

  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->ev_irq, stm32_i2c_isr, priv);
  irq_attach(priv->config->er_irq, stm32_i2c_isr, priv);
  up_enable_irq(priv->config->ev_irq);
  up_enable_irq(priv->config->er_irq);
#endif

  /* Set peripheral frequency, where it must be at least 2 MHz  for 100 kHz
   * or 4 MHz for 400 kHz.  This also disables all I2C interrupts.
   */

  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET,
                  (STM32_PCLK1_FREQUENCY / 1000000));

  /* Force a frequency update */

  priv->frequency = 0;

  stm32_i2c_setclock(priv, 100000);

  /* Enable I2C */

  stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_PE);
  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int stm32_i2c_deinit(struct stm32_i2c_priv_s *priv)
{
  /* Disable I2C */

  stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, 0);

  /* Unconfigure GPIO pins */

  stm32_unconfiggpio(priv->config->scl_pin);
  stm32_unconfiggpio(priv->config->sda_pin);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->ev_irq);
  up_disable_irq(priv->config->er_irq);
  irq_detach(priv->config->ev_irq);
  irq_detach(priv->config->er_irq);
#endif

  /* Disable clocking */

  modifyreg32(STM32_RCC_APB1ENR, priv->config->clk_bit, 0);
  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int stm32_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct stm32_i2c_priv_s *priv = (struct stm32_i2c_priv_s *)dev;
  uint32_t status = 0;
#ifdef I2C1_FSMC_CONFLICT
  uint32_t ahbenr;
#endif
  int ret;

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  ret = nxsem_wait(&priv->sem_excl);
  if (ret < 0)
    {
      return ret;
    }

#ifdef I2C1_FSMC_CONFLICT
  /* Disable FSMC that shares a pin with I2C1 (LBAR) */

  ahbenr = stm32_i2c_disablefsmc(priv);

#else
  /* Wait for any STOP in progress.  NOTE:  If we have to disable the FSMC
   * then we cannot do this at the top of the loop, unfortunately.  The STOP
   * will not complete normally if the FSMC is enabled.
   */

  stm32_i2c_sem_waitstop(priv);
#endif

  /* Clear any pending error interrupts */

  stm32_i2c_putreg(priv, STM32_I2C_SR1_OFFSET, 0);

  /* "Note: When the STOP, START or PEC bit is set, the software must
   *  not perform any write access to I2C_CR1 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PEC request."  However, if the bits are
   *  not cleared by hardware, then we will have to do that from hardware.
   */

  stm32_i2c_clrstart(priv);

  /* Old transfers are done */

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  stm32_i2c_tracereset(priv);

  /* Set I2C clock frequency (on change it toggles I2C_CR1_PE !)
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  stm32_i2c_setclock(priv, msgs->frequency);

  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within stm32_i2c_waitdone().
   */

  priv->status = 0;
  stm32_i2c_sendstart(priv);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (stm32_i2c_sem_waitdone(priv) < 0)
    {
      status = stm32_i2c_getstatus(priv);
      ret = -ETIMEDOUT;

      i2cerr("ERROR: Timed out: CR1: 0x%04x status: 0x%08" PRIx32 "\n",
             stm32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET), status);

      /* "Note: When the STOP, START or PEC bit is set, the software must
       *  not perform any write access to I2C_CR1 before this bit is
       *  cleared by hardware. Otherwise there is a risk of setting a
       *  second STOP, START or PEC request."
       */

      stm32_i2c_clrstart(priv);

      /* Clear busy flag in case of timeout */

      status = priv->status & 0xffff;
    }
  else
    {
      /* clear SR2 (BUSY flag) as we've done successfully */

      status = priv->status & 0xffff;
    }

  /* Check for error status conditions */

  if ((status & I2C_SR1_ERRORMASK) != 0)
    {
      /* I2C_SR1_ERRORMASK is the 'OR' of the following individual bits: */

      if (status & I2C_SR1_BERR)
        {
          /* Bus Error */

          ret = -EIO;
        }
      else if (status & I2C_SR1_ARLO)
        {
          /* Arbitration Lost (master mode) */

          ret = -EAGAIN;
        }
      else if (status & I2C_SR1_AF)
        {
          /* Acknowledge Failure */

          ret = -ENXIO;
        }
      else if (status & I2C_SR1_OVR)
        {
          /* Overrun/Underrun */

          ret = -EIO;
        }
      else if (status & I2C_SR1_PECERR)
        {
          /* PEC Error in reception */

          ret = -EPROTO;
        }
      else if (status & I2C_SR1_TIMEOUT)
        {
          /* Timeout or Tlow Error */

          ret = -ETIME;
        }

      /* This is not an error and should never happen since SMBus is not
       * enabled
       */

      else /* if (status & I2C_SR1_SMBALERT) */
        {
          /* SMBus alert is an optional signal with an interrupt line for
           * devices that want to trade their ability to master for a pin.
           */

          ret = -EINTR;
        }
    }

  /* This is not an error, but should not happen.  The BUSY signal can hang,
   * however, if there are unhealthy devices on the bus that need to be
   * reset.
   * NOTE:  We will only see this busy indication if stm32_i2c_sem_waitdone()
   * fails above;  Otherwise it is cleared.
   */

  else if ((status & (I2C_SR2_BUSY << 16)) != 0)
    {
      /* I2C Bus is for some reason busy */

      ret = -EBUSY;
    }

  /* Dump the trace result */

  stm32_i2c_tracedump(priv);

#ifdef I2C1_FSMC_CONFLICT
  /* Wait for any STOP in progress.  NOTE:  If we have to disable the FSMC
   * then we cannot do this at the top of the loop, unfortunately.  The STOP
   * will not complete normally if the FSMC is enabled.
   */

  stm32_i2c_sem_waitstop(priv);

  /* Re-enable the FSMC */

  stm32_i2c_enablefsmc(ahbenr);
#endif

  /* Ensure that any ISR happening after we finish can't overwrite any user
   * data
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  stm32_i2c_sem_post(priv);
  return ret;
}

/****************************************************************************
 * Name: stm32_i2c_reset
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
static int stm32_i2c_reset(struct i2c_master_s *dev)
{
  struct stm32_i2c_priv_s *priv = (struct stm32_i2c_priv_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t frequency;
  int ret;

  DEBUGASSERT(dev);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxsem_wait_uninterruptible(&priv->sem_excl);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  stm32_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  stm32_configgpio(scl_gpio);
  stm32_configgpio(sda_gpio);

  /* Let SDA go high */

  stm32_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!stm32_gpioread(sda_gpio))
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
      while (!stm32_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      stm32_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      stm32_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  stm32_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  stm32_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  stm32_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  stm32_gpiowrite(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  stm32_unconfiggpio(sda_gpio);
  stm32_unconfiggpio(scl_gpio);

  /* Re-init the port */

  stm32_i2c_init(priv);

  /* Restore the frequency */

  stm32_i2c_setclock(priv, frequency);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  stm32_i2c_sem_post(priv);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *stm32_i2cbus_initialize(int port)
{
  struct stm32_i2c_priv_s * priv = NULL;
  irqstate_t flags;

#if STM32_PCLK1_FREQUENCY < 4000000
#   warning STM32_I2C_INIT: Peripheral clock must be at least 4 MHz to support 400 kHz operation.
#endif

#if STM32_PCLK1_FREQUENCY < 2000000
#   warning STM32_I2C_INIT: Peripheral clock must be at least 2 MHz to support 100 kHz operation.
  return NULL;
#endif

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_STM32_I2C1
    case 1:
      priv = (struct stm32_i2c_priv_s *)&stm32_i2c1_priv;
      break;
#endif
#ifdef CONFIG_STM32_I2C2
    case 2:
      priv = (struct stm32_i2c_priv_s *)&stm32_i2c2_priv;
      break;
#endif
#ifdef CONFIG_STM32_I2C3
    case 3:
      priv = (struct stm32_i2c_priv_s *)&stm32_i2c3_priv;
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
      stm32_i2c_sem_init(priv);
      stm32_i2c_init(priv);
    }

  leave_critical_section(flags);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: stm32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int stm32_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct stm32_i2c_priv_s *priv = (struct stm32_i2c_priv_s *)dev;
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

  /* Disable power and other HW resource (GPIO's) */

  stm32_i2c_deinit(priv);

  /* Release unused resources */

  stm32_i2c_sem_destroy(priv);
  return OK;
}

#endif /* CONFIG_STM32_STM32F10XX || CONFIG_STM32_STM32F20XX || CONFIG_STM32_STM32F4XXX */
#endif /* CONFIG_STM32_I2C1 || CONFIG_STM32_I2C2 || CONFIG_STM32_I2C3 */
