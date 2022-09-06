/****************************************************************************
 * arch/arm/src/stm32/stm32f40xxx_i2c.c
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
 *      by the i2c_init(); it extends the Device structure from the
 *      nuttx/i2c/i2c.h;
 *      Instance points to OPS, to common I2C Hardware private data and
 *      contains its own private data, as frequency, address, mode of
 *      operation (in the future)
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
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_rcc.h"
#include "stm32_i2c.h"
#include "stm32_waste.h"
#include "stm32_dma.h"

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
 * In that case, it is necessary to disable FSMC before each I2C1 access and
 * re-enable FSMC when the I2C access completes.
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

/* I2C DMA priority */

#ifdef CONFIG_STM32_I2C_DMA

# if defined(CONFIG_I2C_DMAPRIO)
#   if (CONFIG_I2C_DMAPRIO & ~DMA_SCR_PL_MASK) != 0
#     error "Illegal value for CONFIG_I2C_DMAPRIO"
#   endif
#   define I2C_DMA_PRIO     CONFIG_I2C_DMAPRIO
# else
#   define I2C_DMA_PRIO     DMA_SCR_PRIMED
# endif

#endif

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
  I2CEVENT_NONE = 0,
  I2CEVENT_STATE_ERROR,
  I2CEVENT_ISR_SHUTDOWN,
  I2CEVENT_ISR_CALL,
  I2CEVENT_ISR_EMPTY_CALL,
  I2CEVENT_MSG_HANDLING,
  I2CEVENT_POLL_NOT_READY,
  I2CEVENT_EMPTY_MSG,
  I2CEVENT_START,
  I2CEVENT_SENDADDR,
  I2CEVENT_ADDRESS_ACKED,
  I2CEVENT_ADDRESS_NACKED,
  I2CEVENT_NACK,
  I2CEVENT_READ,
  I2CEVENT_READ_ERROR,
  I2CEVENT_ADDRESS_ACKED_READ_1,
  I2CEVENT_ADDRESS_ACKED_READ_2,
  I2CEVENT_WRITE_TO_DR,
  I2CEVENT_WRITE_STOP,
  I2CEVENT_WRITE_RESTART,
  I2CEVENT_WRITE_NO_RESTART,
  I2CEVENT_WRITE_ERROR,
  I2CEVENT_WRITE_FLAG_ERROR,
  I2CEVENT_TC_RESTART,
  I2CEVENT_TC_NO_RESTART,
  I2CEVENT_ERROR
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
  mutex_t lock;                /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum stm32_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  uint32_t frequency;          /* Current I2C frequency */
  volatile int dcnt;           /* Current message length */
  uint16_t flags;              /* Current message flags */
  bool check_addr_ack;         /* Flag to signal if on next interrupt address has ACKed */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct stm32_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */

  /* I2C DMA support */

#ifdef CONFIG_STM32_I2C_DMA
  DMA_HANDLE      txdma;       /* TX DMA handle */
  DMA_HANDLE      rxdma;       /* RX DMA handle */
  uint8_t         txch;        /* TX DMA channel */
  uint8_t         rxch;        /* RX DMA channel */
#endif
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

static int stm32_i2c_isr_process(struct stm32_i2c_priv_s *priv);

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

/* DMA support */

#ifdef CONFIG_STM32_I2C_DMA
static void stm32_i2c_dmarxcallback(DMA_HANDLE handle,
                                    uint8_t status, void *arg);
static void stm32_i2c_dmatxcallback(DMA_HANDLE handle,
                                    uint8_t status, void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_STM32_I2C_DMA
#  ifndef CONFIG_STM32_DMA1
#     error "I2C1 enabled with DMA but corresponding DMA controller 1 is not enabled!"
#  endif
  /* TODO: ch for i2c 1 and 2 could be *X_2 based on stream priority */

  .rxch       = DMAMAP_I2C1_RX,
  .txch       = DMAMAP_I2C1_TX,
#endif
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
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_STM32_I2C_DMA
# ifndef CONFIG_STM32_DMA1
#   error "I2C2 enabled with DMA but corresponding DMA controller 1 is not enabled!"
# endif
  .rxch       = DMAMAP_I2C2_RX,
  .txch       = DMAMAP_I2C2_TX,
#endif
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
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_STM32_I2C_DMA
# ifndef CONFIG_STM32_DMA1
#   error "I2C3 enabled with DMA but corresponding DMA controller 1 is not enabled!"
# endif
  .rxch       = DMAMAP_I2C3_RX,
  .txch       = DMAMAP_I2C3_TX,
#endif
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
             "%2d. STATUS: %08x COUNT: %3d EVENT: %2d PARM: %08x TIME: %d\n",
             i + 1, trace->status, trace->count, trace->event, trace->parm,
             (int)(trace->time - priv->start_time));
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

  stm32_i2c_modifyreg(priv,
                      STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, I2C_CR1_START);
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
  uint32_t status;
#ifndef CONFIG_I2C_POLLED
  uint32_t regval;
#endif
#ifdef CONFIG_STM32_I2C_DMA
  uint16_t cr2;
#endif

  i2cinfo("I2C ISR called\n");

  /* Get state of the I2C controller (register SR1 only)
   *
   * Get control register SR1 only as reading both SR1 and SR2 clears the
   * ADDR flag(possibly others) causing the hardware to advance to the next
   * state without the proper action being taken.
   */

  status = stm32_i2c_getreg(priv, STM32_I2C_SR1_OFFSET);

  /* Update private version of the state */

  priv->status = status;

  /* Check if this is a new transmission so to set up the
   * trace table accordingly.
   */

  stm32_i2c_tracenew(priv, status);
  stm32_i2c_traceevent(priv, I2CEVENT_ISR_CALL, 0);

  /* Messages handling (1/2)
   *
   * Message handling should only operate when a message has been completely
   * sent and after the ISR had the chance to run to set bits after the last
   * written/read byte, i.e. priv->dcnt == -1. This is also the case in when
   * the ISR is called for the first time. This can seen in
   * stm32_i2c_process() before entering the stm32_i2c_sem_waitdone() waiting
   * process.
   *
   * Message handling should only operate when:
   *     - A message has been completely sent and there are still messages
   *       to send(i.e. msgc > 0).
   *     - After the ISR had the chance to run to set start bit or
   *       termination flags after the last written/read byte(after last byte
   *       dcnt=0, msg handling dcnt = -1).
   *
   * When the ISR is called for the first time the same conditions hold.
   * This can seen in stm32_i2c_process() before entering the
   * stm32_i2c_sem_waitdone() waiting process.
   */

#ifdef CONFIG_STM32_I2C_DMA
  /* If ISR gets called (ex. polling mode) while DMA is still in
   * progress, we should just return and let the DMA finish.
   */

  cr2 = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  if ((cr2 & I2C_CR2_DMAEN) != 0)
    {
#ifdef CONFIG_DEBUG_I2C_INFO
      size_t left = stm32_dmaresidual(priv->rxdma);

      i2cinfo("DMA in progress: %ld [bytes] remainining. Returning.\n",
              left);
#endif
      return OK;
    }
#endif

  if (priv->dcnt == -1 && priv->msgc > 0)
    {
      /* Any new message should begin with "Start" condition
       * However there were 2 situations where that was not true
       * Situation 1:
       *    Next message continue transmission sequence of previous message
       *
       * Situation 2: If an error is injected that looks like a STOP the
       * interrupt will be reentered with some status that will be incorrect.
       * This will ensure that the error handler will clear the interrupt
       * enables and return the error to the waiting task.
       */

      if (((priv->msgv[0].flags & I2C_M_NOSTART) != 0 &&
           (status & I2C_SR1_TXE) == 0) ||
          ((priv->msgv[0].flags & I2C_M_NOSTART) == 0 &&
           (status & I2C_SR1_SB) == 0))
        {
#if defined(CONFIG_STM32_I2C_DMA) || defined(CONFIG_I2C_POLLED)
          return OK;
#else
          priv->status |= I2C_SR1_TIMEOUT;
          goto state_error;
#endif
        }

      i2cinfo("Switch to new message\n");

      /* Get current message to process data and copy to private structure */

      priv->ptr           = priv->msgv->buffer;   /* Copy buffer to private struct */
      priv->dcnt          = priv->msgv->length;   /* Set counter of current msg length */
      priv->flags         = priv->msgv->flags;    /* Copy flags to private struct */

      i2cinfo("Current flags %i\n", priv->flags);

      /* Decrease counter to indicate the number of messages left to
       * process
       */

      priv->msgc--;

      /* Decrease message pointer.
       * If last message set next message vector to null
       */

      if (priv->msgc == 0)
        {
          /* No more messages, don't need to increment msgv. This pointer
           * will be set to zero when reaching the termination of the ISR
           * calls, i.e.  Messages handling(2/2).
           */
        }
      else
        {
          /* If not last message increment to next message to process */

          priv->msgv++;
        }

      /* Trace event */

      stm32_i2c_traceevent(priv, I2CEVENT_MSG_HANDLING, priv->msgc);
    }

  /* Note the event where we are on the last message and after the last
   * byte is handled at the bottom of this function, as it terminates
   * the repeated calls to the ISR.
   */

  /* I2C protocol logic
   *
   * I2C protocol logic follows. It's organized in an if else chain such that
   * only one mode of operation is executed every time the ISR is called.
   */

  /* Address Handling
   *
   * Check if a start bit was set and transmit address with proper format.
   *
   * Note:
   * On first call the start bit has been set by stm32_i2c_waitdone()
   * Otherwise it will be set from this ISR.
   *
   * Remember that after a start bit an address has always to be sent.
   */

  if ((status & I2C_SR1_SB) != 0)
    {
      /* Start bit is set */

      i2cinfo("Entering address handling, status = %" PRIi32 "\n", status);

      /* Check for empty message (for robustness) */

      if (priv->dcnt > 0)
        {
          /* Set POS bit to zero (can be up from a previous 2 byte receive) */

          stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_POS, 0);

          /* ACK is the expected answer for N>=3 reads and writes */

          stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, 0, I2C_CR1_ACK);

          /* Send address byte with correct 8th bit set
           * (for writing or reading)
           * Transmission happens after having written to the data register
           * STM32_I2C_DR
           */

          stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET,
                           (priv->flags & I2C_M_TEN) ?
                           0 : ((priv->msgv->addr << 1) |
                           (priv->flags & I2C_M_READ)));

          i2cinfo("Address sent. Addr=%#02x Write/Read bit=%i\n",
                  priv->msgv->addr, (priv->flags & I2C_M_READ));

          /* Flag that address has just been sent */

          priv->check_addr_ack = true;

          stm32_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgv->addr);
        }
      else
        {
          /* TODO: untested!! */

          i2cwarn(" An empty message has been detected, "
                  "ignoring and passing to next message.\n");

          /* Trace event */

          stm32_i2c_traceevent(priv, I2CEVENT_EMPTY_MSG, 0);

          /* Set condition to activate msg handling */

          priv->dcnt = -1;

#ifndef CONFIG_I2C_POLLED
          /* Restart ISR by setting an interrupt buffer bit */

          stm32_i2c_modifyreg(priv,
                              STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
#endif
        }
    }

  /* Address cleared event
   *
   * Check if the address cleared, i.e. the driver found a valid address.
   * If a NACK was received the address is invalid, if an ACK was
   * received the address is valid and transmission can continue.
   */

  /* Check for NACK after an address */

#ifndef CONFIG_I2C_POLLED
  /* When polling the i2c ISR it's not possible to determine when
   * an address has been ACKed(i.e. the address is valid).
   *
   * The mechanism to deal a NACKed address is to wait for the I2C
   * call to timeout (value defined in defconfig by one of the
   * following: CONFIG_STM32_I2C_DYNTIMEO, CONFIG_STM32_I2CTIMEOSEC,
   * CONFIG_STM32_I2CTIMEOMS, CONFIG_STM32_I2CTIMEOTICKS).
   *
   * To be safe in the case of a timeout/NACKed address a stop bit
   * is set on the bus to clear it. In POLLED operation it's done
   * stm32_i2c_process() after the call to stm32_i2c_sem_waitdone().
   *
   * In ISR driven operation the stop bit in case of a NACKed address
   * is set in the ISR itself.
   *
   * Note: this commentary is found in both places.
   */

  else if ((status & I2C_SR1_ADDR) == 0 && priv->check_addr_ack)
    {
      i2cinfo("Invalid Address. Setting stop bit and clearing message\n");
      i2cinfo("status %" PRIi32 "\n", status);

      /* Set condition to terminate msg chain transmission as address is
       * invalid.
       */

      priv->dcnt = -1;
      priv->msgc = 0;

      i2cinfo("dcnt %i , msgc %i\n", priv->dcnt, priv->msgc);

      /* Reset flag to check for valid address */

      priv->check_addr_ack = false;

      /* Send stop bit to clear bus */

      stm32_i2c_sendstop(priv);

      /* Trace event */

      stm32_i2c_traceevent(priv, I2CEVENT_ADDRESS_NACKED, priv->msgv->addr);
    }
#endif

  /* ACK in read mode, ACK in write mode is handled separately */

  else if ((priv->flags & I2C_M_READ) != 0 && (status & I2C_SR1_ADDR) != 0 &&
           priv->check_addr_ack)
    {
      /* Reset check addr flag as we are handling this event */

      priv->check_addr_ack = false;

      /* Note:
       *
       * When reading a single byte the stop condition has  to be set
       * immediately after clearing the state flags, which happens
       * when reading SR2(as SR1 has already been read).
       *
       * Similarly when reading 2 bytes the NACK bit has to be set as just
       * after the clearing of the address.
       */

      if (priv->dcnt == 1)
        {
          /* this should only happen when receiving a message of length 1 */

          i2cinfo("short read N=1: setting NACK\n");

          /* Set POS bit to zero (can be up from a previous 2 byte receive) */

          stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_POS, 0);

          /* Immediately set NACK */

          stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, 0);

#ifndef CONFIG_I2C_POLLED
          /* Enable RxNE and TxE buffers in order to receive one or multiple
           * bytes
           */

          stm32_i2c_modifyreg(priv,
                              STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
#endif

          /* Clear ADDR flag by reading SR2 and adding it to status */

          status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);

          /* Send Stop/Restart */

          if (priv->msgc > 0)
            {
              stm32_i2c_sendstart(priv);
            }
          else
            {
              stm32_i2c_sendstop(priv);
            }

          i2cinfo("Address ACKed beginning data reception\n");
          i2cinfo("short read N=1: programming stop bit\n");

          /* Trace */

          stm32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED_READ_1, 0);
        }
      else if (priv->dcnt == 2)
        {
          /* This should only happen when receiving a message of length 2 */

          /* Set POS bit to zero (can be up from a previous 2 byte receive) */

          stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, 0, I2C_CR1_POS);

          /* Immediately set NACK */

          stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, 0);

          /* Clear ADDR flag by reading SR2 and adding it to status */

          status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);

          i2cinfo("Address ACKed beginning data reception\n");
          i2cinfo("short read N=2: programming NACK\n");

          /* Trace */

          stm32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED_READ_2, 0);
        }
      else
        {
          i2cinfo("Address ACKed beginning data reception\n");

          /* Clear ADDR flag by reading SR2 and adding it to status */

          status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);

          /* Trace */

          stm32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED, 0);

#ifdef CONFIG_STM32_I2C_DMA
          /* DMA only when not doing a short read */

          i2cinfo("Starting dma transfer and disabling interrupts\n");

          /* The DMA must be initialized and enabled before the I2C data
           * transfer.
           * The DMAEN bit must be set in the I2C_CR2 register before the
           * ADDR event.
           */

          stm32_dmasetup(priv->rxdma,
                         priv->config->base + STM32_I2C_DR_OFFSET,
                         (uint32_t)priv->ptr, priv->dcnt,
                         DMA_SCR_DIR_P2M |
                         DMA_SCR_MSIZE_8BITS |
                         DMA_SCR_PSIZE_8BITS |
                         DMA_SCR_MINC |
                         I2C_DMA_PRIO);

          /* Do not enable the ITBUFEN bit in the I2C_CR2 register if DMA is
           * used.
           */

          stm32_i2c_modifyreg(priv,
                              STM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);

#ifndef CONFIG_I2C_POLLED
          /* Now let DMA do all the work, disable i2c interrupts */

          regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
          regval &= ~I2C_CR2_ALLINTS;
          stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);
#endif

          /* The user can generate a Stop condition in the DMA Transfer
           * Complete interrupt routine if enabled. This will be done in
           * the dma rx callback Start DMA.
           */

          stm32_dmastart(priv->rxdma, stm32_i2c_dmarxcallback, priv, false);
          stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_DMAEN);
#else
#ifndef CONFIG_I2C_POLLED
          if (priv->dcnt > 3)
            {
              /* Don't enable I2C_CR2_ITBUFEN for messages longer than 3
               * bytes
               */

              stm32_i2c_modifyreg(priv,
                                  STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
            }
#endif
#endif
        }
    }

  /* Write mode
   *
   * Handles all write related I2C protocol logic. Also handles the
   * ACK event after clearing the ADDR flag as the write has to
   * begin immediately after.
   */

  else if ((priv->flags & I2C_M_READ) == 0 &&
           (status & I2C_SR1_BTF) != 0 &&
            priv->dcnt == 0)
    {
      /* After last byte, check what to do based on next message flags */

      if (priv->msgc == 0)
        {
          /* If last message send stop bit */

          stm32_i2c_sendstop(priv);
          i2cinfo("Stop sent dcnt = %i msgc = %i\n", priv->dcnt, priv->msgc);

          /* Decrease counter to get to next message */

          priv->dcnt--;
          i2cinfo("dcnt %i\n", priv->dcnt);
          stm32_i2c_traceevent(priv, I2CEVENT_WRITE_STOP, priv->dcnt);
        }

      /* If there is a next message with no flags or the read flag
       * a restart sequence has to be sent.
       * Note msgv already points to the next message.
       */

      else if (priv->msgc > 0 &&
               (priv->msgv->flags == 0 ||
               (priv->msgv[0].flags & I2C_M_READ) != 0))
        {
          /* Send start */

          stm32_i2c_sendstart(priv);

          stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);

          i2cinfo("Restart detected!\n");
          i2cinfo("Nextflag %i\n", priv->msgv[0].flags);

          /* Decrease counter to get to next message */

          priv->dcnt--;
          i2cinfo("dcnt %i\n", priv->dcnt);
          stm32_i2c_traceevent(priv, I2CEVENT_WRITE_RESTART, priv->dcnt);
        }
      else
        {
          i2cinfo("Write mode: next message has an unrecognized flag.\n");
          stm32_i2c_traceevent(priv,
                               I2CEVENT_WRITE_FLAG_ERROR, priv->msgv->flags);
        }

      status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);
    }
  else if ((priv->flags & I2C_M_READ) == 0 &&
           (status & (I2C_SR1_ADDR | I2C_SR1_TXE)) != 0 &&
            priv->dcnt != 0)
    {
      /* The has cleared(ADDR is set, ACK was received after the address)
       * or the transmit buffer is empty flag has been set(TxE) then we can
       * transmit the next byte.
       */

      i2cinfo("Entering write mode dcnt = %i msgc = %i\n",
              priv->dcnt, priv->msgc);

      /* Clear ADDR flag by reading SR2 and adding it to status */

      status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);

      /* Address has cleared so don't check on next call */

      priv->check_addr_ack = false;

      /* Check if we have transmitted the whole message or we are after
       * the last byte where the stop condition or else(according to the
       * msg flags) has to be set.
       */

#ifdef CONFIG_STM32_I2C_DMA
      /* if DMA is enabled, only makes sense to make use of it for longer
       * than 1 B transfers.
       */

      if (priv->dcnt > 1)
        {
          i2cinfo("Starting DMA transfer and disabling interrupts\n");

          /* The DMA must be initialized and enabled before the I2C data
           * transfer.  The DMAEN bit must be set in the I2C_CR2 register
           * before the ADDR event.
           */

          stm32_dmasetup(priv->txdma,
                         priv->config->base + STM32_I2C_DR_OFFSET,
                         (uint32_t) priv->ptr, priv->dcnt,
                         DMA_SCR_DIR_M2P |
                         DMA_SCR_MSIZE_8BITS |
                         DMA_SCR_PSIZE_8BITS |
                         DMA_SCR_MINC |
                         I2C_DMA_PRIO);

          /* Do not enable the ITBUFEN bit in the I2C_CR2 register if DMA is
           * used.
           */

          stm32_i2c_modifyreg(priv,
                              STM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);

#ifndef CONFIG_I2C_POLLED
          /* Now let DMA do all the work, disable i2c interrupts */

          regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
          regval &= ~I2C_CR2_ALLINTS;
          stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);
#endif

          /* In the interrupt routine after the EOT interrupt, disable DMA
           * requests then wait for a BTF event before programming the Stop
           * condition. To do this, we'll just call the ISR again in
           * DMA tx callback, in which point we fall into the msgc==0 case
           * which ultimately sends the stop..TODO: but we don't explicitly
           * wait for BTF bit being set...
           * Start DMA.
           */

          stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_DMAEN);
          stm32_dmastart(priv->txdma, stm32_i2c_dmatxcallback, priv, false);
        }
      else
#endif /* CONFIG_STM32_I2C_DMA */
        {
#ifndef CONFIG_I2C_POLLED
          if (priv->dcnt == 1 &&
              (priv->msgc == 0 || (priv->msgv->flags & I2C_M_NOSTART) == 0))
            {
              stm32_i2c_modifyreg(priv,
                                  STM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);
            }
#endif

          /* Transmitting message.
           * Send byte == write data into write register
           */

          stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, *priv->ptr++);

          /* Decrease current message length */

          stm32_i2c_traceevent(priv, I2CEVENT_WRITE_TO_DR, priv->dcnt);
          priv->dcnt--;

          if ((status & I2C_SR1_ADDR) != 0 && priv->dcnt > 0)
            {
              /* Transmitting message.
               * ADDR -> BTF & TXE - Send one more byte
               */

              stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, *priv->ptr++);

              /* Decrease current message length */

              stm32_i2c_traceevent(priv, I2CEVENT_WRITE_TO_DR, priv->dcnt);
              priv->dcnt--;
            }

#ifndef CONFIG_I2C_POLLED
          if (((status & I2C_SR1_ADDR) != 0 && priv->dcnt > 0) ||
              (priv->msgc > 0 && (priv->msgv->flags & I2C_M_NOSTART) != 0))
            {
              stm32_i2c_modifyreg(priv,
                                  STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
            }
#endif

          if (priv->dcnt == 0 &&
              priv->msgc > 0 && (priv->msgv->flags & I2C_M_NOSTART) != 0)
            {
              /* Set condition to get to next message */

              priv->dcnt = -1;
              stm32_i2c_traceevent(priv,
                                   I2CEVENT_WRITE_NO_RESTART, priv->dcnt);
            }
        }
    }

  /* Read mode
   *
   * Handles all read related I2C protocol logic.
   *
   * * * * * * * WARNING STM32F1xx HARDWARE ERRATA * * * * * * *
   *
   * source: https://github.com/hikob/openlab/blob/master/drivers/stm32/i2c.c
   *
   * RXNE-only events should not be handled since it sometimes
   * fails. Only BTF & RXNE events should be handled (with the
   * consequence of slowing down the transfer).
   *
   * It seems that when a RXNE interrupt is handled 'around'
   * the end of the next byte reception, the DR register read
   * is ignored by the i2c controller: it does not flush the
   * DR with next byte
   *
   * Thus we read twice the same byte and we read effectively
   * read one byte less than expected from the i2c slave point
   * of view.
   *
   * Example:
   * + we want to receive 6 bytes (B1 to B6)
   * + the problem appear when reading B3
   * -> we read B1 B2 B3 B3 B4 B5(B3 twice)
   * -> the i2c transfer was B1 B2 B3 B4 B5(B6 is not sent)
   */

  else if ((priv->flags & (I2C_M_READ)) != 0 &&
           (status & (I2C_SR1_RXNE | I2C_SR1_BTF)) != 0)
    {
      /* When read flag is set and the receive buffer is not empty
       * (RXNE is set) then the driver can read from the data register.
       */

      status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);

      i2cinfo("Entering read mode dcnt = %i msgc = %i, "
              "status 0x%04" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);

      /* Byte #N-3W, we don't want to manage RxNE interrupt anymore, bytes
       * N, N-1, N-2 will be read with BTF:
       */

#ifndef CONFIG_I2C_POLLED
      if (priv->dcnt < 5)
        {
          stm32_i2c_modifyreg(priv,
                              STM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);
        }
#else
      if (priv->dcnt == 1 ||
          priv->dcnt > 3 || (status & I2C_SR1_BTF) != 0)
#endif
        {
          /*  BTF: N-2/N-1, set NACK, read N-2 */

          if (priv->dcnt == 3)
            {
              stm32_i2c_modifyreg(priv,
                                  STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, 0);
            }

          /*  BTF: N-1/N, STOP/START, read N-1, N */

          else if (priv->dcnt == 2)
            {
              if (priv->msgc > 0)
                {
                  stm32_i2c_sendstart(priv);
                }
              else
                {
                  stm32_i2c_sendstop(priv);
                }

              /* Read byte #N-1 */

              *priv->ptr++ = stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);
              priv->dcnt--;
            }

          /* Read last or current byte */

          *priv->ptr++ = stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);
          priv->dcnt--;

          if (priv->dcnt == 0)
            {
              priv->dcnt = -1;
            }
        }
    }

  /* Empty call handler
   *
   * Case to handle an empty call to the ISR where it only has to
   * Shutdown
   */

  else if (priv->dcnt == -1 && priv->msgc == 0)
    {
      /* Read rest of the state */

      status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);
      i2cinfo("Empty call to ISR: Stopping ISR\n");
      stm32_i2c_traceevent(priv, I2CEVENT_ISR_EMPTY_CALL, 0);
    }

  /* Error handler
   *
   * Gets triggered if the driver does not recognize a situation(state)
   * it can deal with.
   * This should not happen in interrupt based operation(i.e. when
   * CONFIG_I2C_POLLED is not set in the defconfig file).
   * During polled operation(i.e. CONFIG_I2C_POLLED=y in defconfig)
   * this case should do nothing but tracing the event that the
   * device wasn't ready yet.
   */

  else
    {
#ifdef CONFIG_I2C_POLLED
      stm32_i2c_traceevent(priv, I2CEVENT_POLL_DEV_NOT_RDY, 0);
#else
      /* Read rest of the state */

      status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);

      /* No any error bit is set, but driver is in incorrect state, signal
       * it with "Bus error" bit.
       */

      if ((status & I2C_SR1_ERRORMASK) != 0)
        {
          priv->status |= I2C_SR1_BERR;
        }

      i2cinfo(" No correct state detected(start bit, read or write)\n");
      i2cinfo(" state %" PRIi32 "\n", status);

      /* Set condition to terminate ISR and wake waiting thread */

      priv->dcnt = -1;
      priv->msgc = 0;
      stm32_i2c_traceevent(priv, I2CEVENT_STATE_ERROR, 0);
#endif
    }

  /* Messages handling(2/2)
   *
   * Transmission of the whole message chain has been completed. We have to
   * terminate the ISR and wake up stm32_i2c_process() that is waiting for
   * the ISR cycle to handle the sending/receiving of the messages.
   */

  /* First check for errors */

  if ((status & I2C_SR1_ERRORMASK) != 0)
    {
      stm32_i2c_traceevent(priv, I2CEVENT_ERROR, status & I2C_SR1_ERRORMASK);

      /* Clear interrupt flags */

#if !defined(CONFIG_STM32_I2C_DMA) && !defined(CONFIG_I2C_POLLED)
state_error:
#endif
      stm32_i2c_putreg(priv, STM32_I2C_SR1_OFFSET, 0);

      priv->dcnt = -1;
      priv->msgc = 0;
    }

  if (priv->dcnt == -1 && priv->msgc == 0)
    {
      i2cinfo("Shutting down I2C ISR\n");

      stm32_i2c_traceevent(priv, I2CEVENT_ISR_SHUTDOWN, 0);

      /* Clear internal pointer to the message content.
       * Good practice + done by last implementation when messages are
       * finished (compatibility concerns)
       */

      priv->msgv = NULL;

#ifdef CONFIG_I2C_POLLED
      priv->intstate = INTSTATE_DONE;
#else
      /* Clear all interrupts */

      regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
      regval &= ~I2C_CR2_ALLINTS;
      stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);

      /* Is there a thread waiting for this event(there should be) */

      if (priv->intstate == INTSTATE_WAITING)
        {
          /* Yes.. inform the thread that the transfer is complete
           * and wake it up.
           */

          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#endif
    }

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
 * Name: stm32_i2c_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_I2C_DMA
static void stm32_i2c_dmarxcallback(DMA_HANDLE handle,
                                    uint8_t status, void *arg)
{
#ifndef CONFIG_I2C_POLLED
  uint32_t regval;
#endif

  i2cinfo("DMA rx callback, status = %d\n", status);

  struct stm32_i2c_priv_s *priv = (struct stm32_i2c_priv_s *)arg;

  priv->dcnt = -1;

  /* The user can generate a Stop condition in the DMA Transfer Complete
   * interrupt routine if enabled.
   */

  if (priv->msgc > 0)
    {
      stm32_i2c_sendstart(priv);
    }
  else
    {
      stm32_i2c_sendstop(priv);
    }

  /* Let the I2C periph know to stop DMA transfers, also is used by ISR
   * to check if DMA is done.
   */

  stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_DMAEN, 0);

#ifndef CONFIG_I2C_POLLED
  /* Re-enable interrupts */

  regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  regval |= (I2C_CR2_ITERREN | I2C_CR2_ITEVFEN);
  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);
#endif

  /* let the ISR routine take care of shutting down or switching to
   * next msg
   */

  stm32_i2c_isr_process(priv);
}
#endif /* ifdef CONFIG_STM32_I2C_DMA */

/****************************************************************************
 * Name: stm32_i2c_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_I2C_DMA
static void stm32_i2c_dmatxcallback(DMA_HANDLE handle,
                                    uint8_t status, void *arg)
{
#ifndef CONFIG_I2C_POLLED
  uint32_t regval;
#endif

  i2cinfo("DMA tx callback, status = %d\n", status);

  struct stm32_i2c_priv_s *priv = (struct stm32_i2c_priv_s *)arg;

  priv->dcnt = 0;

  /* In the interrupt routine after the EOT interrupt,
   * disable DMA requests
   */

  stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_DMAEN, 0);

#ifndef CONFIG_I2C_POLLED
  /* re-enable interrupts */

  regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  regval |= (I2C_CR2_ITERREN | I2C_CR2_ITEVFEN);
  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);
#endif
}
#endif /* ifdef CONFIG_STM32_I2C_DMA */

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

  stm32_i2c_putreg(priv,
                   STM32_I2C_CR2_OFFSET, (STM32_PCLK1_FREQUENCY / 1000000));

  /* Force a frequency update */

  priv->frequency = 0;

  stm32_i2c_setclock(priv, 100000);

#ifdef CONFIG_STM32_I2C_DMA
  /* If, in the I2C_CR2 register, the LAST bit is set, I2C automatically
   * sends a NACK after the next byte following EOT_1.
   * Clear DMA en just in case.
   */

  stm32_i2c_modifyreg(priv,
                      STM32_I2C_CR2_OFFSET, I2C_CR2_DMAEN, I2C_CR2_LAST);
#endif

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
  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, 0);

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

#ifdef CONFIG_STM32_I2C_DMA
  /* Disable DMA */

  stm32_dmastop(priv->txdma);
  stm32_dmastop(priv->rxdma);
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

  DEBUGASSERT(count);

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_STM32_I2C_DMA
  /* Stop DMA just in case */

  stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_DMAEN, 0);
  stm32_dmastop(priv->rxdma);
  stm32_dmastop(priv->txdma);
#endif

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

  priv->dcnt   = -1;
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

  nxmutex_unlock(&priv->lock);
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

  ret = nxmutex_lock(&priv->lock);
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

  nxmutex_unlock(&priv->lock);
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
  struct stm32_i2c_priv_s *priv = NULL;

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

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      stm32_i2c_init(priv);

#ifdef CONFIG_STM32_I2C_DMA
      /* Get DMA channels.  NOTE: stm32_dmachannel() will always assign the
       * DMA channel.  If the channel is not available, then
       * stm32_dmachannel() will block and wait until the channel becomes
       * available.
       * WARNING: If you have another device sharing a DMA channel with SPI
       * and the code never releases that channel, then the call to
       * stm32_dmachannel()  will hang forever in this function!
       *  Don't let your design do that!
       */

      priv->rxdma = stm32_dmachannel(priv->rxch);
      priv->txdma = stm32_dmachannel(priv->txch);
      DEBUGASSERT(priv->rxdma && priv->txdma);
#endif /* CONFIG_STM32_I2C_DMA */
    }

  nxmutex_unlock(&priv->lock);
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

  stm32_i2c_deinit(priv);

#ifdef CONFIG_STM32_I2C_DMA
  stm32_dmafree(priv->rxdma);
  stm32_dmafree(priv->txdma);
#endif

  nxmutex_unlock(&priv->lock);
  return OK;
}

#endif /* CONFIG_STM32_STM32F10XX || CONFIG_STM32_STM32F20XX || CONFIG_STM32_STM32F4XXX */
#endif /* CONFIG_STM32_I2C1 || CONFIG_STM32_I2C2 || CONFIG_STM32_I2C3 */
