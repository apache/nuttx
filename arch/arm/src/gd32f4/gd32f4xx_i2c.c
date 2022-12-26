/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_i2c.c
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
 *    hardware using the I2C_CTL0_SRESET)
 *  - SMBus support (hardware layer timings are already supported) and add
 *    SMBA gpio pin
 *  - Slave support with multiple addresses (on multiple instances):
 *      - 2 x 7-bit address or
 *      - 1 x 10 bit addresses + 1 x 7 bit address (for the slave in
 *        Dual-Address mode)
 *      - plus the broadcast address (general call)
 *  - Multi-master support
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

#include "chip.h"
#include "gd32f4xx.h"
#include "gd32f4xx_i2c.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_GD32F4_I2C0) || defined(CONFIG_GD32F4_I2C1) || \
    defined(CONFIG_GD32F4_I2C2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_GD32F4_I2C_TIMEOSEC) && !defined(CONFIG_GD32F4_I2C_TIMEOMS)
#  define CONFIG_GD32F4_I2C_TIMEOSEC 0
#  define CONFIG_GD32F4_I2C_TIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_GD32F4_I2C_TIMEOSEC)
#  define CONFIG_GD32F4_I2C_TIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_GD32F4_I2C_TIMEOMS)
#  define CONFIG_GD32F4_I2C_TIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_GD32F4_I2CTIMEOTICKS
#  define CONFIG_GD32F4_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_GD32F4_I2C_TIMEOSEC) + \
     MSEC2TICK(CONFIG_GD32F4_I2C_TIMEOMS))
#endif

#ifndef CONFIG_GD32F4_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_GD32F4_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_GD32F4_I2CTIMEOTICKS)
#endif

/* Macros to convert a I2C pin to a GPIO output */

#define I2C_OUTPUT (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_OD |\
                    GPIO_CFG_SPEED_50MHZ | GPIO_CFG_OUTPUT_SET)

#define MKI2C_OUTPUT(p) (((p) & (GPIO_CFG_PORT_MASK | GPIO_CFG_PIN_MASK)) |\
                         I2C_OUTPUT)

/* I2C DMA priority */

#ifdef CONFIG_GD32F4_I2C_DMA

#  error "Now I2C DMA has not ready"

# if defined(CONFIG_I2C_DMAPRIO)
#   if (CONFIG_I2C_DMAPRIO & ~DMA_CHXCTL_PRIO_MASK) != 0
#     error "Illegal value for CONFIG_I2C_DMAPRIO"
#   endif
#   define I2C_DMA_PRIO     CONFIG_I2C_DMAPRIO
# else
#   define I2C_DMA_PRIO     DMA_PRIO_HIGH_SELECT
# endif
#endif

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define gd32_i2c_tracereset(p)
#  define gd32_i2c_tracenew(p,s)
#  define gd32_i2c_traceevent(p,e,a)
#  define gd32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum gd32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum gd32_trace_e
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

struct gd32_trace_s
{
  uint32_t status;             /* I2C 32-bit STAT0|STAT1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum gd32_intstate_e event;  /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  clock_t time;                /* First of event or first status */
};

/* I2C Device hardware configuration */

struct gd32_i2c_config_s
{
  uint32_t i2c_base;          /* I2C base address */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  uint32_t event_irq;         /* Event IRQ */
  uint32_t error_irq;         /* Error IRQ */
#endif
};

/* I2C Device Private Data */

struct gd32_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct gd32_i2c_config_s *config;

  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum gd32_intstate_e) */

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

  struct gd32_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer STAT0|STAT1 status */

  /* I2C DMA support */

#ifdef CONFIG_GD32F4_I2C_DMA
  DMA_HANDLE      txdma;       /* TX DMA handle */
  DMA_HANDLE      rxdma;       /* RX DMA handle */
  uint8_t         txch;        /* TX DMA channel number */
  uint8_t         rxch;        /* RX DMA channel number */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Clock */

static void            gd32_i2c_periph_reset(uint32_t i2cbase);
static void            gd32_i2c_clock_enable(uint32_t i2cbase);
static void            gd32_i2c_clock_disable(uint32_t i2cbase);

static inline uint16_t gd32_i2c_getreg(struct gd32_i2c_priv_s *priv,
                                       uint8_t offset);
static inline void     gd32_i2c_putreg(struct gd32_i2c_priv_s *priv,
                                       uint8_t offset, uint16_t value);
static inline void     gd32_i2c_modifyreg(struct gd32_i2c_priv_s *priv,
                                          uint8_t offset, uint16_t clearbits,
                                          uint16_t setbits);

#ifdef CONFIG_GD32F4_I2C_DYNTIMEO
static uint32_t    gd32_i2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_GD32F4_I2C_DYNTIMEO */

static inline int  gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_waitstop(struct gd32_i2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void  gd32_i2c_tracereset(struct gd32_i2c_priv_s *priv);
static void  gd32_i2c_tracenew(struct gd32_i2c_priv_s *priv,
                               uint32_t status);
static void  gd32_i2c_traceevent(struct gd32_i2c_priv_s *priv,
                                 enum gd32_trace_e event, uint32_t parm);
static void  gd32_i2c_tracedump(struct gd32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void        gd32_i2c_setclock(struct gd32_i2c_priv_s *priv,
                                     uint32_t frequency);
static inline void gd32_i2c_sendstart(struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_clrstart(struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sendstop(struct gd32_i2c_priv_s *priv);
static inline
uint32_t           gd32_i2c_getstatus(struct gd32_i2c_priv_s *priv);

static int         gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
static int         gd32_i2c_isr(int irq, void *context, void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int         gd32_i2c_init(struct gd32_i2c_priv_s *priv);
static int         gd32_i2c_deinit(struct gd32_i2c_priv_s *priv);
static int         gd32_i2c_transfer(struct i2c_master_s *dev,
                                     struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int         gd32_i2c_reset(struct i2c_master_s *dev);
#endif

/* DMA support */

#ifdef CONFIG_GD32F4_I2C_DMA
static void        gd32_i2c_dmarxcallback(DMA_HANDLE handle,
                                          uint8_t status, void *arg);
static void        gd32_i2c_dmatxcallback(DMA_HANDLE handle,
                                          uint8_t status, void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s gd32_i2c_ops =
{
  .transfer = gd32_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset  = gd32_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_GD32F4_I2C0
static const struct gd32_i2c_config_s gd32_i2c0_config =
{
  .i2c_base   = GD32_I2C0,
  .scl_pin    = GPIO_I2C0_SCL,
  .sda_pin    = GPIO_I2C0_SDA,
#ifndef CONFIG_I2C_POLLED
  .event_irq  = GD32_IRQ_I2C0_EV,
  .error_irq  = GD32_IRQ_I2C0_ER
#endif
};

static struct gd32_i2c_priv_s gd32_i2c0_priv =
{
  .ops        = &gd32_i2c_ops,
  .config     = &gd32_i2c0_config,
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
  .status     = 0,
#ifdef CONFIG_GD32F4_I2C_DMA
  .rxch       = DMA_REQ_I2C0_RX,
  .txch       = DMA_REQ_I2C0_TX,
#endif
};
#endif

#ifdef CONFIG_GD32F4_I2C1
static const struct gd32_i2c_config_s gd32_i2c1_config =
{
  .i2c_base   = GD32_I2C1,
  .scl_pin    = GPIO_I2C1_SCL,
  .sda_pin    = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .event_irq  = GD32_IRQ_I2C1_EV,
  .error_irq  = GD32_IRQ_I2C1_ER
#endif
};

static struct gd32_i2c_priv_s gd32_i2c1_priv =
{
  .ops        = &gd32_i2c_ops,
  .config     = &gd32_i2c1_config,
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
  .status     = 0,
#ifdef CONFIG_GD32F4_I2C_DMA
  .rxch       = DMA_REQ_I2C1_RX,
  .txch       = DMA_REQ_I2C1_TX,
#endif
};
#endif

#ifdef CONFIG_GD32F4_I2C2
static const struct gd32_i2c_config_s gd32_i2c2_config =
{
  .i2c_base   = GD32_I2C2,
  .scl_pin    = GPIO_I2C2_SCL,
  .sda_pin    = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_POLLED
  .event_irq  = GD32_IRQ_I2C2_EV,
  .error_irq  = GD32_IRQ_I2C2_ER
#endif
};

static struct gd32_i2c_priv_s gd32_i2c2_priv =
{
  .ops        = &gd32_i2c_ops,
  .config     = &gd32_i2c2_config,
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
  .status     = 0,
#ifdef CONFIG_GD32F4_I2C_DMA
  .rxch       = DMA_REQ_I2C2_RX,
  .txch       = DMA_REQ_I2C2_TX,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2c_periph_reset
 *
 * Description:
 *   Reset the I2C.
 *
 ****************************************************************************/

static void gd32_i2c_periph_reset(uint32_t i2cbase)
{
  uint32_t rcu_rst;
  uint32_t regaddr;

  /* Determine which I2C to configure */

  switch (i2cbase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_I2C0
    case GD32_I2C0:
      rcu_rst = RCU_APB1RST_I2C0RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_I2C1
    case GD32_I2C1:
      rcu_rst = RCU_APB1RST_I2C1RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_I2C2
    case GD32_I2C2:
      rcu_rst = RCU_APB1RST_I2C2RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
    }

  /* Enable APB 1/2 reset for I2C */

  modifyreg32(regaddr, 0, rcu_rst);

  /* Disable APB 1/2 reset for I2C */

  modifyreg32(regaddr, rcu_rst, 0);
}

/****************************************************************************
 * Name: gd32_i2c_clock_enable
 *
 * Description:
 *   Enable I2C clock
 ****************************************************************************/

static void gd32_i2c_clock_enable(uint32_t i2cbase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which I2C to configure */

  switch (i2cbase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_I2C0
    case GD32_I2C0:
      rcu_en = RCU_APB1EN_I2C0EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_I2C1
    case GD32_I2C1:
      rcu_en = RCU_APB1EN_I2C1EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_I2C2
    case GD32_I2C2:
      rcu_en = RCU_APB1EN_I2C2EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
    }

  /* Enable APB 1/2 clock for I2C */

  modifyreg32(regaddr, 0, rcu_en);
}

/****************************************************************************
 * Name: gd32_i2c_clock_disable
 *
 * Description:
 *   Dinable I2C clock
 ****************************************************************************/

static void gd32_i2c_clock_disable(uint32_t i2cbase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which I2C to configure */

  switch (i2cbase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_I2CI0
    case GD32_I2C0:
      rcu_en = RCU_APB1EN_I2C0EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_I2C1
    case GD32_I2C1:
      rcu_en = RCU_APB1EN_I2C1EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_I2C2
    case GD32_I2C2:
      rcu_en = RCU_APB1EN_I2C2EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
    }

  /* Disable APB 1/2 clock for I2C */

  modifyreg32(regaddr, rcu_en, 0);
}

/****************************************************************************
 * Name: gd32_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t gd32_i2c_getreg(struct gd32_i2c_priv_s *priv,
                                       uint8_t offset)
{
  return getreg16(priv->config->i2c_base + offset);
}

/****************************************************************************
 * Name: gd32_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_i2c_putreg(struct gd32_i2c_priv_s *priv,
                                   uint8_t offset, uint16_t value)
{
  putreg16(value, priv->config->i2c_base + offset);
}

/****************************************************************************
 * Name: gd32_i2c_modifyreg
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_i2c_modifyreg(struct gd32_i2c_priv_s *priv,
                                      uint8_t offset, uint16_t clearbits,
                                      uint16_t setbits)
{
  modifyreg16(priv->config->i2c_base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: gd32_i2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_I2C_DYNTIMEO
static uint32_t gd32_i2c_toticks(int msgc, struct i2c_msg_s *msgs)
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

  return USEC2TICK(CONFIG_GD32F4_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: gd32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv)
{
  irqstate_t flags;
  uint32_t regval;
  int ret;

  flags = enter_critical_section();

  /* Enable I2C interrupts */

  regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
  regval |= (I2C_CTL1_ERRIE | I2C_CTL1_EVIE);
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_tickwait_uninterruptible() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Wait until either the transfer is complete or the timeout expires */

#ifdef CONFIG_GD32F4_I2C_DYNTIMEO
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                           gd32_i2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_GD32F4_I2C_TIMEOTICKS);
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

  regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
  regval &= ~I2C_CTL1_INTS_MASK;
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);

  leave_critical_section(flags);
  return ret;
}
#else
static inline int gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_GD32F4_I2C_DYNTIMEO
  timeout = gd32_i2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_GD32F4_I2C_TIMEOTICKS;
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

      gd32_i2c_isr_process(priv);
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
 * Name: gd32_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ****************************************************************************/

static inline void gd32_i2c_sem_waitstop(struct gd32_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t ctl0;
  uint32_t stat0;

  /* Select a timeout */

#ifdef CONFIG_GD32F4_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_GD32F4_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_GD32F4_I2C_TIMEOTICKS;
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

      ctl0 = gd32_i2c_getreg(priv, GD32_I2C_CTL0_OFFSET);
      if ((ctl0 & I2C_CTL0_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      stat0 = gd32_i2c_getreg(priv, GD32_I2C_STAT0_OFFSET);
      if ((stat0 & I2C_STAT0_SMBTO) != 0)
        {
          return;
        }
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.
   */

  i2cinfo("Timeout with CTL0: %04" PRIx32 " STAT0: %04" PRIx32 "\n", \
          ctl0, stat0);
}

/****************************************************************************
 * Name: gd32_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void gd32_i2c_traceclear(struct gd32_i2c_priv_s *priv)
{
  struct gd32_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit STAT1|STAT0 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with
                                   * this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void gd32_i2c_tracereset(struct gd32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  gd32_i2c_traceclear(priv);
}

static void gd32_i2c_tracenew(struct gd32_i2c_priv_s *priv,
                              uint32_t status)
{
  struct gd32_trace_s *trace = &priv->trace[priv->tndx];

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

      gd32_i2c_traceclear(priv);
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

static void gd32_i2c_traceevent(struct gd32_i2c_priv_s *priv,
                                enum gd32_trace_e event, uint32_t parm)
{
  struct gd32_trace_s *trace;

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
      gd32_i2c_traceclear(priv);
    }
}

static void gd32_i2c_tracedump(struct gd32_i2c_priv_s *priv)
{
  struct gd32_trace_s *trace;
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
 * Name: gd32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void gd32_i2c_setclock(struct gd32_i2c_priv_s *priv,
                              uint32_t frequency)
{
  uint16_t ctl0;
  uint16_t ckfg;
  uint16_t trise;
  uint16_t freq_mhz;
  uint16_t speed;

  /* Has the I2C bus frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Disable the selected I2C peripheral to configure TRISE */

      ctl0 = gd32_i2c_getreg(priv, GD32_I2C_CTL0_OFFSET);
      gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, ctl0 & ~I2C_CTL0_I2CEN);

      /* Update timing and control registers */

      freq_mhz = (uint16_t)(GD32_PCLK1_FREQUENCY / 1000000);
      ckfg = 0;

      /* Configure speed in standard mode */

      if (frequency <= 100000)
        {
          /* Standard mode speed calculation */

          speed = (uint16_t)(GD32_PCLK1_FREQUENCY / (frequency << 1));

          /* The CKCFG fault must be >= 4 */

          if (speed < 4)
            {
              /* Set the minimum allowed value */

              speed = 4;
            }

          ckfg |= speed;

          /* Set Maximum Rise Time for standard mode */

          trise = freq_mhz + 1;
        }

      /* Configure speed in fast mode */

      else /* (frequency <= 400000) */
        {
          /* Fast mode speed calculation with Tlow/Thigh = 16/9 */

#ifdef CONFIG_GD32F4_I2C_DUTY16_9
          speed = (uint16_t)(GD32_PCLK1_FREQUENCY / (frequency * 25));

          /* Set DUTY and fast speed bits */

          ckfg |= (I2C_CKCFG_DTCY | I2C_CKCFG_FAST);
#else
          /* Fast mode speed calculation with Tlow/Thigh = 2 */

          speed = (uint16_t)(GD32_PCLK1_FREQUENCY / (frequency * 3));

          /* Set fast speed bit */

          ckfg |= I2C_CKCFG_FAST;
#endif

          /* Verify that the CKCFG speed value is nonzero */

          if (speed < 1)
            {
              /* Set the minimum allowed value */

              speed = 1;
            }

          ckfg |= speed;

          /* Set Maximum Rise Time for fast mode */

          trise = (uint16_t)(((freq_mhz * 300) / 1000) + 1);
        }

      /* Write the new values of the CKCFG and TRISE registers */

      gd32_i2c_putreg(priv, GD32_I2C_CKCFG_OFFSET, ckfg);
      gd32_i2c_putreg(priv, GD32_I2C_RT_OFFSET, trise);

      /* Re-enable the peripheral (or not) */

      gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, ctl0);

      /* Save the new I2C frequency */

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: gd32_i2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ****************************************************************************/

static inline void gd32_i2c_sendstart(struct gd32_i2c_priv_s *priv)
{
  /* Generate START */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                     0, I2C_CTL0_START);
}

/****************************************************************************
 * Name: gd32_i2c_clrstart
 *
 * Description:
 *   Clear the STOP, START or PECTRANS condition on certain error
 *   recovery steps.
 *
 ****************************************************************************/

static inline void gd32_i2c_clrstart(struct gd32_i2c_priv_s *priv)
{
  /* "Note: When the STOP, START or PECTRANS bit is set, the software must
   *  not perform any write access to I2C_CTL0 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PECTRANS request."
   *
   * "The [STOP] bit is set and cleared by software, cleared by hardware
   *  when a Stop condition is detected, set by hardware when a timeout
   *  error is detected.
   *
   * "This [START] bit is set and cleared by software and cleared by hardware
   *  when start is sent or I2CEN=0." The bit must be cleared by software if
   *  the START is never sent.
   *
   * "This [PECTRANS] bit is set and cleared by software, and cleared by
   *  hardware when PECTRANS is transferred or by a START or Stop condition
   *  or when I2CEN=0."
   */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                     I2C_CTL0_START | I2C_CTL0_STOP | I2C_CTL0_PECTRANS, 0);
}

/****************************************************************************
 * Name: gd32_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ****************************************************************************/

static inline void gd32_i2c_sendstop(struct gd32_i2c_priv_s *priv)
{
  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_ACKEN, \
                     I2C_CTL0_STOP);
}

/****************************************************************************
 * Name: gd32_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status (STAT0 and STAT1 combined)
 *
 ****************************************************************************/

static inline uint32_t gd32_i2c_getstatus(struct gd32_i2c_priv_s *priv)
{
  uint32_t status = gd32_i2c_getreg(priv, GD32_I2C_STAT0_OFFSET);
  status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);
  return status;
}

/****************************************************************************
 * Name: gd32_i2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv)
{
  uint32_t status;
#ifndef CONFIG_I2C_POLLED
  uint32_t regval;
#endif
#ifdef CONFIG_GD32F4_I2C_DMA
  uint16_t ctl1;
#endif

  i2cinfo("I2C ISR called\n");

  /* Get state of the I2C controller (register STAT0 only)
   *
   * Get control register STAT0 only as reading both STAT0 and STAT1
   * clears the ADDR flag(possibly others) causing the hardware to
   * advance to the next state without the proper action being taken.
   */

  status = gd32_i2c_getreg(priv, GD32_I2C_STAT0_OFFSET);

  /* Update private version of the state */

  priv->status = status;

  /* Check if this is a new transmission so to set up the
   * trace table accordingly.
   */

  gd32_i2c_tracenew(priv, status);
  gd32_i2c_traceevent(priv, I2CEVENT_ISR_CALL, 0);

  /* Messages handling (1/2)
   *
   * Message handling should only operate when a message has been completely
   * sent and after the ISR had the chance to run to set bits after the last
   * written/read byte, i.e. priv->dcnt == -1. This is also the case in when
   * the ISR is called for the first time. This can seen in
   * gd32_i2c_process() before entering the gd32_i2c_sem_waitdone() waiting
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
   * This can seen in gd32_i2c_process() before entering the
   * gd32_i2c_sem_waitdone() waiting process.
   */

#ifdef CONFIG_GD32F4_I2C_DMA
  /* If ISR gets called (ex. polling mode) while DMA is still in
   * progress, we should just return and let the DMA finish.
   */

  ctl1 = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
  if ((ctl1 & I2C_CTL1_DMAON) != 0)
    {
#ifdef CONFIG_DEBUG_I2C_INFO
      size_t left = gd32_dma_tansnum_get(priv->rxdma);

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
           (status & I2C_STAT0_TBE) == 0) ||
          ((priv->msgv[0].flags & I2C_M_NOSTART) == 0 &&
           (status & I2C_STAT0_SBSEND) == 0))
        {
#if defined(CONFIG_GD32F4_I2C_DMA) || defined(CONFIG_I2C_POLLED)
          return OK;
#else
          priv->status |= I2C_STAT0_SMBTO;
          goto state_error;
#endif
        }

      i2cinfo("Switch to new message\n");

      /* Get current message to process data and copy to private structure */

      priv->ptr = priv->msgv->buffer;   /* Copy buffer to private struct */
      priv->dcnt = priv->msgv->length;  /* Set counter of current msg length */
      priv->flags = priv->msgv->flags;  /* Copy flags to private struct */

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

      gd32_i2c_traceevent(priv, I2CEVENT_MSG_HANDLING, priv->msgc);
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
   * On first call the start bit has been set by gd32_i2c_waitdone()
   * Otherwise it will be set from this ISR.
   *
   * Remember that after a start bit an address has always to be sent.
   */

  if ((status & I2C_STAT0_SBSEND) != 0)
    {
      /* Start bit is set */

      i2cinfo("Entering address handling, status = %" PRIi32 "\n", status);

      /* Check for empty message (for robustness) */

      if (priv->dcnt > 0)
        {
          /* Send address byte with correct 8th bit set
           * (for writing or reading)
           * Transmission happens after having written to the data register
           * GD32_I2C_DATA
           */

          gd32_i2c_putreg(priv, GD32_I2C_DATA_OFFSET,
                          (priv->flags & I2C_M_TEN) ?
                          0 : ((priv->msgv->addr << 1) |
                          (priv->flags & I2C_M_READ)));

          i2cinfo("Address sent. Addr=%#02x Write/Read bit=%i\n",
                  priv->msgv->addr, (priv->flags & I2C_M_READ));

          /* Flag that address has just been sent */

          priv->check_addr_ack = true;

          gd32_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgv->addr);
        }
      else
        {
          /* TODO: untested!! */

          i2cwarn(" An empty message has been detected, "
                  "ignoring and passing to next message.\n");

          /* Trace event */

          gd32_i2c_traceevent(priv, I2CEVENT_EMPTY_MSG, 0);

          /* Set condition to activate msg handling */

          priv->dcnt = -1;

#ifndef CONFIG_I2C_POLLED
          /* Restart ISR by setting an interrupt buffer bit */

          gd32_i2c_modifyreg(priv,
                             GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_BUFIE);
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
   * following: CONFIG_GD32F4_I2C_DYNTIMEO, CONFIG_GD32F4_I2C_TIMEOSEC,
   * CONFIG_GD32F4_I2C_TIMEOMS, CONFIG_GD32F4_I2C_TIMEOTICKS).
   *
   * To be safe in the case of a timeout/NACKed address a stop bit
   * is set on the bus to clear it. In POLLED operation it's done
   * gd32_i2c_process() after the call to gd32_i2c_sem_waitdone().
   *
   * In ISR driven operation the stop bit in case of a NACKed address
   * is set in the ISR itself.
   *
   * Note: this commentary is found in both places.
   */

  else if ((status & I2C_STAT0_ADDSEND) == 0 && priv->check_addr_ack)
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

      gd32_i2c_sendstop(priv);

      /* Trace event */

      gd32_i2c_traceevent(priv, I2CEVENT_ADDRESS_NACKED, priv->msgv->addr);
    }
#endif

  /* ACK in read mode, ACK in write mode is handled separately */

  else if ((priv->flags & I2C_M_READ) != 0 && \
           (status & I2C_STAT0_ADDSEND) != 0 && \
           priv->check_addr_ack)
    {
      /* Reset check addr flag as we are handling this event */

      priv->check_addr_ack = false;

      /* Note:
       *
       * When reading a single byte the stop condition has  to be set
       * immediately after clearing the state flags, which happens
       * when reading STAT1(as STAT0 has already been read).
       *
       * Similarly when reading 2 bytes the NACK bit has to be set as just
       * after the clearing of the address.
       */

      if (priv->dcnt == 1)
        {
          /* this should only happen when receiving a message of length 1 */

          i2cinfo("short read N=1: setting NACK\n");

          /* Set POS bit to zero (can be up from a previous 2 byte receive) */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_POAP, 0);

          /* Immediately set NACK */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_ACKEN, 0);

#ifndef CONFIG_I2C_POLLED
          /* Enable RxNE and TxE buffers in order to receive one or multiple
           * bytes
           */

          gd32_i2c_modifyreg(priv,
                              GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_BUFIE);
#endif

          /* Clear ADDR flag by reading CTL1 and adding it to status */

          status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

          /* Send Stop/Restart */

          if (priv->msgc > 0)
            {
              gd32_i2c_sendstart(priv);
            }
          else
            {
              gd32_i2c_sendstop(priv);
            }

          i2cinfo("Address ACKed beginning data reception\n");
          i2cinfo("short read N=1: programming stop bit\n");

          /* Trace */

          gd32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED_READ_1, 0);
        }
      else if (priv->dcnt == 2)
        {
          /* This should only happen when receiving a message of length 2 */

          /* Set POS bit to zero (can be up from a previous 2 byte receive) */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_POAP);

          /* Immediately set NACK */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_ACKEN, 0);

          /* Clear ADDR flag by reading STAT1 and adding it to status */

          status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

          i2cinfo("Address ACKed beginning data reception\n");
          i2cinfo("short read N=2: programming NACK\n");

          /* Trace */

          gd32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED_READ_2, 0);
        }
      else
        {
          i2cinfo("Address ACKed beginning data reception\n");

          /* Clear ADDR flag by reading STAT1 and adding it to status */

          status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

          /* Trace */

          gd32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED, 0);

#ifdef CONFIG_GD32F4_I2C_DMA
          /* DMA only when not doing a short read */

          dma_single_data_parameter_struct dma_init_struct;

          i2cinfo("Starting dma transfer and disabling interrupts\n");

          /* The DMA must be initialized and enabled before the I2C data
           * transfer.
           * The DMAEN bit must be set in the I2C_CTL1 register before the
           * ADDR event.
           */

          dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
          dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
          dma_init_struct.memory0_addr = (uint32_t)priv->ptr;
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
          dma_init_struct.number = priv->dcnt;
          dma_init_struct.periph_addr = \
                  GD32_I2C_DATA(priv->config->i2c_base);
          dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
          dma_init_struct.priority = I2C_DMA_PRIO;

          /* Configure the RX DMA */

          gd32_dma_setup(priv->rxdma, &dma_init_struct, 1);

          /* Do not enable the ITBUFEN bit in the I2C_CTL1 register if DMA is
           * used.
           */

          gd32_i2c_modifyreg(priv,
                              GD32_I2C_CTL1_OFFSET, I2C_CTL1_BUFIE, 0);

#ifndef CONFIG_I2C_POLLED
          /* Now let DMA do all the work, disable i2c interrupts */

          regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
          regval &= ~I2C_CTL1_INTS_MASK;
          gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);
#endif

          /* The user can generate a Stop condition in the DMA Transfer
           * Complete interrupt routine if enabled. This will be done in
           * the dma rx callback Start DMA.
           */

          gd32_dma_start(priv->rxdma, gd32_i2c_dmarxcallback, priv, \
                         I2C_RX_DMA_INTEN);

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_DMAON);
#else
#ifndef CONFIG_I2C_POLLED
          if (priv->dcnt > 3)
            {
              /* Don't enable I2C_CTL1_BUFIE for messages longer than 3
               * bytes
               */

              gd32_i2c_modifyreg(priv,
                                  GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_BUFIE);
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
           (status & I2C_STAT0_BTC) != 0 &&
            priv->dcnt == 0)
    {
      /* After last byte, check what to do based on next message flags */

      if (priv->msgc == 0)
        {
          /* If last message send stop bit */

          gd32_i2c_sendstop(priv);
          i2cinfo("Stop sent dcnt = %i msgc = %i\n", priv->dcnt, priv->msgc);

          /* Decrease counter to get to next message */

          priv->dcnt--;
          i2cinfo("dcnt %i\n", priv->dcnt);
          gd32_i2c_traceevent(priv, I2CEVENT_WRITE_STOP, priv->dcnt);
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

          gd32_i2c_sendstart(priv);

          gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);

          i2cinfo("Restart detected!\n");
          i2cinfo("Nextflag %i\n", priv->msgv[0].flags);

          /* Decrease counter to get to next message */

          priv->dcnt--;
          i2cinfo("dcnt %i\n", priv->dcnt);
          gd32_i2c_traceevent(priv, I2CEVENT_WRITE_RESTART, priv->dcnt);
        }
      else
        {
          i2cinfo("Write mode: next message has an unrecognized flag.\n");
          gd32_i2c_traceevent(priv,
                               I2CEVENT_WRITE_FLAG_ERROR, priv->msgv->flags);
        }

      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);
    }
  else if ((priv->flags & I2C_M_READ) == 0 &&
           (status & (I2C_STAT0_ADDSEND | I2C_STAT0_TBE)) != 0 &&
            priv->dcnt != 0)
    {
      /* The has cleared(ADDR is set, ACK was received after the address)
       * or the transmit buffer is empty flag has been set(TxE) then we can
       * transmit the next byte.
       */

      i2cinfo("Entering write mode dcnt = %i msgc = %i\n",
              priv->dcnt, priv->msgc);

      /* Clear ADDR flag by reading STAT1 and adding it to status */

      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

      /* Address has cleared so don't check on next call */

      priv->check_addr_ack = false;

      /* Check if we have transmitted the whole message or we are after
       * the last byte where the stop condition or else(according to the
       * msg flags) has to be set.
       */

#ifdef CONFIG_GD32F4_I2C_DMA
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

          dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
          dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
          dma_init_struct.memory0_addr = (uint32_t)priv->ptr;
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
          dma_init_struct.number = priv->dcnt;
          dma_init_struct.periph_addr = \
                            GD32_I2C_DATA(priv->config->i2c_base);
          dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
          dma_init_struct.priority = I2C_DMA_PRIO;

          /* Configure the RX DMA */

          gd32_dma_setup(priv->rxdma, &dma_init_struct, 1);

          /* Do not enable the ITBUFEN bit in the I2C_CR2 register if DMA is
           * used.
           */

          gd32_i2c_modifyreg(priv,
                              GD32_I2C_CTL1_OFFSET, I2C_CTL1_BUFIE, 0);

#ifndef CONFIG_I2C_POLLED
          /* Now let DMA do all the work, disable i2c interrupts */

          regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
          regval &= ~I2C_CTL1_INTS_MASK;
          gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);
#endif

          /* In the interrupt routine after the EOT interrupt, disable DMA
           * requests then wait for a BTF event before programming the Stop
           * condition. To do this, we'll just call the ISR again in
           * DMA tx callback, in which point we fall into the msgc==0 case
           * which ultimately sends the stop..TODO: but we don't explicitly
           * wait for BTF bit being set...
           * Start DMA.
           */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_DMAON);
          gd32_dma_start(priv->rxdma, gd32_i2c_dmarxcallback, priv, \
                         I2C_TX_DMA_INTEN);
        }
      else
#endif /* CONFIG_GD32F4_I2C_DMA */
        {
#ifndef CONFIG_I2C_POLLED
          if (priv->dcnt == 1 &&
              (priv->msgc == 0 || (priv->msgv->flags & I2C_M_NOSTART) == 0))
            {
              gd32_i2c_modifyreg(priv,
                                 GD32_I2C_CTL1_OFFSET, I2C_CTL1_BUFIE, 0);
            }
#endif

          /* Transmitting message.
           * Send byte == write data into write register
           */

          gd32_i2c_putreg(priv, GD32_I2C_DATA_OFFSET, *priv->ptr++);

          /* Decrease current message length */

          gd32_i2c_traceevent(priv, I2CEVENT_WRITE_TO_DR, priv->dcnt);
          priv->dcnt--;

          if ((status & I2C_STAT0_ADDSEND) != 0 && priv->dcnt > 0)
            {
              /* Transmitting message.
               * ADDR -> BTC & TBE - Send one more byte
               */

              gd32_i2c_putreg(priv, GD32_I2C_DATA_OFFSET, *priv->ptr++);

              /* Decrease current message length */

              gd32_i2c_traceevent(priv, I2CEVENT_WRITE_TO_DR, priv->dcnt);
              priv->dcnt--;
            }

#ifndef CONFIG_I2C_POLLED
          if (((status & I2C_STAT0_ADDSEND) != 0 && priv->dcnt > 0) ||
              (priv->msgc > 0 && (priv->msgv->flags & I2C_M_NOSTART) != 0))
            {
              gd32_i2c_modifyreg(priv,
                                 GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_BUFIE);
            }
#endif

          if (priv->dcnt == 0 &&
              priv->msgc > 0 && (priv->msgv->flags & I2C_M_NOSTART) != 0)
            {
              /* Set condition to get to next message */

              priv->dcnt = -1;
              gd32_i2c_traceevent(priv,
                                  I2CEVENT_WRITE_NO_RESTART, priv->dcnt);
            }
        }
    }

  else if ((priv->flags & (I2C_M_READ)) != 0 &&
           (status & (I2C_STAT0_RBNE | I2C_STAT0_BTC)) != 0)
    {
      /* When read flag is set and the receive buffer is not empty
       * (RBNE is set) then the driver can read from the data register.
       */

      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

      i2cinfo("Entering read mode dcnt = %i msgc = %i, "
              "status 0x%04" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);

      /* Byte #N-3W, we don't want to manage RBNE interrupt anymore, bytes
       * N, N-1, N-2 will be read with BTC:
       */

#ifndef CONFIG_I2C_POLLED
      if (priv->dcnt < 5)
        {
          gd32_i2c_modifyreg(priv,
                            GD32_I2C_CTL1_OFFSET, I2C_CTL1_BUFIE, 0);
        }
#else
      if (priv->dcnt == 1 ||
          priv->dcnt > 3 || (status & I2C_STAT0_BTC) != 0)
#endif
        {
          /*  BTF: N-2/N-1, set NACK, read N-2 */

          if (priv->dcnt == 3)
            {
              gd32_i2c_modifyreg(priv,
                                GD32_I2C_CTL0_OFFSET, I2C_CTL0_ACKEN, 0);
            }

          /*  BTF: N-1/N, STOP/START, read N-1, N */

          else if (priv->dcnt == 2)
            {
              if (priv->msgc > 0)
                {
                  gd32_i2c_sendstart(priv);
                }
              else
                {
                  gd32_i2c_sendstop(priv);
                }

              /* Read byte #N-1 */

              *priv->ptr++ = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
              priv->dcnt--;
            }

          /* Read last or current byte */

          *priv->ptr++ = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
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

      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);
      i2cinfo("Empty call to ISR: Stopping ISR\n");
      gd32_i2c_traceevent(priv, I2CEVENT_ISR_EMPTY_CALL, 0);
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
      gd32_i2c_traceevent(priv, I2CEVENT_POLL_DEV_NOT_RDY, 0);
#else
      /* Read rest of the state */

      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

      /* No any error bit is set, but driver is in incorrect state, signal
       * it with "Bus error" bit.
       */

      if ((status & I2C_STAT0_ERROR_MASK) != 0)
        {
          priv->status |= I2C_STAT0_BERR;
        }

      i2cinfo(" No correct state detected(start bit, read or write)\n");
      i2cinfo(" state %" PRIi32 "\n", status);

      /* Set condition to terminate ISR and wake waiting thread */

      priv->dcnt = -1;
      priv->msgc = 0;
      gd32_i2c_traceevent(priv, I2CEVENT_STATE_ERROR, 0);
#endif
    }

  /* Messages handling(2/2)
   *
   * Transmission of the whole message chain has been completed. We have to
   * terminate the ISR and wake up gd32_i2c_process() that is waiting for
   * the ISR cycle to handle the sending/receiving of the messages.
   */

  /* First check for errors */

  if ((status & I2C_STAT0_ERROR_MASK) != 0)
    {
      gd32_i2c_traceevent(priv, I2CEVENT_ERROR, \
                          status & I2C_STAT0_ERROR_MASK);

      /* Clear interrupt flags */

#if !defined(CONFIG_GD32F4_I2C_DMA) && !defined(CONFIG_I2C_POLLED)
state_error:
#endif
      gd32_i2c_putreg(priv, GD32_I2C_STAT0_OFFSET, 0);

      priv->dcnt = -1;
      priv->msgc = 0;
    }

  if (priv->dcnt == -1 && priv->msgc == 0)
    {
      i2cinfo("Shutting down I2C ISR\n");

      gd32_i2c_traceevent(priv, I2CEVENT_ISR_SHUTDOWN, 0);

      /* Clear internal pointer to the message content.
       * Good practice + done by last implementation when messages are
       * finished (compatibility concerns)
       */

      priv->msgv = NULL;

#ifdef CONFIG_I2C_POLLED
      priv->intstate = INTSTATE_DONE;
#else
      /* Clear all interrupts */

      regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
      regval &= ~I2C_CTL1_INTS_MASK;
      gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);

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
 * Name: gd32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int gd32_i2c_isr(int irq, void *context, void *arg)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return gd32_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: gd32_i2c_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_I2C_DMA
static void gd32_i2c_dmarxcallback(DMA_HANDLE handle,
                                    uint8_t status, void *arg)
{
#ifndef CONFIG_I2C_POLLED
  uint32_t regval;
#endif

  i2cinfo("DMA rx callback, status = %d\n", status);

  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)arg;

  priv->dcnt = -1;

  /* The user can generate a Stop condition in the DMA Transfer Complete
   * interrupt routine if enabled.
   */

  if (priv->msgc > 0)
    {
      gd32_i2c_sendstart(priv);
    }
  else
    {
      gd32_i2c_sendstop(priv);
    }

  /* Let the I2C periph know to stop DMA transfers, also is used by ISR
   * to check if DMA is done.
   */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, I2C_CTL1_DMAON, 0);

#ifndef CONFIG_I2C_POLLED
  /* Re-enable interrupts */

  regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
  regval |= (I2C_CTL1_ERRIE | I2C_CTL1_EVIE);
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);
#endif

  /* let the ISR routine take care of shutting down or switching to
   * next msg
   */

  gd32_i2c_isr_process(priv);
}
#endif /* ifdef CONFIG_GD32F4_I2C_DMA */

/****************************************************************************
 * Name: gd32_i2c_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_I2C_DMA
static void gd32_i2c_dmatxcallback(DMA_HANDLE handle,
                                    uint8_t status, void *arg)
{
#ifndef CONFIG_I2C_POLLED
  uint32_t regval;
#endif

  i2cinfo("DMA tx callback, status = %d\n", status);

  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)arg;

  priv->dcnt = 0;

  /* In the interrupt routine after the EOT interrupt,
   * disable DMA requests
   */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, I2C_CTL1_DMAON, 0);

#ifndef CONFIG_I2C_POLLED
  /* re-enable interrupts */

  regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
  regval |= (I2C_CTL1_ERRIE | I2C_CTL1_EVIE);
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);
#endif
}
#endif /* ifdef CONFIG_GD32F4_I2C_DMA */

/****************************************************************************
 * Name: gd32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int gd32_i2c_init(struct gd32_i2c_priv_s *priv)
{
  /* Power-up and configure GPIOs */

  /* Enable power and reset the peripheral */

  gd32_i2c_clock_enable(priv->config->i2c_base);
  gd32_i2c_periph_reset(priv->config->i2c_base);

  /* Configure pins */

  if (gd32_gpio_config(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (gd32_gpio_config(priv->config->sda_pin) < 0)
    {
      gd32_gpio_unconfig(priv->config->scl_pin);
      return ERROR;
    }

  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->event_irq, gd32_i2c_isr, priv);
  irq_attach(priv->config->error_irq, gd32_i2c_isr, priv);
  up_enable_irq(priv->config->event_irq);
  up_enable_irq(priv->config->error_irq);
#endif

  /* Set peripheral frequency, where it must be at least 2 MHz  for 100 kHz
   * or 4 MHz for 400 kHz.  This also disables all I2C interrupts.
   */

  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET,
                  (GD32_PCLK1_FREQUENCY / 1000000));

  /* Force a frequency update */

  priv->frequency = 0;

  gd32_i2c_setclock(priv, 100000);

#ifdef CONFIG_GD32F4_I2C_DMA
  /* If, in the I2C_CR2 register, the LAST bit is set, I2C automatically
   * sends a NACK after the next byte following EOT_1.
   * Clear DMA en just in case.
   */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET,
                     I2C_CTL1_DMAON, I2C_CTL1_DMALST);
#endif

  /* Enable I2C */

  gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_I2CEN);
  return OK;
}

/****************************************************************************
 * Name: gd32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int gd32_i2c_deinit(struct gd32_i2c_priv_s *priv)
{
  /* Disable I2C */

  gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, 0);
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, 0);

  /* Unconfigure GPIO pins */

  gd32_gpio_unconfig(priv->config->scl_pin);
  gd32_gpio_unconfig(priv->config->sda_pin);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->event_irq);
  up_disable_irq(priv->config->error_irq);
  irq_detach(priv->config->event_irq);
  irq_detach(priv->config->error_irq);
#endif

#ifdef CONFIG_GD32F4_I2C_DMA
  /* Disable DMA */

  gd32_dma_stop(priv->txdma);
  gd32_dma_stop(priv->rxdma);
#endif

  /* Disable clocking */

  gd32_i2c_clock_disable(priv->config->i2c_base);
  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int gd32_i2c_transfer(struct i2c_master_s *dev,
                            struct i2c_msg_s *msgs, int count)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  uint32_t status = 0;
  int ret;

  DEBUGASSERT(count);

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_GD32F4_I2C_DMA
  /* Stop DMA just in case */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, I2C_CTL1_DMAON, 0);
  gd32_dma_stop(priv->rxdma);
  gd32_dma_stop(priv->txdma);
#endif

  /* Wait for any STOP in progress. */

  gd32_i2c_sem_waitstop(priv);

  /* Clear any pending error interrupts */

  gd32_i2c_putreg(priv, GD32_I2C_STAT0_OFFSET, 0);

  /* "Note: When the STOP, START or PECTRANS bit is set, the software must
   *  not perform any write access to I2C_CTL0 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PECTRANS request."  However, if the bits are
   *  not cleared by hardware, then we will have to do that from hardware.
   */

  gd32_i2c_clrstart(priv);

  /* Old transfers are done */

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  gd32_i2c_tracereset(priv);

  /* Set I2C clock frequency (on change it toggles I2C_CTL0_I2CEN !)
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  gd32_i2c_setclock(priv, msgs->frequency);

  /* Enable acknowledge */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_ACKEN);

  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within gd32_i2c_waitdone().
   */

  priv->dcnt   = -1;
  priv->status = 0;
  gd32_i2c_sendstart(priv);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (gd32_i2c_sem_waitdone(priv) < 0)
    {
      status = gd32_i2c_getstatus(priv);
      ret = -ETIMEDOUT;

      i2cerr("ERROR: Timed out: CTL): 0x%04x status: 0x%08" PRIx32 "\n",
             gd32_i2c_getreg(priv, GD32_I2C_CTL0_OFFSET), status);

      /* "Note: When the STOP, START or PECTRANS bit is set, the software
       *  must not perform any write access to I2C_CTL0 before this bit is
       *  cleared by hardware. Otherwise there is a risk of setting a
       *  second STOP, START or PECTRANS request."
       */

      gd32_i2c_clrstart(priv);

      /* Clear busy flag in case of timeout */

      status = priv->status & 0xffff;
    }
  else
    {
      /* clear STAT1 (BUSY flag) as we've done successfully */

      status = priv->status & 0xffff;
    }

  /* Check for error status conditions */

  if ((status & I2C_STAT0_ERROR_MASK) != 0)
    {
      /* I2C_STAT0_ERROR_MASK is the 'OR' of the following individual bits: */

      if (status & I2C_STAT0_BERR)
        {
          /* Bus Error */

          ret = -EIO;
        }
      else if (status & I2C_STAT0_LOSTARB)
        {
          /* Arbitration Lost (master mode) */

          ret = -EAGAIN;
        }
      else if (status & I2C_STAT0_AERR)
        {
          /* Acknowledge Failure */

          ret = -ENXIO;
        }
      else if (status & I2C_STAT0_OUERR)
        {
          /* Overrun/Underrun */

          ret = -EIO;
        }
      else if (status & I2C_STAT0_PECERR)
        {
          /* PEC Error in reception */

          ret = -EPROTO;
        }
      else if (status & I2C_STAT0_SMBTO)
        {
          /* Timeout or Tlow Error */

          ret = -ETIME;
        }

      /* This is not an error and should never happen since SMBus is not
       * enabled
       */

      else /* if (status & I2C_STAT0_SMBALT) */
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
   * NOTE:  We will only see this busy indication if gd32_i2c_sem_waitdone()
   * fails above;  Otherwise it is cleared.
   */

  else if ((status & (I2C_STAT1_I2CBSY << 16)) != 0)
    {
      /* I2C Bus is for some reason busy */

      ret = -EBUSY;
    }

  /* Dump the trace result */

  gd32_i2c_tracedump(priv);

  /* Ensure that any ISR happening after we finish can't overwrite any user
   * data
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_i2c_reset
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
static int gd32_i2c_reset(struct i2c_master_s *dev)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
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

  gd32_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  gd32_gpio_config(scl_gpio);
  gd32_gpio_config(sda_gpio);

  /* Let SDA go high */

  gd32_gpio_write(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!gd32_gpio_read(sda_gpio))
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
      while (!gd32_gpio_read(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      gd32_gpio_write(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      gd32_gpio_write(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  gd32_gpio_write(sda_gpio, 0);
  up_udelay(10);
  gd32_gpio_write(scl_gpio, 0);
  up_udelay(10);
  gd32_gpio_write(scl_gpio, 1);
  up_udelay(10);
  gd32_gpio_write(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  gd32_gpio_unconfig(sda_gpio);
  gd32_gpio_unconfig(scl_gpio);

  /* Re-init the port */

  gd32_i2c_init(priv);

  /* Restore the frequency */

  gd32_i2c_setclock(priv, frequency);
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
 * Name: gd32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *gd32_i2cbus_initialize(int port)
{
  struct gd32_i2c_priv_s * priv = NULL;

#if GD32_PCLK1_FREQUENCY < 4000000
#   warning GD32_I2C_INIT: Peripheral clock must be at least 4 MHz to support 400 kHz operation.
#endif

#if GD32_PCLK1_FREQUENCY < 2000000
#   warning GD32_I2C_INIT: Peripheral clock must be at least 2 MHz to support 100 kHz operation.
  return NULL;
#endif

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_GD32F4_I2C0
    case 0:
      priv = (struct gd32_i2c_priv_s *)&gd32_i2c0_priv;
      break;
#endif
#ifdef CONFIG_GD32F4_I2C1
    case 1:
      priv = (struct gd32_i2c_priv_s *)&gd32_i2c1_priv;
      break;
#endif
#ifdef CONFIG_GD32F4_I2C2
    case 2:
      priv = (struct gd32_i2c_priv_s *)&gd32_i2c2_priv;
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
      gd32_i2c_init(priv);

#ifdef CONFIG_GD32F4_I2C_DMA
      /* Get DMA channels.  NOTE: gd32_dma_channel_alloc() will always
       * assign the DMA channel.  If the channel is not available, then
       * gd32_dma_channel_alloc() will block and wait until the channel
       * becomes available.
       * WARNING: If you have another device sharing a DMA channel with I2C
       * and the code never releases that channel, then the call to
       * gd32_dma_channel_alloc()  will hang forever in this function!
       *  Don't let your design do that!
       */

      priv->txdma = gd32_dma_channel_alloc(priv->txch);
      DEBUGASSERT(priv->rxdma && priv->txdma);
#endif /* CONFIG_GD32F4_I2C_DMA */
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: gd32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int gd32_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;

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

  gd32_i2c_deinit(priv);

#ifdef CONFIG_GD32F4_I2C_DMA
  gd32_dma_channel_free(priv->rxdma);
  gd32_dma_channel_free(priv->txdma);
#endif

  nxmutex_unlock(&priv->lock);
  return OK;
}

#endif /* CONFIG_GD32F4_I2C0 || CONFIG_GD32F4_I2C1 || CONFIG_GD32F4_I2C2 */
