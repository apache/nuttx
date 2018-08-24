/************************************************************************************
 * arch/arm/src/stm32l4/stm32f0_i2c.c
 * STM32L4 I2C driver - based on STM32F3 I2C Hardware Layer - Device Driver
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *   Copyright (C) 2011-2013, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: John Wharington
 *   Author: Sebastien Lorquet
 *   Author: dev@ziggurat29.com
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
 ************************************************************************************/

/* Supports:
 *  - Master operation, 100 kHz (standard) and 400 kHz (full speed)
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained by
 *      the i2c_init(); it extends the Device structure from the nuttx/i2c/i2c.h;
 *      Instance points to OPS, to common I2C Hardware private data and contains
 *      its own private data, as frequency, address, mode of operation (in the future)
 *  - Private: Private data of an I2C Hardware
 *
 * TODO
 *  - Check for all possible deadlocks (as BUSY='1' I2C needs to be reset in HW using
 *     the I2C_CR1_SWRST)
 *  - SMBus support (hardware layer timings are already supported) and add SMBA gpio
 *    pin
 *  - Slave support with multiple addresses (on multiple instances):
 *      - 2 x 7-bit address or
 *      - 1 x 10 bit adresses + 1 x 7 bit address (?)
 *      - plus the broadcast address (general call)
 *  - Multi-master support
 *  - DMA (to get rid of too many CPU wake-ups and interventions)
 *  - Be ready for IPMI
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

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
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "stm32f0_gpio.h"
#include "stm32f0_rcc.h"
#include "stm32f0_i2c.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_STM32F0_I2C1) || defined(CONFIG_STM32F0_I2C2) || defined(CONFIG_STM32F0_I2C3)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.  Instead,
 * CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_STM32F0_I2CTIMEOSEC) && !defined(CONFIG_STM32F0_I2CTIMEOMS)
#  define CONFIG_STM32F0_I2CTIMEOSEC 0
#  define CONFIG_STM32F0_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_STM32F0_I2CTIMEOSEC)
#  define CONFIG_STM32F0_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_STM32F0_I2CTIMEOMS)
#  define CONFIG_STM32F0_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_STM32F0_I2CTIMEOTICKS
#  define CONFIG_STM32F0_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_STM32F0_I2CTIMEOSEC) + MSEC2TICK(CONFIG_STM32F0_I2CTIMEOMS))
#endif

#ifndef CONFIG_STM32F0_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_STM32F0_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_STM32F0_I2CTIMEOTICKS)
#endif

#define I2C_OUTPUT \
  (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_OPENDRAIN | GPIO_SPEED_50MHz)
#define MKI2C_OUTPUT(p) \
  (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

#define I2C_CR1_TXRX \
  (I2C_CR1_RXIE | I2C_CR1_TXIE)
#define I2C_CR1_ALLINTS \
  (I2C_CR1_TXRX | I2C_CR1_TCIE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE)

#define STATUS_NACK(status)    (status & I2C_INT_NACK)
#define STATUS_ADDR(status)    (status & I2C_INT_ADDR)
#define STATUS_ADDR_TX(status) (status & (I2C_INT_ADDR | I2C_ISR_TXIS))
#define STATUS_ADD10(status)   (0)
#define STATUS_RXNE(status)    (status & I2C_ISR_RXNE)
#define STATUS_TC(status)      (status & I2C_ISR_TC)
#define STATUS_BUSY(status)    (status & I2C_ISR_BUSY)

/* Debug ****************************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard, low-level
 * debug interface syslog() but does not require that any other debug
 * is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define stm32f0_i2c_tracereset(p)
#  define stm32f0_i2c_tracenew(p,s)
#  define stm32f0_i2c_traceevent(p,e,a)
#  define stm32f0_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/
/* Interrupt state */

enum stm32f0_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum stm32f0_trace_e
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

struct stm32f0_trace_s
{
  uint32_t status;               /* I2C 32-bit SR2|SR1 status */
  uint32_t count;                /* Interrupt count when status change */
  enum stm32f0_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;                 /* Parameter associated with the event */
  clock_t time;                  /* First of event or first status */
};

/* I2C Device hardware configuration */

struct stm32f0_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t reset_bit;         /* Reset bit */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  uint32_t irq;               /* IRQ */
#endif
};

/* I2C Device Private Data */

struct stm32f0_i2c_priv_s
{
  const struct i2c_ops_s *ops; /* Standard I2C operations */
  const struct stm32f0_i2c_config_s *config; /* Port configuration */
  int refs;                    /* Reference count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum stm32f0_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  uint32_t frequency;          /* Current I2C frequency */
  int dcnt;                    /* Current message length */
  uint16_t flags;              /* Current message flags */
  bool astart;                 /* START sent */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct stm32f0_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static inline uint32_t stm32f0_i2c_getreg32(FAR struct stm32f0_i2c_priv_s *priv,
                                            uint8_t offset);
static inline void stm32f0_i2c_putreg32(FAR struct stm32f0_i2c_priv_s *priv,
                                        uint8_t offset, uint32_t value);
static inline void stm32f0_i2c_modifyreg32(FAR struct stm32f0_i2c_priv_s *priv,
                                         uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits);
static inline void stm32f0_i2c_sem_wait(FAR struct stm32f0_i2c_priv_s *priv);
#ifdef CONFIG_STM32F0_I2C_DYNTIMEO
static useconds_t stm32f0_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs);
#endif /* CONFIG_STM32F0_I2C_DYNTIMEO */
static inline int  stm32f0_i2c_sem_waitdone(FAR struct stm32f0_i2c_priv_s *priv);
static inline void stm32f0_i2c_sem_waitstop(FAR struct stm32f0_i2c_priv_s *priv);
static inline void stm32f0_i2c_sem_post(FAR struct stm32f0_i2c_priv_s *priv);
static inline void stm32f0_i2c_sem_init(FAR struct stm32f0_i2c_priv_s *priv);
static inline void stm32f0_i2c_sem_destroy(FAR struct stm32f0_i2c_priv_s *priv);
#ifdef CONFIG_I2C_TRACE
static void stm32f0_i2c_tracereset(FAR struct stm32f0_i2c_priv_s *priv);
static void stm32f0_i2c_tracenew(FAR struct stm32f0_i2c_priv_s *priv,
                                 uint32_t status);
static void stm32f0_i2c_traceevent(FAR struct stm32f0_i2c_priv_s *priv,
                                   enum stm32f0_trace_e event, uint32_t parm);
static void stm32f0_i2c_tracedump(FAR struct stm32f0_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */
static void stm32f0_i2c_setclock(FAR struct stm32f0_i2c_priv_s *priv,
                                 uint32_t frequency);
static inline void stm32f0_i2c_sendstart(FAR struct stm32f0_i2c_priv_s *priv);
static inline void stm32f0_i2c_clrstart(FAR struct stm32f0_i2c_priv_s *priv);
static inline void stm32f0_i2c_sendstop(FAR struct stm32f0_i2c_priv_s *priv);
static inline uint32_t stm32f0_i2c_getstatus(FAR struct stm32f0_i2c_priv_s *priv);
static int stm32f0_i2c_isr_process(struct stm32f0_i2c_priv_s * priv);
#ifndef CONFIG_I2C_POLLED
static int stm32f0_i2c_isr(int irq, void *context, FAR void *arg);
#endif
static int stm32f0_i2c_init(FAR struct stm32f0_i2c_priv_s *priv);
static int stm32f0_i2c_deinit(FAR struct stm32f0_i2c_priv_s *priv);
static int stm32f0_i2c_transfer(FAR struct i2c_master_s *dev,
                                FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int stm32f0_i2c_reset(FAR struct i2c_master_s *dev);
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Device Structures, Instantiation */

static const struct i2c_ops_s stm32f0_i2c_ops =
{
 .transfer = stm32f0_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = stm32f0_i2c_reset
#endif
};

#ifdef CONFIG_STM32F0_I2C1
static const struct stm32f0_i2c_config_s stm32f0_i2c1_config =
{
  .base       = STM32F0_I2C1_BASE,
  .clk_bit    = RCC_APB1ENR_I2C1EN,
  .reset_bit  = RCC_APB1RSTR_I2C1RST,
  .scl_pin    = GPIO_I2C1_SCL,
  .sda_pin    = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .irq        = STM32F0_IRQ_I2C1
#endif
};

static struct stm32f0_i2c_priv_s stm32f0_i2c1_priv =
{
  .ops        = &stm32f0_i2c_ops,
  .config     = &stm32f0_i2c1_config,
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

#ifdef CONFIG_STM32F0_I2C2
static const struct stm32f0_i2c_config_s stm32f0_i2c2_config =
{
  .base       = STM32F0_I2C2_BASE,
  .clk_bit    = RCC_APB1ENR1_I2C2EN,
  .reset_bit  = RCC_APB1RSTR1_I2C2RST,
  .scl_pin    = GPIO_I2C2_SCL,
  .sda_pin    = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_POLLED
  .irq        = STM32F0_IRQ_I2C2
#endif
};

static struct stm32f0_i2c_priv_s stm32f0_i2c2_priv =
{
  .ops        = &stm32f0_i2c_ops,
  .config     = &stm32f0_i2c2_config,
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

#ifdef CONFIG_STM32F0_I2C3
static const struct stm32f0_i2c_config_s stm32f0_i2c3_config =
{
  .base       = STM32F0_I2C3_BASE,
  .clk_bit    = RCC_APB1ENR1_I2C3EN,
  .reset_bit  = RCC_APB1RSTR1_I2C3RST,
  .scl_pin    = GPIO_I2C3_SCL,
  .sda_pin    = GPIO_I2C3_SDA,
#ifndef CONFIG_I2C_POLLED
  .irq        = STM32F0_IRQ_I2C3
#endif
};

static struct stm32f0_i2c_priv_s stm32f0_i2c3_priv =
{
  .ops        = &stm32f0_i2c_ops,
  .config     = &stm32f0_i2c3_config,
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

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32f0_i2c_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ************************************************************************************/

static inline uint32_t stm32f0_i2c_getreg32(FAR struct stm32f0_i2c_priv_s *priv,
                                            uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32f0_i2c_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ************************************************************************************/

static inline void stm32f0_i2c_putreg32(FAR struct stm32f0_i2c_priv_s *priv,
                                        uint8_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32f0_i2c_modifyreg32
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ************************************************************************************/

static inline void stm32f0_i2c_modifyreg32(FAR struct stm32f0_i2c_priv_s *priv,
                                         uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/************************************************************************************
 * Name: stm32f0_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ************************************************************************************/

static inline void stm32f0_i2c_sem_wait(FAR struct stm32f0_i2c_priv_s *priv)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&priv->sem_excl);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/************************************************************************************
 * Name: stm32f0_i2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be processed.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32F0_I2C_DYNTIMEO
static useconds_t stm32f0_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs)
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

  return (useconds_t)(CONFIG_STM32F0_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/************************************************************************************
 * Name: stm32f0_i2c_enableinterrupts
 *
 * Description:
 *   Enable I2C interrupts
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline void stm32f0_i2c_enableinterrupts(struct stm32f0_i2c_priv_s *priv)
{
  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR1_OFFSET, 0, I2C_CR1_TXRX);
}
#endif

/************************************************************************************
 * Name: stm32f0_i2c_disableinterrupts
 *
 * Description:
 *   Enable I2C interrupts
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline void stm32f0_i2c_disableinterrupts(struct stm32f0_i2c_priv_s *priv)
{
  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR1_OFFSET, I2C_CR1_TXRX, 0);
}
#endif

/************************************************************************************
 * Name: stm32f0_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int stm32f0_i2c_sem_waitdone(FAR struct stm32f0_i2c_priv_s *priv)
{
  struct timespec abstime;
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  /* Enable I2C interrupts */

  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR1_OFFSET, 0,
                        (I2C_CR1_ALLINTS & ~I2C_CR1_TXRX));

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
   */

   priv->intstate = INTSTATE_WAITING;
   do
    {
      /* Get the current time */

      (void)clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_STM32F0_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_STM32F0_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_STM32F0_I2C_DYNTIMEO
      abstime.tv_nsec += 1000 * stm32f0_i2c_tousecs(priv->msgc, priv->msgv);
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_STM32F0_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_STM32F0_I2CTIMEOMS * 1000 * 1000;
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }
#endif
      /* Wait until either the transfer is complete or the timeout expires */

      ret = nxsem_timedwait(&priv->sem_isr, &abstime);
      if (ret < 0 && ret != -EINTR)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by nxsem_timedwait.
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

  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR1_OFFSET, I2C_CR1_ALLINTS, 0);

  leave_critical_section(flags);
  return ret;
}
#else
static inline int stm32f0_i2c_sem_waitdone(FAR struct stm32f0_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_STM32F0_I2C_DYNTIMEO
  timeout = USEC2TICK(stm32f0_i2c_tousecs(priv->msgc, priv->msgv));
#else
  timeout = CONFIG_STM32F0_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  start = clock_systimer();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systimer() - start;

      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      stm32f0_i2c_isr_process(priv);
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cinfo("intstate: %d elapsed: %ld threshold: %ld status: %08x\n",
          priv->intstate, (long)elapsed, (long)timeout, priv->status);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/************************************************************************************
 * Name: stm32f0_i2c_set_7bit_address
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32f0_i2c_set_7bit_address(FAR struct stm32f0_i2c_priv_s *priv)
{
  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET, I2C_CR2_SADD7_MASK,
                        ((priv->msgv->addr & 0x7F) << I2C_CR2_SADD7_SHIFT));
}

/************************************************************************************
 * Name: stm32f0_i2c_set_bytes_to_transfer
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32f0_i2c_set_bytes_to_transfer(FAR struct stm32f0_i2c_priv_s *priv,
                                  uint8_t n_bytes)
{
  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET, I2C_CR2_NBYTES_MASK,
                        (n_bytes << I2C_CR2_NBYTES_SHIFT));
}

/************************************************************************************
 * Name: stm32f0_i2c_set_write_transfer_dir
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32f0_i2c_set_write_transfer_dir(FAR struct stm32f0_i2c_priv_s *priv)
{
  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET, I2C_CR2_RD_WRN, 0);
}

/************************************************************************************
 * Name: stm32f0_i2c_set_read_transfer_dir
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32f0_i2c_set_read_transfer_dir(FAR struct stm32f0_i2c_priv_s *priv)
{
  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET, 0, I2C_CR2_RD_WRN);
}

/************************************************************************************
 * Name: stm32f0_i2c_enable_autoend
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32f0_i2c_enable_autoend(FAR struct stm32f0_i2c_priv_s *priv)
{
  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET, 0, I2C_CR2_AUTOEND);
}

/************************************************************************************
 * Name: stm32f0_i2c_disable_autoend
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32f0_i2c_disable_autoend(FAR struct stm32f0_i2c_priv_s *priv)
{
  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET, I2C_CR2_AUTOEND, 0);
}

/************************************************************************************
 * Name: stm32f0_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ************************************************************************************/

static inline void stm32f0_i2c_sem_waitstop(FAR struct stm32f0_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t cr;
  uint32_t sr;

  /* Select a timeout */

#ifdef CONFIG_STM32F0_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_STM32F0_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_STM32F0_I2CTIMEOTICKS;
#endif

  /* Wait as stop might still be in progress; but stop might also
   * be set because of a timeout error: "The [STOP] bit is set and
   * cleared by software, cleared by hardware when a Stop condition is
   * detected, set by hardware when a timeout error is detected."
   */

  start = clock_systimer();
  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systimer() - start;

      /* Check for STOP condition */

      cr = stm32f0_i2c_getreg32(priv, STM32F0_I2C_CR2_OFFSET);
      if ((cr & I2C_CR2_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      sr = stm32f0_i2c_getreg32(priv, STM32F0_I2C_ISR_OFFSET);
      if ((sr & I2C_INT_TIMEOUT) != 0)
        {
          return;
        }
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.
   */

  i2cinfo("Timeout with CR: %04x SR: %04x\n", cr, sr);
}

/************************************************************************************
 * Name: stm32f0_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ************************************************************************************/

static inline void stm32f0_i2c_sem_post(FAR struct stm32f0_i2c_priv_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/************************************************************************************
 * Name: stm32f0_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ************************************************************************************/

static inline void stm32f0_i2c_sem_init(FAR struct stm32f0_i2c_priv_s *priv)
{
  nxsem_init(&priv->sem_excl, 0, 1);

#ifndef CONFIG_I2C_POLLED
  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->sem_isr, 0, 0);
  nxsem_setprotocol(&priv->sem_isr, SEM_PRIO_NONE);
#endif
}

/************************************************************************************
 * Name: stm32f0_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ************************************************************************************/

static inline void stm32f0_i2c_sem_destroy(FAR struct stm32f0_i2c_priv_s *priv)
{
  nxsem_destroy(&priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&priv->sem_isr);
#endif
}

/************************************************************************************
 * Name: stm32f0_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void stm32f0_i2c_traceclear(FAR struct stm32f0_i2c_priv_s *priv)
{
  struct stm32f0_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void stm32f0_i2c_tracereset(FAR struct stm32f0_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systimer();
  stm32f0_i2c_traceclear(priv);
}

static void stm32f0_i2c_tracenew(FAR struct stm32f0_i2c_priv_s *priv,
                               uint32_t status)
{
  struct stm32f0_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?   Has the status changed? */

  if (trace->count == 0 || status != trace->status)
    {
      /* Yes.. Was it the status changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index (unless we are out of trace entries) */

          if (priv->tndx >= (CONFIG_I2C_NTRACE-1))
            {
              i2cerr("ERROR: Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      stm32f0_i2c_traceclear(priv);
      trace->status = status;
      trace->count  = 1;
      trace->time   = clock_systimer();
    }
  else
    {
      /* Just increment the count of times that we have seen this status */

      trace->count++;
    }
}

static void stm32f0_i2c_traceevent(FAR struct stm32f0_i2c_priv_s *priv,
                                enum stm32f0_trace_e event, uint32_t parm)
{
  struct stm32f0_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event  = event;
      trace->parm   = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE-1))
        {
          i2cerr("ERROR: Trace table overflow\n");
          return;
        }

      priv->tndx++;
      stm32f0_i2c_traceclear(priv);
    }
}

static void stm32f0_i2c_tracedump(FAR struct stm32f0_i2c_priv_s *priv)
{
  struct stm32f0_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %ld\n",
         (long)(clock_systimer() - priv->start_time));

  for (i = 0; i <= priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. STATUS: %08x COUNT: %3d EVENT: %2d PARM: %08x TIME: %d\n",
             i+1, trace->status, trace->count,  trace->event, trace->parm,
             trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/************************************************************************************
 * Name: stm32f0_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ************************************************************************************/

static void stm32f0_i2c_setclock(FAR struct stm32f0_i2c_priv_s *priv, uint32_t frequency)
{
  uint32_t pe;
  uint8_t presc;
  uint8_t s_time;
  uint8_t h_time;
  uint8_t scl_h_period;
  uint8_t scl_l_period;

  /* XXX haque; these are the only freqs we support at the moment, until we can
   * compute the values ourself.  Pick the highest supported frequency that does
   * not exceed the requested frequency.
   */

  if (frequency < 100000)
    {
      frequency = 10000;  /* 0Hz <= frequency < 100KHz: Use 10Khz */
    }
  else if (frequency < 400000)
    {
      frequency = 100000; /* 100KHz <= frequency < 400KHz: Use 100KHz */
    }
  else if (frequency < 1000000)
    {
      frequency = 400000; /* 400KHz <= frequency < 1MHz: Use 400Khz */
    }
  else
    {
      frequency = 1000000; /* 1MHz <= frequency:  Use 1Mhz */
    }

  /* Has the I2C bus frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Disable the selected I2C peripheral to configure TRISE */

      pe = (stm32f0_i2c_getreg32(priv, STM32F0_I2C_CR1_OFFSET) & I2C_CR1_PE);
      if (pe)
        {
          stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR1_OFFSET, I2C_CR1_PE, 0);
        }

      /* Update timing and control registers */

      /* TODO: speed/timing calcs, taking into consideration
       * STM32F0_PCLK1_FREQUENCY, or SYSCLK, or HSI16
       * clock source, RCC_CCIPR, I2CxSEL, 0 = PCKL, 1 = SCLK, 2 = HSI16, 3 = reserved
#warning "check set filters before timing, see RM0351 35.4.4 p 1112"
       * analog filter; suppress spikes up to 50 ns in fast-mode and fast-mode plus
       * ANFOFF cr1
       * DNF cr1; 1-15 I2CCLK periods
       */
      /* RM0351 35.4.9 p 1140 */

      if (frequency == 10000)
        {
          /* 10 KHz values from I2C timing tool with clock 8mhz */

          presc        = 0x01;    /* PRESC - (+1) prescale I2CCLK */
          scl_l_period = 0xc7;    /* SCLL - SCL low period in master mode */
          scl_h_period = 0xc3;    /* SCLH - SCL high period in master mode */
          h_time       = 0x02;    /* SDADEL - (+1) data hold time after SCL falling edge */
          s_time       = 0x04;    /* SCLDEL - (+1) data setup time from SDA edge to SCL rising edge */
        }
     else if (frequency == 100000)
        {
          /* 100 KHz values from I2C timing tool with clock 8mhz */

          presc        = 0x01;    /* PRESC - (+1) prescale I2CCLK */
          scl_l_period = 0x13;    /* SCLL - SCL low period in master mode */
          scl_h_period = 0x0f;    /* SCLH - SCL high period in master mode */
          h_time       = 0x02;    /* SDADEL - (+1) data hold time after SCL falling edge */
          s_time       = 0x04;    /* SCLDEL - (+1) data setup time from SDA edge to SCL rising edge */
        }
      else if (frequency == 400000)
        {
          /* 400 KHz values from I2C timing tool for clock of 8mhz */

          presc        = 0x00;    /* PRESC - (+1) prescale I2CCLK */
          scl_l_period = 0x09;    /* SCLL - SCL low period in master mode */
          scl_h_period = 0x03;    /* SCLH - SCL high period in master mode */
          h_time       = 0x01;    /* SDADEL - (+1) data hold time after SCL falling edge */
          s_time       = 0x03;    /* SCLDEL - (+1) data setup time from SDA edge to SCL rising edge */
        }
      else
        {
          /* 1000 KHhz values from I2C timing tool for clock of 8mhz */

          presc        = 0x00;    /* PRESC - (+1) prescale I2CCLK */
          scl_l_period = 0x06;    /* SCLL - SCL low period in master mode */
          scl_h_period = 0x03;    /* SCLH - SCL high period in master mode */
          h_time       = 0x00;    /* SDADEL - (+1) data hold time after SCL falling edge */
          s_time       = 0x01;    /* SCLDEL - (+1) data setup time from SDA edge to SCL rising edge */
        }

      uint32_t timingr =
        (presc << I2C_TIMINGR_PRESC_SHIFT) |
        (s_time << I2C_TIMINGR_SCLDEL_SHIFT) |
        (h_time << I2C_TIMINGR_SDADEL_SHIFT) |
        (scl_h_period << I2C_TIMINGR_SCLH_SHIFT) |
        (scl_l_period << I2C_TIMINGR_SCLL_SHIFT);

      stm32f0_i2c_putreg32(priv, STM32F0_I2C_TIMINGR_OFFSET, timingr);

      /* Re-enable the peripheral (or not) */

      if (pe)
        {
          stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR1_OFFSET, 0, I2C_CR1_PE);
        }

      /* Save the new I2C frequency */

      priv->frequency = frequency;
    }
}

/************************************************************************************
 * Name: stm32f0_i2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ************************************************************************************/

static inline void stm32f0_i2c_sendstart(FAR struct stm32f0_i2c_priv_s *priv)
{
  /* Get run-time data */

  priv->astart = true;
  priv->ptr   = priv->msgv->buffer;
  priv->dcnt  = priv->msgv->length;
  priv->flags = priv->msgv->flags;

  /* Disable ACK on receive by default and generate START */

  stm32f0_i2c_set_bytes_to_transfer(priv, priv->dcnt);
  stm32f0_i2c_set_7bit_address(priv);
  if (priv->flags & I2C_M_READ)
    {
      stm32f0_i2c_set_read_transfer_dir(priv);
    }
  else
    {
      stm32f0_i2c_set_write_transfer_dir(priv);
    }

  if (priv->msgc == 1)
    {
      /* stm32f0_i2c_enable_autoend(priv); */
    }
  else
    {
      /* stm32f0_i2c_disable_autoend(priv); */
    }

  /* TODO check NACK */
  /* TODO handle NACKR? */

  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET, 0, I2C_CR2_START);
}

/************************************************************************************
 * Name: stm32f0_i2c_clrstart
 *
 * Description:
 *   Clear the STOP, START or PEC condition on certain error recovery steps.
 *
 ************************************************************************************/

static inline void stm32f0_i2c_clrstart(FAR struct stm32f0_i2c_priv_s *priv)
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
   *  when start is sent or PE=0."  The bit must be cleared by software if the
   *  START is never sent.
   *
   * "This [PEC] bit is set and cleared by software, and cleared by hardware
   *  when PEC is transferred or by a START or Stop condition or when PE=0."
   */

  /* TODO check PEC (32 bit separate reg) */

  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET,
                        I2C_CR2_START | I2C_CR2_STOP, 0);
}

/************************************************************************************
 * Name: stm32f0_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ************************************************************************************/

static inline void stm32f0_i2c_sendstop(FAR struct stm32f0_i2c_priv_s *priv)
{
  /* TODO check NACK */

  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR2_OFFSET, 0, I2C_CR2_STOP);
}

/************************************************************************************
 * Name: stm32f0_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status (SR1 and SR2 combined)
 *
 ************************************************************************************/

static inline uint32_t stm32f0_i2c_getstatus(FAR struct stm32f0_i2c_priv_s *priv)
{
  return getreg32(priv->config->base + STM32F0_I2C_ISR_OFFSET);
}

/************************************************************************************
 * Name: stm32f0_i2c_isr_startmessage
 *
 * Description:
 *   Common logic when a message is started.  Just adds the even to the trace buffer
 *   if enabled and adjusts the message pointer and count.
 *
 ************************************************************************************/

static inline void stm32f0_i2c_isr_startmessage(struct stm32f0_i2c_priv_s *priv)
{
  stm32f0_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgc);

  /* Increment to next pointer and decrement message count */

  priv->msgv++;
  priv->msgc--;
}

/************************************************************************************
 * Name: stm32f0_i2c_clearinterrupts
 *
 * Description:
 *  Clear all interrupts
 *
 ************************************************************************************/

static inline void stm32f0_i2c_clearinterrupts(struct stm32f0_i2c_priv_s *priv)
{
#warning "check this clears interrupts?"
    stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_ICR_OFFSET, 0, I2C_ICR_CLEARMASK);
}

/************************************************************************************
 * Name: stm32f0_i2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ************************************************************************************/

static int stm32f0_i2c_isr_process(struct stm32f0_i2c_priv_s *priv)
{
  uint32_t status = stm32f0_i2c_getstatus(priv);

  /* Check for new trace setup */

  stm32f0_i2c_tracenew(priv, status);

#warning "TODO: check clear interrupts after all actions"

  if (STATUS_NACK(status))
    {
      /* wait, reset this? */
    }
  else if (priv->astart)
    {
      stm32f0_i2c_isr_startmessage(priv);
      priv->astart = false;
    }

  /* Was address sent, continue with either sending or reading data */

  if ((priv->flags & I2C_M_READ) == 0 && STATUS_ADDR_TX(status))
    {
#warning "TODO: ADDRCF clear address interrupt flag"
      if (priv->dcnt > 0)
        {
          /* Send a byte */

          stm32f0_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
          stm32f0_i2c_putreg32(priv, STM32F0_I2C_TXDR_OFFSET, *priv->ptr++);
          priv->dcnt--;
        }
    }
  else if ((priv->flags & I2C_M_READ) != 0 && STATUS_ADDR(status))
    {
      /* Enable RxNE and TxE buffers in order to receive one or multiple bytes */

#warning "TODO: ADDRCF clear address interrupt flag"

#ifndef CONFIG_I2C_POLLED
      stm32f0_i2c_traceevent(priv, I2CEVENT_ITBUFEN, 0);
      stm32f0_i2c_enableinterrupts(priv);
#endif
    }

  /* More bytes to read */

  else if (STATUS_RXNE(status))
    {
      /* Read a byte, if dcnt goes < 0, then read dummy bytes to ack ISRs */

      if (priv->dcnt > 0)
        {
          stm32f0_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

          /* No interrupts or context switches may occur in the following
           * sequence.  Otherwise, additional bytes may be sent by the
           * device.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t flags = enter_critical_section();
#endif
          /* Receive a byte */

          *priv->ptr++ = (uint8_t) stm32f0_i2c_getreg32(priv, STM32F0_I2C_RXDR_OFFSET);

          /* Disable acknowledge when last byte is to be received */

          priv->dcnt--;
          if (priv->dcnt == 1)
            {
              /* autoend? */
            }

#ifdef CONFIG_I2C_POLLED
          leave_critical_section(flags);
#endif
        }
    }

  /* Do we have more bytes to send, enable/disable buffer interrupts
   * (these ISRs could be replaced by DMAs)
   */

#ifndef CONFIG_I2C_POLLED
  if (priv->dcnt > 0)
    {
      stm32f0_i2c_traceevent(priv, I2CEVENT_REITBUFEN, 0);
      stm32f0_i2c_enableinterrupts(priv);
    }
  else if ((priv->dcnt == 0) && (priv->msgc == 0))
    {
      stm32f0_i2c_traceevent(priv, I2CEVENT_DISITBUFEN, 0);
      stm32f0_i2c_disableinterrupts(priv);
    }
#endif

  /* Was last byte received or sent?  Hmmm... the F2 and F4 seems to differ from
   * the F1 in that BTF is not set after data is received (only RXNE).
   */

  if (priv->dcnt <= 0 && STATUS_TC(status))
    {
      /* Do we need to terminate or restart after this byte?
       * If there are more messages to send, then we may:
       *
       *  - continue with repeated start
       *  - or just continue sending writeable part
       *  - or we close down by sending the stop bit
       */

      if (priv->msgc > 0)
        {
          if (priv->msgv->flags & I2C_M_NOSTART)
            {
              stm32f0_i2c_traceevent(priv, I2CEVENT_BTFNOSTART, priv->msgc);
              priv->ptr   = priv->msgv->buffer;
              priv->dcnt  = priv->msgv->length;
              priv->flags = priv->msgv->flags;
              priv->msgv++;
              priv->msgc--;

              /* Restart this ISR! */

#ifndef CONFIG_I2C_POLLED
              stm32f0_i2c_enableinterrupts(priv);
#endif
            }
          else
            {
              stm32f0_i2c_traceevent(priv, I2CEVENT_BTFRESTART, priv->msgc);
              stm32f0_i2c_sendstart(priv);
            }
        }
      else if (priv->msgv)
        {
          stm32f0_i2c_traceevent(priv, I2CEVENT_BTFSTOP, 0);

          stm32f0_i2c_sendstop(priv);

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

  if ((status & I2C_ISR_ERRORMASK) != 0)
    {
      stm32f0_i2c_traceevent(priv, I2CEVENT_ERROR, 0);

      /* Clear interrupt flags */

      stm32f0_i2c_clearinterrupts(priv);

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

/************************************************************************************
 * Name: stm32f0_i2c_isr_process
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int stm32f0_i2c_isr(int irq, void *context, FAR void *arg)
{
  struct stm32f0_i2c_priv_s *priv = (struct stm32f0_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return stm32f0_i2c_isr_process(priv);
}
#endif

/************************************************************************************
 * Private Initialization and Deinitialization
 ************************************************************************************/

/************************************************************************************
 * Name: stm32f0_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ************************************************************************************/

static int stm32f0_i2c_init(FAR struct stm32f0_i2c_priv_s *priv)
{
  int ret;

  /* Power-up and configure GPIOs */
  /* Enable power and reset the peripheral */

  modifyreg32(STM32F0_RCC_APB1ENR, 0, priv->config->clk_bit);
  modifyreg32(STM32F0_RCC_APB1RSTR, 0, priv->config->reset_bit);
  modifyreg32(STM32F0_RCC_APB1RSTR, priv->config->reset_bit, 0);

  /* Configure pins */

  ret = stm32f0_configgpio(priv->config->scl_pin);
  if (ret < 0)
    {
      return ret;
    }

  ret = stm32f0_configgpio(priv->config->sda_pin);
  if (ret < 0)
    {
      stm32f0_unconfiggpio(priv->config->scl_pin);
      return ret;
    }

#ifndef CONFIG_I2C_POLLED
  /* Attach and enable the I2C interrupt */

  irq_attach(priv->config->irq, stm32f0_i2c_isr, priv);
  up_enable_irq(priv->config->irq);
#endif

  /* Set peripheral frequency, where it must be at least 2 MHz  for 100 kHz
   * or 4 MHz for 400 kHz.  This also disables all I2C interrupts.
   */

  /* Force a frequency update */

  priv->frequency = 0;

  /* TODO: i2c clock source RCC_CCIPR */
  /* RCC_CCIPR I2CxSEL (default is PCLK clock) */

  stm32f0_i2c_setclock(priv, 100000);

  /* Enable I2C */

  stm32f0_i2c_modifyreg32(priv, STM32F0_I2C_CR1_OFFSET, 0, I2C_CR1_PE);
  return OK;
}

/************************************************************************************
 * Name: stm32f0_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ************************************************************************************/

static int stm32f0_i2c_deinit(FAR struct stm32f0_i2c_priv_s *priv)
{
  /* Disable I2C */

  stm32f0_i2c_putreg32(priv, STM32F0_I2C_CR1_OFFSET, 0);

  /* Unconfigure GPIO pins */

  stm32f0_unconfiggpio(priv->config->scl_pin);
  stm32f0_unconfiggpio(priv->config->sda_pin);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  irq_detach(priv->config->irq);
#endif

  /* Disable clocking */

  modifyreg32(STM32F0_RCC_APB1ENR, priv->config->clk_bit, 0);
  return OK;
}

/************************************************************************************
 * Device Driver Operations
 ************************************************************************************/

/************************************************************************************
 * Name: stm32f0_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ************************************************************************************/

static int stm32f0_i2c_transfer(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs,
                              int count)
{
  FAR struct stm32f0_i2c_priv_s *priv = (struct stm32f0_i2c_priv_s *)dev;
  uint32_t status = 0;
  int ret = OK;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);

  /* Ensure that address or flags don't change meanwhile */

  stm32f0_i2c_sem_wait(priv);

  /* Wait for any STOP in progress. */

  stm32f0_i2c_sem_waitstop(priv);

  /* Clear any pending error interrupts */

  stm32f0_i2c_clearinterrupts(priv);

  /* "Note: When the STOP, START or PEC bit is set, the software must
   *  not perform any write access to I2C_CR1 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PEC request."  However, if the bits are
   *  not cleared by hardware, then we will have to do that from hardware.
   */

  stm32f0_i2c_clrstart(priv);

  /* Old transfers are done */

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  stm32f0_i2c_tracereset(priv);

  /* Set I2C clock frequency (on change it toggles I2C_CR1_PE !)
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  stm32f0_i2c_setclock(priv, msgs->frequency);

  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within stm32f0_i2c_waitdone().
   */

  priv->status = 0;

#ifndef CONFIG_I2C_POLLED
  stm32f0_i2c_enableinterrupts(priv);
#endif

  stm32f0_i2c_sendstart(priv);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (stm32f0_i2c_sem_waitdone(priv) < 0)
    {
      status = stm32f0_i2c_getstatus(priv);
      ret = -ETIMEDOUT;

      i2cerr("ERROR: Timed out: CR1: %08x status: %08x\n",
             stm32f0_i2c_getreg32(priv, STM32F0_I2C_CR1_OFFSET), status);

      /* "Note: When the STOP, START or PEC bit is set, the software must
       *  not perform any write access to I2C_CR1 before this bit is
       *  cleared by hardware. Otherwise there is a risk of setting a
       *  second STOP, START or PEC request."
       */

      stm32f0_i2c_clrstart(priv);

      /* Clear busy flag in case of timeout */

      status = priv->status & 0xffff;
    }
  else
    {
      /* clear SR2 (BUSY flag) as we've done successfully */

      status = priv->status & 0xffff;
    }

  status &= ~I2C_ISR_BUSY;
#if 0
  /* Refresh status */
  do
    {
      status = stm32f0_i2c_getstatus(priv);
    }
  while (STATUS_BUSY(status));
#endif

  /* Check for error status conditions */

  if ((status & I2C_ISR_ERRORMASK) != 0)
    {
      /* I2C_SR1_ERRORMASK is the 'OR' of the following individual bits: */

      if (status & I2C_INT_BERR)
        {
          /* Bus Error */

          ret = -EIO;
        }
      else if (status & I2C_INT_ARLO)
        {
          /* Arbitration Lost (master mode) */

          ret = -EAGAIN;
        }

      /* TODO Acknowledge failure */

      else if (status & I2C_INT_OVR)
        {
          /* Overrun/Underrun */

          ret = -EIO;
        }
      else if (status & I2C_INT_PECERR)
        {
          /* PEC Error in reception */

          ret = -EPROTO;
        }
      else if (status & I2C_INT_TIMEOUT)
        {
          /* Timeout or Tlow Error */

          ret = -ETIME;
        }

      /* This is not an error and should never happen since SMBus is not
       * enabled
       */

      else /* if (status & I2C_INT_ALERT) */
        {
          /* SMBus alert is an optional signal with an interrupt line for devices
           * that want to trade their ability to master for a pin.
           */

          ret = -EINTR;
        }
    }

  /* This is not an error, but should not happen.  The BUSY signal can hang,
   * however, if there are unhealthy devices on the bus that need to be reset.
   * NOTE:  We will only see this buy indication if stm32f0_i2c_sem_waitdone()
   * fails above;  Otherwise it is cleared.
   */

  else if ((status & I2C_ISR_BUSY) != 0)
    {
      /* I2C Bus is for some reason busy */

      ret = -EBUSY;
    }

  /* Dump the trace result */

  stm32f0_i2c_tracedump(priv);
  stm32f0_i2c_sem_post(priv);
  return ret;
}

/************************************************************************************
 * Name: stm32f0_i2c_reset
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
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
static int stm32f0_i2c_reset(FAR struct i2c_master_s * dev)
{
  FAR struct stm32f0_i2c_priv_s *priv = (struct stm32f0_i2c_priv_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t frequency;
  int ret = -EIO;

  DEBUGASSERT(dev);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  stm32f0_i2c_sem_wait(priv);

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  stm32f0_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  /* Let SDA go high */

  stm32f0_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!stm32f0_gpioread(sda_gpio))
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
      while (!stm32f0_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      stm32f0_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      stm32f0_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  stm32f0_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  stm32f0_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  stm32f0_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  stm32f0_gpiowrite(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  stm32f0_unconfiggpio(sda_gpio);
  stm32f0_unconfiggpio(scl_gpio);

  /* Re-init the port */

  ret = stm32f0_i2c_init(priv);
  if (ret < 0)
    {
      i2cerr("ERROR: stm32f0_i2c_init failed: %d\n", ret);
    }

  /* Restore the frequency */

  stm32f0_i2c_setclock(priv, frequency);

out:

  /* Release the port for re-use by other clients */

  stm32f0_i2c_sem_post(priv);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32f0_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ************************************************************************************/

FAR struct i2c_master_s *stm32f0_i2cbus_initialize(int port)
{
  struct stm32f0_i2c_priv_s *priv = NULL;
  irqstate_t flags;
  int ret;

#if STM32F0_PCLK1_FREQUENCY < 4000000
#   warning STM32F0_I2C_INIT: Peripheral clock must be at least 4 MHz to support 400 kHz operation.
#endif

#if STM32F0_PCLK1_FREQUENCY < 2000000
#   warning STM32F0_I2C_INIT: Peripheral clock must be at least 2 MHz to support 100 kHz operation.
  return NULL;
#endif

  /* Get I2C private structure. */

  switch (port)
    {
#ifdef CONFIG_STM32F0_I2C1
      case 1:
        priv = (struct stm32f0_i2c_priv_s *)&stm32f0_i2c1_priv;
        break;
#endif
#ifdef CONFIG_STM32F0_I2C2
      case 2:
        priv = (struct stm32f0_i2c_priv_s *)&stm32f0_i2c2_priv;
        break;
#endif
#ifdef CONFIG_STM32F0_I2C3
      case 3:
        priv = (struct stm32f0_i2c_priv_s *)&stm32f0_i2c3_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Init private data for the first time, increment refs count,
   * power-up hardware and configure GPIOs.
   */

  flags = enter_critical_section();

  if ((volatile int)priv->refs++ == 0)
    {
      stm32f0_i2c_sem_init(priv);
      ret = stm32f0_i2c_init(priv);
      if (ret < 0)
        {
          i2cerr("ERROR: stm32f0_i2c_init failed: %d\n", ret);
        }
    }

  leave_critical_section(flags);
  return (struct i2c_master_s *)priv;
}

/************************************************************************************
 * Name: stm32f0_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ************************************************************************************/

int stm32f0_i2cbus_uninitialize(FAR struct i2c_master_s * dev)
{
  FAR struct stm32f0_i2c_priv_s *priv = (struct stm32f0_i2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Decrement refs and check for underflow */

  if (priv->refs == 0)
    {
      return -ENODEV;
    }

  flags = enter_critical_section();

  if (--priv->refs)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  /* Disable power and other HW resource (GPIO's) */

  stm32f0_i2c_deinit(priv);

  /* Release unused resources */

  stm32f0_i2c_sem_destroy(priv);
  return OK;
}

#endif /* CONFIG_STM32F0_I2C1 || CONFIG_STM32F0_I2C2 || CONFIG_STM32F0_I2C3 */
