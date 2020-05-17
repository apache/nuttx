/****************************************************************************
 * arch/arm/src/am335x/am335x_i2c.c
 *
 *   Copyright (C) 2019 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
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

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_arch.h"
#include "hardware/am335x_pinmux.h"
#include "hardware/am335x_prcm.h"
#include "hardware/am335x_i2c.h"
#include "am335x_i2c.h"
#include "am335x_gpio.h"
#include "am335x_pinmux.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_AM335X_I2C0) || defined(CONFIG_AM335X_I2C1) || \
    defined(CONFIG_AM335X_I2C2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM335X_I2C_SCLK             (48000000)

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_AM335X_I2CTIMEOSEC) && !defined(CONFIG_AM335X_I2CTIMEOMS)
#  define CONFIG_AM335X_I2CTIMEOSEC 0
#  define CONFIG_AM335X_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_AM335X_I2CTIMEOSEC)
#  define CONFIG_AM335X_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_AM335X_I2CTIMEOMS)
#  define CONFIG_AM335X_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_AM335X_I2CTIMEOTICKS
#  define CONFIG_AM335X_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_AM335X_I2CTIMEOSEC) + MSEC2TICK(CONFIG_AM335X_I2CTIMEOMS))
#endif

#ifndef CONFIG_AM335X_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_AM335X_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_AM335X_I2CTIMEOTICKS)
#endif

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-leveldebug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define am335x_i2c_tracereset(p)
#  define am335x_i2c_tracenew(p,s)
#  define am335x_i2c_traceevent(p,e,a)
#  define am335x_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

#ifdef CONFIG_I2C_SLAVE
#  error I2C slave logic is not supported yet for AM335X
#endif

#define I2C_MASTER      1
#define I2C_SLAVE       2

#define MKI2C_OUTPUT(p) (am335x_periph_gpio(p) | PINMUX_SLEW_SLOW | \
                         PINMUX_PULL_UP_DISABLE | PINMUX_RX_ENABLE | \
                         GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum am335x_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum am335x_trace_e
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

struct am335x_trace_s
{
  uint32_t status;              /* I2C 32-bit SR2|SR1 status */
  uint32_t count;               /* Interrupt count when status change */
  enum am335x_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;                /* Parameter associated with the event */
  clock_t time;                 /* First of event or first status */
};

/* I2C Device hardware configuration */

struct am335x_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  gpio_pinset_t scl_pin;      /* GPIO configuration for SCL as SCL */
  gpio_pinset_t sda_pin;      /* GPIO configuration for SDA as SDA */
  uint8_t mode;               /* Master or Slave mode */
#ifndef CONFIG_I2C_POLLED
  uint32_t irq;               /* Event IRQ */
#endif
};

/* I2C Device Private Data */

struct am335x_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct am335x_i2c_config_s *config;

  int refs;                    /* Reference count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum am335x_intstate_e) */

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

  struct am335x_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t am335x_i2c_getreg(FAR struct am335x_i2c_priv_s *priv,
                                         uint16_t offset);
static inline void am335x_i2c_putreg(FAR struct am335x_i2c_priv_s *priv,
                                     uint16_t offset, uint32_t value);
static inline void am335x_i2c_modifyreg(FAR struct am335x_i2c_priv_s *priv,
                                        uint16_t offset, uint32_t clearbits,
                                        uint32_t setbits);
static inline int am335x_i2c_sem_wait(FAR struct am335x_i2c_priv_s *priv);
static int
  am335x_i2c_sem_wait_noncancelable(FAR struct am335x_i2c_priv_s *priv);

#ifdef CONFIG_AM335X_I2C_DYNTIMEO
static useconds_t am335x_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs);
#endif /* CONFIG_AM335X_I2C_DYNTIMEO */

static inline int
  am335x_i2c_sem_waitdone(FAR struct am335x_i2c_priv_s *priv);
static inline bool
  am335x_i2c_sem_waitstop(FAR struct am335x_i2c_priv_s *priv);
static inline void am335x_i2c_sem_post(FAR struct am335x_i2c_priv_s *priv);
static inline void am335x_i2c_sem_init(FAR struct am335x_i2c_priv_s *priv);
static inline void
  am335x_i2c_sem_destroy(FAR struct am335x_i2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void am335x_i2c_tracereset(FAR struct am335x_i2c_priv_s *priv);
static void am335x_i2c_tracenew(FAR struct am335x_i2c_priv_s *priv,
                                uint32_t status);
static void am335x_i2c_traceevent(FAR struct am335x_i2c_priv_s *priv,
                                  enum am335x_trace_e event, uint32_t parm);
static void am335x_i2c_tracedump(FAR struct am335x_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void am335x_i2c_setclock(FAR struct am335x_i2c_priv_s *priv,
                                uint32_t frequency);
static inline void am335x_i2c_sendstart(FAR struct am335x_i2c_priv_s *priv,
                                        uint16_t address);
static inline void am335x_i2c_sendstop(FAR struct am335x_i2c_priv_s *priv);
static inline uint32_t
  am335x_i2c_getstatus(FAR struct am335x_i2c_priv_s *priv);

static int am335x_i2c_isr_process(struct am335x_i2c_priv_s * priv);

#ifndef CONFIG_I2C_POLLED
static int am335x_i2c_isr(int irq, void *context, FAR void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int am335x_i2c_init(FAR struct am335x_i2c_priv_s *priv);
static int am335x_i2c_deinit(FAR struct am335x_i2c_priv_s *priv);
static int am335x_i2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int am335x_i2c_reset(FAR struct i2c_master_s *dev);
#endif

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

static const struct i2c_ops_s am335x_i2c_ops =
{
  .transfer = am335x_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = am335x_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_AM335X_I2C0
static const struct am335x_i2c_config_s am335x_i2c0_config =
{
  .base       = AM335X_I2C0_VADDR,
  .scl_pin    = GPIO_I2C0_SCL,
  .sda_pin    = GPIO_I2C0_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq        = AM335X_IRQ_I2C0,
#endif
};

static struct am335x_i2c_priv_s am335x_i2c0_priv =
{
  .ops        = &am335x_i2c_ops,
  .config     = &am335x_i2c0_config,
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

#ifdef CONFIG_AM335X_I2C1
static const struct am335x_i2c_config_s am335x_i2c1_config =
{
  .base       = AM335X_I2C1_VADDR,
  .scl_pin    = GPIO_I2C1_SCL,
  .sda_pin    = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq        = AM335X_IRQ_I2C1,
#endif
};

static struct am335x_i2c_priv_s am335x_i2c1_priv =
{
  .ops        = &am335x_i2c_ops,
  .config     = &am335x_i2c1_config,
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

#ifdef CONFIG_AM335X_I2C2
static const struct am335x_i2c_config_s am335x_i2c2_config =
{
  .base       = AM335X_I2C2_VADDR,
  .scl_pin    = GPIO_I2C2_SCL,
  .sda_pin    = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif
#ifndef CONFIG_I2C_POLLED
  .irq        = AM335X_IRQ_I2C2,
#endif
};

static struct am335x_i2c_priv_s am335x_i2c2_priv =
{
  .ops        = &am335x_i2c_ops,
  .config     = &am335x_i2c2_config,
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
 * Name: am335x_i2c_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t am335x_i2c_getreg(FAR struct am335x_i2c_priv_s *priv,
                                         uint16_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: am335x_i2c_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void am335x_i2c_putreg(FAR struct am335x_i2c_priv_s *priv,
                                     uint16_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: am335x_i2c_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void am335x_i2c_modifyreg(FAR struct am335x_i2c_priv_s *priv,
                                        uint16_t offset, uint32_t clearbits,
                                        uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: am335x_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary.  May be interrupted by
 *   a signal.
 *
 ****************************************************************************/

static inline int am335x_i2c_sem_wait(FAR struct am335x_i2c_priv_s *priv)
{
  return nxsem_wait(&priv->sem_excl);
}

/****************************************************************************
 * Name: am335x_i2c_sem_wait_noncancelable
 *
 * Description:
 *   Take the exclusive access, waiting as necessary.
 *
 ****************************************************************************/

static int
  am335x_i2c_sem_wait_noncancelable(FAR struct am335x_i2c_priv_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->sem_excl);
}

/****************************************************************************
 * Name: am335x_i2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_AM335X_I2C_DYNTIMEO
static useconds_t am335x_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs)
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

  return (useconds_t)(CONFIG_AM335X_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: am335x_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int am335x_i2c_sem_waitdone(FAR struct am335x_i2c_priv_s *priv)
{
  struct timespec abstime;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  flags = enter_critical_section();

  /* Enable Interrupts when master mode */

  if (priv->config->mode == I2C_MASTER)
    {
      if ((priv->flags & I2C_M_READ) != 0)
        {
          regval = I2C_IRQ_AL | I2C_IRQ_NACK | I2C_IRQ_RRDY
                   | I2C_IRQ_XRDY | I2C_IRQ_AERR | I2C_IRQ_BF;
          am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_SET_OFFSET, regval);
        }
      else
        {
          regval = I2C_IRQ_AL | I2C_IRQ_NACK | I2C_IRQ_XRDY
                   | I2C_IRQ_AERR | I2C_IRQ_BF;
          am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_SET_OFFSET, regval);
        }

      /* Force generate bus free interrupt to start transmission */

      am335x_i2c_putreg(priv, AM335X_I2C_IRQ_STAT_RAW_OFFSET, I2C_IRQ_BF);
    }

  /* Enable Interrupts when slave mode */

  else
    {
      #warning Missing logic for I2C Slave mode
    }

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Get the current time */

      clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_AM335X_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_AM335X_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_AM335X_I2C_DYNTIMEO
      abstime.tv_nsec += 1000 * am335x_i2c_tousecs(priv->msgc, priv->msgv);
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_AM335X_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_AM335X_I2CTIMEOMS * 1000 * 1000;
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

  am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_CLR_OFFSET, I2C_ICR_CLEARMASK);

  leave_critical_section(flags);
  return ret;
}
#else
static inline int am335x_i2c_sem_waitdone(FAR struct am335x_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_AM335X_I2C_DYNTIMEO
  timeout = USEC2TICK(am335x_i2c_tousecs(priv->msgc, priv->msgv));
#else
  timeout = CONFIG_AM335X_I2CTIMEOTICKS;
#endif

  priv->intstate = INTSTATE_WAITING;
  start = clock_systime_ticks();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      am335x_i2c_isr_process(priv);
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

/****************************************************************************
 * Name: am335x_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ****************************************************************************/

static inline bool
  am335x_i2c_sem_waitstop(FAR struct am335x_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t regval;

  /* Select a timeout */

#ifdef CONFIG_AM335X_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_AM335X_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_AM335X_I2CTIMEOTICKS;
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

      regval = am335x_i2c_getreg(priv, AM335X_I2C_IRQ_STAT_RAW_OFFSET);
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

  i2cinfo("Timeout with Status Register: %x\n", regval);
  return false;
}

/****************************************************************************
 * Name: am335x_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

static inline void am335x_i2c_sem_post(struct am335x_i2c_priv_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/****************************************************************************
 * Name: am335x_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ****************************************************************************/

static inline void am335x_i2c_sem_init(FAR struct am335x_i2c_priv_s *priv)
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
 * Name: am335x_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ****************************************************************************/

static inline void am335x_i2c_sem_destroy(FAR struct am335x_i2c_priv_s *priv)
{
  nxsem_destroy(&priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&priv->sem_isr);
#endif
}

/****************************************************************************
 * Name: am335x_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void am335x_i2c_traceclear(FAR struct am335x_i2c_priv_s *priv)
{
  struct am335x_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void am335x_i2c_tracereset(FAR struct am335x_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  am335x_i2c_traceclear(priv);
}

static void am335x_i2c_tracenew(FAR struct am335x_i2c_priv_s *priv,
                                uint32_t status)
{
  struct am335x_trace_s *trace = &priv->trace[priv->tndx];

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

      am335x_i2c_traceclear(priv);
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

static void am335x_i2c_traceevent(FAR struct am335x_i2c_priv_s *priv,
                                  enum am335x_trace_e event, uint32_t parm)
{
  struct am335x_trace_s *trace;

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
      am335x_i2c_traceclear(priv);
    }
}

static void am335x_i2c_tracedump(FAR struct am335x_i2c_priv_s *priv)
{
  struct am335x_trace_s *trace;
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
 * Name: am335x_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void am335x_i2c_setclock(FAR struct am335x_i2c_priv_s *priv,
                                uint32_t frequency)
{
  uint32_t src_freq = AM335X_I2C_SCLK;
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

          men = am335x_i2c_getreg(priv, AM335X_I2C_CON_OFFSET) & I2C_CON_EN;
          if (men)
            {
              am335x_i2c_modifyreg(priv, AM335X_I2C_CON_OFFSET,
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

          am335x_i2c_putreg(priv, AM335X_I2C_PSC_OFFSET, best_prescale);

          am335x_i2c_putreg(priv, AM335X_I2C_SCLL_OFFSET, best_scl_low);

          am335x_i2c_putreg(priv, AM335X_I2C_SCLH_OFFSET, best_scl_hi);

          /* Re-enable I2C if it was enabled previously */

          if (men)
            {
              am335x_i2c_modifyreg(priv, AM335X_I2C_CON_OFFSET,
                                   0, I2C_CON_EN);
            }

          /* Save the new I2C frequency */

          priv->frequency = frequency;
        }
    }
}

/****************************************************************************
 * Name: am335x_i2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ****************************************************************************/

static inline void am335x_i2c_sendstart(FAR struct am335x_i2c_priv_s *priv,
                                        uint16_t address)
{
  uint32_t regval;

  /* Generate START condition and send the address */

  regval = am335x_i2c_getreg(priv, AM335X_I2C_CON_OFFSET) | I2C_CON_STT |
                                   I2C_CON_MST;

  if ((priv->flags & I2C_M_READ) != 0)
    {
      regval &= ~I2C_CON_TRX;
    }
  else
    {
      regval |= I2C_CON_TRX;
    }

  if ((priv->flags & I2C_M_TEN) != 0)
    {
      regval |= I2C_CON_XSA;
    }
  else
    {
      regval &= ~I2C_CON_XSA;
    }

  am335x_i2c_putreg(priv, AM335X_I2C_SA_OFFSET, address);

  am335x_i2c_putreg(priv, AM335X_I2C_CON_OFFSET, regval);
}

/****************************************************************************
 * Name: am335x_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ****************************************************************************/

static inline void am335x_i2c_sendstop(FAR struct am335x_i2c_priv_s *priv)
{
  am335x_i2c_modifyreg(priv, AM335X_I2C_CON_OFFSET, 0, I2C_CON_STP);
}

/****************************************************************************
 * Name: am335x_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status
 *
 ****************************************************************************/

static inline uint32_t
  am335x_i2c_getstatus(FAR struct am335x_i2c_priv_s *priv)
{
#ifndef CONFIG_I2C_POLLED
  return am335x_i2c_getreg(priv, AM335X_I2C_IRQ_STAT_OFFSET);
#else
  return am335x_i2c_getreg(priv, AM335X_I2C_IRQ_STAT_RAW_OFFSET);
#endif
}

/****************************************************************************
 * Name: am335x_i2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int am335x_i2c_isr_process(struct am335x_i2c_priv_s *priv)
{
  uint32_t status = am335x_i2c_getstatus(priv);

  /* Check for new trace setup */

  am335x_i2c_tracenew(priv, status);

  /* Continue with either sending or reading data */

  /* Check if there is more bytes to send */

  if (((priv->flags & I2C_M_READ) == 0) && (status & I2C_IRQ_XRDY) != 0)
    {
      if (priv->dcnt > 0)
        {
          am335x_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);

          /* No interrupts or context switches should occur in the following
           * sequence. Otherwise, additional bytes may be sent by the device.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t flags = enter_critical_section();
#endif

          /* Transmit a byte */

          am335x_i2c_putreg(priv, AM335X_I2C_DATA_OFFSET, *priv->ptr++);
          priv->dcnt--;

#ifdef CONFIG_I2C_POLLED
          leave_critical_section(flags);
#endif
          if ((priv->dcnt == 0) && ((priv->flags & I2C_M_NOSTOP) == 0))
            {
              am335x_i2c_sendstop(priv);
            }
        }
    }

  /* Check if there is more bytes to read */

  else if (((priv->flags & I2C_M_READ) != 0) && (status & I2C_IRQ_RRDY) != 0)
    {
      /* Read a byte, if dcnt goes < 0, then read dummy bytes to ack ISRs */

      if (priv->dcnt > 0)
        {
          am335x_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

          /* No interrupts or context switches should occur in the following
           * sequence. Otherwise, additional bytes may be received by the
           * device.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t flags = enter_critical_section();
#endif

          /* Receive a byte */

          *priv->ptr++ = am335x_i2c_getreg(priv, AM335X_I2C_DATA_OFFSET) &
                         I2C_DATA_MASK;
          priv->dcnt--;

#ifdef CONFIG_I2C_POLLED
          leave_critical_section(flags);
#endif
          if ((priv->msgc <= 0) && (priv->dcnt == 0))
            {
              am335x_i2c_sendstop(priv);
            }
        }
      else
        {
          am335x_i2c_getreg(priv, AM335X_I2C_DATA_OFFSET);
        }
    }

  if (priv->dcnt <= 0)
    {
      if (priv->msgc > 0 && priv->msgv != NULL)
        {
          priv->ptr      = priv->msgv->buffer;
          priv->dcnt     = priv->msgv->length;
          priv->flags    = priv->msgv->flags;

          if ((priv->flags & I2C_M_NOSTART) == 0)
            {
              am335x_i2c_traceevent(priv, I2CEVENT_STARTRESTART, priv->msgc);
              am335x_i2c_sendstart(priv, priv->msgv->addr);
            }
          else
            {
              am335x_i2c_traceevent(priv, I2CEVENT_NOSTART, priv->msgc);
            }

          priv->msgv++;
          priv->msgc--;

          if ((priv->flags & I2C_M_READ) != 0)
            {
#ifndef CONFIG_I2C_POLLED
              /* Stop TX interrupt */

              am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_CLR_OFFSET,
                                I2C_IRQ_XRDY);
              am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_SET_OFFSET,
                                I2C_IRQ_RRDY);
#endif
              /* Set I2C in read mode */

              am335x_i2c_modifyreg(priv, AM335X_I2C_CON_OFFSET,
                                   I2C_CON_TRX, 0);
              am335x_i2c_putreg(priv, AM335X_I2C_CNT_OFFSET, priv->dcnt);
            }
          else
            {
              /* Send the first byte from tx buffer */

              am335x_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
              am335x_i2c_modifyreg(priv, AM335X_I2C_CON_OFFSET,
                                   0, I2C_CON_TRX);

              /* No interrupts or context switches should occur in the
               * following sequence. Otherwise, additional bytes may be sent
               * by the device.
               */

#ifdef CONFIG_I2C_POLLED
              irqstate_t flags = enter_critical_section();
#endif

              /* Transmit a byte */

              am335x_i2c_putreg(priv, AM335X_I2C_DATA_OFFSET, *priv->ptr++);
              priv->dcnt--;

#ifdef CONFIG_I2C_POLLED
              leave_critical_section(flags);
#endif
              if ((priv->dcnt == 0) && ((priv->flags & I2C_M_NOSTOP) == 0))
                {
                  am335x_i2c_sendstop(priv);
                }
            }
        }
      else if (priv->msgv && ((status & I2C_IRQ_BF) != 0))
        {
          am335x_i2c_traceevent(priv, I2CEVENT_STOP, 0);

          /* Check is there thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
          am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_CLR_OFFSET,
                            I2C_ICR_CLEARMASK);

          if (priv->intstate == INTSTATE_WAITING)
            {
              /* inform the thread that transfer is complete
               * and wake it up
               */

              nxsem_post(&priv->sem_isr);
              priv->intstate = INTSTATE_DONE;
            }
#else
          priv->intstate = INTSTATE_DONE;
#endif
          /* Mark that this transaction stopped */

          priv->msgv = NULL;
        }
#ifndef CONFIG_I2C_POLLED
      else
        {
          am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_CLR_OFFSET,
                            I2C_IRQ_XRDY | I2C_IRQ_RRDY);
        }
#endif
    }

  /* Check for errors */

  if ((status & I2C_IRQ_ERRORMASK) != 0)
    {
      am335x_i2c_traceevent(priv, I2CEVENT_ERROR, 0);

#ifndef CONFIG_I2C_POLLED
      am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_CLR_OFFSET,
                        I2C_ICR_CLEARMASK);

      if (priv->intstate == INTSTATE_WAITING)
        {
          /* inform the thread that transfer is complete
           * and wake it up
           */

          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#else
      priv->intstate = INTSTATE_DONE;
#endif
    }

  /* Clear interrupt status */

  am335x_i2c_putreg(priv, AM335X_I2C_IRQ_STAT_OFFSET, status);

  priv->status = status;
  return OK;
}

/****************************************************************************
 * Name: am335x_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int am335x_i2c_isr(int irq, void *context, FAR void *arg)
{
  struct am335x_i2c_priv_s *priv = (struct am335x_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return am335x_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: am335x_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int am335x_i2c_init(FAR struct am335x_i2c_priv_s *priv)
{
  /* Power-up and configure GPIOs */

  /* Configure pins */

  am335x_gpio_config(priv->config->scl_pin);
  am335x_gpio_config(priv->config->sda_pin);

  /* Disable I2C module */

  am335x_i2c_putreg(priv, AM335X_I2C_CON_OFFSET, 0);

  /* Apply soft reset */

  am335x_i2c_putreg(priv, AM335X_I2C_SYSC_OFFSET, I2C_SYSC_SRST);

  /* Disable auto-idle mode */

  am335x_i2c_modifyreg(priv, AM335X_I2C_SYSC_OFFSET, I2C_SYSC_AUTOIDLE, 0);

  /* Force a frequency update */

  priv->frequency = 0;
  am335x_i2c_setclock(priv, 100000);

#ifndef CONFIG_I2C_POLLED
  /* Attach ISRs */

  irq_attach(priv->config->irq, am335x_i2c_isr, priv);
  up_enable_irq(priv->config->irq);
#endif

  /* Enable I2C module */

  am335x_i2c_modifyreg(priv, AM335X_I2C_CON_OFFSET, 0, I2C_CON_EN);

  /* Select free running mode */

  am335x_i2c_modifyreg(priv, AM335X_I2C_SYSTEST_OFFSET, 0, I2C_SYSTEST_FREE);

  /* Wait for I2C module comes out of reset */

  while (!(am335x_i2c_getreg(priv, AM335X_I2C_SYSS_OFFSET) &
           I2C_SYSS_RST_DONE))
    {
    }

  return OK;
}

/****************************************************************************
 * Name: am335x_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int am335x_i2c_deinit(FAR struct am335x_i2c_priv_s *priv)
{
  /* Disable I2C module */

  am335x_i2c_putreg(priv, AM335X_I2C_CON_OFFSET, 0);

  /* Apply soft reset */

  am335x_i2c_putreg(priv, AM335X_I2C_SYSC_OFFSET, I2C_SYSC_SRST);

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
 * Name: am335x_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int am335x_i2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct am335x_i2c_priv_s *priv = (struct am335x_i2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  ret = am335x_i2c_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for any STOP in progress */

  ret = -EBUSY;
  if (am335x_i2c_sem_waitstop(priv))
    {
      /* Clear any pending error interrupts */

      am335x_i2c_putreg(priv, AM335X_I2C_IRQ_STAT_OFFSET,
                        I2C_STS_CLEARMASK);
      am335x_i2c_putreg(priv, AM335X_I2C_IRQ_EN_CLR_OFFSET,
                        I2C_ICR_CLEARMASK);

      /* Old transfers are done */

      /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
       * overwrite stale data.
       */

      priv->dcnt  = 0;
      priv->ptr   = NULL;

      priv->msgv  = msgs;
      priv->msgc  = count;
      priv->flags = msgs->flags;

      i2cinfo("Flags %x, len %d \n", msgs->flags, msgs->length);

      /* Reset I2C trace logic */

      am335x_i2c_tracereset(priv);

      /* Set I2C clock frequency */

      am335x_i2c_setclock(priv, msgs->frequency);

      priv->status = 0;

      /* Wait for an ISR, if there was a timeout, fetch latest status to get
       * the BUSY flag.
       */

      if (am335x_i2c_sem_waitdone(priv) < 0)
        {
          ret = -ETIMEDOUT;

          i2cerr("ERROR: Timed out: IRQ_RAW: status: 0x%x\n", priv->status);
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

      am335x_i2c_tracedump(priv);

      /* Ensure ISR happening after we finish can't overwrite any user data */

      priv->dcnt = 0;
      priv->ptr = NULL;
    }

  am335x_i2c_sem_post(priv);
  return ret;
}

/****************************************************************************
 * Name: am335x_i2c_reset
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
static int am335x_i2c_reset(FAR struct i2c_master_s *dev)
{
  FAR struct am335x_i2c_priv_s *priv = (FAR struct am335x_i2c_priv_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  gpio_pinset_t scl_gpio;
  gpio_pinset_t sda_gpio;
  uint32_t frequency;
  int ret;

  DEBUGASSERT(dev);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = am335x_i2c_sem_wait_noncancelable(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  am335x_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  am335x_gpio_config(scl_gpio);
  am335x_gpio_config(sda_gpio);

  /* Let SDA go high */

  am335x_gpio_write(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!am335x_gpio_read(sda_gpio))
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
      while (!am335x_gpio_read(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      am335x_gpio_write(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      am335x_gpio_write(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  am335x_gpio_write(sda_gpio, 0);
  up_udelay(10);
  am335x_gpio_write(scl_gpio, 0);
  up_udelay(10);
  am335x_gpio_write(scl_gpio, 1);
  up_udelay(10);
  am335x_gpio_write(sda_gpio, 1);
  up_udelay(10);

  /* Re-init the port */

  am335x_i2c_init(priv);

  /* Restore the frequency */

  am335x_i2c_setclock(priv, frequency);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  am335x_i2c_sem_post(priv);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

FAR struct i2c_master_s *am335x_i2cbus_initialize(int port)
{
  struct am335x_i2c_priv_s * priv = NULL;
  irqstate_t flags;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_AM335X_I2C0
    case 0:
      priv = (struct am335x_i2c_priv_s *)&am335x_i2c0_priv;
      break;
#endif
#ifdef CONFIG_AM335X_I2C1
    case 1:
      priv = (struct am335x_i2c_priv_s *)&am335x_i2c1_priv;
      break;
#endif
#ifdef CONFIG_AM335X_I2C2
    case 2:
      priv = (struct am335x_i2c_priv_s *)&am335x_i2c2_priv;
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
      am335x_i2c_sem_init(priv);
      am335x_i2c_init(priv);
    }

  leave_critical_section(flags);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: am335x_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int am335x_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct am335x_i2c_priv_s *priv = (struct am335x_i2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs > 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  /* Disable power and other HW resource (GPIO's) */

  am335x_i2c_deinit(priv);

  /* Release unused resources */

  am335x_i2c_sem_destroy(priv);
  return OK;
}

#endif /* CONFIG_AM335X_I2C0 || CONFIG_AM335X_I2C1 || \
        * CONFIG_AM335X_I2C2 */
