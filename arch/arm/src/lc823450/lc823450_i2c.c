/****************************************************************************
 * arch/arm/src/lc823450/lc823450_i2c.c
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
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
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stddef.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "lc823450_syscontrol.h"
#include "lc823450_clockconfig.h"
#include "lc823450_i2c.h"
#include "lc823450_gpio.h"

#ifdef CONFIG_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LC823450_I2C_TIMEOSEC
#  define CONFIG_LC823450_I2C_TIMEOSEC 0
#endif

#ifndef CONFIG_LC823450_I2C_TIMEOMS
#  define CONFIG_LC823450_I2C_TIMEOMS  500
#endif

#ifdef CONFIG_I2C_SLAVE
#  error "I2C driver cannot support CONFIG_I2C_SLAVE."
#endif

#if CONFIG_LC823450_I2C_TIMEOMS >= 1000
#  error "Unsupported value of CONFIG_LC823450_I2C_TIMEOMS"
#endif

#define GPIO_I2C0_SCL   (GPIO_PORT0 | GPIO_PIN7 | GPIO_MODE_OUTPUT | GPIO_VALUE_ZERO)
#define GPIO_I2C0_SDA   (GPIO_PORT0 | GPIO_PIN8 | GPIO_MODE_OUTPUT | GPIO_VALUE_ZERO)
#define GPIO_I2C1_SCL   (GPIO_PORT2 | GPIO_PINB | GPIO_MODE_OUTPUT | GPIO_VALUE_ZERO)
#define GPIO_I2C1_SDA   (GPIO_PORT2 | GPIO_PINC | GPIO_MODE_OUTPUT | GPIO_VALUE_ZERO)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum lc823450_irqstate_e
{
  IRQSTATE_IDLE = 0,    /* No I2C activity */
  IRQSTATE_WSTART,      /* Waiting for START Condition */
  IRQSTATE_WSEND,       /* Waiting for data transmission */
  IRQSTATE_WRECV,       /* Waiting for data reception */
  IRQSTATE_WSTOP,       /* Waiting for STOP Condition */
  IRQSTATE_DONE,        /* Interrupt activity complete */
};

/* I2C Device hardware configuration */

struct lc823450_i2c_config_s
{
  uint32_t base;                     /* I2C base address */
  uint32_t en_bit;                   /* I2C controller enable bit (clock/reset) */
#ifndef CONFIG_I2C_POLLED
  int      irq;                      /* IRQ number */
#endif
};

/* I2C Device Private Data */

struct lc823450_i2c_priv_s
{
  /* Standard I2C operations */

  FAR const struct i2c_ops_s *ops;

  /* Port configuration */

  FAR const struct lc823450_i2c_config_s *config;

  int   refs;                      /* Referernce count */
  sem_t sem_excl;                  /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;                   /* Interrupt wait semaphore */
#endif
  volatile uint8_t irqstate;       /* Interrupt handshake (see enum lc823450_irqstate_e) */

  uint32_t    frequency;           /* Current I2C bus furequency */
  uint8_t     msgc;                /* Message count */
  FAR struct i2c_msg_s *msgv;      /* Message list */
  FAR uint8_t *ptr;                /* Current message buffer */
  int         dcnt;                /* Current message length */
  uint16_t    flags;               /* Current message flags */

  uint32_t    timeoms;             /* I2C transfer timeout in msec */
  bool        timedout;            /* If true, I2C transfer timed-out */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int
  lc823450_i2c_sem_wait(FAR struct lc823450_i2c_priv_s *priv);
static inline void
  lc823450_i2c_sem_post(FAR struct lc823450_i2c_priv_s *priv);
static inline int
  lc823450_i2c_sem_waitdone(FAR struct lc823450_i2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
static inline void
  lc823450_i2c_enableirq(FAR struct lc823450_i2c_priv_s *priv);
static inline void
  lc823450_i2c_disableirq(FAR struct lc823450_i2c_priv_s *priv);
#endif
static inline bool
  lc823450_i2c_checkirq(FAR struct lc823450_i2c_priv_s *priv);
static inline bool
  lc823450_i2c_checkbusy(FAR struct lc823450_i2c_priv_s *priv);
static inline void
  lc823450_i2c_prepxfer(FAR struct lc823450_i2c_priv_s *priv);
static inline void
  lc823450_i2c_sendstart(FAR struct lc823450_i2c_priv_s *priv);
static inline void
  lc823450_i2c_sendstop(FAR struct lc823450_i2c_priv_s *priv);
static inline uint32_t
  lc823450_i2c_readdata(FAR struct lc823450_i2c_priv_s *priv);
static void lc823450_i2c_starttransfer(FAR struct lc823450_i2c_priv_s *priv);

static int lc823450_i2c_poll(FAR struct lc823450_i2c_priv_s *priv);
#ifndef CONFIG_I2C_POLLED
static int lc823450_i2c_isr(int irq, FAR void *context, FAR void *arg);
#endif

static int lc823450_i2c_init(FAR struct lc823450_i2c_priv_s *priv, int port);
static int
  lc823450_i2c_deinit(FAR struct lc823450_i2c_priv_s *priv, int port);

static int lc823450_i2c_transfer(FAR struct i2c_master_s *dev,
                                 FAR struct i2c_msg_s *msgs, int count);

#ifdef CONFIG_I2C_RESET
static int lc823450_i2c_reset(FAR struct i2c_master_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

struct i2c_ops_s lc823450_i2c_ops =
{
  .transfer  = lc823450_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset     = lc823450_i2c_reset
#endif
};

#ifdef CONFIG_LC823450_I2C0
static const struct lc823450_i2c_config_s lc823450_i2c0_config =
{
  .base   = LC823450_I2C0_REGBASE,
  .en_bit = MCLKCNTAPB_I2C0_CLKEN,  /* Same as MRSTCNTAPB_I2C0_RSTB */
#ifndef CONFIG_I2C_POLLED
  .irq    = LC823450_IRQ_I2C0,
#endif /* CONFIG_I2C_POLLED */
};

static struct lc823450_i2c_priv_s lc823450_i2c0_priv =
{
  .ops      = &lc823450_i2c_ops,
  .config   = &lc823450_i2c0_config,
  .refs     = 0,
  .irqstate = IRQSTATE_IDLE,
  .msgc     = 0,
  .msgv     = NULL,
  .ptr      = NULL,
  .dcnt     = 0,
  .flags    = 0,
  .timeoms  = CONFIG_LC823450_I2C_TIMEOMS,
  .timedout = false,
};
#endif /* CONFIG_LC823450_I2C0 */

#ifdef CONFIG_LC823450_I2C1
static const struct lc823450_i2c_config_s lc823450_i2c1_config =
{
  .base   = LC823450_I2C1_REGBASE,
  .en_bit = MCLKCNTAPB_I2C1_CLKEN,  /* Same as MRSTCNTAPB_I2C1_RSTB */
#ifndef CONFIG_I2C_POLLED
  .irq    = LC823450_IRQ_I2C1,
#endif /* CONFIG_I2C_POLLED */
};

static struct lc823450_i2c_priv_s lc823450_i2c1_priv =
{
  .ops      = &lc823450_i2c_ops,
  .config   = &lc823450_i2c1_config,
  .refs     = 0,
  .irqstate = IRQSTATE_IDLE,
  .msgc     = 0,
  .msgv     = NULL,
  .ptr      = NULL,
  .dcnt     = 0,
  .flags    = 0,
  .timeoms  = CONFIG_LC823450_I2C_TIMEOMS,
  .timedout = false,
};
#endif /* CONFIG_LC823450_I2C1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary.  May be interrupted by
 *   a signal.
 *
 ****************************************************************************/

static inline int lc823450_i2c_sem_wait(FAR struct lc823450_i2c_priv_s *priv)
{
  return nxsem_wait(&priv->sem_excl);
}

/****************************************************************************
 * Name: lc823450_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

static inline void
  lc823450_i2c_sem_post(FAR struct lc823450_i2c_priv_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/****************************************************************************
 * Name: lc823450_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int
  lc823450_i2c_sem_waitdone(FAR struct lc823450_i2c_priv_s *priv)
{
  struct timespec abstime;
  int ret;

  do
    {
      /* Get the current time */

      clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_LC823450_I2C_TIMEOSEC > 0
      abstime.tv_sec += CONFIG_LC823450_I2C_TIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

      abstime.tv_nsec += priv->timeoms * 1000 * 1000;
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

      /* Wait until either the transfer is complete or the timeout expires */

      ret = nxsem_timedwait_uninterruptible(&priv->sem_isr, &abstime);
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by nxsem_timedwait.
           */

          break;
        }
    }
  while (priv->irqstate != IRQSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->irqstate = IRQSTATE_IDLE;

  return ret;
}
#else
static inline int
  lc823450_i2c_sem_waitdone(FAR struct lc823450_i2c_priv_s *priv)
{
  uint32_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

  timeout = MSEC2TICK(priv->timeoms + (MSEC_PER_TICK / 2));
  timeout += SEC2TICK(CONFIG_LC823450_I2C_TIMEOSEC);

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * sem_timedwait() sleeps.
   */

  start = clock_systimer();

  do
    {
      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      lc823450_i2c_poll(priv);

      /* Calculate the elapsed time */

      elapsed = clock_systimer() - start;
    }
  while (priv->irqstate != IRQSTATE_DONE && elapsed < timeout);

  i2cinfo("irqstate: %d elapsed: %d threshold: %d status: %08x\n",
          priv->irqstate, elapsed, timeout);

  /* Set the interrupt state back to IDLE */

  ret = priv->irqstate == IRQSTATE_DONE ? OK : -ETIMEDOUT;
  priv->irqstate = IRQSTATE_IDLE;
  return ret;
}
#endif

/****************************************************************************
 * Name: lc823450_i2c_prepxfer
 *
 * Description:
 *   Set the I2C clock and slave address
 *
 ****************************************************************************/

static void lc823450_i2c_prepxfer(FAR struct lc823450_i2c_priv_s *priv)
{
  uint32_t base = priv->config->base;
  putreg32(((lc823450_get_systemfreq() / priv->msgv->frequency + 7) / 8) &
           0xffff,
           base + I2CCKS);
  putreg32((priv->msgv->addr << 1) | (priv->msgv->flags & I2C_M_READ),
           base + I2CTXD);
}

/****************************************************************************
 * Name: lc823450_i2c_checkbusy
 *
 * Description:
 *   Check if I2C bus is busy. Return true if I2C bus is busy.
 *
 ****************************************************************************/

static inline bool
  lc823450_i2c_checkbusy(FAR struct lc823450_i2c_priv_s *priv)
{
  return (getreg32(priv->config->base + I2CSTR) & I2C_STR_BBSY) != 0;
}

/****************************************************************************
 * Name: lc823450_i2c_checkirq
 *
 * Description:
 *   Check if interrupt occurred. Return true if irq occurs.
 *
 ****************************************************************************/

static inline bool
  lc823450_i2c_checkirq(FAR struct lc823450_i2c_priv_s *priv)
{
  return (getreg32(priv->config->base + I2CSTR) & I2C_STR_IREQ) != 0;
}

/****************************************************************************
 * Name: lc823450_i2c_checkack
 *
 * Description:
 *   Check if ACK detected. Return true if ACK detected.
 *
 ****************************************************************************/

static inline bool
  lc823450_i2c_checkack(FAR struct lc823450_i2c_priv_s *priv)
{
  return (getreg32(priv->config->base + I2CSTR) & I2C_STR_ACKD) != 0;
}

/****************************************************************************
 * Name: lc823450_i2c_sendstart
 *
 * Description:
 *   Send the START condition
 *
 ****************************************************************************/

static inline void
  lc823450_i2c_sendstart(FAR struct lc823450_i2c_priv_s *priv)
{
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_TRX, I2C_CTL_TRX);
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_ST, I2C_CTL_ST);
}

/****************************************************************************
 * Name: lc823450_i2c_sendstop
 *
 * Description:
 *   Send the STOP condition
 *
 ****************************************************************************/

static inline void
  lc823450_i2c_sendstop(FAR struct lc823450_i2c_priv_s *priv)
{
  modifyreg32(priv->config->base + I2CSTR, I2C_STR_IREQ, 0);
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_TRX, I2C_CTL_TRX);
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_ST, 0);
}

/****************************************************************************
 * Name: lc823450_i2c_clrstop
 *
 * Description:
 *   Clear the STOP condition
 *
 ****************************************************************************/

static inline void lc823450_i2c_clrstop(FAR struct lc823450_i2c_priv_s *priv)
{
  modifyreg32(priv->config->base + I2CSTR, I2C_STR_IREQ, 0);
}

/****************************************************************************
 * Name: lc823450_i2c_reset
 *
 * Description:
 *   Reset the I2C peripheral
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int lc823450_i2c_reset(FAR struct i2c_master_s *dev)
{
  FAR struct lc823450_i2c_priv_s *priv = (struct lc823450_i2c_priv_s *)dev;

  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_SRST, I2C_CTL_SRST);
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_IREQEN, 0);
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_FMODE, I2C_CTL_FMODE);
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Name: lc823450_i2c_enableirq
 *
 * Description:
 *   Enable I2C interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline void
  lc823450_i2c_enableirq(FAR struct lc823450_i2c_priv_s *priv)
{
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_IREQEN, I2C_CTL_IREQEN);
}
#endif

/****************************************************************************
 * Name: lc823450_i2c_disableirq
 *
 * Description:
 *   Disable I2C interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline void
  lc823450_i2c_disableirq(FAR struct lc823450_i2c_priv_s *priv)
{
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_IREQEN, 0);
}
#endif

/****************************************************************************
 * Name: lc823450_i2c_readdata
 *
 * Description:
 *   Get data received from transmitter.
 *
 ****************************************************************************/

static inline uint32_t
  lc823450_i2c_readdata(FAR struct lc823450_i2c_priv_s *priv)
{
  return getreg32(priv->config->base + I2CRXD);
}

/****************************************************************************
 * Name: lc823450_i2c_starttransfer
 *
 * Description:
 *  Start read or write request
 *
 ****************************************************************************/

static void lc823450_i2c_sendnack(FAR struct lc823450_i2c_priv_s *priv)
{
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_ACK, 0);
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_BTRIG, I2C_CTL_BTRIG);
}

static void lc823450_i2c_starttransfer(FAR struct lc823450_i2c_priv_s *priv)
{
  if (priv->flags & I2C_M_READ)
    {
      if (priv->dcnt > 1)
        {
          /* If the next byte to be received is not final, we have to send ACK. */

          modifyreg32(priv->config->base + I2CCTL, I2C_CTL_ACK,
                      I2C_CTL_ACK);
          modifyreg32(priv->config->base + I2CCTL, I2C_CTL_BTRIG,
                      I2C_CTL_BTRIG);
        }
      else if (priv->dcnt == 1)
        {
          /* otherwise, we don't have to send ACK when receiving it. */

          if (priv->msgc > 0 && priv->msgv->flags & I2C_M_READ)
            {
              /* But if there is a next message and the direction is READ,
               * we have to send ACK.
               */

              modifyreg32(priv->config->base + I2CCTL, I2C_CTL_ACK,
                          I2C_CTL_ACK);
            }
          else
            {
              modifyreg32(priv->config->base + I2CCTL, I2C_CTL_ACK, 0);
            }

          modifyreg32(priv->config->base + I2CCTL, I2C_CTL_BTRIG,
                      I2C_CTL_BTRIG);
        }

      priv->irqstate = IRQSTATE_WRECV;
    }
  else
    {
      if (priv->dcnt > 0)
        {
          putreg32(*priv->ptr, priv->config->base + I2CTXD);
          modifyreg32(priv->config->base +  I2CCTL,
                      I2C_CTL_BTRIG, I2C_CTL_BTRIG);
        }

      priv->irqstate = IRQSTATE_WSEND;
    }
}

/****************************************************************************
 * Name: lc823450_i2c_poll
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int lc823450_i2c_poll(FAR struct lc823450_i2c_priv_s *priv)
{
  bool ack;

  if (! lc823450_i2c_checkirq(priv))
    {
      return OK;
    }

  ack = lc823450_i2c_checkack(priv);

  /* Clear irq status */

  modifyreg32(priv->config->base + I2CSTR, I2C_STR_IREQ, 0);

  if (priv->timedout)
    {
      uint16_t flags = priv->msgv ? priv->msgv->flags : priv->flags;

      priv->dcnt = 0;
      priv->msgc = 0;

      if (flags & I2C_M_READ)
        {
          /* When READ transaction, terminate it with NACK and then STOP condition */

          lc823450_i2c_sendnack(priv);
        }
      else
        {
          /* When WRITE transaction, do STOP condition */

          priv->msgv = NULL;
          priv->irqstate = IRQSTATE_WSTOP;

          lc823450_i2c_sendstop(priv);
        }

      priv->timedout = false;

      return OK;
    }

  if (priv->irqstate == IRQSTATE_WSTART)
    {
      if (ack)
        {
          /* Wait until START condition is complete. */

          if (priv->msgv->length > 0)
            {
              priv->ptr   = priv->msgv->buffer;
              priv->dcnt  = priv->msgv->length;
              priv->flags = priv->msgv->flags;

              priv->msgv++;
              priv->msgc--;

              i2cinfo("WSTART (dcnt=%d flags=%xh msgc=%d)\n",
                      priv->dcnt, priv->flags, priv->msgc);

              lc823450_i2c_starttransfer(priv);
            }
        }
    }
  else if (priv->irqstate == IRQSTATE_WSEND)
    {
      /* Wait until ACK for the request to send is detected. */

      if (ack)
        {
          priv->ptr++;
          priv->dcnt--;
          i2cinfo("WSEND (dcnt=%d)\n", priv->dcnt);

          lc823450_i2c_starttransfer(priv);
        }
    }
  else if (priv->irqstate == IRQSTATE_WRECV)
    {
      /* When the master is receiver, it shall send ACK instead of the slave
       * when it receives data. In this case, it don't have to check if
       * ACK comes, because the slave does not send ACK.
       */

      *priv->ptr++ = lc823450_i2c_readdata(priv);
      priv->dcnt--;
      i2cinfo("WRECV (dcnt=%d)\n", priv->dcnt);

      lc823450_i2c_starttransfer(priv);
    }
  else if (priv->irqstate == IRQSTATE_WSTOP)
    {
      /* Wait until STOP condition is requested. */

      i2cinfo("WSTOP\n");

      lc823450_i2c_clrstop(priv);

      priv->irqstate = IRQSTATE_DONE;

#ifndef CONFIG_I2C_POLLED
      nxsem_post(&priv->sem_isr);
#endif
    }

  /* Check if a I2C message (struct i2c_msg_s) that has been currently
   * handled is complete.
   */

  if (priv->dcnt <= 0)
    {
      /* The current message is complete */

      i2cinfo("message transferred\n");

      if (priv->msgc > 0)
        {
          /* There are other messages remaining. */

          i2cinfo("other message remaining (msgc=%d)\n", priv->msgc);

          if (priv->msgv->flags & I2C_M_NOSTART)
            {
              /* In this case, we don't have to restart using START condition. */

              i2cinfo("no re-START condition\n");

              if (priv->msgv->length > 0)
                {
                  priv->ptr   = priv->msgv->buffer;
                  priv->dcnt  = priv->msgv->length;
                  priv->flags = priv->msgv->flags;

                  priv->msgv++;
                  priv->msgc--;

                  lc823450_i2c_starttransfer(priv);
                }
            }
          else
            {
              /* We need restart from START condition. If transfer direction
               * is different between current message and next message,
               * restart is necessary.
               */

              i2cinfo("re-START condition\n");

#ifdef CONFIG_I2C_RESET
              /* Reset I2C bus by softreset. There is not description of the
               * reset, but in order to recover I2C bus busy, it must be
               * done.  Please refer to macaron's code.
               */

              lc823450_i2c_reset((FAR struct i2c_master_s *)priv);
#endif

#ifndef CONFIG_I2C_POLLED
              /* We have to enable interrupt again, because all registers
               * are reset by lc823450_i2c_reset().
               */

              lc823450_i2c_enableirq(priv);
#endif

              lc823450_i2c_prepxfer(priv);
              lc823450_i2c_sendstart(priv);

              priv->irqstate = IRQSTATE_WSTART;
            }
        }
      else if (priv->msgv)
        {
          /* There are no other message. We send STOP condition because all
           * messages are transferred.
           */

          i2cinfo("STOP condition\n");

          lc823450_i2c_sendstop(priv);

          priv->msgv = NULL;
          priv->irqstate = IRQSTATE_WSTOP;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: lc823450_i2c_isr
 *
 * Description:
 *  Interrupt Service Routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int lc823450_i2c_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lc823450_i2c_priv_s *priv =
    (FAR struct lc823450_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return lc823450_i2c_poll(priv);
}
#endif

/****************************************************************************
 * Private Initialization and Deinitialization
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int lc823450_i2c_init(FAR struct lc823450_i2c_priv_s *priv, int port)
{
  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->irq, lc823450_i2c_isr, priv);
  up_enable_irq(priv->config->irq);
#endif

  /* Enable the I2C controller */

  modifyreg32(MCLKCNTAPB, 0, priv->config->en_bit);
  modifyreg32(MRSTCNTAPB, 0, priv->config->en_bit);

  /* I2C port settings */

  if (port == 0)
    {
      /* GPIO -> I2C0_SCL, I2C0_SDA */

      lc823450_gpio_mux(GPIO_I2C0_SCL | GPIO_MUX1);
      lc823450_gpio_mux(GPIO_I2C0_SDA | GPIO_MUX1);

#ifdef CONFIG_LC823450_I2C0_OPENDRAIN
      modifyreg32(I2CMODE, I2CMODE0, 0);
#else
      /* I2C SCL: PushPull */

      modifyreg32(I2CMODE, 0, I2CMODE0);
#endif
    }
#ifdef CONFIG_LC823450_I2C1
  else if (port == 1)
    {
      /* GPIO -> I2C1_SCL, I2C1_SDA */

      lc823450_gpio_mux(GPIO_I2C1_SCL | GPIO_MUX1);
      lc823450_gpio_mux(GPIO_I2C1_SDA | GPIO_MUX1);

#ifdef CONFIG_LC823450_I2C1_OPENDRAIN
      modifyreg32(I2CMODE, I2CMODE1, 0);
#else
      /* I2C SCL: PushPull */

      modifyreg32(I2CMODE, 0, I2CMODE1);
#endif
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: lc823450_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int lc823450_i2c_deinit(FAR struct lc823450_i2c_priv_s *priv,
                               int port)
{
  /* Disable I2C */

  modifyreg32(priv->config->base + I2CSTR, I2C_STR_IREQ, 0);
  modifyreg32(priv->config->base + I2CSTR, I2C_CTL_SCLR, I2C_CTL_SCLR);
  modifyreg32(priv->config->base + I2CCTL, I2C_CTL_SRST, I2C_CTL_SRST);

  /* Change pinmux from I2C to GPIO, and i2c mode to Push-Pull to Open Drain */

  if (port == 0)
    {
      /* I2C0_SCL -> GPIO(output,low) */

      lc823450_gpio_mux(GPIO_I2C0_SCL | GPIO_MUX0);
      lc823450_gpio_config(GPIO_I2C0_SCL);
      modifyreg32(I2CMODE, I2CMODE0, 0);

      /* I2C0_SDA -> GPIO(output,low) */

      lc823450_gpio_mux(GPIO_I2C0_SDA | GPIO_MUX0);
      lc823450_gpio_config(GPIO_I2C0_SDA);
    }
#ifdef CONFIG_LC823450_I2C1
  else if (port == 1)
    {
      /* I2C1_SCL -> GPIO(output,low) */

      lc823450_gpio_mux(GPIO_I2C1_SCL | GPIO_MUX0);
      lc823450_gpio_config(GPIO_I2C1_SCL);
      modifyreg32(I2CMODE, I2CMODE1, 0);

      /* I2C1_SDA -> GPIO(output,low) */

      lc823450_gpio_mux(GPIO_I2C1_SDA | GPIO_MUX0);
      lc823450_gpio_config(GPIO_I2C1_SDA);
    }
#endif

  /* Disable the I2C controller */

  modifyreg32(MCLKCNTAPB, priv->config->en_bit, 0);
  modifyreg32(MRSTCNTAPB, priv->config->en_bit, 0);

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
 * Name: lc823450_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int lc823450_i2c_transfer(FAR struct i2c_master_s *dev,
                                 FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct lc823450_i2c_priv_s *priv = (struct lc823450_i2c_priv_s *)dev;
  irqstate_t irqs;
  int ret = 0;

  DEBUGASSERT(count > 0);

  if (count <= 0 || msgs == NULL)
    {
      i2cerr("ERROR: invalid param, %p %d\n", msgs, count);
      return -EINVAL;
    }

  /* Ensure that address or flags don't change meanwhile */

  ret = lc823450_i2c_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->timedout = false;

#ifdef CONFIG_I2C_RESET
  /* Clear I2C peripheral */

  lc823450_i2c_reset(dev);
#endif

#ifndef CONFIG_I2C_POLLED
  /* Enable I2C interrupt */

  lc823450_i2c_enableirq(priv);
#endif

  /* Check I2C bus state */

  if (lc823450_i2c_checkbusy(priv))
    {
      i2cerr("ERROR: I2C bus busy (dev=%02xh)\n", msgs->addr);
      ret = -EBUSY;
      goto exit;
    }

  priv->msgv = msgs;
  priv->msgc = count;

  /* Set the frequency and slave address */

  lc823450_i2c_prepxfer(priv);

  priv->irqstate = IRQSTATE_WSTART;

  /* Trigger START condition, then the process moves into the ISR. */

  lc823450_i2c_sendstart(priv);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (lc823450_i2c_sem_waitdone(priv) < 0)
    {
      irqs = enter_critical_section();

      ret = -ETIMEDOUT;

      if (priv->dcnt != 0 || priv->msgc != 0)
        {
          /* If there is remaining data to be transferred, terminate it in
           * irq handler.
           */

          priv->timedout = true;

          leave_critical_section(irqs);

          /* Wait for irq handler completion. 10msec wait is probably enough
           * to terminate i2c transaction, NACK and STOP contition for read
           * transaction, STOP condition for write transaction
           */

          nxsig_usleep(10 * 1000);
        }
      else
        {
          i2cerr("No need of timeout handling. "
                 "It may be done in irq handler\n");

          leave_critical_section(irqs);
        }

#ifndef CONFIG_LC823450_IPL2
      i2cerr("ERROR: I2C timed out (dev=%xh)\n", msgs->addr);
#endif
    }

#ifndef CONFIG_I2C_POLLED
  /* Disable I2C interrupt */

  lc823450_i2c_disableirq(priv);
#endif

#ifdef CONFIG_I2C_RESET
  /* Do SRST if I2C transfer is complete */

  lc823450_i2c_reset(dev);
#endif

exit:
  lc823450_i2c_sem_post(priv);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

FAR struct i2c_master_s *lc823450_i2cbus_initialize(int port)
{
  FAR struct lc823450_i2c_priv_s *priv = NULL;
  irqstate_t flags;

  switch (port)
    {
#ifdef CONFIG_LC823450_I2C0
      case 0:
        priv = &lc823450_i2c0_priv;
        break;
#endif
#ifdef CONFIG_LC823450_I2C1
      case 1:
        priv = &lc823450_i2c1_priv;
        break;
#endif
    default:
      DEBUGASSERT(false);
      return NULL;
    }

  /* Init private data for the first time, increment refs count,
   * power-up hardware and configure GPIOs.
   */

  flags = enter_critical_section();

  if ((volatile int)priv->refs++ == 0)
    {
      nxsem_init(&priv->sem_excl, 0, 1);
#ifndef CONFIG_I2C_POLLED
      nxsem_init(&priv->sem_isr, 0, 0);
#endif
      lc823450_i2c_init(priv, port);
    }

  leave_critical_section(flags);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: lc823450_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int lc823450_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct lc823450_i2c_priv_s *priv = (struct lc823450_i2c_priv_s *)dev;
  irqstate_t flags;
  int port = -1;

  DEBUGASSERT(dev);

  /* Decrement refs and check for underflow */

  if (priv->refs == 0)
    {
      return OK;
    }

  flags = enter_critical_section();

  if (--priv->refs != 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

#ifdef CONFIG_LC823450_I2C0
  if (priv == &lc823450_i2c0_priv)
    {
      port = 0;
    }
#endif

#ifdef CONFIG_LC823450_I2C1
  if (priv == &lc823450_i2c1_priv)
    {
      port = 1;
    }
#endif

  if (-1 == port)
    {
      DEBUGASSERT(0);
      return -EFAULT;
    }

  /* Disable power and other HW resource */

  lc823450_i2c_deinit(priv, port);

  /* Release unused resources */

  nxsem_destroy(&priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&priv->sem_isr);
#endif

  return OK;
}

/****************************************************************************
 * Name: lc823450_i2cbus_changetimeout
 ****************************************************************************/

void lc823450_i2cbus_changetimeout(FAR struct i2c_master_s *dev,
                                   uint32_t timeoms)
{
  FAR struct lc823450_i2c_priv_s *priv = (struct lc823450_i2c_priv_s *)dev;
  priv->timeoms = timeoms;
}

#endif /* CONFIG_I2C */
