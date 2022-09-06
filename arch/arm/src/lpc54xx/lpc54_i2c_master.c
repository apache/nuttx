/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_i2c_master.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file were adapted from sample code provided for the LPC54xx
 * family from NXP which has a compatible BSD license.
 *
 *   Copyright (c) 2016, Freescale Semiconductor, Inc.
 *   Copyright 2016-2017 NXP
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/lpc54_pinmux.h"
#include "hardware/lpc54_syscon.h"
#include "hardware/lpc54_flexcomm.h"
#include "hardware/lpc54_i2c.h"
#include "lpc54_config.h"
#include "lpc54_clockconfig.h"
#include "lpc54_enableclk.h"
#include "lpc54_gpio.h"
#include "lpc54_i2c_master.h"

#include <arch/board/board.h>

#ifdef HAVE_I2C_MASTER_DEVICE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 20 Millisecond timeout in system clock ticks. */

#ifdef CONFIG_DEBUG_I2C_INFO
#  define I2C_WDOG_TIMEOUT MSEC2TICK(50)
#else
#  define I2C_WDOG_TIMEOUT MSEC2TICK(20)
#endif

/* Default I2C frequency */

#if defined(CONFIG_LPC54_I2C_FAST) || defined(CONFIG_LPC54_I2C_HIGH)
#  define I2C_DEFAULT_FREQUENCY 1000000
#else
#  define I2C_DEFAULT_FREQUENCY 400000
#endif

/* I2C Master Interrupts */

#define I2C_MASTER_INTS \
  (I2C_INT_MSTPENDING | I2C_INT_MSTARBLOSS | I2C_INT_MSTSTSTPERR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C state */

enum lpc54_i2cstate_e
{
  I2CSTATE_IDLE = 0,
  I2CSTATE_TRANSMIT,
  I2CSTATE_RECEIVE,
  I2CSTATE_START,
  I2CSTATE_STOP,
  I2CSTATE_WAITSTOP
};

/* This structure provides the overall state of the I2C driver */

struct lpc54_i2cdev_s
{
  struct i2c_master_s dev;  /* Generic I2C device */
  uintptr_t base;           /* Base address of Flexcomm registers */

  struct wdog_s timeout;    /* Watchdog to timeout when bus hung */
  uint32_t frequency;       /* Current I2C frequency */
  uint32_t fclock;          /* Flexcomm function clock frequency */

  struct i2c_msg_s *msgs;   /* Remaining transfers (first is in progress) */
  int16_t nmsgs;            /* Number of transfer remaining */
  int16_t result;           /* The result of the transfer */

  mutex_t lock;             /* Only one thread can access at a time */
#ifndef CONFIG_I2C_POLLED
  sem_t waitsem;            /* Supports wait for state machine completion */
  uint16_t irq;             /* Flexcomm IRQ number */
#endif
  uint16_t xfrd;            /* Number of bytes transferred */
  volatile uint8_t state;   /* State of state machine */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void lpc54_i2c_putreg(struct lpc54_i2cdev_s *priv,
              unsigned int regoffset, uint32_t regval);
static inline uint32_t lpc54_i2c_getreg(struct lpc54_i2cdev_s *priv,
              unsigned int regoffset);

static void lpc54_i2c_setfrequency(struct lpc54_i2cdev_s *priv,
              uint32_t frequency);
static void lpc54_i2c_timeout(wdparm_t arg);
static void lpc54_i2c_xfrsetup(struct lpc54_i2cdev_s *priv);
static bool lpc54_i2c_nextmsg(struct lpc54_i2cdev_s *priv);
static bool lpc54_i2c_statemachine(struct lpc54_i2cdev_s *priv);
#ifndef CONFIG_I2C_POLLED
static int  lpc54_i2c_interrupt(int irq, void *context, void *arg);
#else
static int  lpc54_i2c_poll(struct lpc54_i2cdev_s *priv);
#endif
static int  lpc54_i2c_transfer(struct i2c_master_s *dev,
              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  lpc54_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct i2c_ops_s lpc54_i2c_ops =
{
  .transfer = lpc54_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = lpc54_i2c_reset
#endif
};

#ifdef CONFIG_LPC54_I2C0_MASTER
static struct lpc54_i2cdev_s g_i2c0_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C1_MASTER
static struct lpc54_i2cdev_s g_i2c1_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C2_MASTER
static struct lpc54_i2cdev_s g_i2c2_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C3_MASTER
static struct lpc54_i2cdev_s g_i2c3_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C4_MASTER
static struct lpc54_i2cdev_s g_i2c4_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C5_MASTER
static struct lpc54_i2cdev_s g_i2c5_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C6_MASTER
static struct lpc54_i2cdev_s g_i2c6_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C7_MASTER
static struct lpc54_i2cdev_s g_i2c7_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C8_MASTER
static struct lpc54_i2cdev_s g_i2c8_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

#ifdef CONFIG_LPC54_I2C9_MASTER
static struct lpc54_i2cdev_s g_i2c9_dev =
{
  .lock = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .waitsem = SEM_INITIALIZER(0),
#  endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_i2c_putreg
 *
 * Description:
 *   Write a value to a register at the offset from the Flexcomm base.
 *
 ****************************************************************************/

static inline void lpc54_i2c_putreg(struct lpc54_i2cdev_s *priv,
                                    unsigned int regoffset, uint32_t regval)
{
  putreg32(regval, priv->base + regoffset);
}

/****************************************************************************
 * Name: lpc54_i2c_getreg
 *
 * Description:
 *   Read the content of a register at the offset from the Flexcomm base.
 *
 ****************************************************************************/

static inline uint32_t lpc54_i2c_getreg(struct lpc54_i2cdev_s *priv,
                                        unsigned int regoffset)
{
  return getreg32(priv->base + regoffset);
}

/****************************************************************************
 * Name: lpc54_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void lpc54_i2c_setfrequency(struct lpc54_i2cdev_s *priv,
                                   uint32_t frequency)
{
  uint32_t scl;
  uint32_t divider;
  uint32_t best_scl;
  uint32_t best_div;
  uint32_t err;
  uint32_t best_err;
  uint32_t regval;

  i2cinfo("frequency %ld (%ld)\n", (long)frequency, (long)priv->frequency);

  /* Has the I2C frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Yes.. instantiate the new I2C frequency */

      best_err = 0;

      for (scl = 9; scl >= 2; scl--)
        {
          /* Calculate ideal divider value for the current scl candidate.
           *
           *   SCL High Time:  Thi = divider * SCLhi
           *   SCL High Time:  Tlo = divider * SCLlo
           *                   Fscl = Finput / (Thi + Tlo)
           *
           * If Thi == TloL:   Fscl = Finput / (divider * SCL * 2)
           * Or:               divider = Finput / (Fscl * SCL * 2)
           */

          divider = priv->fclock / (frequency * scl * 2);

          /* Adjust it if it is out of range */

          if (divider > 0x00010000)
            {
              divider = 0x00010000;
            }

          /* Calculate the frequency error */

          err = priv->fclock - (frequency * scl * 2 * divider);
          if (err < best_err || best_err == 0)
            {
              best_div = divider;
              best_scl = scl;
              best_err = err;
            }

          if (err == 0 || divider >= 0x10000)
            {
              /* Break out of the loop early ifeither exact value was found
               * or the divider is at its maximum value.
               */

              break;
            }
        }

      /* Instantiate the new I2C frequency */

      regval = I2C_CLKDIV(best_div);
      lpc54_i2c_putreg(priv, LPC54_I2C_CLKDIV_OFFSET, regval);

      regval = I2C_MSTTIME_SCLLOW(best_scl) | I2C_MSTTIME_SCLHIGH(best_scl);
      lpc54_i2c_putreg(priv, LPC54_I2C_MSTTIME_OFFSET, regval);

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: lpc54_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void lpc54_i2c_timeout(wdparm_t arg)
{
  struct lpc54_i2cdev_s *priv = (struct lpc54_i2cdev_s *)arg;

#ifndef CONFIG_I2C_POLLED
  irqstate_t flags = enter_critical_section();
#endif

  i2cerr("ERROR: Timeout! state=%u\n", priv->state);

  /* Disable further I2C interrupts and return to the IDLE state with the
   * timeout result.
   */

  lpc54_i2c_putreg(priv, LPC54_I2C_INTENCLR_OFFSET, I2C_MASTER_INTS);
  priv->state  = I2CSTATE_IDLE;

  if (priv->result == OK)
    {
      priv->result = -ETIMEDOUT;
    }

#ifndef CONFIG_I2C_POLLED
  /* Wake up any waiters */

  nxsem_post(&priv->waitsem);
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Name: lpc54_i2c_xfrsetup
 *
 * Description:
 *   Setup to initiate a transfer.
 *
 ****************************************************************************/

static void lpc54_i2c_xfrsetup(struct lpc54_i2cdev_s *priv)
{
  struct i2c_msg_s *msg;

  DEBUGASSERT(priv != NULL && priv->msgs != NULL);
  msg = priv->msgs;

  /* Disable I2C interrupts while configuring for the transfer */

  lpc54_i2c_putreg(priv, LPC54_I2C_INTENCLR_OFFSET, I2C_MASTER_INTS);

  /* Set up for the transfer */

  priv->xfrd = 0;

  /* Select the initial state */

  if ((msg->flags & I2C_M_NOSTART) != 0)
    {
      /* Start condition will be omitted.  Begin the transfer in the data
       * phase.
       */

      if (msg->length == 0)
        {
          priv->state = I2CSTATE_STOP;
        }
      else if ((I2C_M_READ & msg->flags) == I2C_M_READ)
        {
          priv->state = I2CSTATE_RECEIVE;
        }
      else
        {
          priv->state = I2CSTATE_TRANSMIT;
        }
    }
  else
    {
      priv->state = I2CSTATE_START;
    }

  /* Set the I2C frequency if provided in this message.  Otherwise, use the
   * current I2C frequency setting.
   */

  if (msg->frequency > 0)
    {
      lpc54_i2c_setfrequency(priv, msg->frequency);
    }

  /* Clear error status bits */

  lpc54_i2c_putreg(priv, LPC54_I2C_STAT_OFFSET, I2C_INT_MSTARBLOSS |
                         I2C_INT_MSTSTSTPERR);

#ifndef CONFIG_I2C_POLLED
  /* Enable I2C master interrupts */

  lpc54_i2c_putreg(priv, LPC54_I2C_INTENSET_OFFSET, I2C_MASTER_INTS);
#endif
}

/****************************************************************************
 * Name: lpc54_i2c_nextmsg
 *
 * Description:
 *   Called at the completion of each message.  If there are more messages,
 *   this function will perform the setup for the next message.
 *
 ****************************************************************************/

static bool lpc54_i2c_nextmsg(struct lpc54_i2cdev_s *priv)
{
  irqstate_t flags;

  /* Disable interrupts to prevent the timeout while we make the decision
   * here.
   */

  flags = enter_critical_section();

  i2cinfo("nmsgs=%u\n", priv->nmsgs - 1);

  /* Decrement the number of messages remaining. */

  if (--priv->nmsgs > 0 && priv->result == OK)
    {
      /* There are more messages, set up for the next message */

      priv->msgs++;
      lpc54_i2c_xfrsetup(priv);

      i2cinfo("state=%u\n", priv->state);
      leave_critical_section(flags);
      return false;
    }
  else
    {
      /* That was the last message... we are done.
       * Cancel any timeout
       */

      wd_cancel(&priv->timeout);

      /* Disable further I2C interrupts  and return to the IDLE state */

      lpc54_i2c_putreg(priv, LPC54_I2C_INTENCLR_OFFSET, I2C_MASTER_INTS);
      priv->state = I2CSTATE_IDLE;

      i2cinfo("state=%u\n", priv->state);
      leave_critical_section(flags);
      return true;
    }
}

/****************************************************************************
 * Name: lpc54_i2c_statemachine
 *
 * Description:
 *   This is the I2C transfer state machine that implements the actual
 *   transfer.  It may be called from the interrupt level or is may be used
 *   without interrupts in a polled mode.
 *
 ****************************************************************************/

static bool lpc54_i2c_statemachine(struct lpc54_i2cdev_s *priv)
{
  struct i2c_msg_s *msg;
  uint32_t status;
  uint32_t mstate;

  DEBUGASSERT(priv != NULL && priv->msgs != NULL);
  msg = priv->msgs;

  i2cinfo("state=%u\n", priv->state);

  status = lpc54_i2c_getreg(priv, LPC54_I2C_STAT_OFFSET);

  if (status & I2C_INT_MSTARBLOSS)
    {
      i2cerr("ERROR: Arbitation loss\n");

      lpc54_i2c_putreg(priv, LPC54_I2C_STAT_OFFSET, I2C_INT_MSTARBLOSS);
      priv->result = -EIO;
      return true;
    }

  if (status & I2C_INT_MSTSTSTPERR)
    {
      i2cerr("ERROR: Start/stop error\n");

      lpc54_i2c_putreg(priv, LPC54_I2C_STAT_OFFSET, I2C_INT_MSTSTSTPERR);
      priv->result = -EIO;
      return true;
    }

  if ((status & I2C_INT_MSTPENDING) == 0)
    {
      i2cerr("ERROR: Busy\n");

      priv->result = -EBUSY;
      return true;
    }

  /* Get the state of the I2C module */

  mstate = (status & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT;
  i2cinfo("mstate=%" PRIu32 "\n", mstate);

  if ((mstate == I2C_MASTER_STATE_ADDRNAK) ||
      (mstate == I2C_MASTER_STATE_DATANAK))
    {
      /* Slave NACKed last byte, issue stop and return error */

      lpc54_i2c_putreg(priv, LPC54_I2C_MSTCTL_OFFSET, I2C_MSTCTL_MSTSTOP);
      priv->result = -EPERM;
      priv->state  = I2CSTATE_WAITSTOP;

      i2cerr("ERROR:  NAKed, state=%u\n", priv->state);
      return false;
    }

  switch (priv->state)
    {
      case I2CSTATE_START:
        {
          enum lpc54_i2cstate_e newstate;

          if ((msg->flags & I2C_M_READ) == I2C_M_READ)
            {
              lpc54_i2c_putreg(priv, LPC54_I2C_MSTDAT_OFFSET,
                               I2C_READADDR8(msg->addr));
              newstate = I2CSTATE_RECEIVE;
            }
          else
            {
              lpc54_i2c_putreg(priv, LPC54_I2C_MSTDAT_OFFSET,
                               I2C_WRITEADDR8(msg->addr));
              newstate = I2CSTATE_TRANSMIT;
            }

          if (priv->xfrd >= msg->length)
            {
              /* No more data, setup for STOP */

              newstate = I2CSTATE_STOP;
            }

          priv->state = newstate;

          /* Send START condition */

          lpc54_i2c_putreg(priv, LPC54_I2C_MSTCTL_OFFSET,
                           I2C_MSTCTL_MSTSTART);
        }
        break;

      case I2CSTATE_TRANSMIT:
        {
          if (mstate != I2C_MASTER_STATE_TXOK)
            {
              i2cerr("ERROR bad state=%" PRIu32 ", expected %u\n",
                     mstate, I2C_MASTER_STATE_TXOK);

              priv->result = -EINVAL;
              return true;
            }

          lpc54_i2c_putreg(priv, LPC54_I2C_MSTDAT_OFFSET,
                           msg->buffer[priv->xfrd]);
          lpc54_i2c_putreg(priv, LPC54_I2C_MSTCTL_OFFSET,
                           I2C_MSTCTL_MSTCONTINUE);
          priv->xfrd++;

          if (priv->xfrd >= msg->length)
            {
              /* No more data, schedule stop condition */

              priv->state = I2CSTATE_STOP;
            }
        }
        break;

      case I2CSTATE_RECEIVE:
        {
          if (mstate != I2C_MASTER_STATE_RXAVAIL)
            {
              i2cerr("ERROR bad state=%" PRIu32 ", expected %u\n",
                     mstate, I2C_MASTER_STATE_RXAVAIL);

              priv->result = -EINVAL;
              return true;
            }

          msg->buffer[priv->xfrd] =
            lpc54_i2c_getreg(priv, LPC54_I2C_MSTDAT_OFFSET);

          priv->xfrd++;
          if (priv->xfrd < msg->length)
            {
              lpc54_i2c_putreg(priv, LPC54_I2C_MSTCTL_OFFSET,
                               I2C_MSTCTL_MSTCONTINUE);
            }
          else
            {
              /* No more data expected, issue NACK and STOP right away */

              lpc54_i2c_putreg(priv, LPC54_I2C_MSTCTL_OFFSET,
                               I2C_MSTCTL_MSTSTOP);
              priv->state = I2CSTATE_WAITSTOP;
            }
        }
        break;

      case I2CSTATE_STOP:
        {
          bool dostop = true;

          /* Is this the last message? */

          if (priv->nmsgs > 1)
            {
              struct i2c_msg_s *nextmsg;

              /* No.. Is there a start on the next message?  If so, it
               * should be preceded by a STOP.
               */

              nextmsg = msg + 1;
              dostop  = ((nextmsg->flags & I2C_M_NOSTART) != 0);
            }

          if (dostop)
            {
              /* Stop condition is omitted, we are done. Start the next
               * message (or return to the IDLE state if none).
               */

              return lpc54_i2c_nextmsg(priv);
            }
          else
            {
              /* Send stop condition */

              lpc54_i2c_putreg(priv, LPC54_I2C_MSTCTL_OFFSET,
                               I2C_MSTCTL_MSTSTOP);
              priv->state = I2CSTATE_WAITSTOP;
            }
        }
        break;

      case I2CSTATE_WAITSTOP:
        {
          /* Start the next message (or return to the IDLE state if none). */

          return lpc54_i2c_nextmsg(priv);
        }
        break;

      case I2CSTATE_IDLE:
      default:
        priv->result = -EINVAL;
        return true;
    }

  i2cinfo("state=%u\n", priv->state);
  return false;
}

/****************************************************************************
 * Name: lpc54_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int lpc54_i2c_interrupt(int irq, void *context, void *arg)
{
  struct lpc54_i2cdev_s *priv = (struct lpc54_i2cdev_s *)arg;
  bool done;

  DEBUGASSERT(priv != NULL);

  /* Run the I2C state machine */

  done = lpc54_i2c_statemachine(priv);
  if (done)
    {
      /* Disable further I2C interrupts. */

      lpc54_i2c_putreg(priv, LPC54_I2C_INTENCLR_OFFSET, I2C_MASTER_INTS);

      /* Wake up wake up any waiters */

      nxsem_post(&priv->waitsem);
    }

  return OK;
}
#endif /* CONFIG_I2C_POLLED */

/****************************************************************************
 * Name: lpc54_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int lpc54_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct lpc54_i2cdev_s *priv = (struct lpc54_i2cdev_s *)dev;
  int ret;

  i2cinfo("count=%d\n", count);
  DEBUGASSERT(dev != NULL && msgs != NULL);

  /* Get exclusive access to the I2C bus */

  nxmutex_lock(&priv->lock);

  /* Set up for the transfer */

  priv->xfrd   = 0;
  priv->msgs   = msgs;
  priv->nmsgs  = count;
  priv->result = OK;

  /* Set up the transfer timeout */

  wd_start(&priv->timeout, priv->nmsgs * I2C_WDOG_TIMEOUT,
           lpc54_i2c_timeout, (wdparm_t)priv);

  /* Initiate the transfer */

  lpc54_i2c_xfrsetup(priv);

  /* Loop until the transfer is complete or until a timeout occurs */

  do
    {
#ifndef CONFIG_I2C_POLLED
       nxsem_wait(&priv->waitsem);
#else
       lpc54_i2c_statemachine(priv);
#endif
    }
  while (priv->state != I2CSTATE_IDLE);

  ret = priv->result;
  i2cinfo("Done, result=%d\n", ret);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: lpc54_i2c_reset
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
static int lpc54_i2c_reset(struct i2c_master_s *dev)
{
#warning Missing logic
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *lpc54_i2cbus_initialize(int port)
{
  struct lpc54_i2cdev_s *priv;
  irqstate_t flags;
  uint32_t regval;

  i2cinfo("port=%d\n", port);

  flags = enter_critical_section();

  /* Configure the requestin I2C peripheral */

  /* NOTE:  The basic FLEXCOMM initialization was performed in
   * lpc54_lowputc.c.
   */

#ifdef CONFIG_LPC54_I2C0_MASTER
  if (port == 0)
    {
      /* Attach 12 MHz clock to FLEXCOMM0 */

      lpc54_flexcomm0_enableclk();

      /* Set FLEXCOMM0 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM0_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c0_dev;
      priv->base     = LPC54_FLEXCOMM0_BASE;
      priv->fclock   = BOARD_FLEXCOMM0_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM0;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C0_SCL);
      lpc54_gpio_config(GPIO_I2C0_SDA);

      /* Set up the FLEXCOMM0 function clock */

      putreg32(BOARD_FLEXCOMM0_CLKSEL, LPC54_SYSCON_FCLKSEL0);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C1_MASTER
  if (port == 1)
    {
      /* Attach 12 MHz clock to FLEXCOMM1 */

      lpc54_flexcomm1_enableclk();

      /* Set FLEXCOMM1 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM1_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c1_dev;
      priv->base     = LPC54_FLEXCOMM1_BASE;
      priv->fclock   = BOARD_FLEXCOMM1_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM1;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C1_SCL);
      lpc54_gpio_config(GPIO_I2C1_SDA);

      /* Set up the FLEXCOMM1 function clock */

      putreg32(BOARD_FLEXCOMM1_CLKSEL, LPC54_SYSCON_FCLKSEL1);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C2_MASTER
  if (port == 2)
    {
      /* Attach 12 MHz clock to FLEXCOMM2 */

      lpc54_flexcomm2_enableclk();

      /* Set FLEXCOMM2 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM2_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c2_dev;
      priv->base     = LPC54_FLEXCOMM2_BASE;
      priv->fclock   = BOARD_FLEXCOMM2_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM2;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C2_SCL);
      lpc54_gpio_config(GPIO_I2C2_SDA);

      /* Set up the FLEXCOMM2 function clock */

      putreg32(BOARD_FLEXCOMM2_CLKSEL, LPC54_SYSCON_FCLKSEL2);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C3_MASTER
  if (port == 3)
    {
      /* Attach 12 MHz clock to FLEXCOMM3 */

      lpc54_flexcomm3_enableclk();

      /* Set FLEXCOMM3 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM3_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c3_dev;
      priv->base     = LPC54_FLEXCOMM3_BASE;
      priv->fclock   = BOARD_FLEXCOMM3_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM3;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C3_SCL);
      lpc54_gpio_config(GPIO_I2C3_SDA);

      /* Set up the FLEXCOMM3 function clock */

      putreg32(BOARD_FLEXCOMM3_CLKSEL, LPC54_SYSCON_FCLKSEL3);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C4_MASTER
  if (port == 4)
    {
      /* Attach 12 MHz clock to FLEXCOMM4 */

      lpc54_flexcomm4_enableclk();

      /* Set FLEXCOMM4 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM4_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c4_dev;
      priv->base     = LPC54_FLEXCOMM4_BASE;
      priv->fclock   = BOARD_FLEXCOMM4_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM4;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C4_SCL);
      lpc54_gpio_config(GPIO_I2C4_SDA);

      /* Set up the FLEXCOMM4 function clock */

      putreg32(BOARD_FLEXCOMM4_CLKSEL, LPC54_SYSCON_FCLKSEL4);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C5_MASTER
  if (port == 5)
    {
      /* Attach 12 MHz clock to FLEXCOMM5 */

      lpc54_flexcomm5_enableclk();

      /* Set FLEXCOMM5 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM5_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c5_dev;
      priv->base     = LPC54_FLEXCOMM5_BASE;
      priv->fclock   = BOARD_FLEXCOMM5_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM5;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C5_SCL);
      lpc54_gpio_config(GPIO_I2C5_SDA);

      /* Set up the FLEXCOMM5 function clock */

      putreg32(BOARD_FLEXCOMM5_CLKSEL, LPC54_SYSCON_FCLKSEL5);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C6_MASTER
  if (port == 6)
    {
      /* Attach 12 MHz clock to FLEXCOMM6 */

      lpc54_flexcomm6_enableclk();

      /* Set FLEXCOMM6 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM6_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c6_dev;
      priv->base     = LPC54_FLEXCOMM6_BASE;
      priv->fclock   = BOARD_FLEXCOMM6_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM6;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C6_SCL);
      lpc54_gpio_config(GPIO_I2C6_SDA);

      /* Set up the FLEXCOMM6 function clock */

      putreg32(BOARD_FLEXCOMM6_CLKSEL, LPC54_SYSCON_FCLKSEL6);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C7_MASTER
  if (port == 7)
    {
      /* Attach 12 MHz clock to FLEXCOMM7 */

      lpc54_flexcomm7_enableclk();

      /* Set FLEXCOMM7 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM7_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c7_dev;
      priv->base     = LPC54_FLEXCOMM7_BASE;
      priv->fclock   = BOARD_FLEXCOMM7_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM7;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C7_SCL);
      lpc54_gpio_config(GPIO_I2C7_SDA);

      /* Set up the FLEXCOMM7 function clock */

      putreg32(BOARD_FLEXCOMM7_CLKSEL, LPC54_SYSCON_FCLKSEL7);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C8_MASTER
  if (port == 8)
    {
      /* Attach 12 MHz clock to FLEXCOMM8 */

      lpc54_flexcomm8_enableclk();

      /* Set FLEXCOMM8 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM8_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c8_dev;
      priv->base     = LPC54_FLEXCOMM8_BASE;
      priv->fclock   = BOARD_FLEXCOMM8_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM8;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C8_SCL);
      lpc54_gpio_config(GPIO_I2C8_SDA);

      /* Set up the FLEXCOMM8 function clock */

      putreg32(BOARD_FLEXCOMM8_CLKSEL, LPC54_SYSCON_FCLKSEL8);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C9_MASTER
  if (port == 9)
    {
      /* Attach 12 MHz clock to FLEXCOMM9 */

      lpc54_flexcomm9_enableclk();

      /* Set FLEXCOMM9 to the I2C peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_I2C | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM9_PSELID);

      /* Initialize the state structure */

      priv           = &g_i2c9_dev;
      priv->base     = LPC54_FLEXCOMM9_BASE;
      priv->fclock   = BOARD_FLEXCOMM9_FCLK;
#ifndef CONFIG_I2C_POLLED
      priv->irq      = LPC54_IRQ_FLEXCOMM9;
#endif

      /* Configure I2C pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C9_SCL);
      lpc54_gpio_config(GPIO_I2C9_SDA);

      /* Set up the FLEXCOMM9 function clock */

      putreg32(BOARD_FLEXCOMM9_CLKSEL, LPC54_SYSCON_FCLKSEL9);
    }
  else
#endif
    {
      i2cerr("ERROR: Unsupported port=%d\n", port);
      leave_critical_section(flags);
      return NULL;
    }

  leave_critical_section(flags);

  /* Install our operations */

  priv->dev.ops = &lpc54_i2c_ops;

  /* Enable the I2C peripheral in the master  mode */

  regval  = lpc54_i2c_getreg(priv, LPC54_I2C_CFG_OFFSET);
  regval &= I2C_CFG_ALLENABLES;
  regval |= I2C_CFG_MSTEN;
  lpc54_i2c_putreg(priv, LPC54_I2C_CFG_OFFSET, regval);

  /* Set the default I2C frequency */

  lpc54_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

#ifndef CONFIG_I2C_POLLED
  /* Attach Interrupt Handler */

  irq_attach(priv->irq, lpc54_i2c_interrupt, priv);
#endif

  /* Disable interrupts at the I2C peripheral */

  lpc54_i2c_putreg(priv, LPC54_I2C_INTENCLR_OFFSET, I2C_INT_ALL);

#ifndef CONFIG_I2C_POLLED
  /* Enable interrupts at the NVIC */

  up_enable_irq(priv->irq);
#endif
  return &priv->dev;
}

/****************************************************************************
 * Name: lpc54_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int lpc54_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct lpc54_i2cdev_s *priv = (struct lpc54_i2cdev_s *)dev;
  uint32_t regval;

  /* Disable I2C interrupts */

  lpc54_i2c_putreg(priv, LPC54_I2C_INTENCLR_OFFSET, I2C_MASTER_INTS);

  /* Disable the I2C peripheral */

  regval  = lpc54_i2c_getreg(priv, LPC54_I2C_CFG_OFFSET);
  regval &= I2C_CFG_ALLENABLES;
  regval &= ~I2C_CFG_MSTEN;
  lpc54_i2c_putreg(priv, LPC54_I2C_CFG_OFFSET, regval);

#ifndef CONFIG_I2C_POLLED
  /* Disable the Flexcomm interface at the NVIC and detach the interrupt. */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
#endif

  return OK;
}

#endif /* HAVE_I2C_MASTER_DEVICE */
