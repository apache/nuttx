/****************************************************************************
 * arch/arm/src/rm57/rm57_i2c.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* See rm57_i2c.h for what this driver covers.  The register map is in
 * hardware/rm57_i2c.h; the initialization order follows SPNU562A section
 * 30.5.3 and HALCoGen's i2cInit().
 *
 * A transfer is cut into segments, each of which is one START (or repeated
 * START) on the bus: a message plus any following messages flagged
 * I2C_M_NOSTART.  The module counts the bytes of a segment in I2CCNT and,
 * when STP is set with the START, ends it with a STOP on its own - the
 * last byte of a read is NACKed by the same mechanism (Table 30-1).  A
 * segment that is followed by another one leaves STP clear instead: once
 * the count passes zero the module raises ARDY and holds the bus, and the
 * next segment's STT becomes a repeated START (Table 30-16, "Repeat n").
 * ARDY is the only end-of-segment event used.  On the board no TXRDY
 * interrupt followed the copy of the last byte once the count had run
 * out, so a write that also waited for one never finished.  ARDY outranks
 * RXRDY in I2CIVR, so a read finishes only once the last byte has been
 * read as well, whichever of the two is serviced first.
 *
 * I2CCNT = 0 means 65536 bytes, so an address-only probe cannot be
 * expressed with a count.  It is sent in repeat mode instead: ARDY follows
 * the address ACK (Table 30-7) and STP is set from the interrupt.
 *
 * Before every new START the driver waits for both BB and MST to clear.
 * MST drops after BB at the end of a STOP, and a START issued in between is
 * silently lost (SPNU562A section 30.3.1).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_i2c.h"
#include "hardware/rm57l843_memorymap.h"
#include "rm57_i2c.h"

#ifdef CONFIG_RM57_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Module clock = VCLK / (PSC + 1) must lie between 6.7 and 13.3 MHz
 * (SPNU562A section 30.1.3).  Aim for 8 MHz as HALCoGen does: 75 MHz VCLK
 * gives PSC = 8 and 8.33 MHz.
 */

#define RM57_I2C_PSC            (BOARD_VCLK_FREQUENCY / 8000000 - 1)
#define RM57_I2C_MODCLK         (BOARD_VCLK_FREQUENCY / (RM57_I2C_PSC + 1))

#if RM57_I2C_MODCLK < 6700000 || RM57_I2C_MODCLK > 13300000
#  error "I2C module clock out of range for this VCLK"
#endif

/* The clock divider adds d module clocks to both CLKL and CLKH, where d
 * depends on the prescaler (SPNU562A Equation 62).
 */

#if RM57_I2C_PSC == 0
#  define RM57_I2C_D            7
#elif RM57_I2C_PSC == 1
#  define RM57_I2C_D            6
#else
#  define RM57_I2C_D            5
#endif

/* Supported SCL range (SPNS215C section 7.10.1) */

#define RM57_I2C_MINFREQ        10000
#define RM57_I2C_MAXFREQ        400000
#define RM57_I2C_DEFFREQ        100000

/* Above standard mode the low phase has to be about twice the high phase
 * to meet tLOW >= 1.3 us at 400 kHz (SPNS215C Table 7-28).
 */

#define RM57_I2C_STDFREQ        100000

/* Interrupt codes handled per interrupt before giving up */

#define RM57_I2C_MAXCODES       16

/* Bus recovery: SCL half period and the clock pulses that free any slave
 * caught in the middle of a byte.
 */

#define RM57_I2C_RESET_US       5
#define RM57_I2C_RESET_PULSES   9

#define RM57_I2C_MDR_IDLE       (I2C_MDR_NIRS | I2C_MDR_FREE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rm57_i2c_s
{
  struct i2c_master_s dev;      /* Externally visible part; must be first */
  uint32_t         base;        /* Module register base */
  uint8_t          irq;         /* Interrupt line */
  uint8_t          bus;         /* Module number, 1-2 */
  uint8_t          refs;        /* rm57_i2cbus_initialize() references */
  bool             held;        /* Last transfer kept the bus (NOSTOP) */
  uint32_t         frequency;   /* Requested SCL frequency */
  uint32_t         actual;      /* SCL frequency in effect */
  mutex_t          lock;        /* Serializes transfers */
  sem_t            done;        /* Posted when a segment completes */

  /* Segment state, owned by the interrupt handler while a segment runs */

  struct i2c_msg_s *msg;        /* Message being transferred */
  struct i2c_msg_s *msgend;     /* One past the last message */
  uint8_t         *ptr;         /* Next byte in the current message */
  size_t           left;        /* Bytes left in the current message */
  uint32_t         total;       /* Bytes left in the segment */
  bool             write;       /* Master transmitter */
  bool             stop;        /* Segment ends with a STOP */
  bool             probe;       /* Address only, in repeat mode */
  bool             acked;       /* The slave acknowledged its address */
  bool             ardy;        /* Read: count ran out (ARDY seen) */
  uint8_t          ntxrdy;      /* TXRDY events in this segment */
  volatile int     result;      /* Outcome of the segment */

#ifdef CONFIG_RM57_I2C_LOOPBACK
  uint8_t         *echo;        /* Looped-back bytes; NULL = no loopback */
  size_t           echolen;
  size_t           echoidx;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t i2c_getreg(struct rm57_i2c_s *priv,
                                  unsigned int offset);
static inline void i2c_putreg(struct rm57_i2c_s *priv,
                              unsigned int offset, uint32_t value);
static inline void i2c_modifyreg(struct rm57_i2c_s *priv,
                                 unsigned int offset, uint32_t clearbits,
                                 uint32_t setbits);
static inline bool i2c_loopback(struct rm57_i2c_s *priv);

static void i2c_setclock(struct rm57_i2c_s *priv, uint32_t frequency);
static void i2c_drainivr(struct rm57_i2c_s *priv);
static void i2c_hwreset(struct rm57_i2c_s *priv);
static void i2c_finish(struct rm57_i2c_s *priv);
static void i2c_echo(struct rm57_i2c_s *priv);
static uint8_t *i2c_bufptr(struct rm57_i2c_s *priv);
static int  i2c_interrupt(int irq, void *context, void *arg);
static int  i2c_waitidle(struct rm57_i2c_s *priv);
static int  i2c_segment(struct rm57_i2c_s *priv, struct i2c_msg_s *msgs,
                        int nmsgs, uint32_t total, bool stop);

static int  i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs,
                         int count);
#ifdef CONFIG_I2C_RESET
static int  i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2cops =
{
  .transfer = i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset    = i2c_reset,
#endif
};

#define I2C_DEVICE(n, irqno) \
  static struct rm57_i2c_s g_i2c##n##dev = \
  { \
    .dev.ops   = &g_i2cops, \
    .base      = RM57_I2C##n##_BASE, \
    .irq       = irqno, \
    .bus       = n, \
    .frequency = RM57_I2C_DEFFREQ, \
    .lock      = NXMUTEX_INITIALIZER, \
    .done      = NXSEM_INITIALIZER(0, SEM_PRIO_NONE), \
  }

#ifdef CONFIG_RM57_I2C1
I2C_DEVICE(1, RM57_REQ_I2C);
#endif

#ifdef CONFIG_RM57_I2C2
I2C_DEVICE(2, RM57_REQ_I2C2);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_getreg, i2c_putreg and i2c_modifyreg
 ****************************************************************************/

static inline uint32_t i2c_getreg(struct rm57_i2c_s *priv,
                                  unsigned int offset)
{
  return getreg32(priv->base + offset);
}

static inline void i2c_putreg(struct rm57_i2c_s *priv,
                              unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static inline void i2c_modifyreg(struct rm57_i2c_s *priv,
                                 unsigned int offset, uint32_t clearbits,
                                 uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: i2c_loopback
 ****************************************************************************/

static inline bool i2c_loopback(struct rm57_i2c_s *priv)
{
#ifdef CONFIG_RM57_I2C_LOOPBACK
  return priv->echo != NULL;
#else
  return false;
#endif
}

/****************************************************************************
 * Name: i2c_setclock
 *
 * Description:
 *   Program the prescaler and the SCL dividers.  The module must be in
 *   reset (nIRS = 0): all three registers only take effect on the rising
 *   edge of nIRS (SPNU562A sections 30.6.4, 30.6.5 and 30.6.13).
 *
 ****************************************************************************/

static void i2c_setclock(struct rm57_i2c_s *priv, uint32_t frequency)
{
  uint32_t div;
  uint32_t low;

  if (frequency < RM57_I2C_MINFREQ)
    {
      frequency = RM57_I2C_MINFREQ;
    }
  else if (frequency > RM57_I2C_MAXFREQ)
    {
      frequency = RM57_I2C_MAXFREQ;
    }

  /* Module clocks per SCL period, rounded up so that SCL never runs faster
   * than requested.
   */

  div = (RM57_I2C_MODCLK + frequency - 1) / frequency;

  if (frequency > RM57_I2C_STDFREQ)
    {
      low = (2 * div + 2) / 3;
    }
  else
    {
      low = (div + 1) / 2;
    }

  i2c_putreg(priv, RM57_I2C_PSC_OFFSET, RM57_I2C_PSC);
  i2c_putreg(priv, RM57_I2C_CKL_OFFSET, low - RM57_I2C_D);
  i2c_putreg(priv, RM57_I2C_CKH_OFFSET, div - low - RM57_I2C_D);

  priv->actual = RM57_I2C_MODCLK / div;
  i2cinfo("I2C%d: %" PRIu32 " Hz requested, %" PRIu32 " Hz set"
          " (CKL %" PRIu32 " CKH %" PRIu32 ")\n",
          priv->bus, frequency, priv->actual,
          low - RM57_I2C_D, div - low - RM57_I2C_D);
}

/****************************************************************************
 * Name: i2c_drainivr
 *
 * Description:
 *   Read I2CIVR until it reports nothing pending.  A stale code left in it
 *   would otherwise be returned for the next START (SPNU562A section
 *   30.6.11).  ARDY is not cleared by the read, so the status flags are
 *   cleared first.
 *
 ****************************************************************************/

static void i2c_drainivr(struct rm57_i2c_s *priv)
{
  int i;

  i2c_putreg(priv, RM57_I2C_STR_OFFSET, I2C_STR_W1C);

  for (i = 0; i < RM57_I2C_MAXCODES; i++)
    {
      if ((i2c_getreg(priv, RM57_I2C_IVR_OFFSET) &
           I2C_IVR_INTCODE_MASK) == I2C_IVR_NONE)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: i2c_hwreset
 *
 * Description:
 *   Put the module through reset and bring it back up as an idle master
 *   at the requested frequency.  Resetting releases both pins, so it also
 *   abandons whatever was in progress on the bus.
 *
 ****************************************************************************/

static void i2c_hwreset(struct rm57_i2c_s *priv)
{
  uint32_t oar = 0;

  i2c_putreg(priv, RM57_I2C_MDR_OFFSET, 0);

  i2c_putreg(priv, RM57_I2C_IMR_OFFSET, 0);
  i2c_putreg(priv, RM57_I2C_EMDR_OFFSET, 0);
  i2c_putreg(priv, RM57_I2C_DMACR_OFFSET, 0);
  i2c_setclock(priv, priv->frequency);

#ifdef CONFIG_RM57_I2C_LOOPBACK
  if (priv->echo != NULL)
    {
      oar = RM57_I2C_LOOPBACK_ADDR;
    }
#endif

  i2c_putreg(priv, RM57_I2C_OAR_OFFSET, oar);

  /* Both pins on the I2C function, inputs when idle, with the weak
   * internal pull-ups on as HALCoGen does.  PDIS resets to "pulls off".
   */

  i2c_putreg(priv, RM57_I2C_PFNC_OFFSET, 0);
  i2c_putreg(priv, RM57_I2C_PDIR_OFFSET, 0);
  i2c_putreg(priv, RM57_I2C_PSEL_OFFSET, I2C_PIN_SCL | I2C_PIN_SDA);
  i2c_putreg(priv, RM57_I2C_PDIS_OFFSET, 0);

  i2c_putreg(priv, RM57_I2C_MDR_OFFSET, RM57_I2C_MDR_IDLE);
  i2c_drainivr(priv);
  priv->held = false;
}

/****************************************************************************
 * Name: i2c_finish
 *
 * Description:
 *   End the segment from the interrupt handler.  All sources are masked
 *   first, so a late event cannot post the semaphore a second time.
 *
 ****************************************************************************/

static void i2c_finish(struct rm57_i2c_s *priv)
{
  i2c_putreg(priv, RM57_I2C_IMR_OFFSET, 0);
  nxsem_post(&priv->done);
}

/****************************************************************************
 * Name: i2c_echo
 *
 * Description:
 *   In loopback, store a byte waiting in I2CDRR as echo.  Called where a
 *   write segment ends or a new one starts, because RXRDY ranks below ARDY
 *   in I2CIVR and would otherwise be dropped by the end of the segment.
 *
 ****************************************************************************/

static void i2c_echo(struct rm57_i2c_s *priv)
{
#ifdef CONFIG_RM57_I2C_LOOPBACK
  if (priv->echo != NULL &&
      (i2c_getreg(priv, RM57_I2C_STR_OFFSET) & I2C_INT_RXRDY) != 0)
    {
      uint8_t value = (uint8_t)i2c_getreg(priv, RM57_I2C_DRR_OFFSET);

      if (priv->echoidx < priv->echolen)
        {
          priv->echo[priv->echoidx++] = value;
        }
    }
#endif
}

/****************************************************************************
 * Name: i2c_bufptr
 *
 * Description:
 *   Return where the next byte of the segment goes or comes from, moving on
 *   to the next message of the segment when the current one is used up.
 *   Only called while priv->total is non-zero.
 *
 ****************************************************************************/

static uint8_t *i2c_bufptr(struct rm57_i2c_s *priv)
{
  while (priv->left == 0 && priv->msg + 1 < priv->msgend)
    {
      priv->msg++;
      priv->ptr  = priv->msg->buffer;
      priv->left = priv->msg->length;
    }

  priv->left--;
  priv->total--;
  return priv->ptr++;
}

/****************************************************************************
 * Name: i2c_interrupt
 *
 * Description:
 *   Service every pending source, highest priority first, as reported by
 *   I2CIVR.  Reading it clears AL, NACK and SCD; ARDY has to be cleared in
 *   I2CSTR; TXRDY and RXRDY clear when I2CDXR is written and I2CDRR is
 *   read.
 *
 ****************************************************************************/

static int i2c_interrupt(int irq, void *context, void *arg)
{
  struct rm57_i2c_s *priv = arg;
  uint32_t code = I2C_IVR_NONE;
  uint32_t value;
  int i;

  for (i = 0; i < RM57_I2C_MAXCODES; i++)
    {
      code = i2c_getreg(priv, RM57_I2C_IVR_OFFSET) & I2C_IVR_INTCODE_MASK;
      switch (code)
        {
          case I2C_IVR_NONE:
            return OK;

          case I2C_IVR_AL:

            /* The module has already dropped to slave receiver and
             * cleared MST, STT and STP; there is no STOP to wait for.
             */

            priv->result = -EAGAIN;
            i2c_finish(priv);
            return OK;

          case I2C_IVR_NACK:

            /* A NACK clears STP and leaves the bus held.  Release it and
             * finish on the STOP.
             */

            if (priv->result == OK)
              {
                priv->result = priv->acked ? -EIO : -ENXIO;
              }

            i2c_putreg(priv, RM57_I2C_IMR_OFFSET, I2C_INT_AL | I2C_INT_SCD);
            i2c_modifyreg(priv, RM57_I2C_MDR_OFFSET, 0, I2C_MDR_STP);
            break;

          case I2C_IVR_ARDY:
            i2c_putreg(priv, RM57_I2C_STR_OFFSET, I2C_INT_ARDY);
            if (priv->probe)
              {
                priv->acked = true;
                i2c_modifyreg(priv, RM57_I2C_MDR_OFFSET, 0, I2C_MDR_STP);
              }
            else if (!priv->stop && !priv->write)
              {
                /* Count ran out with STP clear.  The last RXRDY may still
                 * be pending behind this higher-priority code.
                 */

                priv->ardy = true;
                if (priv->total == 0)
                  {
                    i2c_finish(priv);
                    return OK;
                  }
              }
            else if (!priv->stop && priv->total == 0)
              {
                /* Write count passed zero: the bus is held for the
                 * repeated START of the next segment.  In loopback the
                 * last byte's echo is pending behind this code.
                 */

                i2c_echo(priv);
                i2c_finish(priv);
                return OK;
              }
            break;

          case I2C_IVR_RXRDY:
            value = i2c_getreg(priv, RM57_I2C_DRR_OFFSET);
            if (!priv->write)
              {
                priv->acked = true;
                if (priv->total > 0)
                  {
                    *i2c_bufptr(priv) = (uint8_t)value;
                  }

                if (!priv->stop && priv->ardy && priv->total == 0)
                  {
                    i2c_finish(priv);
                    return OK;
                  }
              }
#ifdef CONFIG_RM57_I2C_LOOPBACK
            else if (priv->echo != NULL && priv->echoidx < priv->echolen)
              {
                priv->echo[priv->echoidx++] = (uint8_t)value;
              }
#endif
            break;

          case I2C_IVR_TXRDY:

            /* The first TXRDY of a segment only says I2CDXR is free for
             * the first byte; any later one means a byte moved to the
             * shift register, which needs the address to have been ACKed.
             */

            if (++priv->ntxrdy > 1)
              {
                priv->acked = true;
              }

            if (priv->total > 0)
              {
                i2c_putreg(priv, RM57_I2C_DXR_OFFSET, *i2c_bufptr(priv));
              }
            else
              {
                i2c_modifyreg(priv, RM57_I2C_IMR_OFFSET, I2C_INT_TXRDY, 0);
              }
            break;

          case I2C_IVR_SCD:
            if (priv->result == OK && priv->total > 0)
              {
                priv->result = -EIO;
              }

            i2c_finish(priv);
            return OK;

          default:
            break;
        }
    }

  i2cerr("ERROR: I2C%d: interrupt storm, code %" PRIu32 "\n",
         priv->bus, code);
  return OK;
}

/****************************************************************************
 * Name: i2c_waitidle
 *
 * Description:
 *   Wait until the bus is free and the module has left master mode, so
 *   that a new START will be issued.
 *
 ****************************************************************************/

static int i2c_waitidle(struct rm57_i2c_s *priv)
{
  clock_t start = clock_systime_ticks();
  int spin;

  for (spin = 0; ; spin++)
    {
      if ((i2c_getreg(priv, RM57_I2C_STR_OFFSET) & I2C_STR_BB) == 0 &&
          (i2c_getreg(priv, RM57_I2C_MDR_OFFSET) & I2C_MDR_MST) == 0)
        {
          return OK;
        }

      if (clock_systime_ticks() - start >
          MSEC2TICK(CONFIG_RM57_I2C_TIMEOUT_MS))
        {
          i2cerr("ERROR: I2C%d: bus busy, STR %04" PRIx32
                 " MDR %04" PRIx32 "\n", priv->bus,
                 i2c_getreg(priv, RM57_I2C_STR_OFFSET),
                 i2c_getreg(priv, RM57_I2C_MDR_OFFSET));
          return -EBUSY;
        }

      /* MST normally follows BB within a few module clocks; only a bus
       * that stays busy is worth sleeping on.
       */

      if (spin < 100)
        {
          up_udelay(1);
        }
      else
        {
          nxsched_usleep(1000);
        }
    }
}

/****************************************************************************
 * Name: i2c_segment
 *
 * Description:
 *   Run one segment - a START or repeated START, the address and the bytes
 *   of nmsgs messages in one direction - and wait for it to complete.
 *
 * Input Parameters:
 *   priv  - Module, with the bus free or held by the previous segment
 *   msgs  - First message of the segment
 *   nmsgs - Messages in the segment
 *   total - Bytes in all of them; zero for an address-only probe
 *   stop  - End the segment with a STOP
 *
 * Returned Value:
 *   Zero on success; -ENXIO if the address was not acknowledged, -EIO if
 *   data was not, -EAGAIN on lost arbitration, -ETIMEDOUT if the segment
 *   did not complete (the module has then been reset).
 *
 ****************************************************************************/

static int i2c_segment(struct rm57_i2c_s *priv, struct i2c_msg_s *msgs,
                       int nmsgs, uint32_t total, bool stop)
{
  uint64_t nominal;
  uint32_t mdr;
  uint32_t imr;
  uint16_t addr;
  int ret;

  priv->msg        = msgs;
  priv->msgend     = msgs + nmsgs;
  priv->ptr        = msgs->buffer;
  priv->left       = msgs->length;
  priv->total      = total;
  priv->write      = (msgs->flags & I2C_M_READ) == 0;
  priv->probe      = total == 0;
  priv->stop       = stop || priv->probe;
  priv->acked      = false;
  priv->ardy       = false;
  priv->ntxrdy     = 0;
  priv->result     = OK;

  addr = msgs->addr;
  mdr  = RM57_I2C_MDR_IDLE | I2C_MDR_MST | I2C_MDR_STT;
  imr  = I2C_INT_AL | I2C_INT_NACK | I2C_INT_SCD;

  if ((msgs->flags & I2C_M_TEN) != 0)
    {
      mdr |= I2C_MDR_XA;
    }

  if (priv->write)
    {
      mdr |= I2C_MDR_TRX;
    }

  if (priv->probe)
    {
      mdr |= I2C_MDR_RM;
      imr |= I2C_INT_ARDY;
    }
  else
    {
      imr |= priv->write ? I2C_INT_TXRDY : I2C_INT_RXRDY;
      if (priv->stop)
        {
          mdr |= I2C_MDR_STP;
        }
      else
        {
          imr |= I2C_INT_ARDY;
        }
    }

  if (i2c_loopback(priv))
    {
      addr = RM57_I2C_LOOPBACK_ADDR;
      mdr |= I2C_MDR_DLB;
      if (priv->write)
        {
          imr |= I2C_INT_RXRDY;
        }
    }

  /* drainivr clears RXRDY: collect a loopback echo still in I2CDRR */

  i2c_echo(priv);
  i2c_drainivr(priv);
  i2c_putreg(priv, RM57_I2C_SAR_OFFSET, addr & I2C_ADDR_MASK);
  if (!priv->probe)
    {
      i2c_putreg(priv, RM57_I2C_CNT_OFFSET, total);
    }

  i2c_putreg(priv, RM57_I2C_MDR_OFFSET, mdr);
  i2c_putreg(priv, RM57_I2C_IMR_OFFSET, imr);

  /* Four times the nominal duration, address and stop included, plus the
   * configured slack.
   */

  nominal = ((uint64_t)total + 2) * 9 * 1000000 / priv->actual;
  ret = nxsem_tickwait_uninterruptible(&priv->done,
          USEC2TICK(nominal * 4) +
          MSEC2TICK(CONFIG_RM57_I2C_TIMEOUT_MS) + 1);
  if (ret < 0)
    {
      i2cerr("ERROR: I2C%d: segment timed out, %" PRIu32 " of %" PRIu32
             " bytes left, STR %04" PRIx32 " MDR %04" PRIx32 "\n",
             priv->bus, priv->total, total,
             i2c_getreg(priv, RM57_I2C_STR_OFFSET),
             i2c_getreg(priv, RM57_I2C_MDR_OFFSET));

      i2c_putreg(priv, RM57_I2C_IMR_OFFSET, 0);
      nxsem_reset(&priv->done, 0);
      i2c_hwreset(priv);
      return -ETIMEDOUT;
    }

  return priv->result;
}

/****************************************************************************
 * Name: i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs,
                        int count)
{
  struct rm57_i2c_s *priv = (struct rm57_i2c_s *)dev;
  uint32_t total;
  bool fresh;
  bool stop;
  int first;
  int last;
  int ret;

  DEBUGASSERT(priv != NULL && msgs != NULL && count > 0);

  /* A transfer that would need a fresh START in the middle of a NOSTART
   * run, or a byte count the module cannot express, is rejected before
   * anything reaches the bus.
   */

  for (first = 0; first < count; first = last)
    {
      total = msgs[first].length;
      for (last = first + 1;
           last < count && (msgs[last].flags & I2C_M_NOSTART) != 0;
           last++)
        {
          if (((msgs[last].flags ^ msgs[first].flags) & I2C_M_READ) != 0 ||
              msgs[last].addr != msgs[first].addr)
            {
              return -EINVAL;
            }

          total += msgs[last].length;
        }

      if (total > I2C_CNT_MAX ||
          (total == 0 && (msgs[first].flags & I2C_M_READ) != 0))
        {
          return -EINVAL;
        }
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* The frequency of the first message applies to the whole transfer; it
   * can only change while the module is in reset, so never across a
   * repeated START.
   */

  if (!priv->held && msgs[0].frequency != 0 &&
      msgs[0].frequency != priv->frequency)
    {
      priv->frequency = msgs[0].frequency;
      i2c_hwreset(priv);
    }

  fresh = !priv->held;
  for (first = 0; first < count; first = last)
    {
      total = msgs[first].length;
      for (last = first + 1;
           last < count && (msgs[last].flags & I2C_M_NOSTART) != 0;
           last++)
        {
          total += msgs[last].length;
        }

      stop = last == count && (msgs[last - 1].flags & I2C_M_NOSTOP) == 0;

      if (fresh)
        {
          ret = i2c_waitidle(priv);
          if (ret < 0)
            {
              break;
            }
        }

      ret = i2c_segment(priv, &msgs[first], last - first, total, stop);
      if (ret < 0)
        {
          i2cinfo("I2C%d: addr %02x: %d\n", priv->bus, msgs[first].addr,
                  ret);
          break;
        }

      fresh = stop || total == 0;
    }

  /* After a failure the module has either sent a STOP or been reset; only
   * a transfer that ended on I2C_M_NOSTOP keeps the bus.
   */

  priv->held = ret >= 0 && !fresh;
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: i2c_reset
 *
 * Description:
 *   Recover a bus that a slave is holding: switch both pins to GIO, clock
 *   SCL until the slave lets go of SDA, send a STOP and give the pins back
 *   to the module.  Open drain is emulated through the direction register
 *   - a released line is an input, a low one an output driving 0 - so the
 *   ambiguous polarity of I2CPDR does not matter.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int i2c_reset(struct i2c_master_s *dev)
{
  struct rm57_i2c_s *priv = (struct rm57_i2c_s *)dev;
  uint32_t din;
  int ret;
  int i;
  int n;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  i2c_putreg(priv, RM57_I2C_IMR_OFFSET, 0);
  i2c_putreg(priv, RM57_I2C_MDR_OFFSET, 0);

  i2c_putreg(priv, RM57_I2C_PDIR_OFFSET, 0);
  i2c_putreg(priv, RM57_I2C_DOUT_OFFSET, 0);
  i2c_putreg(priv, RM57_I2C_PFNC_OFFSET, I2C_PFNC_GIO);
  up_udelay(RM57_I2C_RESET_US);

  for (i = 0; i < RM57_I2C_RESET_PULSES; i++)
    {
      if ((i2c_getreg(priv, RM57_I2C_DIN_OFFSET) & I2C_PIN_SDA) != 0)
        {
          break;
        }

      i2c_modifyreg(priv, RM57_I2C_PDIR_OFFSET, 0, I2C_PIN_SCL);
      up_udelay(RM57_I2C_RESET_US);
      i2c_modifyreg(priv, RM57_I2C_PDIR_OFFSET, I2C_PIN_SCL, 0);

      /* Let a slave stretch the clock, but not forever */

      for (n = 0; n < 1000; n++)
        {
          if ((i2c_getreg(priv, RM57_I2C_DIN_OFFSET) & I2C_PIN_SCL) != 0)
            {
              break;
            }

          up_udelay(1);
        }

      up_udelay(RM57_I2C_RESET_US);
    }

  /* STOP: SDA low while SCL is low, release SCL, then SDA */

  i2c_modifyreg(priv, RM57_I2C_PDIR_OFFSET, 0, I2C_PIN_SCL);
  up_udelay(RM57_I2C_RESET_US);
  i2c_modifyreg(priv, RM57_I2C_PDIR_OFFSET, 0, I2C_PIN_SDA);
  up_udelay(RM57_I2C_RESET_US);
  i2c_modifyreg(priv, RM57_I2C_PDIR_OFFSET, I2C_PIN_SCL, 0);
  up_udelay(RM57_I2C_RESET_US);
  i2c_modifyreg(priv, RM57_I2C_PDIR_OFFSET, I2C_PIN_SDA, 0);
  up_udelay(RM57_I2C_RESET_US);

  din = i2c_getreg(priv, RM57_I2C_DIN_OFFSET);
  ret = (din & (I2C_PIN_SCL | I2C_PIN_SDA)) ==
        (I2C_PIN_SCL | I2C_PIN_SDA) ? OK : -EIO;
  if (ret < 0)
    {
      i2cerr("ERROR: I2C%d: bus still held after reset, DIN %" PRIx32
             "\n", priv->bus, din);
    }

  i2c_hwreset(priv);
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: i2c_instance
 ****************************************************************************/

static struct rm57_i2c_s *i2c_instance(int bus)
{
  switch (bus)
    {
#ifdef CONFIG_RM57_I2C1
      case 1:
        return &g_i2c1dev;
#endif

#ifdef CONFIG_RM57_I2C2
      case 2:
        return &g_i2c2dev;
#endif

      default:
        return NULL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_i2cbus_initialize
 *
 * Description:
 *   Bring one I2C module out of reset as a master; see rm57_i2c.h.
 *
 ****************************************************************************/

struct i2c_master_s *rm57_i2cbus_initialize(int bus)
{
  struct rm57_i2c_s *priv = i2c_instance(bus);
  int ret;

  if (priv == NULL)
    {
      i2cerr("ERROR: I2C%d not enabled\n", bus);
      return NULL;
    }

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      i2c_hwreset(priv);

      ret = irq_attach(priv->irq, i2c_interrupt, priv);
      if (ret < 0)
        {
          i2cerr("ERROR: I2C%d: irq_attach(%d) failed: %d\n",
                 priv->bus, priv->irq, ret);
          i2c_putreg(priv, RM57_I2C_MDR_OFFSET, 0);
          priv->refs = 0;
          nxmutex_unlock(&priv->lock);
          return NULL;
        }

      up_enable_irq(priv->irq);
    }

  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

/****************************************************************************
 * Name: rm57_i2cbus_uninitialize
 *
 * Description:
 *   Drop one reference to an I2C master; see rm57_i2c.h.
 *
 ****************************************************************************/

int rm57_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct rm57_i2c_s *priv = (struct rm57_i2c_s *)dev;

  DEBUGASSERT(priv != NULL);

  nxmutex_lock(&priv->lock);
  if (priv->refs == 0)
    {
      nxmutex_unlock(&priv->lock);
      return -ENODEV;
    }

  if (--priv->refs == 0)
    {
      up_disable_irq(priv->irq);
      irq_detach(priv->irq);
      i2c_putreg(priv, RM57_I2C_IMR_OFFSET, 0);
      i2c_putreg(priv, RM57_I2C_MDR_OFFSET, 0);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

#ifdef CONFIG_RM57_I2C_LOOPBACK
/****************************************************************************
 * Name: rm57_i2c_loopback
 *
 * Description:
 *   Switch digital loopback on or off; see rm57_i2c.h.
 *
 ****************************************************************************/

int rm57_i2c_loopback(struct i2c_master_s *dev, uint8_t *echo,
                      size_t echolen)
{
  struct rm57_i2c_s *priv = (struct rm57_i2c_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->echo    = echo;
  priv->echolen = echo != NULL ? echolen : 0;
  priv->echoidx = 0;
  i2c_hwreset(priv);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: rm57_i2c_echocount
 ****************************************************************************/

size_t rm57_i2c_echocount(struct i2c_master_s *dev)
{
  return ((struct rm57_i2c_s *)dev)->echoidx;
}
#endif

#endif /* CONFIG_RM57_I2C */
