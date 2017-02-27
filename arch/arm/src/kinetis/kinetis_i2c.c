/****************************************************************************
 * arch/arm/src/kinetis/kinetis_i2c.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Author:  Matias v01d <phreakuencies@gmail.com>
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "kinetis_config.h"
#include "chip.h"
#include "chip/kinetis_i2c.h"
#include "chip/kinetis_sim.h"
#include "chip/kinetis_pinmux.h"
#include "kinetis.h"
#include "kinetis_i2c.h"

#if defined(CONFIG_KINETIS_I2C0) || defined(CONFIG_KINETIS_I2C1) || \
    defined(CONFIG_KINETIS_I2C2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_TIMEOUT  (20*1000/CONFIG_USEC_PER_TICK)   /* 20 mS */

#define I2C_DEFAULT_FREQUENCY 400000

#define STATE_OK                0
#define STATE_ARBITRATION_ERROR 1
#define STATE_TIMEOUT           2
#define STATE_NAK               3

/* TODO:
 * - revisar tamanio de todos los registros (getreg/putreg)
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C device state structure */

struct kinetis_i2cdev_s
{
  struct i2c_master_s dev;    /* Generic I2C device */
  uintptr_t base;             /* Base address of registers */
  uint32_t basefreq;          /* Branch frequency */
  uint32_t frequency;         /* Current I2C frequency */
  uint16_t irqid;             /* IRQ for this device */
  uint16_t nmsg;              /* Number of transfer remaining */
  uint16_t wrcnt;             /* number of bytes sent to tx fifo */
  uint16_t rdcnt;             /* number of bytes read from rx fifo */
  volatile uint8_t state;     /* State of state machine */
  bool restart;               /* Should next transfer restart or not */
  sem_t mutex;                /* Only one thread can access at a time */
  sem_t wait;                 /* Place to wait for state machine completion */
  WDOG_ID timeout;            /* watchdog to timeout when bus hung */
  struct i2c_msg_s *msgs;     /* Remaining transfers - first one is in
                               * progress */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static uint8_t kinetis_i2c_getreg(struct kinetis_i2cdev_s *priv,
                                  uint8_t offset);
static void kinetis_i2c_putreg(struct kinetis_i2cdev_s *priv,
                               uint8_t value, uint8_t offset);

/* I2C helpers */

static void kinetis_i2c_setfrequency(struct kinetis_i2cdev_s *priv,
                                     uint32_t frequency);
static int  kinetis_i2c_start(struct kinetis_i2cdev_s *priv);
static void kinetis_i2c_stop(struct kinetis_i2cdev_s *priv);
static int kinetis_i2c_interrupt(int irq, void *context, void *arg);
static void kinetis_i2c_timeout(int argc, uint32_t arg, ...);
static void kinetis_i2c_setfrequency(struct kinetis_i2cdev_s *priv,
                                     uint32_t frequency);

/* I2C lower half driver methods */

static int  kinetis_i2c_transfer(struct i2c_master_s *dev,
                                 struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  kinetis_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C lower half driver operations */

static const struct i2c_ops_s g_i2c_ops =
{
  .transfer = kinetis_i2c_transfer
#ifdef CONFIG_I2C_RESET
  ,.reset   = kinetis_i2c_reset
#endif
};

/* I2C device state instances */

#ifdef CONFIG_KINETIS_I2C0
static struct kinetis_i2cdev_s g_i2c0_dev;
#endif
#ifdef CONFIG_KINETIS_I2C1
static struct kinetis_i2cdev_s g_i2c1_dev;
#endif
#ifdef CONFIG_KINETIS_I2C2
static struct kinetis_i2cdev_s g_i2c2_dev;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static uint8_t kinetis_i2c_getreg(struct kinetis_i2cdev_s *priv,
                                  uint8_t offset)
{
  return getreg8(priv->base + offset);
}

/****************************************************************************
 * Name: kinetis_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static void kinetis_i2c_putreg(struct kinetis_i2cdev_s *priv, uint8_t value,
                               uint8_t offset)
{
  putreg8(value, priv->base + offset);
}

/****************************************************************************
 * Name: kinetis_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void kinetis_i2c_setfrequency(struct kinetis_i2cdev_s *priv,
                                     uint32_t frequency)
{
  i2cinfo("frequency=%lu\n", (unsigned long)frequency);

  if (frequency == priv->frequency)
    {
      return;
    }

#if BOARD_BUS_FREQ == 120000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV1152, KINETIS_I2C_F_OFFSET);   /* 104 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV288, KINETIS_I2C_F_OFFSET);    /* 416 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV128, KINETIS_I2C_F_OFFSET);    /* 0.94 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 108000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV1024, KINETIS_I2C_F_OFFSET);   /* 105 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV256, KINETIS_I2C_F_OFFSET);    /* 422 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV112, KINETIS_I2C_F_OFFSET);    /* 0.96 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 96000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV960, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV240, KINETIS_I2C_F_OFFSET);    /* 400 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV96, KINETIS_I2C_F_OFFSET);     /* 1.0 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 90000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV896, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV224, KINETIS_I2C_F_OFFSET);    /* 402 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV88, KINETIS_I2C_F_OFFSET);     /* 1.02 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 80000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV768, KINETIS_I2C_F_OFFSET);    /* 104 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV192, KINETIS_I2C_F_OFFSET);    /* 416 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV80, KINETIS_I2C_F_OFFSET);     /* 1.0 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 72000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV640, KINETIS_I2C_F_OFFSET);    /* 112 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV192, KINETIS_I2C_F_OFFSET);    /* 375 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV72, KINETIS_I2C_F_OFFSET);     /* 1.0 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 64000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV640, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV160, KINETIS_I2C_F_OFFSET);    /* 400 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV64, KINETIS_I2C_F_OFFSET);     /* 1.0 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 60000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV576, KINETIS_I2C_F_OFFSET);    /* 104 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV144, KINETIS_I2C_F_OFFSET);    /* 416 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV64, KINETIS_I2C_F_OFFSET);     /* 938 kHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 56000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV512, KINETIS_I2C_F_OFFSET);    /* 109 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV144, KINETIS_I2C_F_OFFSET);    /* 389 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV56_1, KINETIS_I2C_F_OFFSET);   /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 54000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV512, KINETIS_I2C_F_OFFSET);    /* 105 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV128, KINETIS_I2C_F_OFFSET);    /* 422 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV56, KINETIS_I2C_F_OFFSET);     /* 0.96 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 48000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV480, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV112, KINETIS_I2C_F_OFFSET);    /* 400 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV48_1, KINETIS_I2C_F_OFFSET);   /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 40000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV384_2, KINETIS_I2C_F_OFFSET);  /* 104 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV96, KINETIS_I2C_F_OFFSET);     /* 416 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV40_2, KINETIS_I2C_F_OFFSET);   /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(3), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 36000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV320_2, KINETIS_I2C_F_OFFSET);  /* 113 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV96, KINETIS_I2C_F_OFFSET);     /* 375 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV36, KINETIS_I2C_F_OFFSET);     /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(3), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 24000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV240, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV64, KINETIS_I2C_F_OFFSET);     /* 375 kHz */
    }
  else
    {161
      kinetis_i2c_putreg(priv, I2C_F_DIV24, KINETIS_I2C_F_OFFSET);     /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(2), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 16000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV160_2, KINETIS_I2C_F_OFFSET);  /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV40_1, KINETIS_I2C_F_OFFSET);   /* 400 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV20, KINETIS_I2C_F_OFFSET);     /* 800 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(1), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 8000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV80_1, KINETIS_I2C_F_OFFSET);   /* 100 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV20, KINETIS_I2C_F_OFFSET);     /* 400 kHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(1), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 4000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV40_1, KINETIS_I2C_F_OFFSET);   /* 100 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV20, KINETIS_I2C_F_OFFSET);     /* 200 kHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(1), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 2000000
  kinetis_i2c_putreg(priv, I2C_F_DIV20, KINETIS_I2C_F_OFFSET);         /* 100 kHz */
  kinetis_i2c_putreg(priv, I2C_FLT(1), KINETIS_I2C_FLT_OFFSET);

#else
#  error "F_BUS must be 120, 108, 96, 9, 80, 72, 64, 60, 56, 54, 48, 40, 36, 24, 16, 8, 4 or 2 MHz"
#endif

  priv->frequency = frequency;
}

/****************************************************************************
 * Name: kinetis_i2c_start
 *
 * Description:
 *   Initiate I2C transfer (START/RSTART + address)
 *
 ****************************************************************************/

static int kinetis_i2c_start(struct kinetis_i2cdev_s *priv)
{
  struct i2c_msg_s *msg;

  i2cinfo("START msg=%p\n", priv->msgs);
  msg = priv->msgs;

  /* Now take control of the bus */

  if (kinetis_i2c_getreg(priv, KINETIS_I2C_C1_OFFSET) & I2C_C1_MST)
    {
      /* We are already the bus master, so send a repeated start */

      kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST |
                         I2C_C1_RSTA | I2C_C1_TX, KINETIS_I2C_C1_OFFSET);
    }
  else
    {
      /* We are not currently the bus master, so wait for bus ready */

      while (kinetis_i2c_getreg(priv, KINETIS_I2C_S_OFFSET) & I2C_S_BUSY);

      /* Become the bus master in transmit mode (send start) */

      kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST |
                         I2C_C1_TX,  KINETIS_I2C_C1_OFFSET);
    }

  if (I2C_M_READ & msg->flags)  /* DEBUG: should happen always */
    {
      /* Wait until start condition establishes control of the bus */

      while (1)
        {
          if (kinetis_i2c_getreg(priv, KINETIS_I2C_S_OFFSET) & I2C_S_BUSY)
            {
              break;
            }
        }
    }

  /* Initiate actual transfer (send address) */

  kinetis_i2c_putreg(priv, (I2C_M_READ & msg->flags) == I2C_M_READ ?
                     I2C_READADDR8(msg->addr) : I2C_WRITEADDR8(msg->addr),
                     KINETIS_I2C_D_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: kinetis_i2c_stop
 *
 * Description:
 *   Perform a I2C transfer stop
 *
 ****************************************************************************/

static void kinetis_i2c_stop(struct kinetis_i2cdev_s *priv)
{
  i2cinfo("STOP msg=%p\n", priv->msgs);

  kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE,
                     KINETIS_I2C_C1_OFFSET);
  sem_post(&priv->wait);
}

/****************************************************************************
 * Name: kinetis_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void kinetis_i2c_timeout(int argc, uint32_t arg, ...)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)arg;

  DEBUGASSERT(priv != NULL);
  i2cinfo("Timeout msg=%p\n", priv->msgs);

  irqstate_t flags = enter_critical_section();
  priv->state = STATE_TIMEOUT;
  sem_post(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: kinetis_i2c_nextmsg
 *
 * Description:
 *   Setup for the next message.
 *
 ****************************************************************************/

void kinetis_i2c_nextmsg(struct kinetis_i2cdev_s *priv)
{
  priv->nmsg--;
  i2cinfo("nmsg=%u\n", priv->nmsg);

  if (priv->nmsg > 0)
    {
      priv->msgs++;
      i2cinfo("msg=%p\n", priv->msgs);

      priv->wrcnt = 0;
      priv->rdcnt = 0;

      if (priv->restart)
        {
          sem_post(&priv->wait);
        }
    }
  else
    {
      kinetis_i2c_stop(priv);
    }
}

/****************************************************************************
 * Name: kinetis_i2c_interrupt
 *
 * Description:
 *   The I2C common interrupt handler
 *
 ****************************************************************************/

static int kinetis_i2c_interrupt(int irq, void *context, void *arg)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)arg;
  struct i2c_msg_s *msg;
  uint32_t state;
  int regval;
  int dummy;
  UNUSED(dummy);

  DEBUGASSERT(priv != NULL);

  /* Get current state */

  state = kinetis_i2c_getreg(priv, KINETIS_I2C_S_OFFSET);
  msg = priv->msgs;

  /* Arbitration lost */

  if (state & I2C_S_ARBL)
    {
      kinetis_i2c_putreg(priv, I2C_S_IICIF | I2C_S_ARBL,
                         KINETIS_I2C_S_OFFSET);
      priv->state = STATE_ARBITRATION_ERROR;
      kinetis_i2c_stop(priv);
    }
  else
    {
      /* Clear interrupt */

      kinetis_i2c_putreg(priv, I2C_S_IICIF, KINETIS_I2C_S_OFFSET);
      regval = kinetis_i2c_getreg(priv, KINETIS_I2C_C1_OFFSET);

      /* TX mode */

      if (regval & I2C_C1_TX)
        {
          /* Last write was not acknowledged */

          if (state & I2C_S_RXAK)
            {
              priv->state = STATE_NAK;  /* Set error flag */
              kinetis_i2c_stop(priv);   /* Send STOP */
            }
          else
            {
              /* Actually intending to write */

              if ((I2C_M_READ & msg->flags) == 0)
                {
                  /* Wrote everything */

                  if (priv->wrcnt == msg->length)
                    {
                      /* Continue with next message */

                      kinetis_i2c_nextmsg(priv);

                      if (!priv->restart)
                        {
                          /* Initiate transfer of following message */

                          kinetis_i2c_putreg(priv,
                                             priv->msgs->buffer[priv->wrcnt],
                                             KINETIS_I2C_D_OFFSET);
                          priv->wrcnt++;

                          sem_post(&priv->wait);
                        }
                    }
                  else
                    {
                      /* Put next byte */

                      kinetis_i2c_putreg(priv, msg->buffer[priv->wrcnt],
                                         KINETIS_I2C_D_OFFSET);
                      priv->wrcnt++;
                    }
                }

              /* Actually intending to read (address was just sent) */

              else
                {
                  if (msg->length == 1 && priv->restart)
                    {
                      /* Go to RX mode, do not send ACK */

                      kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                         I2C_C1_MST | I2C_C1_TXAK,
                                         KINETIS_I2C_C1_OFFSET);
                    }
                  else
                    {
                      /* Go to RX mode */

                      kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                         I2C_C1_MST, KINETIS_I2C_C1_OFFSET);
                    }

                  /* TODO: handle zero-length reads */
                  /* Dummy read to initiate reception */

                  dummy = kinetis_i2c_getreg(priv, KINETIS_I2C_D_OFFSET);
                }
            }
        }

      /* RX: mode */

      else
        {
          /* If last receiving byte */

          if (priv->rdcnt == (msg->length - 1))
            {
              if (priv->restart)
                {
                  /* Go to TX mode before last read, otherwise a new read is
                   * triggered.
                   */

                  /* Go to TX mode */

                  kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                     I2C_C1_MST | I2C_C1_TX,
                                     KINETIS_I2C_C1_OFFSET);
                }
              else if ((priv->msgs + 1)->length == 1)
                {
                  /* We will continue reception on next message.
                   * if next message is length == 1, this is actually the
                   * 2nd to last byte, so do not send ACK.
                   */

                  /* Do not ACK any more */

                  kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                     I2C_C1_MST | I2C_C1_TXAK,
                                     KINETIS_I2C_C1_OFFSET);
                }

              msg->buffer[priv->rdcnt] =
                kinetis_i2c_getreg(priv, KINETIS_I2C_D_OFFSET);
              priv->rdcnt++;

              kinetis_i2c_nextmsg(priv);
            }

          /* Second to last receiving byte */

          else if (priv->rdcnt == (msg->length - 2))
            {
              if (priv->restart)
                {
                  /* Do not ACK any more */

                  kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                     I2C_C1_MST | I2C_C1_TXAK, KINETIS_I2C_C1_OFFSET);
                }

              msg->buffer[priv->rdcnt] =
                kinetis_i2c_getreg(priv, KINETIS_I2C_D_OFFSET);
              priv->rdcnt++;
            }
          else
            {
              msg->buffer[priv->rdcnt] =
                kinetis_i2c_getreg(priv, KINETIS_I2C_D_OFFSET);
              priv->rdcnt++;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: kinetis_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int kinetis_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)dev;
  int msg_n;

  i2cinfo("msgs=%p count=%d\n", msgs, count);
  DEBUGASSERT(dev != NULL && msgs != NULL && (unsigned)count <= UINT16_MAX);

  /* Get exclusive access to the I2C bus */

  sem_wait(&priv->mutex);

  /* Set up for the transfer */

  msg_n       = 0;
  priv->msgs  = msgs;
  priv->nmsg  = count;
  priv->state = STATE_OK;
  priv->wrcnt = 0;
  priv->rdcnt = 0;

  /* Configure the I2C frequency. REVISIT: Note that the frequency is set
   * only on the first message. This could be extended to support
   * different transfer frequencies for each message segment.
   */

  kinetis_i2c_setfrequency(priv, msgs->frequency);

  /* Clear the status flags */

  kinetis_i2c_putreg(priv, I2C_S_IICIF | I2C_S_ARBL, KINETIS_I2C_S_OFFSET);

  /* Process every message */

  while (priv->nmsg > 0 && priv->state == STATE_OK)
    {
      priv->restart = true;

      /* Process NORESTART flag */

      if (priv->nmsg > 1)
        {
          struct i2c_msg_s* nextmsg = (priv->msgs + 1);

          /* If there is a following message with "norestart" flag of
           * the same type as the current one, we can avoid the restart
           */

          if ((nextmsg->flags & I2C_M_NORESTART) &&
              nextmsg->addr == priv->msgs->addr &&
              nextmsg->frequency == priv->msgs->frequency &&
              (nextmsg->flags & I2C_M_READ) == (priv->msgs->flags & I2C_M_READ))
            {
              /* "no restart" can be performed */

              priv->restart = false;
            }
        }

      /* Only send start when required (we are trusting the flags setting to
       * be correctly used here).
       */

      if (!(priv->msgs->flags & I2C_M_NORESTART))
        {
          /* Initiate the transfer, in case restart is required */

          kinetis_i2c_start(priv);
        }

      /* Wait for transfer complete */

      wd_start(priv->timeout, I2C_TIMEOUT, kinetis_i2c_timeout, 1,
               (uint32_t) priv);
      sem_wait(&priv->wait);

      wd_cancel(priv->timeout);

      msg_n++;
    }

  /* Disable interrupts */

  kinetis_i2c_putreg(priv, I2C_C1_IICEN, KINETIS_I2C_C1_OFFSET);

  /* Release access to I2C bus */

  sem_post(&priv->mutex);

  if (priv->state != STATE_OK)
    {
      return -EIO;
    }
  else
    {
      return 0;
    }
}

/************************************************************************************
 * Name: kinetis_i2c_reset
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
static int kinetis_i2c_reset(struct i2c_master_s *dev)
{
  i2cinfo("No reset...\n");
  return OK;
}
#endif  /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *kinetis_i2cbus_initialize(int port)
{
  struct kinetis_i2cdev_s *priv;

  i2cinfo("port=%d\n", port);

  if (port > 1)
    {
      i2cerr("ERROR: Kinetis I2C Only suppors ports 0 and 1\n");
      return NULL;
    }

  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

#ifdef CONFIG_KINETIS_I2C0
  if (port == 0)
    {
      priv           = &g_i2c0_dev;
      priv->base     = KINETIS_I2C0_BASE;
      priv->irqid    = KINETIS_IRQ_I2C0;
      priv->basefreq = BOARD_BUS_FREQ;

      /* Enable clock */

      regval         = getreg32(KINETIS_SIM_SCGC4);
      regval        |= SIM_SCGC4_I2C0;
      putreg32(regval, KINETIS_SIM_SCGC4);

      /* Disable while configuring */

      kinetis_i2c_putreg(priv, 0, KINETIS_I2C_C1_OFFSET);

      /* Configure pins */

      kinetis_pinconfig(PIN_I2C0_SCL);
      kinetis_pinconfig(PIN_I2C0_SDA);
    }
  else
#endif
#ifdef CONFIG_KINETIS_I2C1
  if (port == 1)
    {
      priv           = &g_i2c1_dev;
      priv->base     = KINETIS_I2C1_BASE;
      priv->irqid    = KINETIS_IRQ_I2C1;
      priv->basefreq = BOARD_BUS_FREQ;

      /* Enable clock */

      regval         = getreg32(KINETIS_SIM_SCGC4);
      regval        |= SIM_SCGC4_I2C1;
      putreg32(regval, KINETIS_SIM_SCGC4);

      /* Disable while configuring */

      kinetis_i2c_putreg(priv, 0, KINETIS_I2C_C1_OFFSET);

      /* Configure pins */

      kinetis_pinconfig(PIN_I2C1_SCL);
      kinetis_pinconfig(PIN_I2C1_SDA);
    }
  else
#endif
#ifdef CONFIG_KINETIS_I2C2
  if (port == 2)
    {
      priv           = &g_i2c2_dev;
      priv->base     = KINETIS_I2C2_BASE;
      priv->irqid    = KINETIS_IRQ_I2C2;
      priv->basefreq = BOARD_BUS_FREQ;

      /* Enable clock */

      regval         = getreg32(KINETIS_SIM_SCGC4);
      regval        |= SIM_SCGC4_I2C2;
      putreg32(regval, KINETIS_SIM_SCGC4);

      /* Disable while configuring */

      kinetis_i2c_putreg(priv, 0, KINETIS_I2C_C1_OFFSET);

      /* Configure pins */

      kinetis_pinconfig(PIN_I2C2_SCL);
      kinetis_pinconfig(PIN_I2C2_SDA);
    }
  else
#endif
    {
      leave_critical_section(flags);
      i2cerr("ERROR: Unsupport I2C bus: %d\n", port);
      return NULL;
    }

  /* Set the default I2C frequency */

  kinetis_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

  /* Enable */

  kinetis_i2c_putreg(priv, I2C_C1_IICEN, KINETIS_I2C_C1_OFFSET);

  /* High-drive select (TODO: why)? */

  regval = kinetis_i2c_getreg(priv, KINETIS_I2C_C2_OFFSET);
  regval |= I2C_C2_HDRS;
  kinetis_i2c_putreg(priv, regval, KINETIS_I2C_C2_OFFSET);

  leave_critical_section(flags);

  /* Initialize semaphores */

  sem_init(&priv->mutex, 0, 1);
  sem_init(&priv->wait, 0, 0);

  /* The wait semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  sem_setprotocol(&priv->wait, SEM_PRIO_NONE);

  /* Allocate a watchdog timer */

  priv->timeout = wd_create();
  DEBUGASSERT(priv->timeout != 0);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, kinetis_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  /* Install our operations */

  priv->dev.ops = &g_i2c_ops;
  return &priv->dev;
}

/****************************************************************************
 * Name: kinetis_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int kinetis_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)dev;

  DEBUGASSERT(priv != NULL);

  kinetis_i2c_putreg(priv, 0, KINETIS_I2C_C1_OFFSET);

  up_disable_irq(priv->irqid);
  irq_detach(priv->irqid);
  return OK;
}

#endif /* CONFIG_KINETIS_I2C0 || CONFIG_KINETIS_I2C1 || CONFIG_KINETIS_I2C2 */
