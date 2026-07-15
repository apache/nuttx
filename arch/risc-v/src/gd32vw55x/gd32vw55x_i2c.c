/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_i2c.c
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

/****************************************************************************
 * The GD32VW55x I2C is the "new" GigaDevice I2C IP (it is not the same IP
 * as the GD32F4 I2C).  The bit timing is programmed through the single
 * TIMING register and the transfer itself is driven by the hardware state
 * machine: the address, the direction and the number of bytes are loaded
 * into CTL1 and the hardware generates START, the address phase, the byte
 * count reload and, in automatic end mode, the STOP condition.
 *
 * This driver implements the I2C master interface only.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "riscv_internal.h"

#include "chip.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_gpio.h"
#include "gd32vw55x_i2c.h"
#include "hardware/gd32vw55x_rcu.h"

#if defined(CONFIG_GD32VW55X_I2C0) || defined(CONFIG_GD32VW55X_I2C1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeout for a complete transfer */

#ifndef CONFIG_GD32VW55X_I2C_TIMEOSEC
#  define CONFIG_GD32VW55X_I2C_TIMEOSEC   (0)
#endif

#ifndef CONFIG_GD32VW55X_I2C_TIMEOMS
#  define CONFIG_GD32VW55X_I2C_TIMEOMS    (500)
#endif

#ifndef CONFIG_GD32VW55X_I2C_TIMEOTICKS
#  define CONFIG_GD32VW55X_I2C_TIMEOTICKS \
     (SEC2TICK(CONFIG_GD32VW55X_I2C_TIMEOSEC) + \
      MSEC2TICK(CONFIG_GD32VW55X_I2C_TIMEOMS))
#endif

/* Timeout, in loop counts, used while waiting for the bus to become idle */

#define I2C_BUSY_TIMEOUT      (100000)

/* GPIO configuration used to bit-bang the bus in gd32_i2c_reset() */

#define I2C_OUTPUT            (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                               GPIO_CFG_OD | GPIO_CFG_SPEED_50MHZ | \
                               GPIO_CFG_OUTPUT_SET)

#define MKI2C_OUTPUT(p)       (((p) & (GPIO_CFG_PORT_MASK | \
                                       GPIO_CFG_PIN_MASK)) | I2C_OUTPUT)

/* The interrupts that are used by this driver */

#define I2C_INT_MASK          (I2C_CTL0_TIE | I2C_CTL0_RBNEIE | \
                               I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | \
                               I2C_CTL0_TCIE | I2C_CTL0_ERRIE)

/* Bit timing: the data setup and data hold times, expressed in prescaled
 * clock ticks.  These values are conservative and work for the standard,
 * fast and fast-plus modes.
 */

#define I2C_SCLDELY_VALUE     (4)
#define I2C_SDADELY_VALUE     (2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum gd32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for a transfer to complete */
  INTSTATE_DONE           /* Interrupt activity complete */
};

/* I2C hardware configuration */

struct gd32_i2c_config_s
{
  uint32_t i2c_base;      /* I2C base address */
  uint32_t clk_freq;      /* Frequency of the I2C source clock */
  uint32_t scl_pin;       /* GPIO configuration of SCL */
  uint32_t sda_pin;       /* GPIO configuration of SDA */
  uint32_t rcu_en;        /* RCU clock enable bit */
  uint32_t rcu_rst;       /* RCU reset bit */
#ifndef CONFIG_I2C_POLLED
  int      event_irq;     /* Event IRQ number */
  int      error_irq;     /* Error IRQ number */
#endif
};

/* I2C device state */

struct gd32_i2c_priv_s
{
  const struct i2c_ops_s *ops;           /* Standard I2C operations */
  const struct gd32_i2c_config_s *config;

  int      refs;                         /* Reference count */
  mutex_t  lock;                         /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t    sem_isr;                      /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;             /* Interrupt handshake */

  uint32_t frequency;                    /* Current bus frequency */

  struct i2c_msg_s *msgv;                /* Message vector */
  int      msgc;                         /* Number of messages */
  int      msgid;                        /* Index of the current message */

  uint8_t *ptr;                          /* Current buffer position */
  int      dcnt;                         /* Bytes left in the message */
  volatile int result;                   /* Result of the transfer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t gd32_i2c_getreg(struct gd32_i2c_priv_s *priv,
                                       uint32_t offset);
static inline void gd32_i2c_putreg(struct gd32_i2c_priv_s *priv,
                                   uint32_t offset, uint32_t value);
static inline void gd32_i2c_modifyreg(struct gd32_i2c_priv_s *priv,
                                      uint32_t offset, uint32_t setbits,
                                      uint32_t clrbits);

static int  gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv);
static int  gd32_i2c_sem_waitidle(struct gd32_i2c_priv_s *priv);

static void gd32_i2c_setclock(struct gd32_i2c_priv_s *priv,
                              uint32_t frequency);
static void gd32_i2c_setbytenum(struct gd32_i2c_priv_s *priv);
static void gd32_i2c_startmsg(struct gd32_i2c_priv_s *priv);
static void gd32_i2c_finish(struct gd32_i2c_priv_s *priv, int result);

static int  gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv);
#ifndef CONFIG_I2C_POLLED
static int  gd32_i2c_isr(int irq, void *context, void *arg);
#endif

static int  gd32_i2c_init(struct gd32_i2c_priv_s *priv);
static int  gd32_i2c_deinit(struct gd32_i2c_priv_s *priv);

static int  gd32_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  gd32_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2c_ops =
{
  .transfer = gd32_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset    = gd32_i2c_reset,
#endif
};

#ifdef CONFIG_GD32VW55X_I2C0
static const struct gd32_i2c_config_s g_i2c0_config =
{
  .i2c_base   = GD32VW55X_I2C0_BASE,
  .clk_freq   = GD32VW55X_PCLK1_FREQ,
  .scl_pin    = GPIO_I2C0_SCL,
  .sda_pin    = GPIO_I2C0_SDA,
  .rcu_en     = RCU_APB1EN_I2C0EN,
  .rcu_rst    = RCU_APB1RST_I2C0RST,
#ifndef CONFIG_I2C_POLLED
  .event_irq  = GD32VW55X_IRQ_I2C0_EV,
  .error_irq  = GD32VW55X_IRQ_I2C0_ER,
#endif
};

static struct gd32_i2c_priv_s g_i2c0_priv =
{
  .ops        = &g_i2c_ops,
  .config     = &g_i2c0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgv       = NULL,
  .msgc       = 0,
  .msgid      = 0,
  .ptr        = NULL,
  .dcnt       = 0,
  .result     = OK,
};
#endif

#ifdef CONFIG_GD32VW55X_I2C1
static const struct gd32_i2c_config_s g_i2c1_config =
{
  .i2c_base   = GD32VW55X_I2C1_BASE,
  .clk_freq   = GD32VW55X_PCLK1_FREQ,
  .scl_pin    = GPIO_I2C1_SCL,
  .sda_pin    = GPIO_I2C1_SDA,
  .rcu_en     = RCU_APB1EN_I2C1EN,
  .rcu_rst    = RCU_APB1RST_I2C1RST,
#ifndef CONFIG_I2C_POLLED
  .event_irq  = GD32VW55X_IRQ_I2C1_EV,
  .error_irq  = GD32VW55X_IRQ_I2C1_ER,
#endif
};

static struct gd32_i2c_priv_s g_i2c1_priv =
{
  .ops        = &g_i2c_ops,
  .config     = &g_i2c1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgv       = NULL,
  .msgc       = 0,
  .msgid      = 0,
  .ptr        = NULL,
  .dcnt       = 0,
  .result     = OK,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2c_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t gd32_i2c_getreg(struct gd32_i2c_priv_s *priv,
                                       uint32_t offset)
{
  return getreg32(priv->config->i2c_base + offset);
}

/****************************************************************************
 * Name: gd32_i2c_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_i2c_putreg(struct gd32_i2c_priv_s *priv,
                                   uint32_t offset, uint32_t value)
{
  putreg32(value, priv->config->i2c_base + offset);
}

/****************************************************************************
 * Name: gd32_i2c_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_i2c_modifyreg(struct gd32_i2c_priv_s *priv,
                                      uint32_t offset, uint32_t setbits,
                                      uint32_t clrbits)
{
  modifyreg32(priv->config->i2c_base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: gd32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for the transfer that is in progress to complete.  In the polled
 *   configuration the state machine is driven from this function.
 *
 * Returned Value:
 *   Zero (OK) if the transfer completed; a negated errno value on a
 *   timeout.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv)
{
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  /* Signal the interrupt handler that we are waiting.  Interrupts are
   * currently disabled but will be re-enabled while
   * nxsem_tickwait_uninterruptible() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;

  do
    {
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_GD32VW55X_I2C_TIMEOTICKS);
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors */

          break;
        }
    }
  while (priv->intstate != INTSTATE_DONE);

  /* Disable the I2C interrupts */

  gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL0_OFFSET, 0, I2C_INT_MASK);

  priv->intstate = INTSTATE_IDLE;
  leave_critical_section(flags);

  return ret;
}
#else
static int gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;

  timeout  = CONFIG_GD32VW55X_I2C_TIMEOTICKS;
  start    = clock_systime_ticks();
  priv->intstate = INTSTATE_WAITING;

  do
    {
      /* Poll by simply calling the timer interrupt handler with the
       * interrupts disabled.
       */

      gd32_i2c_isr_process(priv);

      elapsed = clock_systime_ticks() - start;
    }
  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  /* Disable the I2C interrupt sources */

  gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL0_OFFSET, 0, I2C_INT_MASK);

  if (priv->intstate != INTSTATE_DONE)
    {
      priv->intstate = INTSTATE_IDLE;
      return -ETIMEDOUT;
    }

  priv->intstate = INTSTATE_IDLE;
  return OK;
}
#endif

/****************************************************************************
 * Name: gd32_i2c_sem_waitidle
 *
 * Description:
 *   Wait until the I2C bus is no longer busy
 *
 * Returned Value:
 *   Zero (OK) on success; -EBUSY if the bus never becomes idle
 *
 ****************************************************************************/

static int gd32_i2c_sem_waitidle(struct gd32_i2c_priv_s *priv)
{
  uint32_t timeout;

  for (timeout = 0; timeout < I2C_BUSY_TIMEOUT; timeout++)
    {
      if ((gd32_i2c_getreg(priv, GD32VW55X_I2C_STAT_OFFSET) &
           I2C_STAT_I2CBSY) == 0)
        {
          return OK;
        }
    }

  i2cerr("ERROR: The I2C bus is busy\n");
  return -EBUSY;
}

/****************************************************************************
 * Name: gd32_i2c_setclock
 *
 * Description:
 *   Set the I2C bit timing for the requested SCL frequency.
 *
 *   The SCL period is:
 *
 *     tSCL = (SCLL + 1 + SCLH + 1) x (PSC + 1) / fI2CCLK
 *
 *   A duty cycle of 50% is used in standard mode and of 33% (SCL high) in
 *   the fast and fast plus modes.
 *
 ****************************************************************************/

static void gd32_i2c_setclock(struct gd32_i2c_priv_s *priv,
                              uint32_t frequency)
{
  uint32_t i2cclk;
  uint32_t cycles;
  uint32_t regval;
  uint32_t sclh = 0;
  uint32_t scll = 0;
  uint32_t psc;

  /* Has the frequency changed? */

  if (frequency == priv->frequency || frequency == 0)
    {
      return;
    }

  i2cclk = priv->config->clk_freq;

  /* Find the smallest prescaler that keeps SCLL and SCLH in range */

  for (psc = 0; psc < 16; psc++)
    {
      cycles = (i2cclk / (psc + 1)) / frequency;

      if (cycles < 4)
        {
          /* The requested frequency is too high for this source clock */

          break;
        }

      if (frequency > 100000)
        {
          sclh = cycles / 3;
        }
      else
        {
          sclh = cycles / 2;
        }

      scll = cycles - sclh;

      /* The register fields hold the period minus one */

      sclh = (sclh > 0) ? sclh - 1 : 0;
      scll = (scll > 0) ? scll - 1 : 0;

      if (sclh <= 255 && scll <= 255)
        {
          break;
        }
    }

  if (psc > 15)
    {
      psc  = 15;
      sclh = 255;
      scll = 255;
    }

  regval = I2C_TIMING_PSC(psc) |
           I2C_TIMING_SCLDELY(I2C_SCLDELY_VALUE) |
           I2C_TIMING_SDADELY(I2C_SDADELY_VALUE) |
           I2C_TIMING_SCLH(sclh) |
           I2C_TIMING_SCLL(scll);

  /* The TIMING register can only be written while the peripheral is
   * disabled.
   */

  gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL0_OFFSET, 0, I2C_CTL0_I2CEN);
  gd32_i2c_putreg(priv, GD32VW55X_I2C_TIMING_OFFSET, regval);
  gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL0_OFFSET, I2C_CTL0_I2CEN, 0);

  i2cinfo("Frequency %" PRIu32 " Hz: TIMING=%08" PRIx32 "\n",
          frequency, regval);

  priv->frequency = frequency;
}

/****************************************************************************
 * Name: gd32_i2c_nostart_next
 *
 * Description:
 *   Check if the message that follows the current one continues the same
 *   transfer without a (repeated) START condition.
 *
 ****************************************************************************/

static bool gd32_i2c_nostart_next(struct gd32_i2c_priv_s *priv)
{
  if (priv->msgid + 1 >= priv->msgc)
    {
      return false;
    }

  return (priv->msgv[priv->msgid + 1].flags & I2C_M_NOSTART) != 0;
}

/****************************************************************************
 * Name: gd32_i2c_setbytenum
 *
 * Description:
 *   Program the number of bytes of the next chunk of the current message
 *   and select the reload or the automatic end mode.
 *
 *   A message that is longer than 255 bytes, or that is continued by a
 *   message with the I2C_M_NOSTART flag, is transferred in reload mode: a
 *   TCR interrupt is generated after each chunk.
 *
 ****************************************************************************/

static void gd32_i2c_setbytenum(struct gd32_i2c_priv_s *priv)
{
  uint32_t regval;
  uint32_t bytenum;
  bool reload;
  bool last;

  bytenum = (priv->dcnt > I2C_BYTENUM_MAX) ? I2C_BYTENUM_MAX : priv->dcnt;
  reload  = (priv->dcnt > I2C_BYTENUM_MAX) || gd32_i2c_nostart_next(priv);
  last    = (priv->msgid == priv->msgc - 1);

  regval  = gd32_i2c_getreg(priv, GD32VW55X_I2C_CTL1_OFFSET);
  regval &= ~(I2C_CTL1_BYTENUM_MASK | I2C_CTL1_RELOAD | I2C_CTL1_AUTOEND);
  regval |= I2C_CTL1_BYTENUM(bytenum);

  if (reload)
    {
      regval |= I2C_CTL1_RELOAD;
    }
  else if (last)
    {
      /* The hardware generates the STOP condition when the last byte of the
       * last message has been transferred.
       */

      regval |= I2C_CTL1_AUTOEND;
    }

  gd32_i2c_putreg(priv, GD32VW55X_I2C_CTL1_OFFSET, regval);
}

/****************************************************************************
 * Name: gd32_i2c_startmsg
 *
 * Description:
 *   Start the message that priv->msgid refers to: program the slave
 *   address, the direction and the byte count, and request a START (or a
 *   repeated START) condition.
 *
 ****************************************************************************/

static void gd32_i2c_startmsg(struct gd32_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  uint32_t regval;

  priv->ptr  = msg->buffer;
  priv->dcnt = msg->length;

  /* Set the slave address and the transfer direction */

  regval = gd32_i2c_getreg(priv, GD32VW55X_I2C_CTL1_OFFSET);
  regval &= ~(I2C_CTL1_SADDRESS_MASK | I2C_CTL1_TRDIR | I2C_CTL1_ADD10EN |
              I2C_CTL1_HEAD10R | I2C_CTL1_BYTENUM_MASK | I2C_CTL1_RELOAD |
              I2C_CTL1_AUTOEND);

  if ((msg->flags & I2C_M_TEN) != 0)
    {
      regval |= I2C_CTL1_SADDRESS(msg->addr & 0x3ff);
      regval |= I2C_CTL1_ADD10EN;
    }
  else
    {
      regval |= I2C_CTL1_SADDRESS((msg->addr & 0x7f) << 1);
    }

  if ((msg->flags & I2C_M_READ) != 0)
    {
      regval |= I2C_CTL1_TRDIR_READ;
    }

  gd32_i2c_putreg(priv, GD32VW55X_I2C_CTL1_OFFSET, regval);

  /* Program the byte count and the reload/automatic end mode */

  gd32_i2c_setbytenum(priv);

  /* Enable the interrupts and request the START condition */

  gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL0_OFFSET, I2C_INT_MASK, 0);
  gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL1_OFFSET, I2C_CTL1_START, 0);
}

/****************************************************************************
 * Name: gd32_i2c_finish
 *
 * Description:
 *   Terminate the transfer: save the result, disable the interrupts and
 *   wake up the waiting thread.
 *
 ****************************************************************************/

static void gd32_i2c_finish(struct gd32_i2c_priv_s *priv, int result)
{
  priv->result = result;

  /* Disable all of the interrupt sources */

  gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL0_OFFSET, 0, I2C_INT_MASK);

  priv->intstate = INTSTATE_DONE;

#ifndef CONFIG_I2C_POLLED
  nxsem_post(&priv->sem_isr);
#endif
}

/****************************************************************************
 * Name: gd32_i2c_isr_process
 *
 * Description:
 *   The common I2C interrupt logic.  It handles both the event and the
 *   error interrupts, and it is also called from the polled configuration.
 *
 ****************************************************************************/

static int gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv)
{
  uint32_t status;

  status = gd32_i2c_getreg(priv, GD32VW55X_I2C_STAT_OFFSET);

  /* Check for a NACK from the slave */

  if ((status & I2C_STAT_NACK) != 0)
    {
      gd32_i2c_putreg(priv, GD32VW55X_I2C_STATC_OFFSET, I2C_STATC_NACKC);

      i2cerr("ERROR: NACK, addr=0x%02x\n", priv->msgv[priv->msgid].addr);

      /* Request the STOP condition.  If the automatic end mode is active
       * the hardware has already sent it, in which case this bit is
       * ignored.
       */

      gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL1_OFFSET,
                         I2C_CTL1_STOP, 0);

      gd32_i2c_finish(priv, -ENXIO);
      return OK;
    }

  /* Check for any of the bus errors */

  if ((status & I2C_STAT_ERRORS) != 0)
    {
      gd32_i2c_putreg(priv, GD32VW55X_I2C_STATC_OFFSET, I2C_STATC_ERRORS);

      i2cerr("ERROR: I2C error, STAT=0x%08" PRIx32 "\n", status);

      gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL1_OFFSET,
                         I2C_CTL1_STOP, 0);

      gd32_i2c_finish(priv, -EIO);
      return OK;
    }

  /* Transmit the next byte */

  if ((status & I2C_STAT_TI) != 0 && priv->dcnt > 0)
    {
      gd32_i2c_putreg(priv, GD32VW55X_I2C_TDATA_OFFSET, *priv->ptr++);
      priv->dcnt--;
    }

  /* Receive the next byte */

  if ((status & I2C_STAT_RBNE) != 0 && priv->dcnt > 0)
    {
      *priv->ptr++ = (uint8_t)(gd32_i2c_getreg(priv,
                                               GD32VW55X_I2C_RDATA_OFFSET) &
                               I2C_RDATA_MASK);
      priv->dcnt--;
    }

  /* Reload: the byte count of the previous chunk has been reached but the
   * transfer is not complete.
   */

  if ((status & I2C_STAT_TCR) != 0)
    {
      if (priv->dcnt == 0)
        {
          /* The current message is complete.  It must be continued by a
           * message with the I2C_M_NOSTART flag.
           */

          struct i2c_msg_s *next;

          if (priv->msgid + 1 >= priv->msgc)
            {
              gd32_i2c_finish(priv, -EINVAL);
              return OK;
            }

          next = &priv->msgv[priv->msgid + 1];

          /* The direction cannot be changed without a repeated START */

          if (((next->flags ^ priv->msgv[priv->msgid].flags) &
               I2C_M_READ) != 0)
            {
              i2cerr("ERROR: I2C_M_NOSTART cannot change direction\n");
              gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL1_OFFSET,
                                 I2C_CTL1_STOP, 0);
              gd32_i2c_finish(priv, -EINVAL);
              return OK;
            }

          priv->msgid++;
          priv->ptr  = next->buffer;
          priv->dcnt = next->length;
        }

      /* Program the byte count of the next chunk */

      gd32_i2c_setbytenum(priv);
    }

  /* Transfer complete: the current message has been transferred and the
   * automatic end mode is not active, so a repeated START is needed.
   */

  if ((status & I2C_STAT_TC) != 0)
    {
      if (priv->msgid + 1 < priv->msgc)
        {
          priv->msgid++;
          gd32_i2c_startmsg(priv);
        }
      else
        {
          /* Nothing left to do: generate the STOP condition */

          gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL1_OFFSET,
                             I2C_CTL1_STOP, 0);
        }
    }

  /* The STOP condition has been detected: the transfer is complete */

  if ((status & I2C_STAT_STPDET) != 0)
    {
      gd32_i2c_putreg(priv, GD32VW55X_I2C_STATC_OFFSET, I2C_STATC_STPDETC);
      gd32_i2c_finish(priv, priv->result);
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_i2c_isr
 *
 * Description:
 *   The I2C event and error interrupt handler
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
 * Name: gd32_i2c_init
 *
 * Description:
 *   Set up the I2C hardware, so it is ready for use
 *
 ****************************************************************************/

static int gd32_i2c_init(struct gd32_i2c_priv_s *priv)
{
  const struct gd32_i2c_config_s *config = priv->config;

  /* Enable the I2C clock */

  modifyreg32(GD32VW55X_RCU_APB1EN, 0, config->rcu_en);

  /* Reset the peripheral to be sure that it is in a known state */

  modifyreg32(GD32VW55X_RCU_APB1RST, 0, config->rcu_rst);
  modifyreg32(GD32VW55X_RCU_APB1RST, config->rcu_rst, 0);

  /* Configure the SCL and SDA pins */

  if (gd32_gpio_config(config->scl_pin) < 0)
    {
      return -EINVAL;
    }

  if (gd32_gpio_config(config->sda_pin) < 0)
    {
      gd32_gpio_unconfig(config->scl_pin);
      return -EINVAL;
    }

#ifndef CONFIG_I2C_POLLED
  /* Attach and enable the event and the error interrupts */

  irq_attach(config->event_irq, gd32_i2c_isr, priv);
  irq_attach(config->error_irq, gd32_i2c_isr, priv);

  up_enable_irq(config->event_irq);
  up_enable_irq(config->error_irq);
#endif

  /* The peripheral must be disabled while it is configured.  Use the
   * analog noise filter and no digital filter, and do not stretch the
   * clock in slave mode (this driver is master only).
   */

  gd32_i2c_putreg(priv, GD32VW55X_I2C_CTL0_OFFSET, 0);
  gd32_i2c_putreg(priv, GD32VW55X_I2C_CTL1_OFFSET, 0);

  /* Force the frequency to be re-programmed and set the default frequency
   * of 100 KHz.  This also enables the peripheral.
   */

  priv->frequency = 0;
  gd32_i2c_setclock(priv, 100000);

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
  const struct gd32_i2c_config_s *config = priv->config;

  /* Disable the peripheral */

  gd32_i2c_modifyreg(priv, GD32VW55X_I2C_CTL0_OFFSET, 0, I2C_CTL0_I2CEN);

  /* Unconfigure the GPIO pins */

  gd32_gpio_unconfig(config->scl_pin);
  gd32_gpio_unconfig(config->sda_pin);

#ifndef CONFIG_I2C_POLLED
  /* Disable and detach the interrupts */

  up_disable_irq(config->event_irq);
  up_disable_irq(config->error_irq);

  irq_detach(config->event_irq);
  irq_detach(config->error_irq);
#endif

  /* Disable the clock of the peripheral */

  modifyreg32(GD32VW55X_RCU_APB1EN, config->rcu_en, 0);

  priv->frequency = 0;

  return OK;
}

/****************************************************************************
 * Name: gd32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   msgs  - The vector of the messages to be transferred
 *   count - The number of messages in the vector
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

static int gd32_i2c_transfer(struct i2c_master_s *dev,
                             struct i2c_msg_s *msgs, int count)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL && msgs != NULL && count > 0);

  /* The first message cannot continue a previous transfer */

  if ((msgs[0].flags & I2C_M_NOSTART) != 0)
    {
      return -EINVAL;
    }

  /* Get exclusive access to the I2C bus */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for the bus to become idle */

  ret = gd32_i2c_sem_waitidle(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Set up the message vector */

  priv->msgv   = msgs;
  priv->msgc   = count;
  priv->msgid  = 0;
  priv->ptr    = NULL;
  priv->dcnt   = 0;
  priv->result = OK;

  /* Select the frequency of the first message */

  gd32_i2c_setclock(priv, msgs[0].frequency);

  /* Clear any pending status flags */

  gd32_i2c_putreg(priv, GD32VW55X_I2C_STATC_OFFSET,
                  I2C_STATC_ERRORS | I2C_STATC_NACKC | I2C_STATC_STPDETC);

  /* Start the first message and wait for the transfer to complete */

  gd32_i2c_startmsg(priv);

  ret = gd32_i2c_sem_waitdone(priv);
  if (ret < 0)
    {
      i2cerr("ERROR: Timed out: STAT=0x%08" PRIx32 "\n",
             gd32_i2c_getreg(priv, GD32VW55X_I2C_STAT_OFFSET));

      /* Reset the peripheral to abort the transfer */

      gd32_i2c_deinit(priv);
      gd32_i2c_init(priv);

      ret = -ETIMEDOUT;
    }
  else
    {
      ret = priv->result;
    }

errout:
  priv->msgv = NULL;
  priv->msgc = 0;

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
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

  DEBUGASSERT(priv != NULL && priv->refs > 0);

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

  /* Use a GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  gd32_gpio_config(scl_gpio);
  gd32_gpio_config(sda_gpio);

  /* Let SDA go high */

  gd32_gpio_write(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go */

  clock_count = 0;
  while (!gd32_gpio_read(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that the clock stretching has finished.  If the
       * bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!gd32_gpio_read(scl_gpio))
        {
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

  /* Generate a START followed by a STOP to reset the slave state
   * machines.
   */

  gd32_gpio_write(sda_gpio, 0);
  up_udelay(10);
  gd32_gpio_write(scl_gpio, 0);
  up_udelay(10);
  gd32_gpio_write(scl_gpio, 1);
  up_udelay(10);
  gd32_gpio_write(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration */

  gd32_gpio_unconfig(sda_gpio);
  gd32_gpio_unconfig(scl_gpio);

  /* Re-init the port and restore the frequency */

  gd32_i2c_init(priv);
  gd32_i2c_setclock(priv, frequency);

  ret = OK;

out:
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
 *   Initialize the selected I2C port and return a unique instance of
 *   struct i2c_master_s
 *
 * Input Parameters:
 *   port - The I2C port number: 0 (I2C0) or 1 (I2C1)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *gd32_i2cbus_initialize(int port)
{
  struct gd32_i2c_priv_s *priv = NULL;
  irqstate_t flags;

  switch (port)
    {
#ifdef CONFIG_GD32VW55X_I2C0
    case 0:
      priv = &g_i2c0_priv;
      break;
#endif
#ifdef CONFIG_GD32VW55X_I2C1
    case 1:
      priv = &g_i2c1_priv;
      break;
#endif
    default:
      i2cerr("ERROR: Unsupported I2C port: %d\n", port);
      return NULL;
    }

  /* Initialize the private data of the device, if it has not been done */

  flags = enter_critical_section();

  if (priv->refs++ == 0)
    {
      if (gd32_i2c_init(priv) < 0)
        {
          priv->refs--;
          leave_critical_section(flags);
          return NULL;
        }
    }

  leave_critical_section(flags);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: gd32_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port and power down the device
 *
 * Input Parameters:
 *   dev - Device structure as returned by gd32_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success; ERROR when the internal reference count mismatches or
 *   the dev pointer is invalid
 *
 ****************************************************************************/

int gd32_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

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

  /* Disable power and other HW resources (GPIO pins) */

  gd32_i2c_deinit(priv);

  return OK;
}

#endif /* CONFIG_GD32VW55X_I2C0 || CONFIG_GD32VW55X_I2C1 */
