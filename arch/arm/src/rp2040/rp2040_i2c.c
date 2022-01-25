/****************************************************************************
 * arch/arm/src/rp2040/rp2040_i2c.c
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
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "rp2040_i2c.h"
#include "hardware/rp2040_i2c.h"
#include "hardware/rp2040_resets.h"
#include "rp2040_gpio.h"

#ifdef CONFIG_RP2040_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_TIMEOUT  (20*1000/CONFIG_USEC_PER_TICK) /* 20 mS */

#define I2C_DEFAULT_FREQUENCY 400000
#define I2C_FIFO_MAX_SIZE	    32

#define I2C_INTR_ENABLE ((RP2040_I2C_IC_INTR_STAT_R_STOP_DET) | \
                         (RP2040_I2C_IC_INTR_STAT_R_TX_ABRT)  | \
                         (RP2040_I2C_IC_INTR_STAT_R_TX_OVER)  | \
                         (RP2040_I2C_IC_INTR_STAT_R_RX_OVER)  | \
                         (RP2040_I2C_IC_INTR_STAT_R_RX_UNDER))

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct rp2040_i2cdev_s
{
  struct i2c_master_s dev;     /* Generic I2C device */
  unsigned int     base;       /* Base address of registers */
  uint16_t         irqid;      /* IRQ for this device */
  int8_t           port;       /* Port number */
  uint32_t         base_freq;  /* branch frequency */

  sem_t            mutex;      /* Only one thread can access at a time */
  sem_t            wait;       /* Place to wait for transfer completion */
  struct wdog_s    timeout;    /* watchdog to timeout when bus hung */
  uint32_t         frequency;  /* Current I2C frequency */
  ssize_t          reg_buff_offset;
  ssize_t          rw_size;

  struct i2c_msg_s *msgs;

  int              error;      /* Error status of each transfers */
  int              refs;       /* Reference count */
};

#ifdef CONFIG_RP2040_I2C0
static struct rp2040_i2cdev_s g_i2c0dev =
{
  .port = 0,
  .base = RP2040_I2C0_BASE,
  .irqid = RP2040_I2C0_IRQ,
  .refs = 0,
};
#endif
#ifdef CONFIG_RP2040_I2C1
static struct rp2040_i2cdev_s g_i2c1dev =
{
  .port = 1,
  .base = RP2040_I2C1_BASE,
  .irqid = RP2040_I2C1_IRQ,
  .refs = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int i2c_takesem(FAR sem_t *sem);
static inline int i2c_givesem(FAR sem_t *sem);

static inline uint32_t i2c_reg_read(struct rp2040_i2cdev_s *priv,
                                    uint32_t offset);
static inline void i2c_reg_write(struct rp2040_i2cdev_s *priv,
                                 uint32_t offset,
                                 uint32_t val);
static inline void i2c_reg_rmw(struct rp2040_i2cdev_s *dev,
                               uint32_t offset,
                               uint32_t val, uint32_t mask);

static int rp2040_i2c_disable(struct rp2040_i2cdev_s *priv);
static void rp2040_i2c_init(struct rp2040_i2cdev_s *priv);
static void rp2040_i2c_enable(struct rp2040_i2cdev_s *priv);

static int  rp2040_i2c_interrupt(int irq, FAR void *context, FAR void *arg);
static void rp2040_i2c_timeout(wdparm_t arg);
static void rp2040_i2c_setfrequency(struct rp2040_i2cdev_s *priv,
                                   uint32_t frequency);
static int  rp2040_i2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int rp2040_i2c_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Name: i2c_takesem
 ****************************************************************************/

static inline int i2c_takesem(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: i2c_givesem
 ****************************************************************************/

static inline int i2c_givesem(FAR sem_t *sem)
{
  return nxsem_post(sem);
}

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

struct i2c_ops_s rp2040_i2c_ops =
{
  .transfer = rp2040_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = rp2040_i2c_reset,
#endif
};

/****************************************************************************
 * Name: rp2040_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void rp2040_i2c_setfrequency(struct rp2040_i2cdev_s *priv,
                                   uint32_t frequency)
{
  int32_t lcnt;
  int32_t hcnt;
  uint64_t lcnt64;
  uint64_t hcnt64;
  uint64_t speed;
  uint64_t t_low;
  uint64_t t_high;
  uint32_t base = BOARD_PERI_FREQ;
  uint32_t spklen;

  ASSERT(base);

  if ((priv->frequency == frequency) && (priv->base_freq == base))
    {
      return;
    }

  priv->frequency = frequency;
  priv->base_freq = base;

  base /= 1000;

  if (frequency <= 100000)
    {
      t_low  = 4700000;
      t_high = 4000000;
    }
  else if (frequency <= 400000)
    {
      t_low  = 1300000;
      t_high = 600000;
    }
  else
    {
      t_low  = 500000;
      t_high = 260000;
    }

  if (frequency > 100000)
    {
      if (base < 20032)
        {
          spklen = 1;
        }
      else if (base < 40064)
        {
          spklen = 2;
        }
      else
        {
          spklen = 3;
        }
    }
  else
    {
      spklen = 1;
    }

  lcnt64 = (t_low + 6500ull / 20000ull) * base;
  lcnt   = ((lcnt64 + 999999999ull) / 1000000000ull) - 1; /* ceil */
  lcnt   = lcnt < 8 ? 8 : lcnt;

  hcnt64 = (t_high - 6500ull) * base;
  hcnt   = ((hcnt64 + 999999999ull) / 1000000000ull) - 6 - spklen; /* ceil */
  hcnt   = hcnt < 6 ? 6 : hcnt;

  speed =
    1000000000000000000ull /
    (((lcnt + 1) * 1000000000000ull +
    (hcnt + 6 + spklen) * 1000000000000ull) / base +
     20000ull / 1000ull * 1000000ull);

  if (speed > (frequency * 1000ull))
    {
      uint64_t adj;
      adj = ((1000000000000000000ull / (frequency * 1000ull)) -
             (1000000000000000000ull / speed)) *
            base;
      hcnt += (adj + 999999999999ull) / 1000000000000ull;
    }

  /* use FS register in SS and FS mode */

  i2c_reg_write(priv, RP2040_I2C_IC_FS_SCL_HCNT_OFFSET, hcnt);
  i2c_reg_write(priv, RP2040_I2C_IC_FS_SCL_LCNT_OFFSET, lcnt);
  i2c_reg_rmw(priv, RP2040_I2C_IC_CON_OFFSET,
                    RP2040_I2C_IC_CON_SPEED_FAST,
                    RP2040_I2C_IC_CON_SPEED_MASK);

  i2c_reg_write(priv, RP2040_I2C_IC_FS_SPKLEN_OFFSET, spklen);
}

/****************************************************************************
 * Name: rp2040_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void rp2040_i2c_timeout(wdparm_t arg)
{
  struct rp2040_i2cdev_s *priv = (struct rp2040_i2cdev_s *)arg;
  irqstate_t flags             = enter_critical_section();

  priv->error = -ENODEV;
  i2c_givesem(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rp2040_i2c_drainrxfifo
 *
 * Description:
 *   Receive I2C data
 *
 ****************************************************************************/

static void rp2040_i2c_drainrxfifo(struct rp2040_i2cdev_s *priv)
{
  struct i2c_msg_s *msg = priv->msgs;
  uint32_t status;
  uint32_t dat;
  ssize_t i;

  DEBUGASSERT(msg != NULL);

  status = i2c_reg_read(priv, RP2040_I2C_IC_STATUS_OFFSET);

  for (i = 0; i < priv->rw_size && status & RP2040_I2C_IC_STATUS_RFNE; i++)
    {
      dat            = i2c_reg_read(priv, RP2040_I2C_IC_DATA_CMD_OFFSET);
      msg->buffer[priv->reg_buff_offset + i] = dat & 0xff;
      status         = i2c_reg_read(priv, RP2040_I2C_IC_STATUS_OFFSET);
    }

  priv->reg_buff_offset += priv->rw_size;
}

/****************************************************************************
 * Name: rp2040_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int rp2040_i2c_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rp2040_i2cdev_s *priv = (FAR struct rp2040_i2cdev_s *)arg;
  uint32_t state;
  int ret;

  state = i2c_reg_read(priv, RP2040_I2C_IC_INTR_STAT_OFFSET);

  if (state & RP2040_I2C_IC_INTR_STAT_R_TX_ABRT)
    {
      i2c_reg_read(priv, RP2040_I2C_IC_CLR_TX_ABRT_OFFSET);
      priv->error = -ENODEV;
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_TX_OVER)
    {
      i2c_reg_read(priv, RP2040_I2C_IC_CLR_TX_OVER_OFFSET);
      priv->error = -EIO;
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_RX_OVER)
    {
      i2c_reg_read(priv, RP2040_I2C_IC_CLR_RX_OVER_OFFSET);
      priv->error = -EIO;
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_RX_UNDER)
    {
      i2c_reg_read(priv, RP2040_I2C_IC_CLR_RX_UNDER_OFFSET);
      priv->error = -EIO;
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_TX_EMPTY)
    {
      /* TX_EMPTY is automatically cleared by hardware
       * when the buffer level goes above the threshold.
       */

      i2c_reg_rmw(priv, RP2040_I2C_IC_INTR_MASK_OFFSET,
                  0, RP2040_I2C_IC_INTR_MASK_M_TX_EMPTY);
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_RX_FULL)
    {
      /* RX_FULL is automatically cleared by hardware
       * when the buffer level goes below the threshold.
       */

      i2c_reg_rmw(priv, RP2040_I2C_IC_INTR_MASK_OFFSET,
                  0, RP2040_I2C_IC_INTR_MASK_M_RX_FULL);
      rp2040_i2c_drainrxfifo(priv);
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_STOP_DET)
    {
      i2c_reg_read(priv, RP2040_I2C_IC_CLR_STOP_DET_OFFSET);
    }

  if ((priv->error) || (state & RP2040_I2C_IC_INTR_STAT_R_TX_EMPTY) ||
                       (state & RP2040_I2C_IC_INTR_STAT_R_RX_FULL))
    {
      /* Failure of wd_cancel() means that the timer expired.
       * In this case, nxsem_post() has already been called.
       * Therefore, call nxsem_post() only when wd_cancel() succeeds.
       */

      ret = wd_cancel(&priv->timeout);
      if (ret == OK)
        {
          i2c_givesem(&priv->wait);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rp2040_i2c_receive
 *
 * Description:
 *   Receive data from I2C bus.
 *   Prohibit all interrupt because the STOP condition might happen
 *   if the interrupt occurs when the writing request.
 *   Actual receiving data is in RX_FULL interrupt handler.
 *
 * TODO : The argument "last" is not used.
 ****************************************************************************/

static int rp2040_i2c_receive(struct rp2040_i2cdev_s *priv, int last)
{
  struct i2c_msg_s *msg = priv->msgs;
  int i;
  int en;
  ssize_t msg_length;
  irqstate_t flags;

  priv->reg_buff_offset = 0;

  DEBUGASSERT(msg != NULL);

  for (msg_length = msg->length; msg_length > 0; msg_length -= priv->rw_size)
    {
      if (msg_length <= I2C_FIFO_MAX_SIZE)
        {
          priv->rw_size = msg_length;
          en = 1;
        }
      else
        {
          priv->rw_size = I2C_FIFO_MAX_SIZE;
          en = 0;
        }

      /* update threshold value of the receive buffer */

      i2c_reg_write(priv, RP2040_I2C_IC_RX_TL_OFFSET, priv->rw_size - 1);

      for (i = 0; i < priv->rw_size - 1; i++)
        {
          i2c_reg_write(priv, RP2040_I2C_IC_DATA_CMD_OFFSET,
                              RP2040_I2C_IC_DATA_CMD_CMD);
        }

      flags = enter_critical_section();
      wd_start(&priv->timeout, I2C_TIMEOUT,
               rp2040_i2c_timeout, (wdparm_t)priv);

      /* Set stop flag for indicate the last data */

      i2c_reg_write(priv, RP2040_I2C_IC_DATA_CMD_OFFSET,
                          RP2040_I2C_IC_DATA_CMD_CMD |
                          (en ? RP2040_I2C_IC_DATA_CMD_STOP : 0));

      i2c_reg_rmw(priv, RP2040_I2C_IC_INTR_MASK_OFFSET,
                  RP2040_I2C_IC_INTR_STAT_R_RX_FULL,
                  RP2040_I2C_IC_INTR_STAT_R_RX_FULL);
      leave_critical_section(flags);
      i2c_takesem(&priv->wait);

      if (priv->error != OK)
        {
          break;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: rp2040_i2c_send
 *
 * Description:
 *   Send data to I2C bus.
 *
 ****************************************************************************/

static int rp2040_i2c_send(struct rp2040_i2cdev_s *priv, int last)
{
  struct i2c_msg_s *msg = priv->msgs;
  ssize_t i;
  irqstate_t flags;

  DEBUGASSERT(msg != NULL);

  for (i = 0; i < msg->length - 1; i++)
    {
      while (!(i2c_reg_read(priv, RP2040_I2C_IC_STATUS_OFFSET)
               & RP2040_I2C_IC_STATUS_TFNF))
        ;

      i2c_reg_write(priv, RP2040_I2C_IC_DATA_CMD_OFFSET,
                    (uint32_t)msg->buffer[i]);
    }

  while (!(i2c_reg_read(priv, RP2040_I2C_IC_STATUS_OFFSET)
           & RP2040_I2C_IC_STATUS_TFNF))
    ;

  flags = enter_critical_section();
  wd_start(&priv->timeout, I2C_TIMEOUT,
           rp2040_i2c_timeout, (wdparm_t)priv);
  i2c_reg_write(priv, RP2040_I2C_IC_DATA_CMD_OFFSET,
                (uint32_t)msg->buffer[i] |
                (last ? RP2040_I2C_IC_DATA_CMD_STOP : 0));

  /* Enable TX_EMPTY interrupt for determine transfer done. */

  i2c_reg_rmw(priv, RP2040_I2C_IC_INTR_MASK_OFFSET,
              RP2040_I2C_IC_INTR_STAT_R_TX_EMPTY,
              RP2040_I2C_IC_INTR_STAT_R_TX_EMPTY);
  leave_critical_section(flags);

  i2c_takesem(&priv->wait);

  return 0;
}

/****************************************************************************
 * Name: rp2040_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 * TODO: Multiple i2c_msg_s read operations with the same address are not
 * currently guaranteed.
 ****************************************************************************/

static int rp2040_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  struct rp2040_i2cdev_s *priv = (struct rp2040_i2cdev_s *)dev;
  int i;
  int ret    = 0;
  int semval = 0;
  int addr = -1;
  static int wostop = 0;

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  i2c_takesem(&priv->mutex);

  /* Check wait semaphore value. If the value is not 0, the transfer can not
   * be performed normally.
   */

  ret = nxsem_get_value(&priv->wait, &semval);
  DEBUGASSERT(ret == OK && semval == 0);

  for (i = 0; i < count; i++, msgs++)
    {
      /* Pass msg descriptor via device context */

      priv->msgs  = msgs;
      priv->error = OK;

      if ((addr != msgs->addr) && !wostop)
        {
          rp2040_i2c_disable(priv);

          rp2040_i2c_setfrequency(priv, msgs->frequency);

          i2c_reg_rmw(priv, RP2040_I2C_IC_CON_OFFSET,
                      RP2040_I2C_IC_CON_IC_RESTART_EN,
                      RP2040_I2C_IC_CON_IC_RESTART_EN);
          i2c_reg_write(priv, RP2040_I2C_IC_TAR_OFFSET, msgs->addr & 0x7f);

          rp2040_i2c_enable(priv);
          addr = msgs->addr;
        }

      if (msgs->flags & I2C_M_NOSTOP)
        {
          /* Don't send stop condition even if the last data */

          wostop = 1;
        }
      else
        {
          wostop = 0;
        }

      if (msgs->flags & I2C_M_READ)
        {
          ret = rp2040_i2c_receive(priv, (wostop) ? 0 : (i + 1 == count));
        }
      else
        {
          ret = rp2040_i2c_send(priv, (wostop) ? 0 : (i + 1 == count));
        }

      if (ret < 0)
        {
          break;
        }

      if (priv->error != OK)
        {
          ret = priv->error;
          break;
        }

      /* Clear msg descriptor for prevent illegal access in interrupt */

      priv->msgs = NULL;
    }

  if (!wostop)
    {
      rp2040_i2c_disable(priv);
    }

  i2c_givesem(&priv->mutex);

  return ret;
}

/****************************************************************************
 * Name: rp2040_i2c_reset
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
static int rp2040_i2c_reset(FAR struct i2c_master_s *dev)
{
  FAR struct rp2040_i2cdev_s *priv = (struct rp2040_i2cdev_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t subsys;
  uint32_t frequency;
  int pin;
  int ret;

  DEBUGASSERT(dev);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  i2c_takesem(&priv->mutex);

  ret = -EIO;

  /* De-init the port */

  rp2040_i2c_disable(priv);

  /* Use GPIO configuration to un-wedge the bus */

  pin = rp2040_gpio_get_function_pin(RP2040_GPIO_FUNC_I2C, priv->port);
  if (pin < 0)
    {
      goto out_without_reinit;
    }

  sda_gpio = pin;
  scl_gpio = pin + 1;
  subsys = (priv->port == 0) ? RP2040_RESETS_RESET_I2C0 :
                               RP2040_RESETS_RESET_I2C1;

  rp2040_gpio_init(sda_gpio);
  rp2040_gpio_setdir(sda_gpio, true);
  rp2040_gpio_set_pulls(sda_gpio, true, false);  /* Pull up */
  rp2040_gpio_put(sda_gpio, true);

  rp2040_gpio_init(scl_gpio);
  rp2040_gpio_setdir(scl_gpio, true);
  rp2040_gpio_set_pulls(scl_gpio, true, false);
  rp2040_gpio_put(scl_gpio, true);

  /* Let SDA go high */

  rp2040_gpio_put(sda_gpio, true);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!rp2040_gpio_get(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished. If the bus
       * never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!rp2040_gpio_get(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      rp2040_gpio_put(scl_gpio, false);
      up_udelay(10);

      /* Drive SCL high again */

      rp2040_gpio_put(scl_gpio, true);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave state machines. */

  rp2040_gpio_put(sda_gpio, false);
  up_udelay(10);
  rp2040_gpio_put(scl_gpio, false);
  up_udelay(10);
  rp2040_gpio_put(scl_gpio, true);
  up_udelay(10);
  rp2040_gpio_put(sda_gpio, true);
  up_udelay(10);

  ret = OK;

out:

  /* Revert the GPIO configuration. */

  rp2040_gpio_set_function(sda_gpio, RP2040_GPIO_FUNC_I2C);
  rp2040_gpio_set_function(scl_gpio, RP2040_GPIO_FUNC_I2C);

  /* Reset I2C subsystem */

  setbits_reg32(subsys, RP2040_RESETS_RESET);
  clrbits_reg32(subsys, RP2040_RESETS_RESET);
  while ((getreg32(RP2040_RESETS_RESET_DONE) & subsys) == 0)
    ;

  /* Re-init the port */

  rp2040_i2c_disable(priv);
  rp2040_i2c_init(priv);

  /* Restore the frequency */

  frequency = priv->frequency;
  priv->frequency = 0;
  rp2040_i2c_setfrequency(priv, frequency);

out_without_reinit:

  /* Release the port for re-use by other clients */

  i2c_givesem(&priv->mutex);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

static inline uint32_t i2c_reg_read(struct rp2040_i2cdev_s *priv,
                                    uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void i2c_reg_write(struct rp2040_i2cdev_s *priv,
                                 uint32_t offset, uint32_t val)
{
  putreg32(val, priv->base + offset);
}

static inline void i2c_reg_rmw(struct rp2040_i2cdev_s *priv, uint32_t offset,
                               uint32_t val, uint32_t mask)
{
  modbits_reg32(val, mask, priv->base + offset);
}

static int rp2040_i2c_disable(struct rp2040_i2cdev_s *priv)
{
  int retry = 25000;
  uint32_t stat;

  /* disable all interrupt */

  i2c_reg_write(priv, RP2040_I2C_IC_INTR_MASK_OFFSET, 0x0);

  /* clear all interrupt status */

  i2c_reg_read(priv, RP2040_I2C_IC_CLR_INTR_OFFSET);
  i2c_reg_write(priv, RP2040_I2C_IC_ENABLE_OFFSET, 0);

  do
    {
      stat = i2c_reg_read(priv, RP2040_I2C_IC_ENABLE_STATUS_OFFSET);
    }
  while (--retry && (stat & RP2040_I2C_IC_ENABLE_STATUS_IC_EN));

  if (!retry)
    {
      i2cerr("i2c wait timeout.\n");
      return -EBUSY;
    }

  /* clear all interrupt status again */

  i2c_reg_read(priv, RP2040_I2C_IC_CLR_INTR_OFFSET);

  return 0;
}

static void rp2040_i2c_init(struct rp2040_i2cdev_s *priv)
{
  i2c_reg_write(priv, RP2040_I2C_IC_INTR_MASK_OFFSET, 0x00);
  i2c_reg_read(priv, RP2040_I2C_IC_CLR_INTR_OFFSET);

  /* set threshold level of the Rx/Tx FIFO */

  i2c_reg_write(priv, RP2040_I2C_IC_RX_TL_OFFSET, 0xff);
  i2c_reg_write(priv, RP2040_I2C_IC_TX_TL_OFFSET, 0);

  /* set hold time for margin */

  i2c_reg_write(priv, RP2040_I2C_IC_SDA_HOLD_OFFSET, 1);

  i2c_reg_write(priv, RP2040_I2C_IC_CON_OFFSET,
                (RP2040_I2C_IC_CON_IC_SLAVE_DISABLE |
                 RP2040_I2C_IC_CON_MASTER_MODE |
                 RP2040_I2C_IC_CON_TX_EMPTY_CTRL));
}

static void rp2040_i2c_enable(struct rp2040_i2cdev_s *priv)
{
  i2c_reg_write(priv, RP2040_I2C_IC_INTR_MASK_OFFSET, I2C_INTR_ENABLE);
  i2c_reg_write(priv, RP2040_I2C_IC_ENABLE_OFFSET, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *rp2040_i2cbus_initialize(int port)
{
  struct rp2040_i2cdev_s *priv;

  irqstate_t flags;

  flags = enter_critical_section();

#ifdef CONFIG_RP2040_I2C0
  if (port == 0)
    {
      priv        = &g_i2c0dev;
      priv->dev.ops = &rp2040_i2c_ops;
    }
  else
#endif
#ifdef CONFIG_RP2040_I2C1
  if (port == 1)
    {
      priv        = &g_i2c1dev;
      priv->dev.ops = &rp2040_i2c_ops;
    }
  else
#endif
    {
      leave_critical_section(flags);
      i2cerr("I2C Only support 0,1\n");
      return NULL;
    }

  priv->refs++;

  /* Test if already initialized or not */

  if (1 < priv->refs)
    {
      leave_critical_section(flags);
      return &priv->dev;
    }

  priv->port      = port;
  priv->frequency = 0;

  priv->base_freq = BOARD_PERI_FREQ;

  rp2040_i2c_disable(priv);
  rp2040_i2c_init(priv);
  rp2040_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

  leave_critical_section(flags);

  nxsem_init(&priv->mutex, 0, 1);
  nxsem_init(&priv->wait, 0, 0);
  nxsem_set_protocol(&priv->wait, SEM_PRIO_NONE);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, rp2040_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  return &priv->dev;
}

/****************************************************************************
 * Name: rp2040_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int rp2040_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  struct rp2040_i2cdev_s *priv = (struct rp2040_i2cdev_s *)dev;

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  if (--priv->refs)
    {
      return OK;
    }

  rp2040_i2c_disable(priv);

  up_disable_irq(priv->irqid);
  irq_detach(priv->irqid);

  wd_cancel(&priv->timeout);
  nxsem_destroy(&priv->mutex);
  nxsem_destroy(&priv->wait);

  return OK;
}

#endif /* CONFIG_RP2040_I2C */
