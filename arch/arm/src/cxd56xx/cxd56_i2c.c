/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_i2c.c
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
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_clock.h"
#include "cxd56_i2c.h"
#include "hardware/cxd56_i2c.h"
#include "cxd56_pinconfig.h"

#if defined(CONFIG_CXD56_I2C0_SCUSEQ) || defined(CONFIG_CXD56_I2C1_SCUSEQ)
#include <arch/chip/scu.h>
#endif

#ifdef CONFIG_CXD56_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_TIMEOUT  (20*1000/CONFIG_USEC_PER_TICK) /* 20 mS */

#define I2C_DEFAULT_FREQUENCY 400000
#define I2C_FIFO_MAX_SIZE	    32

#define I2C_INTR_ENABLE ((INTR_STOP_DET) | \
                         (INTR_TX_ABRT)  | \
                         (INTR_TX_OVER)  | \
                         (INTR_RX_OVER)  | \
                         (INTR_RX_UNDER))

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct cxd56_i2cdev_s
{
  struct i2c_master_s dev;     /* Generic I2C device */
  unsigned int     base;       /* Base address of registers */
  uint16_t         irqid;      /* IRQ for this device */
  int8_t           port;       /* Port number */
  uint32_t         base_freq;  /* branch frequency */

  mutex_t          lock;       /* Only one thread can access at a time */
  sem_t            wait;       /* Place to wait for transfer completion */
  struct wdog_s    timeout;    /* watchdog to timeout when bus hung */
  uint32_t         frequency;  /* Current I2C frequency */
  ssize_t          reg_buff_offset;
  ssize_t          rw_size;

  struct i2c_msg_s *msgs;

  int              error;      /* Error status of each transfers */
  int              refs;       /* Reference count */
};

/* Channel 0 as SCU_I2C0
 * Channel 1 as SCU_I2C1
 * Channel 2 as I2CM
 */

#ifdef CONFIG_CXD56_I2C0
static struct cxd56_i2cdev_s g_i2c0dev =
{
  .port = 0,
  .base = CXD56_SCU_I2C0_BASE,
  .irqid = CXD56_IRQ_SCU_I2C0,
  .lock = NXMUTEX_INITIALIZER,
  .wait = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  .refs = 0,
};
#endif
#ifdef CONFIG_CXD56_I2C1
static struct cxd56_i2cdev_s g_i2c1dev =
{
  .port = 1,
  .base = CXD56_SCU_I2C1_BASE,
  .irqid = CXD56_IRQ_SCU_I2C1,
  .lock = NXMUTEX_INITIALIZER,
  .wait = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  .refs = 0,
};
#endif
#ifdef CONFIG_CXD56_I2C2
static struct cxd56_i2cdev_s g_i2c2dev =
{
  .port = 2,
  .base = CXD56_I2CM_BASE,
  .irqid = CXD56_IRQ_I2CM,
  .lock = NXMUTEX_INITIALIZER,
  .wait = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  .refs = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t i2c_reg_read(struct cxd56_i2cdev_s *priv,
                                    uint32_t offset);
static inline void i2c_reg_write(struct cxd56_i2cdev_s *priv,
                                 uint32_t offset,
                                 uint32_t val);
static inline void i2c_reg_rmw(struct cxd56_i2cdev_s *dev,
                               uint32_t offset,
                               uint32_t val, uint32_t mask);

static int cxd56_i2c_disable(struct cxd56_i2cdev_s *priv);
static void cxd56_i2c_enable(struct cxd56_i2cdev_s *priv);

static int  cxd56_i2c_interrupt(int irq, void *context, void *arg);
static void cxd56_i2c_timeout(wdparm_t arg);
static void cxd56_i2c_setfrequency(struct cxd56_i2cdev_s *priv,
                                   uint32_t frequency);
static int  cxd56_i2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int cxd56_i2c_reset(struct i2c_master_s *dev);
#endif
#if defined(CONFIG_CXD56_I2C0_SCUSEQ) || defined(CONFIG_CXD56_I2C1_SCUSEQ)
static int  cxd56_i2c_transfer_scu(struct i2c_master_s *dev,
                                   struct i2c_msg_s *msgs, int count);
#endif

/****************************************************************************
 * Name: cxd56_i2c_pincontrol
 *
 * Description:
 *   Configure the I2C pin
 *
 * Input Parameter:
 *   on - true: enable pin, false: disable pin
 *
 ****************************************************************************/

static void cxd56_i2c_pincontrol(int ch, bool on)
{
  switch (ch)
    {
#ifdef CONFIG_CXD56_I2C0
      case 0:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_I2C0);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_I2C0_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_I2C0 */

#ifdef CONFIG_CXD56_I2C1
      case 1:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_PWMB_I2C1);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_PWMB_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_I2C1 */

#ifdef CONFIG_CXD56_I2C2
      case 2:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI0B_I2C2);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI0B_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_I2C2 */

      default:
        break;
    }
}

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

struct i2c_ops_s cxd56_i2c_ops =
{
  .transfer = cxd56_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = cxd56_i2c_reset,
#endif
};

#if defined(CONFIG_CXD56_I2C0_SCUSEQ) || defined(CONFIG_CXD56_I2C1_SCUSEQ)
struct i2c_ops_s cxd56_i2c_scu_ops =
{
  .transfer = cxd56_i2c_transfer_scu,
#ifdef CONFIG_I2C_RESET
  .reset = cxd56_i2c_reset,
#endif
};
#endif

/****************************************************************************
 * Name: cxd56_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void cxd56_i2c_setfrequency(struct cxd56_i2cdev_s *priv,
                                   uint32_t frequency)
{
  int32_t lcnt;
  int32_t hcnt;
  uint64_t lcnt64;
  uint64_t hcnt64;
  uint64_t speed;
  uint64_t t_low;
  uint64_t t_high;
  uint32_t base = cxd56_get_i2c_baseclock(priv->port);
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

  i2c_reg_write(priv, CXD56_IC_FS_SCL_HCNT, hcnt);
  i2c_reg_write(priv, CXD56_IC_FS_SCL_LCNT, lcnt);
  i2c_reg_rmw(priv, CXD56_IC_CON, IC_SPEED_FS, IC_MAX_SPEED_MODE);

  i2c_reg_write(priv, CXD56_IC_FS_SPKLEN, spklen);
}

/****************************************************************************
 * Name: cxd56_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void cxd56_i2c_timeout(wdparm_t arg)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)arg;
  irqstate_t flags            = enter_critical_section();

  priv->error = -ENODEV;
  nxsem_post(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_i2c_drainrxfifo
 *
 * Description:
 *   Receive I2C data
 *
 ****************************************************************************/

static void cxd56_i2c_drainrxfifo(struct cxd56_i2cdev_s *priv)
{
  struct i2c_msg_s *msg = priv->msgs;
  uint32_t status;
  uint32_t dat;
  ssize_t i;

  DEBUGASSERT(msg != NULL);

  status = i2c_reg_read(priv, CXD56_IC_STATUS);

  for (i = 0; i < priv->rw_size && status & STATUS_RFNE; i++)
    {
      dat            = i2c_reg_read(priv, CXD56_IC_DATA_CMD);
      msg->buffer[priv->reg_buff_offset + i] = dat & 0xff;
      status         = i2c_reg_read(priv, CXD56_IC_STATUS);
    }

  priv->reg_buff_offset += priv->rw_size;
}

/****************************************************************************
 * Name: cxd56_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int cxd56_i2c_interrupt(int irq, void *context, void *arg)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)arg;
  uint32_t state;
  int ret;

  state = i2c_reg_read(priv, CXD56_IC_INTR_STAT);

  if (state & INTR_TX_ABRT)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_TX_ABRT);
      priv->error = -ENODEV;
    }

  if (state & INTR_TX_OVER)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_TX_OVER);
      priv->error = -EIO;
    }

  if (state & INTR_RX_OVER)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_RX_OVER);
      priv->error = -EIO;
    }

  if (state & INTR_RX_UNDER)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_RX_UNDER);
      priv->error = -EIO;
    }

  if (state & INTR_TX_EMPTY)
    {
      /* TX_EMPTY is automatically cleared by hardware
       * when the buffer level goes above the threshold.
       */

      i2c_reg_rmw(priv, CXD56_IC_INTR_MASK, 0, INTR_TX_EMPTY);
    }

  if (state & INTR_RX_FULL)
    {
      /* RX_FULL is automatically cleared by hardware
       * when the buffer level goes below the threshold.
       */

      i2c_reg_rmw(priv, CXD56_IC_INTR_MASK, 0, INTR_RX_FULL);
      cxd56_i2c_drainrxfifo(priv);
    }

  if (state & INTR_STOP_DET)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_STOP_DET);
    }

  if ((priv->error) || (state & INTR_TX_EMPTY) || (state & INTR_RX_FULL))
    {
      /* Failure of wd_cancel() means that the timer expired.
       * In this case, nxsem_post() has already been called.
       * Therefore, call nxsem_post() only when wd_cancel() succeeds.
       */

      ret = wd_cancel(&priv->timeout);
      if (ret == OK)
        {
          nxsem_post(&priv->wait);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_i2c_receive
 *
 * Description:
 *   Receive data from I2C bus.
 *   Prohibit all interrupt because the STOP condition might happen
 *   if the interrupt occurs when the writing request.
 *   Actual receiving data is in RX_FULL interrupt handler.
 *
 * TODO : The argument "last" is not used.
 ****************************************************************************/

static int cxd56_i2c_receive(struct cxd56_i2cdev_s *priv, int last)
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

      i2c_reg_write(priv, CXD56_IC_RX_TL, priv->rw_size - 1);

      for (i = 0; i < priv->rw_size - 1; i++)
        {
          i2c_reg_write(priv, CXD56_IC_DATA_CMD, CMD_READ);
        }

      flags = enter_critical_section();
      wd_start(&priv->timeout, I2C_TIMEOUT,
               cxd56_i2c_timeout, (wdparm_t)priv);

      /* Set stop flag for indicate the last data */

      i2c_reg_write(priv, CXD56_IC_DATA_CMD, CMD_READ | (en ? CMD_STOP : 0));

      i2c_reg_rmw(priv, CXD56_IC_INTR_MASK, INTR_RX_FULL, INTR_RX_FULL);
      leave_critical_section(flags);
      nxsem_wait_uninterruptible(&priv->wait);

      if (priv->error != OK)
        {
          break;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: cxd56_i2c_send
 *
 * Description:
 *   Send data to I2C bus.
 *
 ****************************************************************************/

static int cxd56_i2c_send(struct cxd56_i2cdev_s *priv, int last)
{
  struct i2c_msg_s *msg = priv->msgs;
  ssize_t i;
  irqstate_t flags;

  DEBUGASSERT(msg != NULL);

  for (i = 0; i < msg->length - 1; i++)
    {
      while (!(i2c_reg_read(priv, CXD56_IC_STATUS) & STATUS_TFNF));

      i2c_reg_write(priv, CXD56_IC_DATA_CMD, (uint32_t)msg->buffer[i]);
    }

  while (!(i2c_reg_read(priv, CXD56_IC_STATUS) & STATUS_TFNF));

  flags = enter_critical_section();
  wd_start(&priv->timeout, I2C_TIMEOUT,
           cxd56_i2c_timeout, (wdparm_t)priv);
  i2c_reg_write(priv, CXD56_IC_DATA_CMD,
                (uint32_t)msg->buffer[i] | (last ? CMD_STOP : 0));

  /* Enable TX_EMPTY interrupt for determine transfer done. */

  i2c_reg_rmw(priv, CXD56_IC_INTR_MASK, INTR_TX_EMPTY, INTR_TX_EMPTY);
  leave_critical_section(flags);

  nxsem_wait_uninterruptible(&priv->wait);
  return 0;
}

/****************************************************************************
 * Name: cxd56_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 * TODO: Multiple i2c_msg_s read operations with the same address are not
 * currently guaranteed.
 ****************************************************************************/

static int cxd56_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)dev;
  int i;
  int ret    = 0;
  int semval = 0;
  int addr = -1;
  static int wostop = 0;

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  nxmutex_lock(&priv->lock);

  /* Check wait semaphore value. If the value is not 0, the transfer can not
   * be performed normally.
   */

  ret = nxsem_get_value(&priv->wait, &semval);
  DEBUGASSERT(ret == OK && semval == 0);

  /* Disable clock gating (clock enable) */

  cxd56_i2c_clock_gate_disable(priv->port);

  for (i = 0; i < count; i++, msgs++)
    {
      /* Pass msg descriptor via device context */

      priv->msgs  = msgs;
      priv->error = OK;

      if ((addr != msgs->addr) && !wostop)
        {
          cxd56_i2c_disable(priv);

          cxd56_i2c_setfrequency(priv, msgs->frequency);

          i2c_reg_rmw(priv, CXD56_IC_CON, IC_RESTART_EN, IC_RESTART_EN);
          i2c_reg_write(priv, CXD56_IC_TAR, msgs->addr & 0x7f);

          cxd56_i2c_enable(priv);
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
          ret = cxd56_i2c_receive(priv, (wostop) ? 0 : (i + 1 == count));
        }
      else
        {
          ret = cxd56_i2c_send(priv, (wostop) ? 0 : (i + 1 == count));
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
      cxd56_i2c_disable(priv);
    }

  /* Enable clock gating (clock disable) */

  cxd56_i2c_clock_gate_enable(priv->port);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: cxd56_i2c_reset
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
static int cxd56_i2c_reset(struct i2c_master_s *dev)
{
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Name: cxd56_i2c_transfer_scu
 *
 * Description:
 *   Perform a sequence of I2C transfers with scu oneshot sequencer.
 *
 ****************************************************************************/

#if defined(CONFIG_CXD56_I2C0_SCUSEQ) || defined(CONFIG_CXD56_I2C1_SCUSEQ)

static int cxd56_i2c_scurecv(int port, int addr,
                             uint8_t *buf, ssize_t buflen)
{
  uint16_t inst[2];
  int      instn;
  int      len0;
  int      len1;
  ssize_t  rem;
  int      ret = OK;

  /* Ignore buffer is NULL */

  if (buf == NULL)
    {
      return OK;
    }

  if (buflen > 16)
    {
      return -EINVAL;
    }

  rem = buflen;
  len0 = rem > 8 ? 8 : rem;
  rem -= len0;
  len1 = rem > 8 ? 8 : rem;
  rem -= len1;

  inst[0] = SCU_INST_RECV(len0);
  if (len1)
    {
      inst[1] = SCU_INST_RECV(len1);
      instn = 2;
    }
  else
    {
      instn = 1;
    }

  inst[instn - 1] |= SCU_INST_LAST;

  ret = scu_i2ctransfer(port, addr, inst, instn, buf, buflen);
  if (ret < 0)
    {
      syslog(LOG_ERR, "I2C receive failed. port %d addr %d\n",
             port, addr);
    }

  return ret;
}

static int cxd56_i2c_scusend(int port, int addr,
                             uint8_t *buf, ssize_t buflen)
{
  uint16_t inst[12];
  ssize_t  rem;
  int      i;
  int      ret = OK;

  rem = buflen;

  while (rem)
    {
      for (i = 0; i < 12 && rem > 0; i++)
        {
          inst[i] = SCU_INST_SEND(*buf++);
          rem--;
        }

      if (rem == 0)
        {
          inst[i - 1] |= SCU_INST_LAST;
        }

      if (i > 0)
        {
          ret = scu_i2ctransfer(port, addr, inst, i, NULL, i);
          if (ret < 0)
            {
              syslog(LOG_ERR, "I2C send failed. port %d addr %d\n",
                     port, addr);
              break;
            }
        }
    }

  return ret;
}

static int cxd56_i2c_transfer_scu(struct i2c_master_s *dev,
                                  struct i2c_msg_s *msgs, int count)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)dev;
  ssize_t  len  = 0;
  uint8_t *buf  = NULL;
  uint8_t  addr = msgs->addr;
  int      i;
  int      ret  = 0;

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  nxmutex_lock(&priv->lock);

  /* Apply frequency for request msgs */

  if (priv->frequency != msgs->frequency)
    {
      cxd56_i2c_clock_gate_disable(priv->port);
      cxd56_i2c_disable(priv);
      cxd56_i2c_setfrequency(priv, msgs->frequency);
      cxd56_i2c_clock_gate_enable(priv->port);

      priv->frequency = msgs->frequency;
    }

  for (i = 0; i < count; i++, msgs++)
    {
      len = msgs->length;
      buf = msgs->buffer;

      if (msgs->flags & I2C_M_READ)
        {
          ret = cxd56_i2c_scurecv(priv->port, addr, buf, len);
        }
      else
        {
          ret = cxd56_i2c_scusend(priv->port, addr, buf, len);
        }

      if (ret < 0)
        {
          break;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

static inline uint32_t i2c_reg_read(struct cxd56_i2cdev_s *priv,
                                    uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void i2c_reg_write(struct cxd56_i2cdev_s *priv,
                                 uint32_t offset, uint32_t val)
{
  putreg32(val, priv->base + offset);
}

static inline void i2c_reg_rmw(struct cxd56_i2cdev_s *priv, uint32_t offset,
                               uint32_t val, uint32_t mask)
{
  uint32_t regval;
  regval = getreg32(priv->base + offset);
  putreg32((regval & ~mask) | val, priv->base + offset);
}

static int cxd56_i2c_disable(struct cxd56_i2cdev_s *priv)
{
  int retry = 25000;
  uint32_t stat;

  /* disable all interrupt */

  i2c_reg_write(priv, CXD56_IC_INTR_MASK, 0x0);

  /* clear all interrupt status */

  i2c_reg_read(priv, CXD56_IC_CLR_INTR);
  i2c_reg_write(priv, CXD56_IC_ENABLE, 0);

  do
    {
      stat = i2c_reg_read(priv, CXD56_IC_ENABLE_STATUS);
    }
  while (--retry && (stat & ESTATUS_IC_EN));

  if (!retry)
    {
      i2cerr("i2c wait timeout.\n");
      return -EBUSY;
    }

  /* clear all interrupt status again */

  i2c_reg_read(priv, CXD56_IC_CLR_INTR);

  return 0;
}

static void cxd56_i2c_enable(struct cxd56_i2cdev_s *priv)
{
  i2c_reg_write(priv, CXD56_IC_INTR_MASK, I2C_INTR_ENABLE);
  i2c_reg_write(priv, CXD56_IC_ENABLE, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *cxd56_i2cbus_initialize(int port)
{
  struct cxd56_i2cdev_s *priv;

  irqstate_t flags;

  flags = enter_critical_section();

#ifdef CONFIG_CXD56_I2C0
  if (port == 0)
    {
      priv          = &g_i2c0dev;
#  ifndef CONFIG_CXD56_I2C0_SCUSEQ
      priv->dev.ops = &cxd56_i2c_ops;
#  else
      priv->dev.ops = &cxd56_i2c_scu_ops;
#  endif
    }
  else
#endif
#ifdef CONFIG_CXD56_I2C1
  if (port == 1)
    {
      priv          = &g_i2c1dev;
#  ifndef CONFIG_CXD56_I2C1_SCUSEQ
      priv->dev.ops = &cxd56_i2c_ops;
#  else
      priv->dev.ops = &cxd56_i2c_scu_ops;
#  endif
    }
  else
#endif
#ifdef CONFIG_CXD56_I2C2
  if (port == 2)
    {
      priv          = &g_i2c2dev;
      priv->dev.ops = &cxd56_i2c_ops;
    }
  else
#endif
    {
      leave_critical_section(flags);
      i2cerr("I2C Only support 0,1,2\n");
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

  cxd56_i2c_clock_enable(priv->port);
  priv->base_freq = cxd56_get_i2c_baseclock(priv->port);

  cxd56_i2c_disable(priv);

  i2c_reg_write(priv, CXD56_IC_INTR_MASK, 0x00);
  i2c_reg_read(priv, CXD56_IC_CLR_INTR);

  /* set threshold level of the Rx/Tx FIFO */

  i2c_reg_write(priv, CXD56_IC_RX_TL, 0xff);
  i2c_reg_write(priv, CXD56_IC_TX_TL, 0);

  /* set hold time for margin */

  i2c_reg_write(priv, CXD56_IC_SDA_HOLD, 1);

  i2c_reg_write(priv, CXD56_IC_CON,
                (IC_RX_FIFO_FULL_HLD_CTRL | IC_RESTART_EN |
                 IC_SLAVE_DISABLE | IC_MASTER_MODE | IC_TX_EMPTY_CTRL));

  cxd56_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

  leave_critical_section(flags);

  /* Configure pin */

  cxd56_i2c_pincontrol(port, true);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, cxd56_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  /* Enable Interrupt in SCU */

  if (port == 0 || port == 1)
    {
      putreg32(getreg32(CXD56_SCU_BASE + 0x400) | (1u << (port + 1)),
               CXD56_SCU_BASE + 0x400);
    }

  /* Enable clock gating (clock disable) */

  cxd56_i2c_clock_gate_enable(port);

  return &priv->dev;
}

/****************************************************************************
 * Name: cxd56_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int cxd56_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)dev;

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  if (--priv->refs)
    {
      return OK;
    }

  /* Configure pin */

  cxd56_i2c_pincontrol(priv->port, false);

  /* Disable clock gating (clock enable) */

  cxd56_i2c_clock_gate_disable(priv->port);

  cxd56_i2c_disable(priv);
  cxd56_i2c_clock_disable(priv->port);

  up_disable_irq(priv->irqid);
  irq_detach(priv->irqid);

  wd_cancel(&priv->timeout);

  return OK;
}

#endif
