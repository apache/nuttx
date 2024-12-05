/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_i2c.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>

#include "mx8mp_i2c.h"
#include "mx8mp_ccm.h"
#include "hardware/mx8mp_memorymap.h"
#include "hardware/mx8mp_i2c.h"

#include <debug.h>
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_STALL_TIMEOUT  MSEC2TICK(10)    /* 10 ms */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

struct mx8mp_i2c_s;

static bool is_busy(struct mx8mp_i2c_s *priv);
static void mx8mp_i2c_enable(struct mx8mp_i2c_s *priv);
static void mx8mp_i2c_disable(struct mx8mp_i2c_s *priv);
static int  mx8mp_i2c_interrupt(int irq, void *context, void *arg);
static void mx8mp_i2c_reset_bus(struct mx8mp_i2c_s *dev);

/* I2C lower half driver methods */

static int  mx8mp_i2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count);
static int  mx8mp_i2c_reset(struct i2c_master_s *dev);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C Device Private Data */

struct mx8mp_i2c_s
{
  /* Generic I2C device */

  struct i2c_master_s dev;

  /* Port configuration */

  uint32_t base;              /* I2C base address */
  int      clock;             /* Peripheral clock as described in hardware/mx8mp_ccm.h */
  uint16_t irq;               /* IRQ number */
  uint32_t frequency;         /* Current I2C frequency */
  int      refs;              /* Reference count */
  mutex_t  lock;              /* Only one thread can access at a time */
  sem_t    wait;              /* IRQ wait sync */
};

/* I2C lower half driver operations */

static const struct i2c_ops_s mx8mp_i2c_ops =
{
  .transfer = mx8mp_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = mx8mp_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_MX8MP_I2C1
static struct mx8mp_i2c_s g_i2c1_dev =
{
  .dev.ops    = &mx8mp_i2c_ops,
  .base       = MX8M_I2C1,
  .clock      = I2C1_CLK_ROOT,
  .irq        = MX8MP_IRQ_I2C1,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_MX8MP_I2C2
static struct mx8mp_i2c_s g_i2c2_dev =
{
  .dev.ops    = &mx8mp_i2c_ops,
  .base       = MX8M_I2C2,
  .clock      = I2C2_CLK_ROOT,
  .irq        = MX8MP_IRQ_I2C2,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_MX8MP_I2C3
static struct mx8mp_i2c_s g_i2c3_dev =
{
  .dev.ops    = &mx8mp_i2c_ops,
  .base       = MX8M_I2C3,
  .clock      = I2C3_CLK_ROOT,
  .irq        = MX8MP_IRQ_I2C3,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_MX8MP_I2C4
static struct mx8mp_i2c_s g_i2c4_dev =
{
  .dev.ops    = &mx8mp_i2c_ops,
  .base       = MX8M_I2C4,
  .clock      = I2C4_CLK_ROOT,
  .irq        = MX8MP_IRQ_I2C4,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_MX8MP_I2C5
static struct mx8mp_i2c_s g_i2c5_dev =
{
  .dev.ops    = &mx8mp_i2c_ops,
  .base       = MX8M_I2C5,
  .clock      = I2C5_CLK_ROOT,
  .irq        = MX8MP_IRQ_I2C5,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_MX8MP_I2C6
static struct mx8mp_i2c_s g_i2c6_dev =
{
  .dev.ops    = &mx8mp_i2c_ops,
  .base       = MX8M_I2C6,
  .clock      = I2C6_CLK_ROOT,
  .irq        = MX8MP_IRQ_I2C6,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t DIVIDERS_MAP[] =
{
  30,     32,   36,   42,   48,   52,   60,   72,
  80,     88,  104,  128,  144,  160,  192,  240,
  288,   320,  384,  480,  576,  640,  768,  960,
  1152, 1280, 1536, 1920, 2304, 2560, 3072, 3840,
  22,     24,   26,   28,   32,   36,   40,   44,
  48,     56,   64,   72,   80,   96,  112,  128,
  160,   192,  224,  256,  320,  384,  448,  512,
  640,   768,  896, 1024, 1280, 1536, 1792, 2048
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int mx8mp_i2c_init(struct mx8mp_i2c_s *priv)
{
  /* enable peripheral */

  mx8mp_i2c_enable(priv);

  /* Attach Interrupt Handler */

  irq_attach(priv->irq, mx8mp_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irq);

  /* Force a frequency update */

  priv->frequency = 0;

  return 0;
}

/****************************************************************************
 * Name: mx8mp_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int mx8mp_i2c_deinit(struct mx8mp_i2c_s *priv)
{
  /* Disable and detach interrupts */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Disable peripheral */

  mx8mp_i2c_disable(priv);

  return 0;
}

/****************************************************************************
 * Name: mx8mp_i2c_enable
 *
 * Description:
 *   Enable the I2C peripheral
 *
 ****************************************************************************/

static void mx8mp_i2c_enable(struct mx8mp_i2c_s *priv)
{
  /* Set slave address here to support it (IFDR) */

  putreg16(0,  priv->base + I2SR_OFFSET);
  modreg16(I2CR_IEN,  I2CR_IEN,  priv->base + I2CR_OFFSET);
  modreg16(I2CR_IIEN, I2CR_IIEN, priv->base + I2CR_OFFSET);
}

/****************************************************************************
 * Name: mx8mp_i2c_disable
 *
 * Description:
 *   Disable the I2C peripheral
 *
 ****************************************************************************/

static void mx8mp_i2c_disable(struct mx8mp_i2c_s *priv)
{
  putreg16(0, priv->base + I2CR_OFFSET);
}

/****************************************************************************
 * Name: is_busy
 *
 * Description:
 *   Check if the bus is busy (true) or not (false)
 *
 ****************************************************************************/

static bool is_busy(struct mx8mp_i2c_s *priv)
{
  return (getreg16(priv->base + I2SR_OFFSET) & I2SR_IBB);
}

/****************************************************************************
 * Name: mx8mp_i2c_wait
 *
 * Description:
 *   Wait for the IRQ to be triggered.
 *
 ****************************************************************************/

static inline int mx8mp_i2c_wait_irq(struct mx8mp_i2c_s *priv, int usec)
{
  return nxsem_tickwait(&priv->wait, USEC2TICK(usec));
}

/****************************************************************************
 * Name: mx8mp_i2c_send_start
 *
 * Description:
 *   Emit start bit on the bus
 *
 ****************************************************************************/

static int mx8mp_i2c_send_start(struct mx8mp_i2c_s *priv)
{
  if (is_busy(priv))
    {
      return -EBUSY;
    }

  modreg16(I2CR_MSTA | I2CR_MTX | I2CR_TXAK,
           I2CR_MSTA | I2CR_MTX | I2CR_TXAK,
           priv->base + I2CR_OFFSET);

  return 0;
}

/****************************************************************************
 * Name: mx8mp_i2c_send_repeat_start
 *
 * Description:
 *   Emit repeat start bit on the bus
 *
 ****************************************************************************/

static int mx8mp_i2c_send_repeat_start(struct mx8mp_i2c_s *priv)
{
  if (is_busy(priv) && !(getreg16(priv->base + I2SR_OFFSET) & I2CR_MSTA))
    {
      return -EBUSY;
    }

  modreg16(I2CR_RSTA, I2CR_RSTA, priv->base + I2CR_OFFSET);

  return 0;
}

/****************************************************************************
 * Name: mx8mp_i2c_send_stop
 *
 * Description:
 *   Emit stop bit on the bus
 *
 ****************************************************************************/

static void mx8mp_i2c_send_stop(struct mx8mp_i2c_s *priv)
{
  modreg16(0, I2CR_MSTA | I2CR_MTX | I2CR_TXAK, priv->base + I2CR_OFFSET);
}

/****************************************************************************
 * Name: mx8mp_i2c_write_byte
 *
 * Description:
 *   Write a byte on the bus, checking for arbitration lost and RX ACK.
 *
 ****************************************************************************/

static int mx8mp_i2c_write_byte(struct mx8mp_i2c_s *priv, uint8_t byte)
{
  int ret;
  uint16_t status;

  modreg16(0, I2SR_IIF, priv->base + I2SR_OFFSET);
  putreg16(byte, priv->base + I2DR_OFFSET);

  ret = mx8mp_i2c_wait_irq(priv, 500);
  if (ret < 0)
    {
      return ret;
    }

  status = getreg16(priv->base + I2SR_OFFSET);
  if (status & I2SR_IAL)
    {
      /* Arbitration lost */

      modreg16(0, I2SR_IAL, priv->base + I2SR_OFFSET);
      return -EACCES;
    }

  if (status & I2SR_RXAK)
    {
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: mx8mp_i2c_write_bytes
 *
 * Description:
 *   Write multiple byte on the bus, handling the STOP logic.
 *
 ****************************************************************************/

static int mx8mp_i2c_write_bytes(struct mx8mp_i2c_s *priv,
                                 const uint8_t *data,
                                 uint32_t size,
                                 bool is_last)
{
  uint32_t i;
  int ret;

  for (i = 0; i < size; ++i)
    {
      ret = mx8mp_i2c_write_byte(priv, data[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (is_last)
    {
      mx8mp_i2c_send_stop(priv);
    }

  return 0;
}

/****************************************************************************
 * Name: mx8mp_i2c_read_bytes
 *
 * Description:
 *   Read multiple bytes on the bus, handling the NACK and STOP logic.
 *
 ****************************************************************************/

int mx8mp_i2c_read_bytes(struct mx8mp_i2c_s *priv,
                         uint8_t *data,
                         uint32_t size,
                         bool is_last)
{
  int ret;
  uint8_t temp;
  uint32_t i;

  temp = getreg16(priv->base + I2CR_OFFSET);
  temp &= ~(I2CR_MTX | I2CR_TXAK);
  if (size == 1)
    {
      /* NACK on read if one byte */

      temp |= I2CR_TXAK;
    }

  putreg16(temp, priv->base + I2CR_OFFSET);

  /* dummy read to start reading */

  modreg16(0, I2SR_IIF, priv->base + I2SR_OFFSET);
  data[0] = getreg16(priv->base + I2DR_OFFSET);

  for (i = 0; i < size; ++i)
    {
      ret = mx8mp_i2c_wait_irq(priv, 500);
      if (ret < 0)
        {
          return ret;
        }

      if ((i == (size - 1)) && is_last)
        {
          mx8mp_i2c_send_stop(priv);
        }
      else if (i == (size - 2))
        {
          /* No ACK on last byte */

          modreg16(0, I2CR_TXAK, priv->base + I2CR_OFFSET);
        }

      modreg16(0, I2SR_IIF, priv->base + I2SR_OFFSET);
      data[i] = getreg16(priv->base + I2DR_OFFSET);
    }

  return 0;
}

/****************************************************************************
 * Name: mx8mp_i2c_set_frequency
 *
 * Description:
 *   Set the I2C peripheral clock frequency. Note: the real frequency may be
 *   lower than the targeted one (but never greater).
 *
 ****************************************************************************/

static void mx8mp_i2c_set_frequency(struct mx8mp_i2c_s *priv,
                                    uint32_t frequency)
{
  uint8_t i;
  uint8_t divider;
  uint32_t old_frequency;
  uint32_t clock;

  i2cinfo("frequency=%lu\n", frequency);

  clock = mx8mp_ccm_get_clock(priv->clock);
  DEBUGASSERT(clock > frequency);

  if (frequency == priv->frequency)
    {
      return;
    }

  divider = 0;
  old_frequency = 0;

  for (i = 0; i < 0x40; ++i)
    {
      uint32_t current_frequency = clock / DIVIDERS_MAP[i];
      if ((current_frequency <= frequency) &&
          (current_frequency > old_frequency))
        {
          divider = i;
          old_frequency = current_frequency;
        }
    }

  putreg16(divider, priv->base + IFDR_OFFSET);
  priv->frequency = frequency;
  i2cinfo(" real frequency=%lu (%lu / %d)\n",
    old_frequency, clock, DIVIDERS_MAP[divider]);
}

/****************************************************************************
 * Name: mx8mp_i2c_reset_bus
 *
 * Description:
 *   Reset the bus if this one is stall (stuck I2C device(s))
 *
 ****************************************************************************/

static void mx8mp_i2c_reset_bus(struct mx8mp_i2c_s *priv)
{
  mx8mp_i2c_disable(priv);
  nxsig_usleep(50);
  mx8mp_i2c_enable(priv);
  nxsig_usleep(50);
}

/****************************************************************************
 * Name: mx8mp_i2c_interrupt
 *
 * Description:
 *   The I2C common interrupt handler
 *
 ****************************************************************************/

static int mx8mp_i2c_interrupt(int irq, void *context, void *arg)
{
  struct mx8mp_i2c_s *priv = (struct mx8mp_i2c_s *)arg;
  modreg16(0, I2SR_IIF, priv->base + I2SR_OFFSET);
  nxsem_post(&priv->wait);

  return 0;
}

/****************************************************************************
 * Name: mx8mp_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int mx8mp_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct mx8mp_i2c_s *priv = (struct mx8mp_i2c_s *)dev;
  struct i2c_msg_s *msg;
  clock_t deadline;
  bool last;
  int ret;
  int i;

  i2cinfo("msgs=%p count=%d\n", msgs, count);
  DEBUGASSERT(dev != NULL && msgs != NULL && (unsigned)count <= UINT16_MAX);

  /* Get exclusive access to the I2C bus */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure the I2C frequency. REVISIT: Note that the frequency is set
   * only on the first message. This could be extended to support
   * different transfer frequencies for each message segment.
   */

  mx8mp_i2c_set_frequency(priv, msgs->frequency);

  /* Process every message */

  if ((I2C_M_NOSTART & msgs->flags) == 0)
    {
      ret = mx8mp_i2c_send_start(priv);
      if (ret < 0)
        {
          goto error;
        }
    }

  for (i = 0; i < count; ++i)
    {
      msg = msgs + i;
      last = (i == (count - 1));

      if ((I2C_M_NOSTART & msg->flags) == 0)
        {
          if (I2C_M_READ & msg->flags)
            {
              ret = mx8mp_i2c_write_byte(priv, I2C_READADDR8(msg->addr));
            }
          else
            {
              ret = mx8mp_i2c_write_byte(priv, I2C_WRITEADDR8(msg->addr));
            }
        }

      if (ret == 0)
        {
          if (I2C_M_READ & msg->flags)
            {
              ret = mx8mp_i2c_read_bytes(priv,
                msg->buffer, msg->length, last);
            }
          else
            {
              ret = mx8mp_i2c_write_bytes(priv,
                msg->buffer, msg->length, last);
            }
        }

      if (ret == -EACCES)
        {
          /* arbitration lost */

          goto error;
        }
      else if (ret < 0)
        {
          mx8mp_i2c_send_stop(priv);
          goto error;
        }

      if (!last && ((I2C_M_NOSTART & msg->flags) == 0))
        {
          ret = mx8mp_i2c_send_repeat_start(priv);
          if (ret < 0)
            {
              goto error;
            }
        }
    }

error:

  /* Ensure that the bus is free before leaving */

  deadline = clock_systime_ticks() + I2C_STALL_TIMEOUT;
  while (is_busy(priv))
    {
      if (clock_systime_ticks() > deadline)
        {
          i2cwarn("Bus is stall: try to reset it\n");
          mx8mp_i2c_reset_bus(priv);
          break;
        }

      nxsig_usleep(10);
    }

  /* Release access to I2C bus */

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int mx8mp_i2c_reset(struct i2c_master_s *dev)
{
  struct mx8mp_i2c_s *priv = (struct mx8mp_i2c_s *)dev;
  int ret;

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  mx8mp_i2c_reset_bus(priv);

  /* Release the port for re-use by other clients */

  nxmutex_unlock(&priv->lock);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *mx8mp_i2cbus_initialize(int port)
{
  struct mx8mp_i2c_s *priv;

  i2cinfo("port=%d\n", port);

  switch (port)
    {
#ifdef CONFIG_MX8MP_I2C1
    case 1:
      priv = &g_i2c1_dev;
      break;
#endif

#ifdef CONFIG_MX8MP_I2C2
    case 2:
      priv = &g_i2c2_dev;
      break;
#endif

#ifdef CONFIG_MX8MP_I2C3
    case 3:
      priv = &g_i2c3_dev;
      break;
#endif

#ifdef CONFIG_MX8MP_I2C4
    case 4:
      priv = &g_i2c4_dev;
      break;
#endif

#ifdef CONFIG_MX8MP_I2C5
    case 5:
      priv = &g_i2c5_dev;
      break;
#endif

#ifdef CONFIG_MX8MP_I2C6
    case 6:
      priv = &g_i2c6_dev;
      break;
#endif

    default:
      i2cerr("ERROR: Unsupported I2C port %d\n", port);
      return NULL;
    }

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      mx8mp_i2c_init(priv);
    }

  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

/****************************************************************************
 * Name: mx8mp_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int mx8mp_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct mx8mp_i2c_s *priv = (struct mx8mp_i2c_s *)dev;

  DEBUGASSERT(priv != NULL);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return -1;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return 0;
    }

  /* Disable power and other HW resource (GPIO's) */

  mx8mp_i2c_deinit(priv);

  nxmutex_unlock(&priv->lock);
  return 0;
}
