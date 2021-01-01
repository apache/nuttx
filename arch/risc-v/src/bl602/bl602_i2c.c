/****************************************************************************
 * arch/risc-v/src/bl602/bl602_i2c.c
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
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "riscv_arch.h"

#include "hardware/bl602_glb.h"
#include "hardware/bl602_hbn.h"
#include "hardware/bl602_i2c.h"
#include "bl602_i2c.h"
#include "bl602_gpio.h"

#ifdef CONFIG_BL602_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_TIMEOUT  (20*1000/CONFIG_USEC_PER_TICK) /* 20 mS */

#define I2C_DEFAULT_FREQUENCY 400000
#define I2C_FIFO_MAX_SIZE	    4

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct bl602_i2cdev_s
{
  struct i2c_master_s dev;     /* Generic I2C device */
  uint32_t         base_freq;  /* branch frequency */

  sem_t            mutex;      /* Only one thread can access at a time */
  sem_t            wait;       /* Place to wait for transfer completion */
  struct wdog_s    timeout;    /* watchdog to timeout for bus */
  uint32_t         frequency;  /* Current I2C frequency */
  ssize_t          reg_buff_offset;
  ssize_t          rw_size;

  struct i2c_msg_s *msgs;

  int              error;      /* Error status of each transfers */
  int              refs;       /* Reference count */
};

static struct bl602_i2cdev_s g_i2cdev =
{
  .refs = 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int i2c_takesem(FAR sem_t *sem);
static inline int i2c_givesem(FAR sem_t *sem);

static int  bl602_i2c_interrupt(int irq, FAR void *context, FAR void *arg);
static void bl602_i2c_timeout(wdparm_t arg);
static void bl602_i2c_setfrequency(struct bl602_i2cdev_s *priv,
                                   uint32_t frequency);
static int  bl602_i2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int bl602_i2c_reset(FAR struct i2c_master_s *dev);
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

struct i2c_ops_s bl602_i2c_ops =
{
  .transfer = bl602_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = bl602_i2c_reset,
#endif
};

/****************************************************************************
 * Name: bl602_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void bl602_i2c_setfrequency(struct bl602_i2cdev_s *priv,
                                   uint32_t frequency)
{
  uint16_t bclk_div;
  uint32_t core_freq = getreg32(BL602_HBN_RSV2);
  uint32_t i2c_clk_prd;
  uint16_t i2c_clk_div = 1;
  uint32_t reg;

  i2cinfo("frequency=%lu\n", (unsigned long)frequency);
  if (frequency == priv->frequency)
    {
      return;
    }

  bclk_div = ((getreg32(BL602_CLK_CFG0) & CLK_CFG0_REG_BCLK_DIV_MASK) >> \
              CLK_CFG0_REG_BCLK_DIV_SHIFT) + 1;

  i2c_clk_prd = ((core_freq / bclk_div / i2c_clk_div) / (frequency * 4));
  if ((i2c_clk_prd - 1) > 0xff)
    {
      /* We cannot get enough scaling with just the PRD control
       * the I2C clock could also be scaled here to increase the
       * range.  For now we just cap it as this is not a standard
       * frequency anyway.
       */

      i2c_clk_prd = 0xff;
    }

  i2cinfo("actual frequency=%lu\n",
    (unsigned long)(core_freq / bclk_div / i2c_clk_div / 4 / i2c_clk_prd));

  /* Set the I2C clock */

  modifyreg32(BL602_CLK_CFG3, CLK_CFG3_I2C_CLK_DIV_MASK,
              (i2c_clk_div - 1) << CLK_CFG3_I2C_CLK_DIV_SHIFT);

  modifyreg32(BL602_CLK_CFG3, 0, CLK_CFG3_I2C_CLK_EN);

  i2c_clk_prd -= 1;  /* Divider offest by 1 */
  reg = i2c_clk_prd + \
        (i2c_clk_prd << 8) + \
        (i2c_clk_prd << 16) + \
        (i2c_clk_prd << 24);

  putreg32(reg, BL602_I2C_PRD_DATA);
  putreg32(reg, BL602_I2C_PRD_START);
  putreg32(reg, BL602_I2C_PRD_STOP);

  priv->frequency = frequency;
}

/****************************************************************************
 * Name: bl602_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void bl602_i2c_timeout(wdparm_t arg)
{
  struct bl602_i2cdev_s *priv = (struct bl602_i2cdev_s *)arg;
  irqstate_t flags            = enter_critical_section();

  priv->error = -ENODEV;
  i2c_givesem(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bl602_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int bl602_i2c_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct bl602_i2cdev_s *priv = (FAR struct bl602_i2cdev_s *)arg;
  return OK;
}

/****************************************************************************
 * Name: bl602_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int bl602_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  struct bl602_i2cdev_s *priv = (struct bl602_i2cdev_s *)dev;
  int i;
  int ret    = 0;
  int semval = 0;
  int addr = -1;
  uint32_t reg;

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  i2c_takesem(&priv->mutex);

  ret = nxsem_get_value(&priv->wait, &semval);
  DEBUGASSERT(ret == OK && semval == 0);

  for (i = 0; i < count; i++, msgs++)
    {
      bl602_i2c_setfrequency(priv, msgs->frequency);

      /* Set the slave address */

      modifyreg32(BL602_I2C_CONFIG, I2C_CONFIG_CR_I2C_SLV_ADDR_MASK,
                  msgs->addr << I2C_CONFIG_CR_I2C_SLV_ADDR_SHIFT);

      if (msgs->length > I2C_FIFO_MAX_SIZE)
        {
          /* Too long to do in one transaction */

          i2cinfo("msg is too long\n");
          ret = -EINVAL;
          break;
        }

      modifyreg32(BL602_I2C_CONFIG, I2C_CONFIG_CR_I2C_PKT_LEN_MASK,
                  (msgs->length - 1) << I2C_CONFIG_CR_I2C_PKT_LEN_SHIFT);

      /* Clear INT and FIFO status */

      modifyreg32(BL602_I2C_INT_STS, 0,
                  I2C_INT_STS_CR_NAK_CLR | \
                  I2C_INT_STS_CR_ARB_CLR | \
                  I2C_INT_STS_CR_END_CLR);
      putreg32(0, BL602_I2C_FIFO_WDATA);
      getreg32(BL602_I2C_FIFO_RDATA);
      modifyreg32(BL602_I2C_FIFO_CONFIG_0, 0,
                  I2C_FIFO_CONFIG_0_RX_FIFO_CLR | \
                  I2C_FIFO_CONFIG_0_TX_FIFO_CLR);

      if (msgs->flags & I2C_M_READ)
        {
          modifyreg32(BL602_I2C_CONFIG, 0, I2C_CONFIG_CR_I2C_PKT_DIR);
        }
      else
        {
          modifyreg32(BL602_I2C_CONFIG, I2C_CONFIG_CR_I2C_PKT_DIR, 0);
          reg = 0;
          memcpy((void *)&reg, msgs->buffer, msgs->length);
          putreg32(reg, BL602_I2C_FIFO_WDATA);
        }

      /* Start the transaction */

      modifyreg32(BL602_I2C_CONFIG, 0, I2C_CONFIG_CR_I2C_M_EN);

      /* Wait for transfer to complete */

      while (1)
        {
          reg = getreg32(BL602_I2C_INT_STS);
          if (reg & I2C_INT_STS_NAK_INT)
            {
              i2cinfo("MSG NAK\n");
              ret = -ENXIO;
              break;
            }
          else if (reg & I2C_INT_STS_END_INT)
            {
              if (msgs->flags & I2C_M_READ)
                {
                  reg = getreg32(BL602_I2C_FIFO_RDATA);
                  memcpy(msgs->buffer, (void *)&reg, msgs->length);
                }

              ret = OK;
              break;
            }
        }

      /* End the transaction */

      modifyreg32(BL602_I2C_CONFIG, I2C_CONFIG_CR_I2C_M_EN, 0);

      if (ret < 0)
        {
          break;
        }
    }

  i2c_givesem(&priv->mutex);

  return ret;
}

/****************************************************************************
 * Name: bl602_i2c_reset
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
static int bl602_i2c_reset(FAR struct i2c_master_s *dev)
{
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *bl602_i2cbus_initialize(void)
{
  struct bl602_i2cdev_s *priv = &g_i2cdev;

  irqstate_t flags;

  flags = enter_critical_section();

  priv->refs++;

  /* Test if already initialized or not */

  if (1 < priv->refs)
    {
      leave_critical_section(flags);
      return &priv->dev;
    }

  priv->dev.ops = &bl602_i2c_ops;
  priv->frequency = 0;

  bl602_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

  leave_critical_section(flags);

  /* Configure pin */

  bl602_configgpio(BOARD_I2C_SCL);
  bl602_configgpio(BOARD_I2C_SDA);

  nxsem_init(&priv->mutex, 0, 1);
  nxsem_init(&priv->wait, 0, 0);
  nxsem_set_protocol(&priv->wait, SEM_PRIO_NONE);

  /* Attach Interrupt Handler */

  irq_attach(BL602_IRQ_I2C, bl602_i2c_interrupt, priv);

  return &priv->dev;
}

/****************************************************************************
 * Name: bl602_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int bl602_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  struct bl602_i2cdev_s *priv = (struct bl602_i2cdev_s *)dev;

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  if (--priv->refs)
    {
      return OK;
    }

  up_disable_irq(BL602_IRQ_I2C);
  irq_detach(BL602_IRQ_I2C);

  wd_cancel(&priv->timeout);
  nxsem_destroy(&priv->mutex);
  nxsem_destroy(&priv->wait);

  return OK;
}

#endif
