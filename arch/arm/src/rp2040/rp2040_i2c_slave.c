/****************************************************************************
 * arch/arm/src/rp2040/rp2040_i2c_slave.c
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
#include <nuttx/i2c/i2c_slave.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <arch/chip/i2c_slave.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "rp2040_i2c.h"
#include "hardware/rp2040_i2c.h"
#include "hardware/rp2040_resets.h"
#include "rp2040_gpio.h"

#ifdef CONFIG_RP2040_I2C_SLAVE

#define FIFO_LENGTH 16

#define TX_BUF_LEN 8
#define RX_BUF_LEN 8

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct rp2040_i2c_slave_s
{
  struct i2c_slave_s     dev;          /* Generic I2C device */
  int8_t                 controller;   /* I2C controller number */
  int                    error;        /* Error value */

  uint8_t               *rx_buffer;
  uint8_t               *rx_buf_ptr;
  uint8_t               *rx_buf_end;

  const uint8_t         *tx_buffer;
  const uint8_t         *tx_buf_ptr;
  const uint8_t         *tx_buf_end;

  i2c_slave_callback_t  *callback;     /* Callback function */
  void                  *callback_arg; /* Argument for callback */
} rp2040_i2c_slave_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int i2c_interrupt(int   irq,
                         void *context,
                         void *arg);

static int my_set_own_address(struct i2c_slave_s  *dev,
                              int                  address,
                              int                  nbits);

static int my_write(struct i2c_slave_s  *dev,
                    const uint8_t       *buffer,
                    int                  length);

static int my_read(struct i2c_slave_s  *dev,
                   uint8_t             *buffer,
                   int                  length);

static int my_register_callback(struct i2c_slave_s   *dev,
                                i2c_slave_callback_t *callback,
                                void                 *arg);

static void enable_i2c_slave(struct i2c_slave_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct i2c_slaveops_s i2c_slaveops =
{
  .setownaddress        = my_set_own_address,
  .write                = my_write,
  .read                 = my_read,
  .registercallback     = my_register_callback,
};

#ifdef CONFIG_RP2040_I2C0_SLAVE

rp2040_i2c_slave_t i2c0_slave_dev =
{
  .dev.ops      = &i2c_slaveops, /* Slave operations */
  .controller   =             0, /* I2C controller number */
};

#endif

#ifdef CONFIG_RP2040_I2C1_SLAVE

rp2040_i2c_slave_t i2c1_slave_dev =
{
  .dev.ops      = &i2c_slaveops, /* Slave operations */
  .controller   =             1, /* I2C controller number */
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int i2c_interrupt(int irq, void *context, void *arg)
{
  rp2040_i2c_slave_t *priv = (rp2040_i2c_slave_t *)arg;
  uint32_t  data_cmd;
  uint32_t  state;

  state = getreg32(RP2040_I2C_IC_INTR_STAT(priv->controller));

  /* -- We need to transmit data (Read Request) -- */

  if (state & RP2040_I2C_IC_INTR_STAT_R_RD_REQ)
    {
      if (priv->tx_buf_ptr < priv->tx_buf_end)
        {
          while (priv->tx_buf_ptr < priv->tx_buf_end
                 &&   getreg32(RP2040_I2C_IC_TXFLR(priv->controller))
                    < FIFO_LENGTH)
            {
              putreg32(*priv->tx_buf_ptr++,
                       RP2040_I2C_IC_DATA_CMD(priv->controller));
            }
        }
      else
        {
          putreg32(0, RP2040_I2C_IC_DATA_CMD(priv->controller));
        }

      getreg32(RP2040_I2C_IC_CLR_RD_REQ(priv->controller));
    }

  /* -- We are receiving data (Write Request) -- */

  if (state & RP2040_I2C_IC_INTR_STAT_R_RX_FULL)
    {
      while (getreg32(RP2040_I2C_IC_RXFLR(priv->controller)) > 0)
        {
          data_cmd = getreg32(RP2040_I2C_IC_DATA_CMD(priv->controller));

          if (data_cmd & RP2040_I2C_IC_DATA_CMD_FIRST_DATA_BYTE)
            {
              priv->rx_buf_ptr = priv->rx_buffer;
            }

          if (priv->rx_buf_ptr < priv->rx_buf_end)
            {
              *priv->rx_buf_ptr++ = (uint8_t) data_cmd;
            }
        }
    }

  /* -- Restart -- */

  if (state & RP2040_I2C_IC_INTR_STAT_R_RESTART_DET)
    {
      if (priv->callback != NULL && priv->rx_buf_ptr > priv->rx_buffer)
        {
          priv->callback(priv, priv->rx_buf_ptr - priv->rx_buffer);
          priv->rx_buf_ptr = priv->rx_buffer;
        }

      getreg32(RP2040_I2C_IC_CLR_RESTART_DET(priv->controller));
    }

  /* -- End of transfer -- */

  if (state & RP2040_I2C_IC_INTR_STAT_R_STOP_DET)
    {
      if (priv->callback != NULL && priv->rx_buf_ptr > priv->rx_buffer)
        {
          priv->callback(priv, priv->rx_buf_ptr - priv->rx_buffer);
          priv->rx_buf_ptr = priv->rx_buffer;
        }

      getreg32(RP2040_I2C_IC_CLR_STOP_DET(priv->controller));
    }

  /* -- Transmit Abort -- */

  if (state & RP2040_I2C_IC_INTR_STAT_R_TX_ABRT)
    {
      getreg32(RP2040_I2C_IC_CLR_TX_ABRT(priv->controller));
      priv->error = -ENODEV;
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_TX_OVER)
    {
      getreg32(RP2040_I2C_IC_CLR_TX_OVER(priv->controller));
      priv->error = -EIO;
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_RX_OVER)
    {
      getreg32(RP2040_I2C_IC_CLR_RX_OVER(priv->controller));
      priv->error = -EIO;
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_RX_UNDER)
    {
      getreg32(RP2040_I2C_IC_CLR_RX_UNDER(priv->controller));
      priv->error = -EIO;
    }

#ifdef NEEDED_FOR_MASTER_MODE_
  if (state & RP2040_I2C_IC_INTR_STAT_R_TX_EMPTY)
    {
      /* TX_EMPTY is automatically cleared by hardware
       * when the buffer level goes above the threshold.
       */

      modbits_reg32(RP2040_I2C_IC_INTR_MASK(priv->controller),
                    0,
                    RP2040_I2C_IC_INTR_MASK_M_TX_EMPTY);
    }

  if (state & RP2040_I2C_IC_INTR_STAT_R_RX_FULL)
    {
      /* RX_FULL is automatically cleared by hardware
       * when the buffer level goes below the threshold.
       */

      modbits_reg32(RP2040_I2C_IC_INTR_MASK(priv->controller),
                    0,
                    RP2040_I2C_IC_INTR_MASK_M_RX_FULL);

      rp2040_i2c_drainrxfifo(priv);
    }

  if ((priv->error) || (state & RP2040_I2C_IC_INTR_STAT_R_TX_EMPTY)
                    || (state & RP2040_I2C_IC_INTR_STAT_R_RX_FULL))
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
  #endif

  return OK;
}

/****************************************************************************
 * Name: enable_i2c_slave
 *
 * Description:
 *   Enable the I2C device as a slave and start handing I2C interrupts.
 *
 ****************************************************************************/

static void enable_i2c_slave(struct i2c_slave_s *dev)
{
  rp2040_i2c_slave_t *priv = (rp2040_i2c_slave_t *) dev;
  irqstate_t flags;

  flags = enter_critical_section();

  uint32_t intr_mask =   RP2040_I2C_IC_INTR_STAT_R_RD_REQ
                       | RP2040_I2C_IC_INTR_STAT_R_RX_FULL
                       | RP2040_I2C_IC_INTR_STAT_R_STOP_DET
                       | RP2040_I2C_IC_INTR_STAT_R_RESTART_DET
                       | RP2040_I2C_IC_INTR_STAT_R_TX_ABRT;

  putreg32(0, RP2040_I2C_IC_ENABLE(priv->controller));

  putreg32(intr_mask, RP2040_I2C_IC_INTR_MASK(priv->controller));

  putreg32(0, RP2040_I2C_IC_ENABLE(priv->controller));

  if (priv->controller == 0)
    {
      irq_attach(RP2040_I2C0_IRQ, i2c_interrupt, dev);
      up_enable_irq(RP2040_I2C0_IRQ);
    }
  else
    {
      irq_attach(RP2040_I2C1_IRQ, i2c_interrupt, dev);
      up_enable_irq(RP2040_I2C1_IRQ);
    }

  putreg32(RP2040_I2C_IC_ENABLE_ENABLE,
           RP2040_I2C_IC_ENABLE(priv->controller));

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: my_set_own_address
 *
 * Description:
 *   Called to set the address listened to, and enable I2C as a slave device.
 *
 ****************************************************************************/

static int my_set_own_address(struct i2c_slave_s  *dev,
                              int                  address,
                              int                  nbits)
{
  rp2040_i2c_slave_t *priv = (rp2040_i2c_slave_t *) dev;

  uint32_t con =   RP2040_I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL
                 | RP2040_I2C_IC_CON_SPEED_FAST;
  irqstate_t flags;

  flags = enter_critical_section();

  putreg32(address, RP2040_I2C_IC_SAR(priv->controller));

  if (nbits == 10)
    {
      con |= RP2040_I2C_IC_CON_IC_10BITADDR_SLAVE;
    }

  putreg32(con, RP2040_I2C_IC_CON(priv->controller));

  enable_i2c_slave(dev);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: my_write
 *
 * Description:
 *   Called to set the data to be read on the next I2C read transaction.
 *
 ****************************************************************************/

static int my_write(struct i2c_slave_s  *dev,
                    const uint8_t       *buffer,
                    int                  length)
{
  rp2040_i2c_slave_t *priv = (rp2040_i2c_slave_t *) dev;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->tx_buffer  = buffer;
  priv->tx_buf_ptr = buffer;
  priv->tx_buf_end = priv->tx_buffer + length;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: my_read
 *
 * Description:
 *   Called to register a buffer to receive data from the next I2C write
 *   transaction.
 *
 ****************************************************************************/

static int my_read(struct i2c_slave_s  *dev,
                   uint8_t             *buffer,
                   int                  length)
{
  rp2040_i2c_slave_t *priv = (rp2040_i2c_slave_t *) dev;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->rx_buffer  = buffer;
  priv->rx_buf_ptr = buffer;
  priv->rx_buf_end = priv->rx_buffer + length;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: my_register_callback
 *
 * Description:
 *   Called to register a callback function that will be called when
 *   data becomes available due to an I2C write transaction.
 *
 ****************************************************************************/

static int my_register_callback(struct i2c_slave_s   *dev,
                                i2c_slave_callback_t *callback,
                                void                 *arg)
{
  rp2040_i2c_slave_t *priv = (rp2040_i2c_slave_t *) dev;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->callback     = callback;
  priv->callback_arg = arg;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_i2c0_slave_initialize
 *
 * Description:
 *   Initialize I2C controller zero for slave operation, and return a pointer
 *   to the instance of struct i2c_slave_s.  This function should only be
 *   called once of a give controller.
 *
 * Input Parameters:
 *   sda_pin       - The GPIO pin for the SDA line.
 *   scl_pin       - The GPIO pin for the SCL line.
 *   address       - The slave address to listen to.
 *   ten_bin       - Set true for 10-bit I2C addressing.
 *   rx_buffer     - Buffer for data transmitted to us by an I2C master.
 *   rx_buffer_len - Length of rx_buffer.
 *   callback      - Callback function called when messages are received.
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RP2040_I2C0_SLAVE

struct i2c_slave_s * rp2040_i2c0_slave_initialize
                           (uint8_t              *rx_buffer,
                            size_t                rx_buffer_len,
                            i2c_slave_callback_t *callback)
{
  rp2040_i2c_slave_t *priv = &i2c0_slave_dev;

  rp2040_gpio_set_function(CONFIG_RP2040_I2C0_SDA_GPIO,
                           RP2040_GPIO_FUNC_I2C);

  rp2040_gpio_set_pulls(CONFIG_RP2040_I2C0_SDA_GPIO, true, false);

  rp2040_gpio_set_function(CONFIG_RP2040_I2C0_SCL_GPIO,
                           RP2040_GPIO_FUNC_I2C);

  rp2040_gpio_set_pulls(CONFIG_RP2040_I2C0_SCL_GPIO, true, false);

  priv->rx_buffer  = rx_buffer;
  priv->rx_buf_ptr = rx_buffer;
  priv->rx_buf_end = priv->rx_buffer + rx_buffer_len;

  if (callback != NULL)
    {
      my_register_callback(&(priv->dev), callback, priv);
    }

#ifdef CONFIG_RP2040_I2C0_SLAVE_10BIT
  my_set_own_address(&(priv->dev),
                     CONFIG_RP2040_I2C0_SLAVE_ADDRESS,
                     10);
#else
  my_set_own_address(&(priv->dev),
                     CONFIG_RP2040_I2C0_SLAVE_ADDRESS,
                     7);
#endif

  return &(priv->dev);
}

#endif /* CONFIG_RP2040_I2C0_SLAVE */

/****************************************************************************
 * Name: rp2040_i2c1_slave_initialize
 *
 * Description:
 *   Initialize I2C controller one for slave operation, and return a pointer
 *   to the instance of struct i2c_slave_s.  This function should only be
 *   called once of a give controller.
 *
 * Input Parameters:
 *   sda_pin       - The GPIO pin for the SDA line.
 *   scl_pin       - The GPIO pin for the SCL line.
 *   address       - The slave address to listen to.
 *   ten_bin       - Set true for 10-bit I2C addressing.
 *   rx_buffer     - Buffer for data transmitted to us by an I2C master.
 *   rx_buffer_len - Length of rx_buffer.
 *   callback      - Callback function called when messages are received.
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RP2040_I2C1_SLAVE

struct i2c_slave_s * rp2040_i2c1_slave_initialize
                           (uint8_t              *rx_buffer,
                            size_t                rx_buffer_len,
                            i2c_slave_callback_t *callback)
{
  rp2040_i2c_slave_t *priv = &i2c1_slave_dev;

  rp2040_gpio_set_function(CONFIG_RP2040_I2C1_SDA_GPIO,
                           RP2040_GPIO_FUNC_I2C);

  rp2040_gpio_set_pulls(CONFIG_RP2040_I2C1_SDA_GPIO, true, false);

  rp2040_gpio_set_function(CONFIG_RP2040_I2C1_SCL_GPIO,
                           RP2040_GPIO_FUNC_I2C);

  rp2040_gpio_set_pulls(CONFIG_RP2040_I2C1_SCL_GPIO, true, false);

  priv->rx_buffer  = rx_buffer;
  priv->rx_buf_ptr = rx_buffer;
  priv->rx_buf_end = priv->rx_buffer + rx_buffer_len;

  if (callback != NULL)
    {
      my_register_callback(&(priv->dev), callback, priv);
    }

#ifdef CONFIG_RP2040_I2C1_SLAVE_10BIT
  my_set_own_address(&(priv->dev),
                     CONFIG_RP2040_I2C1_SLAVE_ADDRESS,
                     10);
#else
  my_set_own_address(&(priv->dev),
                     CONFIG_RP2040_I2C1_SLAVE_ADDRESS,
                     7);
#endif

  return &(priv->dev);
}

#endif /* CONFIG_RP2040_I2C1_SLAVE */

#endif /* CONFIG_RP2040_I2C_SLAVE */
