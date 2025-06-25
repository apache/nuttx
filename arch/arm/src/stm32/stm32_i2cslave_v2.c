/****************************************************************************
 * arch/arm/src/stm32/stm32_i2cslave_v2.c
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
 * TODO
 *  - DMA
 *  - SMBus adaptation
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_slave.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/stm32_pinmap.h"
#include "hardware/stm32_i2c_v2.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_CR1_ALLINTS (I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_ADDRIE | \
                         I2C_CR1_STOPIE)

#ifdef CONFIG_STM32_I2C_SLAVE_USEWQ
#  ifdef CONFIG_SCHED_HPWORK
#    define I2CSWORK HPWORK
#  endif

#  ifndef I2CSWORK
#    ifdef CONFIG_SCHED_LPWORK
#      define I2CSWORK LPWORK
#    endif
#  endif

#  ifndef I2CSWORK
#    error "For correct operation, you should define LPWORK or HPWORK."
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_i2cslave_s
{
  const struct i2c_slaveops_s *ops;
  int refs;
  mutex_t lock;
  uint32_t frequency;
  uint32_t base;
  uint32_t clk_bit;
  uint32_t reset_bit;
  uint32_t ev_irq;
  uint32_t scl_pin;
  uint32_t sda_pin;
  const uint8_t *tx_buffer;
  uint8_t *rx_buffer;
  int rx_buflen;
  int rx_curptr;
  int tx_buflen;
  int tx_curptr;
  int rx_received;
  i2c_slave_callback_t *callback;
  void *callback_arg;
  bool read;
#ifdef CONFIG_STM32_I2C_SLAVE_USEWQ
  struct work_s irqwork;          /* For deferring interrupt work to the wq */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_i2c_setup(struct i2c_slave_s *dev);
static int stm32_i2c_shutdown(struct i2c_slave_s *dev);
static int stm32_i2c_setownaddress(struct i2c_slave_s *dev, int addr,
                                   int nbits);
static int stm32_i2c_write(struct i2c_slave_s *dev, const uint8_t *buffer,
                           int buflen);
static int stm32_i2c_read(struct i2c_slave_s *dev, uint8_t *buffer,
                          int buflen);
static int stm32_i2c_registercallback(struct i2c_slave_s *dev,
                              i2c_slave_callback_t *callback,
                              void *arg);
static int stm32_i2c_init(struct stm32_i2cslave_s *priv);
static int stm32_i2c_deinit(struct stm32_i2cslave_s *priv);
static int stm32_i2c_isr(int irq, void *context, void *arg);
static int stm32_i2c_isr_impl(struct stm32_i2cslave_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_slaveops_s stm32_i2cslave_ops =
{
  .setownaddress = stm32_i2c_setownaddress,
  .write = stm32_i2c_write,
  .read = stm32_i2c_read,
  .registercallback = stm32_i2c_registercallback,
  .setup = stm32_i2c_setup,
  .shutdown = stm32_i2c_shutdown
};

#ifdef CONFIG_STM32_I2C1_SLAVE
static struct stm32_i2cslave_s stm32_i2c1_priv =
{
  .ops = &stm32_i2cslave_ops,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
  .frequency = 0,
  .base = STM32_I2C1_BASE,
  .clk_bit = RCC_APB1ENR_I2C1EN,
  .reset_bit = RCC_APB1RSTR_I2C1RST,
  .ev_irq = STM32_IRQ_I2C1EV,
  .scl_pin = GPIO_I2C1_SCL,
  .sda_pin = GPIO_I2C1_SDA,
  .tx_buffer = NULL,
  .rx_buffer = NULL,
  .rx_buflen = 0,
  .rx_curptr = 0,
  .tx_curptr = 0,
};
#endif

#ifdef CONFIG_STM32_I2C2_SLAVE
static struct stm32_i2cslave_s stm32_i2c2_priv =
{
  .ops = &stm32_i2cslave_ops,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
  .frequency = 0,
  .base = STM32_I2C2_BASE,
  .clk_bit = RCC_APB1ENR_I2C2EN,
  .reset_bit = RCC_APB1RSTR_I2C2RST,
  .ev_irq = STM32_IRQ_I2C2EV,
  .scl_pin = GPIO_I2C2_SCL,
  .sda_pin = GPIO_I2C2_SDA,
  .tx_buffer = NULL,
  .rx_buffer = NULL,
  .rx_buflen = 0,
  .rx_curptr = 0,
  .tx_curptr = 0,
};
#endif

#ifdef CONFIG_STM32_I2C3_SLAVE
static struct stm32_i2cslave_s stm32_i2c3_priv =
{
  .ops = &stm32_i2cslave_ops,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
  .frequency = 0,
  .base = STM32_I2C3_BASE,
  .clk_bit = RCC_APB1ENR_I2C3EN,
  .reset_bit = RCC_APB1RSTR_I2C3RST,
  .ev_irq = STM32_IRQ_I2C3EV,
  .scl_pin = GPIO_I2C3_SCL,
  .sda_pin = GPIO_I2C3_SDA,
  .tx_buffer = NULL,
  .rx_buffer = NULL,
  .rx_buflen = 0,
  .rx_curptr = 0,
  .tx_curptr = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t stm32_i2c_getreg(struct stm32_i2cslave_s *priv,
                                        uint8_t offset)
{
  return getreg16(priv->base + offset);
}

/****************************************************************************
 * Name: stm32_i2c_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t stm32_i2c_getreg32(struct stm32_i2cslave_s *priv,
                                          uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: stm32_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32_i2c_putreg(struct stm32_i2cslave_s *priv,
                                    uint8_t offset, uint16_t value)
{
  putreg16(value, priv->base + offset);
}

/****************************************************************************
 * Name: stm32_i2c_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32_i2c_putreg32(struct stm32_i2cslave_s *priv,
                                      uint8_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: stm32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

static int stm32_i2c_isr(int irq, void *context, void *arg)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)arg;
  DEBUGASSERT(priv != NULL);
  return stm32_i2c_isr_impl(priv);
}

/****************************************************************************
 * Name: stm32_i2c_rxdone_work
 *
 * Description:
 *   A routine for delegating frame reception information
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_I2C_SLAVE_USEWQ
static void stm32_i2c_rxdone_work(void *arg)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)arg;
  priv->callback(priv->callback_arg, I2CS_RX_COMPLETE,
                 priv->rx_curptr);
}
#endif

/****************************************************************************
 * Name: stm32_i2c_rxdone_work
 *
 * Description:
 *   A routine for delegating tx frame information
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_I2C_SLAVE_USEWQ
static void stm32_i2c_txdone_work(void *arg)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)arg;
  priv->callback(priv->callback_arg, I2CS_TX_COMPLETE,
                 priv->tx_curptr);
}
#endif

/****************************************************************************
 * Name: stm32_i2c_isr_impl
 *
 * Description:
 *  Interrupt handler
 *
 ****************************************************************************/

static int stm32_i2c_isr_impl(struct stm32_i2cslave_s *priv)
{
  volatile uint32_t isr;
  volatile uint8_t rx;
  volatile uint8_t tx;

  /* Get the status register first. */

  isr = stm32_i2c_getreg32(priv, STM32_I2C_ISR_OFFSET);

  /* Was the TX completed? */

  if ((isr & I2C_ISR_TXIS) != 0)
    {
      /* Check, if anything must be sent */

      if (priv->tx_curptr < priv->tx_buflen - 1)
        {
          /* Yes... */

          priv->tx_curptr++;
          tx = priv->tx_buffer[priv->tx_curptr];
        }
      else if (priv->tx_curptr == priv->tx_buflen - 1)
        {
          tx = CONFIG_STM32_I2C_SLAVE_DEFAULT_TX;
          if (priv->callback)
            {
#ifdef CONFIG_STM32_I2C_SLAVE_USEWQ
              work_queue(I2CSWORK, &priv->irqwork,
                         stm32_i2c_txdone_work, priv, 0);
#else
              priv->callback(priv->callback_arg, I2CS_TX_COMPLETE,
                             priv->tx_curptr);
#endif
            }
        }
      else
        {
          /* No... Send the default value. */

          tx = CONFIG_STM32_I2C_SLAVE_DEFAULT_TX;
        }

      stm32_i2c_putreg(priv, STM32_I2C_TXDR_OFFSET, tx);
    }

  /* Was a byte received? */

  if ((isr & I2C_ISR_RXNE) != 0)
    {
      /* Write it if not overflowed. RXDR must be read to clear int. */

      rx = stm32_i2c_getreg(priv, STM32_I2C_RXDR_OFFSET);
      if (priv->rx_curptr < priv->rx_buflen)
        {
          priv->rx_buffer[priv->rx_curptr++] = rx;
        }
    }

  /* Was the stop condition detected? */

  if ((isr & I2C_INT_STOP) != 0)
    {
      /* Clear the interrupt. */

      stm32_i2c_putreg(priv, STM32_I2C_ICR_OFFSET, I2C_INT_STOP);

      /* If RX was present, notify the upper driver. */

      if (priv->read)
        {
          if (priv->callback)
            {
#ifdef CONFIG_STM32_I2C_SLAVE_USEWQ
              work_queue(I2CSWORK, &priv->irqwork,
                        stm32_i2c_rxdone_work, priv, 0);
#else
              priv->callback(priv->callback_arg, I2CS_RX_COMPLETE,
                            priv->rx_curptr);
              priv->rx_curptr = 0;
#endif
            }
        }
#ifdef CONFIG_STM32_I2C_SLAVE_RETRANSFER
      else
        {
          priv->tx_curptr = 0;
        }
#endif
    }

  /* Was an address matched? */

  if ((isr & I2C_INT_ADDR) != 0)
    {
      /* Clear the address match flag. */

      stm32_i2c_putreg(priv, STM32_I2C_ICR_OFFSET, I2C_INT_ADDR);

      /* Repeated Start */

      if (priv->rx_curptr > 0 && priv->callback)
        {
#ifdef CONFIG_STM32_I2C_SLAVE_USEWQ
          work_queue(I2CSWORK, &priv->irqwork,
                     stm32_i2c_rxdone_work, priv, 0);
#else
          priv->callback(priv->callback_arg, I2CS_RX_COMPLETE,
                         priv->rx_curptr);
#endif
        }

      /* Check whether RX or TX should be done. */

      if ((isr & I2C_ISR_DIR) != 0)
        {
          /* Write transfer. Flush the TX buffer by writing ISR_TXE.
           * Then send any remaining data. Or send the default TX byte.
           */

          stm32_i2c_putreg32(priv, STM32_I2C_ISR_OFFSET, I2C_ISR_TXE);

          if (priv->tx_curptr < priv->tx_buflen)
            {
              tx = priv->tx_buffer[priv->tx_curptr];
            }
          else
            {
              /* Nothing to be sent. */

              tx = CONFIG_STM32_I2C_SLAVE_DEFAULT_TX;
            }

          stm32_i2c_putreg(priv, STM32_I2C_TXDR_OFFSET, tx);
          priv->read = false;
        }
      else
        {
          /* Initialize the reading. */

          priv->read = true;
          priv->rx_curptr = 0;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_setownaddress
 *
 * Description:
 *   Sets up the address of the I2C Slave
 *
 ****************************************************************************/

static int stm32_i2c_setownaddress(struct i2c_slave_s *dev, int addr,
                                   int nbits)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)dev;
  uint16_t oar1;
  uint32_t cr1;

  i2cinfo("SETOWNADDR %d\n", addr);

  /* STM32 supports up to 2 addresses the slave can respond to.
   * However, NuttX supports the setting of only one address.
   * Use only the first "OWN ADDRESS" (I2C_OAR1)
   * Before this, reset the peripheral by disabling it.
   * If the peripheral is launched for the first time, this does nothing.
   */

  cr1 = stm32_i2c_getreg32(priv, STM32_I2C_CR1_OFFSET);
  stm32_i2c_putreg32(priv, STM32_I2C_CR1_OFFSET,
                     cr1 & ~(I2C_CR1_ALLINTS | I2C_CR1_PE));

  /* Clear I2C_OAR1_OA1EN, then configure I2C_OAR1_OA1[9:0].
   * Afterwards, set I2C_OAR1_{OA1MODE, OA1EN}.
   * According to i2c_slave_open in i2c_slave_driver.c, SETOWNADDRESS comes
   * after SETUP. Therefore, enable the peripheral and RX and TX interrupts
   * here.
   *
   * Attention: If a 10 bit address is used, all 10 bits are used.
   * However, bits 7:1 are used (instead of 6:0) in the 7 bit mode.
   * Therefore, the address must be shifted.
   */

  if (nbits == 10)
    {
      oar1 = ((uint16_t) addr) & 0x03ff;
      oar1 |= I2C_OAR1_OA1MODE;
    }
  else if (nbits == 7)
    {
      oar1 = ((uint16_t) addr << 1) & 0x00fe;
    }
  else
    {
      /* Wrong nbits. */

      return ERROR;
    }

  /* Clear OA1EN (whole register can be cleared) */

  stm32_i2c_putreg32(priv, STM32_I2C_OAR1_OFFSET, 0);
  stm32_i2c_putreg32(priv, STM32_I2C_OAR1_OFFSET, oar1);

  /* Enable the address here. */

  oar1 |= I2C_OAR1_OA1EN;
  stm32_i2c_putreg32(priv, STM32_I2C_OAR1_OFFSET, oar1);

  /* Enable the peripheral and interrupts */

  priv->read = false;
  stm32_i2c_putreg32(priv, STM32_I2C_CR1_OFFSET,
                     I2C_CR1_ALLINTS | I2C_CR1_PE);

  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_setup
 *
 * Description:
 *   Sets up the STM32 I2C peripheral
 *
 ****************************************************************************/

static int stm32_i2c_setup(struct i2c_slave_s *dev)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)dev;
  DEBUGASSERT(dev);

  /* Enable the interrupts here. This function is called when the device
   * is opened for the first time.
   */

  irq_attach(priv->ev_irq, stm32_i2c_isr, priv);
  up_enable_irq(priv->ev_irq);
  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_shutdown
 *
 * Description:
 *   Shutdown the STM32 I2C peripheral
 *
 ****************************************************************************/

static int stm32_i2c_shutdown(struct i2c_slave_s *dev)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)dev;
  uint32_t cr1;

  DEBUGASSERT(dev);

  /* Disable I2C_CR1_PE. Disabling the I2C should have no effect
   * on configuration bits, and SCL and SDA lines are released.
   * Disable TX and TX interrupts.
   */

  cr1 = stm32_i2c_getreg32(priv, STM32_I2C_CR1_OFFSET);
  stm32_i2c_putreg32(priv, STM32_I2C_CR1_OFFSET,
                     cr1 & ~(I2C_CR1_ALLINTS | I2C_CR1_PE));

  /* Disable the interrupts here. This function is called when the device
   * is closed by the last task.
   */

  up_disable_irq(priv->ev_irq);
  irq_detach(priv->ev_irq);
  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_write
 *
 * Description:
 *   Receive a pointer to a buffer where to write data to
 *
 ****************************************************************************/

static int stm32_i2c_write(struct i2c_slave_s *dev, const uint8_t *buffer,
                           int buflen)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)dev;
  int flags;

  DEBUGASSERT(dev);
  flags = enter_critical_section();

  /* Initialize the TX buffer. */

  priv->tx_buffer = buffer;
  priv->tx_buflen = buflen;
  priv->tx_curptr = 0;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_read
 *
 * Description:
 *   Receive a pointer to a buffer where to read data to
 *
 ****************************************************************************/

static int stm32_i2c_read(struct i2c_slave_s *dev, uint8_t *buffer,
                          int buflen)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)dev;
  int flags;

  DEBUGASSERT(dev);
  flags = enter_critical_section();

  /* Initialize the RX buffer. */

  priv->rx_buffer = buffer;
  priv->rx_buflen = buflen;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_registercallback
 *
 * Description:
 *   Register a function which notifies the upperhalf driver
 *
 ****************************************************************************/

static int stm32_i2c_registercallback(struct i2c_slave_s *dev,
                                      i2c_slave_callback_t *callback,
                                      void *arg)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)dev;
  int flags;

  DEBUGASSERT(dev);
  flags = enter_critical_section();

  /* Initialize the pointer to a callback. */

  priv->callback = callback;
  priv->callback_arg = arg;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_init
 *
 * Description:
 *   Initialize STM32 I2C peripheral - clocks and pins
 *
 ****************************************************************************/

static int stm32_i2c_init(struct stm32_i2cslave_s *priv)
{
  DEBUGASSERT(priv);

  modifyreg32(STM32_RCC_APB1ENR, 0, priv->clk_bit);
  modifyreg32(STM32_RCC_APB1RSTR, 0, priv->reset_bit);
  modifyreg32(STM32_RCC_APB1RSTR, priv->reset_bit, 0);

  if (stm32_configgpio(priv->scl_pin) < 0)
    {
      return ERROR;
    }

  if (stm32_configgpio(priv->sda_pin) < 0)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_deinit
 *
 * Description:
 *   Deinitialize STM32 I2C peripheral - clocks and pins
 *
 ****************************************************************************/

static int stm32_i2c_deinit(struct stm32_i2cslave_s *priv)
{
  DEBUGASSERT(priv);

  modifyreg32(STM32_RCC_APB1ENR, priv->clk_bit, 0);
  modifyreg32(STM32_RCC_APB1RSTR, 0, priv->reset_bit);
  stm32_unconfiggpio(priv->scl_pin);
  stm32_configgpio(priv->sda_pin);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2cbus_slaveinitialize
 *
 * Description:
 *   Initialize on I2C bus as a slave and get pointer to i2c_slave_s struct
 *
 ****************************************************************************/

struct i2c_slave_s *stm32_i2cbus_slaveinitialize(int port)
{
  int ret;
  struct stm32_i2cslave_s *priv = NULL;

  i2cinfo("SLAVEINIT");

  switch (port)
    {
#ifdef CONFIG_STM32_I2C1_SLAVE
    case 1:
      priv = (struct stm32_i2cslave_s *)&stm32_i2c1_priv;
      break;
#endif
#ifdef CONFIG_STM32_I2C2_SLAVE
    case 2:
      priv = (struct stm32_i2cslave_s *)&stm32_i2c2_priv;
      break;
#endif
#ifdef CONFIG_STM32_I2C3_SLAVE
    case 3:
      priv = (struct stm32_i2cslave_s *)&stm32_i2c3_priv;
      break;
#endif
    default:
      return NULL;
    }

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      ret = stm32_i2c_init(priv);
      if (ret < 0)
        {
          stm32_i2c_deinit(priv);
          priv = NULL;
        }
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_slave_s *)priv;
}

/****************************************************************************
 * Name: stm32_i2cbus_slaveunitialize
 *
 * Description:
 *   Denitialize a given I2C device.
 *
 ****************************************************************************/

int stm32_i2cbus_slaveunitialize(struct i2c_slave_s *dev)
{
  struct stm32_i2cslave_s *priv = (struct stm32_i2cslave_s *)dev;
  stm32_i2c_deinit(priv);
  return OK;
}
