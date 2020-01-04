/****************************************************************************
 * arch/arm/src/nrf52/nrf52_i2c.c
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <errno.h>
#include <debug.h>
#include <semaphore.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "nrf52_gpio.h"
#include "nrf52_i2c.h"

#include "hardware/nrf52_twi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
#  error I2C irq not supported yet
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C Device Private Data */

struct nrf52_i2c_priv_s
{
  const struct i2c_ops_s *ops;      /* Standard I2C operations */
  uint32_t                base;     /* TWI base address */
  uint32_t                scl_pin;  /* SCL pin configuration */
  uint32_t                sda_pin;  /* SDA pin configuration */
  int                     refs;     /* Reference count */
#ifndef CONFIG_I2C_POLLED
  uint32_t                irq;      /* TWI interrupt */
#endif
  uint8_t                 msgc;     /* Message count */
  struct i2c_msg_s       *msgv;     /* Message list */
  uint8_t                *ptr;      /* Current message buffer */
  uint32_t                freq;     /* Current I2C frequency */
  int                     dcnt;     /* Current message length */
  uint16_t                flags;    /* Current message flags */
  uint16_t                addr;     /* Current I2C address*/
  sem_t                   sem_excl; /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;                    /* Interrupt wait semaphore */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void nrf52_i2c_putreg(FAR struct nrf52_i2c_priv_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf52_i2c_getreg(FAR struct nrf52_i2c_priv_s *priv,
                                        uint32_t offset);
static int nrf52_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs,
                              int count);
#ifdef CONFIG_I2C_RESET
static int nrf52_i2c_reset(FAR struct i2c_master_s *dev);
#endif
#ifndef CONFIG_I2C_POLLED
static int nrf52_i2c_isr(int irq, void *context, FAR void *arg);
#endif
static int nrf52_i2c_deinit(FAR struct nrf52_i2c_priv_s *priv);
static int nrf52_i2c_init(FAR struct nrf52_i2c_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C operations */

static const struct i2c_ops_s g_nrf52_i2c_ops =
{
  .transfer = nrf52_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = nrf52_i2c_reset
#endif
};

/* I2C0 (TWI0) device */

#ifdef CONFIG_NRF52_I2C0_MASTER

static struct nrf52_i2c_priv_s g_nrf52_i2c0_priv =
{
  .ops     = &g_nrf52_i2c_ops,
  .base    = NRF52_TWIM0_BASE,
  .scl_pin = BOARD_I2C0_SCL_PIN,
  .sda_pin = BOARD_I2C0_SDA_PIN,
  .refs    = 0,
#ifndef CONFIG_I2C_POLLED
  .irq     = NRF52_IRQ_SPI_TWI_0,
#endif
  .msgc    = 0,
  .msgv    = NULL,
  .ptr     = NULL,
  .freq    = 0,
  .dcnt    = 0,
  .flags   = 0,
  .addr    = 0,
};
#endif

/* I2C1 (TWI1) device */

#ifdef CONFIG_NRF52_I2C1_MASTER
static struct nrf52_i2c_priv_s g_nrf52_i2c1_priv =
{
  .ops     = &g_nrf52_i2c_ops,
  .base    = NRF52_TWIM0_BASE,
  .scl_pin = BOARD_I2C1_SCL_PIN,
  .sda_pin = BOARD_I2C1_SDA_PIN,
  .refs    = 0,
#ifndef CONFIG_I2C_POLLED
  .irq     = NRF52_IRQ_SPI_TWI_1,
#endif
  .msgc    = 0,
  .msgv    = NULL,
  .ptr     = NULL,
  .freq    = 0,
  .dcnt    = 0,
  .flags   = 0,
  .addr    = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_i2c_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf52_i2c_putreg(FAR struct nrf52_i2c_priv_s *priv,
                                    uint32_t offset,
                                    uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_i2c_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf52_i2c_getreg(FAR struct nrf52_i2c_priv_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int nrf52_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs,
                              int count)
{
  FAR struct nrf52_i2c_priv_s *priv = (FAR struct nrf52_i2c_priv_s *)dev;
  uint32_t regval = 0;
  int      ret = OK;

  /* Reset ptr and dcnt */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  /* Do we need change I2C bus freqency ? */

  if (priv->msgv->frequency != priv->freq)
    {
      /* Get TWI frequency */

      switch (priv->msgv->frequency)
        {
            case 100000:
            {
              regval = TWIM_FREQUENCY_100KBPS;
              break;
            }

            case 250000:
            {
              regval = TWIM_FREQUENCY_250KBPS;
              break;
            }

            case 400000:
            {
              regval = TWIM_FREQUENCY_400KBPS;
              break;
            }

            default:
            {
              ret = -EINVAL;
              goto errout;
            }
        }

      /* Write TWI frequency */

      nrf52_i2c_putreg(priv, NRF52_TWIM_FREQUENCY_OFFSET, regval);

      /* Save the new I2C frequency */

      priv->freq = priv->msgv->frequency;
    }

  /* I2C transfer */

  do
    {
      /* Get current message data */

      priv->ptr   = priv->msgv->buffer;
      priv->dcnt  = priv->msgv->length;
      priv->flags = priv->msgv->flags;
      priv->addr  = priv->msgv->addr;

      /* Write TWI address */

      regval = priv->addr;
      nrf52_i2c_putreg(priv, NRF52_TWIM_ADDRESS_OFFSET, regval);

      if ((priv->flags & I2C_M_READ) == 0)
        {
          /* Write TXD data pointer */

          regval = (uint32_t)priv->ptr;
          nrf52_i2c_putreg(priv, NRF52_TWIM_TXDPTR_OFFSET, regval);

          /* Write number of bytes in TXD buffer */

          regval = priv->dcnt;
          nrf52_i2c_putreg(priv, NRF52_TWIM_TXMAXCNT_OFFSET, regval);

          /* Start TX sequence */

          nrf52_i2c_putreg(priv, NRF52_TWIM_TASKS_STARTTX_OFFSET, 1);

          /* Wait for last TX event */

#ifdef CONFIG_I2C_POLLED
          while (nrf52_i2c_getreg(priv,
                                  NRF52_TWIM_EVENTS_LASTTX_OFFSET) != 1);
#endif

          /* TWIM stop */

          nrf52_i2c_putreg(priv, NRF52_TWIM_TASKS_STOP_OFFSET, 1);

          /* Wait for stop event */

#ifdef CONFIG_I2C_POLLED
          while (nrf52_i2c_getreg(priv,
                                  NRF52_TWIM_EVENTS_STOPPED_OFFSET) != 1);
#endif
        }
      else
        {
          /* Write RXD data pointer */

          regval = (uint32_t)priv->ptr;
          nrf52_i2c_putreg(priv, NRF52_TWIM_RXDPTR_OFFSET, regval);

          /* Write number of bytes in RXD buffer */

          regval = priv->dcnt;
          nrf52_i2c_putreg(priv, NRF52_TWIM_RXDMAXCNT_OFFSET, regval);

          /* Start RX sequence */

          nrf52_i2c_putreg(priv, NRF52_TWIM_TASKS_STARTRX_OFFSET, 1);

          /* Wait for last RX done */

#ifdef CONFIG_I2C_POLLED
          while (nrf52_i2c_getreg(priv,
                                  NRF52_TWIM_EVENTS_LASTRX_OFFSET) != 1);
#endif
          /* Stop TWIM */

          nrf52_i2c_putreg(priv, NRF52_TWIM_TASKS_STOP_OFFSET, 1);

          /* Wait for stop event */

#ifdef CONFIG_I2C_POLLED
          while (nrf52_i2c_getreg(priv,
                                  NRF52_TWIM_EVENTS_STOPPED_OFFSET) != 1);
#endif
        }

      /* Next message */

      priv->msgc -= 1;
      priv->msgv += 1;
    }
  while (priv->msgc > 0);

errout:
    return ret;
}

/****************************************************************************
 * Name: nrf52_i2c_reset
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
static int nrf52_i2c_reset(FAR struct i2c_master_s *dev)
{
#error not implemented
}
#endif

/****************************************************************************
 * Name: nrf52_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int nrf52_i2c_isr(int irq, void *context, FAR void *arg)
{
#error not implemented
}
#endif

/****************************************************************************
 * Name: nrf52_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int nrf52_i2c_init(FAR struct nrf52_i2c_priv_s *priv)
{
  uint32_t regval = 0;
  int pin         = 0;
  int port        = 0;

  /* Configure SCL and SDA pins */

  nrf52_gpio_config(priv->scl_pin);
  nrf52_gpio_config(priv->sda_pin);

  /* Select SCL pin */

  pin = (priv->scl_pin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  port = (priv->scl_pin & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  regval = (pin << TWIM_PSELSCL_PIN_SHIFT);
  regval |= (port << TWIM_PSELSCL_PORT_SHIFT);
  nrf52_i2c_putreg(priv, NRF52_TWIM_PSELSCL_OFFSET, regval);

  /* Select SDA pin */

  pin = (priv->sda_pin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  port = (priv->sda_pin & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  regval = (pin << TWIM_PSELSDA_PIN_SHIFT);
  regval |= (port << TWIM_PSELSDA_PORT_SHIFT);
  nrf52_i2c_putreg(priv, NRF52_TWIM_PSELSDA_OFFSET, regval);

  /* Enable TWI interface */

  nrf52_i2c_putreg(priv, NRF52_TWIS_ENABLE_OFFSET, TWIM_ENABLE_EN);

#ifndef CONFIG_I2C_POLLED
  /* Attach error and event interrupts to the ISRs */

  irq_attach(priv->irq, nrf52_i2c_isr, priv);
  up_enable_irq(priv->irq);
#endif

  return OK;
}

/****************************************************************************
 * Name: nrf52_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ****************************************************************************/

static int nrf52_i2c_sem_init(FAR struct nrf52_i2c_priv_s *priv)
{
  /* Initialize semaphores */

  nxsem_init(&priv->sem_excl, 0, 1);

#ifndef CONFIG_I2C_POLLED
  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->sem_isr, 0, 0);
  nxsem_setprotocol(&priv->sem_isr, SEM_PRIO_NONE);
#endif

  return OK;
}

/****************************************************************************
 * Name: nrf52_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ****************************************************************************/

static int nrf52_i2c_sem_destroy(FAR struct nrf52_i2c_priv_s *priv)
{
  /* Release unused resources */

  nxsem_destroy(&priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&priv->sem_isr);
#endif

  return OK;
}

/****************************************************************************
 * Name: nrf52_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int nrf52_i2c_deinit(FAR struct nrf52_i2c_priv_s *priv)
{
  /* Enable TWI interface */

  nrf52_i2c_putreg(priv, TWIM_ENABLE_DIS, NRF52_TWIS_ENABLE_OFFSET);

  /* Unconfigure GPIO pins */

  nrf52_gpio_unconfig(priv->scl_pin);
  nrf52_gpio_unconfig(priv->sda_pin);

  /* Deatach TWI from GPIO */

  nrf52_i2c_putreg(priv, NRF52_TWIM_PSELSCL_OFFSET, TWIM_PSELSCL_RESET);
  nrf52_i2c_putreg(priv, NRF52_TWIM_PSELSDA_OFFSET, TWIM_PSELSDA_RESET);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

FAR struct i2c_master_s *nrf52_i2cbus_initialize(int port)
{
  FAR struct nrf52_i2c_priv_s *priv = NULL;
  irqstate_t flags;

  /* Get interface */

  switch (port)
    {
#ifdef CONFIG_NRF52_I2C0_MASTER
      case 0:
        {
          priv = (FAR struct nrf52_i2c_priv_s *)&g_nrf52_i2c0_priv;
          break;
        }
#endif

#ifdef CONFIG_NRF52_I2C1_MASTER
      case 1:
        {
          priv = (FAR struct nrf52_i2c_priv_s *)&g_nrf52_i2c1_priv;
          break;
        }
#endif

      default:
        {
          return NULL;
        }
    }

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  flags = enter_critical_section();

  if (priv->refs++ == 0)
    {
      /* Initialize sempaphores */

      nrf52_i2c_sem_init(priv);

      /* Initialize I2C */

      nrf52_i2c_init(priv);
    }

  leave_critical_section(flags);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: nrf52_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int nrf52_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct nrf52_i2c_priv_s *priv = (struct nrf52_i2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  /* Disable power and other HW resource (GPIO's) */

  nrf52_i2c_deinit(priv);

  /* Release semaphores */

  nrf52_i2c_sem_destroy(priv);

  return OK;
}
