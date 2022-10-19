/****************************************************************************
 * arch/arm/src/nrf52/nrf52_i2c.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nrf52_gpio.h"
#include "nrf52_i2c.h"

#include "hardware/nrf52_twi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C errors not functional yet */

#undef CONFIG_NRF52_I2C_ERRORS

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
  int                     status;   /* I2C transfer status */
#ifndef CONFIG_I2C_POLLED
  uint32_t                irq;      /* TWI interrupt */
#endif
  uint8_t                 msgc;     /* Message count */
  struct i2c_msg_s       *msgv;     /* Message list */
  uint8_t                *ptr;      /* Current message buffer */
#ifdef CONFIG_NRF52_I2C_MASTER_COPY_BUF_SIZE
  /* Static buffer used for continued messages */

  uint8_t                 copy_buf[CONFIG_NRF52_I2C_MASTER_COPY_BUF_SIZE];
#endif
  uint32_t                freq;      /* Current I2C frequency */
  int                     dcnt;      /* Current message length */
  uint16_t                flags;     /* Current message flags */
  uint16_t                addr;      /* Current I2C address */
  mutex_t                 lock;      /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t                   sem_isr;   /* Interrupt wait semaphore */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void nrf52_i2c_putreg(struct nrf52_i2c_priv_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf52_i2c_getreg(struct nrf52_i2c_priv_s *priv,
                                        uint32_t offset);
static int nrf52_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs,
                              int count);
#ifdef CONFIG_I2C_RESET
static int nrf52_i2c_reset(struct i2c_master_s *dev);
#endif
#ifndef CONFIG_I2C_POLLED
static int nrf52_i2c_isr(int irq, void *context, void *arg);
#endif
static int nrf52_i2c_deinit(struct nrf52_i2c_priv_s *priv);
static int nrf52_i2c_init(struct nrf52_i2c_priv_s *priv);

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
  .lock    = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr = SEM_INITIALIZER(0),
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
  .base    = NRF52_TWIM1_BASE,
  .scl_pin = BOARD_I2C1_SCL_PIN,
  .sda_pin = BOARD_I2C1_SDA_PIN,
  .refs    = 0,
  .lock    = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr = SEM_INITIALIZER(0),
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

static inline void nrf52_i2c_putreg(struct nrf52_i2c_priv_s *priv,
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

static inline uint32_t nrf52_i2c_getreg(struct nrf52_i2c_priv_s *priv,
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

static int nrf52_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs,
                              int count)
{
  struct nrf52_i2c_priv_s *priv = (struct nrf52_i2c_priv_s *)dev;
  uint32_t regval = 0;
  int      ret = OK;
#ifndef CONFIG_NRF52_I2C_MASTER_DISABLE_NOSTART
  uint8_t *pack_buf = NULL;
#endif

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Reset ptr and dcnt */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C transfer status */

  priv->status = OK;

  i2cinfo("I2C TRANSFER count=%d\n", count);

  /* Do we need change I2C bus frequency ? */

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

      i2cinfo("ptr=%p dcnt=%d flags=%d addr=%d\n",
              priv->ptr, priv->dcnt, priv->flags, priv->addr);

      /* Write TWI address */

      regval = priv->addr;
      nrf52_i2c_putreg(priv, NRF52_TWIM_ADDRESS_OFFSET, regval);

      if ((priv->flags & I2C_M_READ) == 0)
        {
#ifndef CONFIG_NRF52_I2C_MASTER_DISABLE_NOSTART
          /* Check if we need to combine messages */

          if (priv->msgc > 1)
            {
              if (priv->msgv[1].flags & I2C_M_NOSTART)
                {
                  /* More than 2 messages not supported */

                  DEBUGASSERT(priv->msgc < 3);

                  /* Combine buffers */

                  if ((priv->msgv[0].length +
                       priv->msgv[1].length) <=
                      CONFIG_NRF52_I2C_MASTER_COPY_BUF_SIZE)
                    {
                      pack_buf = priv->copy_buf;
                    }
                  else
                    {
                      pack_buf = kmm_malloc(priv->msgv[0].length +
                                            priv->msgv[1].length);
                      if (pack_buf == NULL)
                        {
                          return -ENOMEM;
                        }
                    }

                  /* Combine messages */

                  memcpy(pack_buf, priv->msgv[0].buffer,
                         priv->msgv[0].length);
                  memcpy(pack_buf + priv->msgv[0].length,
                         priv->msgv[1].buffer, priv->msgv[1].length);

                  /* Use new buffer to transmit data */

                  priv->ptr  = pack_buf;
                  priv->dcnt = priv->msgv[0].length + priv->msgv[1].length;

                  /* Next message */

                  priv->msgc -= 1;
                  priv->msgv += 1;
                }
            }
#else
          if (priv->msgc > 1)
            {
              if (priv->msgv[1].flags & I2C_M_NOSTART)
                {
                  /* Not supported */

                  DEBUGPANIC();
                }
            }
#endif

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
          while (1)
            {
              regval = nrf52_i2c_getreg(priv,
                                        NRF52_TWIM_ERRORSRC_OFFSET) & 0x7;
              if (regval != 0)
                {
                  i2cerr("Error SRC: 0x%08" PRIx32 "\n", regval);
                  ret = -1;
                  nrf52_i2c_putreg(priv,
                                  NRF52_TWIM_ERRORSRC_OFFSET, 0x7);
                  goto errout;
                }

              if (nrf52_i2c_getreg(priv,
                                  NRF52_TWIM_EVENTS_LASTTX_OFFSET) == 1)
                {
                  break;
                }
            }

          /* Clear event */

          nrf52_i2c_putreg(priv, NRF52_TWIM_EVENTS_LASTTX_OFFSET, 0);
#else
          nxsem_wait(&priv->sem_isr);

          if (priv->status < 0)
            {
              goto errout;
            }
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
        while (1)
          {
            regval = nrf52_i2c_getreg(priv,
                                      NRF52_TWIM_ERRORSRC_OFFSET) & 0x7;
            if (regval != 0)
              {
                i2cerr("Error SRC: 0x%08" PRIx32 "\n", regval);
                ret = -1;
                nrf52_i2c_putreg(priv,
                                 NRF52_TWIM_ERRORSRC_OFFSET, 0x7);
                goto errout;
              }

            if (nrf52_i2c_getreg(priv,
                                 NRF52_TWIM_EVENTS_LASTRX_OFFSET) == 1)
              {
                break;
              }
          }

          /* Clear event */

          nrf52_i2c_putreg(priv, NRF52_TWIM_EVENTS_LASTRX_OFFSET, 0);
#else
          nxsem_wait(&priv->sem_isr);

          if (priv->status < 0)
            {
              goto errout;
            }
#endif
        }

      /* Next message */

      priv->msgc -= 1;
      priv->msgv += 1;
    }
  while (priv->msgc > 0);

  /* TWIM stop */

  nrf52_i2c_putreg(priv, NRF52_TWIM_TASKS_STOP_OFFSET, 1);

  /* Wait for stop event */

#ifdef CONFIG_I2C_POLLED
  while (1)
    {
      regval = nrf52_i2c_getreg(priv,
                                NRF52_TWIM_ERRORSRC_OFFSET) & 0x7;
      if (regval != 0)
        {
          i2cerr("Error SRC: 0x%08" PRIx32 "\n", regval);
          ret = -1;
          nrf52_i2c_putreg(priv,
                           NRF52_TWIM_ERRORSRC_OFFSET, 0x7);
          goto errout;
        }

      if (nrf52_i2c_getreg(priv,
                           NRF52_TWIM_EVENTS_STOPPED_OFFSET) == 1)
        {
          break;
        }
    }

  /* Clear event */

  nrf52_i2c_putreg(priv, NRF52_TWIM_EVENTS_STOPPED_OFFSET, 0);
#else
  nxsem_wait(&priv->sem_isr);

  if (priv->status < 0)
    {
      goto errout;
    }
#endif

errout:
#ifndef CONFIG_NRF52_I2C_MASTER_DISABLE_NOSTART
  if (pack_buf != NULL && pack_buf != priv->copy_buf)
    {
      kmm_free(pack_buf);
    }
#endif

  nxmutex_unlock(&priv->lock);
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
static int nrf52_i2c_reset(struct i2c_master_s *dev)
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
static int nrf52_i2c_isr(int irq, void *context, void *arg)
{
  struct nrf52_i2c_priv_s *priv = (struct nrf52_i2c_priv_s *)arg;

  /* Reset I2C status */

  priv->status = OK;

  if ((priv->flags & I2C_M_READ) == 0)
    {
      if (nrf52_i2c_getreg(priv, NRF52_TWIM_EVENTS_LASTTX_OFFSET) == 1)
        {
          i2cinfo("I2C LASTTX\n");

          /* TX done */

          nxsem_post(&priv->sem_isr);

          /* Clear event */

          nrf52_i2c_putreg(priv, NRF52_TWIM_EVENTS_LASTTX_OFFSET, 0);

          return OK;
        }
    }
  else
    {
      if (nrf52_i2c_getreg(priv, NRF52_TWIM_EVENTS_LASTRX_OFFSET) == 1)
        {
          i2cinfo("I2C LASTRX\n");

          /* RX done */

          nxsem_post(&priv->sem_isr);

          /* Clear event */

          nrf52_i2c_putreg(priv, NRF52_TWIM_EVENTS_LASTRX_OFFSET, 0);

          return OK;
        }
    }

  if (nrf52_i2c_getreg(priv, NRF52_TWIM_EVENTS_STOPPED_OFFSET) == 1)
    {
      i2cinfo("I2C STOPPED\n");

      /* STOPPED event */

      nxsem_post(&priv->sem_isr);

      /* Clear event */

      nrf52_i2c_putreg(priv, NRF52_TWIM_EVENTS_STOPPED_OFFSET, 0);
    }

#ifdef CONFIG_NRF52_I2C_ERRORS
  if (nrf52_i2c_getreg(priv, NRF52_TWIM_EVENTS_ERROR_OFFSET) == 1)
    {
      i2cerr("I2C ERROR\n");

      /* Set ERROR status */

      priv->status = ERROR;

      /* ERROR event */

      nxsem_post(&priv->sem_isr);

      /* Clear event */

      nrf52_i2c_putreg(priv, NRF52_TWIM_EVENTS_ERROR_OFFSET, 0);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: nrf52_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int nrf52_i2c_init(struct nrf52_i2c_priv_s *priv)
{
  uint32_t regval = 0;
  int pin         = 0;
  int port        = 0;

  /* Disable TWI interface */

  nrf52_i2c_putreg(priv, NRF52_TWIM_ENABLE_OFFSET, TWIM_ENABLE_DIS);

  /* Configure SCL and SDA pins */

  nrf52_gpio_config(priv->scl_pin);
  nrf52_gpio_config(priv->sda_pin);

  /* Select SCL pin */

  pin  = GPIO_PIN_DECODE(priv->scl_pin);
  port = GPIO_PORT_DECODE(priv->scl_pin);

  regval = (pin << TWIM_PSELSCL_PIN_SHIFT);
  regval |= (port << TWIM_PSELSCL_PORT_SHIFT);
  nrf52_i2c_putreg(priv, NRF52_TWIM_PSELSCL_OFFSET, regval);

  /* Select SDA pin */

  pin  = GPIO_PIN_DECODE(priv->sda_pin);
  port = GPIO_PORT_DECODE(priv->sda_pin);

  regval = (pin << TWIM_PSELSDA_PIN_SHIFT);
  regval |= (port << TWIM_PSELSDA_PORT_SHIFT);
  nrf52_i2c_putreg(priv, NRF52_TWIM_PSELSDA_OFFSET, regval);

  /* Enable TWI interface */

  nrf52_i2c_putreg(priv, NRF52_TWIM_ENABLE_OFFSET, TWIM_ENABLE_EN);

#ifndef CONFIG_I2C_POLLED
  /* Enable I2C interrupts */

#ifdef CONFIG_NRF52_I2C_ERRORS
  regval = (TWIM_INT_LASTRX | TWIM_INT_LASTTX | TWIM_INT_STOPPED |
            TWIM_INT_ERROR);
#else
  regval = (TWIM_INT_LASTRX | TWIM_INT_LASTTX | TWIM_INT_STOPPED);
#endif
  nrf52_i2c_putreg(priv, NRF52_TWIM_INTEN_OFFSET, regval);

  /* Attach error and event interrupts to the ISRs */

  irq_attach(priv->irq, nrf52_i2c_isr, priv);
  up_enable_irq(priv->irq);
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

static int nrf52_i2c_deinit(struct nrf52_i2c_priv_s *priv)
{
  /* Disable TWI interface */

  nrf52_i2c_putreg(priv, NRF52_TWIM_ENABLE_OFFSET, TWIM_ENABLE_DIS);

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

struct i2c_master_s *nrf52_i2cbus_initialize(int port)
{
  struct nrf52_i2c_priv_s *priv = NULL;

  i2cinfo("I2C INITIALIZE port=%d\n", port);

  /* Get interface */

  switch (port)
    {
#ifdef CONFIG_NRF52_I2C0_MASTER
      case 0:
        {
          priv = (struct nrf52_i2c_priv_s *)&g_nrf52_i2c0_priv;
          break;
        }
#endif

#ifdef CONFIG_NRF52_I2C1_MASTER
      case 1:
        {
          priv = (struct nrf52_i2c_priv_s *)&g_nrf52_i2c1_priv;
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

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      /* Initialize I2C */

      nrf52_i2c_init(priv);
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: nrf52_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int nrf52_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct nrf52_i2c_priv_s *priv = (struct nrf52_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Disable power and other HW resource (GPIO's) */

  nrf52_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}
