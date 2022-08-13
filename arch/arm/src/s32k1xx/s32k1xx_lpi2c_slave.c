/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lpi2c_slave.c
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_slave.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_lpi2c.h"
#include "s32k1xx_lpi2c_slave.h"
#include "s32k1xx_periphclocks.h"

#include <arch/board/board.h>

/* At least one I2C peripheral must be enabled, as well as the I2C slave */

#if defined(CONFIG_S32K1XX_LPI2C) && defined(CONFIG_I2C_SLAVE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_S32K11X
#warning LPI2C slave logic does not support S32K11X (yet)
#endif

#ifdef CONFIG_I2C_POLLED
#warning LPI2C slave logic does not support polling (yet)
#endif

#ifdef CONFIG_I2C_TRACE
#warning LPI2C slave logic does not support I2C trace debugging (yet)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C slave device hardware configuration */

struct s32k1xx_lpi2c_slave_config_s
{
  uint32_t base;      /* LPI2C base address */
  bool     slave_bus; /* Separate I2C slave bus? */
  uint32_t scl_pin;   /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;   /* GPIO configuration for SDA as SDA */
  uint32_t irq;       /* Event IRQ */
};

/* I2C slave device private data */

struct s32k1xx_lpi2c_slave_priv_s
{
  const struct i2c_slaveops_s *ops;                  /* I2C slave operations */
  const struct s32k1xx_lpi2c_slave_config_s *config; /* LPI2C slave configuration */

  int slave_addr; /* I2C address of the slave */
  int addr_nbits; /* 7- or 10-bit addressing */

  uint8_t *read_buffer; /* Read buffer (master wants to write, slave will read data) */
  int read_buflen;      /* Read buffer size */
  int read_bufindex;    /* Read buffer index */

  const uint8_t *write_buffer; /* Write buffer (master wants to read, slave will write data) */
  int write_buflen;            /* Write buffer size */
  int write_bufindex;          /* Write buffer index */

  i2c_slave_callback_t *callback; /* Callback function when data has been received */
  void *callback_arg;             /* Argument of callback function */

  int refs; /* Reference count */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t s32k1xx_lpi2c_slave_getreg(
  struct s32k1xx_lpi2c_slave_priv_s *priv, uint16_t offset);
static inline void s32k1xx_lpi2c_slave_putreg(
  struct s32k1xx_lpi2c_slave_priv_s *priv, uint16_t offset,
  uint32_t value);
static inline void s32k1xx_lpi2c_slave_modifyreg(
  struct s32k1xx_lpi2c_slave_priv_s *priv, uint16_t offset,
  uint32_t clearbits, uint32_t setbits);

static int s32k1xx_lpi2c_slave_isr_process(
  struct s32k1xx_lpi2c_slave_priv_s *priv);
static int s32k1xx_lpi2c_slave_isr(int irq, void *context, void *arg);

static int s32k1xx_lpi2c_slave_init(
  struct s32k1xx_lpi2c_slave_priv_s *priv);
static int s32k1xx_lpi2c_slave_deinit(
  struct s32k1xx_lpi2c_slave_priv_s *priv);

static int s32k1xx_lpi2c_setownaddress(struct i2c_slave_s *dev, int addr,
                                       int nbits);
static int s32k1xx_lpi2c_write(struct i2c_slave_s *dev,
                               const uint8_t *buffer, int buflen);
static int s32k1xx_lpi2c_read(struct i2c_slave_s *dev,
                              uint8_t *buffer, int buflen);
static int s32k1xx_lpi2c_registercallback(struct i2c_slave_s *dev,
                                          i2c_slave_callback_t *callback,
                                          void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C slave interface */

static const struct i2c_slaveops_s s32k1xx_lpi2c_slaveops =
{
  .setownaddress    = s32k1xx_lpi2c_setownaddress,
  .write            = s32k1xx_lpi2c_write,
  .read             = s32k1xx_lpi2c_read,
  .registercallback = s32k1xx_lpi2c_registercallback,
};

/* I2C device structures */

#ifdef CONFIG_S32K1XX_LPI2C0
static const struct s32k1xx_lpi2c_slave_config_s s32k1xx_lpi2c0s_config =
{
  .base      = S32K1XX_LPI2C0_BASE,

#ifdef CONFIG_LPI2C0_SLAVE_BUS
  .slave_bus = true,
  .scl_pin   = PIN_LPI2C0_SCLS,
  .sda_pin   = PIN_LPI2C0_SDAS,
#else
  .slave_bus = false,
  .scl_pin   = PIN_LPI2C0_SCL,
  .sda_pin   = PIN_LPI2C0_SDA,
#endif

  .irq       = S32K1XX_IRQ_LPI2C0S,
};

static struct s32k1xx_lpi2c_slave_priv_s s32k1xx_lpi2c0s_priv =
{
  .ops            = &s32k1xx_lpi2c_slaveops,
  .config         = &s32k1xx_lpi2c0s_config,
  .slave_addr     = CONFIG_LPI2C0_SLAVE_ADDRESS,
  .addr_nbits     = 7,
  .read_buffer    = NULL,
  .read_buflen    = 0,
  .read_bufindex  = 0,
  .write_buffer   = NULL,
  .write_buflen   = 0,
  .write_bufindex = 0,
  .callback       = NULL,
  .callback_arg   = NULL,
  .refs           = 0,
};
#endif /* CONFIG_S32K1XX_LPI2C0 */

#ifdef CONFIG_S32K1XX_LPI2C1
static const struct s32k1xx_lpi2c_slave_config_s s32k1xx_lpi2c1s_config =
{
  .base      = S32K1XX_LPI2C1_BASE,

#ifdef CONFIG_LPI2C1_SLAVE_BUS
  .slave_bus = true,
  .scl_pin   = PIN_LPI2C1S_SCL,
  .sda_pin   = PIN_LPI2C1S_SDA,
#else
  .slave_bus = false,
  .scl_pin   = PIN_LPI2C1_SCL,
  .sda_pin   = PIN_LPI2C1_SDA,
#endif

  .irq       = S32K1XX_IRQ_LPI2C1S,
};

static struct s32k1xx_lpi2c_slave_priv_s s32k1xx_lpi2c1s_priv =
{
  .ops            = &s32k1xx_lpi2c_slaveops,
  .config         = &s32k1xx_lpi2c1s_config,
  .slave_addr     = CONFIG_LPI2C1_SLAVE_ADDRESS,
  .addr_nbits     = 7,
  .read_buffer    = NULL,
  .read_buflen    = 0,
  .read_bufindex  = 0,
  .write_buffer   = NULL,
  .write_buflen   = 0,
  .write_bufindex = 0,
  .callback       = NULL,
  .callback_arg   = NULL,
  .refs           = 0,
};
#endif /* CONFIG_S32K1XX_LPI2C1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_lpi2c_slave_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 * Input Parameters:
 *   priv   - I2C slave device private data
 *   offset - Register offset with respect to the base address of the I2C
 *            peripheral
 *
 * Returned Value:
 *   The 32-bit value retrieved from the register
 *
 ****************************************************************************/

static inline uint32_t s32k1xx_lpi2c_slave_getreg(
  struct s32k1xx_lpi2c_slave_priv_s *priv, uint16_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_slave_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 * Input Parameters:
 *   priv   - I2C slave device private data
 *   offset - Register offset with respect to the base address of the I2C
 *            peripheral
 *   value  - The 32-bit value that should be put into the register
 *
 ****************************************************************************/

static inline void s32k1xx_lpi2c_slave_putreg(
  struct s32k1xx_lpi2c_slave_priv_s *priv, uint16_t offset,
  uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_slave_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 * Input Parameters:
 *   priv      - I2C slave device private data
 *   offset    - Register offset with respect to the base address of the I2C
 *               peripheral
 *   clearbits - Bitmask with the bits that should be cleared (put to 0)
 *   setbits   - Bitmask with the bits that should be set (put to 1)
 *
 ****************************************************************************/

static inline void s32k1xx_lpi2c_slave_modifyreg(
  struct s32k1xx_lpi2c_slave_priv_s *priv, uint16_t offset,
  uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_slave_isr_process
 *
 * Description:
 *   Process LPI2C slave interrupts.  Check for relevant flags and read or
 *   write data to buffers.  After a block of data has been received a
 *   callback function (if any) may be invoked, which might install a new
 *   write buffer to transmit data when requested.
 *
 * Input Parameters:
 *   priv - I2C slave device private data
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_slave_isr_process(
  struct s32k1xx_lpi2c_slave_priv_s *priv)
{
  uint32_t status = s32k1xx_lpi2c_slave_getreg(priv,
                                               S32K1XX_LPI2C_SSR_OFFSET);

  /* Slave Address Valid Flag */

  if (status & LPI2C_SSR_AVF)
    {
      /* A new transfer was initiated by a bus master.  The transfer request
       * was addressed to this particular device.  It needs to be checked if
       * the master wants to read or write.
       */

      uint16_t address = (uint16_t) (s32k1xx_lpi2c_slave_getreg(priv,
        S32K1XX_LPI2C_SASR_OFFSET) & LPI2C_SASR_RADDR_MASK);

      if (address & I2CS_READBIT)
        {
          /* Master wants to read, so the slave needs to write/transmit.
           * The Transmit Data Interrupt will only be enabled now, see
           * erratum 10792.  Also reset the buffer index back to zero to
           * start sending again from the beginning.
           */

          s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_SIER_OFFSET, 0,
                                        LPI2C_SIER_TDIE);

          priv->write_bufindex = 0;
        }
      else
        {
          /* Master wants to write, so the slave needs to read.  The Receive
           * Data Interrupt remains enabled, so just reset the buffer index.
           */

          priv->read_bufindex = 0;
        }
    }

  /* Slave Transmits Data Flag (master wants to read) */

  if (status & LPI2C_SSR_TDF)
    {
      /* Make sure that interrupts are enabled for this event */

      if (s32k1xx_lpi2c_slave_getreg(priv,
        S32K1XX_LPI2C_SIER_OFFSET) & LPI2C_SIER_TDIE)
        {
          if (priv->write_buflen > priv->write_bufindex)
            {
              /* Transmit data from buffer */

              s32k1xx_lpi2c_slave_putreg(priv, S32K1XX_LPI2C_STDR_OFFSET,
                (uint32_t) priv->write_buffer[priv->write_bufindex]);
              priv->write_bufindex++;
            }
          else
            {
              /* Beyond the buffer length.  Transmit dummy data... */

              s32k1xx_lpi2c_slave_putreg(priv, S32K1XX_LPI2C_STDR_OFFSET, 0);
            }
        }
    }

  /* Slave Receives Data (master wants to write) */

  if (status & LPI2C_SSR_RDF)
    {
      /* Make sure that interrupts are enabled for this event */

      if (s32k1xx_lpi2c_slave_getreg(priv,
        S32K1XX_LPI2C_SIER_OFFSET) & LPI2C_SIER_RDIE)
        {
          if (priv->read_buflen > priv->read_bufindex)
            {
              /* Read data into buffer */

              priv->read_buffer[priv->read_bufindex] =
                (uint8_t) s32k1xx_lpi2c_slave_getreg(priv,
                  S32K1XX_LPI2C_SRDR_OFFSET);
              priv->read_bufindex++;
            }
          else
            {
              /* Dummy read, throw away the data */

              s32k1xx_lpi2c_slave_getreg(priv, S32K1XX_LPI2C_SRDR_OFFSET);
            }
        }
    }

  /* Stop or Repeated Start (current transfer is over) */

  if (s32k1xx_lpi2c_slave_getreg(priv, S32K1XX_LPI2C_SSR_OFFSET) & \
      (LPI2C_SSR_SDF | LPI2C_SSR_RSF))
    {
      /* Clear Stop Detect / Repeated Start Flags */

      s32k1xx_lpi2c_slave_putreg(priv, S32K1XX_LPI2C_SSR_OFFSET,
                                 LPI2C_SSR_SDF | LPI2C_SSR_RSF);

      /* Disable the Transmit Data Interrupt again, see erratum 10792 */

      s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_SIER_OFFSET,
                                    LPI2C_SIER_TDIE, 0);

      /* Execute the registered callback function if data was received */

      if ((priv->read_bufindex > 0) && (priv->callback != NULL))
        {
          priv->callback(priv->callback_arg, priv->read_bufindex);
          priv->read_bufindex = 0;
        }
    }

  /* Slave Bit Error (abort current transfer) */

  if (s32k1xx_lpi2c_slave_getreg(priv, S32K1XX_LPI2C_SSR_OFFSET) & \
      LPI2C_SSR_BEF)
    {
      /* Clear Bit Error Flag */

      s32k1xx_lpi2c_slave_putreg(priv, S32K1XX_LPI2C_SSR_OFFSET,
                                 LPI2C_SSR_BEF);

      /* Disable the Transmit Data Interrupt again, see erratum 10792 */

      s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_SIER_OFFSET,
                                    LPI2C_SIER_TDIE, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_slave_isr
 *
 * Description:
 *   Interrupt Service Routine for LPI2C slave devices.  Retrieves the
 *   private data from the argument, further processing is done by
 *   s32k1xx_lpi2c_slave_isr_process().
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg     - I2C slave device private data
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_slave_isr(int irq, void *context, void *arg)
{
  struct s32k1xx_lpi2c_slave_priv_s *priv =
    (struct s32k1xx_lpi2c_slave_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  return s32k1xx_lpi2c_slave_isr_process(priv);
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_slave_init
 *
 * Description:
 *   Initialize the LPI2C slave device.  Enable and configure the peripheral
 *   and enable the interrupts and attach handlers.
 *
 * Input Parameters:
 *   priv - I2C slave device private data
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_slave_init(
  struct s32k1xx_lpi2c_slave_priv_s *priv)
{
  int ret;

  /* Reset LPI2C slave mode logic before configuring it */

  s32k1xx_lpi2c_slave_putreg(priv, S32K1XX_LPI2C_SCR_OFFSET, LPI2C_SCR_RST);
  s32k1xx_lpi2c_slave_putreg(priv, S32K1XX_LPI2C_SCR_OFFSET, 0);

  /* Configure pins and power up peripheral.
   *
   * NOTE: Clocking to the LPI2C peripheral must be provided by
   * board-specific logic as part of the clock configuration logic.
   */

  ret = s32k1xx_pinconfig(priv->config->scl_pin);
  if (ret != OK)
    {
      return ret;
    }

  ret = s32k1xx_pinconfig(priv->config->sda_pin);
  if (ret != OK)
    {
      return ret;
    }

  /* Choose between a combined or separated LPI2C master and slave.  When a
   * separate slave bus is selected, the LPI2C slave will use different pins
   * than the LPI2C master.  These pins should be defined in board.h
   */

  if (priv->config->slave_bus)
    {
      s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_MCFGR1_OFFSET,
        LPI2C_MCFGR1_PINCFG_MASK, LPI2C_MCFGR1_PINCFG4);
    }
  else
    {
      s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_MCFGR1_OFFSET,
        LPI2C_MCFGR1_PINCFG_MASK, LPI2C_MCFGR1_PINCFG0);
    }

  /* Configure slave address
   *
   * TO DO: Allow 10-bit addressing
   */

  s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_SAMR_OFFSET, 0,
                                LPI2C_SAMR_ADDR0(priv->slave_addr));

  /* Enable clock stretching */

  s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_SCFGR1_OFFSET, 0,
    LPI2C_SCFGR1_ADRSTALL | LPI2C_SCFGR1_RXSTALL | LPI2C_SCFGR1_TXSTALL);

  /* Configure LPI2C slave interrupts */

  s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_SIER_OFFSET, 0,
    LPI2C_SIER_RDIE | LPI2C_SIER_AVIE | LPI2C_SIER_RSIE | LPI2C_SIER_SDIE | \
    LPI2C_SIER_BEIE);

  /* Attach ISR and enable interrupt */

  ret = irq_attach(priv->config->irq, s32k1xx_lpi2c_slave_isr, priv);
  if (ret != OK)
    {
      return ret;
    }

  up_enable_irq(priv->config->irq);

  /* Enable I2C slave */

  s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_SCR_OFFSET, 0,
                                LPI2C_SCR_SEN);

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_slave_deinit
 *
 * Description:
 *   Deinitialize the LPI2C slave device.  Disable and reset the peripheral
 *   and disable the interrupts and attached handlers.
 *
 * Input Parameters:
 *   priv - I2C slave device private data
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_slave_deinit(
  struct s32k1xx_lpi2c_slave_priv_s *priv)
{
  int ret;

  /* Disable LPI2C slave */

  s32k1xx_lpi2c_slave_modifyreg(priv, S32K1XX_LPI2C_SCR_OFFSET,
                                LPI2C_SCR_SEN, 0);

  /* Reset LPI2C slave */

  s32k1xx_lpi2c_slave_putreg(priv, S32K1XX_LPI2C_SCR_OFFSET, LPI2C_SCR_RST);
  s32k1xx_lpi2c_slave_putreg(priv, S32K1XX_LPI2C_SCR_OFFSET, 0);

  /* Disable and detach interrupts */

  up_disable_irq(priv->config->irq);
  ret = irq_detach(priv->config->irq);
  if (ret != OK)
    {
      return ret;
    }

  /* NOTE that clocking is left enabled */

  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_lpi2c_setownaddress
 *
 * Description:
 *   Set our own I2C address.
 *
 *   One may register a callback to be notified about reception. During the
 *   slave mode reception, the methods READ and WRITE must be used to
 *   to handle reads and writes from a master.
 *
 * Input Parameters:
 *   dev     - I2C slave device-specific state data
 *   address - Our own slave address
 *   nbits   - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_setownaddress(struct i2c_slave_s *dev, int addr,
                                       int nbits)
{
  struct s32k1xx_lpi2c_slave_priv_s *priv;
  irqstate_t flags;

  DEBUGASSERT(dev);
  priv = (struct s32k1xx_lpi2c_slave_priv_s *)dev;

  flags = enter_critical_section();

  /* Deinit slave before we change its configuration */

  int ret = s32k1xx_lpi2c_slave_deinit(priv);
  if (ret != OK)
    {
      leave_critical_section(flags);
      return ret;
    }

  /* Modify configuration */

  switch (nbits)
    {
      case 7:
        {
          priv->slave_addr = (addr & 0x7f);
          priv->addr_nbits = 7;
        }
        break;

      case 10:
        {
          priv->slave_addr = (addr & 0x03ff);
          priv->addr_nbits = 10;
        }
        break;

      default:
        {
          leave_critical_section(flags);
          return ERROR;
        }
        break;
    }

  /* Reinitialize slave with the changed config */

  s32k1xx_lpi2c_slave_init(priv);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_write
 *
 * Description:
 *   Send a block of data on I2C when a bus master wants to read data from
 *   this particular device.
 *
 * Input Parameters:
 *   dev    - I2C slave device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to the
 *            device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_write(struct i2c_slave_s *dev,
                               const uint8_t *buffer, int buflen)
{
  struct s32k1xx_lpi2c_slave_priv_s *priv;
  irqstate_t flags;

  DEBUGASSERT(dev);
  priv = (struct s32k1xx_lpi2c_slave_priv_s *)dev;

  flags = enter_critical_section();

  /* Update the registered buffer and length */

  priv->write_buffer = buffer;
  priv->write_buflen = buflen;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_read
 *
 * Description:
 *   Receive a block of data from I2C when a bus master writes data addressed
 *   to this particular device.
 *
 * Input Parameters:
 *   dev    - I2C slave device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the
 *            device
 *   buflen - The maximum size of the buffer
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_read(struct i2c_slave_s *dev,
                              uint8_t *buffer, int buflen)
{
  struct s32k1xx_lpi2c_slave_priv_s *priv;
  irqstate_t flags;

  DEBUGASSERT(dev);
  priv = (struct s32k1xx_lpi2c_slave_priv_s *)dev;

  flags = enter_critical_section();

  /* Update the registered buffer and length */

  priv->read_buffer = buffer;
  priv->read_buflen = buflen;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_lpi2c_registercallback
 *
 * Description:
 *   Register a callback function that will be invoked when something is
 *   received on I2C.
 *
 * Input Parameters:
 *   dev      - I2C slave device-specific state data
 *   callback - The function to be called when something has been received.
 *   arg      - User provided argument to be used with the callback
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int s32k1xx_lpi2c_registercallback(struct i2c_slave_s *dev,
                                          i2c_slave_callback_t *callback,
                                          void *arg)
{
  struct s32k1xx_lpi2c_slave_priv_s *priv;
  irqstate_t flags;

  DEBUGASSERT(dev);
  priv = (struct s32k1xx_lpi2c_slave_priv_s *)dev;

  flags = enter_critical_section();

  /* Update the registered callback and argument */

  priv->callback = callback;
  priv->callback_arg = arg;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_i2cbus_slave_initialize
 *
 * Description:
 *   Initialize the I2C slave device and increase the reference counter.
 *   If the device has already been initialized only the reference counter
 *   will be increased.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple I2C interfaces).
 *
 * Returned Value:
 *   A valid I2C device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

struct i2c_slave_s *s32k1xx_i2cbus_slave_initialize(int port)
{
  struct s32k1xx_lpi2c_slave_priv_s *priv;
  irqstate_t flags;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_S32K1XX_LPI2C0
      case 0:
        priv = (struct s32k1xx_lpi2c_slave_priv_s *)&s32k1xx_lpi2c0s_priv;
        break;
#endif

#ifdef CONFIG_S32K1XX_LPI2C1
      case 1:
        priv = (struct s32k1xx_lpi2c_slave_priv_s *)&s32k1xx_lpi2c1s_priv;
        break;
#endif

      default:
        return NULL;
    }

  flags = enter_critical_section();

  if ((volatile int) priv->refs == 0)
    {
      /* Initialize private data for the first time, increment reference
       * count, power-up hardware and configure pins.
       */

      s32k1xx_lpi2c_slave_init(priv);
    }

  priv->refs++;

  leave_critical_section(flags);

  return (struct i2c_slave_s *)priv;
}

/****************************************************************************
 * Name: s32k1xx_i2cbus_slave_uninitialize
 *
 * Description:
 *   Decrease the reference counter of the I2C slave device.  When there are
 *   no more references left the I2C slave device is unitialized.
 *
 * Input Parameters:
 *   dev - Device structure as returned by s32k1xx_i2cbus_slave_initialize().
 *
 * Returned Value:
 *   OK on success, ERROR when there is an internal reference count mismatch
 *   or dev points to an invalid hardware device.
 *
 ****************************************************************************/

int s32k1xx_i2cbus_slave_uninitialize(struct i2c_slave_s *dev)
{
  struct s32k1xx_lpi2c_slave_priv_s *priv =
    (struct s32k1xx_lpi2c_slave_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Check reference count for underflow, then decrement */

  flags = enter_critical_section();

  if (priv->refs == 0)
    {
      leave_critical_section(flags);
      return ERROR;
    }

  priv->refs--;

  if (priv->refs > 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  /* Disable power and other HW resources (pins) */

  s32k1xx_lpi2c_slave_deinit(priv);

  return OK;
}

#endif /* CONFIG_S32K1XX_LPI2C && CONFIG_I2C_SLAVE */
