/****************************************************************************
 * drivers/input/stmpe811_base.c
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

/* References:
 *   "STMPE811 S-Touch advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/input/stmpe811.h>

#include "stmpe811.h"

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE811)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* If only a single STMPE811 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_STMPE811_MULTIPLE
static struct stmpe811_dev_s g_stmpe811;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct stmpe811_dev_s *g_stmpe811list;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_worker
 *
 * Description:
 *   This is the "bottom half" of the STMPE811 interrupt handler
 *
 ****************************************************************************/

static void stmpe811_worker(FAR void *arg)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)arg;
  uint8_t regval;

  DEBUGASSERT(priv && priv->config);

  /* Get the global interrupt status */

  regval =  stmpe811_getreg8(priv, STMPE811_INT_STA);

  /* Check for a touchscreen interrupt */

#ifndef CONFIG_STMPE811_TSC_DISABLE
  if ((regval & (INT_TOUCH_DET | INT_FIFO_TH | INT_FIFO_OFLOW)) != 0)
    {
      /* Dispatch the touchscreen interrupt if it was brought into the link */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
      if (stmpe811_tscworker)
#endif
        {
           stmpe811_tscworker(priv, regval);
        }

      stmpe811_putreg8(priv, STMPE811_INT_STA,
                       (INT_TOUCH_DET | INT_FIFO_TH | INT_FIFO_OFLOW));
      regval &= ~(INT_TOUCH_DET | INT_FIFO_TH | INT_FIFO_OFLOW);
    }
#endif

#if !defined(CONFIG_STMPE811_GPIO_DISABLE) && !defined(CONFIG_STMPE811_GPIOINT_DISABLE)
  if ((regval & INT_GPIO) != 0)
    {
      /* Dispatch the GPIO interrupt if it was brought into the link */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
      if (stmpe811_gpioworker)
#endif
        {
          stmpe811_gpioworker(priv);
        }

      stmpe811_putreg8(priv, STMPE811_INT_STA, INT_GPIO);
      regval &= ~INT_GPIO;
    }
#endif

  /* Clear any other residual, unhandled pending interrupt */

  if (regval != 0)
    {
      stmpe811_putreg8(priv, STMPE811_INT_STA, regval);
    }

  /* Re-enable the STMPE811 GPIO interrupt */

  priv->config->enable(priv->config, true);
}

/****************************************************************************
 * Name: stmpe811_interrupt
 *
 * Description:
 *  The STMPE811 interrupt handler
 *
 ****************************************************************************/

static int stmpe811_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stmpe811_dev_s    *priv = (FAR struct stmpe811_dev_s *)arg;
  FAR struct stmpe811_config_s *config;
  int                           ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Disable further interrupts */

  config->enable(config, false);

  /* Check if interrupt work is already queue.  If it is already busy, then
   * we already have interrupt processing in the pipeline and we need to do
   * nothing more.
   */

  if (work_available(&priv->work))
    {
      /* Yes.. Transfer processing to the worker thread.  Since STMPE811
       * interrupts are disabled while the work is pending, no special
       * action should be required to protect the work queue.
       */

      ret = work_queue(HPWORK, &priv->work, stmpe811_worker, priv, 0);
      if (ret != 0)
        {
          ierr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return OK;
}

/****************************************************************************
 * Name: stmpe811_checkid
 *
 * Description:
 *   Read and verify the STMPE811 chip ID
 *
 ****************************************************************************/

static int stmpe811_checkid(FAR struct stmpe811_dev_s *priv)
{
  uint16_t devid = 0;

  /* Read device ID  */

  devid = stmpe811_getreg8(priv, STMPE811_CHIP_ID);
  devid = (uint32_t)(devid << 8);
  devid |= (uint32_t)stmpe811_getreg8(priv, STMPE811_CHIP_ID + 1);
  iinfo("devid: %04x\n", devid);

  if (devid != (uint16_t)CHIP_ID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: stmpe811_reset
 *
 * Description:
 *  Reset the STMPE811
 *
 ****************************************************************************/

static void stmpe811_reset(FAR struct stmpe811_dev_s *priv)
{
  /* Power Down the STMPE811 */

  stmpe811_putreg8(priv, STMPE811_SYS_CTRL1, SYS_CTRL1_SOFTRESET);

  /* Wait a bit */

  nxsig_usleep(20 * 1000);

  /* Then power on again.  All registers will be in their reset state. */

  stmpe811_putreg8(priv, STMPE811_SYS_CTRL1, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_instantiate
 *
 * Description:
 *   Instantiate and configure the STMPE811 device driver to use the provided
 *   I2C or SPIdevice instance.
 *
 * Input Parameters:
 *   dev     - An I2C or SPI driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   A non-zero handle is returned on success.  This handle may then be used
 *   to configure the STMPE811 driver as necessary.  A NULL handle value is
 *   returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_STMPE811_SPI
STMPE811_HANDLE stmpe811_instantiate(FAR struct spi_dev_s *dev,
                                     FAR struct stmpe811_config_s *config)
#else
STMPE811_HANDLE stmpe811_instantiate(FAR struct i2c_master_s *dev,
                                     FAR struct stmpe811_config_s *config)
#endif
{
  FAR struct stmpe811_dev_s *priv;
  uint8_t regval;
  int ret;

  /* Allocate the device state structure */

#ifdef CONFIG_STMPE811_MULTIPLE
  priv = (FAR struct stmpe811_dev_s *)kmm_zalloc(
    sizeof(struct stmpe811_dev_s));
  if (!priv)
    {
      return NULL;
    }

  /* And save the device structure in the list of STMPE811 so that we can
   * find it later.
   */

  priv->flink   = g_stmpe811list;
  g_stmpe811list = priv;
#else

  /* Use the one-and-only STMPE811 driver instance */

  priv = &g_stmpe811;
#endif

  /* Initialize the device state structure */

  nxmutex_init(&priv->lock);
  priv->config = config;

#ifdef CONFIG_STMPE811_SPI
  priv->spi = dev;
#else
  priv->i2c = dev;
#endif

  /* Read and verify the STMPE811 chip ID */

  ret = stmpe811_checkid(priv);
  if (ret < 0)
    {
#ifdef CONFIG_STMPE811_MULTIPLE
      g_stmpe811list = priv->flink;
      kmm_free(priv);
#endif
      return NULL;
    }

  /* Generate STMPE811 Software reset */

  stmpe811_reset(priv);

  /* Configure the interrupt output pin to generate interrupts on high or
   * low level.
   */

  regval  = stmpe811_getreg8(priv, STMPE811_INT_CTRL);
#ifdef CONFIG_STMPE811_ACTIVELOW
  regval &= ~INT_CTRL_INT_POLARITY; /* Pin polarity: Active low / falling edge */
#else
  regval |= INT_CTRL_INT_POLARITY;  /* Pin polarity: Active high / rising edge */
#endif
#ifdef CONFIG_STMPE811_EDGE
  regval |= INT_CTRL_INT_TYPE;      /* Edge interrupt */
#else
  regval &= ~INT_CTRL_INT_TYPE;     /* Level interrupt */
#endif
  stmpe811_putreg8(priv, STMPE811_INT_CTRL, regval);

  /* Attach the STMPE811 interrupt handler. */

  config->attach(config, stmpe811_interrupt, priv);

  /* Clear any pending interrupts */

  stmpe811_putreg8(priv, STMPE811_INT_STA, INT_ALL);
  config->clear(config);
  config->enable(config, true);

  /* Enable global interrupts */

  regval  = stmpe811_getreg8(priv, STMPE811_INT_CTRL);
  regval |= INT_CTRL_GLOBAL_INT;
  stmpe811_putreg8(priv, STMPE811_INT_CTRL, regval);

  /* Return our private data structure as an opaque handle */

  return (STMPE811_HANDLE)priv;
}

/****************************************************************************
 * Name: stmpe811_getreg8
 *
 * Description:
 *   Read from an 8-bit STMPE811 register
 *
 ****************************************************************************/

uint8_t stmpe811_getreg8(FAR struct stmpe811_dev_s *priv, uint8_t regaddr)
{
  /* 8-bit data read sequence:
   * i2c:
   *  Start - I2C_Write_Address - STMPE811_Reg_Address -
   *    Repeated_Start - I2C_Read_Address  - STMPE811_Read_Data - STOP
   * spi:
   *  [STMPE811_Reg_Address | 0x80] - Dummy Address - Read Register
   */

  uint8_t regval;
#ifdef CONFIG_STMPE811_I2C
  int ret;
  struct i2c_msg_s msg[2];

  /* Setup 8-bit STMPE811 address write message */

  msg[0].frequency = priv->config->frequency; /* I2C frequency */
  msg[0].addr      = priv->config->address;   /* 7-bit address */
  msg[0].flags     = 0;                       /* Write transaction, beginning with START */
  msg[0].buffer    = &regaddr;                /* Transfer from this address */
  msg[0].length    = 1;                       /* Send one byte following the address
                                               * (no STOP) */

  /* Set up the 8-bit STMPE811 data read message */

  msg[1].frequency = priv->config->frequency; /* I2C frequency */
  msg[1].addr      = priv->config->address;   /* 7-bit address */
  msg[1].flags     = I2C_M_READ;              /* Read transaction, beginning with Re-START */
  msg[1].buffer    = &regval;                 /* Transfer to this address */
  msg[1].length    = 1;                       /* Receive one byte following the address
                                               * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }
#else  /* CONFIG_STMPE811_SPI */
  SPI_LOCK(priv->spi, true);

  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, 8);
  SPI_HWFEATURES(priv->spi, 0);
  SPI_SETFREQUENCY(priv->spi, priv->config->frequency);

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, regaddr | 0x80);  /* Issue a read on the address */
  SPI_SEND(priv->spi, 0);               /* Next address (not used) */
  regval = SPI_SEND(priv->spi, 0);      /* Read register */
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);
  SPI_LOCK(priv->spi, false);
#endif
#ifdef CONFIG_STMPE811_REGDEBUG
  _err("%02x->%02x\n", regaddr, regval);
#endif
  return regval;
}

/****************************************************************************
 * Name: stmpe811_putreg8
 *
 * Description:
 *   Write a value to an 8-bit STMPE811 register
 *
 ****************************************************************************/

void stmpe811_putreg8(FAR struct stmpe811_dev_s *priv,
                      uint8_t regaddr, uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - STMPE811_Reg_Address -
   *   STMPE811_Write_Data - STOP
   * spi:
   *  STMPE811_Reg_Address - Dummy Address - Write Data
   */

#ifdef CONFIG_STMPE811_I2C
  int ret;
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];

#ifdef CONFIG_STMPE811_REGDEBUG
  _err("%02x<-%02x\n", regaddr, regval);
#endif

  /* Setup to the data to be transferred.  Two bytes:  The STMPE811 register
   * address followed by one byte of data.
   */

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  /* Setup 8-bit STMPE811 address write message */

  msg.frequency = priv->config->frequency; /* I2C frequency */
  msg.addr      = priv->config->address;   /* 7-bit address */
  msg.flags     = 0;                       /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                /* Transfer from this address */
  msg.length    = 2;                       /* Send two byte following the address
                                            * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
#else  /* CONFIG_STMPE811_SPI */
  SPI_LOCK(priv->spi, true);

  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, 8);
  SPI_HWFEATURES(priv->spi, 0);
  SPI_SETFREQUENCY(priv->spi, priv->config->frequency);

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, regaddr);  /* Issue a read on the address */
  SPI_SEND(priv->spi, 0);        /* Next address (not used) */
  SPI_SEND(priv->spi, regval);   /* write register */
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);
  SPI_LOCK(priv->spi, false);
#endif
}

/****************************************************************************
 * Name: stmpe811_getreg16
 *
 * Description:
 *   Read 16-bits of data from an STMPE-11 register
 *
 ****************************************************************************/

uint16_t stmpe811_getreg16(FAR struct stmpe811_dev_s *priv, uint8_t regaddr)
{
  /* 16-bit data read sequence:
   * i2c:
   *  Start - I2C_Write_Address - STMPE811_Reg_Address -
   *    Repeated_Start - I2C_Read_Address  - STMPE811_Read_Data_1 -
   *      STMPE811_Read_Data_2 - STOP
   * spi:
   *  16 bit registers are MSB.
   *  [STMPE811_Reg_Address | 0x80] - [STMPE811_Reg_Address + 1 | 0x80] -
   *    Read Register - Read Register + 1
   */

  uint8_t rxbuffer[2];
#ifdef CONFIG_STMPE811_I2C
  int ret;
  struct i2c_msg_s msg[2];

  /* Setup 8-bit STMPE811 address write message */

  msg[0].frequency = priv->config->frequency; /* I2C frequency */
  msg[0].addr      = priv->config->address;   /* 7-bit address */
  msg[0].flags     = 0;                       /* Write transaction, beginning with START */
  msg[0].buffer    = &regaddr;                /* Transfer from this address */
  msg[0].length    = 1;                       /* Send one byte following the address
                                               * (no STOP) */

  /* Set up the 8-bit STMPE811 data read message */

  msg[1].frequency = priv->config->frequency; /* I2C frequency */
  msg[1].addr      = priv->config->address;   /* 7-bit address */
  msg[1].flags     = I2C_M_READ;              /* Read transaction, beginning with Re-START */
  msg[1].buffer    = rxbuffer;                /* Transfer to this address */
  msg[1].length    = 2;                       /* Receive two bytes following the address
                                               * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }
#else  /* CONFIG_STMPE811_SPI */

  SPI_LOCK(priv->spi, true);

  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, 8);
  SPI_HWFEATURES(priv->spi, 0);
  SPI_SETFREQUENCY(priv->spi, priv->config->frequency);

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, regaddr | 0x80);         /* Issue a read on the address */
  SPI_SEND(priv->spi, regaddr + 1);            /* Next address  */
  rxbuffer[0] = SPI_SEND(priv->spi, 0);        /* Read MSB */
  rxbuffer[1] = SPI_SEND(priv->spi, 0);        /* Read LSB */
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_LOCK(priv->spi, false);
#endif
#ifdef CONFIG_STMPE811_REGDEBUG
  _err("%02x->%02x%02x\n", regaddr, rxbuffer[0], rxbuffer[1]);
#endif
  return (uint16_t)rxbuffer[0] << 8 | (uint16_t)rxbuffer[1];
}

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE811 */
