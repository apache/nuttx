/****************************************************************************
 * drivers/motor/foc/drv8301.c
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

#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>

#include <nuttx/spi/spi.h>

#include <nuttx/motor/foc/drv8301.h>
#include <nuttx/motor/motor_ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if FOC_BOARDCFG_GAINLIST_LEN < 4
#  error FOC_BOARDCFG_GAINLIST_LEN < 4 not supported
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DRV8301 device */

struct drv8301_priv_s
{
  /* Common FOC power-stage driver - must be first */

  struct focpwr_dev_s       dev;

  FAR struct drv8301_ops_s *ops; /* Board ops */

  FAR struct spi_dev_s     *spi; /* SPI device reference */
  FAR struct drv8301_cfg_s  cfg; /* Configuration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int drv8301_fault_isr(int irq, void *context, void *arg);

static int drv8301_gain_set(FAR struct focpwr_dev_s *dev, int gain);
static int drv8301_gain_get(FAR struct focpwr_dev_s *dev, FAR int *gain);

static int drv8301_setup(FAR struct focpwr_dev_s *dev);
static int drv8301_shutdown(FAR struct focpwr_dev_s *dev);
static int drv8301_calibration(FAR struct focpwr_dev_s *dev, bool state);
static int drv8301_ioctl(FAR struct focpwr_dev_s *dev, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct focpwr_ops_s g_drv8301_ops =
{
  .setup       = drv8301_setup,
  .shutdown    = drv8301_shutdown,
  .calibration = drv8301_calibration,
  .ioctl       = drv8301_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv8301_lock
 ****************************************************************************/

static void drv8301_lock(FAR struct drv8301_priv_s *priv)
{
  SPI_LOCK(priv->spi, 1);
  SPI_SETBITS(priv->spi, 16);
  SPI_SETMODE(priv->spi, SPIDEV_MODE1);
  SPI_SETFREQUENCY(priv->spi, priv->cfg.freq);
}

/****************************************************************************
 * Name: drv8301_unlock
 ****************************************************************************/

static void drv8301_unlock(FAR struct drv8301_priv_s *priv)
{
  SPI_LOCK(priv->spi, 0);
}

/****************************************************************************
 * Name: drv8301_read
 ****************************************************************************/

static void drv8301_read(FAR struct drv8301_priv_s *priv, uint8_t addr,
                         uint16_t *data)
{
  uint16_t regval = 0;

  drv8301_lock(priv);
  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->dev.devno), true);

  /* Read command */

  regval |= (1 << 15);
  regval |= ((addr & 0x0f) << 11);

  /* Send command */

  SPI_SEND(priv->spi, regval);

  /* Toggle CS pin, otherwise read doesn't work */

  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->dev.devno), false);
  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->dev.devno), true);

  /* Read output */

  regval = 0;
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Retrun data */

  *data = (regval & 0x7ff);

  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->dev.devno), false);
  drv8301_unlock(priv);
}

/****************************************************************************
 * Name: drv8301_write
 ****************************************************************************/

static void drv8301_write(FAR struct drv8301_priv_s *priv, uint8_t addr,
                          uint16_t data)
{
  uint16_t regval = 0;

  drv8301_lock(priv);
  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->dev.devno), true);

  /* Write command */

  regval |= (0 << 15);
  regval |= ((addr & 0x0f) << 11);
  regval |= (0x7ff & data);

  /* Send data */

  SPI_SEND(priv->spi, regval);

  SPI_SELECT(priv->spi, SPIDEV_MOTOR(priv->dev.devno), false);
  drv8301_unlock(priv);
}

/****************************************************************************
 * Name: drv8301_fault_isr
 ****************************************************************************/

static int drv8301_fault_isr(int irq, FAR void *context, void *arg)
{
  FAR struct drv8301_priv_s *priv = NULL;

  priv = (struct drv8301_priv_s *)arg;
  DEBUGASSERT(priv != NULL);

  priv->ops->fault_handle(&priv->dev);

  return OK;
}

/****************************************************************************
 * Name: drv8301_setup
 ****************************************************************************/

static int drv8301_setup(FAR struct focpwr_dev_s *dev)
{
  FAR struct drv8301_priv_s *priv    = (FAR struct drv8301_priv_s *)dev;
  uint16_t                   status1 = 0;
  uint16_t                   status2 = 0;
  uint16_t                   ctrl1   = 0;
  uint16_t                   ctrl2   = 0;
  int                        ret     = OK;

  /* Reset chip */

  priv->ops->gate_enable(dev, true);
  up_udelay(30);
  priv->ops->gate_enable(dev, false);
  up_udelay(30);
  priv->ops->gate_enable(dev, true);
  up_mdelay(10);

  /* Attach fault handler */

  priv->ops->fault_attach(dev, drv8301_fault_isr, priv);

  /* Get status registers */

  drv8301_read(priv, DRV8301_REG_STAT1, &status1);
  drv8301_read(priv, DRV8301_REG_STAT2, &status2);

  /* Configure CTRL1 */

  ctrl1  = DRV8301_CTRL1_GCURR(priv->cfg.gate_curr);
  ctrl1 |= DRV8301_CTRL1_OCADJ(priv->cfg.oc_adj);
  ctrl1 |= (priv->cfg.pwm_mode ? DRV8301_CTRL1_PWMMODE : 0);
  drv8301_write(priv, DRV8301_REG_CTRL1, ctrl1);

  /* Configure CTRL2 */

  ctrl2 = DRV8301_CTRL2_GAIN(priv->cfg.gain);
  drv8301_write(priv, DRV8301_REG_CTRL2, ctrl2);

  return ret;
}

/****************************************************************************
 * Name: drv8301_shutdown
 ****************************************************************************/

static int drv8301_shutdown(FAR struct focpwr_dev_s *dev)
{
  FAR struct drv8301_priv_s *priv  = (FAR struct drv8301_priv_s *)dev;

  /* Disable chip */

  priv->ops->gate_enable(dev, false);

  /* Disable nFAULT interrupt */

  priv->ops->fault_attach(dev, NULL, NULL);

  return OK;
}

/****************************************************************************
 * Name: drv8301_gain_get
 ****************************************************************************/

static int drv8301_gain_get(FAR struct focpwr_dev_s *dev, FAR int *gain)
{
  FAR struct drv8301_priv_s *priv  = (FAR struct drv8301_priv_s *)dev;
  uint16_t                   ctrl2 = 0;
  int                        ret   = OK;

  drv8301_read(priv, DRV8301_REG_CTRL2, &ctrl2);
  ctrl2 &= DRV8301_CTRL2_GAIN_MASK;

  if (ctrl2 == DRV8301_CTRL2_GAIN_10)
    {
      *gain = 10;
    }
  else if (ctrl2 == DRV8301_CTRL2_GAIN_20)
    {
      *gain = 20;
    }
  else if (ctrl2 == DRV8301_CTRL2_GAIN_40)
    {
      *gain = 40;
    }
  else if (ctrl2 == DRV8301_CTRL2_GAIN_80)
    {
      *gain = 80;
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: drv8301_gain_set
 ****************************************************************************/

static int drv8301_gain_set(FAR struct focpwr_dev_s *dev, int gain)
{
  FAR struct drv8301_priv_s *priv  = (FAR struct drv8301_priv_s *)dev;
  uint16_t                   ctrl2 = 0;
  int                        ret   = OK;

  drv8301_read(priv, DRV8301_REG_CTRL2, &ctrl2);

  ctrl2 &= ~DRV8301_CTRL2_GAIN_MASK;

  if (gain == 10)
    {
      ctrl2 |= DRV8301_CTRL2_GAIN_10;
    }
  else if (gain == 20)
    {
      ctrl2 |= DRV8301_CTRL2_GAIN_20;
    }
  else if (gain == 40)
    {
      ctrl2 |= DRV8301_CTRL2_GAIN_40;
    }
  else if (gain == 80)
    {
      ctrl2 |= DRV8301_CTRL2_GAIN_80;
    }
  else
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Write CTRL2 */

  drv8301_write(priv, DRV8301_REG_CTRL2, ctrl2);

errout:
  return ret;
}

/****************************************************************************
 * Name: drv8301_calibration
 ****************************************************************************/

static int drv8301_calibration(FAR struct focpwr_dev_s *dev, bool state)
{
  FAR struct drv8301_priv_s *priv   = (FAR struct drv8301_priv_s *)dev;
  uint16_t                   regval = 0;

  drv8301_read(priv, DRV8301_REG_CTRL2, &regval);

  if (state == true)
    {
      regval |= DRV8301_CTRL2_DCCALCH1;
      regval |= DRV8301_CTRL2_DCCALCH2;
    }
  else
    {
      regval &= ~DRV8301_CTRL2_DCCALCH1;
      regval &= ~DRV8301_CTRL2_DCCALCH2;
    }

  drv8301_write(priv, DRV8301_REG_CTRL2, regval);

  return OK;
}

/****************************************************************************
 * Name: drv8301_ioctl
 ****************************************************************************/

static int drv8301_ioctl(FAR struct focpwr_dev_s *dev, int cmd,
                         unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case MTRIOC_SET_BOARDCFG:
        {
          struct foc_set_boardcfg_s *cfg =
            (struct foc_set_boardcfg_s *)arg;

          ret = drv8301_gain_set(dev, cfg->gain);
          break;
        }

      case MTRIOC_GET_BOARDCFG:
        {
          struct foc_get_boardcfg_s *cfg =
            (struct foc_get_boardcfg_s *)arg;

          ret = drv8301_gain_get(dev, &cfg->gain);

          cfg->gain_list[0] = 10;
          cfg->gain_list[1] = 20;
          cfg->gain_list[2] = 40;
          cfg->gain_list[3] = 80;

          break;
        }

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv8301_register
 ****************************************************************************/

int drv8301_register(FAR const char *path,
                     FAR struct foc_dev_s *dev,
                     FAR struct drv8301_board_s *board)
{
  FAR struct drv8301_priv_s *priv = NULL;
  int                        ret  = OK;

  /* Allocate driver */

  priv = kmm_zalloc(sizeof(struct drv8301_priv_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Register FOC device */

  ret = foc_register(path, dev);
  if (ret < 0)
    {
      return ret;
    }

  /* Store board data */

  priv->ops = board->ops;
  priv->spi = board->spi;

  /* Store configuration */

  memcpy(&priv->cfg, board->cfg, sizeof(struct drv8301_cfg_s));

  /* Initialize FOC power stage */

  return focpwr_initialize(&priv->dev, board->devno, dev, &g_drv8301_ops);
}
