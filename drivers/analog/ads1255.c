/****************************************************************************
 * arch/drivers/analog/ads1255.c
 *
 *   Copyright (C) 2010, 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/analog/adc.h>

#if defined(CONFIG_ADC_ADS1255)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADS125X_BUFON   0x02
#define ADS125X_BUFOFF  0x00

#define ADS125X_PGA1    0x00
#define ADS125X_PGA2    0x01
#define ADS125X_PGA4    0x02
#define ADS125X_PGA8    0x03
#define ADS125X_PGA16   0x04
#define ADS125X_PGA32   0x05
#define ADS125X_PGA64   0x06

#define ADS125X_RDATA     0x01    /* Read Data */
#define ADS125X_RDATAC    0x03    /* Read Data Continuously */
#define ADS125X_SDATAC    0x0F    /* Stop Read Data Continuously */
#define ADS125X_RREG      0x10    /* Read from REG */
#define ADS125X_WREG      0x50    /* Write to REG */
#define ADS125X_SELFCAL   0xF0    /* Offset and Gain Self-Calibration */
#define ADS125X_SELFOCAL  0xF1    /* Offset Self-Calibration */
#define ADS125X_SELFGCAL  0xF2    /* Gain Self-Calibration */
#define ADS125X_SYSOCAL   0xF3    /* System Offset Calibration */
#define ADS125X_SYSGCAL   0xF4    /* System Gain Calibration */
#define ADS125X_SYNC      0xFC    /* Synchronize the A/D Conversion */
#define ADS125X_STANDBY   0xFD    /* Begin Standby Mode */
#define ADS125X_RESET     0xFE    /* Reset to Power-Up Values */
#define ADS125X_WAKEUP    0xFF    /* Completes SYNC and Exits Standby Mode */

#ifndef CONFIG_ADS1255_FREQUENCY
#  define CONFIG_ADS1255_FREQUENCY  1000000
#endif
#ifndef CONFIG_ADS1255_MUX
#  define CONFIG_ADS1255_MUX      0x01
#endif
#ifndef CONFIG_ADS1255_CHMODE
#  define CONFIG_ADS1255_CHMODE   0x00
#endif
#ifndef CONFIG_ADS1255_BUFON
#  define CONFIG_ADS1255_BUFON    1
#endif
#ifndef CONFIG_ADS1255_PGA
#  define CONFIG_ADS1255_PGA      ADS125X_PGA2
#endif
#ifndef CONFIG_ADS1255_SPS
#  define CONFIG_ADS1255_SPS      50
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads1255_dev_s
{
  FAR const struct adc_callback_s *cb;
  FAR struct spi_dev_s *spi;      /* Cached SPI device reference */
  struct work_s work;
  uint8_t channel;
  uint32_t sps;
  uint8_t pga;
  uint8_t buf;
  const uint8_t *mux;
  int irq;
  int devno;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void adc_lock(FAR struct spi_dev_s *spi);
static void adc_unlock(FAR struct spi_dev_s *spi);

/* ADC methods */

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

/* Interrupt handling */

static void adc_worker(FAR void *arg);
static int  adc_interrupt(int irq, void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = adc_bind,      /* ao_bind */
  .ao_reset    = adc_reset,     /* ao_reset */
  .ao_setup    = adc_setup,     /* ao_setup */
  .ao_shutdown = adc_shutdown,  /* ao_shutdown */
  .ao_rxint    = adc_rxint,     /* ao_rxint */
  .ao_ioctl    = adc_ioctl      /* ao_read */
};

static struct ads1255_dev_s g_adcpriv =
{
  .mux  = (const uint8_t [])
  {
    CONFIG_ADS1255_MUX, 0
  },
  .sps     = CONFIG_ADS1255_SPS,
  .channel = 0,
  .irq     = CONFIG_ADS1255_IRQ,
};

static struct adc_dev_s g_adcdev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adcpriv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t getspsreg(uint16_t sps)
{
  static const unsigned short sps_tab[] =
  {
      3,     7,     12,    20,    27,    40,    55,    80,
    300,   750,   1500,  3000,  5000, 10000, 20000, 65535,
  };

  static const unsigned char sps_reg[] =
  {
    0x03,  0x13,  0x23,  0x33,  0x43,  0x53,  0x63,  0x72,
    0x82,  0x92,  0xa1,  0xb0,  0xc0,  0xd0,  0xe0,  0xf0,
  };

  int i;

  for (i = 0; i < 16; i++)
    {
      if (sps < sps_tab[i])
        {
          return sps_reg[i];
        }
    }

  return sps_reg[15];
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void adc_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE1);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ADS1255_FREQUENCY);
}

/****************************************************************************
 * Name: adc_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void adc_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct ads1255_dev_s *priv = (FAR struct ads1255_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct ads1255_dev_s *priv = (FAR struct ads1255_dev_s *)dev->ad_priv;
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(priv != NULL && priv->spi != NULL);
  spi = priv->spi;

  adc_lock(spi);
  nxsig_usleep(1000);

  SPI_SELECT(spi, priv->devno, true);
  SPI_SEND(spi, ADS125X_WREG + 0x03);  /* WRITE SPS REG */
  SPI_SEND(spi, 0x00);                 /* count=1 */
  SPI_SEND(spi, 0x63);
  SPI_SELECT(spi, priv->devno, false);

  adc_unlock(spi);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts. Interrupts
 *   are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct ads1255_dev_s *priv = (FAR struct ads1255_dev_s *)dev->ad_priv;
  FAR struct spi_dev_s *spi;
  int ret;

  DEBUGASSERT(priv != NULL && priv->spi != NULL);
  spi = priv->spi;

  ret = irq_attach(priv->irq, adc_interrupt, NULL);
  if (ret == OK)
    {
      adc_lock(spi);

      SPI_SELECT(spi, priv->devno, true);
      SPI_SEND(spi, ADS125X_WREG);         /* WRITE REG from 0 */
      SPI_SEND(spi, 0x03);                 /* count=4+1 */
      if (priv->buf)
        {
          SPI_SEND(spi, ADS125X_BUFON);    /* REG0 STATUS BUFFER ON */
        }
      else
        {
          SPI_SEND(spi, ADS125X_BUFOFF);
        }

      SPI_SEND(spi, priv->mux[0]);
      SPI_SEND(spi, priv->pga);            /* REG2 ADCON PGA=2 */
      SPI_SEND(spi, getspsreg(priv->sps));
      nxsig_usleep(1000);
      SPI_SEND(spi, ADS125X_SELFCAL);
      SPI_SELECT(spi, priv->devno, false);

      adc_unlock(spi);
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct ads1255_dev_s *priv = (FAR struct ads1255_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct ads1255_dev_s *priv = (FAR struct ads1255_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  if (enable)
    {
      up_enable_irq(priv->irq);
    }
  else
    {
      up_disable_irq(priv->irq);
    }
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  _err("ERROR: Fix me; Not Implemented\n");
  return 0;
}

/****************************************************************************
 * Name: adc_worker
 *
 * Description:
 *   ADC interrupt work
 *
 ****************************************************************************/

static void adc_worker(FAR void *arg)
{
  FAR struct ads1255_dev_s *priv = (FAR struct ads1255_dev_s *)arg;
  FAR struct spi_dev_s *spi;
  unsigned char buf[4];
  unsigned char ch;

  DEBUGASSERT(priv != NULL && priv->spi != NULL);
  spi = priv->spi;

  /* REVISIT: Cannot perform SPI operations from an interrupt handler!
   * Need to use the high priority work queue.
   */

  adc_lock(spi);

  SPI_SELECT(spi, priv->devno, true);
  SPI_SEND(spi, ADS125X_RDATA);
  up_udelay(10);
  buf[3] = SPI_SEND(spi, 0xff);
  buf[2] = SPI_SEND(spi, 0xff);
  buf[1] = SPI_SEND(spi, 0xff);
  buf[0] = 0;

  priv->channel++;
  ch = priv->mux[priv->channel];
  if (ch == 0)
    {
      priv->channel = 0;
      ch = priv->mux[0];
    }

  SPI_SEND(spi, ADS125X_WREG + 0x01);
  SPI_SEND(spi, 0x00);
  SPI_SEND(spi, ch);
  SPI_SEND(spi, ADS125X_SYNC);
  up_udelay(2);
  SPI_SEND(spi, ADS125X_WAKEUP);
  SPI_SELECT(spi, priv->devno, false);

  adc_unlock(spi);

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      /* Perform the data received callback */

      DEBUGASSERT(priv->cb->au_receive != NULL);
      priv->cb->au_receive(&g_adcdev, priv->channel, *(int32_t *)buf);
    }

  /* Re-enable ADC interrupts */

  up_enable_irq(priv->irq);
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct ads1255_dev_s *priv =
    (FAR struct ads1255_dev_s *)g_adcdev.ad_priv;

  DEBUGASSERT(priv != NULL);

  /* Disable further ADC interrupts until the worker thread has executed. */

  up_disable_irq(priv->irq);

  /* Schedule the ADC work for the worker thread.  Whent he sample has been
   * processed, the ADC interrupt will be re-enabled.
   */

  DEBUGVERIFY(work_queue(HPWORK, &priv->work, adc_worker, priv, 0));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ads1255initialize
 *
 * Description:
 *   Initialize the selected adc port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple adc interfaces)
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *up_ads1255initialize(FAR struct spi_dev_s *spi,
                                           unsigned int devno)
{
  FAR struct ads1255_dev_s *priv =
    (FAR struct ads1255_dev_s *)g_adcdev.ad_priv;

  DEBUGASSERT(spi != NULL);

  /* Driver state data */

  priv->cb       = NULL;
  priv->spi      = spi;
  priv->devno    = devno;
  return &g_adcdev;
}
#endif
