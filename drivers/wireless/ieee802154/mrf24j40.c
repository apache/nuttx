/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40.c
 *
 *   Copyright (C) 2015-2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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
#include <assert.h>
#include <debug.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/mrf24j40.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mrf24j40.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue required in this driver
#endif

#ifndef CONFIG_IEEE802154_MRF24J40_SPIMODE
#  define CONFIG_IEEE802154_MRF24J40_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_IEEE802154_MRF24J40_FREQUENCY
#  define CONFIG_IEEE802154_MRF24J40_FREQUENCY 8000000
#endif

#ifndef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE required for this driver
#endif

/* Definitions for the device structure */

#define MRF24J40_RXMODE_NORMAL  0
#define MRF24J40_RXMODE_PROMISC 1
#define MRF24J40_RXMODE_NOCRC   2

#define MRF24J40_MODE_DEVICE    0
#define MRF24J40_MODE_COORD     1
#define MRF24J40_MODE_PANCOORD  2

/* Definitions for PA control on high power modules */

#define MRF24J40_PA_AUTO   1
#define MRF24J40_PA_ED     2
#define MRF24J40_PA_SLEEP  3

#define MRF24J40_GTS_SLOTS 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A MRF24J40 device instance */

struct mrf24j40_radio_s
{
  struct ieee802154_radio_s radio;  /* The public device instance */
  FAR struct ieee802154_radiocb_s *radiocb; /* Registered callbacks */

  /* MAC Attributes */

  bool rxonidle : 1;

  /* Low-level MCU-specific support */

  FAR const struct mrf24j40_lower_s *lower;
  FAR struct spi_dev_s *spi; /* Saved SPI interface instance */

  struct work_s   irqwork;   /* For deferring interrupt work to work queue */
  struct work_s   pollwork;  /* For deferring poll work to the work queue */
  sem_t           exclsem;   /* Exclusive access to this struct */

  struct ieee802154_addr_s addr;

  uint8_t         channel;   /* 11 to 26 for the 2.4 GHz band */
  uint8_t         devmode;   /* device mode: device, coord, pancoord */
  uint8_t         paenabled; /* enable usage of PA */
  uint8_t         rxmode;    /* Reception mode: Main, no CRC, promiscuous */
  int32_t         txpower;   /* TX power in mBm = dBm/100 */
  struct ieee802154_cca_s cca;  /* Clear channel assessement method */

  /* Buffer Allocations */

  struct ieee802154_txdesc_s *csma_desc;
  FAR struct iob_s *csma_frame;
  bool csma_busy;

  struct ieee802154_txdesc_s *gts_desc[MRF24J40_GTS_SLOTS];
  FAR struct iob_s *gts_frame[MRF24J40_GTS_SLOTS];
  bool gts_busy[MRF24J40_GTS_SLOTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal operations */

static void mrf24j40_spi_lock(FAR struct spi_dev_s *spi);

static void mrf24j40_setreg(FAR struct spi_dev_s *spi, uint32_t addr,
              uint8_t val);
static uint8_t mrf24j40_getreg(FAR struct spi_dev_s *spi, uint32_t addr);

static int  mrf24j40_resetrfsm(FAR struct mrf24j40_radio_s *dev);
static int  mrf24j40_pacontrol(FAR struct mrf24j40_radio_s *dev, int mode);
static int  mrf24j40_initialize(FAR struct mrf24j40_radio_s *dev);

static int  mrf24j40_setrxmode(FAR struct mrf24j40_radio_s *dev, int mode);
static int  mrf24j40_regdump(FAR struct mrf24j40_radio_s *dev);

static void mrf24j40_irqwork_rx(FAR struct mrf24j40_radio_s *dev);
static void mrf24j40_irqwork_txnorm(FAR struct mrf24j40_radio_s *dev);
static void mrf24j40_irqwork_txgts(FAR struct mrf24j40_radio_s *dev,
              uint8_t gts_num);

static void mrf24j40_irqworker(FAR void *arg);
static int  mrf24j40_interrupt(int irq, FAR void *context, FAR void *arg);

static void mrf24j40_dopoll_csma(FAR void *arg);
static void mrf24j40_dopoll_gts(FAR void *arg);

static int  mrf24j40_csma_setup(FAR struct mrf24j40_radio_s *dev,
              FAR struct iob_s *frame);
static int  mrf24j40_gts_setup(FAR struct mrf24j40_radio_s *dev, uint8_t gts,
              FAR struct iob_s *frame);
static int  mrf24j40_setup_fifo(FAR struct mrf24j40_radio_s *dev,
              FAR struct iob_s *frame, uint32_t fifo_addr);

static int  mrf24j40_setchannel(FAR struct mrf24j40_radio_s *dev,
              uint8_t chan);
static int  mrf24j40_getchannel(FAR struct mrf24j40_radio_s *dev,
              FAR uint8_t *chan);
static int  mrf24j40_setpanid(FAR struct mrf24j40_radio_s *dev,
              uint16_t panid);
static int  mrf24j40_getpanid(FAR struct mrf24j40_radio_s *dev,
              FAR uint16_t *panid);
static int  mrf24j40_setsaddr(FAR struct mrf24j40_radio_s *dev,
              uint16_t saddr);
static int  mrf24j40_getsaddr(FAR struct mrf24j40_radio_s *dev,
              FAR uint16_t *saddr);
static int  mrf24j40_seteaddr(FAR struct mrf24j40_radio_s *dev,
              FAR const uint8_t *eaddr);
static int  mrf24j40_geteaddr(FAR struct mrf24j40_radio_s *dev,
              FAR uint8_t *eaddr);
static int  mrf24j40_setdevmode(FAR struct mrf24j40_radio_s *dev,
              uint8_t mode);
static int  mrf24j40_getdevmode(FAR struct mrf24j40_radio_s *dev,
              FAR uint8_t *mode);
static int  mrf24j40_settxpower(FAR struct mrf24j40_radio_s *dev,
              int32_t txpwr);
static int  mrf24j40_gettxpower(FAR struct mrf24j40_radio_s *dev,
              FAR int32_t *txpwr);
static int  mrf24j40_setcca(FAR struct mrf24j40_radio_s *dev,
              FAR struct ieee802154_cca_s *cca);
static int  mrf24j40_getcca(FAR struct mrf24j40_radio_s *dev,
              FAR struct ieee802154_cca_s *cca);
static int  mrf24j40_energydetect(FAR struct mrf24j40_radio_s *dev,
              FAR uint8_t *energy);
static int  mrf24j40_rxenable(FAR struct mrf24j40_radio_s *dev, bool enable);

/* Driver operations */

static int  mrf24j40_bind(FAR struct ieee802154_radio_s *radio,
              FAR struct ieee802154_radiocb_s *radiocb);
static int  mrf24j40_txnotify_csma(FAR struct ieee802154_radio_s *radio);
static int  mrf24j40_txnotify_gts(FAR struct ieee802154_radio_s *radio);
static int  mrf24j40_get_attr(FAR struct ieee802154_radio_s *radio,
              enum ieee802154_pib_attr_e pib_attr,
              FAR union ieee802154_attr_u *attrval);
static int mrf24j40_set_attr(FAR struct ieee802154_radio_s *radio,
              enum ieee802154_pib_attr_e pib_attr,
              FAR const union ieee802154_attr_u *attrval);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These are pointers to ALL registered MRF24J40 devices.
 * This table is used during irqs to find the context
 * Only one device is supported for now.
 * More devices can be supported in the future by lookup them up
 * using the IRQ number. See the ENC28J60 or CC3000 drivers for reference.
 */

static const struct ieee802154_radioops_s mrf24j40_devops =
{
  mrf24j40_bind,
  mrf24j40_txnotify_csma,
  mrf24j40_txnotify_gts,
  mrf24j40_get_attr,
  mrf24j40_set_attr
};

/****************************************************************************
 * Radio Interface Functions
 ****************************************************************************/

static int mrf24j40_bind(FAR struct ieee802154_radio_s *radio,
                         FAR struct ieee802154_radiocb_s *radiocb)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;

  DEBUGASSERT(dev != NULL);
  dev->radiocb = radiocb;
  return OK;
}

/****************************************************************************
 * Function: mrf24j40_txnotify_csma
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mrf24j40_txnotify_csma(FAR struct ieee802154_radio_s *radio)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&dev->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(HPWORK, &dev->pollwork, mrf24j40_dopoll_csma, dev, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: mrf24j40_txnotify_gts
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mrf24j40_txnotify_gts(FAR struct ieee802154_radio_s *radio)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&dev->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(HPWORK, &dev->pollwork, mrf24j40_dopoll_gts, dev, 0);
    }

  return OK;
}

static int  mrf24j40_get_attr(FAR struct ieee802154_radio_s *radio,
                              enum ieee802154_pib_attr_e pib_attr,
                              FAR union ieee802154_attr_u *attrval)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  int ret;

  switch (pib_attr)
    {
      case IEEE802154_PIB_MAC_EXTENDED_ADDR:
        {
          memcpy(&attrval->mac.eaddr[0], &dev->addr.eaddr[0], 8);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;
      default:
        ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
    }
  return ret;
}

static int mrf24j40_set_attr(FAR struct ieee802154_radio_s *radio,
                             enum ieee802154_pib_attr_e pib_attr,
                             FAR const union ieee802154_attr_u *attrval)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  int ret;

  switch (pib_attr)
    {
      case IEEE802154_PIB_MAC_EXTENDED_ADDR:
        {
          mrf24j40_seteaddr(dev, &attrval->mac.eaddr[0]);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;
      case IEEE802154_PIB_MAC_PROMISCUOUS_MODE:
        {
          if (attrval->mac.promisc_mode)
            {
              mrf24j40_setrxmode(dev, MRF24J40_RXMODE_PROMISC); 
            }
          else
            {
              mrf24j40_setrxmode(dev, MRF24J40_RXMODE_NORMAL); 
            }

          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;
      case IEEE802154_PIB_MAC_RX_ON_WHEN_IDLE:
        {
          dev->rxonidle = attrval->mac.rxonidle;
          mrf24j40_rxenable(dev, dev->rxonidle);
        }
        break;
      default:
        ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
    }
  return ret;
}

/****************************************************************************
 * Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Function: mrf24j40_dopoll_csma
 *
 * Description:
 *   This function is called in order to preform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (mrf24j40_txdone_csma),
 *   2. When new TX data is available (mrf24j40_txnotify_csma), and
 *   3. After a TX timeout to restart the sending process
 *      (mrf24j40_txtimeout_csma).
 *
 * Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mrf24j40_dopoll_csma(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  int len = 0;

  /* Get exclusive access to the driver */

  while (sem_wait(&dev->exclsem) != 0) { }

  /* If this a CSMA transaction and we have room in the CSMA fifo */

  if (!dev->csma_busy)
    {
      /* need to somehow allow for a handle to be passed */

      len = dev->radiocb->poll_csma(dev->radiocb, &dev->csma_desc,
                                    &dev->csma_frame);
      if (len > 0)
        {
          /* Now the txdesc is in use */

          dev->csma_busy = 1;

          /* Setup the transaction on the device in the CSMA FIFO */

          mrf24j40_csma_setup(dev, dev->csma_frame);
        }
    }

  sem_post(&dev->exclsem);
}

/****************************************************************************
 * Function: mrf24j40_dopoll_gts
 *
 * Description:
 *   This function is called in order to preform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (mrf24j40_txdone_gts),
 *   2. When new TX data is available (mrf24j40_txnotify_gts), and
 *   3. After a TX timeout to restart the sending process
 *      (mrf24j40_txtimeout_gts).
 *
 * Parameters:
 *   arg  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mrf24j40_dopoll_gts(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  int gts = 0;
  int len = 0;

  /* Get exclusive access to the driver */

  while (sem_wait(&dev->exclsem) != 0) { }

  for (gts = 0; gts < MRF24J40_GTS_SLOTS; gts++)
    {
      if (!dev->gts_busy[gts])
        {
          len = dev->radiocb->poll_gts(dev->radiocb, &dev->gts_desc[gts],
                                       &dev->gts_frame[0]);
          if (len > 0)
            {
              /* Now the txdesc is in use */

              dev->gts_busy[gts]= 1;

              /* Setup the transaction on the device in the open GTS FIFO */

              mrf24j40_gts_setup(dev, gts, dev->gts_frame[0]);
            }
        }
    }

  sem_post(&dev->exclsem);
}

/****************************************************************************
 * Name: mrf24j40_spi_lock
 *
 * Description:
 *   Acquire exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void mrf24j40_spi_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 1);
  SPI_SETBITS(spi, 8);
  SPI_SETMODE(spi, CONFIG_IEEE802154_MRF24J40_SPIMODE);
  SPI_SETFREQUENCY(spi, CONFIG_IEEE802154_MRF24J40_FREQUENCY);
}

/****************************************************************************
 * Name: mrf24j40_spi_unlock
 *
 * Description:
 *   Release exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static inline void mrf24j40_spi_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi,0);
}

/****************************************************************************
 * Name: mrf24j40_setreg
 *
 * Description:
 *   Define the value of an MRF24J40 device register
 *
 ****************************************************************************/

static void mrf24j40_setreg(FAR struct spi_dev_s *spi, uint32_t addr,
                           uint8_t val)
{
  uint8_t buf[3];
  int     len;

  if (!(addr&0x80000000))
    {
      addr  &= 0x3F; /* 6-bit address */
      addr <<= 1;
      addr  |= 0x01; /* writing */
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      addr  &= 0x3FF; /* 10-bit address */
      addr <<= 5;
      addr  |= 0x8010; /* writing long */
      buf[0] = (addr >>   8);
      buf[1] = (addr & 0xFF);
      len    = 2;
    }

  buf[len++] = val;

  mrf24j40_spi_lock(spi);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), true);
  SPI_SNDBLOCK(spi, buf, len);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), false);
  mrf24j40_spi_unlock(spi);
}

/****************************************************************************
 * Name: mrf24j40_getreg
 *
 * Description:
 *   Return the value of an MRF24J40 device register
 *
 ****************************************************************************/

static uint8_t mrf24j40_getreg(FAR struct spi_dev_s *spi, uint32_t addr)
{
  uint8_t buf[3];
  uint8_t rx[3];
  int     len;

  if (!(addr&0x80000000))
    {
      /* 6-bit address */

      addr  &= 0x3F;
      addr <<= 1;
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      /* 10-bit address */

      addr  &= 0x3FF;
      addr <<= 5;
      addr  |= 0x8000;
      buf[0] = (addr >>   8);
      buf[1] = (addr & 0xFF);
      len    = 2;
    }

  buf[len++] = 0xFF; /* dummy */

  mrf24j40_spi_lock  (spi);
  SPI_SELECT     (spi, SPIDEV_IEEE802154(0), true);
  SPI_EXCHANGE   (spi, buf, rx, len);
  SPI_SELECT     (spi, SPIDEV_IEEE802154(0), false);
  mrf24j40_spi_unlock(spi);

  /* wlinfo("r[%04X]=%02X\n", addr, rx[len - 1]); */
  return rx[len - 1];
}

/****************************************************************************
 * Name: mrf24j40_resetrfsm
 *
 * Description:
 *   Reset the RF state machine. Required at boot, after channel change,
 *   and probably after PA settings.
 *
 ****************************************************************************/

static int mrf24j40_resetrfsm(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RFCTL);
  reg |= 0x04;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);

  reg &= ~0x04;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);
  up_udelay(200);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_pacontrol
 *
 * Description:
 *   Control the external LNA/PA on the MRF24J40MB/MC/MD/ME modules
 *   GPIO 1: PA enable
 *   GPIO 2: LNA enable
 *   GPIO 3: PA power enable (not required on MB)
 ****************************************************************************/

static int mrf24j40_pacontrol(FAR struct mrf24j40_radio_s *dev, int mode)
{
  if (!dev->paenabled)
    {
      return OK;
    }

  if (mode == MRF24J40_PA_AUTO)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x0F);
    }
  else if (mode == MRF24J40_PA_ED)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x0F);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x0C);
    }
  else if (mode == MRF24J40_PA_SLEEP)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x0F);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x00);
    }
  else
    {
      return -EINVAL;
    }

  mrf24j40_resetrfsm(dev);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_initialize
 *
 * Description:
 *   Reset the device and put in in order of operation
 *
 ****************************************************************************/

static int mrf24j40_initialize(FAR struct mrf24j40_radio_s *dev)
{
  /* Software reset */

  mrf24j40_setreg(dev->spi, MRF24J40_SOFTRST  , 0x07); /* 00000111 Reset */
  while(mrf24j40_getreg(dev->spi, MRF24J40_SOFTRST) & 0x07);

  /* Apply recommended settings */

  mrf24j40_setreg(dev->spi, MRF24J40_PACON2 , 0x98); /* 10011000 Enable FIFO (default), TXONTS=6 (recommended), TXONT<8:7>=0 */
  mrf24j40_setreg(dev->spi, MRF24J40_TXSTBL , 0x95); /* 10010101 set the SIFS period. RFSTBL=9, MSIFS=5, aMinSIFSPeriod=14 (min 12) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXPEND , 0x7C); /* 01111100 set the LIFS period, MLIFS=1Fh=31 aMinLIFSPeriod=40 (min 40) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXTIME , 0x30); /* 00110000 set the turnaround time, TURNTIME=3 aTurnAroundTime=12 */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON1 , 0x02); /* 00000010 VCO optimization, recommended value */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON2 , 0x80); /* 10000000 Enable PLL */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON6 , 0x90); /* 10010000 TX filter enable, fast 20M recovery, No bat monitor*/
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON7 , 0x80); /* 10000000 Sleep clock on internal 100 kHz */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON8 , 0x10); /* 00010000 VCO control bit, as recommended */
  mrf24j40_setreg(dev->spi, MRF24J40_SLPCON1, 0x01); /* 00000001 no CLKOUT, default divisor */
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6 , 0x40); /* 01000000 Append RSSI to rx packets */

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setrxmode
 *
 * Description:
 *   Set the RX mode (normal, promiscuous, no CRC)
 *
 ****************************************************************************/

static int mrf24j40_setrxmode(FAR struct mrf24j40_radio_s *dev, int mode)
{
  uint8_t reg;

  if (mode < MRF24J40_RXMODE_NORMAL || mode > MRF24J40_RXMODE_NOCRC)
    {
      return -EINVAL;
    }

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
  reg &= ~0x03;
  reg |= mode;

  /* Set mode options */

  if (mode != MRF24J40_RXMODE_NORMAL)
    {
      /* Promisc and error modes: Disable auto ACK */

      reg |= MRF24J40_RXMCR_NOACKRSP;
    }
  else
    {
      /* Normal mode : enable auto-ACK */

      reg &= ~MRF24J40_RXMCR_NOACKRSP;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);

  dev->rxmode = mode;
  wlinfo("%u\n", (unsigned)mode);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setchannel
 *
 * Description:
 *   Define the current radio channel the device is operating on.
 *   In the 2.4 GHz, there are 16 channels, each 2 MHz wide, 5 MHz spacing:
 *   Chan   MHz       Chan   MHz       Chan   MHz       Chan   MHz
 *     11  2405         15  2425         19  2445         23  2465
 *     12  2410         16  2430         20  2450         24  2470
 *     13  2415         17  2435         21  2455         25  2475
 *     14  2420         18  2440         22  2460         26  2480
 *
 ****************************************************************************/

static int mrf24j40_setchannel(FAR struct mrf24j40_radio_s *dev, uint8_t chan)
{
  if (chan < 11 || chan > 26)
    {
      wlerr("ERROR: Invalid chan: %d\n",chan);
      return -EINVAL;
    }

  /* 15. Set channel – See Section 3.4 “Channel Selection”. */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON0, (chan - 11) << 4 | 0x03);

  /* 17. RFCTL (0x36) = 0x04 – Reset RF state machine.
   * 18. RFCTL (0x36) = 0x00.
   */

  mrf24j40_resetrfsm(dev);

  dev->channel = chan;
  //wlinfo("%u\n", (unsigned)chan);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_getchannel
 *
 * Description:
 *   Get the channel the device is operating on.
 *
 ****************************************************************************/

static int mrf24j40_getchannel(FAR struct mrf24j40_radio_s *dev,
                               FAR uint8_t *chan)
{
  *chan = dev->channel;

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setpanid
 *
 * Description:
 *   Define the PAN ID the device is operating on.
 *
 ****************************************************************************/

static int mrf24j40_setpanid(FAR struct mrf24j40_radio_s *dev,
                             uint16_t panid)
{
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDH, (uint8_t)(panid>>8));
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDL, (uint8_t)(panid&0xFF));

  dev->addr.panid = panid;
  wlinfo("%04X\n", (unsigned)panid);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_getpanid
 *
 * Description:
 *   Define the current PAN ID the device is operating on.
 *
 ****************************************************************************/

static int mrf24j40_getpanid(FAR struct mrf24j40_radio_s *dev,
                             FAR uint16_t *panid)
{
  *panid = dev->addr.panid;

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setsaddr
 *
 * Description:
 *   Define the device short address. The following addresses are special:
 *   FFFEh : Broadcast
 *   FFFFh : Unspecified
 *
 ****************************************************************************/

static int mrf24j40_setsaddr(FAR struct mrf24j40_radio_s *dev,
                             uint16_t saddr)
{
  mrf24j40_setreg(dev->spi, MRF24J40_SADRH, (uint8_t)(saddr>>8));
  mrf24j40_setreg(dev->spi, MRF24J40_SADRL, (uint8_t)(saddr&0xFF));

  dev->addr.saddr = saddr;
  wlinfo("%04X\n", (unsigned)saddr);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_getsaddr
 *
 * Description:
 *   Define the current short address the device is using.
 *
 ****************************************************************************/

static int mrf24j40_getsaddr(FAR struct mrf24j40_radio_s *dev,
                             FAR uint16_t *saddr)
{
  *saddr = dev->addr.saddr;

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_seteaddr
 *
 * Description:
 *   Define the device extended address. The following addresses are special:
 *   FFFFFFFFFFFFFFFFh : Unspecified
 *
 ****************************************************************************/

static int mrf24j40_seteaddr(FAR struct mrf24j40_radio_s *dev,
                             FAR const uint8_t *eaddr)
{
  int i;

  for (i = 0; i < 8; i++)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_EADR0 + i, eaddr[i]);
      dev->addr.eaddr[i] = eaddr[i];
    }

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_geteaddr
 *
 * Description:
 *   Define the current extended address the device is using.
 *
 ****************************************************************************/

static int mrf24j40_geteaddr(FAR struct mrf24j40_radio_s *dev,
                             FAR uint8_t *eaddr)
{
  memcpy(eaddr, dev->addr.eaddr, 8);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setdevmode
 *
 * Description:
 *   Define the device behaviour: normal end device or coordinator
 *
 ****************************************************************************/

static int mrf24j40_setdevmode(FAR struct mrf24j40_radio_s *dev,
                               uint8_t mode)
{
  int ret = OK;
  uint8_t reg;

  /* Disable slotted mode until I decide to implement slotted mode */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_TXMCR);
  reg &= ~MRF24J40_TXMCR_SLOTTED;
  mrf24j40_setreg(dev->spi, MRF24J40_TXMCR, reg);
  mrf24j40_setreg(dev->spi, MRF24J40_ORDER, 0xFF);

  /* Define dev mode */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);

  if (mode == MRF24J40_MODE_PANCOORD)
    {
      reg |=  MRF24J40_RXMCR_PANCOORD;
      reg &= ~MRF24J40_RXMCR_COORD;
    }
  else if (mode == MRF24J40_MODE_COORD)
    {
      reg |=  MRF24J40_RXMCR_COORD;
      reg &= ~MRF24J40_RXMCR_PANCOORD;
    }
  else if (mode == MRF24J40_MODE_DEVICE)
    {
      reg &= ~MRF24J40_RXMCR_PANCOORD;
      reg &= ~MRF24J40_RXMCR_COORD;
    }
  else
    {
      return -EINVAL;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);
  dev->devmode = mode;
  return ret;
}

/****************************************************************************
 * Name: mrf24j40_setdevmode
 *
 * Description:
 *   Return the current device mode
 *
 ****************************************************************************/

static int mrf24j40_getdevmode(FAR struct mrf24j40_radio_s *dev,
                               FAR uint8_t *mode)
{
  *mode = dev->devmode;
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_settxpower
 *
 * Description:
 *   Define the transmit power. Value is passed in mBm, it is rounded to
 *   the nearest value. Some MRF modules have a power amplifier, this routine
 *   does not care about this. We only change the CHIP output power.
 *
 ****************************************************************************/

static int mrf24j40_settxpower(FAR struct mrf24j40_radio_s *dev,
                               int32_t txpwr)
{
  uint8_t reg;
  int save_txpwr = txpwr;

  if (txpwr <= -3000 && txpwr > -3630)
    {
      reg = 0xC0;
      txpwr += 3000;
    }
  else if (txpwr <= -2000)
    {
      reg = 0x80;
      txpwr += 2000;
    }
  else if (txpwr <= -1000)
    {
      reg = 0x40;
      txpwr += 1000;
    }
  else if (txpwr <= 0)
    {
      reg = 0x00;
    }
  else
    {
      return -EINVAL;
    }

  wlinfo("remaining attenuation: %d mBm\n",txpwr);

  switch(txpwr/100)
    {
      case -9:
      case -8:
      case -7:
      case -6:
        reg |= 0x07;
        break;

      case -5:
        reg |= 0x06;
        break;

      case -4:
        reg |= 0x05;
        break;

      case -3:
        reg |= 0x04;
        break;

      case -2:
        reg |= 0x03;
        break;

      case -1:
        reg |= 0x02;
        break;

      case  0:
        reg |= 0x00;  /* value 0x01 is 0.5 db, not used */
        break;

      default:
        return -EINVAL;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON3, reg);
  dev->txpower = save_txpwr;
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_gettxpower
 *
 * Description:
 *   Return the actual transmit power, in mBm.
 *
 ****************************************************************************/

static int mrf24j40_gettxpower(FAR struct mrf24j40_radio_s *dev,
                               FAR int32_t *txpwr)
{
  *txpwr = dev->txpower;
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setcca
 *
 * Description:
 *   Define the Clear Channel Assessement method.
 *
 ****************************************************************************/

static int mrf24j40_setcca(FAR struct mrf24j40_radio_s *dev,
                           FAR struct ieee802154_cca_s *cca)
{
  uint8_t mode;

  if (!cca->use_ed && !cca->use_cs)
    {
      return -EINVAL;
    }

  if (cca->use_cs && cca->csth > 0x0f)
    {
      return -EINVAL;
    }

  mode  = mrf24j40_getreg(dev->spi, MRF24J40_BBREG2);
  mode &= 0x03;

  if (cca->use_ed)
    {
      mode |= MRF24J40_BBREG2_CCAMODE_ED;
      mrf24j40_setreg(dev->spi, MRF24J40_CCAEDTH, cca->edth);
    }

  if (cca->use_cs)
    {
      mode |= MRF24J40_BBREG2_CCAMODE_CS;
      mode |= cca->csth << 2;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG2, mode);

  memcpy(&dev->cca, cca, sizeof(struct ieee802154_cca_s));
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_getcca
 *
 * Description:
 *   Return the Clear Channel Assessement method.
 *
 ****************************************************************************/

static int mrf24j40_getcca(FAR struct mrf24j40_radio_s *dev,
                           FAR struct ieee802154_cca_s *cca)
{
  memcpy(cca, &dev->cca, sizeof(struct ieee802154_cca_s));
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_regdump
 *
 * Description:
 *   Display the value of all registers.
 *
 ****************************************************************************/

static int mrf24j40_regdump(FAR struct mrf24j40_radio_s *dev)
{
  uint32_t i;
  char buf[4+16*3+2+1];
  int len = 0;

  wlinfo("Short regs:\n");

  for (i = 0; i < 0x40; i++)
    {
      if ((i & 15) == 0)
        {
          len=sprintf(buf, "%02x: ",i&0xFF);
        }

      len += sprintf(buf+len, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i & 15) == 15)
        {
          sprintf(buf+len, "\n");
          wlinfo("%s", buf);
        }
    }

  wlinfo("Long regs:\n");
  for (i = 0x80000200; i < 0x80000250; i++)
    {
      if ((i & 15) == 0)
        {
          len=sprintf(buf, "%02x: ",i&0xFF);
        }

      len += sprintf(buf+len, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i & 15) == 15)
        {
          sprintf(buf+len, "\n");
          wlinfo("%s", buf);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: mrf24j40_energydetect
 *
 * Description:
 *   Measure the RSSI level for the current channel.
 *
 ****************************************************************************/

static int mrf24j40_energydetect(FAR struct mrf24j40_radio_s *dev,
                                 FAR uint8_t *energy)
{
  uint8_t reg;

  /* Manually enable the LNA*/

  mrf24j40_pacontrol(dev, MRF24J40_PA_ED);

  /* Set RSSI average duration to 8 symbols */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXBCON1);
  reg |= 0x30;
  mrf24j40_setreg(dev->spi, MRF24J40_TXBCON1, reg);

  /* 1. Set RSSIMODE1 0x3E<7> – Initiate RSSI calculation. */

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, 0x80);

  /* 2. Wait until RSSIRDY 0x3E<0> is set to ‘1’ – RSSI calculation is
   *    complete.
   */

  while(!(mrf24j40_getreg(dev->spi, MRF24J40_BBREG6) & 0x01));

  /* 3. Read RSSI 0x210<7:0> – The RSSI register contains the averaged RSSI
   *    received power level for 8 symbol periods.
   */

  *energy = mrf24j40_getreg(dev->spi, MRF24J40_RSSI);

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, 0x40);

  /* Back to automatic control */

  mrf24j40_pacontrol(dev, MRF24J40_PA_AUTO);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_csma_setup
 *
 * Description:
 *   Setup a CSMA transaction in the normal TX FIFO
 *
 ****************************************************************************/

static int mrf24j40_csma_setup(FAR struct mrf24j40_radio_s *dev,
                               FAR struct iob_s *frame)
{
  uint8_t reg;
  int     ret;

  /* Enable tx int */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg &= ~MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* Setup the FIFO */

  ret = mrf24j40_setup_fifo(dev, frame, MRF24J40_TXNORM_FIFO);

  /* If the frame control field contains
   * an acknowledgment request, set the TXNACKREQ bit.
   * See IEEE 802.15.4/2003 7.2.1.1 page 112 for info.
   */

  reg = MRF24J40_TXNCON_TXNTRIG;
  if (frame->io_data[0] & IEEE802154_FRAMECTRL_ACKREQ)
    {
      reg |= MRF24J40_TXNCON_TXNACKREQ;
    }

  /* Trigger packet emission */

  mrf24j40_setreg(dev->spi, MRF24J40_TXNCON, reg);
  return ret;
}

/****************************************************************************
 * Name: mrf24j40_gts_setup
 *
 * Description:
 *   Setup a GTS transaction in one of the GTS FIFOs
 *
 ****************************************************************************/

static int mrf24j40_gts_setup(FAR struct mrf24j40_radio_s *dev, uint8_t fifo,
                              FAR struct iob_s *frame)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: mrf24j40_setup_fifo
 *
 * Description:
 *
 ****************************************************************************/

static int mrf24j40_setup_fifo(FAR struct mrf24j40_radio_s *dev,
                               FAR struct iob_s *frame, uint32_t fifo_addr)
{
  int      ret;
  int      hlen = 3; /* Include frame control and seq number */
  uint16_t frame_ctrl;

  /* Analyze frame control to compute header length */

  frame_ctrl = frame->io_data[0];
  frame_ctrl |= (frame->io_data[1] << 8);

  if ((frame_ctrl & IEEE802154_FRAMECTRL_DADDR)== IEEE802154_ADDRMODE_SHORT)
    {
      hlen += 2 + 2; /* Destination PAN + shortaddr */
    }
  else if ((frame_ctrl & IEEE802154_FRAMECTRL_DADDR) == IEEE802154_ADDRMODE_EXTENDED)
    {
      hlen += 2 + 8; /* Destination PAN + extaddr */
    }

  if (!(frame_ctrl & IEEE802154_FRAMECTRL_PANIDCOMP))
    {
      hlen += 2; /* No PAN compression, source PAN is different from dest PAN */
    }

  if ((frame_ctrl & IEEE802154_FRAMECTRL_SADDR)== IEEE802154_ADDRMODE_SHORT)
    {
      hlen += 2; /* Source saddr */
    }
  else if ((frame_ctrl & IEEE802154_FRAMECTRL_SADDR) == IEEE802154_ADDRMODE_EXTENDED)
    {
      hlen += 8; /* Ext saddr */
    }

  /* Header len, 0, TODO for security modes */

  mrf24j40_setreg(dev->spi, fifo_addr++, hlen);

  /* Frame length */

  mrf24j40_setreg(dev->spi, fifo_addr++, frame->io_len);

  /* Frame data */

  for (ret = 0; ret < frame->io_len; ret++) /* this sets the correct val for ret */
    {
      mrf24j40_setreg(dev->spi, fifo_addr++, frame->io_data[ret]);
    }

  return ret;
}

/****************************************************************************
 * Name: mrf24j40_irqwork_txnorm
 *
 * Description:
 *   Manage completion of packet transmission.
 *
 ****************************************************************************/

static void mrf24j40_irqwork_txnorm(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t txstat;

  /* Disable tx int */

  txstat  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  txstat |= MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, txstat);

  /* Get the status from the device and copy the status into the tx desc.
   * The status for the normal FIFO is represented with bit TXNSTAT where
   * 0=success, 1= failure.
   */

  txstat = mrf24j40_getreg(dev->spi, MRF24J40_TXSTAT);
  dev->csma_desc->conf->status = txstat & MRF24J40_TXSTAT_TXNSTAT;

  /* Inform the next layer of the transmission success/failure */

  dev->radiocb->txdone(dev->radiocb, dev->csma_desc);

  /* We are now done with the transaction */

  dev->csma_busy = 0;

  /* Free the IOB */

  iob_free(dev->csma_frame);

  mrf24j40_dopoll_csma(dev);
}

/****************************************************************************
 * Name: mrf24j40_irqwork_gts
 *
 * Description:
 *   Manage completion of packet transmission.
 *
 ****************************************************************************/

static void mrf24j40_irqwork_txgts(FAR struct mrf24j40_radio_s *dev,
                                   uint8_t gts)
{
  uint8_t txstat;

  /* Disable tx int */

  txstat  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  txstat |= MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, txstat);

  /* Get the status from the device and copy the status into the tx desc.
   * The status for the normal FIFO is represented with bit TXNSTAT where
   * 0=success, 1= failure.
   */

  txstat = mrf24j40_getreg(dev->spi, MRF24J40_TXSTAT);

  if (gts == 0)
    {
      dev->csma_desc->conf->status = txstat & MRF24J40_TXSTAT_TXG1STAT;
    }
  else if (gts == 1)
    {
      dev->csma_desc->conf->status = txstat & MRF24J40_TXSTAT_TXG2STAT;
    }

  /* Inform the next layer of the transmission success/failure */

  dev->radiocb->txdone(dev->radiocb, dev->gts_desc[gts]);

  /* We are now done with the transaction */

  dev->gts_busy[gts]= 0;

  /* Free the IOB */

  iob_free(dev->gts_frame[gts]);

  mrf24j40_dopoll_gts(dev);
}

/****************************************************************************
 * Name: mrf24j40_rxenable
 *
 * Description:
 *  Enable/Disable receiver.
 *
 ****************************************************************************/

static int mrf24j40_rxenable(FAR struct mrf24j40_radio_s *dev, bool enable)
{
  uint8_t reg;

  if (enable)
    {
      /* Enable rx int */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg &= ~MRF24J40_INTCON_RXIE;
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);
    }
  else
    {
      /* Disable rx int */

      reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg |= MRF24J40_INTCON_RXIE;
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);
    }

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_irqwork_rx
 *
 * Description:
 *   Manage packet reception.
 *
 ****************************************************************************/

static void mrf24j40_irqwork_rx(FAR struct mrf24j40_radio_s *dev)
{
  FAR struct ieee802154_data_ind_s *ind;
  uint32_t addr;
  uint32_t index;
  uint8_t  reg;

  /* Disable rx int */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg |= MRF24J40_INTCON_RXIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* Disable packet reception. See pg. 109 of datasheet */

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, MRF24J40_BBREG1_RXDECINV);

  /* Allocate a data_ind to put the frame in */

  ind = ieee802154_ind_allocate();
  if (ind == NULL)
    {
      wlerr("ERROR: Unable to allocate data_ind. Discarding frame");
      goto done;
    }

  /* Read packet */

  addr = MRF24J40_RXBUF_BASE;

  ind->frame->io_len= mrf24j40_getreg(dev->spi, addr++);

  /* TODO: This needs to be changed.  It is inefficient to do the SPI read byte
   * by byte */

  for (index = 0; index < ind->frame->io_len; index++)
    {
      ind->frame->io_data[index] = mrf24j40_getreg(dev->spi, addr++);
    }

  ind->lqi  = mrf24j40_getreg(dev->spi, addr++);
  ind->rssi = mrf24j40_getreg(dev->spi, addr++);

  /* Reduce len by 2, we only receive frames with correct crc, no check
   * required.
   */

  ind->frame->io_len -= 2;

  /* Callback the receiver in the next highest layer */

  dev->radiocb->rxframe(dev->radiocb, ind);

done:
  /* Enable reception of next packet by flushing the fifo.
   * This is an MRF24J40 errata (no. 1).
   */

  mrf24j40_setreg(dev->spi, MRF24J40_RXFLUSH, 1);

  /* Enable packet reception */

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, 0);

  /* Only enable RX interrupt if we are to be listening when IDLE */

  if (dev->rxonidle)
    {
      reg = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg &= ~MRF24J40_INTCON_RXIE;
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);
    }
}

/****************************************************************************
 * Name: mrf24j40_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Parameters:
 *   arg     - The reference to the driver structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mrf24j40_irqworker(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  uint8_t intstat;

  DEBUGASSERT(dev);
  DEBUGASSERT(dev->spi);

  /* Read and store INTSTAT - this clears the register. */

  intstat = mrf24j40_getreg(dev->spi, MRF24J40_INTSTAT);
  //wlinfo("INT%02X\n", intstat);

  /* Do work according to the pending interrupts */

  if ((intstat & MRF24J40_INTSTAT_RXIF))
    {
      /* A packet was received, retrieve it */

      mrf24j40_irqwork_rx(dev);
    }

  if ((intstat & MRF24J40_INTSTAT_TXNIF))
    {
      /* A packet was transmitted or failed*/

      mrf24j40_irqwork_txnorm(dev);
    }

  if ((intstat & MRF24J40_INTSTAT_TXG1IF))
    {
      /* A packet was transmitted or failed*/

      mrf24j40_irqwork_txgts(dev, 0);
    }

  if ((intstat & MRF24J40_INTSTAT_TXG1IF))
    {
      /* A packet was transmitted or failed*/

      mrf24j40_irqwork_txgts(dev, 1);
    }

  /* Re-enable GPIO interrupts */

  dev->lower->enable(dev->lower, true);
}

/****************************************************************************
 * Name: mrf24j40_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mrf24j40_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;

  DEBUGASSERT(dev != NULL);

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(work_available(&dev->irqwork));

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in enc_irqworker() when the work is completed.
   */

  dev->lower->enable(dev->lower, false);
  return work_queue(HPWORK, &dev->irqwork, mrf24j40_irqworker, (FAR void *)dev, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_init
 *
 * Description:
 *   Return an mrf24j40 device for use by other drivers.
 *
 ****************************************************************************/

FAR struct ieee802154_radio_s *mrf24j40_init(FAR struct spi_dev_s *spi,
                                      FAR const struct mrf24j40_lower_s *lower)
{
  FAR struct mrf24j40_radio_s *dev;
  struct ieee802154_cca_s   cca;

  dev = kmm_zalloc(sizeof(struct mrf24j40_radio_s));
  if (dev == NULL)
    {
      return NULL;
    }

  /* Attach irq */

  if (lower->attach(lower, mrf24j40_interrupt, dev) != OK)
    {
#if 0
      free(dev);
#endif
      return NULL;
    }

  /* Allow exclusive access to the privmac struct */

  sem_init(&dev->exclsem, 0, 1);

  dev->radio.ops = &mrf24j40_devops;

  dev->lower    = lower;
  dev->spi      = spi;

  mrf24j40_initialize(dev);

  mrf24j40_setchannel(dev, 11);
  mrf24j40_setpanid  (dev, 0xFFFF);
  mrf24j40_setsaddr  (dev, 0xFFFF);
  mrf24j40_seteaddr  (dev, (uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF");

  /* Default device params */

  cca.use_ed = 1;
  cca.use_cs = 0;
  cca.edth = 0x60; /* CCA mode ED, no carrier sense, recommenced ED threshold -69 dBm */
  mrf24j40_setcca(dev, &cca);

  mrf24j40_setrxmode(dev, MRF24J40_RXMODE_NORMAL);

  mrf24j40_settxpower(dev, 0); /*16. Set transmitter power .*/

  mrf24j40_pacontrol(dev, MRF24J40_PA_AUTO);

  dev->lower->enable(dev->lower, true);
  return &dev->radio;
}
