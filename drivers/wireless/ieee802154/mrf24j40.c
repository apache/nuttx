/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40.c
 *
 *   Copyright (C) 2015-2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/ieee802154/mrf24j40.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "mrf24j40.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#error High priority work queue required in this driver
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

/* Definitions for PA control on high power modules */

#define MRF24J40_PA_AUTO  1
#define MRF24J40_PA_ED    2
#define MRF24J40_PA_SLEEP 3

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A MRF24J40 device instance */

struct mrf24j40_dev_s
{
  struct ieee802154_dev_s           ieee;      /* The public device instance */
  FAR struct spi_dev_s              *spi;      /* Saved SPI interface instance */
  struct work_s                     irqwork;   /* Interrupt continuation work queue support */
  FAR const struct mrf24j40_lower_s *lower;    /* Low-level MCU-specific support */

  uint16_t                          panid;     /* PAN identifier, FFFF = not set */
  uint16_t                          saddr;     /* short address, FFFF = not set */
  uint8_t                           eaddr[8];  /* extended address, FFFFFFFFFFFFFFFF = not set */
  uint8_t                           channel;   /* 11 to 26 for the 2.4 GHz band */
  uint8_t                           devmode;   /* device mode: device, coord, pancoord */
  uint8_t                           paenabled; /* enable usage of PA */
  uint8_t                           rxmode;    /* Reception mode: Main, no CRC, promiscuous */
  int32_t                           txpower;   /* TX power in mBm = dBm/100 */
  struct ieee802154_cca_s           cca;       /* Clear channel assessement method */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal operations */

static void    mrf24j40_lock      (FAR struct spi_dev_s *spi);

static void    mrf24j40_setreg    (FAR struct spi_dev_s *spi, uint32_t addr, uint8_t val);
static uint8_t mrf24j40_getreg    (FAR struct spi_dev_s *spi, uint32_t addr);

static int     mrf24j40_resetrfsm (FAR struct mrf24j40_dev_s *dev);
static int     mrf24j40_pacontrol (FAR struct mrf24j40_dev_s *dev, int mode);
static int     mrf24j40_initialize(FAR struct mrf24j40_dev_s *dev);

static int     mrf24j40_setrxmode (FAR struct mrf24j40_dev_s *dev, int mode);
static int     mrf24j40_regdump   (FAR struct mrf24j40_dev_s *dev);
static void    mrf24j40_irqwork_rx(FAR struct mrf24j40_dev_s *dev);
static void    mrf24j40_irqwork_tx(FAR struct mrf24j40_dev_s *dev);
static void    mrf24j40_irqworker (FAR void *arg);
static int     mrf24j40_interrupt (int irq, FAR void *context);

/* Driver operations */

static int     mrf24j40_setchannel  (FAR struct ieee802154_dev_s *ieee, uint8_t chan);
static int     mrf24j40_getchannel  (FAR struct ieee802154_dev_s *ieee, FAR uint8_t *chan);
static int     mrf24j40_setpanid    (FAR struct ieee802154_dev_s *ieee, uint16_t panid);
static int     mrf24j40_getpanid    (FAR struct ieee802154_dev_s *ieee, FAR uint16_t *panid);
static int     mrf24j40_setsaddr    (FAR struct ieee802154_dev_s *ieee, uint16_t saddr);
static int     mrf24j40_getsaddr    (FAR struct ieee802154_dev_s *ieee, FAR uint16_t *saddr);
static int     mrf24j40_seteaddr    (FAR struct ieee802154_dev_s *ieee, FAR uint8_t *eaddr);
static int     mrf24j40_geteaddr    (FAR struct ieee802154_dev_s *ieee, FAR uint8_t *eaddr);
static int     mrf24j40_setpromisc  (FAR struct ieee802154_dev_s *ieee, bool promisc);
static int     mrf24j40_getpromisc  (FAR struct ieee802154_dev_s *ieee, FAR bool *promisc);
static int     mrf24j40_setdevmode  (FAR struct ieee802154_dev_s *ieee, uint8_t mode);
static int     mrf24j40_getdevmode  (FAR struct ieee802154_dev_s *ieee, FAR uint8_t *mode);
static int     mrf24j40_settxpower  (FAR struct ieee802154_dev_s *ieee, int32_t txpwr);
static int     mrf24j40_gettxpower  (FAR struct ieee802154_dev_s *ieee, FAR int32_t *txpwr);
static int     mrf24j40_setcca      (FAR struct ieee802154_dev_s *ieee, FAR struct ieee802154_cca_s *cca);
static int     mrf24j40_getcca      (FAR struct ieee802154_dev_s *ieee, FAR struct ieee802154_cca_s *cca);
static int     mrf24j40_ioctl       (FAR struct ieee802154_dev_s *ieee, int cmd, unsigned long arg);
static int     mrf24j40_energydetect(FAR struct ieee802154_dev_s *ieee, FAR uint8_t *energy);
static int     mrf24j40_rxenable    (FAR struct ieee802154_dev_s *ieee, bool state, FAR struct ieee802154_packet_s *packet);
static int     mrf24j40_transmit    (FAR struct ieee802154_dev_s *ieee, FAR struct ieee802154_packet_s *packet);

/* These are pointers to ALL registered MRF24J40 devices.
 * This table is used during irqs to find the context
 * Only one device is supported for now.
 * More devices can be supported in the future by lookup them up
 * using the IRQ number. See the ENC28J60 or CC3000 drivers for reference.
 */

static struct mrf24j40_dev_s g_mrf24j40_devices[1];

static const struct ieee802154_devops_s mrf24j40_devops = 
{
  mrf24j40_setchannel, mrf24j40_getchannel,
  mrf24j40_setpanid  , mrf24j40_getpanid,
  mrf24j40_setsaddr  , mrf24j40_getsaddr,
  mrf24j40_seteaddr  , mrf24j40_geteaddr,
  mrf24j40_setpromisc, mrf24j40_getpromisc,
  mrf24j40_setdevmode, mrf24j40_getdevmode,
  mrf24j40_settxpower, mrf24j40_gettxpower,
  mrf24j40_setcca    , mrf24j40_getcca,
  mrf24j40_ioctl,
  mrf24j40_energydetect,
  mrf24j40_rxenable,
  mrf24j40_transmit
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Hardware access routines */

/****************************************************************************
 * Name: mrf24j40_lock
 *
 * Description:
 *   Acquire exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void mrf24j40_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK        (spi, 1);
  SPI_SETBITS     (spi, 8);
  SPI_SETMODE     (spi, CONFIG_IEEE802154_MRF24J40_SPIMODE);
  SPI_SETFREQUENCY(spi, CONFIG_IEEE802154_MRF24J40_FREQUENCY);
}

/****************************************************************************
 * Name: mrf24j40_unlock
 *
 * Description:
 *   Release exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static inline void mrf24j40_unlock(FAR struct spi_dev_s *spi)
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

static void mrf24j40_setreg(FAR struct spi_dev_s *spi, uint32_t addr, uint8_t val)
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

  mrf24j40_lock(spi);
  SPI_SELECT(spi, SPIDEV_IEEE802154, true);
  SPI_SNDBLOCK(spi, buf, len);
  SPI_SELECT(spi, SPIDEV_IEEE802154, false);
  mrf24j40_unlock(spi);
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

  mrf24j40_lock  (spi);
  SPI_SELECT     (spi, SPIDEV_IEEE802154, true);
  SPI_EXCHANGE   (spi, buf, rx, len);
  SPI_SELECT     (spi, SPIDEV_IEEE802154, false);
  mrf24j40_unlock(spi);

  /*dbg("r[%04X]=%02X\n",addr,rx[len-1]);*/
  return rx[len-1];
}

/****************************************************************************
 * Name: mrf24j40_resetrfsm
 *
 * Description:
 *   Reset the RF state machine. Required at boot, after channel change,
 *   and probably after PA settings.
 *
 ****************************************************************************/

static int mrf24j40_resetrfsm(FAR struct mrf24j40_dev_s *dev)
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

static int mrf24j40_pacontrol(FAR struct mrf24j40_dev_s *dev, int mode)
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

static int mrf24j40_initialize(FAR struct mrf24j40_dev_s *dev)
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

static int mrf24j40_setrxmode(FAR struct mrf24j40_dev_s *dev, int mode)
{
  uint8_t reg;
  if (mode < MRF24J40_RXMODE_NORMAL || mode > MRF24J40_RXMODE_NOCRC)
    {
      return -EINVAL;
    }

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
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
  dbg("%u\n",(unsigned)mode);
  return OK;
}

/* Publicized driver routines */

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

static int mrf24j40_setchannel(FAR struct ieee802154_dev_s *ieee,
                               uint8_t chan)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;
  
  if (chan<11 || chan>26)
    {
      dbg("Invalid chan: %d\n",chan);
      return -EINVAL;
    }

  /* 15. Set channel – See Section 3.4 “Channel Selection”. */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON0, (chan - 11) << 4 | 0x03);

  /* 17. RFCTL (0x36) = 0x04 – Reset RF state machine.
   * 18. RFCTL (0x36) = 0x00.
   */

  mrf24j40_resetrfsm(dev);

  dev->channel = chan;
  //dbg("%u\n",(unsigned)chan);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_getchannel
 *
 * Description:
 *   Define the current radio channel the device is operating on.
 *
 ****************************************************************************/

static int mrf24j40_getchannel(FAR struct ieee802154_dev_s *ieee,
                               FAR uint8_t *chan)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

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

static int mrf24j40_setpanid(FAR struct ieee802154_dev_s *ieee,
                             uint16_t panid)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  mrf24j40_setreg(dev->spi, MRF24J40_PANIDH, (uint8_t)(panid>>8));
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDL, (uint8_t)(panid&0xFF));

  dev->panid = panid;
  dbg("%04X\n",(unsigned)panid);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_getpanid
 *
 * Description:
 *   Define the current PAN ID the device is operating on.
 *
 ****************************************************************************/

static int mrf24j40_getpanid(FAR struct ieee802154_dev_s *ieee,
                             FAR uint16_t *panid)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  *panid = dev->panid;

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

static int mrf24j40_setsaddr(FAR struct ieee802154_dev_s *ieee,
                             uint16_t saddr)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  mrf24j40_setreg(dev->spi, MRF24J40_SADRH, (uint8_t)(saddr>>8));
  mrf24j40_setreg(dev->spi, MRF24J40_SADRL, (uint8_t)(saddr&0xFF));

  dev->saddr = saddr;
  dbg("%04X\n",(unsigned)saddr);  
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_getsaddr
 *
 * Description:
 *   Define the current short address the device is using.
 *
 ****************************************************************************/

static int mrf24j40_getsaddr(FAR struct ieee802154_dev_s *ieee,
                             FAR uint16_t *saddr)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  *saddr = dev->saddr;

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

static int mrf24j40_seteaddr(FAR struct ieee802154_dev_s *ieee,
                             FAR uint8_t *eaddr)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  int i;

  for (i=0; i<8; i++)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_EADR0 + i, eaddr[i]);
      dev->eaddr[i] = eaddr[i];
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

static int mrf24j40_geteaddr(FAR struct ieee802154_dev_s *ieee,
                             FAR uint8_t *eaddr)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  memcpy(eaddr, dev->eaddr, 8);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setpromisc
 *
 * Description:
 *   Set the device into promiscuous mode, e.g do not filter any incoming
 *   frame.
 *
 ****************************************************************************/

static int mrf24j40_setpromisc(FAR struct ieee802154_dev_s *ieee,
                               bool promisc)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  return mrf24j40_setrxmode(dev, promisc ? MRF24J40_RXMODE_PROMISC :
                                 MRF24J40_RXMODE_NORMAL);
}

/****************************************************************************
 * Name: mrf24j40_getpromisc
 *
 * Description:
 *   Get the device receive mode.
 *
 ****************************************************************************/

static int mrf24j40_getpromisc(FAR struct ieee802154_dev_s *ieee,
                               FAR bool *promisc)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  *promisc = (dev->rxmode == MRF24J40_RXMODE_PROMISC);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setdevmode
 *
 * Description:
 *   Define the device behaviour: normal end device or coordinator
 *
 ****************************************************************************/

static int mrf24j40_setdevmode(FAR struct ieee802154_dev_s *ieee,
                               uint8_t mode)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;
  int ret = OK;
  uint8_t reg;

  /* Disable slotted mode until I decide to implement slotted mode */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_TXMCR);
  reg &= ~MRF24J40_TXMCR_SLOTTED;
  mrf24j40_setreg(dev->spi, MRF24J40_TXMCR, reg);
  mrf24j40_setreg(dev->spi, MRF24J40_ORDER, 0xFF);

  /* Define dev mode */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);

  if (mode == IEEE802154_MODE_PANCOORD)
    {
      reg |=  MRF24J40_RXMCR_PANCOORD;
      reg &= ~MRF24J40_RXMCR_COORD;
    }
  else if (mode == IEEE802154_MODE_COORD)
    {
      reg |=  MRF24J40_RXMCR_COORD;
      reg &= ~MRF24J40_RXMCR_PANCOORD;
    }
  else if (mode == IEEE802154_MODE_DEVICE)
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

static int mrf24j40_getdevmode(FAR struct ieee802154_dev_s *ieee,
                               FAR uint8_t *mode)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

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

static int mrf24j40_settxpower(FAR struct ieee802154_dev_s *ieee,
                               int32_t txpwr)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;
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

  lldbg("remaining attenuation: %d mBm\n",txpwr);

  switch(txpwr/100)
    {
      case -9:
      case -8:
      case -7:
      case -6: reg |= 0x07; break;
      case -5: reg |= 0x06; break;
      case -4: reg |= 0x05; break;
      case -3: reg |= 0x04; break;
      case -2: reg |= 0x03; break;
      case -1: reg |= 0x02; break;
      case  0: reg |= 0x00; break; /* value 0x01 is 0.5 db, not used */
      default: return -EINVAL;
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

static int mrf24j40_gettxpower(FAR struct ieee802154_dev_s *ieee,
                               FAR int32_t *txpwr)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

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

static int mrf24j40_setcca(FAR struct ieee802154_dev_s *ieee,
                           FAR struct ieee802154_cca_s *cca)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;
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

static int mrf24j40_getcca(FAR struct ieee802154_dev_s *ieee,
                           FAR struct ieee802154_cca_s *cca)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

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

static int mrf24j40_regdump(FAR struct mrf24j40_dev_s *dev)
{
  uint32_t i;
  char buf[4+16*3+2+1];
  int len=0;

  dbg("Short regs:\n");

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
          dbg("%s",buf);
        }
    }

  dbg("Long regs:\n");
  for (i=0x80000200;i<0x80000250;i++)
    {
      if ((i&15)==0)
        {
          len=sprintf(buf, "%02x: ",i&0xFF);
        }

      len += sprintf(buf+len, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i & 15) == 15)
        {
          sprintf(buf+len, "\n");
          dbg("%s",buf);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: mrf24j40_ioctl
 *
 * Description:
 *   Misc/unofficial device controls.
 *
 ****************************************************************************/

static int mrf24j40_ioctl(FAR struct ieee802154_dev_s *ieee, int cmd,
                          unsigned long arg)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;

  switch(cmd)
    {
      case 1000:
        return mrf24j40_regdump(dev);

      case 1001: dev->paenabled = (uint8_t)arg;
        dbg("PA %sabled\n",arg?"en":"dis");
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: mrf24j40_energydetect
 *
 * Description:
 *   Measure the RSSI level for the current channel.
 *
 ****************************************************************************/

static int mrf24j40_energydetect(FAR struct ieee802154_dev_s *ieee,
                                 FAR uint8_t *energy)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;
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

/* Packet exchange */

/****************************************************************************
 * Name: mrf24j40_transmit
 *
 * Description:
 *   Send a regular packet over the air.
 *
 ****************************************************************************/

static int mrf24j40_transmit(FAR struct ieee802154_dev_s *ieee, FAR struct ieee802154_packet_s *packet)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;
  uint32_t addr;
  uint8_t  reg;
  int      ret;
  int      hlen = 3; /* include frame control and seq number */
  uint8_t  fc1, fc2;

  mrf24j40_pacontrol(dev, MRF24J40_PA_AUTO);

  addr = 0x80000000;

  /* Enable tx int */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg &= ~MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);
 
  /* Analyze frame control to compute header length */

  fc1 = packet->data[0];
  fc2 = packet->data[1];

 // dbg("fc1 %02X fc2 %02X\n", fc1,fc2);

  if ((fc2 & IEEE802154_FC2_DADDR) == IEEE802154_DADDR_SHORT)
    {
      hlen += 2 + 2; /* Destination PAN + shortaddr */
    }
  else if ((fc2 & IEEE802154_FC2_DADDR) == IEEE802154_DADDR_EXT)
    {
      hlen += 2 + 8; /* Destination PAN + extaddr */
    }

  if ((fc2 & IEEE802154_FC2_SADDR) == IEEE802154_SADDR_SHORT)
    {
      if ((fc1 & IEEE802154_FC1_INTRA) != IEEE802154_INTRA)
        {
          hlen += 2; /* No PAN compression, source PAN is different from dest PAN */
        }

      hlen += 2; /* Source saddr */
    }
  else if ((fc2 & IEEE802154_FC2_SADDR) == IEEE802154_SADDR_EXT)
    {
      if ((fc1 & IEEE802154_FC1_INTRA) != IEEE802154_INTRA)
        {
          hlen += 2; /* No PAN compression, source PAN is different from dest PAN */
        }

      hlen += 8; /* Ext saddr */
    }

//  dbg("hlen %d\n",hlen);

  /* Header len, 0, TODO for security modes */

  mrf24j40_setreg(dev->spi, addr++, hlen);

  /* Frame length */

  mrf24j40_setreg(dev->spi, addr++, packet->len);

  /* Frame data */

  for (ret = 0; ret < packet->len; ret++) /* this sets the correct val for ret */
    {
      mrf24j40_setreg(dev->spi, addr++, packet->data[ret]);
    }

  /* If the frame control field contains 
   * an acknowledgment request, set the TXNACKREQ bit.
   * See IEEE 802.15.4/2003 7.2.1.1 page 112 for info.
   */

  reg = MRF24J40_TXNCON_TXNTRIG;
  if (fc1 & IEEE802154_FC1_ACKREQ)
    {
      reg |= MRF24J40_TXNCON_TXNACKREQ;
    }

  /* Trigger packet emission */

  mrf24j40_setreg(dev->spi, MRF24J40_TXNCON, reg);

  /* Suspend calling thread until transmit is complete */

  return sem_wait(&ieee->txsem);
}

/****************************************************************************
 * Name: mrf24j40_irqwork_tx
 *
 * Description:
 *   Manage completion of packet transmission.
 *
 ****************************************************************************/

static void mrf24j40_irqwork_tx(FAR struct mrf24j40_dev_s *dev)
{
  uint8_t txstat;
  uint8_t reg;

  txstat = mrf24j40_getreg(dev->spi, MRF24J40_TXSTAT);

  /* 1 means it failed, we want 1 to mean it worked.
   * tx_ok = !(tmp & ~(1 << TXNSTAT));
   * retries = tmp >> 6;
   * channel_busy = (tmp & (1 << CCAFAIL));
   */

  //dbg("TXSTAT%02X!\n", txstat);
#warning TODO report errors
  UNUSED(txstat);

  /* Disable tx int */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg |= MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* Wake up the thread that triggered the transmission */

  sem_post(&dev->ieee.txsem);
}

/****************************************************************************
 * Name: mrf24j40_enablerx
 *
 * Description:
 *  Enable reception of a packet. The interrupt will signal the rx semaphore.
 *
 ****************************************************************************/

static int mrf24j40_rxenable(FAR struct ieee802154_dev_s *ieee, bool state,
                             FAR struct ieee802154_packet_s *packet)
{
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)ieee;
  uint8_t reg;
  
  if (state)
    {
      mrf24j40_pacontrol(dev, MRF24J40_PA_AUTO);
      ieee->rxbuf = packet;

      /* Enable rx int */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg &= ~MRF24J40_INTCON_RXIE;
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);
    }
  else
    {
      ieee->rxbuf = NULL;
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

static void mrf24j40_irqwork_rx(FAR struct mrf24j40_dev_s *dev)
{
  uint32_t addr;
  uint32_t index;
  uint8_t  reg;

  /*dbg("!\n");*/

  /* Disable rx int */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg |= MRF24J40_INTCON_RXIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* Disable packet reception */

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, MRF24J40_BBREG1_RXDECINV);

  /* Read packet */

  addr = 0x80000300;
  dev->ieee.rxbuf->len = mrf24j40_getreg(dev->spi, addr++);
  /*dbg("len %3d\n", dev->ieee.rxbuf->len);*/

  for (index = 0; index < dev->ieee.rxbuf->len; index++)
    {
      dev->ieee.rxbuf->data[index] = mrf24j40_getreg(dev->spi, addr++);
    }

  dev->ieee.rxbuf->lqi  = mrf24j40_getreg(dev->spi, addr++);
  dev->ieee.rxbuf->rssi = mrf24j40_getreg(dev->spi, addr++);

  /* Reduce len by 2, we only receive frames with correct crc, no check required */

  dev->ieee.rxbuf->len -= 2;

  /* Enable reception of next packet by flushing the fifo.
   * This is an MRF24J40 errata (no. 1).
   */

  mrf24j40_setreg(dev->spi, MRF24J40_RXFLUSH, 1);

  /* Enable packet reception */

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, 0);

  sem_post(&dev->ieee.rxsem);
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
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)arg;
  uint8_t intstat;

  DEBUGASSERT(dev);
  DEBUGASSERT(dev->spi);

  /* Read and store INTSTAT - this clears the register. */

  intstat = mrf24j40_getreg(dev->spi, MRF24J40_INTSTAT);
//  dbg("INT%02X\n", intstat);

  /* Do work according to the pending interrupts */

  if ((intstat & MRF24J40_INTSTAT_RXIF))
    {
      /* A packet was received, retrieve it */

      mrf24j40_irqwork_rx(dev);
    }

  if ((intstat & MRF24J40_INTSTAT_TXNIF))
    {
      /* A packet was transmitted or failed*/

      mrf24j40_irqwork_tx(dev);
    }

  /* Re-Enable GPIO interrupts */

  dev->lower->enable(dev->lower, TRUE);
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

static int mrf24j40_interrupt(int irq, FAR void *context)
{
  /* To support multiple devices,
   * retrieve the priv structure using the irq number
   */

  register FAR struct mrf24j40_dev_s *dev = &g_mrf24j40_devices[0];

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

  dev->lower->enable(dev->lower, FALSE);
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

FAR struct ieee802154_dev_s *mrf24j40_init(FAR struct spi_dev_s *spi,
                                           FAR const struct mrf24j40_lower_s *lower)
{
  FAR struct mrf24j40_dev_s *dev;
  struct ieee802154_cca_s   cca;

#if 0
  dev = kmm_zalloc(sizeof(struct mrf24j40_dev_s));

  if (!dev)
    {
      return NULL;
    }
#else
  dev = &g_mrf24j40_devices[0];
#endif

  /* Attach irq */

  if (lower->attach(lower, mrf24j40_interrupt) != OK)
    {
#if 0
      free(dev);
#endif
      return NULL;
    }

  dev->ieee.ops = &mrf24j40_devops;
  sem_init(&dev->ieee.rxsem, 0, 0);
  sem_init(&dev->ieee.txsem, 0, 0);

  dev->lower    = lower;
  dev->spi      = spi;
  
  mrf24j40_initialize(dev);

  mrf24j40_setchannel(&dev->ieee, 11);
  mrf24j40_setpanid  (&dev->ieee, IEEE802154_PAN_DEFAULT);
  mrf24j40_setsaddr  (&dev->ieee, IEEE802154_SADDR_UNSPEC);
  mrf24j40_seteaddr  (&dev->ieee, IEEE802154_EADDR_UNSPEC);

  /* Default device params */

  cca.use_ed = 1;
  cca.use_cs = 0;
  cca.edth = 0x60; /* CCA mode ED, no carrier sense, recommenced ED threshold -69 dBm */
  mrf24j40_setcca(&dev->ieee, &cca);

  mrf24j40_setrxmode(dev, MRF24J40_RXMODE_NORMAL);

  mrf24j40_settxpower(&dev->ieee, 0); /*16. Set transmitter power .*/

  mrf24j40_pacontrol(dev, MRF24J40_PA_AUTO);

  dev->lower->enable(dev->lower, TRUE);

  return &dev->ieee;
}
