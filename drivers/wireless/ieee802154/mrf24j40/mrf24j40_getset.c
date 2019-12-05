/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_getset.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "mrf24j40.h"
#include "mrf24j40_reg.h"
#include "mrf24j40_regops.h"
#include "mrf24j40_getset.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mrf24j40_resetrfsm(FAR struct mrf24j40_radio_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_resetrfsm
 *
 * Description:
 *   Reset the RF state machine. Required at boot, after channel change,
 *   and probably after PA settings.
 *
 ****************************************************************************/

static void mrf24j40_resetrfsm(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RFCTL);
  reg |= 0x04;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);

  reg &= ~0x04;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);
  up_udelay(200);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_setrxmode
 *
 * Description:
 *   Set the RX mode (normal, promiscuous, no CRC)
 *
 ****************************************************************************/

int mrf24j40_setrxmode(FAR struct mrf24j40_radio_s *dev, int mode)
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

int mrf24j40_setchannel(FAR struct mrf24j40_radio_s *dev, uint8_t chan)
{
  if (chan < 11 || chan > 26)
    {
      wlerr("ERROR: Invalid chan: %d\n", chan);
      return -EINVAL;
    }

  /* 15. Set channel – See Section 3.4 “Channel Selection”. */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON0, (chan - 11) << 4 | 0x03);

  /* 17. RFCTL (0x36) = 0x04 – Reset RF state machine.
   * 18. RFCTL (0x36) = 0x00.
   */

  mrf24j40_resetrfsm(dev);

  dev->chan = chan;
  wlinfo("%u\n", (unsigned)chan);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setpanid
 *
 * Description:
 *   Define the PAN ID the device is operating on.
 *
 ****************************************************************************/

int mrf24j40_setpanid(FAR struct mrf24j40_radio_s *dev,
                      FAR const uint8_t *panid)
{
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDL, panid[0]);
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDH, panid[1]);

  IEEE802154_PANIDCOPY(dev->addr.panid, panid);
  wlinfo("%02X:%02X\n", panid[0], panid[1]);

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

int mrf24j40_setsaddr(FAR struct mrf24j40_radio_s *dev,
                      FAR const uint8_t *saddr)
{
  mrf24j40_setreg(dev->spi, MRF24J40_SADRL, saddr[0]);
  mrf24j40_setreg(dev->spi, MRF24J40_SADRH, saddr[1]);

  IEEE802154_SADDRCOPY(dev->addr.saddr, saddr);

  wlinfo("%02X:%02X\n", saddr[0], saddr[1]);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_seteaddr
 *
 * Description:
 *   Define the device extended address. The following addresses are special:
 *   ffffffffffffffffh : Unspecified
 *
 ****************************************************************************/

int mrf24j40_seteaddr(FAR struct mrf24j40_radio_s *dev,
                      FAR const uint8_t *eaddr)
{
  int i;

  for (i = 0; i < 8; i++)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_EADR0 + i, eaddr[i]);
      dev->addr.eaddr[i] = eaddr[i];
    }

  wlinfo("%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", eaddr[0], eaddr[1],
         eaddr[2], eaddr[3], eaddr[4], eaddr[5], eaddr[6], eaddr[7]);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setcoordsaddr
 *
 * Description:
 *   Define the coordinator short address. The following addresses are
 *   special:
 *
 *     FFFEh : Broadcast
 *     FFFFh : Unspecified
 *
 ****************************************************************************/

int mrf24j40_setcoordsaddr(FAR struct mrf24j40_radio_s *dev,
                           FAR const uint8_t *saddr)
{
  mrf24j40_setreg(dev->spi, MRF24J40_ASSOSADR0, saddr[0]);
  mrf24j40_setreg(dev->spi, MRF24J40_ASSOSADR1, saddr[1]);

  IEEE802154_SADDRCOPY(dev->addr.saddr, saddr);

  wlinfo("%02X:%02X\n", saddr[0], saddr[1]);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setcoordeaddr
 *
 * Description:
 *   Define the coordinator extended address. The following addresses are
 *   special:
 *
 *     FFFFFFFFFFFFFFFFh : Unspecified
 *
 ****************************************************************************/

int mrf24j40_setcoordeaddr(FAR struct mrf24j40_radio_s *dev,
                           FAR const uint8_t *eaddr)
{
  int i;

  for (i = 0; i < 8; i++)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_ASSOEADR0 + i, eaddr[i]);
      dev->addr.eaddr[i] = eaddr[i];
    }

  wlinfo("%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", eaddr[0], eaddr[1],
         eaddr[2], eaddr[3], eaddr[4], eaddr[5], eaddr[6], eaddr[7]);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setdevmode
 *
 * Description:
 *   Define the device behaviour: normal end device or coordinator
 *
 ****************************************************************************/

int mrf24j40_setdevmode(FAR struct mrf24j40_radio_s *dev, uint8_t mode)
{
  uint8_t reg;

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

int mrf24j40_settxpower(FAR struct mrf24j40_radio_s *dev,
                        int32_t txpwr)
{
  uint8_t reg;
  int save_txpwr = txpwr;

  if (txpwr <= -3000 && txpwr > -3630)
    {
      reg = 0xc0;
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

  wlinfo("Remaining attenuation: %d mBm\n", txpwr);

  switch (txpwr / 100)
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
 * Name: mrf24j40_setcca
 *
 * Description:
 *   Define the Clear Channel Assessement method.
 *
 ****************************************************************************/

int mrf24j40_setcca(FAR struct mrf24j40_radio_s *dev,
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
 * Name: mrf24j40_setpamode
 *
 * Description:
 *   Control the external LNA/PA on the MRF24J40MB/MC/MD/ME modules
 *   GPIO 1: PA enable
 *   GPIO 2: LNA enable
 *   GPIO 3: PA power enable (not required on MB)
 ****************************************************************************/

int mrf24j40_setpamode(FAR struct mrf24j40_radio_s *dev, int mode)
{
  if (!dev->paenabled)
    {
      return OK;
    }

  if (mode == MRF24J40_PA_AUTO)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x0f);
    }
  else if (mode == MRF24J40_PA_ED)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x0f);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x0c);
    }
  else if (mode == MRF24J40_PA_SLEEP)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x0f);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x00);
    }
  else
    {
      return -EINVAL;
    }

  mrf24j40_resetrfsm(dev);
  return OK;
}
