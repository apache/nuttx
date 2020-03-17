/****************************************************************************
 * drivers/wireless/ieee802154/at86rf23x/at86rf23x.c
 *
 *   Copyright (C) 2016 Matt Poppe. All rights reserved.
 *   Author: Matt Poppe <matt@poppe.me>
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
#include <nuttx/compiler.h>
#include <assert.h>
#include <debug.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/ieee802154/at86rf23x.h>

#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "at86rf23x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue required in this driver
#endif

#ifndef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE required for this driver
#endif

#ifndef CONFIG_IEEE802154_AT86RF23X_SPIMODE
#  define CONFIG_IEEE802154_AT86RF23X_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_IEEE802154_AT86RF23X_FREQUENCY
#  define CONFIG_IEEE802154_AT86RF23X_FREQUENCY 1000000
#endif

/* Definitions for the device structure */

#define AT86RF23X_RXMODE_NORMAL  0
#define AT86RF23X_RXMODE_PROMISC 1
#define AT86RF23X_RXMODE_NOCRC   2
#define AT86RF23X_RXMODE_AUTOACK 3

/* Definitions for PA control on high power modules */

#define AT86RF23X_PA_AUTO  1
#define AT86RF23X_PA_ED    2
#define AT86RF23X_PA_SLEEP 3

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* AT86RF23x device instance
 *
 * Make sure that struct ieee802154_radio_s remains first.  If not it will
 * break the code
 */

struct at86rf23x_dev_s
{
  struct ieee802154_radio_s           ieee;      /* Public device instance */
  FAR struct spi_dev_s               *spi;       /* Saved SPI interface instance */
  struct work_s                       irqwork;   /* Interrupt continuation work queue support */
  FAR const struct at86rf23x_lower_s *lower;     /* Low-level MCU-specific support */
  uint8_t                             panid[2];  /* PAN identifier, FFFF = not set */
  uint16_t                            saddr;     /* Short address, FFFF = not set */
  uint8_t                             eaddr[8];  /* Extended address, FFFFFFFFFFFFFFFF = not set */
  uint8_t                             channel;   /* 11 to 26 for the 2.4 GHz band */
  uint8_t                             devmode;   /* Device mode: device, coord, pancoord */
  uint8_t                             paenabled; /* Enable usage of PA */
  uint8_t                             rxmode;    /* Reception mode: Main, no CRC, promiscuous */
  int32_t                             txpower;   /* TX power in mBm = dBm/100 */
  struct ieee802154_cca_s             cca;       /* Clear channel assessement method */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal operations */

static void at86rf23x_lock(FAR struct spi_dev_s *spi);
static void at86rf23x_unlock(FAR struct spi_dev_s *spi);
static void at86rf23x_setreg(FAR struct spi_dev_s *spi, uint32_t addr,
              uint8_t val);
static uint8_t at86rf23x_getreg(FAR struct spi_dev_s *spi, uint32_t addr);
static int  at86rf23x_writeframe(FAR struct spi_dev_s *spi,
              FAR uint8_t *frame, uint8_t len);
static uint8_t at86rf23x_readframe(FAR struct spi_dev_s *spi,
              FAR uint8_t *frame_rx);
static int  at86rf23x_set_trxstate(FAR struct at86rf23x_dev_s *dev,
              uint8_t state, uint8_t force);
static  uint8_t at86rf23x_get_trxstate(FAR struct at86rf23x_dev_s *dev);
static int  at86rf23x_resetrf(FAR struct at86rf23x_dev_s *dev);
static int  at86rf23x_initialize(FAR struct at86rf23x_dev_s *dev);

static int  at86rf23x_regdump(FAR struct at86rf23x_dev_s *dev);
static void at86rf23x_irqwork_rx(FAR struct at86rf23x_dev_s *dev);
static void at86rf23x_irqwork_tx(FAR struct at86rf23x_dev_s *dev);
static void at86rf23x_irqworker(FAR void *arg);
static int  at86rf23x_interrupt(int irq, FAR void *context, FAR void *arg);

static int  at86rf23x_setchannel(FAR struct ieee802154_radio_s *ieee,
              uint8_t chan);
static int  at86rf23x_getchannel(FAR struct ieee802154_radio_s *ieee,
              FAR uint8_t *chan);
static int  at86rf23x_setpanid(FAR struct ieee802154_radio_s *ieee,
              uint16_t panid);
static int  at86rf23x_getpanid(FAR struct ieee802154_radio_s *ieee,
              FAR uint16_t *panid);
static int  at86rf23x_setsaddr(FAR struct ieee802154_radio_s *ieee,
              uint16_t saddr);
static int  at86rf23x_getsaddr(FAR struct ieee802154_radio_s *ieee,
              FAR uint16_t *saddr);
static int  at86rf23x_seteaddr(FAR struct ieee802154_radio_s *ieee,
              FAR uint8_t *eaddr);
static int  at86rf23x_geteaddr(FAR struct ieee802154_radio_s *ieee,
              FAR uint8_t *eaddr);
static int  at86rf23x_setpromisc(FAR struct ieee802154_radio_s *ieee,
              bool promisc);
static int  at86rf23x_getpromisc(FAR struct ieee802154_radio_s *ieee,
              FAR bool *promisc);
static int  at86rf23x_setdevmode(FAR struct ieee802154_radio_s *ieee,
              uint8_t mode);
static int  at86rf23x_getdevmode(FAR struct ieee802154_radio_s *ieee,
              FAR uint8_t *mode);
static int  at86rf23x_settxpower(FAR struct ieee802154_radio_s *ieee,
              int32_t txpwr);
static int  at86rf23x_gettxpower(FAR struct ieee802154_radio_s *ieee,
              FAR int32_t *txpwr);
static int  at86rf23x_setcca(FAR struct ieee802154_radio_s *ieee,
              FAR struct ieee802154_cca_s *cca);
static int  at86rf23x_getcca(FAR struct ieee802154_radio_s *ieee,
              FAR struct ieee802154_cca_s *cca);
static int  at86rf23x_energydetect(FAR struct ieee802154_radio_s *ieee,
              FAR uint8_t *energy);

/* Driver operations */

static int  at86rf23x_rxenable(FAR struct ieee802154_radio_s *ieee,
              bool state, FAR struct ieee802154_packet_s *packet);
static int  at86rf23x_transmit(FAR struct ieee802154_radio_s *ieee,
              FAR struct ieee802154_packet_s *packet);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These are pointers to ALL registered at86rf23x devices.
 * This table is used during interrupt handling to find the context.
 * Only one device is supported for now.
 */

static struct at86rf23x_dev_s g_at86rf23x_devices[1];

static const struct ieee802154_radioops_s at86rf23x_devops =
{
  .ioctl        = at86rf23x_ioctl,
  .rxenable     = at86rf23x_rxenable,
  .transmit     = at86rf23x_transmit
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at86rf23x_lock
 *
 * Description:
 *   Acquire exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void at86rf23x_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 1);
  SPI_SETBITS(spi, 8);
  SPI_SETMODE(spi, CONFIG_IEEE802154_AT86RF23X_SPIMODE);
  SPI_SETFREQUENCY(spi, CONFIG_IEEE802154_AT86RF23X_FREQUENCY);
}

/****************************************************************************
 * Name: at86rf23x_unlock
 *
 * Description:
 *   Release exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void at86rf23x_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 0);
}

/****************************************************************************
 * Name: at86rf23x_setreg
 *
 * Description:
 *   Define the value of an at86rf23x device register
 *
 ****************************************************************************/

static void at86rf23x_setreg(FAR struct spi_dev_s *spi, uint32_t addr,
                             uint8_t val)
{
  uint8_t reg[2];

  reg[0] = addr;
  reg[0] |= RF23X_SPI_REG_WRITE;

  reg[1] = val;
  at86rf23x_lock(spi);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), true);
  SPI_SNDBLOCK(spi, reg, 2);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), false);
  at86rf23x_unlock(spi);

  wlinfo("0x%02X->r[0x%02X]\n", val, addr);
}

/****************************************************************************
 * Name: at86rf23x_getreg
 *
 * Description:
 *   Return the value of an at86rf23x device register
 *
 ****************************************************************************/

static uint8_t at86rf23x_getreg(FAR struct spi_dev_s *spi, uint32_t addr)
{
  uint8_t reg[2];
  uint8_t val[2];

  /* Add Read mask to desired register */

  reg[0] = addr | RF23X_SPI_REG_READ;

  at86rf23x_lock (spi);

  SPI_SELECT(spi, SPIDEV_IEEE802154(0), true);
  SPI_EXCHANGE(spi, reg, val, 2);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), false);

  at86rf23x_unlock(spi);

  wlinfo("r[0x%02X]=0x%02X\n", addr, val[1]);

  return val[1];
}

/****************************************************************************
 * Name: at86rf23x_setregbits
 *
 * Description:
 *   Read the current value of the register.  Change only the portion
 *   of the register we need to change and write the value back to the
 *   register.
 *
 ****************************************************************************/

static void at86rf23x_setregbits(FAR struct spi_dev_s *spi, uint8_t addr,
                                 uint8_t pos, uint8_t mask, uint8_t val)
{
  uint8_t reg;

  reg = at86rf23x_getreg(spi, addr);
  reg = (reg & ~(mask << pos)) | (val << pos);
  at86rf23x_setreg(spi, addr, reg);
}

/****************************************************************************
 * Name: at86rf23x_getregbits
 *
 * Description:
 *   Return the value of an section of the at86rf23x device register.
 *
 ****************************************************************************/

static uint8_t at86rf23x_getregbits(FAR struct spi_dev_s *spi, uint8_t addr,
                                    uint8_t pos, uint8_t mask)
{
  uint8_t val;

  val = at86rf23x_getreg(spi, addr);
  val = (val >> pos) & mask;

  return val;
}

/****************************************************************************
 * Name: at86rf23x_writebuffer
 *
 * Description:
 *   Write frame to the transmit buffer of the radio. This does not
 *   initiate the transfer, just write to the buffer.
 *
 ****************************************************************************/

static int at86rf23x_writeframe(FAR struct spi_dev_s *spi, FAR uint8_t *frame,
                                uint8_t len)
{
  uint8_t reg = RF23X_SPI_FRAME_WRITE;

#if 0
  report_packet(frame_wr, len);
#endif

  at86rf23x_lock(spi);

  SPI_SELECT(spi, SPIDEV_IEEE802154(0), true);

  SPI_SNDBLOCK(spi, &reg, 1);
  SPI_SNDBLOCK(spi, &frame, len);

  SPI_SELECT(spi, SPIDEV_IEEE802154(0), false);

  at86rf23x_unlock(spi);

  return len;
}

/****************************************************************************
 * Name: at86rf23x_readframe
 *
 * Description:
 *   Read the buffer memory of the radio.  This is used when the radio
 *   indicates a frame has been received.
 *
 ****************************************************************************/

static uint8_t at86rf23x_readframe(FAR struct spi_dev_s *spi,
                                   FAR uint8_t *frame_rx)
{
  uint8_t len;
  uint8_t reg;

  reg = RF23X_SPI_FRAME_READ;

  at86rf23x_lock(spi);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), true);

  SPI_SNDBLOCK(spi, &reg, 1);
  SPI_RECVBLOCK(spi, &len, 1);
  SPI_RECVBLOCK(spi, frame_rx, len + 3);

  SPI_SELECT(spi, SPIDEV_IEEE802154(0), false);
  at86rf23x_unlock(spi);

  return len;
}

/****************************************************************************
 * Name: at86rf23x_get_trxstate
 *
 * Description:
 *   Return the current status of the TRX state machine.
 *
 ****************************************************************************/

static uint8_t at86rf23x_get_trxstate(FAR struct at86rf23x_dev_s *dev)
{
  return at86rf23x_getregbits(dev->spi, RF23X_TRXSTATUS_STATUS);
}

/****************************************************************************
 * Name: at86rf23x_set_trxstate
 *
 * Description:
 *   Set the TRX state machine to the desired state.  If the state machine
 *   cannot move to the desired state an ERROR is returned.  If the
 *   transition is successful an OK is returned.  This is a long running
 *   function due to waiting for the transition delay between states.  This
 *   can be as long as 510us.
 *
 ****************************************************************************/

static int at86rf23x_set_trxstate(FAR struct at86rf23x_dev_s *dev,
                                  uint8_t state, uint8_t force)
{
  /* Get the current state of the transceiver */

  uint8_t status = at86rf23x_get_trxstate(dev);

  int ret = OK;

  /* TODO I don't have every state included verify this will work with SLEEP */

  if ((status != TRX_STATUS_TRXOFF) &&
     (status != TRX_STATUS_RXON) &&
     (status != TRX_STATUS_PLLON) &&
     (status != TRX_STATUS_RXAACKON) &&
     (status != TRX_STATUS_TXARETON) &&
     (status != TRX_STATUS_PON))
    {
      wlerr("ERROR: Invalid State\n");
      return ERROR;
    }

  int8_t init_status = status;

  /* Start the state change */

  switch (state)
    {
    case TRX_CMD_TRXOFF:
      if (status == TRX_STATUS_TRXOFF)
        {
          break;
        }

      /* verify in a state that will transfer to TRX_OFF if not check if
       * force is required.
       */

      if ((status == TRX_STATUS_RXON) ||
          (status == TRX_STATUS_PLLON) ||
          (status == TRX_STATUS_RXAACKON) ||
          (status == TRX_STATUS_TXARETON))
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE, TRX_CMD_TRXOFF);
          up_udelay(RF23X_TIME_TRANSITION_PLL_ACTIVE);
        }
      else if (status == TRX_STATUS_PON)
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE, TRX_CMD_TRXOFF);
          up_udelay(RF23X_TIME_P_ON_TO_TRXOFF);
        }
      else if (force)
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE,
                               TRX_CMD_FORCETRXOFF);
          up_udelay(RF23X_TIME_FORCE_TRXOFF);
        }

      status = at86rf23x_getregbits(dev->spi, RF23X_TRXSTATUS_STATUS);
      if (status != TRX_STATUS_TRXOFF)
        {
          ret = ERROR;
        }

      break;

    case TRX_CMD_RX_ON:
      if (status == TRX_STATUS_RXON)
        {
          break;
        }

      if (status == TRX_STATUS_TRXOFF)
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE, TRX_CMD_RX_ON);
          up_udelay(RF23X_TIME_TRXOFF_TO_PLL);
        }
      else if ((status == TRX_STATUS_RXAACKON) ||
               (status == TRX_STATUS_RXON) ||
               (status == TRX_STATUS_PLLON) ||
               (status == TRX_STATUS_TXARETON))
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE, TRX_CMD_RX_ON);
          up_udelay(RF23X_TIME_TRANSITION_PLL_ACTIVE);
        }

      status = at86rf23x_getregbits(dev->spi, RF23X_TRXSTATUS_STATUS);
      if (status != TRX_STATUS_RXON)
        {
          ret = ERROR;
        }

      break;

    /* Transition to PLL_ON */

    case TRX_CMD_PLL_ON:
      if (status == TRX_STATUS_PLLON)
        {
          break;
        }

      if (status == TRX_STATUS_TRXOFF)
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE, TRX_CMD_PLL_ON);
          up_udelay(RF23X_TIME_TRXOFF_TO_PLL);
        }
      else
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE, TRX_CMD_PLL_ON);
          up_udelay(RF23X_TIME_TRANSITION_PLL_ACTIVE);
        }

      status = at86rf23x_getregbits(dev->spi, RF23X_TRXSTATUS_STATUS);
      if (status != TRX_STATUS_PLLON)
        {
          ret = ERROR;
        }

      break;

    case TRX_CMD_RX_AACK_ON:
      if (status == TRX_STATUS_RXAACKON)
        {
          break;
       }

      if (status == TRX_STATUS_TRXOFF || status == TRX_STATUS_TXARETON)
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE, TRX_CMD_RX_ON);

          (status == TRX_STATUS_TRXOFF) ?
            up_udelay(RF23X_TIME_TRXOFF_TO_PLL) :
            up_udelay(RF23X_TIME_TRANSITION_PLL_ACTIVE);
        }

      status = at86rf23x_get_trxstate(dev);
      if ((status == TRX_STATUS_RXON) || (status == TRX_STATUS_PLLON))
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE,
                               TRX_CMD_RX_AACK_ON);
          up_udelay(RF23X_TIME_TRANSITION_PLL_ACTIVE);
        }

      status = at86rf23x_getregbits(dev->spi, RF23X_TRXSTATUS_STATUS);
      if (status != TRX_STATUS_RXAACKON)
        {
          ret = ERROR;
        }

      break;

    case TRX_STATUS_TXARETON:
      if (status == TRX_STATUS_TXARETON)
        {
          break;
        }

      if (status == TRX_STATUS_TRXOFF)
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE,
                               TRX_CMD_RX_ON);
          up_udelay(RF23X_TIME_TRXOFF_TO_PLL);
        }

      if ((status == TRX_STATUS_RXON) || (status == TRX_STATUS_PLLON))
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE,
                               TRX_CMD_TX_ARET_ON);
          up_udelay(RF23X_TIME_TRANSITION_PLL_ACTIVE);
        }
      else if (status == TRX_STATUS_RXAACKON)
        {
          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE,
                               TRX_CMD_RX_ON);
          up_udelay(RF23X_TIME_TRANSITION_PLL_ACTIVE);

          at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE,
                               TRX_CMD_TX_ARET_ON);
          up_udelay(RF23X_TIME_TRANSITION_PLL_ACTIVE);
        }

      status = at86rf23x_getregbits(dev->spi, RF23X_TRXSTATUS_STATUS);
      if (status != TRX_STATUS_TXARETON)
        {
          ret = ERROR;
        }

      break;

    case TRX_STATUS_SLEEP:
      at86rf23x_setregbits(dev->spi, RF23X_TRXCMD_STATE, TRX_CMD_FORCETRXOFF);
      up_udelay(RF23X_TIME_CMD_FORCE_TRX_OFF);

      dev->lower->slptr(dev->lower, true);
      up_udelay(RF23X_TIME_TRXOFF_TO_SLEEP);

      status = at86rf23x_getregbits(dev->spi, RF23X_TRXSTATUS_STATUS);
      break;

    default:
      wlerr("ERRPR: %s \n", EINVAL_STR);
      init_status = 0;  /* Placed this here to keep compiler if no debug */
      return -EINVAL;
    }

  if (ret == ERROR)
    {
      wlerr("ERROR: State Transition Error\n");
    }

  wlinfo("Radio state change state[0x%02x]->state[0x%02x]\n",
         init_status, status);
  return ret;
}

/****************************************************************************
 * Name: at86rf23x_setchannel
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
 *
 ****************************************************************************/

static int at86rf23x_setchannel(FAR struct ieee802154_radio_s *ieee,
                                uint8_t chan)
{
  FAR struct at86rf23x_dev_s *dev = (FAR struct at86rf23x_dev_s *)ieee;
  uint8_t state;

  /* Validate if chan is a valid channel */

  if (chan < 11 || chan > 26)
    {
      wlerr("ERROR: Invalid requested chan: %d\n", chan);
      return -EINVAL;
    }

  /* Validate we are in an acceptable mode to change the channel */

  state = at86rf23x_get_trxstate(dev);

  if ((TRX_STATUS_SLEEP == state) || (TRX_STATUS_PON == state))
    {
      wlerr("ERROR: Radio in invalid mode [%02x] to set the channel\n",
            state);
      return ERROR;
    }

  /* Set the Channel on the transceiver */

  at86rf23x_setregbits(dev->spi, RF23X_CCA_BITS_CHANNEL, chan);

  /* Set the channel within local device */

  dev->channel = chan;

  wlinfo("CHANNEL Changed to %d\n", chan);
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_getchannel
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
 *   This section assumes that the transceiver is not in SLEEP or P_ON state.
 *
 *
 ****************************************************************************/

static int at86rf23x_getchannel(FAR struct ieee802154_radio_s *ieee,
                                FAR uint8_t *chan)
{
  FAR struct at86rf23x_dev_s *dev = (FAR struct at86rf23x_dev_s *)ieee;

  /* Set the Channel on the transceiver */

  *chan = at86rf23x_getregbits(dev->spi, RF23X_CCA_BITS_CHANNEL);

  /* Set the channel within local device */

  dev->channel = *chan;
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_setpanid
 *
 * Description:
 *   Define the PAN ID for the network.
 *
 ****************************************************************************/

static int at86rf23x_setpanid(FAR struct ieee802154_radio_s *ieee,
                              uint16_t panid)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;
  uint8_t *pan = (uint8_t *)&panid;

  at86rf23x_setreg(dev->spi, RF23X_REG_PANID0, pan[0]);
  at86rf23x_setreg(dev->spi, RF23X_REG_PANID1, pan[1]);

  return OK;
}

/****************************************************************************
 * Name: at86rf23x_getpanid
 *
 * Description:
 *   Define the PAN ID for the network.
 *
 ****************************************************************************/

static int at86rf23x_getpanid(FAR struct ieee802154_radio_s *ieee,
                              FAR uint16_t *panid)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;
  uint8_t *pan = (uint8_t *)panid;

  /* TODO: Check if we need to pay attention to endianness */

  pan[0] = at86rf23x_getreg(dev->spi, RF23X_REG_PANID0);
  pan[1] = at86rf23x_getreg(dev->spi, RF23X_REG_PANID1);
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_setsaddr
 *
 * Description:
 *   Define the Short Address for the device.
 *
 ****************************************************************************/

static int at86rf23x_setsaddr(FAR struct ieee802154_radio_s *ieee,
                              uint16_t saddr)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;
  uint8_t *addr = (uint8_t *)&saddr;

  at86rf23x_setreg(dev->spi, RF23X_REG_SADDR0, addr[0]);
  at86rf23x_setreg(dev->spi, RF23X_REG_SADDR1, addr[1]);

  return OK;
}

/****************************************************************************
 * Name: at86rf23x_getsaddr
 *
 * Description:
 *   Get the short address of the device.
 *
 ****************************************************************************/

static int at86rf23x_getsaddr(FAR struct ieee802154_radio_s *ieee,
                              FAR uint16_t *saddr)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;
  uint8_t *addr = (uint8_t *)saddr;

  /* TODO: Check if we need to pay attention to endianness */

  addr[0] = at86rf23x_getreg(dev->spi, RF23X_REG_SADDR0);
  addr[1] = at86rf23x_getreg(dev->spi, RF23X_REG_SADDR1);

  return OK;
}

/****************************************************************************
 * Name: at86rf23x_seteaddr
 *
 * Description:
 *   Set the IEEE address of the device.
 *
 ****************************************************************************/

static int at86rf23x_seteaddr(FAR struct ieee802154_radio_s *ieee,
                              FAR uint8_t *eaddr)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;

  /* TODO: Check if we need to pay attention to endianness */

  at86rf23x_setreg(dev->spi, RF23X_REG_IEEEADDR0, eaddr[0]);
  at86rf23x_setreg(dev->spi, RF23X_REG_IEEEADDR1, eaddr[1]);
  at86rf23x_setreg(dev->spi, RF23X_REG_IEEEADDR2, eaddr[2]);
  at86rf23x_setreg(dev->spi, RF23X_REG_IEEEADDR3, eaddr[3]);
  at86rf23x_setreg(dev->spi, RF23X_REG_IEEEADDR4, eaddr[4]);
  at86rf23x_setreg(dev->spi, RF23X_REG_IEEEADDR5, eaddr[5]);
  at86rf23x_setreg(dev->spi, RF23X_REG_IEEEADDR6, eaddr[6]);
  at86rf23x_setreg(dev->spi, RF23X_REG_IEEEADDR7, eaddr[7]);

  return OK;
}

/****************************************************************************
 * Name: at86rf23x_geteaddr
 *
 * Description:
 *   Get the IEEE address of the device.
 *
 ****************************************************************************/

static int at86rf23x_geteaddr(FAR struct ieee802154_radio_s *ieee,
                              FAR uint8_t *eaddr)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;

  /* TODO: Check if we need to pay attention to endianness */

  eaddr[0] = at86rf23x_getreg(dev->spi, RF23X_REG_IEEEADDR0);
  eaddr[1] = at86rf23x_getreg(dev->spi, RF23X_REG_IEEEADDR1);
  eaddr[2] = at86rf23x_getreg(dev->spi, RF23X_REG_IEEEADDR2);
  eaddr[3] = at86rf23x_getreg(dev->spi, RF23X_REG_IEEEADDR3);
  eaddr[4] = at86rf23x_getreg(dev->spi, RF23X_REG_IEEEADDR4);
  eaddr[5] = at86rf23x_getreg(dev->spi, RF23X_REG_IEEEADDR5);
  eaddr[6] = at86rf23x_getreg(dev->spi, RF23X_REG_IEEEADDR6);
  eaddr[7] = at86rf23x_getreg(dev->spi, RF23X_REG_IEEEADDR7);

  return OK;
}

/****************************************************************************
 * Name: at86rf23x_setpromisc
 *
 * Description:
 *   enable/disable promiscuous mode.
 *
 ****************************************************************************/

static int at86rf23x_setpromisc(FAR struct ieee802154_radio_s *ieee,
                                bool promisc)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;

  /* TODO: Check what mode I should be in to activate promiscuous mode:
   * This is way to simple of an implementation.  Many other things should
   * be set and/or checked before we set the device into promiscuous mode
   */

  at86rf23x_setregbits(dev->spi, RF23X_XAHCTRL1_BITS_PROM_MODE, promisc);
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_getpromisc
 *
 * Description:
 *   Check if the device is in promiscuous mode.
 *
 ****************************************************************************/

static int at86rf23x_getpromisc(FAR struct ieee802154_radio_s *ieee,
                                FAR bool *promisc)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;

  *promisc = at86rf23x_getregbits(dev->spi, RF23X_XAHCTRL1_BITS_PROM_MODE);
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_setdevmode
 *
 * Description:
 *   Check if the device is in promiscuous mode.
 *
 ****************************************************************************/

static int at86rf23x_setdevmode(FAR struct ieee802154_radio_s *ieee,
                                uint8_t mode)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;

  /* Define dev mode */

  if (mode == IEEE802154_MODE_PANCOORD)
    {
      at86rf23x_setregbits(dev->spi, RF23X_CSMASEED1_IAMCOORD_BITS, 0x01);
    }
  else if (mode == IEEE802154_MODE_COORD)
    {
      /* ????? */
    }
  else if (mode == IEEE802154_MODE_DEVICE)
    {
      at86rf23x_setregbits(dev->spi, RF23X_CSMASEED1_IAMCOORD_BITS, 0x00);
    }
  else
    {
      return -EINVAL;
    }

  dev->devmode = mode;
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_getdevmode
 *
 * Description:
 *   get the device mode type of the radio.
 *
 ****************************************************************************/

static int at86rf23x_getdevmode(FAR struct ieee802154_radio_s *ieee,
                                FAR uint8_t *mode)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;
  int val;

  val = at86rf23x_getregbits(dev->spi, RF23X_CSMASEED1_IAMCOORD_BITS);
  if (val == 1)
    {
      *mode = IEEE802154_MODE_PANCOORD;
    }
  else
    {
      *mode = IEEE802154_MODE_DEVICE;
    }

  return OK;
}

/****************************************************************************
 * Name: at86rf23x_settxpower
 *
 * Description:
 *   set the tx power attenuation or amplification
 *
 ****************************************************************************/

static int at86rf23x_settxpower(FAR struct ieee802154_radio_s *ieee,
                                int32_t txpwr)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;

  /* TODO: this needs a lot of work to make sure all chips can share this function */

  /* Right now we only set tx power to 0 */

  at86rf23x_setreg(dev->spi, RF23X_REG_TXPWR, RF23X_TXPWR_0);
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_gettxpower
 *
 * Description:
 *  get the tx power attenuation or amplification.
 *
 ****************************************************************************/

static int at86rf23x_gettxpower(FAR struct ieee802154_radio_s *ieee,
                                FAR int32_t *txpwr)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;
  uint8_t reg;

  /* TODO: this needs a lot of work to make sure all chips can share this
   * function.
   */

  /* Right now we only get negative values */

  reg = at86rf23x_getreg(dev->spi, RF23X_REG_TXPWR);
  switch (reg)
    {
      case RF23X_TXPWR_POS_4:
        *txpwr = 0;
         break;

      case RF23X_TXPWR_POS_3_7:
         *txpwr = 0;
         break;

      case RF23X_TXPWR_POS_3_4:
         *txpwr = 0;
         break;

      case RF23X_TXPWR_POS_3:
         *txpwr = 0;
         break;

      case RF23X_TXPWR_POS_2_5:
        *txpwr = 0;
        break;

      case RF23X_TXPWR_POS_2:
         *txpwr = 0;
         break;

      case RF23X_TXPWR_POS_1:
         *txpwr = 0;
         break;

      case RF23X_TXPWR_0:
         *txpwr = 0;
         break;

      case RF23X_TXPWR_NEG_1:
         *txpwr = 1000;
         break;

      case RF23X_TXPWR_NEG_2:
         *txpwr = 2000;
         break;

       case RF23X_TXPWR_NEG_3:
         *txpwr = 3000;
         break;

       case RF23X_TXPWR_NEG_4:
         *txpwr = 4000;
         break;

       case RF23X_TXPWR_NEG_6:
         *txpwr = 6000;
         break;

       case RF23X_TXPWR_NEG_8:
         *txpwr = 8000;
         break;

       case RF23X_TXPWR_NEG_12:
         *txpwr = 12000;
         break;

       case RF23X_TXPWR_NEG_17:
         *txpwr = 17000;
         break;
    }

  return OK;
}

/****************************************************************************
 * Name: at86rf23x_setcca
 *
 * Description:
 *   Configures if energy detection is used or carrier sense.  The base
 *   measurement is configured here as well
 *
 ****************************************************************************/

static
  int at86rf23x_setcca(FAR struct ieee802154_radio_s *ieee,
                       FAR struct ieee802154_cca_s *cca)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;

  /* TODO: This doesn't fit the RF233 completely come back to this */

  if (!cca->use_ed && !cca->use_cs)
    {
      return -EINVAL;
    }

  if (cca->use_cs && cca->csth > 0x0f)
    {
      return -EINVAL;
    }

  if (cca->use_ed)
    {
      at86rf23x_setregbits(dev->spi, RF23X_CCA_BITS_MODE, RF23X_CCA_MODE_ED);
    }

  if (cca->use_cs)
    {
      at86rf23x_setregbits(dev->spi, RF23X_CCA_BITS_MODE, RF23X_CCA_MODE_CS);
    }

  memcpy(&dev->cca, cca, sizeof(struct ieee802154_cca_s));
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_getcca
 *
 * Description:
 *   Get CCA for ???: TODO: need to implement
 *
 ****************************************************************************/

static int at86rf23x_getcca(FAR struct ieee802154_radio_s *ieee,
                            FAR struct ieee802154_cca_s *cca)
{
  FAR struct at86rf23x_dev_s *dev = (struct at86rf23x_dev_s *)ieee;

#warning at86rf23x_getcca not implemented.

  UNUSED(dev);
  UNUSED(cca);

  return ERROR;
}

/****************************************************************************
 * Name: at86rf23x_energydetect
 *
 * Description:
 *   Perform energy detection scan. TODO: Need to implement.
 *
 ****************************************************************************/

static int at86rf23x_energydetect(FAR struct ieee802154_radio_s *ieee,
                                  FAR uint8_t *energy)
{
#warning at86rf23x_energydetect not implemented.

  /* Not yet implemented */

  return ERROR;
}

/****************************************************************************
 * Name: at86rf23x_initialize
 *
 * Description:
 *   Initialize the radio.
 *
 ****************************************************************************/

int at86rf23x_initialize(FAR struct at86rf23x_dev_s *dev)
{
  uint8_t part;
  uint8_t version;

  at86rf23x_resetrf(dev);

  part = at86rf23x_getreg(dev->spi, RF23X_REG_PART);
  version = at86rf23x_getreg(dev->spi, RF23X_REG_VERSION);

  wlinfo("Radio part: 0x%02x version: 0x%02x found\n", part, version);
  return OK;
}

/****************************************************************************
 * Name: at86rf23x_resetrf
 *
 * Description:
 *   Hard Reset of the radio.  The reset also brings the radio into the
 *   TRX_OFF state.
 *
 ****************************************************************************/

static int at86rf23x_resetrf(FAR struct at86rf23x_dev_s *dev)
{
  FAR const struct at86rf23x_lower_s *lower = dev->lower;
  uint8_t trx_status;
  uint8_t retry_cnt = 0;

  /* Reset the radio */

  lower->reset(lower, false);
  lower->slptr(lower, false);

  up_udelay(RF23X_TIME_RESET);
  lower->reset(lower, true);

  /* Dummy read of IRQ register */

  at86rf23x_getreg(dev->spi, RF23X_REG_IRQ_STATUS);

  do
    {
      trx_status = at86rf23x_set_trxstate(dev, TRX_CMD_TRXOFF, true);

      if (retry_cnt == RF23X_MAX_RETRY_RESET_TO_TRX_OFF)
        {
          wlerr("ERROR: Reset of transceiver failed\n");
          return ERROR;
        }

      retry_cnt++;
    }
  while (trx_status != OK);

  return OK;
}

/****************************************************************************
 * Name: at86rf23x_rxenable
 *
 * Description:
 *   puts the radio into RX mode and brings in the buffer to handle
 *   RX messages.
 *
 ****************************************************************************/

static int at86rf23x_rxenable(FAR struct ieee802154_radio_s *ieee, bool state,
                              FAR struct ieee802154_packet_s *packet)
{
  FAR struct at86rf23x_dev_s *dev = (FAR struct at86rf23x_dev_s *)ieee;

  /* Set the radio to the receive state */

  return at86rf23x_set_trxstate(dev, TRX_CMD_RX_ON, false);

  /* Enable the RX IRQ */

  /* TODO:
   * I am not sure what to do here since the at86rf23x shares the
   * irq with the tx finished.  Better planning needs to be done on
   * my end.
   */

  /* Set buffer to receive next packet */

  ieee->rxbuf = packet;
}

/****************************************************************************
 * Name: at86rf23x_interrupt
 *
 * Description:
 *   Actual interrupt handler ran inside privileged space.
 *
 ****************************************************************************/

static int at86rf23x_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct at86rf23x_dev_s *dev = (FAR struct at86rf23x_dev_s *)arg;

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

  return work_queue(HPWORK, &dev->irqwork, at86rf23x_irqworker,
                    (FAR void *)dev, 0);
}

/****************************************************************************
 * Name: at86rf23x_regdump
 *
 * Description:
 *   Dumps all the RF23X radios registers from 00 - 2F there are a few other
 *   registers that don't get dumped but just fore the ease of code I left
 *   them out.
 *
 ****************************************************************************/

static int at86rf23x_regdump(FAR struct at86rf23x_dev_s *dev)
{
  uint32_t i;
  char buf[4 + 16 * 3 + 2 + 1];
  int len = 0;

  wlinfo("RF23X regs:\n");

  for (i = 0; i < 0x30; i++)
    {
      /* First row and every 15 regs */

      if ((i & 0x0f) == 0)
        {
          len = sprintf(buf, "%02x: ", i & 0xff);
        }

      /* Print the register value */

      len += sprintf(buf + len, "%02x ", at86rf23x_getreg(dev->spi, i));

      /* At the end of each 15 regs or end of rf233s regs and actually print
       * debug message.
       */

      if ((i & 15) == 15 || i == 0x2f)
        {
          sprintf(buf + len, "\n");
          wlinfo("%s", buf);
        }
    }

  /* TODO: I have a few more regs that are not consecutive.  Will print later */

  return 0;
}

/****************************************************************************
 * Name: at86rf23x_irqworker
 *
 * Description:
 *   Actual thread to handle the irq outside of privaleged mode.
 *
 ****************************************************************************/

static void at86rf23x_irqworker(FAR void *arg)
{
  FAR struct at86rf23x_dev_s *dev = (FAR struct at86rf23x_dev_s *)arg;
  uint8_t irq_status = at86rf23x_getreg(dev->spi, RF23X_REG_IRQ_STATUS);

  wlinfo("IRQ: 0x%02X\n", irq_status);

  if ((irq_status & (1 << 3)) != 0)
    {
      if ((irq_status & (1 << 2)) != 0)
        {
          at86rf23x_irqwork_rx(dev);
        }
      else
        {
          at86rf23x_irqwork_tx(dev);
        }
    }
  else
    {
      wlerr("ERROR: Unknown IRQ Status: %d\n", irq_status);

      /* Re-enable the IRQ even if we don't know how to handle previous
       * status.
       */

      dev->lower->enable(dev->lower, true);
    }
}

/****************************************************************************
 * Name: at86rf23x_irqwork_rx
 *
 * Description:
 *   Misc/unofficial device controls.
 *
 ****************************************************************************/

static void at86rf23x_irqwork_rx(FAR struct at86rf23x_dev_s *dev)
{
  uint8_t rx_len;

  wlinfo("6LOWPAN:Rx IRQ\n");

  rx_len = at86rf23x_readframe(dev->spi, dev->ieee.rxbuf->data);

  dev->ieee.rxbuf->len = rx_len;
  dev->ieee.rxbuf->lqi = 0;
  dev->ieee.rxbuf->rssi = 0;

  nxsem_post(&dev->ieee.rxsem);

  /* TODO:
   *   Not to sure yet what I should do here.  I will something
   *   soon.
   */

  /* Re-enable the IRQ */

  dev->lower->enable(dev->lower, true);
}

/****************************************************************************
 * Name: at86rf23x_irqwork_tx
 *
 * Description:
 *   Misc/unofficial device controls.
 *
 ****************************************************************************/

static void at86rf23x_irqwork_tx(FAR struct at86rf23x_dev_s *dev)
{
  wlinfo("6LOWPAN:Tx IRQ\n");

  /* TODO:
   *   There needs to be more here but for now just alert the waiting
   *   thread.  Maybe put it back into Rx mode
   */

  /* Re enable the IRQ */

  dev->lower->enable(dev->lower, true);

  nxsem_post(&dev->ieee.txsem);
}

/****************************************************************************
 * Name: at86rf23x_transmit
 *
 * Description:
 *   transmission the packet.  Send the packet to the radio and initiate the
 *   transmit process.  Then block the function till we have a successful
 *   transmission
 *
 ****************************************************************************/

static int at86rf23x_transmit(FAR struct ieee802154_radio_s *ieee,
                              FAR struct ieee802154_packet_s *packet)
{
  FAR struct at86rf23x_dev_s *dev = (FAR struct at86rf23x_dev_s *)ieee;

  /* TODO:
   * A plan needs to be made on when we declare the transmission successful.
   *
   * 1.  If the packet is sent
   * 2.  If we receive an ACK.
   * 3.  Where do we control the retry process?
   */

  if (at86rf23x_set_trxstate(dev, TRX_CMD_PLL_ON, false))
    {
      at86rf23x_writeframe(dev->spi, packet->data, packet->len);
    }
  else
    {
      wlerr("ERROR: Transmit could not put the radio in a Tx state\n");
      return ERROR;
    }

  /* Put the thread that requested transfer to a waiting state */

  nxsem_wait(&dev->ieee.txsem);

  /* TODO Verify that I want to stay in the PLL state or if I want to roll
   * back to RX_ON.
   */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at86rf23x_init
 *
 * Description:
 *   Return an at86rf23x device for use by other drivers.
 *
 ****************************************************************************/

FAR struct ieee802154_radio_s *
  at86rf23x_init(FAR struct spi_dev_s *spi,
                 FAR const struct at86rf23x_lower_s *lower)
{
  FAR struct at86rf23x_dev_s *dev;
  struct ieee802154_cca_s   cca;

  dev = &g_at86rf23x_devices[0];

  /* Attach the interface, lower driver, and devops */

  dev->spi = spi;
  dev->lower = lower;
  dev->ieee.ops = &at86rf23x_devops;

  /* Attach irq */

  if (lower->attach(lower, at86rf23x_interrupt, dev) != OK)
    {
      return NULL;
    }

  nxsem_init(&dev->ieee.rxsem, 0, 0);
  nxsem_init(&dev->ieee.txsem, 0, 0);

  /* Initialize device */

  at86rf23x_initialize(dev);

  /* Configure the desired IRQs of the devices */

  at86rf23x_setreg(dev->spi, RF23X_REG_IRQ_MASK, RF23X_IRQ_MASK_DEFAULT);

  /* Turn the PLL to the on state */

  at86rf23x_set_trxstate(dev, TRX_CMD_PLL_ON, false);

  /* SEED value of the CSMA backoff algorithm. */

#ifdef RF23X_ANTENNA_DIVERSITY
  /* Use antenna diversity */

  trx_bit_write(SR_ANT_CTRL, ANTENNA_DEFAULT);
  trx_bit_write(SR_PDT_THRES, THRES_ANT_DIV_ENABLE);
  trx_bit_write(SR_ANT_DIV_EN, ANT_DIV_ENABLE);
  trx_bit_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_ENABLE);
#endif

#ifdef RF23X_TIMESTAMP
  /* Enable timestamp init code goes here */

#endif

#ifdef RF23X_RF_FRONTEND_CTRL
  /* Init front end control code goes here */

#endif

  /* Set the channel of the radio */

  at86rf23x_setchannel(&dev->ieee, 12);

#if 0
  /* Configure the Pan id */

  at86rf23x_setpanid(&dev->ieee, IEEE802154_PAN_DEFAULT);
#endif

#if 0
  /* Configure the Short Addr */

  at86rf23x_setsaddr(&dev->ieee, IEEE802154_SADDR_UNSPEC);
#endif

#if 0
  /* Configure the IEEE Addr */

  at86rf23x_seteaddr(&dev->ieee, IEEE802154_EADDR_UNSPEC);
#endif

  /* Default device params at86rf23x defaults to energy detect only */

  cca.use_ed = 1;
  cca.use_cs = 0;
  cca.edth   = 0x60; /* CCA mode ED, no carrier sense, recommenced ED
                      * threshold -69 dBm */
  at86rf23x_setcca(&dev->ieee, &cca);

#if 0
  /* Put the Device to RX ON Mode */

  at86rf23x_set_trxstate(dev, TRX_CMD_RX_ON, false);
#endif

  /* Enable Radio IRQ */

  lower->enable(lower, true);
  return &dev->ieee;
}
