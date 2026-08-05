/****************************************************************************
 * drivers/wireless/lpwan/sx1301/sx1301.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Character driver for the Semtech SX1301 LoRa concentrator.
 *
 * The chip has no register level documentation in the public datasheet: the
 * start-up sequence below follows the Semtech reference HAL (lora_gateway,
 * libloragw) step by step, including the parts that look redundant.  The
 * order matters and several of the writes have been verified on hardware:
 *
 *   - the modems must be enabled *before* the MCU firmwares are loaded;
 *   - the calibration firmware has to run for about 2.3 s in the AGC MCU
 *     before the production firmware is loaded;
 *   - register 0x6a is written with the PROM multiplexers explicitly set,
 *     never with a read-modify-write, and the MCUs are released with 0x0c
 *     (not 0x00) so that they keep access to their program memory;
 *   - TX_SWAP_IQ (bit 7 of page 1 register 0x2a) must be set or downlinks
 *     silently fail.
 *
 * Interface:
 *
 *   read()  returns whole multiples of struct lora_gw_rxpkt_s, blocking
 *           until at least one packet is available unless O_NONBLOCK is set.
 *   write() takes exactly one struct lora_gw_txpkt_s.
 *   ioctl() see include/nuttx/wireless/lpwan/lora_gw.h
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/wireless/wireless.h>

#include "sx1301_fw.h"
#include "sx1301_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Multi-SF correlator mask: bit 0 is SF6 ... bit 6 is SF12, so 0x7e enables
 * SF7 to SF12.  Without this the demodulators never detect a preamble.
 */

#define SX1301_CORR_SF7_SF12     0x7e

/* Frame synchronisation words, as peak positions.  The chip powers up for a
 * private network (1 and 2); the public LoRaWAN network uses 3 and 4, which
 * is what the 0x34 sync word of an end device maps to.  Getting this wrong
 * costs nothing visible: the demodulators simply never detect a frame.
 */

#define SX1301_SYNCH_PRIVATE     (1 | (2 << 4))
#define SX1301_SYNCH_PUBLIC      (3 | (4 << 4))

#ifdef CONFIG_LPWAN_SX1301_PRIVATE_NETWORK
#  define SX1301_SYNCH_WORD      SX1301_SYNCH_PRIVATE
#else
#  define SX1301_SYNCH_WORD      SX1301_SYNCH_PUBLIC
#endif

/* Bits of the modem enable register (0x10) */

#define SX1301_EN_MBWSSF         (1 << 0)
#define SX1301_EN_CONCENTRATOR   (1 << 1)
#define SX1301_EN_FSK            (1 << 2)
#define SX1301_EN_GLOBAL         (1 << 3)

/* Bits of the MCU control register (0x6a) */

#define SX1301_MCU_RST_ARB       (1 << 0)
#define SX1301_MCU_RST_AGC       (1 << 1)
#define SX1301_MCU_MUX_ARB       (1 << 2)
#define SX1301_MCU_MUX_AGC       (1 << 3)

/* AGC MCU command channel.  Commands are written to RADIO_SELECT, each one
 * preceded by CMD_WAIT, and the progress is read back from AGC_STATUS.
 */

#define SX1301_AGC_CMD_WAIT      16
#define SX1301_AGC_STATUS_READY  0x10
#define SX1301_AGC_STATUS_DONE   0x40
#define SX1301_CAL_STATUS_OK     0x81

/* Number of entries of the transmit gain look-up table.  With a full table
 * the AGC advances on its own and must not receive CMD_ABORT.
 */

#define SX1301_TX_LUT_SIZE       16

/* Default transmit start delay, in microseconds, and the correction applied
 * for a 125 kHz channel (1.5 us in the reference HAL).
 */

#define SX1301_TX_START_DELAY    1497
#define SX1301_TX_START_DLY_125  1495

/* Payload plus metadata of one packet in the receive FIFO */

#define SX1301_RXBUF_SIZE        (LORA_GW_MAX_PAYLOAD + \
                                  SX1301_RX_METADATA_NB)

/* The five byte FIFO header read from register 0x0b */

#define SX1301_FIFO_HDR_SIZE     5

/* Reduced spreading factor mode, needed by the timestamp correction */

#define SX1301_PPM_ON(bw, sf) \
  ((((bw) == LORA_GW_BW_125K) && ((sf) == 11 || (sf) == 12)) || \
   (((bw) == LORA_GW_BW_250K) && ((sf) == 12)))

/* The bridge that reaches the SX125x radios needs time to shift the byte in
 * and out after the chip select pulse.  The reference HAL gets this for free
 * from the cost of a Linux spidev transaction; here the register writes are
 * fast enough that the read back register has to be given a moment, or it
 * always reads as zero.
 */

#define SX1301_BRIDGE_DELAY_US   20

/* Interval used to poll the receive FIFO of a blocking read().  The SX1301
 * has no usable interrupt line on the reference shields.
 */

#define SX1301_RXPOLL_USEC       10000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sx1301_open(FAR struct file *filep);
static int  sx1301_close(FAR struct file *filep);
static ssize_t sx1301_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t sx1301_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int  sx1301_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sx1301_fops =
{
  sx1301_open,    /* open */
  sx1301_close,   /* close */
  sx1301_read,    /* read */
  sx1301_write,   /* write */
  NULL,           /* seek */
  sx1301_ioctl,   /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  NULL            /* poll */
};

/* Transmit gain look-up table of the RisingHF RHF0M301 reference design.
 * Each entry is mix_gain + 16 * dac_gain + 64 * pa_gain, and g_sx1301_tx_pwr
 * gives the antenna power each entry is calibrated for.
 */

static const uint8_t g_sx1301_tx_lut[SX1301_TX_LUT_SIZE] =
{
  8 + (16 * 3) + (64 * 0),   /* -6 dBm */
  10 + (16 * 3) + (64 * 0),  /* -3 dBm */
  12 + (16 * 3) + (64 * 0),  /*  0 dBm */
  8 + (16 * 3) + (64 * 1),   /*  3 dBm */
  10 + (16 * 3) + (64 * 1),  /*  6 dBm */
  12 + (16 * 3) + (64 * 1),  /* 10 dBm */
  13 + (16 * 3) + (64 * 1),  /* 11 dBm */
  9 + (16 * 3) + (64 * 2),   /* 12 dBm */
  15 + (16 * 3) + (64 * 1),  /* 13 dBm */
  10 + (16 * 3) + (64 * 2),  /* 14 dBm */
  11 + (16 * 3) + (64 * 2),  /* 16 dBm */
  9 + (16 * 3) + (64 * 3),   /* 20 dBm */
  10 + (16 * 3) + (64 * 3),  /* 24 dBm */
  11 + (16 * 3) + (64 * 3),  /* 25 dBm */
  12 + (16 * 3) + (64 * 3),  /* 26 dBm */
  14 + (16 * 3) + (64 * 3)   /* 27 dBm */
};

static const int8_t g_sx1301_tx_pwr[SX1301_TX_LUT_SIZE] =
{
  -6, -3, 0, 3, 6, 10, 11, 12, 13, 14, 16, 20, 24, 25, 26, 27
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx1301_msleep
 ****************************************************************************/

static void sx1301_msleep(unsigned int msec)
{
  nxsig_usleep(msec * 1000);
}

/****************************************************************************
 * Name: sx1301_radio_write
 *
 * Description:
 *   Write an SX125x radio register through the SPI bridge of the SX1301.
 *   The radios are not on the host SPI bus: the concentrator drives them
 *   from page 2 registers, one byte at a time, with an explicit chip select
 *   pulse.
 *
 ****************************************************************************/

static void sx1301_radio_write(FAR struct sx1301_dev_s *priv, uint8_t radio,
                               uint8_t addr, uint8_t value)
{
  uint8_t areg;
  uint8_t dreg;
  uint8_t csreg;

  if (radio == 0)
    {
      areg  = SX1301_REG_RADIO_A_ADDR;
      dreg  = SX1301_REG_RADIO_A_DATA;
      csreg = SX1301_REG_RADIO_A_CS;
    }
  else
    {
      areg  = SX1301_REG_RADIO_B_ADDR;
      dreg  = SX1301_REG_RADIO_B_DATA;
      csreg = SX1301_REG_RADIO_B_CS;
    }

  sx1301_reg_write(priv, SX1301_PAGE_2, csreg, 0);
  sx1301_reg_write(priv, SX1301_PAGE_2, areg, 0x80 | (addr & 0x7f));
  sx1301_reg_write(priv, SX1301_PAGE_2, dreg, value);
  sx1301_reg_write(priv, SX1301_PAGE_2, csreg, 1);
  up_udelay(SX1301_BRIDGE_DELAY_US);
  sx1301_reg_write(priv, SX1301_PAGE_2, csreg, 0);
  up_udelay(SX1301_BRIDGE_DELAY_US);
}

/****************************************************************************
 * Name: sx1301_radio_read
 ****************************************************************************/

static void sx1301_radio_read(FAR struct sx1301_dev_s *priv, uint8_t radio,
                              uint8_t addr, FAR uint8_t *value)
{
  uint8_t areg;
  uint8_t dreg;
  uint8_t csreg;
  uint8_t rbreg;

  if (radio == 0)
    {
      areg  = SX1301_REG_RADIO_A_ADDR;
      dreg  = SX1301_REG_RADIO_A_DATA;
      csreg = SX1301_REG_RADIO_A_CS;
      rbreg = SX1301_REG_RADIO_A_RB;
    }
  else
    {
      areg  = SX1301_REG_RADIO_B_ADDR;
      dreg  = SX1301_REG_RADIO_B_DATA;
      csreg = SX1301_REG_RADIO_B_CS;
      rbreg = SX1301_REG_RADIO_B_RB;
    }

  sx1301_reg_write(priv, SX1301_PAGE_2, csreg, 0);
  sx1301_reg_write(priv, SX1301_PAGE_2, areg, addr & 0x7f);
  sx1301_reg_write(priv, SX1301_PAGE_2, dreg, 0);
  sx1301_reg_write(priv, SX1301_PAGE_2, csreg, 1);
  up_udelay(SX1301_BRIDGE_DELAY_US);
  sx1301_reg_write(priv, SX1301_PAGE_2, csreg, 0);
  up_udelay(SX1301_BRIDGE_DELAY_US);
  sx1301_reg_read(priv, SX1301_PAGE_2, rbreg, value);
}

/****************************************************************************
 * Name: sx1301_setup_radio
 *
 * Description:
 *   Bring up one SX125x front-end: analogue configuration, PLL frequency and
 *   oscillator, then wait for the PLL to lock.  Radio B has been seen to
 *   need a couple of milliseconds more than radio A, hence the retries.
 *
 ****************************************************************************/

static int sx1301_setup_radio(FAR struct sx1301_dev_s *priv, uint8_t radio)
{
  uint32_t freqreg;
  uint8_t version = 0;
  uint8_t status = 0;
  uint8_t clkout;
  int i;

  clkout = (radio == 0) ? 1 : 0;

  sx1301_radio_read(priv, radio, 0x07, &version);
  wlinfo("Radio %c: SX125x version 0x%02x\n", radio == 0 ? 'A' : 'B',
         version);

  /* Analogue configuration, values of the reference HAL setup_sx125x() */

  sx1301_radio_write(priv, radio, 0x10, 0x03); /* TX DAC clock select */
  sx1301_radio_write(priv, radio, 0x13, clkout);
  sx1301_radio_write(priv, radio, 0x10, 0x67); /* Full TX analogue chain */
  sx1301_radio_write(priv, radio, 0x08, 46);   /* TX mix and DAC gain */
  sx1301_radio_write(priv, radio, 0x0a, 32);   /* TX bandwidth */
  sx1301_radio_write(priv, radio, 0x0b, 5);    /* TX DAC bandwidth */
  sx1301_radio_write(priv, radio, 0x0c, 57);   /* LNA and baseband gain */
  sx1301_radio_write(priv, radio, 0x0d, 248);  /* RX ADC trim and bandwidth */
  sx1301_radio_write(priv, radio, 0x0e, 0);    /* RX PLL bandwidth */
  sx1301_radio_write(priv, radio, 0x26, 45);   /* Crystal oscillator */

  /* Fractional-N divider setting for a 32 MHz reference:
   * freqreg = freq_hz * 2^19 / 32000000, computed without floating point.
   */

  freqreg = (uint32_t)(((uint64_t)priv->rf[radio].freq_hz << 19) / 32000000);

  sx1301_radio_write(priv, radio, 0x01, (freqreg >> 16) & 0xff);
  sx1301_radio_write(priv, radio, 0x02, (freqreg >> 8) & 0xff);
  sx1301_radio_write(priv, radio, 0x03, freqreg & 0xff);

  if (priv->rf[radio].tx_enable)
    {
      sx1301_radio_write(priv, radio, 0x04, (freqreg >> 16) & 0xff);
      sx1301_radio_write(priv, radio, 0x05, (freqreg >> 8) & 0xff);
      sx1301_radio_write(priv, radio, 0x06, freqreg & 0xff);
    }

  sx1301_radio_write(priv, radio, 0x00, 1);    /* Crystal on */
  sx1301_msleep(10);
  sx1301_radio_write(priv, radio, 0x00, 3);    /* Crystal plus RX PLL on */
  sx1301_msleep(10);

  for (i = 0; i < 5; i++)
    {
      sx1301_radio_read(priv, radio, 0x11, &status);
      if ((status & 0x02) != 0)
        {
          wlinfo("Radio %c: PLL locked at %" PRIu32 " Hz\n",
                 radio == 0 ? 'A' : 'B', priv->rf[radio].freq_hz);
          return OK;
        }

      sx1301_msleep(1);
    }

  wlwarn("WARNING: radio %c PLL lock not confirmed, status 0x%02x\n",
         radio == 0 ? 'A' : 'B', status);
  return OK;
}

/****************************************************************************
 * Name: sx1301_hwreset
 ****************************************************************************/

static void sx1301_hwreset(FAR struct sx1301_dev_s *priv)
{
  DEBUGASSERT(priv->lower != NULL && priv->lower->reset != NULL);

  priv->lower->reset(priv->lower, false);
  sx1301_msleep(100);
  priv->lower->reset(priv->lower, true);
  sx1301_msleep(100);
  priv->lower->reset(priv->lower, false);
  sx1301_msleep(500);

  priv->page = 0;
  wlinfo("Concentrator reset\n");
}

/****************************************************************************
 * Name: sx1301_loadfw
 *
 * Description:
 *   Copy one firmware image into the program memory of an internal MCU.
 *
 *   Register 0x6a holds one reset bit and one multiplexer bit per MCU.  The
 *   multiplexer must be zero for the MCU being loaded (host owns its memory)
 *   and one for the other, and only one may be in host mode at a time; the
 *   register is therefore written whole rather than bit by bit.
 *
 ****************************************************************************/

static int sx1301_loadfw(FAR struct sx1301_dev_s *priv, uint8_t mcu,
                         FAR const uint8_t *fw)
{
  uint8_t ctrl;
  uint8_t readback = 0;
  uint8_t dummy = 0;
  uint8_t verify[4];
  size_t off;
  size_t len;
  int ret;

  ctrl = SX1301_MCU_RST_ARB | SX1301_MCU_RST_AGC;
  ctrl |= (mcu == 0) ? SX1301_MCU_MUX_AGC : SX1301_MCU_MUX_ARB;

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_MCU_CTRL, ctrl);
  sx1301_reg_read(priv, SX1301_PAGE_0, SX1301_REG_MCU_CTRL, &readback);
  if (readback != ctrl)
    {
      wlwarn("WARNING: MCU%d control wrote 0x%02x read 0x%02x\n",
             mcu, ctrl, readback);
    }

  /* Let the multiplexer settle before touching the program memory */

  sx1301_msleep(1);

  sx1301_reg_write(priv, SX1301_PAGE_ANY, SX1301_REG_PROM_ADDR, 0);

  for (off = 0; off < SX1301_MCU_FW_SIZE; off += len)
    {
      len = SX1301_MCU_FW_SIZE - off;
      if (len > 1024)
        {
          len = 1024;
        }

      ret = sx1301_reg_wrburst(priv, SX1301_PAGE_ANY, SX1301_REG_PROM_DATA,
                               fw + off, len);
      if (ret < 0)
        {
          wlerr("ERROR: MCU%d firmware write failed at %zu: %d\n",
                mcu, off, ret);
          return ret;
        }
    }

  /* The first read after a write returns stale data on this interface, so
   * rewind, throw one byte away and only then compare.
   */

  sx1301_reg_read(priv, SX1301_PAGE_ANY, SX1301_REG_PROM_DATA, &dummy);
  sx1301_reg_write(priv, SX1301_PAGE_ANY, SX1301_REG_PROM_ADDR, 0);
  sx1301_reg_read(priv, SX1301_PAGE_ANY, SX1301_REG_PROM_DATA, &dummy);
  sx1301_reg_rdburst(priv, SX1301_PAGE_ANY, SX1301_REG_PROM_DATA,
                     verify, sizeof(verify));

  if (memcmp(fw, verify, sizeof(verify)) != 0)
    {
      wlerr("ERROR: MCU%d program memory verify failed: "
            "wrote %02x %02x %02x %02x read %02x %02x %02x %02x\n",
            mcu, fw[0], fw[1], fw[2], fw[3],
            verify[0], verify[1], verify[2], verify[3]);
      return -EIO;
    }

  /* Hand the program memory back to the MCU */

  ctrl |= (mcu == 0) ? SX1301_MCU_MUX_ARB : SX1301_MCU_MUX_AGC;
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_MCU_CTRL, ctrl);

  wlinfo("MCU%d firmware loaded (%d bytes)\n", mcu, SX1301_MCU_FW_SIZE);
  return OK;
}

/****************************************************************************
 * Name: sx1301_mcu_fwversion
 *
 * Description:
 *   Read the version byte an MCU firmware publishes at address 0x20 of its
 *   RAM, through the debug port on page 2.
 *
 ****************************************************************************/

static uint8_t sx1301_mcu_fwversion(FAR struct sx1301_dev_s *priv,
                                    uint8_t mcu)
{
  uint8_t version = 0;
  uint8_t addrreg;
  uint8_t datareg;

  addrreg = (mcu == 0) ? SX1301_REG_DBG_ARB_ADDR : SX1301_REG_DBG_AGC_ADDR;
  datareg = (mcu == 0) ? SX1301_REG_DBG_ARB_DATA : SX1301_REG_DBG_AGC_DATA;

  sx1301_reg_write(priv, SX1301_PAGE_2, addrreg, 0x20);
  sx1301_reg_read(priv, SX1301_PAGE_2, datareg, &version);

  return version;
}

/****************************************************************************
 * Name: sx1301_calibrate
 *
 * Description:
 *   Run the calibration firmware in the AGC MCU.  Without this the AGC never
 *   initialises and the receive chain stays deaf.
 *
 ****************************************************************************/

static int sx1301_calibrate(FAR struct sx1301_dev_s *priv)
{
  uint8_t cmd;
  uint8_t status = 0;
  uint8_t version;
  int ret;

  /* Build the calibration command: bit 0 and 1 request the RX IQ mismatch
   * calibration of radio A and B, bit 2 and 3 the TX DC offset calibration,
   * bit 4 selects a DAC gain of 3 and bit 5 stays clear for the SX1257.
   */

  cmd = 0;
  if (priv->rf[0].enable)
    {
      cmd |= 0x01;
      if (priv->rf[0].tx_enable)
        {
          cmd |= 0x04;
        }
    }

  if (priv->rf[1].enable)
    {
      cmd |= 0x02;
      if (priv->rf[1].tx_enable)
        {
          cmd |= 0x08;
        }
    }

  cmd |= 0x10;

  wlinfo("Calibration starting, command 0x%02x\n", cmd);

  ret = sx1301_loadfw(priv, 1, g_sx1301_cal_fw);
  if (ret < 0)
    {
      return ret;
    }

  /* Give the radios back to the AGC, pass the command and release the AGC */

  sx1301_reg_setbit(priv, SX1301_PAGE_0, SX1301_REG_FORCE_CTRL, 1, false);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RADIO_SELECT, cmd);
  sx1301_reg_setbit(priv, SX1301_PAGE_0, SX1301_REG_MCU_CTRL, 1, false);

  /* The version byte only changes once the MCU has restarted and rewritten
   * it, so right after the release it may still show the previous firmware.
   * Report it, do not treat it as a failure.
   */

  version = sx1301_mcu_fwversion(priv, 1);
  wlinfo("Calibration firmware version %d (expected %d)\n",
         version, SX1301_FW_VERSION_CAL);

  /* Release the host control so that the MCU can drive the chip.  Both
   * registers are page independent and must be written raw, since the page
   * register itself is used as part of the sequence.
   */

  sx1301_reg_write(priv, SX1301_PAGE_ANY, SX1301_REG_PAGE, 3);
  sx1301_reg_write(priv, SX1301_PAGE_ANY, SX1301_REG_EMERGENCY, 0);

  sx1301_msleep(2300);

  sx1301_reg_write(priv, SX1301_PAGE_ANY, SX1301_REG_EMERGENCY, 1);
  sx1301_reg_write(priv, SX1301_PAGE_ANY, SX1301_REG_PAGE, 0);

  sx1301_reg_read(priv, SX1301_PAGE_0, SX1301_REG_AGC_STATUS, &status);
  if ((status & SX1301_CAL_STATUS_OK) == SX1301_CAL_STATUS_OK)
    {
      wlinfo("Calibration done, status 0x%02x\n", status);
    }
  else
    {
      wlwarn("WARNING: calibration incomplete, status 0x%02x\n", status);
    }

  return OK;
}

/****************************************************************************
 * Name: sx1301_agc_command
 *
 * Description:
 *   Send one byte to the AGC MCU command channel and return the status the
 *   MCU reports afterwards.
 *
 ****************************************************************************/

static uint8_t sx1301_agc_command(FAR struct sx1301_dev_s *priv,
                                  uint8_t value)
{
  uint8_t status = 0;

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RADIO_SELECT,
                   SX1301_AGC_CMD_WAIT);
  sx1301_msleep(1);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RADIO_SELECT, value);
  sx1301_msleep(1);
  sx1301_reg_read(priv, SX1301_PAGE_0, SX1301_REG_AGC_STATUS, &status);

  return status;
}

/****************************************************************************
 * Name: sx1301_agc_start
 *
 * Description:
 *   Hand the transmit gain table and the channel mapping to the AGC MCU.
 *   The sequence is: sixteen gain table entries, the transmit frequency
 *   range selector, the channel selection option and finally the radio
 *   mapping.  The MCU reports 0x40 once it is running.
 *
 ****************************************************************************/

static void sx1301_agc_start(FAR struct sx1301_dev_s *priv)
{
  uint8_t status;
  uint8_t txfreq;
  int i;

  for (i = 0; i < SX1301_TX_LUT_SIZE; i++)
    {
      status = sx1301_agc_command(priv, g_sx1301_tx_lut[i]);
      wlinfo("AGC gain table entry %d = %d, status 0x%02x\n",
             i, g_sx1301_tx_lut[i], status);
    }

  /* Transmit frequency range: 3 above 768 MHz, 2 below, as in the reference
   * HAL.  A full gain table makes the AGC advance on its own, so no abort
   * command is sent here.
   */

  txfreq = (priv->rf[0].freq_hz > 768000000) ? 3 : 2;

  status = sx1301_agc_command(priv, txfreq);
  wlinfo("AGC transmit range %d, status 0x%02x\n", txfreq, status);

  status = sx1301_agc_command(priv, 0);
  wlinfo("AGC channel option, status 0x%02x\n", status);

  status = sx1301_agc_command(priv, priv->radio_select);
  if (status == SX1301_AGC_STATUS_DONE)
    {
      wlinfo("AGC running, radio map 0x%02x\n", priv->radio_select);
    }
  else
    {
      wlwarn("WARNING: AGC status 0x%02x, expected 0x%02x\n",
             status, SX1301_AGC_STATUS_DONE);
    }
}

/****************************************************************************
 * Name: sx1301_start
 *
 * Description:
 *   Full start-up sequence.  Must be called with the device locked.
 *
 ****************************************************************************/

static int sx1301_start(FAR struct sx1301_dev_s *priv)
{
  uint8_t modem_en;
  uint8_t status = 0;
  uint8_t version;
  uint32_t drift;
  int ret;
  int i;

  if (priv->started)
    {
      return -EALREADY;
    }

  /* Hardware reset and band selection of the front-end filters */

  sx1301_hwreset(priv);

  if (priv->lower->band_select != NULL)
    {
      priv->lower->band_select(priv->lower,
                               priv->rf[0].freq_hz > 900000000 ? 915 : 868);
    }

  ret = sx1301_reg_probe(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Soft reset, then gate every clock while the radios are configured */

  sx1301_reg_setbit(priv, SX1301_PAGE_ANY, SX1301_REG_PAGE, 7, true);
  sx1301_msleep(10);

  sx1301_reg_setbit(priv, SX1301_PAGE_0, SX1301_REG_GLOBAL_EN, 3, false);
  sx1301_reg_setbit(priv, SX1301_PAGE_0, SX1301_REG_CLK_CTRL, 0, false);

  /* Power the radios up and only then pulse their reset line.  Enabling
   * them is what starts the 32 MHz crystal of the front-end, and it needs
   * half a second to stabilise: talking to the radios earlier gets zeros
   * back from the bridge.
   */

  sx1301_reg_write(priv, SX1301_PAGE_2, SX1301_REG_RADIO_CFG, 0x03);
  sx1301_msleep(500);
  sx1301_reg_write(priv, SX1301_PAGE_2, SX1301_REG_RADIO_CFG, 0x07);
  sx1301_msleep(5);
  sx1301_reg_write(priv, SX1301_PAGE_2, SX1301_REG_RADIO_CFG, 0x03);
  sx1301_msleep(5);

  for (i = 0; i < LORA_GW_RF_CHAIN_NB; i++)
    {
      if (priv->rf[i].enable)
        {
          ret = sx1301_setup_radio(priv, i);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  /* Clocks back on */

  sx1301_reg_setbit(priv, SX1301_PAGE_0, SX1301_REG_GLOBAL_EN, 3, true);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_CLK_CTRL, 0x03);

  /* Drive the five concentrator GPIOs as outputs and let the AGC use them
   * for the antenna switch.  Without this the switch stays in receive and
   * nothing leaves the antenna during a downlink.
   */

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_GPIO_MODE, 31);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_GPIO_SELECT, 2);

  /* Fixed tuning constants of the reference HAL */

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RSSI_BB_DFLT, 23);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RSSI_DEC_DFLT, 66);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RSSI_CHAN_DFT, 85);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RSSI_BB_ALPHA, 6);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RSSI_DEC_ALPH, 7);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RSSI_CHN_ALPH, 7);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_GAIN_OFFSET,
                   7 | (6 << 4));
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_CORR_TUNE,
                   4 | (7 << 4));
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_PPM_OFFSET, 0x60);

  drift = 4096000000ul / (priv->rf[0].freq_hz >> 1);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_FREQ_DRIFT,
                   drift > 63 ? 63 : (uint8_t)drift);

  drift = 4096000000ul / (priv->rf[0].freq_hz >> 3);
  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_MBWSSF_DRIFT,
                   drift > 63 ? 63 : (uint8_t)drift);

  /* Transmitter constants.  TX_SWAP_IQ is mandatory for LoRaWAN downlinks,
   * the frame synchronisation words are the public network ones.
   */

  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_GAIN, 0x80);
  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_FRAME_SYNC,
                   SX1301_SYNCH_WORD);
  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_START_DLY,
                   SX1301_TX_START_DELAY & 0xff);
  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_START_DLY + 1,
                   (SX1301_TX_START_DELAY >> 8) & 0xff);

  /* Frame synchronisation of the receive path: the multi-SF demodulators and
   * the LoRa standard one.  Both come up configured for a private network.
   */

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_FRAME_SYNCH,
                   SX1301_SYNCH_WORD);
  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_MBWSSF_SYNCH,
                   SX1301_SYNCH_WORD);

  /* Intermediate frequencies and radio mapping of the eight multi-SF
   * demodulators
   */

  priv->radio_select = 0;
  for (i = 0; i < LORA_GW_MULTI_NB; i++)
    {
      if (priv->ifc[i].enable && priv->ifc[i].rf_chain == 1)
        {
          priv->radio_select |= 1 << i;
        }
    }

  for (i = 0; i < LORA_GW_MULTI_NB; i++)
    {
      int32_t ifreg = 0;

      if (priv->ifc[i].enable)
        {
          ifreg = SX1301_IF_HZ_TO_REG(priv->ifc[i].freq_hz);
        }

      sx1301_reg_write13s(priv, SX1301_PAGE_0,
                          SX1301_REG_IF_FREQ_BASE + (i * 2), ifreg);
    }

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RADIO_SELECT,
                   priv->radio_select);

  /* Correlators.  This is what actually turns a channel on. */

  for (i = 0; i < LORA_GW_MULTI_NB; i++)
    {
      sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_CORR_EN_BASE + i,
                       priv->ifc[i].enable ? SX1301_CORR_SF7_SF12 : 0);
    }

  /* LoRa standard demodulator */

  if (priv->std.enable)
    {
      uint8_t bwsel = 0;

      if (priv->std.bandwidth == LORA_GW_BW_250K)
        {
          bwsel = 1;
        }
      else if (priv->std.bandwidth == LORA_GW_BW_500K)
        {
          bwsel = 2;
        }

      sx1301_reg_write13s(priv, SX1301_PAGE_0, SX1301_REG_IF_FREQ_8,
                          SX1301_IF_HZ_TO_REG(priv->std.freq_hz));
      sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_MBWSSF_BW_SEL,
                       bwsel | ((priv->std.rf_chain & 1) << 2));
      sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_MBWSSF_SF,
                       priv->std.datarate);
    }

  /* The modems have to be enabled before the firmwares are loaded */

  modem_en = SX1301_EN_GLOBAL | SX1301_EN_CONCENTRATOR;
  if (priv->std.enable)
    {
      modem_en |= SX1301_EN_MBWSSF;
    }

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_GLOBAL_EN, modem_en);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_CLK_CTRL, 0x03);

  /* Calibration pass, then the production firmwares */

  ret = sx1301_calibrate(priv);
  if (ret < 0)
    {
      return ret;
    }

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_MCU_CTRL,
                   SX1301_MCU_RST_ARB | SX1301_MCU_RST_AGC);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_FORCE_CTRL, 0x0e);

  ret = sx1301_loadfw(priv, 0, g_sx1301_arb_fw);
  if (ret < 0)
    {
      return ret;
    }

  ret = sx1301_loadfw(priv, 1, g_sx1301_agc_fw);
  if (ret < 0)
    {
      return ret;
    }

  version = sx1301_mcu_fwversion(priv, 0);
  wlinfo("Arbiter firmware version %d (expected %d)\n",
         version, SX1301_FW_VERSION_ARB);

  /* Release the host control and let both MCUs run.  The multiplexer bits
   * must stay set or the MCUs lose access to their program memory.
   */

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_FORCE_CTRL, 0);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RADIO_SELECT, 0);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_MCU_CTRL,
                   SX1301_MCU_MUX_ARB | SX1301_MCU_MUX_AGC);

  sx1301_msleep(10);

  for (i = 0; i < 100; i++)
    {
      sx1301_reg_read(priv, SX1301_PAGE_0, SX1301_REG_AGC_STATUS, &status);
      if (status == SX1301_AGC_STATUS_READY)
        {
          break;
        }

      sx1301_msleep(10);
    }

  if (status != SX1301_AGC_STATUS_READY)
    {
      wlerr("ERROR: AGC did not become ready, status 0x%02x\n", status);
      return -EIO;
    }

  sx1301_agc_start(priv);

  /* Re-assert the modem enables, the MCU initialisation may clear them */

  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_GLOBAL_EN, modem_en);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_CLK_CTRL, 0x03);

  memset(&priv->status, 0, sizeof(priv->status));
  priv->started        = true;
  priv->status.started = true;

  wlinfo("Concentrator started, modems 0x%02x\n", modem_en);
  return OK;
}

/****************************************************************************
 * Name: sx1301_stop
 ****************************************************************************/

static int sx1301_stop(FAR struct sx1301_dev_s *priv)
{
  if (!priv->started)
    {
      return OK;
    }

  sx1301_reg_setbit(priv, SX1301_PAGE_0, SX1301_REG_GLOBAL_EN, 3, false);

  if (priv->lower->reset != NULL)
    {
      priv->lower->reset(priv->lower, true);
    }

  priv->started        = false;
  priv->connected      = false;
  priv->status.started = false;

  wlinfo("Concentrator stopped\n");
  return OK;
}

/****************************************************************************
 * Name: sx1301_tscorrection
 *
 * Description:
 *   Delay, in microseconds, between the end of a packet on the air and the
 *   moment the concentrator latches its timestamp.  The network server
 *   derives the downlink window from the value we report, so the correction
 *   of the reference HAL has to be applied or every downlink is late.
 *
 ****************************************************************************/

static uint32_t sx1301_tscorrection(uint8_t bandwidth, uint8_t sf,
                                    uint8_t cr, uint16_t size, bool crc_en,
                                    bool stdchan)
{
  uint32_t delay_x;
  uint32_t delay_y;
  uint32_t delay_z;
  uint32_t bw_pow;
  uint32_t ppm;
  int32_t payload;

  if (sf < 6 || sf > 12)
    {
      return 0;
    }

  ppm = SX1301_PPM_ON(bandwidth, sf) ? 1 : 0;

  if (stdchan)
    {
      switch (bandwidth)
        {
          case LORA_GW_BW_125K:
            delay_x = 64;
            bw_pow  = 1;
            break;

          case LORA_GW_BW_250K:
            delay_x = 32;
            bw_pow  = 2;
            break;

          case LORA_GW_BW_500K:
            delay_x = 16;
            bw_pow  = 4;
            break;

          default:
            return 0;
        }
    }
  else
    {
      delay_x = 114;
      bw_pow  = 1;
    }

  payload = 2 * (size + (crc_en ? 2 : 0));

  if ((payload - (sf - 7)) <= 0)
    {
      /* The payload fits entirely in the first eight symbols */

      delay_y = (((1 << (sf - 1)) * (sf + 1)) + (3 * (1 << (sf - 4)))) /
                bw_pow;
      delay_z = 32 * (payload + 5) / bw_pow;
    }
  else
    {
      delay_y = (((1 << (sf - 1)) * (sf + 1)) +
                 ((4 - ppm) * (1 << (sf - 4)))) / bw_pow;
      delay_z = (16 + 4 * cr) *
                (((payload - sf + 6) % (sf - 2 * ppm)) + 1) / bw_pow;
    }

  return delay_x + delay_y + delay_z;
}

/****************************************************************************
 * Name: sx1301_receive
 *
 * Description:
 *   Drain the receive FIFO into the caller buffer.  Must be called with the
 *   device locked.
 *
 * Returned Value:
 *   Number of packets stored in pkts, or a negated errno value.
 *
 ****************************************************************************/

static int sx1301_receive(FAR struct sx1301_dev_s *priv,
                          FAR struct lora_gw_rxpkt_s *pkts, int maxpkt)
{
  FAR struct lora_gw_rxpkt_s *pkt;
  FAR uint8_t *meta;
  uint8_t buffer[SX1301_RXBUF_SIZE];
  uint8_t hdr[SX1301_FIFO_HDR_SIZE];
  uint8_t status;
  uint8_t sf;
  uint8_t cr;
  uint16_t size;
  size_t total;
  bool crc_en;
  bool stdchan;
  int npkt = 0;
  int ret;

  while (npkt < maxpkt)
    {
      /* Reading the packet count also latches the data buffer pointer, so
       * the five header bytes must be fetched in one burst.
       */

      ret = sx1301_reg_rdburst(priv, SX1301_PAGE_0,
                               SX1301_REG_RX_NUM_STORED, hdr, sizeof(hdr));
      if (ret < 0)
        {
          priv->status.rx_err++;
          return npkt > 0 ? npkt : ret;
        }

      if (hdr[0] == 0)
        {
          break;
        }

      status = hdr[3];
      size   = hdr[4];
      total  = size + SX1301_RX_METADATA_NB;

      if (total > sizeof(buffer))
        {
          total = sizeof(buffer);
        }

      ret = sx1301_reg_rdburst(priv, SX1301_PAGE_0, SX1301_REG_RX_BUF_DATA,
                               buffer, total);
      if (ret < 0)
        {
          priv->status.rx_err++;
          sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RX_NUM_STORED, 0);
          continue;
        }

      /* Pop the packet before anything else can touch the FIFO pointer */

      sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_RX_NUM_STORED, 0);

      meta = &buffer[size];
      pkt  = &pkts[npkt];

      memset(pkt, 0, sizeof(*pkt));

      /* The three low bits of the FIFO status tell CRC pass from CRC fail
       * from no CRC at all.  Anything else is a demodulator hiccup.
       */

      switch (status & 0x07)
        {
          case 5:
            pkt->status = LORA_GW_STAT_CRC_OK;
            crc_en      = true;
            break;

          case 7:
            pkt->status = LORA_GW_STAT_CRC_BAD;
            crc_en      = true;
            priv->status.rx_bad++;
            break;

          case 1:
            pkt->status = LORA_GW_STAT_NO_CRC;
            crc_en      = false;
            priv->status.rx_nocrc++;
            break;

          default:
            pkt->status = LORA_GW_STAT_UNDEFINED;
            crc_en      = false;
            priv->status.rx_err++;
            break;
        }

#ifndef CONFIG_LPWAN_SX1301_RXBADCRC
      /* A gateway has nothing to do with a corrupted packet, and forwarding
       * it as if it were valid pollutes the network server with correlator
       * false triggers.  Drop it unless the debug option asks for it.
       */

      if (pkt->status == LORA_GW_STAT_CRC_BAD ||
          pkt->status == LORA_GW_STAT_UNDEFINED)
        {
          continue;
        }
#endif

#ifndef CONFIG_LPWAN_SX1301_RXNOCRC
      /* Same for a packet that carried no CRC at all: a LoRaWAN uplink
       * always has one, so what shows up here is either a false trigger of
       * the correlator or the downlink of another gateway.  The reference
       * packet forwarder does not forward those either.
       */

      if (pkt->status == LORA_GW_STAT_NO_CRC)
        {
          continue;
        }
#endif

      pkt->if_chain   = meta[0];
      pkt->modulation = LORA_GW_MOD_LORA;
      pkt->size       = size;

      sf = (meta[1] >> 4) & 0x0f;
      cr = (meta[1] >> 1) & 0x07;

      if (sf < 6 || sf > 12)
        {
          sf = 7;
        }

      if (cr < 1 || cr > 4)
        {
          cr = WLIOC_LORA_CR_4_5;
        }

      pkt->coderate = cr;

      /* Frequency, bandwidth and, for the standard channel, the spreading
       * factor all come from the channel plan rather than the metadata.
       */

      stdchan = (pkt->if_chain == LORA_GW_MULTI_NB);

      if (stdchan && priv->std.enable)
        {
          pkt->rf_chain  = priv->std.rf_chain;
          pkt->bandwidth = priv->std.bandwidth;
          pkt->datarate  = priv->std.datarate;
          pkt->freq_hz   = priv->rf[priv->std.rf_chain].freq_hz +
                           priv->std.freq_hz;
        }
      else if (pkt->if_chain < LORA_GW_MULTI_NB)
        {
          pkt->rf_chain  = priv->ifc[pkt->if_chain].rf_chain;
          pkt->bandwidth = LORA_GW_BW_125K;
          pkt->datarate  = sf;
          pkt->freq_hz   = priv->rf[pkt->rf_chain].freq_hz +
                           priv->ifc[pkt->if_chain].freq_hz;
        }
      else
        {
          /* IF9 is the FSK demodulator, which this driver does not use */

          wlwarn("WARNING: packet from unexpected chain %d\n",
                 pkt->if_chain);
          priv->status.rx_err++;
          continue;
        }

      pkt->rssi_dbm10 = (int16_t)(meta[5] * 10) +
                        priv->rf[pkt->rf_chain].rssi_offset_dbm10;
      pkt->snr_db10   = ((int16_t)((int8_t)meta[2]) * 10) / 4;

      pkt->count_us = (uint32_t)meta[6] |
                      ((uint32_t)meta[7] << 8) |
                      ((uint32_t)meta[8] << 16) |
                      ((uint32_t)meta[9] << 24);

      pkt->count_us -= sx1301_tscorrection(pkt->bandwidth, pkt->datarate,
                                           pkt->coderate, size, crc_en,
                                           stdchan);

      memcpy(pkt->payload, buffer, size);

      if (pkt->status == LORA_GW_STAT_CRC_OK)
        {
          priv->status.rx_ok++;
        }

      wlinfo("RX chain %d SF%d %" PRIu32 " Hz rssi %d.%d dBm snr %d.%d dB "
             "size %d status 0x%02x\n",
             pkt->if_chain, pkt->datarate, pkt->freq_hz,
             pkt->rssi_dbm10 / 10, abs(pkt->rssi_dbm10 % 10),
             pkt->snr_db10 / 10, abs(pkt->snr_db10 % 10),
             pkt->size, pkt->status);

      npkt++;
    }

  return npkt;
}

/****************************************************************************
 * Name: sx1301_send
 *
 * Description:
 *   Queue one packet for transmission.  Must be called with the device
 *   locked.  The transmission itself happens asynchronously, either right
 *   away or at the requested concentrator timestamp, so this does not wait
 *   for completion.
 *
 ****************************************************************************/

static int sx1301_send(FAR struct sx1301_dev_s *priv,
                       FAR const struct lora_gw_txpkt_s *pkt)
{
  uint8_t buffer[SX1301_RX_METADATA_NB + LORA_GW_MAX_PAYLOAD];
  uint32_t part_int;
  uint32_t part_frac;
  uint32_t timestamp;
  uint16_t start_delay;
  uint16_t preamble;
  uint8_t powidx;
  uint8_t status = 0;
  int i;

  if (pkt->size > LORA_GW_MAX_PAYLOAD ||
      pkt->rf_chain >= LORA_GW_RF_CHAIN_NB)
    {
      return -EINVAL;
    }

  if (!priv->rf[pkt->rf_chain].tx_enable)
    {
      wlerr("ERROR: radio %d cannot transmit\n", pkt->rf_chain);
      return -EPERM;
    }

  memset(buffer, 0, SX1301_RX_METADATA_NB);

  /* Radio frequency, as an integer part in units of 4 MHz plus a fraction */

  part_int  = pkt->freq_hz / 4000000;
  part_frac = ((pkt->freq_hz % 4000000) << 8) / 15625;

  buffer[0] = part_int & 0xff;
  if (pkt->bandwidth == LORA_GW_BW_500K)
    {
      buffer[0] |= 0x80;
    }

  buffer[1] = (part_frac >> 8) & 0xff;
  buffer[2] = part_frac & 0xff;

  /* The transmit chain starts start_delay microseconds before the packet
   * appears on the air, so the requested time is moved back by that much.
   */

  start_delay = (pkt->bandwidth == LORA_GW_BW_125K) ?
                SX1301_TX_START_DLY_125 : SX1301_TX_START_DELAY;

  timestamp = pkt->count_us - start_delay;
  buffer[3] = (timestamp >> 24) & 0xff;
  buffer[4] = (timestamp >> 16) & 0xff;
  buffer[5] = (timestamp >> 8) & 0xff;
  buffer[6] = timestamp & 0xff;

  /* Radio chain, modulation and index in the transmit gain table.  Pick the
   * first table entry that reaches the requested power.
   */

  powidx = SX1301_TX_LUT_SIZE - 1;
  for (i = 0; i < SX1301_TX_LUT_SIZE; i++)
    {
      if (pkt->rf_power <= g_sx1301_tx_pwr[i])
        {
          powidx = i;
          break;
        }
    }

  buffer[7] = ((pkt->rf_chain & 0x01) << 5) | (powidx & 0x0f);
  buffer[8] = 0;

  buffer[9] = pkt->no_crc ? 0 : 0x80;
  buffer[9] |= (pkt->coderate & 0x07) << 4;
  buffer[9] |= pkt->datarate & 0x0f;

  buffer[10] = pkt->size;

  buffer[11] = 0;
  if (pkt->invert_pol)
    {
      buffer[11] |= 0x10;
    }

  if (SX1301_PPM_ON(pkt->bandwidth, pkt->datarate))
    {
      buffer[11] |= 0x08;
    }

  if (pkt->no_header)
    {
      buffer[11] |= 0x04;
    }

  if (pkt->bandwidth == LORA_GW_BW_250K)
    {
      buffer[11] |= 0x01;
    }
  else if (pkt->bandwidth == LORA_GW_BW_500K)
    {
      buffer[11] |= 0x02;
    }

  preamble   = (pkt->preamble != 0) ? pkt->preamble : 8;
  buffer[12] = (preamble >> 8) & 0xff;
  buffer[13] = preamble & 0xff;

  memcpy(&buffer[SX1301_RX_METADATA_NB], pkt->payload, pkt->size);

  /* No transmit IQ calibration values are kept, and the digital gain stays
   * at zero while TX_SWAP_IQ has to be preserved.
   */

  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_OFFSET_I, 0);
  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_OFFSET_Q, 0);
  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_GAIN, 0x80);

  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_START_DLY,
                   start_delay & 0xff);
  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_START_DLY + 1,
                   (start_delay >> 8) & 0xff);

  /* Abort anything pending, load the buffer and trigger */

  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_TRIG, 0);
  sx1301_reg_write(priv, SX1301_PAGE_0, SX1301_REG_TX_BUF_ADDR, 0);
  sx1301_reg_wrburst(priv, SX1301_PAGE_0, SX1301_REG_TX_BUF_DATA, buffer,
                     SX1301_RX_METADATA_NB + pkt->size);

  sx1301_reg_write(priv, SX1301_PAGE_1, SX1301_REG_TX_TRIG,
                   pkt->tx_mode == LORA_GW_TX_IMMEDIATE ? 0x01 : 0x02);

  sx1301_reg_read(priv, SX1301_PAGE_1, SX1301_REG_TX_STATUS, &status);

  priv->status.tx_ok++;

  wlinfo("TX %s %" PRIu32 " Hz SF%d %d bytes, gain %d, status 0x%02x\n",
         pkt->tx_mode == LORA_GW_TX_IMMEDIATE ? "now" : "scheduled",
         pkt->freq_hz, pkt->datarate, pkt->size, powidx, status);

  return OK;
}

/****************************************************************************
 * Name: sx1301_open
 ****************************************************************************/

static int sx1301_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sx1301_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->crefs++;
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: sx1301_close
 ****************************************************************************/

static int sx1301_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sx1301_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->crefs > 0)
    {
      priv->crefs--;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: sx1301_read
 ****************************************************************************/

static ssize_t sx1301_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sx1301_dev_s *priv = inode->i_private;
  FAR struct lora_gw_rxpkt_s *pkts;
  int maxpkt;
  int npkt;
  int ret;

  maxpkt = buflen / sizeof(struct lora_gw_rxpkt_s);
  if (maxpkt < 1)
    {
      return -EINVAL;
    }

  pkts = (FAR struct lora_gw_rxpkt_s *)buffer;

  for (; ; )
    {
      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      if (!priv->started)
        {
          nxmutex_unlock(&priv->lock);
          return -ENXIO;
        }

      npkt = sx1301_receive(priv, pkts, maxpkt);
      nxmutex_unlock(&priv->lock);

      if (npkt < 0)
        {
          return npkt;
        }

      if (npkt > 0)
        {
          return npkt * sizeof(struct lora_gw_rxpkt_s);
        }

      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          return -EAGAIN;
        }

      /* There is no interrupt line on the reference shields, so a blocking
       * reader polls the FIFO.
       */

      ret = nxsig_usleep(SX1301_RXPOLL_USEC);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: sx1301_write
 ****************************************************************************/

static ssize_t sx1301_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sx1301_dev_s *priv = inode->i_private;
  FAR const struct lora_gw_txpkt_s *pkt;
  int ret;

  if (buflen != sizeof(struct lora_gw_txpkt_s))
    {
      return -EINVAL;
    }

  pkt = (FAR const struct lora_gw_txpkt_s *)buffer;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->started)
    {
      nxmutex_unlock(&priv->lock);
      return -ENXIO;
    }

  ret = sx1301_send(priv, pkt);
  if (ret < 0)
    {
      priv->status.tx_err++;
    }

  nxmutex_unlock(&priv->lock);

  return ret < 0 ? ret : (ssize_t)buflen;
}

/****************************************************************************
 * Name: sx1301_ioctl
 ****************************************************************************/

static int sx1301_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sx1301_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case WLIOC_GW_START:
        {
          ret = sx1301_start(priv);
        }
        break;

      case WLIOC_GW_STOP:
        {
          ret = sx1301_stop(priv);
        }
        break;

      case WLIOC_GW_RESET:
        {
          sx1301_stop(priv);
          ret = sx1301_start(priv);
        }
        break;

      case WLIOC_GW_SETREGION:
        {
          FAR const char *name = (FAR const char *)((uintptr_t)arg);

          if (name == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = sx1301_region_byname(name);
              if (ret >= 0)
                {
                  ret = sx1301_region_apply(priv, ret);
                }
            }
        }
        break;

      case WLIOC_GW_GETREGION:
        {
          FAR struct lora_gw_regionreq_s *req =
            (FAR struct lora_gw_regionreq_s *)((uintptr_t)arg);

          if (req == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = sx1301_region_getinfo(req->index < 0 ? priv->region :
                                          req->index, &req->info);
            }
        }
        break;

      case WLIOC_GW_GETSTATUS:
        {
          FAR struct lora_gw_status_s *status =
            (FAR struct lora_gw_status_s *)((uintptr_t)arg);

          if (status == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              memcpy(status, &priv->status, sizeof(*status));
              ret = OK;
            }
        }
        break;

      case WLIOC_GW_GETTRIGCNT:
        {
          FAR uint32_t *count = (FAR uint32_t *)((uintptr_t)arg);
          uint8_t buffer[4];

          if (count == NULL)
            {
              ret = -EINVAL;
            }
          else if (!priv->started)
            {
              ret = -ENXIO;
            }
          else
            {
              ret = sx1301_reg_rdburst(priv, SX1301_PAGE_0,
                                      SX1301_REG_TIMESTAMP, buffer,
                                      sizeof(buffer));
              if (ret >= 0)
                {
                  *count = (uint32_t)buffer[0] |
                           ((uint32_t)buffer[1] << 8) |
                           ((uint32_t)buffer[2] << 16) |
                           ((uint32_t)buffer[3] << 24);
                }
            }
        }
        break;

      default:
        {
          ret = -ENOTTY;
        }
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx1301_register
 ****************************************************************************/

int sx1301_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                    FAR const struct sx1301_lower_s *lower)
{
  FAR struct sx1301_dev_s *priv;
  int region;
  int ret;

  DEBUGASSERT(devpath != NULL && spi != NULL && lower != NULL);
  DEBUGASSERT(lower->reset != NULL);

  priv = kmm_zalloc(sizeof(struct sx1301_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->spi   = spi;
  priv->lower = lower;

  nxmutex_init(&priv->lock);

  /* Select the compiled in default region */

  region = sx1301_region_byname(CONFIG_LPWAN_SX1301_DEFAULT_REGION);
  if (region < 0)
    {
      wlwarn("WARNING: unknown default region '%s', using the first one\n",
             CONFIG_LPWAN_SX1301_DEFAULT_REGION);
      region = 0;
    }

  ret = sx1301_region_apply(priv, region);
  if (ret < 0)
    {
      goto errout;
    }

  /* Hold the chip in reset until it is started */

  lower->reset(lower, true);

  ret = register_driver(devpath, &g_sx1301_fops, 0666, priv);
  if (ret < 0)
    {
      wlerr("ERROR: failed to register %s: %d\n", devpath, ret);
      goto errout;
    }

  wlinfo("SX1301 registered at %s\n", devpath);
  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
