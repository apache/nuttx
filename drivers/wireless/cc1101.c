/****************************************************************************
 * drivers/wireless/cc1101.c
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

/* Features:
 *   - Maximum data length: 61 bytes CC1101_PACKET_MAXDATALEN
 *   - Packet length includes two additional bytes: CC1101_PACKET_MAXTOTALLEN
 *   - Requires one GDO to trigger end-of-packets in RX and TX modes.
 *   - Variable packet length with data payload between 1..61 bytes
 *     (three bytes are reserved for packet length, and RSSI and LQI
 *      appended at the end of RXFIFO after each reception)
 *   - Support for General Digital Outputs with overload protection
 *     (single XOSC pin is allowed, otherwise error is returned)
 *   - Loadable RF settings, one for ISM Region 1 (Europe) and one for
 *     ISM Region 2 (Complete America)
 *
 * Todo:
 *   - Extend max packet length up to 255 bytes or rather
 *     infinite < 4096 bytes
 *   - Power up/down modes
 *   - Sequencing between states or add protection for correct termination of
 *     various different state (so that CC1101 does not block in case of
 *     improper use)
 *
 * RSSI and LQI value interpretation
 *
 * The LQI can be read from the LQI status register or it can be appended
 * to the received packet in the RX FIFO. LQI is a metric of the current
 * quality of the received signal. The LQI gives an estimate of how easily
 * a received signal can be demodulated by accumulating the magnitude of
 * the error between ideal constellations and the received signal over
 * the 64 symbols immediately following the sync word. LQI is best used
 * as a relative measurement of the link quality (a high value indicates
 * a better link than what a low value does), since the value is dependent
 * on the modulation format.
 *
 * To simplify: If the received modulation is FSK or GFSK, the receiver
 * will measure the frequency of each "bit" and compare it with the
 * expected frequency based on the channel frequency and the deviation
 * and the measured frequency offset. If other modulations are used, the
 * error of the modulated parameter (frequency for FSK/GFSK, phase for
 * MSK, amplitude for ASK etc) will be measured against the expected
 * ideal value
 *
 * RSSI (Received Signal Strength Indicator) is a signal strength
 * indication. It does not care about the "quality" or "correctness" of
 * the signal. LQI does not care about the actual signal strength, but
 * the signal quality often is linked to signal strength. This is because
 * a strong signal is likely to be less affected by noise and thus will
 * be seen as "cleaner" or more "correct" by the receiver.
 *
 * There are four to five "extreme cases" that can be used to illustrate
 * how RSSI and LQI work:
 *
 *  1. A weak signal in the presence of noise may give low RSSI and low LQI.
 *  2. A weak signal in "total" absence of noise may give low RSSI and high
 *     LQI.
 *  3. Strong noise (usually coming from an interferer) may give high RSSI
 *     and low LQI.
 *  4. A strong signal without much noise may give high RSSI and high LQI.
 *  5. A very strong signal that causes the receiver to saturate may give
 *     high RSSI and low LQI.
 *
 * Note that both RSSI and LQI are best used as relative measurements since
 * the values are dependent on the modulation format.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wireless/cc1101.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CC1101_SPIFREQ_BURST    6500000 /* Hz, no delay */
#define CC1101_SPIFREQ_SINGLE   9000000 /* Hz, single access only - no delay */

#define CC1101_MCSM0_VALUE      0x1c

/****************************************************************************
 * Chipcon CC1101 Internal Registers
 ****************************************************************************/

/* Configuration Registers */

#define CC1101_IOCFG2           0x00        /* GDO2 output pin configuration */
#define CC1101_IOCFG1           0x01        /* GDO1 output pin configuration */
#define CC1101_IOCFG0           0x02        /* GDO0 output pin configuration */
#define CC1101_FIFOTHR          0x03        /* RX FIFO and TX FIFO thresholds */
#define CC1101_SYNC1            0x04        /* Sync word, high byte */
#define CC1101_SYNC0            0x05        /* Sync word, low byte */
#define CC1101_PKTLEN           0x06        /* Packet length */
#define CC1101_PKTCTRL1         0x07        /* Packet automation control */
#define CC1101_PKTCTRL0         0x08        /* Packet automation control */
#define CC1101_ADDR             0x09        /* Device address */
#define CC1101_CHANNR           0x0a        /* Channel number */
#define CC1101_FSCTRL1          0x0b        /* Frequency synthesizer control */
#define CC1101_FSCTRL0          0x0c        /* Frequency synthesizer control */
#define CC1101_FREQ2            0x0d        /* Frequency control word, high byte */
#define CC1101_FREQ1            0x0e        /* Frequency control word, middle byte */
#define CC1101_FREQ0            0x0f        /* Frequency control word, low byte */
#define CC1101_MDMCFG4          0x10        /* Modem configuration */
#define CC1101_MDMCFG3          0x11        /* Modem configuration */
#define CC1101_MDMCFG2          0x12        /* Modem configuration */
#define CC1101_MDMCFG1          0x13        /* Modem configuration */
#define CC1101_MDMCFG0          0x14        /* Modem configuration */
#define CC1101_DEVIATN          0x15        /* Modem deviation setting */
#define CC1101_MCSM2            0x16        /* Main Radio Cntrl State Machine config */
#define CC1101_MCSM1            0x17        /* Main Radio Cntrl State Machine config */
#define CC1101_MCSM0            0x18        /* Main Radio Cntrl State Machine config */
#define CC1101_FOCCFG           0x19        /* Frequency Offset Compensation config */
#define CC1101_BSCFG            0x1a        /* Bit Synchronization configuration */
#define CC1101_AGCCTRL2         0x1b        /* AGC control */
#define CC1101_AGCCTRL1         0x1c        /* AGC control */
#define CC1101_AGCCTRL0         0x1d        /* AGC control */
#define CC1101_WOREVT1          0x1e        /* High byte Event 0 timeout */
#define CC1101_WOREVT0          0x1f        /* Low byte Event 0 timeout */
#define CC1101_WORCTRL          0x20        /* Wake On Radio control */
#define CC1101_FREND1           0x21        /* Front end RX configuration */
#define CC1101_FREND0           0x22        /* Front end TX configuration */
#define CC1101_FSCAL3           0x23        /* Frequency synthesizer calibration */
#define CC1101_FSCAL2           0x24        /* Frequency synthesizer calibration */
#define CC1101_FSCAL1           0x25        /* Frequency synthesizer calibration */
#define CC1101_FSCAL0           0x26        /* Frequency synthesizer calibration */
#define CC1101_RCCTRL1          0x27        /* RC oscillator configuration */
#define CC1101_RCCTRL0          0x28        /* RC oscillator configuration */
#define CC1101_FSTEST           0x29        /* Frequency synthesizer cal control */
#define CC1101_PTEST            0x2a        /* Production test */
#define CC1101_AGCTEST          0x2b        /* AGC test */
#define CC1101_TEST2            0x2c        /* Various test settings */
#define CC1101_TEST1            0x2d        /* Various test settings */
#define CC1101_TEST0            0x2e        /* Various test settings */

/* Status registers */

#define CC1101_PARTNUM          (0x30 | 0xc0)   /* Part number */
#define CC1101_VERSION          (0x31 | 0xc0)   /* Current version number */
#define CC1101_FREQEST          (0x32 | 0xc0)   /* Frequency offset estimate */
#define CC1101_LQI              (0x33 | 0xc0)   /* Demodulator estimate for link quality */
#define CC1101_RSSI             (0x34 | 0xc0)   /* Received signal strength indication */
#define CC1101_MARCSTATE        (0x35 | 0xc0)   /* Control state machine state */
#define CC1101_WORTIME1         (0x36 | 0xc0)   /* High byte of WOR timer */
#define CC1101_WORTIME0         (0x37 | 0xc0)   /* Low byte of WOR timer */
#define CC1101_PKTSTATUS        (0x38 | 0xc0)   /* Current GDOx status and packet status */
#define CC1101_VCO_VC_DAC       (0x39 | 0xc0)   /* Current setting from PLL cal module */
#define CC1101_TXBYTES          (0x3a | 0xc0)   /* Underflow and # of bytes in TXFIFO */
#define CC1101_RXBYTES          (0x3b | 0xc0)   /* Overflow and # of bytes in RXFIFO */
#define CC1101_RCCTRL1_STATUS   (0x3c | 0xc0)   /* Last RC oscillator calibration results */
#define CC1101_RCCTRL0_STATUS   (0x3d | 0xc0)   /* Last RC oscillator calibration results */

/* Multi byte memory locations */

#define CC1101_PATABLE          0x3e
#define CC1101_TXFIFO           0x3f
#define CC1101_RXFIFO           0x3f

/* Definitions for burst/single access to registers */

#define CC1101_WRITE_BURST      0x40
#define CC1101_READ_SINGLE      0x80
#define CC1101_READ_BURST       0xc0

/* Strobe commands */

#define CC1101_SRES             0x30        /* Reset chip. */
#define CC1101_SFSTXON          0x31        /* Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). */
#define CC1101_SXOFF            0x32        /* Turn off crystal oscillator. */
#define CC1101_SCAL             0x33        /* Calibrate frequency synthesizer and turn it off */
#define CC1101_SRX              0x34        /* Enable RX. Perform calibration first if switching from IDLE and MCSM0.FS_AUTOCAL=1. */
#define CC1101_STX              0x35        /* Enable TX. Perform calibration first if IDLE and MCSM0.FS_AUTOCAL=1.  */
                                            /* If switching from RX state and CCA is enabled then go directly to TX if channel is clear. */
#define CC1101_SIDLE            0x36        /* Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable. */
#define CC1101_SAFC             0x37        /* Perform AFC adjustment of the frequency synthesizer */
#define CC1101_SWOR             0x38        /* Start automatic RX polling sequence (Wake-on-Radio) */
#define CC1101_SPWD             0x39        /* Enter power down mode when CSn goes high. */
#define CC1101_SFRX             0x3a        /* Flush the RX FIFO buffer. */
#define CC1101_SFTX             0x3b        /* Flush the TX FIFO buffer. */
#define CC1101_SWORRST          0x3c        /* Reset real time clock. */
#define CC1101_SNOP             0x3d        /* No operation. */

/* Modem Control */

#define CC1101_MCSM0_XOSC_FORCE_ON  0x01

/* Chip Status Byte */

/* Bit fields in the chip status byte */

#define CC1101_STATUS_CHIP_RDYn_BM              0x80
#define CC1101_STATUS_STATE_BM                  0x70
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM   0x0f

/* Chip states */

#define CC1101_STATE_MASK                       0x70
#define CC1101_STATE_IDLE                       0x00
#define CC1101_STATE_RX                         0x10
#define CC1101_STATE_TX                         0x20
#define CC1101_STATE_FSTXON                     0x30
#define CC1101_STATE_CALIBRATE                  0x40
#define CC1101_STATE_SETTLING                   0x50
#define CC1101_STATE_RX_OVERFLOW                0x60
#define CC1101_STATE_TX_UNDERFLOW               0x70

/* Values of the MACRSTATE register */

#define CC1101_MARCSTATE_SLEEP                  0x00
#define CC1101_MARCSTATE_IDLE                   0x01
#define CC1101_MARCSTATE_XOFF                   0x02
#define CC1101_MARCSTATE_VCOON_MC               0x03
#define CC1101_MARCSTATE_REGON_MC               0x04
#define CC1101_MARCSTATE_MANCAL                 0x05
#define CC1101_MARCSTATE_VCOON                  0x06
#define CC1101_MARCSTATE_REGON                  0x07
#define CC1101_MARCSTATE_STARTCAL               0x08
#define CC1101_MARCSTATE_BWBOOST                0x09
#define CC1101_MARCSTATE_FS_LOCK                0x0a
#define CC1101_MARCSTATE_IFADCON                0x0b
#define CC1101_MARCSTATE_ENDCAL                 0x0c
#define CC1101_MARCSTATE_RX                     0x0d
#define CC1101_MARCSTATE_RX_END                 0x0e
#define CC1101_MARCSTATE_RX_RST                 0x0f
#define CC1101_MARCSTATE_TXRX_SWITCH            0x10
#define CC1101_MARCSTATE_RXFIFO_OVERFLOW        0x11
#define CC1101_MARCSTATE_FSTXON                 0x12
#define CC1101_MARCSTATE_TX                     0x13
#define CC1101_MARCSTATE_TX_END                 0x14
#define CC1101_MARCSTATE_RXTX_SWITCH            0x15
#define CC1101_MARCSTATE_TXFIFO_UNDERFLOW       0x16

/* Part number and version */

#define CC1101_PARTNUM_VALUE                    0x00
#define CC1101_VERSION_VALUE                    0x14

/*  Others ... */

#define CC1101_LQI_CRC_OK_BM                    0x80
#define CC1101_LQI_EST_BM                       0x7f

#define FLAGS_RXONLY                            1 /* Indicates receive operation only */
#define FLAGS_XOSCENABLED                       2 /* Indicates that one pin is configured as XOSC/n */

#ifndef CONFIG_WL_CC1101_RXFIFO_LEN
#  define CONFIG_WL_CC1101_RXFIFO_LEN           5
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cc1101_file_open(FAR struct file *filep);
static int cc1101_file_close(FAR struct file *filep);
static ssize_t cc1101_file_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t cc1101_file_write(FAR struct file *filep,
                                 FAR const char *buffer,
                                 size_t buflen);
static int cc1101_file_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cc1101ops =
{
  cc1101_file_open,  /* open */
  cc1101_file_close, /* close */
  cc1101_file_read,  /* read */
  cc1101_file_write, /* write */
  NULL,              /* seek */
  NULL,              /* ioctl */
  cc1101_file_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL             /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc1101_file_open
 *
 * Description:
 *   This function is called whenever the CC1101 device is opened.
 *
 ****************************************************************************/

static int cc1101_file_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct cc1101_dev_s *dev;
  int ret;

  wlinfo("Opening CC1101 dev\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct cc1101_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if device is not already used */

  if (dev->nopens > 0)
    {
      ret = -EBUSY;
      goto errout;
    }

  dev->ops.irq(dev, true);
  cc1101_receive(dev);
  dev->nopens++;

errout:
  nxmutex_unlock(&dev->devlock);
  return ret;
}

/****************************************************************************
 * Name: cc1101_file_close
 *
 * Description:
 *   This routine is called when the CC1101 device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int cc1101_file_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct cc1101_dev_s *dev;
  int ret;

  wlinfo("Closing CC1101 dev\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct cc1101_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  dev->ops.irq(dev, false);
#if 0
  nrf24l01_changestate(dev, ST_POWER_DOWN);
#endif
  dev->nopens--;

  nxmutex_unlock(&dev->devlock);
  return OK;
}

/****************************************************************************
 * Name: cc1101_file_write
 *
 * Description:
 *   Standard driver write method.
 *
 ****************************************************************************/

static ssize_t cc1101_file_write(FAR struct file *filep,
                                 FAR const char *buffer,
                                 size_t buflen)
{
  FAR struct inode *inode;
  FAR struct cc1101_dev_s *dev;
  int ret;

  wlinfo("write CC1101 dev\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct cc1101_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = cc1101_write(dev, (const uint8_t *)buffer, buflen);
  cc1101_send(dev);
  nxmutex_unlock(&dev->devlock);
  return ret;
}

/****************************************************************************
 * Name: fifo_put
 *
 * Description:
 *
 ****************************************************************************/

static void fifo_put(FAR struct cc1101_dev_s *dev, FAR uint8_t *buffer,
                     uint8_t buflen)
{
  int ret;
  int i;

  ret = nxmutex_lock(&dev->lock_rx_buffer);
  if (ret < 0)
    {
      return;
    }

  dev->fifo_len++;
  if (dev->fifo_len > CONFIG_WL_CC1101_RXFIFO_LEN)
    {
      dev->fifo_len = CONFIG_WL_CC1101_RXFIFO_LEN;
      dev->nxt_read = (dev->nxt_read + 1) % CONFIG_WL_CC1101_RXFIFO_LEN;
    }

  for (i = 0; i < (buflen + 1) && i < CC1101_FIFO_SIZE; i++)
    {
      *(dev->rx_buffer + i + dev->nxt_write * CC1101_FIFO_SIZE) = buffer[i];
    }

  dev->nxt_write = (dev->nxt_write + 1) % CONFIG_WL_CC1101_RXFIFO_LEN;
  nxmutex_unlock(&dev->lock_rx_buffer);
}

/****************************************************************************
 * Name: fifo_get
 *
 * Description:
 *
 ****************************************************************************/

static uint8_t fifo_get(FAR struct cc1101_dev_s *dev, FAR uint8_t *buffer,
                        uint8_t buflen)
{
  uint8_t pktlen;
  uint8_t i;
  int ret;

  ret = nxmutex_lock(&dev->lock_rx_buffer);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->fifo_len == 0)
    {
      pktlen = 0;
      goto no_data;
    }

  pktlen = *(dev->rx_buffer + dev->nxt_read * CC1101_FIFO_SIZE);

  for (i = 0; i < pktlen && i < CC1101_PACKET_MAXTOTALLEN; i++)
    {
      *(buffer++) =
          dev->rx_buffer[dev->nxt_read * CC1101_FIFO_SIZE + i + 1];
    }

  dev->nxt_read = (dev->nxt_read + 1) % CONFIG_WL_CC1101_RXFIFO_LEN;
  dev->fifo_len--;

no_data:
  nxmutex_unlock(&dev->lock_rx_buffer);
  return pktlen;
}

/****************************************************************************
 * Name: cc1101_file_read
 *
 * Description:
 *   Standard driver read method
 *
 ****************************************************************************/

static ssize_t cc1101_file_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct cc1101_dev_s *dev;
  FAR struct inode *inode;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct cc1101_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if ((filep->f_oflags & O_NONBLOCK) != 0)
    {
      nxsem_trywait(&dev->sem_rx);
      ret = 0;
    }
  else
    {
      ret = nxsem_wait(&dev->sem_rx);
    }

  if (ret < 0)
    {
      return ret;
    }

  buflen = fifo_get(dev, (uint8_t *)buffer, buflen);
  nxmutex_unlock(&dev->devlock);
  return buflen;
}

/****************************************************************************
 * Name: nrf24l01_poll
 *
 * Description:
 *   Standard driver poll method.
 *
 ****************************************************************************/

static int cc1101_file_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup)
{
  FAR struct inode *inode;
  FAR struct cc1101_dev_s *dev;
  int ret;

  wlinfo("setup: %d\n", (int)setup);

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct cc1101_dev_s *)inode->i_private;

  /* Exclusive access */

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* Check if we can accept this poll.
       * For now, only one thread can poll the device at any time
       * (shorter / simpler code)
       */

      if (dev->pfd)
        {
          ret = -EBUSY;
          goto errout;
        }

      dev->pfd = fds;

      /* Is there is already data in the fifo? then trigger POLLIN now -
       * don't wait for RX.
       */

      nxmutex_lock(&dev->lock_rx_buffer);
      if (dev->fifo_len > 0)
        {
          poll_notify(&dev->pfd, 1, POLLIN);
        }

      nxmutex_unlock(&dev->lock_rx_buffer);
    }
  else /* Tear it down */
    {
      dev->pfd = NULL;
    }

errout:
  nxmutex_unlock(&dev->devlock);
  return ret;
}

/****************************************************************************
 * Name: cc1101_access_begin
 *
 * Description:
 *
 ****************************************************************************/

void cc1101_access_begin(FAR struct cc1101_dev_s *dev)
{
  SPI_LOCK(dev->spi, true);
  SPI_SELECT(dev->spi, dev->dev_id, true);
  SPI_SETMODE(dev->spi, SPIDEV_MODE0); /* CPOL=0, CPHA=0 */
  SPI_SETBITS(dev->spi, 8);
  SPI_HWFEATURES(dev->spi, 0);

  if (dev->ops.wait)
    {
      dev->ops.wait(dev, dev->miso_pin);
    }
  else
    {
      nxsig_usleep(150 * 1000);
    }
}

/****************************************************************************
 * Name: cc1101_access_end
 *
 * Description:
 *
 ****************************************************************************/

void cc1101_access_end(FAR struct cc1101_dev_s *dev)
{
  SPI_SELECT(dev->spi, dev->dev_id, false);
  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: cc1101_access
 *
 * Description:
 *   CC1101 Access with Range Check
 *
 * Input Parameters:
 *   dev    - CC1101 Private Structure
 *   addr   - CC1101 Address
 *   buf    - Pointer to buffer, either for read or write access
 *   length - When >0 it denotes read access, when <0 it denotes write
 *            access of -length. abs(length) greater of 1 implies burst mode,
 *            however
 *
 * Returned Value:
 *   OK on success or a negated errno value on any failure.
 *
 ****************************************************************************/

int cc1101_access(FAR struct cc1101_dev_s *dev, uint8_t addr,
                  FAR uint8_t *buf, int length)
{
  int stabyte;

  /* Address cannot explicitly define READ command while length WRITE.
   * Also access to these cells is only permitted as one byte, even though
   * transfer is marked as BURST!
   */

  if ((addr & CC1101_READ_SINGLE) && length != 1)
    {
      return -EINVAL;
    }

  /* Prepare SPI */

  cc1101_access_begin(dev);

  if (length > 1 || length < -1)
    {
      SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_BURST);
    }
  else
    {
      SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_SINGLE);
    }

  /* Transfer */

  if (length <= 0)
    {
      /* 0 length are command strobes */

      if (length < -1)
        {
          addr |= CC1101_WRITE_BURST;
        }

      stabyte = SPI_SEND(dev->spi, addr);
      if (length)
        {
          SPI_SNDBLOCK(dev->spi, buf, -length);
        }
    }
  else
    {
      addr |= CC1101_READ_SINGLE;
      if (length > 1)
        {
          addr |= CC1101_READ_BURST;
        }

      stabyte = SPI_SEND(dev->spi, addr);
      SPI_RECVBLOCK(dev->spi, buf, length);
    }

  cc1101_access_end(dev);
  return stabyte;
}

/****************************************************************************
 * Name: cc1101_strobe
 *
 * Description:
 *    Strobes command and returns chip status byte
 *
 *    By default commands are send as Write. To a command,
 *    CC1101_READ_SINGLE may be OR'ed to obtain the number of RX bytes
 *    pending in RX FIFO.
 *
 ****************************************************************************/

uint8_t cc1101_strobe(struct cc1101_dev_s *dev, uint8_t command)
{
  uint8_t status;

  cc1101_access_begin(dev);
  SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_SINGLE);

  status = SPI_SEND(dev->spi, command);

  cc1101_access_end(dev);

  return status;
}

/****************************************************************************
 * Name: cc1101_reset
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_reset(struct cc1101_dev_s *dev)
{
  cc1101_strobe(dev, CC1101_SRES);
  return OK;
}

/****************************************************************************
 * Name: cc1101_checkpart
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_checkpart(struct cc1101_dev_s *dev)
{
  uint8_t partnum;
  uint8_t version;

  if (cc1101_access(dev, CC1101_PARTNUM, &partnum, 1) < 0 ||
      cc1101_access(dev, CC1101_VERSION, &version, 1) < 0)
    {
      return -ENODEV;
    }

  wlinfo("CC1101 cc1101_checkpart 0x%X 0x%X\n", partnum, version);

  if (partnum == CC1101_PARTNUM_VALUE && version == CC1101_VERSION_VALUE)
    {
      return OK;
    }

  return -ENOTSUP;
}

/****************************************************************************
 * Name: cc1101_dumpregs
 *
 * Description:
 *   Dump the specified range of registers to the syslog.
 *
 *   WARNING:  Uses around 75 bytes of stack!
 *
 ****************************************************************************/

void cc1101_dumpregs(struct cc1101_dev_s *dev, uint8_t addr, uint8_t length)
{
  char outbuf[3 * 16 + 1];
  uint8_t regbuf[16];
  int readsize;
  int remaining;
  int i;
  int j;

  for (remaining = length; remaining > 0; remaining -= 16, addr += 16)
    {
      /* Read up to 16 registers into a buffer */

      readsize = remaining;
      if (readsize > 16)
        {
          readsize = 16;
        }

      cc1101_access(dev, addr, (FAR uint8_t *)regbuf, readsize);

      /* Format the output data */

      for (i = 0, j = 0; i < readsize; i++, j += 3)
        {
          sprintf(&outbuf[j], " %02x", regbuf[i]);
        }

      /* Dump the formatted data to the syslog output */

      wlinfo("CC1101[%2x]:%s\n", addr, outbuf);
    }
}

/****************************************************************************
 * Name: cc1101_setpacketctrl
 *
 * Description:
 *
 ****************************************************************************/

void cc1101_setpacketctrl(struct cc1101_dev_s *dev)
{
  uint8_t values[3];

  values[0] = dev->rfsettings->FIFOTHR;
  values[1] = dev->rfsettings->SYNC1;
  values[2] = dev->rfsettings->SYNC0;
  cc1101_access(dev, CC1101_FIFOTHR, values, -3);

  /* Packet length
   * Limit it to 61 bytes in total: pktlen, data[61], rssi, lqi
   */

  values[0] = CC1101_PACKET_MAXDATALEN;
  cc1101_access(dev, CC1101_PKTLEN, values, -1);

  /* Packet Control */

  values[0] = dev->rfsettings->PKTCTRL1; /* Append status: RSSI and LQI at the
                                          * end of received packet */

  /* TODO: CRC Auto Flash bit 0x08 ??? */

  values[1] = dev->rfsettings->PKTCTRL0; /* CRC in Rx and Tx Enabled: Variable
                                          * Packet mode, defined by first byte */

  /* TODO: Enable data whitening ... */

  cc1101_access(dev, CC1101_PKTCTRL1, values, -2);

  /* Main Radio Control State Machine */

  values[0] = 0x07; /* No time-out */
  values[1] = 0x03; /* Clear channel if RSSI < thr && !receiving;
                     * TX -> RX, RX -> RX: 0x3f */
  values[2] =
      CC1101_MCSM0_VALUE; /* Calibrate on IDLE -> RX/TX, OSC Timeout = ~500 us
                           * TODO: has XOSC_FORCE_ON */
  cc1101_access(dev, CC1101_MCSM2, values, -3);

  /* Wake-On Radio Control */

  /* Not used yet. */

  /* WOREVT1:WOREVT0 - 16-bit timeout register */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc1101_init2
 *
 * Description:
 *
 ****************************************************************************/

FAR int cc1101_init2(FAR struct cc1101_dev_s *dev)
{
  int ret;

  DEBUGASSERT(dev);

  /* Reset chip, check status bytes */

  ret = cc1101_reset(dev);
  if (ret < 0)
    {
      return ret;
    }

  /* Check part compatibility */

  ret = cc1101_checkpart(dev);
  if (ret < 0)
    {
      return ret;
    }

  cc1101_setgdo(dev, CC1101_PIN_GDO0, CC1101_GDO_HIZ);
  cc1101_setgdo(dev, CC1101_PIN_GDO1, CC1101_GDO_HIZ);
  cc1101_setgdo(dev, CC1101_PIN_GDO2, CC1101_GDO_HIZ);
  cc1101_setrf(dev, dev->rfsettings);
  cc1101_setpacketctrl(dev);
  cc1101_setgdo(dev, dev->gdo, CC1101_GDO_SYNC);
  cc1101_dumpregs(dev, CC1101_PIN_GDO2, 39);
  dev->status = CC1101_IDLE;
  return 0;
}

/****************************************************************************
 * Name: cc1101_init
 *
 * Description:
 *
 ****************************************************************************/

FAR struct cc1101_dev_s *cc1101_init(
    FAR struct spi_dev_s *spi, uint32_t isr_pin, uint32_t miso_pin,
    FAR const struct c1101_rfsettings_s *rfsettings, wait_cc1101_ready wait)
{
  FAR struct cc1101_dev_s *dev;

  DEBUGASSERT(spi);

  dev = kmm_malloc(sizeof(struct cc1101_dev_s));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->isr_pin    = isr_pin;
  dev->miso_pin   = miso_pin;
  dev->rfsettings = rfsettings;
  dev->spi        = spi;
  dev->flags      = 0;
  dev->channel    = rfsettings->CHMIN;
  dev->power      = rfsettings->PAMAX;

  /* Reset chip, check status bytes */

  if (cc1101_reset(dev) < 0)
    {
      kmm_free(dev);
      return NULL;
    }

  /* Check part compatibility */

  if (cc1101_checkpart(dev) < 0)
    {
      kmm_free(dev);
      return NULL;
    }

  /* Configure CC1101:
   *  - disable GDOx for best performance
   *  - load RF
   *  - and packet control
   */

  cc1101_setgdo(dev, CC1101_PIN_GDO0, CC1101_GDO_HIZ);
  cc1101_setgdo(dev, CC1101_PIN_GDO1, CC1101_GDO_HIZ);
  cc1101_setgdo(dev, CC1101_PIN_GDO2, CC1101_GDO_HIZ);
  cc1101_setrf(dev, rfsettings);
  cc1101_setpacketctrl(dev);

  /* Set the ISR to be triggered on falling edge of the:
   *
   * 6 (0x06) Asserts when sync word has been sent / received, and
   * de-asserts at the end of the packet. In RX, the pin will de-assert
   * when the optional address check fails or the RX FIFO overflows.
   * In TX the pin will de-assert if the TX FIFO underflows.
   */

  cc1101_setgdo(dev, dev->gdo, CC1101_GDO_SYNC);

  /* Configure to receive interrupts on the external GPIO interrupt line.
   *
   * REVISIT:  There is no MCU-independent way to do this in this
   * context.
   */

  return dev;
}

/****************************************************************************
 * Name: cc1101_deinit
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_deinit(FAR struct cc1101_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Release the external GPIO interrupt
   *
   * REVISIT:  There is no MCU-independent way to do this in this
   * context.
   */

  /* Power down chip */

  cc1101_powerdown(dev);

  /* Release external interrupt line */

  kmm_free(dev);
  return 0;
}

/****************************************************************************
 * Name: cc1101_powerup
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_powerup(FAR struct cc1101_dev_s *dev)
{
  DEBUGASSERT(dev);
  return 0;
}

/****************************************************************************
 * Name: cc1101_powerdown
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_powerdown(FAR struct cc1101_dev_s *dev)
{
  DEBUGASSERT(dev);
  return 0;
}

/****************************************************************************
 * Name: cc1101_setgdo
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_setgdo(FAR struct cc1101_dev_s *dev, uint8_t pin,
                  uint8_t function)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(pin <= CC1101_IOCFG0);

  if (function >= CC1101_GDO_CLK_XOSC1)
    {
      /* Only one pin can be enabled at a time as XOSC/n */

      if (dev->flags & FLAGS_XOSCENABLED)
        {
          return -EPERM;
        }

      /* Force XOSC to stay active even in sleep mode */

      int value = CC1101_MCSM0_VALUE | CC1101_MCSM0_XOSC_FORCE_ON;
      cc1101_access(dev, CC1101_MCSM0, (FAR uint8_t *)&value, -1);

      dev->flags |= FLAGS_XOSCENABLED;
    }
  else if (dev->flags & FLAGS_XOSCENABLED)
    {
      /* Disable XOSC in sleep mode */

      int value = CC1101_MCSM0_VALUE;
      cc1101_access(dev, CC1101_MCSM0, (FAR uint8_t *)&value, -1);

      dev->flags &= ~FLAGS_XOSCENABLED;
    }

  return cc1101_access(dev, pin, &function, -1);
}

/****************************************************************************
 * Name: cc1101_setrf
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_setrf(FAR struct cc1101_dev_s *dev,
                 FAR const struct c1101_rfsettings_s *settings)
{
  int ret;

  DEBUGASSERT(dev);
  DEBUGASSERT(settings);

  ret = cc1101_access(dev, CC1101_FSCTRL1,
                      (FAR uint8_t *)&settings->FSCTRL1, -11);
  if (ret < 0)
    {
      return -EIO;
    }

  ret = cc1101_access(dev, CC1101_FOCCFG,
                      (FAR uint8_t *)&settings->FOCCFG, -5);
  if (ret < 0)
    {
      return -EIO;
    }

  ret = cc1101_access(dev, CC1101_FREND1,
                      (FAR uint8_t *)&settings->FREND1, -6);
  if (ret < 0)
    {
      return -EIO;
    }

  /* Load Power Table */

  ret = cc1101_access(dev, CC1101_PATABLE, (FAR uint8_t *)settings->PA, -8);
  if (ret < 0)
    {
      return -EIO;
    }

  /* If channel is out of valid range, mark that. Limit power.
   * We are not allowed to send any data, but are allowed to listen
   * and receive.
   */

  cc1101_setchannel(dev, dev->channel);
  cc1101_setpower(dev, dev->power);

  return OK;
}

/****************************************************************************
 * Name: cc1101_setchannel
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_setchannel(FAR struct cc1101_dev_s *dev, uint8_t channel)
{
  DEBUGASSERT(dev);

  /* Store locally in further checks */

  dev->channel = channel;

  /* If channel is out of valid, we are allowed to listen and receive only */

  if (channel < dev->rfsettings->CHMIN || channel > dev->rfsettings->CHMAX)
    {
      dev->flags |= FLAGS_RXONLY;
    }
  else
    {
      dev->flags &= ~FLAGS_RXONLY;
    }

  cc1101_access(dev, CC1101_CHANNR, &dev->channel, -1);
  return dev->flags;
}

/****************************************************************************
 * Name: cc1101_setpower
 *
 * Description:
 *
 ****************************************************************************/

uint8_t cc1101_setpower(FAR struct cc1101_dev_s *dev, uint8_t power)
{
  DEBUGASSERT(dev);

  if (power > dev->rfsettings->PAMAX)
    {
      power = dev->rfsettings->PAMAX;
    }

  dev->power = power;

  if (power == 0)
    {
      dev->flags |= FLAGS_RXONLY;
      return 0;
    }
  else
    {
      dev->flags &= ~FLAGS_RXONLY;
    }

  /* Add remaining part from RF table (to get rid of readback) */

  power--;
  power |= dev->rfsettings->FREND0;

  /* On error, report that as zero power */

  if (cc1101_access(dev, CC1101_FREND0, &power, -1) < 0)
    {
      dev->power = 0;
    }

  return dev->power;
}

/****************************************************************************
 * Name: cc1101_calc_rssi_dbm
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_calc_rssi_dbm(int rssi)
{
  if (rssi >= 128)
    {
      rssi -= 256;
    }

  return (rssi >> 1) - 74;
}

/****************************************************************************
 * Name: cc1101_receive
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_receive(FAR struct cc1101_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* REVISIT: Wait for IDLE before going into another state? */

  dev->status = CC1101_RECV;
  cc1101_strobe(dev, CC1101_SRX | CC1101_READ_SINGLE);
  return 0;
}

/****************************************************************************
 * Name: cc1101_read
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_read(FAR struct cc1101_dev_s *dev, FAR uint8_t *buf, size_t size)
{
  uint8_t nbytes = 0;

  DEBUGASSERT(dev);

  if (buf == NULL || size == 0)
    {
      cc1101_strobe(dev, CC1101_SRX);
      return 0;
    }

  cc1101_access(dev, CC1101_RXFIFO, &nbytes, 1);

  if (nbytes & 0x80)
    {
      wlwarn("RX FIFO full\n");
      nbytes = 0;
      goto breakout;
    }

  nbytes += 2; /* RSSI and LQI */
  buf[0] = nbytes;
  cc1101_access(dev, CC1101_RXFIFO, buf + 1,
                (nbytes > size) ? size : nbytes);

  /* Flush remaining bytes, if there is no room to receive or if there is a
   * BAD CRC
   */

  if (!(buf[nbytes] & 0x80))
    {
      wlwarn("RX CRC error\n");
      nbytes = 0;
    }

breakout:
  cc1101_strobe(dev, CC1101_SIDLE);
  cc1101_strobe(dev, CC1101_SFRX);
  cc1101_strobe(dev, CC1101_SRX);
  return nbytes;
}

/****************************************************************************
 * Name: cc1101_write
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_write(FAR struct cc1101_dev_s *dev, FAR const uint8_t *buf,
                 size_t size)
{
  uint8_t packetlen;

  DEBUGASSERT(dev);
  DEBUGASSERT(buf);

  if (dev->flags & FLAGS_RXONLY)
    {
      return -EPERM;
    }

  cc1101_strobe(dev, CC1101_SIDLE);
  cc1101_strobe(dev, CC1101_SFTX);
  dev->status = CC1101_SEND;

  /* Present limit */

  if (size > CC1101_PACKET_MAXDATALEN)
    {
      packetlen = CC1101_PACKET_MAXDATALEN;
    }
  else
    {
      packetlen = size;
    }

  cc1101_access(dev, CC1101_TXFIFO, &packetlen, -1);
  cc1101_access(dev, CC1101_TXFIFO, (FAR uint8_t *)buf, -size);
  return 0;
}

/****************************************************************************
 * Name: cc1101_send
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_send(FAR struct cc1101_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (dev->flags & FLAGS_RXONLY)
    {
      return -EPERM;
    }

  cc1101_strobe(dev, CC1101_STX);
  nxsem_wait(&dev->sem_tx);

  /* this is set MCSM1, send auto to rx */

  dev->status = CC1101_RECV;
  return 0;
}

/****************************************************************************
 * Name: cc1101_idle
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_idle(FAR struct cc1101_dev_s *dev)
{
  DEBUGASSERT(dev);
  cc1101_strobe(dev, CC1101_SIDLE);
  return 0;
}

/****************************************************************************
 * Name: cc1101_unregister
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_unregister(FAR struct cc1101_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Release IRQ */

  dev->ops.irq(dev, false);

  /* Free memory */

  kmm_free(dev->rx_buffer);
  kmm_free(dev);
  return OK;
}

/****************************************************************************
 * Name: cc1101_register
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_register(FAR const char *path, FAR struct cc1101_dev_s *dev)
{
  DEBUGASSERT(path);
  DEBUGASSERT(dev);

  dev->status = CC1101_INIT;
  dev->rx_buffer =
      kmm_malloc(CC1101_FIFO_SIZE * CONFIG_WL_CC1101_RXFIFO_LEN);
  if (dev->rx_buffer == NULL)
    {
      return -ENOMEM;
    }

  dev->nxt_read  = 0;
  dev->nxt_write = 0;
  dev->fifo_len  = 0;
  nxmutex_init(&dev->devlock);
  nxmutex_init(&dev->lock_rx_buffer);
  nxsem_init(&dev->sem_rx, 0, 0);
  nxsem_init(&dev->sem_tx, 0, 0);

  if (cc1101_init2(dev) < 0)
    {
      nxmutex_destroy(&dev->devlock);
      nxmutex_destroy(&dev->lock_rx_buffer);
      nxsem_destroy(&dev->sem_rx);
      nxsem_destroy(&dev->sem_tx);
      kmm_free(dev);
      wlerr("ERROR: Failed to initialize cc1101_init\n");
      return -ENODEV;
    }

  return register_driver(path, &g_cc1101ops, 0666, dev);
}

/****************************************************************************
 * Name: cc1101_isr_process
 *
 * Description:
 *
 ****************************************************************************/

void cc1101_isr_process(FAR void *arg)
{
  DEBUGASSERT(arg);
  FAR struct cc1101_dev_s *dev = (struct cc1101_dev_s *)arg;
  switch (dev->status)
    {
      case CC1101_SEND:
        nxsem_post(&dev->sem_tx);
        break;

      case CC1101_RECV:
        {
          uint8_t buf[CC1101_FIFO_SIZE];
          uint8_t len;

          memset(buf, 0, sizeof(buf));
          len = cc1101_read(dev, buf, sizeof(buf));
          wlinfo("recv==>[%d]\n", len);

          if (len < 1)
            {
              return;
            }

          fifo_put(dev, buf, len);
          nxsem_post(&dev->sem_rx);

          if (dev->pfd)
            {
              poll_notify(&dev->pfd, 1, POLLIN);
            }
        }
        break;

      default:
        wlwarn("WARNING:  Interrupt not processed\n");
        break;
    }
}

/****************************************************************************
 * Name: cc1101_isr
 *
 * Description:
 *
 ****************************************************************************/

int cc1101_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cc1101_dev_s *dev = (struct cc1101_dev_s *)arg;

  DEBUGASSERT(arg);

  work_queue(HPWORK, &dev->irq_work, cc1101_isr_process, arg, 0);
  return 0;
}
