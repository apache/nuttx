/****************************************************************************
 * arch/sim/src/sim/posix/sim_spilinux.c
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <linux/spi/spidev.h>

#include "sim_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERROR(fmt, ...) \
        syslog(LOG_ERR, "sim_spilinux: " fmt "\n", ##__VA_ARGS__)
#define INFO(fmt, ...) \
        syslog(LOG_ERR, "sim_spilinux: " fmt "\n", ##__VA_ARGS__)
#define DEBUG(fmt, ...)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct linux_spi_dev_s
{
  const struct spi_ops_s *ops; /* SPI vtable */
  int file;                    /* SPI device file descriptor in Linux. */
#ifdef CONFIG_SPI_HWFEATURES
  spi_hwfeatures_t hwfeatures; /* Some hardware features. */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int linux_spi_lock(struct spi_dev_s *dev, bool lock);
static void linux_spi_select(struct spi_dev_s *dev, uint32_t devid,
                            bool selected);
static uint32_t linux_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
#ifdef CONFIG_SPI_CS_DELAY_CONTROL
static int linux_spi_setdelay(struct spi_dev_s *dev, uint32_t a, uint32_t b,
                       uint32_t c);
#endif
static void linux_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void linux_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int linux_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint8_t  linux_spi_status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int linux_spi_cmddata(struct spi_dev_s *dev, uint32_t devid,
                             bool cmd);
#endif
static uint32_t linux_spi_send(struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void linux_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#else
static void linux_spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                               size_t nwords);
static void linux_spi_recvblock(struct spi_dev_s *dev, void *buffer,
                                size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int linux_spi_trigger(struct spi_dev_s *dev);
#endif
static int linux_spi_registercallback(struct spi_dev_s *dev,
                                      spi_mediachange_t callback, void *arg);
static int linux_spi_transfer(struct spi_dev_s *dev, const void *txbuffer,
                              void *rxbuffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_ops_s spi_linux_ops =
{
  /* The operations below are the same as those in nuttx/spi/spi.h.
   * Some perations are dummy, merely for compatiablity with nuttx spi.
   */

  .lock             = linux_spi_lock,          /* Dummy for compatibility. */
  .select           = linux_spi_select,        /* Dummy for compatibility. */
  .setfrequency     = linux_spi_setfrequency,  /* Set max speed. */
#ifdef CONFIG_SPI_CS_DELAY_CONTROL
  .setdelay         = linux_spi_setdelay,      /* Dummy for compatibility. */
#endif
  .setmode          = linux_spi_setmode,       /* Set mode 0~3. */
  .setbits          = linux_spi_setbits,       /* Set bits per word. */
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = linux_spi_hwfeatures,    /* Dummy for compatibility. */
#endif
  .status           = linux_spi_status,        /* Dummy for compatibility. */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = linux_spi_cmddata,       /* Dummy for compatibility. */
#endif
  .send             = linux_spi_send,          /* Send a word. */
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = linux_spi_exchange,      /* Send and receive words. */
#else
  .sndblock         = linux_spi_sndblock,      /* Send several words. */
  .recvblock        = linux_spi_recvblock,     /* Receive several words. */
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = linux_spi_trigger,       /* Dummy for compatibility. */
#endif
  .registercallback = linux_spi_registercallback,  /* Dummy for
                                                    * compatibility. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: linux_spi_lock
 *
 * Description:
 *   Provide spi lock, used for getting exclusive access to the SPI bus.
 *   It's not supported by this driver nor a linux spi, and will directly
 *   return 0.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   lock - TRUE: lock, FALSE: unlock.
 *
 * Returned Value:
 *   0 for success, since it's a necessary step for Nuttx SPI transfer.
 ****************************************************************************/

static int linux_spi_lock(struct spi_dev_s *dev, bool lock)
{
  return 0;
}

/****************************************************************************
 * Name: linux_spi_select
 *
 * Description:
 *   Provide spi select, used for selecting the device before a transfer.
 *   It's not supported by linux spi and will do nothing. For a Linux SPI
 *   device "spidevN.P", the N means bus number and P means CS number. When
 *   you choose the spidevN.P to attach to simulator, only CS_P will be
 *   selected and then de-selected automatically when you call a ioctl()
 *   operation with SPI_IOC_MESSAGE(x).
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   devid - The CS id,
 *   selected - TRUE: slave selected, FALSE: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void linux_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected)
{
}

/****************************************************************************
 * Name: linux_spi_setfrequency
 *
 * Description:
 *   Provide spi setfrequency, used for set SPI clock frequency.
 *   Note that only MAX_SPEED_HZ could be configured out of a transfer for a
 *   Linux SPI port. The Linux SPI may set a exact frequecy using the value
 *   of spi_ioc_transfer.speed_hz when transferring. If the
 *   spi_ioc_transfer.speed_hz is 0, the MAX_SPEED_HZ is used. In practice,
 *   the real frequecy on the CLK wire will be affected by the hardware.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   frequency - The frequencey of SPI clock in Hz.
 *
 * Returned Value:
 *   Returns the actual frequency in Hz.
 *
 ****************************************************************************/

static uint32_t linux_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct linux_spi_dev_s *priv = (struct linux_spi_dev_s *)dev;
  int file = priv->file;
  uint32_t actualfreq;

  ioctl(file, SPI_IOC_WR_MAX_SPEED_HZ, &frequency);
  ioctl(file, SPI_IOC_RD_MAX_SPEED_HZ, &actualfreq);

  return actualfreq;
}

/****************************************************************************
 * Name: linux_spi_setdelay
 *
 * Description:
 *   Provide spi setdelay.
 *   It's not supported by this driver and will return error. DelayS between
 *   CS change and CLK is not supported by a Linux SPI. Delay between CS
 *   inactive and active again is set in spi_ioc_transfer.delay_usecs
 *   (precondition: spi_ioc_transfer.cs_change = true) when using
 *   ioctl(filep, SPI_IOC_MEASSAGE(N), &spi_ioc_transfer) for a Linux SPI.
 *   For a Nuttx SPI driver it's set in spi_trans_s.delay (precondition: call
 *   hwfeatures operation with HWFEAT_FORCE_CS_INACTIVE_AFTER_TRANSFER
 *   first). Thus csdelay need not to be set here. This optional operation
 *   should not be used (let SPI_CS_DELAY_CONTROL = n), or one will get an
 *   error.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   startdelay - The delay between CS active and first CLK
 *   stopdelay  - The delay between last CLK and CS inactive
 *   csdelay    - The delay between CS inactive and CS active again
 *
 * Returned Value:
 *   -ENOSYS for not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CS_DELAY_CONTROL
static int linux_spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                              uint32_t stopdelay, uint32_t csdelay)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: linux_spi_setmode
 *
 * Description:
 *   Provide spi setmode.
 *   SPI mode defination in nuttx is almost the same to that in Linux.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void linux_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct linux_spi_dev_s *priv = (struct linux_spi_dev_s *)dev;
  int file = priv->file;
  uint8_t spilinuxmode;

  switch (mode)
    {
      case SPIDEV_MODE0:
        {
          spilinuxmode = SPI_MODE_0;
        }
        break;

      /* In fact SPIDEV_MODETI is equal to SPIDEV_MODE1 (CPOL=0 CPHA=1). */

      case SPIDEV_MODETI:

      case SPIDEV_MODE1:
        {
          spilinuxmode = SPI_MODE_1;
        }
        break;

      case SPIDEV_MODE2:
        {
          spilinuxmode = SPI_MODE_2;
        }
        break;

      case SPIDEV_MODE3:
        {
          spilinuxmode = SPI_MODE_3;
        }
        break;

      default:
        {
          spilinuxmode = SPI_MODE_0;
        }
        break;
    }

    ioctl(file, SPI_IOC_WR_MODE, &spilinuxmode);
}

/****************************************************************************
 * Name: linux_spi_setbits
 *
 * Description:
 *   Provide spi setbits, used for set bits per word during a transfer.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void linux_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct linux_spi_dev_s *priv = (struct linux_spi_dev_s *)dev;
  int file = priv->file;
  uint8_t bits_per_word = (uint8_t)nbits;

  ioctl(file, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
}

/****************************************************************************
 * Name: linux_spi_hwfeatures
 *
 * Description:
 *   Provide spi hwfeatures.
 *   Note that not all configurations are supported.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   features - Hardware feature flag.
 *
 * Returned Value:
 *   0 for success, and negated errno for error.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int linux_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
  struct linux_spi_dev_s *priv = (struct linux_spi_dev_s *)dev;
  int file = priv->file;
  uint8_t lsb = 0;

  /* These are currently defined feature flags in nuttx/spi/spi.h:
   *
   *   Bit 0: HWFEAT_CRCGENERATION
   *          Hardware CRC generation
   *   Bit 1: HWFEAT_FORCE_CS_INACTIVE_AFTER_TRANSFER
   *          CS rises after every Transmission, also if we provide new data
   *          immediately
   *   Bit 2: HWFEAT_FORCE_CS_ACTIVE_AFTER_TRANSFER
   *          CS does not rise automatically after a transmission, also if
   *          the spi runs out of data (for a long time)
   *   Bit 3: HWFEAT_ESCAPE_LASTXFER
   *          Do not set the LASTXFER-Bit at the last word of the next
   *          exchange, Flag is auto-resetting after the next LASTXFER
   *          condition. (see spi_exchange)
   *   Bit 4: HWFEAT_LSBFIRST
   *          Data transferred LSB first (default is MSB first)
   *   Bit 5: Turn deferred trigger mode on or off.  Primarily used for DMA
   *          mode.  If a transfer is deferred then the DMA will not actually
   *          be triggered until a subsequent call to SPI_TRIGGER to set it
   *          off.
   * Among them, features on bit 1, 2, 4 is supported.
   * And CS_ACTIVE/INACTIVE can not be set immediately until calling
   * linux_spi_transfer. Here it's recorded in linux_spi_dev_s.hwfeatures
   * for furture use.
   */

  priv->hwfeatures = features;

  /* MSB or LSB first can be set immediately here. */

  if (features & HWFEAT_LSBFIRST)
    {
      lsb = 1;
    }

  return ioctl(file, SPI_IOC_WR_LSB_FIRST, &lsb);
}
#endif

/****************************************************************************
 * Name: linux_spi_status
 *
 * Description:
 *   Provide spi status.
 *   It's not supported by linux spi and will directly return.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   devid - The CS id, not supported in Linux SPI. For a Linux SPI device
 *           "spidevN.P", the N means bus number and P means CS number.
 *
 * Returned Value:
 *   0 for no status to be reported.
 *
 ****************************************************************************/

static uint8_t linux_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name: linux_spi_cmddata
 *
 * Description:
 *   Provide spi cmddata toggle.
 *   This need an additional out-of-band bit and is not supported by this
 *   driver. This operation should not be used (let SPI_CMDDATA = n).
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   devid - The CS id, not supported in Linux SPI. For a Linux SPI device
 *           "spidevN.P", the N means bus number and P means CS number.
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   -ENOSYS for not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int linux_spi_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: linux_spi_send
 *
 * Description:
 *   Provide spi send, used for sending one word.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   wd - The word to send. The size of the data is determined by the number
 *        of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received word value.
 *
 ****************************************************************************/

static uint32_t linux_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  uint32_t recvwd = 0;

  linux_spi_transfer(dev, &wd, &recvwd, 1);

  return recvwd;
}

/****************************************************************************
 * Name: linux_spi_exchange
 *
 * Description:
 *   Provide spi exchange, used for transmit and receive N words.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   txbuffer - A pointer to the buffer in which to transmit data.
 *   rxbuffer - A pointer to the buffer in which to receive data.
 *   nwords - The length of data that can be transferred in the buffer
 *            in number of words.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
static void linux_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
{
  linux_spi_transfer(dev, txbuffer, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: linux_spi_sndblock
 *
 * Description:
 *   Provide spi sndblock, used for send several words.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   buffer - A pointer to the buffer in which to transmit data.
 *   nwords - The length of data that can be send in the buffer in number of
 *            words.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void linux_spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                               size_t nwords)
{
  linux_spi_transfer(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: linux_spi_recvblock
 *
 * Description:
 *   Provide spi recvblock, used for receive(read) several words.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   buffer - A pointer to the buffer in which to receive data.
 *   nwords - The length of data that can be received in the buffer in number
 *            of words.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void linux_spi_recvblock(struct spi_dev_s *dev, void *buffer,
                                size_t nwords)
{
  linux_spi_transfer(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: linux_spi_trigger
 *
 * Description:
 *   Provide spi trigger, to trigger a previously configured DMA transfer.
 *   It's not supported by this driver. This operation should not be used
 *   (let SPI_TRIGGER = n).
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *
 * Returned Value:
 *   -ENOSYS for not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int linux_spi_trigger(struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: linux_spi_registercallback
 *
 * Description:
 *   Provide spi registercallback.
 *   It's not supported by linux spi and will directly return.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   callback - The function to call on the media change
 *   arg - A caller provided value to return with the callback
 *
 * Returned Value:
 *   -ENOSYS for not supported.
 *
 ****************************************************************************/

static int linux_spi_registercallback(struct spi_dev_s *dev,
                                      spi_mediachange_t callback, void *arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: linux_spi_transfer
 *
 * Description:
 *   Provide spi transfer as the base of linux_spi_send, linux_spi__exchange,
 *   linux_spi__sndblock and linux_spi_recvblock.
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *   txbuffer - A pointer to the buffer in which to transmit data.
 *   rxbuffer - A pointer to the buffer in which to receive data.
 *   nwords - The length of data that can be transferred in the buffer
 *            in number of words.
 *
 * Returned Value:
 *   Actual number of the words transferred.
 *
 ****************************************************************************/

static int linux_spi_transfer(struct spi_dev_s *dev, const void *txbuffer,
                              void *rxbuffer, size_t nwords)
{
  struct linux_spi_dev_s *priv = (struct linux_spi_dev_s *)dev;
  int file = priv->file;

  /* Some members of struct spi_ioc_transfer transfer_data is default 0:
   * @speed_hz = 0, thus it's ignored, MAX_SEPPD_HZ will be used.
   * @bits_per_word = 0, thus it's ignored, BITS_PER_WORD will be used.
   * @delay_usecs = 0, thus thers's no delay before next transfer.
   */

  struct spi_ioc_transfer transfer_data = {
    .tx_buf = (unsigned long)txbuffer, /* Transmit buffer. */
    .rx_buf = (unsigned long)rxbuffer, /* Receive buffer. */
    .len = (uint32_t)nwords,           /* Number of words. */
    .cs_change = false,                /* In normal, CS remains selected. */
  };

#ifdef CONFIG_SPI_HWFEATURES
  /* If g_hwfeatures is set as HWFEAT_CS_INACTIVE before, using
   * linux_spi_hwfeatures, then that setting will take effect here. If
   * g_hwfeatures is set as HWFEAT_CS_ACTIVE which is the normal case, there
   * is no need to change transfer_data.cs_change.
   */

  if (priv->hwfeatures | HWFEAT_CS_INACTIVE)
    {
      transfer_data.cs_change = true;
    }
#endif

  return ioctl(file, SPI_IOC_MESSAGE(1), &transfer_data);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_spi_initialize
 *
 * Description:
 *   Initialize one SPI port
 *
 * Input Parameters:
 *   filename - the name of SPI device in Linux, e.g. "spidev0.0".
 *
 * Returned Value:
 *   The pointer to the instance of Linux SPI device.
 *
 ****************************************************************************/

struct spi_dev_s *sim_spi_initialize(const char *filename)
{
  struct linux_spi_dev_s *priv;

  priv = (struct linux_spi_dev_s *)malloc(sizeof(priv));
  if (priv == NULL)
    {
      ERROR("Failed to allocate private spi master driver");
      return NULL;
    }

  priv->file = open(filename, O_RDWR);
  if (priv->file < 0)
    {
      ERROR("Failed to open %s: %d", filename, priv->file);
      free(priv);
      return NULL;
    }

  priv->ops = &spi_linux_ops;
#ifdef CONFIG_SPI_HWFEATURES
  priv->hwfeatures = 0;
#endif

  return (struct spi_dev_s *)priv;
}

/****************************************************************************
 * Name: sim_spi_uninitialize
 *
 * Description:
 *   Uninitialize an SPI port
 *
 * Input Parameters:
 *   dev - A pointer to instance of Linux SPI device.
 *
 * Returned Value:
 *   0 for OK.
 *
 ****************************************************************************/

int sim_spi_uninitialize(struct spi_dev_s *dev)
{
  struct linux_spi_dev_s *priv = (struct linux_spi_dev_s *)dev;
  if (priv->file >= 0)
    {
      close(priv->file);
    }

  free(priv);
  return 0;
}
