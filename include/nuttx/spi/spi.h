/****************************************************************************
 * include/nuttx/spi/spi.h
 *
 *   Copyright(C) 2008-2013, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SPI_SPI_H
#define __INCLUDE_NUTTX_SPI_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* These SPI configuration options affect the form of the SPI interface:
 *
 * CONFIG_SPI_EXCHANGE - Driver supports a single exchange method
 *   (vs a recvblock() and sndblock ()methods).
 * CONFIG_SPI_CMDDATA - Devices on the SPI bus require out-of-band support
 *   to distinguish command transfers from data transfers.  Such devices
 *   will often support either 9-bit SPI (yech) or 8-bit SPI and a GPIO
 *   output that selects between command and data.
 * CONFIG_SPI_HWFEATURES - Include an interface method to support special,
 *   hardware-specific SPI features.
 */

/* Access macros ************************************************************/

/****************************************************************************
 * Name: SPI_LOCK
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SPI_LOCK(d,l) (d)->ops->lock(d,l)

/****************************************************************************
 * Name: SPI_SELECT
 *
 * Description:
 *   Enable/disable the SPI chip select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *   Required.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SPI_SELECT(d,id,s) ((d)->ops->select(d,id,s))

/****************************************************************************
 * Name: SPI_SETFREQUENCY
 *
 * Description:
 *   Set the SPI frequency. Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define SPI_SETFREQUENCY(d,f) ((d)->ops->setfrequency(d,f))

/****************************************************************************
 * Name: SPI_SETDELAY
 *
 * Description:
 *   Set the SPI Delays in nanoseconds. Optional.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   startdelay - The delay between CS active and first CLK
 *   stopdelay  - The delay between last CLK and CS inactive
 *   csdelay    - The delay between CS inactive and CS active again
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value is return on any
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CS_DELAY_CONTROL
#  define SPI_SETDELAY(d,a,b,c) ((d)->ops->setdelay(d,a,b,c))
#endif

/****************************************************************************
 * Name: SPI_SETMODE
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SETMODE(d,m) \
  do { if ((d)->ops->setmode) (d)->ops->setmode(d,m); } while (0)

/****************************************************************************
 * Name: SPI_SETBITS
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SETBITS(d,b) \
  do { if ((d)->ops->setbits) (d)->ops->setbits(d,b); } while (0)

/****************************************************************************
 * Name: SPI_HWFEATURES
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
  /* If there are multiple SPI drivers, some may not support hardware
   * feature selection.
   */

#  define SPI_HWFEATURES(d,f) \
  (((d)->ops->hwfeatures) ? (d)->ops->hwfeatures(d,f) : ((f) == 0 ? OK : -ENOSYS))

  /* These are currently defined feature flags:
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
   */

#  ifdef CONFIG_SPI_CRCGENERATION
#    define HWFEAT_CRCGENERATION                     (1 << 0)
#  endif

#  ifdef CONFIG_SPI_CS_CONTROL
#    define HWFEAT_FORCE_CS_CONTROL_MASK             (7 << 1)
#    define HWFEAT_FORCE_CS_INACTIVE_AFTER_TRANSFER  (1 << 1)
#    define HWFEAT_FORCE_CS_ACTIVE_AFTER_TRANSFER    (1 << 2)
#    define HWFEAT_ESCAPE_LASTXFER                   (1 << 3)
#  endif

#  ifdef CONFIG_SPI_BITORDER
#    define HWFEAT_MSBFIRST                          (0 << 4)
#    define HWFEAT_LSBFIRST                          (1 << 4)
#  endif

#else
  /* Any attempt to select hardware features with CONFIG_SPI_HWFEATURES
   * deselected will return an -ENOSYS error.
   */

#  define SPI_HWFEATURES(d,f) (((f) == 0) ? OK : -ENOSYS)
#endif

/****************************************************************************
 * Name: SPI_STATUS
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

#define SPI_STATUS(d,id) \
  ((d)->ops->status ? (d)->ops->status(d, id) : SPI_STATUS_PRESENT)

/* SPI status bits -- Some dedicated for SPI MMC/SD support and may have no
 * relationship to SPI other than needed by the SPI MMC/SD interface
 */

#define SPI_STATUS_PRESENT     0x01 /* Bit 0=1: MMC/SD card present */
#define SPI_STATUS_WRPROTECTED 0x02 /* Bit 1=1: MMC/SD card write protected */

/****************************************************************************
 * Name: SPI_CMDDATA
 *
 * Description:
 *   Some devices require and additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#  define SPI_CMDDATA(d,id,cmd) ((d)->ops->cmddata(d,id,cmd))
#endif

/****************************************************************************
 * Name: SPI_SEND
 *
 * Description:
 *   Exchange one word on SPI. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

#define SPI_SEND(d,wd) ((d)->ops->send(d,(uint16_t)(wd)))

/****************************************************************************
 * Name: SPI_SNDBLOCK
 *
 * Description:
 *   Send a block of data on SPI. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_SNDBLOCK(d,b,l) ((d)->ops->exchange(d,b,0,l))
#else
#  define SPI_SNDBLOCK(d,b,l) ((d)->ops->sndblock(d,b,l))
#endif

/****************************************************************************
 * Name: SPI_RECVBLOCK
 *
 * Description:
 *   Receive a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_RECVBLOCK(d,b,l) ((d)->ops->exchange(d,0,b,l))
#else
#  define SPI_RECVBLOCK(d,b,l) ((d)->ops->recvblock(d,b,l))
#endif

/****************************************************************************
 * Name: SPI_EXCHANGE
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_EXCHANGE(d,t,r,l) ((d)->ops->exchange(d,t,r,l))
#endif

/****************************************************************************
 * Name: SPI_REGISTERCALLBACK
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change (i.e, anything that would be reported differently by SPI_STATUS).
 *   Optional
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#define SPI_REGISTERCALLBACK(d,c,a) \
  ((d)->ops->registercallback ? (d)->ops->registercallback(d,c,a) : -ENOSYS)

/* SPI Device Macros ********************************************************/

/* This builds a SPI devid from its type and index */

#define SPIDEV_ID(type,index) ((((uint32_t)(type)  & 0xffff) << 16) | \
                                ((uint32_t)(index) & 0xffff))

/* This retrieves the fields from a SPI devid */

#define SPIDEVID_TYPE (devid) (((uint32_t)(devid) >> 16) & 0xffff)
#define SPIDEVID_INDEX(devid)  ((uint32_t)(devid)        & 0xffff)

/* These are standard definitions for the defined SPI device IDs.  The index
 * argument, n, is the instance number.  This should be zero if there is
 * only one instance of the SPI device on the SPI bus. Indices greater than
 * zero discriminate the additional devices of the same type on the SPI bus.
 */

#define SPIDEV_NONE(n)          SPIDEV_ID(SPIDEVTYPE_NONE,          (n))
#define SPIDEV_MMCSD(n)         SPIDEV_ID(SPIDEVTYPE_MMCSD,         (n))
#define SPIDEV_FLASH(n)         SPIDEV_ID(SPIDEVTYPE_FLASH,         (n))
#define SPIDEV_ETHERNET(n)      SPIDEV_ID(SPIDEVTYPE_ETHERNET,      (n))
#define SPIDEV_DISPLAY(n)       SPIDEV_ID(SPIDEVTYPE_DISPLAY,       (n))
#define SPIDEV_CAMERA(n)        SPIDEV_ID(SPIDEVTYPE_CAMERA,        (n))
#define SPIDEV_WIRELESS(n)      SPIDEV_ID(SPIDEVTYPE_WIRELESS,      (n))
#define SPIDEV_TOUCHSCREEN(n)   SPIDEV_ID(SPIDEVTYPE_TOUCHSCREEN,   (n))
#define SPIDEV_EXPANDER(n)      SPIDEV_ID(SPIDEVTYPE_EXPANDER,      (n))
#define SPIDEV_MUX(n)           SPIDEV_ID(SPIDEVTYPE_MUX,           (n))
#define SPIDEV_AUDIO_DATA(n)    SPIDEV_ID(SPIDEVTYPE_AUDIO_DATA,    (n))
#define SPIDEV_AUDIO_CTRL(n)    SPIDEV_ID(SPIDEVTYPE_AUDIO_CTRL,    (n))
#define SPIDEV_EEPROM(n)        SPIDEV_ID(SPIDEVTYPE_EEPROM,        (n))
#define SPIDEV_ACCELEROMETER(n) SPIDEV_ID(SPIDEVTYPE_ACCELEROMETER, (n))
#define SPIDEV_BAROMETER(n)     SPIDEV_ID(SPIDEVTYPE_BAROMETER,     (n))
#define SPIDEV_TEMPERATURE(n)   SPIDEV_ID(SPIDEVTYPE_TEMPERATURE,   (n))
#define SPIDEV_IEEE802154(n)    SPIDEV_ID(SPIDEVTYPE_IEEE802154,    (n))
#define SPIDEV_CONTACTLESS(n)   SPIDEV_ID(SPIDEVTYPE_CONTACTLESS,   (n))
#define SPIDEV_CANBUS(n)        SPIDEV_ID(SPIDEVTYPE_CANBUS,        (n))
#define SPIDEV_USBHOST(n)       SPIDEV_ID(SPIDEVTYPE_USBHOST,       (n))
#define SPIDEV_USER(n)          SPIDEV_ID(SPIDEVTYPE_USER,          (n))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The type of the media change callback function */

typedef void (*spi_mediachange_t)(FAR void *arg);

/* If the board supports multiple SPI devices types, this enumeration
 * identifies which is selected or de-selected.
 * There may be more than one instance of each type on a bus, see below.
 */

enum spi_devtype_e
{
  SPIDEVTYPE_NONE = 0,      /* Not a valid value */
  SPIDEVTYPE_MMCSD,         /* Select SPI MMC/SD device */
  SPIDEVTYPE_FLASH,         /* Select SPI FLASH device */
  SPIDEVTYPE_ETHERNET,      /* Select SPI Ethernet device */
  SPIDEVTYPE_DISPLAY,       /* Select SPI LCD/OLED display device */
  SPIDEVTYPE_CAMERA,        /* Select SPI imaging device */
  SPIDEVTYPE_WIRELESS,      /* Select SPI Wireless device */
  SPIDEVTYPE_TOUCHSCREEN,   /* Select SPI touchscreen device */
  SPIDEVTYPE_EXPANDER,      /* Select SPI I/O expander device */
  SPIDEVTYPE_MUX,           /* Select SPI multiplexer device */
  SPIDEVTYPE_AUDIO_DATA,    /* Select SPI audio codec device data port */
  SPIDEVTYPE_AUDIO_CTRL,    /* Select SPI audio codec device control port */
  SPIDEVTYPE_EEPROM,        /* Select SPI EEPROM device */
  SPIDEVTYPE_ACCELEROMETER, /* Select SPI Accelerometer device */
  SPIDEVTYPE_BAROMETER,     /* Select SPI Pressure/Barometer device */
  SPIDEVTYPE_TEMPERATURE,   /* Select SPI Temperature sensor device */
  SPIDEVTYPE_IEEE802154,    /* Select SPI IEEE 802.15.4 wireless device */
  SPIDEVTYPE_CONTACTLESS,   /* Select SPI Contactless device */
  SPIDEVTYPE_CANBUS,        /* Select SPI CAN bus controller over SPI */
  SPIDEVTYPE_USBHOST,       /* Select SPI USB host controller over SPI */
  SPIDEVTYPE_USER           /* Board-specific values start here
                             * This must always be the last definition. */
};

/* Certain SPI devices may required different clocking modes */

enum spi_mode_e
{
  SPIDEV_MODE0 = 0,     /* CPOL=0 CHPHA=0 */
  SPIDEV_MODE1,         /* CPOL=0 CHPHA=1 */
  SPIDEV_MODE2,         /* CPOL=1 CHPHA=0 */
  SPIDEV_MODE3          /* CPOL=1 CHPHA=1 */
};

#ifdef CONFIG_SPI_HWFEATURES
/* This is a type wide enough to support all hardware features */

typedef uint8_t spi_hwfeatures_t;
#endif

/* The SPI vtable */

struct spi_dev_s;
struct spi_ops_s
{
  CODE int      (*lock)(FAR struct spi_dev_s *dev, bool lock);
  CODE void     (*select)(FAR struct spi_dev_s *dev, uint32_t devid,
                  bool selected);
  CODE uint32_t (*setfrequency)(FAR struct spi_dev_s *dev, uint32_t frequency);
#ifdef CONFIG_SPI_CS_DELAY_CONTROL
  CODE int      (*setdelay)(FAR struct spi_dev_s *dev, uint32_t a, uint32_t b,
                  uint32_t c);
#endif
  CODE void     (*setmode)(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
  CODE void     (*setbits)(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
  CODE int      (*hwfeatures)(FAR struct spi_dev_s *dev,
                  spi_hwfeatures_t features);
#endif
  CODE uint8_t  (*status)(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
  CODE int      (*cmddata)(FAR struct spi_dev_s *dev, uint32_t devid,
                  bool cmd);
#endif
  CODE uint16_t (*send)(FAR struct spi_dev_s *dev, uint16_t wd);
#ifdef CONFIG_SPI_EXCHANGE
  CODE void     (*exchange)(FAR struct spi_dev_s *dev,
                  FAR const void *txbuffer, FAR void *rxbuffer,
                  size_t nwords);
#else
  CODE void     (*sndblock)(FAR struct spi_dev_s *dev,
                  FAR const void *buffer, size_t nwords);
  CODE void     (*recvblock)(FAR struct spi_dev_s *dev, FAR void *buffer,
                  size_t nwords);
#endif
  CODE int      (*registercallback)(FAR struct spi_dev_s *dev,
                  spi_mediachange_t callback, void *arg);
};

/* SPI private data.  This structure only defines the initial fields of the
 * structure visible to the SPI client.  The specific implementation may
 * add additional, device specific fields
 */

struct spi_dev_s
{
  FAR const struct spi_ops_s *ops;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_SPI_SPI_H */
