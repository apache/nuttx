/************************************************************************************
 * drivers/spi/spi_flash.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <string.h>

#include <nuttx/spi/spi_flash.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Define the FLASH SIZE in bytes */

#ifdef CONFIG_SPI_FLASH_1M
#  define CONFIG_SPI_FLASH_SIZE            (128 * 1024)
#  define CONFIG_SPI_FLASH_CAPACITY        0x11

#ifndef CONFIG_SPI_FLASH_SECTORSIZE
#  define CONFIG_SPI_FLASH_SECTORSIZE      2048
#endif

#endif

#ifdef CONFIG_SPI_FLASH_8M
#  define CONFIG_SPI_FLASH_SIZE            (1024 * 1024)
#  define CONFIG_SPI_FLASH_CAPACITY_SST26  0x3f
#  define CONFIG_SPI_FLASH_CAPACITY        0x14
#endif

#ifdef CONFIG_SPI_FLASH_32M
#  define CONFIG_SPI_FLASH_SIZE            (4 * 1024 * 1024)
#  define CONFIG_SPI_FLASH_CAPACITY_SST26  0x42
#  define CONFIG_SPI_FLASH_CAPACITY        0x16
#endif

#ifdef CONFIG_SPI_FLASH_64M
#  define CONFIG_SPI_FLASH_SIZE            (8 * 1024 * 1024)
#  define CONFIG_SPI_FLASH_CAPACITY_SST26  0x43
#  define CONFIG_SPI_FLASH_CAPACITY        0x17
#endif

#ifdef CONFIG_SPI_FLASH_128M
#  define CONFIG_SPI_FLASH_SIZE            (16 * 1024 * 1024)
#  define CONFIG_SPI_FLASH_CAPACITY_SST26  0x44
#  define CONFIG_SPI_FLASH_CAPACITY        0x18
#endif

#ifndef CONFIG_SPI_FLASH_MANUFACTURER
#  define CONFIG_SPI_FLASH_MANUFACTURER    0x20
#endif

#ifndef CONFIG_SPI_FLASH_MEMORY_TYPE
#  define CONFIG_SPI_FLASH_MEMORY_TYPE     0x20
#endif

#ifndef CONFIG_SPI_FLASH_SECTORSIZE
#  define CONFIG_SPI_FLASH_SECTORSIZE      65536
#endif

#ifndef CONFIG_SPI_FLASH_SUBSECTORSIZE
#  define CONFIG_SPI_FLASH_SUBSECTORSIZE   4096
#endif

#ifndef CONFIG_SPI_FLASH_SECTORSIZE_MASK
#  define CONFIG_SPI_FLASH_SECTORSIZE_MASK (~(CONFIG_SPI_FLASH_SECTORSIZE-1))
#endif

#ifndef CONFIG_SPI_FLASH_SUBSECTORSIZE_MASK
#  define CONFIG_SPI_FLASH_SUBSECTORSIZE_MASK (~(CONFIG_SPI_FLASH_SUBSECTORSIZE-1))
#endif

#ifndef CONFIG_SPI_FLASH_PAGESIZE
#  define CONFIG_SPI_FLASH_PAGESIZE        256
#endif

#ifndef CONFIG_SPI_FLASH_PAGESIZE_MASK
#  define CONFIG_SPI_FLASH_PAGESIZE_MASK   (CONFIG_SPI_FLASH_PAGESIZE-1)
#endif

/* Define FLASH States */

#define SPI_FLASH_STATE_IDLE         0
#define SPI_FLASH_STATE_RDID1        1
#define SPI_FLASH_STATE_RDID2        2
#define SPI_FLASH_STATE_RDID3        3
#define SPI_FLASH_STATE_WREN         4
#define SPI_FLASH_STATE_RDSR         5
#define SPI_FLASH_STATE_SE1          6
#define SPI_FLASH_STATE_SE2          7
#define SPI_FLASH_STATE_SE3          8
#define SPI_FLASH_STATE_PP1          9
#define SPI_FLASH_STATE_PP2          10
#define SPI_FLASH_STATE_PP3          11
#define SPI_FLASH_STATE_PP4          12
#define SPI_FLASH_STATE_READ1        13
#define SPI_FLASH_STATE_READ2        14
#define SPI_FLASH_STATE_READ3        15
#define SPI_FLASH_STATE_READ4        16
#define SPI_FLASH_STATE_FREAD_WAIT   17

/* Instructions */

/*      Command            Value      N Description             Addr Dummy Data   */

#define SPI_FLASH_WREN      0x06    /* 1 Write Enable              0   0     0     */
#define SPI_FLASH_WRDI      0x04    /* 1 Write Disable             0   0     0     */
#define SPI_FLASH_RDID      0x9f    /* 1 Read Identification       0   0     1-3   */
#define SPI_FLASH_RDSR      0x05    /* 1 Read Status Register      0   0     >=1   */
#define SPI_FLASH_WRSR      0x01    /* 1 Write Status Register     0   0     1     */
#define SPI_FLASH_READ      0x03    /* 1 Read Data Bytes           3   0     >=1   */
#define SPI_FLASH_FAST_READ 0x0b    /* 1 Higher speed read         3   1     >=1   */
#define SPI_FLASH_PP        0x02    /* 1 Page Program              3   0     1-256 */
#define SPI_FLASH_SE        0xd8    /* 1 Sector Erase              3   0     0     */
#define SPI_FLASH_BE        0xc7    /* 1 Bulk Erase                0   0     0     */
#define SPI_FLASH_DP        0xb9    /* 2 Deep power down           0   0     0     */
#define SPI_FLASH_RES       0xab    /* 2 Read Electronic Signature 0   3     >=1   */
#define SPI_FLASH_SSE       0x20    /* 3 Sub-Sector Erase          0   0     0     */

#define SPI_FLASH_DUMMY     0xa5

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct spi_flash_dev_s
{
  struct spi_dev_s  spidev;     /* Externally visible part of the SPI interface */
  FAR const char *  name;       /* Name of the flash type (m25p, w25, etc.) */
  uint8_t           manuf;
  uint8_t           type;
  uint8_t           capacity;
  uint8_t           last_cmd;
  uint32_t          selected;   /* SPIn base address */
  uint32_t          read_data;
  int               wren;
  int               state;
  unsigned long     address;
  unsigned char     data[CONFIG_SPI_FLASH_SIZE];
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* SPI methods */

static int         spi_flash_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t    spi_flash_setfrequency(FAR struct spi_dev_s *dev,
                                          uint32_t frequency);
static void        spi_flash_setmode(FAR struct spi_dev_s *dev,
                                     enum spi_mode_e mode);
static void        spi_flash_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t    spi_flash_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void        spi_flash_exchange(FAR struct spi_dev_s *dev,
                                      FAR const void *txbuffer, FAR void *rxbuffer,
                                      size_t nwords);
static void        spi_flash_select(FAR struct spi_dev_s *dev, uint32_t devid,
                                    bool selected);
static uint8_t     spi_flash_status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int         spi_flash_cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                                     bool cmd);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_flash_sndblock(FAR struct spi_dev_s *dev,
                                      FAR const void *txbuffer, size_t nwords);
static void        spi_flash_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                       size_t nwords);
#endif

static void spi_flash_writeword(FAR struct spi_flash_dev_s *priv, uint16_t data);
static uint32_t spi_flash_readword(FAR struct spi_flash_dev_s *priv);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct spi_ops_s g_spiops =
{
  spi_flash_lock,               /* lock */
  spi_flash_select,             /* select */
  spi_flash_setfrequency,       /* setfrequency */
#ifdef CONFIG_SPI_DELAY_CONTROL
  NULL,                         /* setdelay */
#endif
  spi_flash_setmode,            /* setmode */
  spi_flash_setbits,            /* setbits */
#ifdef CONFIG_SPI_HWFEATURES
  NULL,                         /* hwfeatures */
#endif
  spi_flash_status,             /* status */
#ifdef CONFIG_SPI_CMDDATA
  spi_flash_cmddata,            /* cmddata */
#endif
  spi_flash_send,               /* send */
#ifdef CONFIG_SPI_EXCHANGE
  spi_flash_exchange,           /* exchange */
#else
  spi_flash_sndblock,           /* sndblock */
  spi_flash_recvblock,          /* recvblock */
#endif
#ifdef CONFIG_SPI_TRIGGER
  NULL,                         /* trigger */
#endif
  NULL                          /* registercallback */
};

#ifdef CONFIG_SPI_FLASH_M25P
struct spi_flash_dev_s g_spidev_m25p =
{
  {
    &g_spiops                           /* spidev */
  },
  "m25p",                               /* name */
  0x20,                                 /* manuf */
  0x20,                                 /* type */
  CONFIG_SPI_FLASH_CAPACITY             /* capacity */
};
#endif

#ifdef CONFIG_SPI_FLASH_SST26
struct spi_flash_dev_s g_spidev_sst26 =
{
  {
    &g_spiops                           /* spidev */
  },
  "sst26",                              /* name */
  0xbf,                                 /* manuf */
#ifdef CONFIG_SST26_MEMORY_TYPE
  CONFIG_SST26_MEMORY_TYPE,             /* type */
#else
  0x25,                                 /* type */
#endif
  CONFIG_SPI_FLASH_CAPACITY_SST26       /* capacity */
};
#endif

#ifdef CONFIG_SPI_FLASH_W25
struct spi_flash_dev_s g_spidev_w25 =
{
  {
    &g_spiops                           /* spidev */
  },
  "w25",                                /* name */
  0xef,                                 /* manuf */
  0x30,                                 /* type */
  CONFIG_SPI_FLASH_CAPACITY             /* capacity */
};
#endif

#ifdef CONFIG_SPI_FLASH_CUSTOM
struct spi_flash_dev_s g_spidev_custom =
{
  {
    &g_spiops                           /* spidev */
  },
  "custom",                             /* name */
  CONFIG_SPI_FLASH_MANUFACTURER,        /* manuf */
  CONFIG_SPI_FLASH_MEMORY_TYPE,         /* type */
  CONFIG_SPI_FLASH_CAPACITY             /* capacity */
};
#endif

struct spi_flash_dev_s *gp_spidev[] =
{
#ifdef CONFIG_SPI_FLASH_M25P
  &g_spidev_m25p,
#endif
#ifdef CONFIG_SPI_FLASH_SST26
  &g_spidev_sst26,
#endif
#ifdef CONFIG_SPI_FLASH_W25
  &g_spidev_w25,
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM
  &g_spidev_custom,
#endif

  /* Null termination pointer at end of list */

  NULL
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: spi_flash_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static int spi_flash_lock(FAR struct spi_dev_s *dev, bool lock)
{
  return OK;
}

/************************************************************************************
 * Name: spi_flash_select
 *
 * Description:
 *   Process select logic for the FLASH.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_flash_select(FAR struct spi_dev_s *dev, uint32_t devid,
                             bool selected)
{
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;

  if (devid == SPIDEV_FLASH(0))
    {
      priv->selected = selected;

      /* As part of an de-select, ensure the WREN bit is cleared */

      if (!selected)
        {
          if (priv->last_cmd != SPI_FLASH_WREN)
            {
              priv->wren = 0;
            }

          priv->state = SPI_FLASH_STATE_IDLE;
        }
    }
}

/************************************************************************************
 * Name: spi_flash_cmddata
 *
 * Description:
 *   Perform SPI Command operations
 *
 * Returned Value:
 *   Always returns zero
 *
 ************************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_flash_cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif

/************************************************************************************
 * Name: spi_flash_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static uint32_t spi_flash_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  return frequency;
}

/************************************************************************************
 * Name: spi_flash_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static void spi_flash_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
}

/************************************************************************************
 * Name: spi_flash_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_flash_setbits(FAR struct spi_dev_s *dev, int nbits)
{
}

/************************************************************************************
 * Name: spi_flash_status
 *
 * Description:
 *   Set the SPI bus status
 *
 * Returned Value:
 *   Always returns zero
 *
 ************************************************************************************/

static uint8_t spi_flash_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/************************************************************************************
 * Name: spi_flash_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ************************************************************************************/

static uint32_t spi_flash_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;
  uint32_t ret;

  if (priv->selected)
    {
      spi_flash_writeword(priv, wd);
      ret = spi_flash_readword(priv);
    }
  else
    {
      ret = 0xff;
    }

  return ret;
}

/************************************************************************************
 * Name: spi_flash_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_flash_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p rxbuffer=%p nwords=%zu\n", txbuffer, rxbuffer, nwords);

  /* 8-bit mode */

  FAR const uint8_t *src  = (FAR const uint8_t *)txbuffer;
  FAR uint8_t *dest = (FAR uint8_t *)rxbuffer;
  uint8_t word;

  while (nwords-- > 0)
    {
      /* Get the next word to write.  Is there a source buffer? */

      if (src)
        {
          word = *src++;
        }
      else
        {
          word = 0xff;
        }

      /* Exchange one word */

      word = (uint8_t)spi_flash_send(dev, (uint16_t)word);

      /* Is there a buffer to receive the return value? */

      if (dest)
        {
          *dest++ = word;
        }
    }
}

/************************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_flash_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_flash_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/************************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_flash_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_flash_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/************************************************************************************
 * Name: spi_flash_sectorerase
 *
 * Description:
 *   Erase one sector
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_flash_sectorerase(FAR struct spi_flash_dev_s *priv)
{
  uint32_t  address;
  uint32_t  len = 0;

  /* Ensure the WREN bit is set before any erase operation */

  if (priv->wren)
    {
      address = priv->address;
      if (priv->last_cmd == SPI_FLASH_SE)
        {
          address &= CONFIG_SPI_FLASH_SECTORSIZE_MASK;
          len = CONFIG_SPI_FLASH_SECTORSIZE;
        }
      else if (priv->last_cmd == SPI_FLASH_SSE)
        {
          address &= CONFIG_SPI_FLASH_SUBSECTORSIZE_MASK;
          len = CONFIG_SPI_FLASH_SUBSECTORSIZE;
        }

      /* Now perform the erase */

      memset(&priv->data[address], 0xff, len);
    }
}

/************************************************************************************
 * Name: spi_flash_writeword
 *
 * Description:
 *   Write a word (byte in our case) to the FLASH state machine.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   data     - the data to send to the simulated FLASH
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_flash_writeword(FAR struct spi_flash_dev_s *priv, uint16_t data)
{
  switch (priv->state)
    {
      case SPI_FLASH_STATE_IDLE:
        priv->last_cmd = data;
        priv->read_data = 0xff;
        switch (data)
          {
            case SPI_FLASH_RDID:
              priv->state = SPI_FLASH_STATE_RDID1;
              break;

            case SPI_FLASH_WREN:
              priv->wren = 1;
              priv->state = SPI_FLASH_STATE_WREN;
              break;

            case SPI_FLASH_RDSR:
              priv->state = SPI_FLASH_STATE_RDSR;
              break;

            /* Sector / Subsector erase */

            case SPI_FLASH_SE:
            case SPI_FLASH_SSE:
              priv->state = SPI_FLASH_STATE_SE1;
              break;

            /* Bulk Erase */

            case SPI_FLASH_BE:
              priv->state = SPI_FLASH_STATE_IDLE;
              if (priv->wren)
                {
                  memset(priv->data, 0xff, CONFIG_SPI_FLASH_SIZE);
                }
              break;

            case SPI_FLASH_PP:
              priv->state = SPI_FLASH_STATE_PP1;
              break;

            case SPI_FLASH_READ:
            case SPI_FLASH_FAST_READ:
              priv->state = SPI_FLASH_STATE_READ1;
              break;

            default:
              break;
          }
        break;

      /* Read ID States */

      case SPI_FLASH_STATE_RDID1:
        priv->read_data = priv->manuf;    /* CONFIG_SPI_FLASH_MANUFACTURER; */
        priv->state = SPI_FLASH_STATE_RDID2;
        break;

      case SPI_FLASH_STATE_RDID2:
        priv->read_data = priv->type;     /* CONFIG_SPI_FLASH_MEMORY_TYPE; */
        priv->state = SPI_FLASH_STATE_RDID3;
        break;

      case SPI_FLASH_STATE_RDID3:
        priv->read_data = priv->capacity; /* CONFIG_SPI_FLASH_CAPACITY; */
        priv->state = SPI_FLASH_STATE_IDLE;
        break;

      /* WREN state - if we receive any bytes here, then we abort the WREN */

      case SPI_FLASH_STATE_WREN:
        priv->wren = 0;
        break;

      /* Read Status Register state */

      case SPI_FLASH_STATE_RDSR:
        priv->read_data = 0;
        priv->state = SPI_FLASH_STATE_IDLE;
        break;

      /* Sector and Sub-Sector erase states - Read the address */

      case SPI_FLASH_STATE_SE1:
        priv->address = data << 16;
        priv->state = SPI_FLASH_STATE_SE2;
        break;

      case SPI_FLASH_STATE_SE2:
        priv->address |= data << 8;
        priv->state = SPI_FLASH_STATE_SE3;
        break;

      case SPI_FLASH_STATE_SE3:
        priv->address |= data;

        /* Now perform the sector or sub-sector erase.  Really this should
         * be done during the deselect, but this is just a simulation .
         */

        spi_flash_sectorerase(priv);
        break;

      /* Page Program.  We could reuse the SE states, but let's keep it clean. */

      case SPI_FLASH_STATE_PP1:
        priv->address = data << 16;
        priv->state = SPI_FLASH_STATE_PP2;
        break;

      case SPI_FLASH_STATE_PP2:
        priv->address |= data << 8;
        priv->state = SPI_FLASH_STATE_PP3;
        break;

      case SPI_FLASH_STATE_PP3:
        priv->address |= data;
        priv->state = SPI_FLASH_STATE_PP4;
        break;

      case SPI_FLASH_STATE_PP4:

        /* In this state we actually write data (if WREN enabled) */

        if (priv->wren)
          {
            priv->data[priv->address] = data;
          }

        /* Now increment the address.  We do a page wrap here to simulate
         * the actual FLASH.
         */

        if ((priv->address & CONFIG_SPI_FLASH_PAGESIZE_MASK) ==
              CONFIG_SPI_FLASH_PAGESIZE_MASK)
          {
            priv->address &= !CONFIG_SPI_FLASH_PAGESIZE_MASK;
          }
        else
          {
            priv->address++;
          }
        break;

      /* Read data */

      case SPI_FLASH_STATE_READ1:
        priv->address = data << 16;
        priv->state = SPI_FLASH_STATE_READ2;
        break;

      case SPI_FLASH_STATE_READ2:
        priv->address |= data << 8;
        priv->state = SPI_FLASH_STATE_READ3;
        break;

      case SPI_FLASH_STATE_READ3:
        priv->address |= data;
        if (priv->last_cmd == SPI_FLASH_FAST_READ)
          {
            priv->state = SPI_FLASH_STATE_FREAD_WAIT;
          }
        else
          {
            priv->state = SPI_FLASH_STATE_READ4;
          }
        break;

      case SPI_FLASH_STATE_FREAD_WAIT:
        priv->read_data = 0xff;
        priv->state = SPI_FLASH_STATE_READ4;
        break;

      case SPI_FLASH_STATE_READ4:

        /* In this state perform data reads until de-selected. */

        priv->read_data = priv->data[priv->address++];
        if (priv->address == CONFIG_SPI_FLASH_SIZE)
          {
            priv->address = 0;
          }
        break;

      default:
        priv->state = SPI_FLASH_STATE_IDLE;
        priv->read_data = 0xff;
        break;
    }
}

/************************************************************************************
 * Name: spi_flash_readword
 *
 * Description:
 *   Read a word (byte in our case) from the simulated FLASH.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte read from the simulated FLASH device
 *
 ************************************************************************************/

static uint32_t spi_flash_readword(FAR struct spi_flash_dev_s *priv)
{
  return priv->read_data;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_spi_flashinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *spi_flash_initialize(FAR const char *name)
{
  FAR struct spi_flash_dev_s *priv = NULL;
  int  x;

  /* Default to custom FLASH if not specified */

  if (name == NULL)
    {
      name = "custom";
    }

  /* Loop through all supported flash devices */

  for (x = 0; gp_spidev[x] != NULL; x++)
    {
      /* Search for the specified flash by name */

      if (strcmp(name, gp_spidev[x]->name) == 0)
        {
          break;
        }
    }

  /* Test if flash device found */

  if (gp_spidev[x] == NULL)
    {
      /* Specified device not supported */

      return NULL;
    }

  /* Configure the selected flash device */

  priv = gp_spidev[x];
  priv->selected = 0;
  priv->wren = 0;
  priv->address = 0;
  priv->state = SPI_FLASH_STATE_IDLE;
  priv->read_data = 0xff;
  priv->last_cmd = 0xff;
  memset(&priv->data[0], 0xff, sizeof(priv->data));

  return (FAR struct spi_dev_s *)priv;
}
