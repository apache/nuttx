/************************************************************************************
 * arch/sim/src/up_spiflash.c
 *
 *   Copyright (C) 2014, 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "up_internal.h"

#if defined(CONFIG_SIM_SPIFLASH)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

/* Define the FLASH SIZE in bytes */

#ifdef CONFIG_SIM_SPIFLASH_1M
#  define CONFIG_SPIFLASH_SIZE            (128 * 1024)
#  define CONFIG_SPIFLASH_CAPACITY        0x11

#ifndef CONFIG_SIM_SPIFLASH_SECTORSIZE
#  define CONFIG_SIM_SPIFLASH_SECTORSIZE  2048
#endif

#endif

#ifdef CONFIG_SIM_SPIFLASH_8M
#  define CONFIG_SPIFLASH_SIZE            (1024 * 1024)
#  define CONFIG_SPIFLASH_CAPACITY_SST26  0x3F
#  define CONFIG_SPIFLASH_CAPACITY        0x14
#endif

#ifdef CONFIG_SIM_SPIFLASH_32M
#  define CONFIG_SPIFLASH_SIZE            (4 * 1024 * 1024)
#  define CONFIG_SPIFLASH_CAPACITY_SST26  0x42
#  define CONFIG_SPIFLASH_CAPACITY        0x16
#endif

#ifdef CONFIG_SIM_SPIFLASH_64M
#  define CONFIG_SPIFLASH_SIZE            (8 * 1024 * 1024)
#  define CONFIG_SPIFLASH_CAPACITY_SST26  0x43
#  define CONFIG_SPIFLASH_CAPACITY        0x17
#endif

#ifdef CONFIG_SIM_SPIFLASH_128M
#  define CONFIG_SPIFLASH_SIZE            (16 * 1024 * 1024)
#  define CONFIG_SPIFLASH_CAPACITY_SST26  0x44
#  define CONFIG_SPIFLASH_CAPACITY        0x18
#endif

#ifndef CONFIG_SIM_SPIFLASH_MANUFACTURER
#  define CONFIG_SIM_SPIFLASH_MANUFACTURER  0x20
#endif

#ifndef CONFIG_SIM_SPIFLASH_MEMORY_TYPE
#  define CONFIG_SIM_SPIFLASH_MEMORY_TYPE   0x20
#endif

#ifndef CONFIG_SIM_SPIFLASH_SECTORSIZE
#  define CONFIG_SIM_SPIFLASH_SECTORSIZE    65536
#endif

#ifndef CONFIG_SIM_SPIFLASH_SUBSECTORSIZE
#  define CONFIG_SIM_SPIFLASH_SUBSECTORSIZE    4096
#endif

#ifndef CONFIG_SIM_SPIFLASH_SECTORSIZE_MASK
#  define CONFIG_SIM_SPIFLASH_SECTORSIZE_MASK  (~(CONFIG_SIM_SPIFLASH_SECTORSIZE-1))
#endif

#ifndef CONFIG_SIM_SPIFLASH_SUBSECTORSIZE_MASK
#  define CONFIG_SIM_SPIFLASH_SUBSECTORSIZE_MASK  (~(CONFIG_SIM_SPIFLASH_SUBSECTORSIZE-1))
#endif

#ifndef CONFIG_SIM_SPIFLASH_PAGESIZE
#  define CONFIG_SIM_SPIFLASH_PAGESIZE    256
#endif

#ifndef CONFIG_SIM_SPIFLASH_PAGESIZE_MASK
#  define CONFIG_SIM_SPIFLASH_PAGESIZE_MASK  (CONFIG_SIM_SPIFLASH_PAGESIZE-1)
#endif

/* Define FLASH States */

#define SPIFLASH_STATE_IDLE         0
#define SPIFLASH_STATE_RDID1        1
#define SPIFLASH_STATE_RDID2        2
#define SPIFLASH_STATE_RDID3        3
#define SPIFLASH_STATE_WREN         4
#define SPIFLASH_STATE_RDSR         5
#define SPIFLASH_STATE_SE1          6
#define SPIFLASH_STATE_SE2          7
#define SPIFLASH_STATE_SE3          8
#define SPIFLASH_STATE_PP1          9
#define SPIFLASH_STATE_PP2          10
#define SPIFLASH_STATE_PP3          11
#define SPIFLASH_STATE_PP4          12
#define SPIFLASH_STATE_READ1        13
#define SPIFLASH_STATE_READ2        14
#define SPIFLASH_STATE_READ3        15
#define SPIFLASH_STATE_READ4        16
#define SPIFLASH_STATE_FREAD_WAIT   17

/* Instructions */
/*      Command            Value      N Description             Addr Dummy Data   */
#define SPIFLASH_WREN      0x06    /* 1 Write Enable              0   0     0     */
#define SPIFLASH_WRDI      0x04    /* 1 Write Disable             0   0     0     */
#define SPIFLASH_RDID      0x9f    /* 1 Read Identification       0   0     1-3   */
#define SPIFLASH_RDSR      0x05    /* 1 Read Status Register      0   0     >=1   */
#define SPIFLASH_WRSR      0x01    /* 1 Write Status Register     0   0     1     */
#define SPIFLASH_READ      0x03    /* 1 Read Data Bytes           3   0     >=1   */
#define SPIFLASH_FAST_READ 0x0b    /* 1 Higher speed read         3   1     >=1   */
#define SPIFLASH_PP        0x02    /* 1 Page Program              3   0     1-256 */
#define SPIFLASH_SE        0xd8    /* 1 Sector Erase              3   0     0     */
#define SPIFLASH_BE        0xc7    /* 1 Bulk Erase                0   0     0     */
#define SPIFLASH_DP        0xb9    /* 2 Deep power down           0   0     0     */
#define SPIFLASH_RES       0xab    /* 2 Read Electronic Signature 0   3     >=1   */
#define SPIFLASH_SSE       0x20    /* 3 Sub-Sector Erase          0   0     0     */

#define SPIFLASH_DUMMY     0xa5

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct sim_spiflashdev_s
{
  struct spi_dev_s  spidev;     /* Externally visible part of the SPI interface */
  uint32_t          selected;   /* SPIn base address */
  FAR char *        name;       /* Name of the flash type (m25p, w25, etc.) */
  int               wren;
  int               state;
  uint16_t          read_data;
  uint8_t           last_cmd;
  uint8_t           capacity;
  uint8_t           manuf;
  uint8_t           type;
  unsigned long     address;
  unsigned char     data[CONFIG_SPIFLASH_SIZE];
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* SPI methods */

static int         spiflash_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t    spiflash_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void        spiflash_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void        spiflash_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t    spiflash_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void        spiflash_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
static void        spiflash_select(FAR struct spi_dev_s *dev, uint32_t devid,
                     bool selected);
static uint8_t     spiflash_status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int         spiflash_cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void        spiflash_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                size_t nwords);
static void        spiflash_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                 size_t nwords);
#endif

static void spiflash_writeword(FAR struct sim_spiflashdev_s *priv, uint16_t data);
static uint16_t spiflash_readword(FAR struct sim_spiflashdev_s *priv);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .lock              = spiflash_lock,
  .select            = spiflash_select,
  .setfrequency      = spiflash_setfrequency,
  .setmode           = spiflash_setmode,
  .setbits           = spiflash_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                   /* Not supported */
#endif
  .status            = spiflash_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = spiflash_cmddata,
#endif
  .send              = spiflash_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spiflash_exchange,
#else
  .sndblock          = spiflash_sndblock,
  .recvblock         = spiflash_recvblock,
#endif
  .registercallback  = 0,
};

#ifdef CONFIG_SIM_SPIFLASH_M25P
struct sim_spiflashdev_s g_spidev_m25p =
{
  .spidev   = { &g_spiops },
  .name     = "m25p",
  .manuf    = 0x20,
  .type     = 0x20,
  .capacity = CONFIG_SPIFLASH_CAPACITY
};
#endif

#ifdef CONFIG_SIM_SPIFLASH_SST26
struct sim_spiflashdev_s g_spidev_sst26 =
{
  .spidev   = { &g_spiops },
  .name     = "sst26",
  .manuf    = 0xBF,
#ifdef CONFIG_SST26_MEMORY_TYPE
  .type     = CONFIG_SST26_MEMORY_TYPE,
#else
  .type     = 0x25,
#endif
  .capacity = CONFIG_SPIFLASH_CAPACITY_SST26
};
#endif

#ifdef CONFIG_SIM_SPIFLASH_W25
struct sim_spiflashdev_s g_spidev_w25 =
{
  .spidev   = { &g_spiops },
  .name     = "w25",
  .manuf    = 0xef,
  .type     = 0x30,
  .capacity = CONFIG_SPIFLASH_CAPACITY
};
#endif

#ifdef CONFIG_SIM_SPIFLASH_CUSTOM
struct sim_spiflashdev_s g_spidev_custom =
{
  .spidev   = { &g_spiops },
  .name     = "custom",
  .manuf    = CONFIG_SIM_SPIFLASH_MANUFACTURER,
  .type     = CONFIG_SIM_SPIFLASH_MEMORY_TYPE,
  .capacity = CONFIG_SIM_SPIFLASH_CAPACITY
};
#endif

struct sim_spiflashdev_s *gp_spidev[] =
{
#ifdef CONFIG_SIM_SPIFLASH_M25P
  &g_spidev_m25p,
#endif
#ifdef CONFIG_SIM_SPIFLASH_SST26
  &g_spidev_sst26,
#endif
#ifdef CONFIG_SIM_SPIFLASH_W25
  &g_spidev_w25,
#endif
#ifdef CONFIG_SIM_SPIFLASH_CUSTOM
  &g_spidev_custom,
#endif

  /* Null termination pointer at end of list */

  NULL
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: spiflash_lock
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
 ************************************************************************************/

static int spiflash_lock(FAR struct spi_dev_s *dev, bool lock)
{
  return OK;
}

/************************************************************************************
 * Name: spiflash_select
 *
 * Description:
 *   Process select logic for the FLASH.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spiflash_select(FAR struct spi_dev_s *dev, uint32_t devid,
                            bool selected)
{
  FAR struct sim_spiflashdev_s *priv = (FAR struct sim_spiflashdev_s *)dev;

  if (devid == SPIDEV_FLASH(0))
    {
      priv->selected = selected;

      /* As part of an de-select, ensure the WREN bit is cleared */

      if (!selected)
        {
          if (priv->last_cmd != SPIFLASH_WREN)
            {
              priv->wren = 0;
            }

          priv->state = SPIFLASH_STATE_IDLE;
        }
    }
}

/************************************************************************************
 * Name: spiflash_cmddata
 *
 * Description:
 *   Perform SPI Command operations
 *
 * Returned Value:
 *   Always returns zero
 *
 ************************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spiflash_cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif

/************************************************************************************
 * Name: spiflash_setfrequency
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

static uint32_t spiflash_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  return frequency;
}

/************************************************************************************
 * Name: spiflash_setmode
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

static void spiflash_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
}

/************************************************************************************
 * Name: spiflash_setbits
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

static void spiflash_setbits(FAR struct spi_dev_s *dev, int nbits)
{
}

/************************************************************************************
 * Name: spiflash_status
 *
 * Description:
 *   Set the SPI bus status
 *
 * Returned Value:
 *   Always returns zero
 *
 ************************************************************************************/

static uint8_t spiflash_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/************************************************************************************
 * Name: spiflash_send
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

static uint16_t spiflash_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct sim_spiflashdev_s *priv = (FAR struct sim_spiflashdev_s *)dev;
  uint16_t ret;

  if (priv->selected)
    {
      spiflash_writeword(priv, wd);
      ret = spiflash_readword(priv);
    }
  else
    {
      ret = 0xff;
    }

  return ret;
}

/************************************************************************************
 * Name: spiflash_exchange (no DMA).  aka spi_exchange_nodma
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

static void spiflash_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                              FAR void *rxbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

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

      word = (uint8_t)spiflash_send(dev, (uint16_t)word);

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
static void spiflash_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spiflash_exchange(dev, txbuffer, NULL, nwords);
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
static void spiflash_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                               size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spiflash_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/************************************************************************************
 * Name: spiflash_sectorerase
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

static void spiflash_sectorerase(FAR struct sim_spiflashdev_s *priv)
{
  uint32_t  address;
  uint32_t  len;

  /* Ensure the WREN bit is set before any erase operation */

  if (priv->wren)
    {
      address = priv->address;
      if (priv->last_cmd == SPIFLASH_SE)
        {
          address &= CONFIG_SIM_SPIFLASH_SECTORSIZE_MASK;
          len = CONFIG_SIM_SPIFLASH_SECTORSIZE;
        }
      else if (priv->last_cmd == SPIFLASH_SSE)
        {
          address &= CONFIG_SIM_SPIFLASH_SUBSECTORSIZE_MASK;
          len = CONFIG_SIM_SPIFLASH_SUBSECTORSIZE;
        }

      /* Now perform the erase */

      memset(&priv->data[address], 0xFF, len);
    }
}

/************************************************************************************
 * Name: spiflash_writeword
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

static void spiflash_writeword(FAR struct sim_spiflashdev_s *priv, uint16_t data)
{
  switch (priv->state)
    {
      case SPIFLASH_STATE_IDLE:
        priv->last_cmd = data;
        priv->read_data = 0xff;
        switch (data)
          {
            case SPIFLASH_RDID:
              priv->state = SPIFLASH_STATE_RDID1;
              break;

            case SPIFLASH_WREN:
              priv->wren = 1;
              priv->state = SPIFLASH_STATE_WREN;
              break;

            case SPIFLASH_RDSR:
              priv->state = SPIFLASH_STATE_RDSR;
              break;

            /* Sector / Subsector erase */

            case SPIFLASH_SE:
            case SPIFLASH_SSE:
              priv->state = SPIFLASH_STATE_SE1;
              break;

            /* Bulk Erase */

            case SPIFLASH_BE:
              priv->state = SPIFLASH_STATE_IDLE;
              if (priv->wren)
                {
                  memset(priv->data, 0xff, CONFIG_SPIFLASH_SIZE);
                }
              break;

            case SPIFLASH_PP:
              priv->state = SPIFLASH_STATE_PP1;
              break;

            case SPIFLASH_READ:
            case SPIFLASH_FAST_READ:
              priv->state = SPIFLASH_STATE_READ1;
              break;

            default:
              break;
          }
        break;

      /* Read ID States */

      case SPIFLASH_STATE_RDID1:
        priv->read_data = priv->manuf;    /* CONFIG_SIM_SPIFLASH_MANUFACTURER; */
        priv->state = SPIFLASH_STATE_RDID2;
        break;

      case SPIFLASH_STATE_RDID2:
        priv->read_data = priv->type;     /* CONFIG_SIM_SPIFLASH_MEMORY_TYPE; */
        priv->state = SPIFLASH_STATE_RDID3;
        break;

      case SPIFLASH_STATE_RDID3:
        priv->read_data = priv->capacity; /* CONFIG_SPIFLASH_CAPACITY; */
        priv->state = SPIFLASH_STATE_IDLE;
        break;

      /* WREN state - if we receive any bytes here, then we abort the WREN */

      case SPIFLASH_STATE_WREN:
        priv->wren = 0;
        break;

      /* Read Status Register state */

      case SPIFLASH_STATE_RDSR:
        priv->read_data = 0;
        priv->state = SPIFLASH_STATE_IDLE;
        break;

      /* Sector and Sub-Sector erase states - Read the address */

      case SPIFLASH_STATE_SE1:
        priv->address = data << 16;
        priv->state = SPIFLASH_STATE_SE2;
        break;

      case SPIFLASH_STATE_SE2:
        priv->address |= data << 8;
        priv->state = SPIFLASH_STATE_SE3;
        break;

      case SPIFLASH_STATE_SE3:
        priv->address |= data;

        /* Now perform the sector or sub-sector erase.  Really this should
         * be done during the deselect, but this is just a simulation .
         */

        spiflash_sectorerase(priv);
        break;

      /* Page Program.  We could reuse the SE states, but let's keep it clean. */

      case SPIFLASH_STATE_PP1:
        priv->address = data << 16;
        priv->state = SPIFLASH_STATE_PP2;
        break;

      case SPIFLASH_STATE_PP2:
        priv->address |= data << 8;
        priv->state = SPIFLASH_STATE_PP3;
        break;

      case SPIFLASH_STATE_PP3:
        priv->address |= data;
        priv->state = SPIFLASH_STATE_PP4;
        break;

      case SPIFLASH_STATE_PP4:
        /* In this state we actually write data (if WREN enabled) */

        if (priv->wren)
          {
            priv->data[priv->address] = data;
          }

        /* Now increment the address.  We do a page wrap here to simulate
         * the actual FLASH.
         */

        if ((priv->address & CONFIG_SIM_SPIFLASH_PAGESIZE_MASK) ==
              CONFIG_SIM_SPIFLASH_PAGESIZE_MASK)
          {
            priv->address &= !CONFIG_SIM_SPIFLASH_PAGESIZE_MASK;
          }
        else
          {
            priv->address++;
          }
        break;

      /* Read data */

      case SPIFLASH_STATE_READ1:
        priv->address = data << 16;
        priv->state = SPIFLASH_STATE_READ2;
        break;

      case SPIFLASH_STATE_READ2:
        priv->address |= data << 8;
        priv->state = SPIFLASH_STATE_READ3;
        break;

      case SPIFLASH_STATE_READ3:
        priv->address |= data;
        if (priv->last_cmd == SPIFLASH_FAST_READ)
          {
            priv->state = SPIFLASH_STATE_FREAD_WAIT;
          }
        else
          {
            priv->state = SPIFLASH_STATE_READ4;
          }
        break;

      case SPIFLASH_STATE_FREAD_WAIT:
        priv->read_data = 0xff;
        priv->state = SPIFLASH_STATE_READ4;
        break;

      case SPIFLASH_STATE_READ4:
        /* In this state perform data reads until de-selected. */

        priv->read_data = priv->data[priv->address++];
        if (priv->address == CONFIG_SPIFLASH_SIZE)
          {
            priv->address = 0;
          }
        break;

      default:
        priv->state = SPIFLASH_STATE_IDLE;
        priv->read_data = 0xFF;
        break;
    }
}

/************************************************************************************
 * Name: spiflash_readword
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

static uint16_t spiflash_readword(FAR struct sim_spiflashdev_s *priv)
{
  return priv->read_data;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_spiflashinitialize
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

FAR struct spi_dev_s *up_spiflashinitialize(FAR const char *name)
{
  FAR struct sim_spiflashdev_s *priv = NULL;
  int  x;

  irqstate_t flags = enter_critical_section();

  /* Loop through all supported flash devices */

  /* Default to custom FLASH if not specified */

  if (name == NULL)
    {
      name = "custom";
    }

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
  priv->state = SPIFLASH_STATE_IDLE;
  priv->read_data = 0xFF;
  priv->last_cmd = 0xFF;
  memset(&priv->data[0], 0xFF, sizeof(priv->data));

  leave_critical_section(flags);
  return (FAR struct spi_dev_s *)priv;
}

#endif /* CONFIG_SIM_SPIFLASH */
