/************************************************************************************
 * arch/sim/src/sim/up_qspiflash.c
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
#include <nuttx/kmalloc.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/qspi.h>

#include "up_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Define the FLASH SIZE in bytes */

#ifdef CONFIG_SIM_QSPIFLASH_1M
#  define CONFIG_QSPIFLASH_SIZE        (128 * 1024)
#  define CONFIG_QSPIFLASH_CAPACITY    0x11

#ifndef CONFIG_SIM_QSPIFLASH_SECTORSIZE
#  define CONFIG_SIM_QSPIFLASH_SECTORSIZE  2048
#endif

#endif

#ifdef CONFIG_SIM_QSPIFLASH_8M
#  define CONFIG_QSPIFLASH_SIZE        (1024 * 1024)
#  define CONFIG_QSPIFLASH_CAPACITY    0x14
#endif

#ifdef CONFIG_SIM_QSPIFLASH_32M
#  define CONFIG_QSPIFLASH_SIZE        (4 * 1024 * 1024)
#  define CONFIG_QSPIFLASH_CAPACITY    0x16
#endif

#ifdef CONFIG_SIM_QSPIFLASH_64M
#  define CONFIG_QSPIFLASH_SIZE        (8 * 1024 * 1024)
#  define CONFIG_QSPIFLASH_CAPACITY    0x17
#endif

#ifdef CONFIG_SIM_QSPIFLASH_128M
#  define CONFIG_QSPIFLASH_SIZE        (16 * 1024 * 1024)
#  define CONFIG_QSPIFLASH_CAPACITY    0x18
#endif

#ifndef CONFIG_SIM_QSPIFLASH_MANUFACTURER
#  define CONFIG_SIM_QSPIFLASH_MANUFACTURER  0x20
#endif

#ifndef CONFIG_SIM_QSPIFLASH_MEMORY_TYPE
#  define CONFIG_SIM_QSPIFLASH_MEMORY_TYPE  0xba
#endif

#ifndef CONFIG_SIM_QSPIFLASH_SECTORSIZE
#  define CONFIG_SIM_QSPIFLASH_SECTORSIZE  65536
#endif

#ifndef CONFIG_SIM_QSPIFLASH_SUBSECTORSIZE
#  define CONFIG_SIM_QSPIFLASH_SUBSECTORSIZE  4096
#endif

#ifndef CONFIG_SIM_QSPIFLASH_SECTORSIZE_MASK
#  define CONFIG_SIM_QSPIFLASH_SECTORSIZE_MASK  (~(CONFIG_SIM_QSPIFLASH_SECTORSIZE-1))
#endif

#ifndef CONFIG_SIM_QSPIFLASH_SUBSECTORSIZE_MASK
#  define CONFIG_SIM_QSPIFLASH_SUBSECTORSIZE_MASK  (~(CONFIG_SIM_QSPIFLASH_SUBSECTORSIZE-1))
#endif

#ifndef CONFIG_SIM_QSPIFLASH_PAGESIZE
#  define CONFIG_SIM_QSPIFLASH_PAGESIZE  256
#endif

#ifndef CONFIG_SIM_QSPIFLASH_PAGESIZE_MASK
#  define CONFIG_SIM_QSPIFLASH_PAGESIZE_MASK  (CONFIG_SIM_QSPIFLASH_PAGESIZE-1)
#endif

/* Define FLASH States */

#define QSPIFLASH_STATE_IDLE         0
#define QSPIFLASH_STATE_RDID1        1
#define QSPIFLASH_STATE_RDID2        2
#define QSPIFLASH_STATE_RDID3        3
#define QSPIFLASH_STATE_WREN         4
#define QSPIFLASH_STATE_RDSR         5
#define QSPIFLASH_STATE_SE1          6
#define QSPIFLASH_STATE_SE2          7
#define QSPIFLASH_STATE_SE3          8
#define QSPIFLASH_STATE_PP1          9
#define QSPIFLASH_STATE_PP2          10
#define QSPIFLASH_STATE_PP3          11
#define QSPIFLASH_STATE_PP4          12
#define QSPIFLASH_STATE_READ1        13
#define QSPIFLASH_STATE_READ2        14
#define QSPIFLASH_STATE_READ3        15
#define QSPIFLASH_STATE_READ4        16
#define QSPIFLASH_STATE_FREAD_WAIT   17

/* Instructions */

/*      Command            Value      N Description             Addr Dummy Data   */

#define QSPIFLASH_WREN      0x06    /* 1 Write Enable              0   0     0     */
#define QSPIFLASH_WRDI      0x04    /* 1 Write Disable             0   0     0     */
#define QSPIFLASH_RDID      0x9f    /* 1 Read Identification       0   0     1-3   */
#define QSPIFLASH_RDSR      0x05    /* 1 Read Status Register      0   0     >=1   */
#define QSPIFLASH_WRSR      0x01    /* 1 Write Status Register     0   0     1     */
#define QSPIFLASH_READ      0x03    /* 1 Read Data Bytes           3   0     >=1   */
#define QSPIFLASH_FAST_READ 0x0b    /* 1 Higher speed read         3   1     >=1   */
#define QSPIFLASH_PP        0x02    /* 1 Page Program              3   0     1-256 */
#define QSPIFLASH_SE        0xd8    /* 1 Sector Erase              3   0     0     */
#define QSPIFLASH_BE        0xc7    /* 1 Bulk Erase                0   0     0     */
#define QSPIFLASH_DP        0xb9    /* 2 Deep power down           0   0     0     */
#define QSPIFLASH_RES       0xab    /* 2 Read Electronic Signature 0   3     >=1   */
#define QSPIFLASH_SSE       0x20    /* 3 Sub-Sector Erase          0   0     0     */

#define QSPIFLASH_ID        0x9f    /* JEDEC ID */
#define QSPIFLASH_READ_QUAD 0xeb

#define QSPIFLASH_DUMMY     0xa5

#define QSPIFLASH_WREN_SET  0x02

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct sim_qspiflashdev_s
{
  struct qspi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         selected;    /* SPIn base address */
  int              wren;
  int              state;
  uint16_t         read_data;
  uint8_t          last_cmd;
  unsigned long    address;
  unsigned char    data[CONFIG_QSPIFLASH_SIZE];
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* QSPI methods */

static int         qspiflash_lock(FAR struct qspi_dev_s *dev, bool lock);
static uint32_t    qspiflash_setfrequency(FAR struct qspi_dev_s *dev,
                     uint32_t frequency);
static void        qspiflash_setmode(FAR struct qspi_dev_s *dev,
                     enum qspi_mode_e mode);
static void        qspiflash_setbits(FAR struct qspi_dev_s *dev, int nbits);
static int         qspiflash_command(FAR struct qspi_dev_s *dev,
                     FAR struct qspi_cmdinfo_s *cmd);
static int         qspiflash_memory(FAR struct qspi_dev_s *dev,
                     FAR struct qspi_meminfo_s *mem);
static FAR void   *qspiflash_alloc(FAR struct qspi_dev_s *dev, size_t buflen);
static void        qspiflash_free(FAR struct qspi_dev_s *dev, FAR void *buffer);

static void qspiflash_writeword(FAR struct sim_qspiflashdev_s *priv,
                    uint16_t data, FAR struct qspi_cmdinfo_s *cmdinfo);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct qspi_ops_s g_qspiops =
{
  .lock              = qspiflash_lock,
  .setfrequency      = qspiflash_setfrequency,
  .setmode           = qspiflash_setmode,
  .setbits           = qspiflash_setbits,
  .command           = qspiflash_command,
  .memory            = qspiflash_memory,
  .alloc             = qspiflash_alloc,
  .free              = qspiflash_free
};

struct sim_qspiflashdev_s g_qspidev =
{
  .spidev =
  {
    &g_qspiops
  }
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: qspiflash_lock
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

static int qspiflash_lock(FAR struct qspi_dev_s *dev, bool lock)
{
  return OK;
}

/************************************************************************************
 * Name: qspiflash_memory
 *
 * Description:
 *   Perform QSPI Memory transaction operations
 *
 * Returned Value:
 *   Always returns zero
 *
 ************************************************************************************/

int qspiflash_memory(FAR struct qspi_dev_s *dev, FAR struct qspi_meminfo_s *mem)
{
  FAR struct sim_qspiflashdev_s *priv = (FAR struct sim_qspiflashdev_s *)dev;

  switch (mem->cmd)
  {
    case QSPIFLASH_READ_QUAD:
      priv->wren = 0;
      memcpy(mem->buffer, &priv->data[mem->addr], mem->buflen);
      priv->address += mem->addr + mem->buflen;
      priv->state = QSPIFLASH_STATE_IDLE;
      break;

    case QSPIFLASH_PP:
      if (priv->wren)
        {
          memcpy(&priv->data[mem->addr], mem->buffer, mem->buflen);
        }
      break;

    default:
      return -EINVAL;
  }

  return 0;
}

/************************************************************************************
 * Name: qspiflash_setfrequency
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

static uint32_t qspiflash_setfrequency(FAR struct qspi_dev_s *dev, uint32_t frequency)
{
  return frequency;
}

/************************************************************************************
 * Name: qspiflash_setmode
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

static void qspiflash_setmode(FAR struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
}

/************************************************************************************
 * Name: qspiflash_setbits
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

static void qspiflash_setbits(FAR struct qspi_dev_s *dev, int nbits)
{
}

/************************************************************************************
 * Name: qspiflash_alloc
 *
 * Description:
 *   Allocate a buffer and associate it with the QSPI device
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   buflen - Length of buffer to allocate
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static FAR void *qspiflash_alloc(FAR struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_malloc(buflen);
}

/************************************************************************************
 * Name: qspiflash_free
 *
 * Description:
 *   Allocate a buffer and associate it with the QSPI device
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   buflen - Length of buffer to allocate
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void qspiflash_free(FAR struct qspi_dev_s *dev, FAR void *buffer)
{
  kmm_free(buffer);
}

/************************************************************************************
 * Name: qspiflash_sectorerase
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

static void qspiflash_sectorerase(FAR struct sim_qspiflashdev_s *priv)
{
  uint32_t  address;
  uint32_t  len;

  /* Ensure the WREN bit is set before any erase operation */

  if (priv->wren)
    {
      address = priv->address;
      if (priv->last_cmd == QSPIFLASH_SE)
        {
          address &= CONFIG_SIM_QSPIFLASH_SECTORSIZE_MASK;
          len = CONFIG_SIM_QSPIFLASH_SECTORSIZE;
        }
      else if (priv->last_cmd == QSPIFLASH_SSE)
        {
          address &= CONFIG_SIM_QSPIFLASH_SUBSECTORSIZE_MASK;
          len = CONFIG_SIM_QSPIFLASH_SUBSECTORSIZE;
        }

      /* Now perform the erase */

      memset(&priv->data[address], 0xff, len);
    }
}

/************************************************************************************
 * Name: qspiflash_writeword
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

static void qspiflash_writeword(FAR struct sim_qspiflashdev_s *priv, uint16_t data,
                                FAR struct qspi_cmdinfo_s *cmdinfo)
{
  switch (priv->state)
    {
      case QSPIFLASH_STATE_IDLE:
        priv->last_cmd = data;
        priv->read_data = 0xff;
        switch (data)
          {
            case QSPIFLASH_WREN:
              priv->wren = 1;
              break;

            case QSPIFLASH_WRDI:
              priv->wren = 0;
              break;

            /* Sector / Subsector erase */

            case QSPIFLASH_SE:
            case QSPIFLASH_SSE:
              priv->address = cmdinfo->addr;

              /* Now perform the sector or sub-sector erase.  Really this should
               * be done during the deselect, but this is just a simulation .
               */

              qspiflash_sectorerase(priv);
              break;

            /* Bulk Erase */

            case QSPIFLASH_BE:
              priv->state = QSPIFLASH_STATE_IDLE;
              if (priv->wren)
                {
                  memset(priv->data, 0xff, CONFIG_QSPIFLASH_SIZE);
                }
              break;

            default:
              break;
          }
        break;

      default:
        priv->state = QSPIFLASH_STATE_IDLE;
        priv->read_data = 0xff;
        break;
    }
}

/************************************************************************************
 * Name: qspiflash_command
 *
 * Description:
 *   Perform QSPI Command operations
 *
 * Returned Value:
 *   Always returns zero
 *
 ************************************************************************************/

static int qspiflash_command(FAR struct qspi_dev_s *dev, FAR struct qspi_cmdinfo_s *cmdinfo)
{
  uint8_t  *pBuf;
  FAR struct sim_qspiflashdev_s *priv = (FAR struct sim_qspiflashdev_s *)dev;

  DEBUGASSERT(cmdinfo->cmd < 256);

  /* Does data accompany the command? */

  if (QSPICMD_ISDATA(cmdinfo->flags))
    {
      DEBUGASSERT(cmdinfo->buffer != NULL && cmdinfo->buflen > 0);
      pBuf = (uint8_t *) cmdinfo->buffer;

      /* Read or write operation? */

      if (QSPICMD_ISWRITE(cmdinfo->flags))
        {
          /* Write data operation */

          qspiflash_writeword(priv, cmdinfo->cmd, cmdinfo);
        }
      else
        {
          /* Read data operation */

          switch (cmdinfo->cmd)
          {
            case QSPIFLASH_ID:
              pBuf[0] = CONFIG_SIM_QSPIFLASH_MANUFACTURER;
              pBuf[1] = CONFIG_SIM_QSPIFLASH_MEMORY_TYPE;
              pBuf[2] = CONFIG_QSPIFLASH_CAPACITY;
              break;

            case QSPIFLASH_RDSR:
              if (priv->wren == 1)
                  pBuf[0] = QSPIFLASH_WREN_SET;
              else
                  pBuf[0] = 0;
              break;
          }
        }
    }
  else
    {
      /* Write data operation */

      qspiflash_writeword(priv, cmdinfo->cmd, cmdinfo);
    }

  return 0;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_qspiflashinitialize
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

FAR struct qspi_dev_s *up_qspiflashinitialize()
{
  FAR struct sim_qspiflashdev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

  priv = &g_qspidev;
  priv->selected = 0;
  priv->wren = 0;
  priv->address = 0;
  priv->state = QSPIFLASH_STATE_IDLE;
  priv->read_data = 0xff;
  priv->last_cmd = 0xff;
  memset(&priv->data[0], 0xff, sizeof(priv->data));

  leave_critical_section(flags);
  return (FAR struct qspi_dev_s *)priv;
}
