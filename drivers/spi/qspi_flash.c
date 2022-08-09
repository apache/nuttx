/****************************************************************************
 * drivers/spi/qspi_flash.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spi/qspi_flash.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Define the FLASH SIZE in bytes */

#ifdef CONFIG_QSPI_FLASH_1M
#  define CONFIG_QSPI_FLASH_SIZE        (128 * 1024)
#  define CONFIG_QSPI_FLASH_CAPACITY    0x11

#ifndef CONFIG_QSPI_FLASH_SECTORSIZE
#  define CONFIG_QSPI_FLASH_SECTORSIZE  2048
#endif

#endif

#ifdef CONFIG_QSPI_FLASH_8M
#  define CONFIG_QSPI_FLASH_SIZE        (1024 * 1024)
#  define CONFIG_QSPI_FLASH_CAPACITY    0x14
#endif

#ifdef CONFIG_QSPI_FLASH_32M
#  define CONFIG_QSPI_FLASH_SIZE        (4 * 1024 * 1024)
#  define CONFIG_QSPI_FLASH_CAPACITY    0x16
#endif

#ifdef CONFIG_QSPI_FLASH_64M
#  define CONFIG_QSPI_FLASH_SIZE        (8 * 1024 * 1024)
#  define CONFIG_QSPI_FLASH_CAPACITY    0x17
#endif

#ifdef CONFIG_QSPI_FLASH_128M
#  define CONFIG_QSPI_FLASH_SIZE        (16 * 1024 * 1024)
#  define CONFIG_QSPI_FLASH_CAPACITY    0x18
#endif

#ifndef CONFIG_QSPI_FLASH_MANUFACTURER
#  define CONFIG_QSPI_FLASH_MANUFACTURER 0x20
#endif

#ifndef CONFIG_QSPI_FLASH_MEMORY_TYPE
#  define CONFIG_QSPI_FLASH_MEMORY_TYPE 0xba
#endif

#ifndef CONFIG_QSPI_FLASH_SECTORSIZE
#  define CONFIG_QSPI_FLASH_SECTORSIZE  65536
#endif

#ifndef CONFIG_QSPI_FLASH_SUBSECTORSIZE
#  define CONFIG_QSPI_FLASH_SUBSECTORSIZE 4096
#endif

#ifndef CONFIG_QSPI_FLASH_SECTORSIZE_MASK
#  define CONFIG_QSPI_FLASH_SECTORSIZE_MASK (~(CONFIG_QSPI_FLASH_SECTORSIZE-1))
#endif

#ifndef CONFIG_QSPI_FLASH_SUBSECTORSIZE_MASK
#  define CONFIG_QSPI_FLASH_SUBSECTORSIZE_MASK (~(CONFIG_QSPI_FLASH_SUBSECTORSIZE-1))
#endif

#ifndef CONFIG_QSPI_FLASH_PAGESIZE
#  define CONFIG_QSPI_FLASH_PAGESIZE    256
#endif

#ifndef CONFIG_QSPI_FLASH_PAGESIZE_MASK
#  define CONFIG_QSPI_FLASH_PAGESIZE_MASK (CONFIG_QSPI_FLASH_PAGESIZE-1)
#endif

/* Define FLASH States */

#define QSPI_FLASH_STATE_IDLE         0
#define QSPI_FLASH_STATE_RDID1        1
#define QSPI_FLASH_STATE_RDID2        2
#define QSPI_FLASH_STATE_RDID3        3
#define QSPI_FLASH_STATE_WREN         4
#define QSPI_FLASH_STATE_RDSR         5
#define QSPI_FLASH_STATE_SE1          6
#define QSPI_FLASH_STATE_SE2          7
#define QSPI_FLASH_STATE_SE3          8
#define QSPI_FLASH_STATE_PP1          9
#define QSPI_FLASH_STATE_PP2          10
#define QSPI_FLASH_STATE_PP3          11
#define QSPI_FLASH_STATE_PP4          12
#define QSPI_FLASH_STATE_READ1        13
#define QSPI_FLASH_STATE_READ2        14
#define QSPI_FLASH_STATE_READ3        15
#define QSPI_FLASH_STATE_READ4        16
#define QSPI_FLASH_STATE_FREAD_WAIT   17

/* Instructions */

/*      Command            Value N Description             Addr Dummy Data */

#define QSPI_FLASH_WREN      0x06 /* 1 Write Enable           0   0   0     */
#define QSPI_FLASH_WRDI      0x04 /* 1 Write Disable          0   0   0     */
#define QSPI_FLASH_RDID      0x9f /* 1 Read Identification    0   0   1-3   */
#define QSPI_FLASH_RDSR      0x05 /* 1 Read Status Register   0   0   >=1   */
#define QSPI_FLASH_WRSR      0x01 /* 1 Write Status Register  0   0   1     */
#define QSPI_FLASH_READ      0x03 /* 1 Read Data Bytes        3   0   >=1   */
#define QSPI_FLASH_FAST_READ 0x0b /* 1 Higher speed read      3   1   >=1   */
#define QSPI_FLASH_PP        0x02 /* 1 Page Program           3   0   1-256 */
#define QSPI_FLASH_SE        0xd8 /* 1 Sector Erase           3   0   0     */
#define QSPI_FLASH_BE        0xc7 /* 1 Bulk Erase             0   0   0     */
#define QSPI_FLASH_DP        0xb9 /* 2 Deep power down        0   0   0     */
#define QSPI_FLASH_RES       0xab /* 2 Read Electronic
                                  *          Signature       0   3   >=1   */
#define QSPI_FLASH_SSE       0x20 /* 3 Sub-Sector Erase       0   0   0     */

#define QSPI_FLASH_ID        0x9f /* JEDEC ID */
#define QSPI_FLASH_READ_QUAD 0xeb

#define QSPI_FLASH_DUMMY     0xa5

#define QSPI_FLASH_WREN_SET  0x02

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct qspi_flashdev_s
{
  struct qspi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t          selected;   /* SPIn base address */
  int               wren;
  int               state;
  uint16_t          read_data;
  uint8_t           last_cmd;
  unsigned long     address;
  unsigned char     data[CONFIG_QSPI_FLASH_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* QSPI methods */

static int       qspi_flash_lock(FAR struct qspi_dev_s *dev, bool lock);
static uint32_t  qspi_flash_setfrequency(FAR struct qspi_dev_s *dev,
                                         uint32_t frequency);
static void      qspi_flash_setmode(FAR struct qspi_dev_s *dev,
                                    enum qspi_mode_e mode);
static void      qspi_flash_setbits(FAR struct qspi_dev_s *dev, int nbits);
static int       qspi_flash_command(FAR struct qspi_dev_s *dev,
                                    FAR struct qspi_cmdinfo_s *cmd);
static int       qspi_flash_memory(FAR struct qspi_dev_s *dev,
                                   FAR struct qspi_meminfo_s *mem);
static FAR void *qspi_flash_alloc(FAR struct qspi_dev_s *dev,
                                  size_t buflen);
static void      qspi_flash_free(FAR struct qspi_dev_s *dev,
                                 FAR void *buffer);

static void qspi_flash_writeword(FAR struct qspi_flashdev_s *priv,
                                 uint16_t data,
                                 FAR struct qspi_cmdinfo_s *cmdinfo);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_qspiops =
{
  qspi_flash_lock,              /* lock */
  qspi_flash_setfrequency,      /* setfrequency */
  qspi_flash_setmode,           /* setmode */
  qspi_flash_setbits,           /* setbits */
  qspi_flash_command,           /* command */
  qspi_flash_memory,            /* memory */
  qspi_flash_alloc,             /* alloc */
  qspi_flash_free               /* free */
};

struct qspi_flashdev_s g_qspidev =
{
  {
    &g_qspiops
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qspi_flash_lock
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
 ****************************************************************************/

static int qspi_flash_lock(FAR struct qspi_dev_s *dev, bool lock)
{
  return OK;
}

/****************************************************************************
 * Name: qspi_flash_memory
 *
 * Description:
 *   Perform QSPI Memory transaction operations
 *
 * Returned Value:
 *   Always returns zero
 *
 ****************************************************************************/

int qspi_flash_memory(FAR struct qspi_dev_s *dev,
                      FAR struct qspi_meminfo_s *mem)
{
  FAR struct qspi_flashdev_s *priv = (FAR struct qspi_flashdev_s *)dev;

  switch (mem->cmd)
  {
    case QSPI_FLASH_READ_QUAD:
      priv->wren = 0;
      memcpy(mem->buffer, &priv->data[mem->addr], mem->buflen);
      priv->address += mem->addr + mem->buflen;
      priv->state = QSPI_FLASH_STATE_IDLE;
      break;

    case QSPI_FLASH_PP:
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

/****************************************************************************
 * Name: qspi_flash_setfrequency
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
 ****************************************************************************/

static uint32_t qspi_flash_setfrequency(FAR struct qspi_dev_s *dev,
                                        uint32_t frequency)
{
  return frequency;
}

/****************************************************************************
 * Name: qspi_flash_setmode
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
 ****************************************************************************/

static void qspi_flash_setmode(FAR struct qspi_dev_s *dev,
                               enum qspi_mode_e mode)
{
}

/****************************************************************************
 * Name: qspi_flash_setbits
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
 ****************************************************************************/

static void qspi_flash_setbits(FAR struct qspi_dev_s *dev, int nbits)
{
}

/****************************************************************************
 * Name: qspi_flash_alloc
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
 ****************************************************************************/

static FAR void *qspi_flash_alloc(FAR struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_malloc(buflen);
}

/****************************************************************************
 * Name: qspi_flash_free
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
 ****************************************************************************/

static void qspi_flash_free(FAR struct qspi_dev_s *dev, FAR void *buffer)
{
  kmm_free(buffer);
}

/****************************************************************************
 * Name: qspi_flash_sectorerase
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
 ****************************************************************************/

static void qspi_flash_sectorerase(FAR struct qspi_flashdev_s *priv)
{
  uint32_t address;
  uint32_t len = 0;

  /* Ensure the WREN bit is set before any erase operation */

  if (priv->wren)
    {
      address = priv->address;
      if (priv->last_cmd == QSPI_FLASH_SE)
        {
          address &= CONFIG_QSPI_FLASH_SECTORSIZE_MASK;
          len = CONFIG_QSPI_FLASH_SECTORSIZE;
        }
      else if (priv->last_cmd == QSPI_FLASH_SSE)
        {
          address &= CONFIG_QSPI_FLASH_SUBSECTORSIZE_MASK;
          len = CONFIG_QSPI_FLASH_SUBSECTORSIZE;
        }

      /* Now perform the erase */

      memset(&priv->data[address], 0xff, len);
    }
}

/****************************************************************************
 * Name: qspi_flash_writeword
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
 ****************************************************************************/

static void qspi_flash_writeword(FAR struct qspi_flashdev_s *priv,
                                 uint16_t data,
                                 FAR struct qspi_cmdinfo_s *cmdinfo)
{
  switch (priv->state)
    {
      case QSPI_FLASH_STATE_IDLE:
        priv->last_cmd = data;
        priv->read_data = 0xff;
        switch (data)
          {
            case QSPI_FLASH_WREN:
              priv->wren = 1;
              break;

            case QSPI_FLASH_WRDI:
              priv->wren = 0;
              break;

            /* Sector / Subsector erase */

            case QSPI_FLASH_SE:
            case QSPI_FLASH_SSE:
              priv->address = cmdinfo->addr;

              /* Now perform the sector or sub-sector erase.
               * Really this should be done during the deselect,
               * but this is just a simulation .
               */

              qspi_flash_sectorerase(priv);
              break;

            /* Bulk Erase */

            case QSPI_FLASH_BE:
              priv->state = QSPI_FLASH_STATE_IDLE;
              if (priv->wren)
                {
                  memset(priv->data, 0xff, CONFIG_QSPI_FLASH_SIZE);
                }
              break;

            default:
              break;
          }
        break;

      default:
        priv->state = QSPI_FLASH_STATE_IDLE;
        priv->read_data = 0xff;
        break;
    }
}

/****************************************************************************
 * Name: qspi_flash_command
 *
 * Description:
 *   Perform QSPI Command operations
 *
 * Returned Value:
 *   Always returns zero
 *
 ****************************************************************************/

static int qspi_flash_command(FAR struct qspi_dev_s *dev,
                              FAR struct qspi_cmdinfo_s *cmdinfo)
{
  FAR uint8_t *p_buf;
  FAR struct qspi_flashdev_s *priv = (FAR struct qspi_flashdev_s *)dev;

  DEBUGASSERT(cmdinfo->cmd < 256);

  /* Does data accompany the command? */

  if (QSPICMD_ISDATA(cmdinfo->flags))
    {
      DEBUGASSERT(cmdinfo->buffer != NULL && cmdinfo->buflen > 0);
      p_buf = (FAR uint8_t *)cmdinfo->buffer;

      /* Read or write operation? */

      if (QSPICMD_ISWRITE(cmdinfo->flags))
        {
          /* Write data operation */

          qspi_flash_writeword(priv, cmdinfo->cmd, cmdinfo);
        }
      else
        {
          /* Read data operation */

          switch (cmdinfo->cmd)
          {
            case QSPI_FLASH_ID:
              p_buf[0] = CONFIG_QSPI_FLASH_MANUFACTURER;
              p_buf[1] = CONFIG_QSPI_FLASH_MEMORY_TYPE;
              p_buf[2] = CONFIG_QSPI_FLASH_CAPACITY;
              break;

            case QSPI_FLASH_RDSR:
              p_buf[0] = priv->wren == 1 ? QSPI_FLASH_WREN_SET : 0;
              break;
          }
        }
    }
  else
    {
      /* Write data operation */

      qspi_flash_writeword(priv, cmdinfo->cmd, cmdinfo);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qspi_flash_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct qspi_dev_s *qspi_flash_initialize()
{
  FAR struct qspi_flashdev_s *priv = NULL;

  priv = &g_qspidev;
  priv->selected = 0;
  priv->wren = 0;
  priv->address = 0;
  priv->state = QSPI_FLASH_STATE_IDLE;
  priv->read_data = 0xff;
  priv->last_cmd = 0xff;
  memset(&priv->data[0], 0xff, sizeof(priv->data));

  return (FAR struct qspi_dev_s *)priv;
}
