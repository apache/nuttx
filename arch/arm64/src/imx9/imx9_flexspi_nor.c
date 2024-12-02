/****************************************************************************
 * arch/arm64/src/imx9/imx9_flexspi_nor.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <sys/param.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/nuttx.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>
#include <arch/board/board.h>

#include "chip.h"
#include "imx9_flexspi.h"
#include "imx9_iomuxc.h"
#include "hardware/imx9_flexspi.h"
#include "hardware/imx9_pinmux.h"

#ifdef CONFIG_IMX9_FLEXSPI_NOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_M25P_SUBSECTOR_ERASE
#error This driver currently requires CONFIG_M25P_SUBSECTOR_ERASE
#endif

#  if !defined(ARMV8A_DCACHE_LINESIZE) || ARMV8A_DCACHE_LINESIZE == 0
#    undef ARMV8A_DCACHE_LINESIZE
#    define ARMV8A_DCACHE_LINESIZE 64
#  endif

/* Configuration ************************************************************/

/* Per the data sheet, M25P10 parts can be driven with either SPI mode 0
 * (CPOL=0 and CPHA=0) or mode 3 (CPOL=1 and CPHA=1). But I have heard that
 * other devices can operated in mode 0 or 1.
 * So you may need to specify CONFIG_M25P_SPIMODE to
 * select the best mode for your device.
 * If CONFIG_M25P_SPIMODE is not defined, mode 0 will be used.
 */

#ifndef CONFIG_M25P_SPIMODE
#  define CONFIG_M25P_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_M25P_SPIFREQUENCY
#  define CONFIG_M25P_SPIFREQUENCY 20000000
#endif

/* Various manufacturers may have produced the parts.
 * 0x20 is the manufacturer ID for the STMicro MP25x serial FLASH.
 * If, for example, you are using the a Macronix International MX25
 * serial FLASH, the correct manufacturer ID would be 0xc2.
 */

#ifndef CONFIG_M25P_MANUFACTURER
#  define CONFIG_M25P_MANUFACTURER 0x20
#endif

#ifndef CONFIG_M25P_MEMORY_TYPE
#  define CONFIG_M25P_MEMORY_TYPE  0x20
#endif

#ifndef CONFIG_MT25Q_MEMORY_TYPE
#  define CONFIG_MT25Q_MEMORY_TYPE  0xBA
#endif

#ifndef CONFIG_MT25QU_MEMORY_TYPE
#  define CONFIG_MT25QU_MEMORY_TYPE  0xBB
#endif

/* M25P Registers ***********************************************************/

/* Identification register values */

#define M25P_MANUFACTURER          CONFIG_M25P_MANUFACTURER
#define M25P_MEMORY_TYPE           CONFIG_M25P_MEMORY_TYPE
#define MT25Q_MEMORY_TYPE          CONFIG_MT25Q_MEMORY_TYPE
#define MT25QU_MEMORY_TYPE         CONFIG_MT25QU_MEMORY_TYPE
#define M25P_RES_ID                0x13
#define M25P_M25P1_CAPACITY        0x11 /* 1 M-bit */
#define M25P_EN25F80_CAPACITY      0x14 /* 8 M-bit */
#define M25P_M25P16_CAPACITY       0x15 /* 16 M-bit */
#define M25P_M25P32_CAPACITY       0x16 /* 32 M-bit */
#define M25P_M25P64_CAPACITY       0x17 /* 64 M-bit */
#define M25P_M25P128_CAPACITY      0x18 /* 128 M-bit */
#define M25P_MT25Q128_CAPACITY     0x18 /* 128 M-bit */
#define M25P_MT25Q256_CAPACITY     0x19 /* 256 M-bit */
#define M25P_MT25Q512_CAPACITY     0x20 /* 512 M-bit */
#define M25P_MT25Q1G_CAPACITY      0x21 /* 1 G-bit */

/*  M25P1 capacity is 131,072 bytes:
 *  (4 sectors) * (32,768 bytes per sector)
 *  (512 pages) * (256 bytes per page)
 */

#define M25P_M25P1_SECTOR_SHIFT    15    /* Sector size 1 << 15 = 65,536 */
#define M25P_M25P1_NSECTORS        4
#define M25P_M25P1_PAGE_SHIFT      8     /* Page size 1 << 8 = 256 */
#define M25P_M25P1_NPAGES          512

/*  EN25F80 capacity is 1,048,576 bytes:
 *  (16 sectors) * (65,536 bytes per sector)
 *  (512 pages) * (256 bytes per page)
 */

#define M25P_EN25F80_SECTOR_SHIFT  16    /* Sector size 1 << 15 = 65,536 */
#define M25P_EN25F80_NSECTORS      16
#define M25P_EN25F80_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define M25P_EN25F80_NPAGES        4096
#define M25P_EN25F80_SUBSECT_SHIFT 12   /* Sub-Sector size 1 << 12 = 4,096 */
#define M25P_EN25F80_NSUBSECTORS   256

/*  M25P16 capacity is 2,097,152 bytes:
 *  (32 sectors) * (65,536 bytes per sector)
 *  (8192 pages) * (256 bytes per page)
 */

#define M25P_M25P16_SECTOR_SHIFT   16    /* Sector size 1 << 16 = 65,536 */
#define M25P_M25P16_NSECTORS       32
#define M25P_M25P16_PAGE_SHIFT     8     /* Page size 1 << 8 = 256 */
#define M25P_M25P16_NPAGES         8192
#define M25P_M25PX16_SUBSECT_SHIFT 12   /* Sub-Sector size 1 << 12 = 4,096 */

/*  M25P32 capacity is 4,194,304 bytes:
 *  (64 sectors) * (65,536 bytes per sector)
 *  (16384 pages) * (256 bytes per page)
 */

#define M25P_M25P32_SECTOR_SHIFT   16    /* Sector size 1 << 16 = 65,536 */
#define M25P_M25P32_NSECTORS       64
#define M25P_M25P32_PAGE_SHIFT     8     /* Page size 1 << 8 = 256 */
#define M25P_M25P32_NPAGES         16384
#define M25P_M25PX32_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  M25P64 capacity is 8,338,608 bytes:
 *  (128 sectors) * (65,536 bytes per sector)
 *  (32768 pages) * (256 bytes per page)
 */

#define M25P_M25P64_SECTOR_SHIFT   16    /* Sector size 1 << 16 = 65,536 */
#define M25P_M25P64_NSECTORS       128
#define M25P_M25P64_PAGE_SHIFT     8     /* Page size 1 << 8 = 256 */
#define M25P_M25P64_NPAGES         32768

/*  M25P128 capacity is 16,777,216 bytes:
 *  (64 sectors) * (262,144 bytes per sector)
 *  (65536 pages) * (256 bytes per page)
 */

#define M25P_M25P128_SECTOR_SHIFT  18    /* Sector size 1 << 18 = 262,144 */
#define M25P_M25P128_NSECTORS      64
#define M25P_M25P128_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define M25P_M25P128_NPAGES        65536

/*  MT25Q128 capacity is 16,777,216 bytes:
 *  (256 sectors) * (65,536 bytes per sector)
 *  (65536 pages) * (256 bytes per page)
 */

#define M25P_MT25Q128_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define M25P_MT25Q128_NSECTORS      256
#define M25P_MT25Q128_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define M25P_MT25Q128_NPAGES        65536
#define M25P_MT25Q128_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  MT25Q256 capacity is 33,554,432 bytes:
 *  (512 sectors) * (65,536 bytes per sector)
 *  (131072 pages) * (256 bytes per page)
 */

#define M25P_MT25Q256_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define M25P_MT25Q256_NSECTORS      512
#define M25P_MT25Q256_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define M25P_MT25Q256_NPAGES        131072
#define M25P_MT25Q256_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  MT25Q512 capacity is 67,108,864 bytes:
 *  (1024 sectors) * (65,536 bytes per sector)
 *  (262144 pages) * (256 bytes per page)
 */

#define M25P_MT25Q512_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define M25P_MT25Q512_NSECTORS      1024
#define M25P_MT25Q512_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define M25P_MT25Q512_NPAGES        262144
#define M25P_MT25Q512_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  MT25Q1G capacity is 134,217,728  bytes:
 *  (2048 sectors) * (65,536 bytes per sector)
 *  (524288 pages) * (256 bytes per page)
 */

#define M25P_MT25Q1G_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define M25P_MT25Q1G_NSECTORS      2048
#define M25P_MT25Q1G_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define M25P_MT25Q1G_NPAGES        524288
#define M25P_MT25Q1G_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/* Instructions */

/*    Command          Value    N Description             Addr Dummy Data   */
#define M25P_WREN      0x06  /* 1 Write Enable              0   0     0     */
#define M25P_WRDI      0x04  /* 1 Write Disable             0   0     0     */
#define M25P_RDID      0x9f  /* 1 Read Identification       0   0     1-3   */
#define M25P_RDSR      0x05  /* 1 Read Status Register      0   0     >=1   */
#define M25P_WRSR      0x01  /* 1 Write Status Register     0   0     1     */
#define M25P_READ      0x03  /* 1 Read Data Bytes           3   0     >=1   */
#define M25P_FAST_READ 0x0b  /* 1 Higher speed read         3   1     >=1   */
#define M25P_PP        0x02  /* 1 Page Program              3   0     1-256 */
#define M25P_SE        0xd8  /* 1 Sector Erase              3   0     0     */
#define M25P_BE        0xc7  /* 1 Bulk Erase                0   0     0     */
#define M25P_DP        0xb9  /* 2 Deep power down           0   0     0     */
#define M25P_RES       0xab  /* 2 Read Electronic Signature 0   3     >=1   */
#define M25P_SSE       0x21  /* 3 Sub-Sector Erase          4   0     0     */
#define M25P_WECR      0x61  /* 1 Write Enhanched config    0   0     1     */
#define M25P_RFSR      0x70  /* 1 Read Flag Status Register 0   0     1     */
#define M25P_4B_ENTER  0xB7  /* 1 Enter 4byte addressing    0   0     0     */

/* Quad commands */
#define M25P_Q_FAST_RD 0x6c  /* 1 Quad output fast read     4   0     1-256 */
#define M25P_Q_FAST_PP 0x34  /* 1 Quad input fast program   4   0     1-256 */
#define M25P_Q_ENTER   0x35  /* 1 Enter Quad input/output   0   0     0     */

/* NOTE 1: All parts.
 * NOTE 2: M25P632/M25P64
 * NOTE 3: EN25F80.  In EN25F80 terminology, 0xd8 is a block erase and 0x20
 *         is a sector erase.
 */

#define ADDRESS_24BIT   0x18
#define ADDRESS_32BIT   0x20

enum
{
  /* SPI instructions */

  READ_ID,
  READ_STATUS_REG,
  READ_FLAG_STATUS_REG,
  WRITE_STATUS_REG,
  WRITE_ENABLE,
  ERASE_SECTOR,
  ERASE_CHIP,
  ENTER_DDR,
  READ_FAST,
  ENTER_4BYTE,

  /* Quad SPI instructions */

  READ_FAST_QUAD_OUTPUT,
  PAGE_PROGRAM_QUAD_INPUT,
  ENTER_QPI,
};

/* TODO: Re-define commands if using other than M25 NOR */

static const uint32_t g_flexspi_nor_lut[][4] =
{
  [READ_ID] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_RDID,
        FLEXSPI_COMMAND_READ_SDR, FLEXSPI_1PAD, 0x04),
  },

  [READ_STATUS_REG] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_RDSR,
        FLEXSPI_COMMAND_READ_SDR, FLEXSPI_1PAD, 0x04),
  },

  [READ_FLAG_STATUS_REG] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_RFSR,
        FLEXSPI_COMMAND_READ_SDR, FLEXSPI_1PAD, 0x04),
  },

  [WRITE_STATUS_REG] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_WRSR,
        FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_1PAD, 0x04),
  },

  [WRITE_ENABLE] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, M25P_WREN,
        FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
  },

  [ERASE_SECTOR] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, M25P_SSE, /* note: sub-sector erase */
        FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, ADDRESS_32BIT),
  },

  [ERASE_CHIP] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, M25P_BE,
        FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
  },

  [READ_FAST_QUAD_OUTPUT] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_Q_FAST_RD,
        FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, ADDRESS_32BIT),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_DUMMY_SDR, FLEXSPI_4PAD, 0x08,
        FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_4PAD, 0x04),
  },

  [PAGE_PROGRAM_QUAD_INPUT] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_Q_FAST_PP,
        FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, ADDRESS_32BIT),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_4PAD, 0x04,
        FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
  },

  [ENTER_4BYTE] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, M25P_4B_ENTER,
        FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
  },

  [ENTER_QPI] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_Q_ENTER,
        FLEXSPI_COMMAND_STOP, FLEXSPI_1PAD, 0),
  },

  [ENTER_DDR] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_WECR,
        FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_1PAD, 0x8),
  },

  [READ_FAST] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, M25P_FAST_READ,
        FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, ADDRESS_32BIT),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_DUMMY_SDR, FLEXSPI_1PAD, 0x08,
        FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_1PAD, 0x04)
  },
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* FlexSPI NOR device private data */

struct imx9_flexspi_nor_dev_s
{
  struct mtd_dev_s mtd;
  struct flexspi_dev_s *flexspi;   /* Saved FlexSPI interface instance */
  uint8_t *ahb_base;
  enum flexspi_port_e port;
  struct flexspi_device_config_s *config;
  uint8_t  sectorshift;      /* 16 or 18 */
  uint8_t  pageshift;        /* 8 */
  uint16_t nsectors;         /* 128 or 64 */
  uint32_t npages;           /* 32,768 or 65,536 */
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
  uint8_t  subsectorshift;   /* 0, 12 or 13 (4K or 8K) */
#endif
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int imx9_flexspi_nor_erase(struct mtd_dev_s *dev,
                                   off_t startblock,
                                   size_t nblocks);
static ssize_t imx9_flexspi_nor_read(struct mtd_dev_s *dev,
                                      off_t offset,
                                      size_t nbytes,
                                      uint8_t *buffer);
static ssize_t imx9_flexspi_nor_bread(struct mtd_dev_s *dev,
                                       off_t startblock,
                                       size_t nblocks,
                                       uint8_t *buffer);
static ssize_t imx9_flexspi_nor_bwrite(struct mtd_dev_s *dev,
                                        off_t startblock,
                                        size_t nblocks,
                                        const uint8_t *buffer);
static int imx9_flexspi_nor_ioctl(struct mtd_dev_s *dev,
                                   int cmd,
                                   unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct flexspi_device_config_s g_flexspi_device_config =
{
  .flexspi_root_clk = 133000000,
  .flash_size = 1024 * 64, /* size in kB */
  .cs_interval_unit = FLEXSPI_CS_INTERVAL_UNIT1_SCK_CYCLE,
  .cs_interval = 0,
  .cs_hold_time = 3,
  .cs_setup_time = 3,
  .data_valid_time = 0,
  .columnspace = 0,
  .enable_word_address = 0,
  .awr_seq_index = 0,
  .awr_seq_number = 0,
  .ard_seq_index = READ_FAST_QUAD_OUTPUT,
  .ard_seq_number = 1,
  .ahb_write_wait_unit = FLEXSPI_AHB_WRITE_WAIT_UNIT2_AHB_CYCLE,
  .ahb_write_wait_interval = 0
};

static struct imx9_flexspi_nor_dev_s g_flexspi_nor =
{
  .mtd =
          {
            .erase  = imx9_flexspi_nor_erase,
            .bread  = imx9_flexspi_nor_bread,
            .bwrite = imx9_flexspi_nor_bwrite,
            .read   = imx9_flexspi_nor_read,
            .ioctl  = imx9_flexspi_nor_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
            .write  = NULL,
#endif
            .name   = "imx9_flexspi_nor"
          },
  .flexspi = (void *)0,
  .ahb_base = (uint8_t *) CONFIG_FSPI_PER_BASEADDR,
  .port = FLEXSPI_PORT_A1,
  .config = &g_flexspi_device_config
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imx9_flexspi_nor_get_id(
                struct imx9_flexspi_nor_dev_s *priv)
{
  static uint32_t buffer = 0;
  int stat;

  uint8_t manufacturer;
  uint8_t memory;
  uint8_t capacity;

  struct flexspi_transfer_s transfer =
  {
    .device_address = 0,
    .port = priv->port,
    .cmd_type = FLEXSPI_READ,
    .seq_number = 1,
    .seq_index = READ_ID,
    .data = &buffer,
    .data_size = 3,
  };

  stat = FLEXSPI_TRANSFER(priv->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  manufacturer = buffer;
  memory = buffer >> 8;
  capacity = buffer >> 16;

  /* Check for a valid manufacturer and memory type */

  if (manufacturer == M25P_MANUFACTURER && memory == M25P_MEMORY_TYPE)
    {
      /* Okay.. is it a FLASH capacity that we understand? */

#ifdef CONFIG_M25P_SUBSECTOR_ERASE
      priv->subsectorshift = 0;
#endif

      if (capacity == M25P_M25P1_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_M25P1_SECTOR_SHIFT;
          priv->nsectors       = M25P_M25P1_NSECTORS;
          priv->pageshift      = M25P_M25P1_PAGE_SHIFT;
          priv->npages         = M25P_M25P1_NPAGES;
          return OK;
        }
      else if (capacity == M25P_EN25F80_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->pageshift      = M25P_EN25F80_PAGE_SHIFT;
          priv->npages         = M25P_EN25F80_NPAGES;
          priv->sectorshift    = M25P_EN25F80_SECTOR_SHIFT;
          priv->nsectors       = M25P_EN25F80_NSECTORS;
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
          priv->subsectorshift = M25P_EN25F80_SUBSECT_SHIFT;
#endif
          return OK;
        }
      else if (capacity == M25P_M25P16_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_M25P16_SECTOR_SHIFT;
          priv->nsectors       = M25P_M25P16_NSECTORS;
          priv->pageshift      = M25P_M25P16_PAGE_SHIFT;
          priv->npages         = M25P_M25P16_NPAGES;
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
          priv->subsectorshift = M25P_M25PX16_SUBSECT_SHIFT;
#endif
          return OK;
        }
      else if (capacity == M25P_M25P32_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_M25P32_SECTOR_SHIFT;
          priv->nsectors       = M25P_M25P32_NSECTORS;
          priv->pageshift      = M25P_M25P32_PAGE_SHIFT;
          priv->npages         = M25P_M25P32_NPAGES;
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
          priv->subsectorshift = M25P_M25PX32_SUBSECT_SHIFT;
#endif
          return OK;
        }
      else if (capacity == M25P_M25P64_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_M25P64_SECTOR_SHIFT;
          priv->nsectors       = M25P_M25P64_NSECTORS;
          priv->pageshift      = M25P_M25P64_PAGE_SHIFT;
          priv->npages         = M25P_M25P64_NPAGES;
          return OK;
        }
      else if (capacity == M25P_M25P128_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_M25P128_SECTOR_SHIFT;
          priv->nsectors       = M25P_M25P128_NSECTORS;
          priv->pageshift      = M25P_M25P128_PAGE_SHIFT;
          priv->npages         = M25P_M25P128_NPAGES;
          return OK;
        }
    }
  else if (manufacturer == M25P_MANUFACTURER &&
          (memory == MT25Q_MEMORY_TYPE || memory == MT25QU_MEMORY_TYPE))
    {
      /* Also okay.. is it a FLASH capacity that we understand? */

#ifdef CONFIG_M25P_SUBSECTOR_ERASE
      priv->subsectorshift = 0;
#endif
      if (capacity == M25P_MT25Q128_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_MT25Q128_SECTOR_SHIFT;
          priv->nsectors       = M25P_MT25Q128_NSECTORS;
          priv->pageshift      = M25P_MT25Q128_PAGE_SHIFT;
          priv->npages         = M25P_MT25Q128_NPAGES;
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
          priv->subsectorshift = M25P_MT25Q128_SUBSECT_SHIFT;
#endif
          return OK;
        }
      else if (capacity == M25P_MT25Q256_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_MT25Q256_SECTOR_SHIFT;
          priv->nsectors       = M25P_MT25Q256_NSECTORS;
          priv->pageshift      = M25P_MT25Q256_PAGE_SHIFT;
          priv->npages         = M25P_MT25Q256_NPAGES;
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
          priv->subsectorshift = M25P_MT25Q256_SUBSECT_SHIFT;
#endif
          return OK;
        }
      else if (capacity == M25P_MT25Q512_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_MT25Q512_SECTOR_SHIFT;
          priv->nsectors       = M25P_MT25Q512_NSECTORS;
          priv->pageshift      = M25P_MT25Q512_PAGE_SHIFT;
          priv->npages         = M25P_MT25Q512_NPAGES;
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
          priv->subsectorshift = M25P_MT25Q512_SUBSECT_SHIFT;
#endif
          return OK;
        }
      else if (capacity == M25P_MT25Q1G_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift    = M25P_MT25Q1G_SECTOR_SHIFT;
          priv->nsectors       = M25P_MT25Q1G_NSECTORS;
          priv->pageshift      = M25P_MT25Q1G_PAGE_SHIFT;
          priv->npages         = M25P_MT25Q1G_NPAGES;
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
          priv->subsectorshift = M25P_MT25Q1G_SUBSECT_SHIFT;
#endif
          return OK;
        }
    }

  return -ENODEV;
}

static int imx9_flexspi_nor_read_register(
                const struct imx9_flexspi_nor_dev_s *dev,
                uint32_t *value, uint32_t len, int seq)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = 0,
    .port = dev->port,
    .cmd_type = FLEXSPI_READ,
    .seq_number = 1,
    .seq_index = seq,
    .data = value,
    .data_size = 1,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imx9_flexspi_nor_write_cmd(
                const struct imx9_flexspi_nor_dev_s *dev,
                int cmd)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = 0,
    .port = dev->port,
    .cmd_type = FLEXSPI_COMMAND,
    .seq_number = 1,
    .seq_index = cmd,
    .data = NULL,
    .data_size = 0,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imx9_flexspi_nor_erase_sector(
                const struct imx9_flexspi_nor_dev_s *dev,
  off_t offset)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = offset,
    .port = dev->port,
    .cmd_type = FLEXSPI_COMMAND,
    .seq_number = 1,
    .seq_index = ERASE_SECTOR,
    .data = NULL,
    .data_size = 0,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imx9_flexspi_nor_page_program(
                      const struct imx9_flexspi_nor_dev_s *dev,
                      off_t offset,
                      const void *buffer,
                      size_t len)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = offset,
    .port = dev->port,
    .cmd_type = FLEXSPI_WRITE,
    .seq_number = 1,
    .seq_index = PAGE_PROGRAM_QUAD_INPUT,
    .data = (uint32_t *) buffer,
    .data_size = len,
  };

    stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imx9_flexspi_nor_wait_bus_busy(
                const struct imx9_flexspi_nor_dev_s *dev)
{
  uint32_t status = 0;
  int ret;

  do
    {
      ret = imx9_flexspi_nor_read_register(dev, &status, 1, READ_STATUS_REG);
      if (ret)
        {
          return ret;
        }
    }
  while (status & 1);

  return 0;
}

static ssize_t imx9_flexspi_nor_read(struct mtd_dev_s *dev,
                                      off_t offset,
                                      size_t nbytes,
                                      uint8_t *buffer)
{
  struct imx9_flexspi_nor_dev_s *priv =
                  (struct imx9_flexspi_nor_dev_s *)dev;
  uint8_t *src;

  finfo("Read offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  if (priv->port >= FLEXSPI_PORT_COUNT)
    {
      return -EIO;
    }

  src = priv->ahb_base + offset;

  int n = nbytes;

  while (n-- > 0)
    {
      *buffer++ = *src++;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

static ssize_t imx9_flexspi_nor_bread(struct mtd_dev_s *dev,
                                       off_t startblock,
                                       size_t nblocks,
                                       uint8_t *buffer)
{
  ssize_t nbytes;
  struct imx9_flexspi_nor_dev_s *priv =
                  (struct imx9_flexspi_nor_dev_s *)dev;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = imx9_flexspi_nor_read(dev, startblock << priv->subsectorshift,
                                 nblocks << priv->subsectorshift, buffer);

  if (nbytes < 0)
    {
      return nbytes;
    }

  return nblocks;
}

static ssize_t imx9_flexspi_nor_bwrite(struct mtd_dev_s *dev,
                                        off_t startblock,
                                        size_t nblocks,
                                        const uint8_t *src)
{
  struct imx9_flexspi_nor_dev_s *priv =
                  (struct imx9_flexspi_nor_dev_s *)dev;
  size_t pgsize = 1 << priv->pageshift;
  size_t len = nblocks << priv->subsectorshift;
  off_t offset = startblock << priv->subsectorshift;

  int i = 0;

  up_clean_dcache((uintptr_t)src, (uintptr_t)src +
                  ALIGN_UP(len, ARMV8A_DCACHE_LINESIZE));

  finfo("Wstartblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (len)
    {
      i = MIN(pgsize, len);
      imx9_flexspi_nor_write_cmd(priv, WRITE_ENABLE);
      imx9_flexspi_nor_page_program(priv, offset, src, i);
      imx9_flexspi_nor_wait_bus_busy(priv);
      FLEXSPI_SOFTWARE_RESET(priv->flexspi);
      offset += i;
      src += i;
      len -= i;
    }

  return nblocks;
}

static int imx9_flexspi_nor_erase(struct mtd_dev_s *dev,
                                   off_t startblock,
                                   size_t nblocks)
{
  struct imx9_flexspi_nor_dev_s *priv =
                  (struct imx9_flexspi_nor_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      imx9_flexspi_nor_write_cmd(priv, WRITE_ENABLE);
      imx9_flexspi_nor_erase_sector(priv,
                                    startblock << priv->subsectorshift);
      imx9_flexspi_nor_wait_bus_busy(priv);
      FLEXSPI_SOFTWARE_RESET(priv->flexspi);
      startblock++;
    }

  return (int)nblocks;
}

static int imx9_flexspi_nor_ioctl(struct mtd_dev_s *dev,
                                   int cmd,
                                   unsigned long arg)
{
  struct imx9_flexspi_nor_dev_s *priv =
                  (struct imx9_flexspi_nor_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo =
            (struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the
               * client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

              /* We report 4k blocksize, that is more convient for upper
               * layers
               */

              geo->blocksize = (1 << priv->subsectorshift);
#ifdef CONFIG_M25P_SUBSECTOR_ERASE
              if (priv->subsectorshift > 0)
                {
                  geo->erasesize    = (1 << priv->subsectorshift);
                  geo->neraseblocks = priv->nsectors *
                                     (1 << (priv->sectorshift -
                                      priv->subsectorshift));
                }
              else
#endif
                {
                  geo->erasesize    = (1 << priv->sectorshift);
                  geo->neraseblocks = priv->nsectors;
                }

              ret               = OK;

              finfo("blocksize: %u erasesize: %u neraseblocks: %u\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:

        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          imx9_flexspi_nor_write_cmd(priv, WRITE_ENABLE);
          ret = imx9_flexspi_nor_write_cmd(priv, ERASE_CHIP);
          if (ret)
            {
              ferr("bulk erase failed\n");
            }

          imx9_flexspi_nor_wait_bus_busy(priv);
          FLEXSPI_SOFTWARE_RESET(priv->flexspi);
        }
        break;

      case MTDIOC_PROTECT:

        /* TODO */

        break;

      case MTDIOC_UNPROTECT:

        /* TODO */

        break;

      default:
        ret = -ENOTTY; /* Bad/unsupported command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_flexspi_nor_initialize
 *
 * Description:
 *   Initialize a NOR FLASH on FlexSPI interface
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Pointer to an mtd device, Null on any error
 *
 ****************************************************************************/

struct mtd_dev_s *imx9_flexspi_nor_initialize(int intf)
{
  uint32_t val = 0;

  /* Configure multiplexed pins as connected on the board */

  imx9_iomux_configure(MUX_FLEXSPI_IO0);
  imx9_iomux_configure(MUX_FLEXSPI_IO1);
  imx9_iomux_configure(MUX_FLEXSPI_IO2);
  imx9_iomux_configure(MUX_FLEXSPI_IO3);
  imx9_iomux_configure(MUX_FLEXSPI_CMD);
  imx9_iomux_configure(MUX_FLEXSPI_CLK);

  g_flexspi_nor.flexspi = imx9_flexspi_initialize(intf);
  if (!g_flexspi_nor.flexspi)
    {
      return NULL;
    }

  FLEXSPI_SET_DEVICE_CONFIG(g_flexspi_nor.flexspi,
                            g_flexspi_nor.config,
                            g_flexspi_nor.port);
  FLEXSPI_UPDATE_LUT(g_flexspi_nor.flexspi,
                     0,
                     (const uint32_t *)g_flexspi_nor_lut,
                     sizeof(g_flexspi_nor_lut) / 4);
  FLEXSPI_SOFTWARE_RESET(g_flexspi_nor.flexspi);

  if (imx9_flexspi_nor_get_id(&g_flexspi_nor))
    {
      return NULL;
    }

  imx9_flexspi_nor_write_cmd(&g_flexspi_nor, ENTER_4BYTE);

  imx9_flexspi_nor_read_register(&g_flexspi_nor, &val,
    1, READ_FLAG_STATUS_REG);
  finfo("Flag status register = 0x%x\n", val);

  return &g_flexspi_nor.mtd;
}

#endif /* CONFIG_IMX9_FLEXSPI_NOR */
