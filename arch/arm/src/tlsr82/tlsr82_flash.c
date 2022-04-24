/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_flash.c
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

#include <nuttx/compiler.h>
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <assert.h>
#include <debug.h>

#include "hardware/tlsr82_mspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Manufacturer id definitions */

#if defined(CONFIG_ARCH_CHIP_TLSR8278)
#  define FLASH_MID_GD25LD10C                  0x1160c8
#  define FLASH_MID_GD25LD40C                  0x1360c8
#  define FLASH_MID_GD25LD80C                  0x1460c8
#  define FLASH_MID_ZB25WD10A                  0x11325e
#  define FLASH_MID_ZB25WD40B                  0x13325e
#  define FLASH_MID_ZB25WD80B                  0x14325e
#  define FLASH_MID_ZB25WD20A                  0x12325e
#elif defined(CONFIG_ARCH_CHIP_TLSR8258)
#  define FLASH_MID_ZB25WD40B                  0x13325e
#  define FLASH_MID_ZB25WD80B                  0x14325e
#  define FLASH_MID_GD25LD05C                  0x1060c8
#  define FLASH_MID_GD25LD40C                  0x1360c8
#  define FLASH_MID_GD25LD80C                  0x1460c8
#  define FLASH_MID_GD25LE80C                  0x1460c8
#  define FLASH_MID_GD25LQ80C                  0x1460c8
#  define FLASH_MID_MD25D40D                   0x134051
#  define FLASH_MID_P25Q40L                    0x136085
#  define FLASH_MID_TH25D40LA                  0x1360eb
#  define FLASH_MID_TH25D40UA                  0x1360eb
#endif

/* Flash command definitions */

#define FLASH_INVALID_ADDR                   0xffffffff

#define FLASH_CMD_WRITE                      0x02
#define FLASH_CMD_WRITE_ENABLE               0x06
#define FLASH_CMD_WRITE_DISABLE              0x04
#define FLASH_CMD_WRITE_SECURITY_REG         0x42
#define FLASH_CMD_WRITE_STATUS_LBYTE         0x01
#define FLASH_CMD_WRITE_STATUS_HBYTE         0x31

#define FLASH_CMD_READ                       0x03
#define FLASH_CMD_READ_SECURITY_REG          0x48
#define FLASH_CMD_READ_JEDEC_ID              0x9f
#define FLASH_CMD_READ_STATUS_LBYTE          0x05
#define FLASH_CMD_READ_STATUS_HBYTE          0x35

#define FLASH_CMD_ERASE_SECTOR               0x20
#define FLASH_CMD_ERASE_SECURITY_REG         0x44

/* Read the flash uniqe id command
 * FLASH_CMD_READ_UID1: GD_PUYA_ZB_TH
 * FLASH_CMD_READ_UID2: XTX
 */

#define FLASH_CMD_READ_UID1                  0x4b
#define FLASH_CMD_READ_UID2                  0x5a

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_invalid_uid[16] =
{
  0x51, 0x01, 0x51, 0x01,
  0x51, 0x01, 0x51, 0x01,
  0x51, 0x01, 0x51, 0x01,
  0x51, 0x01, 0x51, 0x01,
};

#if defined(CONFIG_ARCH_CHIP_TLSR8278)
static const uint32_t g_support_mid[] =
{
  0x1160c8, /* FLASH_MID_GD25LD10C */
  0x1360c8, /* FLASH_MID_GD25LD40C */
  0x1460c8, /* FLASH_MID_GD25LD80C */
  0x11325e, /* FLASH_MID_ZB25WD10A */
  0x12325e, /* FLASH_MID_ZB25WD20A */
  0x13325e, /* FLASH_MID_ZB25WD40B */
  0x14325e, /* FLASH_MID_ZB25WD80B */
};
#elif defined(CONFIG_ARCH_CHIP_TLSR8258)
static const uint32_t g_support_mid[] =
{
  0x0013325e, /* FLASH_MID_ZB25WD40B */
  0x0014325e, /* FLASH_MID_ZB25WD80B */
  0x001060c8, /* FLASH_MID_GD25LD05C */
  0x001360c8, /* FLASH_MID_GD25LD40C */
  0x001460c8, /* FLASH_MID_GD25LD80C, FLASH_MID_GD25LE80C */
  0x011460c8, /* FLASH_MID_GD25LQ80C */
  0x00134051, /* FLASH_MID_MD25D40D */
  0x00136085, /* FLASH_MID_P25Q40L */
  0x001360eb, /* FLASH_MID_TH25D40LA, FLASH_MID_TH25D40UA */
};
#endif

static const uint32_t g_support_mid_num =
  sizeof(g_support_mid) / sizeof(uint32_t);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flash_send_cmd
 *
 * Description:
 *   Configurate the gpio drive strength be high/low.
 *
 * Input Parameters:
 *   cmd - GPIO config information
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void locate_code(".ram_code") flash_send_cmd(uint8_t cmd)
{
  MSPI_CS_HIGH();
  up_udelay(1);
  MSPI_CS_LOW();
  MSPI_WRITE(cmd);
  MSPI_WAIT();
}

static void locate_code(".ram_code") flash_send_addr(uint32_t addr)
{
  MSPI_WRITE((uint8_t)(addr >> 16));
  MSPI_WAIT();

  MSPI_WRITE((uint8_t)(addr >> 8));
  MSPI_WAIT();

  MSPI_WRITE((uint8_t)(addr));
  MSPI_WAIT();
}

static void locate_code(".ram_code") flash_wait_done(void)
{
  int i;

  up_udelay(100);
  flash_send_cmd(FLASH_CMD_READ_STATUS_LBYTE);

  for (i = 0; i < 10000000; i++)
    {
      MSPI_WRITE(0);
      MSPI_WAIT();
      if ((MSPI_READ() & 0x01) == 0)
        {
          break;
        }
    }

  MSPI_CS_HIGH();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_flash_read
 *
 * Description:
 *   Write data to addr
 *
 * Input Parameters:
 *   cmd  - flash write command
 *   addr - flash write address
 *   buf  - flash write buffer
 *   len  - flash write buffer length
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void locate_code(".ram_code") tlsr82_flash_read(uint8_t cmd, uint32_t addr,
                                                uint8_t dummy_cnt,
                                                uint8_t *buf, uint32_t len)
{
  int i;
  irqstate_t flags;

  flags = enter_critical_section();

  flash_send_cmd(cmd);

  /* If send read address first, send the address */

  if (addr != FLASH_INVALID_ADDR)
    {
      flash_send_addr(addr);
    }

  /* Write dummy data, why ? */

  for (i = 0; i < dummy_cnt; i++)
    {
      MSPI_WRITE(0);
      MSPI_WAIT();
    }

  /* Issue clock, why ? */

  MSPI_WRITE(0);
  MSPI_WAIT();

  /* Switch to auto mode */

  MSPI_AUTO_MODE();
  MSPI_WAIT();

  /* Read data */

  for (i = 0; i < len; i++)
    {
      buf[i] = MSPI_READ();
      MSPI_WAIT();
    }

  /* Write finish, pull-up the cs */

  MSPI_CS_HIGH();

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tlsr82_flash_write
 *
 * Description:
 *   Write data to addr
 *
 * Input Parameters:
 *   cmd  - flash write command
 *   addr - flash write address
 *   buf  - flash write buffer
 *   len  - flash write buffer length
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void locate_code(".ram_code") tlsr82_flash_write(uint8_t cmd, uint32_t addr,
                                                 const uint8_t *buf,
                                                 uint32_t len)
{
  int i;
  irqstate_t flags;

  flags = enter_critical_section();

  flash_send_cmd(FLASH_CMD_WRITE_ENABLE);
  flash_send_cmd(cmd);

  /* If send write address first, send the address */

  if (addr != FLASH_INVALID_ADDR)
    {
      flash_send_addr(addr);
    }

  /* Write data */

  for (i = 0; i < len; i++)
    {
      MSPI_WRITE(buf[i]);
      MSPI_WAIT();
    }

  /* Write finish, pull-up the cs */

  MSPI_CS_HIGH();

  /* Wait the flash write finish */

  flash_wait_done();

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tlsr82_flash_erase_sector
 *
 * Description:
 *   Erase the section addr at
 *
 * Input Parameters:
 *   addr - flash erase address
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_flash_erase_sector(uint32_t addr)
{
  DEBUGASSERT(addr != FLASH_INVALID_ADDR);

  tlsr82_flash_write(FLASH_CMD_ERASE_SECTOR, addr, NULL, 0);
}

/****************************************************************************
 * Name: tlsr82_flash_read_data
 *
 * Description:
 *   Read the data at address addr
 *
 * Input Parameters:
 *   addr - flash read start address
 *   buf  - flash read buffer
 *   len  - flash read buffer length
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_flash_read_data(uint32_t addr, uint8_t *buf, uint32_t len)
{
  tlsr82_flash_read(FLASH_CMD_READ, addr, 0, buf, len);
}

/****************************************************************************
 * Name: tlsr82_flash_write_data
 *
 * Description:
 *   Write the data at address addr
 *
 * Input Parameters:
 *   addr - flash write start address
 *   buf  - flash write buffer
 *   len  - flash write buffer length
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_flash_write_data(uint32_t addr, const uint8_t *buf, uint32_t len)
{
  tlsr82_flash_write(FLASH_CMD_WRITE, addr, buf, len);
}

/****************************************************************************
 * Name: tlsr82_flash_read_uid
 *
 * Description:
 *   Read the flash unique id
 *
 * Input Parameters:
 *   cmd  - read uid command
 *   buf  - uid read buffer
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_flash_read_uid(uint8_t cmd, uint8_t *buf)
{
  if (cmd == FLASH_CMD_READ_UID1)
    {
      tlsr82_flash_read(FLASH_CMD_READ_UID1, 0, 1, buf, 16);
    }
}

/****************************************************************************
 * Name: tlsr82_flash_read_mid
 *
 * Description:
 *   Read the flash chip manufacaturer ID
 *
 * Input Parameters:
 *   pmid - manufacaturer ID pointer, size 4 bytes
 *   puid - unique ID pointer, size 16 bytes
 *
 * Returned Value:
 *   Positive on success, negative on fail
 *
 ****************************************************************************/

int tlsr82_flash_miduid_check(uint32_t *pmid, uint8_t *puid)
{
  int i;
#ifdef CONFIG_ARCH_CHIP_TLSR8258
  uint8_t buf[4];
#endif

  DEBUGASSERT(pmid != NULL && puid != NULL);

  /* Read the flash manufacaturer ID */

  tlsr82_flash_read(FLASH_CMD_READ_JEDEC_ID, FLASH_INVALID_ADDR,
           0, (uint8_t *)pmid, 3);
#ifdef CONFIG_ARCH_CHIP_TLSR8258
  if (mid == FLASH_MID_GD25LE80C)
    {
      tlsr82_flash_read(FLASH_CMD_READ_UID2, 0, 1, buf, 4);
      if ((buf[0] == 0x53) && (buf[1] == 0x46) &&
          (buf[2] == 0x44) && (buf[3] == 0x50))
        {
          mid = 0x011460c8;
        }
    }
#endif

  /* Check the validation of the manufacaturer ID */

  for (i = 0; i < g_support_mid_num; i++)
    {
      if (g_support_mid[i] == *pmid)
        {
          tlsr82_flash_read_uid(FLASH_CMD_READ_UID1, (uint8_t *)puid);
          break;
        }
    }

  if (i == g_support_mid_num)
    {
      ferr("Current flash device is not supported, mid=0x%lu\n", *pmid);
      return -ENOTSUP;
    }

  /* Check the validation of the unique ID */

  if (memcmp(g_invalid_uid, puid, 16) == 0)
    {
      ferr("Current flash no unique id\n");
      return -ENOTSUP;
    }

  return OK;
}
