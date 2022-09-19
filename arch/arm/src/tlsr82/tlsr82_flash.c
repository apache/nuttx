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
#include "hardware/tlsr82_irq.h"

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

/* Flash status register bit definitions
 * FLASH_STATUS_WIP: The Write In Progress (WIP) bit indicates whether
 *                   the memory is busy in program/erase/write status
 *                   register progress.
 * FLASH_STATUS_WEL: The Write Enable Latch (WEL) bit indicates the
 *                   status of the internal Write Enable Latch.
 * FLASH_STATUS_BP:  The Block Protect (BP2, BP1, BP0) bits are
 *                   non-volatile. They define the size of the area to be
 *                   software protected against Program and Erase commands.
 * FLASH_STATUS_SRP: The Status Register Protect (SRP) bit operates in
 *                   conjunction with the Write Protect (WP#) signal.
 */

#define FLASH_STATUS_WIP_SHIFT               0
#define FLASH_STATUS_WIP_MASK                (0x1 << FLASH_STATUS_WIP_SHIFT)
#define FLASH_STATUS_WEL_SHIFT               1
#define FLASH_STATUS_WEL_MASK                (0x1 << FLASH_STATUS_WEL_SHIFT)
#define FLASH_STATUS_BP_SHIFT                2
#define FLASH_STATUS_BP_MASK                 (0x7 << FLASH_STATUS_BP_SHIFT)
#define FLASH_STATUS_RSVD_SHIFT              5
#define FLASH_STATUS_RSVD_MASK               (0x3 << FLASH_STATUS_RSVD_SHIFT)
#define FLASH_STATUS_SRP_SHIFT               7
#define FLASH_STATUS_SRP_MASK                (0x1 << FLASH_STATUS_SRP_SHIFT)

/* NONE: do not protect, ALL: protect all the flash */

#define FLASH_STATUS_BP_NONE                 (0x0 << FLASH_STATUS_BP_SHIFT)
#define FLASH_STATUS_BP_ALL                  (0x7 << FLASH_STATUS_BP_SHIFT)

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

/* Flash status type definitions */

#define FLASH_STATUS_TYPE_8BIT               0x0
#define FLASH_STATUS_TYPE_16BIT              0x1

/* Flash lock/unlock macros definitions */

#ifndef CONFIG_TLSR8278_BLE_SDK

#define FLASH_LOCK_INIT()                    \
  irqstate_t _flags;

#define FLASH_LOCK()                         \
  do                                         \
    {                                        \
      _flags = enter_critical_section();     \
    }                                        \
  while(0)

#define FLASH_UNLOCK()                       \
  do                                         \
    {                                        \
      leave_critical_section(_flags);        \
    }                                        \
  while(0)

#else

/* When enable the ble sdk, we can not disable the system timer
 * and rf(ble) interrupt to avoid loss of ble packets. We also
 * need to call sched_lock()/sched_unlock() to avoid task switch
 * beacause sem_post() will be called in ble interrupt (Task
 * switch may leads that the chip execute code in flash during
 * the operation of flash).
 */

#define IRQ_MASK_RF_SYSTIMER                 ((1 << NR_SYSTEM_TIMER_IRQ) | \
                                              (1 << NR_RF_IRQ))

#define FLASH_LOCK_INIT()                    \
  irqstate_t _flags;                         \
  uint32_t _mask;

#define FLASH_LOCK()                         \
  do                                         \
    {                                        \
      sched_lock();                          \
      _flags = enter_critical_section();     \
      _mask = IRQ_MASK_REG &                 \
              (~IRQ_MASK_RF_SYSTIMER);       \
      IRQ_MASK_REG &= IRQ_MASK_RF_SYSTIMER;  \
      leave_critical_section(_flags);        \
    }                                        \
  while(0)

#define FLASH_UNLOCK()                       \
  do                                         \
    {                                        \
      _flags = enter_critical_section();     \
      IRQ_MASK_REG = _mask |                 \
      (IRQ_MASK_REG & IRQ_MASK_RF_SYSTIMER); \
      leave_critical_section(_flags);        \
      sched_unlock();                        \
    }                                        \
  while(0)

#endif

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

  FLASH_LOCK_INIT();

  FLASH_LOCK();

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

  FLASH_UNLOCK();
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

  FLASH_LOCK_INIT();

  FLASH_LOCK();

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

  FLASH_UNLOCK();
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
 * Name: tlsr82_flash_read_status
 *
 * Description:
 *   This function reads the status of flash.
 *
 * Input Parameters:
 *   cmd  - the cmd of read status.
 *
 * Returned Value:
 *   status.
 *
 ****************************************************************************/

uint8_t tlsr82_flash_read_status(uint8_t cmd)
{
  uint8_t status = 0;

  tlsr82_flash_read(cmd, FLASH_INVALID_ADDR, 0, &status, 1);

  return status;
}

/****************************************************************************
 * Name: tlsr82_flash_write_status
 *
 * Description:
 *   Write the status to flash status register.
 *
 * Input Parameters:
 *   type   - the type of status.8 bit or 16 bit.
 *   status - the value of status.
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_flash_write_status(uint8_t type, uint16_t status)
{
  uint8_t buf[2];

  /* Follow the sdk to write flash status register */

  buf[0] = (uint8_t)(status & 0x00ff);
  buf[1] = (uint8_t)(status >> 8);

  if (type == FLASH_STATUS_TYPE_8BIT)
    {
      tlsr82_flash_write(FLASH_CMD_WRITE_STATUS_LBYTE,
                         FLASH_INVALID_ADDR, buf, 1);
    }
  else if (type == FLASH_STATUS_TYPE_16BIT)
    {
      tlsr82_flash_write(FLASH_CMD_WRITE_STATUS_LBYTE,
                         FLASH_INVALID_ADDR, buf, 2);
    }
}

/****************************************************************************
 * Name: tlsr82_flash_protect
 *
 * Description:
 *   This function serves to set the protection area of the flash.
 *   Note: after check the sdk code, this function can be applied to all
 *         flash chips:
 *         FLASH_MID_GD25LD10C 0x1160c8 128K
 *         FLASH_MID_GD25LD40C 0x1360c8 512K
 *         FLASH_MID_GD25LD80C 0x1460c8 1M
 *         FLASH_MID_ZB25WD10A 0x11325e 128K
 *         FLASH_MID_ZB25WD40B 0x13325e 512K
 *         FLASH_MID_ZB25WD80B 0x14325e 1M
 *         FLASH_MID_ZB25WD20A 0x12325e 256K
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_FLASH_PROTECT
void tlsr82_flash_protect(void)
{
  uint8_t status;
  uint8_t data;

  status = tlsr82_flash_read_status(FLASH_CMD_READ_STATUS_LBYTE);
  data   = FLASH_STATUS_BP_ALL | (status & (~FLASH_STATUS_BP_MASK));

  tlsr82_flash_write_status(FLASH_STATUS_TYPE_8BIT, data);
}
#endif

/****************************************************************************
 * Name: tlsr82_flash_unprotect
 *
 * Description:
 *   This function serves to release flash protection.
 *   Note: after check the sdk code, this function can be applied to all
 *         flash chips:
 *         FLASH_MID_GD25LD10C 0x1160c8 128K
 *         FLASH_MID_GD25LD40C 0x1360c8 512K
 *         FLASH_MID_GD25LD80C 0x1460c8 1M
 *         FLASH_MID_ZB25WD10A 0x11325e 128K
 *         FLASH_MID_ZB25WD40B 0x13325e 512K
 *         FLASH_MID_ZB25WD80B 0x14325e 1M
 *         FLASH_MID_ZB25WD20A 0x12325e 256K
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_FLASH_PROTECT
void tlsr82_flash_unprotect(void)
{
  uint8_t status;
  uint8_t data;

  status = tlsr82_flash_read_status(FLASH_CMD_READ_STATUS_LBYTE);
  data   = FLASH_STATUS_BP_NONE | (status & (~FLASH_STATUS_BP_MASK));

  tlsr82_flash_write_status(FLASH_STATUS_TYPE_8BIT, data);
}
#endif

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

/****************************************************************************
 * Name: tlsr82_flash_calibrate
 *
 * Description:
 *   Follow telink sdk to do flash voltage calibration, the flash supply
 *   voltage accuracy is very important, the low flash supply voltage will
 *   cause the flash data destoied.
 *
 * Input Parameters:
 *   mid - manufacaturer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_FLASH_CALI
void tlsr82_flash_calibrate(uint32_t mid)
{
#if defined(CONFIG_ARCH_CHIP_TLSR8278)

  uint16_t cali_data;

  tlsr82_flash_read_data(CONFIG_TLSR82_FLASH_CALI_PARA_ADDR, cali_data,
                         sizeof(cali_data));

  finfo("cali_data=0x%x\n");

  if ((0xffff == cali_data) || (0 != (cali_data & 0xf8f8)))
    {
      /* No flash calibration paramters */

      finfo("No flash calibration parameters\n");

      if (mid == FLASH_MID_ZB25WD10A || mid == FLASH_MID_ZB25WD40B ||
          mid == FLASH_MID_ZB25WD80B || mid == FLASH_MID_ZB25WD20A)
        {
          /* LDO mode flash ldo trim 1.95V */

          tlsr82_analog_write(0x09, ((tlsr82_analog_read(0x09) & 0x8f) |
                                     (FLASH_VOLTAGE_1V95 << 4)));

          /* DCDC mode flash ldo trim 1.90V */

          tlsr82_analog_write(0x0c, ((tlsr82_analog_read(0x0c) & 0xf8) |
                                     FLASH_VOLTAGE_1V9));
        }
    }
  else
    {
      /* Flash calibration parameters exist, write the flash calibration
       * parameters to the register
       */

      finfo("Calibrate the flash voltage\n");

      tlsr82_analog_write(0x09, ((tlsr82_analog_read(0x09) & 0x8f) |
                                 ((cali_data & 0xff00) >> 4)));
      tlsr82_analog_write(0x0c, ((tlsr82_analog_read(0x0c) & 0xf8) |
                                 (cali_data & 0xff)));
    }

#elif defined(CONFIG_ARCH_CHIP_TLSR8258)

  uint8_t cali_data;

  tlsr82_flash_read_data(CONFIG_TLSR82_FLASH_CALI_PARA_ADDR, cali_data,
                         sizeof(cali_data));

  finfo("cali_data=0x%x\n");

  if (0xff == cali_data)
    {
      /* No flash calibration paramters */

      finfo("No flash calibration parameters\n");

      if (mid == FLASH_MID_ZB25WD10A || mid == FLASH_MID_ZB25WD40B ||
          mid == FLASH_MID_ZB25WD80B || mid == FLASH_MID_ZB25WD20A)
        {
          /* DCDC mode flash ldo trim 1.95V */

          tlsr82_analog_write(0x0c, ((tlsr82_analog_read(0x0c) & 0xf8) |
                                     FLASH_VOLTAGE_1V95));
        }
    }
  else
    {
      /* Flash calibration parameters exist, write the flash calibration
       * parameters to the register
       */

      finfo("Calibrate the flash voltage\n");

      tlsr82_analog_write(0x0c, ((tlsr82_analog_read(0x0c) & 0xf8) |
                                 (cali_data & 0x7)));
    }
#endif
}
#endif
