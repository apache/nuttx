/****************************************************************************
 * arch/arm/src/phy62xx/flash.h
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
 *    @file     flash.h
 *    @brief    Contains all functions support for flash driver
 *    @version  0.0
 *    @date     27. Nov. 2017
 *    @author   qing.han
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef _FLASH_H_
#define _FLASH_H_

#include "rom_sym_def.h"
#include "clock.h"
#include "types.h"
#include "gpio.h"

#define CHIP_MADDR_LEN          6
#define CHIP_ID_FLASH_ADDRESS           0x11000800
#define CHIP_MADDR_FLASH_ADDRESS        (CHIP_ID_FLASH_ADDRESS+CHIP_ID_LENGTH*4)

#define FLASH_SIZE_256KB        (0)
#define FLASH_SIZE_512KB        (1)
#define FLASH_SIZE_1MB          (2)

#define SPIF_FLASH_SIZE         FLASH_SIZE_512KB

#define SPIF_TIMEOUT       (0x7ffffff)      /* 1000000 */

#define SFLG_WIP    1
#define SFLG_WEL    2
#define SFLG_WELWIP 3

/* define flash ucds */

#define FLASH_BASE_ADDR         (0x11000000)
#define FLASH_UCDS_ADDR_BASE    0x11005000

#define CHIP_ID_LENGTH          64
#define CHIP_ID_PID_LEN         16
#define CHIP_ID_LID_LEN         10
#define CHIP_ID_MID_LEN         16
#define CHIP_ID_TID_LEN         14
#define CHIP_ID_SID_LEN         8

#define CHIP_MADDR_LEN          6

/* xip flash read instrcution */

#define XFRD_FCMD_READ          0x0000003
#define XFRD_FCMD_READ_DUAL     0x801003B
#define XFRD_FCMD_READ_QUAD     0x801006B

#define FCMD_RESET              0x99         /* reset */
#define FCMD_ENRST              0x66         /* enable reset */
#define FCMD_WREN               0x06         /* write enable */
#define FCMD_WRDIS              0x04         /* write disable */
#define FCMD_VSRWREN            0x50         /* Volatile SR Write Enable */

#define FCMD_CERASE             0x60         /* (or 0xC7)chip erase */
#define FCMD_SERASE             0x20         /* sector erase */
#define FCMD_BERASE32           0x52         /* block erease 32k */
#define FCMD_BERASE64           0xD8

#define FCMD_DPWRDN             0xB9         /* deep power down */
#define FCMD_RLSDPD             0xAB         /* release from powerdown(and read device id) */
#define FCMD_WRST               0x01         /* write status */
#define FCMD_RDID               0x9F         /* read ID */
#define FCMD_RDST               0x05         /* read status */
#define FCMD_RDST_H             0x35         /* read status high byte */
#define FCMD_PPROG              0x02         /* page program */
#define FCMD_READ               0x03         /* read */
#define FCMD_READF              0x0B         /* fast read */
#define FCMD_READDO             0x3B         /* dual output fast read */
#define FCMD_READDIO            0xBB         /* dual I/O fast read */
#define FCMD_READQO             0x6B         /* quad output fast read */
#define FCMD_READQIO            0xeB         /* quad I/O fast read */
#define FCMD_READQIOW           0xe7         /* quad I/O fast read word */

typedef struct
{
  sysclk_t      spif_ref_clk;
  uint32_t      rd_instr;
} xflash_Ctx_t;

typedef enum
{
  CHIP_ID_UNCHECK,
  CHIP_ID_EMPTY,
  CHIP_ID_VALID,
  CHIP_ID_INVALID,
}CHIP_ID_STATUS_e;

typedef struct
{
  CHIP_ID_STATUS_e chipMAddrStatus;
  uint8_t mAddr[CHIP_MADDR_LEN];
}chipMAddr_t;

extern int _spif_wait_nobusy(uint8_t flg, uint32_t tout_ns);
extern int spif_write(uint32_t addr, uint8_t *data, uint32_t size);
extern int spif_write_dma(uint32_t addr, uint8_t *data, uint32_t size);
extern int spif_read(uint32_t addr, uint8_t *data, uint32_t size);
extern int spif_read_dma(uint32_t addr, uint8_t *data, uint32_t size);
extern int spif_erase_sector(unsigned int addr);
extern int spif_erase_block64(unsigned int addr);
extern int spif_erase_all(void);
extern uint8_t spif_flash_status_reg_0(void);
extern int spif_write_protect(bool en);
extern void spif_cmd(uint8_t op, uint8_t addrlen,
    uint8_t rdlen, uint8_t wrlen, uint8_t mbit, uint8_t dummy);
extern void spif_rddata(uint8_t *data, uint8_t len);
extern int spif_config(sysclk_t ref_clk, uint8_t div,
    uint32_t rd_instr, uint8_t mode_bit, uint8_t QE);
int hal_spif_cache_init(xflash_Ctx_t cfg);

int hal_flash_write(uint32_t addr, uint8_t *data, uint32_t size);
int hal_flash_write_by_dma(uint32_t addr, uint8_t *data, uint32_t size);
int hal_flash_read(uint32_t addr, uint8_t *data, uint32_t size);
int hal_flash_erase_sector(unsigned int addr);
int hal_flash_erase_block64(unsigned int addr);
int flash_write_word(unsigned int offset, uint32_t  value);

CHIP_ID_STATUS_e chip_id_one_bit_hot_convter(uint8_t *b, uint32_t w);

void LL_PLUS_LoadMACFromFlash(uint32_t addr);

CHIP_ID_STATUS_e LL_PLUS_LoadMACFromChipMAddr(void);

void check_chip_mAddr(void);
void LOG_CHIP_MADDR(void);

#endif
