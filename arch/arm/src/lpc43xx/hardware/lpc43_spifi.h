/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_spifi.h
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

/* NOTE:  The SPIFI ROM interface is not defined in the LPC43xx user manual.
 * Some information in this file drivers from the NXP header file
 * spifi_rom_api.h.  I do not believe that any copyright restrictions apply.
 * But just to be certain:
 *
 *   Copyright(C) 2011, NXP Semiconductor
 *   All rights reserved.
 *
 * Software that is described herein is for illustrative purposes only which
 * provides customers with programming information regarding the products.
 * This software is supplied "AS IS" without any warranties. NXP
 * Semiconductors assumes no responsibility or liability for the use of the
 * software, conveys no license or title under any patent, copyright, or
 * mask work right to the product. NXP Semiconductors reserves the right to
 * make changes in the software without notification. NXP Semiconductors
 * also make no representation or warranty that such application will be
 * suitable for the specified use without further testing or modification.
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' relevant
 * copyright in the software, without fee, provided that it is used in
 * conjunction with NXP Semiconductors microcontrollers.  This copyright,
 * permission, and disclaimer notice must appear in all copies of this code.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SPIFI_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SPIFI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register addresses *******************************************************/

#define LPC43_SPIFI_CTRL_OFFSET    0x000
#define LPC43_SPIFI_CMD_OFFSET     0x004
#define LPC43_SPIFI_ADDR_OFFSET    0x008
#define LPC43_SPIFI_IDATA_OFFSET   0x00c
#define LPC43_SPIFI_CLIMIT_OFFSET  0x010
#define LPC43_SPIFI_DATA_OFFSET    0x014
#define LPC43_SPIFI_MCMD_OFFSET    0x018
#define LPC43_SPIFI_STAT_OFFSET    0x01c

#define LPC43_SPIFI_CTRL           (LPC43_SPIFI_BASE+LPC43_SPIFI_CTRL_OFFSET)
#define LPC43_SPIFI_CMD            (LPC43_SPIFI_BASE+LPC43_SPIFI_CMD_OFFSET)
#define LPC43_SPIFI_ADDR           (LPC43_SPIFI_BASE+LPC43_SPIFI_ADDR_OFFSET)
#define LPC43_SPIFI_IDATA          (LPC43_SPIFI_BASE+LPC43_SPIFI_IDATA_OFFSET)
#define LPC43_SPIFI_CLIMIT         (LPC43_SPIFI_BASE+LPC43_SPIFI_CLIMIT_OFFSET)
#define LPC43_SPIFI_DATA           (LPC43_SPIFI_BASE+LPC43_SPIFI_DATA_OFFSET)
#define LPC43_SPIFI_MCMD           (LPC43_SPIFI_BASE+LPC43_SPIFI_MCMD_OFFSET)
#define LPC43_SPIFI_STAT           (LPC43_SPIFI_BASE+LPC43_SPIFI_STAT_OFFSET)

/* The largest protection block of any serial flash that the ROM driver
 * can handle
 */

#define SPIFI_LONGEST_PROTBLOCK    68

/* Protection flag bit definitions */

#define SPIFI_RWPROT               (1 << 0)

/* Instruction classes for wait_busy */

#define SPIFI_STAT_INST            0
#define SPIFI_BLOCK_ERASE          1
#define SPIFI_PROG_INST            2
#define SPIFI_CHIP_ERASE           3

/* Bit definitions in options operands (MODE3, RCVCLK, and FULLCLK
 * have the same relationship as in the Control register)
 */

#define S_MODE3                    (1 << 0)
#define S_MODE0                    (0)
#define S_MINIMAL                  (1 << 1)
#define S_MAXIMAL                  (0)
#define S_FORCE_ERASE              (1 << 2)
#define S_ERASE_NOT_REQD           (1 << 3)
#define S_CALLER_ERASE             (1 << 3)
#define S_ERASE_AS_REQD            (0)
#define S_VERIFY_PROG              (1 << 4)
#define S_VERIFY_ERASE             (1 << 5)
#define S_NO_VERIFY                (0)
#define S_FULLCLK                  (1 << 6)
#define S_HALFCLK                  (0)
#define S_RCVCLK                   (1 << 7)
#define S_INTCLK                   (0)
#define S_DUAL                     (1 << 8)
#define S_CALLER_PROT              (1 << 9)
#define S_DRIVER_PROT              (0)

/* The length of a standard program command is 256 on all devices */

#define PROG_SIZE 256

/* SPI ROM driver table pointer */

#define SPIFI_ROM_PTR LPC43_ROM_DRIVER_TABLE6
#define pSPIFI *((struct spifi_driver_s **)SPIFI_ROM_PTR)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Protection/sector descriptors */

struct spfi_desc_s
{
  uint32_t base;
  uint8_t  flags;
  int8_t   log2;
  uint16_t rept;
};

/* The SPFI device state structure, passed to all ROM driver methods. */

struct spifi_dev_s
{
  uint32_t base;
  uint32_t regbase;
  uint32_t devsize;
  uint32_t memsize;

  uint8_t  mfger;
  uint8_t  devtype;
  uint8_t  devid;
  uint8_t  busy;

  union
  {
    uint16_t h;
    uint8_t  b[2];
  } stat;
  uint16_t reserved;

  uint16_t setprot;
  uint16_t writeprot;

  uint32_t memcmd;
  uint32_t progcmd;

  uint16_t sectors;
  uint16_t protbytes;

  uint32_t opts;
  uint32_t errcheck;

  uint8_t eraseshifts[4];
  uint8_t eraseops[4];

  struct spfi_desc_s *protents;
  char    prot[SPIFI_LONGEST_PROTBLOCK];
};

/* Operands of program and erase ROM driver methods */

struct spifi_operands_s
{
  uint8_t *dest;
  uint32_t length;
  uint8_t *scratch;
  int32_t  protect;
  uint32_t options;
};

/* Interface to SPIFI ROM driver */

#ifndef CONFIG_SPIFI_LIBRARY
struct spifi_driver_s
{
  int32_t  (*spifi_init)(struct spifi_dev_s *dev, uint32_t cshigh,
               uint32_t options, uint32_t mhz);
  int32_t  (*spifi_program)(struct spifi_dev_s *dev, const uint8_t *source,
               struct spifi_operands_s *opers);
  int32_t  (*spifi_erase)(struct spifi_dev_s *dev,
               struct spifi_operands_s *opers);

  /* Mode switching */

  void     (*cancel_mem_mode)(struct spifi_dev_s *dev);
  void     (*set_mem_mode)(struct spifi_dev_s *dev);

  /* Mid level functions */

  int32_t  (*checkAd)(struct spifi_dev_s *dev,
               struct spifi_operands_s *opers);
  int32_t  (*setProt)(struct spifi_dev_s *dev,
               struct spifi_operands_s *opers,
               uint8_t *change, uint8_t *saveprot);
  int32_t  (*check_block) (struct spifi_dev_s *dev, uint8_t *source,
               struct spifi_operands_s *opers, uint32_t check_program);
  int32_t  (*send_erase_cmd)(struct spifi_dev_s *dev, uint8_t op,
               uint32_t addr);
  uint32_t (*ck_erase) (struct spifi_dev_s *dev, uint32_t *addr,
               uint32_t length);
  int32_t  (*prog_block)(struct spifi_dev_s *dev, uint8_t *source,
               struct spifi_operands_s *opers, uint32_t *left_in_page);
  uint32_t (*ck_prog)(struct spifi_dev_s *dev,
                      uint8_t *source, uint8_t *dest,
                      uint32_t length);

  /* Low level functions */

  void     (*setsize) (struct spifi_dev_s *dev, int32_t value);
  int32_t  (*setdev)(struct spifi_dev_s *dev, uint32_t opts,
               uint32_t mem_cmd, uint32_t prog_cmd);
  uint32_t (*cmd)(uint8_t op, uint8_t addrlen, uint8_t intLen, uint16_t len);
  uint32_t (*readad)(struct spifi_dev_s *dev, uint32_t cmd, uint32_t addr);
  void     (*send04)(struct spifi_dev_s *dev, uint8_t op, uint8_t len,
               uint32_t value);
  void     (*wren_sendad)(struct spifi_dev_s *dev, uint32_t cmd,
               uint32_t addr, uint32_t value);
  int32_t  (*write_stat)(struct spifi_dev_s *dev, uint8_t len,
               uint16_t value);
  int32_t  (*wait_busy)(struct spifi_dev_s *dev, uint8_t prog_or_erase);
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int32_t spifi_init(struct spifi_dev_s *dev, uint32_t cshigh,
                   uint32_t options, uint32_t mhz);
int32_t spifi_program(struct spifi_dev_s *dev, const uint8_t *source,
                      struct spifi_operands_s *opers);
int32_t spifi_erase(struct spifi_dev_s *dev,
                    struct spifi_operands_s *opers);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SPIFI_H */
