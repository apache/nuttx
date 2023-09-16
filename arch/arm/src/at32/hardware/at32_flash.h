/****************************************************************************
 * arch/arm/src/at32/hardware/at32_flash.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32_FLASH_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32_FLASH_H

/****************************************************************************
  * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_AT32_FLASH_CONFIG_DEFAULT) && \
    !defined(CONFIG_AT32_FLASH_CONFIG_C) && \
    !defined(CONFIG_AT32_FLASH_CONFIG_G) && \
    !defined(CONFIG_AT32_FLASH_CONFIG_M)
#  define CONFIG_AT32_FLASH_CONFIG_DEFAULT
#endif

#if defined(CONFIG_AT32_FLASH_CONFIG_DEFAULT)

#  if defined(CONFIG_AT32_AT32F43XX)
#      define AT32_FLASH_NPAGES      512
#      define AT32_FLASH_BANK2_START 256
#      define AT32_FLASH_SIZE        (AT32_FLASH_NPAGES * AT32_FLASH_PAGESIZE)
#      define AT32_FLASH_PAGESIZE    (2 * 1024)
#  endif

#endif /* CONFIG_AT32_FLASH_CONFIG_DEFAULT */

  /* Override of the Flash Has been Chosen */

#if !defined(CONFIG_AT32_FLASH_CONFIG_DEFAULT)

#  if defined(CONFIG_AT32_AT32F43XX)

#    if defined(CONFIG_AT32_FLASH_CONFIG_C)
#      define AT32_FLASH_NPAGES      128
#      undef AT32_FLASH_BANK2_START  /* there is no bank2 */
#      define AT32_FLASH_SIZE        (AT32_FLASH_NPAGES * AT32_FLASH_PAGESIZE)
#      define AT32_FLASH_PAGESIZE    (2 * 1024)

#    elif defined(CONFIG_AT32_FLASH_CONFIG_G)
#      define AT32_FLASH_NPAGES      512
#      define AT32_FLASH_BANK2_START 256
#      define AT32_FLASH_SIZE        (AT32_FLASH_NPAGES * AT32_FLASH_PAGESIZE)
#      define AT32_FLASH_PAGESIZE    (2 * 1024)

#    elif defined(CONFIG_AT32_FLASH_CONFIG_M)
#      define AT32_FLASH_NPAGES      1008
#      define AT32_FLASH_BANK2_START 512
#      define AT32_FLASH_PAGESIZE    (4 * 1024)
#      define AT32_FLASH_SIZE        (AT32_FLASH_NPAGES * AT32_FLASH_PAGESIZE)

#    else
#      define AT32_FLASH_NPAGES      128
#      undef AT32_FLASH_BANK2_START  /* there is no bank2 */
#      define AT32_FLASH_SIZE        (AT32_FLASH_NPAGES * AT32_FLASH_PAGESIZE)
#      define AT32_FLASH_PAGESIZE    (2 * 1024)
#    endif

#  endif
#endif /* !defined(CONFIG_AT32_FLASH_CONFIG_DEFAULT) */

#if defined(CONFIG_AT32_AT32F43XX) && (AT32_FLASH_NPAGES > 128)
#if (AT32_FLASH_NPAGES > 128)
#  define AT32_FLASH_DUAL_BANK      1
#  if defined(CONFIG_AT32_FLASH_CONFIG_G)
#  define AT32_FLASH_BANK0_NPAGES   256
#  else
#  define AT32_FLASH_BANK0_NPAGES   512
#  endif
#  define AT32_FLASH_BANK1_NPAGES   (AT32_FLASH_NPAGES - AT32_FLASH_BANK0_NPAGES)
#  define AT32_FLASH_BANK0_BASE     (AT32_FLASH_BASE)
#  define AT32_FLASH_BANK1_BASE     \
      (AT32_FLASH_BASE + AT32_FLASH_PAGESIZE * AT32_FLASH_BANK0_NPAGES)
#else
#  define AT32_FLASH_BANK0_NPAGES   128
#  define AT32_FLASH_BANK0_BASE     (AT32_FLASH_BASE)
#endif
#endif

/* Register Offsets *********************************************************/

#if defined(CONFIG_AT32_AT32F43XX)

#define AT32_FLASH_PSR_OFFSET             0x00
#define AT32_FLASH_UNLOCK_OFFSET          0x04
#define AT32_FLASH_USD_UNLOCK_OFFSET      0x08
#define AT32_FLASH_STS_OFFSET             0x0C
#define AT32_FLASH_CTRL_OFFSET            0x10
#define AT32_FLASH_ADDR_OFFSET            0x14
#define AT32_FLASH_USD_OFFSET             0x1C
#define AT32_FLASH_EPPS0_OFFSET           0x20
#define AT32_FLASH_EPPS1_OFFSET           0x2C
#define AT32_FLASH_UNLOCK2_OFFSET         0x44
#define AT32_FLASH_STS2_OFFSET            0x4C
#define AT32_FLASH_CTRL2_OFFSET           0x50
#define AT32_FLASH_ADDR2_OFFSET           0x54
#define AT32_FLASH_CONTR_OFFSET           0x58
#define AT32_FLASH_DIVR_OFFSET            0x60
#define AT32_SLIB_STS2_OFFSET             0xC8
#define AT32_SLIB_STS0_OFFSET             0xCC
#define AT32_SLIB_STS1_OFFSET             0xD0
#define AT32_SLIB_PWD_CRL_OFFSET          0xD4
#define AT32_SLIB_MISC_STS_OFFSET         0xD8
#define AT32_SLIB_SET_PWD_OFFSET          0xDC
#define AT32_SLIB_SET_RANGE0_OFFSET       0xE0
#define AT32_SLIB_SET_RANGE1_OFFSET       0xE4
#define AT32_SLIB_UNLOCK_OFFSET           0xF0
#define AT32_FLASH_CRC_CTRL_OFFSET        0xF4
#define AT32_FLASH_CRC_CHKR_OFFSET        0xF8
#endif

/* Register Addresses *******************************************************/

#if defined(CONFIG_AT32_AT32F43XX)

#define AT32_FLASH_PSR             (AT32_FLASHIF_BASE+AT32_FLASH_PSR_OFFSET)
#define AT32_FLASH_UNLOCK          (AT32_FLASHIF_BASE+AT32_FLASH_UNLOCK_OFFSET)
#define AT32_FLASH_USD_UNLOCK      (AT32_FLASHIF_BASE+AT32_FLASH_USD_UNLOCK_OFFSET)
#define AT32_FLASH_STS             (AT32_FLASHIF_BASE+AT32_FLASH_STS_OFFSET)
#define AT32_FLASH_CTRL            (AT32_FLASHIF_BASE+AT32_FLASH_CTRL_OFFSET)
#define AT32_FLASH_ADDR            (AT32_FLASHIF_BASE+AT32_FLASH_ADDR_OFFSET)
#define AT32_FLASH_USD             (AT32_FLASHIF_BASE+AT32_FLASH_USD_OFFSET)
#define AT32_FLASH_EPPS0           (AT32_FLASHIF_BASE+AT32_FLASH_EPPS0_OFFSET)
#define AT32_FLASH_EPPS1           (AT32_FLASHIF_BASE+AT32_FLASH_EPPS1_OFFSET)
#define AT32_FLASH_UNLOCK2         (AT32_FLASHIF_BASE+AT32_FLASH_UNLOCK2_OFFSET)
#define AT32_FLASH_STS2            (AT32_FLASHIF_BASE+AT32_FLASH_STS2_OFFSET)
#define AT32_FLASH_CTRL2           (AT32_FLASHIF_BASE+AT32_FLASH_CTRL2_OFFSET)
#define AT32_FLASH_ADDR2           (AT32_FLASHIF_BASE+AT32_FLASH_ADDR2_OFFSET)
#define AT32_FLASH_CONTR           (AT32_FLASHIF_BASE+AT32_FLASH_CONTR_OFFSET)
#define AT32_FLASH_DIVR            (AT32_FLASHIF_BASE+AT32_FLASH_DIVR_OFFSET)
#define AT32_SLIB_STS2             (AT32_FLASHIF_BASE+AT32_SLIB_STS2_OFFSET)
#define AT32_SLIB_STS0             (AT32_FLASHIF_BASE+AT32_SLIB_STS0_OFFSET)
#define AT32_SLIB_STS1             (AT32_FLASHIF_BASE+AT32_SLIB_STS1_OFFSET)
#define AT32_SLIB_PWD_CRL          (AT32_FLASHIF_BASE+AT32_SLIB_PWD_CRL_OFFSET)
#define AT32_SLIB_MISC_STS         (AT32_FLASHIF_BASE+AT32_SLIB_MISC_STS_OFFSET)
#define AT32_SLIB_SET_PWD          (AT32_FLASHIF_BASE+AT32_SLIB_SET_PWD_OFFSET)
#define AT32_SLIB_SET_RANGE0       (AT32_FLASHIF_BASE+AT32_SLIB_SET_RANGE0_OFFSET)
#define AT32_SLIB_SET_RANGE1       (AT32_FLASHIF_BASE+AT32_SLIB_SET_RANGE1_OFFSET)
#define AT32_SLIB_UNLOCK           (AT32_FLASHIF_BASE+AT32_SLIB_UNLOCK_OFFSET)
#define AT32_FLASH_CRC_CTRL        (AT32_FLASHIF_BASE+AT32_FLASH_CRC_CTRL_OFFSET)
#define AT32_FLASH_CRC_CHKR        (AT32_FLASHIF_BASE+AT32_FLASH_CRC_CHKR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (PSR) */
#if defined(CONFIG_AT32_AT32F43XX)
#define FLASH_PSR_NZW_BST           (1 << 12) /* Flash non-zero wait area boost */
#define FLASH_PSR_NZW_BST_STS       (1 << 13) /* Flash non-zero wait area boost status */
#endif

/* Flash status Register1(for bank1) */
#if defined(CONFIG_AT32_AT32F43XX)
#define FLASH_STS_OBF               (1 << 0) /* Operation busy flag */
#define FLASH_STS_PRGMERR           (1 << 2) /* Program error */
#define FLASH_STS_EPPERR            (1 << 4) /* Erase/Program protection error */
#define FLASH_STS_ODF               (1 << 5) /* Operation done flag */
#endif

/* Flash control Register(for bank1) */
#if defined(CONFIG_AT32_AT32F43XX)
#define FLASH_CTRL_FPRGM            (1 << 0)  /* Flash program */
#define FLASH_CTRL_SECERS           (1 << 1)  /* Sector erase */
#define FLASH_CTRL_BANKERS          (1 << 2)  /* Bank erase */
#define FLASH_CTRL_BLKERS           (1 << 3)  /* Block erase */
#define FLASH_CTRL_USDPRGM          (1 << 4)  /* User system data program */
#define FLASH_CTRL_USDERS           (1 << 5)  /* User system data erase */
#define FLASH_CTRL_ERSTR            (1 << 6)  /* Erasing start */
#define FLASH_CTRL_OPLK             (1 << 7)  /* Operation lock */
#define FLASH_CTRL_USDULKS          (1 << 9)  /* User system data unlock success */
#define FLASH_CTRL_ERRIE            (1 << 10) /* Error interrupt enable */
#define FLASH_CTRL_ODFIE            (1 << 12) /* Operation done flag interrupt enable */
#endif

/* Flash address Register(for bank1) */

#define FLASH_ADDR_FA_SHIFT         (0) /* Flash address */
#define FLASH_ADDR_FA_MASK          (0xffffffff << FLASH_ADDR_FA_SHIFT)
#define FLASH_ADDR_FA(X)            ((X) << FLASH_ADDR_FA_SHIFT)

/* Flash continue read enable */

#define FLASH_CONTR_EN_SHIFT        (31)
#define FLASH_CONTR_EN_MASK         (1 << FLASH_CONTR_EN_SHIFT)
#define FLASH_CONTR_EN              (1 << FLASH_CONTR_EN_SHIFT)

/* Flash divider */

#define FLASH_DIVR_FDIV_SHIFT       (0) /* Flash divider */
#define FLASH_DIVR_FDIV_MASK        (3 << FLASH_DIVR_FDIV_SHIFT)
#define FLASH_DIVR_FDIV_2           (0 << FLASH_DIVR_FDIV_SHIFT)
#define FLASH_DIVR_FDIV_3           (1 << FLASH_DIVR_FDIV_SHIFT)
#define FLASH_DIVR_FDIV_4           (2 << FLASH_DIVR_FDIV_SHIFT)

#define FLASH_DIVR_FDIV_STS_SHIFT   (4) /* Flash divider status */
#define FLASH_DIVR_FDIV_STS_MASK    (3 << FLASH_DIVR_FDIV_STS_SHIFT)
#define FLASH_DIVR_FDIV_STS_2       (0 << FLASH_DIVR_FDIV_STS_SHIFT)
#define FLASH_DIVR_FDIV_STS_3       (1 << FLASH_DIVR_FDIV_STS_SHIFT)
#define FLASH_DIVR_FDIV_STS_4       (2 << FLASH_DIVR_FDIV_STS_SHIFT)

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int at32_flash_lock(void);
int at32_flash_unlock(void);

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32_FLASH_H */
