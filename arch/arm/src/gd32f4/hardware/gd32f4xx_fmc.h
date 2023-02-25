/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_fmc.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_FMC_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_FMC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _K(x) ((x)*1024)

#if !defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULT) && \
    !defined(CONFIG_GD32F4_FLASH_CONFIG_E) && \
    !defined(CONFIG_GD32F4_FLASH_CONFIG_F) && \
    !defined(CONFIG_GD32F4_FLASH_CONFIG_I) && \
    !defined(CONFIG_GD32F4_FLASH_CONFIG_K)
#  define CONFIG_GD32F4_FLASH_CONFIG_DEFAULT
#endif

/* Flash default size is 512K */
#if defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULT)

#  define GD32_FMC_NSECTORS             8
#  define GD32_FMC_SIZE                 _K((4 * 16) + (1 * 64) + (3 * 128))
#  define GD32_FMC_SIZES                {_K(16), _K(16), _K(16), _K(16), \
                                        _K(64),_K(128), _K(128), _K(128)}

/* Override of the FLASH Has been Chosen */

#elif !defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULT)

#  if defined(CONFIG_GD32F4_FLASH_CONFIG_E)
#    define GD32_FMC_NSECTORS          8
#    define GD32_FMC_SIZE              _K((4 * 16) + (1 * 64) + (3 * 128))
#    define GD32_FMC_SIZES             {_K(16), _K(16), _K(16), _K(16),  \
                                         _K(64), _K(128), _K(128), _K(128)}

#  elif defined(CONFIG_GD32F4_FLASH_CONFIG_G)
#    define GD32_FMC_NSECTORS          12
#    define GD32_FMC_SIZE              _K((4 * 16) + (1 * 64) + (7 * 128))
#    define GD32_FMC_SIZES             {_K(16), _K(16), _K(16), _K(16),    \
                                         _K(64), _K(128), _K(128), _K(128),  \
                                         _K(128), _K(128), _K(128), _K(128)}

#  elif defined(CONFIG_GD32F4_FLASH_CONFIG_I)
#    define GD32_FMC_NSECTORS          24
#    define GD32_FMC_SIZE              _K((4 * 16) + (1 * 64) + (7 * 128)) + \
                                         _K((4 * 16) + (1 * 64) + (7 * 128))
#    define GD32_FMC_SIZES             {_K(16), _K(16), _K(16), _K(16),      \
                                         _K(64), _K(128), _K(128), _K(128),    \
                                         _K(128), _K(128), _K(128), _K(128),   \
                                         _K(16), _K(16), _K(16), _K(16),       \
                                         _K(64), _K(128), _K(128), _K(128),    \
                                         _K(128), _K(128), _K(128), _K(128)}

#  elif defined(CONFIG_GD32F4_FLASH_CONFIG_K)
#    define GD32_FMC_NSECTORS          28
#    define GD32_FMC_SIZE              _K((4 * 16) + (1 * 64) + (7 * 128)) + \
                                         _K((4 * 16) + (1 * 64) + (7 * 128))
#    define GD32_FMC_SIZES             {_K(16), _K(16), _K(16), _K(16),      \
                                         _K(64), _K(128), _K(128), _K(128),    \
                                         _K(128), _K(128), _K(128), _K(128),   \
                                         _K(16), _K(16), _K(16), _K(16),       \
                                         _K(64), _K(128), _K(128), _K(128),    \
                                         _K(128), _K(128), _K(128), _K(128)},  \
                                         _K(256), _K(256), _K(256), _K(256)}

#  endif

#endif /* !defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULT) */

/* Register Offsets *********************************************************/

#define GD32_FMC_WS_OFFSET               0x0000 /* FMC wait state register offset */
#define GD32_FMC_KEY_OFFSET              0x0004 /* FMC unlock key register offset */
#define GD32_FMC_OBKEY_OFFSET            0x0008 /* FMC option byte register offset */
#define GD32_FMC_STAT_OFFSET             0x000C /* FMC status register offset */
#define GD32_FMC_CTL_OFFSET              0x0010 /* FMC control register offset */
#define GD32_FMC_OBCTL0_OFFSET           0x0014 /* FMC option byte control register 0 offset */
#define GD32_FMC_OBCTL1_OFFSET           0x0018 /* FMC option byte control register 1 offset */
#define GD32_FMC_WSEN_OFFSET             0x00FC /* FMC wait state enable register offset */
#define GD32_FMC_PID_OFFSET              0x0100 /* FMC product ID register offset */

/* Register Addresses *******************************************************/

#define GD32_FMC_WS                      (GD32_FMC_BASE+GD32_FMC_WS_OFFSET)     /* FMC wait state register */
#define GD32_FMC_KEY                     (GD32_FMC_BASE+GD32_FMC_KEY_OFFSET)    /* FMC unlock key register */
#define GD32_FMC_OBKEY                   (GD32_FMC_BASE+GD32_FMC_OBKEY_OFFSET)  /* FMC option byte register */
#define GD32_FMC_STAT                    (GD32_FMC_BASE+GD32_FMC_STAT_OFFSET)   /* FMC status register */
#define GD32_FMC_CTL                     (GD32_FMC_BASE+GD32_FMC_CTL_OFFSET)    /* FMC control register */
#define GD32_FMC_OBCTL0                  (GD32_FMC_BASE+GD32_FMC_OBCTL0_OFFSET) /* FMC option byte control register 0 */
#define GD32_FMC_OBCTL1                  (GD32_FMC_BASE+GD32_FMC_OBCTL1_OFFSET) /* FMC option byte control register 1 */
#define GD32_FMC_WSEN                    (GD32_FMC_BASE+GD32_FMC_WSEN_OFFSET)   /* FMC wait state enable register */
#define GD32_FMC_PID                     (GD32_FMC_BASE+GD32_FMC_PID_OFFSET)    /* FMC product ID register */

/* Register Bitfield Definitions ********************************************/

/* FMC wait state register */

#define FMC_WS_WSCNT_SHIFT               (0)
#define FMC_WS_WSCNT_MASK                (15 << FMC_WS_WSCNT_SHIFT)
#define FMC_WC_WSCNT(n)                  ((n) << FMC_WS_WSCNT_SHIFT)
#define FMC_WS_WSCNT_0                   WC_WSCNT(0)  /* FMC 0 wait */
#define FMC_WS_WSCNT_1                   WC_WSCNT(1)  /* FMC 1 wait */
#define FMC_WS_WSCNT_2                   WC_WSCNT(2)  /* FMC 2 wait */
#define FMC_WS_WSCNT_3                   WC_WSCNT(3)  /* FMC 3 wait */
#define FMC_WS_WSCNT_4                   WC_WSCNT(4)  /* FMC 4 wait */
#define FMC_WS_WSCNT_5                   WC_WSCNT(5)  /* FMC 5 wait */
#define FMC_WS_WSCNT_6                   WC_WSCNT(6)  /* FMC 6 wait */
#define FMC_WS_WSCNT_7                   WC_WSCNT(7)  /* FMC 7 wait */
#define FMC_WS_WSCNT_8                   WC_WSCNT(8)  /* FMC 8 wait */
#define FMC_WS_WSCNT_9                   WC_WSCNT(9)  /* FMC 9 wait */
#define FMC_WS_WSCNT_10                  WC_WSCNT(10) /* FMC 10 wait */
#define FMC_WS_WSCNT_11                  WC_WSCNT(11) /* FMC 11 wait */
#define FMC_WS_WSCNT_12                  WC_WSCNT(12) /* FMC 12 wait */
#define FMC_WS_WSCNT_13                  WC_WSCNT(13) /* FMC 13 wait */
#define FMC_WS_WSCNT_14                  WC_WSCNT(14) /* FMC 14 wait */
#define FMC_WS_WSCNT_15                  WC_WSCNT(15) /* FMC 15 wait */

/* FMC status register */

#define FMC_STAT_END                     (1 << 0)  /* Bit 0: end of operation flag bit */
#define FMC_STAT_OPERR                   (1 << 1)  /* Bit 1: flash operation error flag bit */
#define FMC_STAT_WPERR                   (1 << 4)  /* Bit 4: erase/Program protection error flag bit */
#define FMC_STAT_PGMERR                  (1 << 5)  /* Bit 5: program size not match error flag bit */
#define FMC_STAT_PGSERR                  (1 << 6)  /* Bit 6: program sequence error flag bit */
#define FMC_STAT_RDDERR                  (1 << 7)  /* Bit 7: read D-bus protection error flag bit */
#define FMC_STAT_BUSY                    (1 << 16) /* Bit 16: flash busy flag bit */

/* FMC control register */

#define FMC_CTL_PG                       (1 << 0)               /* Bit 0: main flash program command bit */
#define FMC_CTL_SER                      (1 << 1)               /* Bit 1: main flash sector erase command bit */
#define FMC_CTL_MER0                     (1 << 2)               /* Bit 2: main flash mass erase for bank0 command bit */

#define FMC_CTL_SN_SHIFT                 (3)                    /* Bits 3-7: select which sector number to be erased */
#define FMC_CTL_SN_MASK                  (31 << FMC_CTL_SN_SHIFT)
#define FMC_CTL_SN(n)                    ((n) << FMC_CTL_SN_SHIFT)     /* Sector n, n=0..11 */
#define FMC_CTL_SN_0_11(n)               ((n) << FMC_CTL_SN_SHIFT)     /* Sector n, n=0..11 */
#define FMC_CTL_SN_12_23(n)              ((n+4) << FMC_CTL_SN_SHIFT))  /* Sector n, n=12..23 */
#define FMC_CTL_SN_24_27(n)              ((n-12) << FMC_CTL_SN_SHIFT)) /* Sector n, n=24..27 */
#define FMC_CTL_SN_0                     (0 << FMC_CTL_SN_SHIFT)       /* sector 0 */
#define FMC_CTL_SN_1                     (1 << FMC_CTL_SN_SHIFT)       /* sector 1 */
#define FMC_CTL_SN_2                     (2 << FMC_CTL_SN_SHIFT)       /* sector 2 */
#define FMC_CTL_SN_3                     (3 << FMC_CTL_SN_SHIFT)       /* sector 3 */
#define FMC_CTL_SN_4                     (4 << FMC_CTL_SN_SHIFT)       /* sector 4 */
#define FMC_CTL_SN_5                     (5 << FMC_CTL_SN_SHIFT)       /* sector 5 */
#define FMC_CTL_SN_6                     (6 << FMC_CTL_SN_SHIFT)       /* sector 6 */
#define FMC_CTL_SN_7                     (7 << FMC_CTL_SN_SHIFT)       /* sector 7 */
#define FMC_CTL_SN_8                     (8 << FMC_CTL_SN_SHIFT)       /* sector 8 */
#define FMC_CTL_SN_9                     (9 << FMC_CTL_SN_SHIFT)       /* sector 9 */
#define FMC_CTL_SN_10                    (10 << FMC_CTL_SN_SHIFT)      /* sector 10 */
#define FMC_CTL_SN_11                    (11 << FMC_CTL_SN_SHIFT)      /* sector 11 */
#define FMC_CTL_SN_12                    (16 << FMC_CTL_SN_SHIFT)      /* sector 12 */
#define FMC_CTL_SN_13                    (17 << FMC_CTL_SN_SHIFT)      /* sector 13 */
#define FMC_CTL_SN_14                    (18 << FMC_CTL_SN_SHIFT)      /* sector 14 */
#define FMC_CTL_SN_15                    (19 << FMC_CTL_SN_SHIFT)      /* sector 15 */
#define FMC_CTL_SN_16                    (20 << FMC_CTL_SN_SHIFT)      /* sector 16 */
#define FMC_CTL_SN_17                    (21 << FMC_CTL_SN_SHIFT)      /* sector 17 */
#define FMC_CTL_SN_18                    (22 << FMC_CTL_SN_SHIFT)      /* sector 18 */
#define FMC_CTL_SN_19                    (23 << FMC_CTL_SN_SHIFT)      /* sector 19 */
#define FMC_CTL_SN_20                    (24 << FMC_CTL_SN_SHIFT)      /* sector 20 */
#define FMC_CTL_SN_21                    (25 << FMC_CTL_SN_SHIFT)      /* sector 21 */
#define FMC_CTL_SN_22                    (26 << FMC_CTL_SN_SHIFT)      /* sector 22 */
#define FMC_CTL_SN_23                    (27 << FMC_CTL_SN_SHIFT)      /* sector 23 */
#define FMC_CTL_SN_24                    (12 << FMC_CTL_SN_SHIFT)      /* sector 24 */
#define FMC_CTL_SN_25                    (13 << FMC_CTL_SN_SHIFT)      /* sector 25 */
#define FMC_CTL_SN_26                    (14 << FMC_CTL_SN_SHIFT)      /* sector 26 */
#define FMC_CTL_SN_27                    (15 << FMC_CTL_SN_SHIFT)      /* sector 27 */

#define FMC_CTL_PSZ_SHIFT                (8) /* Bits 8-9: Program size */
#define FMC_CTL_PSZ_MASK                 (3 << FMC_CTL_PSZ_SHIFT)
#define FMC_CTL_PSZ_BYTE                 (0 << FMC_CTL_PSZ_SHIFT) /* FMC program by byte access */
#define FMC_CTL_PSZ_HALF_WORD            (1 << FMC_CTL_PSZ_SHIFT) /* FMC program by half-word access */
#define FMC_CTL_PSZ_WORD                 (2 << FMC_CTL_PSZ_SHIFT) /* FMC program by word access */

#define FMC_CTL_MER1                     (1 << 15) /* Bit 15: main flash mass erase for bank1 command bit */
#define FMC_CTL_START                    (1 << 16) /* Bit 16: send erase command to FMC bit */
#define FMC_CTL_ENDIE                    (1 << 24) /* Bit 24: end of operation interrupt enable bit */
#define FMC_CTL_ERRIE                    (1 << 25) /* Bit 25: error interrupt enable bit */
#define FMC_CTL_LK                       (1 << 31) /* Bit 31: FMC_CTL lock bit */

/* FMC option byte control register 0 */

#define FMC_OBCTL0_OB_LK                 (1 << 0) /* Bit 0: FMC_OBCTL0 lock bit */
#define FMC_OBCTL0_OB_START              (1 << 1) /* Bit 1: send option byte change command to FMC bit */

#define FMC_OBCTL0_BOR_TH_SHIFT          (2) /* Bits 2-3: Boption byte BOR threshold value */
#define FMC_OBCTL0_BOR_TH_MASK           (3 << FLASH_OPTCR_BORLEV_SHIFT)
#define FMC_OB_BOR_TH_VALUE3             (0 << FLASH_OPTCR_BORLEV_SHIFT) /* BOR threshold value 3 */
#define FMC_OB_BOR_TH_VALUE2             (1 << FLASH_OPTCR_BORLEV_SHIFT) /* BOR threshold value 2 */
#define FMC_OB_BOR_TH_VALUE1             (2 << FLASH_OPTCR_BORLEV_SHIFT) /* BOR threshold value 1 */
#define FMC_OB_BOR_TH_VALUE0             (3 << FLASH_OPTCR_BORLEV_SHIFT) /* no BOR function */

#define FMC_OBCTL0_BB                    (1 << 4) /* Bit 4: option byte boot bank value */
#define FMC_OBCTL0_NWDG_HW               (1 << 5) /* Bit 5: option byte watchdog value */
#define FMC_OBCTL0_NRST_DPSLP            (1 << 5) /* Bit 6: option byte deepsleep value */
#define FMC_OBCTL0_NRST_STDBY            (1 << 7) /* Bit 7: option byte standby value */

#define FMC_OBCTL0_SPC_SHIFT             (8) /* Bits 8-15: option byte Security Protection code */
#define FMC_OBCTL0_SPC_MASK              (0xff << FMC_OBCTL0_SPC_SHIFT)
#define FMC_OBCTL0_WP0_SHIFT             (16) /* Bits 16-27: erase/program protection of each sector when DRP is 0 */
#define FMC_OBCTL0_WP0_MASK              (0xfff << FMC_OBCTL0_WP0_SHIFT)

#define FMC_OBCTL0_DBS                   (1 << 30) /* Bit 30: double banks or single bank selection when flash size is 1M bytes */
#define FMC_OBCTL0_DRP                   (1 << 31) /* Bit 31: D-bus read protection bit */

/* FMC option byte control register 1 */

#define FMC_OBCTL1_WP1_SHIFT        (16) /* Bits 16-27: erase/program protection of each sector when DRP is 0 */
#define FMC_OBCTL1_WP1_MASK              (0xfff << FMC_OBCTL1_WP1_SHIFT)

/* FMC wait state enable register */
#define FMC_WSEN_WSEN                    (1 << 0) /* FMC wait state enable bit */

/* FMC unlock key */
#define FMC_UNLOCK_KEY0                  (0x45670123)       /* Unlock key 0 */
#define FMC_UNLOCK_KEY1                  (0xCDEF89AB)       /* Unlock key 1 */
#define FMC_OB_UNLOCK_KEY0               (0x08192A3B)       /* ob unlock key 0 */
#define FMC_OB_UNLOCK_KEY1               (0x4C5D6E7F)       /* ob unlock key 1 */

/* option bytes write protection */
#define FMC_OB_WP_0                      (0x00000001)        /* erase/program protection of sector 0  */
#define FMC_OB_WP_1                      (0x00000002)        /* erase/program protection of sector 1  */
#define FMC_OB_WP_2                      (0x00000004)        /* erase/program protection of sector 2  */
#define FMC_OB_WP_3                      (0x00000008)        /* erase/program protection of sector 3  */
#define FMC_OB_WP_4                      (0x00000010)        /* erase/program protection of sector 4  */
#define FMC_OB_WP_5                      (0x00000020)        /* erase/program protection of sector 5  */
#define FMC_OB_WP_6                      (0x00000040)        /* erase/program protection of sector 6  */
#define FMC_OB_WP_7                      (0x00000080)        /* erase/program protection of sector 7  */
#define FMC_OB_WP_8                      (0x00000100)        /* erase/program protection of sector 8  */
#define FMC_OB_WP_9                      (0x00000200)        /* erase/program protection of sector 9  */
#define FMC_OB_WP_10                     (0x00000400)        /* erase/program protection of sector 10 */
#define FMC_OB_WP_11                     (0x00000800)        /* erase/program protection of sector 11 */
#define FMC_OB_WP_12                     (0x00010000)        /* erase/program protection of sector 12 */
#define FMC_OB_WP_13                     (0x00020000)        /* erase/program protection of sector 13 */
#define FMC_OB_WP_14                     (0x00040000)        /* erase/program protection of sector 14 */
#define FMC_OB_WP_15                     (0x00080000)        /* erase/program protection of sector 15 */
#define FMC_OB_WP_16                     (0x00100000)        /* erase/program protection of sector 16 */
#define FMC_OB_WP_17                     (0x00200000)        /* erase/program protection of sector 17 */
#define FMC_OB_WP_18                     (0x00400000)        /* erase/program protection of sector 18 */
#define FMC_OB_WP_19                     (0x00800000)        /* erase/program protection of sector 19 */
#define FMC_OB_WP_20                     (0x01000000)        /* erase/program protection of sector 20 */
#define FMC_OB_WP_21                     (0x02000000)        /* erase/program protection of sector 21 */
#define FMC_OB_WP_22                     (0x04000000)        /* erase/program protection of sector 22 */
#define FMC_OB_WP_23_27                  (0x08000000)        /* erase/program protection of sector 23~27 */
#define FMC_OB_WP_ALL                    (0x0FFF0FFF)        /* erase/program protection of all sectors */

/* option bytes D-bus read protection */
#define FMC_OB_DRP_0                     (0x00000001)        /* D-bus read protection protection of sector 0  */
#define FMC_OB_DRP_1                     (0x00000002)        /* D-bus read protection protection of sector 1  */
#define FMC_OB_DRP_2                     (0x00000004)        /* D-bus read protection protection of sector 2  */
#define FMC_OB_DRP_3                     (0x00000008)        /* D-bus read protection protection of sector 3  */
#define FMC_OB_DRP_4                     (0x00000010)        /* D-bus read protection protection of sector 4  */
#define FMC_OB_DRP_5                     (0x00000020)        /* D-bus read protection protection of sector 5  */
#define FMC_OB_DRP_6                     (0x00000040)        /* D-bus read protection protection of sector 6  */
#define FMC_OB_DRP_7                     (0x00000080)        /* D-bus read protection protection of sector 7  */
#define FMC_OB_DRP_8                     (0x00000100)        /* D-bus read protection protection of sector 8  */
#define FMC_OB_DRP_9                     (0x00000200)        /* D-bus read protection protection of sector 9  */
#define FMC_OB_DRP_10                    (0x00000400)        /* D-bus read protection protection of sector 10 */
#define FMC_OB_DRP_11                    (0x00000800)        /* D-bus read protection protection of sector 11 */
#define FMC_OB_DRP_12                    (0x00010000)        /* D-bus read protection protection of sector 12 */
#define FMC_OB_DRP_13                    (0x00020000)        /* D-bus read protection protection of sector 13 */
#define FMC_OB_DRP_14                    (0x00040000)        /* D-bus read protection protection of sector 14 */
#define FMC_OB_DRP_15                    (0x00080000)        /* D-bus read protection protection of sector 15 */
#define FMC_OB_DRP_16                    (0x00100000)        /* D-bus read protection protection of sector 16 */
#define FMC_OB_DRP_17                    (0x00200000)        /* D-bus read protection protection of sector 17 */
#define FMC_OB_DRP_18                    (0x00400000)        /* D-bus read protection protection of sector 18 */
#define FMC_OB_DRP_19                    (0x00800000)        /* D-bus read protection protection of sector 19 */
#define FMC_OB_DRP_20                    (0x01000000)        /* D-bus read protection protection of sector 20 */
#define FMC_OB_DRP_21                    (0x02000000)        /* D-bus read protection protection of sector 21 */
#define FMC_OB_DRP_22                    (0x04000000)        /* D-bus read protection protection of sector 22 */
#define FMC_OB_DRP_23_27                 (0x08000000)        /* D-bus read protection protection of sector 23~27 */
#define FMC_OB_DRP_ALL                   (0x0FFF0FFF)        /* D-bus read protection protection of all sectors */

/* FMC time out */
#define FMC_TIMEOUT_COUNT                (0x4FFFFFFF)        /* count to judge of FMC timeout */

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_FMC_H */
