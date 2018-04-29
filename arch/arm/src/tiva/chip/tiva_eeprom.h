/************************************************************************************
 * arch/arm/src/tiva/chip/tiva_eeprom.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Shirshak Sengupta <sgshirshak@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_TIVA_EEPROM_H
#define __ARCH_ARM_SRC_TIVA_CHIP_TIVA_EEPROM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define TIVA_EEPROM_EESIZE_OFFSET     0x0000  /* EEPROM Size Information */
#define TIVA_EEPROM_EEBLOCK_OFFSET    0x0004  /* EEPROM Current Block */
#define TIVA_EEPROM_EEOFFSET_OFFSET   0x0008  /* EEPROM Current Offset */
#define TIVA_EEPROM_EERDWR_OFFSET     0x0010  /* EEPROM Read-Write */
#define TIVA_EEPROM_EERDWRINC_OFFSET  0x0014  /* EEPROM Read-Write with Increment */
#define TIVA_EEPROM_EEDONE_OFFSET     0x0018  /* EEPROM Done Status */
#define TIVA_EEPROM_EESUPP_OFFSET     0x001c  /* EEPROM Support Control and Status */
#define TIVA_EEPROM_EEUNLOCK_OFFSET   0x0020  /* EEPROM Unlock */
#define TIVA_EEPROM_EEPROT_OFFSET     0x0030  /* EEPROM Protection */
#define TIVA_EEPROM_EEPASS0_OFFSET    0x0034  /* EEPROM Password */
#define TIVA_EEPROM_EEPASS1_OFFSET    0x0038  /* EEPROM Password */
#define TIVA_EEPROM_EEPASS2_OFFSET    0x003c  /* EEPROM Password */
#define TIVA_EEPROM_EEINT_OFFSET      0x0040  /* EEPROM Interrupt */
#define TIVA_EEPROM_EEHIDE_OFFSET     0x0050  /* EEPROM Block Hide */
#define TIVA_EEPROM_EEHIDE0_OFFSET    0x0050  /* EEPROM Block Hide 0 */
#define TIVA_EEPROM_EEHIDE1_OFFSET    0x0054  /* EEPROM Block Hide 1 */
#define TIVA_EEPROM_EEHIDE2_OFFSET    0x0058  /* EEPROM Block Hide 2 */
#define TIVA_EEPROM_EEDBGME_OFFSET    0x0080  /* EEPROM Debug Mass Erase */
#define TIVA_EEPROM_PP_OFFSET         0x0fc0  /* EEPROM Peripheral Properties */

/* Register Addresses ***************************************************************/

#define TIVA_EEPROM_EESIZE            (TIVA_EEPROM_BASE + TIVA_EEPROM_EESIZE_OFFSET)
#define TIVA_EEPROM_EEBLOCK           (TIVA_EEPROM_BASE + TIVA_EEPROM_EEBLOCK_OFFSET)
#define TIVA_EEPROM_EEOFFSET          (TIVA_EEPROM_BASE + TIVA_EEPROM_EEOFFSET_OFFSET)
#define TIVA_EEPROM_EERDWR            (TIVA_EEPROM_BASE + TIVA_EEPROM_EERDWR_OFFSET)
#define TIVA_EEPROM_EERDWRINC         (TIVA_EEPROM_BASE + TIVA_EEPROM_EERDWRINC_OFFSET)
#define TIVA_EEPROM_EEDONE            (TIVA_EEPROM_BASE + TIVA_EEPROM_EEDONE_OFFSET)
#define TIVA_EEPROM_EESUPP            (TIVA_EEPROM_BASE + TIVA_EEPROM_EESUPP_OFFSET)
#define TIVA_EEPROM_EEUNLOCK          (TIVA_EEPROM_BASE + TIVA_EEPROM_EEUNLOCK_OFFSET)
#define TIVA_EEPROM_EEPROT            (TIVA_EEPROM_BASE + TIVA_EEPROM_EEPROT_OFFSET)
#define TIVA_EEPROM_EEPASS0           (TIVA_EEPROM_BASE + TIVA_EEPROM_EEPASS0_OFFSET)
#define TIVA_EEPROM_EEPASS1           (TIVA_EEPROM_BASE + TIVA_EEPROM_EEPASS1_OFFSET)
#define TIVA_EEPROM_EEPASS2           (TIVA_EEPROM_BASE + TIVA_EEPROM_EEPASS2_OFFSET)
#define TIVA_EEPROM_EEINT             (TIVA_EEPROM_BASE + TIVA_EEPROM_EEINT_OFFSET)
#define TIVA_EEPROM_EEHIDE0           (TIVA_EEPROM_BASE + TIVA_EEPROM_EEHIDE0_OFFSET)
#define TIVA_EEPROM_EEHIDE            (TIVA_EEPROM_BASE + TIVA_EEPROM_EEHIDE_OFFSET)
#define TIVA_EEPROM_EEHIDE1           (TIVA_EEPROM_BASE + TIVA_EEPROM_EEHIDE1_OFFSET)
#define TIVA_EEPROM_EEHIDE2           (TIVA_EEPROM_BASE + TIVA_EEPROM_EEHIDE2_OFFSET)
#define TIVA_EEPROM_EEDBGME           (TIVA_EEPROM_BASE + TIVA_EEPROM_EEDBGME_OFFSET)
#define TIVA_EEPROM_PP                (TIVA_EEPROM_BASE + TIVA_EEPROM_PP_OFFSET)

/* Register Bit-Field Definitions ***************************************************/
/* The following are defines for the bit fields in the EEPROM_EESIZE register. */

#define EEPROM_EESIZE_WORDCNT_M       0x0000ffff  /* Number of 32-Bit Words */
#define EEPROM_EESIZE_BLKCNT_M        0x07ff0000  /* Number of 16-Word Blocks */
#define EEPROM_EESIZE_WORDCNT_S       0
#define EEPROM_EESIZE_BLKCNT_S        16

/* The following are defines for the bit fields in the EEPROM_EEBLOCK register. */

#define EEPROM_EEBLOCK_BLOCK_M        0x0000ffff  /* Current Block */
#define EEPROM_EEBLOCK_BLOCK_S        0

/* The following are defines for the bit fields in the EEPROM_EEOFFSET register. */

#define EEPROM_EEOFFSET_OFFSET_M      0x0000000f  /* Current Address Offset */
#define EEPROM_EEOFFSET_OFFSET_S      0

/* The following are defines for the bit fields in the EEPROM_EERDWR register. */

#define EEPROM_EERDWR_VALUE_M         0xffffffff  /* EEPROM Read or Write Data */
#define EEPROM_EERDWR_VALUE_S         0

/* The following are defines for the bit fields in the EEPROM_EERDWRINC register. */

#define EEPROM_EERDWRINC_VALUE_M      0xffffffff  /* EEPROM Read or Write Data with Increment */
#define EEPROM_EERDWRINC_VALUE_S      0

/* The following are defines for the bit fields in the EEPROM_EEDONE register. */

#define EEPROM_EEDONE_WORKING         0x00000001  /* EEPROM Working */
#define EEPROM_EEDONE_WKERASE         0x00000004  /* Working on an Erase */
#define EEPROM_EEDONE_WKCOPY          0x00000008  /* Working on a Copy */
#define EEPROM_EEDONE_NOPERM          0x00000010  /* Write Without Permission */
#define EEPROM_EEDONE_WRBUSY          0x00000020  /* Write Busy */

/* The following are defines for the bit fields in the EEPROM_EESUPP register. */

#define EEPROM_EESUPP_ERETRY          0x00000004  /* Erase Must Be Retried */
#define EEPROM_EESUPP_PRETRY          0x00000008  /* Programming Must Be Retried */

/* The following are defines for the bit fields in the EEPROM_EEUNLOCK register. */

#define EEPROM_EEUNLOCK_UNLOCK_M      0xffffffff  /* EEPROM Unlock */

/* The following are defines for the bit fields in the EEPROM_EEPROT register. */

#define EEPROM_EEPROT_PROT_M          0x00000007  /* Protection Control */
#define EEPROM_EEPROT_PROT_RWNPW      0x00000000  /* This setting is the default. If
                                                   * there is no password, the block
                                                   * is not protected and is readable
                                                   * and writable */
#define EEPROM_EEPROT_PROT_RWPW       0x00000001  /* If there is a password, the
                                                   * block is readable or writable
                                                   * only when unlocked */
#define EEPROM_EEPROT_PROT_RONPW      0x00000002  /* If there is no password, the
                                                   * block is readable, not writable */
#define EEPROM_EEPROT_ACC             0x00000008  /* Access Control */

/* The following are defines for the bit fields in the EEPROM_EEPASS0 register. */

#define EEPROM_EEPASS0_PASS_M         0xffffffff  /* Password */
#define EEPROM_EEPASS0_PASS_S         0

/* The following are defines for the bit fields in the EEPROM_EEPASS1 register. */

#define EEPROM_EEPASS1_PASS_M         0xffffffff  /* Password */
#define EEPROM_EEPASS1_PASS_S         0

/* The following are defines for the bit fields in the EEPROM_EEPASS2 register. */

#define EEPROM_EEPASS2_PASS_M         0xffffffff  /* Password */
#define EEPROM_EEPASS2_PASS_S         0

/* The following are defines for the bit fields in the EEPROM_EEINT register. */

#define EEPROM_EEINT_INT              0x00000001  /* Interrupt Enable */

/* The following are defines for the bit fields in the EEPROM_EEHIDE0 register. */

#define EEPROM_EEHIDE0_HN_M           0xfffffffe  /* Hide Block */

/* The following are defines for the bit fields in the EEPROM_EEHIDE register. */

#define EEPROM_EEHIDE_HN_M            0xfffffffe  /* Hide Block */

/* The following are defines for the bit fields in the EEPROM_EEHIDE1 register. */

#define EEPROM_EEHIDE1_HN_M           0xffffffff  /* Hide Block */

/* The following are defines for the bit fields in the EEPROM_EEHIDE2 register. */

#define EEPROM_EEHIDE2_HN_M           0xffffffff  /* Hide Block */

/* The following are defines for the bit fields in the EEPROM_EEDBGME register. */

#define EEPROM_EEDBGME_ME             0x00000001  /* Mass Erase */
#define EEPROM_EEDBGME_KEY_M          0xffff0000  /* Erase Key */
#define EEPROM_EEDBGME_KEY_S          16

/* The following are defines for the bit fields in the EEPROM_PP register. */

#define EEPROM_PP_SIZE_M              0x0000ffff  /* EEPROM Size */
#define EEPROM_PP_SIZE_64             0x00000000  /* 64 bytes of EEPROM */
#define EEPROM_PP_SIZE_128            0x00000001  /* 128 bytes of EEPROM */
#define EEPROM_PP_SIZE_256            0x00000003  /* 256 bytes of EEPROM */
#define EEPROM_PP_SIZE_512            0x00000007  /* 512 bytes of EEPROM */
#define EEPROM_PP_SIZE_1K             0x0000000f  /* 1 KB of EEPROM */
#define EEPROM_PP_SIZE_2K             0x0000001f  /* 2 KB of EEPROM */
#define EEPROM_PP_SIZE_3K             0x0000003f  /* 3 KB of EEPROM */
#define EEPROM_PP_SIZE_4K             0x0000007f  /* 4 KB of EEPROM */
#define EEPROM_PP_SIZE_5K             0x000000ff  /* 5 KB of EEPROM */
#define EEPROM_PP_SIZE_6K             0x000001ff  /* 6 KB of EEPROM */
#define EEPROM_PP_SIZE_S              0

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_TIVA_EEPROM_H */
