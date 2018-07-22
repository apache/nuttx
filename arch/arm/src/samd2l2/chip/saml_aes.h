/********************************************************************************************
 * arch/arm/src/samd2l2/chip/saml_aes.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_AES_H
#define __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_AES_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* AES register offsets ********************************************************************/

#define SAM_AES_CTRLA_OFFSET       0x0000 /* Control A Register */
#define SAM_AES_CTRLB_OFFSET       0x0004 /*  Control B Register */
#define SAM_AES_INTENCLR_OFFSET    0x0005 /* Interrupt Enable Clear Register */
#define SAM_AES_INTENSET_OFFSET    0x0006 /* Interrupt Enable Set Register */
#define SAM_AES_INTENFLAG_OFFSET   0x0007 /* Interrupt Flag Status and Clear Register */
#define SAM_AES_DATABUFPTR_OFFSET  0x0008 /* Data Buffer Pointer Register */
#define SAM_AES_DBGCTRL_OFFSET     0x0009 /* Debug Register */
#define SAM_AES_KEYWORD_OFFSET(n)  (0x0010 + ((n) << 2))
#  define SAM_AES_KEWORD0_OFFSET   0x0010 /* Keyword 0 Register */
#  define SAM_AES_KEWORD1_OFFSET   0x0014 /* Keyword 1 Register */
#  define SAM_AES_KEWORD2_OFFSET   0x0018 /* Keyword 2 Register */
#  define SAM_AES_KEWORD3_OFFSET   0x001c /* Keyword 3 Register */
#  define SAM_AES_KEWORD4_OFFSET   0x0020 /* Keyword 4 Register */
#  define SAM_AES_KEWORD5_OFFSET   0x0024 /* Keyword 5 Register */
#  define SAM_AES_KEWORD6_OFFSET   0x0028 /* Keyword 6 Register */
#define SAM_AES_DATA_OFFSET        0x0038 /* Data Register */
#define SAM_AES_INTVECT_OFFSET(n)  (0x003c + ((n) << 2))
#  define SAM_AES_INTVECT0_OFFSET  0x003c /* Initialization Vector 0 Register */
#  define SAM_AES_INTVECT1_OFFSET  0x0040 /* Initialization Vector 1 Register */
#  define SAM_AES_INTVECT2_OFFSET  0x0044 /* Initialization Vector 2 Register */
#  define SAM_AES_INTVECT3_OFFSET  0x0048 /* Initialization Vector 3 Register */
#define SAM_AES_HASHKEY_OFFSET(n)  (0x004c + ((n) << 2))
#  define SAM_AES_HASHKEY0_OFFSET  0x004c /* Hash Key 0 Register */
#  define SAM_AES_HASHKEY1_OFFSET  0x0050 /* Hash Key 1 Register */
#  define SAM_AES_HASHKEY2_OFFSET  0x0054 /* Hash Key 2 Register */
#  define SAM_AES_HASHKEY3_OFFSET  0x0058 /* Hash Key 3 Register */
#define SAM_AES_GHASH_OFFSET(n)    (0x005c + ((n) << 2))
#  define SAM_AES_GHASH0_OFFSET    0x005c /* Galois Hash 0 Register */
#  define SAM_AES_GHASH1_OFFSET    0x0060 /* Galois Hash 1 Register */
#  define SAM_AES_GHASH2_OFFSET    0x0064 /* Galois Hash 2 Register */
#  define SAM_AES_GHASH3_OFFSET    0x0068 /* Galois Hash 3 Register */
#define SAM_AES_CIPLEN_OFFSET      0x0070 /* Cipher Length Register */
#define SAM_AES_RANDSEED_OFFSET    0x0074 /* Random Seed Register */

/* AES register addresses ******************************************************************/

#define SAM_AES_CTRLA              (SAM_AES_BASE+SAM_AES_CTRLA_OFFSET)
#define SAM_AES_CTRLB              (SAM_AES_BASE+SAM_AES_CTRLB_OFFSET)
#define SAM_AES_INTENCLR           (SAM_AES_BASE+SAM_AES_INTENCLR_OFFSET)
#define SAM_AES_INTENSET           (SAM_AES_BASE+SAM_AES_INTENSET_OFFSET)
#define SAM_AES_INTENFLAG          (SAM_AES_BASE+SAM_AES_INTENFLAG_OFFSET)
#define SAM_AES_DATABUFPTR         (SAM_AES_BASE+SAM_AES_DATABUFPTR_OFFSET)
#define SAM_AES_DBGCTRL            (SAM_AES_BASE+SAM_AES_DBGCTRL_OFFSET)
#define SAM_AES_KEYWORD(n)         (SAM_AES_BASE+SAM_AES_KEYWORD_OFFSET(n))
#  define SAM_AES_KEWORD0          (SAM_AES_BASE+SAM_AES_KEWORD0_OFFSET)
#  define SAM_AES_KEWORD1          (SAM_AES_BASE+SAM_AES_KEWORD1_OFFSET)
#  define SAM_AES_KEWORD2          (SAM_AES_BASE+SAM_AES_KEWORD2_OFFSET)
#  define SAM_AES_KEWORD3          (SAM_AES_BASE+SAM_AES_KEWORD3_OFFSET)
#  define SAM_AES_KEWORD4          (SAM_AES_BASE+SAM_AES_KEWORD4_OFFSET)
#  define SAM_AES_KEWORD5          (SAM_AES_BASE+SAM_AES_KEWORD5_OFFSET)
#  define SAM_AES_KEWORD6          (SAM_AES_BASE+SAM_AES_KEWORD6_OFFSET)
#define SAM_AES_DATA               (SAM_AES_BASE+SAM_AES_DATA_OFFSET)
#define SAM_AES_INTVECT(n)         (SAM_AES_BASE+SAM_AES_INTVECT_OFFSET(n))
#  define SAM_AES_INTVECT0         (SAM_AES_BASE+SAM_AES_INTVECT0_OFFSET)
#  define SAM_AES_INTVECT1         (SAM_AES_BASE+SAM_AES_INTVECT1_OFFSET)
#  define SAM_AES_INTVECT2         (SAM_AES_BASE+SAM_AES_INTVECT2_OFFSET)
#  define SAM_AES_INTVECT3         (SAM_AES_BASE+SAM_AES_INTVECT3_OFFSET)
#define SAM_AES_HASHKEY(n)         (SAM_AES_BASE+SAM_AES_HASHKEY_OFFSET(n))
#  define SAM_AES_HASHKEY0         (SAM_AES_BASE+SAM_AES_HASHKEY0_OFFSET)
#  define SAM_AES_HASHKEY1         (SAM_AES_BASE+SAM_AES_HASHKEY1_OFFSET)
#  define SAM_AES_HASHKEY2         (SAM_AES_BASE+SAM_AES_HASHKEY2_OFFSET)
#  define SAM_AES_HASHKEY3         (SAM_AES_BASE+SAM_AES_HASHKEY3_OFFSET)
#define SAM_AES_GHASH(n)           (SAM_AES_BASE+SAM_AES_GHASH_OFFSET(n))
#  define SAM_AES_GHASH0           (SAM_AES_BASE+SAM_AES_GHASH0_OFFSET)
#  define SAM_AES_GHASH1           (SAM_AES_BASE+SAM_AES_GHASH1_OFFSET)
#  define SAM_AES_GHASH2           (SAM_AES_BASE+SAM_AES_GHASH2_OFFSET)
#  define SAM_AES_GHASH3           (SAM_AES_BASE+SAM_AES_GHASH3_OFFSET)
#define SAM_AES_CIPLEN             (SAM_AES_BASE+SAM_AES_CIPLEN_OFFSET)
#define SAM_AES_RANDSEED           (SAM_AES_BASE+SAM_AES_RANDSEED_OFFSET)

/* AES register bit definitions ************************************************************/

/* Control A Register */

#define AES_CTRLA_SWRST            (1 << 0)  /* Bit 0:  Software reset */
#define AES_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define AES_CTRLA_AESMODE_SHIFT    (2)       /* Bits 2-4: AES mode of operation */
#define AES_CTRLA_AESMODE_MASK     (7 << AES_CTRLA_AESMODE_SHIFT)
#  define AES_CTRLA_AESMODE_ECB    (0 << AES_CTRLA_AESMODE_SHIFT) /* Electronic code book mode */
#  define AES_CTRLA_AESMODE_CBC    (1 << AES_CTRLA_AESMODE_SHIFT) /* Cipher block chaining mode */
#  define AES_CTRLA_AESMODE_OFB    (2 << AES_CTRLA_AESMODE_SHIFT) /* Output feedback mode */
#  define AES_CTRLA_AESMODE_CFB    (3 << AES_CTRLA_AESMODE_SHIFT) /* Cipher feedback mode */
#  define AES_CTRLA_AESMODE_CNTR   (4 << AES_CTRLA_AESMODE_SHIFT) /* Counter mode */
#  define AES_CTRLA_AESMODE_CCM    (5 << AES_CTRLA_AESMODE_SHIFT) /* CCM mode */
#  define AES_CTRLA_AESMODE_GCM    (6 << AES_CTRLA_AESMODE_SHIFT) /* Galois counter mode */
#define AES_CTRLA_CFBS_SHIFT       (5)       /* Bits 5-7: Cipher feedback block size */
#define AES_CTRLA_CFBS_MASK        (7 << AES_CTRLA_CFBS_SHIFT)
#  define AES_CTRLA_CFBS_128       (0 << AES_CTRLA_CFBS_SHIFT) /* 128-bit data block */
#  define AES_CTRLA_CFBS_64        (1 << AES_CTRLA_CFBS_SHIFT) /* 64-bit data block */
#  define AES_CTRLA_CFBS_32        (2 << AES_CTRLA_CFBS_SHIFT) /* 32-bit data block */
#  define AES_CTRLA_CFBS_16        (3 << AES_CTRLA_CFBS_SHIFT) /* 16-bit data block */
#  define AES_CTRLA_CFBS_8         (4 << AES_CTRLA_CFBS_SHIFT) /* 8-bit data block */
#define AES_CTRLA_KEYSIZE_SHIFT    (8)       /* Bits 8-9: Encryption key size */
#define AES_CTRLA_KEYSIZE_MASK     (3 << AES_CTRLA_KEYSIZE_SHIFT)
#  define AES_CTRLA_KEYSIZE_128    (0 << AES_CTRLA_KEYSIZE_SHIFT) /* 128-bit key */
#  define AES_CTRLA_KEYSIZE_192    (1 << AES_CTRLA_KEYSIZE_SHIFT) /* 192-bit key */
#  define AES_CTRLA_KEYSIZE_256    (2 << AES_CTRLA_KEYSIZE_SHIFT) /* 256-bit key */
#define AES_CTRLA_CIPHER           (1 << 10) /* Bit 10:  Cipher */
#define AES_CTRLA_STARTMODE        (1 << 11) /* Bit 11:  Start mode select */
#define AES_CTRLA_LOD              (1 << 12) /* Bit 12:  Last output data mode */
#define AES_CTRLA_KEYGEN           (1 << 13) /* Bit 13:  Key generation */
#define AES_CTRLA_XORKEY           (1 << 14) /* Bit 14:  XOR key */
#define AES_CTRLA_CTYPE_SHIFT      (16)      /* Bits 16-19: Countermeasure type */
#define AES_CTRLA_CTYPE_MASK       (15 << AES_CTRLA_CTYPE_SHIFT)
#  define AES_CTRLA_CTYPE_CTYPE1   (1 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 1 enabled */
#  define AES_CTRLA_CTYPE_CTYPE2   (2 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 2 enabled */
#  define AES_CTRLA_CTYPE_CTYPE3   (4 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 3 enabled */
#  define AES_CTRLA_CTYPE_CTYPE4   (8 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 4 enabled */

/*  Control B Register */

#define AES_CTRLB_START            (1 << 0)  /* Bit 0:  Start encryption/decryption */
#define AES_CTRLB_NEWMSG           (1 << 1)  /* Bit 1:  New message */
#define AES_CTRLB_EOM              (1 << 2)  /* Bit 2:  End of message */
#define AES_CTRLB_GFMUL            (1 << 3)  /* Bit 3:  GF multiplication */

/* Common Bit Definitions for the Interrupt Enable Clear Register, Interrupt Enable Set
 * Register, and Interrupt Flag Status and Clear Register
 */

#define AES_INT_ENCCMP             (1 << 0)  /* Bit 0:  Encryption complete interrupt */
#define AES_INT_GFMCMP             (1 << 1)  /* Bit 1:  GF multiplication complete interrupt */
#define AES_INT_ALL                0x03

/* Data Buffer Pointer Register */

#define AES_DATABUFPTR_MASK        0x03      /* Bits 0-1: Data pointer */

/* Debug Register */

#define AES_DBGCTRL_DBGRUN         (1 << 0)  /* Bit 0:  Debug run */

/* Keyword n Register, n = 0-7 (32-value) */
/* Data Register (32-bit value) */
/* Initialization Vector n Register, n=0-3 (32-bit value) */
/* Hash Key n Register, n=0-3 (32-bit value) */
/* Galois Hash n Register, n=0-3 (32-bit value) */
/* Cipher Length Register (32-bit vaoue) */
/* Random Seed Register (32-bit value) */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_AES_H */
