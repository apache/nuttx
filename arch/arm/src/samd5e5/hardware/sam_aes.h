/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_aes.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_AES_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_AES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AES register offsets *****************************************************/

#define SAM_AES_CTRLA_OFFSET       0x0000 /* Control A Register */
#define SAM_AES_CTRLB_OFFSET       0x0004 /* Control B Register */
#define SAM_AES_INTENCLR_OFFSET    0x0005 /* Interrupt Enable Clear Register */
#define SAM_AES_INTENSET_OFFSET    0x0006 /* Interrupt Enable Set Register */
#define SAM_AES_INTENFLAG_OFFSET   0x0007 /* Interrupt Flag Status and Clear Register */
#define SAM_AES_DATABUFPTR_OFFSET  0x0008 /* Data Buffer Pointer Register */
#define SAM_AES_DBGCTRL_OFFSET     0x0009 /* Debug Register */
#define SAM_AES_KEYWORD_OFFSET(n)  (0x000c + ((n) << 2))
#  define SAM_AES_KEYWORD0_OFFSET  0x000c /* Keyword 0 Register */
#  define SAM_AES_KEYWORD1_OFFSET  0x0010 /* Keyword 1 Register */
#  define SAM_AES_KEYWORD2_OFFSET  0x0014 /* Keyword 2 Register */
#  define SAM_AES_KEYWORD3_OFFSET  0x0018 /* Keyword 3 Register */
#  define SAM_AES_KEYWORD4_OFFSET  0x001c /* Keyword 4 Register */
#  define SAM_AES_KEYWORD5_OFFSET  0x0020 /* Keyword 5 Register */
#  define SAM_AES_KEYWORD6_OFFSET  0x0024 /* Keyword 6 Register */
#  define SAM_AES_KEYWORD7_OFFSET  0x0028 /* Keyword 7 Register */
#define SAM_AES_DATA_OFFSET        0x0038 /* Data Register */
#define SAM_AES_INTVECT_OFFSET(n)  (0x003c + ((n) << 2))
#  define SAM_AES_INTVECT0_OFFSET  0x003c /* Initialization Vector 0 Register */
#  define SAM_AES_INTVECT1_OFFSET  0x0040 /* Initialization Vector 1 Register */
#  define SAM_AES_INTVECT2_OFFSET  0x0044 /* Initialization Vector 2 Register */
#  define SAM_AES_INTVECT3_OFFSET  0x0048 /* Initialization Vector 3 Register */
#define SAM_AES_HASHKEY_OFFSET(n)  (0x005c + ((n) << 2))
#  define SAM_AES_HASHKEY0_OFFSET  0x005c /* Hash Key 0 Register */
#  define SAM_AES_HASHKEY1_OFFSET  0x0060 /* Hash Key 1 Register */
#  define SAM_AES_HASHKEY2_OFFSET  0x0064 /* Hash Key 2 Register */
#  define SAM_AES_HASHKEY3_OFFSET  0x0068 /* Hash Key 3 Register */
#define SAM_AES_GHASH_OFFSET(n)    (0x006c + ((n) << 2))
#  define SAM_AES_GHASH0_OFFSET    0x006c /* Galois Hash 0 Register */
#  define SAM_AES_GHASH1_OFFSET    0x0070 /* Galois Hash 1 Register */
#  define SAM_AES_GHASH2_OFFSET    0x0074 /* Galois Hash 2 Register */
#  define SAM_AES_GHASH3_OFFSET    0x0078 /* Galois Hash 3 Register */
#define SAM_AES_CIPLEN_OFFSET      0x0080 /* Cipher Length Register */
#define SAM_AES_RANDSEED_OFFSET    0x0084 /* Random Seed Register */

/* AES register addresses ***************************************************/

#define SAM_AES_CTRLA              (SAM_AES_BASE+SAM_AES_CTRLA_OFFSET)
#define SAM_AES_CTRLB              (SAM_AES_BASE+SAM_AES_CTRLB_OFFSET)
#define SAM_AES_INTENCLR           (SAM_AES_BASE+SAM_AES_INTENCLR_OFFSET)
#define SAM_AES_INTENSET           (SAM_AES_BASE+SAM_AES_INTENSET_OFFSET)
#define SAM_AES_INTENFLAG          (SAM_AES_BASE+SAM_AES_INTENFLAG_OFFSET)
#define SAM_AES_DATABUFPTR         (SAM_AES_BASE+SAM_AES_DATABUFPTR_OFFSET)
#define SAM_AES_DBGCTRL            (SAM_AES_BASE+SAM_AES_DBGCTRL_OFFSET)
#define SAM_AES_KEYWORD(n)         (SAM_AES_BASE+SAM_AES_KEYWORD_OFFSET(n))
#  define SAM_AES_KEYWORD0         (SAM_AES_BASE+SAM_AES_KEYWORD0_OFFSET)
#  define SAM_AES_KEYWORD1         (SAM_AES_BASE+SAM_AES_KEYWORD1_OFFSET)
#  define SAM_AES_KEYWORD2         (SAM_AES_BASE+SAM_AES_KEYWORD2_OFFSET)
#  define SAM_AES_KEYWORD3         (SAM_AES_BASE+SAM_AES_KEYWORD3_OFFSET)
#  define SAM_AES_KEYWORD4         (SAM_AES_BASE+SAM_AES_KEYWORD4_OFFSET)
#  define SAM_AES_KEYWORD5         (SAM_AES_BASE+SAM_AES_KEYWORD5_OFFSET)
#  define SAM_AES_KEYWORD6         (SAM_AES_BASE+SAM_AES_KEYWORD6_OFFSET)
#  define SAM_AES_KEYWORD7         (SAM_AES_BASE+SAM_AES_KEYWORD7_OFFSET)
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

/* AES register bit definitions *********************************************/

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
#  define AES_CTRLA_CTYPE1_DISAB   (0 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 1 disabled */
#  define AES_CTRLA_CTYPE1_ENAB    (1 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 1 enabled */
#  define AES_CTRLA_CTYPE2_DISAB   (0 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 2 disabled */
#  define AES_CTRLA_CTYPE2_ENAB    (2 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 2 enabled */
#  define AES_CTRLA_CTYPE3_DISAB   (0 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 3 disabled */
#  define AES_CTRLA_CTYPE3_ENAB    (4 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 3 enabled */
#  define AES_CTRLA_CTYPE4_DISAB   (0 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 4 disabled */
#  define AES_CTRLA_CTYPE4_ENAB    (8 << AES_CTRLA_CTYPE_SHIFT) /* Countermeasure 4 enabled */

/* Control B Register */

#define AES_CTRLB_START            (1 << 0)  /* Bit 0:  Start encryption/decryption */
#define AES_CTRLB_NEWMSG           (1 << 1)  /* Bit 1:  New message */
#define AES_CTRLB_EOM              (1 << 2)  /* Bit 2:  End of message */
#define AES_CTRLB_GFMUL            (1 << 3)  /* Bit 3:  GF multiplication */

/* Common Bit Definitions for the Interrupt Enable Clear Register, Interrupt
 * Enable Set Register, and Interrupt Flag Status and Clear Register
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

/* Cipher Length Register (32-bit value) */

/* Random Seed Register (32-bit value) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_AES_H */
