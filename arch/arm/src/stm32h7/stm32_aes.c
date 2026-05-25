/****************************************************************************
 * arch/arm/src/stm32h7/stm32_aes.c
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

#include <sys/types.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/crypto/crypto.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_rcc.h"
#include "stm32_aes.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AES_BLOCK_SIZE 16

/* Any reserved bits in CRYP_CR must be kept at reset value (0x0) */
#define STM32AES_CLEAR_RESERVED_CR_BITS(regval) \
  (regval & CRYP_CR_RESET_MASK)

/* #define CONFIG_STM32_CRYP_DEBUG */

#ifdef CONFIG_STM32_CRYP_DEBUG
#define cryp_getreg32(reg) stm32aes_getreg32(__FUNCTION__, __LINE__, reg)
#define cryp_putreg32(val, reg) stm32aes_putreg32(__FUNCTION__, __LINE__, val, reg)
#define crypinfo(func, line, format, ...)                             \
  __arch_syslog(LOG_INFO, "%s:%d" format, func, line, ##__VA_ARGS__)
#else
#define crypinfo(func, line, format, ...)
#define cryp_getreg32(reg) getreg32(reg)
#define cryp_putreg32(val, reg) putreg32(val, reg)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void stm32aes_enable(bool on);
static int stm32aes_setkey(const void *key, size_t keysize);
static void stm32aes_setiv(const void *iv);
static void stm32aes_encryptblock(void *block_out,
                                  const void *block_in);
static int  stm32aes_setup_cr(int mode, int encrypt,
                              const void *key, size_t keysize);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_stm32aes_lock = NXMUTEX_INITIALIZER;
static bool    g_stm32aes_initdone = false;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef STM32_CRYP_DEBUG

struct rn
{
  FAR const char *name;
  uint32_t regaddr;
};

static struct rn stm32aes_regnames [] =
{
    {
      "RCC_AHB2RSTR",
      STM32_RCC_AHB2RSTR,
    },

    {
      "RCC_AHB2ENR",
      STM32_RCC_AHB2ENR,
    },

    {
      "CRYP_CR",
      STM32_CRYP_CR,
    },

    {
      "CRYP_SR",
      STM32_CRYP_SR,
    },

    {
      "CRYP_DIN",
      STM32_CRYP_DIN,
    },

    {
      "CRYP_DOUT",
      STM32_CRYP_DOUT,
    },

    {
      "CRYP_DMACR",
      STM32_CRYP_DMACR,
    },

    {
      "CRYP_IMSCR",
      STM32_CRYP_IMSCR,
    },

    {
      "CRYP_RISR",
      STM32_CRYP_RISR,
    },

    {
      "CRYP_MISR",
      STM32_CRYP_MISR,
    },

    {
      "CRYP_K0LR",
      STM32_CRYP_K0LR,
    },

    {
      "CRYP_K0RR",
      STM32_CRYP_K0RR,
    },

    {
      "CRYP_K1LR",
      STM32_CRYP_K1LR,
    },

    {
      "CRYP_K1RR",
      STM32_CRYP_K1RR,
    },

    {
      "CRYP_K2LR",
      STM32_CRYP_K2LR,
    },

    {
      "CRYP_K2RR",
      STM32_CRYP_K2RR,
    },

    {
      "CRYP_K3LR",
      STM32_CRYP_K3LR,
    },

    {
      "CRYP_K3RR",
      STM32_CRYP_K3RR,
    },

    {
      "CRYP_IV0LR",
      STM32_CRYP_IV0LR,
    },

    {
      "CRYP_IV0RR",
      STM32_CRYP_IV0RR,
    },

    {
      "CRYP_IV1LR",
      STM32_CRYP_IV1LR,
    },

    {
      "CRYP_IV1RR",
      STM32_CRYP_IV1RR,
    },

    {
      "CRYP_CSGCMCCM0R",
      STM32_CRYP_CSGCMCCM0R,
    },
};

const char *stm32aes_regname(uint32_t regaddr)
{
  int i;

  for (i = 0; i < nitems(stm32aes_regnames); ++i)
    {
      if (stm32aes_regnames[i].regaddr == regaddr)
        {
          return stm32aes_regnames[i].name;
        }
    }

  return "???";
}

static uint32_t stm32aes_getreg32(const char *func, int line,
                                  uint32_t regaddr)
{
  const char *regname = stm32aes_regname(regaddr);
  char buf[128];
  uint32_t val;

  val = getreg32(regaddr);
  snprintf(buf, sizeof(buf), "%08" PRIx32 "(%s)<=%08" PRIx32 "",
           regaddr, regname, val);
  crypinfo(func, line, " %s", buf);
  return val;
}

static void stm32aes_putreg32(const char *func, int line,
                              uint32_t val, uint32_t regaddr)
{
  const char *regname = stm32aes_regname(regaddr);
  char buf[128];

  snprintf(buf, sizeof(buf), "%08" PRIx32 "(%s)=>%08" PRIx32 "",
           regaddr, regname, val);
  putreg32(val, regaddr);
  crypinfo(func, line, " %s", buf);
}
#endif

static void stm32aes_enable(bool on)
{
  uint32_t regval;

  regval = cryp_getreg32(STM32_CRYP_CR);
  if (on)
    {
      if (regval & CRYP_CR_CRYPEN)
        {
          return;
        }

      regval |= CRYP_CR_CRYPEN;
    }
  else
    {
      if (!(regval & CRYP_CR_CRYPEN))
        {
          return;
        }

      regval &= ~CRYP_CR_CRYPEN;
    }

  regval = STM32AES_CLEAR_RESERVED_CR_BITS(regval);
  cryp_putreg32(regval, STM32_CRYP_CR);
}

static void stm32aes_wait_not_busy(void)
{
  uint32_t regval;

  do
    {
      regval = cryp_getreg32(STM32_CRYP_SR);
    }
  while (regval & CRYP_SR_BUSY);
}

static int stm32aes_setkey(const void *key, size_t keysize)
{
  uint32_t *in = (uint32_t *)key;

  stm32aes_wait_not_busy();

  if (keysize == 32)
    {
      cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_K0LR); /* [255:224] */
      in++;
      cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_K0RR); /* [223:192] */
      in++;
    }

  if (keysize >= 24)
    {
      cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_K1LR); /* [191:160] */
      in++;
      cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_K1RR); /* [159:128] */
      in++;
    }

  cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_K2LR); /* [127:96] */
  in++;
  cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_K2RR); /* [95:64] */
  in++;
  cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_K3LR); /* [63:32] */
  in++;
  cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_K3RR); /* [31:0] */
  return 0;
}

static void stm32aes_setiv(const void *iv)
{
  uint32_t *in = (uint32_t *)iv;

  cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_IV0LR); /* [127:96] */
  in++;
  cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_IV0RR); /* [95:64] */
  in++;
  cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_IV1LR); /* [63:32] */
  in++;
  cryp_putreg32(__builtin_bswap32(*in), STM32_CRYP_IV1RR); /* [31:0] */
}

static void stm32aes_encryptblock(void *block_out,
                                  const void *block_in)
{
  uint32_t *in  = (uint32_t *)block_in;
  uint32_t *out = (uint32_t *)block_out;

  cryp_putreg32(*in, STM32_CRYP_DIN);
  in++;
  cryp_putreg32(*in, STM32_CRYP_DIN);
  in++;
  cryp_putreg32(*in, STM32_CRYP_DIN);
  in++;
  cryp_putreg32(*in, STM32_CRYP_DIN);

  stm32aes_wait_not_busy();

  *out = cryp_getreg32(STM32_CRYP_DOUT);
  out++;
  *out = cryp_getreg32(STM32_CRYP_DOUT);
  out++;
  *out = cryp_getreg32(STM32_CRYP_DOUT);
  out++;
  *out = cryp_getreg32(STM32_CRYP_DOUT);
}

static void stm32aes_set_datatype(void)
{
  uint32_t regval;

  regval = cryp_getreg32(STM32_CRYP_CR);
  regval |= CRYP_CR_DATATYPE_BE;
  regval = STM32AES_CLEAR_RESERVED_CR_BITS(regval);
  cryp_putreg32(regval, STM32_CRYP_CR);
}

static int stm32aes_setup_cr(int mode, int encrypt,
                             const void *key, size_t keysize)
{
  int ret;
  uint32_t regval = 0;

  /** 1) Configure algorithm/chaining mode through ALGOMODE/AOGODIR
   *    bits; also set KEYSIZE
   *    - when decrypting and mode is AES-ECB or AES-CBC, an initial
   *      key derivation must be performed (see section 35.4.7)
   */

  /* Set KEYSIZE field */

  switch (keysize)
    {
      case 16:
        regval |= CRYP_CR_KEYSIZE_128;
        break;
      case 24:
        regval |= CRYP_CR_KEYSIZE_192;
        break;
      case 32:
        regval |= CRYP_CR_KEYSIZE_256;
        break;
      default:
        return -EINVAL;
    }

  /* Set symmetric key */

  ret = stm32aes_setkey(key, keysize);
  if (ret)
    {
      return -EINVAL;
    }

  /* If decrypting and mode is AES-ECB or AES-CBC need to perform
   * key derivation first
   */

  if (!encrypt)
    {
      /* Set DECRYPT field */

      regval |= CRYP_CR_ALGODIR_DECRYPT;
      if ((mode == AES_MODE_ECB) || (mode == AES_MODE_CBC))
        {
          regval |= CRYP_CR_ALGOMODE_AES_KEY_PREP;
          cryp_putreg32(regval, STM32_CRYP_CR);

          /* Start key derivation and wait for not BUSY */

          stm32aes_enable(true);
          stm32aes_wait_not_busy();
          /* CRYPEN is automatically cleared. Clear
           * ALGOMODE field to allow following code to set
           * to the proper mode
           */

          regval &= ~CRYP_CR_ALGOMODE_MASK;
        }
    }
  else
    {
      /* Encrypting, make sure DECRYPT flag is cleared */

      regval &= ~CRYP_CR_ALGODIR_DECRYPT;
    }

  switch (mode)
  {
  case AES_MODE_ECB:
    regval |= CRYP_CR_ALGOMODE_AES_ECB;
    break;

  case AES_MODE_CBC:
    regval |= CRYP_CR_ALGOMODE_AES_CBC;
    break;

  case AES_MODE_CTR:
    regval |= CRYP_CR_ALGOMODE_AES_CTR;
    break;

  default:
    return -EINVAL;
  }

  regval = STM32AES_CLEAR_RESERVED_CR_BITS(regval);
  cryp_putreg32(regval, STM32_CRYP_CR);

  stm32aes_wait_not_busy();

  stm32aes_set_datatype();

  stm32aes_wait_not_busy();

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_aesreset(void)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Clear/set AHB2RSTR to reset CRYP peripheral */

  regval  = cryp_getreg32(STM32_RCC_AHB2RSTR);
  regval |= RCC_AHB2RSTR_CRYPTRST;
  cryp_putreg32(regval, STM32_RCC_AHB2RSTR);
  regval &= ~RCC_AHB2RSTR_CRYPTRST;
  cryp_putreg32(regval, STM32_RCC_AHB2RSTR);

  leave_critical_section(flags);

  return OK;
}

int stm32_aesinitialize(void)
{
  uint32_t regval;

  regval  = cryp_getreg32(STM32_RCC_AHB2ENR);
  regval |= RCC_AHB2ENR_CRYPTEN;
  cryp_putreg32(regval, STM32_RCC_AHB2ENR);

  stm32aes_enable(false);

  return OK;
}

static void stm32aes_flush_fifos(void)
{
  uint32_t regval;

  regval = cryp_getreg32(STM32_CRYP_CR);
  regval |= CRYP_CR_FFLUSH;
  regval = STM32AES_CLEAR_RESERVED_CR_BITS(regval);
  cryp_putreg32(regval, STM32_CRYP_CR);
}

/* See section 35.4.5 CRYP procedure to perform a cipher operation:
 * 0) Clear CRYPEN bit
 * 1) Configure algorithm/chaining mode through ALGOMODE/AOGODIR
 *    bits; also set KEYSIZE
 *    - when decrypting and mode is AES-ECB or AES-CBC, an initial
 *      key derivation must be performed (see section 35.4.7)
 * 2) When previous step is complete (and when applicable), write
 *    key into CRYP_KxL/R registers (see section 35.4.17)
 * 3) Configure DATATYPE
 * 4) When required(eg for CBC or CTR chaining modes), write the
 *    initialization vectors into CRYP_IVx(L/R)R.
 * 5) Flush the IN/OUT FIFOS by setting FFLUSH.
 *
 * Warning: If ECB/CBC mode is selected and data is not a multiple
 *          of 64 bits (for DES) or 128 bits (for AES), the 2nd and last
 *          block management is more comples; see Section 35.4.8
 *
 *  Appending data using the CPU in Polling mode
 * 1) Enable the cryptographic processor by setting to 1 the CRYPEN
 *    bit in the CRYP_CR register.
 * 2) Write data in the IN FIFO (one block or until the FIFO is full).
 * 3) Repeat the following sequence until the second last block of
 *    data has been processed:
 *    a) Wait until the not-empty-flag OFNE is set to 1, then read
 *       the OUT FIFO (one block or until the FIFO is empty).
 *    b) Wait until the not-full-flag IFNF is set to 1, then write
 *       the IN FIFO (one block or until the FIFO is full) except
 *       if it is the last block.
 * 4) The BUSY bit is set automatically by the cryptographic processor.
 *    At the end of the processing, the BUSY bit returns to 0 and both
 *    FIFOs are empty (IN FIFO empty flag IFEM = 1 and OUT FIFO not
 *    empty flag OFNE = 0).
 * 5) If the next processing block is the last block, the CPU must pad
 *    (when applicable) the data with zeroes to obtain a complete block
 * 6) When the operation is complete, the cryptographic processor can
 *    be disabled by clearing the CRYPEN bit in CRYP_CR register.
 *
 * 35.4.7 Preparing the CRYP AES key for decryption:
 *
 * When performing an AES ECB or CBC decryption, the AES key
 * has to be prepared. Indeed, in AES encryption the round 0 key
 * is the one stored in the key registers, and AES decryption must
 * start using the last round key. Hence, as the encryption key
 * is stored in memory, a special key scheduling must be performed
 * to obtain the decryption key. This preparation is not required
 * in any other AES modes than ECB or CBC decryption.
 *
 * When the cryptographic processor is disabled (CRYPEN = 0),
 * the CRYP key preparation process is performed as follows:
 * 1) Program ALGOMODE bits to 0x7 and ALGODIR to 0x0 in CRYP_CR.
 *    In addition, configure the key length with the KEYSIZE bits.
 * 2) Write the symmetric key to the CRYP_KxL/R registers, as
 *    described in Section 35.4.17: CRYP key registers.
 * 3) Enable the cryptographic processor by setting the CRYPEN bit
 *    in the CRYP_CR register. It immediately starts an AES round
 *    for key preparation (BUSY = 1).
 * 4) Wait until the BUSY bit is cleared in the CRYP_SR register.
 *    Then update ALGOMODE bits in the CRYP_CR register to select
 *    the correct chaining mode, that is 0x4 for ECB or 0x5 for CBC.
 * 5) The AES key is available in the CRYP key registers, ready
 *    to use for decryption.
 * Note: As the CRYPEN bitfield is reset by hardware at the end of
 *       the key preparation, the application software must set it
 *       again for the next operation.
 *       The latency of the key preparation operation is 14, 16 or 18
 *       clock cycles depending on the key size (128, 192 or 256 bits).
 *
 * 35.4.8 CRYP stealing and data padding
 *
 * When using DES or AES algorithm in ECB or CBC modes to manage
 * messages that are not multiple of the block size (64 bits for DES,
 * 128 bits for AES), use ciphertext stealing techniques such as those
 * described in NIST Special Publication 800-38A, Recommendation
 * for Block Cipher Modes of Operation: Three Variants of Ciphertext
 * Stealing for CBC Mode. Since the cryptographic processor does not
 * implement such techniques, the last two blocks must be handled in
 * a special way by the application.
 *
 * Note: Ciphertext stealing techniques are not documented in this
 *       reference manual.
 *
 * Similarly, when the AES algorithm is used in other modes than ECB
 * or CBC, incomplete input data blocks (i.e. block shorter than 128
 * bits) have to be padded with zeroes by the application prior to
 * encryption (i.e. extra bits should be appended to the trailing end
 * of the data string). After decryption, the extra bits have to be
 * discarded. The cryptographic processor does not implement automatic
 * data padding operation to the last block, so the application should
 * follow the recommendation given in Section 35.4.5: CRYP procedure to
 * perform a cipher operation to manage messages that are not multiple
 * of 128 bits.
 *
 * Note: Padding data are swapped in a similar way as normal data,
 *       according to the DATATYPE field in CRYP_CR register (see
 *       Section 35.4.16: CRYP data registers and data swapping for
 *       details).
 */

int aes_cypher(void *out, const void *in, size_t size,
               const void *iv, const void *key, size_t keysize,
               int mode, int encrypt)
{
  int ret = OK;

  /* CRYP peripheral clock is already enabled;
   * make sure it gets reset (once)
   */

  if (!g_stm32aes_initdone)
    {
      ret = stm32_aesreset();
      if (ret < 0)
        {
          return ret; /* AES init failed */
        }

      g_stm32aes_initdone = true;
    }

  if ((size & (AES_BLOCK_SIZE - 1)) != 0)
    {
      return -EINVAL;
    }

  if (keysize != 16 && keysize != 24 && keysize != 32)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_stm32aes_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* AES must be disabled before changing mode, key or IV. */

  stm32aes_enable(false);
  ret = stm32aes_setup_cr(mode, encrypt, key, keysize);
  if (ret < 0)
    {
      goto out;
    }

  if (iv != NULL)
    {
      stm32aes_setiv(iv);
    }

  stm32aes_wait_not_busy();
  stm32aes_flush_fifos();
  stm32aes_wait_not_busy();

  stm32aes_enable(true);

  while (size)
    {
      stm32aes_encryptblock(out, in);
      out   = (uint8_t *)out + AES_BLOCK_SIZE;
      in    = (uint8_t *)in  + AES_BLOCK_SIZE;
      size -= AES_BLOCK_SIZE;
    }

  stm32aes_enable(false);

out:
  nxmutex_unlock(&g_stm32aes_lock);

  return ret;
}
