/****************************************************************************
 * arch/arm/src/stm32h7/stm32_crypto.c
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
#include <nuttx/debug.h>
#include <sys/param.h>
#include <errno.h>
#include <stdint.h>

#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <nuttx/crypto/crypto.h>

#include "arm_internal.h"
#include "hardware/stm32h7x3xx_rcc.h"
#include "hardware/stm32h7xxxx_crc.h"
#include "hardware/stm32h7xxxx_hash.h"

/* Following constants used in reverse32() to reverse
 * bit order of 32-bit value
 */
#define REV32_CONST1 0x55555555
#define REV32_CONST2 0x33333333
#define REV32_CONST3 0x0F0F0F0F
#define REV32_CONST4 0x00FF00FF

/* number polling iterations to wait for CRC to reset */

#define STM32H7_CRC_TIMEOUT 1000 /* should be enough... */

/* number polling iterations to wait for HASH to set CDIS/DINIS,
 * or to clear BUSY
 */

#define STM32H7_HASH_TIMEOUT 1000 /* should be enough... */

#define CRC32_XOR_VALUE 0xFFFFFFFFUL

/* #define CONFIG_STM32_CRYPTO_DEBUG */

#ifdef CONFIG_STM32_CRYPTO_DEBUG
#define stm32cryptoinfo(format, ...)                                      \
  do {                                                                  \
    __arch_syslog(LOG_INFO, "%s:%d " format, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
  } while (0)
#define stm32cryptoinfo_at(func, line, format, ...)                      \
  do {                                                                  \
    __arch_syslog(LOG_INFO, "%s:%d " format, func, line, ##__VA_ARGS__); \
  } while (0)
#define stm32crypto_getreg32(reg) stm32_crypto_getreg32(__FUNCTION__, __LINE__, reg)
#define stm32crypto_putreg32(val, reg) stm32_crypto_putreg32(__FUNCTION__, __LINE__, val, reg)
#define stm32crypto_putreg16(val, reg) stm32_crypto_putreg16(__FUNCTION__, __LINE__, val, reg)
#define stm32crypto_putreg8(val, reg) stm32_crypto_putreg8(__FUNCTION__, __LINE__, val, reg)
#else
#define stm32cryptoinfo(format, ...)
#define stm32cryptoinfo_at(func, line, format, ...)
#define stm32crypto_getreg32(reg) getreg32(reg)
#define stm32crypto_putreg32(val, reg) putreg32(val, reg)
#define stm32crypto_putreg16(val, reg) putreg16(val, reg)
#define stm32crypto_putreg8(val, reg) putreg8(val, reg)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_stm32_crc_initialized = false;
static bool g_stm32_hash_initialized = false;

static uint32_t g_stm32_sesnum = 0;

static FAR struct stm32_crypto_data **stm32_crypto_sessions = NULL;

struct stm32_crypto_data
{
  int hw_alg;  /* Algorithm; */
  union
    {
      struct stm32_crc32
        {
          uint32_t init;  /* init/interim CRC value */
        } crc;
      struct stm32_hash
        {
          bool hmac;      /* true if hmac */
          uint8_t *key;   /* hmac key */
          uint32_t klen;  /* hmac key length */
          uint32_t rdata; /* unaligned hash data */
          uint32_t dsize; /* # byte sin digest */
          uint32_t rlen;  /* # bytes accumulated in rdata */
        } hash;
    } u;
  struct stm32_crypto_data *hw_next;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

bool stm32_crypto_debug_flag = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_STM32_CRYPTO_DEBUG
struct rn
{
  FAR const char *name;
  uint32_t regaddr;
};

static struct rn stm32_crypto_regnames [] =
{
    {
      "RCC_AHB4RSTR", STM32_RCC_AHB4RSTR,
    },

    {
      "RCC_AHB4ENR", STM32_RCC_AHB4ENR,
    },

    {
      "CRC_DR", STM32_CRC_DR,
    },

    {
      "CRC_IDR", STM32_CRC_IDR,
    },

    {
      "CRC_CR", STM32_CRC_CR,
    },

    {
      "CRC_INIT", STM32_CRC_INIT,
    },

    {
      "CRC_POL", STM32_CRC_POL,
    },

    {
      "RCC_AHB2RSTR", STM32_RCC_AHB2RSTR,
    },

    {
      "RCC_AHB2ENR", STM32_RCC_AHB2ENR,
    },

    {
      "HASH_CR", STM32_HASH_CR,
    },

    {
      "HASH_DIN", STM32_HASH_DIN,
    },

    {
      "HASH_STR", STM32_HASH_STR,
    },

    {
      "HASH_HRA0", STM32_HASH_HRA0,
    },

    {
      "HASH_HRA1", STM32_HASH_HRA1,
    },

    {
      "HASH_HRA2", STM32_HASH_HRA2,
    },

    {
      "HASH_HRA3", STM32_HASH_HRA3,
    },

    {
      "HASH_HRA4", STM32_HASH_HRA4,
    },

    {
      "HASH_HR0", STM32_HASH_HR0,
    },

    {
      "HASH_HR1", STM32_HASH_HR1,
    },

    {
      "HASH_HR2", STM32_HASH_HR2,
    },

    {
      "HASH_HR3", STM32_HASH_HR3,
    },

    {
      "HASH_HR4", STM32_HASH_HR4,
    },

    {
      "HASH_HR5", STM32_HASH_HR5,
    },

    {
      "HASH_HR6", STM32_HASH_HR6,
    },

    {
      "HASH_HR7", STM32_HASH_HR7,
    },

    {
      "HASH_IMR", STM32_HASH_IMR,
    },

    {
      "HASH_SR", STM32_HASH_SR,
    },

    {
      "HASH_CSR0", STM32_HASH_CSR0,
    },
};

const char *stm32_crypto_regname(uint32_t regaddr)
{
  int i;

  for (i = 0; i < nitems(stm32_crypto_regnames); ++i)
    {
      if (stm32_crypto_regnames[i].regaddr == regaddr)
        {
          return stm32_crypto_regnames[i].name;
        }
    }

  return "???";
}

static uint32_t stm32_crypto_getreg32(const char *func, int line,
                                  uint32_t regaddr)
{
  const char *regname = stm32_crypto_regname(regaddr);
  char buf[128];
  uint32_t val;

  val = getreg32(regaddr);
  if (stm32_crypto_debug_flag)
    {
      snprintf(buf, sizeof(buf), "%08" PRIx32 "(%s)<=%08" PRIx32 "",
               regaddr, regname, val);
      stm32cryptoinfo_at(func, line, " %s", buf);
    }

  return val;
}

static void stm32_crypto_putreg8(const char *func, int line,
                              uint32_t val, uint32_t regaddr)
{
  const char *regname = stm32_crypto_regname(regaddr);
  char buf[128];

  if (stm32_crypto_debug_flag)
    {
      snprintf(buf, sizeof(buf), "%08" PRIx32 "(%s)=>%02" PRIx32 "",
               regaddr, regname, (val & 0xff));
      stm32cryptoinfo_at(func, line, " %s", buf);
    }

  putreg8(val & 0xff, regaddr);
}

static void stm32_crypto_putreg16(const char *func, int line,
                              uint32_t val, uint32_t regaddr)
{
  const char *regname = stm32_crypto_regname(regaddr);
  char buf[128];

  if (stm32_crypto_debug_flag)
    {
      snprintf(buf, sizeof(buf), "%08" PRIx32 "(%s)=>%04" PRIx32 "",
               regaddr, regname, (val & 0xffff));
      stm32cryptoinfo_at(func, line, " %s", buf);
    }

  putreg16(val & 0xffff, regaddr);
}

static void stm32_crypto_putreg32(const char *func, int line,
                              uint32_t val, uint32_t regaddr)
{
  const char *regname = stm32_crypto_regname(regaddr);
  char buf[128];

  if (stm32_crypto_debug_flag)
    {
      snprintf(buf, sizeof(buf), "%08" PRIx32 "(%s)=>%08" PRIx32 "",
               regaddr, regname, val);
      stm32cryptoinfo_at(func, line, " %s", buf);
    }

  putreg32(val, regaddr);
}
#endif

/****************************************************************************
 * Name: stm32_wait_for_clr
 *
 * Description:
 *   wait for all bits in bitmask to clear in HW register
 *
 ****************************************************************************/

static int stm32_wait_for_clr(uintptr_t regaddr, uint32_t bitmask,
                              uint32_t timeout)
{
  uint32_t regval;
  while (timeout--)
    {
      regval = stm32crypto_getreg32(regaddr);
      if ((regval & bitmask) == 0)
        {
          return 0;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: stm32_wait_for_set
 *
 * Description:
 *   wait for all bits in bitmask to set in HW register
 *
 ****************************************************************************/

static int stm32_wait_for_set(uintptr_t regaddr, uint32_t bitmask,
                              uint32_t timeout)
{
  uint32_t regval;
  while (timeout--)
    {
      regval = stm32crypto_getreg32(regaddr);
      if ((regval & bitmask) == bitmask)
        {
          return 0;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: reverse32
 *
 * Description:
 *   reverse bit order of 32-bit value
 *
 ****************************************************************************/

static uint32_t reverse32(uint32_t val)
{
  val = ((val >> 1) & REV32_CONST1) | ((val & REV32_CONST1) << 1);
  val = ((val >> 2) & REV32_CONST2) | ((val & REV32_CONST2) << 2);
  val = ((val >> 4) & REV32_CONST3) | ((val & REV32_CONST3) << 4);
  val = ((val >> 8) & REV32_CONST4) | ((val & REV32_CONST4) << 8);
  return (val >> 16) | (val << 16);
}

static int crc32_init(struct stm32_crypto_data *sw)
{
  irqstate_t flags;
  uint32_t regval;
  int ret;

  /* Set the initial value to start a CRC calculation */

  sw->u.crc.init = CRC32_XOR_VALUE;

  if (!g_stm32_crc_initialized)
    {
      g_stm32_crc_initialized = true;

      flags = enter_critical_section();

      /* Clear/set AHB4RSTR to reset CRC peripheral */

      regval  = stm32crypto_getreg32(STM32_RCC_AHB4RSTR);
      regval |= RCC_AHB4RSTR_CRCRST;
      stm32crypto_putreg32(regval, STM32_RCC_AHB4RSTR);
      regval &= ~RCC_AHB4RSTR_CRCRST;
      stm32crypto_putreg32(regval, STM32_RCC_AHB4RSTR);

      leave_critical_section(flags);
    }

  /* Do not set REV_OUT since that breaks being able to chain
   * buffers by simply reading CRC_DR at end of one update and
   * writing it back into CRC_INIT at start of next buffer
   */

  regval = stm32crypto_getreg32(STM32_CRC_CR);
  regval &= ~(CRC_CR_REV_OUT_MASK | CRC_CR_REV_IN_MASK
              | CRC_CR_POLYSIZE_MASK);
  regval |= CRC_CR_POLYSIZE_32BIT;

  regval |= CRC_CR_REV_IN_BYTE;
  regval |= CRC_CR_RESET;

  stm32crypto_putreg32(regval, STM32_CRC_CR);

  ret = stm32_wait_for_clr(STM32_CRC_CR, CRC_CR_RESET,
                           STM32H7_CRC_TIMEOUT);

  return ret;
}

static int crc32_update(struct stm32_crypto_data *sw,
                        uint8_t *buf, uint32_t len)
{
  const uint8_t *in_byte = buf;
  uint32_t regval;

  stm32crypto_putreg32(sw->u.crc.init, STM32_CRC_INIT);

  while (len >= sizeof(uint32_t))
    {
      regval = (in_byte[0] << 24) | (in_byte[1] << 16)
            |  (in_byte[2] << 8)  | (in_byte[3] << 0);
      stm32crypto_putreg32(regval, STM32_CRC_DR);
      in_byte += sizeof(uint32_t);
      len -= sizeof(uint32_t);
    }

  if (len)
    {
      if (len == 1)
        {
          stm32crypto_putreg8(in_byte[0], STM32_CRC_DR);
        }
      else if (len == 2)
        {
          regval = ((uint16_t)(in_byte[0]) << 8) | (uint16_t)in_byte[1];
          stm32crypto_putreg16(regval, STM32_CRC_DR);
        }
      else
        {
          regval = ((uint16_t)(in_byte[0]) << 8) | (uint16_t)in_byte[1];
          stm32crypto_putreg16(regval, STM32_CRC_DR);
          stm32crypto_putreg8(in_byte[2], STM32_CRC_DR);
        }
    }

  /* Extract current CRC and save for next iteration. */

  regval = stm32crypto_getreg32(STM32_CRC_DR);
  sw->u.crc.init = regval;

  return 0;
}

static int crc32_final(struct stm32_crypto_data *sw, caddr_t digest)
{
  uint32_t val;

  /* CRC is saved in init value - but its reversed due to HW
   * implementation.  Reverse and invert to get true
   * CRC-32 result.
   */

  val = sw->u.crc.init;
  val = reverse32(val) ^ CRC32_XOR_VALUE;
  stm32cryptoinfo("init: %08" PRIx32 " result: %08" PRIx32 "",
                  sw->u.crc.init, val);
  memcpy(digest, &val, sizeof(uint32_t));
  return 0;
}

static int hash_init(struct stm32_crypto_data *sw)
{
  irqstate_t flags;
  uint32_t regval;

  stm32cryptoinfo("");

  if (!g_stm32_hash_initialized)
    {
      g_stm32_hash_initialized = true;

      flags = enter_critical_section();

      /* Clear/set AHB2RSTR to reset CRYP peripheral */

      regval  = stm32crypto_getreg32(STM32_RCC_AHB2RSTR);
      regval |= RCC_AHB2RSTR_HASHRST;
      stm32crypto_putreg32(regval, STM32_RCC_AHB2RSTR);
      regval &= ~RCC_AHB2RSTR_HASHRST;
      stm32crypto_putreg32(regval, STM32_RCC_AHB2RSTR);

      leave_critical_section(flags);
    }

  /* Set the algorithm and digest size */

  regval = 0;
  switch (sw->hw_alg)
    {
      case CRYPTO_SHA1:
      case CRYPTO_SHA1_HMAC:
        regval = HASH_CR_ALGO_SHA1;
        sw->u.hash.dsize = 20;
        break;
      case CRYPTO_MD5:
      case CRYPTO_MD5_HMAC:
        regval = HASH_CR_ALGO_MD5;
        sw->u.hash.dsize = 16;
        break;
      case CRYPTO_SHA2_224:
      case CRYPTO_SHA2_224_HMAC:
        regval = HASH_CR_ALGO_SHA2_224;
        sw->u.hash.dsize = 28;
      break;
      case CRYPTO_SHA2_256:
      case CRYPTO_SHA2_256_HMAC:
        regval = HASH_CR_ALGO_SHA2_256;
        sw->u.hash.dsize = 32;
        break;

      default:
        return -EINVAL;
    }

  /* If HMAC set mode, and key is large set HASH_CR_LKEY bit */

  if (sw->u.hash.hmac)
  {
    regval |= HASH_CR_MODE_HMAC;
    stm32cryptoinfo("klen %" PRIx32 "", sw->u.hash.klen);
    if (sw->u.hash.klen > 64)
    {
      regval |= HASH_CR_LKEY_GT_64;
    }
  }

  regval |= HASH_CR_INIT;       /* set init to configure HASH algorithm */
  regval |= HASH_CR_DATATYPE_8; /* 8-bit data... */
  stm32crypto_putreg32(regval, STM32_HASH_CR);

  sw->u.hash.rlen = 0;
  sw->u.hash.rdata = 0;

  return 0;
}

static int hash_update(struct stm32_crypto_data *sw,
                       uint8_t *buf, uint32_t len)
{
  const uint8_t *in_byte = buf;
  const uint32_t *in_block;
  int ret;

  stm32cryptoinfo("buf %" PRIxPTR " len %" PRIu32 "",
                  (uintptr_t)buf, len);

  if (len == 0)
    {
      return 0;
    }

  /* If any remainder leftover from previous accumulation
   * then accumulate them into hash.rdata to
   * push into DIN
   */

  if (sw->u.hash.rlen)
    {
      while (sw->u.hash.rlen < sizeof(uint32_t))
        {
          if (!len)
            {
              break;
            }

          sw->u.hash.rdata |= (in_byte[0] << (8 * sw->u.hash.rlen));
          in_byte++;
          sw->u.hash.rlen++;
          len--;
        }

      /* If hash_rlen still not sizeof(uint32_t) then input buffer
       * doesn't hold enough bytes to write into DIN.
       */

      if (sw->u.hash.rlen < sizeof(uint32_t))
        {
          return 0;
        }

      stm32crypto_putreg32(sw->u.hash.rdata, STM32_HASH_DIN);
      sw->u.hash.rlen = 0;
      sw->u.hash.rdata = 0x0;
    }

  in_block = (uint32_t *)in_byte;

  /* Loop pushing uint32_t of hash data into HASH_DIN */

  while (len >= sizeof(uint32_t))
    {
      ret = stm32_wait_for_clr(STM32_HASH_SR, HASH_SR_BUSY,
                             STM32H7_HASH_TIMEOUT);
      if (ret < 0)
        {
          return ret;
        }

      stm32crypto_putreg32(*in_block, STM32_HASH_DIN);
      in_block++;
      len -= sizeof(uint32_t);
    }

  /* Accumulate any remaining bytes into hash.rdata */

  if (len)
    {
      stm32cryptoinfo("len %" PRIu32 " hash.rlen: %" PRIu32 "",
                      len, sw->u.hash.rlen);
      in_byte = (uint8_t *)in_block;
      while (len)
        {
          sw->u.hash.rdata |= (in_byte[0] << (8 * sw->u.hash.rlen));
          sw->u.hash.rlen++;
          in_byte++;
          len--;
        }
    }

  return 0;
}

static int hash_final(struct stm32_crypto_data *sw, caddr_t digest,
                      uint32_t wait_flag)
{
  uint32_t *out = (uint32_t *)digest;
  uint32_t regval;
  int ret;

  stm32cryptoinfo("digest %" PRIxPTR "", (uintptr_t)digest);

  /* If any data left in rdata, write it out */

  if (sw->u.hash.rlen)
    {
      stm32crypto_putreg32(sw->u.hash.rdata, STM32_HASH_DIN);
    }

  /* Start digest calculation. Set NBLW to number of
   * bits of valid data in last write to HASH_DIN, and
   * set DCAL to start the digest calculation.
   */

  regval = HASH_STR_NBLW_BYTES((uint32_t)sw->u.hash.rlen);
  stm32crypto_putreg32(regval, STM32_HASH_STR);

  /* Set DCAL _after_ setting NBLW; have seen incorrect HMAC hash
   * result if both done at same time.
   */

  regval |= HASH_STR_DCAL;
  stm32crypto_putreg32(regval, STM32_HASH_STR);

  sw->u.hash.rlen = 0;
  sw->u.hash.rdata = 0x0;

  /* Wait for the digest to compute
   * (for HMAC wait for data to be ready)
   */

  ret = stm32_wait_for_set(STM32_HASH_SR, wait_flag,
                           STM32H7_HASH_TIMEOUT);
  if (ret < 0)
    {
      return ret;
    }

  /* Extract the resultant hash (if desired) */

  if (digest)
    {
      regval = stm32crypto_getreg32(STM32_HASH_HRA0);
      *out++ = __builtin_bswap32(regval);
      regval = stm32crypto_getreg32(STM32_HASH_HRA1);
      *out++ = __builtin_bswap32(regval);
      regval = stm32crypto_getreg32(STM32_HASH_HRA2);
      *out++ = __builtin_bswap32(regval);
      regval = stm32crypto_getreg32(STM32_HASH_HRA3);
      *out++ = __builtin_bswap32(regval);

      if (sw->u.hash.dsize >= 20)
        {
          regval = stm32crypto_getreg32(STM32_HASH_HRA4);
          *out++ = __builtin_bswap32(regval);
        }

      if (sw->u.hash.dsize >= 28)
        {
          regval = stm32crypto_getreg32(STM32_HASH_HR5);
          *out++ = __builtin_bswap32(regval);
          regval = stm32crypto_getreg32(STM32_HASH_HR6);
          *out++ = __builtin_bswap32(regval);
        }

      if (sw->u.hash.dsize == 32)
        {
          regval = stm32crypto_getreg32(STM32_HASH_HR7);
          *out++ = __builtin_bswap32(regval);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: stm32_freesession
 *
 * Description:
 *   free session.
 *
 ****************************************************************************/

static int stm32_freesession(uint64_t tid)
{
  FAR struct stm32_crypto_data *sw;
  uint32_t sid = ((uint32_t) tid) & 0xffffffff;
  int ret = 0;

  if (sid > g_stm32_sesnum || stm32_crypto_sessions == NULL
      || stm32_crypto_sessions[sid] == NULL)
    {
      return -EINVAL;
    }

  /* Silently accept and return */

  if (sid == 0)
    {
      return 0;
    }

  while ((sw = stm32_crypto_sessions[sid]) != NULL)
    {
      stm32_crypto_sessions[sid] = sw->hw_next;

      switch (sw->hw_alg)
        {
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_MD5_HMAC:
          case CRYPTO_SHA2_224_HMAC:
          case CRYPTO_SHA2_256_HMAC:
            if (sw->u.hash.key)
            {
              explicit_bzero(sw->u.hash.key, sw->u.hash.klen);
              kmm_free(sw->u.hash.key);
            }
            break;

          case CRYPTO_CRC32:
          case CRYPTO_SHA1:
          case CRYPTO_MD5:
          case CRYPTO_SHA2_224:
          case CRYPTO_SHA2_256:
          case CRYPTO_AES_CBC:
          case CRYPTO_AES_CTR:
            break;

          default:
            ret = -EINVAL;
            break;
        }

      kmm_free(sw);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_newsession
 *
 * Description:
 *   create new session for crypto.
 *
 ****************************************************************************/

static int stm32_newsession(uint32_t *sid, struct cryptoini *cri)
{
  FAR struct stm32_crypto_data **swd;
  FAR struct stm32_crypto_data *sw;
  uint32_t i;
  int klen;
  int ret;

  stm32cryptoinfo("");

  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  if (stm32_crypto_sessions)
    {
      for (i = 1; i < g_stm32_sesnum; i++)
        {
          if (stm32_crypto_sessions[i] == NULL)
            {
              break;
            }
        }
    }

  if (stm32_crypto_sessions == NULL || i == g_stm32_sesnum)
    {
      if (stm32_crypto_sessions == NULL)
        {
          i = 1; /* We leave stm32_crypto_sessions[0] empty */
          g_stm32_sesnum = CRYPTO_SW_SESSIONS;
        }
      else
        {
          g_stm32_sesnum *= 2;
        }

      swd = kmm_calloc(g_stm32_sesnum, sizeof(struct stm32_crypto_data *));
      if (swd == NULL)
        {
          /* Reset session number */

          if (g_stm32_sesnum == CRYPTO_SW_SESSIONS)
            {
              g_stm32_sesnum = 0;
            }
          else
            {
              g_stm32_sesnum /= 2;
            }

          return -ENOBUFS;
        }

      /* Copy existing sessions */

      if (stm32_crypto_sessions)
        {
          bcopy(stm32_crypto_sessions, swd,
              (g_stm32_sesnum / 2) * sizeof(struct stm32_crypto_data *));
          kmm_free(stm32_crypto_sessions);
        }

      stm32_crypto_sessions = swd;
    }

  swd = &stm32_crypto_sessions[i];
  *sid = i;

  while (cri)
    {
      sw = kmm_zalloc(sizeof(struct stm32_crypto_data));
      *swd = sw;
      if (sw == NULL)
        {
          stm32_freesession(i);
          return -ENOBUFS;
        }

      sw->hw_alg = cri->cri_alg;
      switch (cri->cri_alg)
        {
          case CRYPTO_MD5_HMAC:
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_SHA2_224_HMAC:
          case CRYPTO_SHA2_256_HMAC:
            sw->u.hash.hmac = true;
            sw->u.hash.klen = cri->cri_klen / 8;
            sw->u.hash.key = kmm_malloc(sw->u.hash.klen);
            if (sw->u.hash.key == NULL)
            {
              return -ENOBUFS;
            }

            bcopy(cri->cri_key, sw->u.hash.key, sw->u.hash.klen);

            /* Initialize HW, push the key into DIN, wait for DINIS */

            ret = hash_init(sw);
            if (ret < 0)
              {
                return ret;
              }

            stm32cryptoinfo("Push %" PRIu32 " bytes of outer hash into DIN",
                           sw->u.hash.klen);
            ret = hash_update(sw, sw->u.hash.key, sw->u.hash.klen);
            if (ret < 0)
              {
                return ret;
              }

            stm32cryptoinfo("Wait for DINIS");
            ret = hash_final(sw, NULL, HASH_SR_DINIS);
            if (ret < 0)
              {
                return ret;
              }

            break;

          case CRYPTO_MD5:
          case CRYPTO_SHA1:
          case CRYPTO_SHA2_224:
          case CRYPTO_SHA2_256:
            sw->u.hash.hmac = false;
            ret = hash_init(sw);
            if (ret < 0)
            {
              return ret;
            }
            break;

          case CRYPTO_CRC32:
            crc32_init(sw);
            break;

          case CRYPTO_AES_CBC:
            break;

          case CRYPTO_AES_CTR:
            klen = cri->cri_klen / 8 - 4;
            if ((klen != 16) && (klen != 24) && (klen != 32))
            {
              /* stm32h7 aes-ctr key bits support 128, 192, or 256 */

              return -EINVAL;
            }

            break;

          default:
            stm32_freesession(i);
            return -EINVAL;
        }

      sw->hw_alg = cri->cri_alg;
      cri = cri->cri_next;
      swd = &(sw->hw_next);
    }

  return 0;
}

/****************************************************************************
 * Name: stm32_process
 *
 * Description:
 *   process session to use hardware algorithm.
 *
 ****************************************************************************/

static int stm32_process(struct cryptop *crp)
{
  FAR struct stm32_crypto_data *sw;
  struct cryptodesc *crd;
  uint8_t iv[AESCTR_BLOCKSIZE];
  uint32_t lid;
  int ret;

  stm32cryptoinfo("");

  /* Sanity check */

  if (crp == NULL)
    {
      stm32cryptoinfo("");
      return -EINVAL;
    }

  if (crp->crp_desc == NULL)
    {
      stm32cryptoinfo("");
      crp->crp_etype = -EINVAL;
      goto done;
    }

  lid = crp->crp_sid & 0xffffffff;
  if (lid >= g_stm32_sesnum || lid == 0
      || stm32_crypto_sessions[lid] == NULL)
    {
      stm32cryptoinfo("");
      crp->crp_etype = -ENOENT;
      goto done;
    }

  /* Go through crypto descriptors, processing as we go */

  for (crd = crp->crp_desc; crd; crd = crd->crd_next)
    {
      /* Find the crypto context.
       * XXX Note that the logic here prevents us from having
       * XXX the same algorithm multiple times in a session
       * XXX (or rather, we can but it won't give us the right
       * XXX results). To do that, we'd need some way of differentiating
       * XXX between the various instances of an algorithm (so we can
       * XXX locate the correct crypto context).
       */

      for (sw = stm32_crypto_sessions[lid];
           sw && sw->hw_alg != crd->crd_alg;
           sw = sw->hw_next);

      /* No such context ? */

      if (sw == NULL)
        {
          stm32cryptoinfo("");
          return -EINVAL;
        }

      switch (crd->crd_alg)
        {
          case CRYPTO_MD5:
          case CRYPTO_SHA1:
          case CRYPTO_SHA2_224:
          case CRYPTO_SHA2_256:
            if (crd->crd_flags & CRD_F_UPDATE)
              {
                return hash_update(sw, crp->crp_buf, crd->crd_len);
              }
            else
              {
                return hash_final(sw, crp->crp_mac, HASH_SR_DCIS);
              }
            break;
          case CRYPTO_MD5_HMAC:
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_SHA2_224_HMAC:
          case CRYPTO_SHA2_256_HMAC:
            ret = hash_update(sw, crp->crp_buf, crd->crd_len);
            if (ret < 0 || (crd->crd_flags & CRD_F_UPDATE))
              {
                return ret;
              }

            /* All HMAC data has been hashed, next step is
             * 1) set DCAL and HBLW, wait for DINIS
             */

            ret = hash_final(sw, NULL, HASH_SR_DINIS);
            if (ret < 0)
              {
                stm32cryptoinfo("");
                return ret;
              }

            /* 2) Push outer hash (same as inner hash) into DIN */

            stm32cryptoinfo("Push %" PRIu32 " bytes of inner hash",
                            sw->u.hash.klen);

            ret = hash_update(sw, sw->u.hash.key, sw->u.hash.klen);
            if (ret < 0)
              {
                stm32cryptoinfo("");
                return ret;
              }

            /* 3) normal finalization */

            return hash_final(sw, crp->crp_mac, HASH_SR_DCIS);

            break;

          case CRYPTO_CRC32:
            if (crd->crd_flags & CRD_F_UPDATE)
            {
              return crc32_update(sw, crp->crp_buf, crd->crd_len);
            }
            else
            {
              return crc32_final(sw, crp->crp_mac);
            }
            break;
          case CRYPTO_AES_CBC:
            return aes_cypher(crp->crp_dst, crp->crp_buf, crd->crd_len,
                              crp->crp_iv, crd->crd_key, 16,
                              AES_MODE_CBC, crd->crd_flags & CRD_F_ENCRYPT);
          case CRYPTO_AES_CTR:

            memcpy(iv, crd->crd_key + crd->crd_klen / 8 - AESCTR_NONCESIZE,
                   AESCTR_NONCESIZE);
            memcpy(iv + AESCTR_NONCESIZE, crp->crp_iv, AESCTR_IVSIZE);
            memcpy(iv + AESCTR_NONCESIZE + AESCTR_IVSIZE,
                   (uint8_t *)crp->crp_iv + AESCTR_IVSIZE, 4);

            return aes_cypher(crp->crp_dst, crp->crp_buf,
                              crd->crd_len, iv, crd->crd_key,
                              crd->crd_klen / 8 - AESCTR_NONCESIZE,
                              AES_MODE_CTR, crd->crd_flags & CRD_F_ENCRYPT);
          default:
            stm32cryptoinfo("crc_alg %d", crd->crd_alg);
            return -EINVAL;
        }
    }

  done:
  return OK;
}

/****************************************************************************
 * Name: hwcr_init
 *
 * Description:
 *   register the hardware crypto driver.
 *
 ****************************************************************************/

void hwcr_init(void)
{
  int hwcr_id;
  int algs[CRYPTO_ALGORITHM_MAX + 1];

  hwcr_id = crypto_get_driverid(0);
  DEBUGASSERT(hwcr_id >= 0);

  memset(algs, 0, sizeof(algs));

  algs[CRYPTO_CRC32] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_MD5] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA1] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_224] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_256] = CRYPTO_ALG_FLAG_SUPPORTED;

  algs[CRYPTO_MD5_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA1_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_224_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_256_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;

  algs[CRYPTO_AES_CBC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_CTR] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(hwcr_id, algs, stm32_newsession,
                  stm32_freesession, stm32_process);
}
