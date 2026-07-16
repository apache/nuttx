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

/* Following constants used in reverse32() to reverse
 * bit order of 32-bit value
 */
#define REV32_CONST1 0x55555555
#define REV32_CONST2 0x33333333
#define REV32_CONST3 0x0F0F0F0F
#define REV32_CONST4 0x00FF00FF

#define STM32H7_CRC_RESET_TIMEOUT 1000 /* should be enough loops to reset */
#define CRC32_XOR_VALUE 0xFFFFFFFFUL

#undef CONFIG_STM32_CRYPTO_DEBUG

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

static uint32_t g_stm32_sesnum = 0;

static FAR struct stm32_crypto_data **stm32_crypto_sessions = NULL;

struct stm32_crypto_data
{
  int hw_alg;  /* Algorithm; */
  union
    {
      struct stm32_crc32
        {
          uint32_t init; /* init/interim CRC value */
        } crc;
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
      "RCC_AHB4RSTR",
      STM32_RCC_AHB4RSTR,
    },

    {
      "RCC_AHB4ENR",
      STM32_RCC_AHB4ENR,
    },

    {
      "CRC_DR",
      STM32_CRC_DR,
    },

    {
      "CRC_IDR",
      STM32_CRC_IDR,
    },

    {
      "CRC_CR",
      STM32_CRC_CR,
    },

    {
      "CRC_INIT",
      STM32_CRC_INIT,
    },

    {
      "CRC_POL",
      STM32_CRC_POL,
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
 *   wait for a bitmask to clear in HW register
 *
 ****************************************************************************/

static int stm32_wait_for_clr(uintptr_t regaddr, uint32_t bitmask,
                                      uint32_t timeout)
{
  uint32_t regval;
  while (timeout--)
    {
      regval = stm32crypto_getreg32(regaddr);
      if (regval & bitmask)
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
                           STM32H7_CRC_RESET_TIMEOUT);

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

/****************************************************************************
 * Name: stm32_freesession
 *
 * Description:
 *   free session.
 *
 ****************************************************************************/

static int stm32_freesession(uint64_t tid)
{
  FAR struct stm32_crypto_data *swd;
  uint32_t sid = ((uint32_t) tid) & 0xffffffff;

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

  while ((swd = stm32_crypto_sessions[sid]) != NULL)
    {
      stm32_crypto_sessions[sid] = swd->hw_next;

      switch (swd->hw_alg)
        {
          case CRYPTO_CRC32:
          case CRYPTO_AES_CBC:
          case CRYPTO_AES_CTR:
            break;

          default:
            return -EINVAL;
        }

      kmm_free(swd);
    }

  return 0;
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
  int klen;
  uint32_t i;

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
      *swd = kmm_zalloc(sizeof(struct stm32_crypto_data));
      if (*swd == NULL)
        {
          stm32_freesession(i);
          return -ENOBUFS;
        }

      switch (cri->cri_alg)
        {
          case CRYPTO_CRC32:
            crc32_init(*swd);
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

      (*swd)->hw_alg = cri->cri_alg;
      cri = cri->cri_next;
      swd = &((*swd)->hw_next);
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

  stm32cryptoinfo("");

  /* Sanity check */

  if (crp == NULL)
    {
      return -EINVAL;
    }

  if (crp->crp_desc == NULL || crp->crp_buf == NULL)
    {
      crp->crp_etype = -EINVAL;
      goto done;
    }

  lid = crp->crp_sid & 0xffffffff;
  if (lid >= g_stm32_sesnum || lid == 0
      || stm32_crypto_sessions[lid] == NULL)
    {
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
          return -EINVAL;
        }

      switch (crd->crd_alg)
        {
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

  algs[CRYPTO_AES_CBC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_CTR] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(hwcr_id, algs, stm32_newsession,
                  stm32_freesession, stm32_process);
}
