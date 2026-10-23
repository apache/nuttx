/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_radio.c
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

#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <string.h>
#include <stdio.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nrf54l_gpio.h"
#include "nrf54l_radio.h"

#include "hardware/nrf54l_radio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MLTPAN-6 */

#if defined(CONFIG_ARCH_CHIP_NRF54L15) || defined(CONFIG_ARCH_CHIP_NRF54LM20A)
#  define HAVE_RADIO_MLTPAN6
#endif

#define NRF54L_RADIO_MLTPAN6       (NRF54L_RADIO_BASE + 0x810)
#define NRF54L_ERRATA_REVISION     (0x00ffc344)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Radio registers access ***************************************************/

static void nrf54l_radio_putreg(struct nrf54l_radio_dev_s *dev,
                               uint32_t offset,
                               uint32_t value);
static uint32_t nrf54l_radio_getreg(struct nrf54l_radio_dev_s *dev,
                                   uint32_t offset);

/* Radio operations *********************************************************/

static void nrf54l_radio_reset(struct nrf54l_radio_dev_s *dev);
static void nrf54l_radio_inten(struct nrf54l_radio_dev_s *dev, uint32_t irq);
static void nrf54l_radio_intclr(struct nrf54l_radio_dev_s *dev,
                                uint32_t irq);
static void nrf54l_radio_shorts(struct nrf54l_radio_dev_s *dev, uint32_t sh);
static int nrf54l_radio_power(struct nrf54l_radio_dev_s *dev, bool state);
static int nrf54l_radio_mode_set(struct nrf54l_radio_dev_s *dev,
                                uint8_t mode);
static int nrf54l_radio_freq_set(struct nrf54l_radio_dev_s *dev,
                                uint32_t freq);
static int nrf54l_radio_rssi_get(struct nrf54l_radio_dev_s *dev,
                                int8_t *rssi);
static int nrf54l_radio_txpower_set(struct nrf54l_radio_dev_s *dev,
                                   uint8_t txpower);
static int nrf54l_radio_tifs_set(struct nrf54l_radio_dev_s *dev,
                                 uint16_t us);
static int nrf54l_radio_pkt_cfg(struct nrf54l_radio_dev_s *dev,
                               struct nrf54l_radio_pktcfg_s *cfg);
static int nrf54l_radio_crc_cfg(struct nrf54l_radio_dev_s *dev,
                               struct nrf54l_radio_crc_s *cfg);
static int nrf54l_radio_white_set(struct nrf54l_radio_dev_s *dev,
                                 uint8_t init);
static int nrf54l_radio_addr_set(struct nrf54l_radio_dev_s *dev, uint8_t i,
                                struct nrf54l_radio_addr_s *addr);
static void nrf54l_radio_dumpregs(struct nrf54l_radio_dev_s *dev);

#ifdef CONFIG_NRF54L_HAVE_IEEE802154
static void nrf54l_radio_sfd_set(struct nrf54l_radio_dev_s *dev,
                                 uint8_t sfd);
static void nrf54l_radio_edcnt_set(struct nrf54l_radio_dev_s *dev,
                                  uint32_t edcnt);
static void nrf54l_radio_cca_cfg(struct nrf54l_radio_dev_s *dev,
                                struct nrf54l_radio_cca_s *cca);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NRF54L radio operations */

struct nrf54l_radio_dev_s;
static struct nrf54l_radio_ops_s g_nrf54l_radio_ops =
{
  .reset        = nrf54l_radio_reset,
  .putreg       = nrf54l_radio_putreg,
  .getreg       = nrf54l_radio_getreg,
  .inten        = nrf54l_radio_inten,
  .intclr       = nrf54l_radio_intclr,
  .shorts       = nrf54l_radio_shorts,
  .power        = nrf54l_radio_power,
  .mode_set     = nrf54l_radio_mode_set,
  .freq_set     = nrf54l_radio_freq_set,
  .rssi_get     = nrf54l_radio_rssi_get,
  .txpower_set  = nrf54l_radio_txpower_set,
  .tifs_set     = nrf54l_radio_tifs_set,
  .pkt_cfg      = nrf54l_radio_pkt_cfg,
  .white_set    = nrf54l_radio_white_set,
  .crc_cfg      = nrf54l_radio_crc_cfg,
  .addr_set     = nrf54l_radio_addr_set,
  .dumpregs     = nrf54l_radio_dumpregs,
#ifdef CONFIG_NRF54L_HAVE_IEEE802154
  .sfd_set      = nrf54l_radio_sfd_set,
  .edcnt_set    = nrf54l_radio_edcnt_set,
  .cca_cfg      = nrf54l_radio_cca_cfg,
#endif
};

/* Radio device 1 */

static struct nrf54l_radio_dev_s g_nrf54l_radio_dev_1 =
{
  .ops       = &g_nrf54l_radio_ops,
  .irq       = NRF54L_IRQ_RADIO_0,
  .base      = NRF54L_RADIO_BASE,
  .mode      = 0,
  .lock      = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radio_putreg
 *
 * Description:
 *   Put register value
 *
 ****************************************************************************/

static void nrf54l_radio_putreg(struct nrf54l_radio_dev_s *dev,
                               uint32_t offset,
                               uint32_t value)
{
  putreg32(value, dev->base + offset);
}

/****************************************************************************
 * Name: nrf54l_radio_getreg
 *
 * Description:
 *   Get register value
 *
 ****************************************************************************/

static uint32_t nrf54l_radio_getreg(struct nrf54l_radio_dev_s *dev,
                                   uint32_t offset)
{
  return getreg32(dev->base + offset);
}

/****************************************************************************
 * Name: nrf54l_radio_inten
 *
 * Description:
 *   Enable interrupts.
 *
 ****************************************************************************/

static void nrf54l_radio_inten(struct nrf54l_radio_dev_s *dev, uint32_t irq)
{
  nrf54l_radio_putreg(dev, NRF54L_RADIO_INTENSET00_OFFSET, irq);
}

/****************************************************************************
 * Name: nrf54l_radio_intclr
 *
 * Description:
 *   Disable interrupts.
 *
 ****************************************************************************/

static void nrf54l_radio_intclr(struct nrf54l_radio_dev_s *dev, uint32_t irq)
{
  nrf54l_radio_putreg(dev, NRF54L_RADIO_INTENCLR00_OFFSET, irq);
}

/****************************************************************************
 * Name: nrf54l_radio_shorts
 *
 * Description:
 *   Configure RADIO shorts
 *
 ****************************************************************************/

static void nrf54l_radio_shorts(struct nrf54l_radio_dev_s *dev, uint32_t sh)
{
  nrf54l_radio_putreg(dev, NRF54L_RADIO_SHORTS_OFFSET, sh);
}

/****************************************************************************
 * Name: nrf54l_radio_power
 *
 * Description:
 *   Power on/off radio
 *
 ****************************************************************************/

static int nrf54l_radio_power(struct nrf54l_radio_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  /* RADIO has no POWER register */

  if (state == true)
    {
      /* Reset all registers except DMA, events and interrupts */

      nrf54l_radio_putreg(dev, NRF54L_RADIO_INTENCLR00_OFFSET, 0xffffffff);
      nrf54l_radio_putreg(dev, NRF54L_RADIO_TASKS_SOFTRESET_OFFSET, 1);
    }
  else
    {
      /* SOFTRESET must be used in the DISABLED state */

      nrf54l_radio_putreg(dev, NRF54L_RADIO_TASKS_DISABLE_OFFSET, 1);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radio_mode_set
 *
 * Description:
 *   Set radio mode
 *
 ****************************************************************************/

static int nrf54l_radio_mode_set(struct nrf54l_radio_dev_s *dev,
                                uint8_t mode)
{
  uint32_t regval = 0;
  int ret = OK;

  DEBUGASSERT(dev);

  /* Change mode if needed */

  if (dev->mode == mode)
    {
      goto errout;
    }

  /* Check if mode is valid */

#ifndef CONFIG_NRF54L_HAVE_IEEE802154
  if (mode >= NRF54L_RADIO_MODE_LAST)
#else
  if (mode >= NRF54L_RADIO_MODE_LAST &&
      mode != NRF54L_RADIO_MODE_IEEE802154)
#endif
    {
      wlerr("ERROR: unsupported RADIO mode %d\n", mode);
      ret = -EINVAL;
      goto errout;
    }

  regval = (mode << RADIO_MODE_SHIFT);
  nrf54l_radio_putreg(dev, NRF54L_RADIO_MODE_OFFSET, regval);

#if defined(CONFIG_NRF54L_HAVE_IEEE802154) && defined(HAVE_RADIO_MLTPAN6)
  /* MLTPAN-6, nRF54LM20A only revision 0 */

  if (mode == NRF54L_RADIO_MODE_IEEE802154)
    {
#  ifdef CONFIG_ARCH_CHIP_NRF54LM20A
      if (getreg32(NRF54L_ERRATA_REVISION) == 0)
#  endif
        {
          putreg32(2, NRF54L_RADIO_MLTPAN6);
        }
    }
#endif

  /* Store mode */

  dev->mode = mode;

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radio_freq_set
 *
 * Description:
 *   Set radio frequency (in MHz)
 *
 ****************************************************************************/

static int nrf54l_radio_freq_set(struct nrf54l_radio_dev_s *dev,
                                uint32_t freq)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Check input */

  if (freq < 2360 || freq > 2500)
    {
      wlerr("ERROR: unsupported radio frequency %" PRId32" MHz\n", freq);
      ret = -EINVAL;
      goto errout;
    }

  /* Map frequency to lower band */

  if (freq < 2400)
    {
      regval |= RADIO_FREQUENCY_MAP_2360MHZ;
      freq -= 2360;
    }
  else
    {
      freq -= 2400;
    }

  regval |= freq;
  nrf54l_radio_putreg(dev, NRF54L_RADIO_FREQUENCY_OFFSET, regval);

errout:
  return ret;
};

/****************************************************************************
 * Name: nrf54l_radio_rssi_get
 *
 * Description:
 *   Get RSSI sample
 *
 ****************************************************************************/

static int nrf54l_radio_rssi_get(struct nrf54l_radio_dev_s *dev,
                                int8_t *rssi)
{
  uint32_t regval = 0;

  /* Start the RSSI measurement */

  nrf54l_radio_putreg(dev, NRF54L_RADIO_TASKS_RSSISTART_OFFSET, 1);

  /* Get the RSSI sample */

  regval = nrf54l_radio_getreg(dev, NRF54L_RADIO_RSSISAMPLE_OFFSET);
  *rssi = -(int8_t)regval;

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radio_addr_set
 *
 * Description:
 *   Set radio logical address
 *
 ****************************************************************************/

static int nrf54l_radio_addr_set(struct nrf54l_radio_dev_s *dev, uint8_t i,
                                struct nrf54l_radio_addr_s *addr)
{
  uint32_t basereg      = 0;
  uint32_t prefixreg    = 0;
  uint32_t base_now     = 0;
  uint32_t prefix_now   = 0;
  uint32_t base_new     = 0;
  uint32_t prefix_new   = 0;
  uint32_t prefix_shift = 0;
  int      ret          = OK;

  /* Assert input */

  if (i > NRF54L_RADIO_LOGICAL_ADDRESS_MAX - 1)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get data specific for given logical address */

  if (i == 0)
    {
      /* Logical address 0 - BASE0 and PREFIX0.AP0 */

      basereg   = NRF54L_RADIO_BASE0_OFFSET;
      prefixreg = NRF54L_RADIO_PREFIX0_OFFSET;

      prefix_shift = 0;
    }
  else if (i < 4)
    {
      /* Logical address 1-3 - BASE1 and PREFIX0 */

      basereg   = NRF54L_RADIO_BASE1_OFFSET;
      prefixreg = NRF54L_RADIO_PREFIX0_OFFSET;

      prefix_shift = (i * 8);
    }
  else
    {
      /* Logical address 1-3 - BASE1 and PREFIX1 */

      basereg   = NRF54L_RADIO_BASE1_OFFSET;
      prefixreg = NRF54L_RADIO_PREFIX1_OFFSET;

      prefix_shift = (i - 4) * 8;
    }

  /* Get current BASE and PREFIX registers */

  base_now = nrf54l_radio_getreg(dev, basereg);
  UNUSED(base_now);
  prefix_now = nrf54l_radio_getreg(dev, prefixreg);

  /* TODO: check if new address match to old BASE1 */

  if (basereg == NRF54L_RADIO_BASE1_OFFSET)
    {
    }

  /* Get new BASE */

  base_new = (addr->a1 | addr->a2 << 8 | addr->a3 << 16 | addr->a4 << 24);

  /* Write new base */

  nrf54l_radio_putreg(dev, basereg, base_new);

  /* Write new PREFIX */

  prefix_new = prefix_now;
  prefix_new &= ~(0xff << prefix_shift);
  prefix_new |= (addr->a0 << prefix_shift);

  nrf54l_radio_putreg(dev, prefixreg, prefix_new);

  /* Copy address */

  memcpy(&dev->addr[i], addr, sizeof(struct nrf54l_radio_addr_s));

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radio_txpower_set
 *
 * Description:
 *  Set TX power
 *
 ****************************************************************************/

static int nrf54l_radio_txpower_set(struct nrf54l_radio_dev_s *dev,
                                   uint8_t txpower)
{
  int ret = OK;

  /* Do nothing if already configured */

  if (dev->txpower == txpower)
    {
      goto errout;
    }

  /* Verify input */

  if (txpower > RADIO_TXPOWER_MAXDBM)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf54l_radio_putreg(dev, NRF54L_RADIO_TXPOWER_OFFSET, txpower);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radio_tifs_set
 *
 * Description:
 *   Set interframe spacing in us
 *
 ****************************************************************************/

static int nrf54l_radio_tifs_set(struct nrf54l_radio_dev_s *dev, uint16_t us)
{
  int ret = OK;

  /* Do nothing if already configured */

  if (dev->tifs == us)
    {
      goto errout;
    }

  /* Verify input */

  if (us > RADIO_TIFS_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf54l_radio_putreg(dev, NRF54L_RADIO_TIFS_OFFSET, us);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radio_pkt_cfg
 *
 * Description:
 *   Configure radio packet
 *
 ****************************************************************************/

static int nrf54l_radio_pkt_cfg(struct nrf54l_radio_dev_s *dev,
                               struct nrf54l_radio_pktcfg_s *cfg)
{
  uint32_t pcnf0 = 0;
  uint32_t pcnf1 = 0;
  int      ret   = OK;

  /* LENGTH field length */

  if (cfg->lf_len > RADIO_PCNF0_LFLEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf0 |= (cfg->lf_len << RADIO_PCNF0_LFLEN_SHIFT);

  /* Configure S0 field */

  if (cfg->s0_len > RADIO_PCNF0_S0LEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf0 |= (cfg->s0_len << RADIO_PCNF0_S0LEN_SHIFT);

  /* Configure S1 field */

  if (cfg->s1_len > RADIO_PCNF0_S1LEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf0 |= (cfg->s1_len << RADIO_PCNF0_S1LEN_SHIFT);

  /* S1 in RAM only if S1LEN > 0 */

  pcnf0 &= (~RADIO_PCNF0_S1INCL);

#ifdef HAVE_RADIO_BLELR
  /* Configure code indicator length */

  if (cfg->ci_len > RADIO_PCNF0_CILEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf0 |= (cfg->ci_len << RADIO_PCNF0_CILEN_SHIFT);

  /* Configure preamble length */

  if (cfg->pl_len > NRF54L_RADIO_PREAMBLE_LONGRANGE)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf0 |= (cfg->pl_len << RADIO_PCNF0_PLEN_SHIFT);

  /* Configure TERM length */

  if (cfg->term_len > RADIO_PCNF0_TERMLEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf0 |= (cfg->term_len << RADIO_PCNF0_TERMLEN_SHIFT);

  /* Include CRC in LENGTH or not */

  pcnf0 |= (cfg->crcinc << RADIO_PCNF0_CRCINC_SHIFT);
#endif

  /* Configure maximum payload length */

  if (cfg->max_len > RADIO_PCNF1_MAXLEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf1 |= (cfg->max_len << RADIO_PCNF1_MAXLEN_SHIFT);

  /* Configure static payload length */

  if (cfg->stat_len > RADIO_PCNF1_STATLEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf1 |= (cfg->stat_len << RADIO_PCNF1_STATLEN_SHIFT);

  /* Configure base address length */

  if (cfg->bal_len)
    {
      if (cfg->bal_len < RADIO_PCNF1_BALEN_MIN ||
          cfg->bal_len > RADIO_PCNF1_BALEN_MAX)
        {
          ret = -EINVAL;
          goto errout;
        }

      pcnf1 |= (cfg->bal_len << RADIO_PCNF1_BALEN_SHIFT);
    }

  /* Configure on-air endianness of packet */

  pcnf1 |= (cfg->endian << RADIO_PCNF1_ENDIAN_SHIFT);

  /* Enable whitening */

  pcnf1 |= (cfg->whiteen << RADIO_PCNF1_WHITEEN_SHIFT);

  /* Write registers */

  nrf54l_radio_putreg(dev, NRF54L_RADIO_PCNF0_OFFSET, pcnf0);
  nrf54l_radio_putreg(dev, NRF54L_RADIO_PCNF1_OFFSET, pcnf1);

  /* Copy packet configuration */

  memcpy(&dev->pktcfg, cfg, sizeof(struct nrf54l_radio_pktcfg_s));

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radio_white_set
 *
 * Description:
 *   Configure data whitening initial value
 *
 ****************************************************************************/

static int nrf54l_radio_white_set(struct nrf54l_radio_dev_s *dev,
                                 uint8_t init)
{
  uint32_t regval = 0;
  int      ret    = OK;

  /* Return error if whitening is disabled */

  if (dev->pktcfg.whiteen == false)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Configure whitening initial value */

  if (init > RADIO_DATAWHITE_IV_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  regval = nrf54l_radio_getreg(dev, NRF54L_RADIO_DATAWHITE_OFFSET);
  regval &= ~RADIO_DATAWHITE_IV_MASK;
  regval |= (init << RADIO_DATAWHITE_IV_SHIFT);
  nrf54l_radio_putreg(dev, NRF54L_RADIO_DATAWHITE_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radio_crc_cfg
 *
 * Description:
 *   Configure packet CRC
 *
 ****************************************************************************/

static int nrf54l_radio_crc_cfg(struct nrf54l_radio_dev_s *dev,
                               struct nrf54l_radio_crc_s *cfg)
{
  uint32_t regval = 0;
  int      ret    = OK;

  /* Configure CRC length */

  if (cfg->len > NRF54L_RADIO_CRC_LEN_3B)
    {
      ret = -EINVAL;
      goto errout;
    }

  regval |= (cfg->len << RADIO_CRCCNF_LEN_SHIFT);

  /* Configure CRC SKIPADDR */

  if (cfg->skip > NRF54L_RADIO_CRC_SKIPADDR_IEEE802154)
    {
      ret = -EINVAL;
      goto errout;
    }

  regval |= (cfg->skip << RADIO_CRCCNF_SKIPADDR_SHIFT);
  nrf54l_radio_putreg(dev, NRF54L_RADIO_CRCCNF_OFFSET, regval);

  /* Configure CRC POLY */

  if (cfg->poly > RADIO_CRCPOLY_MASK)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf54l_radio_putreg(dev, NRF54L_RADIO_CRCPOLY_OFFSET, cfg->poly);

  /* Configure CRC INIT */

  if (cfg->init > RADIO_CRCINIT_MASK)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf54l_radio_putreg(dev, NRF54L_RADIO_CRCINIT_OFFSET, cfg->init);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radio_dumpregs
 *
 * Description:
 *   Dump radio registers
 *
 ****************************************************************************/

static void nrf54l_radio_dumpregs(struct nrf54l_radio_dev_s *dev)
{
  printf("\nnrf54l_radio_dumpregs:\n");

  printf("SHORTS       0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_SHORTS_OFFSET));
  printf("INTENSET00   0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_INTENSET00_OFFSET));
  printf("CRCSTATUS    0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_CRCSTATUS_OFFSET));
  printf("RXMATCH      0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_RXMATCH_OFFSET));
  printf("RXCRC        0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_RXCRC_OFFSET));
  printf("DAI          0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAI_OFFSET));
  printf("PDUSTAT      0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_PDUSTAT_OFFSET));
  printf("PACKETPTR    0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_PACKETPTR_OFFSET));
  printf("FREQUENCY    0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_FREQUENCY_OFFSET));
  printf("TXPOWER      0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_TXPOWER_OFFSET));
  printf("MODE         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_MODE_OFFSET));
  printf("PCNF0        0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_PCNF0_OFFSET));
  printf("PCNF1        0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_PCNF1_OFFSET));
  printf("BASE0        0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_BASE0_OFFSET));
  printf("BASE1        0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_BASE1_OFFSET));
  printf("PREFIX0      0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_PREFIX0_OFFSET));
  printf("PREFIX1      0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_PREFIX1_OFFSET));
  printf("TXADDRESS    0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_TXADDRESS_OFFSET));
  printf("RXADDRESSES  0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_RXADDRESSES_OFFSET));
  printf("CRCCNF       0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_CRCCNF_OFFSET));
  printf("CRCPOLY      0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_CRCPOLY_OFFSET));
  printf("CRCINIT      0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_CRCINIT_OFFSET));
  printf("TIFS         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_TIFS_OFFSET));
  printf("RSSISAMPLE   0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_RSSISAMPLE_OFFSET));
  printf("STATE        0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_STATE_OFFSET));
  printf("DATAWHITE    0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DATAWHITE_OFFSET));
  printf("BCC          0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_BCC_OFFSET));
  printf("DAB0         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAB_OFFSET(0)));
  printf("DAB1         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAB_OFFSET(1)));
  printf("DAB2         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAB_OFFSET(2)));
  printf("DAB3         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAB_OFFSET(3)));
  printf("DAB4         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAB_OFFSET(4)));
  printf("DAB5         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAB_OFFSET(5)));
  printf("DAB6         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAB_OFFSET(6)));
  printf("DAB7         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAB_OFFSET(7)));
  printf("DAP0         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAP_OFFSET(0)));
  printf("DAP1         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAP_OFFSET(1)));
  printf("DAP2         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAP_OFFSET(2)));
  printf("DAP3         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAP_OFFSET(3)));
  printf("DAP4         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAP_OFFSET(4)));
  printf("DAP5         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAP_OFFSET(5)));
  printf("DAP6         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAP_OFFSET(6)));
  printf("DAP7         0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DAP_OFFSET(7)));
  printf("DACNF        0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_DACNF_OFFSET));
  printf("MHRMATCHCONF 0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_MHRMATCHCONF_OFFSET));
  printf("MHRMATCHMASK 0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_MHRMATCHMASK_OFFSET));
  printf("TIMING       0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_TIMING_OFFSET));
  printf("SFD          0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_SFD_OFFSET));
  printf("EDCTRL       0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_EDCTRL_OFFSET));
  printf("EDSAMPLE     0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_EDSAMPLE_OFFSET));
  printf("CCACTRL      0x%08" PRIx32 "\n",
         nrf54l_radio_getreg(dev, NRF54L_RADIO_CCACTRL_OFFSET));
}

#ifdef CONFIG_NRF54L_HAVE_IEEE802154

/****************************************************************************
 * Name: nrf54l_radio_sfd_set
 *
 * Description:
 *   Set SFD.
 *
 ****************************************************************************/

static void nrf54l_radio_sfd_set(struct nrf54l_radio_dev_s *dev, uint8_t sfd)
{
  nrf54l_radio_putreg(dev, NRF54L_RADIO_SFD_OFFSET, sfd);
}

/****************************************************************************
 * Name: nrf54l_radio_edcnt_set
 *
 * Description:
 *   Set EDCNT.
 *
 ****************************************************************************/

static void nrf54l_radio_edcnt_set(struct nrf54l_radio_dev_s *dev,
                                  uint32_t edcnt)
{
  uint32_t regval = 0;

  regval = nrf54l_radio_getreg(dev, NRF54L_RADIO_EDCTRL_OFFSET);
  regval &= ~RADIO_EDCNT_MASK;
  regval |= (edcnt << RADIO_EDCNT_SHIFT) & RADIO_EDCNT_MASK;
  nrf54l_radio_putreg(dev, NRF54L_RADIO_EDCTRL_OFFSET, regval);
}

/****************************************************************************
 * Name: nrf54l_radio_cca_cfg
 *
 * Description:
 *   Set CCA configuration
 *
 ****************************************************************************/

static void nrf54l_radio_cca_cfg(struct nrf54l_radio_dev_s *dev,
                                struct nrf54l_radio_cca_s *cca)
{
  uint32_t regval = 0;

  /* CCA mode of operation */

  regval |= (cca->mode << 0);

  /* CCA energy busy threshold */

  regval |= ((cca->edthres << RADIO_CCACTRL_CCAEDTHRES_SHIFT)
             & RADIO_CCACTRL_CCAEDTHRES_MASK);

  /* CCA correlator busy threshold */

  regval |= ((cca->corrthres << RADIO_CCACTRL_CCACORRTHRES_SHIFT)
             & RADIO_CCACTRL_CCACORRTHRES_MASK);

  /* Limit for occurrences above CCACORRTHRES */

  regval |= ((cca->corrcnt << RADIO_CCACTRL_CCACORRCNT_SHIFT)
             & RADIO_CCACTRL_CCACORRCNT_MASK);

  nrf54l_radio_putreg(dev, NRF54L_RADIO_CCACTRL_OFFSET, regval);
}
#endif

/****************************************************************************
 * Name: nrf54l_radio_reset
 *
 * Description:
 *   Reset radio
 *
 ****************************************************************************/

static void nrf54l_radio_reset(struct nrf54l_radio_dev_s *dev)
{
  /* Turn off radio power */

  nrf54l_radio_power(dev, false);
  up_udelay(100);
  nrf54l_radio_power(dev, true);
  up_udelay(100);

  /* Reset radio state */

  dev->mode    = 0;
  dev->tifs    = 0;
  dev->txpower = 0;
  memset(&dev->pktcfg, 0, sizeof(dev->pktcfg));
  memset(&dev->addr, 0, sizeof(dev->addr));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radio_initialize
 *
 * Description:
 *   Initialize NRF54L radio device
 *
 ****************************************************************************/

struct nrf54l_radio_dev_s *
nrf54l_radio_initialize(int intf, struct nrf54l_radio_board_s *board)
{
  struct nrf54l_radio_dev_s *dev = NULL;

  /* Get radio interface */

  switch (intf)
    {
      case 0:
        {
          wlinfo("radio0 selecred\n");
          dev = &g_nrf54l_radio_dev_1;
          break;
        }

      /* For now only one radio interface is available */

      default:
        {
          wlerr("ERROR: No radio interface defined\n");
          goto errout;
        }
    }

  /* Reset some data */

  memset(&dev->pktcfg, 0, sizeof(struct nrf54l_radio_pktcfg_s));

  /* Connect board-specific data */

  dev->board = board;

  /* Reset radio */

  nrf54l_radio_reset(dev);

  return dev;

errout:
  return NULL;
}
