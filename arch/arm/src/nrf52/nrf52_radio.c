/****************************************************************************
 * arch/arm/src/nrf52/nrf52_radio.c
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
#include <debug.h>
#include <string.h>
#include <stdio.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nrf52_gpio.h"
#include "nrf52_radio.h"

#include "hardware/nrf52_radio.h"

#warning NRF52 RADIO support is EXPERIMENTAL!

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF52_RADIO_RXBUFFER (255)
#define NRF52_RADIO_TXBUFFER (255)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Radio registers access ***************************************************/

static void nrf52_radio_putreg(struct nrf52_radio_dev_s *dev,
                               uint32_t offset,
                               uint32_t value);
static uint32_t nrf52_radio_getreg(struct nrf52_radio_dev_s *dev,
                                   uint32_t offset);

/* Radio operations *********************************************************/

static int nrf52_radio_power(struct nrf52_radio_dev_s *dev, bool state);
static int nrf52_radio_mode_set(struct nrf52_radio_dev_s *dev,
                                uint8_t mode);
static int nrf52_radio_freq_set(struct nrf52_radio_dev_s *dev,
                                uint32_t freq);
static int nrf52_radio_rssi_get(struct nrf52_radio_dev_s *dev,
                                int *rssi);
static int nrf52_radio_txpower_set(struct nrf52_radio_dev_s *dev,
                                   uint8_t txpower);
static int nrf52_radio_tifs_set(struct nrf52_radio_dev_s *dev,
                                uint16_t us);
static int nrf52_radio_pkt_cfg(struct nrf52_radio_dev_s *dev,
                               struct nrf52_radio_pktcfg_s *cfg);
static int nrf52_radio_crc_cfg(struct nrf52_radio_dev_s *dev,
                               struct nrf52_radio_crc_s *cfg);
static int nrf52_radio_white_set(struct nrf52_radio_dev_s *dev,
                                 uint8_t init);
static int nrf52_radio_addr_set(struct nrf52_radio_dev_s *dev, uint8_t i,
                                struct nrf52_radio_addr_s *addr);
static int nrf52_radio_write(struct nrf52_radio_dev_s *dev,
                             uint8_t *buf, int len);
static int nrf52_radio_read(struct nrf52_radio_dev_s *dev,
                            uint8_t *buf, int len);
static void nrf52_radio_dumpregs(struct nrf52_radio_dev_s *dev);

/* Radio interrupts *********************************************************/

static int nrf52_radio_isr(int irq, void *context, void *arg);
static int nrf52_radio_isr_rx(struct nrf52_radio_dev_s *dev);
static int nrf52_radio_isr_tx(struct nrf52_radio_dev_s *dev);

/* Radio configuration ******************************************************/

static int nrf52_radio_setup(struct nrf52_radio_dev_s *dev);
static int nrf52_radio_reset(struct nrf52_radio_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NRF52 radio operations */

struct nrf52_radio_dev_s;
struct nrf52_radio_ops_s g_nrf52_radio_ops =
{
  .power        = nrf52_radio_power,
  .mode_set     = nrf52_radio_mode_set,
  .freq_set     = nrf52_radio_freq_set,
  .rssi_get     = nrf52_radio_rssi_get,
  .txpower_set  = nrf52_radio_txpower_set,
  .tifs_set     = nrf52_radio_tifs_set,
  .pkt_cfg      = nrf52_radio_pkt_cfg,
  .white_set    = nrf52_radio_white_set,
  .crc_cfg      = nrf52_radio_crc_cfg,
  .addr_set     = nrf52_radio_addr_set,
  .read         = nrf52_radio_read,
  .write        = nrf52_radio_write,
  .dumpregs     = nrf52_radio_dumpregs
};

/* RX buffer 1 */

uint8_t g_nrf52_radio_dev_rx1[NRF52_RADIO_RXBUFFER];

/* TX buffer 1 */

uint8_t g_nrf52_radio_dev_tx1[NRF52_RADIO_TXBUFFER];

/* Radio device 1 */

struct nrf52_radio_dev_s g_nrf52_radio_dev_1 =
{
  .ops       = &g_nrf52_radio_ops,
  .irq       = NRF52_IRQ_RADIO,
  .base      = NRF52_RADIO_BASE,
  .mode      = 0,
  .rxbuf_len = NRF52_RADIO_RXBUFFER,
  .txbuf_len = NRF52_RADIO_TXBUFFER,
  .rxbuf     = g_nrf52_radio_dev_rx1,
  .txbuf     = g_nrf52_radio_dev_tx1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radio_putreg
 *
 * Description:
 *   Put register value
 *
 ****************************************************************************/

static void nrf52_radio_putreg(struct nrf52_radio_dev_s *dev,
                               uint32_t offset,
                               uint32_t value)
{
  putreg32(value, dev->base + offset);
}

/****************************************************************************
 * Name: nrf52_radio_getreg
 *
 * Description:
 *   Get register value
 *
 ****************************************************************************/

static uint32_t nrf52_radio_getreg(struct nrf52_radio_dev_s *dev,
                                   uint32_t offset)
{
  return getreg32(dev->base + offset);
}

/****************************************************************************
 * Name: nrf52_radio_power
 *
 * Description:
 *   Power on/off radio
 *
 ****************************************************************************/

static int nrf52_radio_power(struct nrf52_radio_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  if (state == true)
    {
      /* Turn on radio */

      nrf52_radio_putreg(dev, NRF52_RADIO_POWER_OFFSET, RADIO_POWER_ENABLE);
    }
  else
    {
      /* Turn off radio */

      nrf52_radio_putreg(dev, NRF52_RADIO_POWER_OFFSET, RADIO_POWER_DISABLE);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_radio_mode_set
 *
 * Description:
 *   Set radio mode
 *
 ****************************************************************************/

static int nrf52_radio_mode_set(struct nrf52_radio_dev_s *dev,
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

  if (mode > NRF52_RADIO_MODE_IEEE802154)
    {
      wlerr("ERROR: unsupported RADIO mode %d\n", mode);
      ret = -EINVAL;
      goto errout;
    }

  regval = (mode << RADIO_MODE_SHIFT);
  nrf52_radio_putreg(dev, NRF52_RADIO_MODE_OFFSET, regval);

  /* Store mode */

  dev->mode = mode;

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_freq_set
 *
 * Description:
 *   Set radio frequency (in MHz)
 *
 ****************************************************************************/

static int nrf52_radio_freq_set(struct nrf52_radio_dev_s *dev,
                                uint32_t freq)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Check input */

  if (freq < 2360 || freq > 2500)
    {
      wlerr("ERROR: unsupported radio frequency %d MHz\n", freq);
      ret = -EINVAL;
      goto errout;
    }

  /* Map frequency to lower band */

  if (freq < 2400)
    {
      regval |= RADIO_FREQUENCY_MAP_2360MHZ;
    }

  regval |= freq;
  nrf52_radio_putreg(dev, NRF52_RADIO_FREQUENCY_OFFSET, regval);

errout:
  return ret;
};

/****************************************************************************
 * Name: nrf52_radio_rssi_get
 *
 * Description:
 *   Get RSSI sample
 *
 ****************************************************************************/

static int nrf52_radio_rssi_get(struct nrf52_radio_dev_s *dev,
                                int *rssi)
{
  uint32_t regval = 0;

  /* Start the RSSI meassurement */

  nrf52_radio_putreg(dev, NRF52_RADIO_TASKS_RSSISTART_OFFSET,
                     RADIO_TASKS_RSSISTART);

  /* Wait for the RSSI sample */

  while (nrf52_radio_getreg(dev, NRF52_RADIO_EVENTS_RSSIEND_OFFSET));

  /* Get the RSSI sample */

  regval = nrf52_radio_getreg(dev, NRF52_RADIO_RSSISAMPLE_OFFSET);
  *rssi = -(int)regval;

  return OK;
}

/****************************************************************************
 * Name: nrf52_radio_addr_set
 *
 * Description:
 *   Set radio logical address
 *
 ****************************************************************************/

static int nrf52_radio_addr_set(struct nrf52_radio_dev_s *dev, uint8_t i,
                                struct nrf52_radio_addr_s *addr)
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

  if (i > NRF52_RADIO_LOGICAL_ADDRESS_MAX - 1)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get data specific for given logical address */

  if (i == 0)
    {
      /* Logical address 0 - BASE0 and PREFIX0.AP0 */

      basereg   = NRF52_RADIO_BASE0_OFFSET;
      prefixreg = NRF52_RADIO_PREFIX0_OFFSET;

      prefix_shift = 0;
    }
  else if (i < 4)
    {
      /* Logical address 1-3 - BASE1 and PREFIX0 */

      basereg   = NRF52_RADIO_BASE1_OFFSET;
      prefixreg = NRF52_RADIO_PREFIX0_OFFSET;

      prefix_shift = (i * 8);
    }
  else
    {
      /* Logical address 1-3 - BASE1 and PREFIX1 */

      basereg   = NRF52_RADIO_BASE1_OFFSET;
      prefixreg = NRF52_RADIO_PREFIX1_OFFSET;

      prefix_shift = (i - 4) * 8;
    }

  /* Get current BASE and PREFIX registers */

  base_now = nrf52_radio_getreg(dev, basereg);
  prefix_now = nrf52_radio_getreg(dev, prefixreg);

  /* TODO: check if new address match to old BASE1 */

  if (basereg == NRF52_RADIO_BASE1_OFFSET)
    {
#warning missing logic!
    }

  /* Get new BASE */

  base_new = (addr->a1 | addr->a2 << 8 | addr->a3 << 16 | addr->a4 << 24);

  /* Write new base */

  nrf52_radio_putreg(dev, basereg, base_new);

  /* Write new PREFIX */

  prefix_new = prefix_now;
  prefix_new &= ~(0xff << prefix_shift);
  prefix_new |= (addr->a0 << prefix_shift);

  nrf52_radio_putreg(dev, prefixreg, prefix_new);

  /* Copy address */

  memcpy(&dev->addr[i], addr, sizeof(struct nrf52_radio_addr_s));

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_txpower_set
 *
 * Description:
 *  Set TX power
 *
 ****************************************************************************/

static int nrf52_radio_txpower_set(struct nrf52_radio_dev_s *dev,
                                   uint8_t txpower)
{
  int ret = OK;

  /* Do nothing if already configured */

  if (dev->txpower == txpower)
    {
      goto errout;
    }

  /* Verify input */

  if (txpower > RADIO_TXPOWER_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf52_radio_putreg(dev, NRF52_RADIO_TXPOWER_OFFSET, txpower);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_tifs_set
 *
 * Description:
 *   Set interframe spacing in us
 *
 ****************************************************************************/

static int nrf52_radio_tifs_set(struct nrf52_radio_dev_s *dev,
                                uint16_t us)
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

  nrf52_radio_putreg(dev, NRF52_RADIO_TIFS_OFFSET, us);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_pkt_cfg
 *
 * Description:
 *   Configure radio packet
 *
 ****************************************************************************/

static int nrf52_radio_pkt_cfg(struct nrf52_radio_dev_s *dev,
                               struct nrf52_radio_pktcfg_s *cfg)
{
  uint32_t pcnf0 = 0;
  uint32_t pcnf1 = 0;
  int ret        = OK;

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

  /* Configure code indicator length */

  if (cfg->ci_len > RADIO_PCNF0_CILEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf0 |= (cfg->ci_len << RADIO_PCNF0_CILEN_SHIFT);

  /* Configure preamble length */

  if (cfg->pl_len > NRF52_RADIO_PREAMBLE_LONGRANGE)
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

  if (cfg->bal_len < RADIO_PCNF1_BALEN_MIN ||
      cfg->bal_len > RADIO_PCNF1_BALEN_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  pcnf1 |= (cfg->bal_len << RADIO_PCNF1_BALEN_SHIFT);

  /* Configure on-air endianness of packet */

  pcnf1 |= (cfg->endian << RADIO_PCNF1_ENDIAN_SHIFT);

  /* Enable whitening */

  pcnf1 |= (cfg->whiteen << RADIO_PCNF1_WHITEEN_SHIFT);

  /* Write registers */

  nrf52_radio_putreg(dev, NRF52_RADIO_PCNF0_OFFSET, pcnf0);
  nrf52_radio_putreg(dev, NRF52_RADIO_PCNF1_OFFSET, pcnf1);

  /* Copy packet configuration */

  memcpy(&dev->pktcfg, cfg, sizeof(struct nrf52_radio_pktcfg_s));

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_white_set
 *
 * Description:
 *   Configure data whitening initial value
 *
 ****************************************************************************/

static int nrf52_radio_white_set(struct nrf52_radio_dev_s *dev,
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

  if (init > RADIO_DATAWHITEIV_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  regval |= (init << RADIO_DATAWHITEIV_SHIFT);
  nrf52_radio_putreg(dev, NRF52_RADIO_DATAWHITEIV_OFFSET, init);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_crc_cfg
 *
 * Description:
 *   Configure packet CRC
 *
 ****************************************************************************/

static int nrf52_radio_crc_cfg(struct nrf52_radio_dev_s *dev,
                               struct nrf52_radio_crc_s *cfg)
{
  uint32_t regval = 0;
  int      ret    = OK;

  /* Configure CRC length */

  if (cfg->len > NRF52_RADIO_CRC_LEN_3B)
    {
      ret = -EINVAL;
      goto errout;
    }

  regval |= (cfg->len << RADIO_CRCCNF_LEN_SHIFT);

  /* Configure CRC SKIPADDR */

  if (cfg->skip > NRF52_RADIO_CRC_SKIPADDR_IEEE802154)
    {
      ret = -EINVAL;
      goto errout;
    }

  regval |= (cfg->skip << RADIO_CRCCNF_SKIPADDR_SHIFT);
  nrf52_radio_putreg(dev, NRF52_RADIO_CRCCNF_OFFSET, regval);

  /* Configure CRC POLY */

  if (cfg->poly > RADIO_CRCPOLY_MASK)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf52_radio_putreg(dev, NRF52_RADIO_CRCPOLY_OFFSET, cfg->poly);

  /* Configure CRC INIT */

  if (cfg->init > RADIO_CRCINIT_MASK)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf52_radio_putreg(dev, NRF52_RADIO_CRCINIT_OFFSET, cfg->init);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_write
 *
 * Description:
 *   Write radio packet
 *
 ****************************************************************************/

static int nrf52_radio_write(struct nrf52_radio_dev_s *dev,
                             uint8_t *buf, int len)
{
  int ret = OK;

  /* Lock device */

  ret = nxsem_wait(&dev->sem_excl);
  if (ret < 0)
    {
      return ret;
    }

  /*  */

  if (len > dev->txbuf_len)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Copy packet */

  memcpy(dev->txbuf, buf, len);

  /* Set packet pointer */

  nrf52_radio_putreg(dev, NRF52_RADIO_PACKETPTR_OFFSET, &dev->txbuf);

  /* Set state to TX */

  dev->state = NRF52_RADIO_STATE_TX;

  /* Start TX.
   * NOTE: shortcut between READ and START is enabled.
   */

  nrf52_radio_putreg(dev, NRF52_RADIO_TASKS_TXEN_OFFSET, RADIO_TASKS_TXEN);

  /* Wait for IRQ */

  ret = nxsem_wait(&dev->sem_isr);

errout:

  /* Unlock device */

  nxsem_post(&dev->sem_excl);

  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_read
 *
 * Description:
 *   Read radio packet
 *
 ****************************************************************************/

static int nrf52_radio_read(struct nrf52_radio_dev_s *dev,
                            uint8_t *buf, int len)
{
  int ret = OK;

  /* Lock radio */

  ret = nxsem_wait(&dev->sem_excl);
  if (ret < 0)
    {
      return ret;
    }

  /*  */

  if (len > dev->rxbuf_len)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Set packet pointer */

  nrf52_radio_putreg(dev, NRF52_RADIO_PACKETPTR_OFFSET, &dev->rxbuf);

  /* Set state to RX */

  dev->state = NRF52_RADIO_STATE_RX;

  /* Start RX.
   * NOTE: shortcut between READ and START is enabled.
   */

  nrf52_radio_putreg(dev, NRF52_RADIO_TASKS_RXEN_OFFSET, RADIO_TASKS_RXEN);

  /* Wait for IRQ */

  ret = nxsem_wait(&dev->sem_isr);

  /* Copy packet */

  memcpy(buf, dev->rxbuf, len);

errout:

  /* Unlock radio */

  nxsem_post(&dev->sem_excl);

  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_dumpregs
 *
 * Description:
 *   Dump radio registers
 *
 ****************************************************************************/

static void nrf52_radio_dumpregs(struct nrf52_radio_dev_s *dev)
{
  printf("\nnrf52_radio_dumpregs:\n");

  printf("SHORTS       0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_SHORTS_OFFSET));
  printf("INTENSET     0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_INTENSET_OFFSET));
  printf("CRCSTATUS    0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_CRCSTATUS_OFFSET));
  printf("RXMATCH      0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_RXMATCH_OFFSET));
  printf("RXCRC        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_RXCRC_OFFSET));
  printf("DAI          0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAI_OFFSET));
  printf("PDUSTAT      0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_PDUSTAT_OFFSET));
  printf("PACKETPTR    0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_PACKETPTR_OFFSET));
  printf("FREQUENCY    0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_FREQUENCY_OFFSET));
  printf("TXPOWER      0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_TXPOWER_OFFSET));
  printf("MODE         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_MODE_OFFSET));
  printf("PCNF0        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_PCNF0_OFFSET));
  printf("PCNF1        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_PCNF1_OFFSET));
  printf("BASE0        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_BASE0_OFFSET));
  printf("BASE1        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_BASE1_OFFSET));
  printf("PREFIX0      0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_PREFIX0_OFFSET));
  printf("PREFIX1      0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_PREFIX1_OFFSET));
  printf("TXADDRESS    0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_TXADDRESS_OFFSET));
  printf("RXADDRESS    0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_RXADDRESS_OFFSET));
  printf("CRCCNF       0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_CRCCNF_OFFSET));
  printf("CRCPOLY      0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_CRCPOLY_OFFSET));
  printf("CRCINIT      0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_CRCINIT_OFFSET));
  printf("TIFS         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_TIFS_OFFSET));
  printf("RSSISAMPLE   0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_RSSISAMPLE_OFFSET));
  printf("STATE        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_STATE_OFFSET));
  printf("DATAWHITEIV  0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DATAWHITEIV_OFFSET));
  printf("BCC          0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_BCC_OFFSET));
  printf("DAB0         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAB_OFFSET(0)));
  printf("DAB1         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAB_OFFSET(1)));
  printf("DAB2         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAB_OFFSET(2)));
  printf("DAB3         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAB_OFFSET(3)));
  printf("DAB4         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAB_OFFSET(4)));
  printf("DAB5         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAB_OFFSET(5)));
  printf("DAB6         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAB_OFFSET(6)));
  printf("DAB7         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAB_OFFSET(6)));
  printf("DAP0         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAP_OFFSET(0)));
  printf("DAP1         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAP_OFFSET(1)));
  printf("DAP2         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAP_OFFSET(2)));
  printf("DAP3         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAP_OFFSET(3)));
  printf("DAP4         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAP_OFFSET(4)));
  printf("DAP5         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAP_OFFSET(5)));
  printf("DAP6         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAP_OFFSET(6)));
  printf("DAP7         0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DAP_OFFSET(6)));
  printf("DACNF        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_DACNF_OFFSET));
  printf("MHRMATCHCONF 0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_MHRMATCHCONF_OFFSET));
  printf("MHRMATCHMAS  0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_MHRMATCHMAS_OFFSET));
  printf("MODECNF0     0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_MODECNF0_OFFSET));
  printf("SFD          0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_SFD_OFFSET));
  printf("EDCNT        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_EDCNT_OFFSET));
  printf("EDSAMPLE     0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_EDSAMPLE_OFFSET));
  printf("CCACTRL      0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_CCACTRL_OFFSET));
  printf("POWER        0x%08x\n",
         nrf52_radio_getreg(dev, NRF52_RADIO_POWER_OFFSET));
}

/****************************************************************************
 * Name: nrf52_radio_isr_rx
 *
 * Description:
 *   RX radio interrupt handler
 *
 ****************************************************************************/

static int nrf52_radio_isr_rx(struct nrf52_radio_dev_s *dev)
{
  /* RX done */

  nxsem_post(&dev->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: nrf52_radio_isr_tx
 *
 * Description:
 *   TX radio interrupt handler
 *
 ****************************************************************************/

static int nrf52_radio_isr_tx(struct nrf52_radio_dev_s *dev)
{
  /* TX done */

  nxsem_post(&dev->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: nrf52_radio_isr
 *
 * Description:
 *   Radio interrupt handler
 *
 ****************************************************************************/

static int nrf52_radio_isr(int irq, void *context, void *arg)
{
  struct nrf52_radio_dev_s *dev   = (struct nrf52_radio_dev_s *)arg;
  int                       ret   = OK;
  uint32_t                  state = 0;

  DEBUGASSERT(dev);

  /* Get radio state */

  wlinfo("RADIO ISR STATE=%d\n", dev->state);

  /* Handle radio state */

  switch (dev->state)
    {
      case NRF52_RADIO_STATE_RX:
        {
          /* Transmit DONE */

          ret = nrf52_radio_isr_rx(dev);

          break;
        }

      case NRF52_RADIO_STATE_TX:
        {
          /* Receive DONE */

          ret = nrf52_radio_isr_tx(dev);

          break;
        }

      case NRF52_RADIO_STATE_DISABLED:
        {
          break;
        }

      default:
        {
          ASSERT(0);
          break;
        }
    }

  /* Clear END event */

  nrf52_radio_putreg(dev, NRF52_RADIO_EVENTS_END_OFFSET, 0);

  /* Update radio state
   * NOTE: shortcut between END and DISABLE is enabled.
   */

  dev->state = NRF52_RADIO_STATE_DISABLED;

  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_setup
 *
 * Description:
 *   Initial RADIO setup
 *
 ****************************************************************************/

static int nrf52_radio_setup(struct nrf52_radio_dev_s *dev)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Configure interrupts:
   *  1. END - packet sent or received
   */

  regval = (RADIO_INT_END);
  nrf52_radio_putreg(dev, NRF52_RADIO_INTENSET_OFFSET, regval);

  /* Configure shortucts:
   *   1. shortcut between READY and START
   *   2. shortcut between END and DISABLE
   */

  regval = RADIO_SHORTS_READY_START;
  regval |= RADIO_SHORTS_END_DISABLE;
  nrf52_radio_putreg(dev, NRF52_RADIO_SHORTS_OFFSET, regval);

  /* Power on radio */

  nrf52_radio_power(dev, true);

  return ret;
}

/****************************************************************************
 * Name: nrf52_radio_reset
 *
 * Description:
 *   Reset radio
 *
 ****************************************************************************/

static int nrf52_radio_reset(struct nrf52_radio_dev_s *dev)
{
  /* Turn off radio power */

  nrf52_radio_power(dev, false);

  /* Wait some time */

  nxsig_usleep(100000);

  /* Turn on radio power */

  nrf52_radio_power(dev, true);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radio_initialize
 *
 * Description:
 *   Initialize NRF52 radio device
 *
 ****************************************************************************/

struct nrf52_radio_dev_s *
nrf52_radio_initialize(int intf, struct nrf52_radio_board_s *board)
{
  struct nrf52_radio_dev_s *dev = NULL;
  int                       ret = OK;

  /* Get radio interface */

  switch (intf)
    {
      case 0:
        {
          wlinfo("radio0 selecred\n");
          dev = &g_nrf52_radio_dev_1;
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

  memset(&dev->pktcfg, 0, sizeof(struct nrf52_radio_pktcfg_s));
  memset(dev->rxbuf, 0, NRF52_RADIO_RXBUFFER);
  memset(dev->txbuf, 0, NRF52_RADIO_TXBUFFER);

  /* Attach radio interrupt */

  irq_attach(dev->irq, nrf52_radio_isr, dev);
  up_enable_irq(dev->irq);

  /* Initialize semaphores */

  nxsem_init(&dev->sem_excl, 0, 1);

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&dev->sem_isr, 0, 0);
  nxsem_set_protocol(&dev->sem_isr, SEM_PRIO_NONE);

  /* Connect board-specific data */

  dev->board = board;

  /* Reset radio */

  ret = nrf52_radio_reset(dev);
  if (ret < 0)
    {
      wlerr("ERROR: failed to reset radio interface %d\n", ret);
      goto errout;
    }

  /* Initial radio setup */

  ret = nrf52_radio_setup(dev);
  if (ret < 0)
    {
      wlerr("ERROR: failed to setup radio interface %d\n", ret);
      goto errout;
    }

  return dev;

errout:
  return NULL;
}
