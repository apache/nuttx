/****************************************************************************
 * arch/arm/src/nrf52/nrf52_radio.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_RADIO_H
#define __ARCH_ARM_SRC_NRF52_NRF52_RADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF52_RADIO_LOGICAL_ADDRESS_MAX (8)

/* Ops */

#define NRF52_RADIO_RESET(lower)         lower->ops->reset(lower)
#define NRF52_RADIO_PUTREG(lower, o, v)  lower->ops->putreg(lower, o, v)
#define NRF52_RADIO_GETREG(lower, o)     lower->ops->getreg(lower, o)
#define NRF52_RADIO_INTEN(lower, i)      lower->ops->inten(lower, i)
#define NRF52_RADIO_INTCLR(lower, i)     lower->ops->intclr(lower, i)
#define NRF52_RADIO_SHRTSET(lower, s)    lower->ops->shorts(lower, s)
#define NRF52_RADIO_PWRSET(lower, pwr)   lower->ops->power(lower, pwr)
#define NRF52_RADIO_MODESET(lower, m)    lower->ops->mode_set(lower, m)
#define NRF52_RADIO_FREQSET(lower, f)    lower->ops->freq_set(lower, f)
#define NRF52_RADIO_RSSIGET(lower, r)    lower->ops->rssi_get(lower, r)
#define NRF52_RADIO_TXPWRSET(lower, p)   lower->ops->txpower_set(lower, p)
#define NRF52_RADIO_TIFSSET(lower, p)    lower->ops->tifs_set(lower, t)
#define NRF52_RADIO_PKTCFG(lower, cfg)   lower->ops->pkt_cfg(lower, cfg)
#define NRF52_RADIO_CRCCFG(lower, cfg)   lower->ops->crc_cfg(lower, cfg)
#define NRF52_RADIO_WHITESET(lower, cfg) lower->ops->white_set(lower, c)
#define NRF52_RADIO_ADDRSET(lower, i, a) lower->ops->addr_set(lower, i, a)
#define NRF52_RADIO_DUMPREGS(lower)      lower->ops->dumpregs(lower)
#define NRF52_RADIO_SFDSET(lower, sfd)   lower->ops->sfd_set(lower, sfd)
#define NRF52_RADIO_EDCNTSET(lower, ec)  lower->ops->edcnt_set(lower, ec)
#define NRF52_RADIO_CCACFG(lower, cca)   lower->ops->cca_cfg(lower, cca)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Radio mode */

enum nrf52_radio_mode_e
{
  NRF52_RADIO_MODE_NRF1MBIT     = 0,
  NRF52_RADIO_MODE_NRF2MBIT     = 1,
  NRF52_RADIO_MODE_NRF250KBIT   = 2,
  NRF52_RADIO_MODE_BLE1MBIT     = 3,
  NRF52_RADIO_MODE_BLE2MBIT     = 4,
#ifdef CONFIG_NRF52_HAVE_BLELR
  NRF52_RADIO_MODE_BLELR125KBIT = 5,
  NRF52_RADIO_MODE_BLELR500KBIT = 6,
#endif
  NRF52_RADIO_MODE_LAST,

#ifdef CONFIG_NRF52_HAVE_IEEE802154
  NRF52_RADIO_MODE_IEEE802154   = 15
#endif
};

/* Preamble configuration */

enum nrf52_radio_preamble_e
{
  NRF52_RADIO_PREAMBLE_8BIT       = 0,
  NRF52_RADIO_PREAMBLE_16BIT      = 1,
  NRF52_RADIO_PREAMBLE_32BITZERO  = 2,
  NRF52_RADIO_PREAMBLE_LONGRANGE  = 3
};

/* Radio packet CRC length */

enum nrf52_radio_crc_len_e
{
  NRF52_RADIO_CRC_LEN_DIS   = 0,
  NRF52_RADIO_CRC_LEN_1B    = 1,
  NRF52_RADIO_CRC_LEN_2B    = 2,
  NRF52_RADIO_CRC_LEN_3B    = 3,
};

/* Radio packet CRC includes address */

enum nrf52_radio_crc_skipaddr_e
{
  NRF52_RADIO_CRC_SKIPADDR_INCLUDE    = 0,
  NRF52_RADIO_CRC_SKIPADDR_SKIP       = 1,
  NRF52_RADIO_CRC_SKIPADDR_IEEE802154 = 2,
};

/* CCA mode of operation */

enum nrf52_radio_cca_mode_e
{
  NRF52_RADIO_CCA_ED             = 0,
  NRF52_RADIO_CCA_CARRIER        = 1,
  NRF52_RADIO_CCA_CARRIER_AND_ED = 2,
  NRF52_RADIO_CCA_CARRIER_OR_ED  = 2,
  NRF52_RADIO_CCA_EDTEST1        = 4,
};

/* On air packet layout (no IEEE802154 mode):
 *
 * +---------------------------------------+
 * | FIRST                                 |
 * |----------+------+--------+----+-------+
 * | PREAMBLE | BASE | PREFIX | CI | TERM1 |
 * | LSB      | LSB  | LSB    |    |       |
 * |          | ADDRESS       |    |       |
 * +----------+---------------+----+-------+
 *
 * +----------------------------+
 * | Stored on RAM              |
 * |----+--------+----+---------|
 * | S0 | LENGTH | S1 | PAYLOAD |
 * |    |        |    |         |
 * |    |        |    |         |
 * +----+--------+----+---------+
 *
 * +---------------+
 * |         LAST  |
 * |-------+-------|
 * | CRC32 | TERM2 |
 * | MSB   |       |
 * |       |       |
 * +-------+-------+
 *
 * For IEEE802154 mode packet layout is different:
 *
 * +--------------------------------------------------------------------+
 * |         PHY protocol data unit (PPDU)                              |
 * +--------------------+-----+---------+-------------------------------+
 * | Preamble sequence  | SFD | Lenght  | PHY payload                   |
 * |--------------------+-----+---------+-------------------------------+
 * | 5 octets synchronization | 1 octet | Maximum 127 octets (PSDU)     |
 * | header (SHR)             | (PHR)   +-------------------------------+
 * |                          |         | MAC protocol data unit (MPDU) |
 * +--------------------------+---------+-------------------------------+
 *                            |  Stored on RAM                          |
 *                            +-----------------------------------------+
 *
 */

/* Radio packet configuration */

struct nrf52_radio_pktcfg_s
{
  uint8_t max_len;              /* Maximum length of payload */
  uint8_t stat_len;             /* Static payload length */
  uint8_t bal_len;              /* Base address length */
  uint8_t lf_len;               /* LENGTH length */
  uint8_t s0_len;               /* S0 length */
  uint8_t s1_len;               /* S1 length */
  uint8_t ci_len;               /* CI length */
  uint8_t pl_len;               /* Preamble length */
  uint8_t term_len;             /* TERM length */
  bool    crcinc;               /* LENGTH includes CRC */
  bool    endian;               /* On air endianness of packet:
                                 * 0 - little
                                 * 1 - big
                                 */
  bool   whiteen;               /* Whitening enabled */
};

/* Radio packet CRC configuration */

struct nrf52_radio_crc_s
{
  uint8_t  len;                 /* CRC length in number of bytes */
  uint8_t  skip;                /* Include or exclude address field out of CRC */
  uint32_t poly;                /* CRC polynominal */
  uint32_t init;                /* CRC initial value */
};

/* NRF52 on air address */

struct nrf52_radio_addr_s
{
  uint8_t a0;                   /* PREFIX */
  uint8_t a1;                   /* BASE[0] */
  uint8_t a2;                   /* BASE[1] */
  uint8_t a3;                   /* BASE[2] */
  uint8_t a4;                   /* BASE[3] */
};

#ifdef CONFIG_NRF52_HAVE_IEEE802154

/* IEEE 802.15.4 clear channel assessment control */

struct nrf52_radio_cca_s
{
  uint8_t mode;                 /* CCA mode of operation */
  uint8_t edthres;              /* CCA energy busy threshold */
  uint8_t corrthres;            /* CCA correlator busy threshold */
  uint8_t corrcnt;              /* Limit for occurances above CCACORRTHRES */
};
#endif

/* NRF52 radio operations */

struct nrf52_radio_dev_s;
struct nrf52_radio_ops_s
{
  /* Reset radio */

  void (*reset)(struct nrf52_radio_dev_s *dev);

  /* Put register value */

  void (*putreg)(struct nrf52_radio_dev_s *dev, uint32_t offset,
                 uint32_t value);

  /* Get register value */

  uint32_t (*getreg)(struct nrf52_radio_dev_s *dev, uint32_t offset);

  /* Enable interrupts */

  void (*inten)(struct nrf52_radio_dev_s *dev, uint32_t irq);

  /* Disable interrupts */

  void (*intclr)(struct nrf52_radio_dev_s *dev, uint32_t irq);

  /* Configure shorts */

  void (*shorts)(struct nrf52_radio_dev_s *dev, uint32_t irq);

  /* Turn-on/turn-off radio power */

  int (*power)(struct nrf52_radio_dev_s *dev, bool state);

  /* Set radio mode */

  int (*mode_set)(struct nrf52_radio_dev_s *dev, uint8_t mode);

  /* Set radio frequency (in MHz) */

  int (*freq_set)(struct nrf52_radio_dev_s *dev, uint32_t freq);

  /* Get RSSI sample */

  int (*rssi_get)(struct nrf52_radio_dev_s *dev, int8_t *rssi);

  /* Set TX power */

  int (*txpower_set)(struct nrf52_radio_dev_s *dev, uint8_t txpower);

  /* Set hardware interframe spacing time */

  int (*tifs_set)(struct nrf52_radio_dev_s *dev, uint16_t us);

  /* Configure radio packet */

  int (*pkt_cfg)(struct nrf52_radio_dev_s *dev,
                 struct nrf52_radio_pktcfg_s *cfg);

  /* Configure packet CRC */

  int (*crc_cfg)(struct nrf52_radio_dev_s *dev,
                 struct nrf52_radio_crc_s *cfg);

  /* Configure data whitening */

  int (*white_set)(struct nrf52_radio_dev_s *dev, uint8_t init);

  /* Configure logical address */

  int (*addr_set)(struct nrf52_radio_dev_s *dev, uint8_t i,
                  struct nrf52_radio_addr_s *addr);

  /* Dump radio registers */

  void (*dumpregs)(struct nrf52_radio_dev_s *dev);

#ifdef CONFIG_NRF52_HAVE_IEEE802154
  /* IEEE 802.15.4 start of frame delimiter */

  void (*sfd_set)(struct nrf52_radio_dev_s *dev, uint8_t sfd);

  /* IEEE 802.15.4 energy detect level */

  void (*edcnt_set)(struct nrf52_radio_dev_s *dev, uint32_t edcnt);

  /* IEEE 802.15.4 clear channel assessment control */

  void (*cca_cfg)(struct nrf52_radio_dev_s *dev,
                  struct nrf52_radio_cca_s *cca);
#endif
};

/* NRF52 radio board specific data */

struct nrf52_radio_board_s
{
  /* TODO: PA/LNA interface */

  uint32_t reserved;
};

/* NRF52 radio device */

struct nrf52_radio_dev_s
{
  struct nrf52_radio_ops_s   *ops;       /* Radio operations */
  struct nrf52_radio_board_s *board;     /* Radio board-specific */
  uint32_t                    base;      /* Radio base */
  uint32_t                    irq;       /* Radio IRQ number */
  uint8_t                     mode;      /* Radio mode */
  struct nrf52_radio_pktcfg_s pktcfg;    /* Current packet */
  mutex_t                     lock;      /* Mutual exclusion mutex */
  uint16_t                    tifs;      /* Interframe spacing time */
  uint8_t                     txpower;   /* TX power */
  struct nrf52_radio_addr_s   addr[NRF52_RADIO_LOGICAL_ADDRESS_MAX];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radio_initialize
 *
 * Description:
 *   Initialize NRF52 radio device
 *
 ****************************************************************************/

struct nrf52_radio_dev_s *
nrf52_radio_initialize(int intf, struct nrf52_radio_board_s *board);

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_RADIO_H */
