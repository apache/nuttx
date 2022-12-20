/****************************************************************************
 * arch/xtensa/src/esp32/esp32_emac.c
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
#if defined(CONFIG_ESP32_EMAC)

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>
#include <sys/types.h>
#include <sys/param.h>
#include <errno.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/crc8.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/list.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/net.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "xtensa.h"
#include "xtensa_attr.h"

#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_emac.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define ETH_HEADER_LEN          (14)   /* Ethernet frame header length */
#define ETH_VLANTAG_LEN         (4)    /* Ethernet frame VLAN TAG length */
#define ETH_MAXPAYLOAD_LEN      (1500) /* Ethernet frame payload length */
#define ETH_CRC_LEN             (4)    /* Ethernet frame CRC32 length */

/* Ethernet packet buffer maximum size */

#define ETH_MAXPKT_LEN          (ETH_HEADER_LEN + ETH_VLANTAG_LEN + \
                                 ETH_MAXPAYLOAD_LEN + ETH_CRC_LEN)

/* RX/TX buffer size */

#define EMAC_BUF_LEN            (ETH_MAXPKT_LEN)

/* RX/TX buffer number */

#define EMAC_RX_BUF_NUM         (CONFIG_ESP32_ETH_NRXDESC)
#define EMAC_TX_BUF_NUM         (CONFIG_ESP32_ETH_NTXDESC)

/* Total buffer number */

#define EMAC_BUF_NUM            (EMAC_RX_BUF_NUM + EMAC_TX_BUF_NUM + 1)

/* Read/Write/Reset PHY delays in loop counts, unit is 100 us */

#define EMAC_READPHY_TO         (10 * 10)
#define EMAC_WRITEPHY_TO        (10 * 10)
#define EMAC_RSTPHY_TO          (10 * 100)

/* Soft reset EMAC delays in loop counts, unit is 10 us */

#define EMAC_RESET_TO           (100 * 1000)

/* Wait ethernet linked in loop counts, unit is 10 us */

#define EMAC_WAITLINK_TO        (100 * 100)

/* Transmit ethernet frame timeout = 60 seconds = 1 minute */

#define EMAC_TX_TO              (60 * CLK_TCK)

/* Ethernet control frame pause timeout */

#define EMAC_PAUSE_TIME         (0x1648)

/* Ethernet stop frame PT-28 */

#define EMAC_PLT_TYPE           (1)

/* SMI clock value */

#define EMAC_SMI_CLK            (4)

/* DMA transmission number of beats */

#define EMAC_DMARXPBL_NUM       (32)
#define EMAC_DMATXPBL_NUM       (32)

/* RMII interface pins, each pin has fixed number  */

#define EMAC_ICLK_PIN           (0)
#define EMAC_TXEN_PIN           (21)
#define EMAC_TXDO_PIN           (19)
#define EMAC_TXD1_PIN           (22)
#define EMAC_RXDO_PIN           (25)
#define EMAC_RXD1_PIN           (26)
#define EMAC_RXDV_PIN           (27)

/* SMI interface pins */

#define EMAC_MDC_PIN            (CONFIG_ESP32_ETH_MDCPIN)
#define EMAC_MDIO_PIN           (CONFIG_ESP32_ETH_MDIOPIN)

/* Reset PHY chip pins */

#define EMAC_PHYRST_PIN         (CONFIG_ESP32_ETH_PHY_RSTPIN)

/* PHY chip address in SMI bus */

#define EMAC_PHY_ADDR           (CONFIG_ESP32_ETH_PHY_ADDR)

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define EMACWORK                LPWORK
#define ETHWORK                 LPWORK

/* Operation ****************************************************************/

/* Get smaller values */

#ifdef MIN
#  undef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/* Check if current TX description is busy */

#define TX_IS_BUSY(_priv)   \
  (((_priv)->txcur->ctrl & EMAC_TXDMA_OWN) != 0)

/* Get EMAC private data from net_driver_s */

#define NET2PRIV(_dev) ((struct esp32_emac_s *)(_dev)->d_private)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_emac_s
{
  /* This holds the information visible to the NuttX network */

  struct net_driver_s   dev;         /* Interface understood by the network */

  uint8_t               ifup    : 1; /* true:ifup false:ifdown */
  uint8_t               mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t               fduplex : 1; /* Full (vs. half) duplex */

  struct wdog_s         txtimeout;   /* TX timeout timer */

  struct work_s         txwork;      /* For deferring TX work to the work queue */
  struct work_s         rxwork;      /* For deferring RX work to the work queue */
  struct work_s         timeoutwork; /* For TX timeout work to the work queue */
  struct work_s         pollwork;    /* For deferring poll work to the work queue */

  int                   cpuint;      /* SPI interrupt ID */

  sq_queue_t            freeb;       /* The free buffer list */

  /* Hardware RX description allocations */

  struct emac_rxdesc_s  rxdesc[EMAC_RX_BUF_NUM];

  /* Hardware TX description allocations */

  struct emac_txdesc_s  txdesc[EMAC_TX_BUF_NUM];

  /* Current used RX description node */

  struct emac_rxdesc_s  *rxcur;

  /* Current used TX description node */

  struct emac_txdesc_s  *txcur;

  /* RX and TX buffer allocations */

  uint8_t alloc[EMAC_BUF_NUM * EMAC_BUF_LEN];

  /* Device specific lock. */

  spinlock_t lock;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32_emac_s s_esp32_emac;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int emac_ifdown(struct net_driver_s *dev);
static int emac_ifup(struct net_driver_s *dev);
static void emac_dopoll(struct esp32_emac_s *priv);
static void emac_txtimeout_expiry(wdparm_t arg);

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

extern uint8_t esp_crc8(const uint8_t *p, uint32_t len);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: emac_set_reg
 *
 * Description:
 *   Set the contents of the EMAC register at offset
 *
 * Input Parameters:
 *   offset - Offset to the register of interest
 *   value  - Value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void emac_set_reg(int offset, uint32_t value)
{
  putreg32(value, EMAC_REG_BASE + offset);
}

/****************************************************************************
 * Name: emac_get_reg
 *
 * Description:
 *   Get the contents of the EMAC register at offset
 *
 * Input Parameters:
 *   offset - Offset to the register of interest
 *
 * Returned Value:
 *   The contents of the register
 *
 ****************************************************************************/

static inline uint32_t emac_get_reg(int offset)
{
  return getreg32(EMAC_REG_BASE + offset);
}

/****************************************************************************
 * Name: emac_set_regbits
 *
 * Description:
 *   Set the bits of the EMAC register at offset
 *
 * Input Parameters:
 *   offset - Offset to the register of interest
 *   bits   - Bits to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void emac_set_regbits(int offset, uint32_t bits)
{
  uint32_t tmp = getreg32(EMAC_REG_BASE + offset);

  putreg32(tmp | bits, EMAC_REG_BASE + offset);
}

/****************************************************************************
 * Name: emac_reset_regbits
 *
 * Description:
 *   Clear the bits of the EMAC register at offset
 *
 * Input Parameters:
 *   offset - Offset to the register of interest
 *   bits   - Bits to be cleared
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void emac_reset_regbits(int offset, uint32_t bits)
{
  uint32_t tmp = getreg32(EMAC_REG_BASE + offset);

  putreg32(tmp & (~bits), EMAC_REG_BASE + offset);
}

/****************************************************************************
 * Function: emac_init_buffer
 *
 * Description:
 *   Initialize the free buffer list
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during early driver initialization before Ethernet interrupts
 *   are enabled
 *
 ****************************************************************************/

static void emac_init_buffer(struct esp32_emac_s *priv)
{
  uint8_t *buffer = priv->alloc;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0; i < EMAC_BUF_NUM; i++)
    {
      sq_addlast((sq_entry_t *)buffer, &priv->freeb);
      buffer += EMAC_BUF_LEN;
    }
}

/****************************************************************************
 * Function: emac_alloc_buffer
 *
 * Description:
 *   Allocate one buffer from the free buffer list
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer on success; NULL on failure
 *
 * Assumptions:
 *   Because we free buffer in interrupt, so add enter/exit critical here
 *
 ****************************************************************************/

static inline uint8_t *emac_alloc_buffer(struct esp32_emac_s *priv)
{
  uint8_t *p;
  irqstate_t flags;

  /* Allocate a buffer by returning the head of the free buffer list */

  flags = spin_lock_irqsave(&priv->lock);

  p = (uint8_t *)sq_remfirst(&priv->freeb);

  spin_unlock_irqrestore(&priv->lock, flags);

  return p;
}

/****************************************************************************
 * Function: emac_free_buffer
 *
 * Description:
 *   Return a buffer to the free buffer list
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   buffer - A pointer to the buffer to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function is called in interrupt, so that this operation can
 *   speed up freeing the buffer which has been transmitted
 *
 ****************************************************************************/

static inline void emac_free_buffer(struct esp32_emac_s *priv,
                                    uint8_t *buffer)
{
  /* Free the buffer by adding it to the end of the free buffer list */

  sq_addlast((sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: emac_read_mac
 *
 * Description:
 *   Read MAC address from eFuse memory
 *
 * Input Parameters:
 *   mac - MAC address read buffer pointer
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

static int emac_read_mac(uint8_t *mac)
{
  uint32_t regval[2];
  uint8_t *data = (uint8_t *)regval;
  uint8_t crc;
  int i;

  /* The MAC address in register is from high byte to low byte */

  regval[0] = getreg32(MAC_ADDR0_REG);
  regval[1] = getreg32(MAC_ADDR1_REG);

  crc = data[6];
  for (i = 0; i < 6; i++)
    {
      mac[i] = data[5 - i];
    }

  if (crc != esp_crc8(mac, 6))
    {
      nerr("ERROR: Failed to check MAC address CRC\n");

      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: emac_init_gpio
 *
 * Description:
 *   Initialize ESP32 ethernet interface GPIO pin
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   NOne
 *
 ****************************************************************************/

static void emac_init_gpio(void)
{
  esp32_configgpio(EMAC_TXEN_PIN, OUTPUT_FUNCTION_6);
  esp32_configgpio(EMAC_TXDO_PIN, OUTPUT_FUNCTION_6);
  esp32_configgpio(EMAC_TXD1_PIN, OUTPUT_FUNCTION_6);

  esp32_configgpio(EMAC_RXDO_PIN, INPUT_FUNCTION_6);
  esp32_configgpio(EMAC_RXD1_PIN, INPUT_FUNCTION_6);
  esp32_configgpio(EMAC_RXDV_PIN, INPUT_FUNCTION_6);

  esp32_configgpio(EMAC_ICLK_PIN, INPUT_FUNCTION_6);

  esp32_configgpio(EMAC_MDC_PIN, OUTPUT | FUNCTION_3);
  esp32_gpio_matrix_out(EMAC_MDC_PIN, EMAC_MDC_O_IDX, 0, 0);

  esp32_configgpio(EMAC_MDIO_PIN, OUTPUT | INPUT | FUNCTION_3);
  esp32_gpio_matrix_out(EMAC_MDIO_PIN, EMAC_MDO_O_IDX, 0, 0);
  esp32_gpio_matrix_in(EMAC_MDIO_PIN, EMAC_MDI_I_IDX, 0);

  esp32_configgpio(EMAC_PHYRST_PIN, OUTPUT | PULLUP);
}

/****************************************************************************
 * Name: emac_config
 *
 * Description:
 *   Configure ESP32 ethernet
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EINVAL is returned if MAC address is invalid
 *     -ETIMEDOUT is returned if reset ESP32 MAC timeout
 *
 ****************************************************************************/

static int emac_config(void)
{
  int i;
  uint32_t regval;
  uint8_t macaddr[6];

  /* Hardware reset PHY chip */

  esp32_gpiowrite(EMAC_PHYRST_PIN, false);
  nxsig_usleep(50);
  esp32_gpiowrite(EMAC_PHYRST_PIN, true);

  /* Open hardware clock */

  modifyreg32(DPORT_WIFI_CLK_EN_REG, 0, DPORT_EMAC_CLK_EN);
  modifyreg32(DPORT_WIFI_RST_EN_REG, DPORT_EMAC_RST_EN, 0);

  /* Configure interface to be RMII */

  emac_set_regbits(EMAC_PIR_OFFSET, EMAC_PIS_RMII);

  /* Use external XTAL clock for RMII */

  emac_set_regbits(EMAC_EOCCR_OFFSET, EMAC_OSEC_E);
  emac_set_regbits(EMAC_ECCR_OFFSET, EMAC_EXC_E);

  /* Configure SMI clock */

  emac_set_regbits(EMAC_MAR_OFFSET, (EMAC_SMI_CLK) << EMAC_SMICS_S);

  /* Reset EMAC */

  emac_set_regbits(EMAC_DMA_BMR_OFFSET, EMAC_SR_E);

  for (i = 0; i < EMAC_RESET_TO; i++)
    {
      if (!(emac_get_reg(EMAC_DMA_BMR_OFFSET) & EMAC_SR_E))
        {
          break;
        }

      nxsig_usleep(10);
    }

  if (i >= EMAC_RESET_TO)
    {
      nerr("ERROR: Failed to reset EMAC\n");

      return -ETIMEDOUT;
    }

  /**
   * Enable transmission options:
   *
   *   - 100M
   *   - Full duplex
   */

  regval = EMAC_FD_E | EMAC_100M_E | EMAC_SS_E | EMAC_RIPCOFFLOAD_E;
  emac_set_reg(EMAC_CR_OFFSET, regval);

  /* Pass all multicast frame */

  emac_set_reg(EMAC_FFR_OFFSET, EMAC_PMF_E);

  /**
   * Enable flow control options:
   *
   *   - PT-28 Time slot
   *   - RX flow control
   *   - TX flow control
   *   - Pause frame
   */

  regval = EMAC_RXFC_E | EMAC_TXFC_E | EMAC_FCBBA_E |
           (EMAC_PLT_TYPE << EMAC_PFPT_S) |
           (EMAC_PAUSE_TIME << EMAC_CFPT_S);
  emac_set_reg(EMAC_FCR_OFFSET, regval);

  /**
   * Enable DMA options:
   *
   *   - Drop error frame
   *   - Send frame when filled into FiFO
   *   - Automatically Send next frame
   */

  regval = EMAC_FSF_E | EMAC_FTF_E | EMAC_OSF_E;
  emac_set_reg(EMAC_DMA_OMR_OFFSET, regval);

  /**
   * Enable DMA bus options:
   *
   *   - Mixed burst mode
   *   - Address align beast
   *   - Separate PBL
   *   - Long DMA description
   */

  regval = EMAC_MB_E | EMAC_AAB_E | EMAC_SPBL_E | EMAC_ADS_E |
           (EMAC_DMARXPBL_NUM << EMAC_RXDMA_PBL_S) |
           (EMAC_DMATXPBL_NUM << EMAC_PBL_S);
  emac_set_reg(EMAC_DMA_BMR_OFFSET, regval);

  if (emac_read_mac(macaddr) || (macaddr[0] & 0x01))
    {
      nerr("ERROR: Failed read MAC address\n");

      return -EINVAL;
    }

  /* Configure hardware MAC address */

  regval = (macaddr[4] << 0) | (macaddr[5] << 8);
  emac_set_reg(EMAC_MA0HR_OFFSET, regval);

  regval = (macaddr[0] <<  0) | (macaddr[1] <<  8) |
           (macaddr[2] << 16) | (macaddr[3] << 24);
  emac_set_reg(EMAC_MA0LR_OFFSET, regval);

  return 0;
}

/****************************************************************************
 * Name: emac_start
 *
 * Description:
 *   Enable ESP32 EMAC transmission
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   NOne
 *
 ****************************************************************************/

static void emac_start(void)
{
  uint32_t val = UINT32_MAX;

  /* Clear and enable all interrupt */

  emac_set_reg(EMAC_DMA_SR_OFFSET, val);
  emac_set_reg(EMAC_DMA_IER_OFFSET, val);

  emac_set_regbits(EMAC_DMA_OMR_OFFSET, EMAC_SST_E | EMAC_SSR_E);

  emac_set_regbits(EMAC_CR_OFFSET, EMAC_TX_E | EMAC_RX_E);
}

/****************************************************************************
 * Name: emac_init_dma
 *
 * Description:
 *   Initialize DMA of EMAC
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void emac_init_dma(struct esp32_emac_s *priv)
{
  int i;
  struct emac_rxdesc_s *rxdesc = priv->rxdesc;
  struct emac_txdesc_s *txdesc = priv->txdesc;

  emac_init_buffer(priv);

  for (i = 0 ; i < EMAC_RX_BUF_NUM; i++)
    {
      rxdesc[i].status = EMAC_RXDMA_OWN;
      rxdesc[i].ctrl = EMAC_BUF_LEN | EMAC_RXDMA_RCH;

      /* Allocate buffer to prepare for receiving packets */

      rxdesc[i].pbuf = emac_alloc_buffer(priv);
      DEBUGASSERT(rxdesc[i].pbuf);
      rxdesc[i].next = &rxdesc[i + 1];
    }

  rxdesc[i - 1].next = rxdesc;
  priv->rxcur = &rxdesc[0];

  for (i = 0 ; i < EMAC_TX_BUF_NUM; i++)
    {
      txdesc[i].pbuf = NULL;
      txdesc[i].next = &txdesc[i + 1];
    }

  txdesc[i - 1].next = txdesc;
  priv->txcur  = &txdesc[0];

  emac_set_reg(EMAC_DMA_RDBR_OFFSET, (uint32_t)rxdesc);
  emac_set_reg(EMAC_DMA_TDBR_OFFSET, (uint32_t)txdesc);
}

/****************************************************************************
 * Name: emac_deinit_dma
 *
 * Description:
 *   Deinitialize DMA of EMAC by force to free RX & TX buffer
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void emac_deinit_dma(struct esp32_emac_s *priv)
{
  int i;
  struct emac_rxdesc_s *rxdesc = priv->rxdesc;
  struct emac_txdesc_s *txdesc = priv->txdesc;

  emac_init_buffer(priv);

  for (i = 0 ; i < EMAC_RX_BUF_NUM; i++)
    {
      if (rxdesc[i].pbuf)
        {
          emac_free_buffer(priv, rxdesc[i].pbuf);
          rxdesc[i].pbuf = NULL;
        }
    }

  for (i = 0 ; i < EMAC_TX_BUF_NUM; i++)
    {
      if (txdesc[i].pbuf)
        {
          emac_free_buffer(priv, txdesc[i].pbuf);
          txdesc[i].pbuf = NULL;
        }
    }
}

/****************************************************************************
 * Function: emac_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EBUSY is returned if no TX descrption is valid.
 *
 ****************************************************************************/

static int emac_transmit(struct esp32_emac_s *priv)
{
  int ret;
  struct emac_txdesc_s *txcur = priv->txcur;

  if (txcur->ctrl & EMAC_TXDMA_OWN)
    {
      return -EBUSY;
    }

  if (txcur->pbuf)
    {
      emac_free_buffer(priv, txcur->pbuf);
    }

  txcur->pbuf = priv->dev.d_buf;
  txcur->ctrl = EMAC_TXDMA_OWN | EMAC_TXDMA_FS | EMAC_TXDMA_LS |
                EMAC_TXDMA_CI | EMAC_TXDMA_TTSS | EMAC_TXDMA_TCH;
  txcur->ext_ctrl = priv->dev.d_len;

  priv->txcur = txcur->next;

  emac_set_reg(EMAC_DMA_STR_OFFSET, 0);

  ninfo("d_buf=%p d_len=%d\n", priv->dev.d_buf, priv->dev.d_len);

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  ret = wd_start(&priv->txtimeout, EMAC_TX_TO,
                 emac_txtimeout_expiry, (wdparm_t)priv);
  if (ret)
    {
      nerr("ERROR: Failed to start TX timeout timer");
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Function: emac_recvframe
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int emac_recvframe(struct esp32_emac_s *priv)
{
  uint32_t len;
  struct emac_rxdesc_s *rxcur = priv->rxcur;

  if (rxcur->status & EMAC_RXDMA_OWN)
    {
      return -EBUSY;
    }

  if (!rxcur->pbuf)
    {
      return -EINVAL;
    }

  len = (rxcur->status >> EMAC_RXDMA_FL_S) & EMAC_RXDMA_FL_V;
  priv->dev.d_buf = (uint8_t *)rxcur->pbuf;
  priv->dev.d_len = len - ETH_CRC_LEN;

  rxcur->pbuf = emac_alloc_buffer(priv);
  DEBUGASSERT(rxcur->pbuf);
  rxcur->status = EMAC_RXDMA_OWN;
  rxcur->ctrl = EMAC_BUF_LEN | EMAC_RXDMA_RCH;

  priv->rxcur = rxcur->next;

  emac_set_reg(EMAC_DMA_SRR_OFFSET, 0);

  ninfo("RX bytes %d\n", priv->dev.d_len);

  return 0;
}

/****************************************************************************
 * Name: emac_read_phy
 *
 * Description:
 *   Read PHY chip register value
 *
 * Input Parameters:
 *   dev_addr - PHY chip address
 *   reg_addr - register address
 *   pdata    - buffer pointer of register value
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EBUSY is returned if device is busy
 *     -ETIMEDOUT is returned if read timeout
 *
 ****************************************************************************/

static int emac_read_phy(uint16_t dev_addr,
                         uint16_t reg_addr,
                         uint16_t *pdata)
{
  uint16_t val;
  int i;

  val = emac_get_reg(EMAC_MAR_OFFSET);
  if (val & EMAC_PIB)
    {
      return -EBUSY;
    }

  val = ((dev_addr & EMAC_PCA_V) << EMAC_PCA_S) |
        ((reg_addr & EMAC_PCRA_V) << EMAC_PCRA_S) |
         EMAC_PIB;

  emac_set_reg(EMAC_MAR_OFFSET, val);

  for (i = 0; i < EMAC_READPHY_TO; i++)
    {
      nxsig_usleep(100);

      val = emac_get_reg(EMAC_MAR_OFFSET);
      if (!(val & EMAC_PIB))
        {
          break;
        }
    }

  if (i >= EMAC_READPHY_TO)
    {
      return -ETIMEDOUT;
    }

  *pdata = emac_get_reg(EMAC_MDR_OFFSET);

  return 0;
}

/****************************************************************************
 * Name: emac_write_phy
 *
 * Description:
 *   Write value to PHY chip register
 *
 * Input Parameters:
 *   dev_addr - PHY chip address
 *   reg_addr - register address
 *   data     - register value
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EBUSY is returned if device is busy
 *     -ETIMEDOUT is returned if write timeout
 *
 ****************************************************************************/

static int emac_write_phy(uint16_t dev_addr,
                          uint16_t reg_addr,
                          uint16_t data)
{
  uint16_t val;
  int i;

  val = emac_get_reg(EMAC_MAR_OFFSET);
  if (val & EMAC_PIB)
    {
      return -EBUSY;
    }

  emac_set_reg(EMAC_MDR_OFFSET, data);

  val = ((dev_addr & EMAC_PCA_V) << EMAC_PCA_S) |
        ((reg_addr & EMAC_PCRA_V) << EMAC_PCRA_S) |
         EMAC_PIB;

  emac_set_reg(EMAC_MAR_OFFSET, val);

  for (i = 0; i < EMAC_WRITEPHY_TO; i++)
    {
      nxsig_usleep(100);

      val = emac_get_reg(EMAC_MAR_OFFSET);
      if (!(val & EMAC_PIB))
        {
          break;
        }
    }

  if (i >= EMAC_WRITEPHY_TO)
    {
      return -ETIMEDOUT;
    }

  return 0;
}

/****************************************************************************
 * Name: emac_wait_linkup
 *
 * Description:
 *   Wait for ethernet link up or timeout
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EBUSY is returned if device is busy
 *     -ETIMEDOUT is returned if write timeout
 *
 ****************************************************************************/

static int emac_wait_linkup(struct esp32_emac_s *priv)
{
  int ret;
  int i;
  uint16_t val;

  for (i = 0; i < EMAC_WAITLINK_TO; i++)
    {
      nxsig_usleep(10);

      ret = emac_read_phy(EMAC_PHY_ADDR, MII_MSR, &val);
      if (ret != 0)
        {
          priv->ifup = false;
          nerr("ERROR: Failed to read PHY Register 0x%x: %d\n",
               MII_MSR, ret);
          return ret;
        }

      if (val & MII_MSR_LINKSTATUS)
        {
          break;
        }
    }

  ninfo("PHY register 0x%x is: 0x%04x\n", MII_MSR, val);

  if (i >= EMAC_WAITLINK_TO)
    {
      nerr("ERROR: Timeout to wait for PHY LINK UP");
      priv->ifup = false;
      return -ETIMEDOUT;
    }
  else
    {
      priv->ifup = true;
    }

  if (val & MII_MSR_100BASETXFULL)
    {
      priv->mbps100 = true;
      priv->fduplex = true;
    }
  else if (val & MII_MSR_100BASETXHALF)
    {
      priv->mbps100 = true;
      priv->fduplex = false;
    }
  else if (val & MII_MSR_10BASETXFULL)
    {
      priv->mbps100 = false;
      priv->fduplex = true;
    }
  else if (val & MII_MSR_10BASETXHALF)
    {
      priv->mbps100 = false;
      priv->fduplex = false;
    }
  else if (val & MII_MSR_100BASET2HALF)
    {
      priv->mbps100 = true;
      priv->fduplex = false;
    }
  else if (val & MII_MSR_100BASET2FULL)
    {
      priv->mbps100 = true;
      priv->fduplex = true;
    }

  return 0;
}

/****************************************************************************
 * Name: emac_init_phy
 *
 * Description:
 *   Brings up and initializes PHY chip
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EBUSY is returned if phy is busy
 *     -ETIMEDOUT is returned if read/write phy register timeout
 *     -EPERM is returned if operation fails
 *
 ****************************************************************************/

static int emac_init_phy(struct esp32_emac_s *priv)
{
  int ret;
  int i;
  uint16_t val;

  /* power on PHY chip */

  ret = emac_read_phy(EMAC_PHY_ADDR, MII_MCR, &val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to read PHY Register 0x%x: %d\n",
           MII_MCR, ret);
      return ret;
    }

  ninfo("PHY register 0x%x is: 0x%04x\n", MII_MCR, val);

  val &= ~(MII_MCR_PDOWN);

  ret = emac_write_phy(EMAC_PHY_ADDR, MII_MCR, val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to write PHY Register 0x%x to be 0x%x: %d\n",
           MII_MCR, val, ret);
      return ret;
    }

  ret = emac_read_phy(EMAC_PHY_ADDR, MII_MCR, &val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to read PHY Register 0x%x: %d\n",
           MII_MCR, ret);
      return ret;
    }

  if (val & MII_MCR_PDOWN)
    {
      nerr("ERROR: Failed to power on PHY\n");
      return -EPERM;
    }

  val |= MII_MCR_RESET;

  ret = emac_write_phy(EMAC_PHY_ADDR, MII_MCR, val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to write PHY Register 0x%x to be 0x%x: %d\n",
           MII_MCR, val, ret);
      return ret;
    }

  for (i = 0; i < EMAC_RSTPHY_TO; i++)
    {
      nxsig_usleep(100);

      ret = emac_read_phy(EMAC_PHY_ADDR, MII_MCR, &val);
      if (ret != 0)
        {
          nerr("ERROR: Failed to read PHY Register 0x%x: %d\n",
               MII_MCR, ret);
          return ret;
        }

      if (!(val & MII_MCR_RESET))
        {
          break;
        }
    }

  if (i >= EMAC_RSTPHY_TO)
    {
      return -ETIMEDOUT;
    }

  ret = emac_read_phy(EMAC_PHY_ADDR, MII_PHYID1, &val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to read PHY Register 0x%x: %d\n",
           MII_PHYID1, ret);
      return ret;
    }

  ninfo("PHY register 0x%x is: 0x%04x\n", MII_PHYID1, val);

  ret = emac_read_phy(EMAC_PHY_ADDR, MII_PHYID2, &val);
  if (ret != 0)
    {
      nerr("ERROR: Failed to read PHY Register 0x%x: %d\n",
           MII_PHYID2, ret);
      return ret;
    }

  ninfo("PHY register 0x%x is: 0x%04x\n", MII_PHYID2, val);

  ret = emac_wait_linkup(priv);
  if (ret != 0)
    {
      nerr("ERROR: Failed to wait LINK UP error=%d\n", ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Function: phy_enable_interrupt
 *
 * Description:
 *  Enable link up/down PHY interrupts.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EBUSY is returned if IP101 is busy.
 *     -ETIMEDOUT is returned if read/write IP101 register timeout.
 *
 ****************************************************************************/

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int phy_enable_interrupt(void)
{
  uint16_t phyval;
  int ret;

  ret = emac_read_phy(EMAC_PHY_ADDR, MII_INT_REG, &regval);
  if (ret == OK)
    {
      /* Enable link up/down interrupts */

      ret = emac_write_phy(EMAC_PHY_ADDR, MII_INT_REG,
                           (regval & ~MII_INT_CLREN) | MII_INT_SETEN);
    }

  return ret;
}
#endif

/****************************************************************************
 * Function: emac_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   0 on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void emac_txtimeout_work(void *arg)
{
  struct esp32_emac_s *priv = (struct esp32_emac_s *)arg;

  /* Reset the hardware.  Just take the interface down, then back up again. */

  net_lock();
  emac_ifdown(&priv->dev);
  emac_ifup(&priv->dev);

  /* Then poll for new XMIT data */

  emac_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: emac_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void emac_txtimeout_expiry(wdparm_t arg)
{
  struct esp32_emac_s *priv = (struct esp32_emac_s *)arg;

  nerr("ERROR: Timeout!\n");

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   *
   * Interrupts will be re-enabled when emac_ifup() is called.
   */

  up_disable_irq(ESP32_IRQ_EMAC);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * perhaps canceling any pending IRQ processing.
   */

  work_queue(ETHWORK, &priv->timeoutwork, emac_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: emac_rx_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   0 on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void emac_rx_interrupt_work(void *arg)
{
  struct esp32_emac_s *priv = (struct esp32_emac_s *)arg;
  struct net_driver_s *dev = &priv->dev;

  net_lock();

  /* Loop while while emac_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  while (emac_recvframe(priv) == 0)
    {
      struct eth_hdr_s *eth_hdr = (struct eth_hdr_s *)dev->d_buf;

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap
       */

     pkt_input(&priv->dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets
       */

#ifdef CONFIG_NET_IPv4
      if (eth_hdr->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field d_len will set to a value > 0
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              emac_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (eth_hdr->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field d_len will set to a value > 0
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              emac_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (eth_hdr->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Handle ARP packet */

          arp_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field d_len will set to a value > 0
           */

          if (priv->dev.d_len > 0)
            {
              emac_transmit(priv);
            }
        }
      else
#endif
        {
          nerr("ERROR: Dropped, Unknown type: %04x\n", eth_hdr->type);
        }

      if (dev->d_buf)
        {
          emac_free_buffer(priv, dev->d_buf);

          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: emac_tx_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   0 on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void emac_tx_interrupt_work(void *arg)
{
  struct esp32_emac_s *priv = (struct esp32_emac_s *)arg;

  net_lock();

  wd_cancel(&priv->txtimeout);

  emac_dopoll(priv);

  net_unlock();
}

/****************************************************************************
 * Function: emac_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   0 on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int emac_interrupt(int irq, void *context, void *arg)
{
  struct esp32_emac_s *priv = (struct esp32_emac_s *)arg;
  uint32_t value = emac_get_reg(EMAC_DMA_SR_OFFSET);

  emac_set_reg(EMAC_DMA_SR_OFFSET, 0xffffffff);

  /* If the netcard is disabled then exit directly */

  if (!priv->ifup)
    {
      return 0;
    }

  if (value & EMAC_RI)
    {
      work_queue(EMACWORK, &priv->rxwork, emac_rx_interrupt_work, priv, 0);
    }

  if (value & EMAC_TI)
    {
      work_queue(EMACWORK, &priv->txwork, emac_tx_interrupt_work, priv, 0);
    }

  return 0;
}

/****************************************************************************
 * Function: emac_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int emac_txpoll(struct net_driver_s *dev)
{
  struct esp32_emac_s *priv = NET2PRIV(dev);

  DEBUGASSERT(priv->dev.d_buf != NULL);

  /* Send the packet */

  emac_transmit(priv);
  DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

  /* Check if the current TX descriptor is owned by the Ethernet DMA
   * or CPU. We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   */

  if (TX_IS_BUSY(priv))
    {
      /* We have to terminate the poll if we have no more descriptors
       * available for another transfer.
       */

      return -EBUSY;
    }

  dev->d_buf = (uint8_t *)emac_alloc_buffer(priv);
  if (dev->d_buf == NULL)
    {
      return -ENOMEM;
    }

  dev->d_len = EMAC_BUF_LEN;

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: emac_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (esp32_txdone),
 *   2. When new TX data is available (emac_txavail_work), and
 *   3. After a TX timeout to restart the sending process
 *      (esp32_txtimeout_process).
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void emac_dopoll(struct esp32_emac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  if (!TX_IS_BUSY(priv))
    {
      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

      dev->d_buf = (uint8_t *)emac_alloc_buffer(priv);
      if (!dev->d_buf)
        {
          /* never reach */

          return ;
        }

      dev->d_len = EMAC_BUF_LEN;

      devif_poll(dev, emac_txpoll);

      if (dev->d_buf)
        {
          emac_free_buffer(priv, dev->d_buf);

          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Function: emac_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg  - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void emac_txavail_work(void *arg)
{
  struct esp32_emac_s *priv = (struct esp32_emac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      emac_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: emac_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EINVAL is returned if MAC address is invalid.
 *     -ETIMEDOUT is returned if reset ESP32 MAC timeout.
 *
 ****************************************************************************/

static int emac_ifup(struct net_driver_s *dev)
{
  int ret;
  irqstate_t flags;
  struct esp32_emac_s *priv = NET2PRIV(dev);

  flags = enter_critical_section();

  /* initialize ESP32 ethernet GPIO */

  emac_init_gpio();

  /* configure ESP32 ethernet MAC */

  ret = emac_config();
  if (ret)
    {
      leave_critical_section(flags);
      nerr("ERROR: Failed to configure ESP32 MAC\n");

      return ret;
    }

  /* initialize IP101 phy chip */

  ret = emac_init_phy(priv);
  if (ret)
    {
      leave_critical_section(flags);
      nerr("ERROR: Failed to initialize IP101 phy chip\n");

      return ret;
    }

  /* initialize ESP32 ethernet MAC DMA */

  emac_init_dma(priv);

  /* start ESP32 ethernet MAC */

  emac_start();

  /* Enable the Ethernet interrupt */

  up_enable_irq(ESP32_IRQ_EMAC);

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Function: emac_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int emac_ifdown(struct net_driver_s *dev)
{
  struct esp32_emac_s *priv = NET2PRIV(dev);
  irqstate_t flags;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();

  /* Disable TX and RX */

  emac_reset_regbits(EMAC_CR_OFFSET, EMAC_TX_E | EMAC_RX_E);

  up_disable_irq(priv->cpuint);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Reset ethernet MAC and disable clock */

  modifyreg32(DPORT_WIFI_RST_EN_REG, 0, DPORT_EMAC_RST_EN);
  modifyreg32(DPORT_WIFI_CLK_EN_REG, DPORT_EMAC_CLK_EN, 0);

  /* Free DMA resource */

  emac_deinit_dma(priv);

  /* Mark the device "down" */

  priv->ifup = false;

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Function: emac_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int emac_txavail(struct net_driver_s *dev)
{
  struct esp32_emac_s *priv = NET2PRIV(dev);

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(EMACWORK, &priv->pollwork, emac_txavail_work, priv, 0);
    }

  return 0;
}

/****************************************************************************
 * Function: emac_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Note:
 *   The default option can allow EMAC receive all multicast frame.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int emac_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  return 0;
}
#endif /* CONFIG_NET_MCASTGROUP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: emac_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Note:
 *   The default option can allow EMAC receive all multicast frame.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int emac_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  return 0;
}
#endif

/****************************************************************************
 * Function: emac_ioctl
 *
 * Description:
 *  Executes the SIOCxMIIxxx command and responds using the request struct
 *  that must be provided as its 2nd parameter.
 *
 *  When called with SIOCGMIIPHY it will get the PHY address for the device
 *  and write it to the req->phy_id field of the request struct.
 *
 *  When called with SIOCGMIIREG it will read a register of the PHY that is
 *  specified using the req->reg_no struct field and then write its output
 *  to the req->val_out field.
 *
 *  When called with SIOCSMIIREG it will write to a register of the PHY that
 *  is specified using the req->reg_no struct field and use req->val_in as
 *  its input.
 *
 * Input Parameters:
 *   dev - Ethernet device structure
 *   cmd - SIOCxMIIxxx command code
 *   arg - Request structure also used to return values
 *
 * Returned Value:
 *   0 on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int emac_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
  struct esp32_emacmac_s *priv = NET2PRIV(dev);
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          struct mii_ioctl_notify_s *req =
                    (struct mii_ioctl_notify_s *)((uintptr_t)arg);

          ret = phy_notify_subscribe(dev->d_ifname, req->pid, &req->event);
          if (ret == 0)
            {
              /* Enable PHY link up/down interrupts */

              ret = phy_enable_interrupt();
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
                    (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = EMAC_PHY_ADDR;
          ret = 0;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
                    (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = emac_read_phy(req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
                    (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = emac_write_phy(req->phy_id, req->reg_num, req->val_in);
        }
        break;
#endif /* CONFIG_NETDEV_PHY_IOCTL */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
#else
  return -EINVAL;
#endif /* CONFIG_NETDEV_IOCTL */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_emac_init
 *
 * Description:
 *   Initialize ESP32 ethernet device driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -ENOMEM is returned if no memory resource.
 *
 ****************************************************************************/

int esp32_emac_init(void)
{
  struct esp32_emac_s *priv = &s_esp32_emac;
  int ret;

  memset(priv, 0, sizeof(struct esp32_emac_s));

  priv->cpuint = esp32_setup_irq(0, ESP32_PERIPH_EMAC,
                                 1, ESP32_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      nerr("ERROR: Failed alloc interrupt\n");

      ret = -ENOMEM;
      goto error;
    }

  ret = irq_attach(ESP32_IRQ_EMAC, emac_interrupt, priv);
  if (ret != 0)
    {
      nerr("ERROR: Failed attach interrupt\n");

      ret = -ENOMEM;
      goto errout_with_attachirq;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = emac_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = emac_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = emac_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = emac_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = emac_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = emac_ioctl;    /* Support PHY ioctl() calls */
#endif

  /* Used to recover private state from dev */

  priv->dev.d_private = priv;

  emac_read_mac(priv->dev.d_mac.ether.ether_addr_octet);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
  if (ret != 0)
    {
      nerr("ERROR: Failed to register net device\n");

      goto errout_with_attachirq;
    }

  return 0;

errout_with_attachirq:
  esp32_teardown_irq(0, ESP32_PERIPH_EMAC, priv->cpuint);

error:
  return ret;
}

/****************************************************************************
 * Function: xtensa_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in up_initialize.c.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void xtensa_netinitialize(void)
{
  esp32_emac_init();
}
#endif

#endif /* CONFIG_ESP32_EMAC */
