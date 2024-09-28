/****************************************************************************
 * drivers/net/lan9250.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/lan9250.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#ifdef CONFIG_LAN9250_SPI
#  include <nuttx/spi/spi.h>
#else
#  include <nuttx/spi/qspi.h>
#endif

#include "lan9250.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LAN9250 SPI clock */

#define LAN9250_FREQUENCY           CONFIG_LAN9250_FREQUENCY

/* LAN9250 read and write command */

#ifdef CONFIG_LAN9250_SPI
#  if LAN9250_FREQUENCY > (30 * 1000 * 1000)
#    define LAN9250_SPI_READ        LAN9250_SPI_CMD_FREAD
#  else
#    define LAN9250_SPI_READ        LAN9250_SPI_CMD_READ
#  endif

#  define LAN9250_SPI_WRITE         LAN9250_SPI_CMD_WRITE
#else
#  define LAN9250_SPI_READ          LAN9250_QSPI_CMD_READ
#  define LAN9250_SPI_WRITE         LAN9250_QSPI_CMD_WRITE

#  define LAN9250_SPI_ENABLE_SQI    LAN9250_SPI_CMD_ENABLE_SQI
#endif

/* LAN9250 interrupt trigger source:
 *
 *   - PHY for checking link up or down
 *   - TX data FIFO available
 *   - RX data FIFO reaches programmed level
 */

#define LAN9250_INT_SOURCE          (IER_PHY | IER_TDFA | IER_RSFL)

/* The low-priority work queue is preferred. If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE: However, the network should NEVER run on the high priority work
 * queue! That queue is intended only to service short back end interrupt
 * processing that never suspends. Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define LAN9250_WORK                LPWORK

/* Timing *******************************************************************/

/* Reset timeout in second */

#define LAN9250_RESET_TIMEOUT       (5)

/* TX timeout = 1 minute */

#define LAN9250_TX_TIMEOUT          (60 * CLK_TCK)

/* Poll timeout */

#define LAN9250_POLL_TIMEOUT        MSEC2TICK(50)

/* Read/Write MAC register timeout in second */

#define LAN9250_MAC_TIMEOUT         2

/* Read/Write PHY register timeout in second */

#define LAN9250_PHY_TIMEOUT         2

/* Read/Write PHY register timeout in millisecond */

#define LAN9250_SQI_TIMEOUT         20

/* Packet Memory ************************************************************/

/* Misc. Helper Macros ******************************************************/

#define LAN9250_ALIGN(v)            (((v) + 3) & (~3))

/* Packet buffer size with 4-CRC */

#define LAN9250_PKTBUF_SIZE         LAN9250_ALIGN(MAX_NETDEV_PKTSIZE +   \
                                                  CONFIG_NET_GUARDSIZE + \
                                                  4)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF                         ((struct eth_hdr_s *)priv->dev.d_buf)

/* Debug ********************************************************************/

#ifdef CONFIG_LAN9250_REGDEBUG
#  define lan9250_setreg_dump(a, v)    \
   syslog(LOG_DEBUG, "LAN9250 REG: 0x%04x<-0x%08x\n", a, v)
#  define lan9250_getreg_dump(a, v)    \
   syslog(LOG_DEBUG, "LAN9250 REG: 0x%04x->0x%08x\n", a, v)
#  define lan9250_setmacreg_dump(a, v) \
   syslog(LOG_DEBUG, "LAN9250 MAC: 0x%02x<-0x%08x\n", a, v)
#  define lan9250_getmacreg_dump(a, v) \
   syslog(LOG_DEBUG, "LAN9250 MAC: 0x%02x->0x%08x\n", a, v)
#  define lan9250_setphyreg_dump(a, v) \
   syslog(LOG_DEBUG, "LAN9250 PHY: 0x%02x<-0x%08x\n", a, v)
#  define lan9250_getphyreg_dump(a, v) \
   syslog(LOG_DEBUG, "LAN9250 PHY: 0x%02x->0x%08x\n", a, v)
#  define lan9250_buffer_dump(c, b, s) \
   syslog(LOG_DEBUG, "LAN9250 BUF: cmd: %04x buffer: %p length: %d\n", c, b, s)
#else
#  define lan9250_setreg_dump(a, v)
#  define lan9250_getreg_dump(a, v)
#  define lan9250_setmacreg_dump(a, v)
#  define lan9250_getmacreg_dump(a, v)
#  define lan9250_setphyreg_dump(a, v)
#  define lan9250_getphyreg_dump(a, v)
#  define lan9250_buffer_dump(c, b, s)
#endif

/* CONFIG_LAN9250_DUMPPACKET will dump the contents of each packet. */

#ifdef CONFIG_LAN9250_DUMPPACKET
#  define lan9250_dump_buf(m, a, n) lib_dumpbuffer(m, a, n)
#else
#  define lan9250_dump_buf(m, a, n)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lan9250_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct lan9250_driver_s
{
  /* Low-level MCU-specific support */

  const struct lan9250_lower_s *lower;

  /* This is the contained (Q)SPI driver instance */

#ifdef CONFIG_LAN9250_SPI
  struct spi_dev_s    *spi;
#else
  bool                 sqi_mode;
  struct qspi_dev_s   *qspi;
#endif

  bool                tx_available;   /* TX is available */

  /* Read/Write buffer for SPI transmission */

  uint32_t            pktbuf[LAN9250_PKTBUF_SIZE / 4];

  struct wdog_s       txtout_timer;   /* TX timeout timer */
  struct work_s       txtout_work;    /* Tx timeout work */
  struct work_s       txpoll_work;    /* TX poll work */
  struct work_s       irq_work;       /* Interrupt work */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;            /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Driver status structure */

static struct lan9250_driver_s g_lan9250;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpers */

static inline void lan9250_config_spi(FAR struct lan9250_driver_s *priv);
static void lan9250_lock_spi(FAR struct lan9250_driver_s *priv);
static inline void lan9250_unlock_spi(FAR struct lan9250_driver_s *priv);

/* SPI control register access */

static uint32_t lan9250_get_reg(FAR struct lan9250_driver_s *priv,
                                uint16_t address);
static void lan9250_set_reg(FAR struct lan9250_driver_s *priv,
                            uint16_t address, uint32_t data);
static void lan9250_wait_ready(FAR struct lan9250_driver_s *priv,
                               uint16_t address, uint32_t mask,
                               uint32_t expected, uint32_t timeout);

/* MAC register access */

static uint32_t lan9250_get_macreg(FAR struct lan9250_driver_s *priv,
                                   uint8_t address);
static void lan9250_set_macreg(FAR struct lan9250_driver_s *priv,
                               uint8_t address, uint32_t data);
static void lan9250_wait_mac_ready(FAR struct lan9250_driver_s *priv,
                                   uint8_t address, uint32_t mask,
                                   uint32_t expected, uint32_t timeout);

/* PHY register access */

static uint16_t lan9250_get_phyreg(FAR struct lan9250_driver_s *priv,
                                   uint8_t phyaddr);
static void lan9250_set_phyreg(FAR struct lan9250_driver_s *priv,
                               uint8_t phyaddr, uint16_t phydata);

/* SPI buffer transfers */

static void lan9250_recv_buffer(FAR struct lan9250_driver_s *priv,
                                FAR uint8_t *buffer, size_t buflen);
static inline void lan9250_send_buffer(FAR struct lan9250_driver_s *priv,
                                       FAR const uint8_t *buffer,
                                       size_t buflen);

/* Misc handling */

#ifdef CONFIG_LAN9250_SQI
static void lan9250_enable_sqi(FAR struct lan9250_driver_s *priv);
#endif
static inline void lan9250_sw_reset(FAR struct lan9250_driver_s *priv);
static void lan9250_set_txavailabe(FAR struct lan9250_driver_s *priv,
                                   bool enable);
static int lan9250_reset(FAR struct lan9250_driver_s *priv);
static void lan9250_set_macaddr(FAR struct lan9250_driver_s *priv);

/* Common TX logic */

static int lan9250_transmit(FAR struct lan9250_driver_s *priv);
static int lan9250_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void lan9250_netdev_rx(FAR struct lan9250_driver_s *priv);
static void lan9250_phy_isr(FAR struct lan9250_driver_s *priv);
static void lan9250_txavailable_isr(FAR struct lan9250_driver_s *priv);
static void lan9250_rxdone_isr(FAR struct lan9250_driver_s *priv);
static int  lan9250_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void lan9250_txtout_timercb(wdparm_t arg);

/* Driver worker */

static void lan9250_txavail_work(FAR void *arg);
static void lan9250_txtout_worker(FAR void *arg);
static void lan9250_int_worker(FAR void *arg);

/* NuttX callback functions */

static int lan9250_ifup(FAR struct net_driver_s *dev);
static int lan9250_ifdown(FAR struct net_driver_s *dev);
static int lan9250_txavail(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int lan9250_addmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac);
static int lan9250_rmmac(FAR struct net_driver_s *dev,
                         FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int lan9250_ioctl(FAR struct net_driver_s *dev, int cmd,
                         unsigned long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lan9250_config_spi
 *
 * Description:
 *   Configure the SPI for use with the LAN9250
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lan9250_config_spi(FAR struct lan9250_driver_s *priv)
{
#ifdef CONFIG_LAN9250_SPI
  FAR struct spi_dev_s *spi = priv->spi;

  /* Configure SPI for the LAN9250. */

  SPI_SETMODE(spi, CONFIG_LAN9250_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, LAN9250_FREQUENCY);
#else
  FAR struct qspi_dev_s *qspi = priv->qspi;

  /* Configure QSPI for the LAN9250. */

  QSPI_SETMODE(qspi, CONFIG_LAN9250_SPIMODE);
  QSPI_SETBITS(qspi, 8);
  QSPI_HWFEATURES(qspi, 0);
  QSPI_SETFREQUENCY(qspi, LAN9250_FREQUENCY);
#endif
}

/****************************************************************************
 * Name: lan9250_lock_spi
 *
 * Description:
 *   Select the SPI, locking it and re-configuring it if necessary.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_lock_spi(FAR struct lan9250_driver_s *priv)
{
#ifdef CONFIG_LAN9250_SPI

  /* Lock the SPI bus in case there are multiple devices or tasks competing
   * for the SPI bus.
   */

  SPI_LOCK(priv->spi, true);
#else

  /* Lock the QSPI bus in case there are multiple devices or tasks competing
   * for the QSPI bus.
   */

  QSPI_LOCK(priv->qspi, true);
#endif

  /* Now make sure that the (Q)SPI bus is configured for the LAN9250 (it
   * might have gotten configured for a different device while unlocked)
   */

#ifndef CONFIG_LAN9250_SPI_EXCLUSIVE
  lan9250_config_spi(priv);
#endif
}

/****************************************************************************
 * Name: lan9250_unlock_spi
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lan9250_unlock_spi(FAR struct lan9250_driver_s *priv)
{
#ifdef CONFIG_LAN9250_SPI

  /* Relinquish the lock on the bus. */

  SPI_LOCK(priv->spi, false);
#else

  /* Relinquish the lock on the bus. */

  QSPI_LOCK(priv->qspi, false);
#endif
}

/****************************************************************************
 * Name: lan9250_get_reg
 *
 * Description:
 *   Read a global register value.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   address - Register address
 *
 * Returned Value:
 *   The value read from the register.
 *
 ****************************************************************************/

static uint32_t lan9250_get_reg(FAR struct lan9250_driver_s *priv,
                                uint16_t address)
{
#ifdef CONFIG_LAN9250_SPI
#  if LAN9250_SPI_READ == LAN9250_SPI_CMD_FREAD
  uint8_t cmd_buffer[4];
#  else
  uint8_t cmd_buffer[3];
#  endif
  uint32_t regval;

  cmd_buffer[0] = LAN9250_SPI_READ;
  cmd_buffer[1] = (uint8_t)(address >> 8);
  cmd_buffer[2] = (uint8_t)(address >> 0);
#  if LAN9250_SPI_READ == LAN9250_SPI_CMD_FREAD
  cmd_buffer[3] = 0xff; /* SPI read dummy */
#  endif

  /* Select LAN9250 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the read command and register address */

  SPI_SNDBLOCK(priv->spi, cmd_buffer, sizeof(cmd_buffer));

  /* Receive register value, total 4 bytes */

  SPI_RECVBLOCK(priv->spi, &regval, 4);

  /* De-select LAN9250 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
#else
  struct qspi_cmdinfo_s cmdinfo;
  uint32_t regval;
  uint32_t buffer[2];
  int ret;

  cmdinfo.cmd     = LAN9250_SPI_READ;
  cmdinfo.addr    = address;
  cmdinfo.addrlen = sizeof(address);
  cmdinfo.buffer  = buffer;
  cmdinfo.buflen  = sizeof(buffer); /* 4 dummy bytes */
  cmdinfo.flags   = QSPICMD_ADDRESS | QSPICMD_READDATA;
  if (priv->sqi_mode)
    {
      cmdinfo.flags |= QSPICMD_IQUAD;
    }

  ret = QSPI_COMMAND(priv->qspi, &cmdinfo);
  DEBUGASSERT(ret == 0);

  regval = buffer[1];
#endif

  lan9250_getreg_dump(address, regval);
  return regval;
}

/****************************************************************************
 * Name: lan9250_set_reg
 *
 * Description:
 *   Write to a global register.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   address - Register address
 *   data    - The data to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_set_reg(FAR struct lan9250_driver_s *priv,
                            uint16_t address,
                            uint32_t data)
{
#ifdef CONFIG_LAN9250_SPI
  uint8_t cmd_buffer[3];

  DEBUGASSERT(priv && priv->spi);

  cmd_buffer[0] = LAN9250_SPI_WRITE;
  cmd_buffer[1] = (uint8_t)(address >> 8);
  cmd_buffer[2] = (uint8_t)(address >> 0);

  /* Select LAN9250 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the read command and register address */

  SPI_SNDBLOCK(priv->spi, cmd_buffer, sizeof(cmd_buffer));

  /* Receive register value, total 4 bytes */

  SPI_SNDBLOCK(priv->spi, &data, 4);

  /* De-select LAN9250 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
#else
  struct qspi_cmdinfo_s cmdinfo;
  int ret;

  cmdinfo.cmd     = LAN9250_SPI_WRITE;
  cmdinfo.addr    = address;
  cmdinfo.addrlen = sizeof(address);
  cmdinfo.buffer  = &data;
  cmdinfo.buflen  = sizeof(data);
  cmdinfo.flags   = QSPICMD_ADDRESS | QSPICMD_WRITEDATA;
  if (priv->sqi_mode)
    {
      cmdinfo.flags |= QSPICMD_IQUAD;
    }

  ret = QSPI_COMMAND(priv->qspi, &cmdinfo);
  DEBUGASSERT(ret == 0);
#endif

  lan9250_setreg_dump(address, data);
}

/****************************************************************************
 * Name: lan9250_wait_ready
 *
 * Description:
 *   Wait until LAN9250 is ready.
 *
 * Input Parameters:
 *   priv     - Reference to the driver state structure
 *   address  - Register address
 *   mask     - Register value mask
 *   expected - Expected register value
 *   second   - Wait timeout in second
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_wait_ready(FAR struct lan9250_driver_s *priv,
                               uint16_t address, uint32_t mask,
                               uint32_t expected, uint32_t second)
{
  clock_t tout_ticks = clock_systime_ticks() + SEC2TICK(second);
  bool timeout = false;

  while (1)
    {
      if ((lan9250_get_reg(priv, address) & mask) == expected)
        {
          break;
        }
      else if (clock_systime_ticks() > tout_ticks)
        {
          timeout = true;
          break;
        }
    }

  if (timeout)
    {
      nerr("ERROR: wait register:0x%02x, mask:0x%08x, expected:0x%08x\n",
            address, mask, expected);
    }
}

/****************************************************************************
 * Name: lan9250_get_macreg
 *
 * Description:
 *   Read a MAC register value.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   address - MAC register address
 *
 * Returned Value:
 *   The value read from the MAC register.
 *
 ****************************************************************************/

static uint32_t lan9250_get_macreg(FAR struct lan9250_driver_s *priv,
                                   uint8_t address)
{
  uint32_t regval = address | HMCSRICR_CSRB | HMCSRICR_RNW;

  /* Wait for MAC to be ready and send reading register command */

  lan9250_wait_ready(priv, LAN9250_HMCSRICR, HMCSRICR_CSRB, 0,
                     LAN9250_MAC_TIMEOUT);
  lan9250_set_reg(priv, LAN9250_HMCSRICR, regval);

  /* Wait for MAC to be ready and read register value */

  lan9250_wait_ready(priv, LAN9250_HMCSRICR, HMCSRICR_CSRB, 0,
                     LAN9250_MAC_TIMEOUT);
  regval = lan9250_get_reg(priv, LAN9250_HMCSRIDR);

  lan9250_getmacreg_dump(address, regval);
  return regval;
}

/****************************************************************************
 * Name: lan9250_set_macreg
 *
 * Description:
 *   Write to a MAC register.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   cmd  - MAC register address
 *   data - The data to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_set_macreg(FAR struct lan9250_driver_s *priv,
                               uint8_t address, uint32_t data)
{
  uint32_t regval = address | HMCSRICR_CSRB;

  lan9250_setmacreg_dump(address, data);

  /* Wait for MAC to be ready and send writing register command and data */

  lan9250_wait_ready(priv, LAN9250_HMCSRICR, HMCSRICR_CSRB, 0,
                     LAN9250_MAC_TIMEOUT);
  lan9250_set_reg(priv, LAN9250_HMCSRIDR, data);
  lan9250_set_reg(priv, LAN9250_HMCSRICR, regval);

  /* Wait until writing MAC is done */

  lan9250_wait_ready(priv, LAN9250_HMCSRICR, HMCSRICR_CSRB, 0,
                     LAN9250_MAC_TIMEOUT);
}

/****************************************************************************
 * Name: lan9250_wait_mac_ready
 *
 * Description:
 *   Wait until LAN9250 MAC is ready
 *
 * Input Parameters:
 *   priv     - Reference to the driver state structure
 *   address  - MAC register address
 *   mask     - MAC register value mask
 *   expected - Expected MAC register value
 *   timeout  - Wait timeout in second
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_wait_mac_ready(FAR struct lan9250_driver_s *priv,
                                   uint8_t address, uint32_t mask,
                                   uint32_t expected, uint32_t second)
{
  clock_t tout_ticks = clock_systime_ticks() + SEC2TICK(second);
  bool timeout = false;

  while (1)
    {
      if ((lan9250_get_macreg(priv, address) & mask) == expected)
        {
          break;
        }
      else if (clock_systime_ticks() > tout_ticks)
        {
          timeout = true;
          break;
        }
    }

  if (timeout)
    {
      nerr("ERROR: wait MAC register:0x%02x, mask:0x%08x, expected:0x%08x\n",
            address, mask, expected);
    }
}

/****************************************************************************
 * Name: lan9250_get_phyreg
 *
 * Description:
 *   Read 16-bits of PHY data.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   address - The PHY register address
 *
 * Returned Value:
 *   16-bit value read from the PHY
 *
 ****************************************************************************/

static uint16_t lan9250_get_phyreg(FAR struct lan9250_driver_s *priv,
                                   uint8_t address)
{
  uint32_t regval = (1 << HMACMIIAR_PHYA_S) |
                    (address << HMACMIIAR_MIIRX_S);

  /* Wait PHY to be ready and send reading register command */

  lan9250_wait_mac_ready(priv, LAN9250_HMACMIIAR, HMACMIIAR_MIIB, 0,
                         LAN9250_PHY_TIMEOUT);
  lan9250_set_macreg(priv, LAN9250_HMACMIIAR, regval);

  /* Wait PHY to be ready and read register value */

  lan9250_wait_mac_ready(priv, LAN9250_HMACMIIAR, HMACMIIAR_MIIB, 0,
                         LAN9250_PHY_TIMEOUT);
  regval = lan9250_get_macreg(priv, LAN9250_HMACMIIDR);

  lan9250_getphyreg_dump(address, regval);

  return regval;
}

/****************************************************************************
 * Name: lan9250_set_phyreg
 *
 * Description:
 *   Write 16-bits of PHY data.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   address - The PHY register address
 *   data    - 16-bit data to write to the PHY
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_set_phyreg(FAR struct lan9250_driver_s *priv,
                               uint8_t address,
                               uint16_t data)
{
  uint32_t regval = (1 << HMACMIIAR_PHYA_S) |
                    (address << HMACMIIAR_MIIRX_S) |
                    HMACMIIAR_MIIW;

  DEBUGASSERT((address & (HMACMIIAR_MIIRX_M >> HMACMIIAR_MIIRX_S))
              == address);

  lan9250_setphyreg_dump(address, data);

  /* Wait PHY to be ready and send writing register command and data */

  lan9250_wait_mac_ready(priv, LAN9250_HMACMIIAR, HMACMIIAR_MIIB, 0,
                         LAN9250_PHY_TIMEOUT);
  lan9250_set_macreg(priv, LAN9250_HMACMIIDR, data);
  lan9250_set_macreg(priv, LAN9250_HMACMIIAR, regval);

  /* Wait PHY until writing is done */

  lan9250_wait_mac_ready(priv, LAN9250_HMACMIIAR, HMACMIIAR_MIIB, 0,
                         LAN9250_PHY_TIMEOUT);
}

/****************************************************************************
 * Name: lan9250_recv_buffer
 *
 * Description:
 *   Read a buffer of data.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   buffer  - A pointer to the buffer to read into
 *   buflen  - The number of bytes to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_recv_buffer(FAR struct lan9250_driver_s *priv,
                                FAR uint8_t *buffer,
                                size_t buflen)
{
#ifdef CONFIG_LAN9250_SPI
#  if LAN9250_SPI_READ == LAN9250_SPI_CMD_FREAD
  uint8_t cmd_buffer[4];
#  else
  uint8_t cmd_buffer[3];
#  endif

  DEBUGASSERT(priv && priv->spi);
  DEBUGASSERT(buflen <= sizeof(priv->pktbuf));

  /* Read dummy data */

  lan9250_get_reg(priv, LAN9250_RXDFR);

  /* Read valid buffer data */

  cmd_buffer[0] = LAN9250_SPI_READ;
  cmd_buffer[1] = (uint8_t)(LAN9250_RXDFR >> 8);
  cmd_buffer[2] = (uint8_t)(LAN9250_RXDFR >> 0);
#  if LAN9250_SPI_READ == LAN9250_SPI_CMD_FREAD
  cmd_buffer[3] = 0xff; /* SPI read dummy */
#  endif

  /* Select LAN9250 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the read command and register address */

  SPI_SNDBLOCK(priv->spi, cmd_buffer, sizeof(cmd_buffer));
  SPI_RECVBLOCK(priv->spi, buffer, buflen);

  /* De-select LAN9250 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
#else
  struct qspi_meminfo_s meminfo;
  int ret;

  meminfo.cmd     = LAN9250_SPI_READ;
  meminfo.addr    = LAN9250_RXDFR;
  meminfo.addrlen = sizeof(uint16_t);
  meminfo.buffer  = buffer;
  meminfo.buflen  = buflen;
  meminfo.dummies = 16; /* 8 SPI dummy clock and 4 dummy RX bytes  */
  meminfo.flags   = QSPIMEM_READ;
  if (priv->sqi_mode)
    {
      meminfo.flags |= QSPIMEM_IQUAD;
    }

  ret = QSPI_MEMORY(priv->qspi, &meminfo);
  DEBUGASSERT(ret == 0);
#endif /* CONFIG_LAN9250_SPI */

  lan9250_buffer_dump(LAN9250_RXDFR, buffer, buflen);
}

/****************************************************************************
 * Name: lan9250_send_buffer
 *
 * Description:
 *   Write a buffer of data.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   buffer  - A pointer to the buffer to write from
 *   buflen  - The number of bytes to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lan9250_send_buffer(FAR struct lan9250_driver_s *priv,
                                       FAR const uint8_t *buffer,
                                       size_t buflen)
{
#ifdef CONFIG_LAN9250_SPI
  uint8_t cmd_buffer[3];
  uint32_t regval;
#else
  struct qspi_meminfo_s meminfo;
  uint32_t regval;
  int ret;
#endif

  lan9250_buffer_dump(LAN9250_TXDFR, buffer, buflen);

  /* LAN9250 SPI write command A fields:
   *
   *   - TX packet length 4 bytes align
   *   - First frame
   *   - Last frame
   */

  regval = SPI_CMD_WRITE_A_4BA | SPI_CMD_WRITE_A_FS |
           SPI_CMD_WRITE_A_LS  | buflen;
  lan9250_set_reg(priv, LAN9250_TXDFR, regval);

  /* LAN9250 SPI write command B fields:
   *
   *   - Packet TAG
   */

  regval = SPI_CMD_WRITE_B_PT | buflen;
  lan9250_set_reg(priv, LAN9250_TXDFR, regval);

#ifdef CONFIG_LAN9250_SPI

  /* Read valid buffer data */

  cmd_buffer[0] = LAN9250_SPI_WRITE;
  cmd_buffer[1] = (uint8_t)(LAN9250_TXDFR >> 8);
  cmd_buffer[2] = (uint8_t)(LAN9250_TXDFR >> 0);

  /* Select LAN9250 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the read command and register address, total 3 bytes */

  SPI_SNDBLOCK(priv->spi, cmd_buffer, 3);

  /* Write data into FIFO with 4-byte align */

  SPI_SNDBLOCK(priv->spi, buffer, LAN9250_ALIGN(buflen));

  /* De-select LAN9250 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
#else
  meminfo.cmd     = LAN9250_SPI_WRITE;
  meminfo.addr    = LAN9250_TXDFR;
  meminfo.addrlen = sizeof(uint16_t);
  meminfo.buffer  = (FAR void *)buffer;
  meminfo.buflen  = LAN9250_ALIGN(buflen);
  meminfo.dummies = 0;
  meminfo.flags   = QSPIMEM_WRITE;
  if (priv->sqi_mode)
    {
      meminfo.flags |= QSPIMEM_IQUAD;
    }

  ret = QSPI_MEMORY(priv->qspi, &meminfo);
  DEBUGASSERT(ret == 0);
#endif /* CONFIG_LAN9250_SPI */
}

/****************************************************************************
 * Name: lan9250_enable_sqi
 *
 * Description:
 *   Send a command to LAN9250 to enable SQI mode.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LAN9250_SQI
static void lan9250_enable_sqi(FAR struct lan9250_driver_s *priv)
{
  clock_t tout_ticks = clock_systime_ticks() +
      MSEC2TICK(LAN9250_SQI_TIMEOUT);
  struct qspi_cmdinfo_s cmdinfo;
  int ret;

  cmdinfo.cmd     = LAN9250_SPI_ENABLE_SQI;
  cmdinfo.addrlen = 0;
  cmdinfo.buflen  = 0;
  cmdinfo.flags   = 0;

  ret = QSPI_COMMAND(priv->qspi, &cmdinfo);
  DEBUGASSERT(ret == 0);

  /* USE SQI mode interface */

  priv->sqi_mode = true;

  /* Check read register value in SQI mode */

  while (1)
    {
      if ((lan9250_get_reg(priv, LAN9250_BOTR) & BOTR_MASK) == BOTR_VAL)
        {
          break;
        }
      else if (clock_systime_ticks() > tout_ticks)
        {
          /* Check timeout and continue to use SPI mode */

          priv->sqi_mode = false;
          break;
        }
    }
}
#endif

/****************************************************************************
 * Name: lan9250_sw_reset
 *
 * Description:
 *   Send a command to reset LAN9250.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lan9250_sw_reset(FAR struct lan9250_driver_s *priv)
{
  /* Wait until LAN9250 SPI bus is ready */

  lan9250_wait_ready(priv, LAN9250_BOTR, BOTR_MASK,
                     BOTR_VAL, LAN9250_RESET_TIMEOUT);

  /* Send a command to reset:
   *
   *   - Digital controller
   *   - HMAC
   *   - PHY
   */

  lan9250_set_reg(priv, LAN9250_RSTCR, RSTCR_DR | RSTCR_PHYR | RSTCR_HMACR);

  /* Wait some time for hardware ready */

  lan9250_wait_ready(priv, LAN9250_HWCFGR, HWCFGR_READY,
                     HWCFGR_READY, LAN9250_RESET_TIMEOUT);

#ifdef CONFIG_LAN9250_SQI

  /* Enable SQI mode */

  lan9250_enable_sqi(priv);
#endif
}

/****************************************************************************
 * Name: lan9250_set_txavailabe
 *
 * Description:
 *   Enable or disable TX data FIFO available interrupt.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   enable - true: enable this interrupt, false: disable this interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_set_txavailabe(FAR struct lan9250_driver_s *priv,
                                   bool enable)
{
  uint32_t regval;

  if (enable)
    {
      /* Configure FIFO level interrupt:
       *
       *   - TX data available level: Ethernet maximum packet size, transform
       *     this value to be block number with 64-byte align
       *   - TX status level: 255, use maximum value so that this will not be
       *     actually used by this driver
       *   - RX status level: 0, so that RX interrupt can trigger once
       *     receiving one packet
       */

      regval = (((LAN9250_PKTBUF_SIZE + 63) / 64) << FLIR_FITXDAL_S) |
               FLIR_FITXSL_M;

      /* Mark TX data FIFO is unavailable */

      priv->tx_available = false;
    }
  else
    {
      /* Configure FIFO level interrupt:
       *
       *   - TX data available level: 255, so that no interrupt triggers
       *   - TX status level: 255, use maximum value so that this will not be
       *     actually used by this driver
       *   - RX status level: 0, so that RX interrupt can trigger once
       *     receiving one packet
       */

      regval = FLIR_FITXDAL_M | FLIR_FITXSL_M;

      /* Mark TX data FIFO is available */

      priv->tx_available = true;
    }

  lan9250_set_reg(priv, LAN9250_FLIR, regval);
}

/****************************************************************************
 * Name: lan9250_reset
 *
 * Description:
 *   Stop, reset, re-initialize, and restart the LAN9250.  This is done
 *   initially, on ifup, and after a TX timeout.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int lan9250_reset(FAR struct lan9250_driver_s *priv)
{
  uint32_t regval;

  nwarn("WARNING: Reset\n");

  /* Configure SPI for the LAN9250 */

  lan9250_config_spi(priv);

  /* reset the LAN9250 by sending command */

  lan9250_sw_reset(priv);

  /* Read LAN9250 hardware ID */

  regval = lan9250_get_reg(priv, LAN9250_CIARR);
  if ((regval & CIARR_CID_M) != CIARR_CID_V)
    {
      nerr("ERROR: Bad Rev ID: %08x\n", regval);
      return -ENODEV;
    }

  ninfo("Rev ID: %08x\n", regval & CIARR_CREV_M);

  /* Configure TX FIFO size mode to be 8:
   *
   *   - TX data FIFO size:   7680
   *   - RX data FIFO size:   7680
   *   - TX status FIFO size: 512
   *   - RX status FIFO size: 512
   */

  regval = HWCFGR_MBO | (8 << HWCFGR_TXFS_S);
  lan9250_set_reg(priv, LAN9250_HWCFGR,  regval);

  /* Configure MAC automatic flow control:
   *
   *   - Automatic flow control high level: 110
   *   - Automatic flow control low level: 55
   *   - Backpressure duration: 4
   *   - Flow control on any frame
   */

  regval = (110 << HMAFCCFGR_AFCHL_S) |
           (55  << HMAFCCFGR_AFCLL_S) |
           (4   << HMAFCCFGR_BPD_S) |
           HMAFCCFGR_FCOAF;
  lan9250_set_reg(priv, LAN9250_HMAFCCFGR, regval);

  /* Configure host MAC flow control:
   *
   *   - Pause time: 15
   *   - Flow control
   */

  regval = (0xf << HMACFCR_PT_S) | HMACFCR_FLE;
  lan9250_set_macreg(priv, LAN9250_HMACFCR, regval);

  /* Configure interrupt:
   *
   *   - Interrupt De-assertion interval: 10
   *   - Interrupt output to pin
   *   - Interrupt pin active output low
   *   - Interrupt pin push-pull driver
   */

  regval = (10 << ICFGR_IDAI_S) |
           ICFGR_IRQE | ICFGR_IRQBT;
  lan9250_set_reg(priv, LAN9250_ICFGR, regval);

  /* Configure interrupt trigger source, please refer to macro
   * LAN9250_INT_SOURCE.
   */

  lan9250_set_reg(priv, LAN9250_IER, LAN9250_INT_SOURCE);

  /* Disable TX data FIFO available interrupt */

  lan9250_set_txavailabe(priv, false);

  /* Configure RX:
   *
   *   - RX DMA counter: Ethernet maximum packet size
   *   - RX data offset: 4, so that need read dummy before reading data
   */

  regval = (LAN9250_PKTBUF_SIZE << RXCFGR_RXDMAC_S) | (4 << RXCFGR_RXDO_S);
  lan9250_set_reg(priv, LAN9250_RXCFGR, regval);

  /* Configure remote power management:
   *
   *   - Auto wakeup
   *   - Disable 1588 clock
   *   - Disable 1588 timestamp unit clock
   *   - Energy-detect
   *   - Wake on
   *   - PME pin push-pull driver
   *   - Clear wakeon
   *   - PME active high
   *   - PME pin
   */

  regval = PMCR_PMWU | PMCR_1588CLKD | PMCR_1588TSUCLKD |
           PMCR_EDE  | PMCR_WOE      | PMCR_PMEBT       |
           PMCR_WOLS | PMCR_PMEP     | PMCR_PMEE;
  lan9250_set_reg(priv, LAN9250_PMCR, regval);

  /* Configure PHY basic control:
   *
   *   - Auto-Negotiation for speed(10 or 100Mbsp) and direction
   *     (half or full duplex)
   */

  lan9250_set_phyreg(priv, LAN9250_PHYBCR, PHYBCR_ANE);

  /* Configure PHY auto-negotiation advertisement capability:
   *
   *   - Asymmetric pause
   *   - Symmetric pause
   *   - 100Base-X full deplex if !CONFIG_LAN9250_HALFDUPPLEX
   *   - 100Base-X half deplex
   *   - 10Base-X full deplex if !CONFIG_LAN9250_HALFDUPPLEX
   *   - 10Base-X half deplex
   *   - Select IEEE802.3
   */

  regval  = PHYANAR_AP     | PHYANAR_SP | PHYANAR_100BXHD |
            PHYANAR_10BXHD | PHYANAR_SF;
#ifndef CONFIG_LAN9250_HALFDUPPLEX
  regval |= PHYANAR_100BXFD | PHYANAR_10BXFD;
#endif
  lan9250_set_phyreg(priv, LAN9250_PHYANAR, regval);

  /* Configure PHY special mode:
   *
   *   - PHY mode = 111b, enable all capable and auto-nagotiation
   *   - PHY address = 1, default value is fixed to 1 by manufacturer
   */

  regval = PHYSMR_PM_M | 1;
  lan9250_set_phyreg(priv, LAN9250_PHYSMR, regval);

  /* Configure PHY special control or status indication:
   *
   *   - Port auto-MDIX determined by bits 14 and 13
   *   - Auto-MDIX
   *   - Disable SQE tests
   */

  regval = PHYSCOSIR_AMDIXC | PHYSCOSIR_AMDIXE | PHYSCOSIR_SQETD;
  lan9250_set_phyreg(priv, LAN9250_PHYSCOSIR, regval);

  /* Configure PHY interrupt source:
   *
   *   - Link up
   *   - Link down
   */

  regval = PHYIER_LU | PHYIER_LD;
  lan9250_set_phyreg(priv, LAN9250_PHYIER, regval);

  /* Configure special control or status:
   *
   *   - Fixed to write 0000010b to reserved filed
   */

  lan9250_set_phyreg(priv, LAN9250_PHYSCOSR, PHYSCOSR_RD);

  /* Clear interrupt status */

  lan9250_set_reg(priv, LAN9250_ISR, UINT32_MAX);

  /* Configure HMAC control:
   *
   *   - Automaticaly strip the pad field on incoming packets
   *   - TX enable
   *   - RX enable
   *   - Full duplex mode if !CONFIG_LAN9250_HALFDUPPLEX
   */

  regval  = HMACCR_APS | HMACCR_TXE | HMACCR_RXE;
#ifndef CONFIG_LAN9250_HALFDUPPLEX
  regval |= HMACCR_FDM;
#endif
  lan9250_set_macreg(priv, LAN9250_HMACCR, regval);

  /** Configure TX:
   *
   *   - TX enable
   */

  lan9250_set_reg(priv, LAN9250_TXCFGR, TXCFGR_TXE);

  return OK;
}

/****************************************************************************
 * Name: lan9250_set_macaddr
 *
 * Description:
 *   Set the MAC address to the configured value.  This is done after ifup
 *   or after a TX timeout.  Note that this means that the interface must
 *   be down before configuring the MAC addr.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_set_macaddr(FAR struct lan9250_driver_s *priv)
{
  uint32_t high_addr;
  uint32_t low_addr;
  FAR uint8_t *mac = priv->dev.d_mac.ether.ether_addr_octet;

  high_addr = (((uint32_t)mac[5] <<  8) | ((uint32_t)mac[4] <<  0));
  low_addr  = (((uint32_t)mac[3] << 24) | ((uint32_t)mac[2] << 16) |
               ((uint32_t)mac[1] <<  8) | ((uint32_t)mac[0] <<  0));

  lan9250_set_macreg(priv, LAN9250_HMACAHR, high_addr);
  lan9250_set_macreg(priv, LAN9250_HMACALR, low_addr);
}

/****************************************************************************
 * Name: lan9250_transmit
 *
 * Description:
 *   Start hardware transmission. Called either from:
 *
 *   -  pkif interrupt when an application responds to the receipt of data
 *      by trying to send something, or
 *   -  From watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int lan9250_transmit(FAR struct lan9250_driver_s *priv)
{
  uint32_t regval;
  uint16_t free_size;
  uint8_t status_size;

  regval = lan9250_get_reg(priv, LAN9250_TXFIR);
  status_size = (regval & TXFIR_TXSFUS_M) >> TXFIR_TXSFUS_S;
  free_size = regval & TXFIR_TXDFFS_M;

  ninfo("availabe status size:%d, free space size:%d\n",
        status_size, free_size);

  /* Clear TX status FIFO if it is no empty by reading data */

  for (int i = 0; i < status_size; i++)
    {
       lan9250_get_reg(priv, LAN9250_TXSFR);
    }

  if (free_size > priv->dev.d_len)
    {
      /* Increment statistics */

      ninfo("Sending packet, pktlen: %d\n", priv->dev.d_len);
      NETDEV_TXPACKETS(&priv->dev);

      /* Send the packet: address=priv->dev.d_buf, length=priv->dev.d_len */

      lan9250_dump_buf("Transmit Packet", priv->dev.d_buf,
                       priv->dev.d_len);

      lan9250_send_buffer(priv, priv->dev.d_buf, priv->dev.d_len);

      if ((free_size - priv->dev.d_len) < LAN9250_PKTBUF_SIZE)
        {
          /* Enable TX data FIFO available interrupt */

          lan9250_set_txavailabe(priv, true);

          /* Enable the TX timeout watchdog (perhaps restarting the timer)
           * when free data space is not enough.
           */

          wd_start(&priv->txtout_timer, LAN9250_TX_TIMEOUT,
                  lan9250_txtout_timercb, (wdparm_t)priv);
        }
    }
  else
    {
      /* This condition is not supported. */

      DEBUGASSERT(0);
    }

  return OK;
}

/****************************************************************************
 * Name: lan9250_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is
 *      reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int lan9250_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct lan9250_driver_s *priv =
        (FAR struct lan9250_driver_s *)dev->d_private;

  /* Send the packet */

  lan9250_transmit(priv);

  /* Stop the poll now if free FIFO buffer is not enough */

  return priv->tx_available ? OK : -EBUSY;
}

/****************************************************************************
 * Name: lan9250_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   dev - The reference to the driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_txavail_work(FAR void *arg)
{
  FAR struct lan9250_driver_s *priv = (FAR struct lan9250_driver_s *)arg;
  FAR struct net_driver_s *dev = &priv->dev;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();
  lan9250_lock_spi(priv);

  /* Ignore the notification if the interface is not yet up */

  if (IFF_IS_UP(dev->d_flags) && priv->tx_available)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      /* If so, then poll the network for new XMIT data */

      devif_poll(dev, lan9250_txpoll);
    }

  /* Release lock on the SPI bus and the network */

  lan9250_unlock_spi(priv);
  net_unlock();
}

/****************************************************************************
 * Name: lan9250_phy_isr
 *
 * Description:
 *   Process LAN9250 PHY interrupt.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_phy_isr(FAR struct lan9250_driver_s *priv)
{
  uint16_t regval;

  regval = lan9250_get_phyreg(priv, LAN9250_PHYISFR);
  if (regval & PHYISFR_LU)
    {
      ninfo("Link up\n");

      IFF_SET_UP(priv->dev.d_flags);
    }
  else if (regval & PHYISFR_LD)
    {
      ninfo("Link down\n");

      IFF_CLR_UP(priv->dev.d_flags);
    }
}

/****************************************************************************
 * Name: lan9250_txavailable_isr
 *
 * Description:
 *   TX data FIFO available interrupt function.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_txavailable_isr(FAR struct lan9250_driver_s *priv)
{
  /* Update statistics */

  NETDEV_TXDONE(&priv->dev);

  /* Disable TX data FIFO available interrupt */

  lan9250_set_txavailabe(priv, false);

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(&priv->txtout_timer);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dev, lan9250_txpoll);
}

/****************************************************************************
 * Name: lan9250_netdev_rx
 *
 * Description:
 *   Give the newly received packet to the network.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_netdev_rx(FAR struct lan9250_driver_s *priv)
{
#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(&priv->dev);
#endif

  /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->dev);

      /* Receive an IPv4 packet from the network device */

      ipv4_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* And send the packet */

          lan9250_transmit(priv);
        }
    }
  else
#endif /* CONFIG_NET_IPv4 */
#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->dev);

      /* Give the IPv6 packet to the network layer */

      ipv6_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* And send the packet */

          lan9250_transmit(priv);
        }
    }
  else
#endif /* CONFIG_NET_IPv6 */
#ifdef CONFIG_NET_ARP
  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      ninfo("ARP packet received (%02x)\n", BUF->type);
      NETDEV_RXARP(&priv->dev);

      arp_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          lan9250_transmit(priv);
        }
    }
  else
#endif /* CONFIG_NET_ARP */
    {
      nwarn("WARNING: Unsupported packet type dropped (%02x)\n",
            HTONS(BUF->type));
      NETDEV_RXDROPPED(&priv->dev);
    }
}

/****************************************************************************
 * Name: lan9250_rxdone_isr
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

static void lan9250_rxdone_isr(FAR struct lan9250_driver_s *priv)
{
  uint8_t pktcnt;
  uint16_t pktlen;
  uint32_t regval;

  /* Update statistics */

  NETDEV_RXPACKETS(&priv->dev);

  /* Check valiad packet count */

  regval = lan9250_get_reg(priv, LAN9250_RXFIR);
  pktcnt = (regval & RXFIR_RXSFUS_M) >> RXFIR_RXSFUS_S;

  ninfo("pktcnt:%d\n", pktcnt);

  while (pktcnt--)
    {
      /* Check valiad packet data size */

      regval = lan9250_get_reg(priv, LAN9250_RXSFR);
      pktlen = (regval & RXSFF_PL_M) >> RXSFF_PL_S;

      ninfo("pktlen:%d\n", pktlen);

      if (pktlen)
        {
          /* Copy the data from the receive buffer to priv->dev.d_buf.
           * ERDPT should be correctly positioned from the last call to
           * end_rdbuffer (above).
           */

          lan9250_recv_buffer(priv, priv->dev.d_buf, pktlen);

          /* Save the packet length (without the 4 byte CRC)
           * in priv->dev.d_len.
           */

          priv->dev.d_len = pktlen - 4;

          lan9250_dump_buf("Received Packet", priv->dev.d_buf,
                           priv->dev.d_len);

          /* Dispatch the packet to the network */

          lan9250_netdev_rx(priv);
        }
    }
}

/****************************************************************************
 * Name: lan9250_int_worker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Input Parameters:
 *   arg - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_int_worker(FAR void *arg)
{
  FAR struct lan9250_driver_s *priv = (FAR struct lan9250_driver_s *)arg;
  uint32_t regval;

  DEBUGASSERT(priv);

  /* Get exclusive access to both the network and the SPI bus. */

  net_lock();
  lan9250_lock_spi(priv);

  /* There is no infinite loop check... if there are always pending
   * interrupts, we are just broken.
   */

  regval = lan9250_get_reg(priv, LAN9250_ISR);
  if ((regval & LAN9250_INT_SOURCE) != 0)
    {
      /* Handle interrupts according to interrupt resource bit
       * settings.
       */

      ninfo("Interrupt status: %08x\n", regval);

#if LAN9250_INT_SOURCE & IER_SW
      if ((regval & ISR_SW) != 0)
        {
          ninfo("\tSoftware\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RAEDY
      if ((regval & ISR_READY) != 0)
        {
          ninfo("\tDevice ready\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_1588
      if ((regval & ISR_1588) != 0)
        {
          ninfo("\t1588\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_PHY
      if ((regval & ISR_PHY) != 0)
        {
          ninfo("\tPHY\n");

          lan9250_phy_isr(priv);
        }
#endif

#if LAN9250_INT_SOURCE & IER_TXSTOP
      if ((regval & ISR_TXS) != 0)
        {
          ninfo("\tTX stop\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RXSTOP
      if ((regval & ISR_RXS) != 0)
        {
          ninfo("\tRX stop\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RXDFH
      if ((regval & ISR_RXDFH) != 0)
        {
          ninfo("\tRX dropped frame count halfway\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_TXIOC
      if ((regval & ISR_TXIOC) != 0)
        {
          ninfo("\tTX IOC\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RXD
      if ((regval & ISR_RXD) != 0)
        {
          ninfo("\tRX DMA\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_GPT
      if ((regval & ISR_GPT) != 0)
        {
          ninfo("\tGP-timer\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_PME
      if ((regval & ISR_PME) != 0)
        {
          ninfo("\tPower management\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_TXSO
      if ((regval & ISR_TXSO) != 0)
        {
          ninfo("\tTX FIFO status overflow\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RWT
      if ((regval & ISR_RWT) != 0)
        {
          ninfo("\tRX watchdog timeout\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RXE
      if ((regval & ISR_RXE) != 0)
        {
          ninfo("\tRX error\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_TXE
      if ((regval & ISR_TXE) != 0)
        {
          ninfo("\tTX error\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_GPIO
      if ((regval & ISR_GPIO) != 0)
        {
          ninfo("\tISR_GPIO event\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_TDFO
      if ((regval & ISR_TDFO) != 0)
        {
          ninfo("\tTX data FIFO overrun\n");
        }
#endif

      if ((regval & ISR_TDFA) != 0)
        {
          ninfo("\tTX data FIFO available\n");

          lan9250_txavailable_isr(priv);
        }

#if LAN9250_INT_SOURCE & IER_TSFF
      if ((regval & ISR_TSFF) != 0)
        {
          ninfo("\tTX status FIFO full\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_TSFL
      if ((regval & ISR_TSFL) != 0)
        {
          ninfo("\tTX status FIFO level\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RXDF
      if ((regval & ISR_RXDF) != 0)
        {
          ninfo("\tRX dropped frame\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RSFF
      if ((regval & ISR_RSFF) != 0)
        {
          ninfo("\tRX status FIFO full\n");
        }
#endif

#if LAN9250_INT_SOURCE & IER_RSFL
      if ((regval & ISR_RSFL) != 0)
        {
          ninfo("\tRX status FIFO level\n");

          lan9250_rxdone_isr(priv);
        }
#endif

      lan9250_set_reg(priv, LAN9250_ISR, regval);
    }

  /* Release lock on the SPI bus and the network */

  lan9250_unlock_spi(priv);
  net_unlock();

  /* Enable ISR_GPIO interrupts after unlocking net so that application
   * could have chance to process Ethernet packet and free iob.
   */

  priv->lower->enable(priv->lower);
}

/****************************************************************************
 * Name: lan9250_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg     - Interrupt private data
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int lan9250_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lan9250_driver_s *priv;

  DEBUGASSERT(arg != NULL);
  priv = (FAR struct lan9250_driver_s *)arg;

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(work_available(&priv->irq_work));

  /* Notice that further ISR_GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in lan9250_int_worker() when the work is done.
   */

  priv->lower->disable(priv->lower);
  return work_queue(LAN9250_WORK, &priv->irq_work, lan9250_int_worker,
                    (FAR void *)priv, 0);
}

/****************************************************************************
 * Name: lan9250_txtout_worker
 *
 * Description:
 *   Our TX watchdog timed out.  This is the worker thread continuation of
 *   the watchdog timer interrupt.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg     - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_txtout_worker(FAR void *arg)
{
  FAR struct lan9250_driver_s *priv = (FAR struct lan9250_driver_s *)arg;
  int ret;

  nerr("ERROR: Tx timeout\n");
  DEBUGASSERT(priv);

  /* Get exclusive access to the network */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(&priv->dev);

  /* Then reset the hardware: Take the interface down, then bring it
   * back up
   */

  ret = lan9250_ifdown(&priv->dev);
  DEBUGASSERT(ret == OK);
  ret = lan9250_ifup(&priv->dev);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dev, lan9250_txpoll);

  /* Release lock on the network */

  net_unlock();
}

/****************************************************************************
 * Name: lan9250_txtout_timercb
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Perform work on the worker thread.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_txtout_timercb(wdparm_t arg)
{
  FAR struct lan9250_driver_s *priv = (FAR struct lan9250_driver_s *)arg;
  int ret;

  /* In complex environments, we cannot do SPI transfers from the timeout
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(priv && work_available(&priv->txtout_work));

  /* Notice that Tx timeout watchdog is not active so further Tx timeouts
   * can occur until we restart the Tx timeout watchdog.
   */

  ret = work_queue(LAN9250_WORK, &priv->txtout_work,
                   lan9250_txtout_worker, priv, 0);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

/****************************************************************************
 * Name: lan9250_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int lan9250_ifup(FAR struct net_driver_s *dev)
{
  FAR struct lan9250_driver_s *priv =
        (FAR struct lan9250_driver_s *)dev->d_private;
  int ret;
#ifdef CONFIG_LAN9250_REGDEBUG
  uint32_t mac_addr[2];
#endif

  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->d_ipaddr), ip4_addr2(dev->d_ipaddr),
        ip4_addr3(dev->d_ipaddr), ip4_addr4(dev->d_ipaddr));

  /* Lock the SPI bus so that we have exclusive access */

  lan9250_lock_spi(priv);

  /* Initialize Ethernet interface, set the MAC address, and make sure that
   * the LAN9250 is not in power save mode.
   */

  ret = lan9250_reset(priv);
  if (ret == OK)
    {
      lan9250_set_macaddr(priv);

      /* Enable interrupts at the LAN9250.  Interrupts are still disabled
       * at the interrupt controller.
       */

      /* Enable the receiver */

      /* Mark the interface up and enable the Ethernet interrupt at the
       * controller
       */

      priv->lower->enable(priv->lower);

      /* Read MAC address here in debug mode */

#ifdef CONFIG_LAN9250_REGDEBUG
      mac_addr[0] = lan9250_get_macreg(priv, LAN9250_HMACALR);
      mac_addr[1] = lan9250_get_macreg(priv, LAN9250_HMACAHR);

      ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
            (uint8_t)(mac_addr[1] >>  0), (uint8_t)(mac_addr[1] >>  8),
            (uint8_t)(mac_addr[0] >> 24), (uint8_t)(mac_addr[0] >> 16),
            (uint8_t)(mac_addr[0] >>  8), (uint8_t)(mac_addr[0] >>  0));
#endif
    }

  /* Un-lock the SPI bus */

  lan9250_unlock_spi(priv);
  return ret;
}

/****************************************************************************
 * Name: lan9250_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int lan9250_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct lan9250_driver_s *priv =
        (FAR struct lan9250_driver_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking down: %u.%u.%u.%u\n",
        ip4_addr1(dev->d_ipaddr), ip4_addr2(dev->d_ipaddr),
        ip4_addr3(dev->d_ipaddr), ip4_addr4(dev->d_ipaddr));

  /* Lock the SPI bus so that we have exclusive access */

  lan9250_lock_spi(priv);

  /* Reset the device and leave in the power save state */

  lan9250_sw_reset(priv);

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  priv->lower->disable(priv->lower);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtout_timer);

  IFF_CLR_UP(priv->dev.d_flags);
  leave_critical_section(flags);

  /* Un-lock the SPI bus */

  lan9250_unlock_spi(priv);
  return OK;
}

/****************************************************************************
 * Name: lan9250_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int lan9250_txavail(FAR struct net_driver_s *dev)
{
  FAR struct lan9250_driver_s *priv =
        (FAR struct lan9250_driver_s *)dev->d_private;

  if (work_available(&priv->txpoll_work))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LAN9250_WORK, &priv->txpoll_work,
                 lan9250_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: lan9250_addmac
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
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int lan9250_addmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac)
{
  /* #warning "Multicast MAC support not implemented" */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: lan9250_rmmac
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
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int lan9250_rmmac(FAR struct net_driver_s *dev,
                         FAR const uint8_t *mac)
{
  /* #warning "Multicast MAC support not implemented" */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: lan9250_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int lan9250_ioctl(FAR struct net_driver_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct lan9250_driver_s *priv =
        (FAR struct lan9250_driver_s *)dev->d_private;
  int ret = OK;

  /* Lock the SPI bus so that we have exclusive access */

  lan9250_lock_spi(priv);

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
                    (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = 0;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
                    (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->val_out = lan9250_get_phyreg(priv, req->reg_num);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
                    (struct mii_ioctl_data_s *)((uintptr_t)arg);
          lan9250_set_phyreg(priv, req->reg_num, req->val_in);
        }
        break;
#endif /* CONFIG_NETDEV_PHY_IOCTL */

    default:
      nerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
      ret = -ENOTTY; /* Special return value for this case */
    }

  /* Un-lock the SPI bus */

  lan9250_unlock_spi(priv);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lan9250_initialize
 *
 * Description:
 *   Initialize the LAN9250 Ethernet driver.
 *
 * Input Parameters:
 *   spi   - A reference to the platform's SPI driver for the LAN9250 when
 *           enable CONFIG_LAN9250_SPI
 *   qspi  - A reference to the platform's SQI driver for the LAN9250 when
 *           enable CONFIG_LAN9250_SQI
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., LAN9250 GPIO interrupts).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int lan9250_initialize(
#ifdef CONFIG_LAN9250_SPI
                       FAR struct spi_dev_s *spi,
#else
                       FAR struct qspi_dev_s *qspi,
#endif
                       FAR const struct lan9250_lower_s *lower)
{
  FAR struct lan9250_driver_s *priv = &g_lan9250;
  FAR struct net_driver_s *dev = &priv->dev;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lan9250_driver_s));

  dev->d_buf     = (FAR uint8_t *)priv->pktbuf;
  dev->d_ifup    = lan9250_ifup;
  dev->d_ifdown  = lan9250_ifdown;
  dev->d_txavail = lan9250_txavail;
#ifdef CONFIG_NET_MCASTGROUP
  dev->d_addmac  = lan9250_addmac;
  dev->d_rmmac   = lan9250_rmmac;
#endif
#ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl   = lan9250_ioctl;             /* Handle network IOCTL commands */
#endif
  dev->d_private = priv;

#ifdef CONFIG_LAN9250_SPI
  priv->spi      = spi;
#else
  priv->qspi     = qspi;
#endif

  priv->lower    = lower;

  /* Attach the interrupt to the driver (but don't enable it yet) */

  if (lower->attach(lower, lan9250_interrupt, priv) < 0)
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Try to read MAC address from low-level function. */

  if (lower->getmac)
    {
      if (lower->getmac(lower, priv->dev.d_mac.ether.ether_addr_octet) < 0)
        {
          nerr("ERROR: Failed read MAC address\n");
          return -EAGAIN;
        }
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  return netdev_register(dev, NET_LL_ETHERNET);
}

/****************************************************************************
 * Function: lan9250_uninitialize
 *
 * Description:
 *   Un-initialize the Ethernet driver
 *
 * Input Parameters:
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., LAN9250 GPIO interrupts).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int lan9250_uninitialize(FAR const struct lan9250_lower_s *lower)
{
  FAR struct lan9250_driver_s *priv = &g_lan9250;
  FAR struct net_driver_s *dev = &priv->dev;

  int ret = netdev_unregister(dev); /* No such interface yet */
  if (ret < 0)
    {
      nerr("ERROR: netdev_unregister failed: %d\n", ret);
      return ret;
    }

  /* Detach the interrupt to the driver */

  if (lower->attach(lower, NULL, NULL) < 0)
    {
      nerr("ERROR: irq_detach failed: %d\n", ret);
      return -EAGAIN;
    }

  return OK;
}
