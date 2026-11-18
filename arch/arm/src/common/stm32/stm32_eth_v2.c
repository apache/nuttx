/****************************************************************************
 * arch/arm/src/common/stm32/stm32_eth_v2.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <syslog.h>
#include <errno.h>

#include <sys/param.h>

#include <arch/barriers.h>
#include <arpa/inet.h>
#include <netinet/if_ether.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#ifdef CONFIG_NET_PHYLIB
#  include <nuttx/ethtool.h>
#  include <nuttx/net/phy.h>
#endif
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/crc64.h>
#include <nuttx/timers/ptp_clock.h>

#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include <nuttx/cache.h>
#include "arm_internal.h"

#include "hardware/stm32_pinmap.h"
#include "stm32.h"
#include "stm32_eth.h"
#include "stm32_eth_phyif.h"
#include "stm32_uid.h"

#ifdef CONFIG_STM32_ETH_MDIO_GPIO
#  include "stm32_mdio_gpio.h"
#else
#  include "stm32_mdio.h"
#endif

#include <arch/board/board.h>

/* STM32_NETHERNET determines the number of physical interfaces that can
 * be supported by the hardware.  CONFIG_STM32_ETHMAC will defined if
 * any STM32 Ethernet support is enabled in the configuration.
 */

#if STM32_NETHERNET > 0 && defined(CONFIG_STM32_ETHMAC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if STM32_NETHERNET > 1
#  error "Logic to support multiple Ethernet interfaces is incomplete"
#endif

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

/* Select work queue */

#  if defined(CONFIG_STM32_ETHMAC_HPWORK)
#    define ETHWORK HPWORK
#  elif defined(CONFIG_STM32_ETHMAC_LPWORK)
#    define ETHWORK LPWORK
#  else
#    define ETHWORK LPWORK
#  endif
#endif

#if defined(CONFIG_ETH0_PHY_AM79C874)
#  define STM32_PHYID1       MII_PHYID1_AM79C874
#  define STM32_PHYID2       MII_PHYID2_AM79C874
#elif defined(CONFIG_ETH0_PHY_AR8031)
#  define STM32_PHYID1       MII_PHYID1_AR8031
#  define STM32_PHYID2       MII_PHYID2_AR8031
#elif defined(CONFIG_ETH0_PHY_KS8721)
#  define STM32_PHYID1       MII_PHYID1_KS8721
#  define STM32_PHYID2       MII_PHYID2_KS8721
#elif defined(CONFIG_ETH0_PHY_KSZ8041)
#  define STM32_PHYID1       MII_PHYID1_KSZ8041
#  define STM32_PHYID2       MII_PHYID2_KSZ8041
#elif defined(CONFIG_ETH0_PHY_KSZ8051)
#  define STM32_PHYID1       MII_PHYID1_KSZ8051
#  define STM32_PHYID2       MII_PHYID2_KSZ8051
#elif defined(CONFIG_ETH0_PHY_KSZ8061)
#  define STM32_PHYID1       MII_PHYID1_KSZ8061
#  define STM32_PHYID2       MII_PHYID2_KSZ8061
#elif defined(CONFIG_ETH0_PHY_KSZ8081)
#  define STM32_PHYID1       MII_PHYID1_KSZ8081
#  define STM32_PHYID2       MII_PHYID2_KSZ8081
#elif defined(CONFIG_ETH0_PHY_DP83848C)
#  define STM32_PHYID1       MII_PHYID1_DP83848C
#  define STM32_PHYID2       MII_PHYID2_DP83848C
#elif defined(CONFIG_ETH0_PHY_DP83825I)
#  define STM32_PHYID1       MII_PHYID1_DP83825I
#  define STM32_PHYID2       MII_PHYID2_DP83825I
#elif defined(CONFIG_ETH0_PHY_TJA1100)
#  define STM32_PHYID1       MII_PHYID1_TJA1100
#  define STM32_PHYID2       MII_PHYID2_TJA1100
#elif defined(CONFIG_ETH0_PHY_TJA1101)
#  define STM32_PHYID1       MII_PHYID1_TJA1101
#  define STM32_PHYID2       MII_PHYID2_TJA1101
#elif defined(CONFIG_ETH0_PHY_TJA1103)
#  define STM32_PHYID1       MII_PHYID1_TJA1103
#  define STM32_PHYID2       MII_PHYID2_TJA1103
#elif defined(CONFIG_ETH0_PHY_LAN8720)
#  define STM32_PHYID1       MII_PHYID1_LAN8720
#  define STM32_PHYID2       MII_PHYID2_LAN8720
#elif defined(CONFIG_ETH0_PHY_LAN8740)
#  define STM32_PHYID1       MII_PHYID1_LAN8740
#  define STM32_PHYID2       MII_PHYID2_LAN8740
#elif defined(CONFIG_ETH0_PHY_LAN8740A)
#  define STM32_PHYID1       MII_PHYID1_LAN8740A
#  define STM32_PHYID2       MII_PHYID2_LAN8740A
#elif defined(CONFIG_ETH0_PHY_LAN8742A)
#  define STM32_PHYID1       MII_PHYID1_LAN8742A
#  define STM32_PHYID2       MII_PHYID2_LAN8742A
#elif defined(CONFIG_ETH0_PHY_DM9161)
#  define STM32_PHYID1       MII_PHYID1_DM9161
#  define STM32_PHYID2       MII_PHYID2_DM9161
#elif defined(CONFIG_ETH0_PHY_YT8512)
#  define STM32_PHYID1       MII_PHYID1_YT8512
#  define STM32_PHYID2       MII_PHYID2_YT8512
#endif

/* The IRQ of the MAC is ETH1 on parts that number their MACs */

#if !defined(STM32_IRQ_ETH) && defined(STM32_IRQ_ETH1)
#  define STM32_IRQ_ETH      STM32_IRQ_ETH1
#endif

/* The reset of the MAC on the RCC */

#if defined(RCC_AHB1RSTR_ETH1MACRST)
#  define STM32_RCC_ETH_RSTR STM32_RCC_AHB1RSTR
#  define STM32_RCC_ETH_RST  RCC_AHB1RSTR_ETH1MACRST
#elif defined(RCC_AHB1RSTR_ETHRST)
#  define STM32_RCC_ETH_RSTR STM32_RCC_AHB1RSTR
#  define STM32_RCC_ETH_RST  RCC_AHB1RSTR_ETHRST
#elif defined(RCC_AHB5RSTR_ETH1RST)
#  define STM32_RCC_ETH_RSTR STM32_RCC_AHB5RSTR
#  define STM32_RCC_ETH_RST  RCC_AHB5RSTR_ETH1RST
#else
#  error "Unknown RCC reset of the Ethernet MAC"
#endif

#ifndef CONFIG_STM32_PHYADDR
#  error "CONFIG_STM32_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_STM32_MII) && !defined(CONFIG_STM32_RMII) && \
    !defined(CONFIG_STM32_RGMII)
#  warning "None of CONFIG_STM32_MII, CONFIG_STM32_RMII or CONFIG_STM32_RGMII"
#endif

#if (defined(CONFIG_STM32_MII) + defined(CONFIG_STM32_RMII) + \
     defined(CONFIG_STM32_RGMII)) > 1
#  error "Only one of CONFIG_STM32_{MII,RMII,RGMII} can be defined"
#endif

#if defined(CONFIG_STM32_RGMII) && !defined(STM32_ETH_PHYIF_RGMII)
#  error "This chip has no RGMII interface"
#endif

#ifdef CONFIG_STM32_MII
#  if !defined(CONFIG_STM32_MII_MCO1) && !defined(CONFIG_STM32_MII_MCO2) && \
      !defined(CONFIG_STM32_MII_EXTCLK)
#    warning "Neither CONFIG_STM32_MII_MCO1, CONFIG_STM32_MII_MCO2, nor CONFIG_STM32_MII_EXTCLK defined"
#  endif
#  if defined(CONFIG_STM32_MII_MCO1) && defined(CONFIG_STM32_MII_MCO2)
#    error "Both CONFIG_STM32_MII_MCO1 and CONFIG_STM32_MII_MCO2 defined"
#  endif
#endif

#ifdef CONFIG_STM32_RMII
#  if !defined(CONFIG_STM32_RMII_MCO1) && !defined(CONFIG_STM32_RMII_MCO2) && \
      !defined(CONFIG_STM32_RMII_EXTCLK)
#    warning "Neither CONFIG_STM32_RMII_MCO1, CONFIG_STM32_RMII_MCO2, nor CONFIG_STM32_RMII_EXTCLK defined"
#  endif
#  if defined(CONFIG_STM32_RMII_MCO1) && defined(CONFIG_STM32_RMII_MCO2)
#    error "Both CONFIG_STM32_RMII_MCO1 and CONFIG_STM32_RMII_MCO2 defined"
#  endif
#endif

#if defined(CONFIG_STM32_AUTONEG) && !defined(CONFIG_NET_PHYLIB)
#  ifndef CONFIG_STM32_PHYSR
#    error "CONFIG_STM32_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_STM32_PHYSR_ALTCONFIG
#    ifndef CONFIG_STM32_PHYSR_ALTMODE
#      error "CONFIG_STM32_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_10HD
#      error "CONFIG_STM32_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_100HD
#      error "CONFIG_STM32_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_10FD
#      error "CONFIG_STM32_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_100FD
#      error "CONFIG_STM32_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_STM32_PHYSR_SPEED
#      error "CONFIG_STM32_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_100MBPS
#      error "CONFIG_STM32_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_MODE
#      error "CONFIG_STM32_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_FULLDUPLEX
#      error "CONFIG_STM32_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

#ifdef CONFIG_STM32_ETH_PTP
/* The timestamp unit is clocked from HCLK. The system time advances by
 * STM32_PTP_SSINC nanoseconds each time the 32-bit accumulator of the
 * addend register overflows, so its rate is HCLK * addend / 2^32. Aim at
 * an update rate of half of HCLK: the nominal addend is then close to 2^31
 * and the frequency can be trimmed by +/- 50 %.
 */

#  define STM32_PTP_CLOCK      STM32_HCLK_FREQUENCY
#  define STM32_PTP_SSINC      ((2 * NSEC_PER_SEC + STM32_PTP_CLOCK / 2) / \
                                STM32_PTP_CLOCK)
#  define STM32_PTP_ADDEND     ((uint32_t)((((uint64_t)1 << 32) * \
                                            NSEC_PER_SEC) / \
                                           ((uint64_t)STM32_PTP_SSINC * \
                                            STM32_PTP_CLOCK)))

/* Time to wait for the unit to take an update of the addend or of the
 * system time, in microseconds.
 */

#  define STM32_PTP_UPDATE_USTIMEOUT  (1000)

#  ifdef CONFIG_STM32_ETH_TIMESTAMP_RX

/* Time to wait for the DMA to write the context descriptor that follows
 * the last descriptor of a timestamped frame, in microseconds.
 */

#    define STM32_PTP_CTX_USTIMEOUT   (10)
#  endif

/* The addend can be trimmed by up to 50% each way from its nominal value,
 * in parts per billion.
 */

#  define STM32_PTP_MAX_ADJ           (500000000)

#  ifdef CONFIG_STM32_ETH_PTP_GPIO

/* The pulse-per-second output is a pulse train with a period of one
 * second and a width of half of it, that starts at a whole second of the
 * system time. The first pulse is at least STM32_PTP_PPS_MARGIN_NS ahead,
 * so the target time is loaded before it. The interval and the width are
 * in increments of the system time.
 */

#    define STM32_PTP_PPS_MARGIN_NS   (100000000)
#    define STM32_PTP_PPS_INTERVAL    (NSEC_PER_SEC / STM32_PTP_SSINC)
#    define STM32_PTP_PPS_WIDTH       ((NSEC_PER_SEC / 2) / STM32_PTP_SSINC)
#  endif
#endif

/* Each DMA buffer is one IOB.  The frame starts at netpkt_getdata(), the
 * buffer size is a multiple of 4 bytes (the DMA bus width) and fills the
 * IOB, so that the network does not need to pack the received buffers.
 */

#if CONFIG_NET_LL_GUARDSIZE < ETH_HDRLEN
#  error "CONFIG_NET_LL_GUARDSIZE must be at least ETH_HDRLEN"
#endif

/* The DMA buffers must be word aligned */

#if (CONFIG_IOB_ALIGNMENT % 4) != 0 || \
    ((CONFIG_NET_LL_GUARDSIZE - ETH_HDRLEN) % 4) != 0
#  error "The Ethernet DMA buffers must be word aligned"
#endif

#ifdef ARMV8M_DCACHE_LINESIZE
#  define ETH_DCACHE_LINESIZE ARMV8M_DCACHE_LINESIZE
#else
#  define ETH_DCACHE_LINESIZE ARMV7M_DCACHE_LINESIZE
#endif

/* With the D-Cache, the data of each IOB must fill whole D-Cache lines:
 * the RX DMA corrupts other data in the lines of its buffers, also in the
 * write-through mode.
 */

#if (defined(CONFIG_ARMV7M_DCACHE) || defined(CONFIG_ARMV8M_DCACHE)) && \
    ((CONFIG_IOB_ALIGNMENT % ETH_DCACHE_LINESIZE) != 0 || \
     (CONFIG_IOB_BUFSIZE % ETH_DCACHE_LINESIZE) != 0 || \
     defined(CONFIG_IOB_ALLOC))
#  error "The IOBs must be aligned to the D-Cache lines"
#endif

#define STM32_ETH_BUFSIZE \
  ((CONFIG_IOB_BUFSIZE - (CONFIG_NET_LL_GUARDSIZE - ETH_HDRLEN)) & ~3)

#if STM32_ETH_BUFSIZE > ETH_CTX_TDES2_MSS_MASK
#  error "CONFIG_IOB_BUFSIZE is too large"
#endif

/* DMA buffers of the largest frame, with the 4 byte CRC */

#define STM32_ETH_NSEGS \
  ((CONFIG_NET_ETH_PKTSIZE + 4 + STM32_ETH_BUFSIZE - 1) / STM32_ETH_BUFSIZE)

/* Number of RX and TX frames.  The descriptor rings have STM32_ETH_NSEGS
 * descriptors for each frame.
 */

#ifndef CONFIG_STM32_ETH_NRXDESC
#  define CONFIG_STM32_ETH_NRXDESC 8
#endif
#ifndef CONFIG_STM32_ETH_NTXDESC
#  define CONFIG_STM32_ETH_NTXDESC 4
#endif

#define STM32_ETH_NRXBUF  (CONFIG_STM32_ETH_NRXDESC * STM32_ETH_NSEGS)
#define STM32_ETH_NTXBUF  (CONFIG_STM32_ETH_NTXDESC * STM32_ETH_NSEGS)

/* IOBs held by the driver: the RX ring and the new IOBs of one received
 * frame, and one packet for each TX frame.
 */

#define STM32_ETH_RXIOBS  (STM32_ETH_NRXBUF + STM32_ETH_NSEGS)
#define STM32_ETH_TXPKTS  CONFIG_STM32_ETH_NTXDESC

#if STM32_NETHERNET * (STM32_ETH_RXIOBS + STM32_ETH_TXPKTS) > \
    CONFIG_IOB_NBUFFERS
#  error "CONFIG_IOB_NBUFFERS is too small for the Ethernet DMA rings"
#endif

#ifndef CONFIG_STM32_ETH_TXTIMEOUT
#  define CONFIG_STM32_ETH_TXTIMEOUT 60
#endif

/* Buffers used for DMA access must begin on an address aligned with the
 * D-Cache line and must be an even multiple of the D-Cache line size.
 * These size/alignment requirements are necessary so that D-Cache flush
 * and invalidate operations will not have any additional effects.
 *
 * The TX and RX descriptors are 16 bytes in size
 */

#define DMA_BUFFER_MASK    (ETH_DCACHE_LINESIZE - 1)
#define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)

/* The RX descriptor errors that make a received frame unusable */

#define RDES3_FATAL_ERRORS (ETH_RDES3_WB_RE | ETH_RDES3_WB_OE |   \
                            ETH_RDES3_WB_RWT | ETH_RDES3_WB_GP |  \
                            ETH_RDES3_WB_CE)

#define DESC_SIZE       16
#define DESC_PADSIZE        DMA_ALIGN_UP(DESC_SIZE)

#define RXTABLE_SIZE        (STM32_NETHERNET * STM32_ETH_NRXBUF)
#define TXTABLE_SIZE        (STM32_NETHERNET * STM32_ETH_NTXBUF)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_NET_INFO
#  undef CONFIG_STM32_ETHMAC_REGDEBUG
#endif

/* Clocking *****************************************************************/

/* Set MACMDIOAR CR bits depending on HCLK setting */

#if STM32_HCLK_FREQUENCY >= 20000000 && STM32_HCLK_FREQUENCY < 35000000
#  define ETH_MACMDIOAR_CR ETH_MACMDIOAR_CR_DIV16
#elif STM32_HCLK_FREQUENCY >= 35000000 && STM32_HCLK_FREQUENCY < 60000000
#  define ETH_MACMDIOAR_CR ETH_MACMDIOAR_CR_DIV26
#elif STM32_HCLK_FREQUENCY >= 60000000 && STM32_HCLK_FREQUENCY < 100000000
#  define ETH_MACMDIOAR_CR ETH_MACMDIOAR_CR_DIV42
#elif STM32_HCLK_FREQUENCY >= 100000000 && STM32_HCLK_FREQUENCY < 150000000
#  define ETH_MACMDIOAR_CR ETH_MACMDIOAR_CR_DIV62
#elif STM32_HCLK_FREQUENCY >= 150000000 && STM32_HCLK_FREQUENCY <= 250000000
#  define ETH_MACMDIOAR_CR ETH_MACMDIOAR_CR_DIV102
#elif STM32_HCLK_FREQUENCY >= 250000000 && STM32_HCLK_FREQUENCY <= 300000000
#  define ETH_MACMDIOAR_CR ETH_MACMDIOAR_CR_DIV124
#else
#  error "STM32_HCLK_FREQUENCY not supportable"
#endif

/* Timing *******************************************************************/

/* TX timeout */

#define STM32_TXTIMEOUT   (CONFIG_STM32_ETH_TXTIMEOUT*CLK_TCK)

/* PHY reset/configuration delays in milliseconds */

#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT  (0x0004ffff)
#define PHY_WRITE_TIMEOUT (0x0004ffff)
/* The PHY is polled every PHY_POLL_MS milliseconds and given up to
 * PHY_LINK_TIMEOUT_MS milliseconds to bring the link up or to complete
 * auto-negotiation. The time is in milliseconds so that it does not depend
 * on the period of the system tick.
 */

#define PHY_POLL_MS         (10)
#define PHY_LINK_TIMEOUT_MS (5000)
#define PHY_RETRY_TIMEOUT   (PHY_LINK_TIMEOUT_MS / PHY_POLL_MS)

/* With PHYLIB, the PHY state machine polls the link every
 * CONFIG_NET_PHYLIB_POLL_INTERVAL milliseconds, and auto-negotiation can
 * take a few seconds more.
 */

#define PHYLIB_LINK_TIMEOUT_MS (PHY_LINK_TIMEOUT_MS + \
                                2 * CONFIG_NET_PHYLIB_POLL_INTERVAL)

/* The PHY interface mode given to PHYLIB.  A board selects an RGMII mode
 * with PHY internal delays with BOARD_ETH_PHY_INTERFACE in its board.h.
 */

#ifndef BOARD_ETH_PHY_INTERFACE
#  if defined(CONFIG_STM32_RGMII)
#    define BOARD_ETH_PHY_INTERFACE PHY_INTERFACE_RGMII
#  elif defined(CONFIG_STM32_MII)
#    define BOARD_ETH_PHY_INTERFACE PHY_INTERFACE_MII
#  else
#    define BOARD_ETH_PHY_INTERFACE PHY_INTERFACE_RMII
#  endif
#endif

/* MAC software reset timeout in microseconds.  The reset completes only
 * once all clock domains, including the PHY receive clock, are running;
 * this can take several milliseconds.
 */

#define MAC_READY_USTIMEOUT (10000)

/* Register values **********************************************************/

/* Clear the MACCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_MACCR_RE     Bit 0:  Receiver enable
 * ETH_MACCR_TE     Bit 1:  Transmitter enable
 * ETH_MACCR_PRELEN Bits 2-3: Preamble length for transmit packets
 * ETH_MACCR_DC     Bit 4:  Deferral check
 * ETH_MACCR_BL     Bits 5-6: Back-off limit
 * ETH_MACCR_DR     Bit 8:  Retry disable
 * ETH_MACCR_DCRS   Bit 9:  Carrier sense disable
 * ETH_MACCR_DO     Bit 10: Receive own disable
 * ETH_MACCR_LM     Bit 12: Loopback mode
 * ETH_MACCR_DM     Bit 13: Duplex mode
 * ETH_MACCR_FES    Bit 14: Fast Ethernet speed
 * ETH_MACCR_PS     Bit 15: Port select (gigabit MACs only)
 * ETH_MACCR_JE     Bit 16: Jumbo packet enable
 * ETH_MACCR_JD     Bit 17: Jabber disable
 * ETH_MACCR_WD     Bit 19: Watchdog disable
 * ETH_MACCR_ACS    Bit 20: Automatic pad/CRC stripping
 * ETH_MACCR_CST    Bit 21: CRC stripping for Type frames
 * ETH_MACCR_S2KP   Bit 22: IEEE 802.3as Support for 2K Packets
 * ETH_MACCR_GPSLCE Bit 23: Giant Packet Size Limit Control Enable
 * ETH_MACCR_IPG    Bits 24-26: Inter-packet gap
 * ETH_MACCR_IPC    Bit 27: IPv4 checksum offload
 * ETH_MACCR_SARC   Bits 28-30: Src address insertion or replacement
 * ETH_MACCR_ARPEN  Bit 31: ARP Offload Enable
 */

#ifdef CONFIG_STM32_HAVE_ETH_RGMII
#  define MACCR_PS_BIT ETH_MACCR_PS
#else
#  define MACCR_PS_BIT 0
#endif

#define MACCR_CLEAR_BITS                                                \
  (ETH_MACCR_RE | ETH_MACCR_TE | ETH_MACCR_PRELEN_MASK | ETH_MACCR_DC | \
   ETH_MACCR_BL_MASK | ETH_MACCR_DR | ETH_MACCR_DCRS | ETH_MACCR_DO |   \
   ETH_MACCR_LM | ETH_MACCR_DM | ETH_MACCR_FES | ETH_MACCR_JE |         \
   ETH_MACCR_JD | ETH_MACCR_WD | ETH_MACCR_ACS | ETH_MACCR_CST |        \
   ETH_MACCR_S2KP | ETH_MACCR_GPSLCE | ETH_MACCR_IPG_MASK |             \
   ETH_MACCR_IPC | ETH_MACCR_SARC_MASK | ETH_MACCR_ARPEN |              \
   MACCR_PS_BIT)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACCR_RE    Receiver enable                0 (disabled)
 * ETH_MACCR_TE    Transmitter enable             0 (disabled)
 * ETH_MACCR_DC    Deferral check                 0 (disabled)
 * ETH_MACCR_BL    Back-off limit                 0 (10)
 * ETH_MACCR_ACS   Automatic pad/CRC stripping    0 (disabled)
 * ETH_MACCR_DR    Retry disable                  1 (disabled)
 * ETH_MACCR_IPC   IPv4 checksum offload
 *                 Depends on CONFIG_STM32_ETH_HWCHECKSUM
 * ETH_MACCR_LM    Loopback mode                  0 (disabled)
 * ETH_MACCR_DO    Receive own disable            0 (enabled)
 * ETH_MACCR_DCRS  Carrier sense disable          0 (enabled)
 * ETH_MACCR_IPG   Inter-packet gap               0 (96 bits)
 * ETH_MACCR_JD    Jabber disable                 0 (enabled)
 * ETH_MACCR_WD    Watchdog disable               0 (enabled)
 * ETH_MACCR_CST   CRC stripping for Type frames  0 (disabled)
 *
 * The following are set conditionally based on mode and speed.
 *
 * ETH_MACCR_DM       Duplex mode                    Depends on priv->fduplex
 * ETH_MACCR_FES      Fast Ethernet speed            Depends on priv->mbps100
 * ETH_MACCR_PS       Port select                    Depends on priv->gbit
 */

#ifdef CONFIG_STM32_ETH_HWCHECKSUM
#  define MACCR_SET_BITS                                                \
  (ETH_MACCR_BL_10 | ETH_MACCR_DR | ETH_MACCR_IPC | ETH_MACCR_IPG(96))
#else
#  define MACCR_SET_BITS                                \
  (ETH_MACCR_BL_10 | ETH_MACCR_DR | ETH_MACCR_IPG(96))
#endif

/* Clear the MACPFR bits that will be setup during MAC initialization (or
 * that are cleared unconditionally).  Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ETH_MACPFR_PM    Bit 0: Promiscuous mode
 * ETH_MACPFR_HUC   Bit 1: Hash unicast
 * ETH_MACPFR_HMC   Bit 2: Hash multicast
 * ETH_MACPFR_DAIF  Bit 3: Destination address inverse filtering
 * ETH_MACPFR_PAM   Bit 4: Pass all multicast
 * ETH_MACPFR_DBF   Bit 5: Broadcast frames disable
 * ETH_MACPFR_PCF   Bits 6-7: Pass control frames
 * ETH_MACPFR_SAIF  Bit 8: Source address inverse filtering
 * ETH_MACPFR_SAF   Bit 9: Source address filter
 * ETH_MACPFR_HPF   Bit 10: Hash or perfect filter
 * ETH_MACPFR_VTFE  Bit 16: VLAN Tag Filter Enable
 * ETH_MACPFR_IPFE  Bit 20: Layer 3 and Layer 4 Filter Enable
 * ETH_MACPFR_DNTU  Bit 21: Drop Non-TCP/UDP over IP Packets
 * ETH_MACPFR_RA    Bit 31: Receive all
 */

#define MACPFR_CLEAR_BITS                                               \
  (ETH_MACPFR_PM | ETH_MACPFR_HUC | ETH_MACPFR_HMC | ETH_MACPFR_DAIF |  \
   ETH_MACPFR_PAM | ETH_MACPFR_DBF | ETH_MACPFR_PCF_MASK | ETH_MACPFR_SAIF | \
   ETH_MACPFR_SAF | ETH_MACPFR_HPF | ETH_MACPFR_VTFE | ETH_MACPFR_IPFE | \
   ETH_MACPFR_DNTU | ETH_MACPFR_RA)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACPFR_HUC   Hash unicast                           0 (perfect
 *                                                            dest filtering)
 * ETH_MACPFR_HMC   Hash multicast                         0 (perfect
 *                                                            dest filtering)
 * ETH_MACPFR_DAIF  Destination address inverse filtering  0 (normal)
 * ETH_MACPFR_PAM   Pass all multicast                     0 (Depends on HMC
 *                                                            bit)
 * ETH_MACPFR_DBF   Broadcast frames disable               0 (enabled)
 * ETH_MACPFR_PCF   Pass control frames                    1 (block all but
 *                                                            PAUSE)
 * ETH_MACPFR_SAIF  Source address inverse filtering       0 (not used)
 * ETH_MACPFR_SAF   Source address filter                  0 (disabled)
 * ETH_MACPFR_HPF   Hash or perfect filter                 0 (Only matching
 *                                                            frames passed)
 * ETH_MACPFR_RA    Receive all                            0 (disabled)
 */

#ifdef CONFIG_NET_PROMISCUOUS
#  define MACPFR_SET_BITS (ETH_MACPFR_PCF_PAUSE | ETH_MACPFR_PM)
#else
#  define MACPFR_SET_BITS (ETH_MACPFR_PCF_PAUSE)
#endif

/* Clear the MACQTXFCR and MACRXFCR bits that will be setup during MAC
 * initialization (or that are cleared unconditionally).  Per the reference
 * manual, all reserved bits must be retained at their reset value.
 *
 * ETH_MACQTXFCR_FCB_BPA Bit 0: Flow control busy/back pressure activate
 * ETH_MACQTXFCR_TFE     Bit 1: Transmit flow control enable

 * ETH_MACQTXFCR_PLT     Bits 4-6: Pause low threshold
 * ETH_MACQTXFCR_DZPQ    Bit 7: Zero-quanta pause disable
 * ETH_MACQTXFCR_PT      Bits 16-31: Pause time

 * ETH_MACRXFCR_RFE      Bit 0: Receive flow control enable
 * ETH_MACRXFCR_UP       Bit 1: Unicast pause frame detect
 */

#define MACQTXFCR_CLEAR_MASK                                            \
  (ETH_MACQTXFCR_FCB_BPA | ETH_MACQTXFCR_TFE |                          \
   ETH_MACQTXFCR_PLT_MASK | ETH_MACQTXFCR_DZPQ | ETH_MACQTXFCR_PT_MASK)

#define MACRXFCR_CLEAR_MASK                     \
  (ETH_MACRXFCR_RFE | ETH_MACRXFCR_UP)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACQTXFCR_FCB_BPA Flow control busy/back pressure activate   0
 *   (no pause control frame)
 * ETH_MACQTXFCR_TFE     Transmit flow control enable               0
 *   (disabled)
 * ETH_MACQTXFCR_PLT     Pause low threshold                        0
 *   (pause time - 4)
 * ETH_MACQTXFCR_DZPQ    Zero-quanta pause disable                  1
 *   (disabled)
 * ETH_MACQTXFCR_PT      Pause time                                 0
 * ETH_MACRXFCR_RFE      Receive flow control enable                0
 *   (disabled)
 * ETH_MACRXFCR_UP       Unicast pause frame detect                 0
 *   (disabled)
 */

#define MACQTXFCR_SET_MASK (ETH_MACQTXFCR_PLT_M4 | ETH_MACQTXFCR_DZPQ)
#define MACRXFCR_SET_MASK (0)

/* Clear the MTLTXQOMR bits that will be setup during MAC initialization
 * (or that are cleared unconditionally).  Per the reference manual, all
 * reserved bits must be retained at their reset value.
 * ETH_MTLTXQOMR_FTQ      Bit 0: Flush Transmit Queue
 * ETH_MTLTXQOMR_TSF      Bit 1: Transmit Store and Forward
 * ETH_MTLTXQOMR_TXQEN    Bits 2-3: Transmit Queue Enable
 * ETH_MTLTXQOMR_TTC      Bits 4-6: Transmit Threshold Control
 *
 * The queue size is left at its reset value: it is read-only on the MACs
 * with one queue and the reset value splits the FIFO between the queues
 * on the MACs with more.
 */

#define MTLTXQOMR_CLEAR_MASK                            \
  (ETH_MTLTXQOMR_FTQ | ETH_MTLTXQOMR_TSF |              \
   ETH_MTLTXQOMR_TXQEN_MASK | ETH_MTLTXQOMR_TTC_MASK)

/* Clear the MTLRXQOMR bits that will be setup during MAC initialization
 * (or that are cleared unconditionally).  Per the reference manual, all
 * reserved bits must be retained at their reset value.
 *
 * ETH_MTLRXQOMR_RTC_MASK    Bits 0-1: Receive Queue Threshold Control
 * ETH_MTLRXQOMR_FUP         Bit 3: Forward Undersized Good Packets
 * ETH_MTLRXQOMR_FEP         Bit 4: Forward Error Packets
 * ETH_MTLRXQOMR_RSF         Bit 5: Receive Queue Store and Forward
 * ETH_MTLRXQOMR_DIS_TCP_EF  Bit 6: Disable Dropping of TCP/IP Checksum Error
 *                                  Packets
 * ETH_MTLRXQOMR_EHFC        Bit 7: Enable Hardware Flow Control
 * ETH_MTLRXQOMR_RFA_MASK    Bits 8-10: Threshold for Activating Flow Control
 * ETH_MTLRXQOMR_RFD_MASK    Bits 14-16: Threshold for Deactivating Flow
 *                                       Control
 *
 * The queue size is left at its reset value, as for the TX queue.
 */

#define MTLRXQOMR_CLEAR_MASK                            \
  (ETH_MTLRXQOMR_RTC_MASK | ETH_MTLRXQOMR_FUP |         \
   ETH_MTLRXQOMR_FEP | ETH_MTLRXQOMR_RSF |              \
   ETH_MTLRXQOMR_DIS_TCP_EF | ETH_MTLRXQOMR_EHFC |      \
   ETH_MTLRXQOMR_RFA_MASK | ETH_MTLRXQOMR_RFD_MASK)

/* Store and forward is needed by the checksum offload engine */

#define MTLTXQOMR_SET_MASK                                      \
  (ETH_MTLTXQOMR_TTC_64 | ETH_MTLTXQOMR_TSF |                   \
   ETH_MTLTXQOMR_TXQEN_ENABLED | ETH_MTLTXQOMR_FTQ)

/* Receive in store and forward mode: a frame that overflows the RX queue
 * is dropped there, it does not take RX descriptors with an error.
 *
 * TODO: Check whether use flow control (bits RFD, RFA EHFC)
 */

#define MTLRXQOMR_SET_MASK  (ETH_MTLRXQOMR_RTC_64 | ETH_MTLRXQOMR_RSF)

/* Clear the DMAMR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_DMAMR_SWR     Bit 0: Software Reset
 * ETH_DMAMR_DA      Bit 1: DMA Tx or Rx Arbitration Scheme
 * ETH_DMAMR_TXPR    Bit 11: Transmit priority
 * ETH_DMAMR_PR      Bits 12-14: Priority ratio
 * ETH_DMAMR_INTM    Bits 16-17: Interrupt Mode
 */

#define DMAMR_CLEAR_MASK                                                \
  (ETH_DMAMR_SWR | ETH_DMAMR_DA | ETH_DMAMR_TXPR | ETH_DMAMR_PR_MASK |  \
   ETH_DMAMR_INTM_MASK)

#define DMAMR_SET_MASK (ETH_DMAMR_PR_2TO1)

/* TODO: Comment these bits */

#define DMACCR_CLEAR_MASK                                               \
  ( ETH_DMACCR_MSS_MASK | ETH_DMACCR_PBLX8 | ETH_DMACCR_DSL_MASK)

/* Set the descriptor skip length, counted in words of the DMA data bus:
 * 64 bits on an AXI bus master, 32 bits on an AHB one.
 */

#ifdef CONFIG_STM32_HAVE_ETH_AXI
#  define STM32_ETH_DMA_BUSWIDTH 8
#else
#  define STM32_ETH_DMA_BUSWIDTH 4
#endif

#define DMACCR_SET_MASK                                 \
  (ETH_DMACCR_DSL((DESC_PADSIZE - DESC_SIZE) / STM32_ETH_DMA_BUSWIDTH))

#define DMASBMR_CLEAR_MASK                              \
  (STM32_ETH_DMASBMR_FB | STM32_ETH_DMASBMR_AAL |       \
   STM32_ETH_DMASBMR_MB | STM32_ETH_DMASBMR_RB)

/* An AXI bus master also needs the burst lengths it may use: up to 16
 * beats.
 */

#ifdef CONFIG_STM32_HAVE_ETH_AXI
#  define DMASBMR_SET_MASK                              \
  (STM32_ETH_DMASBMR_FB | STM32_ETH_DMASBMR_AAL |       \
   STM32_ETH_DMASBMR_BLEN4 | STM32_ETH_DMASBMR_BLEN8 |  \
   STM32_ETH_DMASBMR_BLEN16)
#else
#  define DMASBMR_SET_MASK                              \
  (STM32_ETH_DMASBMR_FB | STM32_ETH_DMASBMR_AAL)
#endif

#define DMACTXCR_CLEAR_MASK                                     \
  (ETH_DMACTXCR_ST | ETH_DMACTXCR_ST | ETH_DMACTXCR_OSF |       \
   ETH_DMACTXCR_TSE | ETH_DMACTXCR_TXPBL_MASK)

#define DMACTXCR_SET_MASK                       \
  (ETH_DMACTXCR_OSF | ETH_DMACTXCR_TXPBL(32))

#define DMACRXCR_CLEAR_MASK                     \
  (ETH_DMACRXCR_SR | ETH_DMACRXCR_RBSZ_MASK |   \
   ETH_DMACRXCR_RXPBL_MASK | ETH_DMACRXCR_RPF)

#define DMACRXCR_SET_MASK                       \
  (ETH_DMACRXCR_RBSZ(STM32_ETH_BUFSIZE) | ETH_DMACRXCR_RXPBL(32))

/* Interrupt bit sets *******************************************************/

/* All interrupts in the normal and abnormal interrupt summary.  Early
 * transmit interrupt (ETI) is excluded from the abnormal set because it
 * causes too many interrupts and is not interesting.
 */

#define ETH_DMAINT_NORMAL                                               \
  (ETH_DMACIER_TIE | ETH_DMACIER_TBUE | ETH_DMACIER_RIE | ETH_DMACIER_ERIE)

#define ETH_DMAINT_ABNORMAL                                             \
  (ETH_DMACIER_TXSE | ETH_DMACIER_RBUE | ETH_DMACIER_RSE |              \
   ETH_DMACIER_RWTE | /* ETH_DMACIER_ETIE | */ ETH_DMACIER_FBEE)

/* Normal receive, transmit, error interrupt enable bit sets */

#define ETH_DMAINT_RECV_ENABLE    (ETH_DMACIER_NIE | ETH_DMACIER_RIE)
#define ETH_DMAINT_XMIT_ENABLE    (ETH_DMACIER_NIE | ETH_DMACIER_TIE)
#define ETH_DMAINT_XMIT_DISABLE   (ETH_DMACIER_TIE)

#define ETH_DMAINT_ERROR_ENABLE (ETH_DMACIER_AIE | ETH_DMAINT_ABNORMAL)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This union type forces the allocated size of TX&RX descriptors to be the
 * padded to a exact multiple of the Cortex-M7 D-Cache line size.
 */

union stm32_desc_u
{
  uint8_t             pad[DESC_PADSIZE];
  struct eth_desc_s   desc;
};

/* The stm32_ethmac_s encapsulates all state information for a single
 * hardware interface
 */

struct stm32_ethmac_s
{
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s dev;    /* Interface understood by the network */

  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  uint8_t              mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t              gbit    : 1; /* 1000MBps operation (RGMII only) */
  uint8_t              fduplex : 1; /* Full (vs. half) duplex */
  uint8_t              intf;        /* Ethernet interface number */
  struct wdog_s        txtimeout;   /* TX timeout timer */
  struct work_s        irqwork;     /* For deferring interrupt work to the work queue */
  mutex_t              lock;        /* Protects the descriptor rings */

  /* Used to track transmit and receive descriptors */

  struct eth_desc_s *txhead;        /* Next available TX descriptor */
  struct eth_desc_s *rxhead;        /* Next available RX descriptor */

  struct eth_desc_s *txchbase;      /* TX descriptor ring base address */
  struct eth_desc_s *rxchbase;      /* RX descriptor ring base address */

  struct eth_desc_s *txtail;        /* First "in_flight" TX descriptor */
  struct eth_desc_s *rxcurr;        /* First RX descriptor of the segment */
  uint16_t             segments;    /* RX segment count */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */

  /* The packet of each RX descriptor, and the packet sent by each TX
   * descriptor, held by the descriptor with the last segment.
   */

  netpkt_t            *rxpkt[STM32_ETH_NRXBUF];
  netpkt_t            *txpkt[STM32_ETH_NTXBUF];

  struct mdio_bus_s *mdio;
#ifdef CONFIG_NET_PHYLIB
  struct phy_device_s *phydev; /* PHYLIB device of the PHY */
#endif

#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
  struct iob_queue_s txtstampq;     /* Timestamped frames to loop back */

  /* Per TX descriptor with the last segment: the copy of the frame that is
   * waiting for its timestamp.
   */

  struct iob_s *txmeta[STM32_ETH_NTXBUF];
#endif

#if defined(CONFIG_STM32_ETH_PTP) && defined(CONFIG_PTP_CLOCK)
  struct ptp_lowerhalf_s ptp_lower; /* PTP hardware clock lower half */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMA buffers.  DMA buffers must:
 *
 * 1. Be a multiple of the D-Cache line size.  This requirement is assured
 *    by the definition of RXDMA buffer size above.
 * 2. Be aligned a D-Cache line boundaries, and
 * 3. Be positioned in DMA-able memory.  This must be managed by logic
 *    in the linker script file.
 *
 * These DMA buffers are defined sequentially here to best assure optimal
 * packing of the buffers.
 */

/* Descriptor allocations */

static union stm32_desc_u g_rxtable[RXTABLE_SIZE]
aligned_data(ETH_DCACHE_LINESIZE);
static union stm32_desc_u g_txtable[TXTABLE_SIZE]
aligned_data(ETH_DCACHE_LINESIZE);

/* These are the pre-allocated Ethernet device structures */

static struct stm32_ethmac_s g_stm32ethmac[STM32_NETHERNET];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_STM32_ETHMAC_REGDEBUG
static uint32_t stm32_getreg(uint32_t addr);
static void stm32_putreg(uint32_t val, uint32_t addr);
static void stm32_checksetup(void);
#else
#  define stm32_getreg(addr)     getreg32(addr)
#  define stm32_putreg(val,addr) putreg32(val,addr)
#  define stm32_checksetup()
#endif

/* Common TX logic */

static netpkt_t *stm32_txrepack(netpkt_t *pkt);
static int  stm32_txiov(netpkt_t *pkt, struct iovec *iov);
static int  stm32_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt);

/* Interrupt handling */

static void stm32_enableint(struct stm32_ethmac_s *priv, uint32_t ierbit);
static void stm32_disableint(struct stm32_ethmac_s *priv, uint32_t ierbit);

static void stm32_freesegment(struct stm32_ethmac_s *priv,
                              struct eth_desc_s *rxfirst, int segments);
static netpkt_t *stm32_rxframe(struct stm32_ethmac_s *priv,
                               struct eth_desc_s *rxfirst,
                               struct eth_desc_s *rxlast, int segments);
static netpkt_t *stm32_receive(struct netdev_lowerhalf_s *dev);
static int  stm32_freeframe(struct stm32_ethmac_s *priv);
static int  stm32_txreap(struct stm32_ethmac_s *priv);
#ifdef CONFIG_NET_NETPOLL
static void stm32_txpoll(struct netdev_lowerhalf_s *dev);
#endif
static void stm32_txdone(struct stm32_ethmac_s *priv);
#ifdef CONFIG_STM32_ETH_TIMESTAMP_RX
static void stm32_rxtimestamp(struct stm32_ethmac_s *priv,
                              struct eth_desc_s *last, netpkt_t *pkt);
#endif
#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
static void stm32_txtstamp_flush(struct stm32_ethmac_s *priv);
#endif

static void stm32_interrupt_work(void *arg);
static int  stm32_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void stm32_txtimeout_work(void *arg);
static void stm32_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  stm32_ifup(struct netdev_lowerhalf_s *lower);
static int  stm32_ifdown(struct netdev_lowerhalf_s *lower);
static int  stm32_down(struct stm32_ethmac_s *priv, bool freerx);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int  stm32_addmac(struct netdev_lowerhalf_s *dev,
                         const uint8_t *mac);
#endif
#ifdef CONFIG_NET_IGMP
static int  stm32_rmmac(struct netdev_lowerhalf_s *dev,
                        const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_PHY_IOCTL
static int  stm32_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                        unsigned long arg);
#endif

/* Descriptor Initialization */

static void stm32_txdescinit(struct stm32_ethmac_s *priv,
                             union stm32_desc_u *txtable);
static int  stm32_rxdescinit(struct stm32_ethmac_s *priv,
                             union stm32_desc_u *rxtable);

/* PHY Initialization */
#ifndef CONFIG_STM32_NO_PHY
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int  stm32_phyintenable(struct stm32_ethmac_s *priv);
#endif

#ifdef CONFIG_ETH0_PHY_DM9161
static inline int stm32_dm9161(struct stm32_ethmac_s *priv);
#endif
#ifdef CONFIG_NET_PHYLIB
static int  stm32_phylib_init(struct stm32_ethmac_s *priv);
#endif
static int  stm32_phyinit(struct stm32_ethmac_s *priv);
#ifdef CONFIG_STM32_ETHMAC_REGDEBUG
static void  stm32_phyregdump(struct stm32_ethmac_s *priv);
#endif
#endif

/* MAC/DMA Initialization */

#ifdef CONFIG_STM32_MII
static inline void stm32_selectmii(void);
#endif
#ifdef CONFIG_STM32_RGMII
static inline void stm32_selectrgmii(void);
#endif
#ifdef CONFIG_STM32_RMII
static inline void stm32_selectrmii(void);
#endif
static inline void stm32_ethgpioconfig(struct stm32_ethmac_s *priv);
static int  stm32_ethreset(struct stm32_ethmac_s *priv);
#ifdef CONFIG_STM32_ETH_PTP
static int  stm32_eth_ptp_init(void);
#endif
static int  stm32_macconfig(struct stm32_ethmac_s *priv);
static void stm32_macaddress(struct stm32_ethmac_s *priv);
static int  stm32_macenable(struct stm32_ethmac_s *priv);
static int  stm32_ethconfig(struct stm32_ethmac_s *priv);

static const struct netdev_ops_s g_stm32_ops =
{
  .ifup     = stm32_ifup,
  .ifdown   = stm32_ifdown,
  .transmit = stm32_transmit,
  .receive  = stm32_receive,
#ifdef CONFIG_NET_IGMP
  .addmac   = stm32_addmac,
  .rmmac    = stm32_rmmac,
#endif
#ifdef CONFIG_NETDEV_PHY_IOCTL
  .ioctl    = stm32_ioctl,
#endif
#ifdef CONFIG_NET_NETPOLL
  .txpoll   = stm32_txpoll,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ETHMAC_REGDEBUG
static uint32_t stm32_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              ninfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          ninfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  ninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: stm32_putreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   addr - The register address to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ETHMAC_REGDEBUG
static void stm32_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  ninfo("%08" PRIx32 "<-%08" PRIx32 "\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: stm32_checksetup
 *
 * Description:
 *   Show the state of critical configuration registers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ETHMAC_REGDEBUG
static void stm32_checksetup(void)
{
}
#endif

/****************************************************************************
 * Function: stm32_get_next_txdesc
 *
 * Description:
 *   Returns the next tx descriptor in the list
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *   curr - Pointer to a tx descriptor
 *
 * Returned Value:
 *   pointer to the next tx descriptor for the current interface
 *
 ****************************************************************************/

static struct eth_desc_s *stm32_get_next_txdesc(struct stm32_ethmac_s *priv,
                                                struct eth_desc_s * curr)
{
  union stm32_desc_u *first =
    &g_txtable[priv->intf * STM32_ETH_NTXBUF];
  union stm32_desc_u *last =
    &g_txtable[priv->intf * STM32_ETH_NTXBUF + STM32_ETH_NTXBUF - 1];
  union stm32_desc_u *next = ((union stm32_desc_u *)curr) + 1;

  if (next > last)
    {
      next = first;
    }

  return &next->desc;
}

/****************************************************************************
 * Function: stm32_txrepack
 *
 * Description:
 *   Copy a packet that has more buffers than a TX frame can use to a new
 *   packet with full buffers.  The old packet is freed on success.
 *
 * Parameters:
 *   pkt - The packet to copy
 *
 * Returned Value:
 *   The new packet, or NULL on failure.
 *
 ****************************************************************************/

static netpkt_t *stm32_txrepack(netpkt_t *pkt)
{
  netpkt_t *copy;

  copy = iob_tryalloc(false);
  if (copy == NULL)
    {
      return NULL;
    }

  iob_reserve(copy, CONFIG_NET_LL_GUARDSIZE);
  if (iob_clone_partial(pkt, pkt->io_pktlen, 0, copy, 0, false, false) < 0)
    {
      iob_free_chain(copy);
      return NULL;
    }

  /* The Ethernet header is in front of the data of the first buffer */

  memcpy(IOB_DATA(copy) - ETH_HDRLEN, IOB_DATA(pkt) - ETH_HDRLEN,
         ETH_HDRLEN);
  iob_free_chain(pkt);
  return copy;
}

/****************************************************************************
 * Function: stm32_txiov
 *
 * Description:
 *   Get the buffers of a packet, one for each TX descriptor.
 *
 * Parameters:
 *   pkt - The packet
 *   iov - The STM32_ETH_NSEGS buffers
 *
 * Returned Value:
 *   The number of buffers, or -E2BIG if there are more than
 *   STM32_ETH_NSEGS.
 *
 ****************************************************************************/

static int stm32_txiov(netpkt_t *pkt, struct iovec *iov)
{
  int n = 1;

  iov[0].iov_base = IOB_DATA(pkt) - ETH_HDRLEN;
  iov[0].iov_len  = pkt->io_len + ETH_HDRLEN;

  for (pkt = pkt->io_flink; pkt != NULL; pkt = pkt->io_flink)
    {
      if (pkt->io_len == 0)
        {
          continue;
        }

      if (n >= STM32_ETH_NSEGS)
        {
          return -E2BIG;
        }

      iov[n].iov_base = IOB_DATA(pkt);
      iov[n].iov_len  = pkt->io_len;
      n++;
    }

  return n;
}

/****************************************************************************
 * Function: stm32_transmit
 *
 * Description:
 *   NuttX Callback: start the hardware transmission of a packet.  The
 *   buffers of the packet are sent in place, one TX descriptor each.  The
 *   packet is held until the transmission is complete.
 *
 * Parameters:
 *   dev - Reference to the lower half driver state structure
 *   pkt - The packet to send
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int stm32_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev;
  struct eth_desc_s *txdesc;
  struct eth_desc_s *txfirst;
  struct iovec iov[STM32_ETH_NSEGS];
  uint32_t des3;
  int nsegs;
  int ret = OK;
  int i;
#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
  struct iob_s *clone = NULL;
#endif

  /* A packet with too many buffers is copied to full buffers, which
   * need at most STM32_ETH_NSEGS descriptors.
   */

  nsegs = stm32_txiov(pkt, iov);

  nxmutex_lock(&priv->lock);

  if (!priv->ifup)
    {
      ret = -ENETDOWN;
      goto errout;
    }

  /* Free the completed frames, then check that the TX descriptors are
   * free.  If not, -EBUSY: the upper half sends the packet again after
   * netdev_lower_txdone().
   *
   * In a race condition, ETH_TDES3_RD_OWN may be cleared BUT still
   * not available because stm32_freeframe() has not yet run. If
   * stm32_freeframe() has run, the buffer1 pointer (des0) will be
   * nullified.
   */

  stm32_freeframe(priv);

  txfirst = priv->txhead;
  txdesc  = txfirst;

  for (i = 0; i < (nsegs < 0 ? STM32_ETH_NSEGS : nsegs); i++)
    {
      up_invalidate_dcache((uintptr_t)txdesc,
                           (uintptr_t)txdesc + sizeof(struct eth_desc_s));

      if ((txdesc->des3 & ETH_TDES3_RD_OWN) != 0 || txdesc->des0 != 0)
        {
          ret = -EBUSY;
          goto errout;
        }

      txdesc = stm32_get_next_txdesc(priv, txdesc);
    }

  /* Nothing can fail after the packet is copied: on failure the upper half
   * frees the packet that it passed.
   */

  if (nsegs < 0)
    {
      netpkt_t *copy = stm32_txrepack(pkt);

      if (copy == NULL)
        {
          ret = -ENOMEM;
          goto errout;
        }

      pkt   = copy;
      nsegs = stm32_txiov(pkt, iov);
      DEBUGASSERT(nsegs > 0);

      txdesc = txfirst;
      for (i = 0; i < nsegs; i++)
        {
          txdesc = stm32_get_next_txdesc(priv, txdesc);
        }
    }

  priv->txhead = txdesc;

  ninfo("len: %u nsegs: %d txhead: %p\n",
        netpkt_getdatalen(dev, pkt), nsegs, txfirst);

#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
  /* If a packet socket asked for the transmit timestamp, keep a copy of
   * the frame to loop back with it, and ask the MAC to timestamp.
   */

  if (pkt->io_conn != NULL)
    {
      clone = iob_tryalloc(false);
      if (clone != NULL)
        {
          iob_reserve(clone, CONFIG_NET_LL_GUARDSIZE);
          if (iob_clone_partial(pkt, pkt->io_pktlen, 0, clone, 0,
                                false, false) < 0)
            {
              iob_free_chain(clone);
              clone = NULL;
            }
        }

      if (clone == NULL)
        {
          nerr("ERROR: Failed to clone IOB for TX timestamp\n");
        }
    }
#endif

  /* Set up the descriptors of all segments, but give the first one to the
   * DMA last, so that the DMA never sees an incomplete frame.  If hardware
   * checksum is enabled, IP header + payload + pseudo-header checksum will
   * be computed.
   */

  txdesc = txfirst;
  for (i = 0; i < nsegs; i++)
    {
      /* Flush the contents of the TX buffer into physical memory */

      up_clean_dcache((uintptr_t)iov[i].iov_base,
                      (uintptr_t)iov[i].iov_base + iov[i].iov_len);

      des3 = 0;

      txdesc->des0 = (uint32_t)iov[i].iov_base;
      txdesc->des1 = 0;
      txdesc->des2 = iov[i].iov_len;

      if (i == 0)
        {
#ifdef CONFIG_STM32_ETH_HWCHECKSUM
          des3 |= ETH_TDES3_RD_FD | ETH_TDES3_RD_CIC_ALL;
#else
          des3 |= ETH_TDES3_RD_FD;
#endif
#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
          if (clone != NULL)
            {
              txdesc->des2 |= ETH_TDES2_RD_TTSE;
            }
#endif
        }

      if (i == nsegs - 1)
        {
          /* This is the last segment.  Hold the packet here and ask for
           * an interrupt when the transfer completes.
           */

          int txindex = (union stm32_desc_u *)txdesc -
                        (union stm32_desc_u *)priv->txchbase;

          des3 |= ETH_TDES3_RD_LD;
          txdesc->des2 |= ETH_TDES2_RD_IOC;
          priv->txpkt[txindex] = pkt;
#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
          priv->txmeta[txindex] = clone;
#endif
        }

      if (i > 0)
        {
          des3 |= ETH_TDES3_RD_OWN;
        }

      txdesc->des3 = des3;

      /* Flush the contents of the modified TX descriptor into physical
       * memory.
       */

      up_clean_dcache((uintptr_t)txdesc,
                      (uintptr_t)txdesc + sizeof(struct eth_desc_s));

      txdesc = stm32_get_next_txdesc(priv, txdesc);
    }

  UP_MB();

  txfirst->des3 |= ETH_TDES3_RD_OWN;
  up_clean_dcache((uintptr_t)txfirst,
                  (uintptr_t)txfirst + sizeof(struct eth_desc_s));

  /* If there is no other TX buffer, in flight, then remember the location
   * of the TX descriptor.  This is the location to check for TX done events.
   */

  if (!priv->txtail)
    {
      DEBUGASSERT(priv->inflight == 0);
      priv->txtail = txfirst;
    }

  /* Increment the number of TX transfer in-flight */

  priv->inflight++;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  UP_MB();

  /* Enable TX interrupts */

  stm32_enableint(priv, ETH_DMACIER_TIE);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, STM32_TXTIMEOUT,
           stm32_txtimeout_expiry, (wdparm_t)priv);

  /* Update the tx descriptor tail pointer register to start the DMA */

  putreg32((uintptr_t)priv->txhead, STM32_ETH_DMACTXDTPR);

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Function: stm32_enableint
 *
 * Description:
 *   Enable a "normal" interrupt
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with priv->lock held.
 *
 ****************************************************************************/

static void stm32_enableint(struct stm32_ethmac_s *priv, uint32_t ierbit)
{
  uint32_t regval;

  /* Enable the specified "normal" interrupt */

  regval  = stm32_getreg(STM32_ETH_DMACIER);
  regval |= (ETH_DMACIER_NIE | ierbit);
  stm32_putreg(regval, STM32_ETH_DMACIER);
}

/****************************************************************************
 * Function: stm32_disableint
 *
 * Description:
 *   Disable a normal interrupt.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with priv->lock held.
 *
 ****************************************************************************/

static void stm32_disableint(struct stm32_ethmac_s *priv, uint32_t ierbit)
{
  uint32_t regval;

  /* Disable the "normal" interrupt */

  regval  = stm32_getreg(STM32_ETH_DMACIER);
  regval &= ~ierbit;

  /* Are all "normal" interrupts now disabled? */

  if ((regval & ETH_DMAINT_NORMAL) == 0)
    {
      /* Yes.. disable normal interrupts */

      regval &= ~ETH_DMACIER_NIE;
    }

  stm32_putreg(regval, STM32_ETH_DMACIER);
}

/****************************************************************************
 * Function: stm32_get_next_rxdesc
 *
 * Description:
 *   Returns the next rx descriptor in the list
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *   curr - Pointer to a rx descriptor
 *
 * Returned Value:
 *   pointer to the next rx descriptor for the current interface
 *
 ****************************************************************************/

static struct eth_desc_s *stm32_get_next_rxdesc(struct stm32_ethmac_s *priv,
                                                struct eth_desc_s * curr)
{
  union stm32_desc_u *first =
    &g_rxtable[priv->intf * STM32_ETH_NRXBUF];
  union stm32_desc_u *last =
    &g_rxtable[priv->intf * STM32_ETH_NRXBUF + STM32_ETH_NRXBUF - 1];
  union stm32_desc_u *next = ((union stm32_desc_u *)curr) + 1;

  if (next > last)
    {
      next = first;
    }

  return &next->desc;
}

/****************************************************************************
 * Function: stm32_freesegment
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the received frame.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with priv->lock held.
 *
 ****************************************************************************/

static void stm32_freesegment(struct stm32_ethmac_s *priv,
                              struct eth_desc_s *rxfirst, int segments)
{
  struct eth_desc_s *rxdesc;
  int i;

  ninfo("rxfirst: %p segments: %d\n", rxfirst, segments);

  /* Give the freed RX buffers back to the Ethernet MAC to be refilled */

  rxdesc = rxfirst;
  for (i = 0; i < segments; i++)
    {
      int rxindex = (union stm32_desc_u *)rxdesc -
                    (union stm32_desc_u *)priv->rxchbase;

      /* Set the buffer of the descriptor, the DMA write-back format
       * overwrites des0.
       */

      rxdesc->des0 = (uint32_t)netpkt_getdata(&priv->dev,
                                              priv->rxpkt[rxindex]);

      /* Set OWN bit in RX descriptors.  This gives the buffers back to DMA */

      rxdesc->des3 = ETH_RDES3_RD_OWN | ETH_RDES3_RD_IOC |
                     ETH_RDES3_RD_BUF1V;

      /* Make sure that the modified RX descriptor is written to physical
       * memory.
       */

      up_clean_dcache((uintptr_t)rxdesc,
                      (uintptr_t)rxdesc + sizeof(struct eth_desc_s));

      /* Get the next RX descriptor in the chain */

      rxdesc = stm32_get_next_rxdesc(priv, rxdesc);

      /* Update the tail pointer */

      stm32_putreg((uintptr_t)rxdesc, STM32_ETH_DMACRXDTPR);
    }

  /* Reset the segment management logic */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Check if the RX Buffer unavailable flag is set */

  if ((stm32_getreg(STM32_ETH_DMACSR) & ETH_DMACSR_RBU) != 0)
    {
      /* Clear the RBU flag */

      stm32_putreg(ETH_DMACSR_RBU, STM32_ETH_DMACSR);

      nerr("ETH_DMACSR_RBU\n");

      /* To resume processing Rx descriptors, the application should change
       * the ownership of the descriptor and issue a Receive Poll Demand
       * command. If this command is not issued, the Rx process resumes when
       * the next recognized incoming packet is received. In ring mode, the
       * application should advance the Receive Descriptor Tail Pointer
       * register of a channel. This bit is set only when the DMA owns the
       * previous Rx descriptor.
       */
    }
}

/****************************************************************************
 * Function: stm32_rxframe
 *
 * Description:
 *   Take the buffers of a received frame from its RX descriptors and give
 *   the descriptors new buffers.  If there are no new buffers, the frame is
 *   dropped and the descriptors keep their buffers.
 *
 * Parameters:
 *   priv     - Reference to the driver state structure
 *   rxfirst  - The descriptor of the first segment
 *   rxlast   - The descriptor of the last segment
 *   segments - The number of segments
 *
 * Returned Value:
 *   The packet of the frame, or NULL if it was dropped.
 *
 * Assumptions:
 *   The descriptor ring lock is held.
 *
 ****************************************************************************/

static netpkt_t *stm32_rxframe(struct stm32_ethmac_s *priv,
                               struct eth_desc_s *rxfirst,
                               struct eth_desc_s *rxlast, int segments)
{
  struct netdev_lowerhalf_s *dev = &priv->dev;
  struct eth_desc_s *rxdesc;
  netpkt_t *newpkt[STM32_ETH_NSEGS];
  netpkt_t *head = NULL;
  netpkt_t *tail = NULL;
  unsigned int remain;
  unsigned int len;
  int i;

  /* Get the Frame Length of the received packet: subtract 4 bytes of the
   * CRC
   */

  len = ((rxlast->des3 & ETH_RDES3_WB_PL_MASK) >> ETH_RDES3_WB_PL_SHIFT) - 4;

  /* Check if the packet is a valid size for the network buffer
   * configuration.  A frame of this size has at most STM32_ETH_NSEGS
   * segments.
   */

  if (len > CONFIG_NET_ETH_PKTSIZE || len < ETH_HDRLEN ||
      segments > STM32_ETH_NSEGS)
    {
      nwarn("WARNING: DROPPED Too big: %u in %d segments\n", len, segments);
      NETDEV_RXERRORS(&dev->netdev);
      stm32_freesegment(priv, rxfirst, segments);
      return NULL;
    }

  for (i = 0; i < segments; i++)
    {
      newpkt[i] = netpkt_alloc(dev, NETPKT_RX);
      if (newpkt[i] == NULL)
        {
          while (i-- > 0)
            {
              netpkt_free(dev, newpkt[i], NETPKT_RX);
            }

          nwarn("WARNING: No free buffer, packet dropped\n");
          NETDEV_RXDROPPED(&dev->netdev);
          stm32_freesegment(priv, rxfirst, segments);

          return NULL;
        }
    }

  /* Link the buffers of the segments.  The last segment can hold the CRC
   * only.
   */

  remain = len;
  rxdesc = rxfirst;

  for (i = 0; i < segments; i++)
    {
      int rxindex = (union stm32_desc_u *)rxdesc -
                    (union stm32_desc_u *)priv->rxchbase;
      netpkt_t *iob = priv->rxpkt[rxindex];
      unsigned int seglen = MIN(remain, STM32_ETH_BUFSIZE);
      uint8_t *buffer;

      /* Force the received data to be re-read from physical memory */

      buffer = netpkt_getdata(dev, iob);
      up_invalidate_dcache((uintptr_t)buffer,
                           (uintptr_t)buffer + STM32_ETH_BUFSIZE);

      /* No cached data of the new buffer may be written over the received
       * data.
       */

      buffer = netpkt_getdata(dev, newpkt[i]);
      up_invalidate_dcache((uintptr_t)buffer,
                           (uintptr_t)buffer + STM32_ETH_BUFSIZE);

      priv->rxpkt[rxindex] = newpkt[i];

      if (i == 0)
        {
          /* The Ethernet header is in front of the data of the packet */

          iob->io_len    = seglen - ETH_HDRLEN;
          iob->io_pktlen = len - ETH_HDRLEN;
          head           = iob;
          tail           = iob;
        }
      else if (seglen > 0)
        {
          iob->io_offset  = CONFIG_NET_LL_GUARDSIZE - ETH_HDRLEN;
          iob->io_len     = seglen;
          tail->io_flink  = iob;
          tail            = iob;
        }
      else
        {
          netpkt_free(dev, iob, NETPKT_RX);
        }

      remain -= seglen;
      rxdesc  = stm32_get_next_rxdesc(priv, rxdesc);
    }

#ifdef CONFIG_STM32_ETH_TIMESTAMP_RX
  stm32_rxtimestamp(priv, rxlast, head);
#endif

  /* Give the descriptors with the new buffers back to the DMA */

  stm32_freesegment(priv, rxfirst, segments);
  return head;
}

/****************************************************************************
 * Function: stm32_rxtimestamp
 *
 * Description:
 *   Get the hardware timestamp of a received frame.  The timestamp is in
 *   the context descriptor that the DMA writes right after the last
 *   descriptor of the frame.  A frame without a timestamp gets a time of
 *   zero.  The scan of the descriptors gives the context descriptor back to
 *   the DMA.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *   last - The last descriptor of the frame
 *   pkt  - The packet of the frame
 *
 * Assumptions:
 *   The descriptor ring lock is held.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ETH_TIMESTAMP_RX
static void stm32_rxtimestamp(struct stm32_ethmac_s *priv,
                              struct eth_desc_s *last, netpkt_t *pkt)
{
  struct eth_desc_s *ctxdesc;
  int i;

  pkt->io_time.tv_sec  = 0;
  pkt->io_time.tv_nsec = 0;

  if ((last->des3 & ETH_RDES3_WB_RS1V) == 0 ||
      (last->des1 & ETH_RDES1_WB_TSA) == 0)
    {
      return;
    }

  ctxdesc = stm32_get_next_rxdesc(priv, last);

  for (i = 0; i < STM32_PTP_CTX_USTIMEOUT; i++)
    {
      up_invalidate_dcache((uintptr_t)ctxdesc,
                           (uintptr_t)ctxdesc + sizeof(struct eth_desc_s));

      if ((ctxdesc->des3 & ETH_RDES3_WB_OWN) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if ((ctxdesc->des3 & ETH_RDES3_WB_OWN) != 0 ||
      (ctxdesc->des3 & ETH_RDES3_WB_CTXT) == 0)
    {
      return;
    }

  /* The nanoseconds are in the first word and the seconds in the second,
   * and all ones is a timestamp that is not valid.
   */

  if (ctxdesc->des0 != UINT32_MAX || ctxdesc->des1 != UINT32_MAX)
    {
      pkt->io_time.tv_sec  = ctxdesc->des1;
      pkt->io_time.tv_nsec = ctxdesc->des0;
    }
}
#endif /* CONFIG_STM32_ETH_TIMESTAMP_RX */

/****************************************************************************
 * Function: stm32_txtstamp_flush
 *
 * Description:
 *   Deliver pending TX hardware timestamp loopback packets to the network
 *   stack.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called without the network device lock, from stm32_receive().
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
static void stm32_txtstamp_flush(struct stm32_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev.netdev;
  struct iob_s *saved_iob;
  uint16_t saved_len;
  struct iob_s *iob;

  for (; ; )
    {
      nxmutex_lock(&priv->lock);
      iob = iob_remove_queue(&priv->txtstampq);
      nxmutex_unlock(&priv->lock);

      if (iob == NULL)
        {
          break;
        }

      /* The TX path uses d_iob under the device lock too */

      netdev_lock(dev);
      saved_iob = dev->d_iob;
      saved_len = dev->d_len;
      dev->d_iob = iob;
      dev->d_len = iob->io_pktlen;
#ifdef CONFIG_NET_PKT
      pkt_input(dev);
#endif
      dev->d_iob = saved_iob;
      dev->d_len = saved_len;
      netdev_unlock(dev);

      iob->io_conn = NULL;
      iob_free_chain(iob);
    }
}
#endif

/****************************************************************************
 * Function: stm32_receive
 *
 * Description:
 *   NuttX Callback: take the next received frame from the RX descriptor
 *   ring.
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Parameters:
 *   dev - Reference to the lower half driver state structure
 *
 * Returned Value:
 *   The received packet, or NULL if there is none.
 *
 ****************************************************************************/

static netpkt_t *stm32_receive(struct netdev_lowerhalf_s *dev)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev;
  struct eth_desc_s *rxdesc;
  netpkt_t *pkt = NULL;
  int i;

#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
  /* Return any TX hardware timestamp loopback packets first */

  stm32_txtstamp_flush(priv);
#endif

  nxmutex_lock(&priv->lock);

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  /* Scan descriptors owned by the CPU.  Scan until:
   *
   *   1) We find a descriptor still owned by the DMA,
   *   2) We have examined all of the RX descriptors, or
   *   3) We have a frame.
   */

  rxdesc = priv->rxhead;

  /* Forces the first RX descriptor to be re-read from physical memory */

  up_invalidate_dcache((uintptr_t)rxdesc,
                       (uintptr_t)rxdesc + sizeof(struct eth_desc_s));

  for (i = 0;
       priv->ifup && pkt == NULL &&
       (rxdesc->des3 & ETH_RDES3_WB_OWN) == 0 &&
       i < STM32_ETH_NRXBUF;
       i++)
    {
      struct eth_desc_s *next = stm32_get_next_rxdesc(priv, rxdesc);

      if ((rxdesc->des3 & ETH_RDES3_WB_CTXT) != 0)
        {
          /* Drop the context descriptors, we are not interested */

          stm32_freesegment(priv, rxdesc, 1);
        }

      /* Check if this is the first segment in the frame */

      else if ((rxdesc->des3 & ETH_RDES3_WB_FD) == 0 &&
               priv->rxcurr == NULL)
        {
          /* A segment without its first segment, drop it */

          stm32_freesegment(priv, rxdesc, 1);
        }
      else
        {
          if ((rxdesc->des3 & ETH_RDES3_WB_FD) != 0)
            {
              if (priv->rxcurr != NULL)
                {
                  /* Drop the incomplete previous frame */

                  stm32_freesegment(priv, priv->rxcurr, priv->segments);
                }

              priv->rxcurr   = rxdesc;
              priv->segments = 1;
            }
          else
            {
              priv->segments++;
            }

          if ((rxdesc->des3 & ETH_RDES3_WB_LD) == 0)
            {
              /* The frame is longer than the whole ring, drop it */

              if (priv->segments >= STM32_ETH_NRXBUF)
                {
                  stm32_freesegment(priv, priv->rxcurr, priv->segments);
                }
            }
          else
            {
              /* A dribble bit error alone also sets the error summary,
               * but the CRC of the frame is good: keep such frames.
               */

              bool err = ((rxdesc->des3 & ETH_RDES3_WB_ES) != 0 &&
                          (rxdesc->des3 & RDES3_FATAL_ERRORS) != 0);

              ninfo("rxhead: %p rxcurr: %p segments: %d\n",
                    priv->rxhead, priv->rxcurr, priv->segments);

#ifdef CONFIG_STM32_ETH_HWCHECKSUM
              /* Check if any errors are reported in the frame.
               * If hardware checksum is enabled, check if:
               * - RDES1 is valid
               * - IP checksum was not bypassed
               * - IP hdr checksum error or IP payload checksum error is set
               *
               * By the previous checks the Last Descriptor is set.
               * RS1V is valid.
               */

              if (!err && (rxdesc->des3 & ETH_RDES3_WB_RS1V) != 0 &&
                  (rxdesc->des1 & ETH_RDES1_WB_IPCB) == 0 &&
                  (rxdesc->des1 & (ETH_RDES1_WB_IPHE |
                                   ETH_RDES1_WB_IPCE)) != 0)
                {
                  err = true;
                }
#endif

              if (!err)
                {
                  pkt = stm32_rxframe(priv, priv->rxcurr, rxdesc,
                                      priv->segments);
                }
              else
                {
                  /* Drop the frame that contains the errors, reset the
                   * segment scanning logic, and continue scanning with the
                   * next frame.
                   */

                  nwarn("WARNING: DROPPED RX descriptor errors: "
                        "%08" PRIx32 "\n",
                        rxdesc->des3);
                  NETDEV_RXERRORS(&priv->dev.netdev);
                  stm32_freesegment(priv, priv->rxcurr, priv->segments);
                }
            }
        }

      /* Try the next descriptor */

      rxdesc = next;

      /* Force the next RX descriptor to be re-read from physical memory */

      up_invalidate_dcache((uintptr_t)rxdesc,
                           (uintptr_t)rxdesc + sizeof(struct eth_desc_s));
    }

  /* Remember where we left off */

  priv->rxhead = rxdesc;

  ninfo("rxhead: %p rxcurr: %p segments: %d pkt: %p\n",
        priv->rxhead, priv->rxcurr, priv->segments, pkt);

  nxmutex_unlock(&priv->lock);
  return pkt;
}

/****************************************************************************
 * Function: stm32_freeframe
 *
 * Description:
 *   Scans the TX descriptors and frees the packets of completed TX
 *   transfers.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   The number of frames freed.
 *
 * Assumptions:
 *   The descriptor ring lock is held.
 *
 ****************************************************************************/

static int stm32_freeframe(struct stm32_ethmac_s *priv)
{
  struct eth_desc_s *txdesc;
  uint32_t des3_tmp;
#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
  uint32_t des0_tmp;
  uint32_t des1_tmp;
#endif
  int freed = 0;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txdesc = priv->txtail;
  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

      /* Force re-reading of the TX descriptor for physical memory */

      up_invalidate_dcache((uintptr_t)txdesc,
                           (uintptr_t)txdesc + sizeof(struct eth_desc_s));

      while ((txdesc->des3 & ETH_TDES3_RD_OWN) == 0)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          ninfo("txtail: %p des0: %08" PRIx32
                " des2: %08" PRIx32 " des3: %08" PRIx32 "\n",
                txdesc, txdesc->des0, txdesc->des2, txdesc->des3);

          DEBUGASSERT(txdesc->des0 != 0);

#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
          des0_tmp = txdesc->des0;
          des1_tmp = txdesc->des1;
#endif

          /* In any event, make sure that des0-3 are nullified. */

          txdesc->des0 = 0;
          txdesc->des1 = 0;
          txdesc->des2 = 0;
          des3_tmp = txdesc->des3;
          txdesc->des3 = 0;

          /* Flush the contents of the modified TX descriptor into
           * physical memory.
           */

          up_clean_dcache((uintptr_t)txdesc,
                          (uintptr_t)txdesc + sizeof(struct eth_desc_s));

          /* Check if this was the last segment of a TX frame */

          if ((des3_tmp & ETH_TDES3_RD_LD) != 0)
            {
              int tail = (union stm32_desc_u *)txdesc -
                         (union stm32_desc_u *)priv->txchbase;
#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
              struct iob_s *clone = priv->txmeta[tail];

              /* The MAC wrote the timestamp over the buffer address, and
               * the seconds over the second word.  All ones is not valid.
               */

              priv->txmeta[tail] = NULL;
              if (clone != NULL)
                {
                  if ((des3_tmp & ETH_TDES3_WB_TTSS) != 0 &&
                      (des0_tmp != UINT32_MAX || des1_tmp != UINT32_MAX))
                    {
                      clone->io_time.tv_sec  = des1_tmp;
                      clone->io_time.tv_nsec = des0_tmp;
                      iob_add_queue(clone, &priv->txtstampq);
                    }
                  else
                    {
                      iob_free_chain(clone);
                    }
                }
#endif

              /* Yes.. Free the packet of the frame */

              netpkt_free(&priv->dev, priv->txpkt[tail], NETPKT_TX);
              priv->txpkt[tail] = NULL;
              freed++;

              /* Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return freed;
                }
            }

          /* Try the next descriptor in the TX chain */

          txdesc = stm32_get_next_txdesc(priv, txdesc);

          /* Force re-reading of the TX descriptor for physical memory */

          up_invalidate_dcache((uintptr_t)txdesc,
                               (uintptr_t)txdesc +
                               sizeof(struct eth_desc_s));
        }

      /* We get here if (1) there are still frames "in-flight". Remember
       * where we left off.
       */

      priv->txtail = txdesc;

      ninfo("txhead: %p txtail: %p inflight: %d\n",
            priv->txhead, priv->txtail, priv->inflight);
    }

  return freed;
}

/****************************************************************************
 * Function: stm32_txreap
 *
 * Description:
 *   Free the packets of completed transfers.  The TX timeout is cancelled
 *   and the TX interrupts are disabled when no transfer is pending.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   The number of frames freed.
 *
 ****************************************************************************/

static int stm32_txreap(struct stm32_ethmac_s *priv)
{
  int freed;

  nxmutex_lock(&priv->lock);

  freed = stm32_freeframe(priv);

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      /* Cancel the TX timeout */

      wd_cancel(&priv->txtimeout);

      /* And disable further TX interrupts. */

      stm32_disableint(priv, ETH_DMACIER_TIE);
    }

  nxmutex_unlock(&priv->lock);
  return freed;
}

/****************************************************************************
 * Function: stm32_txpoll
 *
 * Description:
 *   NuttX Callback: free the packets of completed transfers for netpoll.
 *   It must not block, so it skips the work when the lock is busy: the
 *   holder frees the packets then.
 *
 * Input Parameters:
 *   dev - Reference to the lower half driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NETPOLL
static void stm32_txpoll(struct netdev_lowerhalf_s *dev)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev;

  if (nxmutex_trylock(&priv->lock) == OK)
    {
      stm32_freeframe(priv);
      nxmutex_unlock(&priv->lock);
    }
}
#endif

/****************************************************************************
 * Function: stm32_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_txdone(struct stm32_ethmac_s *priv)
{
  /* Scan the TX descriptor chain, then let the network send again */

  if (stm32_txreap(priv) > 0)
    {
      netdev_lower_txdone(&priv->dev);
    }

#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
  /* stm32_receive() delivers the TX timestamps */

  if (!IOB_QEMPTY(&priv->txtstampq))
    {
      netdev_lower_rxready(&priv->dev);
    }
#endif
}

/****************************************************************************
 * Function: stm32_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void stm32_interrupt_work(void *arg)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)arg;
  uint32_t dmasr;

  DEBUGASSERT(priv);

  /* Process pending Ethernet interrupts */

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dmasr = stm32_getreg(STM32_ETH_DMACSR);

  /* Mask only enabled interrupts.  This depends on the fact that the
   * interrupt related bits (0-16) correspond in these two registers.
   */

  dmasr &= stm32_getreg(STM32_ETH_DMACIER);

  /* Check if there are pending "normal" interrupts */

  if ((dmasr & ETH_DMACSR_NIS) != 0)
    {
      /* Yes.. Check if we received an incoming packet, if so, let the
       * upper half take it
       */

      if ((dmasr & ETH_DMACSR_RI) != 0)
        {
          /* Clear the pending receive interrupt */

          stm32_putreg(ETH_DMACSR_RI, STM32_ETH_DMACSR);

          /* Handle the received package */

          netdev_lower_rxready(&priv->dev);
        }

      /* Check if a packet transmission just completed.  If so, call
       * stm32_txdone(). This may disable further TX interrupts if there
       * are no pending transmissions.
       */

      if ((dmasr & ETH_DMACSR_TI) != 0)
        {
          /* Clear the pending receive interrupt */

          stm32_putreg(ETH_DMACSR_TI, STM32_ETH_DMACSR);

          /* Check if there are pending transmissions */

          stm32_txdone(priv);
        }

      /* Clear the pending normal summary interrupt */

      stm32_putreg(ETH_DMACSR_NIS, STM32_ETH_DMACSR);
    }

  /* Check if there are pending "abnormal" interrupts */

  if ((dmasr & ETH_DMACSR_AIS) != 0)
    {
      /* Just let the user know what happened */

      nerr("ERROR: Abnormal event(s): %08" PRIx32 "\n", dmasr);

      /* Clear all pending abnormal events */

      stm32_putreg(ETH_DMAINT_ABNORMAL, STM32_ETH_DMACSR);

      /* Clear the pending abnormal summary interrupt */

      stm32_putreg(ETH_DMACSR_AIS, STM32_ETH_DMACSR);

      /* In case of any error that stops the DMA, reset the MAC. */

      if (dmasr & (ETH_DMACIER_CDEE | ETH_DMACSR_FBE |
          ETH_DMACSR_RPS | ETH_DMACSR_TPS))
        {
          /* As per the datasheet's recommendation, the MAC
           * needs to be reset for all fatal errors. The
           * scheduled job will take the interface down and
           * up again.
           */

          work_queue(ETHWORK, &priv->irqwork, stm32_txtimeout_work, priv, 0);

          /* Interrupts need to remain disabled, no other
           * processing will take place. After reset
           * everything will be restored.
           */

          return;
        }
    }

  /* Re-enable Ethernet interrupts at the NVIC */

  up_enable_irq(STM32_IRQ_ETH);
}

/****************************************************************************
 * Function: stm32_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_interrupt(int irq, void *context, void *arg)
{
  struct stm32_ethmac_s *priv = &g_stm32ethmac[0];
  uint32_t dmasr;

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dmasr = stm32_getreg(STM32_ETH_DMACSR);
  if (dmasr != 0)
    {
      /* Disable further Ethernet interrupts.  Because Ethernet interrupts
       * are also disabled if the TX timeout event occurs, there can be no
       * race condition here.
       */

      up_disable_irq(STM32_IRQ_ETH);

      /* Check if a packet transmission just completed. */

      if ((dmasr & ETH_DMACSR_TI) != 0)
        {
          /* If a TX transfer just completed, then cancel the TX timeout so
           * there will be no race condition between any subsequent timeout
           * expiration and the deferred interrupt processing.
           */

          wd_cancel(&priv->txtimeout);
        }

      DEBUGASSERT(work_available(&priv->irqwork));

      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(ETHWORK, &priv->irqwork, stm32_interrupt_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: stm32_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void stm32_txtimeout_work(void *arg)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)arg;

  /* Reset the hardware.  Just take the interface down, then back up again.
   * The TX packets are freed, the RX packets are kept for the descriptors.
   */

  NETDEV_TXTIMEOUTS(&priv->dev.netdev);
  if (priv->ifup)
    {
      stm32_down(priv, false);
      if (stm32_ifup(&priv->dev) < 0)
        {
          nerr("ERROR: Failed to bring the interface up again\n");
        }

      /* Then let the network send again */

      netdev_lower_txdone(&priv->dev);
    }
}

/****************************************************************************
 * Function: stm32_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void stm32_txtimeout_expiry(wdparm_t arg)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)arg;

  nerr("ERROR: Timeout!\n");

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   *
   * Interrupts will be re-enabled when stm32_ifup() is called.
   */

  up_disable_irq(STM32_IRQ_ETH);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  DEBUGASSERT(work_available(&priv->irqwork));

  work_queue(ETHWORK, &priv->irqwork, stm32_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: stm32_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ifup(struct netdev_lowerhalf_s *lower)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)lower;
  struct net_driver_s *dev = &priv->dev.netdev;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->d_ipaddr), ip4_addr2(dev->d_ipaddr),
        ip4_addr3(dev->d_ipaddr), ip4_addr4(dev->d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Configure the Ethernet interface for DMA operation. */

  nxmutex_lock(&priv->lock);
  ret = stm32_ethconfig(priv);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  up_enable_irq(STM32_IRQ_ETH);
  nxmutex_unlock(&priv->lock);

  stm32_checksetup();
  netdev_lower_carrier_on(lower);
  return OK;
}

/****************************************************************************
 * Function: stm32_down
 *
 * Description:
 *   Reset the hardware and free the TX packets.  The RX packets are freed
 *   only if freerx is true, so that a reset does not need free IOBs to
 *   bring the interface up again.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   freerx - Free the RX packets too
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static int stm32_down(struct stm32_ethmac_s *priv, bool freerx)
{
  struct netdev_lowerhalf_s *lower = &priv->dev;
  irqstate_t flags;
  int i;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  nxmutex_lock(&priv->lock);
  flags = enter_critical_section();
  up_disable_irq(STM32_IRQ_ETH);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the stm32_ifup() always
   * successfully brings the interface back up.
   */

  stm32_ethreset(priv);

#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
  /* Drop the frames that were waiting for their transmit timestamp */

  iob_free_queue(&priv->txtstampq);
  for (i = 0; i < STM32_ETH_NTXBUF; i++)
    {
      if (priv->txmeta[i] != NULL)
        {
          iob_free_chain(priv->txmeta[i]);
          priv->txmeta[i] = NULL;
        }
    }
#endif

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);

  /* Free the packets held by the descriptors */

  for (i = 0; i < STM32_ETH_NTXBUF; i++)
    {
      if (priv->txpkt[i] != NULL)
        {
          netpkt_free(lower, priv->txpkt[i], NETPKT_TX);
          priv->txpkt[i] = NULL;
        }
    }

  for (i = 0; freerx && i < STM32_ETH_NRXBUF; i++)
    {
      if (priv->rxpkt[i] != NULL)
        {
          netpkt_free(lower, priv->rxpkt[i], NETPKT_RX);
          priv->rxpkt[i] = NULL;
        }
    }

  nxmutex_unlock(&priv->lock);

  netdev_lower_carrier_off(lower);
  return OK;
}

/****************************************************************************
 * Function: stm32_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   lower - Reference to the lower half driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ifdown(struct netdev_lowerhalf_s *lower)
{
  return stm32_down((struct stm32_ethmac_s *)lower, true);
}

/****************************************************************************
 * Function: stm32_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used by STM32 to check an ethernet frame
 *
 * Parameters:
 *   data   - the data to be checked
 *   length - length of the data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static uint32_t stm32_calcethcrc(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  size_t i;
  int j;

  for (i = 0; i < length; i++)
    {
      for (j = 0; j < 8; j++)
        {
          if (((crc >> 31) ^ (data[i] >> j)) & 0x01)
            {
              /* x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1 */

              crc = (crc << 1) ^ 0x04c11db7;
            }
          else
            {
              crc = crc << 1;
            }
        }
    }

  return ~crc;
}
#endif

/****************************************************************************
 * Function: stm32_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int stm32_addmac(struct netdev_lowerhalf_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast hash table */

  crc = stm32_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = STM32_ETH_MACHT1R;
      hashindex -= 32;
    }
  else
    {
      registeraddress = STM32_ETH_MACHT0R;
    }

  temp  = stm32_getreg(registeraddress);
  temp |= 1 << hashindex;
  stm32_putreg(temp, registeraddress);

  temp  = stm32_getreg(STM32_ETH_MACPFR);
  temp |= (ETH_MACPFR_HMC | ETH_MACPFR_HPF);
  stm32_putreg(temp, STM32_ETH_MACPFR);

  return OK;
}
#endif /* CONFIG_NET_IGMP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: stm32_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int stm32_rmmac(struct netdev_lowerhalf_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Remove the MAC address to the hardware multicast hash table */

  crc = stm32_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = STM32_ETH_MACHT0R;
      hashindex -= 32;
    }
  else
    {
      registeraddress = STM32_ETH_MACHT1R;
    }

  temp = stm32_getreg(registeraddress);
  temp &= ~(1 << hashindex);
  stm32_putreg(temp, registeraddress);

  /* If there is no address registered any more, delete multicast filtering */

  if (stm32_getreg(STM32_ETH_MACHT0R) == 0 &&
      stm32_getreg(STM32_ETH_MACHT1R) == 0)
    {
      temp = stm32_getreg(STM32_ETH_MACPFR);
      temp &= ~(ETH_MACPFR_HMC | ETH_MACPFR_HPF);
      stm32_putreg(temp, STM32_ETH_MACPFR);
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: stm32_txdescinit
 *
 * Description:
 *   Initializes the DMA TX descriptors in chain mode.
 *
 * Parameters:
 *   priv     - Reference to the driver state structure
 *   txtable  - List of pre-allocated TX descriptors for the Ethernet
 *              interface
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_txdescinit(struct stm32_ethmac_s *priv,
                             union stm32_desc_u *txtable)
{
  struct eth_desc_s *txdesc;
  int i;

  /* priv->txhead will point to the first, available TX descriptor in the
   * chain.  Set the priv->txhead pointer to the first descriptor in the
   * table.
   */

  priv->txhead = &txtable[0].desc;
  priv->txchbase = &txtable[0].desc;

  /* priv->txtail will point to the first segment of the oldest pending
   * "in-flight" TX transfer.  NULL means that there are no active TX
   * transfers.
   */

  priv->txtail   = NULL;
  priv->inflight = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < STM32_ETH_NTXBUF; i++)
    {
      txdesc = &txtable[i].desc;

#ifdef CHECKSUM_BY_HARDWARE
#if 0
      /* Enable the checksum insertion for the TX frames TODO! */

      txdesc->des0 |= ETH_TDES0_CIC_ALL;
#endif
#endif

      /* Clear Buffer1 address pointer (buffers will be assigned as they
       * are used)
       */

      txdesc->des0 = 0;

      /* Clear the rest of the descriptor as well */

      txdesc->des1 = 0;
      txdesc->des2 = 0;
      txdesc->des3 = 0;
    }

  /* Flush all of the initialized TX descriptors to physical memory */

  up_clean_dcache((uintptr_t)txtable,
                  (uintptr_t)txtable +
                  TXTABLE_SIZE * sizeof(union stm32_desc_u));

  /* Set Channel Tx descriptor ring length register
   * TODO: Why -1 is needed? Without this the ring doesn't wrap around
   * properly but the DMACCATXDR advances to outside the descriptor ring
   */

  stm32_putreg(STM32_ETH_NTXBUF - 1, STM32_ETH_DMACTXRLR);

  /* Set Transmit Descriptor List Address Register */

  stm32_putreg((uint32_t)&txtable[0].desc, STM32_ETH_DMACTXDLAR);

  /* Set Transmit Descriptor Tail pointer */

  stm32_putreg((uint32_t)&txtable[0].desc, STM32_ETH_DMACTXDTPR);
}

/****************************************************************************
 * Function: stm32_rxdescinit
 *
 * Description:
 *   Initializes the DMA RX descriptors in chain mode.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   rxtable  - List of pre-allocated RX descriptors for the Ethernet
 *              interface
 *
 * Returned Value:
 *   Zero on success; -ENOMEM if no IOB is free.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_rxdescinit(struct stm32_ethmac_s *priv,
                            union stm32_desc_u *rxtable)
{
  struct eth_desc_s *rxdesc;
  int i;

  /* priv->rxhead will point to the first,  RX descriptor in the chain.
   * This will be where we receive the first incomplete frame.
   */

  priv->rxhead = &rxtable[0].desc;
  priv->rxchbase = &rxtable[0].desc;

  /* If we accumulate the frame in segments, priv->rxcurr points to the
   * RX descriptor of the first segment in the current TX frame.
   */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Initialize each RX descriptor */

  for (i = 0; i < STM32_ETH_NRXBUF; i++)
    {
      rxdesc = &rxtable[i].desc;

      /* Each descriptor owns one packet */

      if (priv->rxpkt[i] == NULL)
        {
          priv->rxpkt[i] = netpkt_alloc(&priv->dev, NETPKT_RX);
          if (priv->rxpkt[i] == NULL)
            {
              nerr("ERROR: No RX buffers\n");
              return -ENOMEM;
            }
        }

      /* Make sure that no dirty cache line is written over the DMA data */

      up_invalidate_dcache((uintptr_t)netpkt_getdata(&priv->dev,
                                                     priv->rxpkt[i]),
                           (uintptr_t)netpkt_getdata(&priv->dev,
                                                     priv->rxpkt[i]) +
                           STM32_ETH_BUFSIZE);

      /* Set Buffer1 address pointer */

      rxdesc->des0 = (uint32_t)netpkt_getdata(&priv->dev, priv->rxpkt[i]);

      /* Set Buffer1 address high bytes */

      rxdesc->des1 = 0;

      /* Set Buffer2 address high bytes */

      rxdesc->des2 = 0;

      /* Set Own bit of the RX descriptor des3 and buffer address valid */

      rxdesc->des3 = ETH_RDES3_RD_OWN | ETH_RDES3_RD_IOC |
                     ETH_RDES3_RD_BUF1V;
    }

  /* Flush all of the initialized RX descriptors to physical memory */

  up_clean_dcache((uintptr_t)rxtable,
                  (uintptr_t)rxtable +
                  RXTABLE_SIZE * sizeof(union stm32_desc_u));

  /* Set Receive Descriptor ring length register
   * TODO: Why -1 is needed? Without this the ring doesn't wrap around
   * properly but the DMACCARXDR advances to outside the descriptor ring
   */

  stm32_putreg(STM32_ETH_NRXBUF - 1, STM32_ETH_DMACRXRLR);

  /* Set Receive Descriptor List Address Register */

  stm32_putreg((uint32_t)&rxtable[0].desc, STM32_ETH_DMACRXDLAR);

  /* Set Receive Descriptor Tail pointer Address */

  stm32_putreg((uint32_t)&rxtable[STM32_ETH_NRXBUF - 1].desc,
               STM32_ETH_DMACRXDTPR);
  return OK;
}

/****************************************************************************
 * Function: stm32_ioctl
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
 * Parameters:
 *   dev - Ethernet device structure
 *   cmd - SIOCxMIIxxx command code
 *   arg - Request structure also used to return values
 *
 * Returned Value: Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_PHY_IOCTL
static int stm32_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                       unsigned long arg)
{
#ifndef CONFIG_STM32_NO_PHY
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev;
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          struct mii_iotcl_notify_s *req =
            (struct mii_iotcl_notify_s *)((uintptr_t)arg);

          ret = phy_notify_subscribe(dev->netdev.d_ifname, req->pid,
                                     req->signo,
                                     req->arg);
          if (ret == OK)
            {
              /* Enable PHY link up/down interrupts */

              ret = stm32_phyintenable(priv);
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);

          req->phy_id = CONFIG_STM32_PHYADDR;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);

          ret = mdio_read(priv->mdio,
            req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);

          ret = mdio_write(priv->mdio,
            req->phy_id, req->reg_num, req->val_in);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
#else
  return -EIO;
#endif
}
#endif /* CONFIG_NETDEV_PHY_IOCTL */

#ifndef CONFIG_STM32_NO_PHY
/****************************************************************************
 * Function: stm32_phyintenable
 *
 * Description:
 *  Enable link up/down PHY interrupts.  The interrupt protocol is like this:
 *
 *  - Interrupt status is cleared when the interrupt is enabled.
 *  - Interrupt occurs.  Interrupt is disabled (at the processor level) when
 *    is received.
 *  - Interrupt status is cleared when the interrupt is re-enabled.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno (-ETIMEDOUT) on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int stm32_phyintenable(struct stm32_ethmac_s *priv)
{
#warning Missing logic
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Function: stm32_dm9161
 *
 * Description:
 *   Special workaround for the Davicom DM9161 PHY is required.  On power,
 *   up, the PHY is not usually configured correctly but will work after
 *   a powered-up reset.  This is really a workaround for some more
 *   fundamental issue with the PHY clocking initialization, but the
 *   root cause has not been studied (nor will it be with this workaround).
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ETH0_PHY_DM9161
static inline int stm32_dm9161(struct stm32_ethmac_s *priv)
{
  uint16_t phyval;
  int ret;

  /* Read the PHYID1 register;  A failure to read the PHY ID is one
   * indication that check if the DM9161 PHY CHIP is not ready.
   */

  ret = mdio_read(priv->mdio,
    CONFIG_STM32_PHYADDR, MII_PHYID1, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read the PHY ID1: %d\n", ret);
      return ret;
    }

  /* If we failed to read the PHY ID1 register, the reset the MCU to
   * recover
   */

  else if (phyval == 0xffff)
    {
      up_systemreset();
    }

  ninfo("PHY ID1: 0x%04X\n", phyval);

  /* Now check the "DAVICOM Specified Configuration Register (DSCR)",
   * Register 16
   */

  ret = mdio_read(priv->mdio,
    CONFIG_STM32_PHYADDR, 16, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read the PHY Register 0x10: %d\n", ret);
      return ret;
    }

  /* Bit 8 of the DSCR register is zero, then the DM9161 has not selected
   * RMII.  If RMII is not selected, then reset the MCU to recover.
   */

  else if ((phyval & (1 << 8)) == 0)
    {
      up_systemreset();
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: stm32_phyregdump
 *
 * Description:
 *  Dump the MII registers 0-31
 *
 * Parameters:
 *
 * Returned Value:
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ETHMAC_REGDEBUG
static void stm32_phyregdump(struct stm32_ethmac_s *priv)
{
  uint16_t phyval;
  int ret;
  int i;

  for (i = 0; i < 0x20; i++)
    {
      ret = mdio_read(priv->mdio,
        CONFIG_STM32_PHYADDR, i, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read reg: 0%2x\n", i);
        }
      else
        {
          ninfo("Phy reg 0x%02x == 0x%x\n", i, phyval);
        }
    }
}
#endif

/****************************************************************************
 * Function: stm32_phylib_init
 *
 * Description:
 *   Hand the PHY over to PHYLIB: create and connect the PHY device on the
 *   first call and start its state machine, then wait for the link and
 *   take its speed and duplex mode.  The PHY driver, registered by the
 *   board, configures the PHY for the interface mode.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PHYLIB
static int stm32_phylib_init(struct stm32_ethmac_s *priv)
{
  struct phy_device_s *phydev = priv->phydev;
  uint32_t speed = 0;
  bool link = false;
  bool fduplex = false;
  int ret;
  int to;

  if (phydev == NULL)
    {
      phydev = phy_device_create(priv->mdio, CONFIG_STM32_PHYADDR);
      if (phydev == NULL)
        {
          nerr("ERROR: No PHY at address %d\n", CONFIG_STM32_PHYADDR);
          return -ENODEV;
        }

      phydev->interface = BOARD_ETH_PHY_INTERFACE;

      ret = phy_connect(phydev, NULL);
      if (ret < 0)
        {
          nerr("ERROR: Failed to connect the PHY: %d\n", ret);
          phy_device_free(phydev);
          return ret;
        }

      ret = phy_start(phydev);
      if (ret < 0)
        {
          nerr("ERROR: Failed to start the PHY: %d\n", ret);
          phy_disconnect(phydev);
          return ret;
        }

      priv->phydev = phydev;
    }

  /* The PHY keeps running across ifdown, so that its clocks let the next
   * MAC reset complete.  Wait for its state machine to report the link.
   */

  for (to = 0; to < PHYLIB_LINK_TIMEOUT_MS; to += PHY_POLL_MS)
    {
      nxmutex_lock(&phydev->lock);
      link    = phydev->link;
      speed   = phydev->speed;
      fduplex = phydev->duplex == DUPLEX_FULL;
      nxmutex_unlock(&phydev->lock);

      if (link)
        {
          break;
        }

      nxsched_msleep(PHY_POLL_MS);
    }

  if (!link)
    {
      nerr("ERROR: Timed out waiting for the link\n");
      return -ETIMEDOUT;
    }

  priv->fduplex = fduplex;
  priv->mbps100 = speed == 100;
  priv->gbit    = speed == 1000;

  syslog(LOG_INFO, "stm32_eth: link %s-duplex %d Mbps\n",
         priv->fduplex ? "full" : "half",
         priv->gbit ? 1000 : priv->mbps100 ? 100 : 10);

  return OK;
}
#endif

/****************************************************************************
 * Function: stm32_phyinit
 *
 * Description:
 *  Configure the PHY and determine the link speed/duplex.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_phyinit(struct stm32_ethmac_s *priv)
{
#ifndef CONFIG_NET_PHYLIB
#ifdef CONFIG_STM32_AUTONEG
  volatile uint32_t timeout;
#endif
  uint16_t phyval;
  int ret;
  int to;
#endif
  uint32_t regval;

  /* Assume 10MBps and half duplex */

  priv->mbps100 = 0;
  priv->gbit    = 0;
  priv->fduplex = 0;

  /* Setup up PHY clocking by setting the CR field in the MACMDIOAR reg */

  regval  = stm32_getreg(STM32_ETH_MACMDIOAR);
  regval &= ~ETH_MACMDIOAR_CR_MASK;
  regval |= ETH_MACMDIOAR_CR;
  stm32_putreg(regval, STM32_ETH_MACMDIOAR);

#ifdef CONFIG_NET_PHYLIB
  return stm32_phylib_init(priv);
#else
  /* Put the PHY in reset mode */

  ret = mdio_write(priv->mdio,
    CONFIG_STM32_PHYADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: Failed to reset the PHY: %d\n", ret);
      return ret;
    }

  to = PHY_RESET_DELAY;
  do
    {
      up_mdelay(10);
      to -= 10;
      phyval = 0xffff;
      ret = mdio_read(priv->mdio,
        CONFIG_STM32_PHYADDR, MII_MCR, &phyval);

      ninfo("MII_MCR: phyval: %u ret: %d\n", phyval, ret);
    }
  while (phyval & MII_MCR_RESET && to > 0);

  if (to <= 0)
    {
      nerr("ERROR: Phy reset timeout\n");
      return -ETIMEDOUT;
    }
  else
    {
      ninfo("Phy reset in %d ms\n", PHY_RESET_DELAY - to);
    }

#ifdef STM32_PHYID1
  /* Check the identifier of the PHY, if the configuration names one */

  ret = mdio_read(priv->mdio,
    CONFIG_STM32_PHYADDR, MII_PHYID1, &phyval);

  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID1: %d\n", ret);
      return ret;
    }

  if (phyval != STM32_PHYID1)
    {
      nerr("ERROR: Incorrect PHYID1: %u expected: %u\n",
            phyval, STM32_PHYID1);
      return -ENXIO;
    }

  ninfo("MII_PHYID1: phyval: %u ret: %d\n", phyval, ret);

  ret = mdio_read(priv->mdio,
    CONFIG_STM32_PHYADDR, MII_PHYID2, &phyval);

  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID2: %d\n", ret);
      return ret;
    }

  if ((phyval & 0xfff0) != (STM32_PHYID2 & 0xfff0))
    {
      nerr("ERROR: Incorrect PHYID2: %u expected: %u\n",
            phyval, STM32_PHYID2);
      return -ENXIO;
    }

  ninfo("MII_PHYID2: phyval: %u ret: %d\n", phyval, ret);
#endif

#ifdef CONFIG_STM32_ETHMAC_REGDEBUG
  stm32_phyregdump(priv);
#endif

  /* Special workaround for the Davicom DM9161 PHY is required. */

#ifdef CONFIG_ETH0_PHY_DM9161
  ret = stm32_dm9161(priv);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Perform auto-negotiation if so configured */

#ifdef CONFIG_STM32_AUTONEG
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = mdio_read(priv->mdio,
        CONFIG_STM32_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_LINKSTATUS) != 0)
        {
          ninfo("MII_MSR: phyval: %u ret: %d \n", phyval, ret);
          break;
        }

      nxsched_msleep(PHY_POLL_MS);
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for link status: %04x\n", phyval);
      return -ETIMEDOUT;
    }

#ifdef CONFIG_STM32_AUTONEG_10FD_ONLY
  /* Advertise only 10BASE-T full duplex, so that negotiation lands
   * there on both ends.  See the help text of the option for why a
   * board would want a slower link on purpose.
   */

  ret = mdio_write(priv->mdio, CONFIG_STM32_PHYADDR, MII_ADVERTISE,
                   MII_ADVERTISE_10BASETXFULL | MII_ADVERTISE_CSMA);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write the PHY ANAR: %d\n", ret);
      return ret;
    }
#endif

  /* Enable and restart auto-negotiation */

  ret = mdio_write(priv->mdio,
    CONFIG_STM32_PHYADDR, MII_MCR,
    MII_MCR_ANENABLE | MII_MCR_ANRESTART);
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = mdio_read(priv->mdio,
        CONFIG_STM32_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_ANEGCOMPLETE) != 0)
        {
          break;
        }

      nxsched_msleep(PHY_POLL_MS);
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for auto-negotiation\n");
      return -ETIMEDOUT;
    }

  /* Read the result of the auto-negotiation from the PHY-specific register */

  ret = mdio_read(priv->mdio,
    CONFIG_STM32_PHYADDR, CONFIG_STM32_PHYSR, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  ninfo("PHYSR[%d]: %04x\n", CONFIG_STM32_PHYSR, phyval);

  /* Different PHYs present speed and mode information in different ways.  IF
   * This CONFIG_STM32_PHYSR_ALTCONFIG is selected, this indicates that
   * the PHY represents speed and mode information are combined, for
   * example, with separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_STM32_PHYSR_ALTCONFIG
  switch (phyval & CONFIG_STM32_PHYSR_ALTMODE)
    {
      default:
        nerr("ERROR: Unrecognized PHY status setting\n");

      /* Falls through */

      case CONFIG_STM32_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_STM32_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_STM32_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_STM32_PHYSR_100FD:
        priv->fduplex = 1;
        priv->mbps100 = 1;
        break;
    }

  /* Different PHYs present speed and mode information in different ways.
   * Some will present separate information for speed and mode (this is the
   * default).  Those PHYs, for example, may provide a 10/100 Mbps
   * indication and a separate full/half duplex indication.
   */

#else
  if ((phyval & CONFIG_STM32_PHYSR_MODE) ==
      CONFIG_STM32_PHYSR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_STM32_PHYSR_SPEED) == CONFIG_STM32_PHYSR_100MBPS)
    {
      priv->mbps100 = 1;
    }
#endif

#else /* Auto-negotiation not selected */

  phyval = 0;
#ifdef CONFIG_STM32_ETHFD
  phyval |= MII_MCR_FULLDPLX;
#endif
#ifdef CONFIG_STM32_ETH100MBPS
  phyval |= MII_MCR_SPEED100;
#endif

  /* mdio_write, not stm32_phywrite:  this driver never had the latter.
   * The call was carried over from the F7 driver and nothing had ever
   * built this path.
   */

  ret = mdio_write(priv->mdio, CONFIG_STM32_PHYADDR, MII_MCR, phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes */

#ifdef CONFIG_STM32_ETHFD
  priv->fduplex = 1;
#endif
#ifdef CONFIG_STM32_ETH100MBPS
  priv->mbps100 = 1;
#endif
#endif

  /* Diagnostic: say what was negotiated even without net debug.  A
   * duplex mismatch looks exactly like a bad cable and nothing else
   * says which of the two it is.
   */

  syslog(LOG_INFO, "stm32_eth: link %s-duplex %d Mbps\n",
         priv->fduplex ? "full" : "half",
         priv->gbit ? 1000 : priv->mbps100 ? 100 : 10);

  return OK;
#endif /* CONFIG_NET_PHYLIB */
}

#endif

/****************************************************************************
 * Name: stm32_selectmii
 *
 * Description:
 *   Selects the MII interface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_MII
static inline void stm32_selectmii(void)
{
  uint32_t regval;

  regval  = getreg32(STM32_ETH_PHYIF_REG);
  regval &= ~STM32_ETH_PHYIF_MASK;
  regval |= STM32_ETH_PHYIF_MII;
  putreg32(regval, STM32_ETH_PHYIF_REG);
}
#endif

/****************************************************************************
 * Name: stm32_selectrmii
 *
 * Description:
 *   Selects the RMII interface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_RMII
static inline void stm32_selectrmii(void)
{
  uint32_t regval;

  regval  = getreg32(STM32_ETH_PHYIF_REG);
  regval &= ~STM32_ETH_PHYIF_MASK;
  regval |= STM32_ETH_PHYIF_RMII;
  putreg32(regval, STM32_ETH_PHYIF_REG);
}
#endif

/****************************************************************************
 * Name: stm32_selectrgmii
 *
 * Description:
 *   Selects the RGMII interface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_RGMII
static inline void stm32_selectrgmii(void)
{
  uint32_t regval;

  regval  = getreg32(STM32_ETH_PHYIF_REG);
  regval &= ~STM32_ETH_PHYIF_MASK;
  regval |= STM32_ETH_PHYIF_RGMII;
  putreg32(regval, STM32_ETH_PHYIF_REG);
}
#endif

/****************************************************************************
 * Function: stm32_ethgpioconfig
 *
 * Description:
 *  Configure GPIOs for the Ethernet interface.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void stm32_ethgpioconfig(struct stm32_ethmac_s *priv)
{
  /* Configure GPIO pins to support Ethernet */

#if defined(CONFIG_STM32_MII) || defined(CONFIG_STM32_RMII) || \
    defined(CONFIG_STM32_RGMII)

  /* MDC and MDIO are common to all modes */
# ifndef CONFIG_STM32_NO_PHY
#   ifdef CONFIG_STM32_ETH_MDIO_GPIO
  stm32_configgpio(GPIO_ETH_MDC_GPIO);
  stm32_configgpio(GPIO_ETH_MDIO_GPIO_OUT);
#   else
  stm32_configgpio(GPIO_ETH_MDC);
  stm32_configgpio(GPIO_ETH_MDIO);
#   endif
# endif

  /* Set up the MII interface */

#  if defined(CONFIG_STM32_MII)

  /* Select the MII interface */

  stm32_selectmii();

  /* Provide clocking via MCO, MCO1 or MCO2:
   *
   * "MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or
   *  PLL clock (through a configurable prescaler) on PA8 pin."
   *
   * "MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or
   *  PLLI2S clock (through a configurable prescaler) on PC9 pin."
   */

#    if defined(CONFIG_STM32_MII_MCO1)
  /* Configure MC01 to drive the PHY.  Board logic must provide MC01 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO1);
  stm32_mco1config(BOARD_CFGR_MC01_SOURCE, BOARD_CFGR_MC01_DIVIDER);

#    elif defined(CONFIG_STM32_MII_MCO2)
  /* Configure MC02 to drive the PHY.  Board logic must provide MC02 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO2);
  stm32_mco2config(BOARD_CFGR_MC02_SOURCE, BOARD_CFGR_MC02_DIVIDER);

#    elif defined(CONFIG_STM32_MII_MCO)
  /* Setup MCO pin for alternative usage */

  stm32_configgpio(GPIO_MCO);
  stm32_mcoconfig(BOARD_CFGR_MCO_SOURCE);
#    endif

  /* MII interface pins (17):
   *
   * MII_TX_CLK, MII_TXD[3:0], MII_TX_EN, MII_RX_CLK, MII_RXD[3:0],
   * MII_RX_ER, MII_RX_DV, MII_CRS, MII_COL, MDC, MDIO
   */

  stm32_configgpio(GPIO_ETH_MII_COL);
  stm32_configgpio(GPIO_ETH_MII_CRS);
  stm32_configgpio(GPIO_ETH_MII_RXD0);
  stm32_configgpio(GPIO_ETH_MII_RXD1);
  stm32_configgpio(GPIO_ETH_MII_RXD2);
  stm32_configgpio(GPIO_ETH_MII_RXD3);
  stm32_configgpio(GPIO_ETH_MII_RX_CLK);
  stm32_configgpio(GPIO_ETH_MII_RX_DV);
  stm32_configgpio(GPIO_ETH_MII_RX_ER);
  stm32_configgpio(GPIO_ETH_MII_TXD0);
  stm32_configgpio(GPIO_ETH_MII_TXD1);
  stm32_configgpio(GPIO_ETH_MII_TXD2);
  stm32_configgpio(GPIO_ETH_MII_TXD3);
  stm32_configgpio(GPIO_ETH_MII_TX_CLK);
  stm32_configgpio(GPIO_ETH_MII_TX_EN);

  /* Set up the RMII interface. */

#  elif defined(CONFIG_STM32_RMII)

  /* Select the RMII interface */

  stm32_selectrmii();

  /* Provide clocking via MCO, MCO1 or MCO2:
   *
   * "MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or
   *  PLL clock (through a configurable prescaler) on PA8 pin."
   *
   * "MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or
   *  PLLI2S clock (through a configurable prescaler) on PC9 pin."
   */

#    if defined(CONFIG_STM32_RMII_MCO1)
  /* Configure MC01 to drive the PHY.  Board logic must provide MC01 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO1);
  stm32_mco1config(BOARD_CFGR_MC01_SOURCE, BOARD_CFGR_MC01_DIVIDER);

#    elif defined(CONFIG_STM32_RMII_MCO2)
  /* Configure MC02 to drive the PHY.  Board logic must provide MC02 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO2);
  stm32_mco2config(BOARD_CFGR_MC02_SOURCE, BOARD_CFGR_MC02_DIVIDER);

#    elif defined(CONFIG_STM32_RMII_MCO)
  /* Setup MCO pin for alternative usage */

  stm32_configgpio(GPIO_MCO);
  stm32_mcoconfig(BOARD_CFGR_MCO_SOURCE);
#    endif

  /* RMII interface pins (7):
   *
   * RMII_TXD[1:0], RMII_TX_EN, RMII_RXD[1:0], RMII_CRS_DV, MDC, MDIO,
   * RMII_REF_CLK
   */

  stm32_configgpio(GPIO_ETH_RMII_CRS_DV);
  stm32_configgpio(GPIO_ETH_RMII_REF_CLK);
  stm32_configgpio(GPIO_ETH_RMII_RXD0);
  stm32_configgpio(GPIO_ETH_RMII_RXD1);
  stm32_configgpio(GPIO_ETH_RMII_TXD0);
  stm32_configgpio(GPIO_ETH_RMII_TXD1);
  stm32_configgpio(GPIO_ETH_RMII_TX_EN);

  /* Set up the RGMII interface. */

#  elif defined(CONFIG_STM32_RGMII)

  /* Select the RGMII interface */

  stm32_selectrgmii();

  /* RGMII interface pins (12):
   *
   * RGMII_GTX_CLK, RGMII_TXD[3:0], RGMII_TX_CTL, RGMII_RX_CLK,
   * RGMII_RXD[3:0], RGMII_RX_CTL, and the 125 MHz reference clock from the
   * PHY if the board uses one.
   */

  stm32_configgpio(GPIO_ETH_RGMII_GTX_CLK);
  stm32_configgpio(GPIO_ETH_RGMII_TXD0);
  stm32_configgpio(GPIO_ETH_RGMII_TXD1);
  stm32_configgpio(GPIO_ETH_RGMII_TXD2);
  stm32_configgpio(GPIO_ETH_RGMII_TXD3);
  stm32_configgpio(GPIO_ETH_RGMII_TX_CTL);
  stm32_configgpio(GPIO_ETH_RGMII_RX_CLK);
  stm32_configgpio(GPIO_ETH_RGMII_RXD0);
  stm32_configgpio(GPIO_ETH_RGMII_RXD1);
  stm32_configgpio(GPIO_ETH_RGMII_RXD2);
  stm32_configgpio(GPIO_ETH_RGMII_RXD3);
  stm32_configgpio(GPIO_ETH_RGMII_RX_CTL);
#    ifdef GPIO_ETH_RGMII_CLK125
  stm32_configgpio(GPIO_ETH_RGMII_CLK125);
#    endif
#  endif
#endif

#ifdef CONFIG_STM32_ETH_PTP_GPIO
  /* Enable pulse-per-second (PPS) output signal */

  stm32_configgpio(GPIO_ETH_PPS_OUT);
#endif
}

#ifdef CONFIG_STM32_ETH_PTP
/****************************************************************************
 * Name: stm32_eth_ptp_wait
 *
 * Description:
 *   Wait for the timestamp unit to clear a bit that is set while it takes
 *   an update of the addend, of the system time or of the PPS target time.
 *
 * Input Parameters:
 *   reg - The register that holds the bit
 *   bit - The bit to wait for
 *
 * Returned Value:
 *   OK on success, -ETIMEDOUT if the bit was not cleared in time.
 *
 ****************************************************************************/

static int stm32_eth_ptp_wait(uint32_t reg, uint32_t bit)
{
  int timeout;

  for (timeout = 0; timeout < STM32_PTP_UPDATE_USTIMEOUT; timeout++)
    {
      if ((stm32_getreg(reg) & bit) == 0)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

#if defined(CONFIG_STM32_ETH_PTP_GPIO) || defined(CONFIG_PTP_CLOCK)
/****************************************************************************
 * Name: stm32_eth_ptp_read
 *
 * Description:
 *   Read the system time. The seconds and the nanoseconds are in two
 *   registers, so the seconds are read again to detect a rollover between
 *   the reads.
 *
 * Input Parameters:
 *   sec  - The location to store the seconds
 *   nsec - The location to store the nanoseconds
 *
 ****************************************************************************/

static void stm32_eth_ptp_read(uint32_t *sec, uint32_t *nsec)
{
  uint32_t sec1;
  uint32_t sec2;
  uint32_t ns;

  sec1 = stm32_getreg(STM32_ETH_MACSTSR);
  ns   = stm32_getreg(STM32_ETH_MACSTNR);
  sec2 = stm32_getreg(STM32_ETH_MACSTSR);

  if (sec1 != sec2)
    {
      ns = stm32_getreg(STM32_ETH_MACSTNR);
    }

  *sec  = sec2;
  *nsec = ns & ETH_MACSTNR_TSSS_MASK;
}
#endif

/****************************************************************************
 * Name: stm32_eth_ptp_setaddend
 *
 * Description:
 *   Load the addend, that sets the rate of the system time.
 *
 * Input Parameters:
 *   addend - The value to load
 *
 * Returned Value:
 *   OK on success, -ETIMEDOUT if the unit did not take it in time.
 *
 ****************************************************************************/

static int stm32_eth_ptp_setaddend(uint32_t addend)
{
  stm32_putreg(addend, STM32_ETH_MACTSAR);
  stm32_putreg(stm32_getreg(STM32_ETH_MACTSCR) | ETH_MACTSCR_TSADDREG,
               STM32_ETH_MACTSCR);
  return stm32_eth_ptp_wait(STM32_ETH_MACTSCR, ETH_MACTSCR_TSADDREG);
}

#ifdef CONFIG_STM32_ETH_PTP_GPIO
/****************************************************************************
 * Name: stm32_eth_ptp_pps_start
 *
 * Description:
 *   Start the pulse-per-second output, a pulse train with a period of one
 *   second and a duty cycle of 50%, aligned to the system time. The fixed
 *   frequency mode of the MAC gives a pulse too short to be seen, so the
 *   flexible mode is used.
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_eth_ptp_pps_start(void)
{
  uint32_t sec;
  uint32_t nsec;
  int ret;

  /* The first pulse is at the next whole second, or at the one after it
   * if that is too close to be loaded in time.
   */

  stm32_eth_ptp_read(&sec, &nsec);
  sec += (nsec < NSEC_PER_SEC - STM32_PTP_PPS_MARGIN_NS) ? 1 : 2;

  stm32_putreg(sec, STM32_ETH_MACPPSTTSR);
  stm32_putreg(0, STM32_ETH_MACPPSTTNR);

  ret = stm32_eth_ptp_wait(STM32_ETH_MACPPSTTNR, ETH_MACPPSTTNR_TRGTBUSY0);
  if (ret < 0)
    {
      nerr("ERROR: Timed out loading the PPS target time\n");
      return ret;
    }

  stm32_putreg(STM32_PTP_PPS_INTERVAL, STM32_ETH_MACPPSIR);
  stm32_putreg(STM32_PTP_PPS_WIDTH, STM32_ETH_MACPPSWR);
  stm32_putreg(ETH_MACPPSCR_PPSEN0 | ETH_MACPPSCR_PPSCMD_START_TRAIN,
               STM32_ETH_MACPPSCR);
  return OK;
}

#ifdef CONFIG_PTP_CLOCK
/****************************************************************************
 * Name: stm32_eth_ptp_pps_restart
 *
 * Description:
 *   Align the pulse-per-second output to the whole seconds of the system
 *   time again. The pulse train counts by itself, and does not follow a
 *   step of the system time, so after a step the pulses would be displaced
 *   by the same amount.
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_eth_ptp_pps_restart(void)
{
  int ret;

  stm32_putreg(ETH_MACPPSCR_PPSEN0 | ETH_MACPPSCR_PPSCMD_STOP_NOW,
               STM32_ETH_MACPPSCR);
  ret = stm32_eth_ptp_wait(STM32_ETH_MACPPSCR, ETH_MACPPSCR_PPSCTRL_MASK);
  if (ret < 0)
    {
      nerr("ERROR: Timed out stopping the PPS output\n");
      return ret;
    }

  return stm32_eth_ptp_pps_start();
}
#endif /* CONFIG_PTP_CLOCK */
#endif /* CONFIG_STM32_ETH_PTP_GPIO */

/****************************************************************************
 * Name: stm32_eth_ptp_init
 *
 * Description:
 *   Start the system time of the timestamp unit of the MAC, at zero and at
 *   its nominal rate. The unit needs to be enabled before the addend and
 *   the system time can be loaded. It does not depend on the link, but it
 *   needs the MAC to be out of reset, and a reset of the MAC clears it.
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_eth_ptp_init(void)
{
  uint32_t tscr;
  int ret;

  /* Enable the unit, with the fine update method (the addend sets the
   * rate) and a digital rollover (the nanoseconds count from 0 to
   * 999999999).
   */

  tscr = ETH_MACTSCR_TSENA | ETH_MACTSCR_TSCFUPDT | ETH_MACTSCR_TSCTRLSSR;

#ifdef CONFIG_STM32_ETH_TIMESTAMP_RX
  /* Timestamp the received PTP version 2 messages, over Ethernet and over
   * UDP, except for the announce, management and signaling messages.
   */

  tscr |= ETH_MACTSCR_TSVER2ENA | ETH_MACTSCR_TSIPENA |
          ETH_MACTSCR_TSIPV4ENA | ETH_MACTSCR_TSIPV6ENA |
          ETH_MACTSCR_SNAPTYPSEL_1;
#endif
  stm32_putreg(tscr, STM32_ETH_MACTSCR);

  /* Set the increment of the system time */

  stm32_putreg(STM32_PTP_SSINC << ETH_MACSSIR_SSINC_SHIFT,
               STM32_ETH_MACSSIR);

  /* Load the addend for the nominal rate */

  ret = stm32_eth_ptp_setaddend(STM32_PTP_ADDEND);
  if (ret < 0)
    {
      nerr("ERROR: Timed out loading the PTP addend\n");
      return ret;
    }

  /* Initialize the system time to zero */

  stm32_putreg(0, STM32_ETH_MACSTSUR);
  stm32_putreg(0, STM32_ETH_MACSTNUR);
  stm32_putreg(tscr | ETH_MACTSCR_TSINIT, STM32_ETH_MACTSCR);
  ret = stm32_eth_ptp_wait(STM32_ETH_MACTSCR, ETH_MACTSCR_TSINIT);
  if (ret < 0)
    {
      nerr("ERROR: Timed out initializing the PTP system time\n");
      return ret;
    }

#ifdef CONFIG_STM32_ETH_PTP_GPIO
  ret = stm32_eth_ptp_pps_start();
  if (ret < 0)
    {
      return ret;
    }
#endif

  ninfo("PTP: system time started, SSINC=%u addend=%" PRIu32 "\n",
        (unsigned int)STM32_PTP_SSINC, (uint32_t)STM32_PTP_ADDEND);
  return OK;
}

#ifdef CONFIG_PTP_CLOCK
/****************************************************************************
 * Name: stm32_ptp_adjfine
 *
 * Description:
 *   Adjust the frequency of the system time.
 *
 * Input Parameters:
 *   lower - The PTP clock lower half (unused)
 *   ppb   - The offset from the nominal frequency, in parts per billion.
 *           Positive makes the system time run faster.
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_ptp_adjfine(struct ptp_lowerhalf_s *lower, long ppb)
{
  irqstate_t flags;
  int64_t addend;
  int ret;

  addend = STM32_PTP_ADDEND;
  addend += addend * ppb / NSEC_PER_SEC;

  if (addend <= 0 || addend > UINT32_MAX)
    {
      nerr("ERROR: PTP adjustment out of range: %ld ppb\n", ppb);
      return -EINVAL;
    }

  flags = enter_critical_section();
  ret = stm32_eth_ptp_setaddend((uint32_t)addend);
  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: stm32_ptp_adjtime
 *
 * Description:
 *   Step the system time by a signed amount, without changing its rate.
 *
 * Input Parameters:
 *   lower - The PTP clock lower half (unused)
 *   delta - The amount to add, in nanoseconds
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_ptp_adjtime(struct ptp_lowerhalf_s *lower, int64_t delta)
{
  irqstate_t flags;
  uint64_t abs_ns;
  uint64_t sec;
  uint32_t nsec;
  uint32_t nsreg;
  int ret;

  abs_ns = delta < 0 ? (uint64_t)0 - (uint64_t)delta : (uint64_t)delta;
  sec    = abs_ns / NSEC_PER_SEC;
  nsec   = abs_ns % NSEC_PER_SEC;

  if (sec > UINT32_MAX)
    {
      return -EINVAL;
    }

  if (delta < 0)
    {
      /* To subtract, the seconds register holds the negated seconds and
       * the nanoseconds register 10^9 minus the nanoseconds.
       */

      sec   = (uint32_t)(0 - (uint32_t)sec);
      nsreg = ETH_MACSTNUR_ADDSUB | (NSEC_PER_SEC - nsec);
    }
  else
    {
      nsreg = nsec;
    }

  flags = enter_critical_section();
  stm32_putreg((uint32_t)sec, STM32_ETH_MACSTSUR);
  stm32_putreg(nsreg, STM32_ETH_MACSTNUR);
  stm32_putreg(stm32_getreg(STM32_ETH_MACTSCR) | ETH_MACTSCR_TSUPDT,
               STM32_ETH_MACTSCR);
  ret = stm32_eth_ptp_wait(STM32_ETH_MACTSCR, ETH_MACTSCR_TSUPDT);
  if (ret < 0)
    {
      nerr("ERROR: Timed out stepping the PTP system time\n");
    }
#ifdef CONFIG_STM32_ETH_PTP_GPIO
  else
    {
      ret = stm32_eth_ptp_pps_restart();
    }
#endif

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: stm32_ptp_adjphase
 *
 * Description:
 *   Step the system time by a signed amount, in nanoseconds.
 *
 ****************************************************************************/

static int stm32_ptp_adjphase(struct ptp_lowerhalf_s *lower, int32_t phase)
{
  return stm32_ptp_adjtime(lower, phase);
}

/****************************************************************************
 * Name: stm32_ptp_gettime
 *
 * Description:
 *   Read the system time. The system clock timestamps are not supported.
 *
 ****************************************************************************/

static int stm32_ptp_gettime(struct ptp_lowerhalf_s *lower,
                             struct timespec *ts,
                             struct ptp_system_timestamp *sts)
{
  uint32_t sec;
  uint32_t nsec;

  stm32_eth_ptp_read(&sec, &nsec);

  ts->tv_sec  = sec;
  ts->tv_nsec = nsec;
  return OK;
}

/****************************************************************************
 * Name: stm32_ptp_settime
 *
 * Description:
 *   Set the system time.
 *
 * Input Parameters:
 *   lower - The PTP clock lower half (unused)
 *   ts    - The time to set. The seconds must fit in 32 bits.
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_ptp_settime(struct ptp_lowerhalf_s *lower,
                             const struct timespec *ts)
{
  irqstate_t flags;
  int ret;

  if (ts->tv_sec < 0 || ts->tv_sec > UINT32_MAX ||
      ts->tv_nsec < 0 || ts->tv_nsec >= NSEC_PER_SEC)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  stm32_putreg((uint32_t)ts->tv_sec, STM32_ETH_MACSTSUR);
  stm32_putreg((uint32_t)ts->tv_nsec, STM32_ETH_MACSTNUR);
  stm32_putreg(stm32_getreg(STM32_ETH_MACTSCR) | ETH_MACTSCR_TSINIT,
               STM32_ETH_MACTSCR);
  ret = stm32_eth_ptp_wait(STM32_ETH_MACTSCR, ETH_MACTSCR_TSINIT);
  if (ret < 0)
    {
      nerr("ERROR: Timed out setting the PTP system time\n");
    }
#ifdef CONFIG_STM32_ETH_PTP_GPIO
  else
    {
      ret = stm32_eth_ptp_pps_restart();
    }
#endif

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: stm32_ptp_getres
 *
 * Description:
 *   Get the resolution of the system time, that is its increment.
 *
 ****************************************************************************/

static int stm32_ptp_getres(struct ptp_lowerhalf_s *lower,
                            struct timespec *res)
{
  res->tv_sec  = 0;
  res->tv_nsec = STM32_PTP_SSINC;
  return OK;
}

static const struct ptp_ops_s g_stm32_ptp_ops =
{
  stm32_ptp_adjfine,  /* adjfine */
  stm32_ptp_adjphase, /* adjphase */
  stm32_ptp_adjtime,  /* adjtime */
  stm32_ptp_gettime,  /* gettime */
  NULL,               /* getcrosststamp */
  stm32_ptp_settime,  /* settime */
  stm32_ptp_getres,   /* getres */
};
#endif /* CONFIG_PTP_CLOCK */
#endif /* CONFIG_STM32_ETH_PTP */

/****************************************************************************
 * Function: stm32_ethreset
 *
 * Description:
 *  Reset the Ethernet block.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; -ETIMEDOUT if the MAC software reset does not complete,
 *   for example without the PHY clocks.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ethreset(struct stm32_ethmac_s *priv)
{
  uint32_t regval;
  volatile uint32_t timeout;

  /* Reset the Ethernet on the AHB bus */

  regval  = stm32_getreg(STM32_RCC_ETH_RSTR);
  regval |= STM32_RCC_ETH_RST;
  stm32_putreg(regval, STM32_RCC_ETH_RSTR);

  regval &= ~STM32_RCC_ETH_RST;
  stm32_putreg(regval, STM32_RCC_ETH_RSTR);

  /* Read the reset register back, so that the MAC leaves the reset before
   * the software reset below starts.
   */

  stm32_getreg(STM32_RCC_ETH_RSTR);

  /* Perform a software reset by setting the SR bit in the DMAMR register.
   * This Resets all MAC subsystem internal registers and logic.  After this
   * reset all the registers holds their reset values.
   */

  regval  = stm32_getreg(STM32_ETH_DMAMR);
  regval |= ETH_DMAMR_SWR;
  stm32_putreg(regval, STM32_ETH_DMAMR);

  /* Wait for software reset to complete. The SR bit is cleared
   * automatically after the reset operation has completed in all of the
   * core clock domains.
   */

  timeout = MAC_READY_USTIMEOUT;
  while (timeout-- && (stm32_getreg(STM32_ETH_DMAMR) & ETH_DMAMR_SWR) != 0)
    {
      up_udelay(1);
    }

  if ((stm32_getreg(STM32_ETH_DMAMR) & ETH_DMAMR_SWR) != 0)
    {
      nerr("ERROR: MAC software reset timed out\n");
      return -ETIMEDOUT;
    }

  /* According to the spec, these need to be done before creating
   * the descriptor lists, so initialize these already here
   */

  /* Set up the DMAMR register */

  regval  = stm32_getreg(STM32_ETH_DMAMR);
  regval &= ~DMAMR_CLEAR_MASK;
  regval |= DMAMR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_DMAMR);

  /* Set up the DMASBMR register */

  regval  = stm32_getreg(STM32_ETH_DMASBMR);
  regval &= ~DMASBMR_CLEAR_MASK;
  regval |= DMASBMR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_DMASBMR);
  return OK;
}

/****************************************************************************
 * Function: stm32_eth_setqsize
 *
 * Description:
 *  Program the size of MTL queue 0 on MACs with several queues.  A MAC
 *  with a single queue is left as it is.
 *
 ****************************************************************************/

static void stm32_eth_setqsize(void)
{
  uint32_t hwf1 = stm32_getreg(STM32_ETH_MACHWF1R);
  uint32_t hwf2 = stm32_getreg(STM32_ETH_MACHWF2R);
  uint32_t regval;
  uint32_t nq;
  uint32_t size;

  nq = ((hwf2 & ETH_MACHWF2R_TXQCNT_MASK) >> ETH_MACHWF2R_TXQCNT_SHIFT) + 1;
  if (nq > 1)
    {
      size = (128 << ((hwf1 & ETH_MACHWF1R_TXFIFOSIZE_MASK) >>
                      ETH_MACHWF1R_TXFIFOSIZE_SHIFT)) / nq;
      regval  = stm32_getreg(STM32_ETH_MTLTXQOMR);
      regval &= ~ETH_MTLTXQOMR_TQS_MASK;
      regval |= (size / 256 - 1) << ETH_MTLTXQOMR_TQS_SHIFT;
      stm32_putreg(regval, STM32_ETH_MTLTXQOMR);
    }

  nq = ((hwf2 & ETH_MACHWF2R_RXQCNT_MASK) >> ETH_MACHWF2R_RXQCNT_SHIFT) + 1;
  if (nq > 1)
    {
      size = (128 << ((hwf1 & ETH_MACHWF1R_RXFIFOSIZE_MASK) >>
                      ETH_MACHWF1R_RXFIFOSIZE_SHIFT)) / nq;
      regval  = stm32_getreg(STM32_ETH_MTLRXQOMR);
      regval &= ~ETH_MTLRXQOMR_RQS_MASK;
      regval |= (size / 256 - 1) << ETH_MTLRXQOMR_RQS_SHIFT;
      stm32_putreg(regval, STM32_ETH_MTLRXQOMR);
    }
}

/****************************************************************************
 * Function: stm32_macconfig
 *
 * Description:
 *  Configure the Ethernet MAC for DMA operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_macconfig(struct stm32_ethmac_s *priv)
{
  uint32_t regval;

  /* DMA Configuration */

  /* Set up the ETH_DMACCR register */

  regval  = stm32_getreg(STM32_ETH_DMACCR);
  regval &= ~DMACCR_CLEAR_MASK;
  regval |= DMACCR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_DMACCR);

  /* Set up the ETH_DMACTXCR register */

  regval  = stm32_getreg(STM32_ETH_DMACTXCR);
  regval &= ~DMACTXCR_CLEAR_MASK;
  regval |= DMACTXCR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_DMACTXCR);

  /* Set up the ETH_DMACRXCR register */

  regval  = stm32_getreg(STM32_ETH_DMACRXCR);
  regval &= ~DMACRXCR_CLEAR_MASK;
  regval |= DMACRXCR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_DMACRXCR);

  /* MTL configuration */

  /* Set up the MTLTXQOMR register */

  regval  = stm32_getreg(STM32_ETH_MTLTXQOMR);
  regval &= ~MTLTXQOMR_CLEAR_MASK;
  regval |= MTLTXQOMR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_MTLTXQOMR);

  /* Set up the MTLRXQOMR register */

  regval  = stm32_getreg(STM32_ETH_MTLRXQOMR);
  regval &= ~MTLRXQOMR_CLEAR_MASK;
  regval |= MTLRXQOMR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_MTLRXQOMR);

  /* A MAC with more than one RX queue starts with all of them disabled.
   * Enable queue 0, that the MTL maps to DMA channel 0, for generic
   * traffic.
   */

  if ((stm32_getreg(STM32_ETH_MACHWF2R) & ETH_MACHWF2R_RXQCNT_MASK) != 0)
    {
      regval  = stm32_getreg(STM32_ETH_MACRXQC0R);
      regval &= ~ETH_MACRXQC0R_RXQ0EN_MASK;
      regval |= ETH_MACRXQC0R_RXQ0EN_GT;
      stm32_putreg(regval, STM32_ETH_MACRXQC0R);
    }

  /* On a MAC with several queues, the MTL FIFOs are shared by the queues
   * with 256 byte queues after reset.  In store-and-forward mode a queue
   * must hold a whole frame, so give queue 0 its share of each FIFO.
   */

  stm32_eth_setqsize();

  /* MAC Configuration */

  /* Set up the MACCR register */

  regval  = stm32_getreg(STM32_ETH_MACCR);
  regval &= ~MACCR_CLEAR_BITS;
  regval |= MACCR_SET_BITS;

  if (priv->fduplex)
    {
      /* Set the DM bit for full duplex support */

      regval |= ETH_MACCR_DM;
    }

  if (priv->mbps100)
    {
      /* Set the FES bit for 100Mbps fast ethernet support */

      regval |= ETH_MACCR_FES;
    }

#ifdef CONFIG_STM32_HAVE_ETH_RGMII
  if (!priv->gbit)
    {
      /* On a gigabit MAC, set the PS bit to select the 10/100 Mbps port,
       * also with MII and RMII.
       */

      regval |= ETH_MACCR_PS;
    }
#endif

  stm32_putreg(regval, STM32_ETH_MACCR);

  /* Set up the MACPFR register */

  regval  = stm32_getreg(STM32_ETH_MACPFR);
  regval &= ~MACPFR_CLEAR_BITS;
  regval |= MACPFR_SET_BITS;
  stm32_putreg(regval, STM32_ETH_MACPFR);

  /* Set up the MACHT0R and MACHT1R registers */

  stm32_putreg(0, STM32_ETH_MACHT0R);
  stm32_putreg(0, STM32_ETH_MACHT1R);

  /* Setup up the MACQTXFCR register */

  regval  = stm32_getreg(STM32_ETH_MACQTXFCR);
  regval &= ~MACQTXFCR_CLEAR_MASK;
  regval |= MACQTXFCR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_MACQTXFCR);

  /* Setup up the MACRXFCR register */

  regval  = stm32_getreg(STM32_ETH_MACRXFCR);
  regval &= ~MACRXFCR_CLEAR_MASK;
  regval |= MACRXFCR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_MACRXFCR);

  /* Setup up the MACVTR register */

  stm32_putreg(0, STM32_ETH_MACVTR);
  return OK;
}

/****************************************************************************
 * Function: stm32_macaddress
 *
 * Description:
 *   Configure the selected MAC address.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_macaddress(struct stm32_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev.netdev;
  uint32_t regval;

  ninfo("%s MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        dev->d_ifname,
        dev->d_mac.ether.ether_addr_octet[0],
        dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
        dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
        dev->d_mac.ether.ether_addr_octet[5]);

  /* Set the MAC address high register */

  regval = ((uint32_t)dev->d_mac.ether.ether_addr_octet[5] << 8) |
            (uint32_t)dev->d_mac.ether.ether_addr_octet[4];
  stm32_putreg(regval, STM32_ETH_MACA0HR);

  /* Set the MAC address low register */

  regval = ((uint32_t)dev->d_mac.ether.ether_addr_octet[3] << 24) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[2] << 16) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[1] <<  8) |
            (uint32_t)dev->d_mac.ether.ether_addr_octet[0];
  stm32_putreg(regval, STM32_ETH_MACA0LR);
}

/****************************************************************************
 * Function: stm32_macenable
 *
 * Description:
 *  Enable normal MAC operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_macenable(struct stm32_ethmac_s *priv)
{
  uint32_t regval;

  /* Set the MAC address */

  stm32_macaddress(priv);

  /* Enable transmit state machine of the MAC for transmission on the MII */

  regval  = stm32_getreg(STM32_ETH_MACCR);
  regval |= ETH_MACCR_TE;
  stm32_putreg(regval, STM32_ETH_MACCR);

  /* Flush Transmit FIFO */

  regval  = stm32_getreg(STM32_ETH_MTLTXQOMR);
  regval |= ETH_MTLTXQOMR_FTQ;
  stm32_putreg(regval, STM32_ETH_MTLTXQOMR);

  /* Enable receive state machine of the MAC for reception from the MII */

  /* Enables or disables the MAC reception. */

  regval  = stm32_getreg(STM32_ETH_MACCR);
  regval |= ETH_MACCR_RE;
  stm32_putreg(regval, STM32_ETH_MACCR);

  /* Start DMA transmission */

  regval  = stm32_getreg(STM32_ETH_DMACTXCR);
  regval |= ETH_DMACTXCR_ST;
  stm32_putreg(regval, STM32_ETH_DMACTXCR);

  /* Start DMA reception */

  regval  = stm32_getreg(STM32_ETH_DMACRXCR);
  regval |= ETH_DMACRXCR_SR;
  stm32_putreg(regval, STM32_ETH_DMACRXCR);

  /* Enable Ethernet MAC interrupts, except the one of the timestamp unit.
   * It is set each time the target time of the PPS output is reached and
   * is cleared by reading MACTSSR, which the interrupt handler does not,
   * so it would stay pending and keep the handler running.
   */

  stm32_putreg(ETH_MACIER_ALLINTS & ~ETH_MACIER_TSIE, STM32_ETH_MACIER);

  /* Ethernet DMA supports two classes of interrupts: Normal interrupt
   * summary (NIS) and Abnormal interrupt summary (AIS) with a variety
   * individual normal and abnormal interrupting events.  Here only
   * the normal receive event is enabled (unless DEBUG is enabled).  Transmit
   * events will only be enabled when a transmit interrupt is expected.
   */

  stm32_putreg((ETH_DMAINT_RECV_ENABLE | ETH_DMAINT_XMIT_ENABLE |
                ETH_DMAINT_ERROR_ENABLE), STM32_ETH_DMACIER);

  /* Clear Tx and Rx process stopped flags */

  regval  = stm32_getreg(STM32_ETH_DMACSR);
  regval |= (ETH_DMACSR_TPS | ETH_DMACSR_RPS);
  stm32_putreg(regval, STM32_ETH_DMACSR);

  return OK;
}

/****************************************************************************
 * Function: stm32_ethconfig
 *
 * Description:
 *  Configure the Ethernet interface for DMA operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ethconfig(struct stm32_ethmac_s *priv)
{
  int ret;

  /* NOTE: The Ethernet clocks were initialized early in the boot-up
   * sequence in stm32_rcc.c.
   */

#ifdef CONFIG_STM32_PHYINIT
  /* Perform any necessary, board-specific PHY initialization */

  ret = stm32_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Reset the Ethernet block */

  ninfo("Reset the Ethernet block\n");
  ret = stm32_ethreset(priv);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_STM32_ETH_PTP
  /* Start the system time of the timestamp unit. It does not depend on the
   * link, so it is started before the PHY, which fails to initialize when
   * there is no link.
   */

  ret = stm32_eth_ptp_init();
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Initialize TX Descriptors list */

  stm32_txdescinit(priv, &g_txtable[priv->intf * STM32_ETH_NTXBUF]);

  /* Initialize RX Descriptors list */

  ret = stm32_rxdescinit(priv, &g_rxtable[priv->intf * STM32_ETH_NRXBUF]);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the PHY */

#ifdef CONFIG_STM32_NO_PHY
  ninfo("MAC without PHY\n");
#ifdef CONFIG_STM32_ETHFD
  priv->fduplex = 1;
#else
  priv->fduplex = 0;
#endif
#ifdef CONFIG_STM32_ETH100MBPS
  priv->mbps100 = 1;
#else
  priv->mbps100 = 0;
#endif
#else
  ninfo("Initialize the PHY\n");
  ret = stm32_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

#endif
  /* Initialize the MAC and DMA */

  ninfo("Initialize the MAC and DMA\n");
  ret = stm32_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");
  return stm32_macenable(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the STM32 chip
 *   supports multiple Ethernet controllers, then board specific logic
 *   must implement arm_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if STM32_NETHERNET > 1 || defined(CONFIG_NETDEV_LATEINIT)
int stm32_ethinitialize(int intf)
#else
static inline int stm32_ethinitialize(int intf)
#endif
{
  struct stm32_ethmac_s *priv;
  uint8_t uid[12];
  uint64_t crc;
  int ret = OK;

  ninfo("intf: %d\n", intf);

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < STM32_NETHERNET);
  priv = &g_stm32ethmac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct stm32_ethmac_s));
  priv->dev.ops              = &g_stm32_ops;
  priv->dev.rxtype           = NETDEV_RX_DIRECT; /* From the IRQ work */
  priv->dev.priority         = ETHWORK;
  priv->intf                 = intf;    /* Remember the interface number */
  nxmutex_init(&priv->lock);

  stm32_get_uniqueid(uid);
  crc = crc64(uid, 12);

  /* Specify as locally administrated address */

  priv->dev.netdev.d_mac.ether.ether_addr_octet[0]  = (crc >> 0) | 0x02;
  priv->dev.netdev.d_mac.ether.ether_addr_octet[0] &= ~0x1;

  priv->dev.netdev.d_mac.ether.ether_addr_octet[1]  = crc >> 8;
  priv->dev.netdev.d_mac.ether.ether_addr_octet[2]  = crc >> 16;
  priv->dev.netdev.d_mac.ether.ether_addr_octet[3]  = crc >> 24;
  priv->dev.netdev.d_mac.ether.ether_addr_octet[4]  = crc >> 32;
  priv->dev.netdev.d_mac.ether.ether_addr_octet[5]  = crc >> 40;

  /* Configure GPIO pins to support Ethernet */

  stm32_ethgpioconfig(priv);

  /* Initialize the MDIO device */

#ifdef CONFIG_STM32_ETH_MDIO_GPIO
  priv->mdio = stm32_mdiogpio_bus_initialize(GPIO_ETH_MDC_GPIO,
                                             GPIO_ETH_MDIO_GPIO_OUT,
                                             GPIO_ETH_MDIO_GPIO_IN);
#else
  priv->mdio = stm32_mdio_bus_initialize();
#endif
  if (!priv->mdio)
    {
      nerr("ERROR: Failed to initialize MDIO bus\n");
      return -ENOMEM;
    }

  /* Attach the IRQ to the driver */

  if (irq_attach(STM32_IRQ_ETH, stm32_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

#ifdef CONFIG_STM32_PHYINIT
  /* Perform any necessary, board-specific PHY initialization */

  ret = stm32_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Put the EMAC in its reset, non-operational state. */

  stm32_ethreset(priv);

#ifdef CONFIG_STM32_ETH_TIMESTAMP_RX
  priv->dev.netdev.d_features |= NETDEV_RX_STAMP;
#endif

#ifdef CONFIG_STM32_ETH_TIMESTAMP_TX
  priv->dev.netdev.d_features |= NETDEV_TX_STAMP;
#endif

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_lower_register(&priv->dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      nerr("ERROR: netdev_lower_register failed: %d\n", ret);
      return ret;
    }

#if defined(CONFIG_STM32_ETH_PTP) && defined(CONFIG_PTP_CLOCK)
  /* Register the PTP hardware clock, /dev/ptp0 */

  priv->ptp_lower.ops = &g_stm32_ptp_ops;
  if (ptp_clock_register(&priv->ptp_lower, STM32_PTP_MAX_ADJ, intf) < 0)
    {
      nerr("ERROR: Failed to register the PTP clock\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c. If STM32_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of arm_netinitialize() that calls stm32_ethinitialize() with
 *   the appropriate interface number.
 *
 * Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if STM32_NETHERNET == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  stm32_ethinitialize(0);
}
#endif

#endif /* STM32_NETHERNET > 0 && CONFIG_STM32_ETHMAC */
