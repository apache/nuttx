/****************************************************************************
 * arch/arm/src/stm32h7/stm32_ethernet.c
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
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/crc64.h>

#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include <nuttx/cache.h>
#include "arm_internal.h"
#include "barriers.h"

#include "hardware/stm32_syscfg.h"
#include "hardware/stm32_pinmap.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "stm32_ethernet.h"
#include "stm32_uid.h"

#include <arch/board/board.h>

/* STM32H7_NETHERNET determines the number of physical interfaces that can
 * be supported by the hardware.  CONFIG_STM32H7_ETHMAC will defined if
 * any STM32H7 Ethernet support is enabled in the configuration.
 */

#if STM32H7_NETHERNET > 0 && defined(CONFIG_STM32H7_ETHMAC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory synchronization */

#define MEMORY_SYNC() do { ARM_DSB(); ARM_ISB(); } while (0)

/* Configuration ************************************************************/

/* See boards/arm/stm32/stm3240g-eval/README.txt for an explanation of the
 * configuration settings.
 */

#if STM32H7_NETHERNET > 1
#  error "Logic to support multiple Ethernet interfaces is incomplete"
#endif

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

/* Select work queue */

#  if defined(CONFIG_STM32H7_ETHMAC_HPWORK)
#    define ETHWORK HPWORK
#  elif defined(CONFIG_STM32H7_ETHMAC_LPWORK)
#    define ETHWORK LPWORK
#  else
#    define ETHWORK LPWORK
#  endif
#endif

#ifndef CONFIG_STM32H7_PHYADDR
#  error "CONFIG_STM32H7_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_STM32H7_MII) && !defined(CONFIG_STM32H7_RMII)
#  warning "Neither CONFIG_STM32H7_MII nor CONFIG_STM32H7_RMII defined"
#endif

#if defined(CONFIG_STM32H7_MII) && defined(CONFIG_STM32H7_RMII)
#  error "Both CONFIG_STM32H7_MII and CONFIG_STM32H7_RMII defined"
#endif

#ifdef CONFIG_STM32H7_MII
#  if !defined(CONFIG_STM32H7_MII_MCO1) && !defined(CONFIG_STM32H7_MII_MCO2) && \
      !defined(CONFIG_STM32H7_MII_EXTCLK)
#    warning "Neither CONFIG_STM32H7_MII_MCO1, CONFIG_STM32H7_MII_MCO2, nor CONFIG_STM32H7_MII_EXTCLK defined"
#  endif
#  if defined(CONFIG_STM32H7_MII_MCO1) && defined(CONFIG_STM32H7_MII_MCO2)
#    error "Both CONFIG_STM32H7_MII_MCO1 and CONFIG_STM32H7_MII_MCO2 defined"
#  endif
#endif

#ifdef CONFIG_STM32H7_RMII
#  if !defined(CONFIG_STM32H7_RMII_MCO1) && !defined(CONFIG_STM32H7_RMII_MCO2) && \
      !defined(CONFIG_STM32H7_RMII_EXTCLK)
#    warning "Neither CONFIG_STM32H7_RMII_MCO1, CONFIG_STM32H7_RMII_MCO2, nor CONFIG_STM32H7_RMII_EXTCLK defined"
#  endif
#  if defined(CONFIG_STM32H7_RMII_MCO1) && defined(CONFIG_STM32H7_RMII_MCO2)
#    error "Both CONFIG_STM32H7_RMII_MCO1 and CONFIG_STM32H7_RMII_MCO2 defined"
#  endif
#endif

#ifdef CONFIG_STM32H7_AUTONEG
#  ifndef CONFIG_STM32H7_PHYSR
#    error "CONFIG_STM32H7_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_STM32H7_PHYSR_ALTCONFIG
#    ifndef CONFIG_STM32H7_PHYSR_ALTMODE
#      error "CONFIG_STM32H7_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32H7_PHYSR_10HD
#      error "CONFIG_STM32H7_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32H7_PHYSR_100HD
#      error "CONFIG_STM32H7_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32H7_PHYSR_10FD
#      error "CONFIG_STM32H7_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32H7_PHYSR_100FD
#      error "CONFIG_STM32H7_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_STM32H7_PHYSR_SPEED
#      error "CONFIG_STM32H7_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32H7_PHYSR_100MBPS
#      error "CONFIG_STM32H7_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32H7_PHYSR_MODE
#      error "CONFIG_STM32H7_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32H7_PHYSR_FULLDUPLEX
#      error "CONFIG_STM32H7_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

#ifdef CONFIG_STM32H7_ETH_PTP
#  warning "CONFIG_STM32H7_ETH_PTP is not yet supported"
#endif

#undef CONFIG_STM32H7_ETH_HWCHECKSUM

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, or 16 bytes (depending on buswidth).  We
 * will use the 16-byte alignment in all cases.
 */

#define OPTIMAL_ETH_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 4 + 15) & ~15)

#ifdef CONFIG_STM32H7_ETH_BUFSIZE
#  define ETH_BUFSIZE CONFIG_STM32H7_ETH_BUFSIZE
#else
#  define ETH_BUFSIZE OPTIMAL_ETH_BUFSIZE
#endif

#if ETH_BUFSIZE > ETH_CTX_TDES2_MSS_MASK
#  error "ETH_BUFSIZE is too large"
#endif

#if (ETH_BUFSIZE & 15) != 0
#  error "ETH_BUFSIZE must be aligned"
#endif

#if ETH_BUFSIZE != OPTIMAL_ETH_BUFSIZE
#  warning "You are using an incomplete/untested configuration"
#endif

#ifndef CONFIG_STM32H7_ETH_NRXDESC
#  define CONFIG_STM32H7_ETH_NRXDESC 8
#endif
#ifndef CONFIG_STM32H7_ETH_NTXDESC
#  define CONFIG_STM32H7_ETH_NTXDESC 4
#endif

#ifndef min
#  define min(a,b) ((a) < (b) ? (a) : (b))
#endif

/* We need at least one more free buffer than transmit buffers */

#define STM32_ETH_NFREEBUFFERS (CONFIG_STM32H7_ETH_NTXDESC+1)

/* Buffers used for DMA access must begin on an address aligned with the
 * D-Cache line and must be an even multiple of the D-Cache line size.
 * These size/alignment requirements are necessary so that D-Cache flush
 * and invalidate operations will not have any additional effects.
 *
 * The TX and RX descriptors are 16 bytes in size
 */

#define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#define DMA_ALIGN_DOWN(n)  ((n) & ~DMA_BUFFER_MASK)

#define DESC_SIZE       16
#define DESC_PADSIZE        DMA_ALIGN_UP(DESC_SIZE)
#define ALIGNED_BUFSIZE     DMA_ALIGN_UP(ETH_BUFSIZE)

#define RXTABLE_SIZE        (STM32H7_NETHERNET * CONFIG_STM32H7_ETH_NRXDESC)
#define TXTABLE_SIZE        (STM32H7_NETHERNET * CONFIG_STM32H7_ETH_NTXDESC)

#define RXBUFFER_SIZE       (CONFIG_STM32H7_ETH_NRXDESC * ALIGNED_BUFSIZE)
#define RXBUFFER_ALLOC      (STM32H7_NETHERNET * RXBUFFER_SIZE)

#define TXBUFFER_SIZE       (STM32_ETH_NFREEBUFFERS * ALIGNED_BUFSIZE)
#define TXBUFFER_ALLOC      (STM32H7_NETHERNET * TXBUFFER_SIZE)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_NET_INFO
#  undef CONFIG_STM32H7_ETHMAC_REGDEBUG
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

/* TX timeout = 1 minute */

#define STM32_TXTIMEOUT   (60*CLK_TCK)

/* PHY reset/configuration delays in milliseconds */

#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT  (0x0004ffff)
#define PHY_WRITE_TIMEOUT (0x0004ffff)
#define PHY_RETRY_TIMEOUT (0x0001998)

/* MAC reset ready delays in loop counts */

#define MAC_READY_USTIMEOUT (200)

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

#define MACCR_CLEAR_BITS                                                \
  (ETH_MACCR_RE | ETH_MACCR_TE | ETH_MACCR_PRELEN_MASK | ETH_MACCR_DC | \
   ETH_MACCR_BL_MASK | ETH_MACCR_DR | ETH_MACCR_DCRS | ETH_MACCR_DO |   \
   ETH_MACCR_LM | ETH_MACCR_DM | ETH_MACCR_FES | ETH_MACCR_JE |         \
   ETH_MACCR_JD | ETH_MACCR_WD | ETH_MACCR_ACS | ETH_MACCR_CST |        \
   ETH_MACCR_S2KP | ETH_MACCR_GPSLCE | ETH_MACCR_IPG_MASK |             \
   ETH_MACCR_IPC | ETH_MACCR_SARC_MASK | ETH_MACCR_ARPEN)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACCR_RE    Receiver enable                0 (disabled)
 * ETH_MACCR_TE    Transmitter enable             0 (disabled)
 * ETH_MACCR_DC    Deferral check                 0 (disabled)
 * ETH_MACCR_BL    Back-off limit                 0 (10)
 * ETH_MACCR_ACS   Automatic pad/CRC stripping    0 (disabled)
 * ETH_MACCR_DR    Retry disable                  1 (disabled)
 * ETH_MACCR_IPC   IPv4 checksum offload
 *                 Depends on CONFIG_STM32H7_ETH_HWCHECKSUM
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
 */

#ifdef CONFIG_STM32H7_ETH_HWCHECKSUM
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
 * ETH_MTLTXQOMR_TQS      Bits 16-24: Transmit Queue Size
 */

#define MTLTXQOMR_CLEAR_MASK                            \
  (ETH_MTLTXQOMR_FTQ | ETH_MTLTXQOMR_TSF |              \
   ETH_MTLTXQOMR_TXQEN_MASK | ETH_MTLTXQOMR_TTC_MASK |  \
   ETH_MTLTXQOMR_TQS_MASK)

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
 * ETH_MTLRXQOMR_RQS_MASK    Bits 20-22: Receive Queue Size
 */

#define MTLRXQOMR_CLEAR_MASK                            \
  (ETH_MTLRXQOMR_RTC_MASK | ETH_MTLRXQOMR_FUP |         \
   ETH_MTLRXQOMR_FEP | ETH_MTLRXQOMR_RSF |              \
   ETH_MTLRXQOMR_DIS_TCP_EF | ETH_MTLRXQOMR_EHFC |      \
   ETH_MTLRXQOMR_RFA_MASK | ETH_MTLRXQOMR_RFD_MASK |    \
   ETH_MTLRXQOMR_RQS_MASK)

#define MTLTXQOMR_SET_MASK                                      \
  ((0xd << ETH_MTLTXQOMR_TQS_SHIFT) | ETH_MTLTXQOMR_TTC_64 |    \
   ETH_MTLTXQOMR_TXQEN_ENABLED | ETH_MTLTXQOMR_FTQ)

/* TODO: Check whether use flow control (bits RFD, RFA EHFC) */

#define MTLRXQOMR_SET_MASK                                      \
  ((0x7 << ETH_MTLRXQOMR_RQS_SHIFT) | ETH_MTLRXQOMR_RTC_64)

#ifdef CONFIG_STM32H7_ETH_HWCHECKSUM
/* TODO */

#  error CONFIG_STM32H7_ETH_HWCHECKSUM not supported
#endif

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

/* Set the descriptor skip length in 32-bit words */

#define DMACCR_SET_MASK                                 \
  (ETH_DMACCR_DSL((DESC_PADSIZE - DESC_SIZE)/4))

#define DMASBMR_CLEAR_MASK                              \
  (STM32_ETH_DMASBMR_FB | STM32_ETH_DMASBMR_AAL |       \
   STM32_ETH_DMASBMR_MB | STM32_ETH_DMASBMR_RB)

#define DMASBMR_SET_MASK                                \
  (STM32_ETH_DMASBMR_FB | STM32_ETH_DMASBMR_AAL)

#define DMACTXCR_CLEAR_MASK                                     \
  (ETH_DMACTXCR_ST | ETH_DMACTXCR_ST | ETH_DMACTXCR_OSF |       \
   ETH_DMACTXCR_TSE | ETH_DMACTXCR_TXPBL_MASK)

#define DMACTXCR_SET_MASK                       \
  (ETH_DMACTXCR_OSF | ETH_DMACTXCR_TXPBL(32))

#define DMACRXCR_CLEAR_MASK                     \
  (ETH_DMACRXCR_SR | ETH_DMACRXCR_RBSZ_MASK |   \
   ETH_DMACRXCR_RXPBL_MASK | ETH_DMACRXCR_RPF)

#define DMACRXCR_SET_MASK                       \
  (ETH_DMACRXCR_RBSZ(ALIGNED_BUFSIZE) | ETH_DMACRXCR_RXPBL(32))

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

#ifdef CONFIG_DEBUG_NET
#  define ETH_DMAINT_ERROR_ENABLE (ETH_DMACIER_AIE | ETH_DMAINT_ABNORMAL)
#else
#  define ETH_DMAINT_ERROR_ENABLE (0)
#endif

/* Helpers ******************************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

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
  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  uint8_t              mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t              fduplex : 1; /* Full (vs. half) duplex */
  uint8_t              intf;        /* Ethernet interface number */
  struct wdog_s        txtimeout;   /* TX timeout timer */
  struct work_s        irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s        pollwork;    /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s  dev;         /* Interface understood by the network */

  /* Used to track transmit and receive descriptors */

  struct eth_desc_s *txhead;        /* Next available TX descriptor */
  struct eth_desc_s *rxhead;        /* Next available RX descriptor */

  struct eth_desc_s *txchbase;      /* TX descriptor ring base address */
  struct eth_desc_s *rxchbase;      /* RX descriptor ring base address */

  struct eth_desc_s *txtail;        /* First "in_flight" TX descriptor */
  struct eth_desc_s *rxcurr;        /* First RX descriptor of the segment */
  uint16_t             segments;    /* RX segment count */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */
  sq_queue_t           freeb;       /* The free buffer list */
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
aligned_data(ARMV7M_DCACHE_LINESIZE);
static union stm32_desc_u g_txtable[TXTABLE_SIZE]
aligned_data(ARMV7M_DCACHE_LINESIZE);

/* Buffer allocations */

static uint8_t g_rxbuffer[RXBUFFER_ALLOC]
aligned_data(ARMV7M_DCACHE_LINESIZE);
static uint8_t g_txbuffer[TXBUFFER_ALLOC]
aligned_data(ARMV7M_DCACHE_LINESIZE);

/* These are the pre-allocated Ethernet device structures */

static struct stm32_ethmac_s g_stm32ethmac[STM32H7_NETHERNET];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_STM32H7_ETHMAC_REGDEBUG
static uint32_t stm32_getreg(uint32_t addr);
static void stm32_putreg(uint32_t val, uint32_t addr);
static void stm32_checksetup(void);
#else
# define stm32_getreg(addr)      getreg32(addr)
# define stm32_putreg(val,addr)  putreg32(val,addr)
# define stm32_checksetup()
#endif

/* Free buffer management */

static void stm32_initbuffer(struct stm32_ethmac_s *priv,
                             uint8_t *txbuffer);
static inline uint8_t *stm32_allocbuffer(struct stm32_ethmac_s *priv);
static inline void stm32_freebuffer(struct stm32_ethmac_s *priv,
                                    uint8_t *buffer);
static inline bool stm32_isfreebuffer(struct stm32_ethmac_s *priv);

/* Common TX logic */

static int  stm32_transmit(struct stm32_ethmac_s *priv);
static int  stm32_txpoll(struct net_driver_s *dev);
static void stm32_dopoll(struct stm32_ethmac_s *priv);

/* Interrupt handling */

static void stm32_enableint(struct stm32_ethmac_s *priv, uint32_t ierbit);
static void stm32_disableint(struct stm32_ethmac_s *priv, uint32_t ierbit);

static void stm32_freesegment(struct stm32_ethmac_s *priv,
                              struct eth_desc_s *rxfirst, int segments);
static int  stm32_recvframe(struct stm32_ethmac_s *priv);
static void stm32_receive(struct stm32_ethmac_s *priv);
static void stm32_freeframe(struct stm32_ethmac_s *priv);
static void stm32_txdone(struct stm32_ethmac_s *priv);

static void stm32_interrupt_work(void *arg);
static int  stm32_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void stm32_txtimeout_work(void *arg);
static void stm32_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  stm32_ifup(struct net_driver_s *dev);
static int  stm32_ifdown(struct net_driver_s *dev);

static void stm32_txavail_work(void *arg);
static int  stm32_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int  stm32_addmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_IGMP
static int  stm32_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_PHY_IOCTL
static int  stm32_ioctl(struct net_driver_s *dev, int cmd,
                        unsigned long arg);
#endif

/* Descriptor Initialization */

static void stm32_txdescinit(struct stm32_ethmac_s *priv,
                             union stm32_desc_u *txtable);
static void stm32_rxdescinit(struct stm32_ethmac_s *priv,
                             union stm32_desc_u *rxtable, uint8_t *rxbuffer);

/* PHY Initialization */

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int  stm32_phyintenable(struct stm32_ethmac_s *priv);
#endif
static int  stm32_phyread(uint16_t phydevaddr, uint16_t phyregaddr,
                          uint16_t *value);
static int  stm32_phywrite(uint16_t phydevaddr, uint16_t phyregaddr,
                           uint16_t value, uint16_t mask);
#ifdef CONFIG_ETH0_PHY_DM9161
static inline int stm32_dm9161(struct stm32_ethmac_s *priv);
#endif
static int  stm32_phyinit(struct stm32_ethmac_s *priv);
#ifdef CONFIG_STM32H7_ETHMAC_REGDEBUG
static void  stm32_phyregdump(void);
#endif

/* MAC/DMA Initialization */

#ifdef CONFIG_STM32H7_MII
static inline void stm32_selectmii(void);
#endif
#ifdef CONFIG_STM32H7_RMII
static inline void stm32_selectrmii(void);
#endif
static inline void stm32_ethgpioconfig(struct stm32_ethmac_s *priv);
static void stm32_ethreset(struct stm32_ethmac_s *priv);
static int  stm32_macconfig(struct stm32_ethmac_s *priv);
static void stm32_macaddress(struct stm32_ethmac_s *priv);
#ifdef CONFIG_NET_ICMPv6
static void stm32_ipv6multicast(struct stm32_ethmac_s *priv);
#endif
static int  stm32_macenable(struct stm32_ethmac_s *priv);
static int  stm32_ethconfig(struct stm32_ethmac_s *priv);

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

#ifdef CONFIG_STM32H7_ETHMAC_REGDEBUG
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

  ninfo("%08x->%08x\n", addr, val);
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

#ifdef CONFIG_STM32H7_ETHMAC_REGDEBUG
static void stm32_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  ninfo("%08x<-%08x\n", addr, val);

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

#ifdef CONFIG_STM32H7_ETHMAC_REGDEBUG
static void stm32_checksetup(void)
{
}
#endif

/****************************************************************************
 * Function: stm32_initbuffer
 *
 * Description:
 *   Initialize the free buffer list.
 *
 * Parameters:
 *   priv     - Reference to the driver state structure
 *   txbuffer - DMA memory allocated for TX buffers.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during early driver initialization before Ethernet interrupts
 *   are enabled.
 *
 ****************************************************************************/

static void stm32_initbuffer(struct stm32_ethmac_s *priv, uint8_t *txbuffer)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = txbuffer;
       i < STM32_ETH_NFREEBUFFERS;
       i++, buffer += ALIGNED_BUFSIZE)
    {
      sq_addlast((sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: stm32_allocbuffer
 *
 * Description:
 *   Allocate one buffer from the free buffer list.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer on success; NULL on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline uint8_t *stm32_allocbuffer(struct stm32_ethmac_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: stm32_freebuffer
 *
 * Description:
 *   Return a buffer to the free buffer list.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   buffer - A pointer to the buffer to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline void stm32_freebuffer(struct stm32_ethmac_s *priv,
                                    uint8_t *buffer)
{
  /* Free the buffer by adding it to the end of the free buffer list */

  sq_addlast((sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: stm32_isfreebuffer
 *
 * Description:
 *   Return TRUE if the free buffer list is not empty.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   True if there are one or more buffers in the free buffer list;
 *   false if the free buffer list is empty
 *
 * Assumptions:
 *   None.
 *
 ****************************************************************************/

static inline bool stm32_isfreebuffer(struct stm32_ethmac_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
}

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
    &g_txtable[priv->intf * CONFIG_STM32H7_ETH_NTXDESC];
  union stm32_desc_u *last =
    &g_txtable[priv->intf * CONFIG_STM32H7_ETH_NTXDESC +
               CONFIG_STM32H7_ETH_NTXDESC - 1];
  union stm32_desc_u *next = ((union stm32_desc_u *)curr) + 1;

  if (next > last)
    {
      next = first;
    }

  return &next->desc;
}

/****************************************************************************
 * Function: stm32_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_transmit(struct stm32_ethmac_s *priv)
{
  struct eth_desc_s *txdesc;
  struct eth_desc_s *txfirst;

  /* The internal (optimal) network buffer size may be configured to be
   * larger than the Ethernet buffer size.
   */

#if OPTIMAL_ETH_BUFSIZE > ALIGNED_BUFSIZE
  uint8_t *buffer;
  int bufcount;
  int lastsize;
  int i;
#endif

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  txdesc  = priv->txhead;
  txfirst = txdesc;

  ninfo("d_len: %d d_buf: %p txhead: %p tdes3: %08" PRIx32 "\n",
        priv->dev.d_len, priv->dev.d_buf, txdesc, txdesc->des3);

  DEBUGASSERT(txdesc);

  /* Flush the contents of the TX buffer into physical memory */

  up_clean_dcache((uintptr_t)priv->dev.d_buf,
                  (uintptr_t)priv->dev.d_buf + priv->dev.d_len);

  /* Is the size to be sent greater than the size of the Ethernet buffer? */

  DEBUGASSERT(priv->dev.d_len > 0 && priv->dev.d_buf != NULL);

#if OPTIMAL_ETH_BUFSIZE > ALIGNED_BUFSIZE
  if (priv->dev.d_len > ALIGNED_BUFSIZE)
    {
      /* Yes... how many buffers will be need to send the packet? */

      bufcount = (priv->dev.d_len + (ALIGNED_BUFSIZE - 1)) / ALIGNED_BUFSIZE;
      lastsize = priv->dev.d_len - (bufcount - 1) * ALIGNED_BUFSIZE;

      ninfo("bufcount: %d lastsize: %d\n", bufcount, lastsize);

      /* Set the first segment bit in the first TX descriptor */

      txdesc->des3 = ETH_TDES3_RD_FD;

      /* Set up all but the last TX descriptor */

      buffer = priv->dev.d_buf;

      for (i = 0; i < bufcount; i++)
        {
          DEBUGASSERT((txdesc->des3 & ETH_TDES3_RD_OWN) == 0);

          /* Set the Buffer1 address pointer */

          txdesc->des0 = (uint32_t)buffer;

          /* Set the Buffer2 address pointer */

          txdesc->des1 = 0;

          /* Set the buffer size in all TX descriptors */

          if (i == (bufcount - 1))
            {
              /* This is the last segment.  Set the last segment bit in the
               * last TX descriptor
               */

              txdesc->des3 |= ETH_TDES3_RD_LD;

              /* This segment is, most likely, of fractional buffersize */

              /* ask for an interrupt when this segment transfer completes. */

              txdesc->des2 = lastsize | ETH_TDES2_RD_IOC;
              buffer        += lastsize;
            }
          else
            {
              /* This is not the last segment.  We don't want an interrupt
               * when this segment transfer completes.
               */

              /* The size of the transfer is the whole buffer */

              txdesc->des2  = ALIGNED_BUFSIZE;
              buffer        += ALIGNED_BUFSIZE;
            }

          /* Give the descriptor to DMA */

          txdesc->des3 |= ETH_TDES3_RD_OWN;

          /* Flush the contents of the modified TX descriptor into physical
           * memory.
           */

          up_clean_dcache((uintptr_t)txdesc,
                          (uintptr_t)txdesc + sizeof(struct eth_desc_s));

          /* Point to the next available TX descriptor */

          txdesc = stm32_get_next_txdesc(priv, txdesc);
        }
    }
  else
#endif
    {
      DEBUGASSERT((txdesc->des3 & ETH_TDES3_RD_OWN) == 0);

      /* Set the Buffer1 address pointer */

      txdesc->des0 = (uint32_t)priv->dev.d_buf;

      /* Set the Buffer2 address pointer */

      txdesc->des1 = 0;

      /* Set frame size, and we do
       * want an interrupt when the transfer completes.
       */

      DEBUGASSERT(priv->dev.d_len <= CONFIG_NET_ETH_PKTSIZE);
      txdesc->des2 = priv->dev.d_len | ETH_TDES2_RD_IOC;

      /* The single descriptor is both the first and last segment. */

      /* Set OWN bit of the TX descriptor des3.  This gives the buffer to
       * Ethernet DMA
       */

      txdesc->des3 = (ETH_TDES3_RD_OWN | ETH_TDES3_RD_LD | ETH_TDES3_RD_FD);

      /* Flush the contents of the modified TX descriptor into physical
       * memory.
       */

      up_clean_dcache((uintptr_t)txdesc,
                      (uintptr_t)txdesc + sizeof(struct eth_desc_s));

      /* Point to the next available TX descriptor */

      txdesc = stm32_get_next_txdesc(priv, txdesc);
    }

  /* Remember where we left off in the TX descriptor chain */

  priv->txhead = txdesc;

  /* Detach the buffer from priv->dev structure.  That buffer is now
   * "in-flight".
   */

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

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

  /* If all TX descriptors are in-flight, then we have to disable receive
   * interrupts too.  This is because receive events can trigger more un-
   * stoppable transmit events.
   */

  if (priv->inflight >= CONFIG_STM32H7_ETH_NTXDESC)
    {
      stm32_disableint(priv, ETH_DMACIER_RIE);
    }

  MEMORY_SYNC();

  /* Enable TX interrupts */

  stm32_enableint(priv, ETH_DMACIER_TIE);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, STM32_TXTIMEOUT,
           stm32_txtimeout_expiry, (wdparm_t)priv);

  /* Update the tx descriptor tail pointer register to start the DMA */

  putreg32((uintptr_t)txdesc, STM32_ETH_DMACTXDTPR);

  return OK;
}

/****************************************************************************
 * Function: stm32_txpoll
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
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_txpoll(struct net_driver_s *dev)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev->d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

  /* Send the packet */

  stm32_transmit(priv);
  DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ETH_TDES3_OWN may be cleared BUT still
   * not available because stm32_freeframe() has not yet run. If
   * stm32_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_STM32H7_ETH_NTXDESC).
   */

  if ((priv->txhead->des3 & ETH_TDES3_RD_OWN) != 0 ||
      priv->txhead->des0 != 0)
    {
      /* We have to terminate the poll if we have no more descriptors
       * available for another transfer.
       */

      nerr("No tx descriptors available");

      return -EBUSY;
    }

  /* We have the descriptor, we can continue the poll. Allocate a new
   * buffer for the poll.
   */

  dev->d_buf = stm32_allocbuffer(priv);

  /* We can't continue the poll if we have no buffers */

  if (dev->d_buf == NULL)
    {
      /* Terminate the poll. */

      nerr("No tx buffer available");

      return -ENOMEM;
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: stm32_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (stm32_txdone),
 *   2. When new TX data is available (stm32_txavail_process), and
 *   3. After a TX timeout to restart the sending process
 *      (stm32_txtimeout_process).
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_dopoll(struct stm32_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ETH_TDES3_RD_OWN may be cleared BUT still
   * not available because stm32_freeframe() has not yet run. If
   * stm32_freeframe() has run, the buffer1 pointer (des0) will be
   * nullified (and inflight should be < CONFIG_STM32H7_ETH_NTXDESC).
   */

  if ((priv->txhead->des3 & ETH_TDES3_RD_OWN) == 0 &&
      priv->txhead->des0 == 0)
    {
      /* If we have the descriptor, then poll the network for new XMIT data.
       * Allocate a buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = stm32_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          devif_poll(dev, stm32_txpoll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              stm32_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
      else
        {
          nerr("No tx buffers");
        }
    }
  else
    {
      nerr("No tx descriptors\n");
    }
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
 *   Global interrupts are disabled by interrupt handling logic.
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
 *   Global interrupts are disabled by interrupt handling logic.
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
    &g_rxtable[priv->intf * CONFIG_STM32H7_ETH_NRXDESC];
  union stm32_desc_u *last =
    &g_rxtable[priv->intf * CONFIG_STM32H7_ETH_NRXDESC +
               CONFIG_STM32H7_ETH_NRXDESC - 1];
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
 *   Global interrupts are disabled by interrupt handling logic.
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
 * Function: stm32_recvframe
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors of the received frame.
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK if a packet was successfully returned; -EAGAIN if there are no
 *   further packets available
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_recvframe(struct stm32_ethmac_s *priv)
{
  struct eth_desc_s *rxdesc;
  struct eth_desc_s *rxcurr = NULL;
  uint8_t *buffer;
  int i;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  /* Check if there are free buffers.  We cannot receive new frames in this
   * design unless there is at least one free buffer.
   */

  if (!stm32_isfreebuffer(priv))
    {
      nerr("ERROR: No free buffers\n");
      return -ENOMEM;
    }

  /* Scan descriptors owned by the CPU.  Scan until:
   *
   *   1) We find a descriptor still owned by the DMA,
   *   2) We have examined all of the RX descriptors, or
   *   3) All of the TX descriptors are in flight.
   *
   * This last case is obscure.  It is due to that fact that each packet
   * that we receive can generate an unstoppable transmisson.  So we have
   * to stop receiving when we can not longer transmit.  In this case, the
   * transmit logic should also have disabled further RX interrupts.
   */

  rxdesc = priv->rxhead;

  /* Forces the first RX descriptor to be re-read from physical memory */

  up_invalidate_dcache((uintptr_t)rxdesc,
                       (uintptr_t)rxdesc + sizeof(struct eth_desc_s));

  for (i = 0;
       (rxdesc->des3 & ETH_RDES3_WB_OWN) == 0 &&
         i < CONFIG_STM32H7_ETH_NRXDESC &&
         priv->inflight < CONFIG_STM32H7_ETH_NTXDESC;
       i++)
    {
      /* Check if this is a normal descriptor */

      if (!(rxdesc->des3 & ETH_RDES3_WB_CTXT))
        {
          /* Check if this is the first segment in the frame */

          if ((rxdesc->des3 & ETH_RDES3_WB_FD) != 0 &&
              (rxdesc->des3 & ETH_RDES3_WB_LD) == 0)
            {
              priv->rxcurr   = rxdesc;
              priv->segments = 1;
            }

          /* Check if this is an intermediate segment in the frame */

          else if (((rxdesc->des3 & ETH_RDES3_WB_LD) == 0) &&
                   ((rxdesc->des3 & ETH_RDES3_WB_FD) == 0))
            {
              priv->segments++;
            }

          /* Otherwise, it is the last segment in the frame */

          else
            {
              priv->segments++;

              /* Check if there is only one segment in the frame */

              if (priv->segments == 1)
                {
                  rxcurr = rxdesc;
                }
              else
                {
                  rxcurr = priv->rxcurr;
                }

              ninfo("rxhead: %p rxcurr: %p segments: %d\n",
                    priv->rxhead, priv->rxcurr, priv->segments);

              /* Check if any errors are reported in the frame */

              if ((rxdesc->des3 & ETH_RDES3_WB_ES) == 0)
                {
                  struct net_driver_s *dev = &priv->dev;

                  /* Get the Frame Length of the received packet: subtract 4
                   * bytes of the CRC
                   */

                  dev->d_len = ((rxdesc->des3 & ETH_RDES3_WB_PL_MASK) >>
                               ETH_RDES3_WB_PL_SHIFT) - 4;

                  if (priv->segments > 1 ||
                      dev->d_len > ALIGNED_BUFSIZE)
                    {
                      /* The Frame is to big, it spans segments */

                      nerr("ERROR: Dropped, RX descriptor Too big: %d in %d "
                          "segments\n", dev->d_len, priv->segments);

                      stm32_freesegment(priv, rxcurr, priv->segments);
                    }
                  else
                    {
                      /* Get a buffer from the free list.  We don't even
                       * check if this is successful because we already
                       * assure the free list is not empty above.
                       */

                      buffer = stm32_allocbuffer(priv);

                      /* Take the buffer from the RX descriptor of the first
                       * free segment, put it into the network device
                       * structure, then replace the buffer in the RX
                       * descriptor with the newly allocated buffer.
                       */

                      DEBUGASSERT(dev->d_buf == NULL);
                      dev->d_buf    = (uint8_t *)rxcurr->des0;
                      rxcurr->des0 = (uint32_t)buffer;

                      /* Make sure that the modified RX descriptor is written
                       * to physical memory.
                       */

                      up_clean_dcache((uintptr_t)rxcurr,
                                      (uintptr_t)rxdesc +
                                      sizeof(struct eth_desc_s));

                      /* Remember where we should re-start scanning and reset
                       * the segment scanning logic
                       */

                      priv->rxhead   = stm32_get_next_rxdesc(priv, rxdesc);
                      stm32_freesegment(priv, rxcurr, priv->segments);

                      /* Force the completed RX DMA buffer to be re-read from
                       * physical memory.
                       */

                      up_invalidate_dcache((uintptr_t)dev->d_buf,
                                           (uintptr_t)dev->d_buf +
                                           min(dev->d_len, ALIGNED_BUFSIZE));

                      ninfo("rxhead: %p d_buf: %p d_len: %d\n",
                            priv->rxhead, dev->d_buf, dev->d_len);

                      /* Return success */

                      return OK;
                    }
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
                  stm32_freesegment(priv, rxcurr, priv->segments);
                }
            }
        }
      else
        {
          /* Drop the context descriptors, we are not interested */

          DEBUGASSERT(rxcurr != NULL);
          stm32_freesegment(priv, rxcurr, 1);
        }

      /* Try the next descriptor */

      rxdesc = stm32_get_next_rxdesc(priv, rxdesc);

      /* Force the next RX descriptor to be re-read from physical memory */

      up_invalidate_dcache((uintptr_t)rxdesc,
                           (uintptr_t)rxdesc + sizeof(struct eth_desc_s));
    }

  /* We get here after all of the descriptors have been scanned or when
   * rxdesc points to the first descriptor owned by the DMA. Remember
   * where we left off.
   */

  priv->rxhead = rxdesc;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  return -EAGAIN;
}

/****************************************************************************
 * Function: stm32_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_receive(struct stm32_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while stm32_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  while (stm32_recvframe(priv) == OK)
    {
#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet
       * tap
       */

     pkt_input(&priv->dev);
#endif

      /* Check if the packet is a valid size for the network buffer
       * configuration (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_ETH_PKTSIZE)
        {
          nwarn("WARNING: DROPPED Too big: %d\n", dev->d_len);

          /* Free dropped packet buffer */

          if (dev->d_buf)
            {
              stm32_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
              dev->d_len = 0;
            }

          continue;
        }

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              stm32_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              stm32_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Handle ARP packet */

          arp_input(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              stm32_transmit(priv);
            }
        }
      else
#endif
        {
          nwarn("WARNING: DROPPED Unknown type: %04x\n", BUF->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          stm32_freebuffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Function: stm32_freeframe
 *
 * Description:
 *   Scans the TX descriptors and frees the buffers of completed TX
 *   transfers.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_freeframe(struct stm32_ethmac_s *priv)
{
  struct eth_desc_s *txdesc;
  uint32_t des3_tmp;
  int i;

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

      for (i = 0; (txdesc->des3 & ETH_TDES3_RD_OWN) == 0; i++)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          ninfo("txtail: %p des0: %08" PRIx32
                " des2: %08" PRIx32 " des3: %08" PRIx32 "\n",
                txdesc, txdesc->des0, txdesc->des2, txdesc->des3);

          DEBUGASSERT(txdesc->des0 != 0);

          /* Yes.. Free the buffer */

          stm32_freebuffer(priv, (uint8_t *)txdesc->des0);

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
              /* Yes.. Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If all of the TX descriptors were in-flight,
               * then RX interrupts may have been disabled...
               * we can re-enable them now.
               */

              stm32_enableint(priv, ETH_DMACIER_RIE);

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return;
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
}

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
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void stm32_txdone(struct stm32_ethmac_s *priv)
{
  DEBUGASSERT(priv->txtail != NULL);

  /* Scan the TX descriptor change, returning buffers to free list */

  stm32_freeframe(priv);

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      /* Cancel the TX timeout */

      wd_cancel(&priv->txtimeout);

      /* And disable further TX interrupts. */

      stm32_disableint(priv, ETH_DMACIER_TIE);
    }

  /* Then poll the network for new XMIT data */

  stm32_dopoll(priv);
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

  net_lock();

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dmasr = stm32_getreg(STM32_ETH_DMACSR);

  /* Mask only enabled interrupts.  This depends on the fact that the
   * interrupt related bits (0-16) correspond in these two registers.
   */

  dmasr &= stm32_getreg(STM32_ETH_DMACIER);

  /* Check if there are pending "normal" interrupts */

  if ((dmasr & ETH_DMACSR_NIS) != 0)
    {
      /* Yes.. Check if we received an incoming packet, if so, call
       * stm32_receive()
       */

      if ((dmasr & ETH_DMACSR_RI) != 0)
        {
          /* Clear the pending receive interrupt */

          stm32_putreg(ETH_DMACSR_RI, STM32_ETH_DMACSR);

          /* Handle the received package */

          stm32_receive(priv);
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

  /* Handle error interrupt only if CONFIG_DEBUG_NET is eanbled */

#ifdef CONFIG_DEBUG_NET
  /* Check if there are pending "abnormal" interrupts */

  if ((dmasr & ETH_DMACSR_AIS) != 0)
    {
      /* Just let the user know what happened */

      nerr("ERROR: Abormal event(s): %08x\n", dmasr);

      /* Clear all pending abnormal events */

      stm32_putreg(ETH_DMAINT_ABNORMAL, STM32_ETH_DMACSR);

      /* Clear the pending abnormal summary interrupt */

      stm32_putreg(ETH_DMACSR_AIS, STM32_ETH_DMACSR);
    }
#endif

  net_unlock();

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

  /* Reset the hardware.  Just take the interface down, then back up again. */

  net_lock();
  stm32_ifdown(&priv->dev);
  stm32_ifup(&priv->dev);

  /* Then poll for new XMIT data */

  stm32_dopoll(priv);
  net_unlock();
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

static int stm32_ifup(struct net_driver_s *dev)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev->d_private;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff), (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff), (int)(dev->d_ipaddr >> 24));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Configure the Ethernet interface for DMA operation. */

  ret = stm32_ethconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  up_enable_irq(STM32_IRQ_ETH);

  stm32_checksetup();
  return OK;
}

/****************************************************************************
 * Function: stm32_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
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

static int stm32_ifdown(struct net_driver_s *dev)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(STM32_IRQ_ETH);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the stm32_ifup() always
   * successfully brings the interface back up.
   */

  stm32_ethreset(priv);

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: stm32_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg  - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void stm32_txavail_work(void *arg)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      stm32_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: stm32_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int stm32_txavail(struct net_driver_s *dev)
{
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, stm32_txavail_work, priv, 0);
    }

  return OK;
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
static int stm32_addmac(struct net_driver_s *dev, const uint8_t *mac)
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
static int stm32_rmmac(struct net_driver_s *dev, const uint8_t *mac)
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

  for (i = 0; i < CONFIG_STM32H7_ETH_NTXDESC; i++)
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

  stm32_putreg(CONFIG_STM32H7_ETH_NTXDESC - 1, STM32_ETH_DMACTXRLR);

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
 *   txbuffer - List of pre-allocated DMA buffers for the Ethernet interface
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_rxdescinit(struct stm32_ethmac_s *priv,
                             union stm32_desc_u *rxtable,
                             uint8_t *rxbuffer)
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

  for (i = 0; i < CONFIG_STM32H7_ETH_NRXDESC; i++)
    {
      rxdesc = &rxtable[i].desc;

      /* Set Buffer1 address pointer */

      rxdesc->des0 = (uint32_t)&rxbuffer[i * ALIGNED_BUFSIZE];

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

  stm32_putreg(CONFIG_STM32H7_ETH_NRXDESC - 1, STM32_ETH_DMACRXRLR);

  /* Set Receive Descriptor List Address Register */

  stm32_putreg((uint32_t)&rxtable[0].desc, STM32_ETH_DMACRXDLAR);

  /* Set Receive Descriptor Tail pointer Address */

  stm32_putreg((uint32_t)&rxtable[CONFIG_STM32H7_ETH_NRXDESC - 1].desc,
               STM32_ETH_DMACRXDTPR);
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
static int stm32_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_ARCH_PHY_INTERRUPT
  struct stm32_ethmac_s *priv = (struct stm32_ethmac_s *)dev->d_private;
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          struct mii_iotcl_notify_s *req =
            (struct mii_iotcl_notify_s *)((uintptr_t)arg);

          ret = phy_notify_subscribe(dev->d_ifname, req->pid, req->signo,
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
          req->phy_id = CONFIG_STM32H7_PHYADDR;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = stm32_phyread(req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = stm32_phywrite(req->phy_id, req->reg_num, req->val_in,
                               0xffff);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_PHY_IOCTL */

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
 * Function: stm32_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Parameters:
 *   phydevaddr - The PHY device address
 *   phyregaddr - The PHY register address
 *   value - The location to return the 16-bit PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_phyread(uint16_t phydevaddr, uint16_t phyregaddr,
                         uint16_t *value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MACMDIOAR register, preserving CSR Clock Range CR[3:0]
   * bits
   */

  regval  = stm32_getreg(STM32_ETH_MACMDIOAR);
  regval &= ETH_MACMDIOAR_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the buy bit.
   * the ETH_MACMDIOAR_GOC == 3, indicating a read operation.
   */

  regval |= (((uint32_t)phydevaddr << ETH_MACMDIOAR_PA_SHIFT) &
            ETH_MACMDIOAR_PA_MASK);
  regval |= (((uint32_t)phyregaddr << ETH_MACMDIOAR_RDA_SHIFT) &
            ETH_MACMDIOAR_RDA_MASK);
  regval |= ETH_MACMDIOAR_MB | ETH_MACMDIOAR_GOC_READ;

  stm32_putreg(regval, STM32_ETH_MACMDIOAR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_READ_TIMEOUT; timeout++)
    {
      if ((stm32_getreg(STM32_ETH_MACMDIOAR) & ETH_MACMDIOAR_MB) == 0)
        {
          *value = (uint16_t)stm32_getreg(STM32_ETH_MACMDIODR);
          return OK;
        }
    }

  ninfo("MII transfer timed out: phydevaddr: %04x phyregaddr: %04x\n",
        phydevaddr, phyregaddr);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: stm32_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Parameters:
 *   phydevaddr - The PHY device address
 *   phyregaddr - The PHY register address
 *   value - The 16-bit value to write to the PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_phywrite(uint16_t phydevaddr, uint16_t phyregaddr,
                          uint16_t set, uint16_t clear)
{
  volatile uint32_t timeout;
  uint32_t regval;
  uint16_t value;

  /* Configure the MACMDIOAR register, preserving CSR Clock Range CR[3:0]
   * bits
   */

  regval  = stm32_getreg(STM32_ETH_MACMDIOAR);
  regval &= ETH_MACMDIOAR_CR_MASK;

  /* Read the existing register value, if clear mask is given */

  if (clear != 0xffff)
    {
      if (stm32_phyread(phydevaddr, phyregaddr, &value) != OK)
        {
          return -ETIMEDOUT;
        }

      value &= ~clear;
      value |= set;
    }
  else
    {
      value = set;
    }

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the ETH_MACMDIOAR_GOC == 1, indicating a write operation.
   */

  regval |= (((uint32_t)phydevaddr << ETH_MACMDIOAR_PA_SHIFT) &
            ETH_MACMDIOAR_PA_MASK);
  regval |= (((uint32_t)phyregaddr << ETH_MACMDIOAR_RDA_SHIFT) &
            ETH_MACMDIOAR_RDA_MASK);
  regval |= (ETH_MACMDIOAR_MB | ETH_MACMDIOAR_GOC_WRITE);

  /* Write the value into the MACMDIODR register before setting the new
   * MACMDIOAR register value.
   */

  stm32_putreg(value, STM32_ETH_MACMDIODR);
  stm32_putreg(regval, STM32_ETH_MACMDIOAR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_WRITE_TIMEOUT; timeout++)
    {
      if ((stm32_getreg(STM32_ETH_MACMDIOAR) & ETH_MACMDIOAR_MB) == 0)
        {
          return OK;
        }
    }

  ninfo("MII transfer timed out: phydevaddr: %04x phyregaddr: %04x value: "
        "%04x\n", phydevaddr, phyregaddr, value);

  return -ETIMEDOUT;
}

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

  ret = stm32_phyread(CONFIG_STM32H7_PHYADDR, MII_PHYID1, &phyval);
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

  ret = stm32_phyread(CONFIG_STM32H7_PHYADDR, 16, &phyval);
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

#ifdef CONFIG_STM32H7_ETHMAC_REGDEBUG
static void stm32_phyregdump()
{
  uint16_t phyval;
  int ret;
  int i;

  for (i = 0; i < 0x20; i++)
    {
      ret = stm32_phyread(CONFIG_STM32H7_PHYADDR, i, &phyval);
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
#ifdef CONFIG_STM32H7_AUTONEG
  volatile uint32_t timeout;
#endif
  uint32_t regval;
  uint16_t phyval;
  int ret;
  int to;

  /* Assume 10MBps and half duplex */

  priv->mbps100 = 0;
  priv->fduplex = 0;

  /* Setup up PHY clocking by setting the CR field in the MACMDIOAR reg */

  regval  = stm32_getreg(STM32_ETH_MACMDIOAR);
  regval &= ~ETH_MACMDIOAR_CR_MASK;
  regval |= ETH_MACMDIOAR_CR;
  stm32_putreg(regval, STM32_ETH_MACMDIOAR);

  /* Put the PHY in reset mode */

  ret = stm32_phywrite(CONFIG_STM32H7_PHYADDR, MII_MCR, MII_MCR_RESET,
                       MII_MCR_RESET);
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
      ret = stm32_phyread(CONFIG_STM32H7_PHYADDR, MII_MCR, &phyval);
    }
  while (phyval & MII_MCR_RESET && to > 0);

  if (to <= 0)
    {
      nerr("ERROR: Phy reset timeout\n");
      return ret;
    }
  else
    {
      ninfo("Phy reset in %d ms\n", PHY_RESET_DELAY - to);
    }

#ifdef CONFIG_STM32H7_ETHMAC_REGDEBUG
  stm32_phyregdump();
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

#ifdef CONFIG_STM32H7_AUTONEG
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = stm32_phyread(CONFIG_STM32H7_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_LINKSTATUS) != 0)
        {
          break;
        }

      nxsig_usleep(100);
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for link status: %04x\n", phyval);
      return -ETIMEDOUT;
    }

  /* Enable auto-negotiation */

  ret = stm32_phywrite(CONFIG_STM32H7_PHYADDR, MII_MCR, MII_MCR_ANENABLE,
                       MII_MCR_ANENABLE);
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = stm32_phyread(CONFIG_STM32H7_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_ANEGCOMPLETE) != 0)
        {
          break;
        }

      nxsig_usleep(100);
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for auto-negotiation\n");
      return -ETIMEDOUT;
    }

  /* Read the result of the auto-negotiation from the PHY-specific register */

  ret = stm32_phyread(CONFIG_STM32H7_PHYADDR, CONFIG_STM32H7_PHYSR, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  ninfo("PHYSR[%d]: %04x\n", CONFIG_STM32H7_PHYSR, phyval);

  /* Different PHYs present speed and mode information in different ways.  IF
   * This CONFIG_STM32H7_PHYSR_ALTCONFIG is selected, this indicates that
   * the PHY represents speed and mode information are combined, for
   * example, with separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_STM32H7_PHYSR_ALTCONFIG
  switch (phyval & CONFIG_STM32H7_PHYSR_ALTMODE)
    {
      default:
        nerr("ERROR: Unrecognized PHY status setting\n");

      /* Falls through */

      case CONFIG_STM32H7_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_STM32H7_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_STM32H7_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_STM32H7_PHYSR_100FD:
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
  if ((phyval & CONFIG_STM32H7_PHYSR_MODE) ==
      CONFIG_STM32H7_PHYSR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_STM32H7_PHYSR_SPEED) == CONFIG_STM32H7_PHYSR_100MBPS)
    {
      priv->mbps100 = 1;
    }
#endif

#else /* Auto-negotiation not selected */

  phyval = 0;
#ifdef CONFIG_STM32H7_ETHFD
  phyval |= MII_MCR_FULLDPLX;
#endif
#ifdef CONFIG_STM32H7_ETH100MBPS
  phyval |= MII_MCR_SPEED100;
#endif

  ret = stm32_phywrite(CONFIG_STM32H7_PHYADDR, MII_MCR, phyval, 0xffff);
  if (ret < 0)
    {
      nerr("ERROR: Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes */

#ifdef CONFIG_STM32H7_ETHFD
  priv->fduplex = 1;
#endif
#ifdef CONFIG_STM32H7_ETH100MBPS
  priv->mbps100 = 1;
#endif
#endif

  ninfo("Duplex: %s Speed: %d MBps\n",
        priv->fduplex ? "FULL" : "HALF",
        priv->mbps100 ? 100 : 10);

  return OK;
}

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

#ifdef CONFIG_STM32H7_MII
static inline void stm32_selectmii(void)
{
  uint32_t regval;

  regval  = getreg32(STM32_SYSCFG_PMC);
  regval &= ~SYSCFG_PMC_EPIS_MASK;
  regval |= SYSCFG_PMC_EPIS_MII;
  putreg32(regval, STM32_SYSCFG_PMC);
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

#ifdef CONFIG_STM32H7_RMII
static inline void stm32_selectrmii(void)
{
  uint32_t regval;

  regval  = getreg32(STM32_SYSCFG_PMC);
  regval &= ~SYSCFG_PMC_EPIS_MASK;
  regval |= SYSCFG_PMC_EPIS_RMII;
  putreg32(regval, STM32_SYSCFG_PMC);
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

#if defined(CONFIG_STM32H7_MII) || defined(CONFIG_STM32H7_RMII)

  /* MDC and MDIO are common to both modes */

  stm32_configgpio(GPIO_ETH_MDC);
  stm32_configgpio(GPIO_ETH_MDIO);

  /* Set up the MII interface */

#  if defined(CONFIG_STM32H7_MII)

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

#    if defined(CONFIG_STM32H7_MII_MCO1)
  /* Configure MC01 to drive the PHY.  Board logic must provide MC01 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO1);
  stm32_mco1config(BOARD_CFGR_MC01_SOURCE, BOARD_CFGR_MC01_DIVIDER);

#    elif defined(CONFIG_STM32H7_MII_MCO2)
  /* Configure MC02 to drive the PHY.  Board logic must provide MC02 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO2);
  stm32_mco2config(BOARD_CFGR_MC02_SOURCE, BOARD_CFGR_MC02_DIVIDER);

#    elif defined(CONFIG_STM32H7_MII_MCO)
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

#  elif defined(CONFIG_STM32H7_RMII)

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

#    if defined(CONFIG_STM32H7_RMII_MCO1)
  /* Configure MC01 to drive the PHY.  Board logic must provide MC01 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO1);
  stm32_mco1config(BOARD_CFGR_MC01_SOURCE, BOARD_CFGR_MC01_DIVIDER);

#    elif defined(CONFIG_STM32H7_RMII_MCO2)
  /* Configure MC02 to drive the PHY.  Board logic must provide MC02 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO2);
  stm32_mco2config(BOARD_CFGR_MC02_SOURCE, BOARD_CFGR_MC02_DIVIDER);

#    elif defined(CONFIG_STM32H7_RMII_MCO)
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
#  endif
#endif

#ifdef CONFIG_STM32H7_ETH_PTP
  /* Enable pulse-per-second (PPS) output signal */

  stm32_configgpio(GPIO_ETH_PPS_OUT);
#endif
}

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
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_ethreset(struct stm32_ethmac_s *priv)
{
  uint32_t regval;
  volatile uint32_t timeout;

  /* Reset the Ethernet on the AHB1 bus */

  regval  = stm32_getreg(STM32_RCC_AHB1RSTR);
  regval |= RCC_AHB1RSTR_ETH1MACRST;
  stm32_putreg(regval, STM32_RCC_AHB1RSTR);

  regval &= ~RCC_AHB1RSTR_ETH1MACRST;
  stm32_putreg(regval, STM32_RCC_AHB1RSTR);

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
  struct net_driver_s *dev = &priv->dev;
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
 * Function: stm32_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
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

#ifdef CONFIG_NET_ICMPv6
static void stm32_ipv6multicast(struct stm32_ethmac_s *priv)
{
  struct net_driver_s *dev;
  uint16_t tmp16;
  uint8_t mac[6];

  /* For ICMPv6, we need to add the IPv6 multicast address
   *
   * For IPv6 multicast addresses, the Ethernet MAC is derived by
   * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
   * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
   * to the Ethernet MAC address 33:33:00:01:00:03.
   *
   * NOTES:  This appears correct for the ICMPv6 Router Solicitation
   * Message, but the ICMPv6 Neighbor Solicitation message seems to
   * use 33:33:ff:01:00:03.
   */

  mac[0] = 0x33;
  mac[1] = 0x33;

  dev    = &priv->dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  stm32_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  stm32_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  stm32_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

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

#ifdef CONFIG_NET_ICMPv6
  /* Set up the IPv6 multicast address */

  stm32_ipv6multicast(priv);
#endif

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

  /* Enable Ethernet DMA interrupts */

  stm32_putreg(ETH_MACIER_ALLINTS, STM32_ETH_MACIER);

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

#ifdef CONFIG_STM32H7_PHYINIT
  /* Perform any necessary, board-specific PHY initialization */

  ret = stm32_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Initialize the free buffer list */

  stm32_initbuffer(priv, &g_txbuffer[priv->intf * TXBUFFER_SIZE]);

  /* Reset the Ethernet block */

  ninfo("Reset the Ethernet block\n");
  stm32_ethreset(priv);

  /* Initialize TX Descriptors list */

  stm32_txdescinit(priv,
                   &g_txtable[priv->intf * CONFIG_STM32H7_ETH_NTXDESC]);

  /* Initialize RX Descriptors list */

  stm32_rxdescinit(priv,
                   &g_rxtable[priv->intf * CONFIG_STM32H7_ETH_NRXDESC],
                   &g_rxbuffer[priv->intf * RXBUFFER_SIZE]);

  /* Initialize the PHY */

  ninfo("Initialize the PHY\n");
  ret = stm32_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

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

#if STM32H7_NETHERNET > 1 || defined(CONFIG_NETDEV_LATEINIT)
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

  DEBUGASSERT(intf < STM32H7_NETHERNET);
  priv = &g_stm32ethmac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct stm32_ethmac_s));
  priv->dev.d_ifup    = stm32_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = stm32_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = stm32_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = stm32_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = stm32_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_PHY_IOCTL
  priv->dev.d_ioctl   = stm32_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = g_stm32ethmac;  /* Used to recover private state */
  priv->intf          = intf;           /* Remember the interface number */

  stm32_get_uniqueid(uid);
  crc = crc64(uid, 12);

  /* Specify as localy administrated address */

  priv->dev.d_mac.ether.ether_addr_octet[0]  = (crc >> 0) | 0x02;
  priv->dev.d_mac.ether.ether_addr_octet[0] &= ~0x1;

  priv->dev.d_mac.ether.ether_addr_octet[1]  = crc >> 8;
  priv->dev.d_mac.ether.ether_addr_octet[2]  = crc >> 16;
  priv->dev.d_mac.ether.ether_addr_octet[3]  = crc >> 24;
  priv->dev.d_mac.ether.ether_addr_octet[4]  = crc >> 32;
  priv->dev.d_mac.ether.ether_addr_octet[5]  = crc >> 40;

  /* Configure GPIO pins to support Ethernet */

  stm32_ethgpioconfig(priv);

  /* Attach the IRQ to the driver */

  if (irq_attach(STM32_IRQ_ETH, stm32_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

#ifdef CONFIG_STM32H7_PHYINIT
  /* Perform any necessary, board-specific PHY initialization */

  ret = stm32_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Put the interface in the down state. */

  stm32_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_ETHERNET);
  return ret;
}

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c. If STM32H7_NETHERNET
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

#if STM32H7_NETHERNET == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  stm32_ethinitialize(0);
}
#endif

#endif /* STM32H7_NETHERNET > 0 && CONFIG_STM32H7_ETHMAC */
