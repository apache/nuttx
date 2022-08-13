/****************************************************************************
 * arch/arm/src/tiva/tm4c/tm4c_ethernet.c
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
#if defined(CONFIG_NET) && defined(CONFIG_TIVA_ETHERNET)

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <queue.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_TIVA_PHY_INTERRUPTS
#  include <nuttx/net/phy.h>
#endif

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "arm_internal.h"
#include "chip.h"
#include "tiva_gpio.h"
#include "tiva_sysctrl.h"
#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"
#include "tiva_periphrdy.h"
#include "tiva_ethernet.h"

#include "hardware/tiva_pinmap.h"
#include <arch/board/board.h>

/* TIVA_NETHCONTROLLERS determines the number of physical interfaces
 * that will be supported.
 */

#if TIVA_NETHCONTROLLERS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if TIVA_NETHCONTROLLERS > 1
#  error Logic to support multiple Ethernet interfaces is incomplete
#endif

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define ETHWORK LPWORK

/* Are we using the internal PHY or an external PHY? */

#if defined(CONFIG_TIVA_PHY_INTERNAL)

/* Internal PHY */

#  if defined(CONFIG_TIVA_PHY_MII) ||defined(CONFIG_TIVA_PHY_RMII)
#    warning CONFIG_TIVA_PHY_MII or CONFIG_TIVA_PHY_RMII defined with internal PHY
#  endif

#  undef CONFIG_TIVA_PHY_MII
#  undef CONFIG_TIVA_PHY_RMII

/* Properties of the internal PHY are hard-coded */

#  undef CONFIG_TIVA_PHYADDR
#  undef CONFIG_TIVA_PHYSR_ALTCONFIG
#  undef CONFIG_TIVA_PHYSR_ALTMODE
#  undef CONFIG_TIVA_PHYSR_10HD
#  undef CONFIG_TIVA_PHYSR_100HD
#  undef CONFIG_TIVA_PHYSR_10FD
#  undef CONFIG_TIVA_PHYSR_100FD
#  undef CONFIG_TIVA_PHYSR_SPEED
#  undef CONFIG_TIVA_PHYSR_100MBPS
#  undef CONFIG_TIVA_PHYSR_MODE
#  undef CONFIG_TIVA_PHYSR_FULLDUPLEX

#  define CONFIG_TIVA_PHYADDR          0
#  define CONFIG_TIVA_PHYSR            TIVA_EPHY_STS
#  define CONFIG_TIVA_PHYSR_SPEED      EPHY_STS_SPEED
#  define CONFIG_TIVA_PHYSR_100MBPS    0
#  define CONFIG_TIVA_PHYSR_MODE       EPHY_STS_DUPLEX
#  define CONFIG_TIVA_PHYSR_FULLDUPLEX EPHY_STS_DUPLEX

#else

/* External PHY. Properties must be provided in the configuration */

#  if !defined(CONFIG_TIVA_PHY_MII) && !defined(CONFIG_TIVA_PHY_RMII)
#    warning None of CONFIG_TIVA_PHY_INTERNAL, CONFIG_TIVA_PHY_MII, or CONFIG_TIVA_PHY_RMII defined
#  endif

#  if defined(CONFIG_TIVA_PHY_MII) && defined(CONFIG_TIVA_PHY_RMII)
#    error Both CONFIG_TIVA_PHY_MII and CONFIG_TIVA_PHY_RMII defined
#  endif
#endif

#ifndef CONFIG_TIVA_PHYADDR
#  error CONFIG_TIVA_PHYADDR must be defined in the NuttX configuration
#endif

#ifdef CONFIG_TIVA_AUTONEG
#  ifndef CONFIG_TIVA_PHYSR
#    error CONFIG_TIVA_PHYSR must be defined in the NuttX configuration
#  endif
#  ifdef CONFIG_TIVA_PHYSR_ALTCONFIG
#    ifndef CONFIG_TIVA_PHYSR_ALTMODE
#      error CONFIG_TIVA_PHYSR_ALTMODE must be defined in the NuttX configuration
#    endif
#    ifndef CONFIG_TIVA_PHYSR_10HD
#      error CONFIG_TIVA_PHYSR_10HD must be defined in the NuttX configuration
#    endif
#    ifndef CONFIG_TIVA_PHYSR_100HD
#      error CONFIG_TIVA_PHYSR_100HD must be defined in the NuttX configuration
#    endif
#    ifndef CONFIG_TIVA_PHYSR_10FD
#      error CONFIG_TIVA_PHYSR_10FD must be defined in the NuttX configuration
#    endif
#    ifndef CONFIG_TIVA_PHYSR_100FD
#      error CONFIG_TIVA_PHYSR_100FD must be defined in the NuttX configuration
#    endif
#  else
#    ifndef CONFIG_TIVA_PHYSR_SPEED
#      error CONFIG_TIVA_PHYSR_SPEED must be defined in the NuttX configuration
#    endif
#    ifndef CONFIG_TIVA_PHYSR_100MBPS
#      error CONFIG_TIVA_PHYSR_100MBPS must be defined in the NuttX configuration
#    endif
#    ifndef CONFIG_TIVA_PHYSR_MODE
#      error CONFIG_TIVA_PHYSR_MODE must be defined in the NuttX configuration
#    endif
#    ifndef CONFIG_TIVA_PHYSR_FULLDUPLEX
#      error CONFIG_TIVA_PHYSR_FULLDUPLEX must be defined in the NuttX configuration
#    endif
#  endif
#endif

#ifdef CONFIG_TIVA_EMAC_PTP
#  warning CONFIG_TIVA_EMAC_PTP is not yet supported
#endif

/* This driver does not use enhanced descriptors.  Enhanced descriptors must
 * be used, however, if time stamping or and/or IPv4 checksum offload is
 * supported.
 */

#undef CONFIG_TIVA_EMAC_ENHANCEDDESC
#undef CONFIG_TIVA_EMAC_HWCHECKSUM

/* Ethernet buffer sizes, number of buffers, and number of descriptors */

#ifndef CONFIG_TIVA_EMAC_NRXDESC
#  define CONFIG_TIVA_EMAC_NRXDESC 8
#endif

#ifndef CONFIG_TIVA_EMAC_NTXDESC
#  define CONFIG_TIVA_EMAC_NTXDESC 4
#endif

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, or 16 bytes (depending on buswidth).  We
 * will use the 16-byte alignment in all cases.
 */

#define OPTIMAL_EMAC_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 4 + 15) & ~15)

#if OPTIMAL_EMAC_BUFSIZE > EMAC_TDES1_TBS1_MASK
#  error OPTIMAL_EMAC_BUFSIZE is too large
#endif

#if (OPTIMAL_EMAC_BUFSIZE & 15) != 0
#  error OPTIMAL_EMAC_BUFSIZE must be aligned
#endif

#if OPTIMAL_EMAC_BUFSIZE != OPTIMAL_EMAC_BUFSIZE
#  warning You using an incomplete/untested configuration
#endif

/* We need at least one more free buffer than transmit buffers */

#define TIVA_EMAC_NFREEBUFFERS (CONFIG_TIVA_EMAC_NTXDESC+1)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_INFO
#  undef CONFIG_TIVA_ETHERNET_REGDEBUG
#endif

/* Clocking *****************************************************************/

/* Set MIIADDR CR bits depending on SysClk frequency */

#if SYSCLK_FREQUENCY >= 20000000 && SYSCLK_FREQUENCY < 35000000
#  define EMAC_MIIADDR_CR EMAC_MIIADDR_CR_20_35
#elif SYSCLK_FREQUENCY >= 35000000 && SYSCLK_FREQUENCY <= 64000000
#  define EMAC_MIIADDR_CR EMAC_MIIADDR_CR_35_60
#elif SYSCLK_FREQUENCY >= 60000000 && SYSCLK_FREQUENCY <= 104000000
#  define EMAC_MIIADDR_CR EMAC_MIIADDR_CR_60_100
#elif SYSCLK_FREQUENCY >= 100000000 && SYSCLK_FREQUENCY <= 150000000
#  define EMAC_MIIADDR_CR EMAC_MIIADDR_CR_100_150
#elif SYSCLK_FREQUENCY >= 150000000 && SYSCLK_FREQUENCY <= 168000000
#  define EMAC_MIIADDR_CR EMAC_MIIADDR_CR_150_168
#else
#  error SYSCLK_FREQUENCY not supportable
#endif

/* Timing *******************************************************************/

/* TX timeout = 1 minute */

#define TIVA_TXTIMEOUT   (60*CLK_TCK)

/* PHY reset/configuration delays in milliseconds */

#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT  (0x0004ffff)
#define PHY_WRITE_TIMEOUT (0x0004ffff)
#define PHY_RETRY_TIMEOUT (0x0004ffff)

/* Register values **********************************************************/

/* Clear the MACCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * EMAC_CFG_RE       Bit 2:  Receiver enable
 * EMAC_CFG_TE       Bit 3:  Transmitter enable
 * EMAC_CFG_DC       Bit 4:  Deferral check
 * EMAC_CFG_BL       Bits 5-6: Back-off limit
 * EMAC_CFG_ACS      Bit 7:  Automatic pad/CRC stripping
 * EMAC_CFG_DR       Bit 9:  Retry disable
 * EMAC_CFG_IPC      Bit 10: IPv4 checksum offload
 * EMAC_CFG_DUPM     Bit 11: Duplex mode
 * EMAC_CFG_LOOPBM   Bit 12: Loopback mode
 * EMAC_CFG_DRO      Bit 13: Receive own disable
 * EMAC_CFG_FES      Bit 14: Fast Ethernet speed
 * EMAC_CFG_PS       Bit 15: Port Select
 * EMAC_CFG_DISCRS   Bit 16: Carrier sense disable
 * EMAC_CFG_IFG      Bits 17-19: Interframe gap
 * EMAC_CFG_JFEN     Bit 20: Jumbo Frame Enable
 * EMAC_CFG_JD       Bit 22: Jabber disable
 * EMAC_CFG_WDDIS    Bit 23: Watchdog disable
 * EMAC_CFG_CST      Bit 25: CRC stripping for Type frames
 * EMAC_CFG_TWOKPEN  Bit 27: IEEE 802
 * EMAC_CFG_SADDR    Bits 28-30: Source Address Insertion/Replacement Control
 */

#define MACCR_CLEAR_BITS \
  (EMAC_CFG_RE | EMAC_CFG_TE | EMAC_CFG_DC | EMAC_CFG_BL_MASK | \
   EMAC_CFG_ACS | EMAC_CFG_DR | EMAC_CFG_IPC | EMAC_CFG_DUPM | \
   EMAC_CFG_LOOPBM | EMAC_CFG_DRO | EMAC_CFG_FES | EMAC_CFG_DISCRS | \
   EMAC_CFG_IFG_MASK | EMAC_CFG_JD | EMAC_CFG_WDDIS | EMAC_CFG_CST)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * EMAC_CFG_RE       Receiver enable             0 (disabled)
 * EMAC_CFG_TE       Transmitter enable          0 (disabled)
 * EMAC_CFG_DC       Deferral check              0 (disabled)
 * EMAC_CFG_BL       Back-off limit              0 (10)
 * EMAC_CFG_ACS      Automatic pad/CRC stripping 0 (disabled)
 * EMAC_CFG_DR       Retry disable               1 (disabled)
 * EMAC_CFG_IPC      IPv4 checksum offload       Depends on
 *                                               CONFIG_TIVA_EMAC_HWCHECKSUM
 * EMAC_CFG_LOOPBM   Loopback mode               0 (disabled)
 * EMAC_CFG_DRO      Receive own disable         0 (enabled)
 * EMAC_CFG_PS       Port Select                   (read-only)
 * EMAC_CFG_DISCRS   Carrier sense disable       0 (enabled)
 * EMAC_CFG_IFG      Interframe gap              0 (96 bits)
 * EMAC_CFG_JFEN     Jumbo Frame Enable          0 (jumbo frame create error)
 * EMAC_CFG_JD       Jabber disable              0 (enabled)
 * EMAC_CFG_WDDIS    Watchdog disable            0 (enabled)
 * EMAC_CFG_CST      Type frames CRC stripping   0 (disabled, F2/F4 only)
 * EMAC_CFG_TWOKPEN  IEEE 802                    0 (>1518 == giant frame)
 * EMAC_CFG_SADDR    Source Address Insertion or
 *                   Replacement Control
 *
 * The following are set conditionally based on mode and speed.
 *
 * EMAC_CFG_DUPM     Duplex mode                    Depends on priv->fduplex
 * EMAC_CFG_FES      Fast Ethernet speed            Depends on priv->mbps100
 */

#ifdef CONFIG_TIVA_EMAC_HWCHECKSUM
#  define MACCR_SET_BITS \
     (EMAC_CFG_BL_10 | EMAC_CFG_DR | EMAC_CFG_IPC | EMAC_CFG_IFG_96)
#else
#  define MACCR_SET_BITS \
     (EMAC_CFG_BL_10 | EMAC_CFG_DR | EMAC_CFG_IFG_96)
#endif

/* Clear the MACCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * EMAC_FRAMEFLTR_PR     Bit 0: Promiscuous mode
 * EMAC_FRAMEFLTR_HUC    Bit 1: Hash unicast
 * EMAC_FRAMEFLTR_HMC    Bit 2: Hash multicast
 * EMAC_FRAMEFLTR_DAIF   Bit 3: Destination address inverse filtering
 * EMAC_FRAMEFLTR_PM     Bit 4: Pass all multicast
 * EMAC_FRAMEFLTR_DBF    Bit 5: Broadcast frames disable
 * EMAC_FRAMEFLTR_PCF    Bits 6-7: Pass control frames
 * EMAC_FRAMEFLTR_SAIF   Bit 8: Source address inverse filtering
 * EMAC_FRAMEFLTR_SAF    Bit 9: Source address filter
 * EMAC_FRAMEFLTR_HPF    Bit 10: Hash or perfect filter
 * EMAC_FRAMEFLTR_VTFE   Bit 16: VLAN Tag Filter Enable
 * EMAC_FRAMEFLTR_RA     Bit 31: Receive all
 */

#define FRAMEFLTR_CLEAR_BITS \
  (EMAC_FRAMEFLTR_PR | EMAC_FRAMEFLTR_HUC | EMAC_FRAMEFLTR_HMC | EMAC_FRAMEFLTR_DAIF | \
   EMAC_FRAMEFLTR_PM | EMAC_FRAMEFLTR_DBF | EMAC_FRAMEFLTR_PCF_MASK | EMAC_FRAMEFLTR_SAIF | \
   EMAC_FRAMEFLTR_SAF | EMAC_FRAMEFLTR_HPF | EMAC_FRAMEFLTR_RA)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * EMAC_FRAMEFLTR_PR   Promiscuous mode       0 (disabled)
 * EMAC_FRAMEFLTR_HUC  Hash unicast           0 (perfect dest filtering)
 * EMAC_FRAMEFLTR_HMC  Hash multicast         0 (perfect dest filtering)
 * EMAC_FRAMEFLTR_DAIF Destination address    0 (normal)
 *                     inverse filtering
 * EMAC_FRAMEFLTR_PM   Pass all multicast     0 (Depends on HM bit)
 * EMAC_FRAMEFLTR_DBF  Broadcast frames       0 (enabled)
 *                     disable
 * EMAC_FRAMEFLTR_PCF  Pass control frames    1 (block all but PAUSE)
 * EMAC_FRAMEFLTR_SAIF Source address         0 (not used)
 *                     inverse filtering
 * EMAC_FRAMEFLTR_SAF  Source address filter  0 (disabled)
 * EMAC_FRAMEFLTR_HPF  Hash or perfect filter 0 (Only matching frames passed)
 * EMAC_FRAMEFLTR_VTFE VLAN Tag Filter Enable 0 (VLAN tag ignored)
 * EMAC_FRAMEFLTR_RA   Receive all            0 (disabled)
 */

#define FRAMEFLTR_SET_BITS (EMAC_FRAMEFLTR_PCF_PAUSE)

/* Clear the FLOWCTL bits that will be setup during MAC initialization (or
 * that are cleared unconditionally).  Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * EMAC_FLOWCTL_FCBBPA   Bit 0: Flow control busy/back pressure activate
 * EMAC_FLOWCTL_TFE      Bit 1: Transmit flow control enable
 * EMAC_FLOWCTL_RFE      Bit 2: Receive flow control enable
 * EMAC_FLOWCTL_UP       Bit 3: Unicast pause frame detect
 * EMAC_FLOWCTL_PLT      Bits 4-5: Pause low threshold
 * EMAC_FLOWCTL_DZQP     Bit 7: Zero-quanta pause disable
 * EMAC_FLOWCTL_PT       Bits 16-31: Pause time
 */

#define FLOWCTL_CLEAR_MASK \
  (EMAC_FLOWCTL_FCBBPA | EMAC_FLOWCTL_TFE | EMAC_FLOWCTL_RFE | EMAC_FLOWCTL_UP | \
   EMAC_FLOWCTL_PLT_MASK | EMAC_FLOWCTL_DZQP | EMAC_FLOWCTL_PT_MASK)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * EMAC_FLOWCTL_FCBBPA Flow control busy/back 0     (no pause control frame)
 *                     pressure activate
 * EMAC_FLOWCTL_TFE    Transmit flow control enable 0 (disabled)
 * EMAC_FLOWCTL_RFE    Receive flow control enable  0 (disabled)
 * EMAC_FLOWCTL_UP     Unicast pause frame detect   0 (disabled)
 * EMAC_FLOWCTL_PLT    Pause low threshold          0 (pause time - 4)
 * EMAC_FLOWCTL_DZQP   Zero-quanta pause disable    1 (disabled)
 * EMAC_FLOWCTL_PT     Pause time                   0
 */

#define FLOWCTL_SET_MASK (EMAC_FLOWCTL_PLT_M4 | EMAC_FLOWCTL_DZQP)

/* Clear the DMAOPMODE bits that will be setup during MAC initialization (or
 * that are cleared unconditionally).  Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * EMAC_DMAOPMODE_SR     Bit 1:  Start/stop receive
 * EMAC_DMAOPMODE_OSF    Bit 2:  Operate on second frame
 * EMAC_DMAOPMODE_RTC    Bits 3-4: Receive threshold control
 * EMAC_DMAOPMODE_DGF    Bit 5:  Drop giant frames enable
 * EMAC_DMAOPMODE_FUF    Bit 6:  Forward undersized good frames
 * EMAC_DMAOPMODE_FEF    Bit 7:  Forward error frames
 * EMAC_DMAOPMODE_ST     Bit 13: Start/stop transmission
 * EMAC_DMAOPMODE_TTC    Bits 14-16: Transmit threshold control
 * EMAC_DMAOPMODE_FTF    Bit 20: Flush transmit FIFO
 * EMAC_DMAOPMODE_TSF    Bit 21: Transmit store and forward
 * EMAC_DMAOPMODE_DFF    Bit 24: Disable flushing of received frames
 * EMAC_DMAOPMODE_RSF    Bit 25: Receive store and forward
 * EMAC_DMAOPMODE_DT     Bit 26: Dropping of TCP/IP checksum error
 *                               frames disable
 */

#define DMAOPMODE_CLEAR_MASK \
  (EMAC_DMAOPMODE_SR | EMAC_DMAOPMODE_OSF | EMAC_DMAOPMODE_RTC_MASK | EMAC_DMAOPMODE_DGF | \
   EMAC_DMAOPMODE_FUF | EMAC_DMAOPMODE_FEF | EMAC_DMAOPMODE_ST | EMAC_DMAOPMODE_TTC_MASK | \
   EMAC_DMAOPMODE_FTF | EMAC_DMAOPMODE_TSF | EMAC_DMAOPMODE_DFF | EMAC_DMAOPMODE_RSF | \
   EMAC_DMAOPMODE_DT)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * EMAC_DMAOPMODE_SR  Start/stop receive          0 (not running)
 * EMAC_DMAOPMODE_OSF Operate on second frame     1 (enabled)
 * EMAC_DMAOPMODE_RTC Receive threshold control   0 (64 bytes)
 * EMAC_DMAOPMODE_FUF Forward undersized          0 (disabled)
 *                    good frames
 * EMAC_DMAOPMODE_FEF Forward error frames        0 (disabled)
 * EMAC_DMAOPMODE_ST  Start/stop transmission     0 (not running)
 * EMAC_DMAOPMODE_TTC Transmit threshold control  0 (64 bytes)
 * EMAC_DMAOPMODE_FTF Flush transmit FIFO         0 (no flush)
 * EMAC_DMAOPMODE_TSF Transmit store and forward  Depends on
 *                                                CONFIG_TIVA_EMAC_HWCHECKSUM
 * EMAC_DMAOPMODE_DFF Disable flushing ofs        0 (enabled)
 *                    received frame
 * EMAC_DMAOPMODE_RSF Receive store and forward   Depends on
 *                                                CONFIG_TIVA_EMAC_HWCHECKSUM
 * EMAC_DMAOPMODE_DT  Dropping of TCP/IP checksum Depends on
 *                    error frames disable        CONFIG_TIVA_EMAC_HWCHECKSUM
 *
 * When the checksum offload feature is enabled, we need to enable the Store
 * and Forward mode: the store and forward guarantee that a whole frame is
 * stored in the FIFO, so the MAC can insert/verify the checksum, if the
 * checksum is OK the DMA can handle the frame otherwise the frame is dropped
 */

#ifdef CONFIG_TIVA_EMAC_HWCHECKSUM
#  define DMAOPMODE_SET_MASK \
    (EMAC_DMAOPMODE_OSF | EMAC_DMAOPMODE_RTC_64 | EMAC_DMAOPMODE_TTC_64 | \
     EMAC_DMAOPMODE_TSF | EMAC_DMAOPMODE_RSF)
#else
#  define DMAOPMODE_SET_MASK \
    (EMAC_DMAOPMODE_OSF | EMAC_DMAOPMODE_RTC_64 | EMAC_DMAOPMODE_TTC_64 | \
     EMAC_DMAOPMODE_DT)
#endif

/* Clear the DMABUSMOD bits that will be setup during MAC initialization (or
 * that are cleared unconditionally).  Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * EMAC_DMABUSMOD_SWR    Bit 0: Software reset
 * EMAC_DMABUSMOD_DA     Bit 1: DMA Arbitration
 * EMAC_DMABUSMOD_DSL    Bits 2-6: Descriptor skip length
 * EMAC_DMABUSMOD_ATDS   Bit 7: Enhanced descriptor format enable
 * EMAC_DMABUSMOD_PBL    Bits 8-13: Programmable burst length
 * EMAC_DMABUSMOD_PR     Bits 14-15: RX TX priority ratio
 * EMAC_DMABUSMOD_FB     Bit 16: Fixed burst
 * EMAC_DMABUSMOD_RPBL   Bits 17-22: RX DMA programmable bust length
 * EMAC_DMABUSMOD_USP    Bit 23: Use separate PBL
 * EMAC_DMABUSMOD_8XPBL  Bit 24: 8x programmable burst length mode
 * EMAC_DMABUSMOD_AAL    Bit 25: Address-aligned beats
 * EMAC_DMABUSMOD_MB     Bit 26: Mixed burst (F2/F4 only)
 * EMAC_DMABUSMOD_TXPR   Bit 27: Transmit Priority
 * EMAC_DMABUSMOD_RIB    Bit 31: Rebuild Burst
 */

#define DMABUSMOD_CLEAR_MASK \
  (EMAC_DMABUSMOD_SWR | EMAC_DMABUSMOD_DA | EMAC_DMABUSMOD_DSL_MASK | \
   EMAC_DMABUSMOD_ATDS | EMAC_DMABUSMOD_PBL_MASK | EMAC_DMABUSMOD_PR_MASK | \
   EMAC_DMABUSMOD_FB | EMAC_DMABUSMOD_RPBL_MASK | EMAC_DMABUSMOD_USP | \
   EMAC_DMABUSMOD_8XPBL | EMAC_DMABUSMOD_AAL | EMAC_DMABUSMOD_MB | \
   EMAC_DMABUSMOD_TXPR | EMAC_DMABUSMOD_RIB)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * EMAC_DMABUSMOD_SWR   Software reset         0 (no reset)
 * EMAC_DMABUSMOD_DA    DMA Arbitration        1 (fixed priority)
 * EMAC_DMABUSMOD_DSL   Descriptor skip length 0
 * EMAC_DMABUSMOD_ATDS  Enhanced descriptor    Depends on
 *                      format enable          CONFIG_TIVA_EMAC_ENHANCEDDESC
 * EMAC_DMABUSMOD_PBL   Programmable burst     Depends on EMAC_DMA_RXBURST
 *                      length
 * EMAC_DMABUSMOD_PR    RX TX priority ratio   0 1:1
 * EMAC_DMABUSMOD_FB    Fixed burst            0 (disabled)
 * EMAC_DMABUSMOD_RPBL  RX DMA programmable    Depends on EMAC_DMA_TXBURST
 *                      burst length
 * EMAC_DMABUSMOD_USP   Use separate PBL       Depends on EMAC_DMA_RX/TXBURST
 * EMAC_DMABUSMOD_8XPBL 8x programmable burst  Depends on EMAC_DMA_RX/TXBURST
 *                      length mode
 * EMAC_DMABUSMOD_AAL   Address-aligned beats  0(disabled)
 * EMAC_DMABUSMOD_MB    Mixed burst            1(enabled)
 * EMAC_DMABUSMOD_TXPR  Transmit Priority      0(RX DMA has priority over TX)
 * EMAC_DMABUSMOD_RIB   Rebuild Burst          0
 */

#define EMAC_DMA_RXBURST          4
#define EMAC_DMA_TXBURST          4

#if EMAC_DMA_RXBURST > 32 || EMAC_DMA_TXBURST > 32
#  define __EMAC_DMABUSMOD_8XPBL  0
#  define __EMAC_DMA_RXBURST      EMAC_DMA_RXBURST
#  define __EMAC_DMA_TXBURST      EMAC_DMA_TXBURST
#else
  /* Divide both burst lengths by 8 and set the 8X burst length multiplier */

#  define __EMAC_DMABUSMOD_8XPBL  EMAC_DMABUSMOD_8XPBL
#  define __EMAC_DMA_RXBURST      (EMAC_DMA_RXBURST >> 3)
#  define __EMAC_DMA_TXBURST      (EMAC_DMA_TXBURST >> 3)
#endif

#define __EMAC_DMABUSMOD_PBL      EMAC_DMABUSMOD_PBL(__EMAC_DMA_RXBURST)

/* Are the receive and transmit burst lengths the same? */

#if __EMAC_DMA_RXBURST == __EMAC_DMA_TXBURST
  /* Yes.. Set up to use a single burst length */

#  define __EMAC_DMABUSMOD_USP    0
#  define __EMAC_DMABUSMOD_RPBL   0
#else
  /* No.. Use separate burst lengths for each */

#  define __EMAC_DMABUSMOD_USP    EMAC_DMABUSMOD_USP
#  define __EMAC_DMABUSMOD_RPBL   EMAC_DMABUSMOD_RPBL(__EMAC_DMA_TXBURST)
#endif

#ifdef CONFIG_TIVA_EMAC_ENHANCEDDESC
#  define __EMAC_DMABUSMOD_ATDS  EMAC_DMABUSMOD_ATDS
#else
#  define __EMAC_DMABUSMOD_ATDS  0
#endif

#define DMABUSMOD_SET_MASK \
   (EMAC_DMABUSMOD_DA | EMAC_DMABUSMOD_DSL(0) | __EMAC_DMABUSMOD_ATDS | \
    __EMAC_DMABUSMOD_PBL | __EMAC_DMABUSMOD_RPBL | __EMAC_DMABUSMOD_USP | \
    __EMAC_DMABUSMOD_8XPBL | EMAC_DMABUSMOD_MB)

/* Interrupt bit sets *******************************************************/

/* All interrupts in the normal and abnormal interrupt summary. Early
 * transmit interrupt (ETI) is excluded from the abnormal set because it
 * causes too many interrupts and is not interesting.
 */

#define EMAC_DMAINT_NORMAL \
  (EMAC_DMAINT_TI | EMAC_DMAINT_TBUI | EMAC_DMAINT_RI | EMAC_DMAINT_ERI)

#define EMAC_DMAINT_ABNORMAL \
  (EMAC_DMAINT_TPSI | EMAC_DMAINT_TJTI | EMAC_DMAINT_OVFI | EMAC_EMAINT_UNFI | \
   EMAC_DMAINT_RBUI | EMAC_DMAINT_RPSI | EMAC_DMAINT_RWTI | /* EMAC_DMAINT_ETI | */ \
   EMAC_DMAINT_FBEI)

/* Normal receive, transmit, error interrupt enable bit sets */

#define EMAC_DMAINT_RECV_ENABLE    (EMAC_DMAINT_NIS | EMAC_DMAINT_RI)
#define EMAC_DMAINT_XMIT_ENABLE    (EMAC_DMAINT_NIS | EMAC_DMAINT_TI)
#define EMAC_DMAINT_XMIT_DISABLE   (EMAC_DMAINT_TI)

#ifdef CONFIG_DEBUG_NET
#  define EMAC_DMAINT_ERROR_ENABLE (EMAC_DMAINT_AIS | EMAC_DMAINT_ABNORMAL)
#else
#  define EMAC_DMAINT_ERROR_ENABLE (0)
#endif

/* Helpers ******************************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The tiva_ethmac_s encapsulates all state information for a single hardware
 * interface
 */

struct tiva_ethmac_s
{
  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  uint8_t              mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t              fduplex : 1; /* Full (vs. half) duplex */
  struct wdog_s        txtimeout;   /* TX timeout timer */
  struct work_s        irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s        pollwork;    /* For deferring poll work to the work queue */

#ifdef CONFIG_TIVA_PHY_INTERRUPTS
  xcpt_t               handler;     /* Attached PHY interrupt handler */
  void                *arg;         /* Argument that accompanies the interrupt */
#endif

  /* This holds the information visible to the NuttX network */

  struct net_driver_s  dev;         /* Interface understood by network subsystem */

  /* Used to track transmit and receive descriptors */

  struct emac_txdesc_s *txhead;     /* Next available TX descriptor */
  struct emac_rxdesc_s *rxhead;     /* Next available RX descriptor */

  struct emac_txdesc_s *txtail;     /* First "in_flight" TX descriptor */
  struct emac_rxdesc_s *rxcurr;     /* First RX descriptor of the segment */
  uint16_t             segments;    /* RX segment count */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */
  sq_queue_t           freeb;       /* The free buffer list */

  /* Descriptor allocations */

  struct emac_rxdesc_s rxtable[CONFIG_TIVA_EMAC_NRXDESC];
  struct emac_txdesc_s txtable[CONFIG_TIVA_EMAC_NTXDESC];

  /* Buffer allocations */

  uint8_t rxbuffer[CONFIG_TIVA_EMAC_NRXDESC*OPTIMAL_EMAC_BUFSIZE];
  uint8_t alloc[TIVA_EMAC_NFREEBUFFERS*OPTIMAL_EMAC_BUFSIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tiva_ethmac_s g_tiva_ethmac[TIVA_NETHCONTROLLERS];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#if defined(CONFIG_TIVA_ETHERNET_REGDEBUG) && defined(CONFIG_DEBUG_INFO)
static uint32_t tiva_getreg(uint32_t addr);
static void tiva_putreg(uint32_t val, uint32_t addr);
static void tiva_checksetup(void);
#else
# define tiva_getreg(addr)      getreg32(addr)
# define tiva_putreg(val,addr)  putreg32(val,addr)
# define tiva_checksetup()
#endif

/* Free buffer management */

static void tiva_initbuffer(struct tiva_ethmac_s *priv);
static inline uint8_t *tiva_allocbuffer(struct tiva_ethmac_s *priv);
static inline void tiva_freebuffer(struct tiva_ethmac_s *priv,
              uint8_t *buffer);
static inline bool tiva_isfreebuffer(struct tiva_ethmac_s *priv);

/* Common TX logic */

static int  tiva_transmit(struct tiva_ethmac_s *priv);
static int  tiva_txpoll(struct net_driver_s *dev);
static void tiva_dopoll(struct tiva_ethmac_s *priv);

/* Interrupt handling */

static void tiva_enableint(struct tiva_ethmac_s *priv, uint32_t ierbit);
static void tiva_disableint(struct tiva_ethmac_s *priv,
              uint32_t ierbit);

static void tiva_freesegment(struct tiva_ethmac_s *priv,
              struct emac_rxdesc_s *rxfirst, int segments);
static int  tiva_recvframe(struct tiva_ethmac_s *priv);
static void tiva_receive(struct tiva_ethmac_s *priv);
static void tiva_freeframe(struct tiva_ethmac_s *priv);
static void tiva_txdone(struct tiva_ethmac_s *priv);

static void tiva_interrupt_work(void *arg);
static int  tiva_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void tiva_txtimeout_work(void *arg);
static void tiva_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  tiva_ifup(struct net_driver_s *dev);
static int  tiva_ifdown(struct net_driver_s *dev);

static void tiva_txavail_work(void *arg);
static int  tiva_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  tiva_addmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int  tiva_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  tiva_ioctl(struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif

/* Descriptor Initialization */

static void tiva_txdescinit(struct tiva_ethmac_s *priv);
static void tiva_rxdescinit(struct tiva_ethmac_s *priv);

/* PHY Initialization */

#ifdef CONFIG_TIVA_PHY_INTERRUPTS
static void tiva_phy_intenable(bool enable);
#endif
static int  tiva_phyread(uint16_t phydevaddr, uint16_t phyregaddr,
              uint16_t *value);
static int  tiva_phywrite(uint16_t phydevaddr, uint16_t phyregaddr,
              uint16_t value);
static int  tiva_phyinit(struct tiva_ethmac_s *priv);

/* MAC/DMA Initialization */

static void tiva_phy_configure(struct tiva_ethmac_s *priv);
static inline void tiva_phy_initialize(struct tiva_ethmac_s *priv);

static void tiva_ethreset(struct tiva_ethmac_s *priv);
static int  tiva_macconfig(struct tiva_ethmac_s *priv);
static void tiva_macaddress(struct tiva_ethmac_s *priv);
#ifdef CONFIG_NET_ICMPv6
static void tiva_ipv6multicast(struct tiva_ethmac_s *priv);
#endif
static int  tiva_macenable(struct tiva_ethmac_s *priv);
static int  tive_emac_configure(struct tiva_ethmac_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_getreg
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

#if defined(CONFIG_TIVA_ETHERNET_REGDEBUG) && defined(CONFIG_DEBUG_INFO)
static uint32_t tiva_getreg(uint32_t addr)
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
              _info("...\n");
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

          _info("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  _info("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: tiva_putreg
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

#if defined(CONFIG_TIVA_ETHERNET_REGDEBUG) && defined(CONFIG_DEBUG_INFO)
static void tiva_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  _info("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: tiva_checksetup
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

#if defined(CONFIG_TIVA_ETHERNET_REGDEBUG) && defined(CONFIG_DEBUG_INFO)
static void tiva_checksetup(void)
{
}
#endif

/****************************************************************************
 * Function: tiva_initbuffer
 *
 * Description:
 *   Initialize the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during early driver initialization before Ethernet interrupts
 *   are enabled.
 *
 ****************************************************************************/

static void tiva_initbuffer(struct tiva_ethmac_s *priv)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = priv->alloc;
       i < TIVA_EMAC_NFREEBUFFERS;
       i++, buffer += OPTIMAL_EMAC_BUFSIZE)
    {
      sq_addlast((sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: tiva_allocbuffer
 *
 * Description:
 *   Allocate one buffer from the free buffer list.
 *
 * Input Parameters:
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

static inline uint8_t *tiva_allocbuffer(struct tiva_ethmac_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: tiva_freebuffer
 *
 * Description:
 *   Return a buffer to the free buffer list.
 *
 * Input Parameters:
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

static inline void tiva_freebuffer(struct tiva_ethmac_s *priv,
                                   uint8_t *buffer)
{
  /* Free the buffer by adding it to the end of the free buffer list */

  sq_addlast((sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: tiva_isfreebuffer
 *
 * Description:
 *   Return TRUE if the free buffer list is not empty.
 *
 * Input Parameters:
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

static inline bool tiva_isfreebuffer(struct tiva_ethmac_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
}

/****************************************************************************
 * Function: tiva_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
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

static int tiva_transmit(struct tiva_ethmac_s *priv)
{
  struct emac_txdesc_s *txdesc;
  struct emac_txdesc_s *txfirst;

  /* The internal (optimal) network buffer size may be configured to be
   * larger than the Ethernet buffer size.
   */

#if OPTIMAL_EMAC_BUFSIZE > OPTIMAL_EMAC_BUFSIZE
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

  ninfo("d_len: %d d_buf: %p txhead: %p tdes0: %08" PRIx32 "\n",
        priv->dev.d_len, priv->dev.d_buf, txdesc, txdesc->tdes0);

  DEBUGASSERT(txdesc && (txdesc->tdes0 & EMAC_TDES0_OWN) == 0);

  /* Is the size to be sent greater than the size of the Ethernet buffer? */

  DEBUGASSERT(priv->dev.d_len > 0 && priv->dev.d_buf != NULL);

#if OPTIMAL_EMAC_BUFSIZE > OPTIMAL_EMAC_BUFSIZE
  if (priv->dev.d_len > OPTIMAL_EMAC_BUFSIZE)
    {
      /* Yes... how many buffers will be need to send the packet? */

      bufcount = (priv->dev.d_len + (OPTIMAL_EMAC_BUFSIZE - 1)) /
                 OPTIMAL_EMAC_BUFSIZE;
      lastsize = priv->dev.d_len - (bufcount - 1) * OPTIMAL_EMAC_BUFSIZE;

      ninfo("bufcount: %d lastsize: %d\n", bufcount, lastsize);

      /* Set the first segment bit in the first TX descriptor */

      txdesc->tdes0 |= EMAC_TDES0_FS;

      /* Set up all but the last TX descriptor */

      buffer = priv->dev.d_buf;

      for (i = 0; i < bufcount; i++)
        {
          /* This could be a normal event but the design does not handle it */

          DEBUGASSERT((txdesc->tdes0 & EMAC_TDES0_OWN) == 0);

          /* Set the Buffer1 address pointer */

          txdesc->tdes2 = (uint32_t)buffer;

          /* Set the buffer size in all TX descriptors */

          if (i == (bufcount - 1))
            {
              /* This is the last segment.  Set the last segment bit in the
               * last TX descriptor and ask for an interrupt when this
               * segment transfer completes.
               */

              txdesc->tdes0 |= (EMAC_TDES0_LS | EMAC_TDES0_IC);

              /* This segment is, most likely, of fractional buffersize */

              txdesc->tdes1  = lastsize;
              buffer        += lastsize;
            }
          else
            {
              /* This is not the last segment.  We don't want an interrupt
               * when this segment transfer completes.
               */

              txdesc->tdes0 &= ~EMAC_TDES0_IC;

              /* The size of the transfer is the whole buffer */

              txdesc->tdes1  = OPTIMAL_EMAC_BUFSIZE;
              buffer        += OPTIMAL_EMAC_BUFSIZE;
            }

          /* Give the descriptor to DMA */

          txdesc->tdes0 |= EMAC_TDES0_OWN;
          txdesc         = (struct emac_txdesc_s *)txdesc->tdes3;
        }
    }
  else
#endif
    {
      /* The single descriptor is both the first and last segment.  And we do
       * want an interrupt when the transfer completes.
       */

      txdesc->tdes0 |= (EMAC_TDES0_FS | EMAC_TDES0_LS | EMAC_TDES0_IC);

      /* Set frame size */

      DEBUGASSERT(priv->dev.d_len <= CONFIG_NET_ETH_PKTSIZE);
      txdesc->tdes1 = priv->dev.d_len;

      /* Set the Buffer1 address pointer */

      txdesc->tdes2 = (uint32_t)priv->dev.d_buf;

      /* Set OWN bit of the TX descriptor tdes0.  This gives the buffer to
       * Ethernet DMA
       */

      txdesc->tdes0 |= EMAC_TDES0_OWN;

      /* Point to the next available TX descriptor */

      txdesc = (struct emac_txdesc_s *)txdesc->tdes3;
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
   * interrupts too. This is because receive events can trigger more un-
   * stoppable transmit events.
   */

  if (priv->inflight >= CONFIG_TIVA_EMAC_NTXDESC)
    {
      tiva_disableint(priv, EMAC_DMAINT_RI);
    }

  /* Check if the TX Buffer unavailable flag is set */

  if ((tiva_getreg(TIVA_EMAC_DMARIS) & EMAC_DMAINT_TBUI) != 0)
    {
      /* Clear TX Buffer unavailable flag */

      tiva_putreg(EMAC_DMAINT_TBUI, TIVA_EMAC_DMARIS);

      /* Resume DMA transmission */

      tiva_putreg(0, TIVA_EMAC_TXPOLLD);
    }

  /* Enable TX interrupts */

  tiva_enableint(priv, EMAC_DMAINT_TI);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, TIVA_TXTIMEOUT,
           tiva_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: tiva_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send. This is a callback from devif_poll().
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
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int tiva_txpoll(struct net_driver_s *dev)
{
  struct tiva_ethmac_s *priv =
    (struct tiva_ethmac_s *)dev->d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
        {
          arp_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->dev))
        {
          /* Send the packet */

          tiva_transmit(priv);
          DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

          /* Check if the next TX descriptor is owned by the Ethernet DMA or
           * CPU. We cannot perform the TX poll if we are unable to accept
           * another packet for transmission.
           *
           * In a race condition, EMAC_TDES0_OWN may be cleared BUT still
           * not available because tiva_freeframe() has not yet run. If
           * tiva_freeframe() has run, the buffer1 pointer (tdes2) will be
           * nullified (and inflight should be < CONFIG_TIVA_EMAC_NTXDESC).
           */

          if ((priv->txhead->tdes0 & EMAC_TDES0_OWN) != 0 ||
               priv->txhead->tdes2 != 0)
            {
              /* We have to terminate the poll if we have no more descriptors
               * available for another transfer.
               */

              return -EBUSY;
            }

          /* We have the descriptor, we can continue the poll. Allocate a new
           * buffer for the poll.
           */

          dev->d_buf = tiva_allocbuffer(priv);

          /* We can't continue the poll if we have no buffers */

          if (dev->d_buf == NULL)
            {
              /* Terminate the poll. */

              return -ENOMEM;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: tiva_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (tiva_txdone),
 *   2. When new TX data is available (tiva_txavail_process), and
 *   3. After a TX timeout to restart the sending process
 *      (tiva_txtimeout_process).
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

static void tiva_dopoll(struct tiva_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, EMAC_TDES0_OWN may be cleared BUT still
   * not available because tiva_freeframe() has not yet run. If
   * tiva_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_TIVA_EMAC_NTXDESC).
   */

  if ((priv->txhead->tdes0 & EMAC_TDES0_OWN) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then poll the network for new XMIT data.
       * Allocate a buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = tiva_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          devif_poll(dev, tiva_txpoll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              tiva_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }
}

/****************************************************************************
 * Function: tiva_enableint
 *
 * Description:
 *   Enable a "normal" interrupt
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

static void tiva_enableint(struct tiva_ethmac_s *priv, uint32_t ierbit)
{
  uint32_t regval;

  /* Enable the specified "normal" interrupt */

  regval  = tiva_getreg(TIVA_EMAC_DMAIM);
  regval |= (EMAC_DMAINT_NIS | ierbit);
  tiva_putreg(regval, TIVA_EMAC_DMAIM);
}

/****************************************************************************
 * Function: tiva_disableint
 *
 * Description:
 *   Disable a normal interrupt.
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

static void tiva_disableint(struct tiva_ethmac_s *priv, uint32_t ierbit)
{
  uint32_t regval;

  /* Disable the "normal" interrupt */

  regval  = tiva_getreg(TIVA_EMAC_DMAIM);
  regval &= ~ierbit;

  /* Are all "normal" interrupts now disabled? */

  if ((regval & EMAC_DMAINT_NORMAL) == 0)
    {
      /* Yes.. disable normal interrupts */

      regval &= ~EMAC_DMAINT_NIS;
    }

  tiva_putreg(regval, TIVA_EMAC_DMAIM);
}

/****************************************************************************
 * Function: tiva_freesegment
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the received frame.
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

static void tiva_freesegment(struct tiva_ethmac_s *priv,
                             struct emac_rxdesc_s *rxfirst, int segments)
{
  struct emac_rxdesc_s *rxdesc;
  int i;

  ninfo("rxfirst: %p segments: %d\n", rxfirst, segments);

  /* Set OWN bit in RX descriptors.  This gives the buffers back to DMA */

  rxdesc = rxfirst;
  for (i = 0; i < segments; i++)
    {
      rxdesc->rdes0 = EMAC_RDES0_OWN;
      rxdesc = (struct emac_rxdesc_s *)rxdesc->rdes3;
    }

  /* Reset the segment management logic */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Check if the RX Buffer unavailable flag is set */

  if ((tiva_getreg(TIVA_EMAC_DMARIS) & EMAC_DMAINT_RBUI) != 0)
    {
      /* Clear RBUS Ethernet DMA flag */

      tiva_putreg(EMAC_DMAINT_RBUI, TIVA_EMAC_DMARIS);

      /* Resume DMA reception */

      tiva_putreg(0, TIVA_EMAC_RXPOLLD);
    }
}

/****************************************************************************
 * Function: tiva_recvframe
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors of the received frame.
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Input Parameters:
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

static int tiva_recvframe(struct tiva_ethmac_s *priv)
{
  struct emac_rxdesc_s *rxdesc;
  struct emac_rxdesc_s *rxcurr;
  uint8_t *buffer;
  int i;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  /* Check if there are free buffers.  We cannot receive new frames in this
   * design unless there is at least one free buffer.
   */

  if (!tiva_isfreebuffer(priv))
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
  for (i = 0;
       (rxdesc->rdes0 & EMAC_RDES0_OWN) == 0 &&
        i < CONFIG_TIVA_EMAC_NRXDESC &&
        priv->inflight < CONFIG_TIVA_EMAC_NTXDESC;
       i++)
    {
      /* Check if this is the first segment in the frame */

      if ((rxdesc->rdes0 & EMAC_RDES0_FS) != 0 &&
          (rxdesc->rdes0 & EMAC_RDES0_LS) == 0)
        {
          priv->rxcurr   = rxdesc;
          priv->segments = 1;
        }

      /* Check if this is an intermediate segment in the frame */

      else if (((rxdesc->rdes0 & EMAC_RDES0_LS) == 0) &&
               ((rxdesc->rdes0 & EMAC_RDES0_FS) == 0))
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

          if ((rxdesc->rdes0 & EMAC_RDES0_ES) == 0)
            {
              struct net_driver_s *dev = &priv->dev;

              /* Get the Frame Length of the received packet: substruct 4
               * bytes of the CRC
               */

              dev->d_len = ((rxdesc->rdes0 & EMAC_RDES0_FL_MASK) >>
                            EMAC_RDES0_FL_SHIFT) - 4;

              /* Get a buffer from the free list.  We don't even check if
               * this is successful because we already assure the free
               * list is not empty above.
               */

              buffer = tiva_allocbuffer(priv);

              /* Take the buffer from the RX descriptor of the first free
               * segment, put it into the network device structure, then
               * replace the buffer in the RX descriptor with the newly
               * allocated buffer.
               */

              DEBUGASSERT(dev->d_buf == NULL);
              dev->d_buf    = (uint8_t *)rxcurr->rdes2;
              rxcurr->rdes2 = (uint32_t)buffer;

              /* Return success, remembering where we should re-start
               * scanning and resetting the segment scanning logic
               */

              priv->rxhead   = (struct emac_rxdesc_s *)rxdesc->rdes3;
              tiva_freesegment(priv, rxcurr, priv->segments);

              ninfo("rxhead: %p d_buf: %p d_len: %d\n",
                    priv->rxhead, dev->d_buf, dev->d_len);

              return OK;
            }
          else
            {
              /* Drop the frame that contains the errors, reset the segment
               * scanning logic, and continue scanning with the next frame.
               */

              nwarn("DROPPED: RX descriptor errors: %08" PRIx32 "\n",
                    rxdesc->rdes0);
              tiva_freesegment(priv, rxcurr, priv->segments);
            }
        }

      /* Try the next descriptor */

      rxdesc = (struct emac_rxdesc_s *)rxdesc->rdes3;
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
 * Function: tiva_receive
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
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void tiva_receive(struct tiva_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while tiva_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  while (tiva_recvframe(priv) == OK)
    {
#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

     pkt_input(&priv->dev);
#endif

      /* Check if the packet is a valid size for the network buffer
       * configuration (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_ETH_PKTSIZE)
        {
          nwarn("DROPPED: Too big: %d\n", dev->d_len);
        }
      else

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&priv->dev);
          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
              if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
                {
                  arp_out(&priv->dev);
                }
#ifdef CONFIG_NET_IPv6
              else
                {
                  neighbor_out(&priv->dev);
                }
#endif

              /* And send the packet */

              tiva_transmit(priv);
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

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
              if (IFF_IS_IPv4(priv->dev.d_flags))
                {
                  arp_out(&priv->dev);
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
                {
                  neighbor_out(&priv->dev);
                }
#endif

              /* And send the packet */

              tiva_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Handle ARP packet */

          arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              tiva_transmit(priv);
            }
        }
      else
#endif
        {
          nwarn("DROPPED: Unknown type: %04x\n", BUF->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          tiva_freebuffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Function: tiva_freeframe
 *
 * Description:
 *   Scans the TX descriptors and frees the buffers of completed transfers.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void tiva_freeframe(struct tiva_ethmac_s *priv)
{
  struct emac_txdesc_s *txdesc;
  int i;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txdesc = priv->txtail;
  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

      for (i = 0; (txdesc->tdes0 & EMAC_TDES0_OWN) == 0; i++)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          ninfo("txtail: %p tdes0: %08" PRIx32 " tdes2: %08" PRIx32
                " tdes3: %08" PRIx32 "\n",
                txdesc, txdesc->tdes0, txdesc->tdes2, txdesc->tdes3);

          DEBUGASSERT(txdesc->tdes2 != 0);

          /* Check if this is the first segment of a TX frame. */

          if ((txdesc->tdes0 & EMAC_TDES0_FS) != 0)
            {
              /* Yes.. Free the buffer */

              tiva_freebuffer(priv, (uint8_t *)txdesc->tdes2);
            }

          /* In any event, make sure that TDES2 is nullified. */

          txdesc->tdes2 = 0;

          /* Check if this is the last segment of a TX frame */

          if ((txdesc->tdes0 & EMAC_TDES0_LS) != 0)
            {
              /* Yes.. Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If all of the TX descriptors were in-flight,
               * then RX interrupts may have been disabled...
               * we can re-enable them now.
               */

              tiva_enableint(priv, EMAC_DMAINT_RI);

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return;
                }
            }

          /* Try the next descriptor in the TX chain */

          txdesc = (struct emac_txdesc_s *)txdesc->tdes3;
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
 * Function: tiva_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void tiva_txdone(struct tiva_ethmac_s *priv)
{
  struct net_driver_s *dev  = &priv->dev;

  DEBUGASSERT(priv->txtail != NULL);

  /* Scan the TX descriptor change, returning buffers to free list */

  tiva_freeframe(priv);
  dev->d_buf = NULL;
  dev->d_len = 0;

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      /* Cancel the TX timeout */

      wd_cancel(&priv->txtimeout);

      /* And disable further TX interrupts. */

      tiva_disableint(priv, EMAC_DMAINT_TI);
    }

  /* Then poll the network for new XMIT data */

  tiva_dopoll(priv);
}

/****************************************************************************
 * Function: tiva_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void tiva_interrupt_work(void *arg)
{
  struct tiva_ethmac_s *priv = (struct tiva_ethmac_s *)arg;
  uint32_t dmaris;

  DEBUGASSERT(priv);

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dmaris = tiva_getreg(TIVA_EMAC_DMARIS);

  /* Mask only enabled interrupts.  This depends on the fact that the
   * interrupt related bits (0-16) correspond in these two registers.
   */

  dmaris &= tiva_getreg(TIVA_EMAC_DMAIM);

  /* Check if there are pending "normal" interrupts */

  if ((dmaris & EMAC_DMAINT_NIS) != 0)
    {
      /* Yes.. Check if we received an incoming packet, if so, call
       * tiva_receive()
       */

      if ((dmaris & EMAC_DMAINT_RI) != 0)
        {
          /* Clear the pending receive interrupt */

          tiva_putreg(EMAC_DMAINT_RI, TIVA_EMAC_DMARIS);

          /* Handle the received package */

          tiva_receive(priv);
        }

      /* Check if a packet transmission just completed.  If so, call
       * tiva_txdone(). This may disable further TX interrupts if there
       * are no pending transmissions.
       */

      if ((dmaris & EMAC_DMAINT_TI) != 0)
        {
          /* Clear the pending receive interrupt */

          tiva_putreg(EMAC_DMAINT_TI, TIVA_EMAC_DMARIS);

          /* Check if there are pending transmissions */

          tiva_txdone(priv);
        }

      /* Clear the pending normal summary interrupt */

      tiva_putreg(EMAC_DMAINT_NIS, TIVA_EMAC_DMARIS);
    }

  /* Handle error interrupt only if CONFIG_DEBUG_NET is eanbled */

#ifdef CONFIG_DEBUG_NET

  /* Check if there are pending "abnormal" interrupts */

  if ((dmaris & EMAC_DMAINT_AIS) != 0)
    {
      /* Just let the user know what happened */

      nerr("ERROR: Abnormal event(s): %08x\n", dmaris);

      /* Clear all pending abnormal events */

      tiva_putreg(EMAC_DMAINT_ABNORMAL, TIVA_EMAC_DMARIS);

      /* Clear the pending abnormal summary interrupt */

      tiva_putreg(EMAC_DMAINT_AIS, TIVA_EMAC_DMARIS);
    }
#endif

  net_unlock();

  /* Re-enable Ethernet interrupts at the NVIC */

  up_enable_irq(TIVA_IRQ_ETHCON);
}

/****************************************************************************
 * Function: tiva_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_interrupt(int irq, void *context, void *arg)
{
  struct tiva_ethmac_s *priv = &g_tiva_ethmac[0];
  uint32_t dmaris;

  /* Get the raw interrupt status. */

  dmaris = tiva_getreg(TIVA_EMAC_DMARIS);
  if (dmaris != 0)
    {
      /* Disable further Ethernet interrupts.  Because Ethernet interrupts
       * are also disabled if the TX timeout event occurs, there can be no
       * race condition here.
       */

      up_disable_irq(TIVA_IRQ_ETHCON);

      /* Check if a packet transmission just completed. */

      if ((dmaris & EMAC_DMAINT_TI) != 0)
        {
          /* If a TX transfer just completed, then cancel the TX timeout so
           * there will be no race condition between any subsequent timeout
           * expiration and the deferred interrupt processing.
           */

           wd_cancel(&priv->txtimeout);
        }

      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(ETHWORK, &priv->irqwork, tiva_interrupt_work, priv, 0);
    }

#ifdef CONFIG_TIVA_PHY_INTERRUPTS
  /* Check for pending PHY interrupts */

  if ((tiva_getreg(TIVA_EPHY_MISC) & EMAC_PHYMISC_INT) != 0)
    {
      /* Clear the pending PHY interrupt */

      tiva_putreg(EMAC_PHYMISC_INT, TIVA_EPHY_MISC);

      /* Dispatch to the registered handler */

      if (priv->handler != NULL)
        {
          priv->handler(irq, context, priv->arg);
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Function: tiva_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void tiva_txtimeout_work(void *arg)
{
  struct tiva_ethmac_s *priv = (struct tiva_ethmac_s *)arg;

  /* Reset the hardware.  Just take the interface down, then back up again. */

  net_lock();
  tiva_ifdown(&priv->dev);
  tiva_ifup(&priv->dev);

  /* Then poll the network for new XMIT data */

  tiva_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: tiva_txtimeout_expiry
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
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void tiva_txtimeout_expiry(wdparm_t arg)
{
  struct tiva_ethmac_s *priv = (struct tiva_ethmac_s *)arg;

  nerr("ERROR: Timeout!\n");

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   *
   * Interrupts will be re-enabled when tiva_ifup() is called.
   */

  up_disable_irq(TIVA_IRQ_ETHCON);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, tiva_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: tiva_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_ifup(struct net_driver_s *dev)
{
  struct tiva_ethmac_s *priv =
    (struct tiva_ethmac_s *)dev->d_private;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Configure the Ethernet interface for DMA operation. */

  ret = tive_emac_configure(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  up_enable_irq(TIVA_IRQ_ETHCON);

  tiva_checksetup();
  return OK;
}

/****************************************************************************
 * Function: tiva_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_ifdown(struct net_driver_s *dev)
{
  struct tiva_ethmac_s *priv =
    (struct tiva_ethmac_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(TIVA_IRQ_ETHCON);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the tiva_ifup() always
   * successfully brings the interface back up.
   */

  tiva_ethreset(priv);

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: tiva_txavail_work
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

static void tiva_txavail_work(void *arg)
{
  struct tiva_ethmac_s *priv = (struct tiva_ethmac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      tiva_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: tiva_txavail
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

static int tiva_txavail(struct net_driver_s *dev)
{
  struct tiva_ethmac_s *priv =
    (struct tiva_ethmac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, tiva_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: tiva_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used to check an ethernet frame
 *
 * Input Parameters:
 *   data   - the data to be checked
 *   length - length of the data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static uint32_t tiva_calcethcrc(const uint8_t *data, size_t length)
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
#endif /* CONFIG_NET_MCASTGROUP || CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: tiva_addmac
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
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int tiva_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast hash table */

  crc = tiva_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = TIVA_EMAC_HASHTBLH;
      hashindex -= 32;
    }
  else
    {
      registeraddress = TIVA_EMAC_HASHTBLL;
    }

  temp = tiva_getreg(registeraddress);
  temp |= 1 << hashindex;
  tiva_putreg(temp, registeraddress);

  temp = tiva_getreg(TIVA_EMAC_FRAMEFLTR);
  temp |= (EMAC_FRAMEFLTR_HMC | EMAC_FRAMEFLTR_HPF);
  tiva_putreg(temp, TIVA_EMAC_FRAMEFLTR);

  return OK;
}
#endif

/****************************************************************************
 * Function: tiva_rmmac
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
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int tiva_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Remove the MAC address to the hardware multicast hash table */

  crc = tiva_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = TIVA_EMAC_HASHTBLH;
      hashindex -= 32;
    }
  else
    {
      registeraddress = TIVA_EMAC_HASHTBLL;
    }

  temp = tiva_getreg(registeraddress);
  temp &= ~(1 << hashindex);
  tiva_putreg(temp, registeraddress);

  /* If there is no address registered any more, delete multicast filtering */

  if (tiva_getreg(TIVA_EMAC_HASHTBLH) == 0 &&
      tiva_getreg(TIVA_EMAC_HASHTBLL) == 0)
    {
      temp = tiva_getreg(TIVA_EMAC_FRAMEFLTR);
      temp &= ~(EMAC_FRAMEFLTR_HMC | EMAC_FRAMEFLTR_HPF);
      tiva_putreg(temp, TIVA_EMAC_FRAMEFLTR);
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: tiva_txdescinit
 *
 * Description:
 *   Initializes the DMA TX descriptors in chain mode.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_txdescinit(struct tiva_ethmac_s *priv)
{
  struct emac_txdesc_s *txdesc;
  int i;

  /* priv->txhead will point to the first, available TX descriptor in the
   * chain. Set the priv->txhead pointer to the first descriptor in the
   * table.
   */

  priv->txhead = priv->txtable;

  /* priv->txtail will point to the first segment of the oldest pending
   * "in-flight" TX transfer.  NULL means that there are no active TX
   * transfers.
   */

  priv->txtail   = NULL;
  priv->inflight = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < CONFIG_TIVA_EMAC_NTXDESC; i++)
    {
      txdesc = &priv->txtable[i];

      /* Set Second Address Chained bit */

      txdesc->tdes0 = EMAC_TDES0_TCH;

#ifdef CHECKSUM_BY_HARDWARE
      /* Enable the checksum insertion for the TX frames */

      txdesc->tdes0 |= EMAC_TDES0_CIC_ALL;
#endif

      /* Clear Buffer1 address pointer (buffers will be assigned as they
       * are used)
       */

      txdesc->tdes2 = 0;

      /* Initialize the next descriptor with
       * the Next Descriptor Polling Enable
       */

      if (i < (CONFIG_TIVA_EMAC_NTXDESC - 1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          txdesc->tdes3 = (uint32_t)&priv->txtable[i + 1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          txdesc->tdes3 = (uint32_t)priv->txtable;
        }
    }

  /* Set Transmit Descriptor List Address Register */

  tiva_putreg((uint32_t)priv->txtable, TIVA_EMAC_TXDLADDR);
}

/****************************************************************************
 * Function: tiva_rxdescinit
 *
 * Description:
 *   Initializes the DMA RX descriptors in chain mode.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_rxdescinit(struct tiva_ethmac_s *priv)
{
  struct emac_rxdesc_s *rxdesc;
  int i;

  /* priv->rxhead will point to the first,  RX descriptor in the chain.
   * This will be where we receive the first incomplete frame.
   */

  priv->rxhead = priv->rxtable;

  /* If we accumulate the frame in segments, priv->rxcurr points to the
   * RX descriptor of the first segment in the current TX frame.
   */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < CONFIG_TIVA_EMAC_NRXDESC; i++)
    {
      rxdesc = &priv->rxtable[i];

      /* Set Own bit of the RX descriptor rdes0 */

      rxdesc->rdes0 = EMAC_RDES0_OWN;

      /* Set Buffer1 size and Second Address Chained bit and enabled DMA
       * RX desc receive interrupt
       */

      rxdesc->rdes1 = EMAC_RDES1_RCH | (uint32_t)OPTIMAL_EMAC_BUFSIZE;

      /* Set Buffer1 address pointer */

      rxdesc->rdes2 = (uint32_t)&priv->rxbuffer[i * OPTIMAL_EMAC_BUFSIZE];

      /* Initialize the next descriptor with
       * the Next Descriptor Polling Enable
       */

      if (i < (CONFIG_TIVA_EMAC_NRXDESC - 1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          rxdesc->rdes3 = (uint32_t)&priv->rxtable[i + 1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          rxdesc->rdes3 = (uint32_t)priv->rxtable;
        }
    }

  /* Set Receive Descriptor List Address Register */

  tiva_putreg((uint32_t)priv->rxtable, TIVA_EMAC_RXDLADDR);
}

/****************************************************************************
 * Function: tiva_ioctl
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
 * Returned Value: Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int tiva_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
#ifdef CONFIG_TIVA_PHY_INTERRUPTS
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          struct mii_ioctl_notify_s *req =
            (struct mii_ioctl_notify_s *)((uintptr_t)arg);

          ret = phy_notify_subscribe(dev->d_ifname, req->pid, &req->event);
          if (ret == OK)
            {
              /* Enable PHY link up/down interrupts */

              tiva_phy_intenable(true);
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = CONFIG_TIVA_PHYADDR;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = tiva_phyread(req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
       {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = tiva_phywrite(req->phy_id, req->reg_num, req->val_in);
        }
        break;
#endif /* CONFIG_NETDEV_PHY_IOCTL */

      default:
        ret = -ENOTTY;
          break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: tiva_phy_intenable
 *
 * Description:
 *  Enable link up/down PHY interrupts.  The interrupt protocol is like this:
 *
 *  - Interrupt status is cleared when the interrupt is enabled.
 *  - Interrupt occurs.  Interrupt is disabled (at the processor level) when
 *    is received.
 *  - Interrupt status is cleared when the interrupt is re-enabled.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno (-ETIMEDOUT) on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_PHY_INTERRUPTS
static void tiva_phy_intenable(bool enable)
{
#ifdef CONFIG_TIVA_PHY_INTERNAL
  uint16_t phyval;
  int ret;

  /* Disable further PHY interrupts until we complete this setup */

  tiva_putreg(0, TIVA_EPHY_IM);

  /* Enable/disable event based PHY interrupts */

  /* REVISIT:  There is an issue here:  The PHY interrupt handler is called
   * from the interrupt level and it, in turn, will call this function to
   * disabled further interrupts.  Subsequent link status processing will
   * also call tiva_phyread() to access PHY registers and will, eventually,
   * call this function again to re-enable the PHY interrupt.  The control
   * between interrupt level access to the PHY and non-interrupt level
   * access to the PHY is not well enforced but is probably okay just due
   * to the sequencing of things.
   */

  if (enable)
    {
      /* Configure interrupts on link status change events */

      ret = tiva_phywrite(CONFIG_TIVA_PHYADDR, TIVA_EPHY_MISR1,
                          EPHY_MISR1_LINKSTATEN);
      if (ret == OK)
        {
          /* Enable PHY event based interrupts */

          ret = tiva_phyread(CONFIG_TIVA_PHYADDR, TIVA_EPHY_SCR, &phyval);
          if (ret == OK)
            {
              phyval |= EPHY_SCR_INTEN;
              ret = tiva_phywrite(CONFIG_TIVA_PHYADDR,
                                  TIVA_EPHY_SCR, phyval);
              if (ret == OK)
                {
                  /* Enable PHY interrupts */

                  tiva_putreg(EMAC_PHYIM_INT, TIVA_EPHY_IM);
                }
            }
        }
    }
  else
    {
      /* Read the MISR1 register in order to clear any pending link status
       * interrupts.
       */

      ret = tiva_phyread(CONFIG_TIVA_PHYADDR, TIVA_EPHY_MISR1, &phyval);
      if (ret == OK)
        {
          /* Disable PHY event based interrupts */

          ret = tiva_phyread(CONFIG_TIVA_PHYADDR, TIVA_EPHY_SCR, &phyval);
          if (ret == OK)
            {
              phyval |= EPHY_SCR_INTEN;
              tiva_phywrite(CONFIG_TIVA_PHYADDR, TIVA_EPHY_SCR, phyval);
            }
        }
    }

#else
  /* Interrupt configuration logic for external PHYs depends on the
   * particular PHY part connected.
   */

#warning Missing logic
  return -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Function: tiva_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Input Parameters:
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

static int tiva_phyread(uint16_t phydevaddr,
                        uint16_t phyregaddr, uint16_t *value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MIIADDR register,
   * preserving CSR Clock Range CR[2:0] bits
   */

  regval  = tiva_getreg(TIVA_EMAC_MIIADDR);
  regval &= EMAC_MIIADDR_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the buy bit.
   * the EMAC_MIIADDR_MIIW is clear, indicating a read operation.
   */

  regval |= (phydevaddr << EMAC_MIIADDR_PLA_SHIFT) & EMAC_MIIADDR_PLA_MASK;
  regval |= (phyregaddr << EMAC_MIIADDR_MII_SHIFT) & EMAC_MIIADDR_MII_MASK;
  regval |= EMAC_MIIADDR_MIIB;

  tiva_putreg(regval, TIVA_EMAC_MIIADDR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_READ_TIMEOUT; timeout++)
    {
      if ((tiva_getreg(TIVA_EMAC_MIIADDR) & EMAC_MIIADDR_MIIB) == 0)
        {
          *value = (uint16_t)tiva_getreg(TIVA_EMAC_MIIDATA);
          return OK;
        }
    }

  nerr("ERROR: MII transfer timed out: phydevaddr: %04x phyregaddr: %04x\n",
       phydevaddr, phyregaddr);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: tiva_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Input Parameters:
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

static int tiva_phywrite(uint16_t phydevaddr,
                         uint16_t phyregaddr, uint16_t value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MIIADDR register,
   * preserving CSR Clock Range CR[2:0] bits
   */

  regval  = tiva_getreg(TIVA_EMAC_MIIADDR);
  regval &= EMAC_MIIADDR_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the EMAC_MIIADDR_MIIW is set, indicating a write operation.
   */

  regval |= (phydevaddr << EMAC_MIIADDR_PLA_SHIFT) & EMAC_MIIADDR_PLA_MASK;
  regval |= (phyregaddr << EMAC_MIIADDR_MII_SHIFT) & EMAC_MIIADDR_MII_MASK;
  regval |= (EMAC_MIIADDR_MIIB | EMAC_MIIADDR_MIIW);

  /* Write the value into the MACIIDR register before setting the new MIIADDR
   * register value.
   */

  tiva_putreg(value, TIVA_EMAC_MIIDATA);
  tiva_putreg(regval, TIVA_EMAC_MIIADDR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_WRITE_TIMEOUT; timeout++)
    {
      if ((tiva_getreg(TIVA_EMAC_MIIADDR) & EMAC_MIIADDR_MIIB) == 0)
        {
          return OK;
        }
    }

  nerr("ERROR: MII transfer timed out: "
       "phydevaddr: %04x phyregaddr: %04x value: %04x\n",
       phydevaddr, phyregaddr, value);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: tiva_phyinit
 *
 * Description:
 *  Configure the PHY and determine the link speed/duplex.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_phyinit(struct tiva_ethmac_s *priv)
{
#ifdef CONFIG_TIVA_AUTONEG
  volatile uint32_t timeout;
#endif
  uint32_t regval;
  uint16_t phyval;
  int ret;

  /* Assume 10MBps and half duplex */

  priv->mbps100 = 0;
  priv->fduplex = 0;

  /* Setup up PHY clocking by setting the CR field in the MIIADDR register */

  regval  = tiva_getreg(TIVA_EMAC_MIIADDR);
  regval &= ~EMAC_MIIADDR_CR_MASK;
  regval |= EMAC_MIIADDR_CR;
  tiva_putreg(regval, TIVA_EMAC_MIIADDR);

  /* Put the PHY in reset mode */

  ret = tiva_phywrite(CONFIG_TIVA_PHYADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: Failed to reset the PHY: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_RESET_DELAY);

  /* Perform auto-negotiation if so configured */

#ifdef CONFIG_TIVA_AUTONEG
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = tiva_phyread(CONFIG_TIVA_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_LINKSTATUS) != 0)
        {
          break;
        }
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for link status: %04x\n", phyval);
      return -ETIMEDOUT;
    }

  /* Enable auto-negotiation */

  ret = tiva_phywrite(CONFIG_TIVA_PHYADDR, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = tiva_phyread(CONFIG_TIVA_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_ANEGCOMPLETE) != 0)
        {
          break;
        }
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for auto-negotiation\n");
      return -ETIMEDOUT;
    }

  /* Read the result of the auto-negotiation from the PHY-specific register */

  ret = tiva_phyread(CONFIG_TIVA_PHYADDR, CONFIG_TIVA_PHYSR, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  ninfo("PHYSR[%d]: %04x\n", CONFIG_TIVA_PHYSR, phyval);

  /* Different PHYs present speed and mode information in different ways.
   * IF This CONFIG_TIVA_PHYSR_ALTCONFIG is selected, this indicates that
   * the PHY represents speed and mode information are combined, for
   * example, with separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_TIVA_PHYSR_ALTCONFIG
  switch (phyval & CONFIG_TIVA_PHYSR_ALTMODE)
    {
      default:
      case CONFIG_TIVA_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_TIVA_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_TIVA_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_TIVA_PHYSR_100FD:
        priv->fduplex = 1;
        priv->mbps100 = 1;
        break;
    }

  /* Different PHYs present speed and mode information in different ways.
   * Some will present separate information for speed and mode (this is the
   * default). Those PHYs, for example, may provide a 10/100 Mbps indication
   * and a separate full/half duplex indication.
   */

#else
  if ((phyval & CONFIG_TIVA_PHYSR_MODE) == CONFIG_TIVA_PHYSR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_TIVA_PHYSR_SPEED) == CONFIG_TIVA_PHYSR_100MBPS)
    {
      priv->mbps100 = 1;
    }
#endif

#else /* Auto-negotiation not selected */

  phyval = 0;
#ifdef CONFIG_TIVA_ETHFD
  phyval |= MII_MCR_FULLDPLX;
#endif
#ifdef CONFIG_TIVA_ETH100MBPS
  phyval |= MII_MCR_SPEED100;
#endif

  ret = tiva_phywrite(CONFIG_TIVA_PHYADDR, MII_MCR, phyval);
  if (ret < 0)
    {
     nerr("ERROR: Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes */

#ifdef CONFIG_TIVA_ETHFD
  priv->fduplex = 1;
#endif
#ifdef CONFIG_TIVA_ETH100MBPS
  priv->mbps100 = 1;
#endif
#endif

  ninfo("Duplex: %s Speed: %d MBps\n",
        priv->fduplex ? "FULL" : "HALF",
        priv->mbps100 ? 100 : 10);

  return OK;
}

/****************************************************************************
 * Function: tiva_phy_configure
 *
 * Description:
 *  Configure to support the selected PHY.  Called after each reset since
 *  many properties of the PHY configuration are lost at each reset.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_phy_configure(struct tiva_ethmac_s *priv)
{
  uint32_t regval;

  /* Set up the PHY configuration */

#if defined(CONFIG_TIVA_PHY_RMII)
  regval = EMAC_PC_PINTFS_RMII | EMAC_PC_PHYEXT;
#elif defined(CONFIG_TIVA_PHY_MII)
  regval = EMAC_PC_PINTFS_MII | EMAC_PC_PHYEXT;
#else /* defined(CONFIG_TIVA_PHY_INTERNAL) */
  regval = EMAC_PC_MDIXEN | EMAC_PC_ANMODE_100FD | EMAC_PC_ANEN |
           EMAC_PC_PINTFS_MII;
#endif
  tiva_putreg(regval, TIVA_EMAC_PC);

#ifdef CONFIG_TIVA_PHY_INTERNAL
  /* If we are using the internal PHY, reset it to ensure that new
   * configuration is latched.
   */

  regval  = tiva_getreg(TIVA_SYSCON_SREPHY);
  regval |= SYSCON_SREPHY_R0;
  tiva_putreg(regval, TIVA_SYSCON_SREPHY);

  regval &= ~SYSCON_SREPHY_R0;
  tiva_putreg(regval, TIVA_SYSCON_SREPHY);

  /* Wait for the reset to complete */

  while (!tiva_ephy_periphrdy());
  up_udelay(250);
#endif

  /* Disable all MMC interrupts as these are enabled by default at reset */

  tiva_putreg(0xffffffff, TIVA_EMAC_MMCRXIM);
  tiva_putreg(0xffffffff, TIVA_EMAC_MMCTXIM);

  /* If using an external RMII PHY, we must enable the external clock */

  regval = tiva_getreg(TIVA_EMAC_CC);

#if defined(CONFIG_TIVA_PHY_RMII)
  /* Enable the external clock source input to the RMII interface signal
   * EN0RREF_CLK by setting both the CLKEN bit in the Ethernet Clock
   * Configuration (EMACCC) register. The external clock source must be
   * 50 MHz with a frequency tolerance of 50 PPM.
   */

  regval = tiva_getreg(TIVA_EMAC_CC);
#else
  /* Disable the external clock */

  regval &= ~EMAC_CC_CLKEN;
#endif

  tiva_putreg(regval, TIVA_EMAC_CC);
}

/****************************************************************************
 * Function: tiva_phy_initialize
 *
 * Description:
 *  Perform one-time PHY initialization
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void tiva_phy_initialize(struct tiva_ethmac_s *priv)
{
  /* Enable the clock to the PHY module */

  ninfo("Enable EPHY clocking\n");
  tiva_ephy_enableclk();

  /* What until the PREPHY register indicates that the PHY is ready before
   * continuing.
   */

  while (!tiva_ephy_periphrdy());
  up_udelay(250);

  /* Enable power to the Ethernet PHY */

  ninfo("Enable EPHY power\n");
  tiva_ephy_enablepwr();

  /* What until the PREPHY register indicates that the PHY registers are
   * ready to be accessed.
   */

  while (!tiva_ephy_periphrdy());
  up_udelay(250);

  ninfo("RCGCEPHY: %08" PRIx32 " PCEPHY: %08" PRIx32
        " PREPHY: %08" PRIx32 "\n",
        getreg32(TIVA_SYSCON_RCGCEPHY),
        getreg32(TIVA_SYSCON_PCEPHY),
        getreg32(TIVA_SYSCON_PREPHY));
  ninfo("Configure PHY GPIOs\n");

#ifdef CONFIG_TIVA_PHY_INTERNAL
  /* Integrated PHY:
   *
   *   "The Ethernet Controller Module and Integrated PHY receive two clock
   *    inputs:
   *    - A gated system clock acts as the clock source to the Control and
   *      Status registers (CSR) of the Ethernet MAC. The SYSCLK frequency
   *      for Run, Sleep and Deep Sleep mode is programmed in the System
   *      Control module. ...
   *    - The PHY receives the main oscillator (MOSC) which must be 25 MHz
   *      ± 50 ppm for proper operation. The MOSC source can be a single-
   *      ended source or a crystal."
   *
   *   These are currently set up in tiva_clock_reconfigure() before this
   *   function runs.
   *
   * MII/RMII Clocking:
   *
   *   External PHY support is not yet implemented.
   */

  /* PHY interface pins:
   *
   * EN0TXOP - Fixed pin assignment
   * EN0TXON - Fixed pin assignment
   * EN0RXIP - Fixed pin assignment
   * EN0RXIN - Fixed pin assignment
   * RBIAS   - Fixed pin assignment
   * EN0LED0 - Configured GPIO output
   * EN0LED1 - Configured GPIO output
   * EN0LED2 - Configured GPIO output
   */

  tiva_configgpio(GPIO_EN0_LED0);
  tiva_configgpio(GPIO_EN0_LED1);
  tiva_configgpio(GPIO_EN0_LED2);

#else /* if defined(CONFIG_TIVA_PHY_MII) || defined(CONFIG_TIVA_PHY_RMII) */
  /* External PHY interrupt pin */

  tiva_configgpio(GPIO_EN0_INTRN);

  /* Configure GPIO pins to support MII or RMII */

  /* MDC and MDIO are common to both modes */

  tiva_configgpio(GPIO_EN0_MDC);
  tiva_configgpio(GPIO_EN0_MDIO);

#if defined(CONFIG_TIVA_PHY_MII)
  /* Set up the MII interface */

  /* "Four clock inputs are driven into the Ethernet MAC when the MII
   *  configuration is enabled. The clocks are described as follows:
   *
   *  - Gated system clock (SYSCLK): The SYSCLK signal acts as the clock
   *    source to the Control and Status registers (CSR) of the Ethernet
   *    MAC. The SYSCLK frequency for Run, Sleep and Deep Sleep mode is
   *    programmed in the System Control module. ...
   *  - MOSC: A gated version of the MOSC clock is provided as the Precision
   *    Time Protocol (PTP) reference clock (PTPREF_CLK). The MOSC clock
   *    source can be a single-ended source on the OSC0 pin or a crystal
   *    on the OSC0 and OSC1 pins. When advanced timestamping is used and
   *    the Precision Timer Protocol (PTP) module has been enabled by setting
   *    the PTPCEN bit in the EMACCC register, the MOSC drives PTPREF_CLK.
   *    PTPREF_CLK has a minimum frequency requirement of 5 MHz and a
   *    maximum frequency of 25 MHz. ...
   *  - EN0RXCK: This clock signal is driven by the external PHY oscillator
   *    and is either 2.5 or 25 MHz depending on whether the device is
   *    operating at 10 Mbps or 100 Mbps.
   *  - EN0TXCK This clock signal is driven by the external PHY oscillator
   *    and is either 2.5 or 25 MHz depending on whether the device is
   *    operating at 10 Mbps or 100 Mbps."
   */

  /* MII interface pins (17):
   *
   * MII_TX_CLK, MII_TXD[3:0], MII_TX_EN, MII_RX_CLK, MII_RXD[3:0],
   * MII_RX_ER, MII_RX_DV, MII_CRS, MII_COL, MDC, MDIO
   */

  tiva_configgpio(GPIO_EN0_MII_COL);
  tiva_configgpio(GPIO_EN0_MII_CRS);
  tiva_configgpio(GPIO_EN0_MII_RXD0);
  tiva_configgpio(GPIO_EN0_MII_RXD1);
  tiva_configgpio(GPIO_EN0_MII_RXD2);
  tiva_configgpio(GPIO_EN0_MII_RXD3);
  tiva_configgpio(GPIO_EN0_MII_RX_CLK);
  tiva_configgpio(GPIO_EN0_MII_RX_DV);
  tiva_configgpio(GPIO_EN0_MII_RX_ER);
  tiva_configgpio(GPIO_EN0_MII_TXD0);
  tiva_configgpio(GPIO_EN0_MII_TXD1);
  tiva_configgpio(GPIO_EN0_MII_TXD2);
  tiva_configgpio(GPIO_EN0_MII_TXD3);
  tiva_configgpio(GPIO_EN0_MII_TX_CLK);
  tiva_configgpio(GPIO_EN0_MII_TX_EN);

#elif defined(CONFIG_TIVA_PHY_RMII)
  /* Set up the RMII interface. */

  /* "There are three clock sources that interface to the Ethernet MAC in
   *  an RMII configuration:
   *
   *  - Gated system clock (SYSCLK): The SYSCLK signal acts as the clock
   *    source to the Control and Status registers (CSR) of the Ethernet MAC.
   *    The SYSCLK frequency for Run, Sleep and Deep Sleep mode is programmed
   *    in the System Control module. ...
   *  - MOSC: A gated version of the MOSC clock is provided as the Precision
   *    Time Protocol (PTP) reference clock (PTPREF_CLK). The MOSC clock
   *    source can be a single-ended source on the OSC0 pin or a crystal on
   *    the OSC0 and OSC1 pins. When advanced timestamping is used and
   *    the PTP module has been enabled by setting the PTPCEN bit in the
   *    EMACCC register, the MOSC drives PTPREF_CLK. PTPREF_CLK has a minimum
   *    frequency requirement of 5 MHz and a maximum frequency of 25 MHz. ...
   *  - EN0REF_CLK: When using RMII, a 50 MHz external reference clock must
   *    drive the EN0REF_CLK input signal and the external PHY. Depending on
   *    the configuration of the FES bit in the Ethernet MAC Configuration
   *    (EMACCFG) register, the reference clock input (EN0REF_CLK) is divided
   *    by 20 for 10 Mbps or 2 for 100 Mbps operation and used as the clock
   *    for receive and transmit data."
   */

  /* RMII interface pins (7):
   *
   * RMII_TXD[1:0], RMII_TX_EN, RMII_RXD[1:0], RMII_CRS_DV, MDC, MDIO,
   * RMII_REF_CLK
   */

  tiva_configgpio(GPIO_EN0_RMII_CRS_DV);
  tiva_configgpio(GPIO_EN0_RMII_REF_CLK);
  tiva_configgpio(GPIO_EN0_RMII_RXD0);
  tiva_configgpio(GPIO_EN0_RMII_RXD1);
  tiva_configgpio(GPIO_EN0_RMII_TXD0);
  tiva_configgpio(GPIO_EN0_RMII_TXD1);
  tiva_configgpio(GPIO_EN0_RMII_TX_EN);

#endif

  /* Enable pulse-per-second (PPS) output signal */

  tiva_configgpio(GPIO_EN0_PPS);
#endif
}

/****************************************************************************
 * Function: tiva_ethreset
 *
 * Description:
 *  Reset the Ethernet block.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_ethreset(struct tiva_ethmac_s *priv)
{
  uint32_t regval;

#if 0 /* REVISIT: This causes the DMABUSMOD reset to hang. */
  /* Reset the Ethernet MAC */

  regval  = tiva_getreg(TIVA_SYSCON_SREMAC);
  regval |= SYSCON_SREMAC_R0;
  tiva_putreg(regval, TIVA_SYSCON_SREMAC);

  regval &= ~SYSCON_SREMAC_R0;
  tiva_putreg(regval, TIVA_SYSCON_SREMAC);

  /* Wait for the reset to complete */

  while (!tiva_emac_periphrdy());
  up_udelay(250);
#endif

  /* Perform a software reset by setting the SWR bit in the DMABUSMOD
   * register. This Resets all MAC subsystem internal registers and logic.
   * After this reset all the registers holds their reset values.
   */

  regval  = tiva_getreg(TIVA_EMAC_DMABUSMOD);
  regval |= EMAC_DMABUSMOD_SWR;
  tiva_putreg(regval, TIVA_EMAC_DMABUSMOD);

  /* Wait for software reset to complete. The SWR bit is cleared
   * automatically after the reset operation has completed in all
   * core clock domains.
   */

  while ((tiva_getreg(TIVA_EMAC_DMABUSMOD) & EMAC_DMABUSMOD_SWR) != 0);
  up_udelay(250);

  /* Reconfigure the PHY.  Some PHY configurations will be lost as a
   * consequence of the EMAC reset
   */

  tiva_phy_configure(priv);
}

/****************************************************************************
 * Function: tiva_macconfig
 *
 * Description:
 *  Configure the Ethernet MAC for DMA operation.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_macconfig(struct tiva_ethmac_s *priv)
{
  uint32_t regval;

  /* Set up the MACCR register */

  regval  = tiva_getreg(TIVA_EMAC_CFG);
  regval &= ~MACCR_CLEAR_BITS;
  regval |= MACCR_SET_BITS;

  if (priv->fduplex)
    {
      /* Set the DM bit for full duplex support */

      regval |= EMAC_CFG_DUPM;
    }

  if (priv->mbps100)
    {
      /* Set the FES bit for 100Mbps fast Ethernet support */

      regval |= EMAC_CFG_FES;
    }

  tiva_putreg(regval, TIVA_EMAC_CFG);

  /* Set up the FRAMEFLTR register */

  regval  = tiva_getreg(TIVA_EMAC_FRAMEFLTR);
  regval &= ~FRAMEFLTR_CLEAR_BITS;
  regval |= FRAMEFLTR_SET_BITS;
  tiva_putreg(regval, TIVA_EMAC_FRAMEFLTR);

  /* Set up the HASHTBLH and HASHTBLL registers */

  tiva_putreg(0, TIVA_EMAC_HASHTBLH);
  tiva_putreg(0, TIVA_EMAC_HASHTBLL);

  /* Setup up the FLOWCTL register */

  regval  = tiva_getreg(TIVA_EMAC_FLOWCTL);
  regval &= ~FLOWCTL_CLEAR_MASK;
  regval |= FLOWCTL_SET_MASK;
  tiva_putreg(regval, TIVA_EMAC_FLOWCTL);

  /* Setup up the VLANTG register */

  tiva_putreg(0, TIVA_EMAC_VLANTG);

  /* DMA Configuration */

  /* Set up the DMAOPMODE register */

  regval  = tiva_getreg(TIVA_EMAC_DMAOPMODE);
  regval &= ~DMAOPMODE_CLEAR_MASK;
  regval |= DMAOPMODE_SET_MASK;
  tiva_putreg(regval, TIVA_EMAC_DMAOPMODE);

  /* Set up the DMABUSMOD register */

  regval  = tiva_getreg(TIVA_EMAC_DMABUSMOD);
  regval &= ~DMABUSMOD_CLEAR_MASK;
  regval |= DMABUSMOD_SET_MASK;
  tiva_putreg(regval, TIVA_EMAC_DMABUSMOD);

  return OK;
}

/****************************************************************************
 * Function: tiva_macaddress
 *
 * Description:
 *   Configure the selected MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_macaddress(struct tiva_ethmac_s *priv)
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
  tiva_putreg(regval, TIVA_EMAC_ADDR0H);

  /* Set the MAC address low register */

  regval = ((uint32_t)dev->d_mac.ether.ether_addr_octet[3] << 24) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[2] << 16) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[1] <<  8) |
            (uint32_t)dev->d_mac.ether.ether_addr_octet[0];
  tiva_putreg(regval, TIVA_EMAC_ADDR0L);
}

/****************************************************************************
 * Function: tiva_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void tiva_ipv6multicast(struct tiva_ethmac_s *priv)
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

  tiva_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  tiva_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  tiva_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: tiva_macenable
 *
 * Description:
 *  Enable normal MAC operation.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_macenable(struct tiva_ethmac_s *priv)
{
  uint32_t regval;

  /* Set the MAC address */

  tiva_macaddress(priv);

#ifdef CONFIG_NET_ICMPv6
  /* Set up the IPv6 multicast address */

  tiva_ipv6multicast(priv);
#endif

  /* Enable transmit state machine of the MAC for transmission on the MII */

  regval  = tiva_getreg(TIVA_EMAC_CFG);
  regval |= EMAC_CFG_TE;
  tiva_putreg(regval, TIVA_EMAC_CFG);

  /* Flush Transmit FIFO */

  regval  = tiva_getreg(TIVA_EMAC_DMAOPMODE);
  regval |= EMAC_DMAOPMODE_FTF;
  tiva_putreg(regval, TIVA_EMAC_DMAOPMODE);

  /* Enable receive state machine of the MAC for reception from the MII */

  /* Enables or disables the MAC reception. */

  regval  = tiva_getreg(TIVA_EMAC_CFG);
  regval |= EMAC_CFG_RE;
  tiva_putreg(regval, TIVA_EMAC_CFG);

  /* Start DMA transmission */

  regval  = tiva_getreg(TIVA_EMAC_DMAOPMODE);
  regval |= EMAC_DMAOPMODE_ST;
  tiva_putreg(regval, TIVA_EMAC_DMAOPMODE);

  /* Start DMA reception */

  regval  = tiva_getreg(TIVA_EMAC_DMAOPMODE);
  regval |= EMAC_DMAOPMODE_SR;
  tiva_putreg(regval, TIVA_EMAC_DMAOPMODE);

  /* Enable Ethernet DMA interrupts. */

  tiva_putreg(EMAC_IM_ALLINTS, TIVA_EMAC_IM);

  /* Ethernet DMA supports two classes of interrupts: Normal interrupt
   * summary (NIS) and Abnormal interrupt summary (AIS) with a variety
   * individual normal and abnormal interrupting events.  Here only
   * the normal receive event is enabled (unless DEBUG is enabled).  Transmit
   * events will only be enabled when a transmit interrupt is expected.
   */

  tiva_putreg((EMAC_DMAINT_RECV_ENABLE | EMAC_DMAINT_ERROR_ENABLE),
              TIVA_EMAC_DMAIM);
  return OK;
}

/****************************************************************************
 * Function: tive_emac_configure
 *
 * Description:
 *  Configure the Ethernet interface for DMA operation.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tive_emac_configure(struct tiva_ethmac_s *priv)
{
  int ret;

  /* NOTE: The Ethernet clocks were initialized earlier in the start-up
   * sequence.
   */

  /* Reset the Ethernet block */

  ninfo("Reset the Ethernet block\n");
  tiva_ethreset(priv);

  /* Initialize the PHY */

  ninfo("Initialize the PHY\n");
  ret = tiva_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the MAC and DMA */

  ninfo("Initialize the MAC and DMA\n");
  ret = tiva_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the free buffer list */

  tiva_initbuffer(priv);

  /* Initialize TX Descriptors list: Chain Mode */

  tiva_txdescinit(priv);

  /* Initialize RX Descriptors list: Chain Mode  */

  tiva_rxdescinit(priv);

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");
  return tiva_macenable(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tiva_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the Tiva chip
 *   supports multiple Ethernet controllers, then board specific logic
 *   must implement arm_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS == 1 || defined(CONFIG_NETDEV_LATEINIT)
static inline
#endif
int tiva_ethinitialize(int intf)
{
  struct tiva_ethmac_s *priv;
  uint32_t regval;

  ninfo("intf: %d\n", intf);

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < TIVA_NETHCONTROLLERS);
  priv = &g_tiva_ethmac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct tiva_ethmac_s));
  priv->dev.d_ifup    = tiva_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = tiva_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = tiva_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = tiva_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = tiva_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = tiva_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = g_tiva_ethmac; /* Used to recover private state from dev */

#ifdef CONFIG_TIVA_BOARDMAC
  /* If the board can provide us with a MAC address, get the address
   * from the board now.  The MAC will not be applied until tiva_ifup()
   * is called (and the MAC can be overwritten with a netdev ioctl call).
   */

  tiva_ethernetmac(&priv->dev.d_mac.ether);
#endif

  /* Enable power and clocking to the Ethernet MAC
   *
   * - Enable Power: Applies power (only) to the EMAC peripheral. This is not
   *   an essential step since enabling clocking will also apply power.  The
   *   only significance is that the EMAC state will be retained if the EMAC
   *   clocking is subsequently disabled.
   * - Enable Clocking:  Applies both power and clocking to the EMAC
   *   peripheral, bringing it a fully functional state.
   */

  ninfo("Enable EMAC clocking\n");
  tiva_emac_enablepwr();   /* Ethernet MAC Power Control */
  tiva_emac_enableclk();   /* Ethernet MAC Run Mode Clock Gating Control */

  /* What until the PREMAC register indicates that the EMAC registers are
   * ready to be accessed.
   */

  while (!tiva_emac_periphrdy());
  up_udelay(250);

  /* Show all EMAC clocks */

  ninfo("RCGCEMAC: %08" PRIx32 " PCEMAC: %08" PRIx32
        " PREMAC: %08" PRIx32 " MOSCCTL: %08" PRIx32 "\n",
        getreg32(TIVA_SYSCON_RCGCEMAC),
        getreg32(TIVA_SYSCON_PCEMAC),
        getreg32(TIVA_SYSCON_PREMAC),
        getreg32(TIVA_SYSCON_MOSCCTL));

  /* Configure clocking and GPIOs to support the internal/eternal PHY */

  tiva_phy_initialize(priv);

  /* Attach the IRQ to the driver */

  if (irq_attach(TIVA_IRQ_ETHCON, tiva_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Wait for EMAC to come out of reset. The SWR bit is cleared automatically
   * after the reset operation has completed in all core clock domains.
   */

  while ((tiva_getreg(TIVA_EMAC_DMABUSMOD) & EMAC_DMABUSMOD_SWR) != 0);
  up_udelay(250);

#if 0 /* REVISIT: Part of work around to avoid DMABUSMOD SWR hangs */
  /* Put the interface in the down state. */

  tiva_ifdown(&priv->dev);

#else
  /* Reset the Ethernet MAC */

  regval  = tiva_getreg(TIVA_SYSCON_SREMAC);
  regval |= SYSCON_SREMAC_R0;
  tiva_putreg(regval, TIVA_SYSCON_SREMAC);

  regval &= ~SYSCON_SREMAC_R0;
  tiva_putreg(regval, TIVA_SYSCON_SREMAC);

  /* Wait for the reset to complete */

  while (!tiva_emac_periphrdy());
  up_udelay(250);
#endif

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ninfo("Registering Ethernet device\n");
  return netdev_register(&priv->dev, NET_LL_ETHERNET);
}

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the low
 *   -level initialization logic in arm_initialize.c. If TIVA_NETHCONTROLLERS
 *   greater than one, then board specific logic will have to supply a
 *   version of arm_netinitialize() that calls tiva_ethinitialize() with
 *   the appropriate interface number.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  tiva_ethinitialize(0);
}
#endif

/****************************************************************************
 * Name: arch_phy_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a PHY interrupt occurs.  This function both attaches
 *   the interrupt handler and enables the interrupt if 'handler' is non-
 *   NULL.  If handler is NULL, then the interrupt is detached and disabled
 *   instead.
 *
 *   The PHY interrupt is always disabled upon return.  The caller must
 *   call back through the enable function point to control the state of
 *   the interrupt.
 *
 *   This interrupt may or may not be available on a given platform depending
 *   on how the network hardware architecture is implemented.  In a typical
 *   case, the PHY interrupt is provided to board-level logic as a GPIO
 *   interrupt (in which case this is a board-specific interface and really
 *   should be called board_phy_irq()); In other cases, the PHY interrupt
 *   may be cause by the chip's MAC logic (in which case arch_phy_irq()) is
 *   an appropriate name.  Other other boards, there may be no PHY interrupts
 *   available at all.  If client attachable PHY interrupts are available
 *   from the board or from the chip, then CONFIG_ARCH_PHY_INTERRUPT should
 *   be defined to indicate that fact.
 *
 *   Typical usage:
 *   a. OS service logic (not application logic*) attaches to the PHY
 *      PHY interrupt and enables the PHY interrupt.
 *   b. When the PHY interrupt occurs:  (1) the interrupt should be
 *      disabled and () work should be scheduled on the worker thread (or
 *      perhaps a dedicated application thread).
 *   c. That worker thread should use the SIOCGMIIPHY, SIOCGMIIREG,
 *      and SIOCSMIIREG ioctl calls** to communicate with the PHY,
 *      determine what network event took place (Link Up/Down?), and
 *      take the appropriate actions.
 *   d. It should then interact the PHY to clear any pending
 *      interrupts, then re-enable the PHY interrupt.
 *
 *    * This is an OS internal interface and should not be used from
 *      application space.  Rather applications should use the SIOCMIISIG
 *      ioctl to receive a signal when a PHY event occurs.
 *   ** This interrupt is really of no use if the Ethernet MAC driver
 *      does not support these ioctl calls.
 *
 * Input Parameters:
 *   intf    - Identifies the network interface.  For example "eth0".  Only
 *             useful on platforms that support multiple Ethernet interfaces
 *             and, hence, multiple PHYs and PHY interrupts.
 *   handler - The client interrupt handler to be invoked when the PHY
 *             asserts an interrupt.  Must reside in OS space, but can
 *             signal tasks in user space.  A value of NULL can be passed
 *             in order to detach and disable the PHY interrupt.
 *   arg     - The argument that will accompany the interrupt
 *   enable  - A function pointer that be unused to enable or disable the
 *             PHY interrupt.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_PHY_INTERRUPTS
int arch_phy_irq(const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable)
{
  struct tiva_ethmac_s *priv;
  irqstate_t flags;

  DEBUGASSERT(intf);
  ninfo("%s: handler=%p\n", intf, handler);

  /* Get the interface structure associated with this interface. */

#if TIVA_NETHCONTROLLERS > 1
  /* REVISIT: Additional logic needed if there are multiple EMACs */

#  warning Missing logic
#endif
  priv = g_tiva_ethmac;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Save the new interrupt handler information */

  priv->handler = handler;
  priv->arg     = arg;

  /* Return with the interrupt disabled in any case */

  tiva_phy_intenable(false);

  /* Return the enabling function pointer */

  if (enable)
    {
      *enable = handler ? tiva_phy_intenable : NULL;
    }

  leave_critical_section(flags);
  return OK;
}
#endif /* CONFIG_TIVA_PHY_INTERRUPTS */

#endif /* TIVA_NETHCONTROLLERS > 0 */
#endif /* CONFIG_NET && CONFIG_TIVA_ETHERNET */
