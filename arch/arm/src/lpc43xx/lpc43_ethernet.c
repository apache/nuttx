/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_ethernet.c
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
#if defined(CONFIG_NET) && defined(CONFIG_LPC43_ETHERNET)

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
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include "arm_internal.h"
#include "chip.h"
#include "lpc43_pinconfig.h"
#include "lpc43_ethernet.h"
#include "hardware/lpc43_creg.h"
#include "hardware/lpc43_cgu.h"
#include "hardware/lpc43_ccu.h"
#include "lpc43_rgu.h"
#include "lpc43_gpio.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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

#ifndef CONFIG_LPC43_PHYADDR
#  error "CONFIG_LPC43_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_LPC43_MII) && !defined(CONFIG_LPC43_RMII)
#  warning "Neither CONFIG_LPC43_MII nor CONFIG_LPC43_RMII defined"
#endif

#if defined(CONFIG_LPC43_MII) && defined(CONFIG_LPC43_RMII)
#  error "Both CONFIG_LPC43_MII and CONFIG_LPC43_RMII defined"
#endif

#ifdef CONFIG_LPC43_AUTONEG
#  ifndef CONFIG_LPC43_PHYSR
#    error "CONFIG_LPC43_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_LPC43_PHYSR_ALTCONFIG
#    ifndef CONFIG_LPC43_PHYSR_ALTMODE
#      error "CONFIG_LPC43_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_LPC43_PHYSR_10HD
#      error "CONFIG_LPC43_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_LPC43_PHYSR_100HD
#      error "CONFIG_LPC43_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_LPC43_PHYSR_10FD
#      error "CONFIG_LPC43_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_LPC43_PHYSR_100FD
#      error "CONFIG_LPC43_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_LPC43_PHYSR_SPEED
#      error "CONFIG_LPC43_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_LPC43_PHYSR_100MBPS
#      error "CONFIG_LPC43_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_LPC43_PHYSR_MODE
#      error "CONFIG_LPC43_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_LPC43_PHYSR_FULLDUPLEX
#      error "CONFIG_LPC43_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

#ifdef CONFIG_LPC43_ETH_PTP
#  warning "CONFIG_LPC43_ETH_PTP is not yet supported"
#endif

/* This driver does not use enhanced descriptors.  Enhanced descriptors must
 * be used, however, if time stamping or and/or IPv4 checksum offload is
 * supported.
 */

#undef CONFIG_LPC43_ETH_ENHANCEDDESC
#undef CONFIG_LPC43_ETH_HWCHECKSUM

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, or 16 bytes (depending on buswidth).  We
 * will use the 16-byte alignment in all cases.
 */

#define OPTIMAL_ETH_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 4 + 15) & ~15)

#ifndef CONFIG_LPC43_ETH_BUFSIZE
#  define CONFIG_LPC43_ETH_BUFSIZE OPTIMAL_ETH_BUFSIZE
#endif

#if CONFIG_LPC43_ETH_BUFSIZE > ETH_TDES1_TBS1_MASK
#  error "CONFIG_LPC43_ETH_BUFSIZE is too large"
#endif

#if (CONFIG_LPC43_ETH_BUFSIZE & 15) != 0
#  error "CONFIG_LPC43_ETH_BUFSIZE must be aligned"
#endif

#if CONFIG_LPC43_ETH_BUFSIZE != OPTIMAL_ETH_BUFSIZE
#  warning "You using an incomplete/untested configuration"
#endif

#ifndef CONFIG_LPC43_ETH_NRXDESC
#  define CONFIG_LPC43_ETH_NRXDESC 8
#endif
#ifndef CONFIG_LPC43_ETH_NTXDESC
#  define CONFIG_LPC43_ETH_NTXDESC 4
#endif

/* We need at least one more free buffer than transmit buffers */

#define LPC43_ETH_NFREEBUFFERS (CONFIG_LPC43_ETH_NTXDESC+1)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_NET_INFO
#  undef CONFIG_LPC43_ETHMAC_REGDEBUG
#endif

/* Clocking *****************************************************************/

/* Set MACMIIAR CR bits depending on HCLK setting */

#if BOARD_FCLKOUT_FREQUENCY >= 20000000 && BOARD_FCLKOUT_FREQUENCY < 35000000
#  define ETH_MACMIIA_CR ETH_MACMIIA_CR_20_35
#elif BOARD_FCLKOUT_FREQUENCY >= 35000000 && BOARD_FCLKOUT_FREQUENCY < 60000000
#  define ETH_MACMIIA_CR ETH_MACMIIA_CR_35_60
#elif BOARD_FCLKOUT_FREQUENCY >= 60000000 && BOARD_FCLKOUT_FREQUENCY < 100000000
#  define ETH_MACMIIA_CR ETH_MACMIIA_CR_60_100
#elif BOARD_FCLKOUT_FREQUENCY >= 100000000 && BOARD_FCLKOUT_FREQUENCY < 150000000
#  define ETH_MACMIIA_CR ETH_MACMIIA_CR_100_150
#elif BOARD_FCLKOUT_FREQUENCY >= 150000000 && BOARD_FCLKOUT_FREQUENCY <= 250000000
#  define ETH_MACMIIA_CR ETH_MACMIIA_CR_150_250
#elif BOARD_FCLKOUT_FREQUENCY >= 250000000 && BOARD_FCLKOUT_FREQUENCY <= 350000000
#  define ETH_MACMIIA_CR ETH_MACMIIA_CR_250_300
#else
#  error "BOARD_FCLKOUT_FREQUENCY not supportable"
#endif

/* Timing *******************************************************************/

/* TX timeout = 1 minute */

#define LPC43_TXTIMEOUT   (60*CLK_TCK)

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
 * ETH_MACCFG_RE    Bit 2:  Receiver enable
 * ETH_MACCFG_TE   Bit 3:  Transmitter enable
 * ETH_MACCFG_DC   Bit 4:  Deferral check
 * ETH_MACCFG_BL   Bits 5-6: Back-off limit
 * ETH_MACCFG_APCS Bit 7:  Automatic pad/CRC stripping
 * ETH_MACCFG_RD   Bit 9:  Retry disable
 * ETH_MACCFG_DM   Bit 11: Duplex mode
 * ETH_MACCFG_LM   Bit 12: Loopback mode
 * ETH_MACCFG_DO   Bit 13: Receive own disable
 * ETH_MACCFG_FES  Bit 14: Fast Ethernet speed
 * ETH_MACCFG_CSD  Bit 16: Carrier sense disable
 * ETH_MACCFG_IFG  Bits 17-19: Interframe gap
 * ETH_MACCFG_JD   Bit 22: Jabber disable
 * ETH_MACCFG_WD   Bit 23: Watchdog disable
 * ETH_MACCFG_CSTF Bits 25: CRC stripping for Type frames (F2/F4 only)
 */

#define MACCR_CLEAR_BITS \
  (ETH_MACCFG_RE | ETH_MACCFG_TE | ETH_MACCFG_DF | ETH_MACCFG_BL_MASK | \
   ETH_MACCFG_ACS | ETH_MACCFG_RD | ETH_MACCFG_DM | \
   ETH_MACCFG_LM | ETH_MACCFG_DO | ETH_MACCFG_FES | ETH_MACCFG_DCRS | \
   ETH_MACCFG_JE | ETH_MACCFG_IFG_MASK | ETH_MACCFG_JD | ETH_MACCFG_WD)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACCFG_RE   Receiver enable               0 (disabled)
 * ETH_MACCFG_TE   Transmitter enable            0 (disabled)
 * ETH_MACCFG_DC   Deferral check                0 (disabled)
 * ETH_MACCFG_BL   Back-off limit                0 (10)
 * ETH_MACCFG_APCS Automatic pad/CRC stripping   0 (disabled)
 * ETH_MACCFG_RD   Retry disable                 1 (disabled)
 * ETH_MACCFG_LM   Loopback mode                 0 (disabled)
 * ETH_MACCFG_ROD  Receive own disable           0 (enabled)
 * ETH_MACCFG_CSD  Carrier sense disable         0 (enabled)
 * ETH_MACCFG_IFG  Interframe gap                0 (96 bits)
 * ETH_MACCFG_JD   Jabber disable                0 (enabled)
 * ETH_MACCFG_WD   Watchdog disable              0 (enabled)
 * ETH_MACCFG_CSTF CRC stripping for Type frames 0 (disabled, F2/F4 only)
 *
 * The following are set conditioinally based on mode and speed.
 *
 * ETH_MACCFG_DM   Duplex mode                   Depends on priv->fduplex
 * ETH_MACCFG_FES  Fast Ethernet speed           Depends on priv->mbps100
 */

#  define MACCR_SET_BITS \
     (ETH_MACCFG_BL_10 | ETH_MACCFG_RD | ETH_MACCFG_IFG(96))

/* Clear the MACCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_MACFFR_PM   Bit 0: Promiscuous mode
 * ETH_MACFFR_HU   Bit 1: Hash unicast
 * ETH_MACFFR_HM   Bit 2: Hash multicast
 * ETH_MACFFR_DAIF Bit 3: Destination address inverse filtering
 * ETH_MACFFR_PAM  Bit 4: Pass all multicast
 * ETH_MACFFR_BFD  Bit 5: Broadcast frames disable
 * ETH_MACFFR_PCF  Bits 6-7: Pass control frames
 * ETH_MACFFR_SAIF Bit 8: Source address inverse filtering
 * ETH_MACFFR_SAF  Bit 9: Source address filter
 * ETH_MACFFR_HPF  Bit 10: Hash or perfect filter
 * ETH_MACFFR_RA   Bit 31: Receive all
 */

#define MACFFR_CLEAR_BITS \
  (ETH_MACFFLT_PR | ETH_MACFFLT_HUC | ETH_MACFFLT_HMC | ETH_MACFFLT_DAIF | \
   ETH_MACFFLT_PM | ETH_MACFFLT_DBF | ETH_MACFFLT_PCF_MASK | ETH_MACFFLT_HPF | \
   ETH_MACFFLT_RA)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACFFLT_PM   Promiscuous mode         0 (disabled)
 * ETH_MACFFLT_HU   Hash unicast             0 (perfect dest filtering)
 * ETH_MACFFLT_HM   Hash multicast           0 (perfect dest filtering)
 * ETH_MACFFLT_DAIF Destination address      0 (normal)
 *                  inverse filtering
 * ETH_MACFFLT_PAM  Pass all multicast       0 (Depends on HM bit)
 * ETH_MACFFLT_BFD  Broadcast frames disable 0 (enabled)
 * ETH_MACFFLT_PCF  Pass control frames      1 (block all but PAUSE)
 * ETH_MACFFLT_HPF  Hash or perfect filter   0 (Only matching frames passed)
 * ETH_MACFFLT_RA   Receive all              0 (disabled)
 */

#define MACFFR_SET_BITS (ETH_MACFFLT_PCF_PAUSE)

/* Clear the MACFCR bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ETH_MACFCR_FCB_BPA Bit 0: Flow control busy/back pressure activate
 * ETH_MACFCR_TFCE    Bit 1: Transmit flow control enable
 * ETH_MACFCR_RFCE    Bit 2: Receive flow control enable
 * ETH_MACFCR_UPFD    Bit 3: Unicast pause frame detect
 * ETH_MACFCR_PLT     Bits 4-5: Pause low threshold
 * ETH_MACFCR_ZQPD    Bit 7: Zero-quanta pause disable
 * ETH_MACFCR_PT      Bits 16-31: Pause time
 */

#define MACFCR_CLEAR_MASK \
  (ETH_MACFC_FCB | ETH_MACFC_TFE | ETH_MACFC_RFE | ETH_MACFC_UP | \
   ETH_MACFC_PLT_MASK | ETH_MACFC_DZPQ | ETH_MACFC_PT_MASK)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACFCR_FCB_BPA Flow control busy/back       0 (no pause control frame)
 *                    activate pressure
 * ETH_MACFCR_TFCE    Transmit flow control enable 0 (disabled)
 * ETH_MACFCR_RFCE    Receive flow control enable  0 (disabled)
 * ETH_MACFCR_UPFD    Unicast pause frame detect   0 (disabled)
 * ETH_MACFCR_PLT     Pause low threshold          0 (pause time - 4)
 * ETH_MACFCR_ZQPD    Zero-quanta pause disable    1 (disabled)
 * ETH_MACFCR_PT      Pause time                   0
 */

#define MACFCR_SET_MASK (ETH_MACFC_PLT(4) | ETH_MACFC_DZPQ)

/* Clear the DMAOMR bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ETH_DMAOPMODE_SR   Bit 1:  Start/stop receive
 * TH_DMAOMR_OSF      Bit 2:  Operate on second frame
 * ETH_DMAOPMODE_RTC  Bits 3-4: Receive threshold control
 * ETH_DMAOPMODE_FUGF Bit 6:  Forward undersized good frames
 * ETH_DMAOPMODE_FEF  Bit 7:  Forward error frames
 * ETH_DMAOPMODE_ST   Bit 13: Start/stop transmission
 * ETH_DMAOPMODE_TTC  Bits 14-16: Transmit threshold control
 * ETH_DMAOPMODE_FTF  Bit 20: Flush transmit FIFO
 * ETH_DMAOPMODE_TSF  Bit 21: Transmit store and forward
 * ETH_DMAOPMODE_DFRF Bit 24: Disable flushing of received frames
 * ETH_DMAOPMODE_RSF  Bit 25: Receive store and forward
 * TH_DMAOMR_DTCEFD   Bit 26: Dropping of TCP/IP checksum error
 *                            frames disable
 */

#define DMAOMR_CLEAR_MASK \
  (ETH_DMAOPMODE_SR | ETH_DMAOPMODE_OSF | ETH_DMAOPMODE_RTC_MASK | ETH_DMAOPMODE_FUF | \
   ETH_DMAOPMODE_FEF | ETH_DMAOPMODE_ST | ETH_DMAOPMODE_TTC_MASK | ETH_DMAOPMODE_FTF | \
   ETH_DMAOPMODE_DFF)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_DMAOPMODE_SR   Start/stop receive          0 (not running)
 * TH_DMAOMR_OSF      Operate on second frame     1 (enabled)
 * ETH_DMAOPMODE_RTC  Receive threshold control   0 (64 bytes)
 * ETH_DMAOPMODE_FUGF Forward undersized good     0 (disabled)
 *                    frames
 * ETH_DMAOPMODE_FEF  Forward error frames        0 (disabled)
 * ETH_DMAOPMODE_ST   Start/stop transmission     0 (not running)
 * ETH_DMAOPMODE_TTC  Transmit threshold control  0 (64 bytes)
 * ETH_DMAOPMODE_FTF  Flush transmit FIFO         0 (no flush)
 * ETH_DMAOPMODE_TSF  Transmit store and forward  Depends on
 *                                                CONFIG_LPC43_ETH_HWCHECKSUM
 * ETH_DMAOPMODE_DFRF Disable flushing of         0 (enabled)
 *                    received frames
 * ETH_DMAOPMODE_RSF  Receive store and forward   Depends on
 *                                                CONFIG_LPC43_ETH_HWCHECKSUM
 * TH_DMAOMR_DTCEFD   Dropping of TCP/IP checksum Depends on
 *                    error frames disable        CONFIG_LPC43_ETH_HWCHECKSUM
 *
 * When the checksum offload feature is enabled, we need to enable the Store
 * and Forward mode: the store and forward guarantee that a whole frame is
 * stored in the FIFO, so the MAC can insert/verify the checksum, if the
 * checksum is OK the DMA can handle the frame otherwise the frame is dropped
 */

#ifdef CONFIG_LPC43_ETH_HWCHECKSUM
#  define DMAOMR_SET_MASK \
    (ETH_DMAOPMODE_OSF | ETH_DMAOPMODE_RTC_64 | ETH_DMAOPMODE_TTC_64 | \
     ETH_DMAOPMODE_TSF | ETH_DMAOPMODE_RSF)
#else
#  define DMAOMR_SET_MASK \
    (ETH_DMAOPMODE_OSF | ETH_DMAOPMODE_RTC_64 | ETH_DMAOPMODE_TTC_64)
#endif

/* Clear the DMABMR bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ETH_DMABMODE_SR   Bit 0: Software reset
 * ETH_DMABMODE_DA   Bit 1: DMA Arbitration
 * ETH_DMABMODE_DSL  Bits 2-6: Descriptor skip length
 * ETH_DMABMODE_ATDS Bit 7: Enhanced descriptor format enable
 * ETH_DMABMODE_PBL  Bits 8-13: Programmable burst length
 * ETH_DMABMODE_RTPR Bits 14-15: RX TX priority ratio
 * ETH_DMABMODE_FB   Bit 16: Fixed burst
 * ETH_DMABMODE_RDP  Bits 17-22: RX DMA PBL
 * ETH_DMABMODE_USP  Bit 23: Use separate PBL
 * ETH_DMABMODE_FPM  Bit 24: 4xPBL mode
 * ETH_DMABMODE_AAB  Bit 25: Address-aligned beats
 * ETH_DMABMODE_MB   Bit 26: Mixed burst (F2/F4 only)
 */

#define DMABMR_CLEAR_MASK \
  (ETH_DMABMODE_SWR | ETH_DMABMODE_DA | ETH_DMABMODE_DSL_MASK | ETH_DMABMODE_ATDS | \
   ETH_DMABMODE_PBL_MASK | ETH_DMABMODE_PR_MASK | ETH_DMABMODE_FB | ETH_DMABMODE_RPBL_MASK | \
   ETH_DMABMODE_USP | ETH_DMABMODE_PBL8X | ETH_DMABMODE_AAL | ETH_DMABMODE_MB | ETH_DMABMODE_TXPR)

/* The following bits are set or left zero unconditionally in all modes.
 *
 *
 * ETH_DMABMODE_SR   Software reset             0 (no reset)
 * ETH_DMABMODE_DA   DMA Arbitration            0 (round robin)
 * ETH_DMABMODE_DSL  Descriptor skip length     0
 * ETH_DMABMODE_ATDS Enhanced descriptor format Depends on
 * enable                                       CONFIG_LPC43_ETH_ENHANCEDDESC
 * ETH_DMABMODE_PBL  Programmable burst length  32 beats
 * ETH_DMABMODE_RTPR RX TX priority ratio       2:1
 * ETH_DMABMODE_FB   Fixed burst                1 (enabled)
 * ETH_DMABMODE_RDP  RX DMA PBL                 32 beats
 * ETH_DMABMODE_USP  Use separate PBL           1 (enabled)
 * ETH_DMABMODE_FPM  4xPBL mode                 0 (disabled)
 * ETH_DMABMODE_AAB  Address-aligned beats      1 (enabled)
 * ETH_DMABMODE_MB   Mixed burst                0 (disabled, F2/F4 only)
 */

#ifdef CONFIG_LPC43_ETH_ENHANCEDDESC
#  define DMABMR_SET_MASK \
     (ETH_DMABMODE_DSL(0) | ETH_DMABMODE_PBL(32) | ETH_DMABMODE_ATDS | ETH_DMABMODE_RTPR_2TO1 | \
      ETH_DMABMODE_FB | ETH_DMABMODE_RPBL(32) | ETH_DMABMODE_USP | ETH_DMABMODE_AAB)
#else
#  define DMABMR_SET_MASK \
     (ETH_DMABMODE_DSL(0) | ETH_DMABMODE_PBL(32) | ETH_DMABMODE_PR_2TO1 | ETH_DMABMODE_FB | \
      ETH_DMABMODE_RPBL(32) | ETH_DMABMODE_USP | ETH_DMABMODE_AAL)
#endif

/* Interrupt bit sets *******************************************************/

/* All interrupts in the normal and abnormal interrupt summary. Early
 * transmit interrupt (ETI) is excluded from the abnormal set because it
 * causes too many interrupts and is not interesting.
 */

#define ETH_DMAINT_NORMAL \
  (ETH_DMAINT_TI | ETH_DMAINT_TU | ETH_DMAINT_RI | ETH_DMAINT_ERI)

#define ETH_DMAINT_ABNORMAL \
  (ETH_DMAINT_TPS | ETH_DMAINT_TJT | ETH_DMAINT_OVF | ETH_DMAINT_UNF | \
   ETH_DMAINT_RU | ETH_DMAINT_RPS | ETH_DMAINT_RWT | /* ETH_DMAINT_ETI | */ \
   ETH_DMAINT_FBI)

/* Normal receive, transmit, error interrupt enable bit sets */

#define ETH_DMAINT_RECV_ENABLE    (ETH_DMAINT_NIS | ETH_DMAINT_RI)
#define ETH_DMAINT_XMIT_ENABLE    (ETH_DMAINT_NIS | ETH_DMAINT_TI)
#define ETH_DMAINT_XMIT_DISABLE   (ETH_DMAINT_TI)

#ifdef CONFIG_DEBUG_NET
#  define ETH_DMAINT_ERROR_ENABLE (ETH_DMAINT_AIS | ETH_DMAINT_ABNORMAL)
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

/* The lpc43_ethmac_s encapsulates all state information for a single
 * hardware interface
 */

struct lpc43_ethmac_s
{
  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  uint8_t              mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t              fduplex : 1; /* Full (vs. half) duplex */
  struct wdog_s        txtimeout;   /* TX timeout timer */
  struct work_s        irqwork;     /* For deferring work to the work queue */
  struct work_s        pollwork;    /* For deferring work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s  dev;         /* Interface understood by the network */

  /* Used to track transmit and receive descriptors */

  struct eth_txdesc_s *txhead;      /* Next available TX descriptor */
  struct eth_rxdesc_s *rxhead;      /* Next available RX descriptor */

  struct eth_txdesc_s *txtail;      /* First "in_flight" TX descriptor */
  struct eth_rxdesc_s *rxcurr;      /* First RX descriptor of the segment */
  uint16_t             segments;    /* RX segment count */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */
  sq_queue_t           freeb;       /* The free buffer list */

  /* Descriptor allocations */

  struct eth_rxdesc_s rxtable[CONFIG_LPC43_ETH_NRXDESC];
  struct eth_txdesc_s txtable[CONFIG_LPC43_ETH_NTXDESC];

  /* Buffer allocations */

  uint8_t rxbuffer[CONFIG_LPC43_ETH_NRXDESC*CONFIG_LPC43_ETH_BUFSIZE];
  uint8_t alloc[LPC43_ETH_NFREEBUFFERS*CONFIG_LPC43_ETH_BUFSIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lpc43_ethmac_s g_lpc43ethmac;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_LPC43_ETHMAC_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr);
static void lpc43_putreg(uint32_t val, uint32_t addr);
static void lpc43_checksetup(void);
#else
# define lpc43_getreg(addr)      getreg32(addr)
# define lpc43_putreg(val,addr)  putreg32(val,addr)
# define lpc43_checksetup()
#endif

/* Free buffer management */

static void lpc43_initbuffer(struct lpc43_ethmac_s *priv);
static inline uint8_t *lpc43_allocbuffer(struct lpc43_ethmac_s *priv);
static inline void lpc43_freebuffer(struct lpc43_ethmac_s *priv,
              uint8_t *buffer);
static inline bool lpc43_isfreebuffer(struct lpc43_ethmac_s *priv);

/* Common TX logic */

static int  lpc43_transmit(struct lpc43_ethmac_s *priv);
static int  lpc43_txpoll(struct net_driver_s *dev);
static void lpc43_dopoll(struct lpc43_ethmac_s *priv);

/* Interrupt handling */

static void lpc43_enableint(struct lpc43_ethmac_s *priv,
              uint32_t ierbit);
static void lpc43_disableint(struct lpc43_ethmac_s *priv,
              uint32_t ierbit);

static void lpc43_freesegment(struct lpc43_ethmac_s *priv,
              struct eth_rxdesc_s *rxfirst, int segments);
static int  lpc43_recvframe(struct lpc43_ethmac_s *priv);
static void lpc43_receive(struct lpc43_ethmac_s *priv);
static void lpc43_freeframe(struct lpc43_ethmac_s *priv);
static void lpc43_txdone(struct lpc43_ethmac_s *priv);

static void lpc43_interrupt_work(void *arg);
static int  lpc43_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void lpc43_txtimeout_work(void *arg);
static void lpc43_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  lpc43_ifup(struct net_driver_s *dev);
static int  lpc43_ifdown(struct net_driver_s *dev);

static void lpc43_txavail_work(void *arg);
static int  lpc43_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  lpc43_addmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int  lpc43_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  lpc43_ioctl(struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif
/* Descriptor Initialization */

static void lpc43_txdescinit(struct lpc43_ethmac_s *priv);
static void lpc43_rxdescinit(struct lpc43_ethmac_s *priv);

/* PHY Initialization */
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int  lpc43_phyintenable(struct lpc43_ethmac_s *priv);
#endif
static int  lpc43_phyread(uint16_t phydevaddr, uint16_t phyregaddr,
              uint16_t *value);
static int  lpc43_phywrite(uint16_t phydevaddr, uint16_t phyregaddr,
              uint16_t value);
#ifdef CONFIG_ETH0_PHY_DM9161
static inline int lpc43_dm9161(struct lpc43_ethmac_s *priv);
#endif
static int  lpc43_phyinit(struct lpc43_ethmac_s *priv);

/* MAC/DMA Initialization */

#ifdef CONFIG_LPC43_MII
static inline void lpc43_selectmii(void);
#endif
#ifdef CONFIG_LPC43_RMII
static inline void lpc43_selectrmii(void);
#endif
static inline void lpc43_ethgpioconfig(struct lpc43_ethmac_s *priv);
static void lpc43_ethreset(struct lpc43_ethmac_s *priv);
static int  lpc43_macconfig(struct lpc43_ethmac_s *priv);
static void lpc43_macaddress(struct lpc43_ethmac_s *priv);
#ifdef CONFIG_NET_ICMPv6
static void lpc43_ipv6multicast(struct lpc43_ethmac_s *priv);
#endif
static int  lpc43_macenable(struct lpc43_ethmac_s *priv);
static int  lpc43_ethconfig(struct lpc43_ethmac_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_getreg
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

#ifdef CONFIG_LPC43_ETHMAC_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr)
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
 * Name: lpc43_putreg
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

#ifdef CONFIG_LPC43_ETHMAC_REGDEBUG
static void lpc43_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  ninfo("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: lpc43_checksetup
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

#ifdef CONFIG_LPC43_ETHMAC_REGDEBUG
static void lpc43_checksetup(void)
{
}
#endif

/****************************************************************************
 * Function: lpc43_initbuffer
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

static void lpc43_initbuffer(struct lpc43_ethmac_s *priv)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = priv->alloc;
       i < LPC43_ETH_NFREEBUFFERS;
       i++, buffer += CONFIG_LPC43_ETH_BUFSIZE)
    {
      sq_addlast((sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: lpc43_allocbuffer
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

static inline uint8_t *lpc43_allocbuffer(struct lpc43_ethmac_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: lpc43_freebuffer
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

static inline void lpc43_freebuffer(struct lpc43_ethmac_s *priv,
                                    uint8_t *buffer)
{
  /* Free the buffer by adding it to the end of the free buffer list */

  sq_addlast((sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: lpc43_isfreebuffer
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

static inline bool lpc43_isfreebuffer(struct lpc43_ethmac_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
}

/****************************************************************************
 * Function: lpc43_transmit
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

static int lpc43_transmit(struct lpc43_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  struct eth_txdesc_s *txfirst;

  /* The internal (optimal) network buffer size may be configured to be
   * larger than the Ethernet buffer size.
   */

#if OPTIMAL_ETH_BUFSIZE > CONFIG_LPC43_ETH_BUFSIZE
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

  DEBUGASSERT(txdesc && (txdesc->tdes0 & ETH_TDES0_OWN) == 0);

  /* Is the size to be sent greater than the size of the Ethernet buffer? */

  DEBUGASSERT(priv->dev.d_len > 0 && priv->dev.d_buf != NULL);

#if OPTIMAL_ETH_BUFSIZE > CONFIG_LPC43_ETH_BUFSIZE
  if (priv->dev.d_len > CONFIG_LPC43_ETH_BUFSIZE)
    {
      /* Yes... how many buffers will be need to send the packet? */

      bufcount = (priv->dev.d_len + (CONFIG_LPC43_ETH_BUFSIZE - 1)) /
                  CONFIG_LPC43_ETH_BUFSIZE;
      lastsize = priv->dev.d_len - (bufcount - 1) * CONFIG_LPC43_ETH_BUFSIZE;

      ninfo("bufcount: %d lastsize: %d\n", bufcount, lastsize);

      /* Set the first segment bit in the first TX descriptor */

      txdesc->tdes0 |= ETH_TDES0_FS;

      /* Set up all but the last TX descriptor */

      buffer = priv->dev.d_buf;

      for (i = 0; i < bufcount; i++)
        {
          /* This could be a normal event but the design does not handle it */

          DEBUGASSERT((txdesc->tdes0 & ETH_TDES0_OWN) == 0);

          /* Set the Buffer1 address pointer */

          txdesc->tdes2 = (uint32_t)buffer;

          /* Set the buffer size in all TX descriptors */

          if (i == (bufcount - 1))
            {
              /* This is the last segment.  Set the last segment bit in the
               * last TX descriptor and ask for an interrupt when this
               * segment transfer completes.
               */

              txdesc->tdes0 |= (ETH_TDES0_LS | ETH_TDES0_IC);

              /* This segment is, most likely, of fractional buffersize */

              txdesc->tdes1  = lastsize;
              buffer        += lastsize;
            }
          else
            {
              /* This is not the last segment.  We don't want an interrupt
               * when this segment transfer completes.
               */

              txdesc->tdes0 &= ~ETH_TDES0_IC;

              /* The size of the transfer is the whole buffer */

              txdesc->tdes1  = CONFIG_LPC43_ETH_BUFSIZE;
              buffer        += CONFIG_LPC43_ETH_BUFSIZE;
            }

          /* Give the descriptor to DMA */

          txdesc->tdes0 |= ETH_TDES0_OWN;
          txdesc         = (struct eth_txdesc_s *)txdesc->tdes3;
        }
    }
  else
#endif
    {
      /* The single descriptor is both the first and last segment.  And we do
       * want an interrupt when the transfer completes.
       */

      txdesc->tdes0 |= (ETH_TDES0_FS | ETH_TDES0_LS | ETH_TDES0_IC);

      /* Set frame size */

      DEBUGASSERT(priv->dev.d_len <= CONFIG_NET_ETH_PKTSIZE);
      txdesc->tdes1 = priv->dev.d_len;

      /* Set the Buffer1 address pointer */

      txdesc->tdes2 = (uint32_t)priv->dev.d_buf;

      /* Set OWN bit of the TX descriptor tdes0.  This gives the buffer to
       * Ethernet DMA
       */

      txdesc->tdes0 |= ETH_TDES0_OWN;

      /* Point to the next available TX descriptor */

      txdesc = (struct eth_txdesc_s *)txdesc->tdes3;
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
   * interrupts too. This is because receive events can trigger more
   * un-stoppable transmit events.
   */

  if (priv->inflight >= CONFIG_LPC43_ETH_NTXDESC)
    {
      lpc43_disableint(priv, ETH_DMAINT_RI);
    }

  /* Check if the TX Buffer unavailable flag is set */

  if ((lpc43_getreg(LPC43_ETH_DMASTAT) & ETH_DMAINT_TU) != 0)
    {
      /* Clear TX Buffer unavailable flag */

      lpc43_putreg(ETH_DMAINT_TU, LPC43_ETH_DMASTAT);

      /* Resume DMA transmission */

      lpc43_putreg(0, LPC43_ETH_DMATXPD);
    }

  /* Enable TX interrupts */

  lpc43_enableint(priv, ETH_DMAINT_TI);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, LPC43_TXTIMEOUT,
           lpc43_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: lpc43_txpoll
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

static int lpc43_txpoll(struct net_driver_s *dev)
{
  struct lpc43_ethmac_s *priv =
    (struct lpc43_ethmac_s *)dev->d_private;

  /* Send the packet */

  lpc43_transmit(priv);
  DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU. We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ETH_TDES0_OWN may be cleared BUT still
   * not available because lpc43_freeframe() has not yet run. If
   * lpc43_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_LPC43_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & ETH_TDES0_OWN) != 0 ||
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

  dev->d_buf = lpc43_allocbuffer(priv);

  /* We can't continue the poll if we have no buffers */

  if (dev->d_buf == NULL)
    {
      /* Terminate the poll. */

      return -ENOMEM;
    }

  return 0;
}

/****************************************************************************
 * Function: lpc43_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (lpc43_txdone),
 *   2. When new TX data is available (lpc43_txavail_process), and
 *   3. After a TX timeout to restart the sending process
 *      (lpc43_txtimeout_process).
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

static void lpc43_dopoll(struct lpc43_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ETH_TDES0_OWN may be cleared BUT still
   * not available because lpc43_freeframe() has not yet run. If
   * lpc43_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_LPC43_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & ETH_TDES0_OWN) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then poll for new XMIT data.
       * Allocate a buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = lpc43_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          devif_poll(dev, lpc43_txpoll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              lpc43_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }
}

/****************************************************************************
 * Function: lpc43_enableint
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

static void lpc43_enableint(struct lpc43_ethmac_s *priv,
                            uint32_t ierbit)
{
  uint32_t regval;

  /* Enable the specified "normal" interrupt */

  regval  = lpc43_getreg(LPC43_ETH_DMAINTEN);
  regval |= (ETH_DMAINT_NIS | ierbit);
  lpc43_putreg(regval, LPC43_ETH_DMAINTEN);
}

/****************************************************************************
 * Function: lpc43_disableint
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

static void lpc43_disableint(struct lpc43_ethmac_s *priv,
                             uint32_t ierbit)
{
  uint32_t regval;

  /* Disable the "normal" interrupt */

  regval  = lpc43_getreg(LPC43_ETH_DMAINTEN);
  regval &= ~ierbit;

  /* Are all "normal" interrupts now disabled? */

  if ((regval & ETH_DMAINT_NORMAL) == 0)
    {
      /* Yes.. disable normal interrupts */

      regval &= ~ETH_DMAINT_NIS;
    }

  lpc43_putreg(regval, LPC43_ETH_DMAINTEN);
}

/****************************************************************************
 * Function: lpc43_freesegment
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

static void lpc43_freesegment(struct lpc43_ethmac_s *priv,
                              struct eth_rxdesc_s *rxfirst, int segments)
{
  struct eth_rxdesc_s *rxdesc;
  int i;

  ninfo("rxfirst: %p segments: %d\n", rxfirst, segments);

  /* Set OWN bit in RX descriptors.  This gives the buffers back to DMA */

  rxdesc = rxfirst;
  for (i = 0; i < segments; i++)
    {
      rxdesc->rdes0 = ETH_RDES0_OWN;
      rxdesc = (struct eth_rxdesc_s *)rxdesc->rdes3;
    }

  /* Reset the segment management logic */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Check if the RX Buffer unavailable flag is set */

  if ((lpc43_getreg(LPC43_ETH_DMASTAT) & ETH_DMAINT_RU) != 0)
    {
      /* Clear RBUS Ethernet DMA flag */

      lpc43_putreg(ETH_DMAINT_RU, LPC43_ETH_DMASTAT);

      /* Resume DMA reception */

      lpc43_putreg(0, LPC43_ETH_DMARXPD);
    }
}

/****************************************************************************
 * Function: lpc43_recvframe
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

static int lpc43_recvframe(struct lpc43_ethmac_s *priv)
{
  struct eth_rxdesc_s *rxdesc;
  struct eth_rxdesc_s *rxcurr;
  uint8_t *buffer;
  int i;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  /* Check if there are free buffers.  We cannot receive new frames in this
   * design unless there is at least one free buffer.
   */

  if (!lpc43_isfreebuffer(priv))
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
   * that we receive can generate an unstoppable transmission.  So we have
   * to stop receiving when we can not longer transmit.  In this case, the
   * transmit logic should also have disabled further RX interrupts.
   */

  rxdesc = priv->rxhead;
  for (i = 0;
       (rxdesc->rdes0 & ETH_RDES0_OWN) == 0 &&
        i < CONFIG_LPC43_ETH_NRXDESC &&
        priv->inflight < CONFIG_LPC43_ETH_NTXDESC;
       i++)
    {
      /* Check if this is the first segment in the frame */

      if ((rxdesc->rdes0 & ETH_RDES0_FS) != 0 &&
          (rxdesc->rdes0 & ETH_RDES0_LS) == 0)
        {
          priv->rxcurr   = rxdesc;
          priv->segments = 1;
        }

      /* Check if this is an intermediate segment in the frame */

      else if (((rxdesc->rdes0 & ETH_RDES0_LS) == 0) &&
               ((rxdesc->rdes0 & ETH_RDES0_FS) == 0))
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

          if ((rxdesc->rdes0 & ETH_RDES0_ES) == 0)
            {
              struct net_driver_s *dev = &priv->dev;

              /* Get the Frame Length of the received packet: subtract 4
               * bytes of the CRC
               */

              dev->d_len = ((rxdesc->rdes0 & ETH_RDES0_FL_MASK) >>
                            ETH_RDES0_FL_SHIFT) - 4;

              /* Get a buffer from the free list.  We don't even check if
               * this is successful because we already assure the free
               * list is not empty above.
               */

              buffer = lpc43_allocbuffer(priv);

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

              priv->rxhead   = (struct eth_rxdesc_s *)rxdesc->rdes3;
              lpc43_freesegment(priv, rxcurr, priv->segments);

              ninfo("rxhead: %p d_buf: %p d_len: %d\n",
                    priv->rxhead, dev->d_buf, dev->d_len);

              return OK;
            }
          else
            {
              /* Drop the frame that contains the errors, reset the segment
               * scanning logic, and continue scanning with the next frame.
               */

              nwarn("WARNING: Dropped, RX descriptor errors: %08" PRIx32
                    "\n",
                    rxdesc->rdes0);
              lpc43_freesegment(priv, rxcurr, priv->segments);
            }
        }

      /* Try the next descriptor */

      rxdesc = (struct eth_rxdesc_s *)rxdesc->rdes3;
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
 * Function: lpc43_receive
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

static void lpc43_receive(struct lpc43_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while lpc43_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  while (lpc43_recvframe(priv) == OK)
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
          nwarn("WARNING: Dropped, Too big: %d\n", dev->d_len);

          /* Free dropped packet buffer */

          if (dev->d_buf)
            {
              lpc43_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
              dev->d_len = 0;
            }

          continue;
        }

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(&priv->dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              lpc43_transmit(priv);
            }
        }
      else
#endif /* CONFIG_NET_IPv4 */
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
              /* And send the packet */

              lpc43_transmit(priv);
            }
        }
      else
#endif /* CONFIG_NET_IPv6 */
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
              lpc43_transmit(priv);
            }
        }
      else
#endif
        {
          nwarn("WARNING: Dropped, Unknown type: %04x\n", BUF->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          lpc43_freebuffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Function: lpc43_freeframe
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

static void lpc43_freeframe(struct lpc43_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  int i;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txdesc = priv->txtail;
  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

      for (i = 0; (txdesc->tdes0 & ETH_TDES0_OWN) == 0; i++)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          ninfo("txtail: %p tdes0: %08" PRIx32
                " tdes2: %08" PRIx32 " tdes3: %08" PRIx32 "\n",
                txdesc, txdesc->tdes0, txdesc->tdes2, txdesc->tdes3);

          DEBUGASSERT(txdesc->tdes2 != 0);

          /* Check if this is the first segment of a TX frame. */

          if ((txdesc->tdes0 & ETH_TDES0_FS) != 0)
            {
              /* Yes.. Free the buffer */

              lpc43_freebuffer(priv, (uint8_t *)txdesc->tdes2);
            }

          /* In any event, make sure that TDES2 is nullified. */

          txdesc->tdes2 = 0;

          /* Check if this is the last segment of a TX frame */

          if ((txdesc->tdes0 & ETH_TDES0_LS) != 0)
            {
              /* Yes.. Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If all of the TX descriptors were in-flight,
               * then RX interrupts may have been disabled...
               * we can re-enable them now.
               */

              lpc43_enableint(priv, ETH_DMAINT_RI);

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return;
                }
            }

          /* Try the next descriptor in the TX chain */

          txdesc = (struct eth_txdesc_s *)txdesc->tdes3;
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
 * Function: lpc43_txdone
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

static void lpc43_txdone(struct lpc43_ethmac_s *priv)
{
  DEBUGASSERT(priv->txtail != NULL);

  /* Scan the TX descriptor change, returning buffers to free list */

  lpc43_freeframe(priv);

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      /* Cancel the TX timeout */

      wd_cancel(&priv->txtimeout);

      /* And disable further TX interrupts. */

      lpc43_disableint(priv, ETH_DMAINT_TI);
    }

  /* Then poll the network for new XMIT data */

  lpc43_dopoll(priv);
}

/****************************************************************************
 * Function: lpc43_interrupt_work
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

static void lpc43_interrupt_work(void *arg)
{
  struct lpc43_ethmac_s *priv = (struct lpc43_ethmac_s *)arg;
  uint32_t dmasr;

  DEBUGASSERT(priv);

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  net_lock();
  dmasr = lpc43_getreg(LPC43_ETH_DMASTAT);

  /* Mask only enabled interrupts.  This depends on the fact that the
   * interrupt related bits (0-16) correspond in these two registers.
   */

  dmasr &= lpc43_getreg(LPC43_ETH_DMAINTEN);

  /* Check if there are pending "normal" interrupts */

  if ((dmasr & ETH_DMAINT_NIS) != 0)
    {
      /* Yes.. Check if we received an incoming packet, if so, call
       * lpc43_receive()
       */

      if ((dmasr & ETH_DMAINT_RI) != 0)
        {
          /* Clear the pending receive interrupt */

          lpc43_putreg(ETH_DMAINT_RI, LPC43_ETH_DMASTAT);

          /* Handle the received package */

          lpc43_receive(priv);
        }

      /* Check if a packet transmission just completed.  If so, call
       * lpc43_txdone(). This may disable further TX interrupts if there
       * are no pending tansmissions.
       */

      if ((dmasr & ETH_DMAINT_TI) != 0)
        {
          /* Clear the pending receive interrupt */

          lpc43_putreg(ETH_DMAINT_TI, LPC43_ETH_DMASTAT);

          /* Check if there are pending transmissions */

          lpc43_txdone(priv);
        }

      /* Clear the pending normal summary interrupt */

      lpc43_putreg(ETH_DMAINT_NIS, LPC43_ETH_DMASTAT);
    }

  /* Handle error interrupt only if CONFIG_DEBUG_NET is eanbled */

#ifdef CONFIG_DEBUG_NET
  /* Check if there are pending "abnormal" interrupts */

  if ((dmasr & ETH_DMAINT_AIS) != 0)
    {
      /* Just let the user know what happened */

      nerr("ERROR: Abnormal event(s): %08x\n", dmasr);

      /* Clear all pending abnormal events */

      lpc43_putreg(ETH_DMAINT_ABNORMAL, LPC43_ETH_DMASTAT);

      /* Clear the pending abnormal summary interrupt */

      lpc43_putreg(ETH_DMAINT_AIS, LPC43_ETH_DMASTAT);
    }
#endif /* CONFIG_DEBUG_NET */

  net_unlock();

  /* Re-enable Ethernet interrupts at the NVIC */

  up_enable_irq(LPC43M4_IRQ_ETHERNET);
}

/****************************************************************************
 * Function: lpc43_interrupt
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

static int lpc43_interrupt(int irq, void *context, void *arg)
{
  struct lpc43_ethmac_s *priv = &g_lpc43ethmac;
  uint32_t dmasr;

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dmasr = lpc43_getreg(LPC43_ETH_DMASTAT);
  if (dmasr != 0)
    {
      /* Disable further Ethernet interrupts.  Because Ethernet interrupts
       * are also disabled if the TX timeout event occurs, there can be no
       * race condition here.
       */

      up_disable_irq(LPC43M4_IRQ_ETHERNET);

      /* Check if a packet transmission just completed. */

      if ((dmasr & ETH_DMAINT_TI) != 0)
        {
          /* If a TX transfer just completed, then cancel the TX timeout so
           * there will be no race condition between any subsequent timeout
           * expiration and the deferred interrupt processing.
           */

           wd_cancel(&priv->txtimeout);
        }

      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(ETHWORK, &priv->irqwork, lpc43_interrupt_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: lpc43_txtimeout_work
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

static void lpc43_txtimeout_work(void *arg)
{
  struct lpc43_ethmac_s *priv = (struct lpc43_ethmac_s *)arg;

  /* Then reset the hardware.  Just take the interface down, then back
   * up again.
   */

  net_lock();
  lpc43_ifdown(&priv->dev);
  lpc43_ifup(&priv->dev);

  /* Then poll the network for new XMIT data */

  lpc43_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: lpc43_txtimeout_expiry
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

static void lpc43_txtimeout_expiry(wdparm_t arg)
{
  struct lpc43_ethmac_s *priv = (struct lpc43_ethmac_s *)arg;

  ninfo("Timeout!\n");

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   *
   * Interrupts will be re-enabled when lpc43_ifup() is called.
   */

  up_disable_irq(LPC43M4_IRQ_ETHERNET);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * perhaps cancelling any pending IRQ processing.
   */

  work_queue(ETHWORK, &priv->irqwork, lpc43_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: lpc43_ifup
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

static int lpc43_ifup(struct net_driver_s *dev)
{
  struct lpc43_ethmac_s *priv =
    (struct lpc43_ethmac_s *)dev->d_private;
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

  ret = lpc43_ethconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  up_enable_irq(LPC43M4_IRQ_ETHERNET);

  lpc43_checksetup();
  return OK;
}

/****************************************************************************
 * Function: lpc43_ifdown
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

static int lpc43_ifdown(struct net_driver_s *dev)
{
  struct lpc43_ethmac_s *priv =
    (struct lpc43_ethmac_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(LPC43M4_IRQ_ETHERNET);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the lpc43_ifup() always
   * successfully brings the interface back up.
   */

  lpc43_ethreset(priv);

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: lpc43_txavail_work
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

static void lpc43_txavail_work(void *arg)
{
  struct lpc43_ethmac_s *priv = (struct lpc43_ethmac_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  ninfo("ifup: %d\n", priv->ifup);
  if (priv->ifup)
    {
      /* Poll for new XMIT data */

      lpc43_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: lpc43_txavail
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

static int lpc43_txavail(struct net_driver_s *dev)
{
  struct lpc43_ethmac_s *priv =
    (struct lpc43_ethmac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, lpc43_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: lpc43_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used by LPC43 to check an Ethernet frame
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
static uint32_t lpc43_calcethcrc(const uint8_t *data, size_t length)
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
              /* Return success, remembering where we should re-start
               * scanning and resetting the segment scanning logic
               */

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
 * Function: lpc43_addmac
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
static int lpc43_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast hash table */

  crc = lpc43_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = LPC43_ETH_MACHTHI;
      hashindex -= 32;
    }
  else
    {
      registeraddress = LPC43_ETH_MACHTLO;
    }

  temp = lpc43_getreg(registeraddress);
  temp |= 1 << hashindex;
  lpc43_putreg(temp, registeraddress);

  temp = lpc43_getreg(LPC43_ETH_MACFFLT);
  temp |= (ETH_MACFFLT_HM | ETH_MACFFLT_HPF);
  lpc43_putreg(temp, LPC43_ETH_MACFFLT);

  return OK;
}
#endif

/****************************************************************************
 * Function: lpc43_rmmac
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
static int lpc43_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Remove the MAC address to the hardware multicast hash table */

  crc = lpc43_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = LPC43_ETH_MACHTHI;
      hashindex -= 32;
    }
  else
    {
      registeraddress = LPC43_ETH_MACHTLO;
    }

  temp = lpc43_getreg(registeraddress);
  temp &= ~(1 << hashindex);
  lpc43_putreg(temp, registeraddress);

  /* If there is no address registered any more, delete multicast filtering */

  if (lpc43_getreg(LPC43_ETH_MACHTHI) == 0 &&
      lpc43_getreg(LPC43_ETH_MACHTLO) == 0)
    {
      temp = lpc43_getreg(LPC43_ETH_MACFFLT);
      temp &= ~(ETH_MACFFLT_HM | ETH_MACFFLT_HPF);
      lpc43_putreg(temp, LPC43_ETH_MACFFLT);
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: lpc43_txdescinit
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

static void lpc43_txdescinit(struct lpc43_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  int i;

  /* priv->txhead point to the first, available TX descriptor in the chain.
   * Set the priv->txhead pointer to the first descriptor in the table.
   */

  priv->txhead = priv->txtable;

  /* priv->txtail will point to the first segment of the oldest pending
   * "in-flight" TX transfer.  NULL means that there are no active TX
   * transfers.
   */

  priv->txtail   = NULL;
  priv->inflight = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < CONFIG_LPC43_ETH_NTXDESC; i++)
    {
      txdesc = &priv->txtable[i];

      /* Set Second Address Chained bit */

      txdesc->tdes0 = ETH_TDES0_TCH;

#ifdef CHECKSUM_BY_HARDWARE
      /* Enable the checksum insertion for the TX frames */

      txdesc->tdes0 |= ETH_TDES0_CIC_ALL;
#endif

      /* Clear Buffer1 address pointer (buffers will be assigned as they
       * are used)
       */

      txdesc->tdes2 = 0;

      /* Initialize the next descriptor with
       * the Next Descriptor Polling Enable
       */

      if (i < (CONFIG_LPC43_ETH_NTXDESC - 1))
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

  lpc43_putreg((uint32_t)priv->txtable, LPC43_ETH_DMATXDLA);
}

/****************************************************************************
 * Function: lpc43_rxdescinit
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

static void lpc43_rxdescinit(struct lpc43_ethmac_s *priv)
{
  struct eth_rxdesc_s *rxdesc;
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

  for (i = 0; i < CONFIG_LPC43_ETH_NRXDESC; i++)
    {
      rxdesc = &priv->rxtable[i];

      /* Set Own bit of the RX descriptor rdes0 */

      rxdesc->rdes0 = ETH_RDES0_OWN;

      /* Set Buffer1 size and Second Address Chained bit and enabled DMA
       * RX desc receive interrupt
       */

      rxdesc->rdes1 = ETH_RDES1_RCH | (uint32_t)CONFIG_LPC43_ETH_BUFSIZE;

      /* Set Buffer1 address pointer */

      rxdesc->rdes2 = (uint32_t)&priv->rxbuffer[i*CONFIG_LPC43_ETH_BUFSIZE];

      /* Initialize the next descriptor with
       * the Next Descriptor Polling Enable
       */

      if (i < (CONFIG_LPC43_ETH_NRXDESC - 1))
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

  lpc43_putreg((uint32_t)priv->rxtable, LPC43_ETH_DMARXDLA);
}

/****************************************************************************
 * Function: lpc43_ioctl
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
static int lpc43_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
  struct lpc43_ethmac_s *priv =
    (struct lpc43_ethmac_s *)dev->d_private;
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
      if (ret == OK)
          {
            /* Enable PHY link up/down interrupts */

            ret = lpc43_phyintenable(priv);
          }
    }
    break;
#endif

  case SIOCGMIIPHY: /* Get MII PHY address */
    {
      struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
      req->phy_id = CONFIG_LPC43_PHYADDR;
      ret = OK;
    }
    break;

  case SIOCGMIIREG: /* Get register from MII PHY */
    {
      struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
      ret = lpc43_phyread(req->phy_id, req->reg_num, &req->val_out);
    }
    break;

  case SIOCSMIIREG: /* Set register in MII PHY */
    {
      struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
      ret = lpc43_phywrite(req->phy_id, req->reg_num, req->val_in);
    }
    break;
#endif /* ifdef CONFIG_NETDEV_PHY_IOCTL */

  default:
    ret = -ENOTTY;
    break;
  }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: lpc43_phyintenable
 *
 * Description:
 *  Enable link up/down PHY interrupts.  The interrupt protocol is like
 *  this:
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

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int lpc43_phyintenable(struct lpc43_ethmac_s *priv)
{
#warning Missing logic
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Function: lpc43_phyread
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

static int lpc43_phyread(uint16_t phydevaddr,
                         uint16_t phyregaddr, uint16_t *value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MACMIIAR register,
   * preserving CSR Clock Range CR[2:0] bits
   */

  regval  = lpc43_getreg(LPC43_ETH_MACMIIA);
  regval &= ETH_MACMIIA_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the buy bit.
   * the  ETH_MACMIIA_WR is clear, indicating a read operation.
   */

  regval |= (phydevaddr << ETH_MACMIIA_PA_SHIFT) & ETH_MACMIIA_PA_MASK;
  regval |= (phyregaddr << ETH_MACMIIA_MR_SHIFT) & ETH_MACMIIA_MR_MASK;
  regval |= ETH_MACMIIA_GB;

  lpc43_putreg(regval, LPC43_ETH_MACMIIA);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_READ_TIMEOUT; timeout++)
    {
      if ((lpc43_getreg(LPC43_ETH_MACMIIA) & ETH_MACMIIA_GB) == 0)
        {
          *value = (uint16_t)lpc43_getreg(LPC43_ETH_MACMIID);
          return OK;
        }
    }

  nerr("ERROR: MII transfer timed out: phydevaddr: %04x phyregaddr: %04x\n",
       phydevaddr, phyregaddr);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: lpc43_phywrite
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

static int lpc43_phywrite(uint16_t phydevaddr,
                          uint16_t phyregaddr, uint16_t value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MACMIIAR register,
   * preserving CSR Clock Range CR[2:0] bits
   */

  regval  = lpc43_getreg(LPC43_ETH_MACMIIA);
  regval &= ETH_MACMIIA_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the  ETH_MACMIIA_WR is set, indicating a write operation.
   */

  regval |= (phydevaddr << ETH_MACMIIA_PA_SHIFT) & ETH_MACMIIA_PA_MASK;
  regval |= (phyregaddr << ETH_MACMIIA_MR_SHIFT) & ETH_MACMIIA_MR_MASK;
  regval |= (ETH_MACMIIA_GB | ETH_MACMIIA_WR);

  /* Write the value into the MACIIDR register before setting the new
   * MACMIIAR register value.
   */

  lpc43_putreg(value, LPC43_ETH_MACMIID);
  lpc43_putreg(regval, LPC43_ETH_MACMIIA);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_WRITE_TIMEOUT; timeout++)
    {
      if ((lpc43_getreg(LPC43_ETH_MACMIIA) & ETH_MACMIIA_GB) == 0)
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
 * Function: lpc43_dm9161
 *
 * Description:
 *   Special workaround for the Davicom DM9161 PHY is required.  On power,
 *   up, the PHY is not usually configured correctly but will work after
 *   a powered-up reset.  This is really a workaround for some more
 *   fundamental issue with the PHY clocking initialization, but the
 *   root cause has not been studied (nor will it be with this workaround).
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ETH0_PHY_DM9161
static inline int lpc43_dm9161(struct lpc43_ethmac_s *priv)
{
  uint16_t phyval;
  int ret;

  /* Read the PHYID1 register;  A failure to read the PHY ID is one
   * indication that check if the DM9161 PHY CHIP is not ready.
   */

  ret = lpc43_phyread(CONFIG_LPC43_PHYADDR, MII_PHYID1, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read the PHY ID1: %d\n", ret);
      return ret;
    }

  /* If we failed to read the PHY ID1 register,
   * then reset the MCU to recover
   */

  else if (phyval == 0xffff)
    {
      up_systemreset();
    }

  ninfo("PHY ID1: 0x%04X\n", phyval);

  /* Now check the "DAVICOM Specified Configuration Register (DSCR)"(16) */

  ret = lpc43_phyread(CONFIG_LPC43_PHYADDR, 16, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read the PHY Register 0x10: %d\n", ret);
      return ret;
    }

  /* Bit 8 of the DSCR register is zero, then the DM9161 has not selected
   * RMII. If RMII is not selected, then reset the MCU to recover.
   */

  else if ((phyval & (1 << 8)) == 0)
    {
      up_systemreset();
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: lpc43_phyinit
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

static int lpc43_phyinit(struct lpc43_ethmac_s *priv)
{
  volatile uint32_t timeout;
  uint32_t regval;
  uint16_t phyval;
  int ret;

  /* Assume 10MBps and half duplex */

  priv->mbps100 = 0;
  priv->fduplex = 0;

  /* Setup up PHY clocking by setting the SR field in the MACMIIAR register */

  regval  = lpc43_getreg(LPC43_ETH_MACMIIA);
  regval &= ~ETH_MACMIIA_CR_MASK;
  regval |= ETH_MACMIIA_CR;
  lpc43_putreg(regval, LPC43_ETH_MACMIIA);

  /* Put the PHY in reset mode */

  ret = lpc43_phywrite(CONFIG_LPC43_PHYADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: Failed to reset the PHY: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_RESET_DELAY);

#ifdef CONFIG_LPC43_PHYINIT
  /* Perform any necessary, board-specific PHY initialization */

  ret = lpc43_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ETH0_PHY_DM9161
  /* Special workaround for the Davicom DM9161 PHY is required. */

  ret = lpc43_dm9161(priv);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Perform auto-negotiation if so configured */

#ifdef CONFIG_LPC43_AUTONEG
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = lpc43_phyread(CONFIG_LPC43_PHYADDR, MII_MSR, &phyval);
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

  ret = lpc43_phywrite(CONFIG_LPC43_PHYADDR, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = lpc43_phyread(CONFIG_LPC43_PHYADDR, MII_MSR, &phyval);
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

  ret = lpc43_phyread(CONFIG_LPC43_PHYADDR, CONFIG_LPC43_PHYSR, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  ninfo("PHYSR[%d]: %04x\n", CONFIG_LPC43_PHYSR, phyval);

#ifdef CONFIG_ETH0_PHY_LAN8720
  if ((phyval & (MII_MSR_100BASETXHALF | MII_MSR_100BASETXFULL)) != 0)
    {
      priv->mbps100 = 1;
    }

  if ((phyval & (MII_MSR_100BASETXFULL | MII_MSR_10BASETXFULL)) != 0)
    {
      priv->fduplex = 1;
    }

#else
  /* Different PHYs present speed and mode information in different ways.
   * IF This CONFIG_LPC43_PHYSR_ALTCONFIG is selected, this indicates that
   * the PHY represents speed and mode information are combined, for example,
   * with separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_LPC43_PHYSR_ALTCONFIG
  switch (phyval & CONFIG_LPC43_PHYSR_ALTMODE)
    {
      case CONFIG_LPC43_PHYSR_100FD:
        priv->fduplex = 1;
        priv->mbps100 = 1;
        break;

      case CONFIG_LPC43_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_LPC43_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      default:
      case CONFIG_LPC43_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;
    }

  /* Different PHYs present speed and mode information in different ways.
   * Some will present separate information for speed and mode (this is the
   * default). Those PHYs, for example, may provide a 10/100 Mbps indication
   * and a separate full/half duplex indication.
   */

#else
  if ((phyval & CONFIG_LPC43_PHYSR_MODE) == CONFIG_LPC43_PHYSR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_LPC43_PHYSR_SPEED) == CONFIG_LPC43_PHYSR_100MBPS)
    {
      priv->mbps100 = 1;
    }
#endif
#endif

#else /* Auto-negotiation not selected */

#ifdef CONFIG_LPC43_ETHFD
  priv->fduplex = 1;
#endif

#ifdef CONFIG_LPC43_ETH100MBPS
  priv->mbps100 = 1;
#endif

  phyval = 0;

  if (priv->mbps100)
    {
      phyval |= MII_MCR_FULLDPLX;
    }

  if (priv->fduplex)
    {
      phyval |= MII_MCR_SPEED100;
    }

  ret = lpc43_phywrite(CONFIG_LPC43_PHYADDR, MII_MCR, phyval);
  if (ret < 0)
    {
     nerr("ERROR: Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes
   * REVISIT:  Isn't this redundant?
   */

#ifdef CONFIG_LPC43_ETHFD
  priv->fduplex = 1;
#endif

#ifdef CONFIG_LPC43_ETH100MBPS
  priv->mbps100 = 1;
#endif
#endif

  ninfo("Duplex: %s Speed: %d MBps\n",
        priv->fduplex ? "FULL" : "HALF",
        priv->mbps100 ? 100 : 10);

  return OK;
}

/****************************************************************************
 * Name: lpc43_selectmii
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

#ifdef CONFIG_LPC43_MII
static inline void lpc43_selectmii(void)
{
  uint32_t regval;

  regval  = getreg32(LPC43_CREG6);
  regval &= ~SYSCFG_PMC_MII_RMII_SEL;
  putreg32(regval, LPC43_CREG6);
}
#endif

/****************************************************************************
 * Name: lpc43_selectrmii
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

static inline void lpc43_selectrmii(void)
{
  uint32_t regval;

  regval  = getreg32(LPC43_CREG6);
  regval |= CREG6_ETHMODE_RMII;
  putreg32(regval, LPC43_CREG6);
}

/****************************************************************************
 * Function: lpc43_ethgpioconfig
 *
 * Description:
 *  Configure GPIOs for the Ethernet interface.
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

static inline void lpc43_ethgpioconfig(struct lpc43_ethmac_s *priv)
{
  /* Configure GPIO pins to support Ethernet */

#if defined(CONFIG_LPC43_MII) || defined(CONFIG_LPC43_RMII)
  /* MDC and MDIO are common to both modes */

  lpc43_pin_config(PINCONF_ENET_MDC);
  lpc43_pin_config(PINCONF_ENET_MDIO);

  /* Set up the MII interface */

#if defined(CONFIG_LPC43_MII)
  /* Select the MII interface */

  lpc43_selectmii();

  /* MII interface pins (17):
   *
   * MII_TX_CLK, MII_TXD[3:0], MII_TX_EN, MII_RX_CLK, MII_RXD[3:0],
   * MII_RX_ER, MII_RX_DV, MII_CRS, MII_COL, MDC, MDIO
   */

  lpc43_pin_config(PINCONF_ENET_MII_COL);
  lpc43_pin_config(PINCONF_ENET_MII_CRS);
  lpc43_pin_config(PINCONF_ENET_MII_RXD0);
  lpc43_pin_config(PINCONF_ENET_MII_RXD1);
  lpc43_pin_config(PINCONF_ENET_MII_RXD2);
  lpc43_pin_config(PINCONF_ENET_MII_RXD3);
  lpc43_pin_config(PINCONF_ENET_MII_RX_CLK);
  lpc43_pin_config(PINCONF_ENET_MII_RX_DV);
  lpc43_pin_config(PINCONF_ENET_MII_RX_ER);
  lpc43_pin_config(PINCONF_ENET_MII_TXD0);
  lpc43_pin_config(PINCONF_ENET_MII_TXD1);
  lpc43_pin_config(PINCONF_ENET_MII_TXD2);
  lpc43_pin_config(PINCONF_ENET_MII_TXD3);
  lpc43_pin_config(PINCONF_ENET_MII_TX_CLK);
  lpc43_pin_config(PINCONF_ENET_MII_TX_EN);
  lpc43_pin_config(PINCONF_ENET_MII_TX_ER);

  /* Set up the RMII interface. */

#elif defined(CONFIG_LPC43_RMII)
  /* Select the RMII interface */

  lpc43_selectrmii();

  /* RMII interface pins (7):
   *
   * RMII_TXD[1:0], RMII_TX_EN, RMII_RXD[1:0], RMII_CRS_DV, MDC, MDIO,
   * RMII_REF_CLK
   */

  lpc43_pin_config(PINCONF_ENET_RX_DV);
  lpc43_pin_config(PINCONF_ENET_REF_CLK);
  lpc43_pin_config(PINCONF_ENET_RXD0);
  lpc43_pin_config(PINCONF_ENET_RXD1);
  lpc43_pin_config(PINCONF_ENET_TXD0);
  lpc43_pin_config(PINCONF_ENET_TXD1);
  lpc43_pin_config(PINCONF_ENET_TX_EN);

#ifdef PINCONF_ENET_RESET
  lpc43_pin_config(PINCONF_ENET_RESET);
  lpc43_gpio_config(GPIO_ENET_RESET);
  lpc43_gpio_write(GPIO_ENET_RESET, 0);
  up_mdelay(5);
  lpc43_gpio_write(GPIO_ENET_RESET, 1);
  up_mdelay(5);
#endif
#endif
#endif
}

/****************************************************************************
 * Function: lpc43_ethreset
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

static void lpc43_ethreset(struct lpc43_ethmac_s *priv)
{
  uint32_t regval;

  lpc43_putreg(CCU_CLK_CFG_RUN | CCU_CLK_CFG_AUTO | CCU_CLK_CFG_WAKEUP,
               LPC43_CCU1_M4_ETHERNET_CFG);

  /* Reset the Ethernet  */

  lpc43_putreg(RGU_CTRL0_ETHERNET_RST, LPC43_RGU_CTRL0);

  regval  = lpc43_getreg(LPC43_ETH_DMABMODE);
  regval |= ETH_DMABMODE_SWR;
  lpc43_putreg(regval, LPC43_ETH_DMABMODE);

  /* Wait for software reset to complete. The SR bit is cleared automatically
   * after the reset operation has completed in all core clock domains.
   */

  while ((lpc43_getreg(LPC43_ETH_DMABMODE) & ETH_DMABMODE_SWR) != 0);
}

/****************************************************************************
 * Function: lpc43_macconfig
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

static int lpc43_macconfig(struct lpc43_ethmac_s *priv)
{
  uint32_t regval;

  /* Set up the MACCR register */

  regval  = lpc43_getreg(LPC43_ETH_MACCR);
  regval &= ~MACCR_CLEAR_BITS;
  regval |= MACCR_SET_BITS;

  if (priv->fduplex)
    {
      /* Set the DM bit for full duplex support */

      regval |= ETH_MACCFG_DM;
    }

  if (priv->mbps100)
    {
      /* Set the FES bit for 100Mbps fast ethernet support */

      regval |= ETH_MACCFG_FES;
    }

  lpc43_putreg(regval, LPC43_ETH_MACCR);

  /* Set up the MACFFR register */

  regval  = lpc43_getreg(LPC43_ETH_MACFFLT);
  regval &= ~MACFFR_CLEAR_BITS;
  regval |= MACFFR_SET_BITS;
  lpc43_putreg(regval, LPC43_ETH_MACFFLT);

  /* Set up the MACHTHR and MACHTLR registers */

  lpc43_putreg(0, LPC43_ETH_MACHTHI);
  lpc43_putreg(0, LPC43_ETH_MACHTLO);

  /* Setup up the MACFCR register */

  regval  = lpc43_getreg(LPC43_ETH_MACFC);
  regval &= ~MACFCR_CLEAR_MASK;
  regval |= MACFCR_SET_MASK;
  lpc43_putreg(regval, LPC43_ETH_MACFC);

  /* Setup up the MACVLANTR register */

  lpc43_putreg(0, LPC43_ETH_MACVLANT);

  /* DMA Configuration */

  /* Set up the DMAOMR register */

  regval  = lpc43_getreg(LPC43_ETH_DMAOPMODE);
  regval &= ~DMAOMR_CLEAR_MASK;
  regval |= DMAOMR_SET_MASK;
  lpc43_putreg(regval, LPC43_ETH_DMAOPMODE);

  /* Set up the DMABMR register */

  regval  = lpc43_getreg(LPC43_ETH_DMABMODE);
  regval &= ~DMABMR_CLEAR_MASK;
  regval |= DMABMR_SET_MASK;
  lpc43_putreg(regval, LPC43_ETH_DMABMODE);

  return OK;
}

/****************************************************************************
 * Function: lpc43_macaddress
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

static void lpc43_macaddress(struct lpc43_ethmac_s *priv)
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
  lpc43_putreg(regval, LPC43_ETH_MACA0HI);

  /* Set the MAC address low register */

  regval = ((uint32_t)dev->d_mac.ether.ether_addr_octet[3] << 24) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[2] << 16) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[1] <<  8) |
            (uint32_t)dev->d_mac.ether.ether_addr_octet[0];
  lpc43_putreg(regval, LPC43_ETH_MACA0LO);
}

/****************************************************************************
 * Function: lpc43_ipv6multicast
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
static void lpc43_ipv6multicast(struct lpc43_ethmac_s *priv)
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

  lpc43_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  lpc43_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  lpc43_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: lpc43_macenable
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

static int lpc43_macenable(struct lpc43_ethmac_s *priv)
{
  uint32_t regval;

  /* Set the MAC address */

  lpc43_macaddress(priv);

#ifdef CONFIG_NET_ICMPv6
  /* Set up the IPv6 multicast address */

  lpc43_ipv6multicast(priv);
#endif

  /* Enable transmit state machine of the MAC for transmission on the MII */

  regval  = lpc43_getreg(LPC43_ETH_MACCR);
  regval |= ETH_MACCFG_TE;
  lpc43_putreg(regval, LPC43_ETH_MACCR);

  /* Flush Transmit FIFO */

  regval  = lpc43_getreg(LPC43_ETH_DMAOPMODE);
  regval |= ETH_DMAOPMODE_FTF;
  lpc43_putreg(regval, LPC43_ETH_DMAOPMODE);

  /* Enable receive state machine of the MAC for reception from the MII */

  /* Enables or disables the MAC reception. */

  regval  = lpc43_getreg(LPC43_ETH_MACCR);
  regval |= ETH_MACCFG_RE;
  lpc43_putreg(regval, LPC43_ETH_MACCR);

  /* Start DMA transmission */

  regval  = lpc43_getreg(LPC43_ETH_DMAOPMODE);
  regval |= ETH_DMAOPMODE_ST;
  lpc43_putreg(regval, LPC43_ETH_DMAOPMODE);

  /* Start DMA reception */

  regval  = lpc43_getreg(LPC43_ETH_DMAOPMODE);
  regval |= ETH_DMAOPMODE_SR;
  lpc43_putreg(regval, LPC43_ETH_DMAOPMODE);

  /* Enable Ethernet DMA interrupts.
   *
   * The LPC43 hardware supports two interrupts: (1) one dedicated to normal
   * Ethernet operations and the other, used only for the Ethernet wakeup
   * event.  The wake-up interrupt is not used by this driver.
   *
   * The first Ethernet vector is reserved for interrupts generated by the
   * MAC and the DMA. The MAC provides PMT and time stamp trigger interrupts,
   * neither of which are used by this driver.
   */

  lpc43_putreg(ETH_MACIM_ALLINTS, LPC43_ETH_MACIM);

  /* Ethernet DMA supports two classes of interrupts: Normal interrupt
   * summary (NIS) and Abnormal interrupt summary (AIS) with a variety
   * individual normal and abnormal interrupting events.  Here only
   * the normal receive event is enabled (unless DEBUG is enabled).  Transmit
   * events will only be enabled when a transmit interrupt is expected.
   */

  lpc43_putreg((ETH_DMAINT_RECV_ENABLE | ETH_DMAINT_ERROR_ENABLE),
               LPC43_ETH_DMAINTEN);
  return OK;
}

/****************************************************************************
 * Function: lpc43_ethconfig
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

static int lpc43_ethconfig(struct lpc43_ethmac_s *priv)
{
  int ret;

  /* NOTE: The Ethernet clocks were initialized early in the boot-up
   * sequence in lpc43_rcc.c.
   */

  /* Reset the Ethernet block */

  ninfo("Reset the Ethernet block\n");
  lpc43_ethreset(priv);

  /* Initialize the PHY */

  ninfo("Initialize the PHY\n");
  ret = lpc43_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Perform a software reset by setting the SR bit in the DMABMR register.
   * This Resets all MAC subsystem internal registers and logic.  After this
   * reset all the registers holds their reset values.
   */

  /* Initialize the MAC and DMA */

  ninfo("Initialize the MAC and DMA\n");
  ret = lpc43_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the free buffer list */

  lpc43_initbuffer(priv);

  /* Initialize TX Descriptors list: Chain Mode */

  lpc43_txdescinit(priv);

  /* Initialize RX Descriptors list: Chain Mode  */

  lpc43_rxdescinit(priv);

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");
  return lpc43_macenable(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: lpc43_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline int lpc43_ethinitialize(void)
{
  struct lpc43_ethmac_s *priv;

  /* Get the interface structure associated with this interface number. */

  priv = &g_lpc43ethmac;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lpc43_ethmac_s));
  priv->dev.d_ifup    = lpc43_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = lpc43_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = lpc43_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = lpc43_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = lpc43_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = lpc43_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = &g_lpc43ethmac; /* Used to recover private state from dev */

  /* Configure GPIO pins to support Ethernet */

  lpc43_ethgpioconfig(priv);

  /* Attach the IRQ to the driver */

  if (irq_attach(LPC43M4_IRQ_ETHERNET, lpc43_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Put the interface in the down state. */

  lpc43_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c.
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

#ifndef CONFIG_NETDEV_LATEINIT
void arm_netinitialize(void)
{
  lpc43_ethinitialize();
}
#endif

#endif /* CONFIG_NET && CONFIG_LPC43_ETHERNET */
