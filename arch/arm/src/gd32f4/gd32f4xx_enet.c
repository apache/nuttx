/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_enet.c
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

#define GD32_ENET_NUM 1

#if defined(CONFIG_NET) && defined(CONFIG_GD32F4_ENETMAC)

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netdev.h>

#if defined(CONFIG_NET_PKT)
#include <nuttx/net/pkt.h>
#endif /* CONFIG_NET_PKT */

#include "arm_internal.h"

#include "chip.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_syscfg.h"
#include "gd32f4xx_enet.h"

#include <arch/board/board.h>

/* GD32_NETHERNET determines the number of physical interfaces
 * that will be supported.
 */

#if GD32_NETHERNET > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* See boards/arm/gd32/gd3240g-eval/README.txt for an explanation of the
 * configuration settings.
 */

#if GD32_NETHERNET > 1
#  error "Logic to support multiple Ethernet interfaces is incomplete"
#endif

/* Work queue support is required. */

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

#define ENET_WORK LPWORK

#if !defined(CONFIG_GD32F4_SYSCFG)
#  error "CONFIG_GD32F4_SYSCFG must be defined in the NUTTX config.h"
#endif

#ifndef CONFIG_GD32F4_PHY_ADDR
#  error "CONFIG_GD32F4_PHY_ADDR must be defined in the NUTTX config.h"
#endif

#if !defined(CONFIG_GD32F4_MII) && !defined(CONFIG_GD32F4_RMII)
#  warning "Neither CONFIG_GD32F4_MII nor CONFIG_GD32F4_RMII defined"
#endif

#if defined(CONFIG_GD32F4_MII) && defined(CONFIG_GD32F4_RMII)
#  error "Both CONFIG_GD32F4_MII and CONFIG_GD32F4_RMII defined"
#endif

#ifdef CONFIG_GD32F4_MII
#  if !defined(CONFIG_GD32F4_MII_CKOUT0) && !defined(CONFIG_GD32F4_MII_CKOUT1) && !defined(CONFIG_GD32F4_MII_EXTCLK)
#    warning "Neither CONFIG_GD32F4_MII_CKOUT0, CONFIG_GD32F4_MII_CKOUT1, nor CONFIG_GD32F4_MII_EXTCLK defined"
#  endif
#  if defined(CONFIG_GD32F4_MII_CKOUT0) && defined(CONFIG_GD32F4_MII_CKOUT1)
#    error "Both CONFIG_GD32F4_MII_CKOUT0 and CONFIG_GD32F4_MII_CKOUT1 defined"
#  endif
#endif

#ifdef CONFIG_GD32F4_RMII
#  if !defined(CONFIG_GD32F4_RMII_CKOUT0) && !defined(CONFIG_GD32F4_RMII_CKOUT1) && !defined(CONFIG_GD32F4_RMII_EXTCLK)
#    warning "Neither CONFIG_GD32F4_RMII_CKOUT0, CONFIG_GD32F4_RMII_CKOUT1, nor CONFIG_GD32F4_RMII_EXTCLK defined"
#  endif
#  if defined(CONFIG_GD32F4_RMII_CKOUT0) && defined(CONFIG_GD32F4_RMII_CKOUT1)
#    error "Both CONFIG_GD32F4_RMII_CKOUT0 and CONFIG_GD32F4_RMII_CKOUT1 defined"
#  endif
#endif

#ifdef CONFIG_GD32F4_AUTO_NEGOTIATION
#  ifndef CONFIG_GD32F4_PHY_SR
#    error "CONFIG_GD32F4_PHY_SR must be defined in the NUTTX config.h"
#  endif
#  ifdef CONFIG_GD32F4_PHY_SR_ALTCONFIG
#    ifndef CONFIG_GD32F4_PHY_SR_ALTMODE
#      error "CONFIG_GD32F4_PHY_SR_ALTMODE must be defined in the NUTTX config.h"
#    endif
#    ifndef CONFIG_GD32F4_PHY_SR_10HD
#      error "CONFIG_GD32F4_PHY_SR_10HD must be defined in the NUTTX config.h"
#    endif
#    ifndef CONFIG_GD32F4_PHY_SR_100HD
#      error "CONFIG_GD32F4_PHY_SR_100HD must be defined in the NUTTX config.h"
#    endif
#    ifndef CONFIG_GD32F4_PHY_SR_10FD
#      error "CONFIG_GD32F4_PHY_SR_10FD must be defined in the NUTTX config.h"
#    endif
#    ifndef CONFIG_GD32F4_PHY_SR_100FD
#      error "CONFIG_GD32F4_PHY_SR_100FD must be defined in the NUTTX config.h"
#    endif
#  else
#    ifndef CONFIG_GD32F4_PHY_SR_SPEED
#      error "CONFIG_GD32F4_PHY_SR_SPEED must be defined in the NUTTX config.h"
#    endif
#    ifndef CONFIG_GD32F4_PHY_SR_100M
#      error "CONFIG_GD32F4_PHY_SR_100M must be defined in the NUTTX config.h"
#    endif
#    ifndef CONFIG_GD32F4_PHY_SR_MODE
#      error "CONFIG_GD32F4_PHY_SR_MODE must be defined in the NUTTX config.h"
#    endif
#    ifndef CONFIG_GD32F4_PHY_SR_FULLDUPLEX
#      error "CONFIG_GD32F4_PHY_SR_FULLDUPLEX must be defined in the NUTTX config.h"
#    endif
#  endif
#endif

/* These definitions are used to enable the PHY interrupts */

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
#  if defined( CONFIG_ETH0_PHY_AM79C874)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KS8721)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ8041)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ8051)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ8061)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ8081)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_KSZ90x1)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_DP83848C)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_LAN8720)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_LAN8740)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_LAN8740A)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_LAN8742A)
#    error missing logic
#  elif defined( CONFIG_ETH0_PHY_DM9161)
#    error missing logic
#  else
#    error "unknown PHY"
#  endif
#endif

#ifdef CONFIG_NET_ICMPv6
#warning "CONFIG_NET_ICMPv6 is not yet supported"
#endif

#ifdef CONFIG_NET_IPv6
#  warning "CONFIG_NET_IPv6 is not yet supported"
#endif

#ifdef CONFIG_GD32F4_ENET_PTP
#  warning "CONFIG_GD32F4_ENET_PTP is not yet supported"
#endif

/* This driver does not use enhanced descriptors.  Enhanced descriptors must
 * be used, however, if time stamping or and/or IPv4 checksum offload is
 * supported.
 */

#undef CONFIG_GD32F4_ENET_ENHANCEDDESC
#define CONFIG_GD32_ENET_HWCHECKSUM

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, or 16 bytes (depending on buswidth).  We
 * will use the 16-byte alignment in all cases.
 */

#define OPTIMAL_ENET_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 4 + 15) & ~15)

#ifndef CONFIG_GD32F4_ENET_BUFSIZE
#  define CONFIG_GD32F4_ENET_BUFSIZE OPTIMAL_ENET_BUFSIZE
#endif

#if CONFIG_GD32F4_ENET_BUFSIZE > ENET_TDES1_TB1S_MASK
#  error "CONFIG_GD32F4_ENET_BUFSIZE is too large"
#endif

#if (CONFIG_GD32F4_ENET_BUFSIZE & 15) != 0
#  error "CONFIG_GD32F4_ENET_BUFSIZE must be aligned"
#endif

#if CONFIG_GD32F4_ENET_BUFSIZE != OPTIMAL_ENET_BUFSIZE
#  warning "You using an incomplete/untested configuration"
#endif

#ifndef CONFIG_GD32F4_ENET_NRXDESC
#  define CONFIG_GD32F4_ENET_NRXDESC 8
#endif
#ifndef CONFIG_GD32F4_ENET_NTXDESC
#  define CONFIG_GD32F4_ENET_NTXDESC 4
#endif

/* We need at least one more free buffer than transmit buffers */

#define GD32_ENET_NFREEBUFFERS (CONFIG_GD32F4_ENET_NTXDESC + 1)

/* Clock configure **********************************************************/

/* Set ENET_MAC_PHY_CTL CLR[2:0] bits depending on HCLK setting */

#if GD32_HCLK_FREQUENCY >= 20000000 && GD32_HCLK_FREQUENCY < 35000000
#  define ENET_MAC_PHY_CTL_CLR ENET_MDC_HCLK_DIV16
#elif GD32_HCLK_FREQUENCY >= 35000000 && GD32_HCLK_FREQUENCY < 60000000
#  define ENET_MAC_PHY_CTL_CLR ENET_MDC_HCLK_DIV26
#elif GD32_HCLK_FREQUENCY >= 60000000 && GD32_HCLK_FREQUENCY < 100000000
#  define ENET_MAC_PHY_CTL_CLR ENET_MDC_HCLK_DIV42
#elif GD32_HCLK_FREQUENCY >= 100000000 && GD32_HCLK_FREQUENCY < 150000000
#  define ENET_MAC_PHY_CTL_CLR ENET_MDC_HCLK_DIV62
#elif GD32_HCLK_FREQUENCY >= 150000000 && GD32_HCLK_FREQUENCY <= 240000000
#  define ENET_MAC_PHY_CTL_CLR ENET_MDC_HCLK_DIV102
#else
#  error "GD32_HCLK_FREQUENCY not supportable"
#endif

/* Timing *******************************************************************/

/* TX poll delay = 1 seconds.
 * CLK_TCK is the number of clock ticks per second
 */

#define GD32_WDDELAY           (1 * CLK_TCK)

/* TX timeout = 1 minute */

#define GD32_TXTIMEOUT         (60 * CLK_TCK)

/* PHY reset/configuration delays in milliseconds */

#define PHY_RESET_DELAY        (100)
#define PHY_CONFIG_DELAY       (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT       (0x0004FFFF)
#define PHY_WRITE_TIMEOUT      (0x0004FFFF)
#define PHY_RETRY_TIMEOUT      (0x0004FFFF)

/* Register values **********************************************************/

/* Clear the MAC_CFG bits that will be setup during MAC initialization
 * (or that are cleared unconditionally).  Per the reference manual,
 * all reserved bits must be retained at their reset value.
 *
 * ENET_MAC_CFG_REN    Bit 2:      receiver enable
 * ENET_MAC_CFG_TEN    Bit 3:      transmitter enable
 * ENET_MAC_CFG_DFC    Bit 4:      defferal check
 * ENET_MAC_CFG_BOL    Bits 5-6:   back-off limit
 * ENET_MAC_CFG_APCD   Bit 7:      automatic pad/CRC drop
 * ENET_MAC_CFG_RTD    Bit 9:      retry disable
 * ENET_MAC_CFG_IPFCO  Bit 10:     IP frame checksum offload
 * ENET_MAC_CFG_DPM    Bit 11:     duplex mode
 * ENET_MAC_CFG_LBM    Bit 12:     loopback mode
 * ENET_MAC_CFG_ROD    Bit 13:     receive own disable
 * ENET_MAC_CFG_SPD    Bit 14:     fast ethernet speed
 * ENET_MAC_CFG_CSD    Bit 16:     carrier sense disable
 * ENET_MAC_CFG_IGBS   Bits 17-19: inter-frame gap bit selection
 * ENET_MAC_CFG_JBD    Bit 22:     jabber disable
 * ENET_MAC_CFG_WDD    Bit 23:     watchdog disable
 * ENET_MAC_CFG_TFCD   Bits 25:    type frame CRC dropping
 */

#define ENET_MAC_CFG_CLEAR_BITS (ENET_MAC_CFG_REN | \
                                 ENET_MAC_CFG_TEN | \
                                 ENET_MAC_CFG_DFC | \
                                 ENET_MAC_CFG_BOL_MASK | \
                                 ENET_MAC_CFG_APCD | \
                                 ENET_MAC_CFG_RTD | \
                                 ENET_MAC_CFG_IPFCO | \
                                 ENET_MAC_CFG_DPM | \
                                 ENET_MAC_CFG_LBM | \
                                 ENET_MAC_CFG_ROD | \
                                 ENET_MAC_CFG_SPD | \
                                 ENET_MAC_CFG_CSD | \
                                 ENET_MAC_CFG_IGBS_MASK | \
                                 ENET_MAC_CFG_JBD | \
                                 ENET_MAC_CFG_WDD | \
                                 ENET_MAC_CFG_TFCD)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ENET_MAC_CFG_REN   receiver enable               0 (disabled)
 * ENET_MAC_CFG_TEN   transmitter enable            0 (disabled)
 * ENET_MAC_CFG_DFC   deferral check                0 (disabled)
 * ENET_MAC_CFG_BOL   back-off limit                0 (10)
 * ENET_MAC_CFG_APCD  automatic pad/CRC drop        0 (disabled)
 * ENET_MAC_CFG_RTD   retry disable                 1 (disabled)
 * ENET_MAC_CFG_IPFCO IP frame checksum offload     Depends on
 *                                                CONFIG_GD32_ENET_HWCHECKSUM
 * ENET_MAC_CFG_LBM   loopback mode                 0 (disabled)
 * ENET_MAC_CFG_ROD   receive own disable           0 (enabled)
 * ENET_MAC_CFG_CSD   carrier sense disable         0 (enabled)
 * ENET_MAC_CFG_IGBS  inter-frame gap bit selection 0 (96 bits)
 * ENET_MAC_CFG_JBD   jabber disable                0 (enabled)
 * ENET_MAC_CFG_WDD   watchdog disable              0 (enabled)
 * ENET_MAC_CFG_TFCD  type frame CRC dropping       0 (disabled)
 *
 * The following are set conditioinally based on mode and speed.
 *
 * ENET_MAC_CFG_DPM   duplex mode                   Depends on priv->fduplex
 * ENET_MAC_CFG_SPD   fast Ethernet speed           Depends on priv->mbps100
 */

#ifdef CONFIG_GD32_ENET_HWCHECKSUM
  #define ENET_MAC_CFG_SET_BITS (ENET_BACKOFFLIMIT_10 | \
                                 ENET_MAC_CFG_RTD | \
                                 ENET_MAC_CFG_IPFCO | \
                                 ENET_MAC_CFG_IGBS(96))
#else
  #define ENET_MAC_CFG_SET_BITS (ENET_BACKOFFLIMIT_10 | \
                                 ENET_MAC_CFG_RTD | \
                                 ENET_MAC_CFG_IGBS(96))
#endif

/* Clear the MAC_FRMF bits that will be setup during MAC initialization
 * (or that are cleared unconditionally).  Per the reference manual,
 * all reserved bits must be retained at their reset value.
 *
 * ENET_MAC_FRMF_PM      Bit 0:     promiscuous mode
 * ENET_MAC_FRMF_HUF     Bit 1:     hash unicast filter
 * ENET_MAC_FRMF_HMF     Bit 2:     hash multicast filter
 * ENET_MAC_FRMF_DAIFLT  Bit 3:     destination address inverse filtering
 *                                  enable
 * ENET_MAC_FRMF_MFD     Bit 4:     multicast filter disable
 * ENET_MAC_FRMF_BFRMD   Bit 5:     broadcast frame disable
 * ENET_MAC_FRMF_PCFRM   Bits 6-7:  pass control frames
 * ENET_MAC_FRMF_SAIFLT  Bit 8:     source address inverse filtering
 * ENET_MAC_FRMF_SAFLT   Bit 9:     source address filter
 * ENET_MAC_FRMF_HPFLT   Bit 10:    hash or perfect filter
 * ENET_MAC_FRMF_FAR     Bit 31:    receive all frames
 */

#define ENET_MAC_FRMF_CLEAR_BITS (ENET_MAC_FRMF_PM | \
                                  ENET_MAC_FRMF_HUF | \
                                  ENET_MAC_FRMF_HMF | \
                                  ENET_MAC_FRMF_DAIFLT | \
                                  ENET_MAC_FRMF_MFD | \
                                  ENET_MAC_FRMF_BFRMD | \
                                  ENET_MAC_FRMF_PCFRM_MASK | \
                                  ENET_MAC_FRMF_SAIFLT | \
                                  ENET_MAC_FRMF_SAFLT | \
                                  ENET_MAC_FRMF_HPFLT | \
                                  ENET_MAC_FRMF_FAR)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ENET_MAC_FRMF_HUF     hash unicast filter       0 (perfect dest filtering)
 * ENET_MAC_FRMF_HMF     hash multicast filter     0 (perfect dest filtering)
 * ENET_MAC_FRMF_DAIFLT  destination address       0 (normal)
 *                       inverse filtering
 *                       enable inverse filtering
 * ENET_MAC_FRMF_MFD     multicast filter disable  0 (depends on
 *                                                    ENET_MAC_FRMF_HMF bit)
 * ENET_MAC_FRMF_BFRMD   broadcast frame disable   0 (enabled)
 * ENET_MAC_FRMF_PCFRM   pass control frames       1 (block all but PAUSE)
 * ENET_MAC_FRMF_SAIFLT  source address inverse    0 (not used)
 *                       filtering
 * ENET_MAC_FRMF_SAFLT   source address filter     0 (disabled)
 * ENET_MAC_FRMF_HPFLT   hash or perfect filter    0 (Only matching frames
 *                                                    passed)
 * ENET_MAC_FRMF_FAR     receive all frames        0 (disabled)
 */

#ifdef CONFIG_NET_PROMISCUOUS
  #define ENET_MAC_FRMF_SET_BITS (ENET_PCFRM_PREVENT_PAUSEFRAME | ENET_MAC_FRMF_PM)
#else
  #define ENET_MAC_FRMF_SET_BITS (ENET_PCFRM_PREVENT_PAUSEFRAME)
#endif

/* Clear the MAC_FCTL bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ENET_MAC_FCTL_FLCBBKPA  Bit 0:      flow control busy(in full duplex mode)
 *                                     /backpressure activate(in half duplex
 *                                     mode)
 * ENET_MAC_FCTL_TFCEN     Bit 1:      transmit flow control enable
 * ENET_MAC_FCTL_RFCEN     Bit 2:      receive flow control enable
 * ENET_MAC_FCTL_UPFDT     Bit 3:      unicast pause frame detect
 * ENET_MAC_FCTL_PLTS      Bits 4-5:   pause low threshold
 * ENET_MAC_FCTL_DZQP      Bit 7:      disable zero-quanta pause
 * ENET_MAC_FCTL_PTM       Bits 16-31: pause time
 */

#define ENET_MAC_FCTL_CLEAR_MASK (ENET_MAC_FCTL_FLCBBKPA | \
                                  ENET_MAC_FCTL_TFCEN | \
                                  ENET_MAC_FCTL_RFCEN | \
                                  ENET_MAC_FCTL_UPFDT | \
                                  ENET_MAC_FCTL_PLTS_MASK | \
                                  ENET_MAC_FCTL_DZQP | \
                                  ENET_MAC_FCTL_PTM_MASK)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ENET_MAC_FCTL_FLCBBKPA  flow control busy(in full       0 (no pause
 *                         duplex mode)/backpressure          control frame)
 *                         activate(in half duplex mode)
 * ENET_MAC_FCTL_TFCEN    transmit flow control enable     0 (disabled)
 * ENET_MAC_FCTL_RFCEN    receive flow control enable      0 (disabled)
 * ENET_MAC_FCTL_UPFDT    unicast pause frame detect       0 (disabled)
 * ENET_MAC_FCTL_PLTS     pause low threshold              0 (pause time - 4)
 * ENET_MAC_FCTL_DZQP     disable zero-quanta pause        1 (disabled)
 * ENET_MAC_FCTL_PTM      pause time                       0
 */

#define ENET_MAC_FCTL_SET_MASK (ENET_PAUSETIME_MINUS4 | ENET_MAC_FCTL_DZQP)

/* Clear the DMA_CTL bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ENET_DMA_CTL_SRE      Bit 1:       start/stop receive enable
 * ENET_DMA_CTL_OSF      Bit 2:       operate on second frame
 * ENET_DMA_CTL_RTHC     Bits 3-4:    receive threshold control
 * ENET_DMA_CTL_FUF      Bit 6:       forward undersized good frames
 * ENET_DMA_CTL_FERF     Bit 7:       forward error frames
 * ENET_DMA_CTL_STE      Bit 13:      start/stop transmission enable
 * ENET_DMA_CTL_TTHC     Bits 14-16:  transmit threshold control
 * ENET_DMA_CTL_FTF      Bit 20:      flush transmit FIFO
 * ENET_DMA_CTL_TSFD     Bit 21:      transmit store-and-forward
 * ENET_DMA_CTL_DAFRF    Bit 24:      disable flushing of received frames
 * ENET_DMA_CTL_RSFD     Bit 25:      receive store-and-forward
 * ENET_DMA_CTL_DTCERFD  Bit 26:      dropping of TCP/IP checksum error
 *                                    frames disable
 */

#define ENET_DMA_CTL_CLEAR_MASK (ENET_DMA_CTL_SRE | \
                                 ENET_DMA_CTL_OSF | \
                                 ENET_DMA_CTL_RTHC_MASK | \
                                 ENET_DMA_CTL_FUF | \
                                 ENET_DMA_CTL_FERF | \
                                 ENET_DMA_CTL_STE | \
                                 ENET_DMA_CTL_TTHC_MASK | \
                                 ENET_DMA_CTL_FTF | \
                                 ENET_DMA_CTL_TSFD | \
                                 ENET_DMA_CTL_DAFRF | \
                                 ENET_DMA_CTL_RSFD | \
                                 ENET_DMA_CTL_DTCERFD)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ENET_DMA_CTL_SRE      start/stop receive       0 (disabled)
 *                       enable
 * ENET_DMA_CTL_OSF      operate on second frame  0 (disabled)
 * ENET_DMA_CTL_RTHC     receive threshold        0 (64 bytes)
 *                       control
 * ENET_DMA_CTL_FUF      forward undersized       0 (disabled)
 *                       good frames
 * ENET_DMA_CTL_FERF     forward error frames     0 (disabled)
 * ENET_DMA_CTL_STE      start/stop trans-        0 (disabled)
 *                       -mission enable
 * ENET_DMA_CTL_TTHC     transmit threshold       0 (64 bytes)
 *                       control
 * ENET_DMA_CTL_FTF      flush transmit FIFO      0 (disabled)
 * ENET_DMA_CTL_TSFD     transmit store-and       depends on
 *                       -forward                 CONFIG_GD32_ENET_HWCHECKSUM
 * ENET_DMA_CTL_DAFRF    disable flushing of      0 (enabled)
 *                       received frames
 * ENET_DMA_CTL_RSFD     receive store-and        epends on
 *                       -forward                 CONFIG_GD32_ENET_HWCHECKSUM
 * ENET_DMA_CTL_DTCERFD  dropping of TCP/IP       depends on
 *                       checksum error           CONFIG_GD32_ENET_HWCHECKSUM
 *                       frames disable
 *
 * When the checksum offload feature is enabled, we need to enable the Store
 * and Forward mode: the store and forward guarantee that a whole frame is
 * stored in the FIFO, so the MAC can insert/verify the checksum, if the
 * checksum is OK the DMA can handle the frame otherwise the frame is dropped
 */

#ifdef CONFIG_GD32_ENET_HWCHECKSUM
  #define ENET_DMA_CTL_SET_MASK (ENET_RX_THRESHOLD_64BYTES | \
                                 ENET_TX_THRESHOLD_64BYTES | \
                                 ENET_DMA_CTL_TSFD | \
                                 ENET_DMA_CTL_RSFD)
#else
  #define DMAOMR_SET_MASK (ENET_RX_THRESHOLD_64BYTES | \
                           ENET_TX_THRESHOLD_64BYTES | \
                           ENET_DMA_CTL_DTCERFD)
#endif

/* Clear the DMA_BCTL bits that will be setup during MAC initialization (or
 * that are cleared unconditionally). Per the reference manual, all reserved
 * bits must be retained at their reset value.
 *
 * ENET_DMA_BCTL_SWR   Bit 0:       software reset
 * ENET_DMA_BCTL_DAB   Bit 1:       DMA arbitration
 * ENET_DMA_BCTL_DPSL  Bits 2-6:    descriptor skip length
 * ENET_DMA_BCTL_DFM   Bit 7:       descriptor format mode
 * ENET_DMA_BCTL_PGBL  Bits 8-13:   programmable burst length
 * ENET_DMA_BCTL_RTPR  Bits 14-15:  RX TX priority ratio
 * ENET_DMA_BCTL_FB    Bit 16:      fixed burst
 * ENET_DMA_BCTL_RXDP  Bits 17-22:  RX DMA PGBL
 * ENET_DMA_BCTL_UIP   Bit 23:      use independent PGBL
 * ENET_DMA_BCTL_FPBL  Bit 24:      four times PGBL mode
 * ENET_DMA_BCTL_AA    Bit 25:      address-aligned
 * ENET_DMA_BCTL_MB    Bit 26:      mixed burst
 */

#define ENET_DMA_BCTL_CLEAR_MASK (ENET_DMA_BCTL_SWR | \
                                  ENET_DMA_BCTL_DAB | \
                                  ENET_DMA_BCTL_DPSL_MASK | \
                                  ENET_DMA_BCTL_DFM | \
                                  ENET_DMA_BCTL_PGBL_MASK | \
                                  ENET_DMA_BCTL_RTPR_MASK | \
                                  ENET_DMA_BCTL_FB | \
                                  ENET_DMA_BCTL_RXDP_MASK | \
                                  ENET_DMA_BCTL_UIP | \
                                  ENET_DMA_BCTL_FPBL | \
                                  ENET_DMA_BCTL_AA | \
                                  ENET_DMA_BCTL_MB)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ENET_DMA_BCTL_SWR   software reset         0 (no reset)
 * ENET_DMA_BCTL_DAB   DMA arbitration        0 (round robin)
 * ENET_DMA_BCTL_DPSL  descriptor skip        0
 *                     length
 * ENET_DMA_BCTL_DFM   descriptor format      depends on
 *                     mode                   CONFIG_GD32F4_ENET_ENHANCEDDESC
 * ENET_DMA_BCTL_PGBL  programmable burst     32 beats
 *                     length
 * ENET_DMA_BCTL_RTPR  RX TX priority ratio   2:1
 * ENET_DMA_BCTL_FB    fixed burst            1 (enabled)
 * ENET_DMA_BCTL_RXDP  RX DMA PGBL            32 beats
 * ENET_DMA_BCTL_UIP   use independent PGBL   1 (enabled)
 * ENET_DMA_BCTL_FPBL  four times PGBL mode   0 (disabled)
 * ENET_DMA_BCTL_AA    address-aligned        1 (enabled)
 * ENET_DMA_BCTL_MB    mixed burst            0 (disabled)
 */

#ifdef CONFIG_GD32F4_ENET_ENHANCEDDESC
  #define ENET_DMA_BCTL_SET_MASK (ENET_DMA_BCTL_DPSL(0) | \
                                  ENET_DMA_BCTL_PGBL(32) | \
                                  ENET_DMA_BCTL_DFM | \
                                  ENET_ARBITRATION_RXTX_2_1 | \
                                  ENET_DMA_BCTL_FB | \
                                  ENET_DMA_BCTL_RXDP(32) | \
                                  ENET_DMA_BCTL_UIP | \
                                  ENET_DMA_BCTL_AA)
#else
  #define ENET_DMA_BCTL_SET_MASK (ENET_DMA_BCTL_DPSL(0) | \
                                  ENET_DMA_BCTL_PGBL(32) | \
                                  ENET_ARBITRATION_RXTX_2_1 | \
                                  ENET_DMA_BCTL_FB | \
                                  ENET_DMA_BCTL_RXDP(32) | \
                                  ENET_DMA_BCTL_UIP | \
                                  ENET_DMA_BCTL_AA)
#endif

/* Interrupt bit sets *******************************************************/

/* All interrupts in the normal and abnormal interrupt summary. Early
 * transmit interrupt (ETIE) is excluded from the abnormal set because it
 * causes too many interrupts and is not interesting.
 */

#define ENET_DMAINT_NORMAL (ENET_DMA_INTEN_TIE | \
                            ENET_DMA_INTEN_TBUIE | \
                            ENET_DMA_INTEN_RIE | \
                            ENET_DMA_INTEN_ERIE)

#define ENET_DMAINT_ABNORMAL (ENET_DMA_INTEN_TPSIE | \
                              ENET_DMA_INTEN_TJTIE | \
                              ENET_DMA_INTEN_ROIE | \
                              ENET_DMA_INTEN_TUIE | \
                              ENET_DMA_INTEN_RBUIE | \
                              ENET_DMA_INTEN_RPSIE | \
                              ENET_DMA_INTEN_RWTIE | \
                              /* ENET_DMA_INTEN_ETIE | */ \
                              ENET_DMA_INTEN_FBEIE)

/* Normal receive, transmit, error interrupt enable bit sets */

#define ENET_DMA_INTEN_RECV_ENABLE    (ENET_DMA_INTEN_NIE | ENET_DMA_INTEN_RIE)
#define ENET_DMA_INTEN_XMIT_ENABLE    (ENET_DMA_INTEN_NIE | ENET_DMA_INTEN_TIE)
#define ENET_DMA_INTEN_XMIT_DISABLE   (ENET_DMA_INTEN_TIE)

#ifdef CONFIG_DEBUG_NET
#  define ENET_DMA_INTEN_ERROR_ENABLE (ENET_DMA_INTEN_AIE | ENET_DMAINT_ABNORMAL)
#else
#  define ENET_DMA_INTEN_ERROR_ENABLE (0)
#endif

/* Helpers ******************************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The gd32_enet_mac_s encapsulates all state information for a single
 * hardware interface
 */

struct gd32_enet_mac_s
{
    uint8_t              ifup    : 1;  /* true:ifup false:ifdown */
    uint8_t              mbps100 : 1;  /* 100MBps operation (vs 10 MBps) */
    uint8_t              fduplex : 1;  /* Full (vs. half) duplex */
    struct wdog_s        txpoll;       /* TX poll timer */
    struct wdog_s        txtimeout;    /* TX timeout timer */
    struct work_s        irqwork;      /* For deferring interrupt work to the work queue */
    struct work_s        pollwork;     /* For deferring poll work to the work queue */

    /* This holds the information visible to the NuttX network */

    struct net_driver_s  dev;          /* Interface understood by the network */

    /* Used to track transmit and receive descriptors */

    struct enet_txdesc_s *txhead;      /* Next available TX descriptor */
    struct enet_rxdesc_s *rxhead;      /* Next available RX descriptor */

    struct enet_txdesc_s *txtail;      /* First "in_flight" TX descriptor */
    struct enet_rxdesc_s *rxcurr;      /* First RX descriptor of the segment */
    uint16_t             segments;     /* RX segment count */
    uint16_t             inflight;     /* Number of TX transfers "in_flight" */
    sq_queue_t           freeb;        /* The free buffer list */

    /* Descriptor allocations */

    struct enet_rxdesc_s rxtable[CONFIG_GD32F4_ENET_NRXDESC];
    struct enet_txdesc_s txtable[CONFIG_GD32F4_ENET_NTXDESC];

    /* Buffer allocations */

    uint8_t rxbuffer[CONFIG_GD32F4_ENET_NRXDESC *
                     CONFIG_GD32F4_ENET_BUFSIZE];
    uint8_t alloc[GD32_ENET_NFREEBUFFERS * CONFIG_GD32F4_ENET_BUFSIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gd32_enet_mac_s g_enet_mac[GD32_NETHERNET];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

static void gd32_enet_clock_enable(void);

#define gd32_reg_read(addr)       getreg32(addr)
#define gd32_reg_write(val,addr)  putreg32(val,addr)

/* Free buffer management */

static void gd32_buf_init(struct gd32_enet_mac_s *priv);
static inline uint8_t *gd32_buf_alloc(struct gd32_enet_mac_s *priv);
static inline void gd32_buf_free(struct gd32_enet_mac_s *priv,
                                 uint8_t *buffer);
static inline bool gd32_is_free_buf(struct gd32_enet_mac_s *priv);

/* Common TX logic */

static int  gd32_transmit(struct gd32_enet_mac_s *priv);
static int  gd32_tx_poll(struct net_driver_s *dev);
static void gd32_do_poll(struct gd32_enet_mac_s *priv);

/* Interrupt handling */

static void gd32_interrupt_enable(struct gd32_enet_mac_s *priv,
                                  uint32_t ierbit);
static void gd32_interrupt_disable(struct gd32_enet_mac_s *priv,
                                   uint32_t ierbit);

static void gd32_free_segment(struct gd32_enet_mac_s *priv,
                             struct enet_rxdesc_s *rxfirst,
                             int segments);
static int  gd32_receive_frame(struct gd32_enet_mac_s *priv);
static void gd32_receive(struct gd32_enet_mac_s *priv);
static void gd32_freeframe(struct gd32_enet_mac_s *priv);
static void gd32_tx_done(struct gd32_enet_mac_s *priv);

static void gd32_interrupt_work(void *arg);
static int  gd32_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void gd32_txtimeout_work(void *arg);
static void gd32_tx_timeout_expiry(wdparm_t arg);

static void gd32_poll_work(void *arg);
static void gd32_poll_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  gd32_ifup(struct net_driver_s *dev);
static int  gd32_ifdown(struct net_driver_s *dev);

static void gd32_txavail_work(void *arg);
static int  gd32_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP)
static int  gd32_add_mac(struct net_driver_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int  gd32_remove_mac(struct net_driver_s *dev,
                            const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  gd32_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg);
#endif

/* Descriptor Initialization */

static void gd32_txdes_chain_init(struct gd32_enet_mac_s *priv);
static void gd32_rxdes_chain_init(struct gd32_enet_mac_s *priv);

/* PHY Initialization */

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int  gd32_phy_interrupt_enable(struct gd32_enet_mac_s *priv);
#endif
#if defined(CONFIG_GD32F4_AUTO_NEGOTIATION) || defined(CONFIG_NETDEV_PHY_IOCTL)
static int  gd32_phy_read(uint16_t phydevaddr, uint16_t phyregaddr,
                          uint16_t *value);
#endif
static int  gd32_phy_write(uint16_t phydevaddr, uint16_t phyregaddr,
                           uint16_t value);
static int  gd32_phy_init(struct gd32_enet_mac_s *priv);

/* MAC/DMA Initialization */

#ifdef CONFIG_GD32F4_MII
static inline void gd32_select_mii(void);
#endif
#ifdef CONFIG_GD32F4_RMII
static inline void gd32_select_rmii(void);
#endif
static inline void gd32_enet_gpio_config(struct gd32_enet_mac_s *priv);
static int  gd32_enet_reset(struct gd32_enet_mac_s *priv);
static int  gd32_mac_config(struct gd32_enet_mac_s *priv);
static void gd32_mac_address(struct gd32_enet_mac_s *priv);
static int  gd32_mac_enable(struct gd32_enet_mac_s *priv);
static int  gd32_enet_config(struct gd32_enet_mac_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  gd32_enet_clock_enable
 *
 * Description:
 *
 *   Enable ENET clock
 *
 ****************************************************************************/

static void gd32_enet_clock_enable(void)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  rcu_en = RCU_AHB1EN_ENETEN | RCU_AHB1EN_ENETTXEN | RCU_AHB1EN_ENETRXEN;

#ifdef CONFIG_GD32F4_ENET_PTP
  /* Enable PTP clock */

  rcu_en |= RCU_AHB1EN_ENETPTPEN;
#endif

  regaddr = GD32_RCU_AHB1EN;

  /* Check clock if alreay enable. */

  if (rcu_en != (rcu_en & getreg32(regaddr)))
    {
      /* Enable/disable AHB clock for ENET */

      modifyreg32(regaddr, 0, rcu_en);
    }
}

/****************************************************************************
 * Function: gd32_buf_init
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

static void gd32_buf_init(struct gd32_enet_mac_s *priv)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = priv->alloc;
       i < GD32_ENET_NFREEBUFFERS;
       i++, buffer += CONFIG_GD32F4_ENET_BUFSIZE)
    {
      sq_addlast((sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: gd32_buf_alloc
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

static inline uint8_t *gd32_buf_alloc(struct gd32_enet_mac_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: gd32_buf_free
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

static inline void gd32_buf_free(struct gd32_enet_mac_s *priv,
                                    uint8_t *buffer)
{
  /* Free the buffer by adding it to the end of the free buffer list */

  sq_addlast((sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: gd32_is_free_buf
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

static inline bool gd32_is_free_buf(struct gd32_enet_mac_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
}

/****************************************************************************
 * Function: gd32_transmit
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

static int gd32_transmit(struct gd32_enet_mac_s *priv)
{
  struct enet_txdesc_s *txdesc;
  struct enet_txdesc_s *txfirst;

  /* The internal (optimal) network buffer size may be configured to be
   * larger than the Ethernet buffer size.
   */

#if OPTIMAL_ENET_BUFSIZE > CONFIG_GD32F4_ENET_BUFSIZE
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

  DEBUGASSERT(txdesc && (txdesc->tdes0 & ENET_TDES0_DAV) == 0);

  /* Is the size to be sent greater than the size of the Ethernet buffer? */

  DEBUGASSERT(priv->dev.d_len > 0 && priv->dev.d_buf != NULL);

#if OPTIMAL_ENET_BUFSIZE > CONFIG_GD32F4_ENET_BUFSIZE
  if (priv->dev.d_len > CONFIG_GD32F4_ENET_BUFSIZE)
    {
      /* Yes... how many buffers will be need to send the packet? */

      bufcount = (priv->dev.d_len + (CONFIG_GD32F4_ENET_BUFSIZE - 1)) /
                 CONFIG_GD32F4_ENET_BUFSIZE;
      lastsize = priv->dev.d_len - (bufcount - 1) *
                 CONFIG_GD32F4_ENET_BUFSIZE;

      ninfo("bufcount: %d lastsize: %d\n", bufcount, lastsize);

      /* Set the first segment bit in the first TX descriptor */

      txdesc->tdes0 |= ENET_TDES0_FSG;

      /* Set up all but the last TX descriptor */

      buffer = priv->dev.d_buf;

      for (i = 0; i < bufcount; i++)
        {
          /* This could be a normal event but the design does not handle it */

          DEBUGASSERT((txdesc->tdes0 & ENET_TDES0_DAV) == 0);

          /* Set the Buffer1 address pointer */

          txdesc->tdes2 = (uint32_t)buffer;

          /* Set the buffer size in all TX descriptors */

          if (i == (bufcount - 1))
            {
              /* This is the last segment.  Set the last segment bit in the
               * last TX descriptor and ask for an interrupt when this
               * segment transfer completes.
               */

              txdesc->tdes0 |= (ENET_TDES0_LSG | ENET_TDES0_INTC);

              /* This segment is, most likely, of fractional buffersize */

              txdesc->tdes1  = lastsize;
              buffer        += lastsize;
            }
          else
            {
              /* This is not the last segment.  We don't want an interrupt
               * when this segment transfer completes.
               */

              txdesc->tdes0 &= ~ENET_TDES0_INTC;

              /* The size of the transfer is the whole buffer */

              txdesc->tdes1  = CONFIG_GD32F4_ENET_BUFSIZE;
              buffer        += CONFIG_GD32F4_ENET_BUFSIZE;
            }

          /* Give the descriptor to DMA */

          txdesc->tdes0 |= ENET_TDES0_DAV;
          txdesc         = (struct enet_txdesc_s *)txdesc->tdes3;
        }
    }
  else
#endif
    {
      /* The single descriptor is both the first and last segment.  And we do
       * want an interrupt when the transfer completes.
       */

      txdesc->tdes0 |= (ENET_TDES0_FSG | ENET_TDES0_LSG | ENET_TDES0_INTC);

      /* Set frame size */

      DEBUGASSERT(priv->dev.d_len <= CONFIG_NET_ETH_PKTSIZE);
      txdesc->tdes1 = priv->dev.d_len;

      /* Set the Buffer1 address pointer */

      txdesc->tdes2 = (uint32_t)priv->dev.d_buf;

      /* Set DAV bit of the TX descriptor tdes0.  This gives the buffer to
       * Ethernet DMA
       */

      txdesc->tdes0 |= ENET_TDES0_DAV;

      /* Point to the next available TX descriptor */

      txdesc = (struct enet_txdesc_s *)txdesc->tdes3;
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

  if (priv->inflight >= CONFIG_GD32F4_ENET_NTXDESC)
    {
      gd32_interrupt_disable(priv, ENET_DMA_INTEN_RIE);
    }

  /* Check if the TX Buffer unavailable flag is set */

  if ((gd32_reg_read(GD32_ENET_DMA_STAT) & ENET_DMA_INTEN_TBUIE) != 0)
    {
      /* Clear TX Buffer unavailable flag */

      gd32_reg_write(ENET_DMA_INTEN_TBUIE, GD32_ENET_DMA_STAT);

      /* Resume DMA transmission */

      gd32_reg_write(1, GD32_ENET_DMA_TPEN);
    }

  /* Enable TX interrupts */

  gd32_interrupt_enable(priv, ENET_DMA_INTEN_TIE);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->txtimeout, GD32_TXTIMEOUT,
           gd32_tx_timeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: gd32_tx_poll
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

static int gd32_tx_poll(struct net_driver_s *dev)
{
  struct gd32_enet_mac_s *priv =
    (struct gd32_enet_mac_s *)dev->d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

  /* Send the packet */

  gd32_transmit(priv);
  DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU. We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ENET_TDES0_DAV may be cleared BUT still
   * not available because gd32_freeframe() has not yet run. If
   * gd32_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_gd32_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & ENET_TDES0_DAV) != 0 ||
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

  dev->d_buf = gd32_buf_alloc(priv);

  /* We can't continue the poll if we have no buffers */

  if (dev->d_buf == NULL)
    {
      /* Terminate the poll. */

      return -ENOMEM;
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: gd32_do_poll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (gd32_tx_done),
 *   2. When new TX data is available (gd32_txavail_process), and
 *   3. After a TX timeout to restart the sending process
 *      (gd32_txtimeout_process).
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

static void gd32_do_poll(struct gd32_enet_mac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ENET_TDES0_DAV may be cleared BUT still
   * not available because gd32_freeframe() has not yet run. If
   * gd32_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_gd32_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & ENET_TDES0_DAV) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then poll the network for new XMIT data.
       * Allocate a buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = gd32_buf_alloc(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          devif_poll(dev, gd32_tx_poll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              gd32_buf_free(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }
}

/****************************************************************************
 * Function: gd32_interrupt_enable
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

static void gd32_interrupt_enable(struct gd32_enet_mac_s *priv,
                             uint32_t ierbit)
{
  uint32_t regval;

  /* Enable the specified "normal" interrupt */

  regval  = gd32_reg_read(GD32_ENET_DMA_INTEN);
  regval |= (ENET_DMA_INTEN_NIE | ierbit);
  gd32_reg_write(regval, GD32_ENET_DMA_INTEN);
}

/****************************************************************************
 * Function: gd32_interrupt_disable
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

static void gd32_interrupt_disable(struct gd32_enet_mac_s *priv,
                             uint32_t ierbit)
{
  uint32_t regval;

  /* Disable the "normal" interrupt */

  regval  = gd32_reg_read(GD32_ENET_DMA_INTEN);
  regval &= ~ierbit;

  /* Are all "normal" interrupts now disabled? */

  if ((regval & ENET_DMAINT_NORMAL) == 0)
    {
      /* Yes.. disable normal interrupts */

      regval &= ~ENET_DMA_INTEN_NIE;
    }

  gd32_reg_write(regval, GD32_ENET_DMA_INTEN);
}

/****************************************************************************
 * Function: gd32_free_segment
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

static void gd32_free_segment(struct gd32_enet_mac_s *priv,
                              struct enet_rxdesc_s *rxfirst,
                              int segments)
{
  struct enet_rxdesc_s *rxdesc;
  int i;

  ninfo("rxfirst: %p segments: %d\n", rxfirst, segments);

  /* Set DAV bit in RX descriptors.  This gives the buffers back to DMA */

  rxdesc = rxfirst;
  for (i = 0; i < segments; i++)
    {
      rxdesc->rdes0 = ENET_RDES0_DAV;
      rxdesc = (struct enet_rxdesc_s *)rxdesc->rdes3;
    }

  /* Reset the segment management logic */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Check if the RX Buffer unavailable flag is set */

  if ((gd32_reg_read(GD32_ENET_DMA_STAT) & ENET_DMA_INTEN_RBUIE) != 0)
    {
      /* Clear RBUS Ethernet DMA flag */

      gd32_reg_write(ENET_DMA_INTEN_RBUIE, GD32_ENET_DMA_STAT);

      /* Resume DMA reception */

      gd32_reg_write(1, GD32_ENET_DMA_RPEN);
    }
}

/****************************************************************************
 * Function: gd32_receive_frame
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

static int gd32_receive_frame(struct gd32_enet_mac_s *priv)
{
  struct enet_rxdesc_s *rxdesc;
  struct enet_rxdesc_s *rxcurr;
  uint8_t *buffer;
  int i;

  ninfo("rxhead: %p rxcurr: %p segments: %d\n",
        priv->rxhead, priv->rxcurr, priv->segments);

  /* Check if there are free buffers.  We cannot receive new frames in this
   * design unless there is at least one free buffer.
   */

  if (!gd32_is_free_buf(priv))
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
       (rxdesc->rdes0 & ENET_RDES0_DAV) == 0 &&
        i < CONFIG_GD32F4_ENET_NRXDESC &&
        priv->inflight < CONFIG_GD32F4_ENET_NTXDESC;
       i++)
    {
      /* Check if this is the first segment in the frame */

      if ((rxdesc->rdes0 & ENET_RDES0_FDES) != 0 &&
          (rxdesc->rdes0 & ENET_RDES0_LDES) == 0)
        {
          priv->rxcurr   = rxdesc;
          priv->segments = 1;
        }

      /* Check if this is an intermediate segment in the frame */

      else if (((rxdesc->rdes0 & ENET_RDES0_LDES) == 0) &&
               ((rxdesc->rdes0 & ENET_RDES0_FDES) == 0))
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

          if ((rxdesc->rdes0 & ENET_RDES0_ERRS) == 0)
            {
              struct net_driver_s *dev = &priv->dev;

              /* Get the Frame Length of the received packet: substruct 4
               * bytes of the CRC
               */

              dev->d_len = ((rxdesc->rdes0 & ENET_RDES0_FRML_MASK) >>
                            ENET_RDES0_FRML_SHIFT) - 4;

              if (priv->segments > 1 ||
                  dev->d_len > CONFIG_GD32F4_ENET_BUFSIZE)
                {
                  /* The Frame is to big, it spans segments */

                  nerr("ERROR: Dropped, RX descriptor Too big: %d in %d "
                      "segments\n", dev->d_len, priv->segments);

                  gd32_free_segment(priv, rxcurr, priv->segments);
                }

              else
                {
                  /* Get a buffer from the free list.  We don't even check if
                   * this is successful because we already assure the free
                   * list is not empty above.
                   */

                  buffer = gd32_buf_alloc(priv);

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

                  priv->rxhead   = (struct enet_rxdesc_s *)rxdesc->rdes3;
                  gd32_free_segment(priv, rxcurr, priv->segments);

                  ninfo("rxhead: %p d_buf: %p d_len: %d\n",
                        priv->rxhead, dev->d_buf, dev->d_len);

                  return OK;
                }
            }
          else
            {
              /* Drop the frame that contains the errors, reset the segment
               * scanning logic, and continue scanning with the next frame.
               */

              nerr("ERROR: Dropped, RX descriptor errors: %08" PRIx32 "\n",
                   rxdesc->rdes0);
              gd32_free_segment(priv, rxcurr, priv->segments);
            }
        }

      /* Try the next descriptor */

      rxdesc = (struct enet_rxdesc_s *)rxdesc->rdes3;
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
 * Function: gd32_receive
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

static void gd32_receive(struct gd32_enet_mac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while gd32_receive_frame() successfully retrieves valid
   * Ethernet frames.
   */

  while (gd32_receive_frame(priv) == OK)
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
          nwarn("WARNING: DROPPED Too big: %d\n", dev->d_len);

          /* Free dropped packet buffer */

          if (dev->d_buf)
            {
              gd32_buf_free(priv, dev->d_buf);
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

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              gd32_transmit(priv);
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

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              gd32_transmit(priv);
            }
        }
      else
#endif
        {
          nerr("ERROR: Dropped, Unknown type: %04x\n", BUF->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          gd32_buf_free(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Function: gd32_freeframe
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

static void gd32_freeframe(struct gd32_enet_mac_s *priv)
{
  struct enet_txdesc_s *txdesc;
  int i;

  ninfo("txhead: %p txtail: %p inflight: %d\n",
        priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txdesc = priv->txtail;
  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

      for (i = 0; (txdesc->tdes0 & ENET_TDES0_DAV) == 0; i++)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          ninfo("txtail: %p tdes0: %08" PRIx32
                " tdes2: %08" PRIx32 " tdes3: %08" PRIx32 "\n",
                txdesc, txdesc->tdes0, txdesc->tdes2, txdesc->tdes3);

          DEBUGASSERT(txdesc->tdes2 != 0);

          /* Check if this is the first segment of a TX frame. */

          if ((txdesc->tdes0 & ENET_TDES0_FSG) != 0)
            {
              /* Yes.. Free the buffer */

              gd32_buf_free(priv, (uint8_t *)txdesc->tdes2);
            }

          /* In any event, make sure that TDES2 is nullified. */

          txdesc->tdes2 = 0;

          /* Check if this is the last segment of a TX frame */

          if ((txdesc->tdes0 & ENET_TDES0_LSG) != 0)
            {
              /* Yes.. Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If all of the TX descriptors were in-flight,
               * then RX interrupts may have been disabled...
               * we can re-enable them now.
               */

              gd32_interrupt_enable(priv, ENET_DMA_INTEN_RIE);

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return;
                }
            }

          /* Try the next descriptor in the TX chain */

          txdesc = (struct enet_txdesc_s *)txdesc->tdes3;
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
 * Function: gd32_tx_done
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet
 *   transfer(s) are complete.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void gd32_tx_done(struct gd32_enet_mac_s *priv)
{
  DEBUGASSERT(priv->txtail != NULL);

  /* Scan the TX descriptor change, returning buffers to free list */

  gd32_freeframe(priv);

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      /* Cancel the TX timeout */

      wd_cancel(&priv->txtimeout);

      /* And disable further TX interrupts. */

      gd32_interrupt_disable(priv, ENET_DMA_INTEN_TIE);
    }

  /* Then poll the network for new XMIT data */

  gd32_do_poll(priv);
}

/****************************************************************************
 * Function: gd32_interrupt_work
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

static void gd32_interrupt_work(void *arg)
{
  struct gd32_enet_mac_s *priv = (struct gd32_enet_mac_s *)arg;
  uint32_t dma_reg;

  DEBUGASSERT(priv);

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dma_reg = gd32_reg_read(GD32_ENET_DMA_STAT);

  /* Mask only enabled interrupts.  This depends on the fact that the
   * interrupt related bits (0-16) correspond in these two registers.
   */

  dma_reg &= gd32_reg_read(GD32_ENET_DMA_INTEN);

  /* Check if there are pending "normal" interrupts */

  if ((dma_reg & ENET_DMA_INTEN_NIE) != 0)
    {
      /* Yes.. Check if we received an incoming packet, if so, call
       * gd32_receive()
       */

      if ((dma_reg & ENET_DMA_INTEN_RIE) != 0)
        {
          /* Clear the pending receive interrupt */

          gd32_reg_write(ENET_DMA_INTEN_RIE, GD32_ENET_DMA_STAT);

          /* Handle the received package */

          gd32_receive(priv);
        }

      /* Check if a packet transmission just completed.  If so, call
       * gd32_tx_done(). This may disable further TX interrupts if there
       * are no pending transmissions.
       */

      if ((dma_reg & ENET_DMA_INTEN_TIE) != 0)
        {
          /* Clear the pending receive interrupt */

          gd32_reg_write(ENET_DMA_INTEN_TIE, GD32_ENET_DMA_STAT);

          /* Check if there are pending transmissions */

          gd32_tx_done(priv);
        }

      /* Clear the pending normal summary interrupt */

      gd32_reg_write(ENET_DMA_INTEN_NIE, GD32_ENET_DMA_STAT);
    }

  /* Handle error interrupt only if CONFIG_DEBUG_NET is eanbled */

#ifdef CONFIG_DEBUG_NET

  /* Check if there are pending "anormal" interrupts */

  if ((dma_reg & ENET_DMA_STAT_AI) != 0)
    {
      /* Just let the user know what happened */

      nerr("ERROR: Abormal event(s): %08x\n", dma_reg);

      /* Clear all pending abnormal events */

      gd32_reg_write(ENET_DMAINT_ABNORMAL, GD32_ENET_DMA_STAT);

      /* Clear the pending abnormal summary interrupt */

      gd32_reg_write(ENET_DMA_STAT_AI, GD32_ENET_DMA_STAT);
    }
#endif

  net_unlock();

  /* Re-enable Ethernet interrupts at the NVIC */

  up_enable_irq(GD32_IRQ_ENET);
}

/****************************************************************************
 * Function: gd32_interrupt
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

static int gd32_interrupt(int irq, void *context, void *arg)
{
  struct gd32_enet_mac_s *priv = &g_enet_mac[0];
  uint32_t dma_reg;

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dma_reg = gd32_reg_read(GD32_ENET_DMA_STAT);
  if (dma_reg != 0)
    {
      /* Disable further Ethernet interrupts.  Because Ethernet interrupts
       * are also disabled if the TX timeout event occurs, there can be no
       * race condition here.
       */

      up_disable_irq(GD32_IRQ_ENET);

      /* Check if a packet transmission just completed. */

      if ((dma_reg & ENET_DMA_INTEN_TIE) != 0)
        {
          /* If a TX transfer just completed, then cancel the TX timeout so
           * there will be no race condition between any subsequent timeout
           * expiration and the deferred interrupt processing.
           */

           wd_cancel(&priv->txtimeout);
        }

      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(ENET_WORK, &priv->irqwork, gd32_interrupt_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: gd32_txtimeout_work
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

static void gd32_txtimeout_work(void *arg)
{
  struct gd32_enet_mac_s *priv = (struct gd32_enet_mac_s *)arg;

  /* Reset the hardware.  Just take the interface down, then back up again. */

  net_lock();
  gd32_ifdown(&priv->dev);
  gd32_ifup(&priv->dev);

  /* Then poll for new XMIT data */

  gd32_do_poll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: gd32_tx_timeout_expiry
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

static void gd32_tx_timeout_expiry(wdparm_t arg)
{
  struct gd32_enet_mac_s *priv = (struct gd32_enet_mac_s *)arg;

  nerr("ERROR: Timeout!\n");

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   *
   * Interrupts will be re-enabled when gd32_ifup() is called.
   */

  up_disable_irq(GD32_IRQ_ENET);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * perhaps canceling any pending IRQ processing.
   */

  work_queue(ENET_WORK, &priv->irqwork, gd32_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: gd32_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
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

static void gd32_poll_work(void *arg)
{
  struct gd32_enet_mac_s *priv = (struct gd32_enet_mac_s *)arg;
  struct net_driver_s *dev  = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ENET_TDES0_DAV may be cleared BUT still
   * not available because gd32_freeframe() has not yet run. If
   * gd32_freeframe() has run, the buffer1 pointer (tdes2) will be
   * nullified (and inflight should be < CONFIG_gd32_ETH_NTXDESC).
   */

  net_lock();
  if ((priv->txhead->tdes0 & ENET_TDES0_DAV) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then perform the timer poll.  Allocate a
       * buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = gd32_buf_alloc(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          /* Update TCP timing states and poll the network for
           * new XMIT data.
           */

          devif_poll(dev, gd32_tx_poll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              gd32_buf_free(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }

  /* Setup the watchdog poll timer again */

  wd_start(&priv->txpoll, GD32_WDDELAY,
           gd32_poll_expiry, (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Function: gd32_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
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

static void gd32_poll_expiry(wdparm_t arg)
{
  struct gd32_enet_mac_s *priv = (struct gd32_enet_mac_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ENET_WORK, &priv->pollwork, gd32_poll_work, priv, 0);
}

/****************************************************************************
 * Function: gd32_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int gd32_ifup(struct net_driver_s *dev)
{
  struct gd32_enet_mac_s *priv =
    (struct gd32_enet_mac_s *)dev->d_private;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));
#endif

  /* Configure the Ethernet interface for DMA operation. */

  ret = gd32_enet_config(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set and activate a timer process */

  wd_start(&priv->txpoll, GD32_WDDELAY,
           gd32_poll_expiry, (wdparm_t)priv);

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  up_enable_irq(GD32_IRQ_ENET);

  return OK;
}

/****************************************************************************
 * Function: gd32_ifdown
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

static int gd32_ifdown(struct net_driver_s *dev)
{
  struct gd32_enet_mac_s *priv =
    (struct gd32_enet_mac_s *)dev->d_private;
  irqstate_t flags;
  int ret = OK;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(GD32_IRQ_ENET);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->txpoll);
  wd_cancel(&priv->txtimeout);

  /* Put the ENET_MAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the gd32_ifup() always
   * successfully brings the interface back up.
   */

  ret = gd32_enet_reset(priv);
  if (ret < 0)
    {
      nerr("ERROR: gd32_enet_reset failed (timeout), "
           "still assuming it's going down.\n");
    }

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Function: gd32_txavail_work
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

static void gd32_txavail_work(void *arg)
{
  struct gd32_enet_mac_s *priv = (struct gd32_enet_mac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      gd32_do_poll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: gd32_txavail
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

static int gd32_txavail(struct net_driver_s *dev)
{
  struct gd32_enet_mac_s *priv =
    (struct gd32_enet_mac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ENET_WORK, &priv->pollwork, gd32_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: gd32_calcethcrc
 *
 * Description:
 *   Function to calculate the CRC used by gd32 to check an ethernet frame
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

#if defined(CONFIG_NET_MCASTGROUP)
static uint32_t gd32_calcethcrc(const uint8_t *data, size_t length)
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
 * Function: gd32_add_mac
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

#if defined(CONFIG_NET_MCASTGROUP)
static int gd32_add_mac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast hash table */

  crc = gd32_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = GD32_ENET_MAC_HLH;
      hashindex -= 32;
    }
  else
    {
      registeraddress = GD32_ENET_MAC_HLL;
    }

  temp = gd32_reg_read(registeraddress);
  temp |= 1 << hashindex;
  gd32_reg_write(temp, registeraddress);

  temp = gd32_reg_read(GD32_ENET_MAC_FRMF);
  temp |= (ENET_MAC_FRMF_HMF | ENET_MAC_FRMF_HPFLT);
  gd32_reg_write(temp, GD32_ENET_MAC_FRMF);

  return OK;
}
#endif /* CONFIG_NET_MCASTGROUP */

/****************************************************************************
 * Function: gd32_remove_mac
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
static int gd32_remove_mac(struct net_driver_s *dev, const uint8_t *mac)
{
  uint32_t crc;
  uint32_t hashindex;
  uint32_t temp;
  uint32_t registeraddress;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Remove the MAC address to the hardware multicast hash table */

  crc = gd32_calcethcrc(mac, 6);

  hashindex = (crc >> 26) & 0x3f;

  if (hashindex > 31)
    {
      registeraddress = GD32_ENET_MAC_HLH;
      hashindex -= 32;
    }
  else
    {
      registeraddress = GD32_ENET_MAC_HLL;
    }

  temp = gd32_reg_read(registeraddress);
  temp &= ~(1 << hashindex);
  gd32_reg_write(temp, registeraddress);

  /* If there is no address registered any more, delete multicast filtering */

  if (gd32_reg_read(GD32_ENET_MAC_HLH) == 0 &&
      gd32_reg_read(GD32_ENET_MAC_HLL) == 0)
    {
      temp = gd32_reg_read(GD32_ENET_MAC_FRMF);
      temp &= ~(ENET_MAC_FRMF_HMF | ENET_MAC_FRMF_HPFLT);
      gd32_reg_write(temp, GD32_ENET_MAC_FRMF);
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: gd32_txdes_chain_init
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

static void gd32_txdes_chain_init(struct gd32_enet_mac_s *priv)
{
  struct enet_txdesc_s *txdesc;
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

  for (i = 0; i < CONFIG_GD32F4_ENET_NTXDESC; i++)
    {
      txdesc = &priv->txtable[i];

      /* Set Second Address Chained bit */

      txdesc->tdes0 = ENET_TDES0_TCHM;

#ifdef CHECKSUM_BY_HARDWARE
      /* Enable the checksum insertion for the TX frames */

      txdesc->tdes0 |= ENET_CHECKSUM_TCPUDPICMP_FULL;
#endif

      /* Clear Buffer1 address pointer (buffers will be assigned as they
       * are used)
       */

      txdesc->tdes2 = 0;

      /* Initialize the next descriptor with
       * the Next Descriptor Polling Enable
       */

      if (i < (CONFIG_GD32F4_ENET_NTXDESC - 1))
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

  gd32_reg_write((uint32_t)priv->txtable, GD32_ENET_DMA_TDTADDR);
}

/****************************************************************************
 * Function: gd32_rxdes_chain_init
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

static void gd32_rxdes_chain_init(struct gd32_enet_mac_s *priv)
{
  struct enet_rxdesc_s *rxdesc;
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

  for (i = 0; i < CONFIG_GD32F4_ENET_NRXDESC; i++)
    {
      rxdesc = &priv->rxtable[i];

      /* Set DAV bit of the RX descriptor rdes0 */

      rxdesc->rdes0 = ENET_RDES0_DAV;

      /* Set Buffer1 size and Second Address Chained bit and enabled DMA
       * RX desc receive interrupt
       */

      rxdesc->rdes1 = ENET_RDES1_RCHM | (uint32_t)CONFIG_GD32F4_ENET_BUFSIZE;

      /* Set Buffer1 address pointer */

      rxdesc->rdes2 = (uint32_t)&priv->rxbuffer[i *
                      CONFIG_GD32F4_ENET_BUFSIZE];

      /* Initialize the next descriptor with
       * the Next Descriptor Polling Enable
       */

      if (i < (CONFIG_GD32F4_ENET_NRXDESC - 1))
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

  gd32_reg_write((uint32_t)priv->rxtable, GD32_ENET_DMA_RDTADDR);
}

/****************************************************************************
 * Function: gd32_ioctl
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
static int gd32_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
  struct gd32_enet_mac_s *priv =
    (struct gd32_enet_mac_s *)dev->d_private;
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

              ret = gd32_phy_interrupt_enable(priv);
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = CONFIG_GD32F4_PHY_ADDR;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = gd32_phy_read(req->phy_id, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ret = gd32_phy_write(req->phy_id, req->reg_num, req->val_in);
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
 * Function: gd32_phy_interrupt_enable
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

#if defined(CONFIG_NETDEV_PHY_IOCTL) && defined(CONFIG_ARCH_PHY_INTERRUPT)
static int gd32_phy_interrupt_enable(struct gd32_enet_mac_s *priv)
{
  uint16_t phyval;
  int ret;

  ret = gd32_phy_read(CONFIG_GD32F4_PHY_ADDR, MII_INT_REG, &phyval);
  if (ret == OK)
    {
      /* Enable link up/down interrupts */

      ret = gd32_phy_write(CONFIG_GD32F4_PHY_ADDR, MII_INT_REG,
                           (phyval & ~MII_INT_CLREN) | MII_INT_SETEN);
    }

  return ret;
}
#endif

/****************************************************************************
 * Function: gd32_phy_read
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

#if defined(CONFIG_GD32F4_AUTO_NEGOTIATION) || defined(CONFIG_NETDEV_PHY_IOCTL)
static int gd32_phy_read(uint16_t phydevaddr,
                         uint16_t phyregaddr, uint16_t *value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MAC_PHY_CTL register,
   * preserving CSR Clock Range CR[2:0] bits
   */

  regval  = gd32_reg_read(GD32_ENET_MAC_PHY_CTL);
  regval &= ENET_MAC_PHY_CTL_CLR_MASK;

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the  ENET_MAC_PHY_CTL_PW is clear, indicating a read operation.
   */

  regval |= (phydevaddr << ENET_MAC_PHY_CTL_PA_SHIFT) &
            ENET_MAC_PHY_CTL_PA_MASK;
  regval |= (phyregaddr << ENET_MAC_PHY_CTL_PR_SHIFT) &
            ENET_MAC_PHY_CTL_PR_MASK;
  regval |= ENET_MAC_PHY_CTL_PB;

  gd32_reg_write(regval, GD32_ENET_MAC_PHY_CTL);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_READ_TIMEOUT; timeout++)
    {
      if ((gd32_reg_read(GD32_ENET_MAC_PHY_CTL) & ENET_MAC_PHY_CTL_PB) == 0)
        {
          *value = (uint16_t)gd32_reg_read(GD32_ENET_MAC_PHY_DATA);
          return OK;
        }
    }

  nerr("ERROR: MII transfer timed out: phydevaddr: %04x phyregaddr: %04x\n",
       phydevaddr, phyregaddr);

  return -ETIMEDOUT;
}
#endif

/****************************************************************************
 * Function: gd32_phy_write
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

static int gd32_phy_write(uint16_t phydevaddr,
                          uint16_t phyregaddr, uint16_t value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MAC_PHY_CTL register,
   * preserving CSR Clock Range CR[2:0] bits
   */

  regval  = gd32_reg_read(GD32_ENET_MAC_PHY_CTL);
  regval &= ENET_MAC_PHY_CTL_CLR_MASK;

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the  ENET_MAC_PHY_CTL_PW is set, indicating a write operation.
   */

  regval |= (phydevaddr << ENET_MAC_PHY_CTL_PA_SHIFT) &
            ENET_MAC_PHY_CTL_PA_MASK;
  regval |= (phyregaddr << ENET_MAC_PHY_CTL_PR_SHIFT) &
            ENET_MAC_PHY_CTL_PR_MASK;
  regval |= (ENET_MAC_PHY_CTL_PB | ENET_MAC_PHY_CTL_PW);

  /* Write the value into the MAC_PHY_DATA register before setting the new
   * MAC_PHY_CTL register value.
   */

  gd32_reg_write(value, GD32_ENET_MAC_PHY_DATA);
  gd32_reg_write(regval, GD32_ENET_MAC_PHY_CTL);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_WRITE_TIMEOUT; timeout++)
    {
      if ((gd32_reg_read(GD32_ENET_MAC_PHY_CTL) & ENET_MAC_PHY_CTL_PB) == 0)
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
 * Function: gd32_phy_init
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

static int gd32_phy_init(struct gd32_enet_mac_s *priv)
{
#ifdef CONFIG_GD32F4_AUTO_NEGOTIATION
  volatile uint32_t timeout;
#endif

  uint32_t regval;
  uint16_t phyval;
  int ret;

  /* Assume 10MBps and half duplex */

  priv->mbps100 = 0;
  priv->fduplex = 0;

  /* Setup up PHY clocking by setting the SR field in
   * the MAC_PHY_CTL register
   */

  regval  = gd32_reg_read(GD32_ENET_MAC_PHY_CTL);
  regval &= ~ENET_MAC_PHY_CTL_CLR_MASK;
  regval |= ENET_MAC_PHY_CTL_CLR;
  gd32_reg_write(regval, GD32_ENET_MAC_PHY_CTL);

  /* Put the PHY in reset mode */

  ret = gd32_phy_write(CONFIG_GD32F4_PHY_ADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: Failed to reset the PHY: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_RESET_DELAY);

  /* Perform any necessary, board-specific PHY initialization */

#ifdef CONFIG_GD32F4_PHY_INIT
  ret = gd32_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Perform auto-negotiation if so configured */

#ifdef CONFIG_GD32F4_AUTO_NEGOTIATION
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = gd32_phy_read(CONFIG_GD32F4_PHY_ADDR, MII_MSR, &phyval);
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

  ret = gd32_phy_write(CONFIG_GD32F4_PHY_ADDR, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = gd32_phy_read(CONFIG_GD32F4_PHY_ADDR, MII_MSR, &phyval);
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

  ret = gd32_phy_read(CONFIG_GD32F4_PHY_ADDR, CONFIG_GD32F4_PHY_SR, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  ninfo("PHYSR[%d]: %04x\n", CONFIG_GD32F4_PHY_SR, phyval);

  /* Different PHYs present speed and mode information in different ways.
   * IF This CONFIG_gd32_PHYSR_ALTCONFIG is selected, this indicates that
   * the PHY represents speed and mode information are combined, for example,
   * with separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_GD32F4_PHY_SR_ALTCONFIG
  switch (phyval & CONFIG_GD32F4_PHY_SR_ALTMODE)
    {
      default:
      case CONFIG_GD32F4_PHY_SR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_GD32F4_PHY_SR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_GD32F4_PHY_SR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_GD32F4_PHY_SR_100FD:
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
  if ((phyval & CONFIG_GD32F4_PHY_SR_MODE) ==
       CONFIG_GD32F4_PHY_SR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_GD32F4_PHY_SR_SPEED) == CONFIG_GD32F4_PHY_SR_100M)
    {
      priv->mbps100 = 1;
    }
#endif

#else /* Auto-negotiation not selected */

  phyval = 0;
#ifdef CONFIG_GD32F4_ENET_MODE_FULLDUPLEX
  phyval |= MII_MCR_FULLDPLX;
#endif
#ifdef CONFIG_GD32F4_ENET_SPEEDMODE_100M
  phyval |= MII_MCR_SPEED100;
#endif

  ret = gd32_phy_write(CONFIG_GD32F4_PHY_ADDR, MII_MCR, phyval);
  if (ret < 0)
    {
     nerr("ERROR: Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }

  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes */

#ifdef CONFIG_GD32F4_ENET_MODE_FULLDUPLEX
  priv->fduplex = 1;
#endif
#ifdef CONFIG_GD32F4_ENET_SPEEDMODE_100M
  priv->mbps100 = 1;
#endif
#endif

  ninfo("Duplex: %s Speed: %d MBps\n",
        priv->fduplex ? "FULL" : "HALF",
        priv->mbps100 ? 100 : 10);

  return OK;
}

/****************************************************************************
 * Name: gd32_select_mii
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

#ifdef CONFIG_GD32F4_MII
static inline void gd32_select_mii(void)
{
  uint32_t regval;

  regval  = getreg32(GD32_SYSCFG_CFG1);
  regval &= ~SYSCFG_CFG1_ENET_PHY_SEL_MASK;
  putreg32(regval, GD32_SYSCFG_CFG1);
}
#endif

/****************************************************************************
 * Name: gd32_select_rmii
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

static inline void gd32_select_rmii(void)
{
  gd32_syscfg_clock_enable();
  gd32_syscfg_enet_phy_interface_config(SYSCFG_ENET_PHY_RMII);
}

/****************************************************************************
 * Function: gd32_enet_gpio_config
 *
 * Description:
 *  Configure GPIOs for the ENET interface.
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

static inline void gd32_enet_gpio_config(struct gd32_enet_mac_s *priv)
{
  /* Configure GPIO pins to support ENET */

#if defined(CONFIG_GD32F4_MII) || defined(CONFIG_GD32F4_RMII)

  /* MDC and MDIO are common to both modes */

  gd32_gpio_config(GPIO_ENET_MDC);
  gd32_gpio_config(GPIO_ENET_MDIO);

  /* Set up the MII interface */

#if defined(CONFIG_GD32F4_MII)

  /* Select the MII interface */

  gd32_syscfg_enet_phy_interface_config(SYSCFG_ENET_PHY_MII);

  /* Provide clocking via CKOUT0 or CKOUT1:
   *
   * "CKOUT0, used to output CK_IRC16M, CK_LXTAL, CK_HXTAL or
   *  CK_PLLP clock (through a configurable prescaler) on PA8 pin."
   *
   * "CKOUT1, used to output CK_SYS, CK_PLL2SR, CK_HXTAL or
   *  CK_PLLP clock (through a configurable prescaler) on PC9 pin."
   */

# if defined(CONFIG_GD32F4_MII_CKOUT0)
  /* Configure CKOUT0 to drive the PHY.  Board logic must provide
   * CKOUT0 clocking info.
   */

  gd32_gpio_config(GPIO_CKOUT0);
  gd32_rcu_ckout0_config(BOARD_CFG_CKOUT0_SOURCE, BOARD_CFG_CKOUT0_DIVIDER);

# elif defined(CONFIG_GD32F4_MII_CKOUT1)
  /* Configure CKOUT1 to drive the PHY.  Board logic must provide
   * CKOUT1 clocking info.
   */

  gd32_gpio_config(GPIO_CKOUT1);
  gd32_rcu_ckout1_config(BOARD_CFG_CKOUT1_SOURCE, BOARD_CFG_CKOUT1_DIVIDER);
# endif

  /* MII interface pins (17):
   *
   * MII_TX_CLK, MII_TXD[3:0], MII_TX_EN, MII_RX_CLK, MII_RXD[3:0],
   * MII_RX_ER, MII_RX_DV, MII_CRS, MII_COL, MDC, MDIO
   */

  gd32_gpio_config(GPIO_ENET_MII_COL);
  gd32_gpio_config(GPIO_ENET_MII_CRS);
  gd32_gpio_config(GPIO_ENET_MII_RXD0);
  gd32_gpio_config(GPIO_ENET_MII_RXD1);
  gd32_gpio_config(GPIO_ENET_MII_RXD2);
  gd32_gpio_config(GPIO_ENET_MII_RXD3);
  gd32_gpio_config(GPIO_ENET_MII_RX_CLK);
  gd32_gpio_config(GPIO_ENET_MII_RX_DV);
  gd32_gpio_config(GPIO_ENET_MII_RX_ER);
  gd32_gpio_config(GPIO_ENET_MII_TXD0);
  gd32_gpio_config(GPIO_ENET_MII_TXD1);
  gd32_gpio_config(GPIO_ENET_MII_TXD2);
  gd32_gpio_config(GPIO_ENET_MII_TXD3);
  gd32_gpio_config(GPIO_ENET_MII_TX_CLK);
  gd32_gpio_config(GPIO_ENET_MII_TX_EN);

  /* Set up the RMII interface. */

#elif defined(CONFIG_GD32F4_RMII)

  /* Select the RMII interface */

  gd32_select_rmii();

  /* Provide clocking via CKOUT0 or CKOUT1:
   *
   * "CKOUT0, used to output CK_IRC16M, CK_LXTAL, CK_HXTAL or
   *  CK_PLLP clock (through a configurable prescaler) on PA8 pin."
   *
   * "CKOUT1, used to output CK_SYS, CK_PLL2SR, CK_HXTAL or
   *  CK_PLLP clock (through a configurable prescaler) on PC9 pin."
   */

# if defined(CONFIG_GD32F4_RMII_CKOUT0)
  /* Configure CKOUT0 to drive the PHY.  Board logic must provide
   * CKOUT0 clocking info.
   */

  gd32_gpio_config(GPIO_CKOUT0);
  gd32_rcu_ckout0_config(BOARD_CFG_CKOUT0_SOURCE, BOARD_CFG_CKOUT0_DIV);

# elif defined(CONFIG_GD32F4_RMII_CKOUT1)
  /* Configure CKOUT1 to drive the PHY.  Board logic must provide
   * CKOUT1 clocking info.
   */

  gd32_gpio_config(GPIO_CKOUT1);
  gd32_rcu_ckout1_config(BOARD_CFG_CKOUT1_SOURCE, BOARD_CFG_CKOUT1_DIVIDER);

# endif

  /* RMII interface pins (7):
   *
   * RMII_TXD[1:0], RMII_TX_EN, RMII_RXD[1:0], RMII_CRS_DV, MDC, MDIO,
   * RMII_REF_CLK
   */

  gd32_gpio_config(GPIO_ENET_RMII_CRS_DV);
  gd32_gpio_config(GPIO_ENET_RMII_REF_CLK);
  gd32_gpio_config(GPIO_ENET_RMII_RXD0);
  gd32_gpio_config(GPIO_ENET_RMII_RXD1);
  gd32_gpio_config(GPIO_ENET_RMII_TX_EN);
  gd32_gpio_config(GPIO_ENET_RMII_TXD0);
  gd32_gpio_config(GPIO_ENET_RMII_TXD1);

#endif
#endif

#ifdef CONFIG_GD32F4_ENET_PTP
  /* Enable pulse-per-second (PPS) output signal */

  gd32_gpio_config(GPIO_ENET_PPS_OUT);
#endif
}

/****************************************************************************
 * Function: gd32_enet_reset
 *
 * Description:
 *  Reset the ENET block.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   Zero on success, or a negated errno value on any failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int gd32_enet_reset(struct gd32_enet_mac_s *priv)
{
  uint32_t regval;
  uint32_t retries;

  /* Reset the ENET on the AHB1 bus */

  regval  = gd32_reg_read(GD32_RCU_AHB1RST);
  regval |= RCU_AHB1RST_ENETRST;
  gd32_reg_write(regval, GD32_RCU_AHB1RST);

  regval &= ~RCU_AHB1RST_ENETRST;
  gd32_reg_write(regval, GD32_RCU_AHB1RST);

  /* Perform a software reset by setting the SR bit in the DMA_BCTL register.
   * This Resets all MAC subsystem internal registers and logic.  After this
   * reset all the registers holds their reset values.
   */

  regval  = gd32_reg_read(GD32_ENET_DMA_BCTL);
  regval |= ENET_DMA_BCTL_SWR;
  gd32_reg_write(regval, GD32_ENET_DMA_BCTL);

  /* Wait for software reset to complete. The SWR bit is cleared
   * automatically after the reset operation has completed
   * in all core clock domains.
   */

  retries = 10;
  while (((gd32_reg_read(GD32_ENET_DMA_BCTL) & ENET_DMA_BCTL_SWR) != 0) &&
         retries > 0)
    {
      retries--;
      up_mdelay(10);
    }

  if (retries == 0)
    {
      return -ETIMEDOUT;
    }

  return 0;
}

/****************************************************************************
 * Function: gd32_mac_config
 *
 * Description:
 *  Configure the ENET MAC for DMA operation.
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

static int gd32_mac_config(struct gd32_enet_mac_s *priv)
{
  uint32_t regval;

  /* Set up the MAC_CFG register */

  regval  = gd32_reg_read(GD32_ENET_MAC_CFG);
  regval &= ~ENET_MAC_CFG_CLEAR_BITS;
  regval |= ENET_MAC_CFG_SET_BITS;

  if (priv->fduplex)
    {
      /* Set the DPM bit for full duplex support */

      regval |= ENET_MAC_CFG_DPM;
    }

  if (priv->mbps100)
    {
      /* Set the SPD bit for 100Mbps fast ENET support */

      regval |= ENET_MAC_CFG_SPD;
    }

  gd32_reg_write(regval, GD32_ENET_MAC_CFG);

  /* Set up the MAC_FRMF register */

  regval  = gd32_reg_read(GD32_ENET_MAC_FRMF);
  regval &= ~ENET_MAC_FRMF_CLEAR_BITS;
  regval |= ENET_MAC_FRMF_SET_BITS;
  gd32_reg_write(regval, GD32_ENET_MAC_FRMF);

  /* Set up the MAC_HLH and MAC_HLL registers */

  gd32_reg_write(0, GD32_ENET_MAC_HLH);
  gd32_reg_write(0, GD32_ENET_MAC_HLL);

  /* Setup up the MAC_FCTL register */

  regval  = gd32_reg_read(GD32_ENET_MAC_FCTL);
  regval &= ~ENET_MAC_FCTL_CLEAR_MASK;
  regval |= ENET_MAC_FCTL_SET_MASK;
  gd32_reg_write(regval, GD32_ENET_MAC_FCTL);

  /* Setup up the MAC_VLT register */

  gd32_reg_write(0, GD32_ENET_MAC_VLT);

  /* DMA Configuration */

  /* Set up the DMA_CTL register */

  regval  = gd32_reg_read(GD32_ENET_DMA_CTL);
  regval &= ~ENET_DMA_CTL_CLEAR_MASK;
  regval |= ENET_DMA_CTL_SET_MASK;
  gd32_reg_write(regval, GD32_ENET_DMA_CTL);

  /* Set up the DMA_BCTL register */

  regval  = gd32_reg_read(GD32_ENET_DMA_BCTL);
  regval &= ~ENET_DMA_BCTL_CLEAR_MASK;
  regval |= ENET_DMA_BCTL_SET_MASK;
  gd32_reg_write(regval, GD32_ENET_DMA_BCTL);

  return OK;
}

/****************************************************************************
 * Function: gd32_mac_address
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

static void gd32_mac_address(struct gd32_enet_mac_s *priv)
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
  gd32_reg_write(regval, GD32_ENET_MAC_ADDR0H);

  /* Set the MAC address low register */

  regval = ((uint32_t)dev->d_mac.ether.ether_addr_octet[3] << 24) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[2] << 16) |
           ((uint32_t)dev->d_mac.ether.ether_addr_octet[1] <<  8) |
            (uint32_t)dev->d_mac.ether.ether_addr_octet[0];
  gd32_reg_write(regval, GD32_ENET_MAC_ADDR0L);
}

/****************************************************************************
 * Function: gd32_mac_enable
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

static int gd32_mac_enable(struct gd32_enet_mac_s *priv)
{
  uint32_t regval;

  /* Set the MAC address */

  gd32_mac_address(priv);

  /* Enable transmit state machine of the MAC for transmission on the MII */

  regval  = gd32_reg_read(GD32_ENET_MAC_CFG);
  regval |= ENET_MAC_CFG_TEN;
  gd32_reg_write(regval, GD32_ENET_MAC_CFG);

  /* Flush Transmit FIFO */

  regval  = gd32_reg_read(GD32_ENET_DMA_CTL);
  regval |= ENET_DMA_CTL_FTF;
  gd32_reg_write(regval, GD32_ENET_DMA_CTL);

  /* Enable receive state machine of the MAC for reception from the MII */

  /* Enables or disables the MAC reception. */

  regval  = gd32_reg_read(GD32_ENET_MAC_CFG);
  regval |= ENET_MAC_CFG_REN;
  gd32_reg_write(regval, GD32_ENET_MAC_CFG);

  /* Start DMA transmission */

  regval  = gd32_reg_read(GD32_ENET_DMA_CTL);
  regval |= ENET_DMA_CTL_STE;
  gd32_reg_write(regval, GD32_ENET_DMA_CTL);

  /* Start DMA reception */

  regval  = gd32_reg_read(GD32_ENET_DMA_CTL);
  regval |= ENET_DMA_CTL_SRE;
  gd32_reg_write(regval, GD32_ENET_DMA_CTL);

  /* Enable ENET DMA interrupts.
   *
   * The gd32 hardware supports two interrupts: (1) one dedicated to normal
   * ENET operations and the other, used only for the ENET wakeup
   * event.  The wake-up interrupt is not used by this driver.
   *
   * The first ENET vector is reserved for interrupts generated by the
   * MAC and the DMA. The MAC provides PMT and time stamp trigger interrupts,
   * neither of which are used by this driver.
   */

  gd32_reg_write(ENET_MAC_INTMSK_ALLINTS, GD32_ENET_MAC_INTMSK);

  /* ENET DMA supports two classes of interrupts: Normal interrupt
   * summary (NIS) and Abnormal interrupt summary (AIS) with a variety
   * individual normal and abnormal interrupting events.  Here only
   * the normal receive event is enabled (unless DEBUG is enabled).  Transmit
   * events will only be enabled when a transmit interrupt is expected.
   */

  gd32_reg_write(ENET_DMA_INTEN_RECV_ENABLE | ENET_DMA_INTEN_ERROR_ENABLE,
               GD32_ENET_DMA_INTEN);
  return OK;
}

/****************************************************************************
 * Function: gd32_enet_config
 *
 * Description:
 *  Configure the ENET interface for DMA operation.
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

static int gd32_enet_config(struct gd32_enet_mac_s *priv)
{
  int ret;

  /* NOTE: The ENET clocks were initialized early in the boot-up
   * sequence in gd32f4xx_rcu.c.
   */

  /* Enable ENET clocks */

  gd32_enet_clock_enable();

  /* Reset the ENET block */

  ninfo("Reset the ENET block\n");
  ret = gd32_enet_reset(priv);
  if (ret < 0)
    {
      nerr("ERROR: Reset of ENET block failed\n");
      return ret;
    }

  /* Initialize the PHY */

  ninfo("Initialize the PHY\n");
  ret = gd32_phy_init(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the MAC and DMA */

  ninfo("Initialize the MAC and DMA\n");
  ret = gd32_mac_config(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the free buffer list */

  gd32_buf_init(priv);

  /* Initialize TX Descriptors list: Chain Mode */

  gd32_txdes_chain_init(priv);

  /* Initialize RX Descriptors list: Chain Mode  */

  gd32_rxdes_chain_init(priv);

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");
  return gd32_mac_enable(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: gd32_enet_init
 *
 * Description:
 *   Initialize the ENET driver for one interface.  If the gd32 chip
 *   supports multiple ENET controllers, then board specific logic
 *   must implement arm_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Input Parameters:
 *   intf - In the case where there are multiple ENET_MACs, this value
 *          identifies which ENET_MAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if GD32_NETHERNET == 1 || defined(CONFIG_NETDEV_LATEINIT)
static inline
#endif
int gd32_enet_init(int intf)
{
  struct gd32_enet_mac_s *priv;
  int ret;

  ninfo("intf: %d\n", intf);

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < GD32_NETHERNET);
  priv = &g_enet_mac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct gd32_enet_mac_s));
  priv->dev.d_ifup    = gd32_ifup;          /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = gd32_ifdown;        /* I/F down callback */
  priv->dev.d_txavail = gd32_txavail;       /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = gd32_add_mac;       /* Add multicast MAC address */
  priv->dev.d_rmmac   = gd32_remove_mac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = gd32_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = g_enet_mac;  /* Used to recover private state from dev */

  /* Configure GPIO pins to support ENET */

  gd32_enet_gpio_config(priv);

  /* Attach the IRQ to the driver */

  if (irq_attach(GD32_IRQ_ENET, gd32_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Put the interface in the down state. */

  ret = gd32_ifdown(&priv->dev);
  if (ret < 0)
    {
      nerr("ERROR: Initialization of ENET block failed: %d\n", ret);
      return ret;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c.  If GD32_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of arm_netinitialize() that calls gd32_enet_init() with
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

#if GD32_NETHERNET == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  gd32_enet_init(0);
}
#endif

#endif /* GD32_NETHERNET > 0 */
#endif /* CONFIG_NET && CONFIG_GD32F4_ENETMAC */
